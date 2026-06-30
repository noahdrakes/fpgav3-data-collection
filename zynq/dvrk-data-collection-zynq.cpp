/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Noah Drakes

  (C) Copyright 2024 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

// stdlibs
#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <unistd.h>
#include <string>
#include <chrono>
#include <pthread.h>
#include <atomic>
#include <cmath>
#include <robot_config.h>
#include <nlohmann/json.hpp>
#include "unit_conversion.h"
#include <onnxruntime_cxx_api.h>

// mmap mio pins
#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>
#include <sys/mman.h>

// dvrk libs
#include "BasePort.h"
#include "PortFactory.h"
#include "ZynqEmioPort.h"
#include "AmpIO.h"

// shared header
#include "data_collection_shared.h"

using namespace std;

// UDP_MAX_PACKET_SIZE (in bytes) is calculated from GetMaxWriteDataSize (based
// on the MTU) in EthUdpPort.cpp
// PK: This is now defined in data_collection_shared.h
const int UDP_MAX_PACKET_SIZE = UDP_REAL_MTU;

// defines and variables for MIO Memory mapping for reading MIO pins
const uint32_t GPIO_BASE_ADDR = 0xE000A000;
const unsigned int GPIO_BANK1_OFFSET = 0x8;
const uint32_t SCLR_CLK_BASE_ADDR = 0xF8000000;
volatile uint32_t *GPIO_MEM_REGION;

// FLAG for including Processor IO in data packets
bool use_ps_io_flag = false;
// FLAG for including joint Potentiometer readings in data packets
bool use_pot_flag = false;

// FLAG for including contact model predictions
bool use_contact_model = false;
float contact_velocity[16]; // AmpIO::MAX_CHANNELS
float contact_torque[16];   // AmpIO::MAX_CHANNELS

bool use_si_units = false;

RobotConfig cfg;

///////////////////////////////////
///// STATE MACHINE VARIABLES /////
//////////////////////////////////

// Double Buffer Struct handling double buffer between collecting and transmitting data between
// threads
struct Double_Buffer_Info {
    uint32_t double_buffer[2][UDP_MAX_PACKET_SIZE/4]; //note: changed from 1500 which makes sense
    uint16_t buffer_size;
    uint8_t prod_buf;
    uint8_t cons_buf;
    atomic_uint8_t cons_busy;
};

// have methods return new state
// slim down sm struct to non local

DataCollectionMeta data_collection_meta;
Double_Buffer_Info db;
char recvd_cmd[CMD_MAX_STRING_SIZE] = {0};


struct Dvrk_Controller {
    BasePort *Port;
    AmpIO *Board;
} dvrk_controller;

struct SM{
    // states
    int state = 0;
    int last_state = 0;

    // return codes
    int ret = 0;
    int udp_ret;
};


// DEBUGGING VARIABLES
int data_packet_count = 0;
int sample_count = 0;

// Motor Current/Status arrays to store data
// for emio timeout error
int32_t emio_read_error_counter = 0;

// start time for data collection timestamps
double last_timestamp = 0;

// timespec vaiables for getting timestamp using gettime() method
// with CLOCK_MONOTONIC_RAW.

timespec t_data_collection_start;

// keeps track of deadline to control sample rate
timespec deadline;

// expected period of sample capture
long period_ns;


int SAMPLE_RATE = 0;
bool useSampleRate = false;

// FLAG set when the host terminates data collection
bool stop_data_collection_flag = false;

// State Machine states
enum DataCollectionStateMachine {
    SM_READY = 0,
    SM_SEND_READY_STATE_TO_HOST,
    SM_WAIT_FOR_HOST_HANDSHAKE,
    SM_WAIT_FOR_HOST_FLAG_CMD,
    SM_WAIT_FOR_HOST_FLAG_VALUE,
    SM_WAIT_FOR_HOST_START_CMD,
    SM_START_DATA_COLLECTION,
    SM_CHECK_FOR_STOP_DATA_COLLECTION_CMD,
    SM_START_CONSUMER_THREAD,
    SM_PACKAGE_DATA_COLLECTION_METADATA,
    SM_SEND_DATA_COLLECTION_METADATA,
    SM_WAIT_FOR_HOST_RECV_METADATA,
    SM_PRODUCE_DATA,
    SM_CONSUME_DATA,
    SM_TERMINATE,
    SM_EXIT
};

// UDP Return Codes
enum UDP_RETURN_CODES {
    UDP_DATA_IS_AVAILABLE = 0,
    UDP_DATA_IS_NOT_AVAILABLE_WITHIN_TIMEOUT = -1,
    UDP_SELECT_ERROR = -2,
    UDP_CONNECTION_CLOSED_ERROR = -3,
    UDP_SOCKET_ERROR = -4,
    UDP_NON_UDP_DATA_IS_AVAILABLE = -5
};

// Socket Data struct
struct UDP_Info {
    int socket;
    struct sockaddr_in Addr;
    socklen_t AddrLen;
} udp_host; // this is global bc there will only be one

struct ORT_Object {
    Ort::Env env;
    Ort::Session session;
    Ort::MemoryInfo memory_info;
    std::vector<int64_t> input_shape;
    Ort::AllocatedStringPtr input_name;
    Ort::AllocatedStringPtr output_name;

    ORT_Object(const std::string& model_path, int num_threads = 2)
        : env(ORT_LOGGING_LEVEL_WARNING, "contact_lstm")
        , session(env, model_path.c_str(), [&]{ Ort::SessionOptions o; o.SetIntraOpNumThreads(num_threads); return o; }())
        , memory_info(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault))
        , input_shape({1, 1, 12})
        , input_name(session.GetInputNameAllocated(0, Ort::AllocatorWithDefaultOptions{}))
        , output_name(session.GetOutputNameAllocated(0, Ort::AllocatorWithDefaultOptions{}))
    {}
};

std::optional<ORT_Object> ORT;

bool contact_detection_prediction(AmpIO *board, ORT_Object& ort, const float* encoder_vel, const float* torque_feedback){

    uint8_t num_encoders = 6;
    uint8_t num_motors = 6;

    // lets combine the enc velocity and torque into one vector

    std::vector<float> input(num_encoders + num_motors);

    for (int i=0; i<num_encoders; i++){
        input[i] = encoder_vel[i];
    }

    for (int i=num_encoders; i < num_encoders + num_motors; i++){
        input[i] = torque_feedback[i - num_encoders];
    }

    // normalization stats from training config (normalize=true)
    const std::array<float, 12> feat_mean = {
        -0.0002824742114171386f, -3.650841608759947e-05f,  1.22635419756989e-05f,
         0.001427539624273777f,  -8.851468010107055e-05f,  0.0002031530166277662f,
        -0.6210437417030334f,    3.9810116291046143f,      -4.167459487915039f,
        -0.003561527468264103f,   0.006435718387365341f,    0.03592592105269432f
    };
    const std::array<float, 12> feat_std = {
        0.2544010877609253f,  0.1120990440249443f,  0.010202210396528244f,
        0.2805119752883911f,  0.25634005665779114f, 0.6305540204048157f,
        1.3269498348236084f,  1.6260298490524292f,  5.9530720710754395f,
        0.030428461730480194f, 0.03421192616224289f, 0.04092755541205406f
    };

    std::array<float, 12> normalized;
    for (int i = 0; i < 12; ++i)
        normalized[i] = (input[i] - feat_mean[i]) / feat_std[i];


    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        ort.memory_info,
        normalized.data(),
        normalized.size(),
        ort.input_shape.data(),
        ort.input_shape.size()
    );

    const char* input_names[] = {ort.input_name.get()};
    const char* output_names[] = {ort.output_name.get()};

    auto outputs = ort.session.Run(Ort::RunOptions{nullptr}, input_names, &input_tensor, 1, output_names, 1);

    float* output = outputs[0].GetTensorMutableData<float>();

    float contact_prob = output[0];
    int contact_pred = contact_prob >= 0.5f ? 1 : 0;

    // cout << "prediction : " << (bool) contact_pred << endl;

    return (bool) contact_pred;

}


static int mio_mmap_init()
{
    int mem_fd;

    // Open /dev/mem for accessing physical memory
    if ((mem_fd = open("/dev/mem", O_RDWR | O_SYNC)) < 0) {
        cout << "Failed to open /dev/mem" << endl;
        return -1;
    }

    void *gpio_mmap = mmap(
        NULL,
        0x1000,
        PROT_READ | PROT_WRITE,
        MAP_SHARED,
        mem_fd,
        GPIO_BASE_ADDR
    );

    if (gpio_mmap == MAP_FAILED) {
        perror("Failed to mmap");
        close(mem_fd);  // Always close the file descriptor on failure
        return -1;
    }

    GPIO_MEM_REGION = (volatile uint32_t * ) gpio_mmap;

    // Following code ensures that APER_CLK is enabled; should not be necessary
    // since fpgav3_emio_mmap also does this.
    void *clk_map = mmap(
        NULL,
        0x00000130,
        PROT_READ | PROT_WRITE,
        MAP_SHARED,
        mem_fd,
        SCLR_CLK_BASE_ADDR
    );

    volatile unsigned long *clock_map = (volatile unsigned long *)clk_map;

    uint32_t bitmsk = (1 << 22);
    uint32_t aper_clk_reg = clock_map[0x12C/4];

    if ((aper_clk_reg & bitmsk) == 0) {
        clock_map[0x12C/4] |= bitmsk;
    }

    munmap(clk_map, 0x00000130);
    // End of APER_CLK check

    close(mem_fd);

    return 0;
}

uint8_t returnMIOPins(){

    if (GPIO_MEM_REGION == NULL) {
        cout << "[ERROR] MIO mmap region initialized incorrectly!" << endl;
        return 0;
    }

    const uint16_t MIO_PINS_MSK = 0x3C;
    uint32_t gpio_bank1 = GPIO_MEM_REGION[GPIO_BANK1_OFFSET/4];

    return (gpio_bank1 & MIO_PINS_MSK) >> 2;
}

// checks if data is available from udp buffer (for noblocking udp recv)
int udp_nonblocking_receive(UDP_Info *udp_host, void *data, int size)
{
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(udp_host->socket, &readfds);

    int ret_code;

    struct timeval timeout;

    // Timeout values
    timeout.tv_sec = 0;
    timeout.tv_usec = 0;

    int max_fd = udp_host->socket + 1;
    int activity = select(max_fd, &readfds, NULL, NULL, &timeout);

    if (activity < 0) {
        return UDP_SELECT_ERROR;
    } else if (activity == 0) {
        return UDP_DATA_IS_NOT_AVAILABLE_WITHIN_TIMEOUT;
    } else {
        if (FD_ISSET(udp_host->socket, &readfds)) {
            ret_code = recvfrom(udp_host->socket, data, size, 0, (struct sockaddr *)&udp_host->Addr, &udp_host->AddrLen);

            if (ret_code == 0) {
                return UDP_CONNECTION_CLOSED_ERROR;
            } else if (ret_code < 0) {
                return UDP_SOCKET_ERROR;
            } else {
                return ret_code; // Return the number of bytes received
            }
        }
        else {
            return UDP_NON_UDP_DATA_IS_AVAILABLE;
        }
    }
}

// udp transmit function. wrapper for sendo that abstracts the UDP_Info_struct
static int udp_transmit(UDP_Info *udp_host, void * data, int size)
{

    if (size > UDP_MAX_PACKET_SIZE) {
        return -1;
    }

    return sendto(udp_host->socket, data, size, 0, (struct sockaddr *)&udp_host->Addr, udp_host->AddrLen);
}


static bool initiate_socket_connection(int &host_socket)
{
    cout << endl << "Initiating Socket Connection with host..." << endl;

    udp_host.AddrLen = sizeof(udp_host.Addr);

    // Create a UDP socket
    host_socket = socket(AF_INET, SOCK_DGRAM, 0);

    if (host_socket < 0) {
        cerr << "[UDP ERROR] Failed to create socket [" << host_socket << "]" << endl;
        return false;
    }

    struct sockaddr_in serverAddr;
    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(12345);
    serverAddr.sin_addr.s_addr = INADDR_ANY;

    if (bind(host_socket, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) < 0) {
        cerr << "[UDP ERROR] Failed to bind socket" << endl;
        close(host_socket);
        return false;
    }

    cout << "UDP Connection Success !" << endl << endl;
    return true;
}

// calculate the size of a sample in quadlets
static uint16_t calculate_quadlets_per_sample(uint8_t num_encoders, uint8_t num_motors)
{
    // SAMPLE STRUCTURE

    // 1 quadlet = 4 bytes

    // Timestamp (Double -> 64 bits stored as two separate quadlets)               [2 quadlets]
    // Encoder Position (32 * num of encoders)                                      [1 quadlet * num of encoders]
    // Encoder Velocity Predicted (64 * num of encoders -> truncated to 32bits)     [1 quadlet * num of encoders]
    // Motor Current and Motor Status (32 * num of Motors -> each are 16 bits)      [1 quadlet * num of motors]
    //   (SI mode: motor_torque + cmd_torque each as float -> 2 quadlets per motor)
    // Digital IO Values  (optional, used if PS IO is enabled) 32 bits             [1 quadlet * digital IO]
    // MIO Pins (optional, used if PS IO is enabled) 4 bits -> pad 32 bits         [1 quadlet * MIO PINS]
    // POT pins (optional, used if Pot is enabled) num_motors * 16 bits

    int motor_quadlets = use_si_units ? (2 * num_motors) : num_motors;
    int default_quadlets_per_sample = (2 + (2*(num_encoders)) + motor_quadlets);
    int quadlets_per_sample = default_quadlets_per_sample;

    if (use_ps_io_flag){
        quadlets_per_sample += 2;
    }

    if (use_pot_flag) {
        quadlets_per_sample += (1 * num_motors);
    }

    if (use_contact_model){
        quadlets_per_sample += 1;
    }

    return quadlets_per_sample;
}

// calculates the # of samples per packet in quadlets
static uint16_t calculate_samples_per_packet(uint8_t num_encoders, uint8_t num_motors)
{
    return ((UDP_MAX_PACKET_SIZE/4)/ calculate_quadlets_per_sample(num_encoders, num_motors) );
}

// calculate # of quadlets per packet
static uint16_t calculate_quadlets_per_packet(uint8_t num_encoders, uint8_t num_motors)
{
    return (calculate_samples_per_packet(num_encoders, num_motors) * calculate_quadlets_per_sample(num_encoders, num_motors));
}

static double ts_diff_s(const timespec &start, const timespec &end) {
    time_t  dsec  = end.tv_sec  - start.tv_sec;
    long    dnsec = end.tv_nsec - start.tv_nsec;
    return double(dsec) + double(dnsec) * 1e-9;
}


// loads data buffer for data collection
    // size of the data buffer is dependent on encoder count and motor count
    // see calculate_quadlets_per_sample method for data formatting
static bool load_data_packet(Dvrk_Controller dvrk_controller, uint32_t *data_packet, uint8_t num_encoders, uint8_t num_motors)
{

    if (data_packet == NULL) {
        cout << "[ERROR - load_data_packet] databuffer pointer is null" << endl;
        return false;
    }

    if (sizeof(data_packet) == 0) {
        cout << "[ERROR - load_data_packet] len of databuffer == 0" << endl;
        return false;
    }

    uint16_t samples_per_packet = calculate_samples_per_packet(num_encoders, num_motors);
    uint16_t count = 0;

    // printf("inside load data packet\n");

    // CAPTURE DATA
    for (int j = 0; j < samples_per_packet; j++) {

        timespec t0;
        clock_gettime(CLOCK_MONOTONIC_RAW, &t0);

        if (!dvrk_controller.Port->ReadAllBoards()) {
            emio_read_error_counter++;
            return false;
        }

        if (!dvrk_controller.Board->ValidRead()) {
            cout << "[ERROR in load_data_packet] invalid read for ReadAllBoards" << endl;
            return false;
        }

        double time_elapsed = ts_diff_s(t_data_collection_start, t0);

        last_timestamp = time_elapsed;

        uint64_t timestamp_uint64 = *reinterpret_cast<uint64_t *>(&time_elapsed);

        uint32_t timestamp_high = (timestamp_uint64  >> 32);
        uint32_t timestamp_low =  (uint32_t) (timestamp_uint64 & 0xFFFFFFFF);

        data_packet[count++] = timestamp_high;
        data_packet[count++] = timestamp_low;

        // DATA 2: encoder position
        for (int i = 0; i < num_encoders; i++) {
            int32_t encoder_pos = dvrk_controller.Board->GetEncoderPosition(i) + dvrk_controller.Board->GetEncoderMidRange();

            if (use_si_units) {
                float encoder_pos_si = convert_enc_pos_to_si_units(cfg, encoder_pos, i);
                data_packet[count++] = *reinterpret_cast<uint32_t*>(&encoder_pos_si);
            } else {
                data_packet[count++] = static_cast<uint32_t>(encoder_pos + dvrk_controller.Board->GetEncoderMidRange());
            }
        }

        // DATA 3: encoder velocity
        for (int i = 0; i < num_encoders; i++) {
            float encoder_velocity_float = static_cast<float>(dvrk_controller.Board->GetEncoderVelocityPredicted(i));

            float encoder_velocity_si = convert_enc_vel_to_si_units(cfg, encoder_velocity_float, i);
            contact_velocity[i] = encoder_velocity_si;
            if (use_si_units) {
                data_packet[count++] = *reinterpret_cast<uint32_t *>(&encoder_velocity_si);
            } else {
                data_packet[count++] = *reinterpret_cast<uint32_t *>(&encoder_velocity_float);
            }
        }

        // DATA 4 & 5: motor current and commanded current
        for (int i = 0; i < num_motors; i++) {
            uint32_t raw_quadlet;
            // dvrk_controller.Port->ReadQuadlet(dvrk_controller.Port->GetBoardId(0), ((i+1) << 4) | 1, raw_quadlet);

            uint16_t motor_curr = dvrk_controller.Board->GetMotorCurrent(i); // (uint16_t)(raw_quadlet & 0x0000FFFF);
            uint16_t cmd_curr   = 0; // (uint16_t)((raw_quadlet >> 16) & 0x0000FFFF);

            float motor_torque = convert_torque_to_si_units(cfg, motor_curr, i);
            contact_torque[i] = motor_torque;
            if (use_si_units) {
                float cmd_torque = convert_torque_command_to_si_units(cfg, cmd_curr, i);
                data_packet[count++] = *reinterpret_cast<uint32_t*>(&motor_torque);
                data_packet[count++] = *reinterpret_cast<uint32_t*>(&cmd_torque);
            } else {
                data_packet[count++] = (uint32_t)(((uint32_t)cmd_curr << 16) | motor_curr);
            }
        }

        if (use_contact_model){

            // printf("using contact model\n");

            uint32_t contact_pred = (uint32_t) contact_detection_prediction(dvrk_controller.Board, *ORT, contact_velocity, contact_torque);

            // printf("we good ?\n");
            data_packet[count++] = contact_pred;
        }

        if (use_ps_io_flag){
            data_packet[count++] = dvrk_controller.Board->ReadDigitalIO();
            data_packet[count++] = (uint32_t) returnMIOPins();
        }

        if (use_pot_flag){
            for (int i = 0; i < num_motors; i++) {
                data_packet[count++] = dvrk_controller.Board->GetAnalogInput(i);
            }
        }

        if (useSampleRate){
            deadline.tv_nsec += period_ns;
            if (deadline.tv_nsec >= 1'000'000'000) {
                deadline.tv_sec++;
                deadline.tv_nsec -= 1'000'000'000;
            }

            // busy spin until deadline
            timespec now;
            do {
                clock_gettime(CLOCK_MONOTONIC_RAW, &now);
            } while ((  now.tv_sec  < deadline.tv_sec) ||
                        (now.tv_sec == deadline.tv_sec && now.tv_nsec < deadline.tv_nsec));
        }

        sample_count++;
    }

    return true;
}

void package_meta_data(DataCollectionMeta *dc_meta, AmpIO *board)
{
    uint8_t num_encoders = (uint8_t) board->GetNumEncoders();
    uint8_t num_motors = (uint8_t) board->GetNumMotors();

    dc_meta->hwvers = board->GetHardwareVersion();
    dc_meta->num_encoders = (uint32_t) num_encoders;
    dc_meta->num_motors = (uint32_t) num_motors;

    dc_meta->data_packet_size = (uint32_t)calculate_quadlets_per_packet(num_encoders, num_motors) * 4;
    dc_meta->size_of_sample = (uint32_t) calculate_quadlets_per_sample(num_encoders, num_motors);
    dc_meta->samples_per_packet = (uint32_t) calculate_samples_per_packet(num_encoders, num_motors);
}

void reset_double_buffer_info(Double_Buffer_Info *db, AmpIO *board)
{
    db->cons_buf = 0;
    db->prod_buf = 0;
    db->cons_busy = 0;
    db->buffer_size = calculate_quadlets_per_packet(board->GetNumEncoders(), board->GetNumMotors()) * 4;

    memset(db->double_buffer, 0, sizeof(db->double_buffer));
}

void *consume_data(void *arg)
{
    Double_Buffer_Info* db = (Double_Buffer_Info*)arg;

    while (!stop_data_collection_flag) {

        if (db->prod_buf != db->cons_buf) {

            db->cons_busy = 1;
            udp_transmit(&udp_host, db->double_buffer[db->cons_buf], db->buffer_size);
            data_packet_count++;
            db->cons_busy = 0;

            db->cons_buf = (db->cons_buf + 1) % 2;
        }
    }

    return nullptr;
}

SM wait_for_host_handshake( SM sm ){
    memset(recvd_cmd, 0, CMD_MAX_STRING_SIZE);
    sm.udp_ret = udp_nonblocking_receive(&udp_host, recvd_cmd, CMD_MAX_STRING_SIZE);

    if (sm.udp_ret > 0) {
        if (strcmp(recvd_cmd,  HOST_READY_CMD) == 0) {
            cout << "Received Message - " <<  HOST_READY_CMD << endl;
            sm.state = SM_WAIT_FOR_HOST_FLAG_CMD;
        }
        else {
            sm.ret = SM_OUT_OF_SYNC;
            sm.last_state = sm.state;
            sm.state = SM_TERMINATE;
        }
    }
    else if (sm.udp_ret == UDP_DATA_IS_NOT_AVAILABLE_WITHIN_TIMEOUT || sm.udp_ret == UDP_NON_UDP_DATA_IS_AVAILABLE) {
        sm.state = SM_WAIT_FOR_HOST_HANDSHAKE;
    }
    else {
        sm.ret = SM_UDP_ERROR;
        sm.last_state = sm.state;
        sm.state = SM_TERMINATE;
    }

    return sm;
}

SM wait_for_host_flag_cmd(SM sm){
    memset(recvd_cmd, 0, CMD_MAX_STRING_SIZE);
    sm.udp_ret = udp_nonblocking_receive(&udp_host, recvd_cmd, CMD_MAX_STRING_SIZE);

    if (sm.udp_ret > 0) {
        if (strcmp(recvd_cmd, HOST_FLAG_CMD) == 0) {
            cout << "Received Message - " << HOST_FLAG_CMD << endl;
            sm.state = SM_WAIT_FOR_HOST_FLAG_VALUE;
        } else {
            sm.ret = SM_OUT_OF_SYNC;
            sm.last_state = sm.state;
            sm.state = SM_TERMINATE;
        }
    }
    else if (sm.udp_ret == UDP_DATA_IS_NOT_AVAILABLE_WITHIN_TIMEOUT || sm.udp_ret == UDP_NON_UDP_DATA_IS_AVAILABLE) {
        sm.state = SM_WAIT_FOR_HOST_FLAG_CMD;
    }
    else {
        sm.ret = SM_UDP_ERROR;
        sm.last_state = sm.state;
        sm.state = SM_TERMINATE;
    }

    return sm;
}

static RobotConfig parse_robot_config_from_json_str(const char* json_str) {
    nlohmann::json j = nlohmann::json::parse(json_str);
    RobotConfig result;
    result.actuators.resize(j.size());

    for (size_t i = 0; i < j.size(); i++) {
        auto& actuator = result.actuators[i];
        const auto& actuator_json = j[i];

        if (actuator_json.contains("Enc_B2P")) {
            actuator.Enc_B2P.Scale = actuator_json["Enc_B2P"].value("Scale", 0.0);
            actuator.Enc_B2P.Offset = actuator_json["Enc_B2P"].value("Offset", 0.0);
        } else if (actuator_json.contains("es")) {
            actuator.Enc_B2P.Scale = actuator_json.value("es", 0.0);
            actuator.Enc_B2P.Offset = actuator_json.value("eo", 0.0);
        } else {
            actuator.Enc_B2P.Scale = actuator_json.value("enc_scale", 0.0);
            actuator.Enc_B2P.Offset = actuator_json.value("enc_offset", 0.0);
        }

        if (actuator_json.contains("Curr_B2C")) {
            actuator.Curr_B2C.Scale = actuator_json["Curr_B2C"].value("Scale", 0.0);
            actuator.Curr_B2C.Offset = actuator_json["Curr_B2C"].value("Offset", 0.0);
        } else if (actuator_json.contains("b2cs")) {
            actuator.Curr_B2C.Scale = actuator_json.value("b2cs", 0.0);
            actuator.Curr_B2C.Offset = actuator_json.value("b2co", 0.0);
        } else {
            actuator.Curr_B2C.Scale = actuator_json.value("cur_scale", 0.0);
            actuator.Curr_B2C.Offset = actuator_json.value("cur_offset", 0.0);
        }

        if (actuator_json.contains("Curr_C2B")) {
            actuator.Curr_C2B.Scale = actuator_json["Curr_C2B"].value("Scale", 1.0);
            actuator.Curr_C2B.Offset = actuator_json["Curr_C2B"].value("Offset", 0.0);
        } else if (actuator_json.contains("c2bs")) {
            actuator.Curr_C2B.Scale = actuator_json.value("c2bs", 1.0);
            actuator.Curr_C2B.Offset = actuator_json.value("c2bo", 0.0);
        } else {
            actuator.Curr_C2B.Scale = actuator_json.value("curr_c2b_scale", 1.0);
            actuator.Curr_C2B.Offset = actuator_json.value("curr_c2b_offset", 0.0);
        }

        if (actuator_json.contains("Curr_Nm2C")) {
            actuator.Curr_Nm2C.Scale = actuator_json["Curr_Nm2C"].value("Scale", 1.0);
            actuator.Curr_Nm2C.Offset = actuator_json["Curr_Nm2C"].value("Offset", 0.0);
        } else if (actuator_json.contains("n2cs")) {
            actuator.Curr_Nm2C.Scale = actuator_json.value("n2cs", 1.0);
            actuator.Curr_Nm2C.Offset = actuator_json.value("n2co", 0.0);
        } else {
            actuator.Curr_Nm2C.Scale = actuator_json.value("curr_nm2c_scale", 1.0);
            actuator.Curr_Nm2C.Offset = actuator_json.value("curr_nm2c_offset", 0.0);
        }

        if (actuator_json.contains("Pot_B2V")) {
            actuator.Pot_B2V.Scale = actuator_json["Pot_B2V"].value("Scale", 0.0);
            actuator.Pot_B2V.Offset = actuator_json["Pot_B2V"].value("Offset", 0.0);
        } else {
            actuator.Pot_B2V.Scale = actuator_json.value("pot_b2v_scale", 0.0);
            actuator.Pot_B2V.Offset = actuator_json.value("pot_b2v_offset", 0.0);
        }

        if (actuator_json.contains("Pot_V2P")) {
            actuator.Pot_V2P.Scale = actuator_json["Pot_V2P"].value("Scale", 0.0);
            actuator.Pot_V2P.Offset = actuator_json["Pot_V2P"].value("Offset", 0.0);
        } else {
            actuator.Pot_V2P.Scale = actuator_json.value("pot_v2p_scale", 0.0);
            actuator.Pot_V2P.Offset = actuator_json.value("pot_v2p_offset", 0.0);
        }
        actuator.midrange = std::pow(2.0, actuator_json.value("eb", actuator_json.value("enc_bits", 24)) - 1);
        actuator.unit = actuator_json.value("u", actuator_json.value("unit", M_PI / 180.0));
    }

    return result;
}

SM wait_for_host_flag_value(SM sm){
    uint8_t flag_cmd = 0x00;
    sm.udp_ret = udp_nonblocking_receive(&udp_host, &flag_cmd, sizeof(flag_cmd));

    if (sm.udp_ret > 0) {
        use_ps_io_flag      = (flag_cmd & ENABLE_PSIO_MSK);
        use_pot_flag        = (flag_cmd & ENABLE_POT_MSK);
        useSampleRate       = (flag_cmd & ENABLE_SAMPLE_RATE_MSK);
        use_si_units        = (flag_cmd & ENABLE_SI_UNITS_MSK);
        use_contact_model   = (flag_cmd & ENABLE_CONTACT_DETECTION_MSK);

        if (use_contact_model && !ORT.has_value()) {
            const char* model_path = "/media/contact_model/contact_lstm_model_working.onnx";
            if (access(model_path, R_OK) != 0) {
                perror("[ERROR] Cannot open contact model");
                use_contact_model = false;
            } else {
                ORT.emplace(model_path);
            }
        }

        cout << "Received Flag Byte: 0x" << std::hex << static_cast<int>(flag_cmd) << std::dec << endl;

        if (useSampleRate) {
            int host_sample_rate = 0;
            do {
                sm.udp_ret = udp_nonblocking_receive(&udp_host, &host_sample_rate, sizeof(host_sample_rate));
            } while (sm.udp_ret <= 0);

            SAMPLE_RATE = host_sample_rate;
            printf("NEW SAMPLE RATE: %d\n", SAMPLE_RATE);
            use_ps_io_flag = true;
        }

        if (use_si_units) {
            char json_buf[4096] = {0};
            do {
                sm.udp_ret = udp_nonblocking_receive(&udp_host, json_buf, sizeof(json_buf));
            } while (sm.udp_ret <= 0);

            cfg = parse_robot_config_from_json_str(json_buf);
            cout << "Received robot config with " << cfg.actuators.size() << " actuators" << endl;
        }

        if (use_ps_io_flag || use_pot_flag) {
            reset_double_buffer_info(&db, dvrk_controller.Board);
        }

        sm.state = SM_SEND_DATA_COLLECTION_METADATA;
    }
    else if (sm.udp_ret == UDP_DATA_IS_NOT_AVAILABLE_WITHIN_TIMEOUT || sm.udp_ret == UDP_NON_UDP_DATA_IS_AVAILABLE) {
        sm.state = SM_WAIT_FOR_HOST_FLAG_VALUE;
    }
    else {
        sm.ret = SM_UDP_ERROR;
        sm.last_state = sm.state;
        sm.state = SM_TERMINATE;
    }

    return sm;
}

SM send_data_collection_meta_data( SM sm ){
    package_meta_data(&data_collection_meta, dvrk_controller.Board);

    if (udp_transmit(&udp_host,  &data_collection_meta, sizeof(struct DataCollectionMeta )) < 1 ) {
        sm.ret = SM_UDP_INVALID_HOST_ADDR;
        sm.last_state = SM_TERMINATE;
    }
    else {
        sm.state = SM_WAIT_FOR_HOST_RECV_METADATA;
    }

    return sm;
}

SM wait_for_host_to_recv_metadata( SM sm ){
    memset(recvd_cmd, 0, CMD_MAX_STRING_SIZE);
    sm.udp_ret = udp_nonblocking_receive(&udp_host, recvd_cmd, CMD_MAX_STRING_SIZE);

    if (sm.udp_ret > 0) {
        if (strcmp(recvd_cmd, HOST_RECVD_METADATA) == 0) {
            cout << "Received Message: " << HOST_RECVD_METADATA << endl;
            cout << "Handshake Complete!" << endl;

            sm.state = SM_SEND_READY_STATE_TO_HOST;
        } else {
            sm.ret = SM_OUT_OF_SYNC;
            sm.last_state = sm.state;
            sm.state = SM_TERMINATE;
        }
    }
    else if (sm.udp_ret == UDP_DATA_IS_NOT_AVAILABLE_WITHIN_TIMEOUT || sm.udp_ret == UDP_NON_UDP_DATA_IS_AVAILABLE) {
        // Stay in same state
        sm.state = SM_WAIT_FOR_HOST_RECV_METADATA;
    } else {
        sm.ret = SM_UDP_ERROR;
        sm.last_state = sm.state;
        sm.state = SM_TERMINATE;
    }

    return sm;
}

SM send_ready_state_to_host( SM sm ){
    if (udp_transmit(&udp_host,  (char *) ZYNQ_READY_CMD, sizeof(ZYNQ_READY_CMD)) < 1 ) {
        sm.ret = SM_UDP_INVALID_HOST_ADDR;
        sm.state = SM_TERMINATE;
    }
    else {
        sm.state = SM_WAIT_FOR_HOST_START_CMD;
        cout << endl << "Waiting for Host to start data collection..." << endl << endl;
    }

    return sm;
}

SM wait_for_host_start_cmd( SM sm ){
    memset(recvd_cmd, 0, CMD_MAX_STRING_SIZE);
    sm.udp_ret = udp_nonblocking_receive(&udp_host, recvd_cmd, CMD_MAX_STRING_SIZE);

    if (sm.udp_ret > 0) {
        if (strcmp(recvd_cmd, HOST_START_DATA_COLLECTION) == 0) {
            cout << "Received Message: " <<  recvd_cmd << endl;
            sm.state = SM_START_DATA_COLLECTION;
        }
        else if (strcmp(recvd_cmd, HOST_TERMINATE_SERVER) == 0) {
            cout << "Received Message: " <<  recvd_cmd << endl;
            sm.ret = SM_SUCCESS;
            sm.state = SM_TERMINATE;
        }
        else {
            sm.ret = SM_OUT_OF_SYNC;
            sm.last_state = sm.state;
            sm.state = SM_TERMINATE;
        }
    }
    else if (sm.udp_ret == UDP_DATA_IS_NOT_AVAILABLE_WITHIN_TIMEOUT || sm.udp_ret == UDP_NON_UDP_DATA_IS_AVAILABLE) {
        sm.state = SM_WAIT_FOR_HOST_START_CMD;
    }
    else {
        sm.ret = SM_UDP_ERROR;
        sm.last_state = sm.state;
        sm.state = SM_TERMINATE;
    }

    return sm;
}

SM start_consumer_thread( SM sm , pthread_t *consumer_t){
    // Starting Consumer Thread: sends packets to host
    if (pthread_create(consumer_t, nullptr, consume_data, &db) != 0) {
        std::cerr << "Error creating consumer thread" << std::endl;
        sm.state = SM_EXIT;
        sm.ret = SM_FAILED_TO_CREATE_THREAD;
    }

    pthread_detach(*consumer_t);

    sm.state = SM_PRODUCE_DATA;

    return sm;
}

SM produce_data( SM sm ){

    if ( !load_data_packet(dvrk_controller, db.double_buffer[db.prod_buf], data_collection_meta.num_encoders, data_collection_meta.num_motors)) {
        cout << "[ERROR]load data buffer fail" << endl;
        sm.state = SM_EXIT;
        sm.ret = SM_BOARD_ERROR;
        return sm;
    }

    while (db.cons_busy) {}

    // Switch to the next buffer
    db.prod_buf = (db.prod_buf + 1) % 2;

    sm.state = SM_CHECK_FOR_STOP_DATA_COLLECTION_CMD;

    return sm;
}

SM check_for_stop_data_collection(SM sm, pthread_t consumer_t){

    sm.udp_ret = udp_nonblocking_receive(&udp_host, recvd_cmd, CMD_MAX_STRING_SIZE);

    if (sm.udp_ret > 0) {
        if (strcmp(recvd_cmd, HOST_STOP_DATA_COLLECTION) == 0) {
            cout << "Message from Host: STOP DATA COLLECTION" << endl;

            stop_data_collection_flag = true;

            pthread_join(consumer_t, nullptr);

            cout << "------------------------------------------------" << endl;
            cout << "UDP DATA PACKETS SENT TO HOST: " << data_packet_count << endl;
            cout << "SAMPLES SENT TO HOST: " << sample_count << endl;
            cout << "EMIO ERROR COUNT: " << emio_read_error_counter << endl;
            cout << "TIME ELAPSED: " << last_timestamp << endl;
            cout << "AVERAGE SAMPLE RATE: " << (float) (sample_count / last_timestamp) << "Hz" << endl;
            cout << "------------------------------------------------" << endl << endl;

            emio_read_error_counter = 0;
            data_packet_count = 0;
            sample_count = 0;

            sm.state = SM_WAIT_FOR_HOST_START_CMD;
            cout << "Waiting for command from host..." << endl;

        } else {
            sm.ret = SM_OUT_OF_SYNC;
            sm.last_state = sm.state;
            sm.state = SM_TERMINATE;
        }
    } else if (sm.udp_ret == UDP_DATA_IS_NOT_AVAILABLE_WITHIN_TIMEOUT || sm.udp_ret == UDP_NON_UDP_DATA_IS_AVAILABLE){
        sm.state = SM_PRODUCE_DATA;
    } else {
        sm.ret = SM_UDP_ERROR;
        sm.last_state = sm.state;
        sm.state = SM_TERMINATE;
    }

    return sm;
}

SM start_data_collection(SM sm){

    stop_data_collection_flag = false;
    clock_gettime(CLOCK_MONOTONIC_RAW, &t_data_collection_start);

    if (useSampleRate){
        clock_gettime(CLOCK_MONOTONIC_RAW, &deadline);
        // compute the nanoseconds between samples
        period_ns = 1'000'000'000L / SAMPLE_RATE;
    }

    sm.state = SM_START_CONSUMER_THREAD;
    return sm;
}

SM terminate_data_collection(SM sm){

    if (sm.ret != SM_SUCCESS) {

        cout << "[ERROR] STATEMACHINE TERMINATING" << endl;

        cout << "At STATE " << sm.last_state << " ";

        switch (sm.ret){

            case SM_OUT_OF_SYNC:
                cout << "Zynq of sync with Host. Received unexpected command: " << recvd_cmd << endl;
                break;
            case SM_UDP_ERROR:
                cout << "Udp ERROR. Make sure host program is running." << endl;
                break;
            case SM_UDP_INVALID_HOST_ADDR:
                cout << "Udp ERROR. Invalid Host Address format" << endl;
                break;
            case SM_FAILED_TO_CREATE_THREAD:
                cout << "Failed to Create Thread" << endl;
            case SM_BOARD_ERROR:
                cout << "Board Error" << endl;
        }

    } else {
        cout << "STATE MACHINE SUCCESS !" << endl;
    }

    cout << endl << ZYNQ_TERMINATATION_SUCCESSFUL << endl;

    udp_transmit(&udp_host,  (void*) ZYNQ_TERMINATATION_SUCCESSFUL, sizeof(ZYNQ_TERMINATATION_SUCCESSFUL));

    close(udp_host.socket);
    sm.state = SM_EXIT;

    return sm;
}


static int dataCollectionStateMachine()
{
    SM sm;
    pthread_t consumer_t;

    cout << "Starting Handshake Routine..." << endl << endl;
    cout << "Start Data Collection Client on HOST to complete handshake..." << endl;

    reset_double_buffer_info(&db, dvrk_controller.Board);

    if (mio_mmap_init() != 0) {
        sm.state = SM_TERMINATE;
        sm.ret = SM_PS_IO_FAIL;
    }

    if (use_contact_model){
        const char* model_path = "/media/contact_model/contact_detection_model_working.onnx";
        if (access(model_path, R_OK) != 0) {
            perror("[ERROR] Cannot open contact model");
            return -1;
        }
        ORT.emplace(model_path);
    }
    

    sm.state = SM_WAIT_FOR_HOST_HANDSHAKE;

    while (sm.state != SM_EXIT) {

        switch (sm.state) {
            case SM_WAIT_FOR_HOST_HANDSHAKE:
                sm = wait_for_host_handshake(sm);
                break;

            case SM_WAIT_FOR_HOST_FLAG_CMD:
                sm = wait_for_host_flag_cmd(sm);
                break;

            case SM_WAIT_FOR_HOST_FLAG_VALUE:
                sm = wait_for_host_flag_value(sm);
                break;

            case SM_SEND_DATA_COLLECTION_METADATA:
                sm = send_data_collection_meta_data(sm);
                break;

            case SM_WAIT_FOR_HOST_RECV_METADATA:
                sm = wait_for_host_to_recv_metadata(sm);
                break;

            case SM_SEND_READY_STATE_TO_HOST:
                sm = send_ready_state_to_host(sm);
                break;

            case SM_WAIT_FOR_HOST_START_CMD:
                sm = wait_for_host_start_cmd(sm);
                break;

            case SM_START_DATA_COLLECTION:
                sm = start_data_collection(sm);
                break;

            case SM_START_CONSUMER_THREAD:
                sm = start_consumer_thread(sm, &consumer_t);
                break;

            case SM_PRODUCE_DATA:
                sm = produce_data(sm);
                break;

            case SM_CHECK_FOR_STOP_DATA_COLLECTION_CMD:
                sm = check_for_stop_data_collection(sm, consumer_t);
                break;

            case SM_TERMINATE:
                sm = terminate_data_collection(sm);
                break;
        }
    }
    return sm.ret;
}



int main()
{
    string portDescription = BasePort::DefaultPort();
    dvrk_controller.Port = PortFactory(portDescription.c_str());

    if (!dvrk_controller.Port->IsOK()) {
        std::cerr << "Failed to initialize " << dvrk_controller.Port->GetPortTypeString() << std::endl;
        return -1;
    }

    if (dvrk_controller.Port->GetNumOfNodes() == 0) {
        std::cerr << "Failed to find any boards" << std::endl;
        return -1;
    }

    ZynqEmioPort *EmioPort = dynamic_cast<ZynqEmioPort *>(dvrk_controller.Port);
    if (EmioPort) {
        cout << "Verbose: " << EmioPort->GetVerbose() << std::endl;
        // EmioPort->SetVerbose(true);
        EmioPort->SetTimeout_us(80);
    }
    else {
      cout << "[warning] failed to dynamic cast to ZynqEmioPort" << endl;
    }

    dvrk_controller.Board = new AmpIO(dvrk_controller.Port->GetBoardId(0));

    dvrk_controller.Port->AddBoard(dvrk_controller.Board);

    bool isOK = initiate_socket_connection(udp_host.socket);

    if (!isOK) {
        cout << "[error] failed to establish socket connection !!" << endl;
        return -1;
    }

    dataCollectionStateMachine();

    return 0;
}