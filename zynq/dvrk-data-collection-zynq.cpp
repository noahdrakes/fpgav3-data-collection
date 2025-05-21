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
#include <netdb.h>
#include <arpa/inet.h>


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

enum FS_RETURN_CODES  {
    FS_SUCCESS,
    FS_NO_DATA_AVAILABLE,
    FS_FAIL
};

// Socket Data struct
struct UDP_Info {
    int socket;
    struct sockaddr_in Addr;
    socklen_t AddrLen;
};

struct UDP_Info udp_host, udp_fs;

// FORCE SENSOR 

typedef struct response_struct {
	uint64_t rdt_sequence;
	uint64_t ft_sequence;
	uint64_t status;
} RESPONSE;

// struct sockaddr_in addr;	/* Address of Net F/T. */
// struct hostent *he;			/* Host entry for Net F/T. */
struct sockaddr_in addr;	/* Address of Net F/T. */
struct hostent *he;			/* Host entry for Net F/T. */
unsigned char request[8];			/* The request data sent to the Net F/T. */
RESPONSE resp;				/* The structured response received from the Net F/T. */

float last_fs_sample[FORCE_SAMPLE_NUM_DEGREES] = {0,0,0};
float FS_X_BIAS = 0;
float FS_Y_BIAS = 0;
float FS_Z_BIAS = 0;


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

// int udp_nonblocking_recieve()

// checks if data is available from udp buffer (for noblocking udp recv)
int udp_nonblocking_receive(UDP_Info *udp_info, void *data, int size)
{
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(udp_info->socket, &readfds);

    int ret_code;

    struct timeval timeout;

    // Timeout values
    timeout.tv_sec = 0;   
    timeout.tv_usec = 0;    

    int max_fd = udp_info->socket + 1;
    int activity = select(max_fd, &readfds, NULL, NULL, &timeout);

    if (activity < 0) {
        return UDP_SELECT_ERROR;
    } else if (activity == 0) {
        return UDP_DATA_IS_NOT_AVAILABLE_WITHIN_TIMEOUT;
    } else {
        if (FD_ISSET(udp_info->socket, &readfds)) {
            ret_code = recvfrom(udp_info->socket, data, size, 0, (struct sockaddr *)&udp_info->Addr, &udp_info->AddrLen);

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

void convert_force_torque(int32_t raw_counts[FORCE_SAMPLE_NUM_DEGREES], float (&forces)[FORCE_SAMPLE_NUM_DEGREES]) {
    // Scaling Factors from Calibration Config
    // const float scaling_factors[6] = {6104, 6104, 6104, 153, 153, 153};

    // Convert raw counts to physical values
    for (unsigned int i = 0; i < FORCE_SAMPLE_NUM_DEGREES; i++) {
        forces[i] = static_cast<float>(raw_counts[i]) / 1000000; // divide by Counts per Force/Torque specified in the force sensor web
    }
}


// udp transmit function. wrapper for sendo that abstracts the UDP_Info_struct
static int udp_transmit(UDP_Info *udp_info, void * data, int size)
{

    if (size > UDP_MAX_PACKET_SIZE) {
        return -1;
    }

    return sendto(udp_info->socket, data, size, 0, (struct sockaddr *)&udp_info->Addr, udp_info->AddrLen);
}


// Function to return force-torque sample (fully non-blocking)
FS_RETURN_CODES return_force_sample_3dof(float (&real_force_sample)[FORCE_SAMPLE_NUM_DEGREES]) {
    unsigned char response[36]; /* The raw response data received from the Net F/T. */

    int32_t force_sample[FORCE_SAMPLE_NUM_DEGREES] = {0,0,0};

    // Non-blocking receive
    int received_bytes = udp_nonblocking_receive(&udp_fs, response, sizeof(response));

    // If no data is available, return false and do nothing
    if (received_bytes == UDP_DATA_IS_NOT_AVAILABLE_WITHIN_TIMEOUT || received_bytes <= 0) {
        return FS_NO_DATA_AVAILABLE;
    }

    // Parse response
    resp.rdt_sequence = ntohl(*(uint32_t*)&response[0]);
    resp.ft_sequence = ntohl(*(uint32_t*)&response[4]);
    resp.status = ntohl(*(uint32_t*)&response[8]);

    // printf("resp.status: %llu\n", resp.status);

    if (resp.status != 0x00000000){
        printf("[ERROR] Unable to interface with force sensor. Check FS initialization code\n");
        return FS_FAIL;
    }

    for (unsigned int i = 0; i < FORCE_SAMPLE_NUM_DEGREES; i++) {
        force_sample[i] = ntohl(*(int32_t*)&response[12 + i * 4]);
    }

    convert_force_torque(force_sample, real_force_sample);

    
    // memcpy(real_force_sample, force_sample, sizeof(force_sample));
    return FS_SUCCESS;
}



// Initialize force sensor connection
void init_force_sensor_connection() {
    int PORT = 49152; 

    udp_fs.socket = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_fs.socket == -1) {
        perror("ERROR: Socket creation failed");
        exit(1);
    }

    // Set socket to non-blocking mode
    // fcntl(udp_fs.socket, F_SETFL, O_NONBLOCK);
    // ✅ Enable Non-Blocking Mode
    int flags = fcntl(udp_fs.socket, F_GETFL, 0);
    if (flags == -1 || fcntl(udp_fs.socket, F_SETFL, flags | O_NONBLOCK) == -1) {
        perror("[ERROR] Failed to set non-blocking mode");
        exit(1);
    }


    char ip_address[] = "169.254.10.100";

    he = gethostbyname(ip_address);
    if (!he) {
        perror("ERROR: Unable to resolve hostname");
        exit(2);
    }

    memset(&udp_fs.Addr, 0, sizeof(udp_fs.Addr));
    udp_fs.Addr.sin_family = AF_INET;
    udp_fs.Addr.sin_port = htons(PORT);
    memcpy(&udp_fs.Addr.sin_addr, he->h_addr_list[0], he->h_length);
    udp_fs.AddrLen = sizeof(udp_fs.Addr);
}

void force_sensor_stop_streaming(){
    *(uint16_t*)&request[0] = htons(0x1234);  /* Standard header */
    *(uint16_t*)&request[2] = htons(0); /* Command code */
    // *(uint32_t*)&request[4] = htonl(NUM_SAMPLES); /* Number of samples */
    *(uint32_t*)&request[4] = htonl(0);

    udp_transmit(&udp_fs, request, 8);
}

int force_sensor_start_streaming(){
    unsigned char response[36]; /* The raw response data received from the Net F/T. */
    int COMMAND = 2;
    int NUM_SAMPLES = 0;

    *(uint16_t*)&request[0] = htons(0x1234);  /* Standard header */
    *(uint16_t*)&request[2] = htons(COMMAND); /* Command code */
    *(uint32_t*)&request[4] = htonl(NUM_SAMPLES); /* Number of samples */
    // *(uint32_t*)&request[4] = htonl(0);

    udp_transmit(&udp_fs, request, 8);

    int received_bytes = 0;

    while(received_bytes <= 0){
        received_bytes = udp_nonblocking_receive(&udp_fs, response, sizeof(response));
    }
    
    // Parse response
    resp.rdt_sequence = ntohl(*(uint32_t*)&response[0]);
    resp.ft_sequence = ntohl(*(uint32_t*)&response[4]);
    resp.status = ntohl(*(uint32_t*)&response[8]);

    if (resp.status != 0x00000000){
        printf("[ERROR] Unable to interface with force sensor. Check FS initialization code\n");
        return FS_FAIL;
    }

    return FS_SUCCESS;
}


void force_sensor_update_bias_values(){

    float real_force_sample[3] = {0,0,0};

    force_sensor_start_streaming();

    for (int i = 0; i < 1000; i++){
        return_force_sample_3dof(real_force_sample);
        FS_X_BIAS += real_force_sample[0];
        FS_Y_BIAS += real_force_sample[1];
        FS_Z_BIAS += real_force_sample[2];
    }

    FS_X_BIAS /= (float)1000;
    FS_Y_BIAS /= (float)1000;
    FS_Z_BIAS /= (float)1000;

    cout << "FORCE SENSOR BIAS VALUES" << endl;
    cout << "X_BIAS: " << FS_X_BIAS << ", Y_BIAS: " << FS_Y_BIAS << ", Z_BIAS: " << FS_Z_BIAS << endl;

    force_sensor_stop_streaming();
}



static bool initiate_socket_connection()
{
    int port = 12345;
    std::cout << "\nInitiating UDP Socket Connection on port " << port << "...\n";

    udp_host.AddrLen = sizeof(udp_host.Addr);

    // Create a UDP socket
    udp_host.socket = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_host.socket < 0) {
        std::cerr << "[UDP ERROR] Failed to create socket\n";
        return false;
    }

    int optval = 1;
    if (setsockopt(udp_host.socket, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval)) < 0) {
        std::cerr << "[UDP ERROR] Failed to set SO_REUSEADDR\n";
        close(udp_host.socket);
        return false;
    }

    //  Set socket to non-blocking mode
    int flags = fcntl(udp_host.socket, F_GETFL, 0);
    if (flags == -1 || fcntl(udp_host.socket, F_SETFL, flags | O_NONBLOCK) == -1) {
        std::cerr << "[UDP ERROR] Failed to set non-blocking mode\n";
        close(udp_host.socket);
        return false;
    }

    // Set up the server address
    memset(&udp_host.Addr, 0, sizeof(udp_host.Addr));
    udp_host.Addr.sin_family = AF_INET;
    udp_host.Addr.sin_port = htons(port);
    udp_host.Addr.sin_addr.s_addr = INADDR_ANY; // Listen on all available interfaces

    // Bind the socket to the port
    if (bind(udp_host.socket, (struct sockaddr *)&udp_host.Addr, sizeof(udp_host.Addr)) < 0) {
        std::cerr << "[UDP ERROR] Failed to bind socket to port " << port << "\n";
        close(udp_host.socket);
        return false;
    }

    std::cout << " UDP Listening Socket Initialized on Port " << port << "!\n";
    return true;
}


// calculate the size of a sample in quadlets
static uint16_t calculate_quadlets_per_sample(uint8_t num_encoders, uint8_t num_motors)
{
    // SAMPLE STRUCTURE

    // 1 quadlet = 4 bytes

    // Timestamp (32 bit)                                                           [1 quadlet]
    // Encoder Position (32 * num of encoders)                                      [1 quadlet * num of encoders]
    // Encoder Velocity Predicted (64 * num of encoders -> truncated to 32bits)     [1 quadlet * num of encoders]
    // Motur Current and Motor Status (32 * num of Motors -> each are 16 bits)      [1 quadlet * num of motors]
    // Digtial IO Values  (optional, used if PS IO is enabled ) 32 bits             [1 quadlet * digital IO]
    // MIO Pins (optional, used if PS IO is enabled ) 4 bits -> pad 32 bits         [1 quadlet * MIO PINS]  
    // Force Torque Readings -> 6 * 32 bits                                         [1 quadlet * 6 Fore/Torque Readings]                 
    if (use_ps_io_flag){
        return (1 + 1 + 1 + (2*(num_encoders)) + (num_motors) + FORCE_SAMPLE_NUM_DEGREES);
    } else {
        return (1 + (2*(num_encoders)) + (num_motors) + FORCE_SAMPLE_NUM_DEGREES);
    }
    
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

    // CAPTURE DATA 
    for (int i = 0; i < samples_per_packet; i++) {

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

        // DATA 1: timestamp
        double time_elapsed = ts_diff_s(t_data_collection_start, t0);

        last_timestamp = time_elapsed;

        float time_elapsed_float = static_cast<float>(time_elapsed);
        data_packet[count++] = *reinterpret_cast<uint32_t *> (&time_elapsed_float);

        // DATA 2: encoder position
        for (int i = 0; i < num_encoders; i++) {
            int32_t encoder_pos = dvrk_controller.Board->GetEncoderPosition(i);
            data_packet[count++] = static_cast<uint32_t>(encoder_pos + dvrk_controller.Board->GetEncoderMidRange());
        }

        // DATA 3: encoder velocity
        for (int i = 0; i < num_encoders; i++) {
            float encoder_velocity_float= static_cast<float>(dvrk_controller.Board->GetEncoderVelocityPredicted(i));
            data_packet[count++] = *reinterpret_cast<uint32_t *>(&encoder_velocity_float);
        }

        // DATA 4 & 5: motor current and motor status (for num_motors)
        for (int i = 0; i < num_motors; i++) {
            uint32_t motor_curr = dvrk_controller.Board->GetMotorCurrent(i); 
            // uint32_t motor_status = dvrk_controller.Board->GetMotorStatus(i);
            // data_packet[count++] = (uint32_t)(((motor_status & 0x0000FFFF) << 16) | (motor_curr & 0x0000FFFF));


            uint32_t raw_cmd_current;
            dvrk_controller.Port->ReadQuadlet(dvrk_controller.Port->GetBoardId(0), ((i+1) << 4) | 1, raw_cmd_current);
            // int16_t raw_cmd_current_16_bit = static_cast<int16_t>(raw_cmd_current);
            // uint16_t cmd_current_casted = *reinterpret_cast<uint16_t *>(&raw_cmd_current_16_bit);

            data_packet[count++] = (uint32_t)(((raw_cmd_current & 0x0000FFFF) << 16) | (motor_curr & 0x0000FFFF));
        }

         // DATA 6: force readings
        float force_sensor[FORCE_SAMPLE_NUM_DEGREES];

        // d_start_time = std::chrono::high_resolution_clock::now();
        int fs_ret = return_force_sample_3dof(force_sensor);

        force_sensor[0] -= FS_X_BIAS;
        force_sensor[1] -= FS_Y_BIAS;
        force_sensor[2] -= FS_Z_BIAS;
                // d_end_time = std::chrono::high_resolution_clock::now();

         
        if(fs_ret == FS_NO_DATA_AVAILABLE){

            for (unsigned int i = 0; i < FORCE_SAMPLE_NUM_DEGREES; i++){
                data_packet[count++] = *reinterpret_cast<uint32_t *>(&last_fs_sample[i]);
           }
        } else if (fs_ret == FS_SUCCESS) {
            memcpy( last_fs_sample, force_sensor, sizeof(last_fs_sample));

            for (unsigned int i = 0; i < FORCE_SAMPLE_NUM_DEGREES; i++){
                data_packet[count++] = *reinterpret_cast<uint32_t *>(&force_sensor[i]);
           }
        } else if (fs_ret == FS_FAIL){
            cout << "[ERROR] Failed to grab data from force sensor" << endl;
            return false;
        }
 

        if (use_ps_io_flag){
            data_packet[count++] = dvrk_controller.Board->ReadDigitalIO();
            data_packet[count++] = (uint32_t) returnMIOPins();
        }

        if (useSampleRate){
            deadline.tv_nsec += period_ns;
            if (deadline.tv_nsec >= 1'000'000'000) {
                deadline.tv_sec++;
                deadline.tv_nsec -= 1'000'000'000;
            }

            // 4) busy spin until deadline, subtracting work_ns if you like
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
            sm.state = SM_SEND_DATA_COLLECTION_METADATA;
        }                    
        
        else if (strcmp(recvd_cmd, HOST_READY_CMD_W_PS_IO) == 0){
            cout << "Received Message - " <<  HOST_READY_CMD_W_PS_IO << endl;
            use_ps_io_flag = true;

            // special case: need to resize double_buffer size to account
            // for extra ps io data
            reset_double_buffer_info(&db, dvrk_controller.Board); 
            sm.state = SM_SEND_DATA_COLLECTION_METADATA;
        }

        else if (strcmp(recvd_cmd, HOST_READY_CMD_W_SAMPLE_RATE) == 0){
            cout << "Received Message - " <<  HOST_READY_CMD_W_SAMPLE_RATE << endl;
            useSampleRate = true;
            
            while(udp_nonblocking_receive(&udp_host, recvd_cmd, CMD_MAX_STRING_SIZE) <= 0){}
            
            SAMPLE_RATE = (int) recvd_cmd;

        } else {
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
            force_sensor_stop_streaming();

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
            case SM_FORCE_SENSOR_FAIL:
                cout << "Failed to Connect to Force Sensor" << endl;
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

    cout << "Initializing_force_sensor..." << endl;
    init_force_sensor_connection();

    force_sensor_start_streaming();
    float sample[FORCE_SAMPLE_NUM_DEGREES];

    if (return_force_sample_3dof(sample) == FS_FAIL){
        cout << "[ERROR] - Force sensor reading incorrect data. Check Initialization" << endl;
    } else {
        cout << "Initializing Force Sensor Success..." << endl;
    }
    force_sensor_stop_streaming();

    force_sensor_update_bias_values();


    bool isOK = initiate_socket_connection();

    if (!isOK) {
        cout << "[error] failed to establish socket connection !!" << endl;
        return -1;
    }


    sm.state = SM_WAIT_FOR_HOST_HANDSHAKE;

    while (sm.state != SM_EXIT) {

        switch (sm.state) {
            case SM_WAIT_FOR_HOST_HANDSHAKE:
                sm = wait_for_host_handshake(sm);
                // printf("handshake\n");
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

    dataCollectionStateMachine();

    return 0;
}
