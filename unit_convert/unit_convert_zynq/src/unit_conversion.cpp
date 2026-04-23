#include "config_loader.h"
#include "robot_config.h"


// converts encoder counts to encoder position (si units)
void convert_enc_pos_to_si_units(RobotConfig cfg, uint8_t num_enc,uint32_t* raw_encoder_counts, float* position_out){

    for (int i = 0; i < num_enc; i++){
        position_out[i] = (raw_encoder_counts[i] - cfg.actuators[i].enc.midrange) * cfg.actuators[i].enc.scale * cfg.actuators[i].enc.unit;
    } 
}

// converts encoder velocity (counts/sec) to si units
void convert_enc_vel_to_si_units(RobotConfig cfg, uint8_t num_enc,uint32_t* raw_encoder_velocity, float* vel_out){

    for (int i = 0; i < num_enc; i++){
            vel_out[i] = raw_encoder_velocity[i] * cfg.actuators[i].enc.scale * cfg.actuators[i].enc.unit;
    } 
}

void convert_torque_to_si_units(RobotConfig cfg, uint8_t num_motors,uint32_t* raw_current, float* current_out){

    for (int i = 0; i < num_motors; i++){       
        current_out[i] = raw_current[i] * cfg.actuators[i].cur.scale + cfg.actuators[i].cur.offset;
    }
}
