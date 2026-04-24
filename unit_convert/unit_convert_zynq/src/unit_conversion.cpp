#include "config_loader.h"
#include "robot_config.h"


// converts encoder counts to encoder position (si units)
float convert_enc_pos_to_si_units(RobotConfig cfg, uint8_t num_enc,int32_t raw_encoder_counts, uint8_t idx){

        return (raw_encoder_counts - cfg.actuators[idx].enc.midrange) * cfg.actuators[idx].enc.scale * cfg.actuators[idx].enc.unit;
}

// converts encoder velocity (counts/sec) to si units
float convert_enc_vel_to_si_units(RobotConfig cfg, uint8_t num_enc,float raw_encoder_velocity, uint8_t idx){

    return raw_encoder_velocity * cfg.actuators[idx].enc.scale * cfg.actuators[idx].enc.unit;

}

void convert_torque_to_si_units(RobotConfig cfg, uint8_t num_motors,uint32_t* raw_current, float* current_out){

    for (int i = 0; i < num_motors; i++){       
        current_out[i] = raw_current[i] * cfg.actuators[i].cur.scale + cfg.actuators[i].cur.offset;
    }
}
