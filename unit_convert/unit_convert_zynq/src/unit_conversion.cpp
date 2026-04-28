#include "config_loader.h"
#include "robot_config.h"


// converts encoder counts to encoder position (si units)
float convert_enc_pos_to_si_units(RobotConfig cfg, int32_t raw_encoder_counts, uint8_t idx){

        return (raw_encoder_counts - cfg.actuators[idx].enc.midrange) * cfg.actuators[idx].enc.scale * cfg.actuators[idx].enc.unit;
}

// converts encoder velocity (counts/sec) to si units
float convert_enc_vel_to_si_units(RobotConfig cfg, float raw_encoder_velocity, uint8_t idx){

    return raw_encoder_velocity * cfg.actuators[idx].enc.scale * cfg.actuators[idx].enc.unit;

}

float convert_torque_to_si_units(RobotConfig cfg, uint16_t raw_current, uint8_t idx){

        return raw_current * cfg.actuators[idx].cur.scale + cfg.actuators[idx].cur.offset;
}
