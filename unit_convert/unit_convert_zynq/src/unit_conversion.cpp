#include "unit_conversion.h"


// converts encoder counts to encoder position (si units)
float convert_enc_pos_to_si_units(RobotConfig cfg, int32_t raw_encoder_counts, uint8_t idx){

        return (raw_encoder_counts - cfg.actuators[idx].midrange) * cfg.actuators[idx].Enc_B2P.Scale * cfg.actuators[idx].unit;
}

// converts encoder velocity (counts/sec) to si units
float convert_enc_vel_to_si_units(RobotConfig cfg, float raw_encoder_velocity, uint8_t idx){

    return raw_encoder_velocity * cfg.actuators[idx].Enc_B2P.Scale * cfg.actuators[idx].unit;

}

float convert_torque_to_si_units(RobotConfig cfg, uint16_t raw_current, uint8_t idx){

        return ((raw_current * cfg.actuators[idx].Curr_B2C.Scale) + cfg.actuators[idx].Curr_B2C.Offset) /
               cfg.actuators[idx].Curr_Nm2C.Scale;
}

float convert_torque_command_to_si_units(RobotConfig cfg, uint16_t raw_motor_status, uint8_t idx){

        return ((raw_motor_status - cfg.actuators[idx].Curr_C2B.Offset) / cfg.actuators[idx].Curr_C2B.Scale) /
               cfg.actuators[idx].Curr_Nm2C.Scale;
}
