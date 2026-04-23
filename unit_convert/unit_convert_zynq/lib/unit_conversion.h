#include "config_loader.h"
#include "robot_config.h"


// converts encoder counts to encoder position (si units)
void convert_enc_pos_to_si_units(RobotConfig cfg, uint8_t num_enc,uint32_t* raw_encoder_counts, float* position_out);
void convert_enc_vel_to_si_units(RobotConfig cfg, uint8_t num_enc,uint32_t* raw_encoder_velocity, float* vel_out);
void convert_torque_to_si_units(RobotConfig cfg, uint8_t num_motors,uint32_t* raw_current, float* current_out);
