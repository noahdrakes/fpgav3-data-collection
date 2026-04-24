#include "config_loader.h"
#include "robot_config.h"

float convert_enc_pos_to_si_units(RobotConfig cfg, uint8_t num_enc,int32_t raw_encoder_counts, uint8_t idx);
float convert_enc_vel_to_si_units(RobotConfig cfg, uint8_t num_enc,float raw_encoder_velocity, uint8_t idx);
void convert_torque_to_si_units(RobotConfig cfg, uint8_t num_motors,uint32_t* raw_current, float* current_out);
