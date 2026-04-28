#include "config_loader.h"
#include "robot_config.h"

float convert_enc_pos_to_si_units(RobotConfig cfg, int32_t raw_encoder_counts, uint8_t idx);
float convert_enc_vel_to_si_units(RobotConfig cfg, float raw_encoder_velocity, uint8_t idx);
float convert_torque_to_si_units(RobotConfig cfg, uint16_t raw_current, uint8_t idx);
