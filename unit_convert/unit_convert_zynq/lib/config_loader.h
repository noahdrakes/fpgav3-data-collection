#pragma once
#include <string>
#include "robot_config.h"

RobotConfig load_config(const std::string& path);
void print_config_values(const RobotConfig& cfg);
std::string find_first_json_config(const std::string& directory_path);
RobotConfig load_first_json_config(const std::string& directory_path);
