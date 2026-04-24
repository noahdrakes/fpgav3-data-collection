#pragma once
#include <string>
#include "robot_config.h"

RobotConfig load_config(const std::string& path);
std::string find_first_json_config(const std::string& directory_path);
RobotConfig load_first_json_config(const std::string& directory_path);
