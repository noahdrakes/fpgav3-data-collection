#pragma once
#include <vector>

struct EncoderConfig {
    double scale;     // bits → deg (from JSON)
    double unit;      // SI conversion factor (deg→rad, rad→1, mm→m)
    double midrange;  // pre-scale offset
};

struct CurrentConfig {
    double scale;
    double offset;
};

struct ActuatorConfig {
    EncoderConfig enc;
    CurrentConfig cur;
};

struct RobotConfig {
    std::vector<ActuatorConfig> actuators;
};
