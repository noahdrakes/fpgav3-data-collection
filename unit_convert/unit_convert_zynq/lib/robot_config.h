#pragma once
#include <vector>

struct ScaleOffset {
    double Scale;
    double Offset;
};

struct ActuatorConfig {
    ScaleOffset Curr_B2C;
    ScaleOffset Curr_C2B;
    ScaleOffset Curr_Nm2C;
    ScaleOffset Enc_B2P;
    ScaleOffset Pot_B2V;
    ScaleOffset Pot_V2P;
    double unit;      // SI conversion factor (deg->rad, mm->m)
    double midrange;  // pre-scale encoder offset
};

struct RobotConfig {
    std::vector<ActuatorConfig> actuators;
};
