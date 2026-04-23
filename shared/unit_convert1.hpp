#include <fstream>
#include <nlohmann/json.hpp>
#include <array>
#pragma once

using json = nlohmann::json;


#include <vector>

struct EncoderConfig {
    double scale;
    double unit;
    double midrange;
};

struct CurrentConfig {
    double scale;
    double offset;
};

struct RobotConfig {
    std::vector<EncoderConfig> enc;
    std::vector<CurrentConfig> cur;
};

RobotConfig load_config(const std::string& path, N) {
    std::ifstream f(path);
    json j;
    f >> j;

    RobotConfig cfg;

    auto& actuators = j["Robots"][0]["Actuators"];

    for (int i = 0; i < NUM_JOINTS; ++i) {

        // -------- POSITION / VELOCITY --------
        double scale = actuators[i]["Encoder"]["BitsToPosition"]["Scale"];

        // convert deg → rad once
        double unit = M_PI / 180.0;

        // ⚠️ midrange (NOT in your JSON usually)
        // must be inferred from encoder bit depth
        // example: 24-bit encoder
        double midrange = pow(2.0, 23);  

        cfg.enc[i] = {scale, unit, midrange};

        // -------- CURRENT --------
        double cur_scale = actuators[i]["Drive"]["BitsToCurrent"]["Scale"];
        double cur_offset = actuators[i]["Drive"]["BitsToCurrent"]["Offset"];

        cfg.cur[i] = {cur_scale, cur_offset};
    }

    return cfg;
}