#include <../lib/config_loader.h>
#include <fstream>
#include <cmath>
#include <stdexcept>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

static double infer_midrange(const json& encoder_json) {
    // If your config has Bits, use it
    if (encoder_json.contains("Bits")) {
        int bits = encoder_json["Bits"];
        return std::pow(2.0, bits - 1);
    }

    // Fallback (common dVRK case: 24-bit encoder)
    return std::pow(2.0, 23);
}

RobotConfig load_config(const std::string& path) {
    std::ifstream f(path);
    if (!f.is_open()) {
        throw std::runtime_error("Failed to open config file: " + path);
    }

    json j;
    f >> j;

    RobotConfig cfg;

    auto& actuators = j["Robots"][0]["Actuators"];
    size_t n = actuators.size();

    cfg.actuators.resize(n);

    for (size_t i = 0; i < n; ++i) {

        auto& a = actuators[i];

        // -------- ENCODER (REQUIRED) --------
        if (!a.contains("Encoder") ||
            !a["Encoder"].contains("BitsToPosition")) {
            throw std::runtime_error("Missing encoder config for actuator " + std::to_string(i));
        }

        auto& enc = a["Encoder"];

        double scale = enc["BitsToPosition"]["Scale"];
        double unit = M_PI / 180.0;  // deg → rad
        double midrange = infer_midrange(enc);

        cfg.actuators[i].enc = {scale, unit, midrange};

        // -------- CURRENT (motor) --------
        if (!a.contains("Drive") ||
            !a["Drive"].contains("BitsToCurrent")) {
            throw std::runtime_error("Missing motor config for actuator " + std::to_string(i));
        }

        auto& drv = a["Drive"];

        double cur_scale = drv["BitsToCurrent"]["Scale"];
        double cur_offset = drv["BitsToCurrent"]["Offset"];

        cfg.actuators[i].cur = {cur_scale, cur_offset};
    }

    return cfg;
}