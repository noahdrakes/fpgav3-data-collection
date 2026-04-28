#include <../lib/config_loader.h>
#include <algorithm>
#include <dirent.h>
#include <fstream>
#include <cmath>
#include <stdexcept>
#include <sys/stat.h>
#include <vector>
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
        std::string unit_str = enc["BitsToPosition"]["Unit"].get<std::string>();
        double unit;
        if (unit_str == "deg") {
            unit = M_PI / 180.0;
        } else if (unit_str == "rad") {
            unit = 1.0;
        } else if (unit_str == "mm") {
            unit = 0.001;
        } else {
            throw std::runtime_error("Unknown BitsToPosition unit '" + unit_str + "' for actuator " + std::to_string(i));
        }
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

std::string find_first_json_config(const std::string& directory_path) {
    struct stat path_stat;
    if (stat(directory_path.c_str(), &path_stat) != 0) {
        throw std::runtime_error("Config directory does not exist: " + directory_path);
    }
    if (!S_ISDIR(path_stat.st_mode)) {
        throw std::runtime_error("Config path is not a directory: " + directory_path);
    }

    DIR* dir = opendir(directory_path.c_str());
    if (!dir) {
        throw std::runtime_error("Failed to open config directory: " + directory_path);
    }

    std::vector<std::string> json_files;
    struct dirent* entry;
    while ((entry = readdir(dir)) != nullptr) {
        const std::string file_name(entry->d_name);
        if (file_name.size() < 5 || file_name.substr(file_name.size() - 5) != ".json") {
            continue;
        }

        const std::string full_path = directory_path + "/" + file_name;
        if (stat(full_path.c_str(), &path_stat) == 0 && S_ISREG(path_stat.st_mode)) {
            json_files.push_back(full_path);
        }
    }
    closedir(dir);

    if (json_files.empty()) {
        throw std::runtime_error("No .json config file found in directory: " + directory_path);
    }

    std::sort(json_files.begin(), json_files.end());
    return json_files.front();
}

RobotConfig load_first_json_config(const std::string& directory_path) {
    return load_config(find_first_json_config(directory_path));
}
