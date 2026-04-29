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

enum class ConfigSchema {
    LegacyXmlJson,
    SawRobotIoV6
};

static double infer_midrange(const json& encoder_json) {
    // If your config has Bits, use it
    if (encoder_json.contains("Bits")) {
        int bits = encoder_json["Bits"];
        return std::pow(2.0, bits - 1);
    }

    // Fallback (common dVRK case: 24-bit encoder)
    return std::pow(2.0, 23);
}

static double unit_to_si_factor(const std::string& unit_str, size_t actuator_index) {
    if (unit_str == "deg") {
        return M_PI / 180.0;
    }
    if (unit_str == "rad") {
        return 1.0;
    }
    if (unit_str == "mm") {
        return 0.001;
    }
    throw std::runtime_error("Unknown BitsToPosition unit '" + unit_str + "' for actuator " + std::to_string(actuator_index));
}

RobotConfig load_config(const std::string& path) {
    std::ifstream f(path);
    if (!f.is_open()) {
        throw std::runtime_error("Failed to open config file: " + path);
    }

    json j;
    f >> j;

    RobotConfig cfg;

    ConfigSchema schema;
    json* actuators = nullptr;
    if (j.contains("Robots")) {
        schema = ConfigSchema::LegacyXmlJson;
        actuators = &j["Robots"][0]["Actuators"];
    } else if (j.contains("robots")) {
        schema = ConfigSchema::SawRobotIoV6;
        actuators = &j["robots"][0]["actuators"];
    } else {
        throw std::runtime_error("Unsupported robot config schema: missing 'Robots' or 'robots'");
    }

    if (!actuators->is_array()) {
        throw std::runtime_error("Invalid robot config: actuators is not an array");
    }

    size_t n = actuators->size();

    cfg.actuators.resize(n);

    for (size_t i = 0; i < n; ++i) {

        auto& a = (*actuators)[i];

        // -------- ENCODER (REQUIRED) --------
        double scale;
        double unit;
        double midrange;

        if (schema == ConfigSchema::LegacyXmlJson) {
            if (!a.contains("Encoder") ||
                !a["Encoder"].contains("BitsToPosition")) {
                throw std::runtime_error("Missing encoder config for actuator " + std::to_string(i));
            }

            auto& enc = a["Encoder"];
            auto& b2p = enc["BitsToPosition"];

            scale = b2p["Scale"];
            unit = unit_to_si_factor(b2p["Unit"].get<std::string>(), i);
            midrange = infer_midrange(enc);
        } else {
            if (!a.contains("encoder") ||
                !a["encoder"].contains("bits_to_position")) {
                throw std::runtime_error("Missing encoder config for actuator " + std::to_string(i));
            }

            auto& enc = a["encoder"];
            auto& b2p = enc["bits_to_position"];

            scale = b2p["scale"];
            unit = 1.0; // saw-robot-io schema v6 stores encoder scale in SI units.
            midrange = infer_midrange(enc);
        }

        cfg.actuators[i].enc = {scale, unit, midrange};

        // -------- CURRENT (motor) --------
        double cur_scale;
        double cur_offset;

        if (schema == ConfigSchema::LegacyXmlJson) {
            if (!a.contains("Drive") ||
                !a["Drive"].contains("BitsToCurrent")) {
                throw std::runtime_error("Missing motor config for actuator " + std::to_string(i));
            }

            auto& b2c = a["Drive"]["BitsToCurrent"];
            cur_scale = b2c["Scale"];
            cur_offset = b2c["Offset"];
        } else {
            if (!a.contains("drive") ||
                !a["drive"].contains("bits_to_current")) {
                throw std::runtime_error("Missing motor config for actuator " + std::to_string(i));
            }

            auto& b2c = a["drive"]["bits_to_current"];
            cur_scale = b2c["scale"];
            cur_offset = b2c["offset"];
        }

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
