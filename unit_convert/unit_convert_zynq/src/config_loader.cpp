#include <../lib/config_loader.h>
#include <algorithm>
#include <dirent.h>
#include <fstream>
#include <iomanip>
#include <iostream>
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
    if (encoder_json.contains("bits")) {
        int bits = encoder_json["bits"];
        return std::pow(2.0, bits - 1);
    }

    // Fallback (common dVRK case: 24-bit encoder)
    return std::pow(2.0, 23);
}

static double unit_to_si_factor(const std::string& unit_str, size_t actuator_index) {
    if (unit_str == "deg" || unit_str == "REVOLUTE") {
        return M_PI / 180.0;
    }
    if (unit_str == "rad") {
        return 1.0;
    }
    if (unit_str == "mm" || unit_str == "PRISMATIC") {
        return 0.001;
    }
    throw std::runtime_error("Unknown encoder unit '" + unit_str + "' for actuator " + std::to_string(actuator_index));
}

static ScaleOffset load_scale_offset(const json& parent,
                                     const std::string& conversion_name,
                                     const std::string& scale_key,
                                     const std::string& offset_key,
                                     const std::string& context,
                                     size_t actuator_index) {
    if (!parent.contains(conversion_name)) {
        throw std::runtime_error("Missing " + context + " config for actuator " + std::to_string(actuator_index));
    }

    const auto& conversion = parent[conversion_name];
    if (!conversion.contains(scale_key)) {
        throw std::runtime_error("Missing " + context + " scale for actuator " + std::to_string(actuator_index));
    }

    return {
        conversion[scale_key].get<double>(),
        conversion.value(offset_key, 0.0)
    };
}

static ScaleOffset load_optional_scale_offset(const json& parent,
                                              const std::string& conversion_name,
                                              const std::string& scale_key,
                                              const std::string& offset_key) {
    if (!parent.contains(conversion_name) || !parent[conversion_name].contains(scale_key)) {
        return {0.0, 0.0};
    }

    const auto& conversion = parent[conversion_name];
    return {
        conversion[scale_key].get<double>(),
        conversion.value(offset_key, 0.0)
    };
}

void print_config_values(const RobotConfig& cfg) {
    std::cout << std::setprecision(17);
    for (size_t i = 0; i < cfg.actuators.size(); ++i) {
        const auto& a = cfg.actuators[i];
        std::cout << "--------------\n";
        std::cout << "ACTUATOR: " << i << "\n";
        std::cout << "---------------\n";
        std::cout << "unit: " << a.unit << "\n";
        std::cout << "Enc_B2P Scale: " << a.Enc_B2P.Scale << "\n";
        std::cout << "Enc_B2P Offset: " << a.Enc_B2P.Offset << "\n";
        std::cout << "Curr_B2C Scale: " << a.Curr_B2C.Scale << "\n";
        std::cout << "Curr_B2C Offset: " << a.Curr_B2C.Offset << "\n";
        std::cout << "Curr_C2B Scale: " << a.Curr_C2B.Scale << "\n";
        std::cout << "Curr_C2B Offset: " << a.Curr_C2B.Offset << "\n";
        std::cout << "Curr_Nm2C Scale: " << a.Curr_Nm2C.Scale << "\n";
        std::cout << "Curr_Nm2C Offset: " << a.Curr_Nm2C.Offset << "\n";
        std::cout << "Pot_B2V Scale: " << a.Pot_B2V.Scale << "\n";
        std::cout << "Pot_B2V Offset: " << a.Pot_B2V.Offset << "\n";
        std::cout << "Pot_V2P Scale: " << a.Pot_V2P.Scale << "\n";
        std::cout << "Pot_V2P Offset: " << a.Pot_V2P.Offset << "\n";
    }
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

        if (schema == ConfigSchema::LegacyXmlJson) {
            if (!a.contains("Drive")) {
                throw std::runtime_error("Missing Drive config for actuator " + std::to_string(i));
            }
            if (!a.contains("Encoder")) {
                throw std::runtime_error("Missing Encoder config for actuator " + std::to_string(i));
            }
            if (!a.contains("Pot")) {
                throw std::runtime_error("Missing Pot config for actuator " + std::to_string(i));
            }

            const auto& drive = a["Drive"];
            const auto& encoder = a["Encoder"];
            const auto& pot = a["Pot"];

            cfg.actuators[i].Curr_B2C = load_scale_offset(drive, "BitsToCurrent", "Scale", "Offset", "Curr_B2C", i);
            cfg.actuators[i].Curr_C2B = load_scale_offset(drive, "CurrentToBits", "Scale", "Offset", "Curr_C2B", i);
            cfg.actuators[i].Curr_Nm2C = load_scale_offset(drive, "EffortToCurrent", "Scale", "Offset", "Curr_Nm2C", i);
            cfg.actuators[i].Enc_B2P = load_scale_offset(encoder, "BitsToPosition", "Scale", "Offset", "Enc_B2P", i);
            cfg.actuators[i].Pot_B2V = load_scale_offset(pot, "BitsToVoltage", "Scale", "Offset", "Pot_B2V", i);
            cfg.actuators[i].Pot_V2P = load_scale_offset(pot, "SensorToPosition", "Scale", "Offset", "Pot_V2P", i);
            cfg.actuators[i].unit = unit_to_si_factor(a.value("JointType", ""), i);
            cfg.actuators[i].midrange = infer_midrange(encoder);
        } else {
            if (!a.contains("drive")) {
                throw std::runtime_error("Missing drive config for actuator " + std::to_string(i));
            }
            if (!a.contains("encoder")) {
                throw std::runtime_error("Missing encoder config for actuator " + std::to_string(i));
            }

            const auto& drive = a["drive"];
            const auto& encoder = a["encoder"];
            const json empty_pot = json::object();
            const auto& pot = a.contains("pot") ? a["pot"] : empty_pot;

            cfg.actuators[i].Curr_B2C = load_scale_offset(drive, "bits_to_current", "scale", "offset", "Curr_B2C", i);
            cfg.actuators[i].Curr_C2B = load_scale_offset(drive, "current_to_bits", "scale", "offset", "Curr_C2B", i);
            cfg.actuators[i].Curr_Nm2C = load_scale_offset(drive, "effort_to_current", "scale", "offset", "Curr_Nm2C", i);
            cfg.actuators[i].Enc_B2P = load_scale_offset(encoder, "bits_to_position", "scale", "offset", "Enc_B2P", i);
            cfg.actuators[i].Pot_B2V = load_optional_scale_offset(pot, "bits_to_voltage", "scale", "offset");
            cfg.actuators[i].Pot_V2P = load_optional_scale_offset(pot, "sensor_to_position", "scale", "offset");
            cfg.actuators[i].unit = 1.0; // saw-robot-io schema v6 stores encoder scale in SI units.
            cfg.actuators[i].midrange = infer_midrange(encoder);
        }
    }

    print_config_values(cfg);
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
