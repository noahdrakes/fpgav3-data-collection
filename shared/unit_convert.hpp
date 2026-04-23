#pragma once
#include <array>
#include <cstdint>
#include <cmath>

namespace UnitConversion {

// -------- CONSTANTS --------

constexpr int NUM_JOINTS = 6;
constexpr double DEG_TO_RAD = M_PI / 180.0;
constexpr double MM_TO_M = 0.001;
constexpr double ENCODER_MIDRANGE = 0x800000;

enum class JointType {
    Revolute,
    Prismatic
};

// -------- STRUCT --------
struct Conversion {
    double scale;
    double offset;
};

// -------- PER-JOINT CONFIG --------
// Fill these from your JSON ONCE

constexpr std::array<JointType, NUM_JOINTS> joint_types = {{
    JointType::Revolute,
    JointType::Revolute,
    JointType::Prismatic,
    JointType::Revolute,
    JointType::Revolute,
    JointType::Revolute
}};

// Encoder BitsToPosition scale from JSON. Revolute joints are in deg/bit;
// prismatic joints are in mm/bit. SI conversion happens in convert_packet.
constexpr std::array<double, NUM_JOINTS> encoder_bits_to_position = {{
    5.2734396094e-05,  // joint 1
    5.2734396094e-05,
    5.2734396094e-05,
    5.2734396094e-05,
    5.2734396094e-05,
    5.2734396094e-05
}};

// Drive BitsToCurrent: motor current bits -> amps.
constexpr std::array<Conversion, NUM_JOINTS> bits_to_current = {{
    {-0.000208333, 6.8267},
    {-0.000208333, 6.8267},
    {-0.000208333, 6.8267},
    {-0.000208333, 6.8267},
    {-0.000208333, 6.8267},
    {-0.000208333, 6.8267}
}};

// -------- API --------

constexpr double encoder_unit(JointType joint_type) {
    return joint_type == JointType::Prismatic ? MM_TO_M : DEG_TO_RAD;
}

inline void convert_packet(
    const std::array<int32_t, NUM_JOINTS>& raw_pos,
    const std::array<int32_t, NUM_JOINTS>& raw_vel,
    const std::array<int32_t, NUM_JOINTS>& raw_motor_current,

    std::array<double, NUM_JOINTS>& pos_out,
    std::array<double, NUM_JOINTS>& vel_out,
    std::array<double, NUM_JOINTS>& current_out
) {
    for (int i = 0; i < NUM_JOINTS; ++i) {
        const double unit = encoder_unit(joint_types[i]);

        pos_out[i] = (static_cast<double>(raw_pos[i]) - ENCODER_MIDRANGE) * encoder_bits_to_position[i] * unit;
        vel_out[i] = static_cast<double>(raw_vel[i]) * encoder_bits_to_position[i] * unit;
        current_out[i] = static_cast<double>(raw_motor_current[i]) * bits_to_current[i].scale + bits_to_current[i].offset;
    }
}

} // namespace UnitConversion
