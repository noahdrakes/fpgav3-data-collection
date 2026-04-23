#include "unit_conversion.hpp"

struct RawPacket {
    int32_t pos[6];
    int32_t vel[6];
    int32_t current[6];
};

struct ConvertedPacket {
    double pos[6];
    double vel[6];
    double current[6];
};

ConvertedPacket load_data_packet(const RawPacket& raw) {
    ConvertedPacket out;

    std::array<int32_t, 6> pos, vel, cur;
    std::array<double, 6> pos_o, vel_o, cur_o;

    for (int i = 0; i < 6; ++i) {
        pos[i] = raw.pos[i];
        vel[i] = raw.vel[i];
        cur[i] = raw.current[i];
    }

    UnitConversion::convert_packet(pos, vel, cur, pos_o, vel_o, cur_o);

    for (int i = 0; i < 6; ++i) {
        out.pos[i] = pos_o[i];
        out.vel[i] = vel_o[i];
        out.current[i] = cur_o[i];
    }

    return out;
}