#pragma once
#include <cstdint>

namespace alice_lri::Trigonometry {
    constexpr uint32_t TRIG_TABLE_SIZE = 65536;

    double sinIndex(int32_t index);
    double cosIndex(int32_t index);
    double sin(double radians);
    double cos(double radians);
}
