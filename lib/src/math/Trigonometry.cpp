#include "math/Trigonometry.h"
#include <cmath>
#include <array>
#include "Constants.h"

namespace accurate_ri::Trigonometry {
    const std::array<double, TRIG_TABLE_SIZE>& getSinTable() {
        static const std::array<double, TRIG_TABLE_SIZE> lut = [] {
            std::array<double, TRIG_TABLE_SIZE> table{};
            for (int i = 0; i < TRIG_TABLE_SIZE; ++i) {
                const double angle = (Constant::TWO_PI * i) / TRIG_TABLE_SIZE;
                table[i] = std::sin(angle);
            }
            return table;
        }();

        return lut;
    }

    const std::array<double, TRIG_TABLE_SIZE>& getCosTable() {
        static const std::array<double, TRIG_TABLE_SIZE> lut = [] {
            std::array<double, TRIG_TABLE_SIZE> table{};
            for (int i = 0; i < TRIG_TABLE_SIZE; ++i) {
                const double angle = (Constant::TWO_PI * i) / TRIG_TABLE_SIZE;
                table[i] = std::cos(angle);
            }
            return table;
        }();

        return lut;
    }

    int32_t radiansToIndex(const double radians) {
        int32_t index = static_cast<int32_t>(std::round(radians * TRIG_TABLE_SIZE / Constant::TWO_PI));
        index += index < 0 ? TRIG_TABLE_SIZE : 0;
        index %= TRIG_TABLE_SIZE;

        return index;
    }

    double sinIndex(const int32_t index) {
        return getSinTable()[index];
    }

    double cosIndex(const int32_t index) {
        return getCosTable()[index];
    }

    double sin(const double radians) {
        return sinIndex(radiansToIndex(radians));
    }

    double cos(const double radians) {
        return cosIndex(radiansToIndex(radians));
    }
}
