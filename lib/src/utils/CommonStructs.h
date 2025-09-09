#pragma once
#include <iomanip>

#include "accurate_ri/public_structs.hpp"

namespace accurate_ri {
    struct ValueConfInterval {
        double value;
        Interval ci;
    };

    inline std::ostream &operator<<(std::ostream &os, const Interval &interval) {
        os << std::fixed << std::setprecision(5) << "[" << interval.lower << ", " << interval.upper << "]";
        return os;
    }
}
