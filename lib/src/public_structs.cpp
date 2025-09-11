#include <algorithm>
#include <accurate_ri/public_structs.hpp>

#include "utils/Utils.h"

namespace accurate_ri {
    bool Interval::anyContained(const Interval &other) const {
        return Utils::compare(lower, other.lower) * Utils::compare(upper, other.upper) != 1;
    }

    void Interval::clampBoth(const double minValue, const double maxValue) {
        lower = std::clamp(lower, minValue, maxValue);
        upper = std::clamp(upper, minValue, maxValue);
    }
}
