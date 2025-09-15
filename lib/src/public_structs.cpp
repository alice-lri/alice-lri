#include <algorithm>
#include <alice_lri/public_structs.hpp>

#include "utils/Utils.h"

namespace alice_lri {
    bool Interval::anyContained(const Interval &other) const noexcept {
        return Utils::compare(lower, other.lower) * Utils::compare(upper, other.upper) != 1;
    }

    void Interval::clampBoth(const double minValue, const double maxValue) noexcept {
        lower = std::clamp(lower, minValue, maxValue);
        upper = std::clamp(upper, minValue, maxValue);
    }
}
