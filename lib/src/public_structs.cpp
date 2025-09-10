#include <vector>
#include <algorithm>
#include <accurate_ri/public_structs.hpp>

namespace accurate_ri {

    void Interval::clampBoth(const double minValue, const double maxValue) {
        lower = std::clamp(lower, minValue, maxValue);
        upper = std::clamp(upper, minValue, maxValue);
    }

}
