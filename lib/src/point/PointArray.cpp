#include "PointArray.h"
#include "PointUtils.h"

namespace accurate_ri {
    void PointArray::computeExtraInfo() {
        extraInfo.coordsEps = PointUtils::computeCoordsEps(*this);

        extraInfo.range.reserve(size());
        extraInfo.rangeXy.reserve(size());
        extraInfo.phi.reserve(size());
        extraInfo.theta.reserve(size());

        for (size_t i = 0; i < size(); ++i) {
            const double x = getX(i);
            const double y = getY(i);
            const double z = getZ(i);
            const double rangeXySquared = x * x + y * y;
            const double range = std::sqrt(rangeXySquared + z * z);
            const double rangeXy = std::sqrt(rangeXySquared);
            const double phi = std::asin(z / range);
            const double theta = std::atan2(y, x);

            extraInfo.range.emplace_back(range);
            extraInfo.rangeXy.emplace_back(rangeXy);
            extraInfo.phi.emplace_back(phi);
            extraInfo.theta.emplace_back(theta);
        }
    }
}
