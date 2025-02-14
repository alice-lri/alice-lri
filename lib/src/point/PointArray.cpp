#include "PointArray.h"
#include "PointUtils.h"

namespace accurate_ri {
    void PointArray::computeExtraInfo() {
        extraInfo.coordsEps = PointUtils::computeCoordsEps(*this);

        extraInfo.range.resize(size());
        extraInfo.rangeXy.resize(size());
        extraInfo.phi.resize(size());
        extraInfo.theta.resize(size());

        for (size_t i = 0; i < size(); ++i) {
            const double x = getX(i);
            const double y = getY(i);
            const double z = getZ(i);
            const double rangeXySquared = x * x + y * y;
            const double range = std::sqrt(rangeXySquared + z * z);
            const double rangeXy = std::sqrt(rangeXySquared);
            const double phi = std::asin(z / range);
            const double theta = std::atan2(y, x);

            extraInfo.range[i] = range;
            extraInfo.rangeXy[i] = rangeXy;
            extraInfo.phi[i] = phi;
            extraInfo.theta[i] = theta;
        }
    }
}
