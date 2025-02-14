#include "PointArray.h"
#include "PointUtils.h"

namespace accurate_ri {
    void PointArray::computeExtraInfo() {
        extraInfo.coordsEps = PointUtils::computeCoordsEps(*this);

        auto rangeXySquared = x.array().square() + y.array().square();

        extraInfo.rangeXy = rangeXySquared.sqrt();
        extraInfo.range = (rangeXySquared + z.array().square()).sqrt();
        extraInfo.phi = (z.array() / extraInfo.range.array()).asin();
        extraInfo.theta = y.binaryExpr(x, [](double yi, double xi) { return std::atan2(yi, xi); });
    }
}
