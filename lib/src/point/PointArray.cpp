#include "PointArray.h"
#include "PointUtils.h"

namespace alice_lri {
    void PointArray::computeExtraInfo() {
        extraInfo.coordsEps = PointUtils::computeCoordsEps(*this);

        const Eigen::ArrayXd &rangeXySquared = x.square() + y.square();

        extraInfo.rangeXy = rangeXySquared.sqrt();
        extraInfo.range = (rangeXySquared + z.square()).sqrt();
        extraInfo.phi = (z / extraInfo.range).asin();
        extraInfo.theta = y.binaryExpr(x, [](const double yi, const double xi) {
            return std::atan2(yi, xi);
        });

        extraInfo.invRange = extraInfo.range.inverse();
        extraInfo.invRangeXy = extraInfo.rangeXy.inverse();
        extraInfo.maxRange = extraInfo.range.maxCoeff();
        extraInfo.minRange = extraInfo.range.minCoeff();
    }
}
