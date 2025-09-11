#include "PointArray.h"
#include "PointUtils.h"

namespace accurate_ri {
    void PointArray::computeExtraInfo() {
        extraInfo.coordsEps = PointUtils::computeCoordsEps(*this);

        const Eigen::ArrayXd &rangeXySquared = x.array().square() + y.array().square();

        extraInfo.rangeXy = rangeXySquared.sqrt();
        extraInfo.range = (rangeXySquared + z.array().square()).sqrt();
        extraInfo.phi = (z.array() / extraInfo.range.array()).asin();
        extraInfo.theta = y.binaryExpr(
            x, [](double yi, double xi) {
                return std::atan2(yi, xi);
            }
        );

        extraInfo.invRange = extraInfo.range.array().inverse();
        extraInfo.invRangeXy = extraInfo.rangeXy.array().inverse();
        extraInfo.maxRange = extraInfo.range.maxCoeff();
        extraInfo.minRange = extraInfo.range.minCoeff();
    }
}
