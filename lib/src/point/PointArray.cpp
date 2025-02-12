#include "PointArray.h"
#include "PointUtils.h"

namespace accurate_ri {
    void PointArray::computeExtraInfo() {
        extraInfo.range = PointUtils::computeRanges(*this);
        extraInfo.phi = PointUtils::computePhis(*this);
        extraInfo.theta = PointUtils::computeThetas(*this);
        extraInfo.coordsEps = PointUtils::computeCoordsEps(*this);
    }
}
