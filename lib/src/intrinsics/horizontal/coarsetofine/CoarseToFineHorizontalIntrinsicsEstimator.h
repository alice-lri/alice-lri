#pragma once
#include "accurate_ri/public_structs.hpp"
#include "point/PointArray.h"

namespace accurate_ri {

class CoarseToFineHorizontalIntrinsicsEstimator {

public:
    HorizontalIntrinsicsResult estimate(const PointArray &points, const VerticalIntrinsicsResult &vertical);

private:
    double computeCoarseLoss(const Eigen::ArrayXd &thetas, const Eigen::ArrayXd &ranges, double offset, double resolution);
    double computePreciseLoss(const Eigen::ArrayXd &thetas, const Eigen::ArrayXd &ranges, double offset, double resolution);

    int32_t optimizeResolutionCoarse(const Eigen::ArrayXd &thetas, const Eigen::ArrayXd &ranges);
    std::pair<double, double> optimizeOffsetCoarse(const Eigen::ArrayXd &thetas, const Eigen::ArrayXd &ranges, double resolution);

    int32_t refineResolutionPrecise(
        const Eigen::ArrayXd &thetas, const Eigen::ArrayXd &ranges, int32_t initialResInt, double offset
    );
    std::pair<double, double> optimizeOffsetPrecise(const Eigen::ArrayXd &thetas, const Eigen::ArrayXd &ranges, double resolution);
};

} // accurate_ri
