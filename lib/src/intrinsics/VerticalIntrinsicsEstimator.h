#pragma once
#include <memory>
#include <tuple>
#include <tuple>
#include <tuple>

#include "hough/HoughTransform.h"
#include "point/PointArray.h"

namespace accurate_ri {
    class VerticalIntrinsicsEstimator {
    private:
        std::unique_ptr<HoughTransform> hough = nullptr;

    public:
        void estimate(const PointArray &points);

    private:
        void initHough(const PointArray &points);

        static std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd> computeErrorBounds(
            const PointArray &points, double offset
        );

        std::tuple<Eigen::ArrayXi, Eigen::ArrayXd, Eigen::ArrayXd> computeScanlineLimits(
            const PointArray &points, const Eigen::ArrayXd &errorBounds, double offset, double angle, double invRangesShift
        ) const;
    };
} // namespace accurate_ri
