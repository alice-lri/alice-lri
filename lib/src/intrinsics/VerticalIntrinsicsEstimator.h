#pragma once
#include <memory>
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
    };
} // namespace accurate_ri
