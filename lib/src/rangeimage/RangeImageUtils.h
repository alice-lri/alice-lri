#pragma once
#include "alice_lri/Structs.hpp"

namespace alice_lri::RangeImageUtils {
    RangeImage projectToRangeImage(
        const Intrinsics &intrinsics, const PointCloud::Float &points, double emptyValue = 0.0
    );
    RangeImage projectToRangeImage(
        const Intrinsics &intrinsics, const PointCloud::Double &points, double emptyValue = 0.0
    );

    RangeImage projectValuesToRangeImage(
        const Intrinsics &intrinsics, const PointCloud::Float &points, const AliceArray<float> &values,
        double emptyValue = 0.0
    );
    RangeImage projectValuesToRangeImage(
        const Intrinsics &intrinsics, const PointCloud::Double &points, const AliceArray<double> &values,
        double emptyValue = 0.0
    );

    PointCloud::Double unProjectToPointCloud(const Intrinsics &intrinsics, const RangeImage &image);
}
