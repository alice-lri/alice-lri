#pragma once
#include "alice_lri/Structs.hpp"

namespace alice_lri::RangeImageUtils {
    RangeImage projectToRangeImage(const Intrinsics &intrinsics, const PointCloud::Float &points);
    RangeImage projectToRangeImage(const Intrinsics &intrinsics, const PointCloud::Double &points);

    RangeImage projectValuesToRangeImage(const Intrinsics &intrinsics, const PointCloud::Float &points, const AliceArray<float> &values);
    RangeImage projectValuesToRangeImage(const Intrinsics &intrinsics, const PointCloud::Double &points, const AliceArray<double> &values);

    PointCloud::Double unProjectToPointCloud(const Intrinsics &intrinsics, const RangeImage &image);
}
