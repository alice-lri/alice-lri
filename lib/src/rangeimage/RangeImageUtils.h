#pragma once
#include "alice_lri/public_structs.hpp"

namespace alice_lri::RangeImageUtils {
    RangeImage projectToRangeImage(const Intrinsics &intrinsics, const PointCloud::Float &points);
    RangeImage projectToRangeImage(const Intrinsics &intrinsics, const PointCloud::Double &points);

    PointCloud::Double unProjectToPointCloud(const Intrinsics &intrinsics, const RangeImage &image);
}
