#pragma once
#include "accurate_ri/public_structs.hpp"

namespace accurate_ri::RangeImageUtils {
    RangeImage projectToRangeImage(const Intrinsics &intrinsics, const PointCloud::Float &points);
    RangeImage projectToRangeImage(const Intrinsics &intrinsics, const PointCloud::Double &points);

    PointCloud::Double unProjectToPointCloud(const Intrinsics &intrinsics, const RangeImage &image);
}
