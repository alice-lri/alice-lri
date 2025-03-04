
#pragma once
#include "point/PointArray.h"

namespace accurate_ri {

class RansacHOffset {
    void computeOffset(PointArray &points, uint32_t scanlineId, uint32_t resolution);
};

} // accurate_ri
