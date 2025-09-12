#include "accurate_ri/Result.h"

namespace accurate_ri {
    AliceString errorMessage(const ErrorCode code) {
        switch (code) {
            case ErrorCode::NONE:
                return AliceString();
            case ErrorCode::MISMATCHED_SIZES:
                return AliceString("Input cloud X, Y and Z sizes do not match");
            case ErrorCode::EMPTY_POINT_CLOUD:
                return AliceString("Point cloud is empty");
            case ErrorCode::RANGES_XY_ZERO:
                return AliceString("Point cloud contains points at (x,y) = (0,0): geometric error");
            case ErrorCode::INTERNAL_ERROR:
                return AliceString("Internal error");
            default:
                return AliceString("Unknown data validation error");
        }
    }
}
