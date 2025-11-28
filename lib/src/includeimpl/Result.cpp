#include "alice_lri/Result.hpp"

namespace alice_lri {
    AliceString errorMessage(const ErrorCode code) {
        switch (code) {
            case ErrorCode::NONE:
                return AliceString();
            case ErrorCode::MISMATCHED_SIZES:
                return AliceString("Sizes of X, Y and Z (or other values if provided) do not match");
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
