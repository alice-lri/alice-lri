#include "accurate_ri/Result.h"

namespace accurate_ri {
    AliceString errorMessage(const ErrorCode code) {
        switch (code) {
            case ErrorCode::NONE:
                return AliceString();
            case ErrorCode::RANGES_XY_ZERO:
                return AliceString("Point cloud contains points at (x,y) = (0,0): geometric error");
            default:
                return AliceString("Unknown data validation error");
        }
    }
}
