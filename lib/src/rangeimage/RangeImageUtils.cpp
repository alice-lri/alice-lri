#include "RangeImageUtils.h"
#include "accurate_ri/public_structs.hpp"
#include <Eigen/Core>

#include "utils/Logger.h"
#include "utils/Timer.h"

namespace accurate_ri::RangeImageUtils {
    RangeImage computeRangeImage(
        const IntrinsicsResult &intrinsics, const Eigen::ArrayXd &x, const Eigen::ArrayXd &y, const Eigen::ArrayXd &z
    ) {
        PROFILE_SCOPE("RangeImageUtils::computeRangeImage");
        const Eigen::ArrayXd ranges = (x.square() + y.square() + z.square()).sqrt();
        const Eigen::ArrayXd phis = (z / ranges).asin();

        Eigen::ArrayXi minIndices(phis.size());
        for (int pointIdx = 0; pointIdx < phis.size(); ++pointIdx) {
            double minDiff = std::numeric_limits<double>::max();
            int32_t minIdx = -1;
            for (int laserIdx = 0; laserIdx < intrinsics.vertical.scanlinesCount; ++laserIdx) {
                // TODO full scanlines here is craaaazy
                const double vOffset = intrinsics.vertical.fullScanlines.scanlines[laserIdx].values.offset;
                const double vAngle = intrinsics.vertical.fullScanlines.scanlines[laserIdx].values.angle;

                const double phiCorrection = std::asin(vOffset / ranges(pointIdx));
                const double diff = std::abs(phis(pointIdx) - phiCorrection - vAngle);

                if (diff < minDiff) {
                    minDiff = diff;
                    minIdx = laserIdx;
                }
            }
            minIndices(pointIdx) = minIdx;

            if (minIndices(pointIdx) != intrinsics.vertical.fullScanlines.pointsScanlinesIds[pointIdx]) {
                LOG_ERROR(
                    "MISMATCH: ", pointIdx, " ", intrinsics.vertical.fullScanlines.pointsScanlinesIds[pointIdx], " ", minIndices(pointIdx)
                );
                LOG_INFO("Phi :", phis(pointIdx));
                throw std::runtime_error("MISMATCH");
            }
        }

        return RangeImage();
    }
}
