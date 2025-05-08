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
        const auto ranges = (x.square() + y.square() + z.square()).sqrt();
        const auto phis = (z / ranges).asin();

        Eigen::ArrayXd vOffsets(intrinsics.vertical.scanlinesCount);
        Eigen::ArrayXd vAngles(intrinsics.vertical.scanlinesCount);
        for (int laserIdx = 0; laserIdx < vOffsets.size(); ++laserIdx) {
            vOffsets(laserIdx) = intrinsics.vertical.fullScanlines.scanlines[laserIdx].values.offset;
            // TODO full scanlines here is craaaazy
            vAngles(laserIdx) = intrinsics.vertical.fullScanlines.scanlines[laserIdx].values.angle;
        }

        const auto phiCorrectionMat = (vOffsets.matrix() * ranges.inverse().transpose().matrix()).array().asin();
        const auto correctedPhisMat = phis.transpose().replicate(vOffsets.size(), 1) - phiCorrectionMat;

        const auto vAnglesMat = vAngles.replicate(1, phis.size());
        const Eigen::ArrayXXd diffToIdealMat = (correctedPhisMat - vAnglesMat).abs();

        Eigen::ArrayXi minIndices(phis.size());
        for (int i = 0; i < vOffsets.size(); ++i) {
            int32_t minRow;
            diffToIdealMat.col(i).minCoeff(&minRow);
            minIndices(i) = minRow;

            if (minIndices(i) != intrinsics.vertical.fullScanlines.pointsScanlinesIds[i]) {
                LOG_ERROR(
                    "MISMATCH: ", i, " ", intrinsics.vertical.fullScanlines.pointsScanlinesIds[i], " ", minIndices(i)
                );
                LOG_INFO("Phi :", phis(i));
                LOG_INFO(
                    "V angle :", vAngles(intrinsics.vertical.fullScanlines.pointsScanlinesIds[i]), " ",
                    vAngles(minIndices(i))
                );
                LOG_INFO(
                    "Diffs :", diffToIdealMat(intrinsics.vertical.fullScanlines.pointsScanlinesIds[i], i), " ",
                    diffToIdealMat(minIndices(i), i)
                );
                throw std::runtime_error("MISMATCH");
            }
        }


        return RangeImage();
    }
}
