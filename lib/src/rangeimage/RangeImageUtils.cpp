#include "RangeImageUtils.h"
#include "accurate_ri/public_structs.hpp"
#include <Eigen/Core>

#include "utils/Logger.h"

namespace accurate_ri::RangeImageUtils {
    RangeImage computeRangeImage(
        const IntrinsicsResult &intrinsics, const Eigen::ArrayXd &x, const Eigen::ArrayXd &y, const Eigen::ArrayXd &z
    ) {
        const Eigen::ArrayXd ranges = (x.square() + y.square() + z.square()).sqrt();
        const Eigen::ArrayXd phis = (z / ranges).asin();

        Eigen::ArrayXd vOffsets(intrinsics.vertical.scanlinesCount);
        Eigen::ArrayXd vAngles(intrinsics.vertical.scanlinesCount);
        for (int laserIdx = 0; laserIdx < vOffsets.size(); ++laserIdx) {
            vOffsets(laserIdx) = intrinsics.vertical.fullScanlines.scanlines[laserIdx].values.offset;
            // TODO full scanlines here is craaaazy
            vAngles(laserIdx) = intrinsics.vertical.fullScanlines.scanlines[laserIdx].values.angle;
        }

        const Eigen::ArrayXXd phiCorrectionMat = (vOffsets.matrix() * ranges.inverse().transpose().matrix()).array().
                asin();
        LOG_INFO("Correction Mat: ", phiCorrectionMat.rows(), " x ", phiCorrectionMat.cols());

        for (int offsetIdx = 0; offsetIdx < vOffsets.size(); ++offsetIdx) {
            for (int pointIdx = 0; pointIdx < phis.size(); ++pointIdx) {
                double expected = std::asin(vOffsets(offsetIdx) / ranges(pointIdx));

                if (std::abs(phiCorrectionMat(offsetIdx, pointIdx) - expected) >= 1e-6) {
                    LOG_ERROR("Correction mismatch");
                    LOG_INFO("Expected: ", expected);
                    LOG_INFO("Actual: ", phiCorrectionMat(offsetIdx, pointIdx));
                    throw std::runtime_error("Correction mismatch");
                }
            }
        }

        const Eigen::ArrayXXd phisMat = phis.transpose().replicate(vOffsets.size(), 1);
        const Eigen::ArrayXXd correctedPhisMat = phisMat - phiCorrectionMat;
        // const Eigen::ArrayXXd diffToIdeal = vAngles.replicate()
        LOG_INFO("Corrected Phis Mat: ", correctedPhisMat.rows(), " x ", correctedPhisMat.cols());

        for (int offsetIdx = 0; offsetIdx < vOffsets.size(); ++offsetIdx) {
            for (int pointIdx = 0; pointIdx < phis.size(); ++pointIdx) {
                double expected = phis(pointIdx) - std::asin(vOffsets(offsetIdx) / ranges(pointIdx));

                if (std::abs(correctedPhisMat(offsetIdx, pointIdx) - expected) >= 1e-6) {
                    LOG_ERROR("Corrected mismatch");
                    LOG_INFO("Expected: ", expected);
                    LOG_INFO("Actual: ", correctedPhisMat(offsetIdx, pointIdx));

                    LOG_INFO("Expected phis: ", phis(pointIdx));
                    LOG_INFO("Actual phis: ", phisMat(offsetIdx, pointIdx));

                    LOG_INFO("Expected correction: ", std::asin(vOffsets(offsetIdx) / ranges(pointIdx)));
                    LOG_INFO("Actual correction: ", phiCorrectionMat(offsetIdx, pointIdx));

                    throw std::runtime_error("Correction mismatch");
                }
            }
        }

        const Eigen::ArrayXXd vAnglesMat = vAngles.replicate(1, phis.size());
        LOG_INFO("V Angles Mat: ", vAnglesMat.rows(), " x ", vAnglesMat.cols());

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
