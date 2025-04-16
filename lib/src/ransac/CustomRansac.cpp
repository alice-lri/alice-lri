#include "CustomRansac.h"
#include <limits>

#include "intrinsics/horizontal/helper/HorizontalMath.h"
#include "utils/Logger.h"
#include "utils/Utils.h"

namespace accurate_ri {
    struct MultiLineItem {
        double xMean = 0;
        double yMean = 0;
        int64_t count = 0;

        inline void push(const double x, const double y) {
            count++;
            xMean += (x - xMean) / static_cast<double>(count);
            yMean += (y - yMean) / static_cast<double>(count);
        }
    };

    std::optional<CustomRansacResult> CustomRansac::fit(
        const HorizontalScanlineArray &scanlineArray, const int32_t scanlineIdx
    ) {
        const Eigen::ArrayXd &thetas = scanlineArray.getThetas(scanlineIdx);
        const Eigen::ArrayXd &x = scanlineArray.getInvRangesXy(scanlineIdx);
        const Eigen::ArrayXd y = HorizontalMath::computeDiffToIdeal(thetas, resolution, false);

        const Eigen::ArrayXd &yBounds = scanlineArray.getThetasUpperBounds(scanlineIdx);

        uint32_t trial = 0;

        for (trial = 0; trial < maxTrials; ++trial) {
            const uint32_t sampleIndex1 = std::rand() % y.size();
            uint32_t sampleIndex2 = std::rand() % y.size();
            sampleIndex2 = sampleIndex1 != sampleIndex2 ? sampleIndex2 : (sampleIndex2 + 1) % y.size();

            const auto sampleIndices = Eigen::Array2i(sampleIndex1, sampleIndex2);
            const auto sampleX = x(sampleIndices);
            const auto sampleY = y(sampleIndices);

            model = estimator.fit(sampleX, sampleY);
            refineFit(x, y);

            const Eigen::ArrayXd &residuals = estimator.computeResiduals(x, y);

            const double hOffset = model->slope;
            Eigen::ArrayXd looseBounds = scanlineArray.getCorrectionBounds(scanlineIdx, hOffset);
            looseBounds = 2 * (looseBounds + yBounds) + 1e-6;

            const uint32_t inliersCount = (residuals.abs() < looseBounds).count();

            LOG_DEBUG("CustomRANSAC iteration with offset ", hOffset, " and ", inliersCount, " inliers");

            if (inliersCount == x.size()) {
                LOG_INFO("Found consensus set at trial ", trial, " with offset ", hOffset);
                break;
            }
        }

        if (trial == maxTrials) {
            LOG_DEBUG("RANSAC could not find a valid consensus set at trial ", trial);
            return std::nullopt;
        }

        const std::optional<double> loss = fitToBounds(x, y, scanlineArray, scanlineIdx);

        return loss? std::make_optional<CustomRansacResult>({.model = *model, .loss = *loss}) : std::nullopt;
    }

    void CustomRansac::refineFit(const Eigen::ArrayXd &x, const Eigen::ArrayXd &y) {
        const MultiLineResult &multi = estimator.getLastMultiLine();
        std::unordered_map<int64_t, MultiLineItem> multiLineMap;

        for (uint32_t pointIdx = 0; pointIdx < multi.linesIdx.size(); ++pointIdx) {
            int64_t lineIdx = multi.linesIdx[pointIdx];

            MultiLineItem &multiLineItem = multiLineMap[lineIdx];
            multiLineItem.push(x(pointIdx), y(pointIdx));
        }

        auto shiftedX = Eigen::ArrayXd(x.size());
        auto shiftedY = Eigen::ArrayXd(y.size());

        for (uint32_t pointIdx = 0; pointIdx < multi.linesIdx.size(); ++pointIdx) {
            const MultiLineItem &multiLineItem = multiLineMap.at(multi.linesIdx[pointIdx]);
            shiftedX(pointIdx) = x(pointIdx) - multiLineItem.xMean;
            shiftedY(pointIdx) = y(pointIdx) - multiLineItem.yMean;
        }

        model->slope = (shiftedX * shiftedY).sum() / shiftedX.square().sum();
        estimator.setModel(*model);

        const double interceptCorrection = estimator.computeResiduals(x, y).mean();
        model->intercept += interceptCorrection;
        estimator.setModel(*model);
    }

    std::optional<double> CustomRansac::fitToBounds(
        const Eigen::ArrayXd &x, const Eigen::ArrayXd &y, const HorizontalScanlineArray &scanlineArray,
        const int32_t scanlineIdx
    ) {
        std::optional<double> loss = std::nullopt;
        int32_t iteration;

        for (iteration = 0; iteration < maxFitToBoundsIterations; ++iteration) {
            const double hOffset = model->slope;
            Eigen::ArrayXd residualBounds = scanlineArray.getCorrectionBounds(scanlineIdx, hOffset);
            residualBounds += scanlineArray.getThetasUpperBounds(scanlineIdx) + 1e-6;

            const Eigen::ArrayXd &residuals = estimator.computeResiduals(x, y);

            const auto outlierResidualMask = residuals.abs() >= residualBounds;
            const auto outlierIndices = Utils::eigenMaskToIndices(outlierResidualMask);

            if (outlierIndices.size() == 0) {
                LOG_INFO("Fitted to bounds after ", iteration, " iterations");
                loss = residuals.square().sum();
                break;
            }

            double pivotPoint;
            if (outlierIndices.size() > 1) {
                pivotPoint = x(outlierIndices).mean();
            } else {
                pivotPoint = x.mean();
            }

            const auto leftMask = x <= pivotPoint;
            const auto rightMask = x > pivotPoint;
            const auto leftOutlierMask = leftMask && outlierResidualMask;
            const auto rightOutlierMask = rightMask && outlierResidualMask;
            const auto leftOutlierIndices = Utils::eigenMaskToIndices(leftOutlierMask);
            const auto rightOutlierIndices = Utils::eigenMaskToIndices(rightOutlierMask);

            const bool canPivot = leftOutlierIndices.size() > 0 && rightOutlierIndices.size() > 0;

            if (!canPivot) {
                LOG_DEBUG("Cannot pivot");

                if (std::rand() % 2 == 0) {
                    fitToBoundsModifyIntercept(residuals, residualBounds, outlierIndices);
                } else {
                    fitToBoundsModifySlope(x, residuals, residualBounds, outlierIndices, pivotPoint);
                }

                continue;
            }

            const double leftOutlierMean = residuals(leftOutlierIndices).mean();
            const double rightOutlierMean = residuals(rightOutlierIndices).mean();

            if (leftOutlierMean * rightOutlierMean > 0) {
                fitToBoundsModifyIntercept(residuals, residualBounds, outlierIndices);
            } else if (leftOutlierMean * rightOutlierMean < 0) {
                fitToBoundsModifySlope(x, residuals, residualBounds, outlierIndices, pivotPoint);
            } else {
                LOG_ERROR("Inconsistency found on data when fitting to bounds");
                return std::nullopt;
            }
        }

        if (iteration == maxFitToBoundsIterations) {
            LOG_DEBUG("Fit to bounds not possible");
            return std::nullopt;
        }

        return loss;
    }

    void CustomRansac::fitToBoundsModifyIntercept(
        const Eigen::ArrayXd &residuals, const Eigen::ArrayXd &residualBounds, const Eigen::ArrayXi &outlierIndices
    ) {
        const auto outlierResiduals = residuals(outlierIndices);
        const auto outlierBounds = residualBounds(outlierIndices);
        const auto outlierResidualSigns = outlierResiduals.sign();

        const double interceptCorrectionAbs = (outlierResiduals.abs() - outlierBounds).mean();
        double interceptCorrection = (interceptCorrectionAbs * outlierResidualSigns).mean();

        if (std::abs(interceptCorrection) < 1e-10) {
            interceptCorrection = 1e-10 * Utils::sign(interceptCorrection);
        }

        LOG_DEBUG("Applying intercept correction of ", interceptCorrection);

        model->intercept += interceptCorrection;
    }

    void CustomRansac::fitToBoundsModifySlope(
        const Eigen::ArrayXd &x, const Eigen::ArrayXd &residuals, const Eigen::ArrayXd &residualBounds,
        const Eigen::ArrayXi &outlierIndices, const double pivotPoint
    ) {
        const auto xOutliers = x(outlierIndices);
        const auto outlierResiduals = residuals(outlierIndices);
        const auto outlierBounds = residualBounds(outlierIndices);
        const auto outlierResidualSigns = outlierResiduals.sign();

        const auto diffsToPivot = xOutliers - pivotPoint;

        if ((diffsToPivot == 0).any()) {
            return;
        }

        Eigen::ArrayXd diffsToBounds = outlierResiduals.abs() - outlierBounds;
        diffsToBounds *= outlierResidualSigns;

        Eigen::ArrayXd slopeDeltas = diffsToBounds / diffsToPivot;
        double slopeCorrection = Utils::medianInPlace(slopeDeltas);

        if (std::abs(slopeCorrection) < 1e-10) {
            slopeCorrection = 1e-10 * Utils::sign(slopeCorrection);
        }

        LOG_DEBUG("Applying slope correction of ", slopeCorrection);

        model->slope += slopeCorrection;
    }
} // accurate_ri
