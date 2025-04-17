#include "CustomRansac.h"
#include <limits>

#include "intrinsics/horizontal/helper/HorizontalMath.h"
#include "utils/Logger.h"
#include "utils/Utils.h"

namespace accurate_ri {
    struct MultiLineItem {
        double xWeightedSum = 0;
        double yWeightedSum = 0;
        double weightSum = 0;

        inline void push(const double x, const double y, const double weight) {
            xWeightedSum += x * weight;
            yWeightedSum += y * weight;
            weightSum += weight;
        }

        [[nodiscard]] inline double xMean() const { return xWeightedSum / weightSum; }
        [[nodiscard]] inline double yMean() const { return yWeightedSum / weightSum; }
    };

    uint32_t my_random() {
        static uint32_t state = 42;
        static const uint32_t a = 1664525;
        static const uint32_t c = 1013904223;
        state = a * state + c;  // wraps automatically on 32-bit unsigned
        return state;
    }

    std::optional<CustomRansacResult> CustomRansac::fit(
        const HorizontalScanlineArray &scanlineArray, const int32_t scanlineIdx
    ) {
        const Eigen::ArrayXd &thetas = scanlineArray.getThetas(scanlineIdx);
        const Eigen::ArrayXd &x = scanlineArray.getInvRangesXy(scanlineIdx);
        const Eigen::ArrayXd y = HorizontalMath::computeDiffToIdeal(thetas, resolution, false);

        const Eigen::ArrayXd &yBounds = scanlineArray.getThetasUpperBounds(scanlineIdx);

        std::optional<double> loss = std::nullopt;
        uint32_t trial = 0;

        for (trial = 0; trial < maxTrials; ++trial) {
            const uint32_t sampleIndex1 = my_random() % y.size();
            uint32_t sampleIndex2 = my_random() % y.size();
            sampleIndex2 = sampleIndex1 != sampleIndex2 ? sampleIndex2 : (sampleIndex2 + 1) % y.size();

            const auto sampleIndices = Eigen::Array2i(sampleIndex1, sampleIndex2);
            const auto sampleX = x(sampleIndices);
            const auto sampleY = y(sampleIndices);

            model = estimator.fit(sampleX, sampleY);
            const Eigen::ArrayXd weights = yBounds.inverse().square();
            refineFit(x, y, weights);

            const double hOffset = model->slope;
            if (std::abs(hOffset) >= scanlineArray.getRangesXy(scanlineIdx).minCoeff()) { // TODO optimize this
                continue;
            }

            Eigen::ArrayXd looseBounds = scanlineArray.getCorrectionBounds(scanlineIdx, hOffset);
            looseBounds = 1.1 * (looseBounds + yBounds) + 1e-6;

            const Eigen::ArrayXd &residuals = estimator.computeResiduals(x, y);
            const uint32_t inliersCount = (residuals.abs() < looseBounds).count();

            LOG_DEBUG("CustomRANSAC iteration with offset ", hOffset, " and ", inliersCount, " inliers");

            if (inliersCount == x.size()) {
                LOG_DEBUG("Found consensus set at trial ", trial, " with offset ", hOffset);
                loss = residuals.square().sum();
                break;
            }
        }

        if (trial == maxTrials) {
            LOG_DEBUG("RANSAC could not find a valid consensus set at trial ", trial);
            return std::nullopt;
        }

        return loss? std::make_optional<CustomRansacResult>({.model = *model, .loss = *loss}) : std::nullopt;
    }

    void CustomRansac::refineFit(const Eigen::ArrayXd &x, const Eigen::ArrayXd &y, const Eigen::ArrayXd &weights) {
        estimator.computeResiduals(x, y); // TODO this is done to update the multilineresult, but maybe it can be avoided
        const MultiLineResult &multi = estimator.getLastMultiLine();
        std::unordered_map<int64_t, MultiLineItem> multiLineMap;

        const double totalWeight = weights.sum();

        for (uint32_t pointIdx = 0; pointIdx < multi.linesIdx.size(); ++pointIdx) {
            int64_t lineIdx = multi.linesIdx[pointIdx];

            MultiLineItem &multiLineItem = multiLineMap[lineIdx];
            multiLineItem.push(x(pointIdx), y(pointIdx), weights(pointIdx));
        }

        auto shiftedX = Eigen::ArrayXd(x.size());
        auto shiftedY = Eigen::ArrayXd(y.size());

        for (uint32_t pointIdx = 0; pointIdx < multi.linesIdx.size(); ++pointIdx) {
            const MultiLineItem &multiLineItem = multiLineMap.at(multi.linesIdx[pointIdx]);
            shiftedX(pointIdx) = x(pointIdx) - multiLineItem.xMean();
            shiftedY(pointIdx) = y(pointIdx) - multiLineItem.yMean();
        }

        model->slope = (weights * shiftedX * shiftedY).sum() / (weights * shiftedX.square()).sum();
        estimator.setModel(*model);

        const double interceptCorrection = (estimator.computeResiduals(x, y) * weights).sum() / totalWeight;
        model->intercept += interceptCorrection;
        estimator.setModel(*model); // TODO having to sync the model every time like this is crazy and shitty
    }

    // TODO maybe at some point recover this, but not for now
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
                LOG_DEBUG("Fitted to bounds after ", iteration, " iterations");
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
                LOG_INFO("Cannot pivot");

                if (my_random() % 2 == 0) {
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
            LOG_INFO("Fit to bounds not possible");
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

        LOG_INFO("Applying intercept correction of ", interceptCorrection);

        model->intercept += interceptCorrection;
        estimator.setModel(*model);
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

        LOG_INFO("Applying slope correction of ", slopeCorrection);

        model->slope += slopeCorrection;
        estimator.setModel(*model);
    }
} // accurate_ri
