#include "HorizontalIntrinsicsEstimator.h"

#include <map>
#include <numeric>

#include "intrinsics/horizontal/offset/RansacHOffset.h"
#include "intrinsics/horizontal/resolution/MadResolutionLoss.h"
#include "intrinsics/vertical/VerticalStructs.h"
#include "utils/Logger.h"

namespace accurate_ri {
    template<typename T>
    int32_t computeOptimalResolution(const Eigen::ArrayBase<T> &invRangesXy, const Eigen::ArrayBase<T> &thetas) {
        // TODO do not hardcode these
        const int32_t minResolution = invRangesXy.innerSize();
        constexpr int32_t maxResolution = 10000;

        double minLoss = std::numeric_limits<double>::infinity();
        int32_t optimalResolution = -1;

        for (int32_t resolution = minResolution; resolution < maxResolution; ++resolution) {
            const double loss = MadResolutionLoss::computeResolutionLoss(invRangesXy, thetas, resolution);

            if (loss < minLoss) {
                minLoss = loss;
                optimalResolution = resolution;
            }
        }

        return optimalResolution;
    }

    HorizontalIntrinsicsResult HorizontalIntrinsicsEstimator::estimate(
        const PointArray &points, const VerticalIntrinsicsResult &vertical
    ) {
        std::vector<std::vector<int32_t>> pointsByScanline(vertical.scanlinesCount);

        for (int32_t pointIdx = 0; pointIdx < vertical.pointsCount; ++pointIdx) {
            const int32_t scanlineIdx = vertical.fullScanlines.pointsScanlinesIds[pointIdx];

            if (scanlineIdx >= 0) {
                pointsByScanline[scanlineIdx].emplace_back(pointIdx);
            }
        }

        for (auto &scanline: pointsByScanline) {
            std::ranges::sort(
                scanline, [&](const int32_t a, const int32_t b) {
                    return points.getRangeXy(a) < points.getRangeXy(b);
                }
            );
        }

        std::vector<Eigen::ArrayXd> invRangesXyByScanline;
        std::vector<Eigen::ArrayXd> thetasByScanline;

        invRangesXyByScanline.reserve(vertical.scanlinesCount);
        thetasByScanline.reserve(vertical.scanlinesCount);

        for (int scanlineIdx = 0; scanlineIdx < vertical.scanlinesCount; ++scanlineIdx) {
            invRangesXyByScanline.emplace_back(points.getInvRangesXy()(pointsByScanline[scanlineIdx]));

            Eigen::ArrayXd scanlineThetas = points.getThetas()(pointsByScanline[scanlineIdx]);
            scanlineThetas -= scanlineThetas.minCoeff();
            thetasByScanline.emplace_back(std::move(scanlineThetas));
        }

        std::vector<uint32_t> heuristicScanlines;
        std::unordered_map<uint32_t, ScanlineHorizontalInfo> scanlinesInfoMap;

        for (uint32_t scanlineIdx = 0; scanlineIdx < vertical.scanlinesCount; ++scanlineIdx) {
            const auto &invRangesXy = invRangesXyByScanline[scanlineIdx];
            const auto &thetas = thetasByScanline[scanlineIdx];
            const double coordsEps = points.getCoordsEps();

            if (invRangesXy.size() < 2) {
                LOG_WARN("Warning: Scanline ", scanlineIdx, " has less than 2 points, queueing for heuristics");
                heuristicScanlines.emplace_back(scanlineIdx);
                continue;
            }

            int32_t resolution = computeOptimalResolution(invRangesXy, thetas);
            const std::optional<RansacHOffsetResult> rhResult  = RansacHOffset::computeOffset(
                invRangesXy, thetas, resolution, coordsEps
            );

            if (!rhResult.has_value()) {
                LOG_WARN("Warning: Scanline ", scanlineIdx, " has no valid consensus set, queueing for heuristics");
                heuristicScanlines.emplace_back(scanlineIdx);
                continue;
            }

            double offset = rhResult->offset;

            scanlinesInfoMap.emplace(
                scanlineIdx, ScanlineHorizontalInfo{
                    .resolution = resolution,
                    .offset = offset,
                    .heuristic = false
                }
            );

            LOG_INFO(
                "Scanline ", scanlineIdx, ", optimal resolution: ", resolution, ", h: ", offset, ", length: ",
                thetas.size()
            );
        }

        updateScanlinesUseHeuristics(scanlinesInfoMap, heuristicScanlines);

        std::vector<ScanlineHorizontalInfo> scanlinesInfo(vertical.scanlinesCount);

        for (auto it = scanlinesInfoMap.begin(); it != scanlinesInfoMap.end(); ) {
            auto node = scanlinesInfoMap.extract(it++);
            scanlinesInfo[node.key()] = node.mapped();
        }

        return HorizontalIntrinsicsResult{
            .scanlines = std::move(scanlinesInfo)
        };
    }

    void HorizontalIntrinsicsEstimator::updateScanlinesUseHeuristics(
        std::unordered_map<uint32_t, ScanlineHorizontalInfo> &scanlineInfoMap,
        const std::vector<uint32_t> &heuristicScanlines
    ) {
        std::vector<double> otherHOffsets;
        std::vector<int32_t> otherResolutions;

        for (const auto &entry: scanlineInfoMap) {
            otherHOffsets.emplace_back(entry.second.offset);
            otherResolutions.emplace_back(entry.second.resolution);
        }

        for (const auto &scanlineId: heuristicScanlines) {
            auto [resolution, hOffset] = heuristicHAndResolution(otherHOffsets, otherResolutions);

            scanlineInfoMap.emplace(
                scanlineId, ScanlineHorizontalInfo{
                    .resolution = resolution,
                    .offset = hOffset,
                    .heuristic = true
                }
            );

            LOG_INFO("Scanline ", scanlineId, ", resolution: ", resolution, ", h: ", hOffset);
        }
    }

    std::pair<int32_t, double> HorizontalIntrinsicsEstimator::heuristicHAndResolution(
        const std::vector<double> &hOffsets, const std::vector<int32_t> &resolutions
    ) {
        int32_t mostCommonResolution = Stats::intMode(resolutions);
        double medianHOffset = hOffsets[hOffsets.size() / 2];

        return {mostCommonResolution, medianHOffset};
    }
} // accurate_ri
