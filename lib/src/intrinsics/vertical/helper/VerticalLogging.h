#pragma once
#include <string>
#include "utils/logger/Logger.h"
#include <filesystem>
#include <variant>
#include <Eigen/Dense>
#include <nlohmann/json.hpp>

#include "hough/HoughTransform.h"
#include "intrinsics/vertical/pool/VerticalScanlinePool.h"
#include "utils/json/JsonConverters.h"

using EigenDataVariant = std::variant<Eigen::ArrayXd, Eigen::ArrayX<bool>>; // List all possible Eigen types

namespace accurate_ri::VerticalLogging {

    inline void printHeaderDebugInfo(const PointArray &points, const VerticalScanlinePool &hough) {
        LOG_INFO("==| Parameters |==");
        LOG_INFO("Number of points: ", points.size());
        LOG_INFO("Coord distortion compensation strategy: ", "upper_bound");
        LOG_INFO("Hough model: ", "arcsin");
        LOG_INFO("Voting strategy: ", "single");
        LOG_INFO("Noise radius use square: ", "False");
        LOG_INFO("Fit type: ", "linear");
        LOG_INFO(
            "Offset min: ", hough.getXMin(), ", Offset max: ", hough.getXMax(), ", Offset res: ", hough.getXStep()
        );
        LOG_INFO("Angle min: ", hough.getYMin(), ", Angle max: ", hough.getYMax(), ", Angle res: ", hough.getYStep());
        LOG_INFO("Coords eps: ", points.getCoordsEps());
        LOG_INFO("");
        LOG_INFO("==| Execution |==");
    }

    inline void logHoughInfo(const int64_t iteration, const HoughCell &houghMax) {
        const OffsetAngle &houghValues = houghMax.maxValues;

        LOG_INFO("ITERATION ", iteration);
        LOG_INFO(
            "Offset: ", houghValues.offset, ", Angle: ", houghValues.angle, ", Votes: ", houghMax.votes,
            ", Hash: ", houghMax.hash, ", Hough indices: [", houghMax.maxAngleIndex, "   ", houghMax.maxOffsetIndex,
            "]"
        );
    }

    inline void logScanlineAssignation(const ScanlineInfo& scanline) {
        LOG_INFO("Scanline ", scanline.id, " assigned with ", scanline.pointsCount, " points");
        LOG_INFO(
            "Scanline parameters: Offset: ", scanline.values.offset, ", Angle: ", scanline.values.angle,
            ", Votes: ", scanline.houghVotes, ", Count: ", scanline.pointsCount,
            ", Lower min theoretical angle: ", scanline.theoreticalAngleBounds.bottom.lower,
            ", Lower max theoretical angle: ", scanline.theoreticalAngleBounds.bottom.upper,
            ", Upper min theoretical angle: ", scanline.theoreticalAngleBounds.top.lower,
            ", Upper max theoretical angle: ", scanline.theoreticalAngleBounds.top.upper,
            ", Uncertainty: ", scanline.uncertainty
        );
    }

    // TODO remove these and below after debugging
    template<typename T>
    bool writeBinaryFile(const std::filesystem::path &filePath, const T &data, const std::string &dataName) {
        std::ofstream outFile(filePath, std::ios::binary);

        if (outFile.is_open()) {
            outFile.write(reinterpret_cast<const char *>(data.data()), data.size() * sizeof(data[0]));
            outFile.close();
            return true;
        }

        LOG_ERROR("Failed to open file for writing: " + dataName + ".bin");
        return false;
    }

    inline void plotDebugInfo(
        const PointArray &points, const std::vector<ScanlineInfo> &scanlines, const ScanlineLimits &limits,
        const Eigen::ArrayXi &pointsScanlinesIds, const uint32_t iteration, const std::string &prefix,
        const OffsetAngle &offsetAngle, const double uncertainty
    ) {
        return; // TODO enable disable as required

        if (iteration < 335) {
            return;
        }

        const std::string folder = "scripts/buffers/";
        std::filesystem::path folderPath = folder;

        if (!exists(folderPath) && !create_directories(folderPath)) {
            LOG_ERROR("Failed to create directory: " + folder);
            return; // Or handle directory creation failure as needed
        }

        std::vector<std::pair<std::string, EigenDataVariant> > dataToWrite = {
            {"ranges", points.getRanges()},
            {"phis", points.getPhis()},
            {"scanline_lower_limit", limits.lowerLimit},
            {"scanline_upper_limit", limits.upperLimit},
            {"points_in_scanline_mask", limits.mask}
        };

        for (const auto &item: dataToWrite) {
            std::visit(
                [&](const auto &data) { // Use std::visit to work with variant
                    writeBinaryFile(folderPath / (item.first + ".bin"), data, item.first);
                }, item.second
            ); // Apply lambda to each variant alternative
        }

        Eigen::ArrayX<bool> unassignedMask = (pointsScanlinesIds == -1).cast<bool>(); // Explicitly cast to bool Array
        writeBinaryFile(folderPath / "unassigned_mask.bin", unassignedMask, "unassigned_mask");

        nlohmann::json scanlinesJson;
        for (const ScanlineInfo & scanline : scanlines) {
            scanlinesJson.push_back(scanlineInfoToJson(scanline));
        }

        std::ofstream scanlinesFile(folderPath / "scanlines.json");
        scanlinesFile << scanlinesJson.dump(4);
        scanlinesFile.flush();
        scanlinesFile.close();

        std::string command = "python scripts/plot_debug_info.py " + folder + " " + prefix + " " +
                              std::to_string(iteration) + " " + std::to_string(offsetAngle.offset) + " " +
                              std::to_string(offsetAngle.angle) + " " + std::to_string(uncertainty);
        std::system(command.c_str());
    }
}
