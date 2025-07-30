#pragma once
#include <string>
#include "utils/Logger.h"
#include <filesystem>
#include <variant>
#include <Eigen/Dense>
#include <nlohmann/json.hpp>

#include "hough/HoughTransform.h"
#include "intrinsics/vertical/pool/VerticalScanlinePool.h"
#include "utils/json/JsonConverters.h"

using EigenDataVariant = std::variant<Eigen::ArrayXd, Eigen::ArrayX<bool>>; // List all possible Eigen types

// TODO remove this whole file after debugging
namespace accurate_ri::VerticalLogging {
#if LOG_LEVEL <= LOG_LEVEL_INFO
    void printHeaderDebugInfo(const PointArray &points, const VerticalScanlinePool &hough);
#else
    void printHeaderDebugInfo(const PointArray &points, const VerticalScanlinePool &hough);
#endif

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

        if (iteration % 100 != 64) {
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
