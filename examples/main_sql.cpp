#include <chrono>
#include <fstream>
#include <iostream>
#include <optional>
#include "alice_lri/Core.hpp"
#include "FileUtils.h"
#include <SQLiteCpp/SQLiteCpp.h>
#include <nlohmann/json.hpp>

struct DatasetFrame {
    int64_t id;
    int64_t datasetId;
    std::string datasetName;
    std::string relativePath;
};

struct Config {
    std::string dbDir;
    std::unordered_map<std::string, std::string> datasetRootPath;
};

Config loadConfig() {
    std::ifstream configFile("config.json");
    nlohmann::json jsonConfig;
    configFile >> jsonConfig;

    Config config;
    config.dbDir = jsonConfig["db_dir"];

    for (const auto &[key, value]: jsonConfig["dataset_root_path"].items()) {
        config.datasetRootPath[key] = value;
    }

    return config;
}

std::string endReasonToString(const alice_lri::EndReason &endReason) {
    switch (endReason) {
        case alice_lri::EndReason::ALL_ASSIGNED:
            return "ALL_ASSIGNED";
        case alice_lri::EndReason::MAX_ITERATIONS:
            return "MAX_ITERATIONS";
        case alice_lri::EndReason::NO_MORE_PEAKS:
            return "NO_MORE_PEAKS";
        default:
            throw std::runtime_error("endReasonToString: Unknown endReason");
    }
}

void storeResult(
    const SQLite::Database &db, const int64_t experimentId, const int64_t frameId,
    const alice_lri::DebugIntrinsics &result
) {
    SQLite::Statement frameQuery(
        db, R"(
            INSERT INTO intrinsics_frame_result(experiment_id, dataset_frame_id, points_count, scanlines_count,
                                                vertical_iterations, unassigned_points, end_reason)
            VALUES (?, ?, ?, ?, ?, ?, ?);
            )"
    );

    frameQuery.bind(1, experimentId);
    frameQuery.bind(2, frameId);
    frameQuery.bind(3, result.pointsCount);
    frameQuery.bind(4, static_cast<uint32_t>(result.scanlines.size()));
    frameQuery.bind(5, result.verticalIterations);
    frameQuery.bind(6, result.unassignedPoints);
    frameQuery.bind(7, endReasonToString(result.endReason));

    frameQuery.exec();

    const int64_t frameResultId = db.getLastInsertRowid();

    for (int scanlineIdx = 0; scanlineIdx < result.scanlines.size(); ++scanlineIdx) {
        SQLite::Statement scanlineQuery(
            db, R"(
            INSERT INTO intrinsics_scanline_result(intrinsics_result_id, scanline_idx, points_count, vertical_offset,
                                                        vertical_angle, vertical_ci_offset_lower, vertical_ci_offset_upper,
                                                        vertical_ci_angle_lower, vertical_ci_angle_upper,
                                                        vertical_theoretical_angle_bottom_lower,
                                                        vertical_theoretical_angle_bottom_upper,
                                                        vertical_theoretical_angle_top_lower, vertical_theoretical_angle_top_upper,
                                                        vertical_uncertainty, vertical_last_scanline, vertical_hough_votes,
                                                        vertical_hough_hash, horizontal_offset, horizontal_resolution,
                                                        horizontal_angle_offset, horizontal_heuristic)
            values (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            )"
        );

        const auto &scanline = result.scanlines[scanlineIdx];

        scanlineQuery.bind(1, frameResultId);
        scanlineQuery.bind(2, scanlineIdx);
        scanlineQuery.bind(3, static_cast<int>(scanline.pointsCount));
        scanlineQuery.bind(4, scanline.verticalOffset.value);
        scanlineQuery.bind(5, scanline.verticalAngle.value);
        scanlineQuery.bind(6, scanline.verticalOffset.ci.lower);
        scanlineQuery.bind(7, scanline.verticalOffset.ci.upper);
        scanlineQuery.bind(8, scanline.verticalAngle.ci.lower);
        scanlineQuery.bind(9, scanline.verticalAngle.ci.upper);
        scanlineQuery.bind(10, scanline.theoreticalAngleBounds.lowerLine.lower);
        scanlineQuery.bind(11, scanline.theoreticalAngleBounds.lowerLine.upper);
        scanlineQuery.bind(12, scanline.theoreticalAngleBounds.upperLine.lower);
        scanlineQuery.bind(13, scanline.theoreticalAngleBounds.upperLine.upper);
        scanlineQuery.bind(14, scanline.uncertainty);
        scanlineQuery.bind(15, false);
        scanlineQuery.bind(16, scanline.houghVotes);
        scanlineQuery.bind(17, std::to_string(scanline.houghHash));
        scanlineQuery.bind(18, scanline.horizontalOffset);
        scanlineQuery.bind(19, scanline.resolution);
        scanlineQuery.bind(20, scanline.azimuthalOffset);
        scanlineQuery.bind(21, scanline.horizontalHeuristic);

        scanlineQuery.exec();
    }
}

int main(const int argc, const char **argv) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <process id> <total processes>";
        return -1;
    }

    const int processId = std::stoi(argv[1]);
    const int totalProcesses = std::stoi(argv[2]);
    const Config config = loadConfig();

    std::filesystem::path dbPath = std::filesystem::path(config.dbDir) / std::to_string(processId);
    dbPath += ".sqlite";
    const SQLite::Database db(dbPath, SQLite::OPEN_READWRITE);

    int64_t experimentId = -1;
    SQLite::Statement experimentIdQuery(db, "select max(id) from intrinsics_experiment");

    while (experimentIdQuery.executeStep()) {
        experimentId = experimentIdQuery.getColumn(0);
    }

    if (experimentId <= 0) {
        std::cerr << "No experiment found" << std::endl;
        return -1;
    }

    SQLite::Statement datasetsQuery(db, "select id, name from dataset");
    std::unordered_map<int64_t, std::string> datasetsMap;

    while (datasetsQuery.executeStep()) {
        datasetsMap.emplace(datasetsQuery.getColumn(0), datasetsQuery.getColumn(1));
    }

    SQLite::Statement framesQuery(db, "select id, dataset_id, relative_path from dataset_frame where id % ? == ?");
    framesQuery.bind(1, totalProcesses);
    framesQuery.bind(2, processId);

    std::vector<DatasetFrame> frames;
    while (framesQuery.executeStep()) {
        frames.emplace_back(
            DatasetFrame{
                .id = framesQuery.getColumn(0),
                .datasetId = framesQuery.getColumn(1),
                .datasetName = datasetsMap.at(framesQuery.getColumn(1)),
                .relativePath = framesQuery.getColumn(2),
            }
        );
    }

    std::cout << "Number of frames: " << frames.size() << std::endl;

    for (const DatasetFrame &frame: frames) {
        std::filesystem::path framePath = config.datasetRootPath.at(frame.datasetName);
        framePath /= frame.relativePath;

        std::cout << "Processing frame: " << framePath << std::endl;

        FileUtils::Points points = FileUtils::loadBinaryFile(framePath.string(), std::nullopt);

        const alice_lri::PointCloud::Double cloud(std::move(points.x), std::move(points.y), std::move(points.z));

        auto start = std::chrono::high_resolution_clock::now();
        auto result = alice_lri::debugTrain(cloud);
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> duration = end - start;

        if (!result) {
            std::cerr << result.status().message.c_str();
            throw std::runtime_error("Could not train intrinsics for file: " + framePath.string());
        }

        storeResult(db, experimentId, frame.id, *result);
        std::cout << "Execution time: " << duration.count() << " seconds" << std::endl;
    }

    return 0;
}
