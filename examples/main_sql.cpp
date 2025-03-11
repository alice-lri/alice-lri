#include <chrono>
#include <fstream>
#include <iostream>
#include <optional>
#include "accurate_ri/accurate_ri.hpp"
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

std::string endReasonToString(const accurate_ri::EndReason &endReason) {
    switch (endReason) {
        case accurate_ri::EndReason::ALL_ASSIGNED:
            return "ALL_ASSIGNED";
        case accurate_ri::EndReason::MAX_ITERATIONS:
            return "MAX_ITERATIONS";
        case accurate_ri::EndReason::NO_MORE_PEAKS:
            return "NO_MORE_PEAKS";
        default:
            throw std::runtime_error("endReasonToString: Unknown endReason");
    }
}

void storeResult(
    const SQLite::Database &db, const int64_t experimentId, const int64_t frameId,
    const accurate_ri::IntrinsicsResult &result
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
    frameQuery.bind(3, result.vertical.pointsCount);
    frameQuery.bind(4, result.vertical.scanlinesCount);
    frameQuery.bind(5, result.vertical.iterations);
    frameQuery.bind(6, result.vertical.unassignedPoints);
    frameQuery.bind(7, endReasonToString(result.vertical.endReason));

    frameQuery.exec();

    const int64_t frameResultId = db.getLastInsertRowid();

    for (int scanlineIdx = 0; scanlineIdx < result.vertical.scanlinesCount; ++scanlineIdx) {
        SQLite::Statement scanlineQuery(
            db, R"(
            INSERT INTO intrinsics_result_scanline_info(intrinsics_result_id, scanline_idx, points_count, vertical_offset,
                                                        vertical_angle, vertical_ci_offset_lower, vertical_ci_offset_upper,
                                                        vertical_ci_angle_lower, vertical_ci_angle_upper,
                                                        vertical_theoretical_angle_bottom_lower,
                                                        vertical_theoretical_angle_bottom_upper,
                                                        vertical_theoretical_angle_top_lower, vertical_theoretical_angle_top_upper,
                                                        vertical_uncertainty, vertical_last_scanline, vertical_hough_votes,
                                                        vertical_hough_hash, horizontal_offset, horizontal_resolution,
                                                        horizontal_heuristic)
            values (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            )"
        );

        const auto &verticalScanline = result.vertical.fullScanlines.scanlines[scanlineIdx];
        const auto &horizontalScanline = result.horizontal.scanlines[scanlineIdx];

        scanlineQuery.bind(1, frameResultId);
        scanlineQuery.bind(2, scanlineIdx);
        scanlineQuery.bind(3, static_cast<int>(verticalScanline.pointsCount));
        scanlineQuery.bind(4, verticalScanline.values.offset);
        scanlineQuery.bind(5, verticalScanline.values.angle);
        scanlineQuery.bind(6, verticalScanline.ci.offset.lower);
        scanlineQuery.bind(7, verticalScanline.ci.offset.upper);
        scanlineQuery.bind(8, verticalScanline.ci.angle.lower);
        scanlineQuery.bind(9, verticalScanline.ci.angle.upper);
        scanlineQuery.bind(10, verticalScanline.theoreticalAngleBounds.bottom.lower);
        scanlineQuery.bind(11, verticalScanline.theoreticalAngleBounds.bottom.upper);
        scanlineQuery.bind(12, verticalScanline.theoreticalAngleBounds.top.lower);
        scanlineQuery.bind(13, verticalScanline.theoreticalAngleBounds.top.upper);
        scanlineQuery.bind(14, verticalScanline.uncertainty);
        scanlineQuery.bind(15, false); // TODO
        scanlineQuery.bind(16, verticalScanline.houghVotes);
        scanlineQuery.bind(17, std::to_string(verticalScanline.houghHash));
        scanlineQuery.bind(18, horizontalScanline.offset);
        scanlineQuery.bind(19, horizontalScanline.resolution);
        scanlineQuery.bind(20, horizontalScanline.heuristic);

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
    SQLite::Statement experimentIdQuery(db, "select max(id) from experiment");

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
        if (frame.datasetName == "kitti") {
            accurate_ri::setResidualThreshold(5e-4);
        } else if (frame.datasetName == "durlar") {
            accurate_ri::setResidualThreshold(1e-6);
        } else {
            throw std::runtime_error("Unknown dataset name");
        }

        std::filesystem::path framePath = config.datasetRootPath.at(frame.datasetName);
        framePath /= frame.relativePath;

        const FileUtils::Points points = FileUtils::loadBinaryFile(framePath.string(), std::nullopt);

        auto start = std::chrono::high_resolution_clock::now();
        accurate_ri::IntrinsicsResult result = accurate_ri::execute(points.x, points.y, points.z);
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> duration = end - start;

        storeResult(db, experimentId, frame.id, result);

        std::cout << "Execution time: " << duration.count() << " seconds" << std::endl;
    }

    return 0;
}
