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

    for (const auto& [key, value] : jsonConfig["dataset_root_path"].items()) {
        config.datasetRootPath[key] = value;
    }

    return config;
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
        std::cout << "Execution time: " << duration.count() << " seconds" << std::endl;
    }

    return 0;
}
