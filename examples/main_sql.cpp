#include <chrono>
#include <iostream>
#include <optional>
#include "accurate_ri/accurate_ri.hpp"
#include "FileUtils.h"
#include <SQLiteCpp/SQLiteCpp.h>

int main(const int argc, const char **argv) {
    std::string path;
    std::optional<int> accurateDigits = std::nullopt;
    std::optional<std::string> outputPath = std::nullopt;

    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <process id> <total processes>";
        return -1;
    }

    const char *dbDirEnv = std::getenv("LIDAR_SCANLINES_EXPERIMENTS_DB_DIR");
    if (dbDirEnv == nullptr) {
        std::cerr << "Environment variable LIDAR_SCANLINES_EXPERIMENTS_DB_DIR is not set." << std::endl;
        return -1;
    }

    const std::string processId = argv[1];
    const std::string totalProcesses = argv[2];

    std::filesystem::path dbPath = std::filesystem::path(dbDirEnv) / processId;
    dbPath += ".sqlite";
    const SQLite::Database db(dbPath, SQLite::OPEN_READWRITE);

    SQLite::Statement datasetsQuery(db, "select (id, name) from dataset");
    std::unordered_map<int64_t, std::string> results;

    while (datasetsQuery.executeStep()) {
        results.emplace(datasetsQuery.getColumn(0), datasetsQuery.getColumn(1));
    }

    SQLite::Statement query(db, "select * from dataset_frame where id % ? == ?");
    query.bind(1, totalProcesses);
    query.bind(2, processId);

    std::vector<int64_t> framesIds;
    while (query.executeStep()) {
        framesIds.a
    }


    // TODO remove this abomination
    if (path.find("kitti") != std::string::npos) {
        accurate_ri::setResidualThreshold(5e-4);
    } else if (path.find("durlar") != std::string::npos && !accurateDigits.has_value()) {
        accurate_ri::setResidualThreshold(1e-6);
    } else if (path.find("durlar") != std::string::npos && accurateDigits.value() == 6) {
        accurate_ri::setResidualThreshold(1e-5);
    } else if (path.find("durlar") != std::string::npos && accurateDigits.value() == 4) {
        accurate_ri::setResidualThreshold(2e-4);
    }

    FileUtils::Points points = FileUtils::loadBinaryFile(path, accurateDigits);
    accurate_ri::setOutputPath(outputPath);

    auto start = std::chrono::high_resolution_clock::now();
    accurate_ri::IntrinsicsResult result = accurate_ri::execute(points.x, points.y, points.z);
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start;
    std::cout << "Execution time: " << duration.count() << " seconds" << std::endl;

    return 0;
}
