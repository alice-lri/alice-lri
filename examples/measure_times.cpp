#include <chrono>
#include <iostream>
#include <vector>
#include <string>
#include <filesystem>
#include <fstream>
#include <algorithm>
#include <iomanip>
#include <map>
#include "alice_lri/alice_lri.hpp"
#include "FileUtils.h"

struct TimingResult {
    std::string dataset;
    std::string filename;
    double trainTime;
    double projectTime;
    double unprojectTime;
};

struct DatasetConfig {
    std::string name;
    std::string basePath;
    std::vector<std::string> sequences;
};

class TimeMeasurer {
private:
    std::vector<TimingResult> results;
    
    // Dataset configurations based on the existing structure
    std::vector<DatasetConfig> getDatasetConfigs() {
        return {
            {
                "kitti",
                "../../Datasets/LiDAR/kitti/",
                {
                    "2011_09_26/2011_09_26_drive_0001_sync/velodyne_points/data/",
                    // "2011_09_26/2011_09_26_drive_0117_sync/velodyne_points/data/",
                    // "2011_09_28/2011_09_28_drive_0001_sync/velodyne_points/data/",
                    // "2011_09_28/2011_09_28_drive_0222_sync/velodyne_points/data/",
                    // "2011_09_29/2011_09_29_drive_0004_sync/velodyne_points/data/",
                    // "2011_09_29/2011_09_29_drive_0071_sync/velodyne_points/data/",
                    // "2011_09_30/2011_09_30_drive_0016_sync/velodyne_points/data/",
                    // "2011_09_30/2011_09_30_drive_0034_sync/velodyne_points/data/",
                    // "2011_10_03/2011_10_03_drive_0027_sync/velodyne_points/data/",
                    // "2011_10_03/2011_10_03_drive_0047_sync/velodyne_points/data/",
                }
            },
            // {
            //     "durlar",
            //     "../../Datasets/LiDAR/durlar/dataset/DurLAR/",
            //     {
            //         "DurLAR_20210716/ouster_points/data/",
            //         "DurLAR_20210901/ouster_points/data/",
            //         "DurLAR_20211012/ouster_points/data/",
            //         "DurLAR_20211208/ouster_points/data/",
            //         "DurLAR_20211209/ouster_points/data/"
            //     }
            // }
        };
    }
    
    std::pair<std::string, std::string> getFirstAndLastFrameFile(const std::string& sequencePath) {
        try {
            std::filesystem::path dir(sequencePath);
            if (!std::filesystem::exists(dir)) {
                std::cerr << "Directory does not exist: " << sequencePath << std::endl;
                return {"", ""};
            }

            std::vector<std::string> files;
            for (const auto& entry : std::filesystem::directory_iterator(dir)) {
                if (entry.is_regular_file() && entry.path().extension() == ".bin") {
                    files.push_back(entry.path().filename().string());
                }
            }

            if (files.empty()) {
                std::cerr << "No .bin files found in: " << sequencePath << std::endl;
                return {"", ""};
            }

            std::sort(files.begin(), files.end());
            return {files.front(), files.back()};
        } catch (const std::exception& e) {
            std::cerr << "Error accessing directory " << sequencePath << ": " << e.what() << std::endl;
            return {"", ""};
        }
    }
    
    TimingResult measureFrame(const std::string& dataset, const std::string& filePath, const std::string& displayPath) {
        TimingResult result;
        result.dataset = dataset;
        result.filename = displayPath;
        
        try {
            std::cout << "Processing: " << filePath << std::endl;
            
            // Load the point cloud
            FileUtils::Points points = FileUtils::loadBinaryFile(filePath, std::nullopt);
            const alice_lri::PointCloud::Double cloud(std::move(points.x), std::move(points.y), std::move(points.z));
            
            std::cout << "  Loaded " << cloud.x.size() << " points" << std::endl;
            
            // Measure training time
            auto start = std::chrono::high_resolution_clock::now();
            auto intrinsics = alice_lri::train(cloud);

            if (!intrinsics) {
                std::cerr << intrinsics.status().message.c_str() << std::endl;
                throw std::runtime_error("Could not train intrinsics for file: " + filePath);
            }

            auto end = std::chrono::high_resolution_clock::now();
            result.trainTime = std::chrono::duration<double>(end - start).count();
            
            std::cout << "  Train time: " << std::fixed << std::setprecision(6) << result.trainTime << "s" << std::endl;
            
            // Measure projection time
            start = std::chrono::high_resolution_clock::now();
            alice_lri::Result<alice_lri::RangeImage> rangeImage = alice_lri::projectToRangeImage(*intrinsics, cloud);
            end = std::chrono::high_resolution_clock::now();
            result.projectTime = std::chrono::duration<double>(end - start).count();
            
            std::cout << "  Project time: " << std::fixed << std::setprecision(6) << result.projectTime << "s" << std::endl;
            
            // Measure unprojection time
            start = std::chrono::high_resolution_clock::now();
            alice_lri::PointCloud::Double reconstructed = alice_lri::unProjectToPointCloud(*intrinsics, *rangeImage);
            end = std::chrono::high_resolution_clock::now();
            result.unprojectTime = std::chrono::duration<double>(end - start).count();
            
            std::cout << "  Unproject time: " << std::fixed << std::setprecision(6) << result.unprojectTime << "s" << std::endl;
            std::cout << "  Reconstructed " << reconstructed.x.size() << " points" << std::endl;
            
        } catch (const std::exception& e) {
            std::cerr << "Error processing " << filePath << ": " << e.what() << std::endl;
            result.trainTime = -1.0;
            result.projectTime = -1.0;
            result.unprojectTime = -1.0;
        }
        
        return result;
    }
    
public:
    void measureAllDatasets() {
        auto datasets = getDatasetConfigs();
        
        for (const auto& dataset : datasets) {
            std::cout << "\n=== Processing dataset: " << dataset.name << " ===" << std::endl;
            
            for (const auto& sequence : dataset.sequences) {
                std::string sequencePath = dataset.basePath + sequence;
                std::cout << "\n--- Processing sequence: " << sequence << " ---" << std::endl;
                
                auto [firstFrame, lastFrame] = getFirstAndLastFrameFile(sequencePath);
                if (firstFrame.empty()) {
                    std::cerr << "Could not find first frame for sequence: " << sequence << std::endl;
                    continue;
                }

                std::vector<std::string> frames;
                frames.push_back(firstFrame);
                if (dataset.name == "durlar") {
                    frames.push_back(lastFrame);
                }

                for (const auto& frame : frames) {
                    std::string fullPath = sequencePath + frame;
                    std::string displayPath = sequence + frame;
                    TimingResult result = measureFrame(dataset.name, fullPath, displayPath);

                    if (result.trainTime >= 0) {  // Only add valid results
                        results.push_back(result);
                    } else {
                        throw std::runtime_error("Problem measuring times for file: " + fullPath);
                    }
                }
            }
        }
    }
    
    void printResults() {
        if (results.empty()) {
            std::cout << "\nNo valid timing results collected." << std::endl;
            return;
        }
        
        std::cout << "\n" << std::string(100, '=') << std::endl;
        std::cout << "TIMING RESULTS SUMMARY" << std::endl;
        std::cout << std::string(100, '=') << std::endl;
        
        // Print individual results
        std::cout << std::left;
        std::cout << std::setw(10) << "Dataset" 
                  << std::setw(60) << "Filename"
                  << std::setw(12) << "Train(s)"
                  << std::setw(12) << "Project(s)"
                  << std::setw(12) << "Unproject(s)" << std::endl;
        std::cout << std::string(100, '-') << std::endl;
        
        for (const auto& result : results) {
            std::cout << std::setw(10) << result.dataset
                      << std::setw(60) << result.filename
                      << std::setw(12) << std::fixed << std::setprecision(6) << result.trainTime
                      << std::setw(12) << std::fixed << std::setprecision(6) << result.projectTime
                      << std::setw(12) << std::fixed << std::setprecision(6) << result.unprojectTime << std::endl;
        }
        
        // Calculate and print aggregated results by dataset
        std::cout << "\n" << std::string(100, '=') << std::endl;
        std::cout << "AGGREGATED RESULTS BY DATASET (MEAN)" << std::endl;
        std::cout << std::string(100, '=') << std::endl;
        
        // Group by dataset
        std::map<std::string, std::vector<TimingResult>> datasetGroups;
        for (const auto& result : results) {
            datasetGroups[result.dataset].push_back(result);
        }
        
        std::cout << std::left;
        std::cout << std::setw(15) << "Dataset"
                  << std::setw(8) << "Count"
                  << std::setw(15) << "Mean Train(s)"
                  << std::setw(15) << "Mean Project(s)"
                  << std::setw(15) << "Mean Unproject(s)"
                  << std::setw(15) << "Total Mean(s)" << std::endl;
        std::cout << std::string(100, '-') << std::endl;
        
        for (const auto& [datasetName, datasetResults] : datasetGroups) {
            double meanTrain = 0.0, meanProject = 0.0, meanUnproject = 0.0;
            int validCount = 0;
            
            for (const auto& result : datasetResults) {
                if (result.trainTime >= 0) {
                    meanTrain += result.trainTime;
                    meanProject += result.projectTime;
                    meanUnproject += result.unprojectTime;
                    validCount++;
                }
            }
            
            if (validCount > 0) {
                meanTrain /= validCount;
                meanProject /= validCount;
                meanUnproject /= validCount;
                double totalMean = meanTrain + meanProject + meanUnproject;
                
                std::cout << std::setw(15) << datasetName
                          << std::setw(8) << validCount
                          << std::setw(15) << std::fixed << std::setprecision(6) << meanTrain
                          << std::setw(15) << std::fixed << std::setprecision(6) << meanProject
                          << std::setw(15) << std::fixed << std::setprecision(6) << meanUnproject
                          << std::setw(15) << std::fixed << std::setprecision(6) << totalMean << std::endl;
            }
        }
    }
    
    void saveToCSV(const std::string& filename = "timing_results.csv") {
        std::ofstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Error: Could not open file " << filename << " for writing." << std::endl;
            return;
        }
        
        // Write header
        file << "dataset,filename,train_time_s,project_time_s,unproject_time_s,total_time_s\n";
        
        // Write data
        for (const auto& result : results) {
            if (result.trainTime >= 0) {  // Only write valid results
                double totalTime = result.trainTime + result.projectTime + result.unprojectTime;
                file << result.dataset << ","
                     << result.filename << ","
                     << std::fixed << std::setprecision(6) << result.trainTime << ","
                     << std::fixed << std::setprecision(6) << result.projectTime << ","
                     << std::fixed << std::setprecision(6) << result.unprojectTime << ","
                     << std::fixed << std::setprecision(6) << totalTime << "\n";
            }
        }
        
        file.close();
        std::cout << "\nResults saved to: " << filename << std::endl;
    }
};

int main(int argc, char** argv) {
    std::cout << "ALICE-LRI Timing Benchmark" << std::endl;
    std::cout << "Measuring train, project, and unproject times" << std::endl;
    std::cout << "Datasets: KITTI and DurLAR" << std::endl;
    std::cout << std::string(100, '=') << std::endl;
    
    TimeMeasurer measurer;
    
    // Measure timing for all datasets
    measurer.measureAllDatasets();
    
    // Print results to console
    measurer.printResults();
    
    // Save results to CSV file
    measurer.saveToCSV();
    
    std::cout << "\nBenchmark completed!" << std::endl;
    
    return 0;
}
