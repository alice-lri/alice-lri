#pragma once
#include <iostream>
#include <fstream>
#include <vector>
#include <Eigen/Dense>

namespace accurate_ri::TestUtils {
    template <typename DataType>
    Eigen::Matrix<DataType, Eigen::Dynamic, Eigen::Dynamic> loadBinToEigenMatrix(const std::string& filename, int rows, int cols) {
        std::ifstream file(filename, std::ios::binary);
        if (!file.is_open()) {
            std::cerr << "Error: Could not open file " << filename << std::endl;
            return Eigen::Matrix<DataType, Eigen::Dynamic, Eigen::Dynamic>::Zero(0, 0); // Return an empty matrix on error
        }

        // Calculate the total number of elements in the matrix
        long long totalElements = static_cast<long long>(rows) * cols;
        std::vector<DataType> buffer(totalElements);

        // Check sizes match
        file.seekg(0, std::ios::end);
        long long fileSize = file.tellg();
        if (fileSize != totalElements * sizeof(DataType)) {
            std::cerr << "Error: File " << filename << " has incorrect size" << std::endl;
            std::cerr << "Expected: " << totalElements * sizeof(DataType) << " bytes, got: " << fileSize << " bytes" << std::endl;
            file.close();
            return Eigen::Matrix<DataType, Eigen::Dynamic, Eigen::Dynamic>::Zero(0, 0); // Return an empty matrix on error
        }
        file.seekg(0, std::ios::beg);

        // Read the binary data into the buffer
        if (!file.read(reinterpret_cast<char*>(buffer.data()), totalElements * sizeof(DataType))) {
            std::cerr << "Error: Could not read enough data from file " << filename << std::endl;
            file.close();
            return Eigen::Matrix<DataType, Eigen::Dynamic, Eigen::Dynamic>::Zero(0, 0); // Return an empty matrix on error
        }

        file.close();

        // Create Eigen Matrix and map data from buffer
        Eigen::Matrix<DataType, Eigen::Dynamic, Eigen::Dynamic> matrix(rows, cols);
        for (int i = 0; i < rows; ++i) {
            for (int j = 0; j < cols; ++j) {
                matrix(i, j) = buffer[i * cols + j];
            }
        }

        return matrix;
    }
}
