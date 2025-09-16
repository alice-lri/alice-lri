#include <optional>
#include "alice_lri/alice_lri.hpp"
#include "FileUtils.h"
#include <vector>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <numeric>
#include <bits/ostream.tcc>

template <typename T>
std::vector<size_t> argsort(const alice_lri::AliceArray<T>& v) {
    std::vector<size_t> indices(v.size());
    for (size_t i = 0; i < indices.size(); ++i) {
        indices[i] = i;
    }
    std::stable_sort(indices.begin(), indices.end(),
              [&v](size_t i1, size_t i2) { return v[i1] < v[i2]; });
    return indices;
}

template <typename T>
alice_lri::AliceArray<T> apply_argsort(const alice_lri::AliceArray<T>& data, const std::vector<size_t>& indices) {
    alice_lri::AliceArray<T> sorted;
    sorted.reserve(indices.size());
    for (size_t idx : indices) {
        sorted.emplace_back(data[idx]);
    }
    return sorted;
}

template <typename T>
std::vector<size_t> lex_argsort(
    const alice_lri::AliceArray<T>& x, const alice_lri::AliceArray<T>& y, const alice_lri::AliceArray<T>& z
) {
    std::vector<size_t> indices(x.size());
    std::iota(indices.begin(), indices.end(), 0);
    std::stable_sort(indices.begin(), indices.end(), [&](size_t i, size_t j) {
        if (x[i] != x[j]) return x[i] < x[j];
        if (y[i] != y[j]) return y[i] < y[j];
        return z[i] < z[j];
    });
    return indices;
}


int main(int argc, char **argv) {
    const std::string trainPath = "../../Datasets/LiDAR/durlar/dataset/DurLAR/DurLAR_20210901/ouster_points/data/0000000000.bin";
    const std::string targetPath = "../../Datasets/LiDAR/durlar/dataset/DurLAR/DurLAR_20211209/ouster_points/data/0000007398.bin";

    FileUtils::Points trainPoints = FileUtils::loadBinaryFile(trainPath, std::nullopt);
    FileUtils::Points targetPoints = FileUtils::loadBinaryFile(targetPath, std::nullopt);
    // FileUtils::Points targetPoints = FileUtils::loadBinaryFile(trainPath, std::nullopt);

    const alice_lri::PointCloud::Double trainCloud(std::move(trainPoints.x), std::move(trainPoints.y), std::move(trainPoints.z));
    const alice_lri::PointCloud::Double targetCloud(std::move(targetPoints.x), std::move(targetPoints.y), std::move(targetPoints.z));

    alice_lri::Result<alice_lri::Intrinsics> result = alice_lri::train(trainCloud);

    if (!result) {
        std::cout << result.status().message.c_str();
        return 1;
    }

    const auto ri = alice_lri::projectToRangeImage(*result, targetCloud);
    const alice_lri::PointCloud::Double reconstructed = alice_lri::unProjectToPointCloud(*result, *ri);

    const auto originalSort = lex_argsort(targetCloud.x, targetCloud.y, targetCloud.z);
    const auto reconstructedSort = lex_argsort(reconstructed.x, reconstructed.y, reconstructed.z);

    const auto sortedTargetX = apply_argsort(targetCloud.x, originalSort);
    const auto sortedTargetY = apply_argsort(targetCloud.y, originalSort);
    const auto sortedTargetZ = apply_argsort(targetCloud.z, originalSort);

    const auto sortedReconstructedX = apply_argsort(reconstructed.x, reconstructedSort);
    const auto sortedReconstructedY = apply_argsort(reconstructed.y, reconstructedSort);
    const auto sortedReconstructedZ = apply_argsort(reconstructed.z, reconstructedSort);

    for (int i = 0; i < 5; ++i) {
        std::cout << "Target: " << sortedTargetX[i] << ", " << sortedTargetY[i] << ", " << sortedTargetZ[i] << std::endl;
        std::cout << "Reconstructed: " << sortedReconstructedX[i] << ", " << sortedReconstructedY[i] << ", " << sortedReconstructedZ[i] << std::endl;
    }

    std::vector<double> targetRanges(sortedTargetX.size(), 0);
    std::vector<double> reconstructedRanges(sortedReconstructedX.size(), 0);
    std::vector<double> rangesDiff(sortedTargetX.size(), 0);
    double meanRangesDiff = 0;

    for (int i = 0; i < targetRanges.size(); ++i) {
        targetRanges[i] = std::sqrt(
            sortedTargetX[i] * sortedTargetX[i] +
            sortedTargetY[i] * sortedTargetY[i] +
            sortedTargetZ[i] * sortedTargetZ[i]
        );

        reconstructedRanges[i] = std::sqrt(
            sortedReconstructedX[i] * sortedReconstructedX[i] +
            sortedReconstructedY[i] * sortedReconstructedY[i] +
            sortedReconstructedZ[i] * sortedReconstructedZ[i]
        );

        rangesDiff[i] = std::abs(targetRanges[i] - reconstructedRanges[i]);
        meanRangesDiff += rangesDiff[i] / static_cast<double>(targetRanges.size());
    }

    std::cout << meanRangesDiff << std::endl;

    return 0;
}
