#include <eigen3/Eigen/Core>
#include <iostream>
#include <chrono>
#include <cstdlib> // for rand()

int main() {
    using namespace Eigen;
    using namespace std;
    using namespace std::chrono;

    // Define matrix dimensions.
    const int yCount = 30000;
    const int xCount = 1000;

    // Create matrices:
    // hashAccumulator is filled with random (absolute) values.
    Matrix<uint64_t, Dynamic, Dynamic> hashAccumulator =
        Matrix<uint64_t, Dynamic, Dynamic>::Random(yCount, xCount).cwiseAbs();
    // accumulator is initialized with a constant value.
    Matrix<uint64_t, Dynamic, Dynamic> accumulator =
        Matrix<uint64_t, Dynamic, Dynamic>::Constant(yCount, xCount, 42);

    // Define the hash value we're looking for.
    uint64_t hash = 12345;

    // For demonstration, set roughly 1% of hashAccumulator's entries equal to hash.
    for (int i = 0; i < yCount; ++i) {
        for (int j = 0; j < xCount; ++j) {
            if (rand() % 100 == 0) {
                hashAccumulator(i, j) = hash;
            }
        }
    }

    const int iterations = 100;

    // ------------------------
    // Explicit Loop Approach
    // ------------------------
    auto start = high_resolution_clock::now();
    for (int iter = 0; iter < iterations; ++iter) {
        // Make a copy of accumulator so each run starts fresh.
        Matrix<uint64_t, Dynamic, Dynamic> accLoop = accumulator;
        for (int i = 0; i < yCount; ++i) {
            for (int j = 0; j < xCount; ++j) {
                if (hashAccumulator(i, j) == hash) {
                    accLoop(i, j) = 0;
                }
            }
        }
    }
    auto end = high_resolution_clock::now();
    auto loopDuration = duration_cast<milliseconds>(end - start).count();

    // ------------------------
    // Vectorized (Eigen.select) Approach
    // ------------------------
    start = high_resolution_clock::now();
    for (int iter = 0; iter < iterations; ++iter) {
        Matrix<uint64_t, Dynamic, Dynamic> accVectorized = accumulator;
        // Replace elements where hashAccumulator equals hash with 0.
        accVectorized = (hashAccumulator.array() == hash).select(0, accVectorized.array());
        volatile uint64_t dummy = accVectorized.sum();
        (void)dummy;
    }
    end = high_resolution_clock::now();
    auto vectorizedDuration = duration_cast<milliseconds>(end - start).count();

    // Output the timing results.
    cout << "Loop duration: " << loopDuration << " ms, per iteration: " << loopDuration / iterations << " ms" << endl;
    cout << "Vectorized duration: " << vectorizedDuration << " ms, per iteration: " << vectorizedDuration / iterations << " ms" << endl;

    return 0;
}

