#include <iostream>
#include <Eigen/Dense>
#include <chrono>

// Lazy version: returns an Eigen expression (unevaluated)
template<typename Derived>
inline auto lazy_diff(const Eigen::ArrayBase<Derived>& arr) {
    return arr.tail(arr.size() - 1) - arr.head(arr.size() - 1);
}

// Eager version: returns an evaluated result
template<typename Derived>
inline Eigen::ArrayX<Derived> eval_diff(const Eigen::ArrayX<Derived>& arr) {
    return (arr.tail(arr.size() - 1) - arr.head(arr.size() - 1));
}

int main() {
    const int N = 1000000;       // Size of the array
    const int iterations = 10000; // Number of iterations for benchmarking

    // Create a large random Eigen array
    Eigen::ArrayXd arr = Eigen::ArrayXd::Random(N);

    // Prevent the optimizer from eliminating our loop
    double dummy = 0.0;

    // Benchmark lazy_diff (forcing evaluation via .sum())
    auto start_lazy = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < iterations; i++) {
        // The lazy expression is evaluated when .sum() is called
        dummy += lazy_diff(arr).sum();
    }
    auto end_lazy = std::chrono::high_resolution_clock::now();
    auto lazy_duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_lazy - start_lazy).count();

    // Benchmark eval_diff (which is already evaluated)
    auto start_eval = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < iterations; i++) {
        dummy += eval_diff(arr).sum();
    }
    auto end_eval = std::chrono::high_resolution_clock::now();
    auto eval_duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_eval - start_eval).count();

    std::cout << "Lazy diff time: " << lazy_duration << " ms" << std::endl;
    std::cout << "Eval diff time: " << eval_duration << " ms" << std::endl;
    std::cout << "Dummy (to prevent optimization): " << dummy << std::endl;

    return 0;
}
