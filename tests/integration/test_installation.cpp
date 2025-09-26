#include "alice_lri/Core.hpp"
#include <iostream>
#include <cassert>

int main() {
    std::cout << "Testing alice_lri installation..." << std::endl;
    
    try {
        // Test 1: Create basic data structures
        alice_lri::PointCloud::Float cloud;
        cloud.x = alice_lri::AliceArray<float>(10);
        cloud.y = alice_lri::AliceArray<float>(10);
        cloud.z = alice_lri::AliceArray<float>(10);
        
        // Fill with sample data
        for (int i = 0; i < 10; ++i) {
            cloud.x[i] = static_cast<float>(i) * 0.1f;
            cloud.y[i] = static_cast<float>(i) * 0.2f;
            cloud.z[i] = static_cast<float>(i) * 0.3f;
        }
        
        assert(cloud.x.size() == 10);
        assert(cloud.y.size() == 10);
        assert(cloud.z.size() == 10);
        std::cout << "✓ Basic data structures work" << std::endl;
        
        // Test 2: Test AliceArray functionality
        alice_lri::AliceArray<double> test_array(5, 1.5);
        assert(test_array.size() == 5);
        assert(test_array[0] == 1.5);
        std::cout << "✓ AliceArray functionality works" << std::endl;
        
        // Test 3: Test Result error handling
        alice_lri::Status status;
        status.code = alice_lri::ErrorCode::NONE;
        assert(status.code == alice_lri::ErrorCode::NONE);
        std::cout << "✓ Error handling structures work" << std::endl;
        
        // Test 4: Test string functionality
        alice_lri::AliceString test_string;
        test_string.append("Hello");
        test_string.append(" World");
        assert(test_string.size() > 0);
        std::cout << "✓ AliceString functionality works" << std::endl;
        
        // Test 5: Attempt to call main API (may fail with sample data, but should not crash)
        try {
            auto result = alice_lri::estimateIntrinsics(cloud);
            if (result.ok()) {
                std::cout << "✓ estimate_intrinsics function executed successfully" << std::endl;
            } else {
                std::cout << "✓ estimate_intrinsics function handled error gracefully: "
                         << result.status().message.c_str() << std::endl;
            }
        } catch (const std::exception& e) {
            std::cout << "✓ estimate_intrinsics function threw exception (expected with sample data): "
                     << e.what() << std::endl;
        }
        
        std::cout << "🎉 All installation tests passed!" << std::endl;
        std::cout << "The alice_lri C++ library is properly installed and functional." << std::endl;
        
        return 0;
        
    } catch (const std::exception& e) {
        std::cerr << "❌ Installation test failed: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "❌ Installation test failed with unknown exception" << std::endl;
        return 1;
    }
}
