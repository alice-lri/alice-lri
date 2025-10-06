#include <gtest/gtest.h>
#include "alice_lri/util/AliceArray.hpp"
#include <memory>

namespace alice_lri {

// Test helper for tracking constructor/destructor calls
class TestObject {
public:
    static int construct_count;
    static int destruct_count;
    
    int value;
    
    TestObject() : value(0) { ++construct_count; }
    explicit TestObject(int v) : value(v) { ++construct_count; }
    TestObject(const TestObject& other) : value(other.value) { ++construct_count; }
    TestObject(TestObject&& other) noexcept : value(other.value) { 
        other.value = -1;
        ++construct_count; 
    }
    
    TestObject& operator=(const TestObject& other) {
        if (this != &other) value = other.value;
        return *this;
    }
    
    TestObject& operator=(TestObject&& other) noexcept {
        if (this != &other) {
            value = other.value;
            other.value = -1;
        }
        return *this;
    }
    
    ~TestObject() { ++destruct_count; }
    
    static void reset_counters() {
        construct_count = 0;
        destruct_count = 0;
    }
    
    static bool is_balanced() {
        return construct_count == destruct_count;
    }
};

int TestObject::construct_count = 0;
int TestObject::destruct_count = 0;

class AliceArrayTest : public ::testing::Test {
protected:
    void SetUp() override {
        TestObject::reset_counters();
    }
    
    void TearDown() override {
        // Check for memory leaks after each test
        EXPECT_TRUE(TestObject::is_balanced()) 
            << "Memory leak detected! Constructed: " << TestObject::construct_count 
            << ", Destructed: " << TestObject::destruct_count;
    }
};

TEST_F(AliceArrayTest, DefaultConstructor) {
    AliceArray<int> arr;
    EXPECT_EQ(arr.size(), 0);
    EXPECT_TRUE(arr.empty());
    EXPECT_EQ(arr.capacity(), 0);
    EXPECT_EQ(arr.data(), nullptr);
}

TEST_F(AliceArrayTest, SizeConstructor) {
    AliceArray<int> arr(5);
    EXPECT_EQ(arr.size(), 5);
    EXPECT_FALSE(arr.empty());
    EXPECT_GE(arr.capacity(), 5);
    EXPECT_NE(arr.data(), nullptr);
    
    // Check default initialization
    for (uint64_t i = 0; i < arr.size(); ++i) {
        EXPECT_EQ(arr[i], 0);
    }
}

TEST_F(AliceArrayTest, SizeValueConstructor) {
    AliceArray<int> arr(3, 42);
    EXPECT_EQ(arr.size(), 3);
    EXPECT_GE(arr.capacity(), 3);
    
    for (uint64_t i = 0; i < arr.size(); ++i) {
        EXPECT_EQ(arr[i], 42);
    }
}

TEST_F(AliceArrayTest, DataConstructor) {
    int data[] = {1, 2, 3, 4, 5};
    AliceArray<int> arr(data, 5);
    
    EXPECT_EQ(arr.size(), 5);
    EXPECT_GE(arr.capacity(), 5);
    
    for (uint64_t i = 0; i < arr.size(); ++i) {
        EXPECT_EQ(arr[i], data[i]);
    }
}

TEST_F(AliceArrayTest, CopyConstructor) {
    AliceArray<int> original(3, 100);
    AliceArray<int> copy(original);
    
    EXPECT_EQ(copy.size(), original.size());
    EXPECT_EQ(copy.size(), 3);
    
    for (uint64_t i = 0; i < copy.size(); ++i) {
        EXPECT_EQ(copy[i], original[i]);
        EXPECT_EQ(copy[i], 100);
    }
    
    // Modify original to ensure deep copy
    original[0] = 999;
    EXPECT_EQ(copy[0], 100);  // Copy should be unchanged
}

TEST_F(AliceArrayTest, MoveConstructor) {
    AliceArray<int> original(3, 200);
    int* original_data = original.data();
    uint64_t original_size = original.size();
    uint64_t original_capacity = original.capacity();
    
    AliceArray<int> moved(std::move(original));
    
    // Moved-to object should have original's data
    EXPECT_EQ(moved.size(), original_size);
    EXPECT_EQ(moved.capacity(), original_capacity);
    EXPECT_EQ(moved.data(), original_data);
    EXPECT_EQ(moved[0], 200);
    
    // Original should be empty
    EXPECT_EQ(original.size(), 0);
    EXPECT_EQ(original.capacity(), 0);
    EXPECT_EQ(original.data(), nullptr);
}

TEST_F(AliceArrayTest, CopyAssignment) {
    AliceArray<int> original(2, 300);
    AliceArray<int> target(5, 400);
    
    target = original;
    
    EXPECT_EQ(target.size(), original.size());
    EXPECT_EQ(target.size(), 2);
    
    for (uint64_t i = 0; i < target.size(); ++i) {
        EXPECT_EQ(target[i], original[i]);
        EXPECT_EQ(target[i], 300);
    }
}

TEST_F(AliceArrayTest, MoveAssignment) {
    AliceArray<int> original(4, 500);
    AliceArray<int> target(2, 600);
    
    int* original_data = original.data();
    uint64_t original_size = original.size();
    
    target = std::move(original);
    
    EXPECT_EQ(target.size(), original_size);
    EXPECT_EQ(target.data(), original_data);
    EXPECT_EQ(target[0], 500);
    
    // Original should be empty
    EXPECT_EQ(original.size(), 0);
    EXPECT_EQ(original.data(), nullptr);
}

TEST_F(AliceArrayTest, PushBack) {
    AliceArray<int> arr;
    
    for (int i = 0; i < 10; ++i) {
        arr.push_back(i * 10);
        EXPECT_EQ(arr.size(), static_cast<uint64_t>(i + 1));
        EXPECT_EQ(arr[i], i * 10);
    }
    
    EXPECT_EQ(arr.size(), 10);
    EXPECT_GE(arr.capacity(), 10);
}

TEST_F(AliceArrayTest, Resize) {
    AliceArray<int> arr;
    
    // Resize up
    arr.resize(5);
    EXPECT_EQ(arr.size(), 5);
    for (uint64_t i = 0; i < arr.size(); ++i) {
        EXPECT_EQ(arr[i], 0);
    }
    
    // Add values
    for (uint64_t i = 0; i < arr.size(); ++i) {
        arr[i] = static_cast<int>(i * 100);
    }
    
    // Resize down
    arr.resize(3);
    EXPECT_EQ(arr.size(), 3);
    for (uint64_t i = 0; i < arr.size(); ++i) {
        EXPECT_EQ(arr[i], static_cast<int>(i * 100));
    }
    
    // Resize up again
    arr.resize(7);
    EXPECT_EQ(arr.size(), 7);
    for (uint64_t i = 0; i < 3; ++i) {
        EXPECT_EQ(arr[i], static_cast<int>(i * 100));
    }
    for (uint64_t i = 3; i < arr.size(); ++i) {
        EXPECT_EQ(arr[i], 0);
    }
}

TEST_F(AliceArrayTest, Reserve) {
    AliceArray<int> arr;
    arr.push_back(1);
    arr.push_back(2);
    
    uint64_t old_capacity = arr.capacity();
    arr.reserve(100);
    
    EXPECT_GE(arr.capacity(), 100);
    EXPECT_GE(arr.capacity(), old_capacity);
    EXPECT_EQ(arr.size(), 2);
    EXPECT_EQ(arr[0], 1);
    EXPECT_EQ(arr[1], 2);
}

TEST_F(AliceArrayTest, Clear) {
    AliceArray<int> arr(5, 999);
    uint64_t old_capacity = arr.capacity();
    
    arr.clear();
    
    EXPECT_EQ(arr.size(), 0);
    EXPECT_TRUE(arr.empty());
    EXPECT_EQ(arr.capacity(), old_capacity);
}

TEST_F(AliceArrayTest, ShrinkToFit) {
    AliceArray<int> arr;
    arr.reserve(100);
    arr.push_back(1);
    arr.push_back(2);
    
    EXPECT_GE(arr.capacity(), 100);
    EXPECT_EQ(arr.size(), 2);
    
    arr.shrink_to_fit();
    
    EXPECT_EQ(arr.capacity(), arr.size());
    EXPECT_EQ(arr.size(), 2);
    EXPECT_EQ(arr[0], 1);
    EXPECT_EQ(arr[1], 2);
}

TEST_F(AliceArrayTest, Iterators) {
    AliceArray<int> arr;
    for (int i = 0; i < 5; ++i) {
        arr.push_back(i * 2);
    }
    
    EXPECT_EQ(arr.begin(), arr.data());
    EXPECT_EQ(arr.end(), arr.data() + arr.size());
    
    int expected = 0;
    for (auto it = arr.begin(); it != arr.end(); ++it) {
        EXPECT_EQ(*it, expected);
        expected += 2;
    }
}

TEST_F(AliceArrayTest, ObjectLifecycleTracking) {
    TestObject::reset_counters();
    
    {
        AliceArray<TestObject> arr;
        arr.push_back(TestObject(42));
        arr.push_back(TestObject(84));
        
        EXPECT_EQ(arr.size(), 2);
        EXPECT_EQ(arr[0].value, 42);
        EXPECT_EQ(arr[1].value, 84);
        
        arr.resize(5);
        arr.resize(2);
    }
    
    // Objects should be properly destructed
    EXPECT_GE(TestObject::destruct_count, 0);
}

TEST_F(AliceArrayTest, EdgeCases) {
    // Empty array operations
    AliceArray<int> empty;
    empty.clear();
    empty.shrink_to_fit();
    empty.reserve(0);
    
    // Self-assignment
    AliceArray<int> self_test(3, 123);
    self_test = self_test;
    EXPECT_EQ(self_test.size(), 3);
    EXPECT_EQ(self_test[0], 123);
    
    // Null data constructor
    AliceArray<int> null_test(nullptr, 5);
    EXPECT_EQ(null_test.size(), 0);
    EXPECT_TRUE(null_test.empty());
}

} // namespace alice_lri