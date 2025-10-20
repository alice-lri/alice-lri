# Tests Directory

This directory contains all tests for the alice_lri project.

## Structure

```
tests/
├── README.md                    # This file
├── INSTALLATION_TESTING.md     # Installation testing documentation
├── test_installation.sh        # Manual installation test script
├── integration/                 # Integration tests
│   ├── CMakeLists.txt          # CMake project for C++ installation tests
│   └── test_installation.cpp   # C++ installation test executable
├── *_tests.cpp                 # Unit tests for C++ library
├── CMakeLists.txt              # CMake configuration for unit tests
└── tests.py                    # Python tests (if any)
```

## Running Tests

### All Tests (via GitHub Actions)
Tests run automatically on push/PR to main branches and during releases.

### C++ Unit Tests
```bash
# From repository root
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make
ctest --verbose
```

### Python Tests
```bash
# From python/ directory
pip install -e .[test]
pytest tests/
```

### Installation Tests
```bash
# Manual test script
./tests/test_installation.sh

# C++ integration test
cd tests/integration
mkdir build && cd build
cmake .. -DCMAKE_PREFIX_PATH=/path/to/installation
make && ./installation_test
```

### Individual Test Files

- **`alice_lri_tests.cpp`**: Core library functionality
- **`hough_transform_tests.cpp`**: Hough transform algorithms
- **`point_array_tests.cpp`**: Point cloud data structures
- **`stats_tests.cpp`**: Statistical functions
- **`utils_tests.cpp`**: Utility functions
- **`vertical_intrinsics_tests.cpp`**: Vertical intrinsic estimation

## Test Categories

1. **Unit Tests**: Individual component testing
2. **Integration Tests**: End-to-end functionality
3. **Installation Tests**: Verify installability and basic usage
4. **Platform Tests**: Cross-platform compatibility via CI/CD
