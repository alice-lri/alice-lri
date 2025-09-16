#!/bin/bash

# Installation test script for alice_lri
# This script tests both C++ and Python installations

set -e

echo "🧪 Testing alice_lri Installation"
echo "================================="

# Test Python installation
echo ""
echo "📦 Testing Python package..."
python3 -c "
import sys
print(f'Python version: {sys.version}')

try:
    import alice_lri
    print(f'✅ alice_lri imported successfully')
    print(f'   Version: {alice_lri.__version__}')
    
    # Test basic functionality
    available_attrs = [attr for attr in dir(alice_lri) if not attr.startswith('_')]
    print(f'   Available functions: {len(available_attrs)}')
    
    # Test specific functions exist
    required_functions = ['train', 'project_to_range_image_float', 'unproject_to_point_cloud']
    missing_functions = [f for f in required_functions if not hasattr(alice_lri, f)]
    
    if missing_functions:
        print(f'❌ Missing functions: {missing_functions}')
        sys.exit(1)
    else:
        print(f'✅ All required functions available')
        
except ImportError as e:
    print(f'❌ Failed to import alice_lri: {e}')
    sys.exit(1)
except Exception as e:
    print(f'❌ Error testing alice_lri: {e}')
    sys.exit(1)
"

echo ""
echo "🔧 Testing C++ library (if installed)..."

# Check if alice_lri can be found by CMake
CMAKE_TEST_DIR=$(mktemp -d)
cd "$CMAKE_TEST_DIR"

cat > CMakeLists.txt << 'EOF'
cmake_minimum_required(VERSION 3.20)
project(test_find_alice_lri)
find_package(alice_lri QUIET)

if(alice_lri_FOUND)
    message(STATUS "alice_lri found successfully")
    message(STATUS "alice_lri version: ${alice_lri_VERSION}")
else()
    message(STATUS "alice_lri not found - may not be installed system-wide")
endif()
EOF

if cmake . > /dev/null 2>&1; then
    if grep -q "alice_lri found successfully" CMakeCache.txt 2>/dev/null; then
        echo "✅ C++ library found by CMake"
        VERSION=$(grep "alice_lri_VERSION" CMakeCache.txt | cut -d'=' -f2 2>/dev/null || echo "unknown")
        echo "   Version: $VERSION"
    else
        echo "ℹ️  C++ library not found system-wide (this is normal for local builds)"
        echo "   Use CMAKE_PREFIX_PATH to point to installation directory"
    fi
else
    echo "ℹ️  CMake test failed - this is normal if CMake is not installed"
fi

# Cleanup
cd - > /dev/null
rm -rf "$CMAKE_TEST_DIR"

echo ""
echo "✅ Installation test completed successfully!"
echo ""
echo "📚 Usage examples:"
echo "   Python: python3 -c 'import alice_lri; print(alice_lri.__version__)'"
echo "   C++:    cmake -Dalice_lri_DIR=/path/to/alice_lri/lib/cmake/alice_lri"
