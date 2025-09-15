#!/bin/bash
# Script to regenerate stub files
set -e

echo "Building extension..."
pip install -e .

# Find where libaccurate_ri.so is located
LIBRARY_PATH=$(find build -name "libaccurate_ri.so" -exec dirname {} \; | head -n1)
if [ -z "$LIBRARY_PATH" ]; then
    echo "Could not find libaccurate_ri.so, trying alternative locations..."
    LIBRARY_PATH=$(find . -name "libaccurate_ri.so" -exec dirname {} \; | head -n1)
fi

if [ -z "$LIBRARY_PATH" ]; then
    echo "❌ Could not find libaccurate_ri.so"
    exit 1
fi

echo "📍 Found library at: $LIBRARY_PATH"

echo "Generating stub files..."
# Set LD_LIBRARY_PATH so Python can find the library
export LD_LIBRARY_PATH="$LIBRARY_PATH:$LD_LIBRARY_PATH"

python -c "
import sys
sys.path.insert(0, 'accurate_ri')
"
pybind11-stubgen accurate_ri._accurate_ri -o .

echo "✅ Stubs generated in accurate_ri/_accurate_ri.pyi"
echo "Don't forget to commit the updated stub file!"
