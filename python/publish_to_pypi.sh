#!/bin/bash
set -e

echo "🧹 Cleaning previous builds..."
rm -rf dist/ build/ *.egg-info/

echo "🔧 Setting up Conan dependencies..."
rm -rf build
mkdir -p build/lib
conan install ../lib -s compiler.cppstd=gnu20 -s build_type=Release -of build/lib --build=missing

echo "📦 Building wheel..."
python -m build --wheel

echo "🔍 Checking wheel..."
python -m twine check dist/*

echo "🛠️ Repairing wheel with auditwheel..."
auditwheel repair dist/alice_lri*.whl --plat linux_x86_64

echo "📤 Ready to upload to PyPI!"
echo "Run: python -m twine upload wheelhouse/*"
