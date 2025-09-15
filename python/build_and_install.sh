#!/bin/bash
set -e

echo "🔧 Setting up Conan dependencies..."
rm -Rf build
mkdir -p build/lib
conan install ../lib -s compiler.cppstd=gnu20 -s build_type=Release -of build/lib --build=missing

echo "🐍 Installing Python package..."
pip install -e .

# Copy bindings and core shared objects
mkdir -p alice_lri/lib
cp build/lib/*.so alice_lri/lib/
cp build/*.so alice_lri/

# Generate Python stubs
echo "📦 Generating Python stubs..."
pybind11-stubgen alice_lri._alice_lri -o .

echo "✅ Installation and stub generation complete!"
