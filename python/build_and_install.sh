#!/bin/bash
set -e

echo "🔧 Setting up Conan dependencies..."
mkdir -p build/lib
conan install ../lib -s compiler.cppstd=gnu20 -s build_type=Release -of build/lib --build=missing

echo "🐍 Installing Python package..."
pip install -e .

# Copy bindings and core shared objects
mkdir -p accurate_ri/lib
cp build/lib/*.so accurate_ri/lib/
cp build/*.so accurate_ri/

# Generate Python stubs
echo "📦 Generating Python stubs..."
pybind11-stubgen accurate_ri._accurate_ri -o .

echo "✅ Installation and stub generation complete!"
