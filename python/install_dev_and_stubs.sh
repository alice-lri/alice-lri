#!/bin/bash
set -e

pushd "$(dirname "${BASH_SOURCE[0]}")" > /dev/null || exit

echo "🐍 Installing Python package in editable mode..."
pip install -e .

echo "📦 Generating Python stubs..."
pybind11-stubgen alice_lri._alice_lri -o .

echo "✅ Installation and stub generation complete!"

popd > /dev/null || exit