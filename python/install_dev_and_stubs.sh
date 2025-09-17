#!/bin/bash
set -e

echo "🐍 Installing Python package in editable mode..."
pip install -e .

echo "📦 Generating Python stubs..."
pybind11-stubgen alice_lri._alice_lri -o .

echo "✅ Installation and stub generation complete!"
