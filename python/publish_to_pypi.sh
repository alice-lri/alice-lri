#!/bin/bash
set -e

echo "🧹 Cleaning previous builds..."
rm -rf dist/ build/ *.egg-info/

./install_dev_and_stubs.sh

echo "📦 Building wheel..."
python -m build --wheel

echo "🔍 Checking wheel..."
python -m twine check dist/*

echo "🛠️ Repairing wheel with auditwheel..."
auditwheel repair dist/alice_lri*.whl --plat linux_x86_64

echo "📤 Ready to upload to PyPI!"
echo "Run: python -m twine upload wheelhouse/*"
