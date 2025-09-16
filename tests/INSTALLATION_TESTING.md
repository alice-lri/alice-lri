# Installation Testing

This directory contains tests to verify that both the C++ and Python libraries can be properly installed and used.

## Running Installation Tests

### Automated Testing (GitHub Actions)

The GitHub Actions workflow automatically tests installation on every push and pull request:

- **C++ Installation Test**: Builds and installs the C++ library, then creates a test consumer project
- **Python Installation Test**: Installs the Python package from source and runs basic functionality tests
- **Source Distribution Test**: Creates and tests installation from Python source distribution

### Manual Testing

#### Quick Test Script

Run the provided test script to check your installation:

```bash
# From repository root
chmod +x tests/test_installation.sh
./tests/test_installation.sh
```

#### Python Package Testing

```bash
# Test installation from source
cd python
pip install -e .[test]
pytest tests/

# Test installation from built package
pip install .
python -c "import alice_lri; print('Installation successful!')"
```

#### C++ Library Testing

```bash
# Build and install
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local
make install

# Test with consumer project
cd ../tests/integration
mkdir build && cd build
cmake .. -DCMAKE_PREFIX_PATH=/usr/local
make
./installation_test
```

## Test Coverage

### Python Tests (`python/tests/test_installation.py`)

- ✅ Package import
- ✅ Function availability
- ✅ Error code enums
- ✅ Basic data structures
- ✅ Minimal functionality test

### C++ Tests (`tests/integration/test_installation.cpp`)

- ✅ Header inclusion
- ✅ Basic data structure creation
- ✅ AliceArray functionality
- ✅ Error handling structures
- ✅ String operations
- ✅ Main API function calls

### GitHub Actions Tests

- ✅ Multi-platform builds (Ubuntu, Windows, macOS)
- ✅ Multiple Python versions (3.8-3.12)
- ✅ Both Debug and Release builds for C++
- ✅ Source distribution creation and testing
- ✅ Wheel building and testing
- ✅ Consumer project compilation

## Troubleshooting

### Common Issues

1. **Missing Conan dependencies**: Run `conan install` in the lib directory
2. **CMake not finding alice_lri**: Set `CMAKE_PREFIX_PATH` to installation directory
3. **Python import errors**: Ensure all dependencies are installed with `pip install -e .[test]`

### Platform-Specific Notes

- **Windows**: May require Visual Studio Build Tools
- **macOS**: Ensure Xcode Command Line Tools are installed  
- **Linux**: May need to install development packages (build-essential, cmake)

## Integration with CI/CD

The installation tests are integrated into the GitHub Actions workflow and must pass before:

- ✅ Merging pull requests
- ✅ Publishing releases
- ✅ Uploading to PyPI

This ensures that every release is installable and functional across all supported platforms.
