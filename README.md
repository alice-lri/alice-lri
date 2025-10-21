
# ALICE-LRI

[![Release](https://github.com/alice-lri/alice-lri/actions/workflows/release.yml/badge.svg)](https://github.com/alice-lri/alice-lri/actions/workflows/release.yml)
[![Documentation](https://img.shields.io/badge/docs-online-blue.svg)](https://alice-lri.github.io/alice-lri/)
[![PyPI](https://img.shields.io/pypi/v/alice-lri.svg)](https://pypi.org/project/alice-lri/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

ALICE-LRI is a C++ and Python library for lossless range image generation and reconstruction from spinning 3D LiDAR point clouds. It achieves losslessness by automatically estimating all intrinsic sensor parameters from data, so you do not need calibration files or manufacturer metadata. This enables accurate, sensor-agnostic projection to range images and full recovery of 3D LiDAR data.

This repository is the **core library** of the ALICE-LRI ecosystem, providing the main C++ and Python implementation. It is part of a family of repositories grouped in the [ALICE-LRI GitHub Organization](https://github.com/alice-lri), which together enable reproducible research, experiments, and applications based on ALICE-LRI.

## Organization and Related Repositories

- [ALICE-LRI GitHub Organization](https://github.com/alice-lri): Main organization hosting the ALICE-LRI ecosystem and related projects.
- [ALICE-LRI Experiments](https://github.com/alice-lri/alice-lri-experiments): Code, scripts, and configuration to reproduce experiments for the ALICE-LRI paper. Includes this core library as a submodule.
- [RTST-Modified](https://github.com/alice-lri/rtst-modified): Fork of the original RTST compression algorithm, used for evaluation and comparison in experiments.

You can find more details and usage examples in the related repositories above.

## ALICE-LRI Features

- **Intrinsic Parameter Estimation**: Estimate LiDAR intrinsic parameters from point cloud data.
- **Range Image Projection**: Convert 3D point clouds to 2D range images with no loss of information.
- **Point Cloud Reconstruction**: Unproject range images back to 3D point clouds, recovering the original data up to numerical precision.
- **Cross-platform**: Supports Windows, Linux, and macOS.
- **Dual Interface**: Native C++ API and Python bindings.

## Quick Start

### Python

```python
import alice_lri
import numpy as np

# Sample point cloud data
x = [1.0, 2.0, 3.0]
y = [0.5, 1.5, 2.5] 
z = [0.1, 0.2, 0.3]

# Estimate intrinsic parameters
intrinsics = alice_lri.estimate_intrinsics(x, y, z)

# Project to range image
range_image = alice_lri.project_to_range_image(intrinsics, x, y, z)

# Reconstruct point cloud
rx, ry, rz = alice_lri.unproject_to_point_cloud(intrinsics, range_image)
```

### C++

```cpp
#include <alice_lri/Core.hpp>
#include <iostream> // For std::cerr

int main() {
    // AliceArray is designed to have a similar interface to std::vector
    // Sample point cloud data
    alice_lri::AliceArray<double> x = {1.0, 2.0, 3.0};
    alice_lri::AliceArray<double> y = {4.0, 5.0, 6.0};
    alice_lri::AliceArray<double> z = {7.0, 8.0, 9.0};

    // Create PointCloud::Double using std::move to avoid unnecessary copies
    const alice_lri::PointCloud::Double cloud{std::move(x), std::move(y), std::move(z)};
    
    // Estimate intrinsics
    auto result = alice_lri::estimateIntrinsics(cloud);
    if (!result.ok()) {
        // Handle error
        std::cerr << result.status().message.c_str() << std::endl;
        return 1;
    }
    
    // Project to range image
    auto range_image = alice_lri::projectToRangeImage(*result, cloud);
    
    return 0;
}
```

## Installation

### Python
You only need Python >= 3.8 and pip:
```bash
pip install alice-lri
```

### C++
Pre-built C++ binaries are not available. To use the C++ library, see the "Installation from Source" section below.

## Installation from Source

### Build Dependencies
- **C++20** compatible compiler with **CMake** >= 3.20
- **Python** >= 3.8 and **pip** (for Python bindings)
- **Conan** >= 2.0 (can be installed with `pip install conan`)
  - First time using Conan? Run `conan profile detect` after installing to create a default profile.

Other dependencies are automatically managed by Conan and pip.

### Python
```bash
git clone https://github.com/alice-lri/alice-lri.git
cd alice-lri
pip install ./python
```

### C++
#### Building and installing the C++ library
```bash
# Clone the repository
git clone https://github.com/alice-lri/alice-lri.git
cd alice-lri/lib

# Install dependencies and build
conan install . -s compiler.cppstd=20 -s build_type=Release -of build/ --build=missing
cmake -DCMAKE_BUILD_TYPE=Release -B build

# Install the library
cd build
sudo make install
sudo ldconfig # Update shared library cache on Linux
```
By default, this installs the library and headers to standard system locations (e.g., `/usr/local/lib`, `/usr/local/include` on Linux).

#### Using ALICE-LRI in Your C++ Project
After installing, you no longer need access to the source code. You can link against the installed library.

##### With CMake

You can link ALICE-LRI in your CMake project from anywhere as follows:
```cmake
# ...
find_package(alice_lri REQUIRED)
# ...
target_link_libraries(YOUR_TARGET alice_lri::alice_lri)
```


##### With g++ (or similar compilers)

If you are compiling manually, link with `-lalice_lri`:
```bash
g++ your_source.cpp -lalice_lri -o your_program
```

#### Controlling Log Levels

ALICE-LRI supports configurable log levels at build time. By default, the log level is set to `WARN`, which means only warnings and errors will be printed (recommended for most users; normal operation will not print anything unless something abnormal happens).

You can change the log level when building with CMake by passing the `-DLOG_LEVEL` flag:
```bash
cmake -DCMAKE_BUILD_TYPE=Release -DLOG_LEVEL=DEBUG -B build
```
Valid options are: `DEBUG`, `INFO`, `WARN`, `ERROR`, `NONE`.

For Python, you can control the log level by editing the `pyproject.toml` file in the `python/` directory before building or installing from source. The default is also `WARN`.

**Recommended:** Keep the default `WARN` level unless you need more verbose output for debugging or development.

## Documentation

Full documentation is available online at:

**[https://alice-lri.github.io/alice-lri/](https://alice-lri.github.io/alice-lri/)**

## License

MIT License - see [LICENSE](LICENSE) file.

## Contributing

Please read [CONTRIBUTING.md](CONTRIBUTING.md) for details on our code of conduct and submission process.

## Citation

If you use this library in your research, please cite:

```bibtex
TODO
```
