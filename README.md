# ALICE-LRI
<!-- TODO add CI badges -->
<!-- TODO update this readme -->

A high-performance C++ library with Python bindings for LiDAR Range Image processing and intrinsic parameter estimation.

## Features

- **Intrinsic Parameter Estimation**: Estimate LiDAR intrinsic parameters from point cloud data
- **Range Image Projection**: Convert 3D point clouds to 2D range images
- **Point Cloud Reconstruction**: Unproject range images back to 3D point clouds
- **Cross-platform**: Supports Windows, Linux, and macOS
- **Dual Interface**: Native C++ API and Python bindings

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
reconstructed_x, reconstructed_y, reconstructed_z = alice_lri.unproject_to_point_cloud(intrinsics, range_image)
```

### C++

```cpp
#include <alice_lri/Core.hpp>

int main() {
    // Create point cloud
    alice_lri::PointCloud::Float cloud;
    // ... populate cloud data
    
    // Estimate intrinsics
    auto result = alice_lri::estimateIntrinsics(cloud);
    if (!result.ok()) {
        // Handle error
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
- Other dependencies are automatically managed by Conan and pip.

### Python
You can install the Python package from source:
```bash
git clone https://github.com/alice-lri/alice-lri.git
cd alice-lri/python
pip install .
```

### C++


#### Building and installing the C++ library
```bash
git clone https://github.com/alice-lri/alice-lri.git
cd alice-lri/lib
conan install . -s compiler.cppstd=20 -s build_type=Release -of build/ --build=missing
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
sudo make install
```
By default, this will install the library and headers to standard system locations (e.g., `/usr/local/lib`, `/usr/local/include` on Linux).

#### Using ALICE-LRI in your C++ project

##### With CMake

You can link ALICE-LRI in your CMake project as follows:
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

## License

MIT License - see [LICENSE](LICENSE) file.

## Contributing

Please read [CONTRIBUTING.md](CONTRIBUTING.md) for details on our code of conduct and submission process.

## Citation

If you use this library in your research, please cite:

```bibtex
TODO
```
