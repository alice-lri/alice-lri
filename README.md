# ALICE-LRI

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
#include <alice_lri/alice_lri.hpp>

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

### Python Package

```bash
pip install alice-lri
```

### C++ Library

#### Using Conan
```bash
conan install alice_lri/0.1.0@
```

#### Using vcpkg
```bash
vcpkg install alice-lri
```

#### Building from source
```bash
git clone https://github.com/alice-lri/alice-lri.git
cd alice-lri
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make install
```

## Dependencies

- **C++20** compatible compiler
- **Eigen3** >= 3.4.0
- **nlohmann/json** >= 3.11.3
- **Python** >= 3.7 (for Python bindings)
- **pybind11** (for Python bindings)

## License

MIT License - see [LICENSE](LICENSE) file.

## Contributing

Please read [CONTRIBUTING.md](CONTRIBUTING.md) for details on our code of conduct and submission process.

## Citation

If you use this library in your research, please cite:

```bibtex
@software{alice_lri_2024,
  title={AccurateRI: High-Performance LiDAR Range Image Processing},
  author={Samuel Soutullo},
  year={2024},
  url={https://github.com/alice-lri/alice-lri}
}
```
