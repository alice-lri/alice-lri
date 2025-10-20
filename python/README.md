# ALICE-LRI Python

ALICE-LRI is a C++ and Python library for lossless range image generation and reconstruction from spinning 3D LiDAR point clouds. It automatically estimates all intrinsic sensor parameters from data, so you do not need calibration files or manufacturer metadata. This enables accurate, sensor-agnostic projection to range images and full recovery of 3D LiDAR data.

This package provides the official Python bindings for the ALICE-LRI core C++ library. It is cross-platform and easy to install with pip.

## Features

- **Intrinsic Parameter Estimation**: Estimate LiDAR intrinsic parameters directly from point cloud data.
- **Range Image Projection**: Convert 3D point clouds to 2D range images with no loss of information.
- **Point Cloud Reconstruction**: Unproject range images back to 3D point clouds, recovering the original data up to numerical precision.
- **No Calibration Files Needed**: Works without manufacturer metadata or calibration files.
- **Fast and Cross-platform**: Powered by a C++ backend, works on Linux, Windows, and macOS.

## Installation

You only need Python >= 3.8 and pip:

```bash
pip install alice-lri
```

## Quick Start

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

## Documentation

- Full documentation and examples: [https://github.com/alice-lri/alice-lri](https://github.com/alice-lri/alice-lri)
- For advanced usage, see the C++ API in the main repository.

## License

MIT License - see [LICENSE](../LICENSE)

## Citation

If you use this library in your research, please cite the ALICE-LRI paper.

TODO: Add BibTeX citation

