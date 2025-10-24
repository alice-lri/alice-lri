# ALICE-LRI

ALICE-LRI (Automatic LiDAR Intrinsic Calibration Estimation for Lossless Range Images) is a C++ and Python library for lossless range image generation and reconstruction from spinning 3D LiDAR point clouds. It automatically estimates all intrinsic sensor parameters from data, so you do not need calibration files or manufacturer metadata. This enables accurate, sensor-agnostic projection to range images and full recovery of 3D LiDAR data.

This package provides the official Python bindings for the ALICE-LRI core C++ library. It is cross-platform and easy to install with pip.

## Features

- **Automatic Intrinsic Estimation**: Estimate LiDAR intrinsic parameters from point cloud data. No calibration files or manufacturer metadata needed.
- **Lossless Range Image Projection**: Convert 3D point clouds to 2D range images with zero information loss.
- **Point Cloud Reconstruction**: Unproject range images back to 3D point clouds, recovering original data up to numerical precision.
- **JSON Serialization**: Save and load intrinsic parameters to/from JSON files or strings for easy storage and sharing.
- **Cross-Platform**: Supports Windows, Linux, and macOS (macOS requires building from source).
- **Dual Interface**: Native C++ API and Python bindings for maximum flexibility.

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

```bibtex
@misc{soutullo2025alicelri,
      title={ALICE-LRI: A General Method for Lossless Range Image Generation for Spinning LiDAR Sensors without Calibration Metadata}, 
      author={Samuel Soutullo and Miguel Yermo and David L. Vilariño and Óscar G. Lorenzo and José C. Cabaleiro and Francisco F. Rivera},
      year={2025},
      eprint={2510.20708},
      archivePrefix={arXiv},
      primaryClass={cs.CV},
      url={https://arxiv.org/abs/2510.20708}, 
}
```

**Link to paper**: https://arxiv.org/abs/2510.20708
