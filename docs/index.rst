ALICE-LRI
=========

Lossless range image generation and reconstruction for spinning 3D LiDAR point clouds.

What is ALICE-LRI?
------------------

ALICE-LRI is a C++ and Python library for lossless range image generation and reconstruction from spinning 3D LiDAR point clouds. It automatically estimates all intrinsic sensor parameters from data—no calibration files or vendor metadata needed. This enables accurate, sensor-agnostic projection to range images and full recovery of 3D LiDAR data.

Features
--------

- **Automatic Intrinsic Estimation**: Estimate LiDAR intrinsic parameters from point cloud data. No calibration files or manufacturer metadata needed.
- **Lossless Range Image Projection**: Convert 3D point clouds to 2D range images with zero information loss.
- **Point Cloud Reconstruction**: Unproject range images back to 3D point clouds, recovering original data up to numerical precision.
- **JSON Serialization**: Save and load intrinsic parameters to/from JSON files or strings for easy storage and sharing.
- **Cross-Platform**: Supports Windows, Linux, and macOS (macOS requires building from source).
- **Dual Interface**: Native C++ API and Python bindings for maximum flexibility.

Get Started
-----------

New here? Start with installation, examples, logging, and integration tips:

.. grid:: 1 2 2 2
   :gutter: 2

   .. grid-item-card:: Installation
      :link: installation.html
      :text-align: left

      Install from PyPI or build from source. Link the C++ library with CMake or g++, configure logging.

   .. grid-item-card:: Getting Started
      :link: getting_started.html
      :text-align: left

      Quick intro, features, and your first Python and C++ examples.

API Reference
-------------

When you're ready to dive deeper, browse the auto-generated references:

.. grid:: 1 2 2 2
   :gutter: 2

   .. grid-item-card:: Python API
      :link: python_api.html
      :text-align: left

      Modules, functions, and types available from the Python package.

   .. grid-item-card:: C++ API
      :link: cpp_api.html
      :text-align: left

      Classes, functions, and namespaces generated from headers and sources.

Project Info
------------

- **Repository:** `https://github.com/alice-lri/alice-lri <https://github.com/alice-lri/alice-lri>`_
- **License:** MIT — see the :file:`LICENSE` file in the repository.
- **Contributing:** See :file:`CONTRIBUTING.md` for the code of conduct and submission process.

Citation
--------

If you use this library in your research, please cite:

.. code-block:: bibtex

   @misc{soutullo2025alicelri,
        title={ALICE-LRI: A General Method for Lossless Range Image Generation for Spinning LiDAR Sensors without Calibration Metadata},
        author={Samuel Soutullo and Miguel Yermo and David L. Vilariño and Óscar G. Lorenzo and José C. Cabaleiro and Francisco F. Rivera},
        year={2025},
        eprint={2510.20708},
        archivePrefix={arXiv},
        primaryClass={cs.CV},
        url={https://arxiv.org/abs/2510.20708},
  }

**Link to paper**: https://arxiv.org/abs/2510.20708

.. toctree::
   :hidden:
   :maxdepth: 2

   installation
   getting_started
   python_api
   cpp_api
