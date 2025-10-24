Getting Started
===============

This guide helps you get up and running with ALICE-LRI quickly. You'll learn how to install the library and run your first examples in Python and C++.

Installation
------------

For detailed installation instructions (PyPI, from source, linking, logging), see the :doc:`installation` guide.

Quick Python Example
--------------------

.. code-block:: python

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

Quick C++ Example
-----------------

.. code-block:: cpp

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

Working with JSON
-----------------

Both Python and C++ APIs support serialization and deserialization of intrinsic parameters to/from JSON format. This is useful for saving estimated intrinsics for later use or sharing them.

**Python**: Use ``intrinsics_to_json_file()`` and ``intrinsics_from_json_file()`` (or the string variants). See the :doc:`python_api_advanced` for full details.

**C++**: Use ``intrinsicsToJsonFile()`` and ``intrinsicsFromJsonFile()`` (or the string variants). See the :doc:`cpp_api_advanced` for full details.

Next Steps
----------

- :doc:`python_api` - Complete Python API reference.
- :doc:`cpp_api` - Complete C++ API reference.
 
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