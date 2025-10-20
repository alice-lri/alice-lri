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

Next Steps
----------

- Explore the :doc:`python_api` and :doc:`cpp_api` for complete API references.

Project Info
------------

- **License:** MIT — see the :file:`LICENSE` file in the repository.
- **Contributing:** See :file:`CONTRIBUTING.md` for the code of conduct and submission process.
 
Citation
--------

If you use this library in your research, please cite:

.. code-block:: bibtex

   TODO

