C++ API Reference
=================

The C++ API provides the core functionality for lossless range image generation and reconstruction. All symbols are in the ``alice_lri`` namespace.

Main Functions
--------------

Common entry points for intrinsics estimation, projection, and reconstruction.

Intrinsics Estimation
^^^^^^^^^^^^^^^^^^^^^

.. doxygenfunction:: alice_lri::estimateIntrinsics(const PointCloud::Float &points)
   :project: ALICE-LRI

.. doxygenfunction:: alice_lri::estimateIntrinsics(const PointCloud::Double &points)
   :project: ALICE-LRI

Range Image Projection
^^^^^^^^^^^^^^^^^^^^^^^

.. doxygenfunction:: alice_lri::projectToRangeImage(const Intrinsics &intrinsics, const PointCloud::Float &points)
   :project: ALICE-LRI

.. doxygenfunction:: alice_lri::projectToRangeImage(const Intrinsics &intrinsics, const PointCloud::Double &points)
   :project: ALICE-LRI

Point Cloud Reconstruction
^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. doxygenfunction:: alice_lri::unProjectToPointCloud
   :project: ALICE-LRI

Main Data Structures
--------------------

Core types for working with intrinsics, range images, and point clouds.

.. doxygenstruct:: alice_lri::Intrinsics
   :project: ALICE-LRI
   :members:

.. doxygenstruct:: alice_lri::RangeImage
   :project: ALICE-LRI
   :members:

.. doxygenstruct:: alice_lri::Scanline
   :project: ALICE-LRI
   :members:

Point Cloud Types
^^^^^^^^^^^^^^^^^

.. doxygenstruct:: alice_lri::PointCloud::Float
   :project: ALICE-LRI
   :members:

.. doxygenstruct:: alice_lri::PointCloud::Double
   :project: ALICE-LRI
   :members:

Result Type
^^^^^^^^^^^

The main API functions return ``Result<T>`` to handle success and error cases.

.. doxygenclass:: alice_lri::Result
   :project: ALICE-LRI
   :members:

Utility Classes
---------------

AliceArray
^^^^^^^^^^

.. doxygenclass:: alice_lri::AliceArray
   :project: ALICE-LRI
   :members:

AliceString
^^^^^^^^^^^

.. doxygenclass:: alice_lri::AliceString
   :project: ALICE-LRI
   :members:

Additional Resources
--------------------

- :doc:`cpp_api_advanced` - Detailed intrinsics, error handling, JSON I/O, and advanced utilities

.. toctree::
   :hidden:

   cpp_api_advanced
