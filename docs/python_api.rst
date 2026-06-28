Python API Reference
====================

The Python package exposes the core ALICE-LRI functionality.

Main Functions
--------------

Common entry points for typical workflows.

.. autofunction:: alice_lri.estimate_intrinsics

.. autofunction:: alice_lri.project_to_range_image

.. autofunction:: alice_lri.project_values_to_range_image

.. autofunction:: alice_lri.unproject_to_point_cloud

Custom Scalar Images
--------------------

Use ``project_values_to_range_image`` when you need the ALICE-LRI pixel layout but want each occupied pixel to store a per-point scalar other than range, such as intensity, labels, confidence scores, or residuals. The point coordinates still define the row and column, while ``values`` defines the stored pixel value. The optional ``empty_value`` argument initializes pixels with no corresponding point.

For point-cloud reconstruction, keep using ``project_to_range_image`` so pixels store geometric ranges.

Range Image Arrays
------------------

Python range images are represented as 2D ``numpy.ndarray`` objects with ``float64`` values and shape ``(height, width)``. ``project_to_range_image`` and ``project_values_to_range_image`` return arrays directly, and ``unproject_to_point_cloud`` accepts a 2D array as input.

Arrays returned by ALICE-LRI are writable and support standard NumPy indexing:

.. code-block:: python

   value = range_image[0, 0]
   range_image[0, 0] = value

Main Data Structures
--------------------

Types commonly used when interacting with ALICE-LRI.

.. autoclass:: alice_lri.Intrinsics
   :members:
   :undoc-members:

Additional Resources
--------------------

- :doc:`python_api_advanced` - Advanced functions, detailed intrinsics, and utilities

.. toctree::
   :hidden:

   python_api_advanced
