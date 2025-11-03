Python API Reference
====================

The Python package exposes the core ALICE-LRI functionality.

Main Functions
--------------

Common entry points for typical workflows.

.. autofunction:: alice_lri.estimate_intrinsics

.. autofunction:: alice_lri.project_to_range_image

.. autofunction:: alice_lri.unproject_to_point_cloud

Main Data Structures
--------------------

Types commonly used when interacting with ALICE-LRI.

.. autoclass:: alice_lri.Intrinsics
   :members:
   :undoc-members:

.. autoclass:: alice_lri.RangeImage
   :members:
   :undoc-members:
   :special-members: __getitem__, __setitem__, __array__

   **Indexing and Array Access**

   This class supports indexing syntax for getting and setting pixel values:

   - ``value = range_image[row, col]`` — Get pixel value at position (row, col)
   - ``range_image[row, col] = value`` — Set pixel value at position (row, col)
   - ``array = np.asarray(range_image)`` — Convert to NumPy array (zero-copy view)

Additional Resources
--------------------

- :doc:`python_api_advanced` - Advanced functions, detailed intrinsics, and utilities

.. toctree::
   :hidden:

   python_api_advanced
