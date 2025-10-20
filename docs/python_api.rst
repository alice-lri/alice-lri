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

Additional Resources
--------------------

- :doc:`python_api_advanced` - Advanced functions, detailed intrinsics, and utilities

.. toctree::
   :hidden:

   python_api_advanced
