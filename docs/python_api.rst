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

Other
-----

Additional functions, data structures, and utilities for advanced use cases.

Functions
^^^^^^^^^

.. autofunction:: alice_lri.estimate_intrinsics_detailed

.. autofunction:: alice_lri.intrinsics_from_json_file

.. autofunction:: alice_lri.intrinsics_from_json_str

.. autofunction:: alice_lri.intrinsics_to_json_file

.. autofunction:: alice_lri.intrinsics_to_json_str

.. autofunction:: alice_lri.error_message

Data Structures
^^^^^^^^^^^^^^^

.. autoclass:: alice_lri.IntrinsicsDetailed
   :members:
   :undoc-members:

.. autoclass:: alice_lri.Scanline
   :members:
   :undoc-members:

.. autoclass:: alice_lri.ScanlineDetailed
   :members:
   :undoc-members:

.. autoclass:: alice_lri.ScanlineAngleBounds
   :members:
   :undoc-members:

.. autoclass:: alice_lri.Interval
   :members:
   :undoc-members:

.. autoclass:: alice_lri.ValueConfInterval
   :members:
   :undoc-members:

Enums and Constants
^^^^^^^^^^^^^^^^^^^

.. autoclass:: alice_lri.ErrorCode
   :members:
   :undoc-members:

.. autoclass:: alice_lri.EndReason
   :members:
   :undoc-members:
