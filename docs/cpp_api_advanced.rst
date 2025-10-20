Advanced C++ API
=================

Advanced functions, detailed intrinsics, error handling, and utility classes for C++ developers.

Functions
---------

Detailed Intrinsics Estimation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. doxygenfunction:: alice_lri::estimateIntrinsicsDetailed(const PointCloud::Float &points)
   :project: ALICE-LRI

.. doxygenfunction:: alice_lri::estimateIntrinsicsDetailed(const PointCloud::Double &points)
   :project: ALICE-LRI

JSON Serialization
^^^^^^^^^^^^^^^^^^

.. doxygenfunction:: alice_lri::intrinsicsFromJsonStr
   :project: ALICE-LRI

.. doxygenfunction:: alice_lri::intrinsicsToJsonStr
   :project: ALICE-LRI

.. doxygenfunction:: alice_lri::intrinsicsFromJsonFile
   :project: ALICE-LRI

.. doxygenfunction:: alice_lri::intrinsicsToJsonFile
   :project: ALICE-LRI

Data Structures
---------------

Detailed Intrinsics
^^^^^^^^^^^^^^^^^^^

.. doxygenstruct:: alice_lri::IntrinsicsDetailed
   :project: ALICE-LRI
   :members:

.. doxygenstruct:: alice_lri::ScanlineDetailed
   :project: ALICE-LRI
   :members:

.. doxygenstruct:: alice_lri::ScanlineAngleBounds
   :project: ALICE-LRI
   :members:

Mathematical Utilities
^^^^^^^^^^^^^^^^^^^^^^

.. doxygenstruct:: alice_lri::Interval
   :project: ALICE-LRI
   :members:

.. doxygenstruct:: alice_lri::ValueConfInterval
   :project: ALICE-LRI
   :members:

Error Handling
--------------

Status and Error Codes
^^^^^^^^^^^^^^^^^^^^^^

.. doxygenstruct:: alice_lri::Status
   :project: ALICE-LRI
   :members:

.. doxygenenum:: alice_lri::ErrorCode
   :project: ALICE-LRI

.. doxygenfunction:: alice_lri::errorMessage
   :project: ALICE-LRI

Enums
-----

.. doxygenenum:: alice_lri::EndReason
   :project: ALICE-LRI
