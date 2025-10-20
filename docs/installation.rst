Installation
============

This guide covers all installation options for ALICE-LRI: from PyPI, from source (Python and C++), linking in your C++ projects, and configuring log levels.

Python
--------------------

.. code-block:: bash

   pip install alice-lri

C++
---

Pre-built C++ binaries are not available. To use the C++ library, install from source as shown below.

Installation from Source
-------------------------

Build Dependencies
^^^^^^^^^^^^^^^^^^

- C++20 compatible compiler with CMake >= 3.20
- Python >= 3.8 and pip (for Python bindings)
- Conan >= 2.0 (can be installed with ``pip install conan``)

  - First time using Conan? Run ``conan profile detect`` after installing to create a default profile.

Other dependencies are automatically managed by Conan and pip.

Python
^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   git clone https://github.com/alice-lri/alice-lri.git
   cd alice-lri
   pip install ./python

C++
^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   # Clone the repository
   git clone https://github.com/alice-lri/alice-lri.git
   cd alice-lri/lib

   # Install dependencies and build
   conan install . -s compiler.cppstd=20 -s build_type=Release -of build/ --build=missing
   cmake -DCMAKE_BUILD_TYPE=Release -B build

   # Install the library
   cd build
   sudo make install
   sudo ldconfig  # Update shared library cache on Linux

By default, this installs the library and headers to standard system locations (e.g., ``/usr/local/lib``, ``/usr/local/include`` on Linux).

Using ALICE-LRI in Your C++ Project
------------------------------------

After installing, you no longer need access to the source code. You can link against the installed library.

With CMake
^^^^^^^^^^

.. code-block:: cmake

   find_package(alice_lri REQUIRED)
   target_link_libraries(YOUR_TARGET alice_lri::alice_lri)

With g++ (or similar compilers)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If you compile manually, link with ``-lalice_lri``:

.. code-block:: bash

   g++ your_source.cpp -lalice_lri -o your_program

Controlling Log Levels
-----------------------

ALICE-LRI supports configurable log levels at build time. By default, the log level is ``WARN`` (only warnings and errors are printed).

Change the log level when building the C++ library with CMake:

.. code-block:: bash

   cmake -DCMAKE_BUILD_TYPE=Release -DLOG_LEVEL=DEBUG -B build

Valid options: ``DEBUG``, ``INFO``, ``WARN``, ``ERROR``, ``NONE``.

For Python, you can control the default log level by editing ``pyproject.toml`` in the ``python/`` directory before building or installing from source. The default is also ``WARN``.

**Recommended:** Keep the default ``WARN`` level unless you need more verbose output for debugging or development.
