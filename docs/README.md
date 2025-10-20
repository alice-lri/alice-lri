# ALICE-LRI Documentation

This folder contains the documentation sources for the ALICE-LRI project. Documentation is built using Sphinx (for Python and site), Doxygen (for C++), and Breathe/Exhale (for C++/Python integration).

## Documentation Generation Steps

1. **Install dependencies:**
   ```bash
   pip install -r requirements.txt
   sudo apt-get install doxygen
   ```

2. **Install Python package and generate stubs:**
   ```bash
   ../python/install_dev_and_stubs.sh
   ```

3. **Build Doxygen XML:**
   ```bash
   doxygen Doxyfile
   ```
4. **Build Sphinx HTML:**
   ```bash
   sphinx-build -b html . _build/html
   ```
5. **Preview:**
   ```bash
   python3 -m http.server -d _build/html
   ```

## Structure
- `Doxyfile`: Doxygen configuration for C++ API docs
- `conf.py`: Sphinx configuration
- `index.rst`: Main documentation index
- `installation.rst`: Installation guide
- `getting_started.rst`: Getting started guide
- `python_api.rst`: Python API reference
- `python_api_advanced.rst`: Advanced Python API reference
- `cpp_api.rst`: C++ API reference
- `cpp_api_advanced.rst`: Advanced C++ API reference
- `requirements.txt`: Python doc dependencies

## Online Documentation

The documentation is automatically built and deployed to GitHub Pages when a release is published:

**[https://alice-lri.github.io/alice-lri/](https://alice-lri.github.io/alice-lri/)**
