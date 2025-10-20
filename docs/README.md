# ALICE-LRI Documentation

This folder contains the documentation sources for the ALICE-LRI project. Documentation is built using Sphinx (for Python and site), Doxygen (for C++), and Breathe/Exhale (for C++/Python integration).

## Local Build Instructions

1. **Install Python package and generate stubs:**
   ```bash
   ../python/install_dev_and_stubs.sh
   ```
2. **Install dependencies:**
   ```bash
   pip install -r requirements.txt
   sudo apt-get install doxygen
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
- `python_api.rst`: Python API reference
- `cpp_api.rst`: C++ API reference
- `requirements.txt`: Python doc dependencies

## CI/CD
Documentation is automatically built and deployed to GitHub Pages on every push to `main` via GitHub Actions (`.github/workflows/docs.yml`).

TODO: More beautiful doc page: split main page (quick start from README), C++ API, Python API, etc.
TODO: Fix ALICE_LRI_API thing in docs
TODO: Make doc CI trigger on release instead of every push to master
TODO: Make GitHub pages work
TODO: Add documentation badge and/or link
