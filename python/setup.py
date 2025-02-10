from setuptools import setup, Extension
from pybind11.setup_helpers import Pybind11Extension

ext_modules = [
    Pybind11Extension(
        "_accurate_ri",
        ["accurate_ri/bindings.cpp"],
        libraries=["accurate_ri"],  # Link against installed C++ library
    ),
]

setup(
    name="accurate_ri",
    version="0.1",
    packages=["accurate_ri"],
    package_dir={"accurate_ri": "src"},
    ext_modules=ext_modules,
    zip_safe=False,
)
