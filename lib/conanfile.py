import re
import os
from conan import ConanFile
from conan.tools.cmake import CMake, cmake_layout, CMakeToolchain, CMakeDeps

class AliceLriConan(ConanFile):
    name = "alice_lri"
    version = "0.1.0"
    license = "MIT"
    author = "Samuel Soutullo <s.soutullo@usc.es>"
    url = "https://github.com/alice-lri/alice-lri"
    description = "LiDAR Range Image processing and intrinsic parameter estimation library"
    topics = ("lidar", "range-image", "point-cloud", "computer-vision", "robotics")
    settings = "os", "compiler", "build_type", "arch"
    generators = "CMakeDeps", "CMakeToolchain"
    exports_sources = "CMakeLists.txt", "src/*", "include/*", "cmake/*", "sources.cmake"
    requires = [
        "nlohmann_json/3.11.3",
        "eigen/3.4.0",
        "boost/1.88.0"
    ]
    options = {
        "shared": [True, False]
    }
    default_options = {
        "shared": True,
        "boost/*:header_only": True
    }

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def package(self):
        cmake = CMake(self)
        cmake.install()

    def package_info(self):
        self.cpp_info.set_property("cmake_file_name", "alice_lri")
        self.cpp_info.set_property("cmake_target_name", "alice_lri::alice_lri")
        self.cpp_info.libs = ["alice_lri"]
