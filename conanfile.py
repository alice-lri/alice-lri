from conan import ConanFile
from conan.tools.cmake import CMakeToolchain, CMake, cmake_layout, CMakeDeps
from conan.tools.files import copy


class AliceLriConan(ConanFile):
    name = "alice_lri"
    version = "0.1.0"
    
    # Metadata
    license = "MIT"
    author = "Samuel Soutullo <s.soutullo@usc.es>"
    url = "https://github.com/samuelss1996/accurate_ri"
    description = "High-performance LiDAR Range Image processing and intrinsic parameter estimation library"
    topics = ("lidar", "range-image", "point-cloud", "computer-vision")
    
    # Configuration
    settings = "os", "compiler", "build_type", "arch"
    options = {
        "shared": [True, False],
        "fPIC": [True, False],
        "enable_python_debug": [True, False],
        "lib_mode": [True, False]
    }
    default_options = {
        "shared": False,
        "fPIC": True,
        "enable_python_debug": False,
        "lib_mode": True
    }
    
    # Sources
    exports_sources = "CMakeLists.txt", "lib/*", "cmake/*"
    
    def config_options(self):
        if self.settings.os == "Windows":
            del self.options.fPIC
            
    def configure(self):
        if self.options.shared:
            self.options.rm_safe("fPIC")
            
    def requirements(self):
        self.requires("eigen/3.4.0")
        self.requires("nlohmann_json/3.11.3")
        
    def layout(self):
        cmake_layout(self)
        
    def generate(self):
        deps = CMakeDeps(self)
        deps.generate()
        tc = CMakeToolchain(self)
        tc.variables["LIB_MODE"] = self.options.lib_mode
        tc.variables["ENABLE_PYTHON_DEBUG"] = self.options.enable_python_debug
        tc.generate()
        
    def build(self):
        cmake = CMake(self)
        cmake.configure(build_script_folder="lib")
        cmake.build()
        
    def package(self):
        cmake = CMake(self)
        cmake.install()
        
    def package_info(self):
        self.cpp_info.libs = ["alice_lri"]
        self.cpp_info.includedirs = ["include"]
        
        if self.settings.os in ["Linux", "FreeBSD"]:
            self.cpp_info.system_libs.append("m")
            
        # CMake integration
        self.cpp_info.set_property("cmake_file_name", "alice_lri")
        self.cpp_info.set_property("cmake_target_name", "alice_lri::alice_lri")
