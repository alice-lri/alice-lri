"""
Custom build backend that runs Conan before scikit-build-core.
"""
import subprocess
from pathlib import Path
from scikit_build_core.build import build_wheel as _build_wheel, build_editable as _build_editable
from scikit_build_core.build import build_sdist as _build_sdist

def setup_conan_deps():
    """Setup Conan dependencies before building."""
    print("🔧 Setting up Conan dependencies...")
    
    # Ensure we're in the right directory
    project_root = Path(__file__).parent
    build_dir = project_root / "build" / "lib"
    build_dir.mkdir(parents=True, exist_ok=True)
    
    # Check if already setup
    if (build_dir / "conan_toolchain.cmake").exists():
        print("✅ Conan dependencies already set up")
        return
    
    # Run conan install
    cmd = [
        "conan", "install", "../lib",
        "-s", "compiler.cppstd=gnu20",
        "-s", "build_type=Release",
        "-of", str(build_dir),
        "--build=missing"
    ]
    
    try:
        subprocess.run(cmd, check=True, cwd=project_root)
        print("✅ Conan dependencies installed successfully")
    except subprocess.CalledProcessError as e:
        raise RuntimeError(f"Conan setup failed: {e}")
    except FileNotFoundError:
        raise RuntimeError("Conan not found. Install with: pip install conan")

def build_wheel(wheel_directory, config_settings=None, metadata_directory=None):
    """Build wheel with Conan setup."""
    setup_conan_deps()
    return _build_wheel(wheel_directory, config_settings, metadata_directory)

def build_editable(wheel_directory, config_settings=None, metadata_directory=None):
    """Build editable with Conan setup."""
    setup_conan_deps()
    return _build_editable(wheel_directory, config_settings, metadata_directory)

def build_sdist(sdist_directory, config_settings=None):
    """Build source distribution."""
    return _build_sdist(sdist_directory, config_settings)
