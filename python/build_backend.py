"""
Custom build backend that runs Conan before scikit-build-core.
"""
import subprocess
import os
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

try:
    from skbuild_core import build_meta as _orig_build_meta
except ImportError:
    # Fallback for older versions
    import scikit_build_core.build_meta as _orig_build_meta

def build_wheel(wheel_directory, config_settings=None, metadata_directory=None):
    """Custom build_wheel that generates stubs after building"""
    
    # First, do the normal build
    print("🔨 Building wheel with scikit-build-core...")
    wheel_name = _orig_build_meta.build_wheel(wheel_directory, config_settings, metadata_directory)
    
    # Now generate stubs
    print("📝 Generating stub files...")
    _generate_stubs()
    
    # Rebuild wheel to include the newly generated stubs
    print("🔨 Rebuilding wheel with stubs...")
    wheel_name = _orig_build_meta.build_wheel(wheel_directory, config_settings, metadata_directory)
    
    return wheel_name

def build_editable(wheel_directory, config_settings=None, metadata_directory=None):
    """Custom build_editable that generates stubs after building"""
    
    # First, do the normal editable build
    print("🔨 Building editable install...")
    wheel_name = _orig_build_meta.build_editable(wheel_directory, config_settings, metadata_directory)
    
    # Generate stubs for development
    print("📝 Generating stub files for development...")
    _generate_stubs()
    
    return wheel_name

def _generate_stubs():
    """Generate .pyi stub files for the C++ extension"""
    
    # Find the built extension
    extension_path = None
    
    # Common locations where the extension might be
    search_paths = [
        Path("build") / "lib",
        Path("build"),
        Path("accurate_ri"),
        Path("."),
    ]
    
    for search_path in search_paths:
        for so_file in search_path.rglob("*accurate_ri*.so"):
            extension_path = so_file.parent
            print(f"📦 Found extension at: {so_file}")
            break
        if extension_path:
            break
    
    if not extension_path:
        print("⚠️  Could not find built extension, skipping stub generation")
        return
    
    # Set up environment
    env = os.environ.copy()
    current_pythonpath = env.get("PYTHONPATH", "")
    env["PYTHONPATH"] = f"{extension_path}:{current_pythonpath}"
    
    try:
        # Generate stubs
        result = subprocess.run([
            sys.executable, "-m", "pybind11_stubgen",
            "accurate_ri._accurate_ri",
            "-o", ".",
            "--skip-signature-downgrade"
        ], env=env, capture_output=True, text=True, timeout=60, cwd=".")
        
        if result.returncode == 0:
            stub_file = Path("accurate_ri") / "_accurate_ri.pyi"
            if stub_file.exists():
                print(f"✅ Stub files generated: {stub_file}")
            else:
                print("⚠️  Stub generation completed but file not found")
        else:
            print(f"⚠️  Stub generation failed: {result.stderr}")
            
    except Exception as e:
        print(f"⚠️  Error during stub generation: {e}")
        print("Continuing build without stubs...")

# Expose the same interface as scikit-build-core
get_requires_for_build_wheel = _orig_build_meta.get_requires_for_build_wheel
get_requires_for_build_editable = _orig_build_meta.get_requires_for_build_editable
prepare_metadata_for_build_wheel = _orig_build_meta.prepare_metadata_for_build_wheel
prepare_metadata_for_build_editable = _orig_build_meta.prepare_metadata_for_build_editable
build_sdist = _orig_build_meta.build_sdist
