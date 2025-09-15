#!/usr/bin/env python3
"""
Build hook to generate stub files during wheel building.
This is called automatically by scikit-build-core.
"""
import subprocess
import sys
import os
import tempfile
import shutil
from pathlib import Path

def generate_stubs():
    """Generate .pyi stub files for the C++ extension"""
    print("🔨 Generating Python stub files...")
    
    # The extension should be built by now, find it in the wheel staging area
    wheel_dir = Path(os.environ.get("SKBUILD_WHEEL_DIR", "."))
    source_dir = Path(".")
    
    # Look for the built extension in common locations
    extension_candidates = [
        wheel_dir / "accurate_ri" / "_accurate_ri.so",
        Path("build") / "lib" / "accurate_ri" / "_accurate_ri.so",
    ]
    
    # Also search for any _accurate_ri*.so files
    for pattern in ["build/**/accurate_ri/_accurate_ri*.so", "build/**/_accurate_ri*.so"]:
        extension_candidates.extend(Path(".").glob(pattern))
    
    extension_path = None
    for candidate in extension_candidates:
        if candidate.exists():
            extension_path = candidate.parent
            print(f"📦 Found extension at: {candidate}")
            break
    
    if not extension_path:
        print("⚠️  Could not find built extension, searching more broadly...")
        # Last resort: find any .so file that looks like our extension
        for so_file in Path(".").rglob("*accurate_ri*.so"):
            extension_path = so_file.parent
            print(f"📦 Found extension at: {so_file}")
            break
    
    if not extension_path:
        print("❌ Could not find built extension, skipping stub generation")
        return
    
    # Create a temporary directory to isolate the stub generation
    with tempfile.TemporaryDirectory() as temp_dir:
        temp_path = Path(temp_dir)
        
        # Set up environment with the extension in Python path
        env = os.environ.copy()
        current_pythonpath = env.get("PYTHONPATH", "")
        env["PYTHONPATH"] = f"{extension_path}:{current_pythonpath}"
        
        try:
            # Generate stubs to temporary location first
            result = subprocess.run([
                sys.executable, "-m", "pybind11_stubgen",
                "accurate_ri._accurate_ri",
                "-o", str(temp_path),
                "--skip-signature-downgrade",
                "--print-safe-value-reprs"
            ], env=env, capture_output=True, text=True, timeout=60)
            
            if result.returncode == 0:
                # Move generated stub to the right location
                stub_src = temp_path / "accurate_ri" / "_accurate_ri.pyi"
                stub_dst = source_dir / "accurate_ri" / "_accurate_ri.pyi"
                
                if stub_src.exists():
                    shutil.copy2(stub_src, stub_dst)
                    print(f"✅ Stub files generated successfully: {stub_dst}")
                else:
                    print(f"⚠️  Stub generation completed but file not found at {stub_src}")
                    print("Generated files:", list(temp_path.rglob("*.pyi")))
            else:
                print(f"❌ Stub generation failed with return code {result.returncode}")
                print("STDOUT:", result.stdout)
                print("STDERR:", result.stderr)
                
        except subprocess.TimeoutExpired:
            print("⚠️  Stub generation timed out")
        except subprocess.CalledProcessError as e:
            print(f"⚠️  Stub generation failed: {e}")
        except Exception as e:
            print(f"⚠️  Unexpected error during stub generation: {e}")
        
        print("Continuing build (stubs are optional)...")

if __name__ == "__main__":
    generate_stubs()
