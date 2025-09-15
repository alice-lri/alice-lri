#!/usr/bin/env python3
"""
Setup script to install Conan dependencies before building.
This is called automatically by scikit-build-core.
"""
import subprocess
import sys
import os
from pathlib import Path

def main():
    # Get the build directory from environment or use default
    build_dir = os.environ.get('SKBUILD_BUILD_DIR', 'build')
    lib_dir = Path(build_dir) / 'lib'
    
    # Check if conan toolchain already exists
    toolchain_file = lib_dir / 'conan_toolchain.cmake'
    if toolchain_file.exists():
        print("Conan dependencies already installed, skipping...")
        return
    
    print("Installing Conan dependencies...")
    lib_dir.mkdir(parents=True, exist_ok=True)
    
    # Run conan install
    cmd = [
        'conan', 'install', '../lib',
        '-s', 'compiler.cppstd=gnu20',
        '-s', 'build_type=Release', 
        '-of', str(lib_dir),
        '--build=missing'
    ]
    
    try:
        result = subprocess.run(cmd, check=True, cwd=Path(__file__).parent)
        print("✅ Conan dependencies installed successfully")
    except subprocess.CalledProcessError as e:
        print(f"❌ Conan install failed: {e}")
        sys.exit(1)
    except FileNotFoundError:
        print("❌ Conan not found. Please install Conan first: pip install conan")
        sys.exit(1)

if __name__ == '__main__':
    main()
