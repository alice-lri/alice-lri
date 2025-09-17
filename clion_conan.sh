#!/bin/bash

conan install lib -s compiler.cppstd=20 -s build_type=Debug -of cmake-build-debug/lib --build=missing
conan install lib -s compiler.cppstd=20 -s build_type=Release -of cmake-build-release/lib --build=missing
conan install lib -s compiler.cppstd=20 -s build_type=Release -of cmake-build-release-debug/lib --build=missing
conan install lib -s compiler.cppstd=20 -s build_type=Release -of cmake-build-release-install/lib --build=missing


conan install examples -s compiler.cppstd=20 -s build_type=Debug -of cmake-build-debug/examples --build=missing
conan install examples -s compiler.cppstd=20 -s build_type=Release -of cmake-build-release/examples --build=missing
conan install examples -s compiler.cppstd=20 -s build_type=Release -of cmake-build-release-debug/examples --build=missing
conan install examples -s compiler.cppstd=20 -s build_type=Release -of cmake-build-release-install/examples --build=missing
