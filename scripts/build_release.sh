#!/bin/bash

pushd "$(dirname "${BASH_SOURCE[0]}")" > /dev/null || exit

cd ..
conan install lib -s compiler.cppstd=20 -s build_type=Release -of build/lib --build=missing
conan install examples -s compiler.cppstd=20 -s build_type=Release -of build/examples --build=missing
conan install tests -s compiler.cppstd=20 -s build_type=Release -of build/tests --build=missing

cd build || exit
cmake -DCMAKE_BUILD_TYPE=Release -G Ninja ..
cmake --build . --config Release

popd > /dev/null || exit