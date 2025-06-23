#!/bin/bash

rm -Rf build
conan install ../lib -s compiler.cppstd=gnu20 -s build_type=Release -of build/lib --build=missing
cmake -DCMAKE_BUILD_TYPE=Release -DLOG_LEVEL=NONE -DENABLE_TRACE_FILE=OFF -DENABLE_PROFILING=OFF -DLIB_MODE=ON -B build -G Ninja
cd build || exit
ninja

# pip install -e ~/GitHub/AccurateRI/python
# on the corresponding project