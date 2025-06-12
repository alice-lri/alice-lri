#!/bin/bash

rm -Rf build
conan install ../lib -s compiler.cppstd=gnu20 -s build_type=Release -of build/lib --build=missing
cmake -DLOG_LEVEL=NONE -DCMAKE_BUILD_TYPE=Release -DENABLE_TRACE_FILE=OFF -B build -G Ninja
cd build || exit
ninja

# pip install -e ~/GitHub/AccurateRI/python
# on the corresponding project