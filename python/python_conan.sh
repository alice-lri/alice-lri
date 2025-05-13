#!/bin/bash

conan install ../lib -s compiler.cppstd=gnu20 -s build_type=Release -of build/lib --build=missing
