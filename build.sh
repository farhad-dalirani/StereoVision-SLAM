#!/bin/bash

# Create a build directory (if it doesn't exist)
mkdir -p build

# Navigate to the build directory
cd build

# Run CMake to generate build files
cmake -DCMAKE_BUILD_TYPE=Debug ..

# Build the project
make

# return main directory
cd ..