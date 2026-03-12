#!/bin/bash
# Build script for Linux/macOS

CXX=g++
CXXFLAGS="-std=c++17 -Wall -Wextra -O2"
LDFLAGS="-lsfml-graphics -lsfml-window -lsfml-system"

echo "Building Serial version..."
$CXX $CXXFLAGS -o TrafficSimulator_serial main.cpp Vehicle.cpp TrafficLight.cpp Simulation.cpp Renderer.cpp $LDFLAGS
if [ $? -ne 0 ]; then echo "Build failed!"; exit 1; fi

echo "Building OpenMP version..."
$CXX $CXXFLAGS -DUSE_OPENMP -fopenmp -o TrafficSimulator_openmp main.cpp Vehicle.cpp TrafficLight.cpp Simulation.cpp Renderer.cpp $LDFLAGS -fopenmp
if [ $? -ne 0 ]; then echo "Build failed!"; exit 1; fi

echo ""
echo "Build successful!"
echo "  TrafficSimulator_serial  - Serial version"
echo "  TrafficSimulator_openmp  - OpenMP parallel version"
