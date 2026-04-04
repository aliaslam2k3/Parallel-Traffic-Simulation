#!/bin/bash
# Build script for Linux/macOS

CXX=g++
CXXFLAGS="-std=c++17 -Wall -Wextra -O2"
LDFLAGS="-lsfml-graphics -lsfml-window -lsfml-system"

SRC="main.cpp Vehicle.cpp TrafficLight.cpp Simulation.cpp Renderer.cpp DomainDecomposition.cpp"
SRC_MPI="mpi_main.cpp Vehicle.cpp TrafficLight.cpp Simulation.cpp Renderer.cpp DomainDecomposition.cpp"

echo "Building Serial version..."
$CXX $CXXFLAGS -o TrafficSimulator_serial $SRC $LDFLAGS
if [ $? -ne 0 ]; then echo "Build failed!"; exit 1; fi

echo "Building OpenMP version..."
$CXX $CXXFLAGS -DUSE_OPENMP -fopenmp -o TrafficSimulator_openmp $SRC $LDFLAGS -fopenmp
if [ $? -ne 0 ]; then echo "Build failed!"; exit 1; fi

if command -v mpic++ >/dev/null 2>&1; then
  echo "Building MPI version..."
  mpic++ $CXXFLAGS -DUSE_MPI -o TrafficSimulator_mpi $SRC_MPI $LDFLAGS
  if [ $? -ne 0 ]; then echo "MPI build failed!"; exit 1; fi
else
  echo "Skipping MPI build: mpic++ not in PATH."
fi

echo ""
echo "Build successful!"
echo "  TrafficSimulator_serial  - Serial version"
echo "  TrafficSimulator_openmp  - OpenMP parallel version"
if [ -f TrafficSimulator_mpi ]; then echo "  TrafficSimulator_mpi   - MPI (mpirun -np N ./TrafficSimulator_mpi)"; fi
