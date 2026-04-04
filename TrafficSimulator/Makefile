# Makefile for Parallel Traffic Simulation Engine
# Supports both serial and OpenMP builds

CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2
LDFLAGS = -lsfml-graphics -lsfml-window -lsfml-system

# Detect OS for SFML linking
UNAME_S := $(shell uname -s 2>/dev/null || echo Windows)
ifeq ($(UNAME_S),Windows_NT)
    # Windows: adjust paths if SFML is in custom location
    LDFLAGS = -lsfml-graphics -lsfml-window -lsfml-system
    ifeq ($(MSYSTEM),MINGW64)
        LDFLAGS += -I/mingw64/include -L/mingw64/lib
    endif
endif

# Default: build serial version
all: serial

# Serial build (no OpenMP)
serial: TrafficSimulator_serial

# OpenMP parallel build
openmp: CXXFLAGS += -DUSE_OPENMP -fopenmp
openmp: LDFLAGS += -fopenmp
openmp: TrafficSimulator_openmp

# Source files (DomainDecomposition used by Simulation for MPI spawn helpers)
SOURCES = main.cpp Vehicle.cpp TrafficLight.cpp Simulation.cpp Renderer.cpp DomainDecomposition.cpp

SOURCES_MPI = mpi_main.cpp Vehicle.cpp TrafficLight.cpp Simulation.cpp Renderer.cpp DomainDecomposition.cpp

# Serial executable
TrafficSimulator_serial: $(SOURCES)
	$(CXX) $(CXXFLAGS) -o $@ $(SOURCES) $(LDFLAGS)

# OpenMP executable
TrafficSimulator_openmp: $(SOURCES)
	$(CXX) $(CXXFLAGS) -o $@ $(SOURCES) $(LDFLAGS)

# MPI + spatial decomposition (requires mpic++ / Open MPI or MPICH in PATH)
mpi: TrafficSimulator_mpi

TrafficSimulator_mpi: $(SOURCES_MPI)
	mpic++ $(CXXFLAGS) -DUSE_MPI -o $@ $(SOURCES_MPI) $(LDFLAGS)

# Build both
both: serial openmp
	@echo "Built both serial and OpenMP versions."

clean:
	rm -f TrafficSimulator_serial TrafficSimulator_openmp TrafficSimulator_mpi
	@echo "Cleaned build artifacts."

.PHONY: all serial openmp mpi both clean
