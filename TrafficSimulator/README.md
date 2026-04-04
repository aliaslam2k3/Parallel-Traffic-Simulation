# Parallel Traffic Simulation Engine

A C++ traffic simulation that models thousands of vehicles moving on a 2D city grid with traffic lights. Designed for future extension with OpenMP, MPI, and GPU acceleration. Currently includes both **serial** and **OpenMP parallel** implementations.

## Features

- **2D City Grid**: 100×100 cell grid with roads and intersections
- **Vehicles**: 750 vehicles (configurable 500–1000) with id, position, speed, direction
- **Traffic Lights**: At all intersections, alternating red/green with configurable cycle times
- **Collision Detection**: Basic collision detection with push-apart resolution
- **SFML Visualization**: Real-time rendering of grid, vehicles (blue rectangles), and traffic lights (red/green circles)
- **Performance Timing**: Chrono-based measurement of simulation step execution time

## Project Structure

```
TrafficSimulator/
├── main.cpp           # Entry point, simulation loop
├── mpi_main.cpp       # MPI entry: spatial decomposition + rank-0 rendering
├── Vehicle.h/cpp      # Vehicle class
├── TrafficLight.h/cpp # Traffic light class
├── Simulation.h/cpp   # Simulation engine (grid, updates, collisions)
├── DomainDecomposition.h/cpp  # y-strip ownership for MPI
├── Renderer.h/cpp     # SFML visualization
├── Makefile           # Build with make
├── build.sh           # Linux/macOS build script
├── build.bat          # Windows build script
└── README.md
```

## Prerequisites

- **C++17** compatible compiler (g++, clang++, or MSVC)
- **SFML 2.x** library
- **OpenMP** (optional, for parallel build)

### Installing SFML

**Ubuntu/Debian:**
```bash
sudo apt-get install libsfml-dev
```

**Fedora:**
```bash
sudo dnf install SFML-devel
```

**macOS (Homebrew):**
```bash
brew install sfml
```

**Windows (MinGW/vcpkg):**
```bash
vcpkg install sfml
# Or download from https://www.sfml-dev.org/download.php
```

## Building

### Using Make (Linux/macOS)

```bash
cd TrafficSimulator

# Serial version only
make serial

# OpenMP parallel version only
make openmp

# Both versions
make both

# MPI spatial decomposition (requires mpic++ / Open MPI or MPICH)
make mpi

# Clean
make clean
```

### Using build script

**Linux/macOS:**
```bash
chmod +x build.sh
./build.sh
```

**Windows (PowerShell/CMD):**
```cmd
build.bat
```

### Manual g++ commands

**Serial version:**
```bash
g++ -std=c++17 -Wall -O2 -o TrafficSimulator_serial \
    main.cpp Vehicle.cpp TrafficLight.cpp Simulation.cpp Renderer.cpp DomainDecomposition.cpp \
    -lsfml-graphics -lsfml-window -lsfml-system
```

**OpenMP version:**
```bash
g++ -std=c++17 -Wall -O2 -DUSE_OPENMP -fopenmp -o TrafficSimulator_openmp \
    main.cpp Vehicle.cpp TrafficLight.cpp Simulation.cpp Renderer.cpp DomainDecomposition.cpp \
    -lsfml-graphics -lsfml-window -lsfml-system -fopenmp
```

**MPI version** (Linux/macOS; requires `mpic++` and MPI development packages):
```bash
cd TrafficSimulator
make mpi
mpirun -np 4 ./TrafficSimulator_mpi
```
On Windows, install [MS-MPI](https://www.microsoft.com/en-us/download/details.aspx?id=105289) and a MinGW-compatible `mpic++` wrapper, or build under WSL using the same `make mpi` command.

### Windows with custom SFML path

If SFML is installed in a custom location (e.g., `C:\SFML`):

```cmd
g++ -std=c++17 -O2 -IC:\SFML\include -LC:\SFML\lib -o TrafficSimulator_serial ^
    main.cpp Vehicle.cpp TrafficLight.cpp Simulation.cpp Renderer.cpp DomainDecomposition.cpp ^
    -lsfml-graphics -lsfml-window -lsfml-system
```

## Running

```bash
# Serial
./TrafficSimulator_serial

# OpenMP (uses all available cores)
./TrafficSimulator_openmp

# MPI: one process must be rank 0 (opens the window); use 2+ ranks for migration
mpirun -np 4 ./TrafficSimulator_mpi
```

Close the window to exit. Statistics (FPS, step time) are printed to the console every 2 seconds.

## OpenMP vs MPI (for presentations)

**OpenMP (implemented in this repo)**

- **Where:** Only [`Simulation.cpp`](Simulation.cpp) inside `Simulation::updateVehicles`, guarded by `#ifdef USE_OPENMP`.
- **What runs in parallel:** A single `#pragma omp parallel for schedule(dynamic, 32)` over **vehicle index** `0 .. N-1`. Each iteration updates **one** vehicle for the current frame.
- **What stays serial:** `updateTrafficLights`, `detectCollisions`, and the main loop in `main.cpp` — no OpenMP there.
- **Memory model:** One OS process; all threads share `m_vehicles` and `m_trafficLights` (shared memory).
- **Role:** Speed up the **per-vehicle physics/update** step on **multiple CPU cores of one machine** when `N` is large.

**MPI (optional `TrafficSimulator_mpi` build)**

- **Where:** [`mpi_main.cpp`](mpi_main.cpp) + [`DomainDecomposition.cpp`](DomainDecomposition.cpp).
- **What it does:** **Spatial decomposition** along **y** (horizontal strips). Each MPI **rank** owns vehicles whose `y` lies in its strip `[yMin, yMax)`. When a vehicle’s position leaves that strip after an update, it is **serialized** and **sent** to the rank that owns the new region (`MPI_Send` / `MPI_Recv`).
- **Traffic lights:** Rank **0** advances phases; state and timers are **`MPI_Bcast`** so all ranks see identical signals for red-light logic.
- **Rendering:** Only **rank 0** opens the SFML window; vehicle positions are **gathered** for drawing. With **2+ ranks**, **cyan horizontal lines** show **MPI region boundaries** (equal **y** strips: rank `r` owns grid `y` in `[r·H/N, (r+1)·H/N)` in grid units, same map as serial).
- **Role:** Distribute the simulation across **processes** (e.g. cluster nodes) so work is partitioned by **space** (region ownership + boundary migration), not only by parallelizing a loop on one node.

**Why both:** OpenMP = **threads** within one rank (shared memory). MPI = **processes** across ranks (distributed memory) with **explicit messages** when vehicles cross region boundaries.

## Simulation Loop

```
while (running) {
    updateVehicles();      // Move vehicles (parallelized with OpenMP)
    updateTrafficLights(); // Cycle traffic lights
    detectCollisions();    // Resolve overlaps
    renderSimulation();   // Draw to screen
}
```

## Configuration

Edit constants in `Simulation.h`:
- `GRID_WIDTH`, `GRID_HEIGHT`: Grid dimensions (default 100×100)
- `INTERSECTION_SPACING`: Distance between traffic lights (default 10)
- `VEHICLE_COUNT` in `main.cpp`: Number of vehicles (default 750)

## Future Extensions

- **GPU (CUDA/OpenCL)**: Massively parallel vehicle updates
- **Advanced traffic**: Lanes, turning, pathfinding

## License

Educational project for parallel computing coursework.
