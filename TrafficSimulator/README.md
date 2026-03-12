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
├── Vehicle.h/cpp      # Vehicle class
├── TrafficLight.h/cpp # Traffic light class
├── Simulation.h/cpp   # Simulation engine (grid, updates, collisions)
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
    main.cpp Vehicle.cpp TrafficLight.cpp Simulation.cpp Renderer.cpp \
    -lsfml-graphics -lsfml-window -lsfml-system
```

**OpenMP version:**
```bash
g++ -std=c++17 -Wall -O2 -DUSE_OPENMP -fopenmp -o TrafficSimulator_openmp \
    main.cpp Vehicle.cpp TrafficLight.cpp Simulation.cpp Renderer.cpp \
    -lsfml-graphics -lsfml-window -lsfml-system -fopenmp
```

### Windows with custom SFML path

If SFML is installed in a custom location (e.g., `C:\SFML`):

```cmd
g++ -std=c++17 -O2 -IC:\SFML\include -LC:\SFML\lib -o TrafficSimulator_serial ^
    main.cpp Vehicle.cpp TrafficLight.cpp Simulation.cpp Renderer.cpp ^
    -lsfml-graphics -lsfml-window -lsfml-system
```

## Running

```bash
# Serial
./TrafficSimulator_serial

# OpenMP (uses all available cores)
./TrafficSimulator_openmp
```

Close the window to exit. Statistics (FPS, step time) are printed to the console every 2 seconds.

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

- **MPI**: Distributed simulation across multiple nodes
- **GPU (CUDA/OpenCL)**: Massively parallel vehicle updates
- **Advanced traffic**: Lanes, turning, pathfinding

## License

Educational project for parallel computing coursework.
