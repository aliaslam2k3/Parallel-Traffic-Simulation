/**
 * main.cpp
 * Parallel Traffic Simulation Engine
 *
 * Serial build:
 *   g++ -std=c++17 -O2 -o sim main.cpp Vehicle.cpp TrafficLight.cpp \
 *       Simulation.cpp Renderer.cpp -lsfml-graphics -lsfml-window -lsfml-system
 *
 * OpenMP build:
 *   g++ -std=c++17 -O2 -DUSE_OPENMP -fopenmp -o sim main.cpp Vehicle.cpp \
 *       TrafficLight.cpp Simulation.cpp Renderer.cpp \
 *       -lsfml-graphics -lsfml-window -lsfml-system -fopenmp
 */

#include "Simulation.h"
#include "Renderer.h"
#include <iostream>
#include <iomanip>
#include <chrono>

#ifdef USE_OPENMP
#include <omp.h>
#endif

using namespace TrafficSim;

constexpr int   VEHICLE_COUNT = 500;   // 500 vehicles on 8x8 = 64 intersections
constexpr float SIM_DT        = 0.12f; // simulation time-step per frame

int main() {
    std::cout << "========================================\n";
    std::cout << "  Parallel Traffic Simulation Engine\n";
    std::cout << "========================================\n";
#ifdef USE_OPENMP
    std::cout << "Mode    : OpenMP PARALLEL\n";
    std::cout << "Threads : " << omp_get_max_threads() << "\n";
#else
    std::cout << "Mode    : SERIAL\n";
#endif
    std::cout << "Grid    : " << GRID_WIDTH << " x " << GRID_HEIGHT
              << "  (" << GRID_WIDTH / INTERSECTION_SPACING
              << " x " << GRID_HEIGHT / INTERSECTION_SPACING
              << " intersections)\n";
    std::cout << "Vehicles: " << VEHICLE_COUNT << "\n";
    std::cout << "----------------------------------------\n";

    Simulation simulation;
    simulation.initialize(VEHICLE_COUNT);

    // 960x960 window = 12 px/cell, road strips = 36 px wide
    Renderer renderer(960, 960);
    if (!renderer.initialize()) {
        std::cerr << "ERROR: Could not open render window.\n";
        return 1;
    }

    bool running   = true;
    int  frames    = 0;
    auto tStart    = std::chrono::high_resolution_clock::now();
    auto tLastStat = tStart;

    std::cout << "Simulation running  (close window to exit)\n";

    while (running && renderer.isOpen()) {
        // --- Event handling ---
        while (auto ev = renderer.pollEvent()) {
            if (ev->is<sf::Event::Closed>()) running = false;
        }

        // --- Simulation step (timed) ---
        auto tStep0 = std::chrono::high_resolution_clock::now();
        simulation.updateVehicles(SIM_DT);
        simulation.updateTrafficLights(SIM_DT);
        simulation.detectCollisions();
        auto stepUs = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::high_resolution_clock::now() - tStep0).count();

        // --- Render ---
        renderer.clear();
        renderer.render(simulation);
        renderer.display();
        ++frames;

        // --- Console stats every 2 s ---
        auto now  = std::chrono::high_resolution_clock::now();
        long msEl = std::chrono::duration_cast<std::chrono::milliseconds>(now - tLastStat).count();
        if (msEl >= 2000) {
            long totalMs = std::chrono::duration_cast<std::chrono::milliseconds>(now - tStart).count();
            double fps = frames * 1000.0 / totalMs;
            std::cout << "Frames: " << std::setw(6) << frames
                      << "  |  Avg FPS: " << std::fixed << std::setprecision(1) << fps
                      << "  |  Step: "   << stepUs << " us\n";
            tLastStat = now;
        }
    }

    // --- Final report ---
    auto totalMs = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::high_resolution_clock::now() - tStart).count();
    std::cout << "----------------------------------------\n";
    std::cout << "Done.  Frames: " << frames
              << "  Runtime: " << totalMs << " ms"
              << "  Avg FPS: " << std::fixed << std::setprecision(1)
              << (totalMs > 0 ? frames * 1000.0 / totalMs : 0.0) << "\n";
    return 0;
}
