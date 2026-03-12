/**
 * main.cpp
 * Step 1: City road map with roads, buildings, parks, traffic lights.
 * No vehicles.
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

constexpr int VEHICLE_COUNT = 0;   // Step 1: no vehicles
constexpr float SIM_DT     = 0.1f;

int main() {
    std::cout << "========================================\n";
    std::cout << "  City Road Map  |  Step 1\n";
    std::cout << "  (Map + Traffic Lights, no vehicles)\n";
    std::cout << "========================================\n";
    std::cout << "Grid    : " << GRID_WIDTH << " x " << GRID_HEIGHT << "\n";
    std::cout << "Roads   : " << (GRID_WIDTH / INTERSECTION_SPACING)
              << " x " << (GRID_HEIGHT / INTERSECTION_SPACING)
              << " intersections (highways, arterials, local)\n";
    std::cout << "Features: U-turn bays, medians, varied blocks\n";
    std::cout << "----------------------------------------\n";

    Simulation simulation;
    simulation.initialize(VEHICLE_COUNT);

    Renderer renderer(1200, 1200);
    if (!renderer.initialize(false)) {  // false = windowed
        std::cerr << "ERROR: Could not open render window.\n";
        return 1;
    }

    bool running   = true;
    int  frames    = 0;
    auto tStart    = std::chrono::high_resolution_clock::now();
    auto tLastStat = tStart;

    std::cout << "Close window to exit.\n";

    while (running && renderer.isOpen()) {
        while (auto ev = renderer.pollEvent()) {
            if (ev->is<sf::Event::Closed>()) running = false;
        }

        simulation.updateVehicles(SIM_DT);
        simulation.updateTrafficLights(SIM_DT);
        simulation.detectCollisions();

        renderer.clear();
        renderer.render(simulation);
        renderer.display();
        ++frames;

        auto now  = std::chrono::high_resolution_clock::now();
        long msEl = std::chrono::duration_cast<std::chrono::milliseconds>(now - tLastStat).count();
        if (msEl >= 2000) {
            long totalMs = std::chrono::duration_cast<std::chrono::milliseconds>(now - tStart).count();
            double fps = (totalMs > 0) ? frames * 1000.0 / totalMs : 0.0;
            std::cout << "Frames: " << std::setw(6) << frames
                      << "  |  FPS: " << std::fixed << std::setprecision(1) << fps << "\n";
            tLastStat = now;
        }
    }

    std::cout << "Done.  Frames: " << frames << "\n";
    return 0;
}
