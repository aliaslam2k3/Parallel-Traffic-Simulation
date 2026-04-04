/**
 * main.cpp
 * Traffic simulation with variable delta time, input for camera, enhanced renderer.
 */

#include "Simulation.h"
#include "Renderer.h"
#include <algorithm>
#include <chrono>
#include <iomanip>
#include <iostream>

using namespace TrafficSim;

constexpr int VEHICLE_COUNT = 120;

int main() {
    std::cout << "========================================\n";
    std::cout << "  Traffic simulation (enhanced visuals)\n";
    std::cout << "========================================\n";
    std::cout << "Grid: " << GRID_WIDTH << " x " << GRID_HEIGHT << "\n";
    std::cout << "Controls: WASD / arrows = pan, +/- / wheel = zoom, Home = reset view\n";
    std::cout << "----------------------------------------\n";

    Simulation simulation;
    simulation.initialize(VEHICLE_COUNT);

    Renderer renderer(1200, 720);
    if (!renderer.initialize(false)) {
        std::cerr << "ERROR: Could not open render window.\n";
        return 1;
    }

    bool running = true;
    int  frames  = 0;
    auto tStart = std::chrono::high_resolution_clock::now();
    auto tLastStat = tStart;
    auto tLastFrame = tStart;
    float fpsDisplay = 60.f;

    std::cout << "Close window to exit.\n";

    while (running && renderer.isOpen()) {
        while (auto ev = renderer.pollEvent()) {
            renderer.handleEvent(*ev);
            if (ev->is<sf::Event::Closed>()) running = false;
        }

        auto nowFrame = std::chrono::high_resolution_clock::now();
        float dt = std::chrono::duration<float>(nowFrame - tLastFrame).count();
        tLastFrame = nowFrame;
        dt = std::clamp(dt, 0.001f, 0.05f);

        if (dt > 0.0001f) {
            float inst = 1.f / dt;
            fpsDisplay = fpsDisplay * 0.88f + inst * 0.12f;
        }

        simulation.updateTrafficLights(dt);
        simulation.updateVehicles(dt);
        simulation.detectCollisions();

        renderer.clear();
        renderer.render(simulation, fpsDisplay, static_cast<int>(simulation.getVehicles().size()));
        renderer.display();
        ++frames;

        auto now = std::chrono::high_resolution_clock::now();
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
