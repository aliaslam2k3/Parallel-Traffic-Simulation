/**
 * Simulation.h
 * Core simulation engine.
 *
 * Grid layout (80x80, 12 px/cell → 960x960 window):
 *   - INTERSECTION_SPACING = 10 → 8 roads per axis
 *   - ROAD_WIDTH = 3 cells (36 px) per road strip
 *   - Building blocks = 7x7 cells (84x84 px) between roads
 *
 * Vehicles are road-constrained:
 *   - EAST/WEST traffic travels on horizontal road rows (y = k * INTERSECTION_SPACING)
 *   - NORTH/SOUTH traffic travels on vertical road columns (x = k * INTERSECTION_SPACING)
 *   - At intersections vehicles may randomly turn onto crossing roads
 */

#ifndef SIMULATION_H
#define SIMULATION_H

#include "Vehicle.h"
#include "TrafficLight.h"
#include <vector>
#include <memory>
#include <random>

namespace TrafficSim {

// ---- grid constants -------------------------------------------------------
constexpr int   GRID_WIDTH           = 80;
constexpr int   GRID_HEIGHT          = 80;
constexpr int   INTERSECTION_SPACING = 10;
constexpr int   ROAD_WIDTH           = 3;   // cells per road strip
constexpr float COLLISION_THRESHOLD  = 1.2f;

class Simulation {
public:
    Simulation();
    ~Simulation() = default;

    void initialize(int vehicleCount = 500);

    // Main simulation steps
    void updateVehicles(float dt = 1.0f);
    void updateTrafficLights(float dt = 1.0f);
    void detectCollisions();

    // Accessors for the renderer
    const std::vector<std::unique_ptr<Vehicle>>&      getVehicles()      const { return m_vehicles; }
    const std::vector<std::unique_ptr<TrafficLight>>& getTrafficLights() const { return m_trafficLights; }
    int getGridWidth()  const { return GRID_WIDTH; }
    int getGridHeight() const { return GRID_HEIGHT; }

    // Returns true when both coordinates are at an intersection grid point
    static bool isIntersection(int gridX, int gridY);

    const TrafficLight* getTrafficLightAt(int gridX, int gridY) const;

    // Returns true if the vehicle should stop (at or approaching a red light)
    bool wouldHitRedLight(const Vehicle& vehicle, float dt = 1.0f) const;

private:
    std::vector<std::unique_ptr<Vehicle>>      m_vehicles;
    std::vector<std::unique_ptr<TrafficLight>> m_trafficLights;

    std::mt19937 m_rng;
    int          m_stepCount = 0;   // Used for deterministic turn decisions

    void createTrafficLights();
    void spawnVehicles(int count);
    bool vehiclesColliding(const Vehicle& a, const Vehicle& b) const;
};

} // namespace TrafficSim

#endif // SIMULATION_H
