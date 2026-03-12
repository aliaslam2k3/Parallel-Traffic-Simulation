/**
 * Simulation.h
 * Larger city: 240×240 grid, hierarchical road network.
 * Traffic lights at all intersections.
 */

#ifndef SIMULATION_H
#define SIMULATION_H

#include "Vehicle.h"
#include "TrafficLight.h"
#include "CityMap.h"
#include <vector>
#include <memory>
#include <random>

namespace TrafficSim {

// Use CityMap grid constants
constexpr int   GRID_WIDTH           = CityMap::GRID_SIZE;
constexpr int   GRID_HEIGHT          = CityMap::GRID_SIZE;
constexpr int   INTERSECTION_SPACING  = CityMap::LOCAL_SPACING;  // For traffic light placement
constexpr float COLLISION_THRESHOLD  = 1.2f;

class Simulation {
public:
    Simulation();
    ~Simulation() = default;

    void initialize(int vehicleCount = 0);

    void updateVehicles(float dt = 1.0f);
    void updateTrafficLights(float dt = 1.0f);
    void detectCollisions();

    const std::vector<std::unique_ptr<Vehicle>>&      getVehicles()      const { return m_vehicles; }
    const std::vector<std::unique_ptr<TrafficLight>>& getTrafficLights() const { return m_trafficLights; }
    int getGridWidth()  const { return GRID_WIDTH; }
    int getGridHeight() const { return GRID_HEIGHT; }

    static bool isIntersection(int gridX, int gridY);
    const TrafficLight* getTrafficLightAt(int gridX, int gridY) const;
    bool wouldHitRedLight(const Vehicle& vehicle, float dt = 1.0f) const;

private:
    std::vector<std::unique_ptr<Vehicle>>      m_vehicles;
    std::vector<std::unique_ptr<TrafficLight>> m_trafficLights;

    std::mt19937 m_rng;
    int          m_stepCount = 0;

    void createTrafficLights();
    void spawnVehicles(int count);
    bool vehiclesColliding(const Vehicle& a, const Vehicle& b) const;
};

} // namespace TrafficSim

#endif // SIMULATION_H
