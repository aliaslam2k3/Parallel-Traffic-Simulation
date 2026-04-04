/**
 * Simulation.h
 * Larger city: 240×240 grid, hierarchical road network.
 * Traffic signals at major (arterial) intersections; local crossings are unsignalized.
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
constexpr int   INTERSECTION_SPACING  = CityMap::LOCAL_SPACING;  // Road / turn grid (every local street)
/// Signals only at major (arterial) intersections — fewer lights, closer to a real city
constexpr int   SIGNAL_SPACING        = CityMap::ARTERIAL_SPACING;
constexpr float COLLISION_THRESHOLD  = 1.8f;

class Simulation {
public:
    Simulation();
    ~Simulation() = default;

    void initialize(int vehicleCount = 0);

    /// Lights only (no vehicles); for MPI when each rank builds its own local vehicle list.
    void createTrafficLightsOnly();

    /// Spawn approximately totalVehicles/worldSize vehicles whose y lies in this rank's horizontal strip.
    void spawnVehiclesForMpiRank(int rank, int worldSize, int totalVehicles);

    void updateVehicles(float dt = 1.0f);
    /// Update an arbitrary vehicle container (same logic as updateVehicles; used for MPI local lists).
    void updateVehicles(std::vector<std::unique_ptr<Vehicle>>& vehicles, float dt);

    void updateTrafficLights(float dt = 1.0f);
    void updateTrafficLights(float dt, const std::vector<std::unique_ptr<Vehicle>>& vehicles);

    void detectCollisions();
    void detectCollisions(std::vector<std::unique_ptr<Vehicle>>& vehicles);

    const std::vector<std::unique_ptr<Vehicle>>&      getVehicles()      const { return m_vehicles; }
    /// Non-const access for MPI migration (remove/send, receive/emplace).
    std::vector<std::unique_ptr<Vehicle>>&            getVehiclesMutable() { return m_vehicles; }
    const std::vector<std::unique_ptr<TrafficLight>>& getTrafficLights() const { return m_trafficLights; }
    int getGridWidth()  const { return GRID_WIDTH; }
    int getGridHeight() const { return GRID_HEIGHT; }

    static bool isIntersection(int gridX, int gridY);
    const TrafficLight* getTrafficLightAt(int gridX, int gridY) const;

private:
    void updateVehiclesImpl(std::vector<std::unique_ptr<Vehicle>>& vehicles, float dt);
    void updateTrafficLightsImpl(const std::vector<std::unique_ptr<Vehicle>>& vehicles, float dt);
    void detectCollisionsImpl(std::vector<std::unique_ptr<Vehicle>>& vehicles);

    /// Returns true if this step should not apply normal movement (may snap vehicle to stop line).
    bool applyRedLightStop(Vehicle& vehicle, float dt);

    /// Keep x/y on the correct road strip (collision pushes can drift off the lane).
    void clampVehicleToRoad(Vehicle& vehicle);

    std::vector<std::unique_ptr<Vehicle>>      m_vehicles;
    std::vector<std::unique_ptr<TrafficLight>> m_trafficLights;

    std::mt19937 m_rng;
    int          m_stepCount = 0;

    void createTrafficLights();
    void spawnVehicles(int count);
};

} // namespace TrafficSim

#endif // SIMULATION_H
