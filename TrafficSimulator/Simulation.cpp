/**
 * Simulation.cpp
 * Road-constrained traffic simulation.
 *
 * Road layout:
 *   Horizontal roads: y = 0, 10, 20, ... (EAST/WEST traffic)
 *   Vertical   roads: x = 0, 10, 20, ... (NORTH/SOUTH traffic)
 *
 * Vehicles stay on their road strips; at intersections they may turn.
 * The grid wraps toroidally so vehicles loop continuously.
 */

#include "Simulation.h"
#include <algorithm>
#include <cmath>

#ifdef USE_OPENMP
#include <omp.h>
#endif

namespace TrafficSim {

Simulation::Simulation() : m_rng(std::random_device{}()) {}

void Simulation::initialize(int vehicleCount) {
    createTrafficLights();
    spawnVehicles(vehicleCount);
}

// ---------------------------------------------------------------------------
// Traffic lights – one per intersection
// ---------------------------------------------------------------------------
void Simulation::createTrafficLights() {
    m_trafficLights.clear();
    for (int x = 0; x < GRID_WIDTH; x += INTERSECTION_SPACING) {
        for (int y = 0; y < GRID_HEIGHT; y += INTERSECTION_SPACING) {
            // Stagger initial state and cycle times for realistic flow
            float cycleTime = 3.0f + static_cast<float>((x / INTERSECTION_SPACING + y / INTERSECTION_SPACING) % 4);
            auto light = std::make_unique<TrafficLight>(x, y, cycleTime);

            // Offset timer so neighboring lights aren't synchronised
            // (We do this by advancing the light's internal state through
            //  a few update calls with a fake dt.)
            float initOffset = static_cast<float>((x + y) % static_cast<int>(cycleTime));
            light->update(initOffset);

            m_trafficLights.push_back(std::move(light));
        }
    }
}

// ---------------------------------------------------------------------------
// Spawn vehicles on actual road cells only
// ---------------------------------------------------------------------------
void Simulation::spawnVehicles(int count) {
    m_vehicles.clear();

    const int numRoads = GRID_WIDTH / INTERSECTION_SPACING;
    std::uniform_int_distribution<int> distRoad(0, numRoads - 1);
    std::uniform_int_distribution<int> distPos(0, GRID_WIDTH - 1);
    std::uniform_real_distribution<float> distSpeed(0.8f, 2.8f);

    for (int i = 0; i < count; ++i) {
        float speed = distSpeed(m_rng);
        float x, y;
        Direction dir;

        if (i % 2 == 0) {
            // EAST / WEST vehicle – placed on a horizontal road row
            y   = static_cast<float>(distRoad(m_rng) * INTERSECTION_SPACING);
            x   = static_cast<float>(distPos(m_rng));
            dir = (i % 4 == 0) ? Direction::EAST : Direction::WEST;
        } else {
            // NORTH / SOUTH vehicle – placed on a vertical road column
            x   = static_cast<float>(distRoad(m_rng) * INTERSECTION_SPACING);
            y   = static_cast<float>(distPos(m_rng));
            dir = (i % 4 == 1) ? Direction::SOUTH : Direction::NORTH;
        }

        m_vehicles.push_back(std::make_unique<Vehicle>(i, x, y, speed, dir));
    }
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
bool Simulation::isIntersection(int gridX, int gridY) {
    return (gridX % INTERSECTION_SPACING == 0) && (gridY % INTERSECTION_SPACING == 0);
}

const TrafficLight* Simulation::getTrafficLightAt(int gridX, int gridY) const {
    for (const auto& light : m_trafficLights) {
        if (light->getGridX() == gridX && light->getGridY() == gridY)
            return light.get();
    }
    return nullptr;
}

bool Simulation::wouldHitRedLight(const Vehicle& vehicle, float dt) const {
    // Stop if currently sitting on a red intersection
    const TrafficLight* here = getTrafficLightAt(vehicle.getGridX(), vehicle.getGridY());
    if (here && here->getState() == LightState::RED) return true;

    // Stop before entering a red intersection at the next position
    float nextX = vehicle.getX(), nextY = vehicle.getY();
    switch (vehicle.getDirection()) {
        case Direction::NORTH: nextY -= vehicle.getSpeed() * dt; break;
        case Direction::SOUTH: nextY += vehicle.getSpeed() * dt; break;
        case Direction::EAST:  nextX += vehicle.getSpeed() * dt; break;
        case Direction::WEST:  nextX -= vehicle.getSpeed() * dt; break;
    }
    int ngx = static_cast<int>(std::round(nextX));
    int ngy = static_cast<int>(std::round(nextY));
    const TrafficLight* ahead = getTrafficLightAt(ngx, ngy);
    return ahead && ahead->getState() == LightState::RED;
}

// ---------------------------------------------------------------------------
// Vehicle update – road-constrained, with intersection turns
// ---------------------------------------------------------------------------
void Simulation::updateVehicles(float dt) {
#ifdef USE_OPENMP
    #pragma omp parallel for schedule(dynamic, 32)
    for (int i = 0; i < static_cast<int>(m_vehicles.size()); ++i) {
        auto& vehicle = m_vehicles[i];
#else
    for (auto& vehicle : m_vehicles) {
#endif
        // --- Red-light stop ---
        if (wouldHitRedLight(*vehicle, dt)) continue;

        // --- Intersection turn decision ---
        float xmod = std::fmod(std::fabs(vehicle->getX()), static_cast<float>(INTERSECTION_SPACING));
        float ymod = std::fmod(std::fabs(vehicle->getY()), static_cast<float>(INTERSECTION_SPACING));
        float tol  = vehicle->getSpeed() * dt + 0.3f;

        bool nearIntersection =
            (xmod < tol || xmod > INTERSECTION_SPACING - tol) &&
            (ymod < tol || ymod > INTERSECTION_SPACING - tol);

        if (nearIntersection) {
            // Convert position to intersection indices
            int iIdx = static_cast<int>(std::round(vehicle->getX() / INTERSECTION_SPACING));
            int jIdx = static_cast<int>(std::round(vehicle->getY() / INTERSECTION_SPACING));

            // Only decide once per intersection visit
            if (iIdx != vehicle->getLastDecisionX() || jIdx != vehicle->getLastDecisionY()) {
                vehicle->setLastDecision(iIdx, jIdx);

                // Snap to exact intersection centre to keep vehicles on-grid
                float snapX = static_cast<float>(iIdx * INTERSECTION_SPACING);
                float snapY = static_cast<float>(jIdx * INTERSECTION_SPACING);
                vehicle->setPosition(snapX, snapY);

                // Deterministic pseudo-random turn: ~25% probability
                int hash = (vehicle->getId() * 7919 + iIdx * 31 + jIdx * 97 + m_stepCount) & 0xFFFF;
                if (hash % 4 == 0) {
                    Direction d = vehicle->getDirection();
                    if (d == Direction::EAST || d == Direction::WEST) {
                        vehicle->setDirection((hash % 2 == 0) ? Direction::NORTH : Direction::SOUTH);
                    } else {
                        vehicle->setDirection((hash % 2 == 0) ? Direction::EAST : Direction::WEST);
                    }
                }
            }
        }

        // --- Move ---
        vehicle->update(dt);

        // --- Toroidal wrap-around ---
        float x = vehicle->getX();
        float y = vehicle->getY();
        if      (x <  0)           x += static_cast<float>(GRID_WIDTH);
        else if (x >= GRID_WIDTH)  x -= static_cast<float>(GRID_WIDTH);
        if      (y <  0)           y += static_cast<float>(GRID_HEIGHT);
        else if (y >= GRID_HEIGHT) y -= static_cast<float>(GRID_HEIGHT);
        vehicle->setPosition(x, y);
#ifdef USE_OPENMP
    }
#else
    }
#endif
    ++m_stepCount;
}

// ---------------------------------------------------------------------------
// Traffic-light update
// ---------------------------------------------------------------------------
void Simulation::updateTrafficLights(float dt) {
    for (auto& light : m_trafficLights)
        light->update(dt);
}

// ---------------------------------------------------------------------------
// Collision detection – O(n²) push-apart (serial; parallelise later)
// ---------------------------------------------------------------------------
bool Simulation::vehiclesColliding(const Vehicle& a, const Vehicle& b) const {
    if (a.getId() == b.getId()) return false;
    float dx = a.getX() - b.getX();
    float dy = a.getY() - b.getY();
    return (dx * dx + dy * dy) < (COLLISION_THRESHOLD * COLLISION_THRESHOLD);
}

void Simulation::detectCollisions() {
    for (size_t i = 0; i < m_vehicles.size(); ++i) {
        for (size_t j = i + 1; j < m_vehicles.size(); ++j) {
            if (vehiclesColliding(*m_vehicles[i], *m_vehicles[j])) {
                Vehicle& a = *m_vehicles[i];
                Vehicle& b = *m_vehicles[j];

                float dx  = b.getX() - a.getX();
                float dy  = b.getY() - a.getY();
                float len = std::sqrt(dx * dx + dy * dy);
                if (len > 0.001f) {
                    float push = COLLISION_THRESHOLD - len;
                    float nx = dx / len;
                    float ny = dy / len;
                    a.setPosition(a.getX() - nx * push * 0.5f, a.getY() - ny * push * 0.5f);
                    b.setPosition(b.getX() + nx * push * 0.5f, b.getY() + ny * push * 0.5f);
                }
            }
        }
    }
}

} // namespace TrafficSim
