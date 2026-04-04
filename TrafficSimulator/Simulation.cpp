/**
 * Simulation.cpp
 * Road-constrained traffic; physics-based vehicles; full signal phases.
 */

#include "Simulation.h"
#include "Vehicle.h"
#include "CityMap.h"
#include "DomainDecomposition.h"
#include <algorithm>
#include <cmath>

#ifdef USE_OPENMP
#include <omp.h>
#endif

namespace {

/// Grid coordinate of the centerline of the road strip whose anchor line is `anchorCoord`
/// (anchor is a multiple of INTERSECTION_SPACING on that axis).
inline float roadCenterAlongAxis(int anchorCoord) {
    const int w = TrafficSim::CityMap::getRoadWidthAt(anchorCoord);
    return static_cast<float>(anchorCoord) + static_cast<float>(w) * 0.5f;
}

/// Road–road overlap in anchor space (matches CityMap drawing): only true inside the wx×wy tile at each junction.
inline bool isInIntersectionOverlap(float ax, float ay) {
    const int step = TrafficSim::INTERSECTION_SPACING;
    const int ix = static_cast<int>(std::floor(ax / static_cast<float>(step)));
    const int iy = static_cast<int>(std::floor(ay / static_cast<float>(step)));
    const int gx = ix * step;
    const int gy = iy * step;
    const int wx = TrafficSim::CityMap::getRoadWidthAt(gx);
    const int wy = TrafficSim::CityMap::getRoadWidthAt(gy);
    return ax >= static_cast<float>(gx) && ax < static_cast<float>(gx + wx) &&
           ay >= static_cast<float>(gy) && ay < static_cast<float>(gy + wy);
}

/// Convert lane-centered position to anchor coordinates for overlap tests (same as road strip origin in CityMap).
inline void vehicleAnchorSpace(const TrafficSim::Vehicle& v, float& ax, float& ay) {
    using TrafficSim::Direction;
    const float stepF = static_cast<float>(TrafficSim::INTERSECTION_SPACING);
    const int step = TrafficSim::INTERSECTION_SPACING;
    const int maxLaneX = TrafficSim::GRID_WIDTH / step - 1;
    const int maxLaneY = TrafficSim::GRID_HEIGHT / step - 1;
    switch (v.getDirection()) {
        case Direction::EAST:
        case Direction::WEST: {
            int lane = static_cast<int>(std::round(v.getY() / stepF));
            lane = std::clamp(lane, 0, maxLaneY);
            const int anchor = lane * step;
            ay = v.getY() - 0.5f * static_cast<float>(TrafficSim::CityMap::getRoadWidthAt(anchor));
            ax = v.getX();
            break;
        }
        case Direction::NORTH:
        case Direction::SOUTH: {
            int lane = static_cast<int>(std::round(v.getX() / stepF));
            lane = std::clamp(lane, 0, maxLaneX);
            const int anchor = lane * step;
            ax = v.getX() - 0.5f * static_cast<float>(TrafficSim::CityMap::getRoadWidthAt(anchor));
            ay = v.getY();
            break;
        }
    }
}

/// E/W traffic: only react to signals on the same horizontal road strip as the vehicle (anchor-space overlap).
inline bool horizontalLaneOverlapsSignalY(float vy, int sy) {
    const float stepF = static_cast<float>(TrafficSim::INTERSECTION_SPACING);
    const int step = TrafficSim::INTERSECTION_SPACING;
    const int maxLaneY = TrafficSim::GRID_HEIGHT / step - 1;
    int lane = static_cast<int>(std::round(vy / stepF));
    lane = std::clamp(lane, 0, maxLaneY);
    const int anchorY = lane * step;
    const float y0 = static_cast<float>(anchorY);
    const float y1 = y0 + static_cast<float>(TrafficSim::CityMap::getRoadWidthAt(anchorY));
    const float gy0 = static_cast<float>(sy);
    const float gy1 = gy0 + static_cast<float>(TrafficSim::CityMap::getRoadWidthAt(sy));
    return !(y1 <= gy0 || gy1 <= y0);
}

/// N/S traffic: only react to signals on the same vertical road strip as the vehicle.
inline bool verticalLaneOverlapsSignalX(float vx, int sx) {
    const float stepF = static_cast<float>(TrafficSim::INTERSECTION_SPACING);
    const int step = TrafficSim::INTERSECTION_SPACING;
    const int maxLaneX = TrafficSim::GRID_WIDTH / step - 1;
    int lane = static_cast<int>(std::round(vx / stepF));
    lane = std::clamp(lane, 0, maxLaneX);
    const int anchorX = lane * step;
    const float x0 = static_cast<float>(anchorX);
    const float x1 = x0 + static_cast<float>(TrafficSim::CityMap::getRoadWidthAt(anchorX));
    const float gx0 = static_cast<float>(sx);
    const float gx1 = gx0 + static_cast<float>(TrafficSim::CityMap::getRoadWidthAt(sx));
    return !(x1 <= gx0 || gx1 <= x0);
}

float shortestDeltaTorus(float a, float b, float period) {
    float d = b - a;
    const float half = period * 0.5f;
    if (d > half) d -= period;
    else if (d < -half) d += period;
    return d;
}

} // namespace

namespace TrafficSim {

Simulation::Simulation() : m_rng(std::random_device{}()) {}

void Simulation::initialize(int vehicleCount) {
    createTrafficLights();
    spawnVehicles(vehicleCount);
}

void Simulation::createTrafficLightsOnly() {
    createTrafficLights();
    m_vehicles.clear();
}

void Simulation::spawnVehiclesForMpiRank(int rank, int worldSize, int totalVehicles) {
    m_vehicles.clear();
    if (worldSize <= 0) return;
    DomainDecomposition dom(rank, worldSize, static_cast<float>(GRID_HEIGHT));

    const int numRoads = GRID_WIDTH / INTERSECTION_SPACING;
    std::uniform_int_distribution<int> distRoad(0, numRoads - 1);
    std::uniform_int_distribution<int> distPos(0, GRID_WIDTH - 1);
    std::uniform_real_distribution<float> distMaxSpeed(2.6f, 7.0f);

    const int base = totalVehicles / worldSize;
    const int rem = totalVehicles % worldSize;
    const int localCount = base + (rank < rem ? 1 : 0);

    int spawned = 0;
    int attempt = 0;
    const int maxAttempts = std::max(localCount * 400, 5000);

    while (spawned < localCount && attempt < maxAttempts) {
        ++attempt;
        const int globalId = rank + spawned * worldSize;
        float maxSp = distMaxSpeed(m_rng);
        float x = 0.f, y = 0.f;
        Direction dir = Direction::EAST;
        VehicleType vt = static_cast<VehicleType>(globalId % 3);

        if (globalId % 2 == 0) {
            const int anchorY = distRoad(m_rng) * INTERSECTION_SPACING;
            y = roadCenterAlongAxis(anchorY);
            x = static_cast<float>(distPos(m_rng));
            dir = (globalId % 4 == 0) ? Direction::EAST : Direction::WEST;
        } else {
            const int anchorX = distRoad(m_rng) * INTERSECTION_SPACING;
            x = roadCenterAlongAxis(anchorX);
            y = static_cast<float>(distPos(m_rng));
            dir = (globalId % 4 == 1) ? Direction::SOUTH : Direction::NORTH;
        }

        if (!dom.containsY(y)) continue;

        m_vehicles.push_back(std::make_unique<Vehicle>(globalId, x, y, maxSp, dir, vt));
        ++spawned;
    }
}

void Simulation::createTrafficLights() {
    m_trafficLights.clear();
    for (int x = 0; x < GRID_WIDTH; x += SIGNAL_SPACING) {
        for (int y = 0; y < GRID_HEIGHT; y += SIGNAL_SPACING) {
            float g = 9.f + static_cast<float>((x / SIGNAL_SPACING + y / SIGNAL_SPACING) % 5);
            float yel = 2.2f;
            float r = 10.f + static_cast<float>((x + y) % 4);
            auto light = std::make_unique<TrafficLight>(x, y, g, yel, r);

            float initOffset = static_cast<float>((x + y) % 8);
            for (float t = 0; t < initOffset; t += 0.5f)
                light->update(0.5f);

            m_trafficLights.push_back(std::move(light));
        }
    }
}

void Simulation::spawnVehicles(int count) {
    m_vehicles.clear();

    const int numRoads = GRID_WIDTH / INTERSECTION_SPACING;
    std::uniform_int_distribution<int> distRoad(0, numRoads - 1);
    std::uniform_int_distribution<int> distPos(0, GRID_WIDTH - 1);
    std::uniform_real_distribution<float> distMaxSpeed(2.6f, 7.0f);

    for (int i = 0; i < count; ++i) {
        float maxSp = distMaxSpeed(m_rng);
        float x, y;
        Direction dir;
        VehicleType vt = static_cast<VehicleType>(i % 3);

        if (i % 2 == 0) {
            const int anchorY = distRoad(m_rng) * INTERSECTION_SPACING;
            y = roadCenterAlongAxis(anchorY);
            x = static_cast<float>(distPos(m_rng));
            dir = (i % 4 == 0) ? Direction::EAST : Direction::WEST;
        } else {
            const int anchorX = distRoad(m_rng) * INTERSECTION_SPACING;
            x = roadCenterAlongAxis(anchorX);
            y = static_cast<float>(distPos(m_rng));
            dir = (i % 4 == 1) ? Direction::SOUTH : Direction::NORTH;
        }

        m_vehicles.push_back(std::make_unique<Vehicle>(i, x, y, maxSp, dir, vt));
    }
}

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

void Simulation::clampVehicleToRoad(Vehicle& vehicle) {
    if (vehicle.isInBezierTurn()) return;

    const float step = static_cast<float>(INTERSECTION_SPACING);
    const int maxLaneX = GRID_WIDTH / INTERSECTION_SPACING - 1;
    const int maxLaneY = GRID_HEIGHT / INTERSECTION_SPACING - 1;
    const float blend = vehicle.isHeadingAnimating() ? 0.42f : 1.f;

    switch (vehicle.getDirection()) {
        case Direction::EAST:
        case Direction::WEST: {
            int lane = static_cast<int>(std::round(vehicle.getY() / step));
            lane = std::clamp(lane, 0, maxLaneY);
            const int anchor = lane * static_cast<int>(step);
            const float targetY = roadCenterAlongAxis(anchor);
            const float y = vehicle.getY();
            vehicle.setPosition(vehicle.getX(), y + (targetY - y) * blend);
            break;
        }
        case Direction::NORTH:
        case Direction::SOUTH: {
            int lane = static_cast<int>(std::round(vehicle.getX() / step));
            lane = std::clamp(lane, 0, maxLaneX);
            const int anchor = lane * static_cast<int>(step);
            const float targetX = roadCenterAlongAxis(anchor);
            const float x = vehicle.getX();
            vehicle.setPosition(x + (targetX - x) * blend, vehicle.getY());
            break;
        }
    }
}

/// Returns true if vehicle must hold / slow for signal (red or yellow with room to stop).
bool Simulation::applyRedLightStop(Vehicle& vehicle, float dt) {
    if (vehicle.isInBezierTurn()) return false;

    const float vx = vehicle.getX();
    const float vy = vehicle.getY();
    const float vel = std::max(vehicle.getVelocity(), 0.01f);

    float nx = vx;
    float ny = vy;
    float rad = vehicle.getHeadingDegrees() * 3.14159265f / 180.f;
    nx += std::cos(rad) * vel * dt;
    ny += std::sin(rad) * vel * dt;

    constexpr float holdTol = 0.55f;
    constexpr float stopEps = 0.06f;

    const float loX = std::min(vx, nx);
    const float hiX = std::max(vx, nx);
    const float loY = std::min(vy, ny);
    const float hiY = std::max(vy, ny);

    for (const auto& light : m_trafficLights) {
        const LightState st = light->getState();
        if (st == LightState::GREEN) continue;

        const float sx = static_cast<float>(light->getGridX());
        const float sy = static_cast<float>(light->getGridY());
        const int sxi = light->getGridX();
        const int syi = light->getGridY();

        // At stop line on the correct approach: hold on red; skip further checks for yellow (original behavior).
        if (vehicle.getDirection() == Direction::EAST || vehicle.getDirection() == Direction::WEST) {
            if (std::fabs(vx - sx) < holdTol && horizontalLaneOverlapsSignalY(vy, syi)) {
                if (st == LightState::RED) return true;
                continue;
            }
        } else {
            if (std::fabs(vy - sy) < holdTol && verticalLaneOverlapsSignalX(vx, sxi)) {
                if (st == LightState::RED) return true;
                continue;
            }
        }

        auto approachingStop = [&](float px, float py, float lineX, float lineY, char axis) -> bool {
            if (st == LightState::YELLOW) {
                float dist = (axis == 'x') ? std::fabs(px - lineX) : std::fabs(py - lineY);
                if (dist > 4.5f) return true;
                return false;
            }
            return true;
        };

        switch (vehicle.getDirection()) {
            case Direction::EAST: {
                if (!horizontalLaneOverlapsSignalY(vy, syi)) break;
                const bool strictCross = (loX < sx && hiX > sx);
                const bool approach = (vx < sx && nx >= sx);
                if ((strictCross || approach) && st == LightState::RED) {
                    vehicle.setPosition(std::max(vx, sx - stopEps), vy);
                    return true;
                }
                if (approach && st == LightState::YELLOW && approachingStop(vx, vy, sx, sy, 'x')) {
                    vehicle.setPosition(std::max(vx, sx - stopEps - 0.4f), vy);
                    return true;
                }
                break;
            }
            case Direction::WEST: {
                if (!horizontalLaneOverlapsSignalY(vy, syi)) break;
                const bool strictCross = (loX < sx && hiX > sx);
                const bool approach = (vx > sx && nx <= sx);
                if ((strictCross || approach) && st == LightState::RED) {
                    vehicle.setPosition(std::min(vx, sx + stopEps), vy);
                    return true;
                }
                if (approach && st == LightState::YELLOW && approachingStop(vx, vy, sx, sy, 'x')) {
                    vehicle.setPosition(std::min(vx, sx + stopEps + 0.4f), vy);
                    return true;
                }
                break;
            }
            case Direction::SOUTH: {
                if (!verticalLaneOverlapsSignalX(vx, sxi)) break;
                const bool strictCross = (loY < sy && hiY > sy);
                const bool approach = (vy < sy && ny >= sy);
                if ((strictCross || approach) && st == LightState::RED) {
                    vehicle.setPosition(vx, std::max(vy, sy - stopEps));
                    return true;
                }
                if (approach && st == LightState::YELLOW && approachingStop(vx, vy, sx, sy, 'y')) {
                    vehicle.setPosition(vx, std::max(vy, sy - stopEps - 0.4f));
                    return true;
                }
                break;
            }
            case Direction::NORTH: {
                if (!verticalLaneOverlapsSignalX(vx, sxi)) break;
                const bool strictCross = (loY < sy && hiY > sy);
                const bool approach = (vy > sy && ny <= sy);
                if ((strictCross || approach) && st == LightState::RED) {
                    vehicle.setPosition(vx, std::min(vy, sy + stopEps));
                    return true;
                }
                if (approach && st == LightState::YELLOW && approachingStop(vx, vy, sx, sy, 'y')) {
                    vehicle.setPosition(vx, std::min(vy, sy + stopEps + 0.4f));
                    return true;
                }
                break;
            }
        }
    }
    return false;
}

void Simulation::updateVehicles(float dt) {
    updateVehiclesImpl(m_vehicles, dt);
}

void Simulation::updateVehicles(std::vector<std::unique_ptr<Vehicle>>& vehicles, float dt) {
    updateVehiclesImpl(vehicles, dt);
}

void Simulation::updateVehiclesImpl(std::vector<std::unique_ptr<Vehicle>>& vehicles, float dt) {
#ifdef USE_OPENMP
    #pragma omp parallel for schedule(dynamic, 32)
    for (int i = 0; i < static_cast<int>(vehicles.size()); ++i) {
        auto& vehicle = vehicles[i];
#else
    for (auto& vehicle : vehicles) {
#endif
        vehicle->setBrakingFull(false);

        if (vehicle->isInBezierTurn()) {
            const float cruise = vehicle->getMaxSpeed();
            vehicle->updatePhysics(dt, cruise);
            float x = vehicle->getX();
            float y = vehicle->getY();
            if (x < 0) x += static_cast<float>(GRID_WIDTH);
            else if (x >= GRID_WIDTH) x -= static_cast<float>(GRID_WIDTH);
            if (y < 0) y += static_cast<float>(GRID_HEIGHT);
            else if (y >= GRID_HEIGHT) y -= static_cast<float>(GRID_HEIGHT);
            vehicle->setPosition(x, y);
            continue;
        }

        if (applyRedLightStop(*vehicle, dt)) {
            vehicle->setBrakingFull(true);
            vehicle->updatePhysics(dt, 0.f);
            clampVehicleToRoad(*vehicle);
            continue;
        }

        const float stepF = static_cast<float>(INTERSECTION_SPACING);
        float ax = 0.f, ay = 0.f;
        vehicleAnchorSpace(*vehicle, ax, ay);
        const bool nearIntersection = isInIntersectionOverlap(ax, ay);

        if (nearIntersection) {
            const int iIdx = static_cast<int>(std::floor(ax / stepF));
            const int jIdx = static_cast<int>(std::floor(ay / stepF));

            if (iIdx != vehicle->getLastDecisionX() || jIdx != vehicle->getLastDecisionY()) {
                vehicle->setLastDecision(iIdx, jIdx);

                const int axInt = iIdx * INTERSECTION_SPACING;
                const int ayInt = jIdx * INTERSECTION_SPACING;
                const int iwx = CityMap::getRoadWidthAt(axInt);
                const int iwy = CityMap::getRoadWidthAt(ayInt);
                const float centerX = static_cast<float>(axInt) + static_cast<float>(iwx) * 0.5f;
                const float centerY = static_cast<float>(ayInt) + static_cast<float>(iwy) * 0.5f;

                int hash = (vehicle->getId() * 7919 + iIdx * 31 + jIdx * 97 + m_stepCount) & 0xFFFF;
                if (hash % 4 == 0) {
                    Direction d = vehicle->getDirection();
                    Direction newDir = d;
                    if (d == Direction::EAST || d == Direction::WEST)
                        newDir = (hash % 2 == 0) ? Direction::NORTH : Direction::SOUTH;
                    else
                        newDir = (hash % 2 == 0) ? Direction::EAST : Direction::WEST;
                    if (!vehicle->beginBezierTurn(centerX, centerY, static_cast<float>(INTERSECTION_SPACING), newDir))
                        vehicle->setDirection(newDir);
                }
            }
        }

        float cruise = vehicle->getMaxSpeed();
        vehicle->updatePhysics(dt, cruise);

        float x = vehicle->getX();
        float y = vehicle->getY();
        if (x < 0) x += static_cast<float>(GRID_WIDTH);
        else if (x >= GRID_WIDTH) x -= static_cast<float>(GRID_WIDTH);
        if (y < 0) y += static_cast<float>(GRID_HEIGHT);
        else if (y >= GRID_HEIGHT) y -= static_cast<float>(GRID_HEIGHT);
        vehicle->setPosition(x, y);
        clampVehicleToRoad(*vehicle);
#ifdef USE_OPENMP
    }
#else
    }
#endif
    ++m_stepCount;
}

static int countNear(const std::vector<std::unique_ptr<Vehicle>>& vehicles, int gx, int gy, float radius) {
    int n = 0;
    for (const auto& v : vehicles) {
        float dx = v->getX() - static_cast<float>(gx);
        float dy = v->getY() - static_cast<float>(gy);
        if (dx * dx + dy * dy < radius * radius) ++n;
    }
    return n;
}

void Simulation::updateTrafficLights(float dt) {
    updateTrafficLightsImpl(m_vehicles, dt);
}

void Simulation::updateTrafficLights(float dt, const std::vector<std::unique_ptr<Vehicle>>& vehicles) {
    updateTrafficLightsImpl(vehicles, dt);
}

void Simulation::updateTrafficLightsImpl(const std::vector<std::unique_ptr<Vehicle>>& vehicles, float dt) {
    for (auto& light : m_trafficLights) {
        int c = countNear(vehicles, light->getGridX(), light->getGridY(), 14.f);
        float scale = 1.f + 0.04f * static_cast<float>(std::min(c, 12));
        light->setAdaptiveGreenScale(scale);
        light->update(dt);
    }
}

void Simulation::detectCollisions() {
    detectCollisionsImpl(m_vehicles);
}

void Simulation::detectCollisions(std::vector<std::unique_ptr<Vehicle>>& vehicles) {
    detectCollisionsImpl(vehicles);
}

void Simulation::detectCollisionsImpl(std::vector<std::unique_ptr<Vehicle>>& vehicles) {
    const float pw = static_cast<float>(GRID_WIDTH);
    const float ph = static_cast<float>(GRID_HEIGHT);

    for (int pass = 0; pass < 4; ++pass) {
        for (size_t i = 0; i < vehicles.size(); ++i) {
            for (size_t j = i + 1; j < vehicles.size(); ++j) {
                Vehicle& va = *vehicles[i];
                Vehicle& vb = *vehicles[j];

                float dx = shortestDeltaTorus(va.getX(), vb.getX(), pw);
                float dy = shortestDeltaTorus(va.getY(), vb.getY(), ph);
                const float dist = std::sqrt(dx * dx + dy * dy);

                if (dist >= COLLISION_THRESHOLD) continue;

                float nx, ny;
                if (dist > 1e-5f) {
                    nx = dx / dist;
                    ny = dy / dist;
                } else {
                    nx = 1.f;
                    ny = 0.f;
                }

                const float push = COLLISION_THRESHOLD - dist;
                va.setPosition(va.getX() - nx * push * 0.5f, va.getY() - ny * push * 0.5f);
                vb.setPosition(vb.getX() + nx * push * 0.5f, vb.getY() + ny * push * 0.5f);
            }
        }
    }

    for (auto& v : vehicles) {
        clampVehicleToRoad(*v);
        float x = v->getX();
        float y = v->getY();
        if (x < 0.f) x += pw;
        else if (x >= pw) x -= pw;
        if (y < 0.f) y += ph;
        else if (y >= ph) y -= ph;
        v->setPosition(x, y);
    }
}

} // namespace TrafficSim
