/**
 * Vehicle.h
 * Represents a single vehicle in the traffic simulation.
 * Vehicles move along road strips (road-constrained), turn at intersections,
 * and are rendered with a unique per-vehicle color.
 */

#ifndef VEHICLE_H
#define VEHICLE_H

#include <cstdint>

namespace TrafficSim {

enum class Direction {
    NORTH,  // -y
    SOUTH,  // +y
    EAST,   // +x
    WEST    // -x
};

class Vehicle {
public:
    Vehicle(int id, float x, float y, float speed, Direction dir);

    // Core getters
    int       getId()        const { return m_id; }
    float     getX()         const { return m_x; }
    float     getY()         const { return m_y; }
    float     getSpeed()     const { return m_speed; }
    Direction getDirection() const { return m_direction; }

    // Unique color (HSV-generated from id)
    uint8_t getR() const { return m_r; }
    uint8_t getG() const { return m_g; }
    uint8_t getB() const { return m_b; }

    // Intersection turn-tracking (prevents re-deciding at same intersection)
    int getLastDecisionX() const { return m_lastDecisionX; }
    int getLastDecisionY() const { return m_lastDecisionY; }
    void setLastDecision(int ix, int iy) { m_lastDecisionX = ix; m_lastDecisionY = iy; }

    // Setters
    void setPosition(float x, float y);
    void setDirection(Direction dir);

    // Move one step in current direction
    void update(float dt = 1.0f);

    // Rounded grid position (for intersection/light checks)
    int getGridX() const;
    int getGridY() const;

private:
    int m_id;
    float m_x, m_y;
    float m_speed;
    Direction m_direction;

    uint8_t m_r, m_g, m_b;      // Unique vehicle color

    // Last intersection where a turn decision was made (in intersection-index space)
    int m_lastDecisionX = -999;
    int m_lastDecisionY = -999;
};

} // namespace TrafficSim

#endif // VEHICLE_H
