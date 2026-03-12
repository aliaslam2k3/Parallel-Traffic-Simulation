/**
 * Vehicle.cpp
 * Implementation of Vehicle. Color is generated using the golden-ratio HSV
 * hue method so every vehicle gets a visually distinct, saturated color.
 */

#include "Vehicle.h"
#include <cmath>

namespace TrafficSim {

Vehicle::Vehicle(int id, float x, float y, float speed, Direction dir)
    : m_id(id), m_x(x), m_y(y), m_speed(speed), m_direction(dir) {
    // Generate a unique, saturated color via HSV -> RGB conversion
    // Golden ratio spreads hues evenly: id * 0.618... mod 1
    float hue = std::fmod(id * 0.618033988749895f, 1.0f) * 360.0f;
    const float S = 0.82f;
    const float V = 0.95f;

    float C = V * S;
    float X = C * (1.0f - std::fabs(std::fmod(hue / 60.0f, 2.0f) - 1.0f));
    float m = V - C;

    float r1 = 0, g1 = 0, b1 = 0;
    if      (hue < 60)  { r1 = C; g1 = X; b1 = 0; }
    else if (hue < 120) { r1 = X; g1 = C; b1 = 0; }
    else if (hue < 180) { r1 = 0; g1 = C; b1 = X; }
    else if (hue < 240) { r1 = 0; g1 = X; b1 = C; }
    else if (hue < 300) { r1 = X; g1 = 0; b1 = C; }
    else                { r1 = C; g1 = 0; b1 = X; }

    m_r = static_cast<uint8_t>((r1 + m) * 255);
    m_g = static_cast<uint8_t>((g1 + m) * 255);
    m_b = static_cast<uint8_t>((b1 + m) * 255);
}

void Vehicle::setPosition(float x, float y) {
    m_x = x;
    m_y = y;
}

void Vehicle::setDirection(Direction dir) {
    m_direction = dir;
}

void Vehicle::update(float dt) {
    switch (m_direction) {
        case Direction::NORTH: m_y -= m_speed * dt; break;
        case Direction::SOUTH: m_y += m_speed * dt; break;
        case Direction::EAST:  m_x += m_speed * dt; break;
        case Direction::WEST:  m_x -= m_speed * dt; break;
    }
}

int Vehicle::getGridX() const {
    return static_cast<int>(std::round(m_x));
}

int Vehicle::getGridY() const {
    return static_cast<int>(std::round(m_y));
}

} // namespace TrafficSim
