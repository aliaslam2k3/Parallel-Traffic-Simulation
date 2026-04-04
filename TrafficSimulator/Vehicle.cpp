/**
 * Vehicle.cpp
 */

#include "Vehicle.h"
#include <cmath>
#include <algorithm>

namespace {

float cubicBezier(float t, float p0, float p1, float p2, float p3) {
    const float u = 1.f - t;
    return u * u * u * p0 + 3.f * u * u * t * p1 + 3.f * u * t * t * p2 + t * t * t * p3;
}

float cubicBezierDeriv(float t, float p0, float p1, float p2, float p3) {
    const float u = 1.f - t;
    return 3.f * u * u * (p1 - p0) + 6.f * u * t * (p2 - p1) + 3.f * t * t * (p3 - p2);
}

/// Exit point on the outgoing lane after a 90° turn (world y increases downward).
void turnExitP3(TrafficSim::Direction from, TrafficSim::Direction to,
                float cx, float cy, float lateral, float exitDist,
                float& p3x, float& p3y) {
    using TrafficSim::Direction;
    switch (from) {
        case Direction::EAST:
            if (to == Direction::NORTH) { p3x = cx + lateral; p3y = cy - exitDist; return; }
            if (to == Direction::SOUTH) { p3x = cx + lateral; p3y = cy + exitDist; return; }
            break;
        case Direction::WEST:
            if (to == Direction::NORTH) { p3x = cx - lateral; p3y = cy - exitDist; return; }
            if (to == Direction::SOUTH) { p3x = cx - lateral; p3y = cy + exitDist; return; }
            break;
        case Direction::NORTH:
            if (to == Direction::EAST) { p3x = cx + exitDist; p3y = cy - lateral; return; }
            if (to == Direction::WEST) { p3x = cx - exitDist; p3y = cy - lateral; return; }
            break;
        case Direction::SOUTH:
            if (to == Direction::EAST) { p3x = cx + exitDist; p3y = cy + lateral; return; }
            if (to == Direction::WEST) { p3x = cx - exitDist; p3y = cy + lateral; return; }
            break;
    }
    p3x = cx;
    p3y = cy;
}

} // namespace

namespace TrafficSim {

Vehicle::Vehicle(int id, float x, float y, float maxSpeed, Direction dir, VehicleType type)
    : m_id(id), m_x(x), m_y(y), m_maxSpeed(maxSpeed), m_direction(dir), m_type(type) {
    m_headingDeg = m_headingFromDeg = m_targetHeadingDeg = directionToHeadingDeg(dir);

    switch (type) {
        case VehicleType::Bus:
            m_accel = 18.f;
            m_decel = 38.f;
            m_maxSpeed *= 0.72f;
            break;
        case VehicleType::Bike:
            m_accel = 22.f;
            m_decel = 36.f;
            m_maxSpeed *= 0.5f;
            break;
        default:
            m_accel = 30.f;
            m_decel = 45.f;
            break;
    }

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

    if (type == VehicleType::Bus) {
        m_r = static_cast<uint8_t>(std::min(255, m_r + 15));
        m_g = static_cast<uint8_t>(std::min(255, m_g + 10));
    } else if (type == VehicleType::Bike) {
        m_g = static_cast<uint8_t>(std::min(255, m_g + 25));
    }

    m_laneOffset = ((id % 2) == 0) ? -0.25f : 0.25f;
}

float Vehicle::directionToHeadingDeg(Direction d) {
    switch (d) {
        case Direction::EAST:  return 0.f;
        case Direction::SOUTH: return 90.f;
        case Direction::WEST:  return 180.f;
        case Direction::NORTH: return 270.f;
    }
    return 0.f;
}

void Vehicle::directionToUnit(Direction d, float& outX, float& outY) {
    switch (d) {
        case Direction::EAST:  outX = 1.f;  outY = 0.f;  break;
        case Direction::WEST:  outX = -1.f; outY = 0.f;  break;
        case Direction::SOUTH: outX = 0.f;  outY = 1.f;  break;
        case Direction::NORTH: outX = 0.f;  outY = -1.f; break;
    }
}

float Vehicle::shortestAngleDelta(float fromDeg, float toDeg) const {
    float d = toDeg - fromDeg;
    while (d > 180.f) d -= 360.f;
    while (d < -180.f) d += 360.f;
    return d;
}

void Vehicle::setPosition(float x, float y) {
    m_x = x;
    m_y = y;
}

void Vehicle::setDirection(Direction dir) {
    m_direction = dir;
    m_targetHeadingDeg = directionToHeadingDeg(dir);
    if (std::fabs(shortestAngleDelta(m_headingDeg, m_targetHeadingDeg)) < 0.5f) {
        m_headingDeg = m_targetHeadingDeg;
        m_headingAnimating = false;
        m_indicatorLeft = m_indicatorRight = false;
        return;
    }
    m_headingFromDeg = m_headingDeg;
    m_headingAnimating = true;
    m_headingAnimT = 0.f;
    float d = shortestAngleDelta(m_headingFromDeg, m_targetHeadingDeg);
    m_indicatorLeft = (d < -2.f);
    m_indicatorRight = (d > 2.f);
}

bool Vehicle::beginBezierTurn(float cx, float cy, float S, Direction toDir) {
    const Direction from = m_direction;
    if (from == toDir) return false;

    float fx = 0.f, fy = 0.f, tx = 0.f, ty = 0.f;
    directionToUnit(from, fx, fy);
    directionToUnit(toDir, tx, ty);
    if (std::fabs(fx * tx + fy * ty) > 0.02f) return false;

    const float lateral = m_laneOffset * S * 0.22f;
    const float exitDist = 0.42f * S;
    const float h = 0.48f * S;

    m_p0x = m_x;
    m_p0y = m_y;
    turnExitP3(from, toDir, cx, cy, lateral, exitDist, m_p3x, m_p3y);

    m_p1x = m_p0x + fx * h;
    m_p1y = m_p0y + fy * h;
    m_p2x = m_p3x - tx * h;
    m_p2y = m_p3y - ty * h;

    m_bezierToDir = toDir;
    m_bezierTurn = true;
    m_headingAnimating = false;
    m_bezierProgress = 0.f;
    m_bezierDuration = std::clamp(0.38f * S / std::max(m_velocity, 0.35f), 0.55f, 1.35f);

    const float dh = shortestAngleDelta(directionToHeadingDeg(from), directionToHeadingDeg(toDir));
    m_indicatorLeft = (dh < -2.f);
    m_indicatorRight = (dh > 2.f);

    return true;
}

void Vehicle::updatePhysics(float dt, float cruiseSpeed) {
    if (dt <= 0.f) return;

    if (m_bezierTurn) {
        m_bezierProgress += dt / m_bezierDuration;
        const float u = std::min(1.f, m_bezierProgress);

        const float x = cubicBezier(u, m_p0x, m_p1x, m_p2x, m_p3x);
        const float y = cubicBezier(u, m_p0y, m_p1y, m_p2y, m_p3y);
        const float dx = cubicBezierDeriv(u, m_p0x, m_p1x, m_p2x, m_p3x);
        const float dy = cubicBezierDeriv(u, m_p0y, m_p1y, m_p2y, m_p3y);

        m_x = x;
        m_y = y;
        m_headingDeg = std::atan2(dy, dx) * 180.f / 3.14159265f;

        if (m_indicatorLeft || m_indicatorRight) {
            m_indicatorPhase += dt * 2.8f;
            if (m_indicatorPhase > 1.f) m_indicatorPhase -= 1.f;
        }

        if (u >= 1.f - 1e-5f) {
            m_direction = m_bezierToDir;
            m_headingDeg = directionToHeadingDeg(m_direction);
            m_bezierTurn = false;
            m_indicatorLeft = m_indicatorRight = false;
        }
        return;
    }

    if (m_headingAnimating) {
        m_headingAnimT += dt;
        float u = std::min(1.f, m_headingAnimT / kHeadingAnimDuration);
        u = u * u * (3.f - 2.f * u);
        float dh = shortestAngleDelta(m_headingFromDeg, m_targetHeadingDeg);
        m_headingDeg = m_headingFromDeg + dh * u;
        if (m_headingAnimT >= kHeadingAnimDuration) {
            m_headingDeg = m_targetHeadingDeg;
            m_headingAnimating = false;
            m_indicatorLeft = m_indicatorRight = false;
        }
    }

    if (m_indicatorLeft || m_indicatorRight) {
        m_indicatorPhase += dt * 2.8f;
        if (m_indicatorPhase > 1.f) m_indicatorPhase -= 1.f;
    }

    float target = std::min(cruiseSpeed, m_maxSpeed);
    if (m_brakeFull) target = 0.f;

    if (m_velocity < target) {
        m_velocity = std::min(target, m_velocity + m_accel * dt);
    } else {
        m_velocity = std::max(target, m_velocity - m_decel * dt);
    }

    float rad = m_headingDeg * 3.14159265f / 180.f;
    float dx = std::cos(rad) * m_velocity * dt;
    float dy = std::sin(rad) * m_velocity * dt;
    float perp = rad + 1.5707963f;
    m_x += dx + std::cos(perp) * m_laneOffset * 0.015f * std::min(m_velocity, 8.f) * dt;
    m_y += dy + std::sin(perp) * m_laneOffset * 0.015f * std::min(m_velocity, 8.f) * dt;
}

int Vehicle::getGridX() const {
    return static_cast<int>(std::round(m_x));
}

int Vehicle::getGridY() const {
    return static_cast<int>(std::round(m_y));
}

namespace {

std::uint8_t dirToU8(Direction d) {
    switch (d) {
        case Direction::EAST:  return 0;
        case Direction::SOUTH: return 1;
        case Direction::WEST:  return 2;
        case Direction::NORTH: return 3;
    }
    return 0;
}

Direction u8ToDir(std::uint8_t u) {
    switch (u % 4) {
        case 0: return Direction::EAST;
        case 1: return Direction::SOUTH;
        case 2: return Direction::WEST;
        default: return Direction::NORTH;
    }
}

std::uint8_t typeToU8(VehicleType t) {
    return static_cast<std::uint8_t>(t);
}

VehicleType u8ToType(std::uint8_t u) {
    return static_cast<VehicleType>(u % 3);
}

} // namespace

void Vehicle::serializeTo(VehicleSerialized& out) const {
    out.id = m_id;
    out.x = m_x;
    out.y = m_y;
    out.velocity = m_velocity;
    out.maxSpeed = m_maxSpeed;
    out.accel = m_accel;
    out.decel = m_decel;
    out.direction = dirToU8(m_direction);
    out.vehicleType = typeToU8(m_type);
    out.r = m_r;
    out.g = m_g;
    out.b = m_b;
    out.headingDeg = m_headingDeg;
    out.headingFromDeg = m_headingFromDeg;
    out.targetHeadingDeg = m_targetHeadingDeg;
    out.headingAnimating = m_headingAnimating ? 1u : 0u;
    out.headingAnimT = m_headingAnimT;
    out.bezierTurn = m_bezierTurn ? 1u : 0u;
    out.bezierProgress = m_bezierProgress;
    out.bezierDuration = m_bezierDuration;
    out.p0x = m_p0x;
    out.p0y = m_p0y;
    out.p1x = m_p1x;
    out.p1y = m_p1y;
    out.p2x = m_p2x;
    out.p2y = m_p2y;
    out.p3x = m_p3x;
    out.p3y = m_p3y;
    out.bezierToDir = dirToU8(m_bezierToDir);
    out.laneOffset = m_laneOffset;
    out.indicatorPhase = m_indicatorPhase;
    out.indicatorLeft = m_indicatorLeft ? 1u : 0u;
    out.indicatorRight = m_indicatorRight ? 1u : 0u;
    out.brakeFull = m_brakeFull ? 1u : 0u;
    out.lastDecisionX = m_lastDecisionX;
    out.lastDecisionY = m_lastDecisionY;
}

void Vehicle::restoreStateFromSerialized(const VehicleSerialized& s) {
    m_id = s.id;
    m_x = s.x;
    m_y = s.y;
    m_velocity = s.velocity;
    m_maxSpeed = s.maxSpeed;
    m_accel = s.accel;
    m_decel = s.decel;
    m_direction = u8ToDir(s.direction);
    m_type = u8ToType(s.vehicleType);
    m_r = s.r;
    m_g = s.g;
    m_b = s.b;
    m_headingDeg = s.headingDeg;
    m_headingFromDeg = s.headingFromDeg;
    m_targetHeadingDeg = s.targetHeadingDeg;
    m_headingAnimating = s.headingAnimating != 0;
    m_headingAnimT = s.headingAnimT;
    m_bezierTurn = s.bezierTurn != 0;
    m_bezierProgress = s.bezierProgress;
    m_bezierDuration = s.bezierDuration;
    m_p0x = s.p0x;
    m_p0y = s.p0y;
    m_p1x = s.p1x;
    m_p1y = s.p1y;
    m_p2x = s.p2x;
    m_p2y = s.p2y;
    m_p3x = s.p3x;
    m_p3y = s.p3y;
    m_bezierToDir = u8ToDir(s.bezierToDir);
    m_laneOffset = s.laneOffset;
    m_indicatorPhase = s.indicatorPhase;
    m_indicatorLeft = s.indicatorLeft != 0;
    m_indicatorRight = s.indicatorRight != 0;
    m_brakeFull = s.brakeFull != 0;
    m_lastDecisionX = s.lastDecisionX;
    m_lastDecisionY = s.lastDecisionY;
}

std::unique_ptr<Vehicle> Vehicle::deserialize(const VehicleSerialized& s) {
    Direction d = u8ToDir(s.direction);
    VehicleType t = u8ToType(s.vehicleType);
    auto v = std::make_unique<Vehicle>(s.id, s.x, s.y, s.maxSpeed, d, t);
    v->restoreStateFromSerialized(s);
    return v;
}

} // namespace TrafficSim
