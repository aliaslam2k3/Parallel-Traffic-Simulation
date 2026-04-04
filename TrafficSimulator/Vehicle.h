/**
 * Vehicle.h
 * Physics-based movement: velocity, acceleration, lane offset.
 * Smooth heading interpolation on direction change; turn indicators; vehicle types.
 */

#ifndef VEHICLE_H
#define VEHICLE_H

#include <cstdint>
#include <cstddef>
#include <memory>

namespace TrafficSim {

#pragma pack(push, 1)
/// Packed state for MPI migration (fixed layout, trivially copyable).
struct VehicleSerialized {
    std::int32_t id = 0;
    float x = 0.f, y = 0.f;
    float velocity = 0.f;
    float maxSpeed = 0.f;
    float accel = 0.f;
    float decel = 0.f;
    std::uint8_t direction = 0;
    std::uint8_t vehicleType = 0;
    std::uint8_t r = 0, g = 0, b = 0;
    float headingDeg = 0.f;
    float headingFromDeg = 0.f;
    float targetHeadingDeg = 0.f;
    std::uint8_t headingAnimating = 0;
    float headingAnimT = 0.f;
    std::uint8_t bezierTurn = 0;
    float bezierProgress = 0.f;
    float bezierDuration = 0.f;
    float p0x = 0.f, p0y = 0.f, p1x = 0.f, p1y = 0.f;
    float p2x = 0.f, p2y = 0.f, p3x = 0.f, p3y = 0.f;
    std::uint8_t bezierToDir = 0;
    float laneOffset = 0.f;
    float indicatorPhase = 0.f;
    std::uint8_t indicatorLeft = 0;
    std::uint8_t indicatorRight = 0;
    std::uint8_t brakeFull = 0;
    std::int32_t lastDecisionX = -999;
    std::int32_t lastDecisionY = -999;
};
#pragma pack(pop)

enum class Direction {
    NORTH, SOUTH, EAST, WEST
};

enum class VehicleType {
    Car,
    Bus,
    Bike
};

class Vehicle {
public:
    Vehicle(int id, float x, float y, float maxSpeed, Direction dir, VehicleType type = VehicleType::Car);

    int         getId()           const { return m_id; }
    float       getX()            const { return m_x; }
    float       getY()            const { return m_y; }
    float       getVelocity()     const { return m_velocity; }
    float       getMaxSpeed()     const { return m_maxSpeed; }
    Direction   getDirection()    const { return m_direction; }
    VehicleType getVehicleType()  const { return m_type; }

    float getHeadingDegrees() const { return m_headingDeg; }

    float getLaneOffset() const { return m_laneOffset; }

    float getIndicatorPhase() const { return m_indicatorPhase; }
    bool  isIndicatorLeft()   const { return m_indicatorLeft; }
    bool  isIndicatorRight()  const { return m_indicatorRight; }

    uint8_t getR() const { return m_r; }
    uint8_t getG() const { return m_g; }
    uint8_t getB() const { return m_b; }

    int getLastDecisionX() const { return m_lastDecisionX; }
    int getLastDecisionY() const { return m_lastDecisionY; }
    void setLastDecision(int ix, int iy) { m_lastDecisionX = ix; m_lastDecisionY = iy; }

    void setPosition(float x, float y);
    /// Sets logical direction and starts smooth heading animation toward it.
    void setDirection(Direction dir);

    /// Follows a cubic Bezier through the intersection (90° only). Logical direction stays
    /// incoming until the curve completes; then it becomes `toDir`. Returns false if not a right-angle turn.
    bool beginBezierTurn(float centerX, float centerY, float roadSpacing, Direction toDir);

    bool isInBezierTurn() const { return m_bezierTurn; }
    bool isHeadingAnimating() const { return m_headingAnimating; }

    void setBrakingFull(bool on) { m_brakeFull = on; }

    void updatePhysics(float dt, float cruiseSpeed);

    int getGridX() const;
    int getGridY() const;

    void serializeTo(VehicleSerialized& out) const;
    /// After construction with same id/type/dir, overwrites full state from migration packet.
    void restoreStateFromSerialized(const VehicleSerialized& s);
    static std::unique_ptr<Vehicle> deserialize(const VehicleSerialized& s);

    static constexpr std::size_t serializedByteSize() { return sizeof(VehicleSerialized); }

private:
    static float directionToHeadingDeg(Direction d);
    static void directionToUnit(Direction d, float& outX, float& outY);
    float shortestAngleDelta(float fromDeg, float toDeg) const;

    int m_id;
    float m_x, m_y;
    float m_velocity = 0.f;
    float m_maxSpeed;
    float m_accel;
    float m_decel;
    Direction m_direction;
    VehicleType m_type;

    float m_headingDeg = 0.f;
    float m_headingFromDeg = 0.f;
    float m_targetHeadingDeg = 0.f;
    bool  m_headingAnimating = false;
    float m_headingAnimT = 0.f;
    static constexpr float kHeadingAnimDuration = 0.4f;

    /// Cubic Bezier cornering (position follows curve; heading follows tangent).
    bool  m_bezierTurn = false;
    float m_bezierProgress = 0.f;
    float m_bezierDuration = 0.75f;
    float m_p0x = 0.f, m_p0y = 0.f, m_p1x = 0.f, m_p1y = 0.f;
    float m_p2x = 0.f, m_p2y = 0.f, m_p3x = 0.f, m_p3y = 0.f;
    Direction m_bezierToDir = Direction::EAST;

    float m_laneOffset = 0.f;

    float m_indicatorPhase = 0.f;
    bool  m_indicatorLeft = false;
    bool  m_indicatorRight = false;

    bool m_brakeFull = false;

    uint8_t m_r, m_g, m_b;

    int m_lastDecisionX = -999;
    int m_lastDecisionY = -999;
};

} // namespace TrafficSim

#endif // VEHICLE_H
