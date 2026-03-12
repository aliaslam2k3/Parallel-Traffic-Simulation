/**
 * TrafficLight.h
 * Represents a traffic light at an intersection.
 * Alternates between red and green states.
 */

#ifndef TRAFFICLIGHT_H
#define TRAFFICLIGHT_H

namespace TrafficSim {

// Traffic light state
enum class LightState {
    RED,
    GREEN
};

class TrafficLight {
public:
    TrafficLight(int gridX, int gridY, float cycleTime = 5.0f);

    // Getters
    int getGridX() const { return m_gridX; }
    int getGridY() const { return m_gridY; }
    LightState getState() const { return m_state; }

    // Update light state based on timer
    void update(float dt = 1.0f);

private:
    int m_gridX;
    int m_gridY;
    LightState m_state;
    float m_timer;
    float m_cycleTime;  // Time to switch between red and green
};

} // namespace TrafficSim

#endif // TRAFFICLIGHT_H
