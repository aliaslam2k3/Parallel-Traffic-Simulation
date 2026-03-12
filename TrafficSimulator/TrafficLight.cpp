/**
 * TrafficLight.cpp
 * Implementation of the TrafficLight class.
 */

#include "TrafficLight.h"

namespace TrafficSim {

TrafficLight::TrafficLight(int gridX, int gridY, float cycleTime)
    : m_gridX(gridX), m_gridY(gridY), m_state(LightState::GREEN),
      m_timer(0.0f), m_cycleTime(cycleTime) {
}

void TrafficLight::update(float dt) {
    m_timer += dt;
    if (m_timer >= m_cycleTime) {
        m_timer = 0.0f;
        m_state = (m_state == LightState::RED) ? LightState::GREEN : LightState::RED;
    }
}

} // namespace TrafficSim
