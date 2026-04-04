/**
 * TrafficLight.cpp
 * Cycle: GREEN -> YELLOW -> RED -> GREEN ...
 */

#include "TrafficLight.h"
#include <algorithm>

namespace TrafficSim {

TrafficLight::TrafficLight(int gridX, int gridY, float greenTime, float yellowTime, float redTime)
    : m_gridX(gridX), m_gridY(gridY),
      m_greenTime(greenTime), m_yellowTime(yellowTime), m_redTime(redTime) {
}

int TrafficLight::getStateIndex() const {
    switch (m_state) {
        case LightState::GREEN:  return 0;
        case LightState::YELLOW: return 1;
        default: return 2;
    }
}

void TrafficLight::setAdaptiveGreenScale(float scale) {
    m_adaptiveScale = std::clamp(scale, 0.5f, 1.5f);
}

void TrafficLight::assignSyncedState(LightState state, float timer) {
    m_state = state;
    m_timer = timer;
}

void TrafficLight::update(float dt) {
    m_timer += dt;

    const float g = m_greenTime * m_adaptiveScale;

    switch (m_state) {
        case LightState::GREEN:
            if (m_timer >= g) {
                m_state = LightState::YELLOW;
                m_timer = 0.f;
            }
            break;
        case LightState::YELLOW:
            if (m_timer >= m_yellowTime) {
                m_state = LightState::RED;
                m_timer = 0.f;
            }
            break;
        case LightState::RED:
            if (m_timer >= m_redTime) {
                m_state = LightState::GREEN;
                m_timer = 0.f;
            }
            break;
    }
}

} // namespace TrafficSim
