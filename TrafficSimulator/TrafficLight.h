/**
 * TrafficLight.h
 * Full three-aspect signal (red / yellow / green) with configurable phase times.
 * Optional adaptive scaling of green time based on nearby traffic density.
 */

#ifndef TRAFFICLIGHT_H
#define TRAFFICLIGHT_H

namespace TrafficSim {

enum class LightState {
    GREEN,
    YELLOW,
    RED
};

class TrafficLight {
public:
    TrafficLight(int gridX, int gridY,
                 float greenTime = 10.f,
                 float yellowTime = 2.5f,
                 float redTime = 12.f);

    int getGridX() const { return m_gridX; }
    int getGridY() const { return m_gridY; }
    LightState getState() const { return m_state; }

    /// 0 = green, 1 = yellow, 2 = red (for HUD / debug)
    int getStateIndex() const;

    void update(float dt);

    /// Scale green duration by factor in [0.5, 1.5] based on congestion (adaptive signals).
    void setAdaptiveGreenScale(float scale);

    /// For MPI: read phase timer after rank 0 updates.
    float getTimer() const { return m_timer; }

    /// For MPI: overwrite state from broadcast (keeps all ranks in sync).
    void assignSyncedState(LightState state, float timer);

private:
    int m_gridX;
    int m_gridY;
    LightState m_state = LightState::GREEN;
    float m_timer = 0.f;

    float m_greenTime;
    float m_yellowTime;
    float m_redTime;
    float m_adaptiveScale = 1.f;
};

} // namespace TrafficSim

#endif // TRAFFICLIGHT_H
