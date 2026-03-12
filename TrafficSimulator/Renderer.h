/**
 * Renderer.h
 * Draws the city road map: big blocks, parks, roads, traffic lights.
 * Fullscreen with square aspect ratio.
 */

#ifndef RENDERER_H
#define RENDERER_H

#include "Simulation.h"
#include <SFML/Graphics.hpp>
#include <memory>
#include <optional>

namespace TrafficSim {

class Renderer {
public:
    Renderer(int windowWidth = 1200, int windowHeight = 1200);
    ~Renderer() = default;

    // fullscreen: if true, use fullscreen; else use windowed
    bool initialize(bool fullscreen = true);
    void render(const Simulation& simulation);

    bool isOpen() const { return m_window && m_window->isOpen(); }
    std::optional<sf::Event> pollEvent() { return m_window ? m_window->pollEvent() : std::nullopt; }
    void display() { if (m_window) m_window->display(); }
    void clear(const sf::Color& color = sf::Color(22, 24, 28)) { if (m_window) m_window->clear(color); }

private:
    std::unique_ptr<sf::RenderWindow> m_window;
    int   m_windowWidth;
    int   m_windowHeight;
    float m_cellWidth;
    float m_cellHeight;

    void drawCityBlocks();                         // Parks, buildings (residential + downtown)
    void drawRoads();                              // Asphalt, lane markings, curbs
    void drawTrafficLights(const Simulation& sim); // Traffic signals at intersections
    void drawVehicles(const Simulation& sim);     // (Empty for Step 1)
};

} // namespace TrafficSim

#endif // RENDERER_H
