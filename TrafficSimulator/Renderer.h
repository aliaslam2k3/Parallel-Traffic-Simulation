/**
 * Renderer.h
 * SFML-based city map renderer.
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
    Renderer(int windowWidth = 960, int windowHeight = 960);
    ~Renderer() = default;

    bool initialize();
    void render(const Simulation& simulation);

    bool isOpen() const { return m_window && m_window->isOpen(); }
    std::optional<sf::Event> pollEvent() { return m_window ? m_window->pollEvent() : std::nullopt; }
    void display() { if (m_window) m_window->display(); }
    void clear(const sf::Color& color = sf::Color(18, 18, 22)) { if (m_window) m_window->clear(color); }

private:
    std::unique_ptr<sf::RenderWindow> m_window;
    int   m_windowWidth;
    int   m_windowHeight;
    float m_cellWidth;
    float m_cellHeight;

    void drawCityBlocks();                              // Building fills between roads
    void drawRoads();                                   // Asphalt + lane markings
    void drawTrafficLights(const Simulation& sim);      // Glowing light circles
    void drawVehicles(const Simulation& sim);           // Coloured, direction-oriented cars

    // Road-center screen position for a vehicle (accounts for ROAD_WIDTH offset)
    sf::Vector2f vehicleScreenPos(float gx, float gy, Direction dir) const;
};

} // namespace TrafficSim

#endif // RENDERER_H
