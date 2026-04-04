/**
 * Renderer.h
 * World rendering: roads, crosswalks, traffic lights (R/Y/G), textured vehicles, shadows.
 * Camera zoom/pan, minimap, HUD, optional day/night tint.
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

    bool initialize(bool fullscreen = false);

    /// Process keyboard/mouse for camera (call each frame after pollEvent queue)
    void handleEvent(const sf::Event& event);

    /// If `mpiWorldSize` > 1, draws horizontal strip boundaries (y-decomposition) and HUD hint.
    void render(const Simulation& simulation, float fps, int vehicleCount, int mpiWorldSize = 0);

    bool isOpen() const { return m_window && m_window->isOpen(); }
    std::optional<sf::Event> pollEvent() { return m_window ? m_window->pollEvent() : std::nullopt; }
    void display() { if (m_window) m_window->display(); }
    void clear(const sf::Color& color = sf::Color(22, 24, 28)) { if (m_window) m_window->clear(color); }

private:
    void applyWorldView();
    void drawPresentationOverlay(float fps, int vehicleCount, unsigned windowW, unsigned windowH, int mpiWorldSize);
    void drawMpiRegionBoundaries(int mpiWorldSize);
    void drawCityBlocks();
    void drawBlockMassing(float px, float py, float blockW, float blockH, CityMap::BlockType bt, int bx, int by);
    void drawRoads();
    void drawRoadDecorations();
    void drawCrosswalks();
    void drawTrafficLights(const Simulation& sim);
    void drawVehicles(const Simulation& sim);
    void drawVehicleShadows(const Simulation& sim);
    void drawMinimap(const Simulation& sim);
    void drawDayNightOverlay();

    void ensureVehicleTextures();
    const sf::Texture& textureFor(VehicleType t) const;

    std::unique_ptr<sf::RenderWindow> m_window;
    int   m_windowWidth;
    int   m_windowHeight;
    float m_cellWidth;
    float m_cellHeight;

    sf::Font m_font;
    bool     m_fontLoaded = false;

    // Camera (world space = logical pixels)
    sf::Vector2f m_cameraCenter;
    float        m_zoom = 2.25f;

    // Procedural vehicle textures
    sf::Texture m_texCar;
    sf::Texture m_texBus;
    sf::Texture m_texBike;
    bool        m_texturesReady = false;

    /// 0 = full day, 1 = night (visual tint)
    float m_dayNight = 0.04f;
};

} // namespace TrafficSim

#endif // RENDERER_H
