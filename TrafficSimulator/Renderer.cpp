/**
 * Renderer.cpp
 * Realistic city map: highways, arterials, local streets.
 * U-turn bays, medians, varied intersections.
 * 240×240 grid, 2400×2400 logical size (10 px/cell).
 */

#include "Renderer.h"
#include "CityMap.h"
#include "TrafficLight.h"
#include <cmath>
#include <algorithm>

namespace TrafficSim {

constexpr int LOGICAL_SIZE = 2400;

// ---- Road colours ----
static const sf::Color C_HIGHWAY   (42,  45,  50);   // Darker
static const sf::Color C_ARTERIAL  (50,  54,  58);
static const sf::Color C_LOCAL     (58,  62,  68);
static const sf::Color C_ISECT     (65,  70,  75);
static const sf::Color C_MEDIAN    (220, 200,  60);   // Yellow
static const sf::Color C_CURB      (85,  90,  98);
static const sf::Color C_LANE      (190, 170,  70);   // Yellow dashes
static const sf::Color C_SKY       (20,  22,  26);

// Parks
static const sf::Color C_GRASS     (42,  92,  52);
static const sf::Color C_GRASS_DARK(36,  78,  45);
static const sf::Color C_TREE      (28,  65,  38);

// Buildings
static const sf::Color C_RESIDENTIAL[4] = {
    sf::Color(68,  74,  84), sf::Color(74,  68,  78),
    sf::Color(64,  78,  74), sf::Color(78,  72,  66),
};
static const sf::Color C_DOWNTOWN[2] = {
    sf::Color(40,  44,  52), sf::Color(46,  40,  50),
};
static const sf::Color C_INDUSTRIAL[2] = {
    sf::Color(55,  58,  52), sf::Color(50,  55,  58),
};
static const sf::Color C_WINDOW(90, 100, 115);
static const sf::Color C_DOWNTOWN_WINDOW(62, 70, 85);

// ---------------------------------------------------------------------------

Renderer::Renderer(int w, int h)
    : m_windowWidth(w), m_windowHeight(h),
      m_cellWidth (static_cast<float>(LOGICAL_SIZE) / GRID_WIDTH),
      m_cellHeight(static_cast<float>(LOGICAL_SIZE) / GRID_HEIGHT) {
}

bool Renderer::initialize(bool fullscreen) {
    sf::VideoMode mode;
    auto style = sf::Style::Titlebar | sf::Style::Close;
    auto state = sf::State::Windowed;

    if (fullscreen) {
        mode = sf::VideoMode::getDesktopMode();
        state = sf::State::Fullscreen;
    } else {
        mode = sf::VideoMode(sf::Vector2u(
            static_cast<unsigned>(m_windowWidth),
            static_cast<unsigned>(m_windowHeight)));
    }

    m_window = std::make_unique<sf::RenderWindow>(
        mode, "City Road Map  |  Traffic Simulation", style, state);
    m_window->setFramerateLimit(60);

    sf::View view(sf::Vector2f(LOGICAL_SIZE / 2.0f, LOGICAL_SIZE / 2.0f),
                  sf::Vector2f(static_cast<float>(LOGICAL_SIZE), static_cast<float>(LOGICAL_SIZE)));

    unsigned w = m_window->getSize().x;
    unsigned h = m_window->getSize().y;
    if (w > 0 && h > 0) {
        float aspectWin = static_cast<float>(w) / h;
        float vpW = 1.0f, vpH = 1.0f, vpX = 0.0f, vpY = 0.0f;
        if (aspectWin > 1.0f) {
            vpW = 1.0f / aspectWin;
            vpX = (1.0f - vpW) / 2.0f;
        } else {
            vpH = aspectWin;
            vpY = (1.0f - vpH) / 2.0f;
        }
        view.setViewport(sf::FloatRect(sf::Vector2f(vpX, vpY), sf::Vector2f(vpW, vpH)));
    }
    m_window->setView(view);
    return m_window != nullptr;
}

// ---------------------------------------------------------------------------
// City blocks (parks, downtown, industrial, residential)
// ---------------------------------------------------------------------------
void Renderer::drawCityBlocks() {
    sf::RectangleShape bg(sf::Vector2f(static_cast<float>(LOGICAL_SIZE), static_cast<float>(LOGICAL_SIZE)));
    bg.setFillColor(C_SKY);
    bg.setPosition(sf::Vector2f(0, 0));
    m_window->draw(bg);

    int numRoads = GRID_WIDTH / CityMap::LOCAL_SPACING;
    float prevY = 0;
    int prevRoadWY = 0;
    int by = 0;
    for (int ry = 0; ry < numRoads; ++ry) {
        int y = ry * CityMap::LOCAL_SPACING;
        int roadWY = CityMap::getRoadWidthAt(y);
        float sy = y * m_cellHeight;
        float blockH = (ry == 0) ? 0 : sy - prevY - prevRoadWY * m_cellHeight;
        if (blockH > 2.0f) {
            float prevX = 0;
            int prevRoadWX = 0;
            int bx = 0;
            for (int rx = 0; rx < numRoads; ++rx) {
                int x = rx * CityMap::LOCAL_SPACING;
                int roadWX = CityMap::getRoadWidthAt(x);
                float sx = x * m_cellWidth;
                float blockW = (rx == 0) ? 0 : sx - prevX - prevRoadWX * m_cellWidth;
                if (blockW > 2.0f) {
                    CityMap::BlockType bt = CityMap::getBlockType(bx, by, numRoads - 1, numRoads - 1);
                    float px = prevX + prevRoadWX * m_cellWidth + 1.0f;
                    float py = prevY + prevRoadWY * m_cellHeight + 1.0f;

                    sf::RectangleShape block(sf::Vector2f(blockW - 2.0f, blockH - 2.0f));
                    block.setPosition(sf::Vector2f(px, py));

                    if (bt == CityMap::BlockType::PARK) {
                        block.setFillColor(((bx + by) % 2 == 0) ? C_GRASS : C_GRASS_DARK);
                        m_window->draw(block);
                        const float treeR = std::min(blockW, blockH) * 0.05f;
                        sf::CircleShape tree(treeR);
                        tree.setFillColor(C_TREE);
                        tree.setOrigin(sf::Vector2f(treeR, treeR));
                        for (int t = 0; t < 12; ++t) {
                            float tx = px + blockW * (0.1f + 0.8f * (t % 4) / 3.0f);
                            float ty = py + blockH * (0.1f + 0.8f * (t / 4) / 3.0f);
                            tree.setPosition(sf::Vector2f(tx, ty));
                            m_window->draw(tree);
                        }
                    } else if (bt == CityMap::BlockType::BUILDING_DOWNTOWN) {
                        block.setFillColor(C_DOWNTOWN[(bx + by) % 2]);
                        m_window->draw(block);
                        float winS = std::min(blockW, blockH) * 0.08f;
                        sf::RectangleShape win(sf::Vector2f(winS * 0.8f, winS));
                        win.setFillColor(C_DOWNTOWN_WINDOW);
                        for (int r = 0; r < 5; ++r)
                            for (int c = 0; c < 5; ++c) {
                                win.setPosition(sf::Vector2f(px + blockW * 0.05f + c * winS * 1.2f, py + blockH * 0.05f + r * winS * 1.1f));
                                m_window->draw(win);
                            }
                    } else if (bt == CityMap::BlockType::BUILDING_INDUSTRIAL) {
                        block.setFillColor(C_INDUSTRIAL[(bx + by) % 2]);
                        m_window->draw(block);
                    } else {
                        block.setFillColor(C_RESIDENTIAL[(bx * 2 + by * 3) % 4]);
                        m_window->draw(block);
                        float winS = std::min(blockW, blockH) * 0.1f;
                        sf::RectangleShape win(sf::Vector2f(winS * 0.7f, winS * 0.9f));
                        win.setFillColor(C_WINDOW);
                        for (int r = 0; r < 3; ++r)
                            for (int c = 0; c < 3; ++c) {
                                win.setPosition(sf::Vector2f(px + blockW * 0.15f + c * winS * 1.5f, py + blockH * 0.15f + r * winS * 1.4f));
                                m_window->draw(win);
                            }
                    }
                    ++bx;
                }
                prevX = sx;
                prevRoadWX = roadWX;
            }
            ++by;
        }
        prevY = sy;
        prevRoadWY = roadWY;
    }
}

// ---------------------------------------------------------------------------
// Roads: highways (median), arterials, local. Lane markings. U-turn bays.
// ---------------------------------------------------------------------------
void Renderer::drawRoads() {
    const int numRoads = GRID_WIDTH / CityMap::LOCAL_SPACING;

    // Draw each horizontal road
    for (int r = 0; r < numRoads; ++r) {
        int y = r * CityMap::LOCAL_SPACING;
        int w = CityMap::getRoadWidthAt(y);
        if (w <= 0) continue;
        float roadPx = w * m_cellHeight;
        float sy = y * m_cellHeight;

        sf::Color c = (y % CityMap::HIGHWAY_SPACING == 0) ? C_HIGHWAY :
                      (y % CityMap::ARTERIAL_SPACING == 0) ? C_ARTERIAL : C_LOCAL;

        sf::RectangleShape hRoad(sf::Vector2f(static_cast<float>(LOGICAL_SIZE), roadPx));
        hRoad.setPosition(sf::Vector2f(0, sy));
        hRoad.setFillColor(c);
        m_window->draw(hRoad);

        // Median on highways
        if (y % CityMap::HIGHWAY_SPACING == 0 && w >= 6) {
            float medW = 3.0f;
            sf::RectangleShape med(sf::Vector2f(static_cast<float>(LOGICAL_SIZE), medW));
            med.setPosition(sf::Vector2f(0, sy + roadPx / 2.0f - medW / 2.0f));
            med.setFillColor(C_MEDIAN);
            m_window->draw(med);
        }

        // Yellow centre line (non-highway)
        if (y % CityMap::HIGHWAY_SPACING != 0) {
            float cy = sy + roadPx / 2.0f - 1.0f;
            sf::RectangleShape dash(sf::Vector2f(14.0f, 2.0f));
            dash.setFillColor(C_LANE);
            for (float dx = 0; dx < LOGICAL_SIZE; dx += 24.0f) {
                dash.setPosition(sf::Vector2f(dx, cy));
                m_window->draw(dash);
            }
        }
    }

    // Draw each vertical road
    for (int r = 0; r < numRoads; ++r) {
        int x = r * CityMap::LOCAL_SPACING;
        int w = CityMap::getRoadWidthAt(x);
        if (w <= 0) continue;
        float roadPx = w * m_cellWidth;
        float sx = x * m_cellWidth;

        sf::Color c = (x % CityMap::HIGHWAY_SPACING == 0) ? C_HIGHWAY :
                      (x % CityMap::ARTERIAL_SPACING == 0) ? C_ARTERIAL : C_LOCAL;

        sf::RectangleShape vRoad(sf::Vector2f(roadPx, static_cast<float>(LOGICAL_SIZE)));
        vRoad.setPosition(sf::Vector2f(sx, 0));
        vRoad.setFillColor(c);
        m_window->draw(vRoad);

        if (x % CityMap::HIGHWAY_SPACING == 0 && w >= 6) {
            float medW = 3.0f;
            sf::RectangleShape med(sf::Vector2f(medW, static_cast<float>(LOGICAL_SIZE)));
            med.setPosition(sf::Vector2f(sx + roadPx / 2.0f - medW / 2.0f, 0));
            med.setFillColor(C_MEDIAN);
            m_window->draw(med);
        }

        if (x % CityMap::HIGHWAY_SPACING != 0) {
            float cx = sx + roadPx / 2.0f - 1.0f;
            sf::RectangleShape dash(sf::Vector2f(2.0f, 14.0f));
            dash.setFillColor(C_LANE);
            for (float dy = 0; dy < LOGICAL_SIZE; dy += 24.0f) {
                dash.setPosition(sf::Vector2f(cx, dy));
                m_window->draw(dash);
            }
        }
    }

    // Intersections (darker overlay) + U-turn bays
    for (int ry = 0; ry < numRoads; ++ry) {
        for (int rx = 0; rx < numRoads; ++rx) {
            int gx = rx * CityMap::LOCAL_SPACING;
            int gy = ry * CityMap::LOCAL_SPACING;
            int wx = CityMap::getRoadWidthAt(gx);
            int wy = CityMap::getRoadWidthAt(gy);
            float isectW = wx * m_cellWidth;
            float isectH = wy * m_cellHeight;
            float sx = gx * m_cellWidth;
            float sy = gy * m_cellHeight;

            sf::RectangleShape isect(sf::Vector2f(isectW, isectH));
            isect.setPosition(sf::Vector2f(sx, sy));
            isect.setFillColor(C_ISECT);
            m_window->draw(isect);

            // U-turn bay: small cutout at major intersections
            if (CityMap::hasUturnBay(gx, gy)) {
                const float bayW = isectW * 0.25f;
                const float bayH = isectH * 0.2f;
                sf::RectangleShape bay(sf::Vector2f(bayW, bayH));
                bay.setFillColor(C_ISECT);
                bay.setOutlineColor(C_CURB);
                bay.setOutlineThickness(1.5f);
                // Place in one corner (NE) - U-turn pocket
                bay.setPosition(sf::Vector2f(sx + isectW - bayW - 4.0f, sy + 4.0f));
                m_window->draw(bay);
                // Second bay (SW corner)
                bay.setPosition(sf::Vector2f(sx + 4.0f, sy + isectH - bayH - 4.0f));
                m_window->draw(bay);
            }
        }
    }

    // Curb edges
    sf::RectangleShape curb;
    curb.setFillColor(C_CURB);
    curb.setSize(sf::Vector2f(static_cast<float>(LOGICAL_SIZE), 2.0f));
    for (int r = 0; r < numRoads; ++r) {
        int y = r * CityMap::LOCAL_SPACING;
        int w = CityMap::getRoadWidthAt(y);
        float sy = y * m_cellHeight;
        float roadPx = w * m_cellHeight;
        curb.setPosition(sf::Vector2f(0, sy));
        m_window->draw(curb);
        curb.setPosition(sf::Vector2f(0, sy + roadPx - 2.0f));
        m_window->draw(curb);
    }
    curb.setSize(sf::Vector2f(2.0f, static_cast<float>(LOGICAL_SIZE)));
    for (int r = 0; r < numRoads; ++r) {
        int x = r * CityMap::LOCAL_SPACING;
        int w = CityMap::getRoadWidthAt(x);
        float sx = x * m_cellWidth;
        float roadPx = w * m_cellWidth;
        curb.setPosition(sf::Vector2f(sx, 0));
        m_window->draw(curb);
        curb.setPosition(sf::Vector2f(sx + roadPx - 2.0f, 0));
        m_window->draw(curb);
    }
}

// ---------------------------------------------------------------------------
// Traffic lights
// ---------------------------------------------------------------------------
void Renderer::drawTrafficLights(const Simulation& sim) {
    for (const auto& light : sim.getTrafficLights()) {
        int gx = light->getGridX();
        int gy = light->getGridY();
        int w = std::max(CityMap::getRoadWidthAt(gx), CityMap::getRoadWidthAt(gy));
        float roadPx = w * m_cellWidth;
        float half = roadPx / 2.0f;
        float sx = gx * m_cellWidth + half;
        float sy = gy * m_cellHeight + half;
        float radius = half * 0.5f;

        bool red = (light->getState() == LightState::RED);
        sf::Color lightCol = red ? sf::Color(240, 50, 50) : sf::Color(50, 220, 80);
        sf::Color glowCol  = red ? sf::Color(200, 30, 30, 50) : sf::Color(30, 180, 60, 50);

        sf::CircleShape glow(radius * 1.8f);
        glow.setOrigin(sf::Vector2f(radius * 1.8f, radius * 1.8f));
        glow.setFillColor(glowCol);
        glow.setPosition(sf::Vector2f(sx, sy));
        m_window->draw(glow);

        sf::CircleShape circle(radius);
        circle.setOrigin(sf::Vector2f(radius, radius));
        circle.setFillColor(lightCol);
        circle.setOutlineColor(red ? sf::Color(255, 120, 120) : sf::Color(120, 255, 150));
        circle.setOutlineThickness(2.0f);
        circle.setPosition(sf::Vector2f(sx, sy));
        m_window->draw(circle);
    }
}

void Renderer::drawVehicles(const Simulation& sim) {
    (void)sim;
}

void Renderer::render(const Simulation& simulation) {
    if (!m_window) return;
    drawCityBlocks();
    drawRoads();
    drawTrafficLights(simulation);
    drawVehicles(simulation);
}

} // namespace TrafficSim
