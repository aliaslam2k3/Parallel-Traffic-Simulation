/**
 * Renderer.cpp
 * Draws the full city map:
 *   1. Dark sky background
 *   2. Building blocks (coloured city blocks between roads)
 *   3. Road asphalt strips (horizontal + vertical)
 *   4. Intersection areas (slightly lighter asphalt)
 *   5. Yellow dashed lane-centre markings
 *   6. White curb edge lines
 *   7. Traffic lights (glowing coloured circles at intersection centres)
 *   8. Vehicles (coloured, direction-oriented rectangles on roads)
 *
 * Grid: 80×80 cells, 12 px/cell → 960×960 window
 *   Road strip   = ROAD_WIDTH (3) cells = 36 px wide
 *   Building block = 7 cells = 84 px per side
 */

#include "Renderer.h"
#include "TrafficLight.h"
#include <cmath>

namespace TrafficSim {

// Colours
static const sf::Color C_SKY        (18,  18,  22);
static const sf::Color C_ASPHALT    (58,  62,  68);
static const sf::Color C_ISECT      (72,  76,  82);
static const sf::Color C_CURB       (95,  98, 105);
static const sf::Color C_LANE       (190, 165,  70);   // yellow dashes

// Six distinct building-block tones (subtle, urban palette)
static const sf::Color BLOCK_COLORS[6] = {
    sf::Color(38, 44, 54),
    sf::Color(44, 50, 40),
    sf::Color(46, 40, 50),
    sf::Color(40, 52, 46),
    sf::Color(50, 42, 38),
    sf::Color(34, 46, 52),
};

// Window markings (small lighter rectangles) on buildings
static const sf::Color C_WINDOW(62, 68, 80);

// ---------------------------------------------------------------------------

Renderer::Renderer(int w, int h)
    : m_windowWidth(w), m_windowHeight(h),
      m_cellWidth (static_cast<float>(w) / GRID_WIDTH),
      m_cellHeight(static_cast<float>(h) / GRID_HEIGHT) {
}

bool Renderer::initialize() {
    m_window = std::make_unique<sf::RenderWindow>(
        sf::VideoMode(sf::Vector2u(
            static_cast<unsigned>(m_windowWidth),
            static_cast<unsigned>(m_windowHeight))),
        "Parallel Traffic Simulation Engine  |  close to exit",
        sf::Style::Titlebar | sf::Style::Close
    );
    m_window->setFramerateLimit(60);
    return m_window != nullptr;
}

// ---------------------------------------------------------------------------
// Coordinate helper
// ---------------------------------------------------------------------------
sf::Vector2f Renderer::vehicleScreenPos(float gx, float gy, Direction dir) const {
    const float half = ROAD_WIDTH / 2.0f;   // 1.5 cells – road-strip centre offset
    float sx, sy;
    if (dir == Direction::EAST || dir == Direction::WEST) {
        // Free coordinate is x; locked coordinate is y (road row)
        sx = (gx + 0.5f) * m_cellWidth;
        sy = (gy + half)  * m_cellHeight;
    } else {
        // Free coordinate is y; locked coordinate is x (road column)
        sx = (gx + half)  * m_cellWidth;
        sy = (gy + 0.5f) * m_cellHeight;
    }
    return { sx, sy };
}

// ---------------------------------------------------------------------------
// 1 + 2. Sky background + building city blocks
// ---------------------------------------------------------------------------
void Renderer::drawCityBlocks() {
    // Full background
    sf::RectangleShape bg(sf::Vector2f(
        static_cast<float>(m_windowWidth),
        static_cast<float>(m_windowHeight)));
    bg.setFillColor(C_SKY);
    bg.setPosition(sf::Vector2f(0, 0));
    m_window->draw(bg);

    const int   numRoads   = GRID_WIDTH  / INTERSECTION_SPACING;
    const float blockCells = static_cast<float>(INTERSECTION_SPACING - ROAD_WIDTH); // 7 cells
    const float blockPx    = blockCells * m_cellWidth;    // 84 px

    sf::RectangleShape block(sf::Vector2f(blockPx - 2.0f, blockPx - 2.0f));

    for (int bx = 0; bx < numRoads; ++bx) {
        for (int by = 0; by < numRoads; ++by) {
            float sx = (bx * INTERSECTION_SPACING + ROAD_WIDTH) * m_cellWidth  + 1.0f;
            float sy = (by * INTERSECTION_SPACING + ROAD_WIDTH) * m_cellHeight + 1.0f;

            block.setPosition(sf::Vector2f(sx, sy));
            int ci = (bx * 3 + by * 7) % 6;
            block.setFillColor(BLOCK_COLORS[ci]);
            m_window->draw(block);

            // Simple "window" details – a 3×3 grid of small lit squares
            sf::RectangleShape win(sf::Vector2f(6.0f, 5.0f));
            win.setFillColor(C_WINDOW);
            float wx0 = sx + 10.0f;
            float wy0 = sy + 10.0f;
            for (int row = 0; row < 3; ++row) {
                for (int col = 0; col < 3; ++col) {
                    win.setPosition(sf::Vector2f(wx0 + col * 16.0f, wy0 + row * 14.0f));
                    m_window->draw(win);
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// 3–6. Road surfaces, intersections, lane markings, curbs
// ---------------------------------------------------------------------------
void Renderer::drawRoads() {
    const int   numRoads = GRID_WIDTH / INTERSECTION_SPACING;
    const float roadPx   = ROAD_WIDTH * m_cellWidth;   // 36 px

    // --- Horizontal road strips ---
    sf::RectangleShape hRoad(sf::Vector2f(
        static_cast<float>(m_windowWidth), roadPx));
    hRoad.setFillColor(C_ASPHALT);
    for (int r = 0; r < numRoads; ++r) {
        hRoad.setPosition(sf::Vector2f(0, r * INTERSECTION_SPACING * m_cellHeight));
        m_window->draw(hRoad);
    }

    // --- Vertical road strips ---
    sf::RectangleShape vRoad(sf::Vector2f(
        roadPx, static_cast<float>(m_windowHeight)));
    vRoad.setFillColor(C_ASPHALT);
    for (int r = 0; r < numRoads; ++r) {
        vRoad.setPosition(sf::Vector2f(r * INTERSECTION_SPACING * m_cellWidth, 0));
        m_window->draw(vRoad);
    }

    // --- Intersection areas (slightly lighter) ---
    sf::RectangleShape isect(sf::Vector2f(roadPx, roadPx));
    isect.setFillColor(C_ISECT);
    for (int r = 0; r < numRoads; ++r) {
        for (int c = 0; c < numRoads; ++c) {
            isect.setPosition(sf::Vector2f(
                c * INTERSECTION_SPACING * m_cellWidth,
                r * INTERSECTION_SPACING * m_cellHeight));
            m_window->draw(isect);
        }
    }

    // --- Yellow dashed centre lines ---
    sf::RectangleShape dash;
    dash.setFillColor(C_LANE);

    const float halfRoad  = roadPx / 2.0f;
    const float dashLen   = 10.0f;
    const float dashGap   = 9.0f;

    // Horizontal road centre lines
    dash.setSize(sf::Vector2f(dashLen, 2.0f));
    for (int r = 0; r < numRoads; ++r) {
        float cy = r * INTERSECTION_SPACING * m_cellHeight + halfRoad - 1.0f;
        for (float dx = 0; dx < m_windowWidth; dx += dashLen + dashGap) {
            dash.setPosition(sf::Vector2f(dx, cy));
            m_window->draw(dash);
        }
    }

    // Vertical road centre lines
    dash.setSize(sf::Vector2f(2.0f, dashLen));
    for (int r = 0; r < numRoads; ++r) {
        float cx = r * INTERSECTION_SPACING * m_cellWidth + halfRoad - 1.0f;
        for (float dy = 0; dy < m_windowHeight; dy += dashLen + dashGap) {
            dash.setPosition(sf::Vector2f(cx, dy));
            m_window->draw(dash);
        }
    }

    // --- White curb edge lines ---
    sf::RectangleShape curb;
    curb.setFillColor(C_CURB);

    // Horizontal road edges
    curb.setSize(sf::Vector2f(static_cast<float>(m_windowWidth), 2.0f));
    for (int r = 0; r < numRoads; ++r) {
        float ry = r * INTERSECTION_SPACING * m_cellHeight;
        curb.setPosition(sf::Vector2f(0, ry));                   m_window->draw(curb);
        curb.setPosition(sf::Vector2f(0, ry + roadPx - 2.0f));   m_window->draw(curb);
    }

    // Vertical road edges
    curb.setSize(sf::Vector2f(2.0f, static_cast<float>(m_windowHeight)));
    for (int r = 0; r < numRoads; ++r) {
        float rx = r * INTERSECTION_SPACING * m_cellWidth;
        curb.setPosition(sf::Vector2f(rx, 0));                   m_window->draw(curb);
        curb.setPosition(sf::Vector2f(rx + roadPx - 2.0f, 0));  m_window->draw(curb);
    }
}

// ---------------------------------------------------------------------------
// 7. Traffic lights – glowing circle at intersection centre
// ---------------------------------------------------------------------------
void Renderer::drawTrafficLights(const Simulation& sim) {
    const float roadPx = ROAD_WIDTH * m_cellWidth;
    const float half   = roadPx / 2.0f;
    const float radius = half * 0.70f;   // ~12.6 px

    sf::CircleShape glow(radius * 2.2f);
    glow.setOrigin(sf::Vector2f(radius * 2.2f, radius * 2.2f));

    sf::CircleShape circle(radius);
    circle.setOrigin(sf::Vector2f(radius, radius));
    circle.setOutlineThickness(2.5f);

    for (const auto& light : sim.getTrafficLights()) {
        float sx = light->getGridX() * m_cellWidth  + half;
        float sy = light->getGridY() * m_cellHeight + half;
        bool  red = (light->getState() == LightState::RED);

        sf::Color lightCol = red ? sf::Color(240,  50,  50)  : sf::Color( 50, 220,  80);
        sf::Color glowCol  = red ? sf::Color(200,  30,  30, 55) : sf::Color( 30, 180,  60, 55);
        sf::Color rimCol   = red ? sf::Color(255, 120, 120) : sf::Color(120, 255, 150);

        glow.setFillColor(glowCol);
        glow.setPosition(sf::Vector2f(sx, sy));
        m_window->draw(glow);

        circle.setFillColor(lightCol);
        circle.setOutlineColor(rimCol);
        circle.setPosition(sf::Vector2f(sx, sy));
        m_window->draw(circle);
    }
}

// ---------------------------------------------------------------------------
// 8. Vehicles – colour-coded, direction-oriented rectangles
// ---------------------------------------------------------------------------
void Renderer::drawVehicles(const Simulation& sim) {
    const float roadPx  = ROAD_WIDTH * m_cellWidth;
    const float vLen    = roadPx * 0.82f;   // along direction of travel
    const float vWide   = roadPx * 0.46f;   // perpendicular

    sf::RectangleShape body(sf::Vector2f(vLen, vWide));
    body.setOrigin(sf::Vector2f(vLen / 2.0f, vWide / 2.0f));
    body.setOutlineThickness(1.5f);

    for (const auto& v : sim.getVehicles()) {
        Direction dir = v->getDirection();

        sf::Vector2f pos = vehicleScreenPos(v->getX(), v->getY(), dir);
        body.setPosition(pos);

        sf::Color fill(v->getR(), v->getG(), v->getB());
        sf::Color rim (v->getR() / 3, v->getG() / 3, v->getB() / 3);
        body.setFillColor(fill);
        body.setOutlineColor(rim);

        float rot = 0.0f;
        switch (dir) {
            case Direction::NORTH: rot = 270.0f; break;
            case Direction::SOUTH: rot =  90.0f; break;
            case Direction::WEST:  rot = 180.0f; break;
            default: rot = 0.0f; break;
        }
        body.setRotation(sf::degrees(rot));

        m_window->draw(body);

        // Small white "windshield" rectangle at the front of the car
        const float wsLen = vWide * 0.55f;
        const float wsW   = vWide * 0.30f;
        sf::RectangleShape ws(sf::Vector2f(wsLen, wsW));
        ws.setOrigin(sf::Vector2f(wsLen / 2.0f, wsW / 2.0f));
        ws.setFillColor(sf::Color(220, 235, 255, 200));
        // Offset windshield toward front of car
        float fwdX = 0.0f, fwdY = 0.0f;
        switch (dir) {
            case Direction::EAST:  fwdX =  vLen * 0.30f; break;
            case Direction::WEST:  fwdX = -vLen * 0.30f; break;
            case Direction::SOUTH: fwdY =  vLen * 0.30f; break;
            case Direction::NORTH: fwdY = -vLen * 0.30f; break;
        }
        ws.setPosition(sf::Vector2f(pos.x + fwdX, pos.y + fwdY));
        ws.setRotation(sf::degrees(rot));
        m_window->draw(ws);
    }
}

// ---------------------------------------------------------------------------
// Main render entry point
// ---------------------------------------------------------------------------
void Renderer::render(const Simulation& simulation) {
    if (!m_window) return;

    drawCityBlocks();
    drawRoads();
    drawTrafficLights(simulation);
    drawVehicles(simulation);
}

} // namespace TrafficSim
