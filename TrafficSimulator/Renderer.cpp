/**
 * Renderer.cpp
 * Anti-aliased window, camera, procedural vehicle textures, crosswalks, minimap.
 */

#include "Renderer.h"
#include "CityMap.h"
#include "TrafficLight.h"
#include "Vehicle.h"
#include <cmath>
#include <algorithm>
#include <cstdio>
#include <cstdint>
#include <string>
#include <vector>

namespace TrafficSim {

constexpr int LOGICAL_SIZE = 2400;

namespace {

void fillEllipse(sf::Image& img, unsigned cx, unsigned cy, unsigned rx, unsigned ry,
                 sf::Color fill, sf::Color border) {
    const unsigned w = img.getSize().x;
    const unsigned h = img.getSize().y;
    for (unsigned y = 0; y < h; ++y) {
        for (unsigned x = 0; x < w; ++x) {
            float dx = (float(x) - float(cx)) / float(rx);
            float dy = (float(y) - float(cy)) / float(ry);
            float d = dx * dx + dy * dy;
            if (d <= 1.f) img.setPixel(sf::Vector2u(x, y), fill);
            else if (d <= 1.08f) img.setPixel(sf::Vector2u(x, y), border);
        }
    }
}

bool buildVehicleTexture(sf::Texture& tex, VehicleType vt, const sf::Color& body) {
    const unsigned W = 72;
    const unsigned H = (vt == VehicleType::Bus) ? 40 : (vt == VehicleType::Bike) ? 28 : 34;
    sf::Image img;
    img.resize(sf::Vector2u(W, H), sf::Color::Transparent);

    sf::Color dark(20, 22, 28);
    sf::Color glass(180, 200, 230, 220);

    if (vt == VehicleType::Bike) {
        fillEllipse(img, W / 2, H / 2, W / 2 - 2, H / 2 - 2, body, dark);
        for (unsigned x = W / 2 - 6; x < W / 2 + 6; ++x)
            if (x < W) img.setPixel(sf::Vector2u(x, 4), glass);
    } else if (vt == VehicleType::Bus) {
        for (unsigned y = 2; y < H - 2; ++y)
            for (unsigned x = 4; x < W - 4; ++x)
                img.setPixel(sf::Vector2u(x, y), body);
        for (unsigned y = 6; y < H - 6; y += 5)
            for (unsigned x = 8; x < W - 8; ++x)
                img.setPixel(sf::Vector2u(x, y), glass);
        for (unsigned x = 2; x < W - 2; ++x) {
            img.setPixel(sf::Vector2u(x, 2), dark);
            img.setPixel(sf::Vector2u(x, H - 3), dark);
        }
    } else {
        fillEllipse(img, W / 2, H / 2, W / 2 - 3, H / 2 - 3, body, dark);
        for (unsigned x = W / 4; x < 3 * W / 4; ++x) {
            img.setPixel(sf::Vector2u(x, 5), glass);
            img.setPixel(sf::Vector2u(x, H - 6), sf::Color(40, 45, 55));
        }
    }
    return tex.loadFromImage(img);
}

static bool tryLoadSystemFont(sf::Font& font) {
    const std::vector<std::string> paths = {
#ifdef _WIN32
        R"(C:\Windows\Fonts\segoeui.ttf)",
        R"(C:\Windows\Fonts\arial.ttf)",
#else
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
#endif
    };
    for (const auto& p : paths) {
        if (font.openFromFile(p)) return true;
    }
    return false;
}

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/// Filled quarter-disk for rounded intersection corners (center, radius, angles in radians).
static void drawCornerFillet(sf::RenderWindow& win, float cx, float cy, float r, float a0, float a1,
                             const sf::Color& fill) {
    if (r < 1.f) return;
    const int n = 12;
    sf::VertexArray fan(sf::PrimitiveType::TriangleFan);
    fan.append(sf::Vertex{sf::Vector2f(cx, cy), fill});
    for (int i = 0; i <= n; ++i) {
        float a = a0 + (a1 - a0) * static_cast<float>(i) / static_cast<float>(n);
        float x = cx + r * std::cos(a);
        float y = cy + r * std::sin(a);
        fan.append(sf::Vertex{sf::Vector2f(x, y), fill});
    }
    win.draw(fan);
}

} // namespace

Renderer::Renderer(int w, int h)
    : m_windowWidth(w), m_windowHeight(h),
      m_cellWidth(static_cast<float>(LOGICAL_SIZE) / GRID_WIDTH),
      m_cellHeight(static_cast<float>(LOGICAL_SIZE) / GRID_HEIGHT),
      m_cameraCenter(LOGICAL_SIZE / 2.f, LOGICAL_SIZE / 2.f) {
}

void Renderer::ensureVehicleTextures() {
    if (m_texturesReady) return;
    sf::Color carBody(100, 140, 200);
    sf::Color busBody(200, 160, 80);
    sf::Color bikeBody(90, 200, 120);
    buildVehicleTexture(m_texCar, VehicleType::Car, carBody);
    buildVehicleTexture(m_texBus, VehicleType::Bus, busBody);
    buildVehicleTexture(m_texBike, VehicleType::Bike, bikeBody);
    m_texturesReady = true;
}

const sf::Texture& Renderer::textureFor(VehicleType t) const {
    switch (t) {
        case VehicleType::Bus:  return m_texBus;
        case VehicleType::Bike: return m_texBike;
        default: return m_texCar;
    }
}

bool Renderer::initialize(bool fullscreen) {
    sf::ContextSettings settings;
    settings.antiAliasingLevel = 8;

    sf::VideoMode mode;
    auto style = sf::Style::Titlebar | sf::Style::Close;
    auto state = sf::State::Windowed;

    if (fullscreen) {
        mode = sf::VideoMode::getDesktopMode();
        state = sf::State::Fullscreen;
    } else {
        mode = sf::VideoMode(sf::Vector2u(static_cast<unsigned>(m_windowWidth),
                                          static_cast<unsigned>(m_windowHeight)));
    }

    m_window = std::make_unique<sf::RenderWindow>(
        mode, "Traffic simulation — enhanced visuals", style, state, settings);
    m_window->setFramerateLimit(120);

    m_fontLoaded = tryLoadSystemFont(m_font);
    ensureVehicleTextures();
    applyWorldView();
    return m_window != nullptr;
}

void Renderer::handleEvent(const sf::Event& ev) {
    if (const auto* key = ev.getIf<sf::Event::KeyPressed>()) {
        const float pan = 120.f / std::max(0.4f, m_zoom);
        switch (key->code) {
            case sf::Keyboard::Key::Equal:
            case sf::Keyboard::Key::Add:
                m_zoom = std::min(4.f, m_zoom * 1.12f);
                break;
            case sf::Keyboard::Key::Hyphen:
            case sf::Keyboard::Key::Subtract:
                m_zoom = std::max(0.35f, m_zoom / 1.12f);
                break;
            case sf::Keyboard::Key::W:
            case sf::Keyboard::Key::Up:
                m_cameraCenter.y -= pan;
                break;
            case sf::Keyboard::Key::S:
            case sf::Keyboard::Key::Down:
                m_cameraCenter.y += pan;
                break;
            case sf::Keyboard::Key::A:
            case sf::Keyboard::Key::Left:
                m_cameraCenter.x -= pan;
                break;
            case sf::Keyboard::Key::D:
            case sf::Keyboard::Key::Right:
                m_cameraCenter.x += pan;
                break;
            case sf::Keyboard::Key::Home:
                m_zoom = 2.25f;
                m_cameraCenter = sf::Vector2f(LOGICAL_SIZE / 2.f, LOGICAL_SIZE / 2.f);
                break;
            default:
                break;
        }
    }
    if (const auto* scroll = ev.getIf<sf::Event::MouseWheelScrolled>()) {
        if (scroll->delta > 0) m_zoom = std::min(4.f, m_zoom * 1.08f);
        else m_zoom = std::max(0.35f, m_zoom / 1.08f);
    }
}

void Renderer::applyWorldView() {
    if (!m_window) return;

    float half = (LOGICAL_SIZE / 2.f) / m_zoom;
    sf::View view(m_cameraCenter, sf::Vector2f(half * 2.f, half * 2.f));

    unsigned w = m_window->getSize().x;
    unsigned h = m_window->getSize().y;
    if (w > 0 && h > 0) {
        float aspectWin = static_cast<float>(w) / h;
        float vpW = 1.f, vpH = 1.f, vpX = 0.f, vpY = 0.f;
        if (aspectWin > 1.f) {
            vpW = 1.f / aspectWin;
            vpX = (1.f - vpW) * 0.5f;
        } else {
            vpH = aspectWin;
            vpY = (1.f - vpH) * 0.5f;
        }
        view.setViewport(sf::FloatRect(sf::Vector2f(vpX, vpY), sf::Vector2f(vpW, vpH)));
    }
    m_window->setView(view);
}

// ---- colours (same as before) ----
static const sf::Color C_HIGHWAY(44, 46, 52);
static const sf::Color C_ARTERIAL(52, 55, 60);
static const sf::Color C_LOCAL(60, 63, 70);
static const sf::Color C_ISECT(68, 72, 78);
static const sf::Color C_MEDIAN(215, 195, 58);
static const sf::Color C_CURB(88, 92, 100);
static const sf::Color C_LANE(195, 175, 72);
static const sf::Color C_SKY(26, 30, 40);
static const sf::Color C_SOLID(220, 220, 230);
static const sf::Color C_GRASS(42, 92, 52);
static const sf::Color C_GRASS_DARK(36, 78, 45);
static const sf::Color C_TREE(28, 65, 38);
static const sf::Color C_RESIDENTIAL[4] = {
    sf::Color(72, 76, 86), sf::Color(78, 72, 80),
    sf::Color(68, 80, 78), sf::Color(82, 76, 70),
};
static const sf::Color C_DOWNTOWN[2] = { sf::Color(44, 48, 56), sf::Color(50, 44, 54) };
static const sf::Color C_INDUSTRIAL[2] = { sf::Color(55, 58, 52), sf::Color(50, 55, 58) };
static const sf::Color C_WINDOW(90, 100, 115);
static const sf::Color C_DOWNTOWN_WINDOW(62, 70, 85);

void Renderer::drawPresentationOverlay(float fps, int vehicleCount, unsigned windowW, unsigned windowH, int mpiWorldSize) {
    if (!m_window || windowW == 0 || windowH == 0) return;

    float barH = std::max(56.f, std::min(88.f, static_cast<float>(windowH) * 0.068f));
    if (mpiWorldSize > 1) barH = std::max(barH, 76.f);
    const float pad = 18.f;

    sf::RectangleShape topBar(sf::Vector2f(static_cast<float>(windowW), barH));
    topBar.setFillColor(sf::Color(12, 14, 18, 235));
    topBar.setOutlineColor(sf::Color(48, 54, 64, 200));
    topBar.setOutlineThickness(1.f);
    m_window->draw(topBar);

    if (!m_fontLoaded) return;

    char buf[160];
    std::snprintf(buf, sizeof(buf),
                  "FPS %.0f   Vehicles %d   Zoom %.2fx   WASD/Arrows pan   +/- zoom   Home reset",
                  static_cast<double>(fps), vehicleCount, static_cast<double>(m_zoom));

    sf::Text title(m_font, "Traffic simulation (enhanced)", static_cast<unsigned>(std::max(17u, windowH / 64u + 14u)));
    title.setFillColor(sf::Color(235, 238, 245));
    title.setPosition(sf::Vector2f(pad, 12.f));
    m_window->draw(title);

    sf::Text stats(m_font, buf, 14u);
    stats.setFillColor(sf::Color(200, 208, 220));
    stats.setPosition(sf::Vector2f(pad, 40.f));
    m_window->draw(stats);

    if (mpiWorldSize > 1) {
        char mpiBuf[128];
        std::snprintf(mpiBuf, sizeof(mpiBuf),
                      "MPI: %d ranks — horizontal y-strips (same map); cyan lines = region boundaries",
                      mpiWorldSize);
        sf::Text mpiLine(m_font, mpiBuf, 13u);
        mpiLine.setFillColor(sf::Color(120, 220, 255));
        mpiLine.setPosition(sf::Vector2f(pad, 58.f));
        m_window->draw(mpiLine);
    }
}

void Renderer::drawMpiRegionBoundaries(int mpiWorldSize) {
    if (mpiWorldSize <= 1 || !m_window) return;
    const float strip = static_cast<float>(GRID_HEIGHT) / static_cast<float>(mpiWorldSize);
    for (int k = 1; k < mpiWorldSize; ++k) {
        const float yGrid = static_cast<float>(k) * strip;
        const float py = yGrid * m_cellHeight;
        sf::RectangleShape line(sf::Vector2f(static_cast<float>(LOGICAL_SIZE), 4.f));
        line.setFillColor(sf::Color(0, 210, 255, 100));
        line.setOutlineColor(sf::Color(0, 240, 255, 160));
        line.setOutlineThickness(1.f);
        line.setPosition(sf::Vector2f(0.f, py));
        m_window->draw(line);
    }
}

void Renderer::drawBlockMassing(float px, float py, float blockW, float blockH, CityMap::BlockType bt, int bx, int by) {
    if (!m_window || blockW < 4.f || blockH < 4.f) return;

    const float innerW = blockW - 2.f;
    const float innerH = blockH - 2.f;
    const float bx0 = px + 1.f;
    const float by0 = py + 1.f;

    if (bt == CityMap::BlockType::PARK) {
        sf::RectangleShape grass(sf::Vector2f(innerW, innerH));
        grass.setPosition(sf::Vector2f(bx0, by0));
        grass.setFillColor(((bx + by) % 2 == 0) ? C_GRASS : C_GRASS_DARK);
        m_window->draw(grass);

        const float treeR = std::min(blockW, blockH) * 0.05f;
        sf::CircleShape tree(treeR);
        tree.setFillColor(C_TREE);
        tree.setOrigin(sf::Vector2f(treeR, treeR));
        for (int t = 0; t < 12; ++t) {
            tree.setPosition(sf::Vector2f(
                bx0 + innerW * (0.1f + 0.8f * static_cast<float>(t % 4) / 3.f),
                by0 + innerH * (0.1f + 0.8f * static_cast<float>(t / 4) / 3.f)));
            m_window->draw(tree);
        }

        const std::uint32_t ph = CityMap::blockHash(bx, by);
        sf::Color pathCol(140, 175, 95, 220);
        sf::VertexArray path1(sf::PrimitiveType::TriangleStrip);
        sf::Vector2f p0(bx0 + innerW * 0.08f, by0 + innerH * 0.55f);
        sf::Vector2f p1(bx0 + innerW * 0.5f, by0 + innerH * 0.12f);
        sf::Vector2f p2(bx0 + innerW * 0.92f, by0 + innerH * 0.48f);
        const int seg = 24;
        const float halfW = 2.2f;
        for (int i = 0; i <= seg; ++i) {
            float t = static_cast<float>(i) / static_cast<float>(seg);
            float u = 1.f - t;
            sf::Vector2f p = p0 * (u * u) + p1 * (2.f * u * t) + p2 * (t * t);
            sf::Vector2f tan = (p1 - p0) * (2.f * u) + (p2 - p1) * (2.f * t);
            float len = std::sqrt(tan.x * tan.x + tan.y * tan.y);
            if (len > 1e-4f) {
                tan.x /= len;
                tan.y /= len;
            }
            sf::Vector2f n(-tan.y, tan.x);
            path1.append(sf::Vertex{p + n * halfW, pathCol});
            path1.append(sf::Vertex{p - n * halfW, pathCol});
        }
        m_window->draw(path1);

        sf::Vector2f q0(bx0 + innerW * 0.15f, by0 + innerH * 0.85f);
        sf::Vector2f q1(bx0 + innerW * 0.42f + static_cast<float>(ph % 17) * 0.3f, by0 + innerH * 0.35f);
        sf::Vector2f q2(bx0 + innerW * 0.88f, by0 + innerH * 0.78f);
        sf::VertexArray path2(sf::PrimitiveType::TriangleStrip);
        for (int i = 0; i <= seg; ++i) {
            float t = static_cast<float>(i) / static_cast<float>(seg);
            float u = 1.f - t;
            sf::Vector2f p = q0 * (u * u) + q1 * (2.f * u * t) + q2 * (t * t);
            sf::Vector2f tan = (q1 - q0) * (2.f * u) + (q2 - q1) * (2.f * t);
            float len = std::sqrt(tan.x * tan.x + tan.y * tan.y);
            if (len > 1e-4f) {
                tan.x /= len;
                tan.y /= len;
            }
            sf::Vector2f n(-tan.y, tan.x);
            path2.append(sf::Vertex{p + n * halfW, pathCol});
            path2.append(sf::Vertex{p - n * halfW, pathCol});
        }
        m_window->draw(path2);
        return;
    }

    std::uint32_t h = CityMap::blockHash(bx, by);
    int nCols = 2 + static_cast<int>(h % 4);
    if (bt == CityMap::BlockType::BUILDING_INDUSTRIAL) nCols = 2 + static_cast<int>((h >> 8) % 2);
    if (bt == CityMap::BlockType::BUILDING_DOWNTOWN) nCols = std::max(3, nCols);
    nCols = std::clamp(nCols, 2, 6);

    const float alley = 3.f;
    float totalAlley = (nCols > 1) ? static_cast<float>(nCols - 1) * alley : 0.f;
    float remW = innerW - totalAlley;
    if (remW < 12.f) nCols = 1;
    totalAlley = (nCols > 1) ? static_cast<float>(nCols - 1) * alley : 0.f;
    remW = innerW - totalAlley;

    if (bt == CityMap::BlockType::BUILDING_INDUSTRIAL && (h % 11) == 0u && innerW > 18.f && innerH > 18.f) {
        const float mw = innerW * 0.58f;
        const float mh = innerH * 0.55f;
        const float wingW = innerW * 0.38f;
        const float wingH = innerH * 0.42f;
        sf::RectangleShape mainR(sf::Vector2f(mw - 1.f, mh - 1.f));
        mainR.setFillColor(C_INDUSTRIAL[(bx + by) % 2]);
        mainR.setPosition(sf::Vector2f(bx0, by0 + innerH - mh));
        m_window->draw(mainR);
        sf::RectangleShape wingR(sf::Vector2f(wingW - 1.f, wingH - 1.f));
        wingR.setFillColor(C_INDUSTRIAL[1 - (bx + by) % 2]);
        wingR.setPosition(sf::Vector2f(bx0 + mw + alley, by0 + innerH - wingH));
        m_window->draw(wingR);
        return;
    }

    std::vector<float> ws(static_cast<size_t>(nCols), 0.f);
    float sum = 0.f;
    for (int i = 0; i < nCols; ++i) {
        std::uint32_t m = CityMap::blockHashMix(h, i);
        ws[static_cast<size_t>(i)] = 0.12f + 0.88f * static_cast<float>(m & 0xFFu) / 255.f;
        sum += ws[static_cast<size_t>(i)];
    }
    for (int i = 0; i < nCols; ++i) ws[static_cast<size_t>(i)] = remW * ws[static_cast<size_t>(i)] / sum;

    float cx = bx0;
    for (int col = 0; col < nCols; ++col) {
        const float w = ws[static_cast<size_t>(col)];
        std::uint32_t h2 = CityMap::blockHashMix(h, col + 17);
        int floors = 2 + static_cast<int>(h2 % 5);
        if (bt == CityMap::BlockType::BUILDING_DOWNTOWN) floors += 2;
        floors = std::clamp(floors, 2, 9);

        float totalFloorsH = innerH * 0.97f;
        float yCursor = by0 + innerH;
        for (int f = 0; f < floors; ++f) {
            float fh = totalFloorsH * (0.14f + 0.11f * static_cast<float>((h2 >> (f * 2)) % 7) / 7.f);
            fh = std::clamp(fh, 4.f, innerH * 0.45f);
            float setback = static_cast<float>(f) * 0.55f;
            float drawW = std::max(3.f, w - 2.f * setback - 1.f);
            yCursor -= fh;
            sf::RectangleShape floorRect(sf::Vector2f(drawW, fh - 0.8f));
            floorRect.setPosition(sf::Vector2f(cx + setback + 0.4f, yCursor));

            if (bt == CityMap::BlockType::BUILDING_DOWNTOWN) {
                floorRect.setFillColor(C_DOWNTOWN[(bx + by + f + col) % 2]);
                m_window->draw(floorRect);
                float winS = std::min(drawW, fh) * 0.18f;
                sf::RectangleShape win(sf::Vector2f(winS * 0.75f, winS * 0.55f));
                win.setFillColor(C_DOWNTOWN_WINDOW);
                int colsW = std::max(2, static_cast<int>(drawW / (winS * 1.15f)));
                for (int r = 0; r < 2; ++r)
                    for (int c = 0; c < colsW; ++c) {
                        win.setPosition(sf::Vector2f(cx + setback + 2.f + static_cast<float>(c) * winS * 1.1f,
                                                     yCursor + fh * 0.2f + static_cast<float>(r) * winS * 1.05f));
                        m_window->draw(win);
                    }
            } else if (bt == CityMap::BlockType::BUILDING_INDUSTRIAL) {
                floorRect.setFillColor(C_INDUSTRIAL[(bx + f + col) % 2]);
                m_window->draw(floorRect);
                sf::RectangleShape band(sf::Vector2f(drawW * 0.92f, std::max(2.f, fh * 0.08f)));
                band.setFillColor(sf::Color(70, 74, 68));
                band.setPosition(sf::Vector2f(cx + setback + drawW * 0.04f, yCursor + fh * 0.45f));
                m_window->draw(band);
            } else {
                floorRect.setFillColor(C_RESIDENTIAL[(bx * 2 + by * 3 + f + col) % 4]);
                m_window->draw(floorRect);
                float winS = std::min(drawW, fh) * 0.22f;
                sf::RectangleShape win(sf::Vector2f(winS * 0.65f, winS * 0.75f));
                win.setFillColor(C_WINDOW);
                for (int r = 0; r < 2; ++r)
                    for (int c = 0; c < 3; ++c) {
                        win.setPosition(sf::Vector2f(cx + setback + drawW * 0.1f + static_cast<float>(c) * winS * 1.4f,
                                                     yCursor + fh * 0.18f + static_cast<float>(r) * winS * 1.35f));
                        m_window->draw(win);
                    }
            }
        }
        cx += w + alley;
    }
}

void Renderer::drawCityBlocks() {
    sf::RectangleShape bg(sf::Vector2f(static_cast<float>(LOGICAL_SIZE), static_cast<float>(LOGICAL_SIZE)));
    bg.setFillColor(C_SKY);
    m_window->draw(bg);

    int numRoads = GRID_WIDTH / CityMap::LOCAL_SPACING;
    float prevY = 0;
    int prevRoadWY = 0;
    int by = 0;
    for (int ry = 0; ry < numRoads; ++ry) {
        int y = ry * CityMap::LOCAL_SPACING;
        int roadWY = CityMap::getRoadWidthAt(y);
        float sy = y * m_cellHeight;
        float blockH = (ry == 0) ? 0.f : sy - prevY - prevRoadWY * m_cellHeight;
        if (blockH > 2.0f) {
            float prevX = 0;
            int prevRoadWX = 0;
            int bx = 0;
            for (int rx = 0; rx < numRoads; ++rx) {
                int x = rx * CityMap::LOCAL_SPACING;
                int roadWX = CityMap::getRoadWidthAt(x);
                float sx = x * m_cellWidth;
                float blockW = (rx == 0) ? 0.f : sx - prevX - prevRoadWX * m_cellWidth;
                if (blockW > 2.0f) {
                    CityMap::BlockType bt = CityMap::getBlockType(bx, by, numRoads - 1, numRoads - 1);
                    float px = prevX + prevRoadWX * m_cellWidth + 1.f;
                    float py = prevY + prevRoadWY * m_cellHeight + 1.f;
                    drawBlockMassing(px, py, blockW, blockH, bt, bx, by);
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

void Renderer::drawRoads() {
    const int numRoads = GRID_WIDTH / CityMap::LOCAL_SPACING;

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

        if (y % CityMap::HIGHWAY_SPACING == 0 && w >= 6) {
            const float my = sy + roadPx * 0.5f;
            sf::VertexArray medV(sf::PrimitiveType::TriangleStrip);
            const float amp = 3.2f;
            for (float dx = 0.f; dx <= static_cast<float>(LOGICAL_SIZE); dx += 10.f) {
                const float oy = amp * std::sin(dx * 0.018f + static_cast<float>(y) * 0.07f);
                medV.append(sf::Vertex{sf::Vector2f(dx, my + oy - 1.6f), C_MEDIAN});
                medV.append(sf::Vertex{sf::Vector2f(dx, my + oy + 1.6f), C_MEDIAN});
            }
            m_window->draw(medV);
        }

        if (y % CityMap::HIGHWAY_SPACING != 0) {
            const float cy = sy + roadPx / 2.f - 1.f;
            sf::RectangleShape dash(sf::Vector2f(14.f, 2.f));
            dash.setFillColor(C_LANE);
            for (float dx = 0.f; dx < static_cast<float>(LOGICAL_SIZE); dx += 24.f) {
                const float oy = 2.4f * std::sin(dx * 0.024f + static_cast<float>(y) * 0.11f);
                dash.setPosition(sf::Vector2f(dx, cy + oy));
                m_window->draw(dash);
            }
        }
        // Solid edge lines
        sf::RectangleShape edge(sf::Vector2f(static_cast<float>(LOGICAL_SIZE), 1.5f));
        edge.setFillColor(C_SOLID);
        edge.setPosition(sf::Vector2f(0, sy + 1.f));
        m_window->draw(edge);
        edge.setPosition(sf::Vector2f(0, sy + roadPx - 2.5f));
        m_window->draw(edge);
    }

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
            const float mx = sx + roadPx * 0.5f;
            sf::VertexArray medV(sf::PrimitiveType::TriangleStrip);
            const float amp = 3.2f;
            for (float dy = 0.f; dy <= static_cast<float>(LOGICAL_SIZE); dy += 10.f) {
                const float ox = amp * std::sin(dy * 0.018f + static_cast<float>(x) * 0.07f);
                medV.append(sf::Vertex{sf::Vector2f(mx + ox - 1.6f, dy), C_MEDIAN});
                medV.append(sf::Vertex{sf::Vector2f(mx + ox + 1.6f, dy), C_MEDIAN});
            }
            m_window->draw(medV);
        }

        if (x % CityMap::HIGHWAY_SPACING != 0) {
            const float cx = sx + roadPx / 2.f - 1.f;
            sf::RectangleShape dash(sf::Vector2f(2.f, 14.f));
            dash.setFillColor(C_LANE);
            for (float dy = 0.f; dy < static_cast<float>(LOGICAL_SIZE); dy += 24.f) {
                const float ox = 2.4f * std::sin(dy * 0.024f + static_cast<float>(x) * 0.11f);
                dash.setPosition(sf::Vector2f(cx + ox, dy));
                m_window->draw(dash);
            }
        }
        sf::RectangleShape edge(sf::Vector2f(1.5f, static_cast<float>(LOGICAL_SIZE)));
        edge.setFillColor(C_SOLID);
        edge.setPosition(sf::Vector2f(sx + 1.f, 0));
        m_window->draw(edge);
        edge.setPosition(sf::Vector2f(sx + roadPx - 2.5f, 0));
        m_window->draw(edge);
    }

    for (int ry = 0; ry < numRoads; ++ry) {
        for (int rx = 0; rx < numRoads; ++rx) {
            int gx = rx * CityMap::LOCAL_SPACING;
            int gy = ry * CityMap::LOCAL_SPACING;
            int wx = CityMap::getRoadWidthAt(gx);
            int wy = CityMap::getRoadWidthAt(gy);
            float iw = wx * m_cellWidth;
            float ih = wy * m_cellHeight;
            float sx = gx * m_cellWidth;
            float sy = gy * m_cellHeight;

            sf::RectangleShape isect(sf::Vector2f(iw, ih));
            isect.setPosition(sf::Vector2f(sx, sy));
            isect.setFillColor(C_ISECT);
            m_window->draw(isect);

            if (CityMap::hasUturnBay(gx, gy)) {
                sf::RectangleShape bay(sf::Vector2f(iw * 0.25f, ih * 0.2f));
                bay.setFillColor(C_ISECT);
                bay.setOutlineColor(C_CURB);
                bay.setOutlineThickness(1.5f);
                bay.setPosition(sf::Vector2f(sx + iw - bay.getSize().x - 4.f, sy + 4.f));
                m_window->draw(bay);
                bay.setPosition(sf::Vector2f(sx + 4.f, sy + ih - bay.getSize().y - 4.f));
                m_window->draw(bay);
            }
        }
    }

    sf::RectangleShape curb;
    curb.setFillColor(C_CURB);
    curb.setSize(sf::Vector2f(static_cast<float>(LOGICAL_SIZE), 2.f));
    for (int r = 0; r < numRoads; ++r) {
        int y = r * CityMap::LOCAL_SPACING;
        int w = CityMap::getRoadWidthAt(y);
        float sy = y * m_cellHeight;
        float roadPx = w * m_cellHeight;
        curb.setPosition(sf::Vector2f(0, sy));
        m_window->draw(curb);
        curb.setPosition(sf::Vector2f(0, sy + roadPx - 2.f));
        m_window->draw(curb);
    }
    curb.setSize(sf::Vector2f(2.f, static_cast<float>(LOGICAL_SIZE)));
    for (int r = 0; r < numRoads; ++r) {
        int x = r * CityMap::LOCAL_SPACING;
        int w = CityMap::getRoadWidthAt(x);
        float sx = x * m_cellWidth;
        float roadPx = w * m_cellWidth;
        curb.setPosition(sf::Vector2f(sx, 0));
        m_window->draw(curb);
        curb.setPosition(sf::Vector2f(sx + roadPx - 2.f, 0));
        m_window->draw(curb);
    }

    drawRoadDecorations();
}

void Renderer::drawRoadDecorations() {
    if (!m_window) return;
    const int numRoads = GRID_WIDTH / CityMap::LOCAL_SPACING;
    for (int ry = 0; ry < numRoads; ++ry) {
        for (int rx = 0; rx < numRoads; ++rx) {
            int gx = rx * CityMap::LOCAL_SPACING;
            int gy = ry * CityMap::LOCAL_SPACING;
            int wx = CityMap::getRoadWidthAt(gx);
            int wy = CityMap::getRoadWidthAt(gy);
            float iw = wx * m_cellWidth;
            float ih = wy * m_cellHeight;
            float sx = gx * m_cellWidth;
            float sy = gy * m_cellHeight;
            float rad = std::clamp(std::min(iw, ih) * 0.14f, 5.f, 18.f);
            sf::Color c(C_CURB.r, C_CURB.g, C_CURB.b, 200);

            drawCornerFillet(*m_window, sx + rad, sy + rad, rad, static_cast<float>(M_PI),
                             static_cast<float>(M_PI) * 1.5f, c);
            drawCornerFillet(*m_window, sx + iw - rad, sy + rad, rad, static_cast<float>(M_PI) * 1.5f,
                             static_cast<float>(M_PI) * 2.f, c);
            drawCornerFillet(*m_window, sx + rad, sy + ih - rad, rad, static_cast<float>(M_PI) * 0.5f,
                             static_cast<float>(M_PI), c);
            drawCornerFillet(*m_window, sx + iw - rad, sy + ih - rad, rad, 0.f,
                             static_cast<float>(M_PI) * 0.5f, c);
        }
    }
}

void Renderer::drawCrosswalks() {
    const int numRoads = GRID_WIDTH / CityMap::LOCAL_SPACING;
    const float stripeW = 4.f;
    const float stripeL = 14.f;
    sf::RectangleShape stripe;

    for (int ry = 0; ry < numRoads; ++ry) {
        for (int rx = 0; rx < numRoads; ++rx) {
            int gx = rx * CityMap::LOCAL_SPACING;
            int gy = ry * CityMap::LOCAL_SPACING;
            int wx = CityMap::getRoadWidthAt(gx);
            int wy = CityMap::getRoadWidthAt(gy);
            float iw = wx * m_cellWidth;
            float ih = wy * m_cellHeight;
            float sx = gx * m_cellWidth;
            float sy = gy * m_cellHeight;

            // Crosswalk strips along each approach (simplified: 4 bars)
            stripe.setFillColor(sf::Color(240, 240, 245, 210));
            for (int i = 0; i < 5; ++i) {
                float t = sx + 6.f + i * (iw - 12.f) / 4.f;
                stripe.setSize(sf::Vector2f(stripeW, stripeL));
                stripe.setPosition(sf::Vector2f(t, sy - stripeL - 2.f));
                m_window->draw(stripe);
                stripe.setPosition(sf::Vector2f(t, sy + ih + 2.f));
                m_window->draw(stripe);
            }
            for (int i = 0; i < 5; ++i) {
                float t = sy + 6.f + i * (ih - 12.f) / 4.f;
                stripe.setSize(sf::Vector2f(stripeL, stripeW));
                stripe.setPosition(sf::Vector2f(sx - stripeL - 2.f, t));
                m_window->draw(stripe);
                stripe.setPosition(sf::Vector2f(sx + iw + 2.f, t));
                m_window->draw(stripe);
            }
        }
    }
}

void Renderer::drawTrafficLights(const Simulation& sim) {
    for (const auto& light : sim.getTrafficLights()) {
        int gx = light->getGridX();
        int gy = light->getGridY();
        int w = std::max(CityMap::getRoadWidthAt(gx), CityMap::getRoadWidthAt(gy));
        float roadPx = w * m_cellWidth;
        float cx = gx * m_cellWidth + roadPx * 0.78f;
        float cy = gy * m_cellHeight + roadPx * 0.22f;

        LightState st = light->getState();
        float r = roadPx * 0.11f;

        auto drawLamp = [&](float dy, bool on, sf::Color col) {
            sf::CircleShape c(r);
            c.setOrigin(sf::Vector2f(r, r));
            c.setPosition(sf::Vector2f(cx, cy + dy));
            c.setFillColor(on ? col : sf::Color(col.r / 3, col.g / 3, col.b / 3));
            c.setOutlineColor(sf::Color(30, 30, 35));
            c.setOutlineThickness(1.5f);
            m_window->draw(c);
        };

        drawLamp(-r * 2.4f, st == LightState::RED, sf::Color(240, 50, 50));
        drawLamp(0.f, st == LightState::YELLOW, sf::Color(240, 220, 60));
        drawLamp(r * 2.4f, st == LightState::GREEN, sf::Color(50, 220, 90));
    }
}

void Renderer::drawVehicleShadows(const Simulation& sim) {
    const float cell = std::min(m_cellWidth, m_cellHeight);
    const float minLogical = 20.f;
    for (const auto& v : sim.getVehicles()) {
        float px = v->getX() * m_cellWidth;
        float py = v->getY() * m_cellHeight;
        float scale = (v->getVehicleType() == VehicleType::Bus) ? 1.35f :
                      (v->getVehicleType() == VehicleType::Bike) ? 0.75f : 1.f;
        const float vis = std::max(cell, minLogical);
        float rw = vis * 1.1f * scale;
        float rh = vis * 0.45f * scale;

        sf::CircleShape sh(std::max(rw, rh) * 0.55f);
        sh.setScale(sf::Vector2f(1.f, 0.45f));
        sh.setFillColor(sf::Color(0, 0, 0, 70));
        sh.setOrigin(sf::Vector2f(sh.getRadius(), sh.getRadius()));
        sh.setPosition(sf::Vector2f(px + 3.f, py + 5.f));
        m_window->draw(sh);
    }
}

void Renderer::drawVehicles(const Simulation& sim) {
    if (!m_window) return;
    ensureVehicleTextures();

    const float cell = std::min(m_cellWidth, m_cellHeight);
    /// Minimum width in logical pixels so cars stay visible at default zoom.
    const float minLogicalW = 22.f;

    for (const auto& v : sim.getVehicles()) {
        float px = v->getX() * m_cellWidth;
        float py = v->getY() * m_cellHeight;

        sf::Sprite spr(textureFor(v->getVehicleType()));
        auto sz = spr.getTexture().getSize();
        spr.setOrigin(sf::Vector2f(static_cast<float>(sz.x) * 0.5f, static_cast<float>(sz.y) * 0.5f));
        float base = (v->getVehicleType() == VehicleType::Bus) ? (cell * 1.25f / static_cast<float>(sz.x)) :
                     (v->getVehicleType() == VehicleType::Bike) ? (cell * 0.9f / static_cast<float>(sz.x)) :
                                                                    (cell * 1.12f / static_cast<float>(sz.x));
        float sc = std::max(base, minLogicalW / static_cast<float>(sz.x));
        spr.setScale(sf::Vector2f(sc, sc));
        spr.setPosition(sf::Vector2f(px, py));
        spr.setRotation(sf::degrees(v->getHeadingDegrees()));
        spr.setColor(sf::Color(v->getR(), v->getG(), v->getB()));

        m_window->draw(spr);

        // Turn indicators (orange corners when blinking phase)
        bool blink = (v->getIndicatorPhase() < 0.45f);
        if (blink && (v->isIndicatorLeft() || v->isIndicatorRight())) {
            const float vis = std::max(cell, minLogicalW * 0.55f);
            float off = vis * 0.55f;
            sf::CircleShape ind(3.f);
            ind.setOrigin(sf::Vector2f(3.f, 3.f));
            ind.setFillColor(sf::Color(255, 180, 40));
            if (v->isIndicatorLeft()) {
                float rad = (v->getHeadingDegrees() - 90.f) * 3.14159265f / 180.f;
                ind.setPosition(sf::Vector2f(px + std::cos(rad) * off, py + std::sin(rad) * off));
                m_window->draw(ind);
            }
            if (v->isIndicatorRight()) {
                float rad = (v->getHeadingDegrees() + 90.f) * 3.14159265f / 180.f;
                ind.setPosition(sf::Vector2f(px + std::cos(rad) * off, py + std::sin(rad) * off));
                m_window->draw(ind);
            }
        }
    }
}

void Renderer::drawMinimap(const Simulation& sim) {
    const float mapSize = 180.f;
    const float pad = 16.f;
    unsigned ww = m_window->getSize().x;
    unsigned wh = m_window->getSize().y;
    if (ww == 0 || wh == 0) return;

    sf::View mini(sf::Vector2f(LOGICAL_SIZE / 2.f, LOGICAL_SIZE / 2.f),
                  sf::Vector2f(static_cast<float>(LOGICAL_SIZE), static_cast<float>(LOGICAL_SIZE)));
    mini.setViewport(sf::FloatRect(
        sf::Vector2f((static_cast<float>(ww) - mapSize - pad) / static_cast<float>(ww),
                     (static_cast<float>(wh) - mapSize - pad) / static_cast<float>(wh)),
        sf::Vector2f(mapSize / static_cast<float>(ww), mapSize / static_cast<float>(wh))));

    sf::View old = m_window->getView();
    m_window->setView(mini);

    sf::RectangleShape bg(sf::Vector2f(static_cast<float>(LOGICAL_SIZE), static_cast<float>(LOGICAL_SIZE)));
    bg.setFillColor(sf::Color(40, 44, 52));
    m_window->draw(bg);

    const int step = CityMap::LOCAL_SPACING * 3;
    sf::RectangleShape road(sf::Vector2f(static_cast<float>(LOGICAL_SIZE), 8.f));
    road.setFillColor(sf::Color(70, 74, 82));
    for (int y = 0; y < GRID_HEIGHT; y += step) {
        road.setPosition(sf::Vector2f(0, y * m_cellHeight));
        m_window->draw(road);
    }
    road.setSize(sf::Vector2f(8.f, static_cast<float>(LOGICAL_SIZE)));
    for (int x = 0; x < GRID_WIDTH; x += step) {
        road.setPosition(sf::Vector2f(x * m_cellWidth, 0));
        m_window->draw(road);
    }

    for (const auto& v : sim.getVehicles()) {
        sf::CircleShape dot(5.f);
        dot.setOrigin(sf::Vector2f(5.f, 5.f));
        dot.setFillColor(sf::Color(v->getR(), v->getG(), v->getB()));
        dot.setPosition(sf::Vector2f(v->getX() * m_cellWidth, v->getY() * m_cellHeight));
        m_window->draw(dot);
    }

    m_window->setView(old);
}

void Renderer::drawDayNightOverlay() {
    sf::RectangleShape overlay(sf::Vector2f(static_cast<float>(LOGICAL_SIZE), static_cast<float>(LOGICAL_SIZE)));
    overlay.setFillColor(sf::Color(20, 28, 55, static_cast<std::uint8_t>(m_dayNight * 120.f)));
    m_window->draw(overlay);
}

void Renderer::render(const Simulation& simulation, float fps, int vehicleCount, int mpiWorldSize) {
    if (!m_window) return;

    applyWorldView();
    drawCityBlocks();
    drawRoads();
    drawCrosswalks();
    drawTrafficLights(simulation);
    drawVehicleShadows(simulation);
    drawVehicles(simulation);
    drawDayNightOverlay();
    if (mpiWorldSize > 1) drawMpiRegionBoundaries(mpiWorldSize);

    const unsigned w = m_window->getSize().x;
    const unsigned h = m_window->getSize().y;
    if (w == 0 || h == 0) return;

    drawMinimap(simulation);

    sf::View hud(sf::Vector2f(static_cast<float>(w) / 2.f, static_cast<float>(h) / 2.f),
                 sf::Vector2f(static_cast<float>(w), static_cast<float>(h)));
    m_window->setView(hud);
    drawPresentationOverlay(fps, vehicleCount, w, h, mpiWorldSize);
}

} // namespace TrafficSim
