/**
 * CityMap.h
 * Realistic city road network: highways, arterials, local streets.
 * U-turn bays at major intersections, varied block types.
 */

#ifndef CITYMAP_H
#define CITYMAP_H

#include <cstdint>

namespace TrafficSim {

namespace CityMap {

// Road hierarchy
enum class RoadType {
    HIGHWAY,   // 8 cells, median, U-turns
    ARTERIAL,  // 5 cells, U-turns at crossings
    LOCAL      // 3 cells
};

// Block land use
enum class BlockType {
    BUILDING_RESIDENTIAL,
    BUILDING_DOWNTOWN,
    BUILDING_INDUSTRIAL,
    PARK
};

// Grid: 240×240
constexpr int GRID_SIZE = 240;

// Road spacing and widths
constexpr int HIGHWAY_SPACING  = 60;   // Every 60 cells
constexpr int HIGHWAY_WIDTH    = 8;
constexpr int ARTERIAL_SPACING = 24;   // Every 24 cells
constexpr int ARTERIAL_WIDTH   = 5;
constexpr int LOCAL_SPACING    = 12;   // Every 12 cells
constexpr int LOCAL_WIDTH      = 3;

// Returns the road type and width at grid position (for horizontal road at y, or vertical at x)
inline RoadType getRoadTypeAt(int pos) {
    if (pos % HIGHWAY_SPACING == 0) return RoadType::HIGHWAY;
    if (pos % ARTERIAL_SPACING == 0) return RoadType::ARTERIAL;
    if (pos % LOCAL_SPACING == 0) return RoadType::LOCAL;
    return RoadType::LOCAL;  // fallback
}

inline int getRoadWidthAt(int pos) {
    if (pos % HIGHWAY_SPACING == 0) return HIGHWAY_WIDTH;
    if (pos % ARTERIAL_SPACING == 0) return ARTERIAL_WIDTH;
    if (pos % LOCAL_SPACING == 0) return LOCAL_WIDTH;
    return 0;
}

// True if this is a road position (horizontal or vertical)
inline bool isRoadPosition(int pos) {
    return pos % LOCAL_SPACING == 0;
}

// True if intersection has U-turn bay (major crossings only)
inline bool hasUturnBay(int gx, int gy) {
    return (gx % ARTERIAL_SPACING == 0 || gx % HIGHWAY_SPACING == 0) &&
           (gy % ARTERIAL_SPACING == 0 || gy % HIGHWAY_SPACING == 0);
}

// Block type for a building block (between roads)
inline BlockType getBlockType(int bx, int by, int numBlocksX, int numBlocksY) {
    int cx = numBlocksX / 2, cy = numBlocksY / 2;
    int dx = (bx > cx) ? (bx - cx) : (cx - bx);
    int dy = (by > cy) ? (by - cy) : (cy - by);

    // Central park
    if (dx <= 2 && dy <= 2) return BlockType::PARK;

    // Downtown ring
    if (dx <= 4 && dy <= 4 && (dx >= 3 || dy >= 3)) return BlockType::BUILDING_DOWNTOWN;

    // Industrial near edges
    if (bx < 2 || bx >= numBlocksX - 2 || by < 2 || by >= numBlocksY - 2)
        return BlockType::BUILDING_INDUSTRIAL;

    return BlockType::BUILDING_RESIDENTIAL;
}

/// Stable hash for block (bx, by) — deterministic façade / tower layout in renderer.
inline std::uint32_t blockHash(int bx, int by) {
    std::uint32_t h = static_cast<std::uint32_t>(bx) * 2654435761u ^ static_cast<std::uint32_t>(by) * 2246822519u;
    h ^= h >> 16;
    h *= 0x85ebca6bu;
    h ^= h >> 13;
    h *= 0xc2b2ae35u;
    h ^= h >> 16;
    return h;
}

/// Extra entropy for sub-building index / floor.
inline std::uint32_t blockHashMix(std::uint32_t h, int salt) {
    std::uint32_t x = h ^ static_cast<std::uint32_t>(salt) * 0x9E3779B9u;
    x ^= x >> 16;
    x *= 0x7feb352du;
    x ^= x >> 15;
    return x;
}

} // namespace CityMap
} // namespace TrafficSim

#endif // CITYMAP_H
