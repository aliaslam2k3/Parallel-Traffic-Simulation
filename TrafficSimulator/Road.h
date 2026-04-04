/**
 * Road.h
 * Road segment metadata (grid axis and index). Kept minimal for future pathfinding.
 */

#ifndef ROAD_H
#define ROAD_H

namespace TrafficSim {

enum class RoadAxis { Horizontal, Vertical };

struct RoadSegment {
    RoadAxis axis = RoadAxis::Horizontal;
    int      gridLine = 0; ///< y for horizontal, x for vertical
};

} // namespace TrafficSim

#endif
