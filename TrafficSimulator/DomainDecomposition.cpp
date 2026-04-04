/**
 * DomainDecomposition.cpp
 */

#include "DomainDecomposition.h"
#include <algorithm>
#include <cmath>

namespace TrafficSim {

DomainDecomposition::DomainDecomposition(int rank, int worldSize, float gridHeight)
    : m_rank(rank), m_worldSize(worldSize), m_gridHeight(gridHeight) {
    if (worldSize <= 0) worldSize = 1;
    if (rank < 0) rank = 0;
    if (rank >= worldSize) rank = worldSize - 1;

    const float strip = gridHeight / static_cast<float>(worldSize);
    m_yMin = static_cast<float>(rank) * strip;
    m_yMax = (rank == worldSize - 1) ? gridHeight : static_cast<float>(rank + 1) * strip;

    m_neighborBelow = (rank > 0) ? rank - 1 : -1;
    m_neighborAbove = (rank < worldSize - 1) ? rank + 1 : -1;
}

int DomainDecomposition::ownerRank(float y) const {
    float yy = y;
    if (yy < 0.f) yy += m_gridHeight;
    else if (yy >= m_gridHeight) yy -= m_gridHeight;
    yy = std::clamp(yy, 0.f, m_gridHeight - 1e-5f);
    const float strip = m_gridHeight / static_cast<float>(m_worldSize);
    int r = static_cast<int>(std::floor(yy / strip));
    if (r >= m_worldSize) r = m_worldSize - 1;
    if (r < 0) r = 0;
    return r;
}

bool DomainDecomposition::containsY(float y) const {
    float yy = y;
    if (yy < 0.f) yy += m_gridHeight;
    else if (yy >= m_gridHeight) yy -= m_gridHeight;
    if (m_rank == m_worldSize - 1)
        return yy >= m_yMin && yy <= m_yMax + 1e-4f;
    return yy >= m_yMin && yy < m_yMax;
}

} // namespace TrafficSim
