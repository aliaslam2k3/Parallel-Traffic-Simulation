/**
 * DomainDecomposition.h
 * 1D horizontal strips along y: each MPI rank owns vehicles with y in [yMin, yMax).
 */

#ifndef DOMAINDECOMPOSITION_H
#define DOMAINDECOMPOSITION_H

namespace TrafficSim {

class DomainDecomposition {
public:
    /// Split [0, gridHeight) into `worldSize` horizontal strips.
    DomainDecomposition(int rank, int worldSize, float gridHeight);

    int rank() const { return m_rank; }
    int worldSize() const { return m_worldSize; }

    /// Which rank owns this y coordinate (after toroidal wrap to [0, gridHeight)).
    int ownerRank(float y) const;

    bool containsY(float y) const;

    float yMin() const { return m_yMin; }
    float yMax() const { return m_yMax; }

    /// MPI_COMM_WORLD neighbors; MPI_PROC_NULL (-1) if none.
    int neighborBelow() const { return m_neighborBelow; }
    int neighborAbove() const { return m_neighborAbove; }

private:
    int   m_rank;
    int   m_worldSize;
    float m_gridHeight;
    float m_yMin = 0.f;
    float m_yMax = 0.f;
    int   m_neighborBelow = -1;
    int   m_neighborAbove = -1;
};

} // namespace TrafficSim

#endif
