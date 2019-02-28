
#pragma once

#include "edge-segments.h"

/// Container for a single edge of dynamic type.
class EdgeHolder {

public:
    EdgeHolder();
    EdgeHolder(EdgeSegment *segment);
    EdgeHolder(LPoint2 p0, LPoint2 p1, EdgeColor edge_color = WHITE);
    EdgeHolder(LPoint2 p0, LPoint2 p1, LPoint2 p2, EdgeColor edge_color = WHITE);
    EdgeHolder(LPoint2 p0, LPoint2 p1, LPoint2 p2, LPoint2 p3, EdgeColor edge_color = WHITE);
    EdgeHolder(const EdgeHolder &orig);
#ifdef MSDFGEN_USE_CPP11
    EdgeHolder(EdgeHolder &&orig);
#endif
    ~EdgeHolder();
    EdgeHolder & operator=(const EdgeHolder &orig);
#ifdef MSDFGEN_USE_CPP11
    EdgeHolder & operator=(EdgeHolder &&orig);
#endif
    EdgeSegment & operator*();
    const EdgeSegment & operator*() const;
    EdgeSegment * operator->();
    const EdgeSegment * operator->() const;
    operator EdgeSegment *();
    operator const EdgeSegment *() const;

private:
    EdgeSegment *edge_segment;

};