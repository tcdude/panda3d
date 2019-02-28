
#include "EdgeHolder.h"

EdgeHolder::
EdgeHolder() : edge_segment(NULL) { }

EdgeHolder::
EdgeHolder(EdgeSegment *segment) : edge_segment(segment) { }

EdgeHolder::
EdgeHolder(LPoint2 p0, LPoint2 p1, EdgeColor edge_color) :
  edge_segment(new LinearSegment(p0, p1, edge_color)) { }

EdgeHolder::
EdgeHolder(LPoint2 p0, LPoint2 p1, LPoint2 p2, EdgeColor edge_color) :
  edge_segment(new QuadraticSegment(p0, p1, p2, edge_color)) { }

EdgeHolder::
EdgeHolder(LPoint2 p0, LPoint2 p1, LPoint2 p2, LPoint2 p3, EdgeColor edge_color) :
  edge_segment(new CubicSegment(p0, p1, p2, p3, edge_color)) { }

EdgeHolder::
EdgeHolder(const EdgeHolder &orig) :
  edge_segment(orig.edge_segment ? orig.edge_segment->clone() : NULL) { }

#ifdef MSDFGEN_USE_CPP11
EdgeHolder::
EdgeHolder(EdgeHolder &&orig) : edge_segment(orig.edge_segment) {
  orig.edge_segment = NULL;
}
#endif

EdgeHolder::
~EdgeHolder() {
  delete edge_segment;
}

EdgeHolder &EdgeHolder::
operator=(const EdgeHolder &orig) {
  delete edge_segment;
  edge_segment = orig.edge_segment ? orig.edge_segment->clone() : NULL;
  return *this;
}

#ifdef MSDFGEN_USE_CPP11
EdgeHolder &EdgeHolder::
operator=(EdgeHolder &&orig) {
  delete edge_segment;
  edge_segment = orig.edge_segment;
  orig.edge_segment = NULL;
  return *this;
}
#endif

EdgeSegment &EdgeHolder::
operator*() {
  return *edge_segment;
}

const EdgeSegment &EdgeHolder::
operator*() const {
  return *edge_segment;
}

EdgeSegment *EdgeHolder::
operator->() {
  return edge_segment;
}

const EdgeSegment *EdgeHolder::
operator->() const {
  return edge_segment;
}

EdgeHolder::
operator EdgeSegment *() {
  return edge_segment;
}

EdgeHolder::
operator const EdgeSegment *() const {
  return edge_segment;
}