
#include "MSDFContour.h"

#include "arithmetics.hpp"

static PN_stdfloat shoelace(const LPoint2 &a, const LPoint2 &b) {
  return (b[0] - a[0]) * (a[1] + b[1]);
}

void MSDFContour::
add_edge(const EdgeHolder &edge) {
  edges.push_back(edge);
}

#ifdef MSDFGEN_USE_CPP11
void MSDFContour::
add_edge(EdgeHolder &&edge) {
  edges.push_back((EdgeHolder &&) edge);
}
#endif

EdgeHolder &MSDFContour::
add_edge() {
  edges.resize(edges.size() + 1);
  return edges[edges.size() - 1];
}

void MSDFContour::
bounds(PN_stdfloat &l, PN_stdfloat &b, PN_stdfloat &r, PN_stdfloat &t) const {
  for (const EdgeHolder &edge : edges) {
    edge->bounds(l, b, r, t);
  }
}

int MSDFContour::
winding() const {
  if (edges.empty()) {
    return 0;
  }
  PN_stdfloat total = 0;
  if (edges.size() == 1) {
    LPoint2 a = edges[0]->point(0);
    LPoint2 b = edges[0]->point(1 / 3.0);
    LPoint2 c = edges[0]->point(2 / 3.0);
    total += shoelace(a, b);
    total += shoelace(b, c);
    total += shoelace(c, a);
  }
  else if (edges.size() == 2) {
    LPoint2 a = edges[0]->point(0);
    LPoint2 b = edges[0]->point(0.5);
    LPoint2 c = edges[1]->point(0);
    LPoint2 d = edges[1]->point(0.5);
    total += shoelace(a, b);
    total += shoelace(b, c);
    total += shoelace(c, d);
    total += shoelace(d, a);
  }
  else {
    LPoint2 prev = edges[edges.size()-1]->point(0);
    std::vector<EdgeHolder>::const_iterator edge;
    for (const EdgeHolder &edge : edges) {
      LPoint2 cur = edge->point(0);
      total += shoelace(prev, cur);
      prev = cur;
    }
  }
  return msdf::sign(total);
}