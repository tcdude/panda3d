
#pragma once

#include <vector>
#include "EdgeHolder.h"

/// A single closed contour of a shape.
class MSDFContour {

public:
  /// The sequence of edges that make up the contour.
  std::vector<EdgeHolder> edges;

  /// Adds an edge to the contour.
  void add_edge(const EdgeHolder &edge);
#ifdef MSDFGEN_USE_CPP11
  void add_edge(EdgeHolder &&edge);
#endif
  /// Creates a new edge in the contour and returns its reference.
  EdgeHolder & add_edge();
  /// Computes the bounding box of the contour.
  void bounds(PN_stdfloat &l, PN_stdfloat &b, PN_stdfloat &r, PN_stdfloat &t) const;
  /// Computes the winding of the contour. Returns 1 if positive, -1 if negative.
  int winding() const;

};