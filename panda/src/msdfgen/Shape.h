
#pragma once

#include <vector>
#include "MSDFContour.h"

/// Vector shape representation.
class Shape {

public:
  /// The list of contours the shape consists of.
  std::vector<MSDFContour> contours;
  /// Specifies whether the shape uses bottom-to-top (false) or top-to-bottom (true) Y coordinates.
  bool inverseYAxis;

  Shape();
  /// Adds a contour.
  void add_contour(const MSDFContour &contour);
#ifdef MSDFGEN_USE_CPP11
  void add_contour(MSDFContour &&contour);
#endif
  /// Adds a blank contour and returns its reference.
  MSDFContour& add_contour();
  /// Normalizes the shape geometry for distance field generation.
  void normalize();
  /// Performs basic checks to determine if the object represents a valid shape.
  bool validate() const;
  /// Computes the shape's bounding box.
  void bounds(PN_stdfloat &l, PN_stdfloat &b, PN_stdfloat &r, PN_stdfloat &t) const;

};