/**
 * PANDA 3D SOFTWARE
 * Copyright (c) Carnegie Mellon University.  All rights reserved.
 *
 * All use of this software is subject to the terms of the revised BSD
 * license.  You should have received a copy of this license along
 * with this source code in a file named "LICENSE."
 *
 * @file msdfgen.I
 * @author tcdude
 * @date 2019-03-01
 */

/**
 * Assures a non zero vector.
 */
INLINE LVector2 normalize_non_zero(const LVector2 &v) {
  LVector2 n = v;
  if (!n.normalize()){
    n[1] = 1;
  }
  return n;
}

/**
 * Returns the middle out of three values.
 */
INLINE PN_stdfloat median(PN_stdfloat a, PN_stdfloat b, PN_stdfloat c) {
  return std::max(std::min(a, b), std::min(std::max(a, b), c));
}

/**
 * Returns the weighted average of a and b.
 */
INLINE LPoint2 mix (const LPoint2 &a, const LPoint2 &b, PN_stdfloat weight) {
  return (PN_stdfloat(1) - weight) * a + weight * b;
}

/**
 * Returns 1 for non-negative values and -1 for negative values.
 */
INLINE int non_zero_sign(PN_stdfloat n) {
return 2 * (n > 0) - 1;
}

/**
 *
 */
INLINE bool is_corner(const LVector2 &a_dir, const LVector2 &b_dir, PN_stdfloat cross_threshold) {
  return dot(a_dir, b_dir) <= 0 || fabs(cross(a_dir, b_dir)) > cross_threshold;
}

/**
 *
 */
INLINE void switch_color(EdgeColor &color, unsigned long long &seed, EdgeColor banned) {
  EdgeColor combined = EdgeColor(color & banned);
  if (combined == RED || combined == GREEN || combined == BLUE) {
    color = EdgeColor(combined ^ WHITE);
    return;
  }
  if (color == BLACK || color == WHITE) {
    static const EdgeColor start[3] = { CYAN, MAGENTA, YELLOW };
    color = start[seed % 3];
    seed /= 3;
    return;
  }
  int shifted = color << (1 + (seed & 1));
  color = EdgeColor((shifted | shifted >> 3) & WHITE);
  seed >>= 1;
}

/**
 *
 */
INLINE void point_bounds(LPoint2 p, PN_stdfloat &l, PN_stdfloat &b, PN_stdfloat &r, PN_stdfloat &t) {
  if (p[0] < l) {
    l = p[0];
  }
  if (p[1] < b) {
    b = p[1];
  }
  if (p[0] > r) {
    r = p[0];
  }
  if (p[1] > t) {
    t = p[1];
  }
}

/**
 * Is here because there seemed to be a noticable performance drop w/o INLINE
 */
INLINE bool pixel_clash(const LRGBColorf &a, const LRGBColorf &b, PN_stdfloat threshold) {
  // Only consider pair where both are on the inside or both are on the outside
  bool aIn = (a[0] > .5f)+(a[1] > .5f)+(a[2] > .5f) >= 2;
  bool bIn = (b[0] > .5f)+(b[1] > .5f)+(b[2] > .5f) >= 2;
  if (aIn != bIn) {
    return false;
  }
  // If the change is 0 <-> 1 or 2 <-> 3 channels and not 1 <-> 1 or 2 <-> 2, it is not a clash
  if ((a[0] > .5f && a[1] > .5f && a[2] > .5f) || (a[0] < .5f && a[1] < .5f && a[2] < .5f)
      || (b[0] > .5f && b[1] > .5f && b[2] > .5f) || (b[0] < .5f && b[1] < .5f && b[2] < .5f)) {
    return false;
  }
  // Find which color is which: _a, _b = the changing channels, _c = the remaining one
  PN_stdfloat aa, ab, ba, bb, ac, bc;
  if ((a[0] > .5f) != (b[0] > .5f) && (a[0] < .5f) != (b[0] < .5f)) {
    aa = a[0], ba = b[0];
    if ((a[1] > .5f) != (b[1] > .5f) && (a[1] < .5f) != (b[1] < .5f)) {
      ab = a[1], bb = b[1];
      ac = a[2], bc = b[2];
    }
    else if ((a[2] > .5f) != (b[2] > .5f) && (a[2] < .5f) != (b[2] < .5f)) {
      ab = a[2], bb = b[2];
      ac = a[1], bc = b[1];
    }
    else {
      return false; // this should never happen
    }
  }
  else if ((a[1] > .5f) != (b[1] > .5f) && (a[1] < .5f) != (b[1] < .5f)
      && (a[2] > .5f) != (b[2] > .5f) && (a[2] < .5f) != (b[2] < .5f)) {
    aa = a[1], ba = b[1];
    ab = a[2], bb = b[2];
    ac = a[0], bc = b[0];
  }
  else {
    return false;
  }
  // Find if the channels are in fact discontinuous
  return (fabs(aa-ba) >= threshold)
      && (fabs(ab-bb) >= threshold)
      && fabs(ac-.5f) >= fabs(bc-.5f); // Out of the pair, only flag the pixel farther from a shape edge
}

/**
 * Calculate the shoelace formula.
 */
INLINE PN_stdfloat shoelace(const LPoint2 &a, const LPoint2 &b) {
  return (b[0] - a[0]) * (a[1] + b[1]);
}

/**
 * Adds a blank contour and returns its reference.
 */
INLINE MSDFGen::Contour& MSDFGen::Shape::
add_contour() {
  contours.resize(contours.size() + 1);
  return contours[contours.size() - 1];
}

/**
 * Computes the shape's bounding box.
 */
INLINE void MSDFGen::Shape::
bounds(PN_stdfloat &l, PN_stdfloat &b, PN_stdfloat &r, PN_stdfloat &t) const {
  for (const MSDFGen::Contour &contour : contours) {
    contour.bounds(l, b, r, t);
  }
}

/**
 * LinearSegment constructor with 2 LPoint2 and respective EdgeColor.
 */
INLINE MSDFGen::LinearSegment::
LinearSegment(LPoint2 p0, LPoint2 p1, EdgeColor edge_color) : MSDFGen::EdgeSegment(edge_color) {
  p[0] = p0;
  p[1] = p1;
}

/**
 * QuadraticSegment constructor with 3 LPoint2 and respective EdgeColor.
 */
INLINE MSDFGen::QuadraticSegment::
QuadraticSegment(LPoint2 p0, LPoint2 p1, LPoint2 p2, EdgeColor edge_color) : MSDFGen::EdgeSegment(edge_color) {
  if (p1 == p0 || p1 == p2) {
    p1 = 0.5 * (p0 + p2);
  }
  p[0] = p0;
  p[1] = p1;
  p[2] = p2;
}

/**
 * CubicSegment constructor with 4 LPoint2 and respective EdgeColor.
 */
INLINE MSDFGen::CubicSegment::
CubicSegment(LPoint2 p0, LPoint2 p1, LPoint2 p2, LPoint2 p3, EdgeColor edge_color) : MSDFGen::EdgeSegment(edge_color) {
  p[0] = p0;
  p[1] = p1;
  p[2] = p2;
  p[3] = p3;
}

/**
 * Creates a copy of the edge segment.
 */
INLINE MSDFGen::LinearSegment* MSDFGen::LinearSegment::
clone() const {
  return new MSDFGen::LinearSegment(p[0], p[1], color);
}

/**
 * Creates a copy of the edge segment.
 */
INLINE MSDFGen::QuadraticSegment* MSDFGen::QuadraticSegment::
clone() const {
  return new MSDFGen::QuadraticSegment(p[0], p[1], p[2], color);
}

/**
 * Creates a copy of the edge segment.
 */
INLINE MSDFGen::CubicSegment* MSDFGen::CubicSegment::
clone() const {
  return new MSDFGen::CubicSegment(p[0], p[1], p[2], p[3], color);
}

/**
 * Returns the point on the edge specified by the parameter (between 0 and 1).
 */
INLINE LPoint2 MSDFGen::LinearSegment::
point(PN_stdfloat param) const {
  return mix(p[0], p[1], param);
}

/**
 * Returns the point on the edge specified by the parameter (between 0 and 1).
 */
INLINE LPoint2 MSDFGen::QuadraticSegment::
point(PN_stdfloat param) const {
  return mix(mix(p[0], p[1], param), mix(p[1], p[2], param), param);
}

/**
 * Returns the point on the edge specified by the parameter (between 0 and 1).
 */
INLINE LPoint2 MSDFGen::CubicSegment::
point(PN_stdfloat param) const {
  LVector2 p12 = mix(p[1], p[2], param);
  return mix(mix(mix(p[0], p[1], param), (LPoint2) p12, param), mix((LPoint2) p12, mix(p[2], p[3], param), param), param);
}

/**
 * Returns the direction the edge has at the point specified by the parameter.
 */
INLINE LVector2 MSDFGen::LinearSegment::
direction(PN_stdfloat param) const {
  return p[1] - p[0];
}

/**
 * Returns the direction the edge has at the point specified by the parameter.
 */
INLINE LVector2 MSDFGen::QuadraticSegment::
direction(PN_stdfloat param) const {
  return mix(p[1] - p[0], p[2] - p[1], param);
}

/**
 * Returns the direction the edge has at the point specified by the parameter.
 */
INLINE LVector2 MSDFGen::CubicSegment::
direction(PN_stdfloat param) const {
  LVector2 tangent = mix(mix(p[1] - p[0], p[2] - p[1], param), mix(p[2] - p[1], p[3] - p[2], param), param);
  if (!tangent[0] && !tangent[1]) {
    if (param == 0) {
      return p[2] - p[0];
    }
    if (param == 1) {
      return p[3] - p[1];
    }
  }
  return tangent;
}

/**
 * Adjusts the bounding box to fit the edge segment.
 */
INLINE void MSDFGen::LinearSegment::
bounds(PN_stdfloat &l, PN_stdfloat &b, PN_stdfloat &r, PN_stdfloat &t) const {
  point_bounds(p[0], l, b, r, t);
  point_bounds(p[1], l, b, r, t);
}

/**
 * Splits the edge segments into thirds which together represent the original edge.
 */
INLINE void MSDFGen::LinearSegment::
split_in_thirds(MSDFGen::EdgeSegment *&part1, MSDFGen::EdgeSegment *&part2, MSDFGen::EdgeSegment *&part3) const {
  part1 = new LinearSegment(p[0], point(1 / 3.0), color);
  part2 = new LinearSegment(point(1 / 3.0), point(2 / 3.0), color);
  part3 = new LinearSegment(point(2 / 3.0), p[1], color);
}

/**
 * Splits the edge segments into thirds which together represent the original edge.
 */
INLINE void MSDFGen::QuadraticSegment::
split_in_thirds(MSDFGen::EdgeSegment *&part1, MSDFGen::EdgeSegment *&part2, MSDFGen::EdgeSegment *&part3) const {
  part1 = new QuadraticSegment(p[0], mix(p[0], p[1], 1 / 3.0), point(1 / 3.0), color);
  part2 = new QuadraticSegment(point(1 / 3.0),
    mix(mix(p[0], p[1], 5 / 9.0), mix(p[1], p[2], 4 / 9.0), 0.5), point(2 / 3.0), color);
  part3 = new QuadraticSegment(point(2 / 3.0), mix(p[1], p[2], 2 / 3.0), p[2], color);
}

/**
 *
 */
INLINE MSDFGen::EdgeHolder::
EdgeHolder() : edge_segment(NULL) { }

/**
 *
 */
INLINE MSDFGen::EdgeHolder::
EdgeHolder(MSDFGen::EdgeSegment *segment) : edge_segment(segment) { }

/**
 *
 */
INLINE MSDFGen::EdgeHolder::
EdgeHolder(LPoint2 p0, LPoint2 p1, EdgeColor edge_color) :
  edge_segment(new MSDFGen::LinearSegment(p0, p1, edge_color)) { }

/**
 *
 */
INLINE MSDFGen::EdgeHolder::
EdgeHolder(LPoint2 p0, LPoint2 p1, LPoint2 p2, EdgeColor edge_color) :
  edge_segment(new MSDFGen::QuadraticSegment(p0, p1, p2, edge_color)) { }

/**
 *
 */
INLINE MSDFGen::EdgeHolder::
EdgeHolder(LPoint2 p0, LPoint2 p1, LPoint2 p2, LPoint2 p3, EdgeColor edge_color) :
  edge_segment(new MSDFGen::CubicSegment(p0, p1, p2, p3, edge_color)) { }

/**
 *
 */
INLINE MSDFGen::EdgeHolder::
EdgeHolder(const MSDFGen::EdgeHolder &orig) :
  edge_segment(orig.edge_segment ? orig.edge_segment->clone() : NULL) { }

/**
 *
 */
INLINE MSDFGen::EdgeHolder::
~EdgeHolder() {
  delete edge_segment;
}

/**
 *
 */
INLINE MSDFGen::EdgeHolder &MSDFGen::EdgeHolder::
operator=(const MSDFGen::EdgeHolder &orig) {
  delete edge_segment;
  edge_segment = orig.edge_segment ? orig.edge_segment->clone() : NULL;
  return *this;
}

/**
 *
 */
INLINE MSDFGen::EdgeSegment &MSDFGen::EdgeHolder::
operator*() {
  return *edge_segment;
}

/**
 *
 */
INLINE const MSDFGen::EdgeSegment &MSDFGen::EdgeHolder::
operator*() const {
  return *edge_segment;
}

/**
 *
 */
INLINE MSDFGen::EdgeSegment *MSDFGen::EdgeHolder::
operator->() {
  return edge_segment;
}

/**
 *
 */
INLINE const MSDFGen::EdgeSegment *MSDFGen::EdgeHolder::
operator->() const {
  return edge_segment;
}

/**
 *
 */
INLINE MSDFGen::EdgeHolder::
operator MSDFGen::EdgeSegment *() {
  return edge_segment;
}

/**
 *
 */
INLINE MSDFGen::EdgeHolder::
operator const MSDFGen::EdgeSegment *() const {
  return edge_segment;
}

/**
 * Adds an edge to the contour.
 */
INLINE void MSDFGen::Contour::
add_edge(const EdgeHolder &edge) {
  edges.push_back(edge);
}

/**
 * Creates a new edge in the contour and returns its reference.
 */
INLINE MSDFGen::EdgeHolder &MSDFGen::Contour::
add_edge() {
  edges.resize(edges.size() + 1);
  return edges[edges.size() - 1];
}

/**
 * Computes the bounding box of the contour.
 */
INLINE void MSDFGen::Contour::
bounds(PN_stdfloat &l, PN_stdfloat &b, PN_stdfloat &r, PN_stdfloat &t) const {
  for (const EdgeHolder &edge : edges) {
    edge->bounds(l, b, r, t);
  }
}

/**
 *
 */
INLINE MSDFGen::SignedDistance::
SignedDistance() : distance(-std::numeric_limits<PN_stdfloat>::max()), dot(1) { }

/**
 *
 */
INLINE MSDFGen::SignedDistance::
SignedDistance(PN_stdfloat dist, PN_stdfloat d) : distance(dist), dot(d) { }

/**
 *
 */
INLINE bool operator<(MSDFGen::SignedDistance a, MSDFGen::SignedDistance b) {
  return fabs(a.distance) < fabs(b.distance)
         || (fabs(a.distance) == fabs(b.distance) && a.dot < b.dot);
}

/**
 *
 */
INLINE bool operator>(MSDFGen::SignedDistance a, MSDFGen::SignedDistance b) {
  return fabs(a.distance) > fabs(b.distance)
         || (fabs(a.distance) == fabs(b.distance) && a.dot > b.dot);
}

/**
 *
 */
INLINE bool operator<=(MSDFGen::SignedDistance a, MSDFGen::SignedDistance b) {
  return fabs(a.distance) < fabs(b.distance)
         || (fabs(a.distance) == fabs(b.distance) && a.dot <= b.dot);
}

/**
 *
 */
INLINE bool operator>=(MSDFGen::SignedDistance a, MSDFGen::SignedDistance b) {
  return fabs(a.distance) > fabs(b.distance)
         || (fabs(a.distance) == fabs(b.distance) && a.dot >= b.dot);
}
