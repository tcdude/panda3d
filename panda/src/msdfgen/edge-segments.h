#pragma once

#include "aa_luse.h"
#include "SignedDistance.h"
#include "EdgeColor.h"


// Parameters for iterative search of closest point on a cubic Bezier curve. Increase for higher precision.
#define MSDFGEN_CUBIC_SEARCH_STARTS 4
#define MSDFGEN_CUBIC_SEARCH_STEPS 4

/// An abstract edge segment.
class EdgeSegment {

public:
  EdgeColor color;

  EdgeSegment(EdgeColor edge_color = WHITE) : color(edge_color) { }
  virtual ~EdgeSegment() { }
  /// Creates a copy of the edge segment.
  virtual EdgeSegment *clone() const = 0;
  /// Returns the point on the edge specified by the parameter (between 0 and 1).
  virtual LPoint2 point(PN_stdfloat param) const = 0;
  /// Returns the direction the edge has at the point specified by the parameter.
  virtual LVector2 direction(PN_stdfloat param) const = 0;
  /// Returns the minimum signed distance between origin and the edge.
  virtual SignedDistance signed_distance(LPoint2 origin, PN_stdfloat &param) const = 0;
  /// Converts a previously retrieved signed distance from origin to pseudo-distance.
  virtual void distance_to_pseudo_distance(SignedDistance &distance, LPoint2 origin, PN_stdfloat param) const;
  /// Adjusts the bounding box to fit the edge segment.
  virtual void bounds(PN_stdfloat &l, PN_stdfloat &b, PN_stdfloat &r, PN_stdfloat &t) const = 0;

  /// Moves the start point of the edge segment.
  virtual void move_start_point(LPoint2 to) = 0;
  /// Moves the end point of the edge segment.
  virtual void move_end_point(LPoint2 to) = 0;
  /// Splits the edge segments into thirds which together represent the original edge.
  virtual void split_in_thirds(EdgeSegment *&part1, EdgeSegment *&part2, EdgeSegment *&part3) const = 0;

};

/// A line segment.
class LinearSegment : public EdgeSegment {

public:
  LPoint2 p[2];

  LinearSegment(LPoint2 p0, LPoint2 p1, EdgeColor edge_color = WHITE);
  LinearSegment * clone() const;
  LPoint2 point(PN_stdfloat param) const;
  LVector2 direction(PN_stdfloat param) const;
  SignedDistance signed_distance(LPoint2 origin, PN_stdfloat &param) const;
  void bounds(PN_stdfloat &l, PN_stdfloat &b, PN_stdfloat &r, PN_stdfloat &t) const;

  void move_start_point(LPoint2 to);
  void move_end_point(LPoint2 to);
  void split_in_thirds(EdgeSegment *&part1, EdgeSegment *&part2, EdgeSegment *&part3) const;

};

/// A quadratic Bezier curve.
class QuadraticSegment : public EdgeSegment {

public:
  LPoint2 p[3];

  QuadraticSegment(LPoint2 p0, LPoint2 p1, LPoint2 p2, EdgeColor edge_color = WHITE);
  QuadraticSegment * clone() const;
  LPoint2 point(PN_stdfloat param) const;
  LVector2 direction(PN_stdfloat param) const;
  SignedDistance signed_distance(LPoint2 origin, PN_stdfloat &param) const;
  void bounds(PN_stdfloat &l, PN_stdfloat &b, PN_stdfloat &r, PN_stdfloat &t) const;

  void move_start_point(LPoint2 to);
  void move_end_point(LPoint2 to);
  void split_in_thirds(EdgeSegment *&part1, EdgeSegment *&part2, EdgeSegment *&part3) const;

};

/// A cubic Bezier curve.
class CubicSegment : public EdgeSegment {

public:
  LPoint2 p[4];

  CubicSegment(LPoint2 p0, LPoint2 p1, LPoint2 p2, LPoint2 p3, EdgeColor edge_color = WHITE);
  CubicSegment * clone() const;
  LPoint2 point(PN_stdfloat param) const;
  LVector2 direction(PN_stdfloat param) const;
  SignedDistance signed_distance(LPoint2 origin, PN_stdfloat &param) const;
  void bounds(PN_stdfloat &l, PN_stdfloat &b, PN_stdfloat &r, PN_stdfloat &t) const;

  void move_start_point(LPoint2 to);
  void move_end_point(LPoint2 to);
  void split_in_thirds(EdgeSegment *&part1, EdgeSegment *&part2, EdgeSegment *&part3) const;

};