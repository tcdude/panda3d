/**
 * PANDA 3D SOFTWARE
 * Copyright (c) Carnegie Mellon University.  All rights reserved.
 *
 * All use of this software is subject to the terms of the revised BSD
 * license.  You should have received a copy of this license along
 * with this source code in a file named "LICENSE."
 *
 * @file msdfgen.h
 * @author tcdude
 * @date 2019-02-28
 */

#ifndef MSDFGEN_H
#define MSDFGEN_H

#ifdef HAVE_FREETYPE

#include "luse.h"
#include <limits>
#include <cstdlib>
#include <cmath>
#include <vector>

enum EdgeColor {
  BLACK = 0,
  RED = 1,
  GREEN = 2,
  YELLOW = 3,
  BLUE = 4,
  MAGENTA = 5,
  CYAN = 6,
  WHITE = 7
};

INLINE LVector2 normalize_non_zero(const LVector2 &v);
INLINE LPoint2 min(const LPoint2 &a, const LPoint2 &b);
INLINE LPoint2 max(const LPoint2 &a, const LPoint2 &b);
INLINE PN_stdfloat median(PN_stdfloat a, PN_stdfloat b, PN_stdfloat c);
INLINE LPoint2 mix(const LPoint2 &a, const LPoint2 &b, PN_stdfloat weight);
INLINE PN_stdfloat sign(PN_stdfloat n);
INLINE int non_zero_sign(PN_stdfloat n);
INLINE bool is_corner(const LVector2 &a_dir, const LVector2 &b_dir, PN_stdfloat cross_threshold);
INLINE void switch_color(EdgeColor &color, unsigned long long &seed, EdgeColor banned = BLACK);
INLINE void point_bounds(LPoint2 p, PN_stdfloat &l, PN_stdfloat &b, PN_stdfloat &r, PN_stdfloat &t);
INLINE bool pixel_clash(const LRGBColorf &a, const LRGBColorf &b, PN_stdfloat threshold);
INLINE PN_stdfloat shoelace(const LPoint2 &a, const LPoint2 &b);

int solve_quadratic(PN_stdfloat x[2], PN_stdfloat a, PN_stdfloat b, PN_stdfloat c);
int solve_cubic(PN_stdfloat x[3], PN_stdfloat a, PN_stdfloat b, PN_stdfloat c, PN_stdfloat d);
int solve_cubic_normed(PN_stdfloat *x, PN_stdfloat a, PN_stdfloat b, PN_stdfloat c);

/**
 * This class enables the generation of Multi-Channel Signed Distance Fields
 * for improved corners in glyphs. This implementation is based on the work
 * from https://github.com/Chlumsky/msdfgen/, adapted to be used together
 * with FreetypeFont derivatives in Panda3D by rdb and tcdude.
 */
class MSDFGen {

public:
  class Contour;
  class Shape;
  class EdgeHolder;
  class EdgeSegment;
  class SignedDistance;


  /**
   * Vector shape representation.
   */
  class Shape {

  public:
    std::vector<Contour> contours;
    /// Specifies whether the shape uses bottom-to-top (false) or top-to-bottom (true) Y coordinates.
    bool inverseYAxis;

    Shape();
    INLINE void add_contour(const Contour &contour);
    INLINE Contour& add_contour();
    void normalize();
    bool validate() const;
    INLINE void bounds(PN_stdfloat &l, PN_stdfloat &b, PN_stdfloat &r, PN_stdfloat &t) const;
  };

  /**
   * An abstract edge segment.
   */
  class EdgeSegment {

  public:
    EdgeColor color;

    EdgeSegment(EdgeColor edge_color = WHITE) : color(edge_color) { }
    virtual ~EdgeSegment() { }
    virtual EdgeSegment *clone() const = 0;
    virtual LPoint2 point(PN_stdfloat param) const = 0;
    virtual LVector2 direction(PN_stdfloat param) const = 0;
    virtual SignedDistance signed_distance(LPoint2 origin, PN_stdfloat &param) const = 0;
    virtual void distance_to_pseudo_distance(SignedDistance &distance, LPoint2 origin, PN_stdfloat param) const;
    virtual void bounds(PN_stdfloat &l, PN_stdfloat &b, PN_stdfloat &r, PN_stdfloat &t) const = 0;

    virtual void move_start_point(LPoint2 to) = 0;
    virtual void move_end_point(LPoint2 to) = 0;
    virtual void split_in_thirds(EdgeSegment *&part1, EdgeSegment *&part2, EdgeSegment *&part3) const = 0;

  };

  /**
   * A line segment.
   */
  class LinearSegment : public EdgeSegment {

  public:
    LPoint2 p[2];

    INLINE LinearSegment(LPoint2 p0, LPoint2 p1, EdgeColor edge_color = WHITE);
    INLINE LinearSegment * clone() const;
    INLINE LPoint2 point(PN_stdfloat param) const;
    INLINE LVector2 direction(PN_stdfloat param) const;
    SignedDistance signed_distance(LPoint2 origin, PN_stdfloat &param) const;
    INLINE void bounds(PN_stdfloat &l, PN_stdfloat &b, PN_stdfloat &r, PN_stdfloat &t) const;

    INLINE void move_start_point(LPoint2 to);
    INLINE void move_end_point(LPoint2 to);
    INLINE void split_in_thirds(EdgeSegment *&part1, EdgeSegment *&part2, EdgeSegment *&part3) const;

  };

  /**
   * A quadratic Bezier curve.
   */
  class QuadraticSegment : public EdgeSegment {

  public:
    LPoint2 p[3];

    INLINE QuadraticSegment(LPoint2 p0, LPoint2 p1, LPoint2 p2, EdgeColor edge_color = WHITE);
    INLINE QuadraticSegment * clone() const;
    INLINE LPoint2 point(PN_stdfloat param) const;
    INLINE LVector2 direction(PN_stdfloat param) const;
    SignedDistance signed_distance(LPoint2 origin, PN_stdfloat &param) const;
    void bounds(PN_stdfloat &l, PN_stdfloat &b, PN_stdfloat &r, PN_stdfloat &t) const;

    void move_start_point(LPoint2 to);
    void move_end_point(LPoint2 to);
    INLINE void split_in_thirds(EdgeSegment *&part1, EdgeSegment *&part2, EdgeSegment *&part3) const;

  };

  /**
   * A cubic Bezier curve.
   */
  class CubicSegment : public EdgeSegment {

  public:
    LPoint2 p[4];

    INLINE CubicSegment(LPoint2 p0, LPoint2 p1, LPoint2 p2, LPoint2 p3, EdgeColor edge_color = WHITE);
    INLINE CubicSegment * clone() const;
    INLINE LPoint2 point(PN_stdfloat param) const;
    INLINE LVector2 direction(PN_stdfloat param) const;
    SignedDistance signed_distance(LPoint2 origin, PN_stdfloat &param) const;
    void bounds(PN_stdfloat &l, PN_stdfloat &b, PN_stdfloat &r, PN_stdfloat &t) const;

    INLINE void move_start_point(LPoint2 to);
    INLINE void move_end_point(LPoint2 to);
    void split_in_thirds(EdgeSegment *&part1, EdgeSegment *&part2, EdgeSegment *&part3) const;

  };

  /**
   * Container for a single edge of dynamic type.
   */
  class EdgeHolder {

  public:
    INLINE EdgeHolder();
    INLINE EdgeHolder(EdgeSegment *segment);
    INLINE EdgeHolder(LPoint2 p0, LPoint2 p1, EdgeColor edge_color = WHITE);
    INLINE EdgeHolder(LPoint2 p0, LPoint2 p1, LPoint2 p2, EdgeColor edge_color = WHITE);
    INLINE EdgeHolder(LPoint2 p0, LPoint2 p1, LPoint2 p2, LPoint2 p3, EdgeColor edge_color = WHITE);
    INLINE EdgeHolder(const EdgeHolder &orig);
    INLINE ~EdgeHolder();
    INLINE EdgeHolder & operator=(const EdgeHolder &orig);
    INLINE EdgeSegment & operator*();
    INLINE const EdgeSegment & operator*() const;
    INLINE EdgeSegment * operator->();
    INLINE const EdgeSegment * operator->() const;
    INLINE operator EdgeSegment *();
    INLINE operator const EdgeSegment *() const;

  private:
      EdgeSegment *edge_segment;
  };

  /**
   * A single closed contour of a shape.
   */
  class Contour {

  public:
    std::vector<EdgeHolder> edges;

    INLINE void add_edge(const EdgeHolder &edge);
    INLINE EdgeHolder & add_edge();
    INLINE void bounds(PN_stdfloat &l, PN_stdfloat &b, PN_stdfloat &r, PN_stdfloat &t) const;
    int winding() const;
  };

  /**
   * Represents a signed distance and alignment, which together can be compared
   * to uniquely determine the closest edge segment.
   */
  class SignedDistance {

  public:
    //static const SignedDistance INFINITE;

    PN_stdfloat distance;
    PN_stdfloat dot;

    INLINE SignedDistance();
    INLINE SignedDistance(PN_stdfloat dist, PN_stdfloat d);

    friend INLINE bool operator<(SignedDistance a, SignedDistance b);
    friend INLINE bool operator>(SignedDistance a, SignedDistance b);
    friend INLINE bool operator<=(SignedDistance a, SignedDistance b);
    friend INLINE bool operator>=(SignedDistance a, SignedDistance b);
  };

  static void edge_coloring_simple(Shape &shape, PN_stdfloat angle_threshold, unsigned long long seed = 0);

};

#include "msdfgen.I"

#endif  // HAVE_FREETYPE

#endif