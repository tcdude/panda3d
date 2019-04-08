/**
 * PANDA 3D SOFTWARE
 * Copyright (c) Carnegie Mellon University.  All rights reserved.
 *
 * All use of this software is subject to the terms of the revised BSD
 * license.  You should have received a copy of this license along
 * with this source code in a file named "LICENSE."
 *
 * @file msdfgen.cxx
 * @author tcdude
 * @date 2019-02-28
 */

#include "msdfgen.h"
#include "config_pnmtext.h"

#ifdef HAVE_FREETYPE

/**
 * ax^2 + bx + c = 0
 */
int solve_quadratic(PN_stdfloat x[2], PN_stdfloat a, PN_stdfloat b, PN_stdfloat c) {
  if (fabs(a) < 1e-14) {
    if (fabs(b) < 1e-14) {
      if (c == 0) {
        return -1;
      }
      return 0;
    }
    x[0] = -c / b;
    return 1;
  }
  PN_stdfloat dscr = b * b - 4 * a * c;
  if (dscr > 0) {
    dscr = sqrt(dscr);
    x[0] = (-b + dscr) / (2 * a);
    x[1] = (-b - dscr) / (2 * a);
    return 2;
  }
  else if (dscr == 0) {
    x[0] = -b / (2 * a);
    return 1;
  }
  else {
    return 0;
  }
}

/**
 *
 */
static int solve_cubic_normed(PN_stdfloat x[3], PN_stdfloat a, PN_stdfloat b, PN_stdfloat c) {
  PN_stdfloat a2 = a * a;
  PN_stdfloat q  = (a2 - 3 * b) / 9;
  PN_stdfloat r  = (a * (2 * a2 - 9 * b) + 27 * c) / 54;
  PN_stdfloat r2 = r * r;
  PN_stdfloat q3 = q * q * q;
  PN_stdfloat A, B;
  if (r2 < q3) {
    PN_stdfloat t = r / sqrt(q3);
    if (t < -1) {
      t = -1;
    }
    if (t > 1) {
      t = 1;
    }
    t = acos(t);
    a /= 3; q = -2 * sqrt(q);
    x[0] = q * cos(t / 3) - a;
    x[1] = q * cos((t + 2 * M_PI) / 3) - a;
    x[2] = q * cos((t - 2 * M_PI) / 3) - a;
    return 3;
  }
  else {
    A = -pow(fabs(r) + sqrt(r2 - q3), 1 / 3.0);
    if (r < 0) {
      A = -A;
    }
    B = A == 0 ? 0 : q / A;
    a /= 3;
    x[0] = (A + B) - a;
    x[1] = -0.5 * (A + B) - a;
    x[2] = 0.5 * sqrt(3.0) * (A - B);
    if (fabs(x[2]) < 1e-14) {
      return 2;
    }
    return 1;
  }
}

/**
 * ax^3 + bx^2 + cx + d = 0
 */
int solve_cubic(PN_stdfloat x[3], PN_stdfloat a, PN_stdfloat b, PN_stdfloat c, PN_stdfloat d) {
  if (fabs(a) < 1e-14) {
    return solve_quadratic(x, b, c, d);
  }
  return solve_cubic_normed(x, b/a, c/a, d/a);
}

/**
 * Assigns colors to edges of the shape in accordance to the multi-channel
 * distance field technique. May split some edges if necessary.
 * angleThreshold specifies the maximum angle (in radians) to be considered
 * a corner, for example 3 (~172 degrees). Values below 1/2 PI will be treated
 * as the external angle.
 */
void MSDFGen::
edge_coloring_simple(MSDFGen::Shape &shape, PN_stdfloat angle_threshold, unsigned long long seed) {
  PN_stdfloat cross_threshold = sin(angle_threshold);
  std::vector<int> corners;
  for (MSDFGen::Contour &contour : shape.contours) {
    // Identify corners
    corners.clear();
    if (!contour.edges.empty()) {
      LVector2 prevDirection = contour.edges.back()->direction(1);
      int index = 0;
      for (const MSDFGen::EdgeHolder &edge : contour.edges) {
        if (is_corner(normalize_non_zero(prevDirection), normalize_non_zero(edge->direction(0)), cross_threshold)) {
          corners.push_back(index);
        }
        prevDirection = edge->direction(1);
        ++index;
      }
    }

    // Smooth contour
    if (corners.empty()) {
      for (MSDFGen::EdgeHolder &edge : contour.edges) {
        edge->color = WHITE;
      }
    }
    // "Teardrop" case
    else if (corners.size() == 1) {
      EdgeColor colors[3] = { WHITE, WHITE };
      switch_color(colors[0], seed);
      switch_color(colors[2] = colors[0], seed);
      int corner = corners[0];
      if (contour.edges.size() >= 3) {
        int m = (int)contour.edges.size();
        for (int i = 0; i < m; ++i) {
          contour.edges[(corner + i) % m]->color =
            (colors + 1)[int(3 + 2.875 * i / (m - 1) - 1.4375 + 0.5) - 3];
        }
      } else if (contour.edges.size() >= 1) {
        // Less than three edge segments for three colors => edges must be split
        MSDFGen::EdgeSegment *parts[7] = { };
        contour.edges[0]->split_in_thirds(parts[0 + 3 * corner],
          parts[1 + 3 * corner], parts[2 + 3 * corner]);
        if (contour.edges.size() >= 2) {
          contour.edges[1]->split_in_thirds(parts[3-3*corner],
            parts[4 - 3 * corner], parts[5 - 3 * corner]);
          parts[0]->color = parts[1]->color = colors[0];
          parts[2]->color = parts[3]->color = colors[1];
          parts[4]->color = parts[5]->color = colors[2];
        } else {
          parts[0]->color = colors[0];
          parts[1]->color = colors[1];
          parts[2]->color = colors[2];
        }
        contour.edges.clear();
        for (int i = 0; parts[i]; ++i)
          contour.edges.push_back(MSDFGen::EdgeHolder(parts[i]));
      }
    }
    // Multiple corners
    else {
        int cornerCount = (int)corners.size();
        int spline = 0;
        int start = corners[0];
        int m = (int)contour.edges.size();
        EdgeColor color = WHITE;
        switch_color(color, seed);
        EdgeColor initialColor = color;
        for (int i = 0; i < m; ++i) {
            int index = (start + i) % m;
            if (spline + 1 < cornerCount && corners[spline + 1] == index) {
                ++spline;
                switch_color(color, seed,
                  EdgeColor((spline == cornerCount - 1) * initialColor));
            }
            contour.edges[index]->color = color;
        }
    }
  }
}

/**
 * Default constructor uses bottom-to-top Y coordinates.
 */
MSDFGen::Shape::
Shape() : inverseYAxis(false) { }

/**
 * Normalizes the shape geometry for distance field generation.
 */
void MSDFGen::Shape::
normalize() {
  for (MSDFGen::Contour &contour : contours)
    if (contour.edges.size() == 1) {
      EdgeSegment *parts[3] = { };
      contour.edges[0]->split_in_thirds(parts[0], parts[1], parts[2]);
      contour.edges.clear();
      contour.edges.push_back(EdgeHolder(parts[0]));
      contour.edges.push_back(EdgeHolder(parts[1]));
      contour.edges.push_back(EdgeHolder(parts[2]));
    }
}

/**
 * Performs basic checks to determine if the object represents a valid shape.
 */
bool MSDFGen::Shape::
validate() const {
  for (const MSDFGen::Contour &contour : contours) {
    if (!contour.edges.empty()) {
      LPoint2 corner = contour.edges.back()->point(1);
      for (const EdgeHolder &edge : contour.edges) {
        if (!edge) {
          return false;
        }
        if (edge->point(0) != corner) {
          return false;
        }
        corner = edge->point(1);
      }
    }
  }
  return true;
}

/**
 * Converts a previously retrieved signed distance from origin to pseudo-distance.
 */
void MSDFGen::EdgeSegment::
distance_to_pseudo_distance(MSDFGen::SignedDistance &distance, LPoint2 origin, PN_stdfloat param) const {
  if (param < 0) {
    LVector2 dir = normalize_non_zero(direction(0));
    LVector2 aq = origin - point(0);
    PN_stdfloat ts = dot(aq, dir);
    if (ts < 0) {
      PN_stdfloat pseudo_distance = cross(aq, dir);
      if (fabs(pseudo_distance) <= fabs(distance.distance)) {
        distance.distance = pseudo_distance;
        distance.dot = 0;
      }
    }
  } else if (param > 1) {
    LVector2 dir = normalize_non_zero(direction(1));
    LVector2 bq = origin - point(1);
    PN_stdfloat ts = dot(bq, dir);
    if (ts > 0) {
      PN_stdfloat pseudo_distance = cross(bq, dir);
      if (fabs(pseudo_distance) <= fabs(distance.distance)) {
        distance.distance = pseudo_distance;
        distance.dot = 0;
      }
    }
  }
}

/**
 * Returns the minimum signed distance between origin and the edge.
 */
MSDFGen::SignedDistance MSDFGen::LinearSegment::
signed_distance(LPoint2 origin, PN_stdfloat &param) const {
  LVector2 aq = origin - p[0];
  LVector2 ab = p[1] - p[0];
  param = dot(aq, ab) / dot(ab, ab);
  LVector2 eq = p[param > 0.5]-origin;
  PN_stdfloat endpoint_distance = eq.length();
  if (param > 0 && param < 1) {
    PN_stdfloat l = ab.length();
    LVector2 orthonormal;
    if (l == 0) {
      orthonormal = LVector2(0, -1);
    }
    else {
      orthonormal = LVector2(ab[1] / l, -ab[0] / l);
    }
    PN_stdfloat ortho_distance = dot(orthonormal, aq);
    if (fabs(ortho_distance) < endpoint_distance)
      return MSDFGen::SignedDistance(ortho_distance, 0);
  }
  LVector2 ab_n = normalize_non_zero(ab), eq_n = normalize_non_zero(eq);
  return MSDFGen::SignedDistance(non_zero_sign(cross(aq, ab)) * endpoint_distance, fabs(dot(ab_n, eq_n)));
}

/**
 * Returns the minimum signed distance between origin and the edge.
 */
MSDFGen::SignedDistance MSDFGen::QuadraticSegment::
signed_distance(LPoint2 origin, PN_stdfloat &param) const {
  LVector2 qa = p[0] - origin;
  LVector2 ab = p[1] - p[0];
  LVector2 br = p[2] - p[1] - ab;
  PN_stdfloat a = dot(br, br);
  PN_stdfloat b = 3 * dot(ab, br);
  PN_stdfloat c = 2 * dot(ab, ab) + dot(qa, br);
  PN_stdfloat d = dot(qa, ab);
  PN_stdfloat t[3];
  int solutions = solve_cubic(t, a, b, c, d);

  PN_stdfloat min_distance = non_zero_sign(cross(ab, qa)) * qa.length(); // distance from A
  param = -dot(qa, ab) / dot(ab, ab);
  {
    PN_stdfloat distance = non_zero_sign(cross(p[2] - p[1], p[2] - origin));
    distance *= (p[2] - origin).length(); // distance from B
    if (fabs(distance) < fabs(min_distance)) {
      min_distance = distance;
      param = dot(origin - p[1], p[2] - p[1]) / dot(p[2] - p[1], p[2] - p[1]);
    }
  }
  for (int i = 0; i < solutions; ++i) {
    if (t[i] > 0 && t[i] < 1) {
      LPoint2 endpoint = p[0] + 2 * t[i] * ab + t[i] * t[i] * br;
      PN_stdfloat distance = non_zero_sign(cross(p[2] - p[0], endpoint - origin));
      distance *= (endpoint - origin).length();
      if (fabs(distance) <= fabs(min_distance)) {
        min_distance = distance;
        param = t[i];
      }
    }
  }

  if (param >= 0 && param <= 1) {
    return MSDFGen::SignedDistance(min_distance, 0);
  }
  if (param < 0.5) {
    LVector2 ab_n = normalize_non_zero(ab), qa_n = normalize_non_zero(qa);
    return MSDFGen::SignedDistance(min_distance, fabs(dot(ab_n, qa_n)));
  }
  else {
    LVector2 ab_n = normalize_non_zero(p[2] - p[1]), qa_n = normalize_non_zero(p[2] - origin);
    return MSDFGen::SignedDistance(min_distance, fabs(dot(ab_n, qa_n)));
  }
}

/**
 * Returns the minimum signed distance between origin and the edge.
 */
MSDFGen::SignedDistance MSDFGen::CubicSegment::
signed_distance(LPoint2 origin, PN_stdfloat &param) const {
  LVector2 qa = p[0] - origin;
  LVector2 ab = p[1] - p[0];
  LVector2 br = p[2] - p[1] - ab;
  LVector2 as = (p[3] - p[2]) - (p[2] - p[1]) - br;

  LVector2 ep_dir = direction(0);
  PN_stdfloat min_distance = non_zero_sign(cross(ep_dir, qa)) * qa.length(); // distance from A
  param = -dot(qa, ep_dir) / dot(ep_dir, ep_dir);
  {
    ep_dir = direction(1);
    PN_stdfloat distance = non_zero_sign(cross(ep_dir, p[3] - origin));
    distance *= (p[3] - origin).length(); // distance from B
    if (fabs(distance) < fabs(min_distance)) {
      min_distance = distance;
      param = dot(origin + ep_dir - p[3], ep_dir) / dot(ep_dir, ep_dir);
    }
  }
  // Iterative minimum distance search
  for (int i = 0; i <= text_msdf_cubic_search_starts; ++i) {
    PN_stdfloat t = (PN_stdfloat) i / text_msdf_cubic_search_starts;
    for (int step = 0;; ++step) {
      LVector2 qpt = point(t) - origin;
      PN_stdfloat distance = non_zero_sign(cross(direction(t), qpt)) * qpt.length();
      if (fabs(distance) < fabs(min_distance)) {
        min_distance = distance;
        param = t;
      }
      if (step == text_msdf_cubic_search_steps) {
        break;
      }
      // Improve t
      LVector2 d1 = (PN_stdfloat) 3.0 * as * t * t + (PN_stdfloat) 6.0 * br * t;
      d1 += (PN_stdfloat) 3.0 * ab;
      LVector2 d2 = (PN_stdfloat) 6.0 * as * t + (PN_stdfloat) 6.0 * br;
      t -= dot(qpt, d1) / (dot(d1, d1) + dot(qpt, d2));
      if (t < 0 || t > 1) {
        break;
      }
    }
  }

  if (param >= 0 && param <= 1) {
    return MSDFGen::SignedDistance(min_distance, 0);
  }
  if (param < 0.5) {
    LVector2 dir_n = normalize_non_zero(direction(0)), qa_n = normalize_non_zero(qa);
    return MSDFGen::SignedDistance(min_distance, fabs(dot(dir_n, qa_n)));
  }
  else {
    LVector2 dir_n = normalize_non_zero(direction(1));
    LVector2 qa_n = normalize_non_zero(p[3]-origin);
    return MSDFGen::SignedDistance(min_distance, fabs(dot(dir_n, qa_n)));
  }
}

/**
 * Adjusts the bounding box to fit the edge segment.
 */
void MSDFGen::QuadraticSegment::
bounds(PN_stdfloat &l, PN_stdfloat &b, PN_stdfloat &r, PN_stdfloat &t) const {
  point_bounds(p[0], l, b, r, t);
  point_bounds(p[2], l, b, r, t);
  LVector2 bot = (p[1] - p[0]) - (p[2] - p[1]);
  if (bot[0]) {
    PN_stdfloat param = (p[1][0] - p[0][0]) / bot[0];
    if (param > 0 && param < 1) {
      point_bounds(point(param), l, b, r, t);
    }
  }
  if (bot[1]) {
    PN_stdfloat param = (p[1][1] - p[0][1]) / bot[1];
    if (param > 0 && param < 1) {
      point_bounds(point(param), l, b, r, t);
    }
  }
}

/**
 * Adjusts the bounding box to fit the edge segment.
 */
void MSDFGen::CubicSegment::
bounds(PN_stdfloat &l, PN_stdfloat &b, PN_stdfloat &r, PN_stdfloat &t) const {
  point_bounds(p[0], l, b, r, t);
  point_bounds(p[3], l, b, r, t);
  LVector2 a0 = p[1] - p[0];
  LVector2 a1 = (PN_stdfloat) 2 * (p[2] - p[1] - a0);
  LVector2 a2 = p[3] - (PN_stdfloat) 3 * p[2] + (PN_stdfloat) 3 * p[1] - p[0];
  PN_stdfloat params[2];
  int solutions;
  solutions = solve_quadratic(params, a2[0], a1[0], a0[0]);
  for (int i = 0; i < solutions; ++i) {
    if (params[i] > 0 && params[i] < 1) {
      point_bounds(point(params[i]), l, b, r, t);
    }
  }
  solutions = solve_quadratic(params, a2[1], a1[1], a0[1]);
  for (int i = 0; i < solutions; ++i) {
    if (params[i] > 0 && params[i] < 1) {
      point_bounds(point(params[i]), l, b, r, t);
    }
  }
}

/**
 * Splits the edge segments into thirds which together represent the original edge.
 */
void MSDFGen::CubicSegment::
split_in_thirds(MSDFGen::EdgeSegment *&part1, MSDFGen::EdgeSegment *&part2, MSDFGen::EdgeSegment *&part3) const {
  part1 = new CubicSegment(p[0], p[0] == p[1] ? p[0] : mix(p[0], p[1], 1 / 3.0),
    mix(mix(p[0], p[1], 1 / 3.0), mix(p[1], p[2], 1 / 3.0), 1 / 3.0), point(1 / 3.0), color);
  part2 = new CubicSegment(point(1 / 3.0),
    mix(mix(mix(p[0], p[1], 1 / 3.0), mix(p[1], p[2], 1 / 3.0), 1 / 3.0),
      mix(mix(p[1], p[2], 1 / 3.0), mix(p[2], p[3], 1 / 3.0), 1 / 3.0), 2 / 3.0),
    mix(mix(mix(p[0], p[1], 2 / 3.0), mix(p[1], p[2], 2 / 3.0), 2 / 3.0),
      mix(mix(p[1], p[2], 2 / 3.0), mix(p[2], p[3], 2 / 3.0), 2 / 3.0), 1 / 3.0),
    point(2 / 3.0), color);
  part3 = new CubicSegment(point(2 / 3.0), mix(mix(p[1], p[2], 2 / 3.0),
    mix(p[2], p[3], 2 / 3.0), 2 / 3.0), p[2] == p[3] ? p[3] : mix(p[2], p[3], 2 / 3.0), p[3], color);
}

/**
 * Computes the winding of the contour. Returns 1 if positive, -1 if negative.
 */
int MSDFGen::Contour::
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
    for (const EdgeHolder &edge : edges) {
      LPoint2 cur = edge->point(0);
      total += shoelace(prev, cur);
      prev = cur;
    }
  }
  return (0 < total) - (total < 0);
}

#endif // HAVE_FREETYPE
