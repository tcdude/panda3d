#include "edge-segments.h"
#include "luse.h"

#include "arithmetics.hpp"
#include "equation-solver.h"


/**
 * Assures a non zero vector.
 */
static void non_zero_vector(LVector2 &v) {
  if (!v[0] && !v[1]) {
    v[1] = 1;
  }
}

void EdgeSegment::
distance_to_pseudo_distance(SignedDistance &distance, LPoint2 origin, PN_stdfloat param) const {
  if (param < 0) {
    LVector2 dir = direction(0).normalized();
    non_zero_vector(dir);
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
    LVector2 dir = direction(1).normalized();
    non_zero_vector(dir);
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

LinearSegment::
LinearSegment(LPoint2 p0, LPoint2 p1, EdgeColor edge_color) : EdgeSegment(edge_color) {
  p[0] = p0;
  p[1] = p1;
}

QuadraticSegment::
QuadraticSegment(LPoint2 p0, LPoint2 p1, LPoint2 p2, EdgeColor edge_color) : EdgeSegment(edge_color) {
  if (p1 == p0 || p1 == p2) {
    p1 = 0.5 * (p0 + p2);
  }
  p[0] = p0;
  p[1] = p1;
  p[2] = p2;
}

CubicSegment::
CubicSegment(LPoint2 p0, LPoint2 p1, LPoint2 p2, LPoint2 p3, EdgeColor edge_color) : EdgeSegment(edge_color) {
  p[0] = p0;
  p[1] = p1;
  p[2] = p2;
  p[3] = p3;
}

LinearSegment* LinearSegment::
clone() const {
  return new LinearSegment(p[0], p[1], color);
}

QuadraticSegment* QuadraticSegment::
clone() const {
  return new QuadraticSegment(p[0], p[1], p[2], color);
}

CubicSegment* CubicSegment::
clone() const {
  return new CubicSegment(p[0], p[1], p[2], p[3], color);
}

LPoint2 LinearSegment::
point(PN_stdfloat param) const {
  return msdf::mix(p[0], p[1], param);
}

LPoint2 QuadraticSegment::
point(PN_stdfloat param) const {
  return msdf::mix(msdf::mix(p[0], p[1], param), msdf::mix(p[1], p[2], param), param);
}

LPoint2 CubicSegment::
point(PN_stdfloat param) const {
  LVector2 p12 = msdf::mix(p[1], p[2], param);
  return msdf::mix(msdf::mix(msdf::mix(p[0], p[1], param), (LPoint2) p12, param), msdf::mix((LPoint2) p12, msdf::mix(p[2], p[3], param), param), param);
}

LVector2 LinearSegment::
direction(PN_stdfloat param) const {
  return p[1] - p[0];
}

LVector2 QuadraticSegment::
direction(PN_stdfloat param) const {
  return msdf::mix(p[1] - p[0], p[2] - p[1], param);
}

LVector2 CubicSegment::
direction(PN_stdfloat param) const {
  LVector2 tangent = msdf::mix(msdf::mix(p[1] - p[0], p[2] - p[1], param), msdf::mix(p[2] - p[1], p[3] - p[2], param), param);
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

SignedDistance LinearSegment::
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
      return SignedDistance(ortho_distance, 0);
  }
  LVector2 ab_n = ab.normalized(), eq_n = eq.normalized();
  non_zero_vector(ab_n);
  non_zero_vector(eq_n);
  return SignedDistance(msdf::non_zero_sign(cross(aq, ab)) * endpoint_distance, fabs(dot(ab_n, eq_n)));
}

SignedDistance QuadraticSegment::
signed_distance(LPoint2 origin, PN_stdfloat &param) const {
  LVector2 qa = p[0] - origin;
  LVector2 ab = p[1] - p[0];
  LVector2 br = p[0] + p[2] - p[1] - p[1];
  PN_stdfloat a = dot(br, br);
  PN_stdfloat b = 3 * dot(ab, br);
  PN_stdfloat c = 2 * dot(ab, ab) + dot(qa, br);
  PN_stdfloat d = dot(qa, ab);
  PN_stdfloat t[3];
  int solutions = solve_cubic(t, a, b, c, d);

  PN_stdfloat min_distance = msdf::non_zero_sign(cross(ab, qa)) * qa.length(); // distance from A
  param = -dot(qa, ab) / dot(ab, ab);
  {
    PN_stdfloat distance = msdf::non_zero_sign(cross(p[2] - p[1], p[2] - origin));
    distance *= (p[2] - origin).length(); // distance from B
    if (fabs(distance) < fabs(min_distance)) {
      min_distance = distance;
      param = dot(origin - p[1], p[2] - p[1]) / dot(p[2] - p[1], p[2] - p[1]);
    }
  }
  for (int i = 0; i < solutions; ++i) {
    if (t[i] > 0 && t[i] < 1) {
      LPoint2 endpoint = p[0] + 2 * t[i] * ab + t[i] * t[i] * br;
      PN_stdfloat distance = msdf::non_zero_sign(cross(p[2] - p[0], endpoint - origin));
      distance *= (endpoint - origin).length();
      if (fabs(distance) <= fabs(min_distance)) {
        min_distance = distance;
        param = t[i];
      }
    }
  }

  if (param >= 0 && param <= 1) {
    return SignedDistance(min_distance, 0);
  }
  if (param < 0.5) {
    LVector2 ab_n = ab.normalized(), qa_n = qa.normalized();
    non_zero_vector(ab_n);
    non_zero_vector(qa_n);
    return SignedDistance(min_distance, fabs(dot(ab_n, qa_n)));
  }
  else {
    LVector2 ab_n = (p[2] - p[1]).normalized(), qa_n = (p[2] - origin).normalized();
    non_zero_vector(ab_n);
    non_zero_vector(qa_n);
    return SignedDistance(min_distance, fabs(dot(ab_n, qa_n)));
  }
}

SignedDistance CubicSegment::
signed_distance(LPoint2 origin, PN_stdfloat &param) const {
  LVector2 qa = p[0] - origin;
  LVector2 ab = p[1] - p[0];
  LVector2 br = p[2] - p[1] - ab;
  LVector2 as = (p[3] - p[2]) - (p[2] - p[1]) - br;

  LVector2 ep_dir = direction(0);
  PN_stdfloat min_distance = msdf::non_zero_sign(cross(ep_dir, qa)) * qa.length(); // distance from A
  param = -dot(qa, ep_dir) / dot(ep_dir, ep_dir);
  {
    ep_dir = direction(1);
    PN_stdfloat distance = msdf::non_zero_sign(cross(ep_dir, p[3] - origin));
    distance *= (p[3] - origin).length(); // distance from B
    if (fabs(distance) < fabs(min_distance)) {
      min_distance = distance;
      param = dot(origin + ep_dir - p[3], ep_dir) / dot(ep_dir, ep_dir);
    }
  }
  // Iterative minimum distance search
  for (int i = 0; i <= MSDFGEN_CUBIC_SEARCH_STARTS; ++i) {
    PN_stdfloat t = (PN_stdfloat) i / MSDFGEN_CUBIC_SEARCH_STARTS;
    for (int step = 0;; ++step) {
      LVector2 qpt = point(t) - origin;
      PN_stdfloat distance = msdf::non_zero_sign(cross(direction(t), qpt)) * qpt.length();
      if (fabs(distance) < fabs(min_distance)) {
        min_distance = distance;
        param = t;
      }
      if (step == MSDFGEN_CUBIC_SEARCH_STEPS) {
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
    return SignedDistance(min_distance, 0);
  }
  if (param < 0.5) {
    LVector2 dir_n = direction(0).normalized(), qa_n = qa.normalized();
    non_zero_vector(dir_n);
    non_zero_vector(qa_n);
    return SignedDistance(min_distance, fabs(dot(dir_n, qa_n)));
  }
  else {
    LVector2 dir_n = direction(1).normalized();
    LVector2 qa_n = (p[3]-origin).normalized();
    non_zero_vector(dir_n);
    non_zero_vector(qa_n);
    return SignedDistance(min_distance, fabs(dot(dir_n, qa_n)));
  }
}

static void point_bounds(LPoint2 p, PN_stdfloat &l, PN_stdfloat &b, PN_stdfloat &r, PN_stdfloat &t) {
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

void LinearSegment::
bounds(PN_stdfloat &l, PN_stdfloat &b, PN_stdfloat &r, PN_stdfloat &t) const {
  point_bounds(p[0], l, b, r, t);
  point_bounds(p[1], l, b, r, t);
}

void QuadraticSegment::
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

void CubicSegment::
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

void LinearSegment::
move_start_point(LPoint2 to) {
    p[0] = to;
}

void QuadraticSegment::
move_start_point(LPoint2 to) {
  LVector2 orig_s_dir = p[0] - p[1];
  LPoint2 orig_p1 = p[1];
  p[1] += cross(p[0] - p[1], to - p[0]) / cross(p[0] - p[1], p[2] - p[1]) * (p[2] - p[1]);
  p[0] = to;
  if (dot(orig_s_dir, p[0] - p[1]) < 0) {
    p[1] = orig_p1;
  }
}

void CubicSegment::
move_start_point(LPoint2 to) {
  p[1] += to - p[0];
  p[0] = to;
}

void LinearSegment::
move_end_point(LPoint2 to) {
  p[1] = to;
}

void QuadraticSegment::
move_end_point(LPoint2 to) {
  LVector2 orig_e_dir = p[2] - p[1];
  LPoint2 orig_p1 = p[1];
  p[1] += cross(p[2] - p[1], to - p[2]) / cross(p[2] - p[1], p[0] - p[1]) * (p[0] - p[1]);
  p[2] = to;
  if (dot(orig_e_dir, p[2] - p[1]) < 0) {
    p[1] = orig_p1;
  }
}

void CubicSegment::
move_end_point(LPoint2 to) {
  p[2] += to - p[3];
  p[3] = to;
}

void LinearSegment::
split_in_thirds(EdgeSegment *&part1, EdgeSegment *&part2, EdgeSegment *&part3) const {
  part1 = new LinearSegment(p[0], point(1 / 3.0), color);
  part2 = new LinearSegment(point(1 / 3.0), point(2 / 3.0), color);
  part3 = new LinearSegment(point(2 / 3.0), p[1], color);
}

void QuadraticSegment::
split_in_thirds(EdgeSegment *&part1, EdgeSegment *&part2, EdgeSegment *&part3) const {
  part1 = new QuadraticSegment(p[0], msdf::mix(p[0], p[1], 1 / 3.0), point(1 / 3.0), color);
  part2 = new QuadraticSegment(point(1 / 3.0),
    msdf::mix(msdf::mix(p[0], p[1], 5 / 9.0), msdf::mix(p[1], p[2], 4 / 9.0), 0.5), point(2 / 3.0), color);
  part3 = new QuadraticSegment(point(2 / 3.0), msdf::mix(p[1], p[2], 2 / 3.0), p[2], color);
}

void CubicSegment::
split_in_thirds(EdgeSegment *&part1, EdgeSegment *&part2, EdgeSegment *&part3) const {
  part1 = new CubicSegment(p[0], p[0] == p[1] ? p[0] : msdf::mix(p[0], p[1], 1 / 3.0),
    msdf::mix(msdf::mix(p[0], p[1], 1 / 3.0), msdf::mix(p[1], p[2], 1 / 3.0), 1 / 3.0), point(1 / 3.0), color);
  part2 = new CubicSegment(point(1 / 3.0),
    msdf::mix(msdf::mix(msdf::mix(p[0], p[1], 1 / 3.0), msdf::mix(p[1], p[2], 1 / 3.0), 1 / 3.0),
      msdf::mix(msdf::mix(p[1], p[2], 1 / 3.0), msdf::mix(p[2], p[3], 1 / 3.0), 1 / 3.0), 2 / 3.0),
    msdf::mix(msdf::mix(msdf::mix(p[0], p[1], 2 / 3.0), msdf::mix(p[1], p[2], 2 / 3.0), 2 / 3.0),
      msdf::mix(msdf::mix(p[1], p[2], 2 / 3.0), msdf::mix(p[2], p[3], 2 / 3.0), 2 / 3.0), 1 / 3.0),
    point(2 / 3.0), color);
  part3 = new CubicSegment(point(2 / 3.0), msdf::mix(msdf::mix(p[1], p[2], 2 / 3.0),
    msdf::mix(p[2], p[3], 2 / 3.0), 2 / 3.0), p[2] == p[3] ? p[3] : msdf::mix(p[2], p[3], 2 / 3.0), p[3], color);
}