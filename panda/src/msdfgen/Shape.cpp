
#include "Shape.h"

Shape::
Shape() : inverseYAxis(false) { }

void Shape::
add_contour(const MSDFContour &contour) {
  contours.push_back(contour);
}

#ifdef MSDFGEN_USE_CPP11
void Shape::
add_contour(MSDFContour &&contour) {
  contours.push_back((MSDFContour &&) contour);
}
#endif

MSDFContour& Shape::
add_contour() {
  contours.resize(contours.size() + 1);
  return contours[contours.size() - 1];
}

bool Shape::
validate() const {
  for (const MSDFContour &contour : contours) {
    if (!contour.edges.empty()) {
      LPoint2 corner = (*(contour.edges.end() - 1))->point(1);
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

void Shape::
normalize() {
  for (MSDFContour &contour : contours)
    if (contour.edges.size() == 1) {
      EdgeSegment *parts[3] = { };
      contour.edges[0]->split_in_thirds(parts[0], parts[1], parts[2]);
      contour.edges.clear();
      contour.edges.push_back(EdgeHolder(parts[0]));
      contour.edges.push_back(EdgeHolder(parts[1]));
      contour.edges.push_back(EdgeHolder(parts[2]));
    }
}

void Shape::
bounds(PN_stdfloat &l, PN_stdfloat &b, PN_stdfloat &r, PN_stdfloat &t) const {
  for (const MSDFContour &contour : contours) {
    contour.bounds(l, b, r, t);
  }
}