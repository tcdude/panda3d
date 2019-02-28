
#include "SignedDistance.h"

#include <cmath>
#include <limits>

const SignedDistance SignedDistance::INFINITE(-std::numeric_limits<PN_stdfloat>::max(), 1);

SignedDistance::SignedDistance() : distance(-std::numeric_limits<PN_stdfloat>::max()), dot(1) { }

SignedDistance::SignedDistance(PN_stdfloat dist, PN_stdfloat d) : distance(dist), dot(d) { }

bool operator<(SignedDistance a, SignedDistance b) {
  return fabs(a.distance) < fabs(b.distance)
         || (fabs(a.distance) == fabs(b.distance) && a.dot < b.dot);
}

bool operator>(SignedDistance a, SignedDistance b) {
  return fabs(a.distance) > fabs(b.distance)
         || (fabs(a.distance) == fabs(b.distance) && a.dot > b.dot);
}

bool operator<=(SignedDistance a, SignedDistance b) {
  return fabs(a.distance) < fabs(b.distance)
         || (fabs(a.distance) == fabs(b.distance) && a.dot <= b.dot);
}

bool operator>=(SignedDistance a, SignedDistance b) {
  return fabs(a.distance) > fabs(b.distance)
         || (fabs(a.distance) == fabs(b.distance) && a.dot >= b.dot);
}