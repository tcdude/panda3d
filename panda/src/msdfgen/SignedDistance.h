
#pragma once

/// Represents a signed distance and alignment, which together can be compared to uniquely determine the closest edge segment.
class SignedDistance {

public:
  static const SignedDistance INFINITE;

  PN_stdfloat distance;
  PN_stdfloat dot;

  SignedDistance();
  SignedDistance(PN_stdfloat dist, PN_stdfloat d);

  friend bool operator<(SignedDistance a, SignedDistance b);
  friend bool operator>(SignedDistance a, SignedDistance b);
  friend bool operator<=(SignedDistance a, SignedDistance b);
  friend bool operator>=(SignedDistance a, SignedDistance b);

};