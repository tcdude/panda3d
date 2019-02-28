#pragma once
#include "numeric_types.h"

// ax^2 + bx + c = 0
int solve_quadratic(PN_stdfloat x[2], PN_stdfloat a, PN_stdfloat b, PN_stdfloat c);

// ax^3 + bx^2 + cx + d = 0
int solve_cubic(PN_stdfloat x[3], PN_stdfloat a, PN_stdfloat b, PN_stdfloat c, PN_stdfloat d);

int solve_cubic_normed(PN_stdfloat *x, PN_stdfloat a, PN_stdfloat b, PN_stdfloat c);