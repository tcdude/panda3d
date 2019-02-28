
#pragma once

#include "Shape.h"

/** Assigns colors to edges of the shape in accordance to the multi-channel distance field technique.
 *  May split some edges if necessary.
 *  angleThreshold specifies the maximum angle (in radians) to be considered a corner, for example 3 (~172 degrees).
 *  Values below 1/2 PI will be treated as the external angle.
 */
void edge_coloring_simple(Shape &shape, PN_stdfloat angle_threshold, unsigned long long seed = 0);