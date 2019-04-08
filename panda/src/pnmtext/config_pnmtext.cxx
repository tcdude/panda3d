/**
 * PANDA 3D SOFTWARE
 * Copyright (c) Carnegie Mellon University.  All rights reserved.
 *
 * All use of this software is subject to the terms of the revised BSD
 * license.  You should have received a copy of this license along
 * with this source code in a file named "LICENSE."
 *
 * @file config_pnmtext.cxx
 * @author drose
 * @date 2003-09-08
 */

#include "config_pnmtext.h"

#include "dconfig.h"
#include "freetypeFace.h"

#if !defined(CPPPARSER) && !defined(LINK_ALL_STATIC) && !defined(BUILDING_PANDA_PNMTEXT)
  #error Buildsystem error: BUILDING_PANDA_PNMTEXT not defined
#endif

Configure(config_pnmtext);
NotifyCategoryDef(pnmtext, "");

ConfigureFn(config_pnmtext) {
  init_libpnmtext();
}

ConfigVariableDouble text_point_size
("text-point-size", 10.0);
ConfigVariableDouble text_pixels_per_unit
("text-pixels-per-unit", 40.0);
ConfigVariableDouble text_scale_factor
("text-scale-factor", 2.0);
ConfigVariableBool text_native_antialias
("text-native-antialias", true);
ConfigVariableInt text_msdf_cubic_search_starts
("text-msdf-cubic-search-starts", 4,
 PRC_DESC("Increase for higher precision in iterative search for closest point"
          "on a cubic Bezier curve."));
ConfigVariableInt text_msdf_cubic_search_steps
("text-msdf-cubic-search-steps", 4,
 PRC_DESC("Increase for higher precision in iterative search for closest point"
          "on a cubic Bezier curve."));
ConfigVariableDouble text_msdf_edge_threshold
("text-msdf-edge-threshold", 1.001,
 PRC_DESC("Threshold value to perform error correction in MSDF generation."));

/**
 * Initializes the library.  This must be called at least once before any of
 * the functions or classes in this library can be used.  Normally it will be
 * called by the static initializers and need not be called explicitly, but
 * special cases exist.
 */
void
init_libpnmtext() {
  static bool initialized = false;
  if (initialized) {
    return;
  }
  initialized = true;

  FreetypeFace::init_type();
}
