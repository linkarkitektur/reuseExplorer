// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

// Qt-free logic of the Rediger graph canvas: zoom limits, the view
// projection and yaw, heading ticks, the A/B slot picker.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <rux_qt/graph_view.hpp>

#include <array>
#include <cmath>
#include <vector>

using namespace rux::qt;
using Catch::Approx;

TEST_CASE("ZoomLimits_ScaleWithTheGraphAndTheViewport",
          "[rux_qt][graphview][zoom]") {
  const auto z = zoom_limits(50.0, 1000);
  CHECK(z.min_units_per_px == Approx(kMinUnitsPerPx));
  CHECK(z.max_units_per_px == Approx(50.0 * 4.0 / 1000.0));
  // A tiny graph still allows a 1 m view; a zero viewport does not divide by 0.
  CHECK(zoom_limits(0.0, 1000).max_units_per_px == Approx(4.0 / 1000.0));
  CHECK(zoom_limits(10.0, 0).max_units_per_px == Approx(40.0));
}

TEST_CASE("ClampZoomFactor_StopsAtTheLimits", "[rux_qt][graphview][zoom]") {
  const ZoomLimits z{0.001, 0.2};
  CHECK(clamp_zoom_factor(0.01, 2.0, z) == Approx(2.0));  // inside: as asked
  CHECK(clamp_zoom_factor(0.002, 4.0, z) == Approx(2.0)); // lands on min
  CHECK(clamp_zoom_factor(0.1, 0.25, z) == Approx(0.5));  // lands on max
  CHECK(clamp_zoom_factor(0.0005, 1.0, z) ==
        Approx(0.5));                                   // already past: back in
  CHECK(clamp_zoom_factor(0.0, 2.0, z) == Approx(1.0)); // degenerate: no-op
  CHECK(clamp_zoom_factor(0.01, -1.0, z) == Approx(1.0));
}
