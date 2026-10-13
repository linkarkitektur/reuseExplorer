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

namespace {
std::array<double, 16> yaw_pose(double deg, double tx = 0, double ty = 0,
                                double tz = 0) {
  const double a = deg * 3.14159265358979323846 / 180.0;
  return {std::cos(a), -std::sin(a),
          0,           tx,
          std::sin(a), std::cos(a),
          0,           ty,
          0,           0,
          1,           tz,
          0,           0,
          0,           1};
}
// RTABMap's optical-from-base: optical z (forward) = base x, x = -base y,
// y = -base z.
const std::array<double, 16> kOpticalInBase = {0, 0,  1, 0, -1, 0, 0, 0,
                                               0, -1, 0, 0, 0,  0, 0, 1};
const std::array<double, 16> kIdentity = {1, 0, 0, 0, 0, 1, 0, 0,
                                          0, 0, 1, 0, 0, 0, 0, 1};
} // namespace

TEST_CASE("ProjectPoint_PlanesPickTwoAxesAfterTheYaw",
          "[rux_qt][graphview][projection]") {
  ViewProjection v;
  CHECK(project_point(v, 1, 2, 3).x == Approx(1));
  CHECK(project_point(v, 1, 2, 3).y == Approx(2));
  v.plane = ViewPlane::xz; // seen from -y: x right, z up
  CHECK(project_point(v, 1, 2, 3).x == Approx(1));
  CHECK(project_point(v, 1, 2, 3).y == Approx(3));
  v.plane = ViewPlane::yz; // seen from +x: y right, z up
  CHECK(project_point(v, 1, 2, 3).x == Approx(2));
  CHECK(project_point(v, 1, 2, 3).y == Approx(3));
  // The yaw turns the model counter-clockwise (seen from above) first.
  v = {ViewPlane::xy, 90.0};
  CHECK(project_point(v, 1, 0, 0).x == Approx(0).margin(1e-12));
  CHECK(project_point(v, 1, 0, 0).y == Approx(1));
  v.plane = ViewPlane::xz;
  CHECK(project_point(v, 1, 0, 5).x == Approx(0).margin(1e-12));
  CHECK(project_point(v, 1, 0, 5).y == Approx(5));
}

TEST_CASE("ViewPlane_KeysRoundTrip", "[rux_qt][graphview][projection]") {
  for (ViewPlane p : {ViewPlane::xy, ViewPlane::xz, ViewPlane::yz})
    CHECK(parse_view_plane(view_plane_key(p)) == p);
  CHECK_FALSE(parse_view_plane("zz").has_value());
}

TEST_CASE("CameraForward_UsesTheOpticalAxisThroughLocalTransform",
          "[rux_qt][graphview][heading]") {
  auto f = camera_forward(kIdentity, kOpticalInBase);
  CHECK(f[0] == Approx(1));
  CHECK(f[1] == Approx(0).margin(1e-12));
  f = camera_forward(yaw_pose(90), kOpticalInBase);
  CHECK(f[0] == Approx(0).margin(1e-12));
  CHECK(f[1] == Approx(1));
  // Identity local transform: the optical axis is the pose's own z.
  f = camera_forward(kIdentity, kIdentity);
  CHECK(f[2] == Approx(1));
}

TEST_CASE("HeadingInView_ProjectsTheForwardAxis",
          "[rux_qt][graphview][heading]") {
  const auto h = heading_in_view({ViewPlane::xy, 0.0}, {1, 0, 0});
  CHECK(h.visible);
  CHECK(h.dir.x == Approx(1));
  const auto r = heading_in_view({ViewPlane::xy, 90.0}, {1, 0, 0});
  CHECK(r.dir.y == Approx(1));
  const auto e = heading_in_view({ViewPlane::xz, 0.0}, {0.6, 0, 0.8});
  CHECK(e.visible);
  CHECK(e.dir.x == Approx(0.6));
  CHECK(e.dir.y == Approx(0.8));
}

TEST_CASE("HeadingInView_LookingAlongTheViewAxis_IsHidden",
          "[rux_qt][graphview][heading]") {
  // A camera looking straight down has no direction in the plan view.
  CHECK_FALSE(heading_in_view({ViewPlane::xy, 0.0}, {0, 0, -1}).visible);
  CHECK_FALSE(heading_in_view({ViewPlane::xy, 0.0}, {0.1, 0, -0.995}).visible);
  CHECK_FALSE(heading_in_view({ViewPlane::xy, 0.0}, {0, 0, 0}).visible);
}

TEST_CASE("AutoYaw_AlignsTheDominantAxisWithTheScreen",
          "[rux_qt][graphview][yaw]") {
  auto dir = [](double deg) {
    const double a = deg * 3.14159265358979323846 / 180.0;
    return std::array<double, 3>{std::cos(a), std::sin(a), 0.0};
  };
  CHECK(auto_yaw_deg({dir(30), dir(30), dir(31), dir(29)}) ==
        Approx(-30).margin(0.1));
  // Perpendicular corridors are one axis system (folded mod 90°).
  CHECK(auto_yaw_deg({dir(30), dir(120), dir(210), dir(-60)}) ==
        Approx(-30).margin(0.1));
  CHECK(auto_yaw_deg({dir(45)}) == Approx(45).margin(1e-9));
  CHECK(auto_yaw_deg({dir(-45)}) == Approx(45).margin(1e-9));
  // Nothing horizontal: no rotation.
  CHECK(auto_yaw_deg({}) == 0.0);
  CHECK(auto_yaw_deg({{0, 0, 1}, {0.1, 0, -1}}) == 0.0);
}

TEST_CASE("YawSetting_ParsesAndFormatsWithoutTheLocale",
          "[rux_qt][graphview][yaw]") {
  CHECK(parse_yaw_setting("12.5") == Approx(12.5));
  CHECK(parse_yaw_setting(" -190 ") == Approx(170));
  CHECK_FALSE(parse_yaw_setting("12,5").has_value());
  CHECK_FALSE(parse_yaw_setting("abc").has_value());
  CHECK_FALSE(parse_yaw_setting("nan").has_value());
  CHECK_FALSE(parse_yaw_setting("").has_value());
  CHECK(format_yaw_setting(12.54) == "12.5");
  CHECK(format_yaw_setting(190.0) == "-170.0");
  CHECK(format_yaw_setting(-0.04) == "0.0");
  CHECK(format_yaw_setting(180.0) == "180.0");
  CHECK(normalize_yaw_deg(-180.0) == Approx(180.0));
  CHECK(normalize_yaw_deg(720.0) == Approx(0.0));
}
