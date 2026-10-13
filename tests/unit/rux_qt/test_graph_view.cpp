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
  CHECK(clamp_zoom_factor(0.01, 2.0, z) == Approx(2.0));   // inside: as asked
  CHECK(clamp_zoom_factor(0.002, 4.0, z) == Approx(2.0));  // lands on min
  CHECK(clamp_zoom_factor(0.1, 0.25, z) == Approx(0.5));   // lands on max
  CHECK(clamp_zoom_factor(0.0005, 1.0, z) == Approx(1.0)); // no step: no change
  CHECK(clamp_zoom_factor(0.0, 2.0, z) == Approx(1.0));    // degenerate: no-op
  CHECK(clamp_zoom_factor(0.01, -1.0, z) == Approx(1.0));
}

TEST_CASE("ClampZoomFactor_NeverReversesTheWheel",
          "[rux_qt][graphview][zoom]") {
  // A very wide viewport fits the graph by its height, past the zoom-out
  // limit: scrolling out must not zoom in, nor scrolling in zoom out.
  const ZoomLimits z{0.001, 0.2};
  CHECK(clamp_zoom_factor(0.5, 0.8, z) == Approx(1.0));
  CHECK(clamp_zoom_factor(0.0005, 1.25, z) == Approx(1.0));
  // Moving back towards the range takes the step as asked, no snap...
  CHECK(clamp_zoom_factor(0.5, 1.25, z) == Approx(1.25));
  CHECK(clamp_zoom_factor(0.0005, 0.8, z) == Approx(0.8));
  // ...and does not overshoot the far limit.
  CHECK(clamp_zoom_factor(0.5, 1000.0, z) == Approx(500.0));
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

TEST_CASE("SlotPicker_AlternatesAThenB", "[rux_qt][graphview][slots]") {
  SlotPicker s;
  CHECK(s.next() == Slot::a);
  CHECK(s.click(10, false) == Slot::a);
  CHECK(s.a() == 10);
  CHECK(s.next() == Slot::b);
  CHECK_FALSE(s.complete());
  CHECK(s.click(20, false) == Slot::b);
  CHECK(s.b() == 20);
  CHECK(s.complete());
  CHECK(s.next() == Slot::a);
  CHECK(s.click(30, false) == Slot::a); // the next click replaces A
  CHECK(s.a() == 30);
  CHECK(s.b() == 20);
}

TEST_CASE("SlotPicker_ShiftForcesB_AndTheNextClickIsA",
          "[rux_qt][graphview][slots]") {
  SlotPicker s;
  s.click(10, false);
  s.click(20, false);
  CHECK(s.click(40, true) == Slot::b);
  CHECK(s.b() == 40);
  CHECK(s.next() == Slot::a);
}

TEST_CASE("SlotPicker_TheSameFrameNeverFillsBothSlots",
          "[rux_qt][graphview][slots]") {
  SlotPicker s;
  s.click(10, false); // A = 10, next B
  CHECK_FALSE(s.click(10, false).has_value());
  CHECK(s.b() == -1);
  CHECK(s.next() == Slot::b); // still waiting for B
  CHECK_FALSE(s.click(10, true).has_value());
}

TEST_CASE("SlotPicker_ForgetMissing_KeepsAPairWhoseFramesStillExist",
          "[rux_qt][graphview][slots]") {
  // A project reload (after "Optimer") keeps the pair to re-inspect it.
  SlotPicker s;
  s.set_pair(5, 9);
  CHECK(s.forget_missing({1, 5, 7, 9}));
  CHECK(s.a() == 5);
  CHECK(s.b() == 9);
  // Only A chosen: kept, and the next click still fills B.
  SlotPicker half;
  half.click(7, false);
  CHECK(half.forget_missing({7}));
  CHECK(half.a() == 7);
  CHECK(half.next() == Slot::b);
}

TEST_CASE("SlotPicker_ForgetMissing_AVanishedFrameClearsThePair",
          "[rux_qt][graphview][slots]") {
  SlotPicker s;
  s.set_pair(5, 9);
  CHECK_FALSE(s.forget_missing({1, 5, 7}));
  CHECK(s.a() == -1);
  CHECK(s.b() == -1);
  CHECK(s.next() == Slot::a);
  SlotPicker none; // an empty pair survives anything
  CHECK(none.forget_missing({}));
}

TEST_CASE("SlotPicker_SetPair_FromAnEdge", "[rux_qt][graphview][slots]") {
  SlotPicker s;
  s.click(1, false);
  s.set_pair(5, 9);
  CHECK(s.a() == 5);
  CHECK(s.b() == 9);
  CHECK(s.next() == Slot::a);
  s.clear();
  CHECK(s.a() == -1);
  CHECK(s.b() == -1);
  CHECK(s.next() == Slot::a);
}

TEST_CASE("ReprojectAfterYaw_KeepsTheSameWorldPoint",
          "[rux_qt][graphview][projection]") {
  // Plan view: the dropped axis does not matter.
  const ViewProjection plan{ViewPlane::xy, 0.0};
  const Vec2 p = project_point(plan, 3.0, 1.0, 0.0);
  const Vec2 q = reproject_after_yaw(plan, 90.0, p);
  const Vec2 want = project_point({ViewPlane::xy, 90.0}, 3.0, 1.0, 0.0);
  CHECK(q.x == Approx(want.x));
  CHECK(q.y == Approx(want.y));
  // Elevations: lifted at the point's own depth, it lands on itself.
  for (ViewPlane plane : {ViewPlane::xz, ViewPlane::yz}) {
    const ViewProjection v{plane, 30.0};
    const Vec2 e = project_point(v, 2.0, 5.0, 1.5);
    const double d = view_depth(v, 2.0, 5.0, 1.5);
    const Vec2 r = reproject_after_yaw(v, 75.0, e, d);
    const Vec2 w = project_point({plane, 75.0}, 2.0, 5.0, 1.5);
    CHECK(r.x == Approx(w.x));
    CHECK(r.y == Approx(w.y));
  }
}

TEST_CASE("MedianStep_ConsecutiveNodesWithinAScan",
          "[rux_qt][graphview][ticks]") {
  const std::vector<Vec2> pts = {{0, 0}, {1, 0}, {3, 0}, {3, 0.5}};
  CHECK(median_step(pts, {}) == Approx(1.0));            // 1, 2, 0.5
  CHECK(median_step(pts, {1, 1, 2, 2}) == Approx(0.75)); // 1, 0.5: mean
  CHECK(median_step({{1, 1}}, {}) == Approx(0.0));
  CHECK(median_step({}, {}) == Approx(0.0));
}

TEST_CASE("HeadingTicks_OnlyOnceNodesAreATickApart",
          "[rux_qt][graphview][ticks]") {
  CHECK(heading_ticks_visible(0.1, 0.01, 7.0));        // 10 px apart
  CHECK_FALSE(heading_ticks_visible(0.05, 0.01, 7.0)); // 5 px: fur
  CHECK(heading_ticks_visible(0.0, 0.01, 7.0)); // one node, or all in one spot
  CHECK(heading_ticks_visible(0.05, 0.0, 7.0)); // degenerate zoom
}

TEST_CASE("FitAvoidingCorner_NoReserve_CentresAndFits",
          "[rux_qt][graphview][fit]") {
  // 10 x 5 content centred at (2, 3) in a 200 x 200 viewport: width binds.
  const FitFrame f = fit_avoiding_corner({2.0, 3.0, 10.0, 5.0}, 200, 200, 0, 0);
  CHECK(f.scale == Catch::Approx(20.0));
  CHECK(f.cx == Catch::Approx(2.0));
  CHECK(f.cy == Catch::Approx(3.0));
}

TEST_CASE("FitAvoidingCorner_TallCorner_ContentGoesBeside",
          "[rux_qt][graphview][fit]") {
  // A square graph in a wide viewport with a tall legend in the lower left:
  // beside it costs less scale than above it.
  const FitFrame side =
      fit_avoiding_corner({0.0, 0.0, 10.0, 10.0}, 400, 200, 100, 150);
  CHECK(side.scale == Catch::Approx(20.0)); // height still binds
  // The content's centre sits at the middle of the free 300 px on the
  // right: the view's centre is 50 px (2.5 units) left of it.
  CHECK(side.cx == Catch::Approx(-2.5));
  CHECK(side.cy == Catch::Approx(0.0));
}

TEST_CASE("FitAvoidingCorner_WideShortCorner_ContentGoesAbove",
          "[rux_qt][graphview][fit]") {
  // A wide graph in a square viewport with a short wide legend: above it.
  const FitFrame above =
      fit_avoiding_corner({0.0, 0.0, 20.0, 10.0}, 200, 200, 150, 40);
  CHECK(above.scale == Catch::Approx(10.0)); // width binds, nothing lost
  CHECK(above.cx == Catch::Approx(0.0));
  // Centred in the top 160 px: the view's centre is 20 px (2 units) below.
  CHECK(above.cy == Catch::Approx(2.0));
}

TEST_CASE("FitAvoidingCorner_CornerCostsOverHalfTheScale_PlainFit",
          "[rux_qt][graphview][fit]") {
  // A large legend in a small pane: avoiding it would shrink the graph to a
  // quarter (5 vs 20 px/unit); a plain fit, partly under the card, is
  // better than a speck.
  const FitFrame f =
      fit_avoiding_corner({1.0, 2.0, 10.0, 10.0}, 200, 200, 150, 150);
  CHECK(f.scale == Catch::Approx(20.0));
  CHECK(f.cx == Catch::Approx(1.0));
  CHECK(f.cy == Catch::Approx(2.0));
}

TEST_CASE("FitAvoidingCorner_DegenerateViewport_StaysFinite",
          "[rux_qt][graphview][fit]") {
  // A pane not laid out yet (0 x 0), with or without a legend, or content
  // with no extent: every fit is 0 — no division by it.
  for (const auto &[vw, vh, rw, rh, cw] : std::vector<std::array<double, 5>>{
           {0, 0, 0, 0, 10}, {0, 0, 120, 40, 10}, {200, 200, 50, 40, 0}}) {
    const FitFrame f = fit_avoiding_corner({1.0, 2.0, cw, cw}, vw, vh, rw, rh);
    INFO(vw << " x " << vh << ", corner " << rw << " x " << rh);
    CHECK(f.scale == Approx(1.0));
    CHECK(f.cx == Approx(1.0));
    CHECK(f.cy == Approx(2.0));
  }
}

TEST_CASE("FitAvoidingCorner_ReserveFillsViewport_IgnoresIt",
          "[rux_qt][graphview][fit]") {
  const FitFrame f =
      fit_avoiding_corner({0.0, 0.0, 10.0, 10.0}, 100, 100, 100, 100);
  CHECK(f.scale == Catch::Approx(10.0));
  CHECK(f.cx == Catch::Approx(0.0));
  CHECK(f.cy == Catch::Approx(0.0));
}
