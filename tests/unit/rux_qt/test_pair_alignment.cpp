// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

// The pair panel's math (spec §5.2): B's manual offset about its centroid,
// keyboard nudges and the edge measurement T_A^-1 · ΔT · T_B. ICP information
// is slam::icp_edge_information's job and edge direction is canonicalised by
// the library upsert, so neither is tested here.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <rux_qt/pair_alignment.hpp>

#include <cmath>
#include <cstddef>
#include <optional>

using namespace rux::qt;
using Catch::Approx;

namespace {
Mat4 translate(double x, double y, double z) {
  return {1, 0, 0, x, 0, 1, 0, y, 0, 0, 1, z, 0, 0, 0, 1};
}
void check_mat(const Mat4 &a, const Mat4 &b) {
  for (std::size_t i = 0; i < 16; ++i) {
    INFO(i);
    CHECK(a[i] == Approx(b[i]).margin(1e-9));
  }
}
} // namespace

TEST_CASE("ManualOffset_RoundTripsThroughTheMatrix", "[rux_qt][pair]") {
  const ManualOffset o{0.2, -0.1, 0.05, 10.0, -5.0, 3.0};
  const Vec3 pivot{1.0, 2.0, 0.5};
  const ManualOffset back = offset_from_matrix(offset_matrix(o, pivot), pivot);
  CHECK(back.x == Approx(o.x));
  CHECK(back.y == Approx(o.y));
  CHECK(back.z == Approx(o.z));
  CHECK(back.yaw_deg == Approx(o.yaw_deg));
  CHECK(back.pitch_deg == Approx(o.pitch_deg));
  CHECK(back.roll_deg == Approx(o.roll_deg));
}

TEST_CASE("ManualOffset_RotatesAboutThePivot", "[rux_qt][pair]") {
  const Vec3 pivot{3.0, -1.0, 1.2};
  const Mat4 m = offset_matrix({0, 0, 0, 30.0, 0, 0}, pivot);
  const Vec3 p = transform_point(m, pivot);
  CHECK(p[0] == Approx(pivot[0]));
  CHECK(p[1] == Approx(pivot[1]));
  CHECK(p[2] == Approx(pivot[2]));
  check_mat(offset_matrix({}, pivot), mat4_identity());
  CHECK(ManualOffset{}.is_zero());
  CHECK_FALSE(ManualOffset{0, 0, 0, 0.1, 0, 0}.is_zero());
}

TEST_CASE("ManualOffset_Yaw90_MapsXAxisToYAxisAboutThePivot",
          "[rux_qt][pair]") {
  const Vec3 pivot{2.0, -3.0, 0.4};
  const Mat4 m = offset_matrix({0, 0, 0, 90.0, 0, 0}, pivot);
  const Vec3 px{pivot[0] + 1.0, pivot[1], pivot[2]};
  const Vec3 py{pivot[0], pivot[1] + 1.0, pivot[2]};
  const Vec3 mapped = transform_point(m, px);
  CHECK(mapped[0] == Approx(py[0]).margin(1e-9));
  CHECK(mapped[1] == Approx(py[1]).margin(1e-9));
  CHECK(mapped[2] == Approx(py[2]).margin(1e-9));
}

TEST_CASE("Nudge_FineAndCoarseSteps", "[rux_qt][pair]") {
  CHECK(nudge({}, NudgeKey::x_plus, false).x == Approx(kNudgeM));
  CHECK(nudge({}, NudgeKey::y_minus, true).y ==
        Approx(-kNudgeM * kNudgeCoarse));
  CHECK(nudge({}, NudgeKey::z_plus, false).z == Approx(kNudgeM));
  CHECK(nudge({}, NudgeKey::yaw_plus, true).yaw_deg ==
        Approx(kNudgeDeg * kNudgeCoarse));
}

TEST_CASE("EdgeMeasurement_IsTFromInverseDeltaTTo", "[rux_qt][pair]") {
  const Mat4 a = translate(1, 0, 0), b = translate(3, 0, 0);
  CHECK(edge_measurement(a, b, mat4_identity())[3] == Approx(2.0));
  CHECK(edge_measurement(a, b, translate(0.5, 0, 0))[3] == Approx(2.5));
}

TEST_CASE("EdgeMeasurement_WithDeltaBridgingRotatedPoses_IsIdentity",
          "[rux_qt][pair]") {
  // Semantic check (not the formula restated): rotate+translate both ends,
  // pick Delta T = TA . rigid_inverse(TB) (the world correction that makes B
  // coincide with A), and edge_measurement(TA, TB, Delta T) must collapse to
  // identity regardless of either pose's rotation.
  const Mat4 ta = offset_matrix({1.0, 2.0, 0.5, 15.0, -10.0, 5.0}, {0, 0, 0});
  const Mat4 tb = offset_matrix({-0.3, 0.4, 1.1, 40.0, 20.0, -8.0}, {0, 0, 0});
  const Mat4 delta = mat4_mul(ta, rigid_inverse(tb));
  check_mat(edge_measurement(ta, tb, delta), mat4_identity());
}

TEST_CASE("ThinPoints_KeepsAtMostTheCapEvenlySpread", "[rux_qt][pair]") {
  CloudPoints pts;
  for (int i = 0; i < 1000; ++i)
    pts.push_back({static_cast<float>(i), 0.0f, 0.0f});
  const CloudPoints few = thin_points(pts, 300);
  REQUIRE(few.size() <= 300);
  CHECK(few.size() >= 250); // a stride, not a truncation to a handful
  CHECK(few.front()[0] == 0.0f);
  // Spread over the whole cloud, not its first 300 points.
  CHECK(few.back()[0] >= 900.0f);
  // Under the cap: untouched.
  CHECK(thin_points(pts, 1000).size() == 1000);
  CHECK(thin_points(pts, 5000).size() == 1000);
  CHECK(thin_points({}, 10).empty());
  CHECK(thin_points(pts, 0).empty());
}

TEST_CASE("Centroid_OfPoints_AndOfNone", "[rux_qt][pair]") {
  const CloudPoints pts{{0, 0, 0}, {2, 4, 6}, {4, 2, 0}};
  const Vec3 c = centroid(pts);
  CHECK(c[0] == Approx(2.0));
  CHECK(c[1] == Approx(2.0));
  CHECK(c[2] == Approx(2.0));
  const Vec3 none = centroid({});
  CHECK(none[0] == 0.0);
  CHECK(none[1] == 0.0);
  CHECK(none[2] == 0.0);
}

TEST_CASE("Normalized_WrapsAnglesIntoMinus180To180", "[rux_qt][pair]") {
  const ManualOffset o = normalized({0, 0, 0, 190.0, -190.0, 540.0});
  CHECK(o.yaw_deg == Approx(-170.0));
  CHECK(o.pitch_deg == Approx(170.0));
  CHECK(o.roll_deg == Approx(180.0)); // (-180, 180]: +180 stays
  CHECK(normalized({0, 0, 0, -180.0, 0, 0}).yaw_deg == Approx(180.0));
  CHECK(normalized({0, 0, 0, 12.5, 0, 0}).yaw_deg == Approx(12.5));
}

TEST_CASE("Normalized_ClampsTranslationToTheFieldRange", "[rux_qt][pair]") {
  const ManualOffset o = normalized({75.0, -60.0, 0.25, 0, 0, 0});
  CHECK(o.x == Approx(kOffsetMaxM));
  CHECK(o.y == Approx(-kOffsetMaxM));
  CHECK(o.z == Approx(0.25));
}

TEST_CASE("Nudge_PastHalfATurn_Wraps", "[rux_qt][pair]") {
  ManualOffset o{0, 0, 0, 179.8, 0, 0};
  o = nudge(o, NudgeKey::yaw_plus, false); // 180.3 -> -179.7
  CHECK(o.yaw_deg == Approx(-179.7));
  ManualOffset far{kOffsetMaxM, 0, 0, 0, 0, 0};
  CHECK(nudge(far, NudgeKey::x_plus, true).x == Approx(kOffsetMaxM));
}

TEST_CASE("RotationAngle_IsTheTrueAngleNotTheEulerSum", "[rux_qt][pair]") {
  CHECK(rotation_angle_deg(mat4_identity()) == Approx(0.0).margin(1e-9));
  CHECK(rotation_angle_deg(offset_matrix({0, 0, 0, 30.0, 0, 0}, {1, 2, 3})) ==
        Approx(30.0));
  // Two 90° turns about different axes make a 120° rotation, not 180°.
  CHECK(rotation_angle_deg(offset_matrix({0, 0, 0, 90.0, 0, 90.0}, {})) ==
        Approx(120.0));
  CHECK(rotation_angle_deg(offset_matrix({0, 0, 0, 180.0, 0, 0}, {})) ==
        Approx(180.0));
}

TEST_CASE("IcpSeedUnchanged_OnlyTheSameSeedMayMoveB", "[rux_qt][pair]") {
  const Mat4 moved =
      offset_matrix({0.05, -0.03, 0.0, 2.0, 0.0, 0.0}, {1.0, 2.0, 0.5});
  // No guess then and none now: the result still applies.
  CHECK(icp_seed_unchanged(std::nullopt, std::nullopt));
  CHECK(icp_seed_unchanged(moved, moved));
  // B moved by hand, reset, or moved again while ICP ran: stale.
  CHECK_FALSE(icp_seed_unchanged(std::nullopt, moved));
  CHECK_FALSE(icp_seed_unchanged(moved, std::nullopt));
  Mat4 nudged = moved;
  nudged[3] += 1e-6;
  CHECK_FALSE(icp_seed_unchanged(moved, nudged));
}

TEST_CASE("IcpFitState_FollowsBsPlacementAfterIcp", "[rux_qt][pair]") {
  const Mat4 placed =
      offset_matrix({0.12, -0.04, 0.03, -3.4, -1.1, -0.5}, {1.0, 2.0, 0.5});
  const Mat4 moved =
      offset_matrix({0.2, 0.0, 0.0, 0.0, 0.0, 0.0}, {1.0, 2.0, 0.5});
  using S = IcpFitState;
  // Failed / dropped runs are what they are, whatever B does.
  CHECK(icp_fit_state(false, false, true, placed, moved) == S::failed);
  CHECK(icp_fit_state(true, true, true, placed, moved) == S::dropped);
  // Converged and B placed on the result: usable while B stays there.
  CHECK(icp_fit_state(true, false, true, placed, placed) == S::aligned);
  CHECK(icp_fit_state(true, false, true, placed, moved) == S::moved_since);
  CHECK(icp_fit_state(true, false, true, placed, std::nullopt) ==
        S::moved_since);
  // Converged but B could not be placed (overlay not ready): usable, and
  // the line must not claim B moved.
  CHECK(icp_fit_state(true, false, true, std::nullopt, std::nullopt) ==
        S::not_placed);
  CHECK(icp_fit_state(true, false, true, std::nullopt, moved) ==
        S::moved_since);
  // Not converged: B stayed at its guess; moving it afterwards outdates it.
  CHECK(icp_fit_state(true, false, false, moved, moved) == S::not_converged);
  CHECK(icp_fit_state(true, false, false, std::nullopt, moved) ==
        S::moved_since);
  CHECK(icp_fit_usable(S::aligned));
  CHECK(icp_fit_usable(S::not_placed));
  for (S s : {S::failed, S::dropped, S::not_converged, S::moved_since})
    CHECK_FALSE(icp_fit_usable(s));
}

TEST_CASE("PairEdgeToAdd_WithAUsableIcpFit_StagesIcpsBToAEdge",
          "[rux_qt][pair]") {
  IcpEdgeMeasurement icp;
  icp.edge_transform = translate(0.4, -0.1, 0.02);
  icp.information.fill(0.0);
  for (std::size_t i = 0; i < 3; ++i)
    icp.information[i * 7] = 400.0;
  icp.information[21] = 1000.0;
  icp.information[28] = 2000.0;
  icp.information[35] = 3000.0;
  const EdgeRecord e = pair_edge_to_add(
      3, 9, "loop_closure", translate(1, 0, 0), translate(2, 0, 0),
      offset_matrix({0.05, 0, 0, 0, 0, 0}, {0, 0, 0}), icp);
  // ICP ran B onto A: the edge is B -> A; the store canonicalises it.
  CHECK(e.key.from == 9);
  CHECK(e.key.to == 3);
  CHECK(e.key.type == "loop_closure");
  REQUIRE(e.transform.has_value());
  check_mat(*e.transform, icp.edge_transform);
  REQUIRE(e.information.has_value());
  CHECK(*e.information == icp.information);
  CHECK(e.source == "icp");
  CHECK(e.status == "confirmed");
  // The store's weight: the mean translational precision.
  CHECK(e.weight == Approx(2000.0));
}

TEST_CASE("PairEdgeToAdd_WithoutIcp_MeasuresTheManualPlacement",
          "[rux_qt][pair]") {
  const Mat4 pa = translate(1, 0, 0), pb = translate(2, 1, 0);
  const Mat4 delta = offset_matrix({0.1, 0, 0, 10, 0, 0}, {2, 1, 0});
  const EdgeRecord e =
      pair_edge_to_add(3, 9, "odometry", pa, pb, delta, std::nullopt);
  CHECK(e.key.from == 3);
  CHECK(e.key.to == 9);
  CHECK(e.key.type == "odometry");
  REQUIRE(e.transform.has_value());
  check_mat(*e.transform, edge_measurement(pa, pb, delta));
  CHECK_FALSE(e.information.has_value()); // the optimizer's default
  CHECK(std::isnan(e.weight));
  CHECK(e.source == "manual");
  CHECK(e.status == "confirmed");
  // No offset: the stored relative pose.
  const EdgeRecord stored =
      pair_edge_to_add(3, 9, "odometry", pa, pb, std::nullopt, std::nullopt);
  REQUIRE(stored.transform.has_value());
  check_mat(*stored.transform, translate(1, 1, 0));
}
