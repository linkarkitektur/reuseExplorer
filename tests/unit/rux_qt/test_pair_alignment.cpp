// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

// The pair panel's math (spec §5.2): B's manual offset about its centroid,
// keyboard nudges and the edge measurement T_A^-1 · ΔT · T_B. ICP information
// is slam::icp_edge_information's job and edge direction is canonicalised by
// the library upsert, so neither is tested here.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <rux_qt/pair_alignment.hpp>

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
  const Mat4 d = offset_matrix({0.1, 0.2, 0, 20, 0, 0}, {3, 0, 0});
  check_mat(edge_measurement(a, b, d),
            mat4_mul(mat4_mul(rigid_inverse(a), d), b));
  check_mat(mat4_mul(rigid_inverse(d), d), mat4_identity());
}
