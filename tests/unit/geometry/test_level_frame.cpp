// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <reusex/geometry/level_frame.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>

using namespace reusex;
using namespace reusex::geometry;
using Catch::Approx;

namespace {
constexpr double kDeg = M_PI / 180.0;
} // namespace

TEST_CASE("make_level_frame: vertical maps to +z and floor point to origin",
          "[geometry][level_frame]") {
  const Eigen::Vector3d vertical =
      Eigen::AngleAxisd(1.0 * kDeg, Eigen::Vector3d(1, 1, 0).normalized()) *
      Eigen::Vector3d::UnitZ();
  const Eigen::Vector3d floor_point(4.0, -2.0, 1.3);

  const LevelFrame f = make_level_frame(vertical, floor_point);

  const Eigen::Vector3d o = f.to_level(floor_point);
  CHECK(o.norm() == Approx(0.0).margin(1e-12));

  // A point 2.5 m up the measured vertical is at level z = 2.5.
  const Eigen::Vector3d up = f.to_level(floor_point + 2.5 * vertical);
  CHECK(up.x() == Approx(0.0).margin(1e-12));
  CHECK(up.y() == Approx(0.0).margin(1e-12));
  CHECK(up.z() == Approx(2.5).margin(1e-12));

  // Round trip.
  const Eigen::Vector3d p(1.0, 2.0, 3.0);
  CHECK((f.to_world(f.to_level(p)) - p).norm() == Approx(0.0).margin(1e-12));
}

TEST_CASE("LevelFrame: from_record round-trips through to_array",
          "[geometry][level_frame]") {
  const Eigen::Vector3d vertical =
      (Eigen::Vector3d(0.01, -0.02, 1.0)).normalized();
  const LevelFrame f = make_level_frame(vertical, {1.0, 2.0, 3.0});

  core::StoreyRecord s;
  s.level_transform = f.to_array();
  const LevelFrame g = LevelFrame::from_record(s);

  const Eigen::Vector3d p(-3.0, 0.5, 7.0);
  CHECK((g.to_level(p) - f.to_level(p)).norm() == Approx(0.0).margin(1e-12));

  // A default record carries the identity.
  const LevelFrame id = LevelFrame::from_record(core::StoreyRecord{});
  CHECK((id.to_level(p) - p).norm() == Approx(0.0).margin(1e-15));
}

TEST_CASE("LevelFrame::world_z_at follows the frame origin's vertical",
          "[geometry][level_frame]") {
  // Untilted: world z = floor z + level z.
  const LevelFrame flat =
      make_level_frame(Eigen::Vector3d::UnitZ(), {5.0, 5.0, 1.25});
  CHECK(flat.world_z_at(0.0) == Approx(1.25));
  CHECK(flat.world_z_at(2.7) == Approx(3.95));

  // Tilted by 1 deg: the origin's vertical leans, so world z grows by
  // cos(1 deg) per level metre.
  const Eigen::Vector3d v =
      Eigen::AngleAxisd(1.0 * kDeg, Eigen::Vector3d::UnitX()) *
      Eigen::Vector3d::UnitZ();
  const LevelFrame tilted = make_level_frame(v, {0.0, 0.0, 2.0});
  CHECK(tilted.world_z_at(3.0) == Approx(2.0 + 3.0 * std::cos(1.0 * kDeg)));
}
