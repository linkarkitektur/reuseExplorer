// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <reusex/geometry/level_frame.hpp>
#include <reusex/segmentation/level.hpp>
#include <reusex/types/point_types.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <Eigen/Geometry>

#include <cmath>
#include <random>
#include <stdexcept>

using namespace reusex;
using namespace reusex::geometry;
using Catch::Approx;

namespace {

constexpr double kDeg = M_PI / 180.0;

/// Synthetic scan builder: axis-aligned planar patches in a "true" level frame
/// (z up, floor of storey 0 at z = 0), then moved to world by `pose`.
struct Scene {
  Eigen::Affine3d pose = Eigen::Affine3d::Identity();
  double noise = 0.0; ///< sigma [m] along the patch normal
  std::mt19937 rng{7};

  struct Patch {
    std::vector<Eigen::Vector3d> pts;
    Eigen::Vector3d normal;
  };
  std::vector<Patch> patches;

  /// Horizontal rectangle [x0,x1]x[y0,y1] at height z. `down` flips the
  /// stored plane normal (a ceiling's normal as segmentation may report it).
  void horizontal(double x0, double x1, double y0, double y1, double z,
                  bool down = false, double step = 0.1) {
    std::normal_distribution<double> nd(0.0, noise > 0 ? noise : 1.0);
    Patch p;
    for (double x = x0; x <= x1 + 1e-9; x += step)
      for (double y = y0; y <= y1 + 1e-9; y += step)
        p.pts.emplace_back(x, y, z + (noise > 0 ? nd(rng) : 0.0));
    p.normal = Eigen::Vector3d(0, 0, down ? -1 : 1);
    patches.push_back(std::move(p));
  }

  /// Vertical wall x = const, spanning y and z.
  void wall_x(double x, double y0, double y1, double z0, double z1,
              double step = 0.1) {
    Patch p;
    for (double y = y0; y <= y1 + 1e-9; y += step)
      for (double z = z0; z <= z1 + 1e-9; z += step)
        p.pts.emplace_back(x, y, z);
    p.normal = Eigen::Vector3d::UnitX();
    patches.push_back(std::move(p));
  }

  /// Fill the four input clouds; plane label k+1 for patch k.
  void build(Cloud &cloud, CloudL &labels, CloudLoc &centroids,
             CloudN &normals) const {
    for (size_t k = 0; k < patches.size(); ++k) {
      Eigen::Vector3d sum = Eigen::Vector3d::Zero();
      for (const auto &q : patches[k].pts) {
        const Eigen::Vector3d w = pose * q;
        PointT pt;
        pt.x = static_cast<float>(w.x());
        pt.y = static_cast<float>(w.y());
        pt.z = static_cast<float>(w.z());
        cloud.push_back(pt);
        LabelT l;
        l.label = static_cast<uint32_t>(k + 1);
        labels.push_back(l);
        sum += w;
      }
      const Eigen::Vector3d c = sum / double(patches[k].pts.size());
      LocT lc;
      lc.x = float(c.x());
      lc.y = float(c.y());
      lc.z = float(c.z());
      centroids.push_back(lc);
      const Eigen::Vector3d n = pose.linear() * patches[k].normal;
      NormalT nn;
      nn.normal_x = float(n.x());
      nn.normal_y = float(n.y());
      nn.normal_z = float(n.z());
      normals.push_back(nn);
    }
  }

  StoreysResult run(const LevelOptions &opt = {}) const {
    Cloud cloud;
    CloudL labels;
    CloudLoc centroids;
    CloudN normals;
    build(cloud, labels, centroids, normals);
    return detect_storeys(cloud, labels, centroids, normals, opt);
  }
};

/// A rigid pose that tilts the scan by `deg` about a horizontal axis and
/// moves it somewhere away from the origin.
Eigen::Affine3d tilted_pose(double deg) {
  Eigen::Affine3d T = Eigen::Affine3d::Identity();
  T.translate(Eigen::Vector3d(12.0, -7.0, 1.3));
  T.rotate(Eigen::AngleAxisd(deg * kDeg,
                             Eigen::Vector3d(1.0, 0.6, 0.0).normalized()));
  T.rotate(Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitZ()));
  return T;
}

double angle_deg(const std::array<double, 3> &a, const Eigen::Vector3d &b) {
  const Eigen::Vector3d va(a[0], a[1], a[2]);
  return std::acos(std::clamp(va.normalized().dot(b.normalized()), -1.0, 1.0)) /
         kDeg;
}

/// One storey: floor at 0, ceiling at `h`, two walls, 8 x 6 m.
void add_storey(Scene &s, double z0, double h, bool with_floor = true,
                bool with_ceiling = true) {
  if (with_floor)
    s.horizontal(0, 8, 0, 6, z0);
  if (with_ceiling)
    s.horizontal(0, 8, 0, 6, z0 + h, /*down=*/true);
  s.wall_x(0, 0, 6, z0 + 0.1, z0 + h - 0.1);
  s.wall_x(8, 0, 6, z0 + 0.1, z0 + h - 0.1);
}

} // namespace

TEST_CASE("detect_storeys: one storey tilted 1 deg recovers the vertical",
          "[segmentation][level][storeys]") {
  Scene s;
  s.pose = tilted_pose(1.0);
  add_storey(s, 0.0, 2.7);

  const auto r = s.run();
  const Eigen::Vector3d true_up = s.pose.linear() * Eigen::Vector3d::UnitZ();

  REQUIRE(r.storeys.size() == 1);
  CHECK(angle_deg(r.vertical, true_up) < 0.1);
  CHECK(r.horizontal_planes == 2);
  CHECK(r.levels == 2);

  const auto &st = r.storeys[0];
  CHECK(st.idx == 0);
  CHECK(st.source == "detected");
  CHECK(st.z_floor == Approx(0.0).margin(1e-9));
  CHECK(st.z_ceiling - st.z_floor == Approx(2.7).margin(0.02));

  // The stored transform takes a world floor point to level z = 0 and a
  // world ceiling point to level z = 2.7.
  const LevelFrame f = LevelFrame::from_record(st);
  CHECK(f.to_level(s.pose * Eigen::Vector3d(1.0, 5.0, 0.0)).z() ==
        Approx(0.0).margin(0.02));
  CHECK(f.to_level(s.pose * Eigen::Vector3d(7.0, 1.0, 2.7)).z() ==
        Approx(2.7).margin(0.02));
}

TEST_CASE("detect_storeys: two storeys tilted 1 deg",
          "[segmentation][level][storeys]") {
  Scene s;
  s.pose = tilted_pose(1.0);
  add_storey(s, 0.0, 2.7); // ceiling at 2.7
  add_storey(s, 3.0, 2.8); // slab 0.3 m, floor at 3.0, ceiling at 5.8

  const auto r = s.run();
  const Eigen::Vector3d true_up = s.pose.linear() * Eigen::Vector3d::UnitZ();

  REQUIRE(r.storeys.size() == 2);
  CHECK(angle_deg(r.vertical, true_up) < 0.1);
  CHECK(r.horizontal_planes == 4);

  CHECK(r.storeys[0].idx == 0);
  CHECK(r.storeys[1].idx == 1);
  CHECK(r.storeys[0].z_floor == Approx(0.0).margin(1e-9));
  CHECK(r.storeys[0].z_ceiling == Approx(2.7).margin(0.02));
  CHECK(r.storeys[1].z_floor == Approx(3.0).margin(0.02));
  CHECK(r.storeys[1].z_ceiling - r.storeys[1].z_floor ==
        Approx(2.8).margin(0.02));
  // Both storeys share one level frame (storey 0's floor at z = 0).
  CHECK(r.storeys[0].level_transform == r.storeys[1].level_transform);
}

TEST_CASE("detect_storeys: a large tilt is still measured, not assumed",
          "[segmentation][level][storeys]") {
  Scene s;
  s.pose = tilted_pose(5.0);
  add_storey(s, 0.0, 2.6);

  const auto r = s.run();
  const Eigen::Vector3d true_up = s.pose.linear() * Eigen::Vector3d::UnitZ();
  REQUIRE(r.storeys.size() == 1);
  CHECK(angle_deg(r.vertical, true_up) < 0.1);
  CHECK(r.storeys[0].z_ceiling == Approx(2.6).margin(0.02));
}

TEST_CASE("detect_storeys: missing ceiling defaults the height",
          "[segmentation][level][storeys]") {
  Scene s;
  s.pose = tilted_pose(0.5);
  add_storey(s, 0.0, 2.7, /*with_floor=*/true, /*with_ceiling=*/false);

  LevelOptions opt;
  opt.default_height_m = 2.5;
  const auto r = s.run(opt);

  REQUIRE(r.storeys.size() == 1);
  CHECK(r.levels == 1);
  CHECK(r.storeys[0].z_floor == Approx(0.0).margin(1e-9));
  CHECK(r.storeys[0].z_ceiling == Approx(2.5).margin(1e-9));
  CHECK_THAT(r.storeys[0].metadata,
             Catch::Matchers::ContainsSubstring("\"ceiling_assumed\":true"));
  // With the floor alone the vertical is the floor normal.
  const Eigen::Vector3d true_up = s.pose.linear() * Eigen::Vector3d::UnitZ();
  CHECK(angle_deg(r.vertical, true_up) < 0.1);
}

TEST_CASE("detect_storeys: noisy floor and a table top",
          "[segmentation][level][storeys]") {
  Scene s;
  s.pose = tilted_pose(1.0);
  s.noise = 0.01; // 1 cm sigma on the horizontal patches
  s.horizontal(0, 8, 0, 6, 0.0);
  // A big table: its level is above peak_min_share but only 0.75 m above
  // the floor, so it must not become a storey boundary.
  s.horizontal(1, 5, 1, 4, 0.75);
  s.horizontal(0, 8, 0, 6, 2.7, /*down=*/true);
  s.noise = 0.0;
  s.wall_x(0, 0, 6, 0.1, 2.6);
  s.wall_x(8, 0, 6, 0.1, 2.6);

  const auto r = s.run();
  const Eigen::Vector3d true_up = s.pose.linear() * Eigen::Vector3d::UnitZ();

  REQUIRE(r.storeys.size() == 1);
  CHECK(r.levels == 3);
  CHECK(angle_deg(r.vertical, true_up) < 0.1);
  CHECK(r.storeys[0].z_ceiling == Approx(2.7).margin(0.02));
  const LevelFrame f = LevelFrame::from_record(r.storeys[0]);
  CHECK(f.to_level(s.pose * Eigen::Vector3d(4.0, 3.0, 0.0)).z() ==
        Approx(0.0).margin(0.02));
}

TEST_CASE("detect_storeys: deterministic across runs",
          "[segmentation][level][storeys]") {
  Scene s;
  s.pose = tilted_pose(1.0);
  s.noise = 0.01;
  add_storey(s, 0.0, 2.7);
  add_storey(s, 3.0, 2.8);

  const auto a = s.run();
  const auto b = s.run();
  REQUIRE(a.storeys.size() == b.storeys.size());
  CHECK(a.vertical == b.vertical);
  for (size_t i = 0; i < a.storeys.size(); ++i) {
    CHECK(a.storeys[i].level_transform == b.storeys[i].level_transform);
    CHECK(a.storeys[i].z_floor == b.storeys[i].z_floor);
    CHECK(a.storeys[i].z_ceiling == b.storeys[i].z_ceiling);
    CHECK(a.storeys[i].metadata == b.storeys[i].metadata);
  }
}

TEST_CASE("detect_storeys: no horizontal plane still yields one row",
          "[segmentation][level][storeys]") {
  Scene s;
  s.wall_x(0, 0, 6, 0.0, 2.5);
  s.wall_x(8, 0, 6, 0.0, 2.5);

  const auto r = s.run();
  REQUIRE(r.storeys.size() == 1);
  CHECK(r.horizontal_planes == 0);
  CHECK(r.levels == 0);
  CHECK_THAT(r.storeys[0].metadata,
             Catch::Matchers::ContainsSubstring("no horizontal plane"));
}

TEST_CASE("detect_storeys: rejects malformed input",
          "[segmentation][level][storeys]") {
  Scene s;
  add_storey(s, 0.0, 2.7);
  Cloud cloud;
  CloudL labels;
  CloudLoc centroids;
  CloudN normals;
  s.build(cloud, labels, centroids, normals);

  SECTION("cloud / planes size mismatch") {
    labels.points.pop_back();
    labels.width = static_cast<uint32_t>(labels.size());
    CHECK_THROWS_AS(detect_storeys(cloud, labels, centroids, normals),
                    std::invalid_argument);
  }
  SECTION("centroids / normals size mismatch") {
    normals.points.pop_back();
    normals.width = static_cast<uint32_t>(normals.size());
    CHECK_THROWS_AS(detect_storeys(cloud, labels, centroids, normals),
                    std::invalid_argument);
  }
  SECTION("no plane") {
    CHECK_THROWS_AS(detect_storeys(cloud, labels, CloudLoc{}, CloudN{}),
                    std::invalid_argument);
  }
  SECTION("a label beyond the plane clouds") {
    labels.points[0].label = 999;
    CHECK_THROWS_AS(detect_storeys(cloud, labels, centroids, normals),
                    std::invalid_argument);
  }
}
