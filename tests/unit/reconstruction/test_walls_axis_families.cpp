// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// W3.1 axis families (reusex-z1v.5.1): the directions walls run in, from
// weighted plane normals.

#include <reusex/core/solid_model.hpp>
#include <reusex/reconstruction/walls.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>
#include <vector>

using Catch::Matchers::WithinAbs;
using reusex::geometry::axis_families;
using reusex::geometry::AxisFamily;
using reusex::geometry::WallOptions;
using reusex::geometry::WeightedNormal;

namespace {

constexpr double kDeg = std::numbers::pi / 180.0;

WeightedNormal at(double deg, double w) {
  return {Eigen::Vector2d(std::cos(deg * kDeg), std::sin(deg * kDeg)), w};
}

/// Undirected angle between two lines [deg], 0..90.
double line_gap(double a, double b) {
  double d = std::fmod(std::abs(a - b), 180.0);
  return std::min(d, 180.0 - d);
}

} // namespace

TEST_CASE("AxisFamilies_ManhattanRotated_ReturnsTwoFamiliesAt90",
          "[walls][axis_families]") {
  // A building rotated 50.8 deg off the grid, both normal signs present,
  // +-1 deg of scatter.
  std::vector<WeightedNormal> n;
  for (const double base : {50.8, 140.8, 230.8, 320.8})
    for (const double jitter : {-1.0, -0.4, 0.0, 0.5, 1.0})
      n.push_back(at(base + jitter, 100.0 + 10.0 * std::abs(jitter)));

  const auto f = axis_families(n);
  REQUIRE(f.size() == 2);
  CHECK(f[0].dominant);
  CHECK(f[1].dominant);
  CHECK(f[0].index == 0);
  CHECK(f[1].index == 1);
  CHECK_THAT(f[0].angle_deg, WithinAbs(50.8, 0.3));
  CHECK_THAT(f[1].angle_deg, WithinAbs(140.8, 0.3));
  CHECK_THAT(std::abs(f[0].d.dot(f[1].d)), WithinAbs(0.0, 1e-9));
  CHECK_THAT(f[0].d.norm(), WithinAbs(1.0, 1e-12));
  CHECK_THAT(f[0].weight + f[1].weight, WithinAbs(2116.0, 1e-6));
}

TEST_CASE("AxisFamilies_OneWallAt30Degrees_ReturnsThreeFamilies",
          "[walls][axis_families]") {
  std::vector<WeightedNormal> n;
  for (const double base : {0.0, 90.0, 180.0, 270.0})
    n.push_back(at(base, 1000.0));
  // One wall at 30 deg (its normal at 120 deg), both faces.
  n.push_back(at(120.0, 300.0));
  n.push_back(at(300.0, 300.0));

  const auto f = axis_families(n);
  REQUIRE(f.size() == 3);
  CHECK_THAT(line_gap(f[0].angle_deg, 0.0), WithinAbs(0.0, 0.5));
  CHECK_THAT(line_gap(f[1].angle_deg, 90.0), WithinAbs(0.0, 0.5));
  CHECK_FALSE(f[2].dominant);
  CHECK(f[2].index == 2);
  CHECK_THAT(f[2].angle_deg, WithinAbs(120.0, 1e-6));
  CHECK_THAT(f[2].weight, WithinAbs(600.0, 1e-6));
}

TEST_CASE("AxisFamilies_MinorCluster_BelowShareIsNoFamily",
          "[walls][axis_families]") {
  std::vector<WeightedNormal> n;
  for (const double base : {0.0, 90.0, 180.0, 270.0})
    n.push_back(at(base, 1000.0));
  n.push_back(at(30.0, 50.0)); // 50 / 4050 = 1.2 % < 5 %

  const auto f = axis_families(n);
  CHECK(f.size() == 2);
}

TEST_CASE("AxisFamilies_ManyExtraClusters_CappedAtMaxFamilies",
          "[walls][axis_families]") {
  std::vector<WeightedNormal> n;
  for (const double base : {0.0, 90.0})
    n.push_back(at(base, 1000.0));
  n.push_back(at(20.0, 300.0));
  n.push_back(at(40.0, 400.0));
  n.push_back(at(65.0, 200.0));

  WallOptions o;
  const auto f = axis_families(n, o);
  REQUIRE(f.size() == 4); // 2 + max_families (2), heaviest extras first
  CHECK_THAT(f[2].angle_deg, WithinAbs(40.0, 1e-6));
  CHECK_THAT(f[3].angle_deg, WithinAbs(20.0, 1e-6));

  o.max_families = 0;
  CHECK(axis_families(n, o).size() == 2);
}

TEST_CASE("AxisFamilies_NoNormals_ReturnsEmpty", "[walls][axis_families]") {
  CHECK(axis_families(std::vector<WeightedNormal>{}).empty());
  CHECK(axis_families({at(10.0, 0.0)}).empty());
}

TEST_CASE("AxisFamilies_InputOrder_DoesNotChangeResult",
          "[walls][axis_families]") {
  std::vector<WeightedNormal> n = {at(12.0, 5.0),  at(101.0, 7.0),
                                   at(193.0, 3.0), at(280.0, 4.0),
                                   at(55.0, 4.0),  at(236.0, 3.0)};
  const auto a = axis_families(n);
  std::reverse(n.begin(), n.end());
  const auto b = axis_families(n);
  REQUIRE(a.size() == b.size());
  for (size_t i = 0; i < a.size(); ++i) {
    CHECK_THAT(a[i].angle_deg, WithinAbs(b[i].angle_deg, 1e-9));
    CHECK_THAT(a[i].weight, WithinAbs(b[i].weight, 1e-9));
  }
}

TEST_CASE("AxisFamilies_FromPlaneClasses_UsesWallPlanesOfTheStoreyInItsFrame",
          "[walls][axis_families]") {
  using reusex::core::PlaneClassRecord;
  using reusex::core::StoreyRecord;
  // Four wall planes on storey 0 at 30 deg (world), a clutter plane at 70 deg
  // and a wall plane of storey 1 at 10 deg that must not vote.
  pcl::PointCloud<pcl::Normal> normals;
  std::vector<PlaneClassRecord> planes;
  auto add = [&](double deg, const char *role, int storey, int points,
                 const char *orientation = "vertical") {
    pcl::Normal pn;
    pn.normal_x = static_cast<float>(std::cos(deg * kDeg));
    pn.normal_y = static_cast<float>(std::sin(deg * kDeg));
    pn.normal_z = 0.0f;
    normals.push_back(pn);
    PlaneClassRecord r;
    r.plane_label = static_cast<int>(normals.size());
    r.orientation = orientation;
    r.role = role;
    r.storey = storey;
    r.point_count = points;
    planes.push_back(r);
  };
  add(30.0, "wall", 0, 500);
  add(120.0, "wall", 0, 400);
  add(210.0, "wall", 0, 300);
  add(300.0, "wall", 0, 200);
  add(70.0, "clutter", 0, 300);
  add(10.0, "wall", 1, 9000);
  add(0.0, "floor", 0, 9000, "horizontal");

  // The storey's level frame is rotated 10 deg about z: world 30 deg is
  // level 40 deg.
  StoreyRecord s;
  s.idx = 0;
  const double c = std::cos(10.0 * kDeg), sn = std::sin(10.0 * kDeg);
  s.level_transform = {c, -sn, 0, 1.0, sn, c, 0, 2.0, 0, 0, 1, 0.5, 0, 0, 0, 1};

  const auto f = axis_families(planes, normals, s);
  REQUIRE(f.size() == 2);
  CHECK_THAT(f[0].angle_deg, WithinAbs(40.0, 1e-4));
  CHECK_THAT(f[0].weight + f[1].weight, WithinAbs(1400.0, 1e-6));

  // Without wall-only, the clutter plane at 70 deg (level 80) is an extra
  // family of its own.
  WallOptions all;
  all.wall_role_only = false;
  const auto g = axis_families(planes, normals, s, all);
  REQUIRE(g.size() == 3);
  CHECK_THAT(g[2].angle_deg, WithinAbs(80.0, 1e-4));

  // A plane label with no normal is an input error.
  planes.back().plane_label = 99;
  planes.back().orientation = "vertical";
  planes.back().role = "wall";
  CHECK_THROWS_AS(axis_families(planes, normals, s), std::invalid_argument);
}

TEST_CASE("AxisFamilies_HeavyObliqueWing_DoesNotPullTheDominantPair",
          "[walls][axis_families]") {
  // A plain 4-theta mean of all normals would land 6.7 deg off the grid;
  // seeding the pair at the weighted mode keeps it on the grid.
  std::vector<WeightedNormal> n = {at(0.0, 1000.0), at(90.0, 1000.0),
                                   at(30.0, 450.0), at(210.0, 450.0)};
  const auto f = axis_families(n);
  REQUIRE(f.size() == 3);
  CHECK_THAT(line_gap(f[0].angle_deg, 0.0), WithinAbs(0.0, 1e-6));
  CHECK_THAT(f[2].angle_deg, WithinAbs(30.0, 1e-6));
}
