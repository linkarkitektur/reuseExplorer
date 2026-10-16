// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// W3.2 wall faces and runs from the offset profile (reusex-z1v.5.2).

#include "wall_synth.hpp"

#include <reusex/reconstruction/walls.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <reusex/core/solid_model.hpp>
#include <reusex/types/point_types.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <cmath>
#include <numbers>
#include <random>
#include <vector>

using Catch::Matchers::WithinAbs;
using reusex::geometry::AxisFamily;
using reusex::geometry::wall_faces;
using reusex::geometry::WallFace;
using reusex::geometry::WallOptions;
using reusex::geometry::WallPoints;
using namespace reusex::test_support::walls;

namespace {

std::vector<AxisFamily> grid_families() {
  AxisFamily x, y;
  x.index = 0;
  x.d = Eigen::Vector2d::UnitX();
  x.angle_deg = 0.0;
  x.dominant = true;
  y.index = 1;
  y.d = Eigen::Vector2d::UnitY();
  y.angle_deg = 90.0;
  y.dominant = true;
  return {x, y};
}

std::vector<WallFace> faces_of(const std::vector<WallFace> &all, int family) {
  std::vector<WallFace> out;
  for (const auto &f : all)
    if (f.family == family)
      out.push_back(f);
  return out;
}

} // namespace

TEST_CASE("WallFaces_RoomWithTwoWallsPerFamily_ReturnsFourFaces",
          "[walls][wall_faces]") {
  // A 4 x 3 m room seen from inside: every face's normal points inward.
  auto wp = storey_points();
  add_face(wp, {0, 0}, {0, 3}, {1, 0}, 0.0, 2.5, 0.005, 1);
  add_face(wp, {4, 0}, {4, 3}, {-1, 0}, 0.0, 2.5, 0.005, 2);
  add_face(wp, {0, 0}, {4, 0}, {0, 1}, 0.0, 2.5, 0.005, 3);
  add_face(wp, {0, 3}, {4, 3}, {0, -1}, 0.0, 2.5, 0.005, 4);

  const auto faces = wall_faces(wp, grid_families());
  REQUIRE(faces.size() == 4);

  const auto fx = faces_of(faces, 0);
  REQUIRE(fx.size() == 2);
  CHECK_THAT(fx[0].offset, WithinAbs(0.0, 0.005));
  CHECK(fx[0].side == 1);
  CHECK_THAT(fx[1].offset, WithinAbs(4.0, 0.005));
  CHECK(fx[1].side == -1);
  // Along-axis extent: s = y for the x family.
  CHECK_THAT(fx[0].s0, WithinAbs(0.0, 0.05));
  CHECK_THAT(fx[0].s1, WithinAbs(3.0, 0.05));
  CHECK_THAT(fx[0].z0, WithinAbs(0.0, 0.05));
  CHECK_THAT(fx[0].z1, WithinAbs(2.5, 0.05));
  CHECK(fx[0].point_count == 60 * 50);
  CHECK_FALSE(fx[0].cells.empty());

  const auto fy = faces_of(faces, 1);
  REQUIRE(fy.size() == 2);
  CHECK_THAT(fy[0].offset, WithinAbs(0.0, 0.005));
  CHECK_THAT(fy[1].offset, WithinAbs(3.0, 0.005));
  // s = -x for the y family (t = perp(d) = (-1, 0)).
  CHECK_THAT(fy[0].s0, WithinAbs(-4.0, 0.05));
  CHECK_THAT(fy[0].s1, WithinAbs(0.0, 0.05));
}

TEST_CASE("WallFaces_DoorwayGap_SplitsTheRun", "[walls][wall_faces]") {
  auto wp = storey_points();
  // A wall at x = 1 seen from +x: 0..2 m, a 1 m doorway, 3..5 m; and a
  // 0.4 m gap (3..5 m continues to 5.4..6.4 m) that does not split it.
  add_face(wp, {1, 0}, {1, 2}, {1, 0}, 0.0, 2.5);
  add_face(wp, {1, 3}, {1, 5}, {1, 0}, 0.0, 2.5);
  add_face(wp, {1, 5.4}, {1, 6.4}, {1, 0}, 0.0, 2.5);
  // A 0.3 m stub further along: shorter than min_run_m, dropped.
  add_face(wp, {1, 8}, {1, 8.3}, {1, 0}, 0.0, 2.5);

  const auto faces = wall_faces(wp, grid_families());
  REQUIRE(faces.size() == 2);
  CHECK_THAT(faces[0].s0, WithinAbs(0.0, 0.05));
  CHECK_THAT(faces[0].s1, WithinAbs(2.0, 0.05));
  CHECK_THAT(faces[1].s0, WithinAbs(3.0, 0.05));
  CHECK_THAT(faces[1].s1, WithinAbs(6.4, 0.05));
  CHECK_THAT(faces[0].offset, WithinAbs(1.0, 1e-9));
}

TEST_CASE("WallFaces_TwoFacesOfAThinWall_SeparatedBySide",
          "[walls][wall_faces]") {
  auto wp = storey_points();
  add_face(wp, {1.0, 0}, {1.0, 3}, {-1, 0}, 0.0, 2.5, 0.005, 5);
  add_face(wp, {1.12, 0}, {1.12, 3}, {1, 0}, 0.0, 2.5, 0.005, 6);

  const auto faces = wall_faces(wp, grid_families());
  REQUIRE(faces.size() == 2);
  CHECK(faces[0].side == -1);
  CHECK(faces[1].side == 1);
  CHECK_THAT(faces[0].offset, WithinAbs(1.0, 0.003));
  CHECK_THAT(faces[1].offset, WithinAbs(1.12, 0.003));
}

TEST_CASE("WallFaces_NormalOffEveryFamily_IsNotEvidence",
          "[walls][wall_faces]") {
  auto wp = storey_points();
  // A face at 45 deg: its normal is 45 deg from both grid families.
  const Eigen::Vector2d n(std::sqrt(0.5), std::sqrt(0.5));
  add_face(wp, {0, 3}, {3, 0}, n, 0.0, 2.5);
  CHECK(wall_faces(wp, grid_families()).empty());
}

TEST_CASE("WallFaces_ShuffledInput_SameFaces", "[walls][wall_faces]") {
  auto wp = storey_points();
  add_face(wp, {0, 0}, {0, 3}, {1, 0}, 0.0, 2.5, 0.01, 7);
  add_face(wp, {0, 0}, {4, 0}, {0, 1}, 0.0, 2.5, 0.01, 8);
  add_face(wp, {2.3, 0}, {2.3, 1.6}, {-1, 0}, 0.0, 2.5, 0.01, 9);
  const auto a = wall_faces(wp, grid_families());

  std::vector<size_t> perm(wp.points.size());
  for (size_t i = 0; i < perm.size(); ++i)
    perm[i] = i;
  std::mt19937 rng(3);
  std::shuffle(perm.begin(), perm.end(), rng);
  WallPoints shuffled = storey_points();
  for (const size_t i : perm) {
    shuffled.points.push_back(wp.points[i]);
    shuffled.normals.push_back(wp.normals[i]);
  }
  const auto b = wall_faces(shuffled, grid_families());

  REQUIRE(a.size() == 3);
  REQUIRE(a.size() == b.size());
  for (size_t i = 0; i < a.size(); ++i) {
    CHECK(a[i].family == b[i].family);
    CHECK(a[i].side == b[i].side);
    CHECK(a[i].offset == b[i].offset);
    CHECK(a[i].s0 == b[i].s0);
    CHECK(a[i].s1 == b[i].s1);
    CHECK(a[i].point_count == b[i].point_count);
  }
}

TEST_CASE("WallFaces_NoPointsOrNoFamilies_ReturnsEmpty",
          "[walls][wall_faces]") {
  auto wp = storey_points();
  CHECK(wall_faces(wp, grid_families()).empty());
  add_face(wp, {0, 0}, {0, 3}, {1, 0}, 0.0, 2.5);
  CHECK(wall_faces(wp, {}).empty());
  wp.normals.pop_back();
  CHECK_THROWS_AS(wall_faces(wp, grid_families()), std::invalid_argument);
}

TEST_CASE("WallFaces_SlightlyDriftedWallPlane_IsSquaredIntoOneFace",
          "[walls][wall_faces]") {
  // A 4 m wall 3 deg off the x family (pose drift), seen from +x, one plane.
  const double a = 3.0 * std::numbers::pi / 180.0;
  auto wp = storey_points();
  const Eigen::Vector2d p0(1.0, 0.0);
  const Eigen::Vector2d p1 =
      p0 + 4.0 * Eigen::Vector2d(std::sin(a), std::cos(a));
  add_face(wp, p0, p1, {std::cos(a), -std::sin(a)}, 0.0, 2.5, 0.003, 10);
  wp.plane.assign(wp.points.size(), 7);

  const auto faces = wall_faces(wp, grid_families());
  REQUIRE(faces.size() == 1);
  // Squared about its centroid: the offset of its middle.
  CHECK_THAT(faces[0].offset, WithinAbs(1.0 + 2.0 * std::sin(a), 0.005));
  CHECK(faces[0].s1 - faces[0].s0 > 3.8);

  // Unsquared, the offsets smear over 21 cm: no single full-length face.
  WallOptions o;
  o.square_deg = 0.0;
  const auto raw = wall_faces(wp, grid_families(), o);
  CHECK_FALSE((raw.size() == 1 && raw[0].s1 - raw[0].s0 > 3.8));
}

TEST_CASE("WallPoints_FromClouds_SplitsWallEvidenceInTheLevelFrame",
          "[walls][wall_faces]") {
  using reusex::core::PlaneClassRecord;
  using reusex::core::StoreyRecord;
  reusex::Cloud cloud;
  reusex::CloudL planes;
  pcl::PointCloud<pcl::Normal> normals;
  pcl::Normal wall_n;
  wall_n.normal_x = 1.0f;
  normals.push_back(wall_n); // plane 1: wall
  pcl::Normal floor_n;
  floor_n.normal_z = 1.0f;
  normals.push_back(floor_n); // plane 2: floor
  auto add = [&](float x, float y, float z, uint32_t label) {
    reusex::PointT p;
    p.x = x;
    p.y = y;
    p.z = z;
    cloud.push_back(p);
    pcl::Label l;
    l.label = label;
    planes.push_back(l);
  };
  add(1.0f, 0.5f, 0.2f, 1);  // wall point, world z 0.2 -> level 1.2
  add(1.0f, 0.5f, 3.0f, 1);  // above the storey
  add(2.0f, 0.5f, -1.0f, 2); // floor point -> others
  add(2.0f, 1.5f, -0.5f, 0); // unsegmented -> others

  PlaneClassRecord w, f;
  w.plane_label = 1;
  w.role = "wall";
  f.plane_label = 2;
  f.role = "floor";
  StoreyRecord s;
  s.z_floor = 0.0;
  s.z_ceiling = 2.6;
  s.level_transform = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 1.0, 0, 0, 0, 1};

  const auto wp =
      reusex::geometry::wall_points(cloud, planes, normals, {w, f}, s);
  REQUIRE(wp.points.size() == 1);
  CHECK_THAT(wp.points[0].z(), WithinAbs(1.2, 1e-6));
  CHECK_THAT(wp.normals[0].x(), WithinAbs(1.0, 1e-6));
  CHECK(wp.plane == std::vector<int>{1});
  CHECK(wp.others.size() == 2);
  CHECK_THAT(wp.z_ceiling, WithinAbs(2.6, 1e-12));

  planes.points.pop_back();
  CHECK_THROWS_AS(
      reusex::geometry::wall_points(cloud, planes, normals, {w, f}, s),
      std::invalid_argument);
}
