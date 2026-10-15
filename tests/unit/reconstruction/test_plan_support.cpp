// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// plan_support (reusex-yf4.2): how well an authored reference plan's walls
// follow the scan cloud, on synthetic clouds whose answers are known.

#include <reusex/core/solid_model.hpp>
#include <reusex/reconstruction/plan_support.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <string>

using Catch::Matchers::WithinAbs;
using reusex::core::SolidModel;
using reusex::core::StoreyRecord;
using reusex::core::WallRecord;
using reusex::geometry::plan_support;
using reusex::geometry::PlanSupportOptions;

namespace {

WallRecord wall(const std::string &guid, double x0, double y0, double x1,
                double y1, double thickness) {
  WallRecord w;
  w.guid = guid;
  w.x0 = x0;
  w.y0 = y0;
  w.x1 = x1;
  w.y1 = y1;
  w.height = 2.5;
  w.thickness = thickness;
  return w;
}

/// A vertical face y = @p y over x in [x0, x1], z in [z0, z0 + 2.5], 5 cm
/// grid.
void add_face(pcl::PointCloud<pcl::PointXYZRGB> &cloud, double y, double x0,
              double x1, double z0 = 0.0) {
  for (double x = x0; x <= x1 + 1e-9; x += 0.05)
    for (double z = z0; z <= z0 + 2.5 + 1e-9; z += 0.05) {
      pcl::PointXYZRGB p;
      p.x = static_cast<float>(x);
      p.y = static_cast<float>(y);
      p.z = static_cast<float>(z);
      cloud.push_back(p);
    }
}

} // namespace

TEST_CASE("plan_support: a wall on its two faces is fully supported",
          "[plan_support]") {
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  add_face(cloud, -0.1, 0.0, 4.0);
  add_face(cloud, 0.1, 0.0, 4.0);
  SolidModel plan;
  plan.walls = {wall("w", 0.0, 0.0, 4.0, 0.0, 0.2)};

  const auto r = plan_support(cloud, plan);
  REQUIRE(r.walls.size() == 1);
  CHECK(r.walls[0].guid == "w");
  CHECK_THAT(r.walls[0].support_share, WithinAbs(1.0, 1e-9));
  REQUIRE(r.walls[0].face_offset_m);
  CHECK_THAT(*r.walls[0].face_offset_m, WithinAbs(0.0, 1e-3));
  CHECK_THAT(r.support_mean, WithinAbs(1.0, 1e-9));
  CHECK(r.weak_walls == 0);
}

TEST_CASE("plan_support: half a wall without evidence is half supported",
          "[plan_support]") {
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  add_face(cloud, 0.0, 0.0, 2.0); // only the first 2 m of a 4 m wall
  SolidModel plan;
  plan.walls = {wall("w", 0.0, 0.0, 4.0, 0.0, -1.0)}; // axis = the face
  const auto r = plan_support(cloud, plan);
  REQUIRE(r.walls.size() == 1);
  CHECK_THAT(r.walls[0].support_share, WithinAbs(0.5, 0.03));
}

TEST_CASE("plan_support: a wall moved off its face loses support and shows "
          "the offset",
          "[plan_support]") {
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  add_face(cloud, 0.0, 0.0, 4.0);
  SolidModel plan;
  PlanSupportOptions opt;
  plan.walls = {wall("near", 0.0, 0.12, 4.0, 0.12, -1.0),
                wall("far", 0.0, 2.0, 4.0, 2.0, -1.0)};
  const auto r = plan_support(cloud, plan, opt);
  REQUIRE(r.walls.size() == 2);
  // Out of the support band (tolerance 5 cm) but within the search radius:
  // the offset says how far to move it.
  CHECK(r.walls[0].support_share == 0.0);
  // The face is 12 cm to the right of the axis (seen from x0 toward x1).
  REQUIRE(r.walls[0].face_offset_m);
  CHECK_THAT(*r.walls[0].face_offset_m, WithinAbs(-0.12, 1e-3));
  // Nowhere near any point: no offset at all.
  CHECK(r.walls[1].support_share == 0.0);
  CHECK_FALSE(r.walls[1].face_offset_m);
  REQUIRE(r.face_offset_abs_median_m);
  CHECK_THAT(*r.face_offset_abs_median_m, WithinAbs(0.12, 1e-3));
  CHECK(r.weak_walls == 2);
}

TEST_CASE("plan_support: the offset finds the face among clutter and says "
          "which way to move",
          "[plan_support]") {
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  add_face(cloud, 0.03, 0.0, 4.0); // the real face, 3 cm left of the axis
  // A sparse shelf 15 cm in front of it, over half the length.
  for (double x = 0.0; x <= 2.0; x += 0.2)
    for (double z = 0.5; z <= 1.5; z += 0.2) {
      pcl::PointXYZRGB p;
      p.x = static_cast<float>(x);
      p.y = -0.15F;
      p.z = static_cast<float>(z);
      cloud.push_back(p);
    }
  SolidModel plan;
  plan.walls = {wall("w", 0.0, 0.0, 4.0, 0.0, -1.0)};
  const auto r = plan_support(cloud, plan);
  REQUIRE(r.walls[0].face_offset_m);
  CHECK_THAT(*r.walls[0].face_offset_m, WithinAbs(0.03, 2e-3));

  // A wall of known thickness drawn 4 cm too thin: + 2 cm on each face.
  pcl::PointCloud<pcl::PointXYZRGB> slab;
  add_face(slab, -0.12, 0.0, 4.0);
  add_face(slab, 0.12, 0.0, 4.0);
  plan.walls = {wall("w", 0.0, 0.0, 4.0, 0.0, 0.2)};
  const auto t = plan_support(slab, plan);
  REQUIRE(t.walls[0].face_offset_m);
  CHECK_THAT(*t.walls[0].face_offset_m, WithinAbs(0.02, 2e-3));
}

TEST_CASE("plan_support: walls are measured in their storey's level frame",
          "[plan_support]") {
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  add_face(cloud, 0.0, 0.0, 4.0, /*z0=*/-1.27); // floor at world z = -1.27
  SolidModel plan;
  StoreyRecord s;
  s.idx = 0;
  s.level_transform[11] = 1.27; // world -> level: lift by 1.27
  plan.storeys = {s};
  plan.walls = {wall("w", 0.0, 0.0, 4.0, 0.0, -1.0)};
  const auto r = plan_support(cloud, plan);
  REQUIRE(r.walls.size() == 1);
  CHECK_THAT(r.walls[0].support_share, WithinAbs(1.0, 1e-9));

  // The same wall a storey up (z_base 3 m) has no evidence.
  plan.walls[0].z_base = 3.0;
  CHECK(plan_support(cloud, plan).walls[0].support_share == 0.0);
}

TEST_CASE("plan_support: an empty plan or cloud reports nothing",
          "[plan_support]") {
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  SolidModel plan;
  plan.walls = {wall("w", 0.0, 0.0, 4.0, 0.0, -1.0)};
  const auto r = plan_support(cloud, plan);
  REQUIRE(r.walls.size() == 1);
  CHECK(r.walls[0].support_share == 0.0);
  CHECK(r.walls[0].points == 0);
  CHECK(plan_support(cloud, SolidModel{}).walls.empty());
}
