// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Orientation of the stored `normals` cloud written by
// `reconstruct_point_clouds()` (reusex-0pi).
//
// The stored normals are re-estimated on the voxelized cloud. Without a
// viewpoint PCL orients every one of them toward the world origin, so the two
// faces of an interior wall — each seen from its own side — end up with the
// SAME sign whenever the origin lies on one side of the wall. Downstream,
// `make_pairs` (which needs opposite-facing planes) cannot pair the faces and
// invents back faces, and `merge_planes` cannot tell the two faces of a wall
// from a duplicate.
//
// The fixture is a 12 cm thick wall at x = 10 m, its two faces each seen by
// one camera from its own side. The world origin lies on the -x side of both,
// so "toward the origin" and "toward the sensor" disagree on the +x face: the
// old behaviour fails the +x face check, the fix passes both.

#include <catch2/catch_test_macros.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/SensorIntrinsics.hpp>
#include <reusex/segmentation/reconstruct.hpp>

#include "../../support/pose_fixture.hpp"
#include "../../support/temp_path.hpp"

#include <opencv2/core.hpp>

#include <array>
#include <cmath>
#include <cstddef>

using namespace reusex;
using namespace reusex::test_support;

namespace {

constexpr int kW = 64;
constexpr int kH = 48;
constexpr double kWallX = 10.0;     // wall mid-plane
constexpr double kHalfThick = 0.06; // faces at x = 9.94 and x = 10.06
constexpr double kStandoff = 1.44;  // camera-to-face distance along the axis

/// Row-major pose whose optical axis (camera +z) points along world
/// `dir_x` (= ±1) on the x axis, camera +y pointing down (world -z), placed at
/// (x, 0, 0). Right-handed in both cases.
std::array<double, 16> camera_looking_along_x(double x, double dir_x) {
  // Columns: images of the camera x, y, z axes in world coordinates.
  //   dir_x = -1: x_cam -> (0, 1, 0), y_cam -> (0, 0, -1), z_cam -> (-1, 0, 0)
  //   dir_x = +1: x_cam -> (0,-1, 0), y_cam -> (0, 0, -1), z_cam -> ( 1, 0, 0)
  const double xy = -dir_x;      // world y of the camera x axis
  return {0.0, 0.0,  dir_x, x,   //
          xy,  0.0,  0.0,   0.0, //
          0.0, -1.0, 0.0,   0.0, //
          0.0, 0.0,  0.0,   1.0};
}

void seed_frame(ProjectDB &db, int node_id,
                const std::array<double, 16> &pose) {
  db.save_sensor_frame(node_id, make_color(kW, kH),
                       make_depth(kW, kH, kStandoff), cv::Mat(), pose,
                       make_intrinsics(64.0, 64.0, kW / 2.0, kH / 2.0, kW, kH));
}

geometry::ReconstructionParams test_params() {
  geometry::ReconstructionParams p;
  p.resolution = 0.05f;
  p.min_distance = 0.0f;
  p.max_distance = 4.0f;
  p.sampling_factor = 1;
  p.confidence_threshold = 2;
  return p;
}

} // namespace

TEST_CASE("ReconstructPointClouds_TwoSidedWall_StoredNormalsFaceTheirSensor",
          "[segmentation][reconstruct][normals]") {
  TempPath tmp("test_reconstruct_normals_two_sided");

  {
    ProjectDB db(tmp.path);
    // Camera on the +x side looking -x sees the face at x = 10.06.
    seed_frame(db, 1,
               camera_looking_along_x(kWallX + kHalfThick + kStandoff, -1.0));
    // Camera on the -x side looking +x sees the face at x = 9.94.
    seed_frame(db, 2,
               camera_looking_along_x(kWallX - kHalfThick - kStandoff, +1.0));
  }

  ProjectDB db(tmp.path);
  geometry::reconstruct_point_clouds(db, test_params());

  auto cloud = db.point_cloud_xyzrgb("cloud");
  auto normals = db.point_cloud_normal("normals");
  REQUIRE(cloud);
  REQUIRE(normals);
  REQUIRE(cloud->size() == normals->size());

  std::size_t plus_face = 0, plus_ok = 0;
  std::size_t minus_face = 0, minus_ok = 0;
  for (std::size_t i = 0; i < cloud->size(); ++i) {
    const float nx = (*normals)[i].normal_x;
    if ((*cloud)[i].x > kWallX) {
      ++plus_face;
      plus_ok += nx > 0.9f ? 1u : 0u; // toward the +x camera
    } else {
      ++minus_face;
      minus_ok += nx < -0.9f ? 1u : 0u; // toward the -x camera
    }
  }

  INFO("+x face: " << plus_ok << "/" << plus_face << " toward sensor; -x face: "
                   << minus_ok << "/" << minus_face << " toward sensor");
  REQUIRE(plus_face > 100);
  REQUIRE(minus_face > 100);
  CHECK(plus_ok == plus_face);
  CHECK(minus_ok == minus_face);
}

TEST_CASE("ReconstructPointClouds_TwoSidedWall_NormalsAreDeterministic",
          "[segmentation][reconstruct][normals]") {
  // STANDARDS §6: same input, same output — the orientation pass must not
  // depend on thread scheduling.
  auto run = [] {
    TempPath tmp("test_reconstruct_normals_determinism");
    {
      ProjectDB db(tmp.path);
      seed_frame(db, 1,
                 camera_looking_along_x(kWallX + kHalfThick + kStandoff, -1.0));
      seed_frame(db, 2,
                 camera_looking_along_x(kWallX - kHalfThick - kStandoff, +1.0));
    }
    ProjectDB db(tmp.path);
    geometry::reconstruct_point_clouds(db, test_params());
    auto normals = db.point_cloud_normal("normals");
    REQUIRE(normals);
    return *normals;
  };

  const auto a = run();
  const auto b = run();
  REQUIRE(a.size() == b.size());
  for (std::size_t i = 0; i < a.size(); ++i) {
    CHECK(a[i].normal_x == b[i].normal_x);
    CHECK(a[i].normal_y == b[i].normal_y);
    CHECK(a[i].normal_z == b[i].normal_z);
  }
}
