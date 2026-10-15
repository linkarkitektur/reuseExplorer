// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Manhattan-world prior on the plane-landmark pose graph (reusex-rqn.1).
//
// The building: a row of rectangular rooms along world x, each seen by three
// frames and by no other room's frames (no landmark is shared between rooms,
// so the plane term alone cannot relate their orientations). The seed
// trajectory carries a YAW DRIFT that grows room by room, exactly the
// signature measured on house1/house2 (wall azimuth vs. capture time). With
// the Manhattan prior every vertical wall landmark is pulled onto one of the
// dominant horizontal axes, which makes the yaw drift observable and removes
// it. A wall genuinely off the grid (30 deg) must not be forced.

#include <reusex/segmentation/Surfel.hpp>
#include <reusex/slam/PlaneGraphOptimizer.hpp>
#include <reusex/slam/optimize_parameters.hpp>
#include <reusex/types/point_types.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <Eigen/Geometry>

#include <cmath>
#include <vector>

using namespace reusex;
using namespace reusex::geometry;

namespace {

constexpr double kDeg = M_PI / 180.0;

/// A rectangular planar patch in world coordinates.
struct Patch {
  Eigen::Vector3d c, u, v, n; ///< centre, in-plane axes, normal
  double hu = 1.0, hv = 1.0;  ///< half extents along u / v
};

/// Optical-frame surfels of every patch as seen from the TRUE pose @p truth
/// (optical->world). Normals face the camera, like real surfels. The frame's
/// world_pose is left at @p truth; callers corrupt it.
FrameSurfels view(int id, const Eigen::Affine3d &truth,
                  const std::vector<Patch> &patches) {
  auto pts = std::make_shared<Cloud>();
  auto nrm = std::make_shared<CloudN>();
  const Eigen::Affine3d w2o = truth.inverse();
  const Eigen::Vector3d cam = truth.translation();
  const double step = 0.05;
  for (const auto &p : patches) {
    Eigen::Vector3d n = p.n.normalized();
    if (n.dot(cam - p.c) < 0.0)
      n = -n;
    const Eigen::Vector3d no = w2o.linear() * n;
    for (double a = -p.hu; a <= p.hu + 1e-9; a += step)
      for (double b = -p.hv; b <= p.hv + 1e-9; b += step) {
        const Eigen::Vector3d q = w2o * (p.c + a * p.u + b * p.v);
        PointT pt;
        pt.x = static_cast<float>(q.x());
        pt.y = static_cast<float>(q.y());
        pt.z = static_cast<float>(q.z());
        pt.r = pt.g = pt.b = 200;
        pts->push_back(pt);
        NormalT nn;
        nn.normal_x = static_cast<float>(no.x());
        nn.normal_y = static_cast<float>(no.y());
        nn.normal_z = static_cast<float>(no.z());
        nn.curvature = 0.0f;
        nrm->push_back(nn);
      }
  }
  FrameSurfels f;
  f.node_id = id;
  f.points = pts;
  f.normals = nrm;
  f.world_pose = truth;
  return f;
}

/// Room @p r of the row: floor, ceiling and three walls, centred at x = 4 r.
/// @p wall_yaw rotates the room's walls about its vertical centre line
/// (0 = on the grid).
std::vector<Patch> room(int r, double wall_yaw = 0.0) {
  const double X = 4.0 * r;
  const Eigen::Vector3d ex = Eigen::Vector3d::UnitX();
  const Eigen::Vector3d ey = Eigen::Vector3d::UnitY();
  const Eigen::Vector3d ez = Eigen::Vector3d::UnitZ();
  const Eigen::Matrix3d R =
      Eigen::AngleAxisd(wall_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Vector3d o(X, 0.0, 0.0);
  auto wall = [&](const Eigen::Vector3d &c, const Eigen::Vector3d &u,
                  const Eigen::Vector3d &n, double hu) {
    return Patch{o + R * (c - o), R * u, ez, R * n, hu, 1.0};
  };
  return {
      Patch{{X, 0.0, 0.0}, ex, ey, ez, 1.3, 1.3},  // floor
      Patch{{X, 0.0, 2.5}, ex, ey, -ez, 1.3, 1.3}, // ceiling
      wall({X, 2.0, 1.25}, ex, -ey, 1.3),          // north wall
      wall({X + 1.6, 0.3, 1.25}, ey, -ex, 1.2),    // east wall
      wall({X - 1.6, 0.3, 1.25}, ey, ex, 1.2),     // west wall
  };
}

/// True optical->world pose of a camera at @p pos looking along world +y
/// rotated by @p yaw (optical z forward, y down).
Eigen::Affine3d camera(const Eigen::Vector3d &pos, double yaw) {
  Eigen::Matrix3d R;
  R.col(0) = Eigen::Vector3d(1, 0, 0);  // optical x -> world x
  R.col(1) = Eigen::Vector3d(0, 0, -1); // optical y (down) -> world -z
  R.col(2) = Eigen::Vector3d(0, 1, 0);  // optical z (forward) -> world y
  Eigen::Affine3d T = Eigen::Affine3d::Identity();
  T.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * R;
  T.translation() = pos;
  return T;
}

/// Yaw of @p a relative to @p b (rad), about world z.
double yaw_error(const Eigen::Affine3d &a, const Eigen::Affine3d &b) {
  const Eigen::Matrix3d D = a.linear() * b.linear().transpose();
  return std::atan2(D(1, 0), D(0, 0));
}

struct Scene {
  std::vector<FrameSurfels> frames;
  std::vector<Eigen::Affine3d> truth;
};

/// @p rooms rooms, three frames each; room r's seed poses carry a yaw drift of
/// r * @p drift_per_room about the trajectory start (the way accumulated
/// odometry yaw drift rotates everything after it). @p skew_room / @p skew_yaw
/// put that room's walls off the grid.
Scene build(int rooms, double drift_per_room, int skew_room = -1,
            double skew_yaw = 0.0) {
  Scene s;
  int id = 0;
  for (int r = 0; r < rooms; ++r) {
    const auto patches = room(r, r == skew_room ? skew_yaw : 0.0);
    const Eigen::Affine3d D(
        Eigen::AngleAxisd(r * drift_per_room, Eigen::Vector3d::UnitZ()));
    for (int k = 0; k < 3; ++k) {
      const Eigen::Affine3d T =
          camera({4.0 * r + 0.3 * (k - 1), -0.6 + 0.2 * (k - 1), 1.4},
                 (k - 1) * 8.0 * kDeg);
      FrameSurfels f = view(id++, T, patches);
      f.world_pose = D * T;
      s.truth.push_back(T);
      s.frames.push_back(std::move(f));
    }
  }
  return s;
}

PlaneGraphOptions options(bool manhattan) {
  PlaneGraphOptions o;
  o.max_planes_per_frame = 6;
  o.min_plane_inliers = 200;
  o.ransac_iterations = 200;
  o.min_landmark_observations = 2;
  o.min_landmark_spread_ratio = 0.0f;
  o.assoc_rounds = 2;
  o.underconstrained_odom_scale = 1.0f;
  // Loose odometry: the seed chain carries the drift, so it must not hold it.
  o.odometry_sigma_rot = 0.25f;
  o.odometry_sigma_trans = 0.5f;
  o.plane_sigma_normal = 0.05f;
  o.plane_sigma_distance = 0.03f;
  o.manhattan = manhattan;
  o.manhattan_up = ManhattanUp::z;
  return o;
}

double max_abs_yaw_error(const Scene &s) {
  double m = 0.0;
  for (size_t i = 0; i < s.frames.size(); ++i)
    m = std::max(m, std::abs(yaw_error(s.frames[i].world_pose, s.truth[i])));
  return m;
}

} // namespace

TEST_CASE("ManhattanPrior_YawDriftAcrossRooms_IsRecovered",
          "[plane_graph][optimize][manhattan]") {
  // 4 rooms, 3 deg of yaw drift per room: the last room is 9 deg off.
  const double drift = 3.0 * kDeg;

  Scene off = build(4, drift);
  const double seed_err = max_abs_yaw_error(off);
  REQUIRE(seed_err == Catch::Approx(9.0 * kDeg).margin(1e-6));
  const PlaneGraphResult r_off =
      PlaneGraphOptimizer(options(false)).optimize(off.frames);
  REQUIRE(r_off.converged);
  const double err_off = max_abs_yaw_error(off);

  Scene on = build(4, drift);
  const PlaneGraphResult r_on =
      PlaneGraphOptimizer(options(true)).optimize(on.frames);
  REQUIRE(r_on.converged);
  const double err_on = max_abs_yaw_error(on);

  INFO("seed " << seed_err / kDeg << " deg, planes only " << err_off / kDeg
               << " deg, with Manhattan " << err_on / kDeg << " deg; "
               << r_on.manhattan_vertical << " vertical + "
               << r_on.manhattan_horizontal << " horizontal factors, axis "
               << r_on.manhattan_axis_deg << " deg");
  // Without the prior nothing relates the rooms: the drift stays.
  CHECK(err_off > 0.8 * seed_err);
  // With it every room is put back on the grid of the gauge room.
  CHECK(r_on.manhattan_vertical >= 4 * 3);
  CHECK(r_on.manhattan_horizontal >= 4);
  CHECK(err_on < 0.5 * kDeg);
  // The jointly estimated axis ends on the gauge room's grid (0 deg mod 90).
  CHECK(std::abs(r_on.manhattan_axis_deg) < 0.5);
}

TEST_CASE("ManhattanPrior_OffGridWall_IsNotForced",
          "[plane_graph][optimize][manhattan]") {
  // No drift at all; room 1's walls stand 30 deg off the grid. If the prior
  // forced them onto an axis, room 1's frames would rotate by ~30 deg.
  SECTION("default gate leaves the 30 deg room alone") {
    Scene s = build(3, 0.0, /*skew_room=*/1, 30.0 * kDeg);
    const PlaneGraphResult r =
        PlaneGraphOptimizer(options(true)).optimize(s.frames);
    REQUIRE(r.converged);
    INFO("max yaw error " << max_abs_yaw_error(s) / kDeg << " deg, "
                          << r.manhattan_skipped << " landmarks off the grid");
    CHECK(r.manhattan_skipped >= 3);
    CHECK(max_abs_yaw_error(s) < 0.2 * kDeg);
  }
  SECTION("control: an open gate would force it (the test is sensitive)") {
    Scene s = build(3, 0.0, 1, 30.0 * kDeg);
    PlaneGraphOptions o = options(true);
    o.manhattan_max_angle_deg = 45.0f;
    const PlaneGraphResult r = PlaneGraphOptimizer(o).optimize(s.frames);
    REQUIRE(r.converged);
    INFO("max yaw error " << max_abs_yaw_error(s) / kDeg << " deg");
    CHECK(max_abs_yaw_error(s) > 5.0 * kDeg);
  }
}

TEST_CASE("ManhattanPrior_Off_IsBitIdenticalToTheShippedSolve",
          "[plane_graph][optimize][manhattan]") {
  Scene a = build(3, 2.0 * kDeg);
  Scene b = build(3, 2.0 * kDeg);
  PlaneGraphOptions o = options(false);
  PlaneGraphOptions defaults_off = o;
  defaults_off.manhattan_sigma_deg = 0.3f; // ignored while the prior is off
  defaults_off.manhattan_max_angle_deg = 5.0f;
  const PlaneGraphResult ra = PlaneGraphOptimizer(o).optimize(a.frames);
  const PlaneGraphResult rb =
      PlaneGraphOptimizer(defaults_off).optimize(b.frames);
  REQUIRE(ra.converged);
  CHECK(ra.manhattan_vertical == 0);
  CHECK(ra.manhattan_horizontal == 0);
  for (size_t i = 0; i < a.frames.size(); ++i)
    CHECK(a.frames[i].world_pose.matrix() == b.frames[i].world_pose.matrix());
  CHECK(ra.final_error == rb.final_error);
}

TEST_CASE("ManhattanPrior_IsDeterministic",
          "[plane_graph][optimize][manhattan]") {
  Scene a = build(4, 3.0 * kDeg);
  Scene b = build(4, 3.0 * kDeg);
  PlaneGraphOptimizer(options(true)).optimize(a.frames);
  PlaneGraphOptimizer(options(true)).optimize(b.frames);
  for (size_t i = 0; i < a.frames.size(); ++i)
    CHECK(a.frames[i].world_pose.matrix() == b.frames[i].world_pose.matrix());
}

TEST_CASE("ManhattanPrior_AutoUp_FindsTheGravityAxis",
          "[plane_graph][optimize][manhattan]") {
  // The cameras walk at constant height (spread along x and y, none along z),
  // so `automatic` must pick z and behave exactly like an explicit z.
  Scene a = build(4, 3.0 * kDeg);
  Scene b = build(4, 3.0 * kDeg);
  PlaneGraphOptions oa = options(true);
  oa.manhattan_up = ManhattanUp::automatic;
  const PlaneGraphResult ra = PlaneGraphOptimizer(oa).optimize(a.frames);
  const PlaneGraphResult rb =
      PlaneGraphOptimizer(options(true)).optimize(b.frames);
  CHECK(ra.manhattan_up_axis == 2);
  for (size_t i = 0; i < a.frames.size(); ++i)
    CHECK(a.frames[i].world_pose.matrix() == b.frames[i].world_pose.matrix());
  CHECK(ra.final_error == rb.final_error);
}

TEST_CASE("ManhattanPrior_StageParameters_AreRead",
          "[slam][optimize_parameters][manhattan]") {
  PlaneGraphOptions o;
  bool dry = false;
  CHECK_FALSE(o.manhattan);
  apply_optimize_parameters(o, dry,
                            R"({"manhattan":true,"manhattan_sigma_deg":0.5,
          "manhattan_max_angle_deg":12,"manhattan_level_max_angle_deg":2})");
  CHECK(o.manhattan);
  CHECK(o.manhattan_level_max_angle_deg == Catch::Approx(2.0));
  CHECK(o.manhattan_sigma_deg == Catch::Approx(0.5));
  CHECK(o.manhattan_max_angle_deg == Catch::Approx(12.0));
  apply_optimize_parameters(o, dry, R"({"manhattan":false})");
  CHECK_FALSE(o.manhattan);
}

TEST_CASE("ManhattanPrior_SlopedCeiling_IsNotLevelled",
          "[plane_graph][optimize][manhattan]") {
  // A mono-pitch ceiling (7 deg, like house1's) over one room: within the
  // wall gate, but it must not be tied to horizontal, or the room's frames
  // would tilt to level it.
  Scene s = build(3, 0.0);
  const Eigen::Vector3d ex = Eigen::Vector3d::UnitX();
  const Eigen::Matrix3d tilt =
      Eigen::AngleAxisd(7.0 * kDeg, Eigen::Vector3d::UnitY())
          .toRotationMatrix();
  auto patches = room(1);
  patches[1] = Patch{{4.0, 0.0, 2.5},
                     tilt * ex,
                     Eigen::Vector3d::UnitY(),
                     tilt * -Eigen::Vector3d::UnitZ(),
                     1.3,
                     1.3};
  for (int k = 3; k < 6; ++k) {
    const Eigen::Affine3d T = s.truth[k];
    s.frames[k] = view(k, T, patches);
  }
  const PlaneGraphResult r =
      PlaneGraphOptimizer(options(true)).optimize(s.frames);
  REQUIRE(r.converged);
  double max_rot = 0.0;
  for (size_t i = 0; i < s.frames.size(); ++i)
    max_rot =
        std::max(max_rot, Eigen::AngleAxisd(s.frames[i].world_pose.linear() *
                                            s.truth[i].linear().transpose())
                              .angle());
  INFO("max rotation error " << max_rot / kDeg << " deg");
  CHECK(r.manhattan_skipped >= 1);
  CHECK(max_rot < 0.2 * kDeg);
}
