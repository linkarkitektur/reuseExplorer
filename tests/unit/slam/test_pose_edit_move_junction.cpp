// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// reusex-9fx: a `move` that is correct must survive the full solve with plane
// factors on. Calibration (docs/research/visual-drift-loop.md §9.7): edits
// that won in `rux optimize --watch --fast` (no plane factors) lost part of
// the gain in the final full solve, because a move was lowered as absolute
// per-node targets at delta * initial: the block's DRIFTED shape pinned at
// one world place, which the plane factors then pull against.
//
// The scene: two passes round a box room, every frame seeing walls, floor
// and ceiling. The odometry drifts smoothly (a small tilt and turn per step,
// which the plane factors can take out) and has one break in the second pass
// (3 deg / 0.3 m, which they cannot: the walls no longer associate). The
// move that fixes the break is written the way a person or the watch loop
// writes it: a world delta on the initial poses that puts the block where
// the unbroken odometry would have it.

#include <reusex/core/pose_edits.hpp>
#include <reusex/slam/PlaneGraphOptimizer.hpp>
#include <reusex/slam/pose_edit_lowering.hpp>
#include <reusex/types/point_types.hpp>

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <vector>

using namespace reusex;
using namespace reusex::geometry;

namespace {

constexpr double kDeg = M_PI / 180.0;
constexpr int kPerPass = 30;
constexpr int kFrames = 2 * kPerPass;
constexpr int kBreak = kPerPass + 12;
constexpr double kRoomX = 4.0, kRoomY = 3.0, kRoomZ = 2.6;

/// Optical->world pose at @p pos looking along heading @p yaw (0 = +x),
/// optical y down.
Eigen::Matrix4d camera(const Eigen::Vector3d &pos, double yaw) {
  const Eigen::Vector3d z(std::cos(yaw), std::sin(yaw), 0.0);
  const Eigen::Vector3d x(std::sin(yaw), -std::cos(yaw), 0.0);
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3, 1>(0, 0) = x;
  T.block<3, 1>(0, 1) = Eigen::Vector3d(0, 0, -1);
  T.block<3, 1>(0, 2) = z;
  T.block<3, 1>(0, 3) = pos;
  return T;
}

/// Two passes on an ellipse inside the room, looking outward-forward so
/// every frame sees two walls, the floor and the ceiling.
Eigen::Matrix4d true_pose(int k) {
  const int pass = k / kPerPass;
  const double s = 2.0 * M_PI * (k % kPerPass) / kPerPass;
  const double rx = pass == 0 ? 1.0 : 1.2, ry = pass == 0 ? 0.6 : 0.8;
  const Eigen::Vector3d c(kRoomX / 2 + rx * std::cos(s),
                          kRoomY / 2 + ry * std::sin(s), 1.4);
  return camera(c, s + 0.6);
}

/// The room's six surfaces as the camera at @p truth sees them, in its
/// optical frame (pose-independent, like real surfels).
FrameSurfels frame(int id, const Eigen::Matrix4d &truth,
                   const Eigen::Matrix4d &seed) {
  auto pts = std::make_shared<Cloud>();
  auto nrm = std::make_shared<CloudN>();
  const Eigen::Matrix4d w2o = truth.inverse();
  const Eigen::Matrix3d R = w2o.topLeftCorner<3, 3>();
  auto add = [&](const Eigen::Vector3d &w, const Eigen::Vector3d &n_in) {
    const Eigen::Vector3d q = (w2o * w.homogeneous()).head<3>();
    if (q.z() < 0.3 || q.norm() > 4.0 || std::abs(q.x()) > 1.2 * q.z() ||
        std::abs(q.y()) > 1.0 * q.z())
      return;
    const Eigen::Vector3d n = R * n_in;
    PointT p;
    p.x = static_cast<float>(q.x());
    p.y = static_cast<float>(q.y());
    p.z = static_cast<float>(q.z());
    p.r = p.g = p.b = 200;
    pts->push_back(p);
    NormalT nn;
    nn.normal_x = static_cast<float>(n.x());
    nn.normal_y = static_cast<float>(n.y());
    nn.normal_z = static_cast<float>(n.z());
    nn.curvature = 0.0f;
    nrm->push_back(nn);
  };
  const double h = 0.08;
  for (double a = 0.0; a <= kRoomX + 1e-9; a += h)
    for (double b = 0.0; b <= kRoomY + 1e-9; b += h) {
      add({a, b, 0.0}, {0, 0, 1});     // floor
      add({a, b, kRoomZ}, {0, 0, -1}); // ceiling
    }
  for (double a = 0.0; a <= kRoomX + 1e-9; a += h)
    for (double z = 0.0; z <= kRoomZ + 1e-9; z += h) {
      add({a, 0.0, z}, {0, 1, 0});
      add({a, kRoomY, z}, {0, -1, 0});
    }
  for (double b = 0.0; b <= kRoomY + 1e-9; b += h)
    for (double z = 0.0; z <= kRoomZ + 1e-9; z += h) {
      add({0.0, b, z}, {1, 0, 0});
      add({kRoomX, b, z}, {-1, 0, 0});
    }
  FrameSurfels f;
  f.node_id = id;
  f.points = pts;
  f.normals = nrm;
  f.world_pose.matrix() = seed;
  return f;
}

Eigen::Matrix4d rot(double deg, const Eigen::Vector3d &axis) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.topLeftCorner<3, 3>() =
      Eigen::AngleAxisd(deg * kDeg, axis.normalized()).toRotationMatrix();
  return T;
}

/// The smooth per-step odometry error (local frame): a little tilt and turn
/// and 3 mm sideways.
Eigen::Matrix4d step_error() {
  Eigen::Matrix4d E = rot(0.12, {1.0, 0.0, 0.0}) * rot(0.10, {0.0, 1.0, 0.0});
  E(0, 3) = 0.003;
  return E;
}

/// The break (local frame of the first broken node): 3 deg and 0.3 m.
Eigen::Matrix4d break_error() {
  Eigen::Matrix4d B = rot(3.0, {0.0, 1.0, 0.0});
  B(0, 3) = 0.30;
  return B;
}

struct Scene {
  std::vector<Eigen::Matrix4d> truth;
  std::vector<Eigen::Matrix4d> seed;     ///< drifted + broken odometry
  std::vector<Eigen::Matrix4d> unbroken; ///< drifted, no break
  std::vector<FrameSurfels>
  frames(const std::vector<Eigen::Matrix4d> &s) const {
    std::vector<FrameSurfels> out;
    for (int k = 0; k < kFrames; ++k)
      out.push_back(
          frame(k, truth[static_cast<size_t>(k)], s[static_cast<size_t>(k)]));
    return out;
  }
};

Scene scene() {
  Scene sc;
  for (int k = 0; k < kFrames; ++k)
    sc.truth.push_back(true_pose(k));
  sc.seed.push_back(sc.truth[0]);
  sc.unbroken.push_back(sc.truth[0]);
  for (int k = 1; k < kFrames; ++k) {
    const Eigen::Matrix4d link =
        sc.truth[static_cast<size_t>(k - 1)].inverse() *
        sc.truth[static_cast<size_t>(k)] * step_error();
    sc.unbroken.push_back(sc.unbroken.back() * link);
    sc.seed.push_back(sc.seed.back() * link *
                      (k == kBreak
                           ? break_error()
                           : Eigen::Matrix4d(Eigen::Matrix4d::Identity())));
  }
  return sc;
}

PlaneGraphOptions options(bool planes) {
  PlaneGraphOptions o;
  o.use_plane_factors = planes;
  o.max_planes_per_frame = 6;
  o.min_plane_inliers = 60;
  o.ransac_distance = 0.02f;
  o.ransac_iterations = 200;
  o.assoc_normal_angle = 10.0f;
  o.assoc_distance = 0.15f;
  o.min_landmark_observations = 2;
  o.assoc_overlap_margin = 2.0f;
  o.min_landmark_spread_ratio = 0.0f;
  o.plane_sigma_normal = 0.05f;
  o.plane_sigma_distance = 0.03f;
  o.underconstrained_odom_scale = 1.0f;
  o.assoc_rounds = 3;
  o.max_iterations = 100;
  o.seed = 42;
  return o;
}

/// The move a person writes for the break: a world delta on the initial
/// poses that puts the block's first node where the unbroken odometry from
/// the node before it has it.
core::PoseEdit the_move(const Scene &sc) {
  const Eigen::Matrix4d good_link =
      sc.unbroken[kBreak - 1].inverse() * sc.unbroken[kBreak];
  const Eigen::Matrix4d D =
      sc.seed[kBreak - 1] * good_link * sc.seed[kBreak].inverse();
  core::PoseEdit m;
  m.id = "m-second-half";
  m.kind = core::PoseEditKind::move;
  m.nodes.first = kBreak;
  m.nodes.last = kFrames - 1;
  std::array<double, 16> a{};
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c)
      a[static_cast<size_t>(4 * r + c)] = D(r, c);
  m.delta.matrix = a;
  m.sigma_trans_m = 0.005;
  m.sigma_rot_deg = 0.1;
  return m;
}

std::vector<int> block() {
  std::vector<int> b;
  for (int k = kBreak; k < kFrames; ++k)
    b.push_back(k);
  return b;
}

/// The old lowering (before reusex-9fx), for contrast: one prior per node at
/// delta * initial with information / n.
std::vector<PosePrior> absolute_priors(const Scene &sc,
                                       const core::PoseEdit &m) {
  const Eigen::Matrix4d D = m.delta.to_matrix();
  std::vector<PosePrior> out;
  const double n = kFrames - kBreak;
  for (int k = kBreak; k < kFrames; ++k) {
    PosePrior p;
    p.i = k;
    p.pose = D * sc.seed[static_cast<size_t>(k)];
    p.information = Eigen::Matrix<double, 6, 6>::Zero();
    for (int d = 0; d < 6; ++d)
      p.information(d, d) =
          1.0 / n /
          std::pow(d < 3 ? m.sigma_rot_deg * kDeg : m.sigma_trans_m, 2);
    p.tag = m.id;
    out.push_back(p);
  }
  return out;
}

double rms_error(const std::vector<FrameSurfels> &f, const Scene &sc) {
  double s = 0.0;
  for (int k = 0; k < kFrames; ++k)
    s += (f[static_cast<size_t>(k)].world_pose.translation() -
          sc.truth[static_cast<size_t>(k)].topRightCorner<3, 1>())
             .squaredNorm();
  return std::sqrt(s / kFrames);
}

double max_diff(const std::vector<FrameSurfels> &a,
                const std::vector<FrameSurfels> &b) {
  double m = 0.0;
  for (size_t k = 0; k < a.size(); ++k)
    m = std::max(
        m,
        (a[k].world_pose.translation() - b[k].world_pose.translation()).norm());
  return m;
}

struct Solve {
  std::vector<FrameSurfels> frames;
  PlaneGraphResult result;
};

/// Solve with the move lowered by lower_move() (junction + cut).
Solve junction_solve(const Scene &sc, bool planes) {
  const auto m = the_move(sc);
  slam::LoweredEdits low;
  std::vector<Eigen::Matrix4d> local(kFrames, Eigen::Matrix4d::Identity());
  slam::lower_move(m, block(), sc.seed, local, Eigen::Matrix4d::Identity(),
                   low);
  Solve s;
  s.frames = sc.frames(sc.seed);
  std::vector<GraphEdge> edges;
  for (const auto &e : legacy_graph_edges(s.frames, {}))
    if (!(e.kind == GraphEdge::Kind::odometry &&
          slam::cuts_odometry_edge(low, e.i, e.j)))
      edges.push_back(e);
  s.result = PlaneGraphOptimizer(options(planes))
                 .optimize_graph(s.frames, edges, low.priors, low.surfaces);
  return s;
}

} // namespace

TEST_CASE("PoseEditLowering_Move_IsAJunctionThatCutsTheOdometry",
          "[slam][pose_edits]") {
  const Scene sc = scene();
  const auto m = the_move(sc);
  slam::LoweredEdits low;
  const std::vector<Eigen::Matrix4d> local(kFrames,
                                           Eigen::Matrix4d::Identity());
  slam::lower_move(m, block(), sc.seed, local, Eigen::Matrix4d::Identity(),
                   low);
  CHECK(low.priors.empty());
  REQUIRE(low.surfaces.size() == 1);
  REQUIRE(low.cuts.size() == 1);
  const auto &j = low.surfaces.front();
  CHECK(j.i == kBreak - 1);
  CHECK(j.j == kBreak);
  CHECK(j.tag == m.id);
  // The junction is the unbroken odometry link.
  const Eigen::Matrix4d want =
      sc.unbroken[kBreak - 1].inverse() * sc.unbroken[kBreak];
  CHECK((j.T_ij - want).cwiseAbs().maxCoeff() < 1e-9);
  // Only the odometry INTO the block is cut, not the one out of it nor any
  // inside it.
  CHECK(slam::cuts_odometry_edge(low, kBreak - 1, kBreak));
  CHECK(slam::cuts_odometry_edge(low, kBreak, kBreak - 2));
  CHECK_FALSE(slam::cuts_odometry_edge(low, kBreak, kBreak + 1));
  CHECK_FALSE(slam::cuts_odometry_edge(low, kBreak - 2, kBreak - 1));

  // Fallbacks to absolute node priors: no node before the block, a block
  // with a hole, a DoF mask.
  for (int variant = 0; variant < 3; ++variant) {
    auto e = m;
    std::vector<int> b = variant == 0 ? std::vector<int>{0, 1, 2} : block();
    if (variant == 1)
      b.erase(b.begin() + 3);
    if (variant == 2)
      e.dofs = core::Dof::x | core::Dof::y | core::Dof::yaw;
    slam::LoweredEdits f;
    slam::lower_move(e, b, sc.seed, local, Eigen::Matrix4d::Identity(), f);
    INFO("variant " << variant);
    CHECK(f.surfaces.empty());
    CHECK(f.cuts.empty());
    CHECK(f.priors.size() == b.size());
  }
}

TEST_CASE("PoseEditsSolver_CorrectMove_SurvivesThePlaneFactors",
          "[slam][pose_edits][optimize]") {
  const Scene sc = scene();
  const double drifted = rms_error(sc.frames(sc.seed), sc);

  // The truth's own solve of the unbroken odometry: what fixing the break
  // and letting the plane factors take out the drift should give.
  auto oracle = sc.frames(sc.unbroken);
  const auto ro = PlaneGraphOptimizer(options(true))
                      .optimize_graph(oracle, legacy_graph_edges(oracle, {}));
  REQUIRE(ro.converged);
  REQUIRE(ro.landmarks >= 4);
  const double oracle_err = rms_error(oracle, sc);

  // The move, the way rux optimize lowers it: --fast (no plane factors) and
  // the full solve.
  const auto fast = junction_solve(sc, false);
  const auto full = junction_solve(sc, true);
  REQUIRE(fast.result.converged);
  REQUIRE(full.result.converged);
  REQUIRE(full.result.edit_residuals.size() == 1);
  CHECK(full.result.edit_residuals[0].gnc_inlier);
  const double fast_err = rms_error(fast.frames, sc);
  const double full_err = rms_error(full.frames, sc);

  // The old lowering, for contrast.
  auto abs_full = sc.frames(sc.seed);
  const auto ra =
      PlaneGraphOptimizer(options(true))
          .optimize_graph(abs_full, legacy_graph_edges(abs_full, {}),
                          absolute_priors(sc, the_move(sc)));
  REQUIRE(ra.converged);
  const double abs_err = rms_error(abs_full, sc);

  INFO("rms vs truth: drifted "
       << drifted << ", oracle " << oracle_err << ", move --fast " << fast_err
       << ", move full " << full_err << ", old absolute lowering full "
       << abs_err << "; full vs oracle max " << max_diff(full.frames, oracle)
       << " m");
  // The plane factors help the drift here (the scene is fair to them).
  REQUIRE(oracle_err < 0.8 * rms_error(sc.frames(sc.unbroken), sc));
  // The bar of reusex-9fx: the full solve keeps what --fast won, and more.
  CHECK(full_err <= fast_err);
  // It is the solve of the unbroken odometry: the move fixed the break and
  // nothing else.
  CHECK(max_diff(full.frames, oracle) < 0.02);
  CHECK(full_err < oracle_err + 0.005);
  // The old lowering pinned the drifted block in the world and lost it.
  CHECK(abs_err > full_err + 0.01);
}
