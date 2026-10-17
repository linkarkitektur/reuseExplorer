// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Pose edits as solver factors (reusex-rcv.1.3): node priors with selectable
// fixed axes, block moves, the same-surface constraint along the wall normal
// only, and per-edit residuals.
//
// The trajectory: two passes round the same rectangle, the second with a
// break injected in its middle (a wrong odometry link: everything after it is
// turned 3 deg and shifted 0.3 m), the way a bad loop or a tracking jump
// leaves a pass. The regression-221 lesson is checked on the way: a move on
// the broken block alone also bends the trajectory BEFORE the block (the
// odometry chain is the softer spring); a node prior in the middle stops
// that.

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/logging.hpp>
#include <reusex/core/pose_edits.hpp>
#include <reusex/core/pose_graph.hpp>
#include <reusex/segmentation/Surfel.hpp>
#include <reusex/slam/PlaneGraphOptimizer.hpp>
#include <reusex/slam/pose_edit_lowering.hpp>
#include <reusex/slam/pose_graph.hpp>
#include <reusex/types/point_types.hpp>

#include "../../support/pose_fixture.hpp"
#include "../../support/temp_path.hpp"

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Geometry>

#include <cmath>
#include <optional>
#include <string>
#include <vector>

using namespace reusex;
using namespace reusex::geometry;
namespace pg = reusex::core::pose_graph;
using reusex::test_support::TempPath;

namespace {

constexpr double kDeg = M_PI / 180.0;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

/// Optical->world pose of a camera at @p pos looking along heading @p yaw
/// (0 = world +y), optical y down.
Eigen::Matrix4d camera(const Eigen::Vector3d &pos, double yaw) {
  Eigen::Matrix3d R;
  R.col(0) = Eigen::Vector3d(1, 0, 0);
  R.col(1) = Eigen::Vector3d(0, 0, -1);
  R.col(2) = Eigen::Vector3d(0, 1, 0);
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.topLeftCorner<3, 3>() =
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() * R;
  T.topRightCorner<3, 1>() = pos;
  return T;
}

/// A frame whose surfels are a floor patch below it (so plane detection has
/// something to find); its world_pose is @p pose.
FrameSurfels frame(int id, const Eigen::Matrix4d &pose) {
  auto pts = std::make_shared<Cloud>();
  auto nrm = std::make_shared<CloudN>();
  const Eigen::Matrix4d w2o = pose.inverse();
  const Eigen::Vector3d up_o =
      w2o.topLeftCorner<3, 3>() * Eigen::Vector3d::UnitZ();
  for (double a = -1.0; a <= 1.0 + 1e-9; a += 0.05)
    for (double b = -1.0; b <= 1.0 + 1e-9; b += 0.05) {
      const Eigen::Vector4d w(pose(0, 3) + a, pose(1, 3) + b, 0.0, 1.0);
      const Eigen::Vector3d q = (w2o * w).head<3>();
      PointT p;
      p.x = static_cast<float>(q.x());
      p.y = static_cast<float>(q.y());
      p.z = static_cast<float>(q.z());
      p.r = p.g = p.b = 200;
      pts->push_back(p);
      NormalT n;
      n.normal_x = static_cast<float>(up_o.x());
      n.normal_y = static_cast<float>(up_o.y());
      n.normal_z = static_cast<float>(up_o.z());
      n.curvature = 0.0f;
      nrm->push_back(n);
    }
  FrameSurfels f;
  f.node_id = id;
  f.points = pts;
  f.normals = nrm;
  f.world_pose.matrix() = pose;
  return f;
}

constexpr int kPerPass = 30;
constexpr int kBreak = kPerPass + 15; // the middle of the second pass

/// Perimeter of a 4 x 3 m rectangle, s in [0, 1): position and heading.
std::pair<Eigen::Vector3d, double> walk(double s, double inset) {
  const double L = 14.0;
  double d = s * L;
  const double w = 4.0 - 2 * inset, h = 3.0 - 2 * inset;
  const Eigen::Vector3d o(inset, inset, 1.4);
  if (d < w)
    return {o + Eigen::Vector3d(d, 0, 0), -M_PI / 2};
  d -= w;
  if (d < h)
    return {o + Eigen::Vector3d(w, d, 0), 0.0};
  d -= h;
  if (d < w)
    return {o + Eigen::Vector3d(w - d, h, 0), M_PI / 2};
  d -= w;
  return {o + Eigen::Vector3d(0, h - std::min(d, h), 0), M_PI};
}

/// The break: turn by 3 deg about the break frame's position, shift 0.3 m.
Eigen::Matrix4d break_transform(const Eigen::Matrix4d &at) {
  Eigen::Matrix4d B = Eigen::Matrix4d::Identity();
  const Eigen::Matrix3d R =
      Eigen::AngleAxisd(3.0 * kDeg, Eigen::Vector3d::UnitZ())
          .toRotationMatrix();
  const Eigen::Vector3d c = at.topRightCorner<3, 1>();
  B.topLeftCorner<3, 3>() = R;
  B.topRightCorner<3, 1>() = c - R * c + Eigen::Vector3d(0.3, 0.0, 0.0);
  return B;
}

struct Loop {
  std::vector<Eigen::Matrix4d> truth;
  std::vector<FrameSurfels> frames; ///< seeded with the broken trajectory
  std::vector<GraphEdge> edges;     ///< odometry from the broken seeds
  Eigen::Matrix4d B;
};

Loop two_pass_loop() {
  Loop l;
  for (int k = 0; k < 2 * kPerPass; ++k) {
    const int pass = k / kPerPass;
    const auto [pos, yaw] =
        walk(static_cast<double>(k % kPerPass) / kPerPass, 0.1 * pass);
    l.truth.push_back(camera(pos, yaw));
  }
  l.B = break_transform(l.truth[kBreak]);
  for (int k = 0; k < 2 * kPerPass; ++k)
    l.frames.push_back(
        frame(k, k >= kBreak ? Eigen::Matrix4d(l.B * l.truth[k]) : l.truth[k]));
  l.edges = legacy_graph_edges(l.frames, {});
  return l;
}

PlaneGraphOptions graph_options() {
  PlaneGraphOptions o;
  o.use_plane_factors = false; // odometry + gauge + edits: the pose graph
  o.underconstrained_odom_scale = 1.0f;
  o.min_landmark_observations = 2;
  return o;
}

Matrix6d diag_information(double sigma_t, double sigma_rot_deg,
                          std::uint8_t dofs = core::Dof::all,
                          double scale = 1.0) {
  Matrix6d I = Matrix6d::Zero();
  const std::uint8_t bits[6] = {core::Dof::roll, core::Dof::pitch,
                                core::Dof::yaw,  core::Dof::x,
                                core::Dof::y,    core::Dof::z};
  for (int k = 0; k < 6; ++k)
    if (dofs & bits[k])
      I(k, k) = scale / std::pow(k < 3 ? sigma_rot_deg * kDeg : sigma_t, 2);
  return I;
}

/// One prior per node of [first, last] at truth, information / n (the
/// lowering of a `move` whose delta undoes the break exactly).
std::vector<PosePrior> move_priors(const Loop &l, int first, int last,
                                   double sigma_t, double sigma_r) {
  std::vector<PosePrior> out;
  const int n = last - first + 1;
  for (int k = first; k <= last; ++k) {
    PosePrior p;
    p.i = k;
    p.pose =
        l.B.inverse() * l.frames[static_cast<size_t>(k)].world_pose.matrix();
    p.information = diag_information(sigma_t, sigma_r, core::Dof::all, 1.0 / n);
    p.tag = "m-broken-block";
    out.push_back(p);
  }
  return out;
}

double pos_err(const Eigen::Matrix4d &a, const Eigen::Matrix4d &b) {
  return (a.topRightCorner<3, 1>() - b.topRightCorner<3, 1>()).norm();
}

double rot_err_deg(const Eigen::Matrix4d &a, const Eigen::Matrix4d &b) {
  return Eigen::AngleAxisd(Eigen::Matrix3d(a.topLeftCorner<3, 3>() *
                                           b.topLeftCorner<3, 3>().transpose()))
             .angle() /
         kDeg;
}

double max_pos_err(const Loop &l, int first, int last) {
  double m = 0.0;
  for (int k = first; k <= last; ++k)
    m = std::max(m,
                 pos_err(l.frames[static_cast<size_t>(k)].world_pose.matrix(),
                         l.truth[static_cast<size_t>(k)]));
  return m;
}

double max_rot_err(const Loop &l, int first, int last) {
  double m = 0.0;
  for (int k = first; k <= last; ++k)
    m = std::max(
        m, rot_err_deg(l.frames[static_cast<size_t>(k)].world_pose.matrix(),
                       l.truth[static_cast<size_t>(k)]));
  return m;
}

/// Whitened odometry residual sqrt(2 * error) of every odometry edge but the
/// broken one, the largest.
double max_odometry_sigma(const PlaneGraphResult &r) {
  double m = 0.0;
  for (const auto &e : r.edges)
    if (e.type == "odometry" && !(e.i == kBreak - 1 && e.j == kBreak))
      m = std::max(m, std::sqrt(2.0 * e.residual));
  return m;
}

/// Captures `warn` lines from the library for the duration of a test.
class LogCapture {
    public:
  LogCapture() : previous_(core::get_log_level()) {
    core::set_log_level(core::LogLevel::info);
    core::set_log_handler([this](core::LogLevel level, std::string_view m) {
      if (level == core::LogLevel::warn)
        warnings_.emplace_back(m);
    });
  }
  ~LogCapture() {
    core::reset_log_handler();
    core::set_log_level(previous_);
  }
  bool saw(std::string_view needle) const {
    for (const auto &w : warnings_)
      if (w.find(needle) != std::string::npos)
        return true;
    return false;
  }

    private:
  core::LogLevel previous_;
  std::vector<std::string> warnings_;
};

/// Yaw (about world z) and the tilt left after removing it, of a * b^-1.
std::pair<double, double> yaw_and_tilt_deg(const Eigen::Matrix4d &a,
                                           const Eigen::Matrix4d &b) {
  const Eigen::Quaterniond q(Eigen::Matrix3d(
      a.topLeftCorner<3, 3>() * b.topLeftCorner<3, 3>().transpose()));
  const double twist = 2.0 * std::atan2(q.z(), q.w());
  const Eigen::Quaterniond qt(
      Eigen::AngleAxisd(twist, Eigen::Vector3d::UnitZ()));
  const double tilt = Eigen::AngleAxisd(qt.inverse() * q).angle();
  return {twist / kDeg, tilt / kDeg};
}

} // namespace

TEST_CASE("PoseEditsSolver_TwoPassBreak_MovePlusNodePriorRecoversIt",
          "[slam][pose_edits][optimize]") {
  // The break as seeded: the broken block is 0.3 m / 3 deg off.
  {
    const Loop seed = two_pass_loop();
    REQUIRE(max_pos_err(seed, kBreak, 2 * kPerPass - 1) > 0.29);
    REQUIRE(max_rot_err(seed, kBreak, 2 * kPerPass - 1) > 2.99);
  }
  // Tight sigmas: "I checked this in the renders". The wrong link is ONE
  // 1 cm / 0.29 deg odometry factor, so the edits must be clearly stiffer
  // than it to make it give way (a person who sees a jump can also reject
  // the link with a reject_edge edit; this test does without).
  const double st = 0.0005, sr = 0.01;
  PlaneGraphOptimizer opt(graph_options());

  // (a) The move alone: the block lands, but the chain before it bends
  // toward it (regression 221: the correction spreads over the softer
  // odometry chain instead of the one wrong link).
  Loop alone = two_pass_loop();
  const auto priors = move_priors(alone, kBreak, 2 * kPerPass - 1, st, sr);
  const auto ra = opt.optimize_graph(alone.frames, alone.edges, priors);
  REQUIRE(ra.converged);
  REQUIRE(ra.edit_factors == static_cast<int>(priors.size()));
  const double block_alone = max_pos_err(alone, kBreak, 2 * kPerPass - 1);
  const double before_alone = max_pos_err(alone, 0, kBreak - 1);
  INFO("move alone: block " << block_alone << " m, before the block "
                            << before_alone << " m");
  CHECK(block_alone < 0.03);
  CHECK(before_alone > 0.10); // the shear the node prior must prevent

  // (b) Move + a node prior on the last frame before the break (x, y, yaw):
  // the wrong link takes the correction, everything else is truth again.
  Loop both = two_pass_loop();
  auto all = move_priors(both, kBreak, 2 * kPerPass - 1, st, sr);
  PosePrior anchor;
  anchor.i = kBreak - 1;
  anchor.pose = both.truth[kBreak - 1];
  anchor.information =
      diag_information(st, sr, core::Dof::x | core::Dof::y | core::Dof::yaw);
  anchor.tag = "a-before-break";
  all.push_back(anchor);
  const auto rb = opt.optimize_graph(both.frames, both.edges, all);
  REQUIRE(rb.converged);
  const double block = max_pos_err(both, kBreak, 2 * kPerPass - 1);
  const double before = max_pos_err(both, 0, kBreak - 1);
  const double block_rot = max_rot_err(both, kBreak, 2 * kPerPass - 1);
  INFO("move + node prior: block " << block << " m / " << block_rot
                                   << " deg, before " << before << " m");
  CHECK(block < 0.1 * 0.3);     // >= 90 % of the 0.3 m break removed
  CHECK(block_rot < 0.1 * 3.0); // and of the 3 deg
  CHECK(before < 0.1 * 0.3);    // no shear before the block
  // The good links stay at the noise level: a whitened 6-DoF residual of
  // a link with only measurement noise would be ~sqrt(6) = 2.4.
  CHECK(max_odometry_sigma(rb) < 2.0);

  // Per-edit residuals: both edits reported, honoured, small.
  REQUIRE(rb.edit_residuals.size() == 2);
  CHECK(rb.edit_residuals[0].tag == "m-broken-block");
  CHECK(rb.edit_residuals[0].factors == 2 * kPerPass - kBreak);
  CHECK(rb.edit_residuals[1].tag == "a-before-break");
  for (const auto &r : rb.edit_residuals) {
    INFO(r.tag << ": " << r.trans_m << " m, " << r.rot_deg << " deg, chi2 "
               << r.chi2);
    CHECK(r.gnc_inlier);
    CHECK(r.trans_m < 0.03);
    CHECK(r.chi2 >= 0.0);
    // reusex-rcv.1.28: the RMS over the edit's factors, never above the max.
    CHECK(r.rms_trans_m <= r.trans_m + 1e-12);
    CHECK(r.rms_rot_deg <= r.rot_deg + 1e-12);
  }
  // A move's RMS is the block's own residual: positive, below its largest
  // node residual; an anchor's (one factor) is that factor's.
  CHECK(rb.edit_residuals[0].rms_trans_m > 0.0);
  CHECK(rb.edit_residuals[1].rms_trans_m == rb.edit_residuals[1].trans_m);
  CHECK(rb.edit_residuals[1].rms_rot_deg == rb.edit_residuals[1].rot_deg);
}

TEST_CASE("PoseEditsSolver_AnchorDofMask_LeavesMaskedAxesAlone",
          "[slam][pose_edits][optimize]") {
  PlaneGraphOptimizer opt(graph_options());
  const int k = 20; // the MIDDLE of the first pass
  const Eigen::Matrix4d seed = two_pass_loop().frames[k].world_pose.matrix();
  const Eigen::Matrix3d axes = Eigen::Matrix3d::Identity();
  const Eigen::Matrix3d C = prior_levelling(seed.topLeftCorner<3, 3>(), axes);
  const Eigen::Vector3d rpy = prior_rpy(seed.topLeftCorner<3, 3>(), C, axes);
  const std::uint8_t xy_yaw = core::Dof::x | core::Dof::y | core::Dof::yaw;

  /// The target: x, y, yaw moved by (dx, dy, dyaw) from the seed, and z,
  /// roll, pitch off by `garbage` (which the mask must ignore).
  auto target = [&](double dx, double dy, double dyaw, bool garbage) {
    Eigen::Vector3d r = rpy + Eigen::Vector3d(0.0, 0.0, dyaw * kDeg);
    Eigen::Matrix4d T = seed;
    T(0, 3) += dx;
    T(1, 3) += dy;
    if (garbage) {
      r += Eigen::Vector3d(4.0 * kDeg, -3.0 * kDeg, 0.0);
      T(2, 3) += 0.40;
    }
    T.topLeftCorner<3, 3>() = prior_rotation(r, C, axes);
    return T;
  };
  auto solve = [&](const std::optional<Eigen::Matrix4d> &pose) {
    Loop l = two_pass_loop();
    std::vector<PosePrior> priors;
    if (pose) {
      PosePrior p;
      p.i = k;
      p.pose = *pose;
      p.information = diag_information(0.002, 0.05, xy_yaw);
      p.tag = "a-mid";
      priors.push_back(p);
    }
    REQUIRE(opt.optimize_graph(l.frames, l.edges, priors).converged);
    std::vector<Eigen::Matrix4d> out;
    for (const auto &f : l.frames)
      out.push_back(f.world_pose.matrix());
    return out;
  };
  auto max_diff = [](const std::vector<Eigen::Matrix4d> &a,
                     const std::vector<Eigen::Matrix4d> &b) {
    double m = 0.0;
    for (size_t i = 0; i < a.size(); ++i)
      m = std::max(m, (a[i] - b[i]).cwiseAbs().maxCoeff());
    return m;
  };

  // (i) A target that agrees with the seed in x, y and yaw exerts no force,
  // however wrong its z, roll and pitch: the solve is the no-edit solve.
  const auto ref = solve(std::nullopt);
  const auto masked = solve(target(0.0, 0.0, 0.0, /*garbage=*/true));
  CHECK(max_diff(ref, masked) < 1e-6);

  // (ii) Moving x, y and yaw: the node follows them, and the masked target
  // components still change nothing (garbage or not, same solution).
  const auto moved = solve(target(0.10, -0.05, 2.0, /*garbage=*/true));
  const auto clean = solve(target(0.10, -0.05, 2.0, /*garbage=*/false));
  CHECK(max_diff(moved, clean) < 1e-6);
  const Eigen::Matrix4d got = moved[k];
  const Eigen::Vector3d got_rpy = prior_rpy(got.topLeftCorner<3, 3>(), C, axes);
  CHECK(std::abs(got(0, 3) - (seed(0, 3) + 0.10)) < 0.005);
  CHECK(std::abs(got(1, 3) - (seed(1, 3) - 0.05)) < 0.005);
  CHECK(std::abs(std::remainder(got_rpy[2] - rpy[2], 2.0 * M_PI) / kDeg - 2.0) <
        0.1);
  // ...and its height, roll and pitch stay exactly where the graph alone
  // puts them (bead acceptance: < 1e-6).
  const Eigen::Vector3d ref_rpy =
      prior_rpy(ref[k].topLeftCorner<3, 3>(), C, axes);
  INFO("z moved " << got(2, 3) - ref[k](2, 3) << " m, roll "
                  << (got_rpy[0] - ref_rpy[0]) / kDeg << " deg, pitch "
                  << (got_rpy[1] - ref_rpy[1]) / kDeg << " deg");
  CHECK(std::abs(got(2, 3) - ref[k](2, 3)) < 1e-6);
  CHECK(std::abs(got_rpy[0] - ref_rpy[0]) < 1e-6);
  CHECK(std::abs(got_rpy[1] - ref_rpy[1]) < 1e-6);
}

TEST_CASE("PoseEditsSolver_NoEdits_IsBitIdentical",
          "[slam][pose_edits][optimize]") {
  PlaneGraphOptions o = graph_options();
  o.use_plane_factors = true; // the full default graph
  PlaneGraphOptimizer opt(o);
  Loop a = two_pass_loop(), b = two_pass_loop();
  const auto ra = opt.optimize_graph(a.frames, a.edges);
  const auto rb = opt.optimize_graph(b.frames, b.edges, {}, {});
  REQUIRE(ra.converged == rb.converged);
  CHECK(ra.final_error == rb.final_error);
  CHECK(rb.edit_residuals.empty());
  for (size_t k = 0; k < a.frames.size(); ++k)
    CHECK(a.frames[k].world_pose.matrix() == b.frames[k].world_pose.matrix());
}

TEST_CASE("PoseEditsSolver_ContradictoryAnchors_AreReportedNotObeyed",
          "[slam][pose_edits][optimize]") {
  LogCapture log;
  PlaneGraphOptimizer opt(graph_options());
  Loop l = two_pass_loop();
  const int k = 20;
  PosePrior a, b;
  a.i = b.i = k;
  a.pose = b.pose = l.frames[k].world_pose.matrix();
  a.pose(0, 3) += 0.5;
  b.pose(0, 3) -= 0.5;
  a.information = b.information = diag_information(0.02, 0.5);
  a.tag = "a-east";
  b.tag = "a-west";
  const auto r = opt.optimize_graph(l.frames, l.edges, std::vector{a, b});
  REQUIRE(r.converged);
  REQUIRE(r.edit_residuals.size() == 2);
  bool any_outlier = false;
  for (const auto &e : r.edit_residuals) {
    INFO(e.tag << ": " << e.trans_m << " m, chi2 " << e.chi2 << ", inlier "
               << e.gnc_inlier);
    any_outlier = any_outlier || !e.gnc_inlier;
    CHECK(e.trans_m > 0.2);            // neither is met: they cannot both be
    CHECK(e.rms_trans_m == e.trans_m); // one factor each
  }
  CHECK(any_outlier);
  CHECK(log.saw("is NOT honoured"));
}

// ---------------------------------------------------------------------------
// Through the database: lower_pose_edits + optimize_sensor_poses.
// Frames look at a wall 1.5 m ahead (+z, identity rotation), so their depth
// back-projects to the plane z = 1.5 in their own frame.
// ---------------------------------------------------------------------------

namespace {

using namespace reusex::test_support;

constexpr int kW = 64;
constexpr int kH = 48;

geometry::PlaneGraphOptions wall_options() {
  geometry::PlaneGraphOptions o;
  o.surfel.min_distance = 0.0f;
  o.surfel.max_distance = 4.0f;
  o.surfel.sampling_factor = 2;
  o.surfel.confidence_threshold = 2;
  o.surfel.normal_radius = 0.1f;
  o.surfel.voxel_size = 0.0f;
  o.surfel.apply_depth_filters = false;
  o.min_plane_inliers = 50;
  o.min_landmark_observations = 2;
  o.min_landmark_spread_ratio = 0.0f;
  o.assoc_overlap_margin = 2.0f;
  o.assoc_rounds = 1;
  o.use_plane_factors = false; // isolate the edits
  o.underconstrained_odom_scale = 1.0f;
  return o;
}

/// Two passes of five frames along x in front of one wall; pass b's poses
/// are seeded 0.10 m back from the wall, so its copy of the wall sits
/// 0.10 m in front of pass a's: a ghost wall.
void ghost_wall_project(ProjectDB &db) {
  for (int k = 0; k < 10; ++k) {
    const double x = 0.1 * (k % 5);
    const double z = k < 5 ? 0.0 : -0.10;
    db.save_sensor_frame(
        k, make_color(kW, kH), make_depth(kW, kH, 1.5), cv::Mat(),
        translation_pose(x, 0.0, z),
        make_intrinsics(64.0, 64.0, kW / 2.0, kH / 2.0, kW, kH),
        static_cast<double>(k));
  }
  slam::init_pose_graph(db); // initial = the ghosted poses; odometry from them
}

Eigen::Vector3d position(const ProjectDB &db, int id) {
  return pg::matrix_from_row_major(db.sensor_frame_pose(id))
      .topRightCorner<3, 1>();
}

} // namespace

TEST_CASE("PoseEditsSolver_SameSurface_ConstrainsOnlyTheNormal",
          "[slam][pose_edits][optimize]") {
  TempPath tmp("test_pose_edits_same_surface");
  ProjectDB db(tmp.path);
  ghost_wall_project(db);
  std::vector<Eigen::Vector3d> before;
  for (int k = 0; k < 10; ++k)
    before.push_back(position(db, k));

  core::PoseEdits edits;
  core::PoseEdit s;
  s.id = "s-wall";
  s.kind = core::PoseEditKind::same_surface;
  s.nodes.first = 0;
  s.nodes.last = 4;
  s.nodes_b.first = 5;
  s.nodes_b.last = 9;
  // Regions in world xy; the wall is the plane z = 1.5 (a) / 1.4 (b), which
  // spans x and y, so the boxes just cover the views.
  s.region = std::array<double, 4>{-2.0, -2.0, 2.5, 2.0};
  s.region_b = s.region;
  s.sigma_m = 0.001;
  edits.edits.push_back(s);

  const auto r =
      geometry::optimize_sensor_poses(db, wall_options(), false, &edits);
  REQUIRE(r.converged);
  REQUIRE(r.edit_residuals.size() == 1);
  CHECK(r.edit_residuals[0].gnc_inlier);

  // The walls: pass a at z = 1.5 + z_a, pass b at 1.5 + z_b; both blocks
  // act as wholes, so every pair of frames agrees.
  for (const auto &[a, b] :
       {std::pair{0, 5}, std::pair{4, 9}, std::pair{2, 7}}) {
    const double gap_before = before[a].z() - before[b].z();
    const double gap_after = position(db, a).z() - position(db, b).z();
    INFO("frames " << a << "/" << b << ": ghost gap " << gap_before << " -> "
                   << gap_after << " m");
    CHECK(std::abs(gap_before) > 0.099);
    CHECK(std::abs(gap_after) < 0.01);
  }

  // The correction is along the normal (world z), not in the wall plane.
  for (int k = 5; k < 10; ++k) {
    const Eigen::Vector3d d = position(db, k) - before[k];
    INFO("frame " << k << " moved " << d.transpose());
    CHECK(std::abs(d.z()) > 0.95 * d.norm());
  }
}

TEST_CASE("PoseEditsSolver_MoveAndRejectEdge_ThroughTheDatabase",
          "[slam][pose_edits][optimize]") {
  TempPath tmp("test_pose_edits_move_db");
  ProjectDB db(tmp.path);
  ghost_wall_project(db);
  // A wrong stored loop edge for reject_edge to remove.
  {
    ProjectDB::PoseGraphEdge e;
    e.from_node_id = 2;
    e.to_node_id = 8;
    e.edge_type = std::string(pg::kTypeLoopClosure);
    e.source = std::string(pg::kSourceManual);
    e.status = std::string(pg::kStatusAuto);
    e.transform = translation_pose(5.0, 0.0, 0.0);
    db.upsert_pose_graph_edge(e);
  }

  core::PoseEdits edits;
  core::PoseEdit m;
  m.id = "m-pass-b";
  m.kind = core::PoseEditKind::move;
  m.nodes.first = 5;
  m.nodes.last = 9;
  m.delta.xyz = {0.0, 0.0, 0.10}; // undo the ghost
  m.sigma_trans_m = 0.002;
  m.sigma_rot_deg = 0.05;
  edits.edits.push_back(m);
  core::PoseEdit x;
  x.id = "x-bad-loop";
  x.kind = core::PoseEditKind::reject_edge;
  x.nodes.first = x.nodes.last = 8;
  x.nodes_b.first = x.nodes_b.last = 2;
  x.edge_type = std::string(pg::kTypeLoopClosure);
  edits.edits.push_back(x);

  const auto r =
      geometry::optimize_sensor_poses(db, wall_options(), false, &edits);
  REQUIRE(r.converged);
  CHECK(r.edges_rejected_by_edits == 1);
  REQUIRE(r.edit_residuals.size() == 1);
  CHECK(r.edit_residuals[0].tag == "m-pass-b");
  CHECK(r.edit_residuals[0].factors == 5);
  for (int k = 5; k < 10; ++k) {
    INFO("frame " << k << " at " << position(db, k).transpose());
    CHECK(std::abs(position(db, k).z()) < 0.01);
  }
  // The stored edge itself is untouched (the edit is a tombstone in memory).
  bool found = false;
  for (const auto &e : db.list_pose_graph_edges())
    if (e.from_node_id == 2 && e.to_node_id == 8) {
      found = true;
      CHECK(e.status == pg::kStatusAuto);
    }
  CHECK(found);
}

TEST_CASE("PoseEditsSolver_Relative_TombstonesTheStoredEdge",
          "[slam][pose_edits][optimize]") {
  // docs/design/pose-edits.md, ### relative: a relative edit leaves the
  // stored edge between a and b, in either direction, out of the solve, as
  // if it had been rejected -- the same as reject_edge, just implied by
  // naming the pair. Without that tombstone the stored edge and the edit
  // both act and the solve settles on a compromise instead of the edit.
  TempPath tmp("test_pose_edits_relative_tombstone");
  ProjectDB db(tmp.path);
  ghost_wall_project(db);
  // A stored loop-closure edge between 2 and 7 that claims the OPPOSITE of
  // the true relation (b 0.10 m ABOVE a; the ghost actually put it 0.10 m
  // below): a plausible wrong loop match, not an outlier so extreme that
  // GNC would zero it on its own.
  {
    ProjectDB::PoseGraphEdge e;
    e.from_node_id = 2;
    e.to_node_id = 7;
    e.edge_type = std::string(pg::kTypeLoopClosure);
    e.source = std::string(pg::kSourceManual);
    e.status = std::string(pg::kStatusAuto);
    e.transform = translation_pose(0.0, 0.0, 0.10);
    db.upsert_pose_graph_edge(e);
  }

  // The relative edit restates the TRUE relation ('current', i.e. the
  // poses exactly as seeded): if the stored edge above is left out of the
  // solve as the doc says, this edit is the only force on the pair and it
  // is already satisfied, so nothing should move.
  core::PoseEdits edits;
  core::PoseEdit r;
  r.id = "r-2-7";
  r.kind = core::PoseEditKind::relative;
  r.nodes.first = r.nodes.last = 2;
  r.nodes_b.first = r.nodes_b.last = 7;
  r.pose.source = core::PoseSpec::Source::current;
  edits.edits.push_back(r);

  const double gap_before = position(db, 2).z() - position(db, 7).z();
  REQUIRE(std::abs(gap_before - 0.10) < 1e-9);

  const auto res =
      geometry::optimize_sensor_poses(db, wall_options(), false, &edits);
  REQUIRE(res.converged);
  REQUIRE(res.edit_residuals.size() == 1);
  // The stored loop edge between 2 and 7 is tombstoned by the relative
  // edit naming that pair, exactly as reject_edge would have done.
  CHECK(res.edges_rejected_by_edits == 1);

  const double gap_after = position(db, 2).z() - position(db, 7).z();
  INFO("gap before " << gap_before << " m, after " << gap_after << " m");
  CHECK(std::abs(gap_after - 0.10) < 0.001);
}

TEST_CASE("PoseEditsSolver_EmptyEdits_SameAsNoEdits",
          "[slam][pose_edits][optimize]") {
  TempPath ta("test_pose_edits_empty_a"), tb("test_pose_edits_empty_b");
  ProjectDB a(ta.path), b(tb.path);
  ghost_wall_project(a);
  ghost_wall_project(b);
  auto o = wall_options();
  o.use_plane_factors = true;
  const core::PoseEdits none;
  const auto ra = geometry::optimize_sensor_poses(a, o);
  const auto rb = geometry::optimize_sensor_poses(b, o, false, &none);
  CHECK(ra.final_error == rb.final_error);
  for (int k = 0; k < 10; ++k)
    CHECK(a.sensor_frame_pose(k) == b.sensor_frame_pose(k));
}

TEST_CASE("PoseEditLowering_StoreyFrame_MapsPosesAndAxes",
          "[slam][pose_edits]") {
  TempPath tmp("test_pose_edits_storey");
  ProjectDB db(tmp.path);
  ghost_wall_project(db);
  // A storey whose level frame is the world turned 90 deg about z and
  // lowered 1 m: level = Rz(90) * world + (0, 0, -1).
  core::StoreyRecord s;
  s.guid = "st-0";
  s.level_transform = {0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 1};
  db.save_storeys({s});

  core::PoseEdits edits;
  edits.frame = "storey:st-0";
  core::PoseEdit a;
  a.id = "a-level";
  a.kind = core::PoseEditKind::anchor;
  a.nodes.first = a.nodes.last = 3;
  a.pose.source = core::PoseSpec::Source::given;
  a.pose.value.xyz = {1.0, 2.0, 3.0};
  a.dofs = core::Dof::x; // level x == world y
  edits.edits.push_back(a);

  std::vector<int> order;
  for (int k = 0; k < 10; ++k)
    order.push_back(k);
  const auto low = slam::lower_pose_edits(db, edits, order);
  REQUIRE(low.priors.size() == 1);
  // Level (1, 2, 3) is world (2, -1, 4).
  CHECK((low.priors[0].pose.topRightCorner<3, 1>() -
         Eigen::Vector3d(2.0, -1.0, 4.0))
            .norm() < 1e-9);
  // The residual is taken in level axes, so the x mask is level x.
  const Matrix6d &I = low.priors[0].information;
  CHECK(I(3, 3) > 1.0);
  CHECK(std::abs(I(4, 4)) < 1e-9);
  CHECK(std::abs(I(5, 5)) < 1e-9);
  CHECK(
      (low.priors[0].axes * Eigen::Vector3d::UnitY() + Eigen::Vector3d::UnitX())
          .norm() < 1e-9); // world y is level -x

  edits.frame = "storey:nope";
  CHECK_THROWS(slam::lower_pose_edits(db, edits, order));
}
