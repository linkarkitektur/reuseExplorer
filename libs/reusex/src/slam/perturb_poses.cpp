// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Synthetic trajectory drift (#338). See slam/perturb_poses.hpp for the model
// and the reason it exists.

#include "slam/perturb_poses.hpp"

#include "core/ProjectDB.hpp"
#include "core/logging.hpp"
#include "core/pose_graph.hpp"
#include "geometry/transform_utils.hpp"
#include "slam/LoopClosure.hpp"
#include "slam/pose_graph.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <random>
#include <stdexcept>
#include <vector>

namespace reusex::geometry {

namespace {

using Vector6d = se3::Vector6d;

/// Median of a vector, by value (sorts a copy). Empty -> 0.
double median_of(std::vector<double> v) {
  if (v.empty())
    return 0.0;
  const size_t mid = v.size() / 2;
  std::nth_element(v.begin(), v.begin() + mid, v.end());
  return v[mid];
}

/// Deterministic stride for the pair-disagreement sample. Bounded work
/// (~64x64/2 pairs) regardless of sequence length, and a pure function of the
/// frame count, so the statistic is reproducible across runs and machines
/// (STANDARDS §6).
int pair_stride(int n) { return std::max(1, n / 64); }

/// Row-major double[16] -> 4x4, without the float round-trip to_affine() would
/// impose. Matters here: a zero-amplitude perturbation must return the stored
/// poses BIT-IDENTICAL, which is the no-op guard the benchmark relies on.
Eigen::Matrix4d from_array16(const std::array<double, 16> &m) {
  Eigen::Matrix4d M;
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c)
      M(r, c) = m[r * 4 + c];
  return M;
}

std::array<double, 16> to_array16_double(const Eigen::Matrix4d &M) {
  std::array<double, 16> m{};
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c)
      m[r * 4 + c] = M(r, c);
  return m;
}

/// Angle of the rotation taking A's orientation to B's, in degrees.
double rotation_angle_deg(const Eigen::Matrix4d &A, const Eigen::Matrix4d &B) {
  const Eigen::Matrix3d dR =
      A.block<3, 3>(0, 0).transpose() * B.block<3, 3>(0, 0);
  const double c = std::clamp((dR.trace() - 1.0) * 0.5, -1.0, 1.0);
  return std::acos(c) * 180.0 / M_PI;
}

} // namespace

double median_pair_disagreement(const std::vector<Eigen::Matrix4d> &reference,
                                const std::vector<Eigen::Matrix4d> &other,
                                int min_frame_gap) {
  const int n = static_cast<int>(reference.size());
  if (n < 2 || static_cast<int>(other.size()) != n)
    return 0.0;
  // Clamp the gap so short sequences still yield samples: a 40-frame scan has
  // no pairs 50 apart, and silently returning 0 would let the calibration
  // believe it had achieved any target (STANDARDS §5 — no silent degeneracy).
  const int gap = std::max(1, std::min(min_frame_gap, n / 4));
  const int stride = pair_stride(n);

  std::vector<double> d;
  d.reserve(static_cast<size_t>(64 * 64));
  for (int i = 0; i < n; i += stride) {
    for (int j = i + gap; j < n; j += stride) {
      const Eigen::Matrix4d rel_ref = reference[i].inverse() * reference[j];
      const Eigen::Matrix4d rel_oth = other[i].inverse() * other[j];
      d.push_back(
          (rel_ref.block<3, 1>(0, 3) - rel_oth.block<3, 1>(0, 3)).norm());
    }
  }
  return median_of(std::move(d));
}

PoseDriftResult perturb_trajectory(std::vector<Eigen::Matrix4d> &poses,
                                   const PoseDriftOptions &opt) {
  PoseDriftResult r;
  const int n = static_cast<int>(poses.size());
  r.frames = n;
  if (n < 2)
    return r;

  r.trajectory_extent = trajectory_extent(poses);

  // Seed relative chain. The perturbation is applied here, not to the absolute
  // poses, because that is where real drift is born (§8.3: the error is spread
  // smoothly across thousands of individually-good relative measurements).
  std::vector<Eigen::Matrix4d> rel(n - 1);
  std::vector<double> step(n - 1);
  for (int i = 0; i + 1 < n; ++i) {
    rel[i] = poses[i].inverse() * poses[i + 1];
    step[i] = rel[i].block<3, 1>(0, 3).norm();
    r.path_length += step[i];
  }

  const double target = opt.target_drift_ratio * opt.drift_scale;
  if (!(target > 0.0) || !(r.trajectory_extent > 0.0)) {
    // drift_scale 0 (or a degenerate trajectory): return the input untouched
    // and bit-identical. This is the no-op guard the benchmark uses to prove
    // the harness itself injects nothing.
    r.calibrated = true;
    return r;
  }

  // --- Draw the noise realisation ONCE ------------------------------------
  // The calibration below rescales this fixed realisation. Drawing inside the
  // search would make every trial a different trajectory and the solve
  // meaningless.
  std::mt19937 rng(opt.seed);
  std::normal_distribution<double> gauss(0.0, 1.0);

  std::vector<Vector6d> walk(n - 1);
  std::vector<Vector6d> bias(n - 1);
  Vector6d b;
  for (int k = 0; k < 6; ++k)
    b[k] = gauss(rng);
  const double L = std::max(1e-6, opt.bias_correlation_length);
  for (int i = 0; i + 1 < n; ++i) {
    // Ornstein-Uhlenbeck step over the distance actually travelled, so the
    // systematic component decorrelates per metre rather than per frame — a
    // scan that pauses does not accumulate bias while standing still.
    const double rho = std::exp(-step[i] / L);
    const double mix = std::sqrt(std::max(0.0, 1.0 - rho * rho));
    for (int k = 0; k < 6; ++k)
      b[k] = rho * b[k] + mix * gauss(rng);
    bias[i] = b;
    for (int k = 0; k < 6; ++k)
      walk[i][k] = gauss(rng);
  }

  // --- Integrate the drifted chain at amplitude `a` ------------------------
  std::vector<Eigen::Matrix4d> drifted(n);
  const bool yaw_only = opt.mode == DriftMode::yaw;
  const Eigen::Vector3d up = opt.up.normalized();
  auto integrate = [&](double a) {
    drifted[0] = poses[0]; // gauge: frame 0 stays on the seed
    for (int i = 0; i + 1 < n; ++i) {
      const double s = step[i];
      const double rs = std::sqrt(s);
      Vector6d xi;
      xi.head<3>() = a * (opt.rot_bias_gain * s * bias[i].head<3>() +
                          opt.rot_walk_gain * rs * walk[i].head<3>());
      xi.tail<3>() = a * (opt.trans_bias_gain * s * bias[i].tail<3>() +
                          opt.trans_walk_gain * rs * walk[i].tail<3>());
      const Eigen::Matrix4d predicted = drifted[i] * rel[i];
      if (yaw_only) {
        // Rotate only about the world up axis (expressed in the body frame of
        // the predicted pose) and translate only horizontally. Both survive
        // se3::exp: its V matrix maps a vector orthogonal to the rotation
        // axis to one orthogonal to it, so heights and tilts stay on the seed
        // exactly. One noise channel per DoF: the calibration rescales the
        // amplitude, so the reduced number of channels changes nothing else.
        const Eigen::Matrix3d R = predicted.block<3, 3>(0, 0);
        const Eigen::Vector3d up_body = R.transpose() * up;
        const double yaw = a * (opt.rot_bias_gain * s * bias[i][2] +
                                opt.rot_walk_gain * rs * walk[i][2]);
        Eigen::Vector3d t_world = R * xi.tail<3>();
        t_world -= t_world.dot(up) * up;
        xi.head<3>() = yaw * up_body;
        xi.tail<3>() = R.transpose() * t_world;
      }
      drifted[i + 1] = predicted * se3::exp(xi);
    }
  };
  auto ratio_at = [&](double a) {
    integrate(a);
    return median_pair_disagreement(poses, drifted, opt.min_frame_gap) /
           r.trajectory_extent;
  };

  // --- Solve for the amplitude that realises the requested ratio ----------
  // Bracket by doubling, then bisect. ratio_at(0) == 0 by construction, so a
  // bracket exists as soon as the upper end overshoots.
  double lo = 0.0, hi = 1.0;
  double hi_ratio = ratio_at(hi);
  int expansions = 0;
  while (hi_ratio < target && expansions < 40) {
    lo = hi;
    hi *= 2.0;
    hi_ratio = ratio_at(hi);
    ++expansions;
  }
  if (hi_ratio < target) {
    // Not reachable with this drift character — report loudly rather than
    // silently benchmarking a milder scan than asked for (STANDARDS §5).
    core::warn("PoseDrift: could not reach target drift ratio {:.3f} "
               "(reached {:.3f} at amplitude {:.4g}); using the largest "
               "amplitude found. Raise the base gains or lower drift_scale.",
               target, hi_ratio, hi);
    r.amplitude = hi;
    r.calibrated = false;
  } else {
    for (int it = 0; it < opt.calibration_iterations; ++it) {
      const double mid = 0.5 * (lo + hi);
      const double f = ratio_at(mid);
      if (std::abs(f - target) <= opt.calibration_tolerance * target) {
        lo = hi = mid;
        break;
      }
      if (f < target)
        lo = mid;
      else
        hi = mid;
    }
    r.amplitude = 0.5 * (lo + hi);
    r.calibrated = true;
  }

  integrate(r.amplitude);

  // --- Realised statistics -------------------------------------------------
  std::vector<double> disp(n);
  for (int i = 0; i < n; ++i) {
    disp[i] =
        (drifted[i].block<3, 1>(0, 3) - poses[i].block<3, 1>(0, 3)).norm();
    r.max_position_error = std::max(r.max_position_error, disp[i]);
    r.max_rotation_error = std::max(r.max_rotation_error,
                                    rotation_angle_deg(poses[i], drifted[i]));
  }
  r.final_position_error = disp.back();
  r.median_position_error = median_of(disp);
  r.median_pair_disagreement =
      median_pair_disagreement(poses, drifted, opt.min_frame_gap);
  r.drift_ratio = r.median_pair_disagreement / r.trajectory_extent;

  // A drift request can be arithmetically satisfiable and still physically
  // meaningless: on a capture that orbits a small room, hitting a large
  // fraction of the (small) extent needs orientation errors no SLAM front-end
  // would ever produce, and a benchmark built on that measures an
  // impossibility rather than a capability. Say so with the number
  // (STANDARDS §5) instead of quietly returning it.
  constexpr double kImplausibleRotationDeg = 60.0;
  if (r.max_rotation_error > kImplausibleRotationDeg)
    core::warn("PoseDrift: worst-frame orientation error is {:.1f} deg — "
               "beyond a plausible SLAM drift regime. Requested ratio {:.3f} "
               "of a {:.2f} m extent over a {:.2f} m path; lower drift_scale "
               "if the benchmark is meant to be recoverable.",
               r.max_rotation_error, target, r.trajectory_extent,
               r.path_length);

  poses = std::move(drifted);
  return r;
}

PoseDriftResult perturb_sensor_poses(ProjectDB &db, const PoseDriftOptions &opt,
                                     bool dry_run) {
  auto ids = db.sensor_frame_ids();
  std::sort(ids.begin(), ids.end()); // temporal ordering, as the optimizer uses

  std::vector<int> posed_ids;
  std::vector<Eigen::Matrix4d> poses;
  posed_ids.reserve(ids.size());
  poses.reserve(ids.size());
  for (int id : ids) {
    if (!db.has_sensor_frame_pose(id))
      continue; // poseless frames are left alone (#330)
    posed_ids.push_back(id);
    poses.push_back(from_array16(db.sensor_frame_pose(id)));
  }

  if (poses.size() < 2)
    throw std::runtime_error(
        "PoseDrift: fewer than 2 sensor frames carry a pose — nothing to "
        "perturb (import a capture with poses first)");

  const size_t skipped = ids.size() - posed_ids.size();
  if (skipped > 0)
    core::warn("PoseDrift: {} of {} sensor frames carry no pose and were left "
               "untouched",
               skipped, ids.size());

  const auto r = perturb_trajectory(poses, opt);

  core::info("PoseDrift: {} frames, extent {:.2f} m, path {:.2f} m, seed {}, "
             "drift_scale {:.3f}, mode {}",
             r.frames, r.trajectory_extent, r.path_length, opt.seed,
             opt.drift_scale, opt.mode == DriftMode::yaw ? "yaw" : "full");
  core::info("PoseDrift: amplitude {:.5g} ({}), realised drift ratio {:.3f} "
             "(target {:.3f}) = {:.3f} m median distant-pair disagreement",
             r.amplitude, r.calibrated ? "calibrated" : "NOT calibrated",
             r.drift_ratio, opt.target_drift_ratio * opt.drift_scale,
             r.median_pair_disagreement);
  core::info("PoseDrift: camera-centre error median {:.3f} m, max {:.3f} m, "
             "final {:.3f} m; max rotation error {:.2f} deg",
             r.median_position_error, r.max_position_error,
             r.final_position_error, r.max_rotation_error);

  if (dry_run) {
    core::info("PoseDrift: dry run — no poses written");
    return r;
  }

  for (size_t k = 0; k < posed_ids.size(); ++k)
    db.update_sensor_frame_pose(posed_ids[k], to_array16_double(poses[k]));
  core::info("PoseDrift: wrote {} drifted poses back to the database",
             posed_ids.size());

  // The drifted trajectory stands in for a new INPUT capture: `rux optimize`
  // seeds from initial_transform (spec 2026-10-13 §4.3), and odometry derived
  // from the old poses would quietly undo the drift.
  namespace pg = core::pose_graph;
  const auto stored = db.list_pose_graph_edges();
  const bool has_init_edges =
      std::any_of(stored.begin(), stored.end(), [](const auto &e) {
        return e.source == pg::kSourceOdometryInit &&
               e.status != pg::kStatusRejected;
      });
  const auto foreign =
      std::count_if(stored.begin(), stored.end(), [](const auto &e) {
        return e.edge_type == pg::kTypeOdometry &&
               e.source != pg::kSourceOdometryInit &&
               e.status != pg::kStatusRejected && e.transform.has_value();
      });
  if (has_init_edges) {
    slam::init_pose_graph(db, {.force = true});
  } else {
    for (size_t k = 0; k < posed_ids.size(); ++k)
      db.set_initial_pose(posed_ids[k], to_array16_double(poses[k]));
  }
  if (foreign > 0)
    core::warn("PoseDrift: {} stored odometry edges (e.g. from 'rux import "
               "rtabmap') were measured on the undrifted capture and will "
               "pull the solve back to it; reject them ('rux graph prune "
               "--source rtabmap --type odometry') for an honest drift "
               "benchmark",
               foreign);
  return r;
}

} // namespace reusex::geometry
