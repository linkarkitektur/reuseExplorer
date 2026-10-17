// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <reusex/slam/pose_metrics.hpp>

#include <reusex/core/logging.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <map>
#include <numbers>
#include <stdexcept>

namespace reusex::slam {

namespace {

struct MatchedPair {
  Eigen::Matrix4d est;
  Eigen::Matrix4d ref;
};

double rad_to_deg(double rad) { return rad * 180.0 / std::numbers::pi; }

/// Rotation angle between two rotation matrices, in degrees, in [0, 180].
///
/// Each matrix is first taken to a unit quaternion, the same projection
/// io::write_tum applies, and the angle is read with atan2 (Eigen's
/// angularDistance), not acos((trace - 1) / 2). A stored pose imported from
/// a float source is orthonormal only to ~1e-7, and acos is ill-conditioned
/// at 0: a 1e-8 trace error reads as ~1e-4 rad. That made a project scored
/// against its own 'export trajectory' report 0.009 deg (reusex-dh8).
double rotation_angle_deg(const Eigen::Matrix3d &a, const Eigen::Matrix3d &b) {
  const Eigen::Quaterniond qa = Eigen::Quaterniond(a).normalized();
  const Eigen::Quaterniond qb = Eigen::Quaterniond(b).normalized();
  return rad_to_deg(qa.angularDistance(qb));
}

/// @p pose with its rotation projected onto a unit quaternion and back, the
/// projection io::write_tum applies. A stored pose is orthonormal only to the
/// precision it was imported at; projecting both sides the same way makes a
/// project and its own TUM export identical to the printed digits.
Eigen::Matrix4d orthonormal(const Eigen::Matrix4d &pose) {
  Eigen::Matrix4d out = pose;
  out.topLeftCorner<3, 3>() =
      Eigen::Quaterniond(Eigen::Matrix3d(pose.topLeftCorner<3, 3>()))
          .normalized()
          .toRotationMatrix();
  return out;
}

double rmse(const std::vector<double> &v) {
  if (v.empty())
    return 0.0;
  double sum_sq = 0.0;
  for (double x : v)
    sum_sq += x * x;
  return std::sqrt(sum_sq / static_cast<double>(v.size()));
}

double median(std::vector<double> v) {
  if (v.empty())
    return 0.0;
  std::sort(v.begin(), v.end());
  const std::size_t n = v.size();
  return (n % 2 == 1) ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
}

/// Match two trajectories by EXACT timestamp (core/trajectory.hpp /
/// slam/pose_metrics.hpp explain why: both of this module's real uses --
/// comparing two exports of the same project, and a drifted trajectory
/// against its undrifted seed -- share identical timestamp values, not merely
/// close ones). Returned pairs are in ascending timestamp order.
std::vector<MatchedPair>
match_by_timestamp(std::span<const core::StampedPose> est,
                   std::span<const core::StampedPose> ref) {
  std::map<double, std::size_t> ref_by_ts;
  for (std::size_t i = 0; i < ref.size(); ++i)
    ref_by_ts.emplace(ref[i].timestamp, i); // first wins on a duplicate

  std::vector<std::size_t> est_order(est.size());
  for (std::size_t i = 0; i < est_order.size(); ++i)
    est_order[i] = i;
  std::sort(est_order.begin(), est_order.end(),
            [&](std::size_t a, std::size_t b) {
              return est[a].timestamp < est[b].timestamp;
            });

  std::vector<MatchedPair> pairs;
  pairs.reserve(std::min(est.size(), ref.size()));
  for (std::size_t i : est_order) {
    auto it = ref_by_ts.find(est[i].timestamp);
    if (it == ref_by_ts.end())
      continue;
    pairs.push_back(
        {orthonormal(est[i].pose), orthonormal(ref[it->second].pose)});
  }
  return pairs;
}

/// Umeyama alignment of `est` translations onto `ref` translations. Returns
/// (scale, rotation, translation) such that `scale * R * p_est + t ~= p_ref`.
struct Alignment {
  double scale = 1.0;
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  Eigen::Vector3d translation = Eigen::Vector3d::Zero();
};

Alignment compute_alignment(const std::vector<MatchedPair> &pairs,
                            bool with_scaling) {
  Eigen::Matrix3Xd src(3, static_cast<Eigen::Index>(pairs.size()));
  Eigen::Matrix3Xd dst(3, static_cast<Eigen::Index>(pairs.size()));
  for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(pairs.size()); ++i) {
    src.col(i) = pairs[static_cast<std::size_t>(i)].est.topRightCorner<3, 1>();
    dst.col(i) = pairs[static_cast<std::size_t>(i)].ref.topRightCorner<3, 1>();
  }
  const Eigen::Matrix4d T = Eigen::umeyama(src, dst, with_scaling);
  Alignment a;
  const Eigen::Matrix3d sr = T.topLeftCorner<3, 3>();
  a.scale = with_scaling ? sr.col(0).norm() : 1.0;
  a.rotation = sr / a.scale;
  a.translation = T.topRightCorner<3, 1>();
  return a;
}

Eigen::Matrix4d apply_alignment(const Alignment &a, const Eigen::Matrix4d &p) {
  Eigen::Matrix4d out = Eigen::Matrix4d::Identity();
  out.topLeftCorner<3, 3>() = a.rotation * p.topLeftCorner<3, 3>();
  out.topRightCorner<3, 1>() =
      a.scale * (a.rotation * p.topRightCorner<3, 1>()) + a.translation;
  return out;
}

/// Sturm et al. relative pose error between a (start, end) pair, no
/// alignment needed: a shared global offset cancels out of both relative
/// transforms.
void accumulate_rpe(const Eigen::Matrix4d &est_a, const Eigen::Matrix4d &est_b,
                    const Eigen::Matrix4d &ref_a, const Eigen::Matrix4d &ref_b,
                    std::vector<double> &trans_err,
                    std::vector<double> &rot_err_deg) {
  const Eigen::Matrix4d rel_est = est_a.inverse() * est_b;
  const Eigen::Matrix4d rel_ref = ref_a.inverse() * ref_b;
  const Eigen::Matrix4d err = rel_ref.inverse() * rel_est;
  trans_err.push_back(err.topRightCorner<3, 1>().norm());
  rot_err_deg.push_back(rotation_angle_deg(Eigen::Matrix3d::Identity(),
                                           err.topLeftCorner<3, 3>()));
}

} // namespace

PoseMetrics compare_trajectories(std::span<const core::StampedPose> est,
                                 std::span<const core::StampedPose> ref,
                                 const PoseMetricsOptions &opt) {
  if (opt.rpe_distance_m <= 0.0)
    throw std::invalid_argument(
        "compare_trajectories: rpe_distance_m must be > 0");
  if (opt.rpe_frames <= 0)
    throw std::invalid_argument("compare_trajectories: rpe_frames must be > 0");
  if (opt.segments <= 0)
    throw std::invalid_argument("compare_trajectories: segments must be > 0");

  const auto pairs = match_by_timestamp(est, ref);

  PoseMetrics m;
  m.matched = pairs.size();
  if (pairs.size() < 2) {
    core::warn("compare_trajectories: {} of {} est / {} ref poses matched by "
               "timestamp -- every metric is 0",
               pairs.size(), est.size(), ref.size());
    return m;
  }

  // --- ATE (needs alignment to be meaningful when requested) --------------
  Alignment alignment;
  if (opt.align != PoseMetricsOptions::Align::none)
    alignment =
        compute_alignment(pairs, opt.align == PoseMetricsOptions::Align::sim3);
  m.scale = alignment.scale;

  std::vector<double> pos_err(pairs.size());
  std::vector<double> rot_err_deg(pairs.size());
  for (std::size_t i = 0; i < pairs.size(); ++i) {
    const Eigen::Matrix4d aligned =
        opt.align == PoseMetricsOptions::Align::none
            ? pairs[i].est
            : apply_alignment(alignment, pairs[i].est);
    pos_err[i] =
        (pairs[i].ref.topRightCorner<3, 1>() - aligned.topRightCorner<3, 1>())
            .norm();
    rot_err_deg[i] = rotation_angle_deg(pairs[i].ref.topLeftCorner<3, 3>(),
                                        aligned.topLeftCorner<3, 3>());
  }
  m.ate_rmse_m = rmse(pos_err);
  m.ate_median_m = median(pos_err);
  m.ate_max_m = *std::max_element(pos_err.begin(), pos_err.end());
  m.rot_rmse_deg = rmse(rot_err_deg);

  // --- RPE at a fixed matched-index gap ------------------------------------
  {
    std::vector<double> trans, rot;
    const auto delta = static_cast<std::size_t>(opt.rpe_frames);
    if (delta < pairs.size()) {
      for (std::size_t i = 0; i + delta < pairs.size(); ++i)
        accumulate_rpe(pairs[i].est, pairs[i + delta].est, pairs[i].ref,
                       pairs[i + delta].ref, trans, rot);
    }
    m.rpe_trans_frames_m = rmse(trans);
    m.rpe_rot_frames_deg = rmse(rot);
  }

  // --- RPE at a fixed path length travelled (along the reference) ---------
  {
    std::vector<double> trans, rot;
    std::vector<double> cum(pairs.size(), 0.0);
    for (std::size_t i = 1; i < pairs.size(); ++i)
      cum[i] = cum[i - 1] + (pairs[i].ref.topRightCorner<3, 1>() -
                             pairs[i - 1].ref.topRightCorner<3, 1>())
                                .norm();
    std::size_t j = 0;
    for (std::size_t i = 0; i < pairs.size(); ++i) {
      if (j < i)
        j = i;
      while (j + 1 < pairs.size() && cum[j] - cum[i] < opt.rpe_distance_m)
        ++j;
      if (j < pairs.size() && cum[j] - cum[i] >= opt.rpe_distance_m && j > i)
        accumulate_rpe(pairs[i].est, pairs[j].est, pairs[i].ref, pairs[j].ref,
                       trans, rot);
    }
    m.rpe_trans_m = rmse(trans);
    m.rpe_rot_deg = rmse(rot);
  }

  // --- Per-segment ATE, under the one whole-trajectory alignment -----------
  const int segments = std::min(opt.segments, static_cast<int>(pairs.size()));
  m.segment_ate_m.resize(static_cast<std::size_t>(segments));
  for (int s = 0; s < segments; ++s) {
    const std::size_t begin = pairs.size() * static_cast<std::size_t>(s) /
                              static_cast<std::size_t>(segments);
    const std::size_t end =
        (s == segments - 1) ? pairs.size()
                            : pairs.size() * static_cast<std::size_t>(s + 1) /
                                  static_cast<std::size_t>(segments);
    std::vector<double> chunk(
        pos_err.begin() + static_cast<std::ptrdiff_t>(begin),
        pos_err.begin() + static_cast<std::ptrdiff_t>(end));
    m.segment_ate_m[static_cast<std::size_t>(s)] = rmse(chunk);
  }

  return m;
}

} // namespace reusex::slam
