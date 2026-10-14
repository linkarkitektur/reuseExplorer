// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

// Rediger's correspondence alignment (reusex-xlg.2). The fit reuses
// LoopClosure's RANSAC 3D-3D (detail::ransac_rigid); matching (reusex-xlg.2.2)
// reuses the panorama front-end's ORB + depth lifting (pano_detail).

#include "slam/frame_pair_matches.hpp"

#include "slam/LoopClosure.hpp"

#include "core/ProjectDB.hpp"
#include "core/SensorIntrinsics.hpp"
#include "core/logging.hpp"
#include "panorama_features.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

#include <fmt/format.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <stdexcept>

namespace reusex::slam {
namespace {

using RowMajor4d = Eigen::Matrix<double, 4, 4, Eigen::RowMajor>;

Eigen::Matrix4d to_eigen(const std::array<double, 16> &a) {
  return Eigen::Map<const RowMajor4d>(a.data());
}

std::array<double, 16> to_array(const Eigen::Matrix4d &m) {
  std::array<double, 16> a{};
  Eigen::Map<RowMajor4d>(a.data()) = m;
  return a;
}

bool finite(const Point3 &p) {
  return std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2]);
}

namespace pd = reusex::geometry::pano_detail;

/// Throws, naming the frame, when it cannot be matched: no depth, no pose,
/// or intrinsics that cannot place a pixel. (A missing colour image is
/// diagnosed after extraction, which decodes it anyway.)
void require_frame(const ProjectDB &db, int id, bool need_pose) {
  if (db.sensor_frame_depth(id).empty())
    throw std::runtime_error(
        fmt::format("frame {} has no stored depth image", id));
  if (need_pose && !db.has_sensor_frame_pose(id))
    throw std::runtime_error(fmt::format("frame {} has no stored pose", id));
  const auto k = db.sensor_frame_intrinsics(id);
  if (k.width <= 0 || k.height <= 0 || k.fx <= 0.0 || k.fy <= 0.0)
    throw std::runtime_error(
        fmt::format("frame {} has unusable intrinsics ({}x{}, f {:.1f}/{:.1f})",
                    id, k.width, k.height, k.fx, k.fy));
}

Eigen::Matrix4d local_of(const ProjectDB &db, int id) {
  return to_eigen(db.sensor_frame_intrinsics(id).local_transform);
}

/// A keypoint at colour pixel (u, v); @p optical (if any) is lifted into the
/// stored-pose frame: local_transform · optical point.
FrameKeypoint keypoint(float u, float v,
                       const std::optional<Eigen::Vector3d> &optical,
                       const Eigen::Matrix4d &local) {
  FrameKeypoint k;
  k.u = u;
  k.v = v;
  if (optical) {
    const Eigen::Vector3d p = (local * optical->homogeneous()).head<3>();
    k.xyz = std::array<double, 3>{p.x(), p.y(), p.z()};
  }
  return k;
}

std::vector<FrameKeypoint> keypoints_of(const pd::FrameFeatures &f,
                                        const Eigen::Matrix4d &local) {
  std::vector<FrameKeypoint> out;
  out.reserve(f.keypoints.size());
  for (std::size_t i = 0; i < f.keypoints.size(); ++i)
    out.push_back(
        keypoint(f.keypoints[i].pt.x, f.keypoints[i].pt.y,
                 f.valid[i] ? std::optional(f.local[i]) : std::nullopt, local));
  return out;
}

std::optional<cv::Rect> to_cv(const std::optional<PixelRect> &r) {
  if (!r || r->w <= 0 || r->h <= 0)
    return std::nullopt;
  return cv::Rect(r->x, r->y, r->w, r->h);
}

std::string describe(const std::optional<PixelRect> &r) {
  if (!to_cv(r))
    return "whole image";
  return fmt::format("region {}x{} at ({}, {})", r->w, r->h, r->x, r->y);
}

} // namespace

RigidFit rigid_from_matches(const std::array<double, 16> &pose_a,
                            const std::array<double, 16> &pose_b,
                            const std::vector<PointPair> &pairs,
                            double inlier_dist_m, int iterations,
                            std::uint32_t seed) {
  if (iterations <= 0)
    throw std::invalid_argument(fmt::format(
        "rigid_from_matches: iterations must be positive, got {}", iterations));
  if (!std::isfinite(inlier_dist_m) || inlier_dist_m < 0.0)
    throw std::invalid_argument(fmt::format(
        "rigid_from_matches: inlier_dist_m must be finite and >= 0, got {}",
        inlier_dist_m));
  for (const auto *p : {&pose_a, &pose_b})
    for (double x : *p)
      if (!std::isfinite(x))
        throw std::invalid_argument(
            fmt::format("rigid_from_matches: pose {} is not finite",
                        p == &pose_a ? "A" : "B"));

  RigidFit fit;
  const int n = static_cast<int>(pairs.size());
  for (int k = 0; k < n; ++k)
    if (!finite(pairs[static_cast<std::size_t>(k)].first) ||
        !finite(pairs[static_cast<std::size_t>(k)].second))
      throw std::invalid_argument(
          fmt::format("rigid_from_matches: pair {} is not finite", k));
  if (n < 3) {
    fit.failure = RigidFitFailure::too_few_pairs;
    fit.reason = fmt::format("3 matches with depth needed, {} given", n);
    return fit;
  }

  // Fit in A's frame (well-conditioned whatever the world origin):
  // src = T_A^-1 T_B p_b, dst = p_a, dst ≈ M src.
  const Eigen::Matrix4d TA = to_eigen(pose_a);
  const Eigen::Matrix4d b_in_a = TA.inverse() * to_eigen(pose_b);
  Eigen::Matrix3Xd src(3, n), dst(3, n);
  for (int k = 0; k < n; ++k) {
    const auto &[pa, pb] = pairs[static_cast<std::size_t>(k)];
    src.col(k) = (b_in_a * Eigen::Vector4d(pb[0], pb[1], pb[2], 1.0)).head<3>();
    dst.col(k) = Eigen::Vector3d(pa[0], pa[1], pa[2]);
  }
  geometry::LoopClosureOptions o;
  o.ransac_iterations = iterations;
  o.ransac_inlier_dist = static_cast<float>(inlier_dist_m);
  std::mt19937 rng(seed);
  Eigen::Matrix4d M_fit = Eigen::Matrix4d::Identity();
  const std::vector<int> consensus =
      geometry::detail::ransac_rigid(src, dst, o, rng, M_fit);

  // ransac_rigid returns the best HYPOTHESIS' inliers with the transform
  // refit on them; the refit can gain or lose pairs at the gate. Report the
  // inliers and RMS of the transform actually returned.
  std::vector<int> inliers;
  double sq = 0.0;
  if (consensus.size() >= 3) {
    const double thr2 = inlier_dist_m * inlier_dist_m;
    for (int k = 0; k < n; ++k) {
      const double e2 =
          ((M_fit * src.col(k).homogeneous()).head<3>() - dst.col(k))
              .squaredNorm();
      if (e2 <= thr2) {
        inliers.push_back(k);
        sq += e2;
      }
    }
  }
  const int m = static_cast<int>(inliers.size());
  if (m < 3) {
    fit.failure = RigidFitFailure::no_consensus;
    fit.inliers = std::move(inliers);
    fit.reason =
        fmt::format("no rigid fit: {} of {} pairs agree within {:.3f} m", m, n,
                    inlier_dist_m);
    return fit;
  }

  // Collinear inliers leave the rotation about their line free. The RMS
  // distance of the (centred) inliers off their best-fit line is
  // sqrt((s2^2 + s3^2) / m) from the singular values s1 >= s2 >= s3.
  Eigen::Matrix3Xd d(3, m);
  for (int k = 0; k < m; ++k)
    d.col(k) = dst.col(inliers[static_cast<std::size_t>(k)]);
  d.colwise() -= d.rowwise().mean();
  const Eigen::JacobiSVD<Eigen::Matrix3Xd> svd(d);
  const Eigen::Vector3d s = svd.singularValues();
  const double spread =
      std::sqrt((s(1) * s(1) + s(2) * s(2)) / static_cast<double>(m));
  if (spread < kMinFitSpreadM) {
    fit.failure = RigidFitFailure::degenerate;
    fit.inliers = std::move(inliers);
    fit.reason = fmt::format("inlier points are (nearly) collinear: {:.4f} m "
                             "RMS off their line, {:.3f} m needed",
                             spread, kMinFitSpreadM);
    return fit;
  }

  fit.rms_m = std::sqrt(sq / m);
  fit.world_delta = to_array(TA * M_fit * TA.inverse());
  fit.inliers = std::move(inliers);
  fit.ok = true;
  return fit;
}

FramePairMatches match_frame_pair(const ProjectDB &db, int a_id, int b_id,
                                  const FrameMatchOptions &opts) {
  if (a_id == b_id)
    throw std::invalid_argument(
        fmt::format("match_frame_pair: A and B are the same frame ({})", a_id));
  require_frame(db, a_id, /*need_pose=*/true);
  require_frame(db, b_id, /*need_pose=*/true);

  auto orb = cv::ORB::create(opts.max_features);
  const pd::FrameFeatures fa = pd::extract_frame_features(
      db, a_id, orb, opts.min_depth_m, opts.max_depth_m, to_cv(opts.region_a));
  const pd::FrameFeatures fb = pd::extract_frame_features(
      db, b_id, orb, opts.min_depth_m, opts.max_depth_m, to_cv(opts.region_b));
  for (const auto &[id, f] : {std::pair{a_id, &fa}, std::pair{b_id, &fb}})
    if (f->keypoints.empty() && db.sensor_frame_image(id).empty())
      throw std::runtime_error(
          fmt::format("frame {} has no stored colour image", id));

  FramePairMatches out;
  out.keypoints_a = keypoints_of(fa, local_of(db, a_id));
  out.keypoints_b = keypoints_of(fb, local_of(db, b_id));
  if (fa.descriptors.empty() || fb.descriptors.empty()) {
    reusex::warn("match_frame_pair: nothing to match: {} keypoints in frame {} "
                 "({}), {} in frame {} ({})",
                 fa.keypoints.size(), a_id, describe(opts.region_a),
                 fb.keypoints.size(), b_id, describe(opts.region_b));
    return out;
  }
  // LoopClosure's matcher: B queried against A, Lowe ratio test.
  cv::BFMatcher matcher(cv::NORM_HAMMING);
  std::vector<std::vector<cv::DMatch>> knn;
  matcher.knnMatch(fb.descriptors, fa.descriptors, knn, 2);
  for (const auto &m : knn) {
    if (m.size() < 2 || m[0].distance > opts.ratio_test * m[1].distance)
      continue;
    out.matches.push_back({m[0].trainIdx, m[0].queryIdx, m[0].distance});
  }
  reusex::debug("match_frame_pair: {} matches between frames {} ({} "
                "keypoints) and {} ({} keypoints)",
                out.matches.size(), a_id, fa.keypoints.size(), b_id,
                fb.keypoints.size());
  return out;
}

std::optional<FrameKeypoint> lift_pixel(const ProjectDB &db, int frame_id,
                                        float u, float v, int snap_px,
                                        float min_depth_m, float max_depth_m) {
  require_frame(db, frame_id, /*need_pose=*/false);
  const cv::Mat color = db.sensor_frame_image(frame_id);
  if (color.empty())
    throw std::runtime_error(
        fmt::format("frame {} has no stored colour image", frame_id));
  if (!(u >= 0.0f && v >= 0.0f && u <= color.cols - 1.0f &&
        v <= color.rows - 1.0f))
    return std::nullopt;
  cv::Mat depth_m;
  db.sensor_frame_depth(frame_id).convertTo(depth_m, CV_32FC1, 1.0 / 1000.0);
  const double sx = static_cast<double>(depth_m.cols) / color.cols;
  const double sy = static_cast<double>(depth_m.rows) / color.rows;
  const pd::DepthPinhole k = pd::depth_pinhole(
      db.sensor_frame_intrinsics(frame_id), depth_m.cols, depth_m.rows);
  // The keypoint mapping (extract_frame_features): colour px -> depth px.
  const int u0 = static_cast<int>(std::lround(u * sx));
  const int v0 = static_cast<int>(std::lround(v * sy));
  const int r =
      std::max(0, static_cast<int>(std::lround(snap_px * std::max(sx, sy))));
  // Nearest valid depth pixel within r; ties: first in row-major order.
  int best_d2 = std::numeric_limits<int>::max(), bu = 0, bv = 0;
  std::optional<Eigen::Vector3d> best;
  for (int dv = -r; dv <= r; ++dv)
    for (int du = -r; du <= r; ++du) {
      const int d2 = du * du + dv * dv;
      if (d2 > r * r || d2 >= best_d2)
        continue;
      if (auto p = pd::lift_depth_pixel(depth_m, k, u0 + du, v0 + dv,
                                        min_depth_m, max_depth_m)) {
        best = p;
        best_d2 = d2;
        bu = u0 + du;
        bv = v0 + dv;
      }
    }
  if (!best)
    return std::nullopt;
  return keypoint(static_cast<float>(bu / sx), static_cast<float>(bv / sy),
                  best, local_of(db, frame_id));
}

} // namespace reusex::slam
