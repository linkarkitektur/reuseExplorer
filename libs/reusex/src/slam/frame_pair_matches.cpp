// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

// Rediger's correspondence alignment (reusex-xlg.2). The fit reuses
// LoopClosure's RANSAC 3D-3D (detail::ransac_rigid); matching (reusex-xlg.2.2)
// reuses the panorama front-end's ORB + depth lifting (pano_detail).

#include "slam/frame_pair_matches.hpp"

#include "slam/LoopClosure.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <fmt/format.h>

#include <cmath>
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

} // namespace

RigidFit rigid_from_matches(const std::array<double, 16> &pose_a,
                            const std::array<double, 16> &pose_b,
                            const std::vector<PointPair> &pairs,
                            double inlier_dist_m, int iterations,
                            std::uint32_t seed) {
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
  std::vector<int> inliers =
      geometry::detail::ransac_rigid(src, dst, o, rng, M_fit);
  const int m = static_cast<int>(inliers.size());
  if (m < 3) {
    fit.failure = RigidFitFailure::no_consensus;
    fit.inliers = std::move(inliers);
    fit.reason =
        fmt::format("no rigid fit: {} of {} pairs agree within {:.3f} m", m, n,
                    inlier_dist_m);
    return fit;
  }

  // Collinear inliers leave the rotation about their line free.
  Eigen::Matrix3Xd d(3, m);
  for (int k = 0; k < m; ++k)
    d.col(k) = dst.col(inliers[static_cast<std::size_t>(k)]);
  d.colwise() -= d.rowwise().mean();
  const Eigen::JacobiSVD<Eigen::Matrix3Xd> svd(d);
  const double spread =
      svd.singularValues()(1) / std::sqrt(static_cast<double>(m));
  if (spread < kMinFitSpreadM) {
    fit.failure = RigidFitFailure::degenerate;
    fit.inliers = std::move(inliers);
    fit.reason = fmt::format("inlier points are (nearly) collinear: {:.4f} m "
                             "spread off their line, {:.3f} m needed",
                             spread, kMinFitSpreadM);
    return fit;
  }

  double sq = 0.0;
  for (int i : inliers)
    sq += ((M_fit * src.col(i).homogeneous()).head<3>() - dst.col(i))
              .squaredNorm();
  fit.rms_m = std::sqrt(sq / m);
  fit.world_delta = to_array(TA * M_fit * TA.inverse());
  fit.inliers = std::move(inliers);
  fit.ok = true;
  return fit;
}

} // namespace reusex::slam
