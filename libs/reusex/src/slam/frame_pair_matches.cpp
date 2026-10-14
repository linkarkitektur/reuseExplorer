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

} // namespace reusex::slam
