// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// See PlaneGraphOptimizer.hpp for the method overview and attribution.

#include "slam/PlaneGraphOptimizer.hpp"
#include "core/logging.hpp"
#include "core/pose_graph.hpp"

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/OrientedPlane3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/GncOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/OrientedPlane3Factor.h>
#include <gtsam/slam/PriorFactor.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <optional>
#include <random>
#include <utility>
#include <vector>

namespace reusex::geometry {

namespace {

gtsam::Pose3 to_pose3(const Eigen::Matrix4d &T) {
  return gtsam::Pose3(gtsam::Rot3(T.block<3, 3>(0, 0)),
                      gtsam::Point3(T.block<3, 1>(0, 3)));
}

gtsam::SharedNoiseModel sigmas_noise(double sr, double st) {
  return gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << sr, sr, sr, st, st, st).finished());
}

gtsam::SharedNoiseModel
information_noise(const Eigen::Matrix<double, 6, 6> &I) {
  if (I.isDiagonal(1e-12))
    return gtsam::noiseModel::Diagonal::Precisions(gtsam::Vector(I.diagonal()));
  return gtsam::noiseModel::Gaussian::Information(I);
}

/// A pose-edit factor's noise: its (possibly rank-deficient) information,
/// regularised by 1e-9 on the diagonal so it stays a valid Gaussian.
gtsam::SharedNoiseModel edit_noise(const Eigen::Matrix<double, 6, 6> &I) {
  Eigen::Matrix<double, 6, 6> J = 0.5 * (I + I.transpose());
  J.diagonal().array() += 1e-9;
  return information_noise(J);
}

double wrap_pi(double a) { return std::remainder(a, 2.0 * M_PI); }

/// Absolute pose prior of a pose edit (reusex-rcv.1.3): residual
/// [roll, pitch, yaw, x, y, z] in the prior's axes (see PosePrior). gtsam's
/// PriorFactor<Pose3> measures the error in the camera's own frame, where
/// "x, y and yaw only" is not a mask at all; this one is exact.
class EditPosePriorFactor : public gtsam::NoiseModelFactorN<gtsam::Pose3> {
  using Base = gtsam::NoiseModelFactorN<gtsam::Pose3>;
  Eigen::Matrix3d axes_, C_;
  Eigen::Vector3d rpy_t_, t_t_;

  gtsam::Vector6 residual(const gtsam::Pose3 &pose) const {
    const Eigen::Vector3d rpy = prior_rpy(pose.rotation().matrix(), C_, axes_);
    gtsam::Vector6 e;
    e << wrap_pi(rpy[0] - rpy_t_[0]), wrap_pi(rpy[1] - rpy_t_[1]),
        wrap_pi(rpy[2] - rpy_t_[2]), axes_ * (pose.translation() - t_t_);
    return e;
  }

    public:
  EditPosePriorFactor(gtsam::Key key, const Eigen::Matrix4d &target,
                      const Eigen::Matrix3d &axes,
                      const gtsam::SharedNoiseModel &noise)
      : Base(noise, key), axes_(axes) {
    const Eigen::Matrix3d Rt = target.topLeftCorner<3, 3>();
    C_ = prior_levelling(Rt, axes_);
    rpy_t_ = prior_rpy(Rt, C_, axes_);
    t_t_ = target.topRightCorner<3, 1>();
  }
  gtsam::Vector evaluateError(
      const gtsam::Pose3 &pose,
      boost::optional<gtsam::Matrix &> H = boost::none) const override {
    if (H)
      *H = gtsam::numericalDerivative11<gtsam::Vector6, gtsam::Pose3>(
          [this](const gtsam::Pose3 &x) { return residual(x); }, pose, 1e-7);
    return residual(pose);
  }
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return gtsam::NonlinearFactor::shared_ptr(new EditPosePriorFactor(*this));
  }
};

/// Norm of @p e projected onto the range of the symmetric @p B (the
/// directions an edit constrains).
double constrained_norm(const Eigen::Vector3d &e, const Eigen::Matrix3d &B) {
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(0.5 * (B + B.transpose()));
  const double top = es.eigenvalues().maxCoeff();
  if (!(top > 0.0))
    return 0.0;
  double sq = 0.0;
  for (int k = 0; k < 3; ++k)
    if (es.eigenvalues()(k) > 1e-6 * top)
      sq += std::pow(es.eigenvectors().col(k).dot(e), 2);
  return std::sqrt(sq);
}

/// gtsam's OrientedPlane3Factor does not implement clone(), which
/// GncOptimizer requires (it clones the graph to build its robustified
/// inner copy and throws NonlinearFactor::clone() otherwise). Thin subclass
/// adding the missing override.
class ClonableOrientedPlane3Factor : public gtsam::OrientedPlane3Factor {
    public:
  using gtsam::OrientedPlane3Factor::OrientedPlane3Factor;
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return gtsam::NonlinearFactor::shared_ptr(
        new ClonableOrientedPlane3Factor(*this));
  }
};

/// Normal of an OrientedPlane3 as a unit 3-vector, with its Jacobian with
/// respect to the plane's 3-dim tangent space.
Eigen::Vector3d plane_normal(const gtsam::OrientedPlane3 &plane,
                             gtsam::Matrix33 *H) {
  gtsam::Matrix23 Hn;
  gtsam::Matrix32 Hp;
  const gtsam::Unit3 n = plane.normal(H ? &Hn : nullptr);
  const gtsam::Point3 v = n.point3(H ? &Hp : nullptr);
  if (H)
    *H = Hp * Hn;
  return v;
}

/// Manhattan prior on a VERTICAL plane landmark (reusex-rqn.1): its normal
/// must lie along the horizontal axis at azimuth `theta + offset` (or its
/// opposite). Residual (2-DoF, ~rad): [n.up, n.perp(theta)] — the wall's tilt
/// from vertical and its azimuth error. `theta` is a shared scalar variable,
/// so the grid's azimuth is estimated jointly with the poses.
class ManhattanWallFactor
    : public gtsam::NoiseModelFactorN<gtsam::OrientedPlane3, double> {
  using Base = gtsam::NoiseModelFactorN<gtsam::OrientedPlane3, double>;
  Eigen::Vector3d up_, e1_, e2_;
  double offset_;

    public:
  ManhattanWallFactor(gtsam::Key plane, gtsam::Key theta,
                      const Eigen::Vector3d &up, const Eigen::Vector3d &e1,
                      const Eigen::Vector3d &e2, double offset,
                      const gtsam::SharedNoiseModel &noise)
      : Base(noise, plane, theta), up_(up), e1_(e1), e2_(e2), offset_(offset) {}
  gtsam::Vector evaluateError(
      const gtsam::OrientedPlane3 &plane, const double &theta,
      boost::optional<gtsam::Matrix &> H1 = boost::none,
      boost::optional<gtsam::Matrix &> H2 = boost::none) const override {
    gtsam::Matrix33 Hv;
    const Eigen::Vector3d v = plane_normal(plane, H1 ? &Hv : nullptr);
    const double a = theta + offset_;
    const Eigen::Vector3d dir = std::cos(a) * e1_ + std::sin(a) * e2_;
    const Eigen::Vector3d perp = -std::sin(a) * e1_ + std::cos(a) * e2_;
    if (H1) {
      gtsam::Matrix23 P;
      P.row(0) = up_.transpose();
      P.row(1) = perp.transpose();
      *H1 = P * Hv;
    }
    if (H2) // d(perp)/d(theta) = -dir
      *H2 = (gtsam::Matrix21() << 0.0, -v.dot(dir)).finished();
    return gtsam::Vector2(v.dot(up_), v.dot(perp));
  }
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return gtsam::NonlinearFactor::shared_ptr(new ManhattanWallFactor(*this));
  }
};

/// Manhattan prior on a HORIZONTAL plane landmark (floor, ceiling, table top):
/// its normal must be +-up. Residual: the normal's two horizontal components.
class ManhattanLevelFactor
    : public gtsam::NoiseModelFactorN<gtsam::OrientedPlane3> {
  using Base = gtsam::NoiseModelFactorN<gtsam::OrientedPlane3>;
  Eigen::Vector3d e1_, e2_;

    public:
  ManhattanLevelFactor(gtsam::Key plane, const Eigen::Vector3d &e1,
                       const Eigen::Vector3d &e2,
                       const gtsam::SharedNoiseModel &noise)
      : Base(noise, plane), e1_(e1), e2_(e2) {}
  gtsam::Vector evaluateError(
      const gtsam::OrientedPlane3 &plane,
      boost::optional<gtsam::Matrix &> H = boost::none) const override {
    gtsam::Matrix33 Hv;
    const Eigen::Vector3d v = plane_normal(plane, H ? &Hv : nullptr);
    if (H) {
      gtsam::Matrix23 P;
      P.row(0) = e1_.transpose();
      P.row(1) = e2_.transpose();
      *H = P * Hv;
    }
    return gtsam::Vector2(v.dot(e1_), v.dot(e2_));
  }
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return gtsam::NonlinearFactor::shared_ptr(new ManhattanLevelFactor(*this));
  }
};

/// Wrap an angle (rad) to [-pi/4, pi/4): the residual of an azimuth to the
/// nearest of four axes 90 deg apart.
double wrap_quarter(double a) {
  const double q = M_PI / 2.0;
  return a - q * std::floor((a + q / 2.0) / q);
}

/// The Manhattan frame: up and a right-handed horizontal basis (e1, e2).
struct ManhattanFrame {
  int up_axis = 2;
  Eigen::Vector3d up = Eigen::Vector3d::UnitZ();
  Eigen::Vector3d e1 = Eigen::Vector3d::UnitX();
  Eigen::Vector3d e2 = Eigen::Vector3d::UnitY();
};

/// Up axis for the Manhattan prior. `automatic` takes the world axis along
/// which the seed camera positions spread least (see ManhattanUp).
ManhattanFrame manhattan_frame(ManhattanUp choice,
                               const std::vector<Eigen::Matrix4d> &poses) {
  ManhattanFrame f;
  int axis = 2;
  switch (choice) {
  case ManhattanUp::x:
    axis = 0;
    break;
  case ManhattanUp::y:
    axis = 1;
    break;
  case ManhattanUp::z:
    axis = 2;
    break;
  case ManhattanUp::automatic: {
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    for (const auto &T : poses)
      mean += T.block<3, 1>(0, 3);
    mean /= static_cast<double>(std::max<size_t>(1, poses.size()));
    Eigen::Vector3d var = Eigen::Vector3d::Zero();
    for (const auto &T : poses)
      var += (T.block<3, 1>(0, 3) - mean).cwiseAbs2();
    var.minCoeff(&axis);
    Eigen::Vector3d sorted = var;
    std::sort(sorted.data(), sorted.data() + 3);
    core::info("PlaneGraph: Manhattan up axis {} (camera position spread "
               "{:.2f} m along it vs {:.2f} m along the next axis)",
               "xyz"[axis],
               std::sqrt(sorted(0) / std::max<size_t>(1, poses.size())),
               std::sqrt(sorted(1) / std::max<size_t>(1, poses.size())));
    if (sorted(1) < 4.0 * sorted(0))
      core::warn("PlaneGraph: the Manhattan up axis is ambiguous (camera "
                 "positions spread similarly along two axes); pass "
                 "--manhattan-up to choose it");
    break;
  }
  }
  f.up_axis = axis;
  f.up = Eigen::Vector3d::Unit(axis);
  f.e1 = Eigen::Vector3d::Unit((axis + 1) % 3);
  f.e2 = f.up.cross(f.e1);
  return f;
}

/// A plane detected in one frame, expressed in that frame's OPTICAL frame as a
/// unit normal + signed offset d so that n.dot(p) + d = 0 for points on it.
struct FramePlane {
  int frame = -1; ///< index into the frames vector
  /// Eigen's default constructor leaves storage indeterminate, so every vector
  /// member is zero-initialized explicitly: a FramePlane is copied into the
  /// plane vector before `refresh_world` fills the world-frame fields, and
  /// copying indeterminate doubles is UB even when the values are overwritten
  /// before anyone reads them.
  Eigen::Vector3d normal = Eigen::Vector3d::Zero(); ///< unit normal (optical)
  double d = 0.0;  ///< plane offset (optical frame)
  int inliers = 0; ///< supporting surfel count
  /// inlier centroid (optical frame)
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  double radius = 0.0;       ///< in-plane RMS extent of inliers (m)
  double residual_rms = 0.0; ///< RMS point-to-plane distance of inliers (m)
  double extent_minor = 0.0; ///< in-plane RMS extent, weaker axis (m)
  /// normal in world (current pose)
  Eigen::Vector3d world_normal = Eigen::Vector3d::Zero();
  double world_d = 0.0; ///< offset in world (current pose)
  /// inlier centroid in world (current pose)
  Eigen::Vector3d world_centroid = Eigen::Vector3d::Zero();
};

/// Transform an optical-frame plane (n, d) by the affine T (optical->world).
/// For a rigid transform p_w = R p + t, a plane n.p_o + d = 0 becomes
/// n_w = R n, d_w = d - n_w.dot(t).
void plane_to_world(const Eigen::Matrix3d &R, const Eigen::Vector3d &t,
                    const Eigen::Vector3d &n, double d, Eigen::Vector3d &n_w,
                    double &d_w) {
  n_w = (R * n).normalized();
  d_w = d - n_w.dot(t);
}

/// Refresh the world-frame fields of a FramePlane from the frame's current
/// pose. Called once per round so association always operates on the poses the
/// previous round produced (EM-style alternation).
void refresh_world(FramePlane &p, const Eigen::Matrix4d &pose) {
  const Eigen::Matrix3d R = pose.block<3, 3>(0, 0);
  const Eigen::Vector3d t = pose.block<3, 1>(0, 3);
  plane_to_world(R, t, p.normal, p.d, p.world_normal, p.world_d);
  p.world_centroid = R * p.centroid + t;
}

/// A cross-frame plane landmark: running world plane (normal + offset) plus the
/// pooled inlier geometry the hygiene checks need (overlap gate, degeneracy
/// test).
struct Landmark {
  Eigen::Vector3d normal;
  double d = 0.0;
  Eigen::Vector3d centroid_sum = Eigen::Vector3d::Zero(); ///< sum(w*centroid_w)
  double weight = 0.0;  ///< total inlier weight
  std::vector<int> obs; ///< indices into all_planes
};

/// Inlier-weighted footprint radius of a landmark: the mean in-plane RMS extent
/// of its member detections. Used by the overlap gate to size the merge window.
double landmark_radius(const Landmark &lm,
                       const std::vector<FramePlane> &planes) {
  double num = 0.0, den = 0.0;
  for (int idx : lm.obs) {
    const double w = static_cast<double>(std::max(1, planes[idx].inliers));
    num += w * planes[idx].radius;
    den += w;
  }
  return den > 0.0 ? num / den : 0.0;
}

/// Rejects a landmark ONLY when its member observation centroids form a
/// genuine, extended near-collinear LINE — the one geometry that leaves a
/// rotation about that line unobservable and lets the landmark bend the
/// trajectory. Two cases are explicitly kept:
///   - fewer than 3 observations (nothing to judge), and
///   - centroids that are effectively coincident (primary spread below an
///     absolute floor): identical viewpoints agree strongly, they do not form
///     a harmful line and their normal+offset still constrain 3 DoF.
/// Only when the primary spread is real (above the floor) AND the secondary
/// spread is a small fraction of it (SV ratio < min_landmark_spread_ratio) is
/// the landmark treated as a degenerate line and rejected.
bool landmark_well_conditioned(const Landmark &lm,
                               const std::vector<FramePlane> &planes,
                               const PlaneGraphOptions &opt) {
  if (opt.min_landmark_spread_ratio <= 0.0f)
    return true;
  const size_t m = lm.obs.size();
  if (m < 3)
    return true; // too few points to judge collinearity; keep it
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  for (int idx : lm.obs)
    mean += planes[idx].world_centroid;
  mean /= static_cast<double>(m);
  Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
  for (int idx : lm.obs) {
    const Eigen::Vector3d r = planes[idx].world_centroid - mean;
    cov += r * r.transpose();
  }
  cov /= static_cast<double>(m); // per-observation variance -> length^2 units
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);
  // Eigenvalues ascending; sqrt gives an RMS length along each axis.
  const double s1 = std::sqrt(std::max(0.0, es.eigenvalues()(2))); // primary
  const double s2 = std::sqrt(std::max(0.0, es.eigenvalues()(1))); // secondary
  // Absolute floor (m): below this the centroids are effectively coincident
  // (identical viewpoints), which is agreement, not a degenerate line.
  constexpr double kCoincidentFloor = 0.05; // 5 cm RMS
  if (s1 < kCoincidentFloor)
    return true;
  return (s2 / s1) >= static_cast<double>(opt.min_landmark_spread_ratio);
}

/// Number of independent normal directions (1, 2 or 3) among a frame's landmark
/// observations, via the rank of their outer-product scatter under a fixed
/// relative tolerance. Used by the observability guard: a frame spanning < 2
/// directions is free along the missing ones on plane factors alone.
int normal_span(const std::vector<Eigen::Vector3d> &normals) {
  if (normals.empty())
    return 0;
  Eigen::Matrix3d m = Eigen::Matrix3d::Zero();
  for (const auto &n : normals)
    m += n * n.transpose();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(m);
  const double lmax = es.eigenvalues()(2);
  if (lmax <= 1e-12)
    return 0;
  const double tol = 0.02 * lmax; // 2% of the dominant direction's energy
  int rank = 0;
  for (int k = 0; k < 3; ++k)
    if (es.eigenvalues()(k) > tol)
      ++rank;
  return rank;
}

/// Sequential RANSAC plane extraction on one frame's surfels. Points and
/// normals are in the optical frame. Detected planes are oriented so their
/// normal agrees with the (camera-facing) surfel normals.
std::vector<FramePlane> detect_frame_planes(const FrameSurfels &f, int frame,
                                            const PlaneGraphOptions &opt,
                                            std::mt19937 &rng) {
  std::vector<FramePlane> planes;
  const auto &pts = *f.points;
  const auto &nrm = *f.normals;
  const size_t n = pts.size();
  if (n < static_cast<size_t>(opt.min_plane_inliers))
    return planes;

  std::vector<char> used(n, 0);
  const double cos_thresh = std::cos(opt.ransac_normal_angle * M_PI / 180.0);
  const double dist_thresh = opt.ransac_distance;

  std::vector<size_t> pool;
  pool.reserve(n);

  for (int p = 0; p < opt.max_planes_per_frame; ++p) {
    pool.clear();
    for (size_t i = 0; i < n; ++i)
      if (!used[i])
        pool.push_back(i);
    if (pool.size() < static_cast<size_t>(opt.min_plane_inliers))
      break;

    std::uniform_int_distribution<size_t> pick(0, pool.size() - 1);
    Eigen::Vector3d best_normal = Eigen::Vector3d::UnitZ();
    double best_d = 0.0;
    int best_inliers = 0;

    for (int it = 0; it < opt.ransac_iterations; ++it) {
      const size_t seed_i = pool[pick(rng)];
      const auto &sp = pts[seed_i];
      const auto &sn = nrm[seed_i];
      Eigen::Vector3d nrm_h(sn.normal_x, sn.normal_y, sn.normal_z);
      if (!nrm_h.allFinite() || nrm_h.squaredNorm() < 1e-9)
        continue;
      nrm_h.normalize();
      const Eigen::Vector3d ph(sp.x, sp.y, sp.z);
      const double dh = -nrm_h.dot(ph); // plane through the seed point

      int cnt = 0;
      for (size_t k : pool) {
        const auto &pk = pts[k];
        const Eigen::Vector3d pp(pk.x, pk.y, pk.z);
        if (std::abs(nrm_h.dot(pp) + dh) > dist_thresh)
          continue;
        const auto &nk = nrm[k];
        Eigen::Vector3d nn(nk.normal_x, nk.normal_y, nk.normal_z);
        if (!nn.allFinite() || nn.squaredNorm() < 1e-9)
          continue;
        if (std::abs(nrm_h.dot(nn.normalized())) < cos_thresh)
          continue;
        ++cnt;
      }
      if (cnt > best_inliers) {
        best_inliers = cnt;
        best_normal = nrm_h;
        best_d = dh;
      }
    }

    if (best_inliers < opt.min_plane_inliers)
      break;

    // Refit the plane to its inliers (least squares centroid + mean normal),
    // and mark them consumed.
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    Eigen::Vector3d mean_normal = Eigen::Vector3d::Zero();
    std::vector<size_t> inlier_idx;
    inlier_idx.reserve(best_inliers);
    for (size_t k : pool) {
      const auto &pk = pts[k];
      const Eigen::Vector3d pp(pk.x, pk.y, pk.z);
      if (std::abs(best_normal.dot(pp) + best_d) > dist_thresh)
        continue;
      const auto &nk = nrm[k];
      Eigen::Vector3d nn(nk.normal_x, nk.normal_y, nk.normal_z);
      if (!nn.allFinite() || nn.squaredNorm() < 1e-9)
        continue;
      if (std::abs(best_normal.dot(nn.normalized())) < cos_thresh)
        continue;
      centroid += pp;
      mean_normal += nn.normalized();
      inlier_idx.push_back(k);
    }
    if (inlier_idx.size() < static_cast<size_t>(opt.min_plane_inliers))
      break;
    centroid /= static_cast<double>(inlier_idx.size());
    if (mean_normal.squaredNorm() < 1e-9)
      break;
    mean_normal.normalize();
    // Orient the plane normal to agree with the surfel normals (camera-facing).
    if (mean_normal.dot(best_normal) < 0.0)
      mean_normal = -mean_normal;
    const double d = -mean_normal.dot(centroid);

    // One pass over the inliers yields all three fit statistics:
    //   * `radius`       — total in-plane RMS extent about the centroid, the
    //                      footprint used by the association overlap gate;
    //   * `scatter`      — the in-plane second-moment matrix, whose principal
    //                      axes give the MINOR extent (the direction along
    //                      which the fit constrains the normal least);
    //   * `residual_rms` — RMS out-of-plane distance, i.e. how planar the
    //                      supporting surface actually is.
    // All are computed from (p - centroid) split into its components parallel
    // and orthogonal to the plane normal, so in-plane spread never inflates the
    // residual and thickness never inflates the extent.
    double sum_sq = 0.0;
    double off_sq = 0.0;
    Eigen::Matrix3d scatter = Eigen::Matrix3d::Zero();
    for (size_t k : inlier_idx) {
      const auto &pk = pts[k];
      const Eigen::Vector3d rp = Eigen::Vector3d(pk.x, pk.y, pk.z) - centroid;
      // Signed point-to-plane distance: d = -n.centroid, so n.p + d = n.rp.
      const double e = mean_normal.dot(rp);
      const Eigen::Vector3d in_plane = rp - e * mean_normal;
      sum_sq += in_plane.squaredNorm();
      off_sq += e * e;
      scatter += in_plane * in_plane.transpose();
    }
    const double m_inl = static_cast<double>(inlier_idx.size());
    const double radius = std::sqrt(sum_sq / m_inl);
    const double residual_rms = std::sqrt(off_sq / m_inl);
    // `in_plane` vectors are all orthogonal to mean_normal, so the scatter has
    // one structurally-zero eigenvalue along the normal. Eigenvalues come out
    // ascending: index 0 is that null direction, 1 and 2 are the two in-plane
    // principal variances. The minor extent is therefore sqrt of index 1.
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> in_plane_es(scatter / m_inl);
    const double extent_minor =
        std::sqrt(std::max(0.0, in_plane_es.eigenvalues()(1)));

    for (size_t k : inlier_idx)
      used[k] = 1;

    FramePlane fp;
    fp.frame = frame;
    fp.normal = mean_normal;
    fp.d = d;
    fp.inliers = static_cast<int>(inlier_idx.size());
    fp.centroid = centroid;
    fp.radius = radius;
    fp.residual_rms = residual_rms;
    fp.extent_minor = extent_minor;
    planes.push_back(fp);
  }

  return planes;
}

/// Median of a vector, by value (the copy is the point: nth_element reorders).
/// Returns 0 for an empty input.
double median_of(std::vector<double> v) {
  if (v.empty())
    return 0.0;
  const size_t mid = v.size() / 2;
  std::nth_element(v.begin(), v.begin() + mid, v.end());
  return v[mid];
}

} // namespace

std::vector<OdometrySigmaScale>
odometry_motion_scales(const std::vector<OdometryEdgeMotion> &motion,
                       double min_scale, double max_scale) {
  std::vector<OdometrySigmaScale> scales(motion.size());
  if (motion.empty())
    return scales;
  if (!(min_scale > 0.0) || !(max_scale >= min_scale))
    return scales; // caller gave a degenerate clamp: stay at the fixed model

  std::vector<double> trans, rot;
  trans.reserve(motion.size());
  rot.reserve(motion.size());
  for (const auto &m : motion) {
    trans.push_back(std::abs(m.translation));
    rot.push_back(std::abs(m.rotation));
  }
  const double med_t = median_of(trans);
  const double med_r = median_of(rot);

  // A zero (or non-finite) median means the run has no motion to normalise
  // against — a stationary capture, or a channel that never moved. Leaving the
  // scale at 1.0 degenerates to OdometryNoiseModel::fixed, which is the honest
  // answer; dividing would be a divide-by-zero dressed up as a noise model.
  const bool use_t = std::isfinite(med_t) && med_t > 0.0;
  const bool use_r = std::isfinite(med_r) && med_r > 0.0;

  for (size_t i = 0; i < motion.size(); ++i) {
    if (use_t && std::isfinite(trans[i]))
      scales[i].translation =
          std::clamp(trans[i] / med_t, min_scale, max_scale);
    if (use_r && std::isfinite(rot[i]))
      scales[i].rotation = std::clamp(rot[i] / med_r, min_scale, max_scale);
  }
  return scales;
}

PlaneFitSigmas plane_fit_sigmas(const PlaneFitQuality &q) {
  // Guards, not corrections: these clamps only bound the degenerate corners so
  // the graph never sees an infinite or zero sigma. Real detections are far
  // above both floors.
  constexpr double kMinResidual = 1e-4; ///< 0.1 mm — under any real depth noise
  constexpr double kMinExtent = 1e-3;   ///< 1 mm — a patch this small pins no
                                        ///< normal at all, so cap the lever arm
  const double n = static_cast<double>(std::max(1, q.inliers));
  const double sigma = std::max(kMinResidual, q.residual_rms);
  const double extent = std::max(kMinExtent, q.extent_minor);
  const double sqrt_n = std::sqrt(n);

  PlaneFitSigmas s;
  s.distance = sigma / sqrt_n;
  s.normal = sigma / (extent * sqrt_n);
  return s;
}

Eigen::Matrix3d prior_levelling(const Eigen::Matrix3d &R_target,
                                const Eigen::Matrix3d &axes) {
  const Eigen::Matrix3d L = axes * R_target;
  int k = 0;
  for (int c = 1; c < 3; ++c)
    if (std::abs(L(2, c)) > std::abs(L(2, k)))
      k = c;
  const double s = L(2, k) < 0.0 ? -1.0 : 1.0;
  const Eigen::Vector3d z = s * Eigen::Vector3d::Unit(k);
  const Eigen::Vector3d x = Eigen::Vector3d::Unit((k + 1) % 3);
  Eigen::Matrix3d C;
  C.col(0) = x;
  C.col(1) = z.cross(x);
  C.col(2) = z;
  return C;
}

Eigen::Vector3d prior_rpy(const Eigen::Matrix3d &R, const Eigen::Matrix3d &C,
                          const Eigen::Matrix3d &axes) {
  const Eigen::Matrix3d L = axes * R * C;
  return {std::atan2(L(2, 1), L(2, 2)),
          std::asin(std::clamp(-L(2, 0), -1.0, 1.0)),
          std::atan2(L(1, 0), L(0, 0))};
}

Eigen::Matrix3d prior_rotation(const Eigen::Vector3d &rpy,
                               const Eigen::Matrix3d &C,
                               const Eigen::Matrix3d &axes) {
  const Eigen::Matrix3d L =
      (Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
       Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX()))
          .toRotationMatrix();
  return axes.transpose() * L * C.transpose();
}

PlaneGraphOptimizer::PlaneGraphOptimizer(PlaneGraphOptions options)
    : options_(std::move(options)) {}

PlaneGraphResult PlaneGraphOptimizer::optimize_graph(
    std::vector<FrameSurfels> &frames, const std::vector<GraphEdge> &edges,
    std::span<const PosePrior> priors,
    std::span<const SurfaceConstraint> surfaces) const {
  using gtsam::symbol_shorthand::P; // plane landmarks
  using gtsam::symbol_shorthand::X; // poses

  PlaneGraphResult result;
  const int N = static_cast<int>(frames.size());
  result.frames = N;
  if (N < 2) {
    core::warn("PlaneGraph: need at least 2 frames, got {}", N);
    return result;
  }
  auto in_range = [N](const GraphEdge &e) {
    return e.i >= 0 && e.j >= 0 && e.i < N && e.j < N && e.i != e.j;
  };
  const bool has_loop_edges =
      std::any_of(edges.begin(), edges.end(), [](const GraphEdge &e) {
        return e.kind == GraphEdge::Kind::loop;
      });
  // Pose edits (reusex-rcv.1.3). Out-of-range ones are skipped, with a warn.
  std::vector<const PosePrior *> edit_priors;
  std::vector<const SurfaceConstraint *> edit_surfaces;
  {
    int bad = 0;
    for (const auto &p : priors)
      if (p.i >= 0 && p.i < N)
        edit_priors.push_back(&p);
      else
        ++bad;
    for (const auto &c : surfaces)
      if (c.i >= 0 && c.j >= 0 && c.i < N && c.j < N && c.i != c.j)
        edit_surfaces.push_back(&c);
      else
        ++bad;
    if (bad > 0)
      core::warn("PlaneGraph: skipped {} pose-edit factors with an "
                 "out-of-range frame index or a self-loop ({} frames)",
                 bad, N);
  }
  const bool has_edits = !edit_priors.empty() || !edit_surfaces.empty();
  // Edges that cannot enter the graph (an endpoint outside frames[] or a
  // self-loop) are skipped. Odometry now comes from the edge list, so a
  // skipped odometry edge can disconnect the trajectory: say so (STANDARDS §5).
  {
    int bad_odom = 0, bad_loop = 0;
    for (const auto &e : edges)
      if (!in_range(e))
        ++(e.kind == GraphEdge::Kind::loop ? bad_loop : bad_odom);
    if (bad_odom + bad_loop > 0)
      core::warn("PlaneGraph: skipped {} odometry and {} loop edges with an "
                 "out-of-range index or a self-loop ({} frames). Edge indices "
                 "must be positions in frames[]; check the node_id -> index "
                 "mapping",
                 bad_odom, bad_loop, N);
  }

  // --- Seed poses ----------------------------------------------------------
  // seed[] is the ORIGINAL input pose of each frame, held fixed for the whole
  // run: the gauge prior is derived from it (odometry measurements come from
  // the edge list; legacy_graph_edges derives them from the same seed), and
  // max_pose_shift is measured against it. cur[] is
  // the working pose, updated after each round and used to re-associate.
  std::vector<Eigen::Matrix4d> seed(N);
  for (int i = 0; i < N; ++i)
    seed[i] = frames[i].world_pose.matrix();
  std::vector<Eigen::Matrix4d> cur = seed;

  // --- 1. Per-frame plane detection (once; geometry is pose-independent) ----
  std::mt19937 rng(options_.seed);
  std::vector<FramePlane> all_planes;
  for (int i = 0; i < N; ++i) {
    auto fp = detect_frame_planes(frames[i], i, options_, rng);
    for (auto &p : fp)
      all_planes.push_back(p);
  }
  result.planes_detected = static_cast<int>(all_planes.size());
  core::info("PlaneGraph: {} frames, {} plane detections", N,
             all_planes.size());
  if (all_planes.empty() && !has_edits) {
    core::warn("PlaneGraph: no planes detected; poses unchanged");
    return result;
  }
  // STANDARDS §5: a configuration that makes the stage a near-no-op must say
  // so with its numbers, not silently write the seed poses back.
  if (!options_.use_plane_factors)
    core::warn("PlaneGraph: --no-plane-factors is set; {} detections will be "
               "associated and reported but NO plane factor enters the graph "
               "(odometry + gauge prior only), so the solve cannot move the "
               "seed trajectory",
               all_planes.size());
  else if (options_.plane_sigma_scale != 1.0f)
    core::info("PlaneGraph: plane sigmas scaled by {:.4g} (plane-term weight "
               "x{:.4g} relative to the shipped calibration)",
               options_.plane_sigma_scale,
               1.0 / (static_cast<double>(options_.plane_sigma_scale) *
                      options_.plane_sigma_scale));

  const double assoc_cos = std::cos(options_.assoc_normal_angle * M_PI / 180.0);
  using GncParamsLM = gtsam::GncParams<gtsam::LevenbergMarquardtParams>;

  // --- Per-observation plane-factor noise scales ---------------------------
  // Computed ONCE, outside the round loop: they depend only on each detection's
  // own fit statistics, which are pose-independent (the RANSAC runs in the
  // optical frame), so re-association cannot change them.
  //
  // Both non-uniform models normalise by the MEDIAN over this run. That is what
  // makes the models interchangeable without retuning: plane_sigma_normal /
  // plane_sigma_distance keep their meaning as the sigmas of a median-quality
  // observation, the plane term's aggregate authority against odometry is
  // unchanged, and only the DISTRIBUTION of authority across observations
  // differs. Normalising by the median (rather than a constant) additionally
  // keeps the weighting invariant to --sampling-factor, which scales every
  // inlier count at once.
  const size_t n_obs = all_planes.size();
  std::vector<double> scale_normal(n_obs, 1.0);
  std::vector<double> scale_distance(n_obs, 1.0);
  {
    const double lo = static_cast<double>(options_.plane_weight_min);
    const double hi = static_cast<double>(options_.plane_weight_max);
    auto apply = [&](size_t i, double raw_n, double raw_d) {
      const double cn = std::clamp(raw_n, lo, hi);
      const double cd = std::clamp(raw_d, lo, hi);
      if (cn != raw_n || cd != raw_d) {
        if (cn > raw_n || cd > raw_d)
          ++result
                .plane_noise_clamped_low; // wanted MORE authority than allowed
        else
          ++result.plane_noise_clamped_high; // wanted LESS authority
      }
      scale_normal[i] = cn;
      scale_distance[i] = cd;
    };

    switch (options_.plane_noise_model) {
    case PlaneNoiseModel::uniform:
      break; // scales stay 1.0

    case PlaneNoiseModel::inlier_count: {
      std::vector<double> counts(n_obs);
      for (size_t i = 0; i < n_obs; ++i)
        counts[i] = static_cast<double>(std::max(1, all_planes[i].inliers));
      const double ref = std::max(1.0, median_of(counts));
      for (size_t i = 0; i < n_obs; ++i) {
        const double s = std::sqrt(ref / counts[i]);
        apply(i, s, s); // one scalar on BOTH channels (the legacy behaviour)
      }
      core::info("PlaneGraph: inlier-count plane noise, median support {:.0f} "
                 "surfels over {} detections",
                 ref, n_obs);
      break;
    }

    case PlaneNoiseModel::fit_geometry: {
      std::vector<double> sig_n(n_obs), sig_d(n_obs);
      for (size_t i = 0; i < n_obs; ++i) {
        const auto s =
            plane_fit_sigmas({all_planes[i].inliers, all_planes[i].residual_rms,
                              all_planes[i].extent_minor});
        sig_n[i] = s.normal;
        sig_d[i] = s.distance;
      }
      // Guard a pathological all-zero median so the division below stays
      // finite; plane_fit_sigmas already returns strictly positive values, so
      // this only fires on an empty detection set (handled earlier) or a
      // compiler bug.
      const double ref_n = std::max(1e-12, median_of(sig_n));
      const double ref_d = std::max(1e-12, median_of(sig_d));
      result.median_fit_sigma_normal = ref_n;
      result.median_fit_sigma_distance = ref_d;
      for (size_t i = 0; i < n_obs; ++i)
        apply(i, sig_n[i] / ref_n, sig_d[i] / ref_d);
      core::info(
          "PlaneGraph: fit-geometry plane noise over {} detections — median "
          "fit "
          "sigma {:.5f} rad / {:.5f} m; scales clamped to [{:.2f}, {:.2f}] for "
          "{} strong and {} weak detections",
          n_obs, ref_n, ref_d, lo, hi, result.plane_noise_clamped_low,
          result.plane_noise_clamped_high);
      break;
    }
    }

    // STANDARDS §5: say so, with numbers, when the clamp rather than the fit
    // statistics is deciding the weights — that means the range is too narrow
    // and the model has been silently reduced back to a near-uniform one.
    const int clamped =
        result.plane_noise_clamped_low + result.plane_noise_clamped_high;
    if (n_obs > 0 &&
        static_cast<double>(clamped) > 0.5 * static_cast<double>(n_obs))
      core::warn(
          "PlaneGraph: {}/{} plane observations hit the [{:.2f}, {:.2f}] "
          "sigma-scale clamp, so the clamp — not the fit quality — is "
          "setting most weights. Widen --plane-weight-min/max to let the "
          "noise model act.",
          clamped, n_obs, lo, hi);
  }

  // --- Manhattan prior set-up (reusex-rqn.1) -------------------------------
  // The prior lives on the plane LANDMARK variables, so it needs them.
  const bool manhattan = options_.manhattan && options_.use_plane_factors;
  if (options_.manhattan && !options_.use_plane_factors)
    core::warn("PlaneGraph: --manhattan is ignored under --no-plane-factors: "
               "the prior constrains plane landmarks and there are none");
  ManhattanFrame mframe;
  if (manhattan) {
    mframe = manhattan_frame(options_.manhattan_up, seed);
    result.manhattan_up_axis = mframe.up_axis;
  }
  const double man_gate = options_.manhattan_max_angle_deg * M_PI / 180.0;
  const double man_level_gate =
      options_.manhattan_level_max_angle_deg * M_PI / 180.0;
  const double man_sigma = options_.manhattan_sigma_deg * M_PI / 180.0;
  // Azimuth of the grid, carried from round to round (nullopt: estimate it).
  std::optional<double> man_theta;
  const gtsam::Key man_key = gtsam::Symbol('m', 0);

  double round_max_shift = 0.0;

  // --- Alternating rounds: associate at cur poses -> optimize -> refit ------
  for (int round = 0; round < std::max(1, options_.assoc_rounds); ++round) {
    result.rounds = round + 1;

    // Refresh every detection's world quantities from the current poses.
    for (auto &p : all_planes)
      refresh_world(p, cur[p.frame]);

    // -- Overlap-aware greedy association ----------------------------------
    // Detections are processed largest-first so dominant planes seed clusters.
    // A detection joins the best-scoring landmark that agrees in normal AND
    // offset AND whose in-plane footprint OVERLAPS it (centroid gap projected
    // into the shared plane < sum of radii + margin). The overlap gate is what
    // keeps two offset/disjoint parallel walls from aliasing into one landmark.
    std::vector<int> order(all_planes.size());
    for (size_t i = 0; i < order.size(); ++i)
      order[i] = static_cast<int>(i);
    std::sort(order.begin(), order.end(), [&](int a, int b) {
      return all_planes[a].inliers > all_planes[b].inliers;
    });

    std::vector<Landmark> landmarks;
    int rejected_overlap = 0;
    for (int idx : order) {
      const auto &pl = all_planes[idx];
      const double w = static_cast<double>(std::max(1, pl.inliers));
      int best = -1;
      double best_score = -1.0;
      for (size_t l = 0; l < landmarks.size(); ++l) {
        const double c = pl.world_normal.dot(landmarks[l].normal);
        if (c < assoc_cos)
          continue;
        if (std::abs(pl.world_d - landmarks[l].d) > options_.assoc_distance)
          continue;
        if (options_.assoc_overlap_margin > 0.0f) {
          // In-plane gap between this detection's centroid and the landmark
          // centroid (component orthogonal to the landmark normal). If it
          // exceeds the two footprint radii plus the margin, the surfaces do
          // not overlap and must NOT merge even though normal+offset agree.
          const Eigen::Vector3d lm_centroid =
              landmarks[l].centroid_sum / landmarks[l].weight;
          const Eigen::Vector3d gap = pl.world_centroid - lm_centroid;
          const Eigen::Vector3d in_plane =
              gap - landmarks[l].normal.dot(gap) * landmarks[l].normal;
          const double allowed = pl.radius +
                                 landmark_radius(landmarks[l], all_planes) +
                                 options_.assoc_overlap_margin;
          if (in_plane.norm() > allowed) {
            ++rejected_overlap;
            continue;
          }
        }
        if (c > best_score) {
          best_score = c;
          best = static_cast<int>(l);
        }
      }
      if (best < 0) {
        Landmark lm;
        lm.normal = pl.world_normal;
        lm.d = pl.world_d;
        lm.centroid_sum = pl.world_centroid * w;
        lm.weight = w;
        lm.obs.push_back(idx);
        landmarks.push_back(std::move(lm));
      } else {
        auto &lm = landmarks[best];
        const double m = static_cast<double>(lm.obs.size());
        lm.normal = (lm.normal * m + pl.world_normal).normalized();
        lm.d = (lm.d * m + pl.world_d) / (m + 1.0);
        lm.centroid_sum += pl.world_centroid * w;
        lm.weight += w;
        lm.obs.push_back(idx);
      }
    }
    result.landmarks_rejected_overlap = rejected_overlap;

    // -- Landmark hygiene: distinct-frame count + degeneracy rejection ------
    // Keep only landmarks seen by enough distinct frames whose inlier centroids
    // are NOT near-collinear. Collinear centroids leave a rotation about that
    // line unconstrained; such a landmark can bend the trajectory freely and
    // must be dropped.
    std::vector<int> landmark_key(landmarks.size(), -1);
    int kept = 0;
    int rejected_degenerate = 0;
    for (size_t l = 0; l < landmarks.size(); ++l) {
      std::vector<int> distinct;
      for (int idx : landmarks[l].obs)
        distinct.push_back(all_planes[idx].frame);
      std::sort(distinct.begin(), distinct.end());
      distinct.erase(std::unique(distinct.begin(), distinct.end()),
                     distinct.end());
      if (static_cast<int>(distinct.size()) <
          options_.min_landmark_observations)
        continue;
      if (!landmark_well_conditioned(landmarks[l], all_planes, options_)) {
        ++rejected_degenerate;
        continue;
      }
      landmark_key[l] = kept++;
    }
    result.landmarks_rejected_degenerate = rejected_degenerate;
    result.landmarks = kept;
    core::info("PlaneGraph round {}: {} raw landmarks, {} kept (>= {} obs, "
               "non-degenerate), {} overlap-rejected, {} degenerate-rejected",
               round + 1, landmarks.size(), kept,
               options_.min_landmark_observations, rejected_overlap,
               rejected_degenerate);

    // No plane landmarks: only bail out if there are ALSO no loop edges. When
    // loop edges are present the graph (gauge + odometry + loop factors) is
    // still well-posed and worth solving, so fall through to build it.
    if (kept == 0 && !has_loop_edges && !has_edits) {
      if (round == 0) {
        core::warn("PlaneGraph: no landmark passed the hygiene checks; poses "
                   "unchanged. Lower --min-observations or relax association / "
                   "overlap thresholds.");
        return result;
      }
      core::warn(
          "PlaneGraph: round {} produced no valid landmarks; keeping the "
          "previous round's poses",
          round + 1);
      break;
    }

    // -- Observability guard: which frames under-constrain the trajectory ---
    // For each frame, collect the normals of the landmark observations it
    // contributes. A frame spanning < 2 independent normal directions is free
    // to slide/rotate along the missing directions on plane factors alone; we
    // tighten its odometry so the trusted trajectory holds it there.
    std::vector<std::vector<Eigen::Vector3d>> frame_normals(N);
    for (size_t l = 0; l < landmarks.size(); ++l) {
      if (landmark_key[l] < 0)
        continue;
      for (int idx : landmarks[l].obs)
        frame_normals[all_planes[idx].frame].push_back(landmarks[l].normal);
    }
    std::vector<bool> underconstrained(N, false);
    int nuc = 0;
    for (int i = 0; i < N; ++i) {
      if (normal_span(frame_normals[i]) < 2) {
        underconstrained[i] = true;
        ++nuc;
      }
    }
    result.underconstrained_frames = nuc;

    // -- Build the factor graph --------------------------------------------
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial;
    GncParamsLM::IndexVector trusted_factors;
    // Graph positions of the loop-edge factors that get the relaxed GNC-TLS
    // inlier threshold (only populated when loop_edges_trusted is set).
    std::vector<size_t> loop_factors;
    // Graph positions of the odometry factors, populated only when
    // options_.odometry_robust leaves them OUT of the known-inlier set so GNC
    // can down-weight them; they then need their own 6-DoF TLS threshold.
    std::vector<size_t> odom_factors;
    // Graph positions of the Manhattan wall / level factors (each with the
    // TLS threshold of its own gate).
    std::vector<size_t> manhattan_factors;
    std::vector<size_t> manhattan_level_factors;

    // Edge records for pose-graph persistence: populated alongside graph
    // construction, residuals filled post-solve.  Cleared and rebuilt each
    // round so result.edges always reflects the FINAL round.
    struct EdgeRec {
      int i, j;
      std::string type;
      size_t factor_idx;
      double weight; // 1 / sigma_trans² at build time
      int tag;
    };
    std::vector<EdgeRec> edge_recs;

    for (int i = 0; i < N; ++i) {
      const gtsam::Rot3 R(cur[i].block<3, 3>(0, 0));
      const gtsam::Point3 tt(cur[i].block<3, 1>(0, 3));
      initial.insert(X(i), gtsam::Pose3(R, tt));
    }

    // Gauge: soft prior on the first pose (from the fixed seed).
    {
      auto prior_noise = gtsam::noiseModel::Diagonal::Sigmas(
          (gtsam::Vector(6) << options_.prior_sigma_rot,
           options_.prior_sigma_rot, options_.prior_sigma_rot,
           options_.prior_sigma_trans, options_.prior_sigma_trans,
           options_.prior_sigma_trans)
              .finished());
      const gtsam::Rot3 R0(seed[0].block<3, 3>(0, 0));
      const gtsam::Point3 t0(seed[0].block<3, 1>(0, 3));
      graph.addPrior(X(0), gtsam::Pose3(R0, t0), prior_noise);
      trusted_factors.push_back(graph.size() - 1);
    }

    // Odometry factors, one per odometry-kind edge. Edges without an
    // information matrix keep the historical weighting: odometry_sigma_*,
    // tightened on under-constrained frames, optionally motion-scaled. A
    // stored information is the measurement's own noise (only the
    // observability guard applies to it).
    {
      const double base_r = options_.odometry_sigma_rot;
      const double base_t = options_.odometry_sigma_trans;
      const double scale = options_.underconstrained_odom_scale;

      std::vector<int> motion_slot(edges.size(), -1);
      std::vector<OdometryEdgeMotion> motion;
      for (size_t k = 0; k < edges.size(); ++k) {
        const auto &e = edges[k];
        if (e.kind != GraphEdge::Kind::odometry || e.information ||
            !in_range(e))
          continue;
        const gtsam::Pose3 rel = to_pose3(e.T_ij);
        motion_slot[k] = static_cast<int>(motion.size());
        motion.push_back({rel.translation().norm(),
                          gtsam::Rot3::Logmap(rel.rotation()).norm()});
      }
      std::vector<OdometrySigmaScale> motion_scale;
      if (options_.odometry_noise_model == OdometryNoiseModel::motion &&
          !motion.empty()) {
        motion_scale = odometry_motion_scales(
            motion, static_cast<double>(options_.odometry_weight_min),
            static_cast<double>(options_.odometry_weight_max));
        std::vector<double> mt, mr;
        for (const auto &m : motion) {
          mt.push_back(m.translation);
          mr.push_back(m.rotation);
        }
        core::info("PlaneGraph: motion-scaled odometry noise over {} edges "
                   "(median motion {:.4f} m / {:.4f} rad)",
                   motion.size(), median_of(mt), median_of(mr));
      }

      for (size_t k = 0; k < edges.size(); ++k) {
        const auto &e = edges[k];
        if (e.kind != GraphEdge::Kind::odometry || !in_range(e))
          continue;
        const double s =
            (underconstrained[e.i] || underconstrained[e.j]) ? scale : 1.0;
        gtsam::SharedNoiseModel noise;
        double weight = 0.0;
        if (e.information) {
          const Eigen::Matrix<double, 6, 6> I = *e.information / (s * s);
          noise = information_noise(I);
          weight = I.diagonal().tail<3>().mean();
        } else {
          const int slot = motion_slot[k];
          const double mr =
              motion_scale.empty() ? 1.0 : motion_scale[slot].rotation;
          const double mt =
              motion_scale.empty() ? 1.0 : motion_scale[slot].translation;
          const double sr = base_r * s * mr;
          const double st = base_t * s * mt;
          noise = sigmas_noise(sr, st);
          weight = 1.0 / (st * st);
        }
        graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
            X(e.i), X(e.j), to_pose3(e.T_ij), noise);
        // Odometry is a KNOWN INLIER by default: GNC may not demote it. With
        // --odometry-robust it is left out of the trusted set so GNC can
        // (the #225 "grossly-wrong seed edge" probe) — unless the user
        // confirmed the edge.
        if (options_.odometry_robust && !e.known_inlier)
          odom_factors.push_back(graph.size() - 1);
        else
          trusted_factors.push_back(graph.size() - 1);
        edge_recs.push_back(
            {e.i, e.j, "odometry", graph.size() - 1, weight, e.tag});
      }
    }

    // Wide-baseline loop edges (P2). Each is a relative-pose BetweenFactor
    // between two temporally distant frames, derived from feature matching +
    // depth (see LoopClosure). A loop edge that CORRECTS drift has, by
    // construction, a large residual at the (drifted) seed poses — which is
    // exactly what GNC-TLS classifies as an outlier and zeros, discarding the
    // constraint we need.
    //
    // loop_edges_trusted resolves that WITHOUT surrendering the outlier bound,
    // by raising the TLS inlier threshold for these factors only (see
    // barcSq below): a genuine large-drift edge stays an inlier and pulls,
    // while an edge whose residual exceeds even the generous
    // loop_trust_inlier_cost is still truncated to zero weight.
    //
    // What we must NOT do is wrap them in noiseModel::Robust: GncOptimizer's
    // constructor (GTSAM 4.2.1, GncOptimizer.h — `robust ?
    // factor->cloneWithNewNoiseModel(robust->noise()) : factor`)
    // unconditionally strips the robust kernel from every factor in the graph,
    // so a Huber-wrapped known-inlier loop edge would silently become an
    // unbounded Gaussian. Under --no-gnc there is no GNC machinery at all, so
    // there the Huber kernel IS the bound and is applied instead.
    //
    // Endpoints out of range are skipped.
    int loop_edge_count = 0;
    for (const auto &e : edges) {
      if (e.kind != GraphEdge::Kind::loop || !in_range(e))
        continue;
      gtsam::SharedNoiseModel base_noise;
      double weight = 0.0;
      if (e.information) {
        base_noise = information_noise(*e.information);
        weight = e.information->diagonal().tail<3>().mean();
      } else {
        const double sr = options_.loop_default_sigma_rot;
        const double st = options_.loop_default_sigma_trans;
        base_noise = sigmas_noise(sr, st);
        weight = 1.0 / (st * st);
      }
      gtsam::SharedNoiseModel noise = base_noise;
      if (options_.loop_edges_trusted && !options_.use_gnc && !e.known_inlier)
        noise = gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Huber::Create(1.345), base_noise);
      graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
          X(e.i), X(e.j), to_pose3(e.T_ij), noise);
      if (e.known_inlier)
        trusted_factors.push_back(graph.size() - 1);
      else if (options_.loop_edges_trusted)
        loop_factors.push_back(graph.size() - 1);
      edge_recs.push_back(
          {e.i, e.j, "loop_closure", graph.size() - 1, weight, e.tag});
      ++loop_edge_count;
    }
    result.loop_edges = loop_edge_count;

    // Pose-edit factors (reusex-rcv.1.3): absolute priors and rank-deficient
    // relative constraints. Not known inliers: they get the generous but
    // finite edit_inlier_cost GNC threshold, so a contradictory edit is
    // down-weighted and reported rather than obeyed silently.
    struct EditRec {
      size_t factor_idx;
      const std::string *tag;
      Eigen::Matrix<double, 6, 6> information;
    };
    std::vector<EditRec> edit_recs;
    for (const PosePrior *p : edit_priors) {
      graph.emplace_shared<EditPosePriorFactor>(X(p->i), p->pose, p->axes,
                                                edit_noise(p->information));
      edit_recs.push_back({graph.size() - 1, &p->tag, p->information});
    }
    for (const SurfaceConstraint *c : edit_surfaces) {
      graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
          X(c->i), X(c->j), to_pose3(c->T_ij), edit_noise(c->information));
      edit_recs.push_back({graph.size() - 1, &c->tag, c->information});
    }
    result.edit_factors = static_cast<int>(edit_recs.size());

    // Plane landmark variables + one OrientedPlane3Factor per observation.
    // Each factor's noise is (optionally) scaled by the reliability of its
    // supporting fit: sigma *= clamp(sqrt(ref / inliers), min, max). A plane
    // backed by many surfels earns a tighter sigma (pulls harder); a small,
    // weak plane is loosened so it cannot warp the trajectory. This is the fix
    // for the measured "denser extraction hurts" regression — the extra planes
    // are mostly small, and equal weighting let them dominate wrongly.
    // plane_sigma_scale is the one-dimensional authority knob: it multiplies
    // both channels, so the plane term's weight in the objective goes as
    // 1 / scale^2. use_plane_factors == false is its infinite limit, taken
    // exactly (no landmark variables, no plane factors -> odometry + prior).
    const double base_sig_n =
        options_.plane_sigma_normal * options_.plane_sigma_scale;
    const double base_sig_d =
        options_.plane_sigma_distance * options_.plane_sigma_scale;

    int plane_factors = 0;
    for (size_t l = 0; options_.use_plane_factors && l < landmarks.size();
         ++l) {
      const int key = landmark_key[l];
      if (key < 0)
        continue;
      // gtsam::OrientedPlane3 stores (normal, d) in Hessian form n.x + d = 0 —
      // the SAME convention as FramePlane, so d passes through directly (see
      // the "consistent observation has zero residual" regression test).
      const gtsam::OrientedPlane3 lm_plane(gtsam::Unit3(landmarks[l].normal),
                                           landmarks[l].d);
      initial.insert(P(key), lm_plane);

      for (int idx : landmarks[l].obs) {
        const auto &fp = all_planes[idx];
        // Two independent scales: the tilt channels and the offset channel are
        // limited by different things (see plane_fit_sigmas), so a small patch
        // can be trusted on WHERE it is while being distrusted on HOW it is
        // oriented. Under the legacy models both scales are equal.
        const double sig_n = base_sig_n * scale_normal[idx];
        const double sig_d = base_sig_d * scale_distance[idx];
        auto plane_noise = gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(3) << sig_n, sig_n, sig_d).finished());
        const gtsam::Vector4 measured(fp.normal.x(), fp.normal.y(),
                                      fp.normal.z(), fp.d);
        graph.emplace_shared<ClonableOrientedPlane3Factor>(
            measured, plane_noise, X(fp.frame), P(key));
        ++plane_factors;
      }
    }
    result.plane_factors = plane_factors;

    // Manhattan factors on the kept landmarks. Classification uses the
    // landmark normal at the CURRENT poses; the gate is the same angle that
    // later becomes the factors' GNC-TLS threshold.
    bool man_theta_in_graph = false;
    if (manhattan) {
      struct Vert {
        size_t l;
        double phi;
        double w;
      };
      std::vector<Vert> verts;
      std::vector<size_t> levels;
      int skipped = 0;
      const double sin_gate = std::sin(man_gate);
      const double cos_level_gate = std::cos(man_level_gate);
      for (size_t l = 0; l < landmarks.size(); ++l) {
        if (landmark_key[l] < 0)
          continue;
        const Eigen::Vector3d &n = landmarks[l].normal;
        const double c = std::abs(n.dot(mframe.up));
        if (c >= cos_level_gate)
          levels.push_back(l);
        else if (c <= sin_gate)
          verts.push_back({l, std::atan2(n.dot(mframe.e2), n.dot(mframe.e1)),
                           std::max(1.0, landmarks[l].weight)});
        else
          ++skipped; // slanted (a sloped ceiling, a ramp): neither
      }
      // Grid azimuth: start from the weighted circular mean of 4*phi (the
      // four axes fold onto one direction), then re-centre on the walls that
      // fall inside the gate, so off-grid walls do not bias it.
      if (!man_theta && !verts.empty()) {
        double sx = 0.0, sy = 0.0;
        for (const auto &v : verts) {
          sx += v.w * std::cos(4.0 * v.phi);
          sy += v.w * std::sin(4.0 * v.phi);
        }
        double theta = std::atan2(sy, sx) / 4.0;
        for (int it = 0; it < 3; ++it) {
          double num = 0.0, den = 0.0;
          for (const auto &v : verts) {
            const double r = wrap_quarter(v.phi - theta);
            if (std::abs(r) <= man_gate) {
              num += v.w * r;
              den += v.w;
            }
          }
          if (den > 0.0)
            theta += num / den;
        }
        man_theta = theta;
      }
      auto noise = gtsam::noiseModel::Isotropic::Sigma(2, man_sigma);
      gtsam::SharedNoiseModel man_noise = noise;
      if (!options_.use_gnc)
        man_noise = gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Huber::Create(1.345), noise);
      int n_vert = 0, n_level = 0;
      for (const auto &v : verts) {
        const double r = wrap_quarter(v.phi - *man_theta);
        if (std::abs(r) > man_gate) {
          ++skipped;
          continue;
        }
        // Which of the four axes: phi - r = theta + k * 90 deg.
        const double offset =
            std::round((v.phi - r - *man_theta) / (M_PI / 2.0)) * (M_PI / 2.0);
        graph.emplace_shared<ManhattanWallFactor>(P(landmark_key[v.l]), man_key,
                                                  mframe.up, mframe.e1,
                                                  mframe.e2, offset, man_noise);
        manhattan_factors.push_back(graph.size() - 1);
        ++n_vert;
      }
      for (size_t l : levels) {
        graph.emplace_shared<ManhattanLevelFactor>(
            P(landmark_key[l]), mframe.e1, mframe.e2, man_noise);
        manhattan_level_factors.push_back(graph.size() - 1);
        ++n_level;
      }
      if (n_vert > 0) {
        initial.insert(man_key, *man_theta);
        man_theta_in_graph = true;
      }
      result.manhattan_vertical = n_vert;
      result.manhattan_horizontal = n_level;
      result.manhattan_skipped = skipped;
      core::info("PlaneGraph round {}: Manhattan prior on {} wall and {} level "
                 "landmarks ({} outside the {:.1f} / {:.1f} deg gates), grid "
                 "azimuth "
                 "{:.2f} deg, sigma {:.2f} deg",
                 round + 1, n_vert, n_level, skipped,
                 options_.manhattan_max_angle_deg,
                 options_.manhattan_level_max_angle_deg,
                 man_theta ? *man_theta * 180.0 / M_PI : 0.0,
                 options_.manhattan_sigma_deg);
      if (n_vert + n_level == 0)
        core::warn("PlaneGraph: --manhattan found no landmark within {:.1f} "
                   "deg of the grid among {} kept landmarks; the prior has "
                   "no effect",
                   options_.manhattan_max_angle_deg, kept);
    }

    // -- Solve --------------------------------------------------------------
    const double round_initial_error = graph.error(initial);
    if (round == 0)
      result.initial_error = round_initial_error;

    gtsam::LevenbergMarquardtParams lm_params;
    lm_params.setMaxIterations(options_.max_iterations);

    gtsam::Values solution;
    gtsam::Vector gnc_weights; // empty under --no-gnc
    try {
      if (options_.use_gnc) {
        using GncLM = gtsam::GncOptimizer<GncParamsLM>;
        GncParamsLM gnc_params(lm_params);
        gnc_params.setKnownInliers(trusted_factors);
        GncLM optimizer(graph, initial, gnc_params);
        const bool any_manhattan =
            !manhattan_factors.empty() || !manhattan_level_factors.empty();
        if (loop_factors.empty() && odom_factors.empty() && !any_manhattan &&
            edit_recs.empty()) {
          optimizer.setInlierCostThresholds(
              static_cast<double>(options_.gnc_inlier_cost));
        } else if (loop_factors.empty() && !any_manhattan &&
                   edit_recs.empty()) {
          // Robust odometry only: plane factors keep the 3-DoF chi-square
          // threshold, odometry gets the 6-DoF one.
          gtsam::Vector barc = gtsam::Vector::Constant(
              static_cast<Eigen::Index>(graph.size()),
              static_cast<double>(options_.gnc_inlier_cost));
          for (size_t k : odom_factors)
            barc[static_cast<Eigen::Index>(k)] =
                static_cast<double>(options_.odometry_gnc_inlier_cost);
          optimizer.setInlierCostThresholds(barc);
          core::info("PlaneGraph: {} odometry factors are GNC-demotable at a "
                     "TLS threshold of {:.2f} (planes {:.2f})",
                     odom_factors.size(), options_.odometry_gnc_inlier_cost,
                     options_.gnc_inlier_cost);
        } else {
          // Per-factor TLS thresholds: everything keeps gnc_inlier_cost, the
          // trusted loop edges get the far more generous (but finite)
          // loop_trust_inlier_cost so a genuine large-drift correction is not
          // classified as an outlier — while a grossly-wrong edge still is.
          gtsam::Vector barc = gtsam::Vector::Constant(
              static_cast<Eigen::Index>(graph.size()),
              static_cast<double>(options_.gnc_inlier_cost));
          for (size_t k : loop_factors)
            barc[static_cast<Eigen::Index>(k)] =
                static_cast<double>(options_.loop_trust_inlier_cost);
          for (size_t k : odom_factors)
            barc[static_cast<Eigen::Index>(k)] =
                static_cast<double>(options_.odometry_gnc_inlier_cost);
          // A Manhattan factor stays an inlier while its normal is within
          // the gate: 0.5 * (gate / sigma)^2.
          const double man_barc =
              0.5 * (man_gate / man_sigma) * (man_gate / man_sigma);
          for (size_t k : manhattan_factors)
            barc[static_cast<Eigen::Index>(k)] = man_barc;
          const double level_barc =
              0.5 * (man_level_gate / man_sigma) * (man_level_gate / man_sigma);
          for (size_t k : manhattan_level_factors)
            barc[static_cast<Eigen::Index>(k)] = level_barc;
          for (const auto &rec : edit_recs)
            barc[static_cast<Eigen::Index>(rec.factor_idx)] =
                static_cast<double>(options_.edit_inlier_cost);
          optimizer.setInlierCostThresholds(barc);
          core::info("PlaneGraph: per-factor GNC thresholds: {} trusted loop "
                     "factors at {:.2f}, {} Manhattan wall / {} level "
                     "factors at {:.2f} / {:.2f}, others {:.2f}",
                     loop_factors.size(), options_.loop_trust_inlier_cost,
                     manhattan_factors.size(), manhattan_level_factors.size(),
                     man_barc, level_barc, options_.gnc_inlier_cost);
          if (!edit_recs.empty())
            core::info("PlaneGraph: {} pose-edit factors at a GNC threshold "
                       "of {:.2f}",
                       edit_recs.size(), options_.edit_inlier_cost);
        }
        solution = optimizer.optimize();
        gnc_weights = optimizer.getWeights();
      } else {
        gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial, lm_params);
        solution = optimizer.optimize();
        result.iterations = static_cast<int>(optimizer.iterations());
      }
    } catch (const std::exception &e) {
      core::warn("PlaneGraph: optimizer failed in round {} ({}); keeping "
                 "previous poses",
                 round + 1, e.what());
      if (round == 0)
        return result; // never solved -> converged stays false
      break;
    }
    result.converged = true;
    result.final_error = graph.error(solution);
    if (man_theta_in_graph) {
      man_theta = solution.at<double>(man_key);
      result.manhattan_axis_deg = wrap_quarter(*man_theta) * 180.0 / M_PI;
    }

    // Extract per-edge residuals for pose-graph persistence.  Repopulated
    // each round so result.edges always reflects the FINAL round's solve.
    result.edges.clear();
    result.edges.reserve(edge_recs.size());
    for (const auto &rec : edge_recs) {
      const double res = graph.at(rec.factor_idx)->error(solution);
      result.edges.push_back(
          {rec.i, rec.j, rec.type, res, rec.weight, rec.tag});
    }

    // Per-edit residuals, aggregated by tag in the order of first factor.
    result.edit_residuals.clear();
    {
      std::map<std::string, size_t> slot;
      for (const auto &rec : edit_recs) {
        const auto &f = graph.at(rec.factor_idx);
        const auto *nf = dynamic_cast<const gtsam::NoiseModelFactor *>(f.get());
        const gtsam::Vector e = nf->unwhitenedError(solution);
        const double err = f->error(solution);
        const bool inlier =
            gnc_weights.size() > 0
                ? gnc_weights[static_cast<Eigen::Index>(rec.factor_idx)] >= 0.5
                : err <= static_cast<double>(options_.edit_inlier_cost);
        auto [it, fresh] = slot.emplace(*rec.tag, result.edit_residuals.size());
        if (fresh) {
          EditResidual r;
          r.tag = *rec.tag;
          result.edit_residuals.push_back(r);
        }
        auto &r = result.edit_residuals[it->second];
        const double t = constrained_norm(
            e.tail<3>(), rec.information.bottomRightCorner<3, 3>());
        const double a =
            constrained_norm(e.head<3>(),
                             rec.information.topLeftCorner<3, 3>()) *
            180.0 / M_PI;
        r.trans_m = std::max(r.trans_m, t);
        r.rot_deg = std::max(r.rot_deg, a);
        r.rms_trans_m += t * t; // sums of squares until the loop ends
        r.rms_rot_deg += a * a;
        r.chi2 += 2.0 * err;
        r.gnc_inlier = r.gnc_inlier && inlier;
        ++r.factors;
      }
      for (auto &r : result.edit_residuals) {
        r.rms_trans_m = std::sqrt(r.rms_trans_m / r.factors);
        r.rms_rot_deg = std::sqrt(r.rms_rot_deg / r.factors);
      }
    }

    // Adopt the round's poses into cur[] and measure this round's motion.
    round_max_shift = 0.0;
    for (int i = 0; i < N; ++i) {
      const gtsam::Pose3 pose = solution.at<gtsam::Pose3>(X(i));
      const Eigen::Matrix4d M = pose.matrix();
      round_max_shift =
          std::max(round_max_shift,
                   (M.block<3, 1>(0, 3) - cur[i].block<3, 1>(0, 3)).norm());
      cur[i] = M;
    }

    core::info("PlaneGraph round {}: error {:.4f} -> {:.4f} ({} plane factors, "
               "{} under-constrained frames tightened, round shift {:.4f} m)",
               round + 1, round_initial_error, result.final_error,
               plane_factors, nuc, round_max_shift);

    if (round_max_shift < options_.assoc_round_tol) {
      core::info(
          "PlaneGraph: round shift {:.4f} m below tol {:.4f} m; stopping",
          round_max_shift, options_.assoc_round_tol);
      break;
    }
  }

  if (!result.converged)
    return result;

  for (const auto &r : result.edit_residuals)
    if (!r.gnc_inlier)
      core::warn("PlaneGraph: pose edit '{}' is NOT honoured: it contradicts "
                 "the graph or another edit (residual {:.3f} m / {:.2f} deg, "
                 "chi2 {:.1f} over {} factors, GNC threshold {:.1f}). Check "
                 "the edit against the renders, loosen its sigma, or drop it",
                 r.tag, r.trans_m, r.rot_deg, r.chi2, r.factors,
                 options_.edit_inlier_cost);

  // --- Write final poses back into the frames; report shift vs seed --------
  double max_shift = 0.0;
  for (int i = 0; i < N; ++i) {
    max_shift =
        std::max(max_shift,
                 (cur[i].block<3, 1>(0, 3) - seed[i].block<3, 1>(0, 3)).norm());
    Eigen::Affine3d aff;
    aff.matrix() = cur[i];
    frames[i].world_pose = aff;
  }
  result.max_pose_shift = max_shift;

  core::info("PlaneGraph: {} rounds, error {:.4f} -> {:.4f}, max pose shift "
             "{:.4f} m",
             result.rounds, result.initial_error, result.final_error,
             max_shift);
  return result;
}

PlaneGraphResult
PlaneGraphOptimizer::optimize(std::vector<FrameSurfels> &frames,
                              const std::vector<LoopEdge> &loop_edges) const {
  return optimize_graph(frames, legacy_graph_edges(frames, loop_edges));
}

std::vector<GraphEdge>
legacy_graph_edges(const std::vector<FrameSurfels> &frames,
                   const std::vector<LoopEdge> &loop_edges) {
  std::vector<GraphEdge> edges;
  edges.reserve(frames.size() + loop_edges.size());
  for (size_t i = 0; i + 1 < frames.size(); ++i) {
    GraphEdge e;
    e.i = static_cast<int>(i);
    e.j = static_cast<int>(i + 1);
    e.T_ij = frames[i].world_pose.matrix().inverse() *
             frames[i + 1].world_pose.matrix();
    edges.push_back(e);
  }
  for (const auto &l : loop_edges) {
    GraphEdge e;
    e.i = l.i;
    e.j = l.j;
    e.T_ij = l.T_ij;
    e.information =
        core::pose_graph::information_from_sigmas(l.sigma_rot, l.sigma_trans);
    e.kind = GraphEdge::Kind::loop;
    edges.push_back(e);
  }
  return edges;
}

// ---------------------------------------------------------------------------
// Convention check exposed for the gtsam-free regression test
// "OrientedPlane3 conventions: consistent observation has zero residual".
//
// Given an off-origin optical-frame plane (n_o.x + d_o = 0, d_o != 0) and a
// known non-trivial pose, it transforms the plane to world by hand, builds the
// world landmark and the optical measurement with the SAME Hessian convention
// the optimizer uses (distance/w = +d, no negation), constructs an
// OrientedPlane3Factor and returns the factor error norm. A correct convention
// must yield ~0; the previous (negated) encoding returned ~2.1 here, which is
// what mis-scaled the real-data graph. d_o != 0 is essential: for planes
// through the origin (d = 0) the sign is invisible.
double plane_factor_consistent_residual() {
  using gtsam::symbol_shorthand::P;
  using gtsam::symbol_shorthand::X;

  const gtsam::Rot3 R = gtsam::Rot3::RzRyRx(0.3, -0.2, 0.15);
  const gtsam::Point3 t(0.5, -0.3, 1.0);
  const gtsam::Pose3 pose(R, t);

  const Eigen::Vector3d n_o = Eigen::Vector3d(0.2, -0.3, 1.0).normalized();
  const double d_o = -0.8; // optical Hessian offset (n_o.x + d_o = 0)

  // Transform optical plane -> world (rigid): n_w = R n_o, d_w = d_o - n_w.t.
  const Eigen::Vector3d n_w = R.matrix() * n_o;
  const double d_w = d_o - n_w.dot(t);

  // Same encoding as optimize(): landmark = (Unit3(n_w), d_w); measurement =
  // (n_o, d_o). No negation — gtsam OrientedPlane3 uses n.x + d = 0 directly.
  const gtsam::OrientedPlane3 lm(gtsam::Unit3(n_w), d_w);
  const gtsam::Vector4 measured(n_o.x(), n_o.y(), n_o.z(), d_o);
  auto noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(1, 1, 1));
  gtsam::OrientedPlane3Factor factor(measured, noise, X(0), P(0));

  gtsam::Values v;
  v.insert(X(0), pose);
  v.insert(P(0), lm);
  return factor.unwhitenedError(v).norm();
}

} // namespace reusex::geometry
