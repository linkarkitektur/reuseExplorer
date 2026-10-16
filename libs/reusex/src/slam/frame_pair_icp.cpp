// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Depth-cloud ICP between two stored sensor frames (#465). Moved here from
// the web GUI's ICP callback (now ruxd's own src/icp.cpp, in its own repo)
// so that callback and the Qt client's pair strip share one implementation.

#include "slam/frame_pair_icp.hpp"

#include "core/ProjectDB.hpp"
#include "core/SensorIntrinsics.hpp"
#include "core/logging.hpp"

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <Eigen/LU>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <pcl/cloud_iterator.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation.h>
#include <pcl/search/kdtree.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace reusex::slam {
namespace {

using RowMajor4d = Eigen::Matrix<double, 4, 4, Eigen::RowMajor>;

using PointN = pcl::PointNormal;
using CloudN = pcl::PointCloud<PointN>;

/// Neighbours per normal estimate. A pixel_step-4 frame is ~3k points, so 12
/// neighbours span a few centimetres of surface: enough to average depth
/// noise, small enough to keep a box edge from bleeding into the wall.
constexpr int kNormalNeighbours = 12;

/// A direction of the (scaled) point-to-plane normal equations whose
/// eigenvalue is below this share of the largest is unconstrained: a bare
/// wall leaves exactly 0 there, depth and normal noise ~1e-4. A feature
/// covering more than ~0.1% of the points keeps its directions.
constexpr double kUnconstrainedEigenRatio = 1e-3;

/// @p pts shifted by -@p origin, with surface normals (NaN where the
/// neighbourhood is degenerate; the solve skips those).
CloudN::Ptr
centred_cloud_with_normals(const std::vector<std::array<float, 3>> &pts,
                           const Eigen::Vector3d &origin) {
  auto cloud = std::make_shared<CloudN>();
  cloud->reserve(pts.size());
  for (const auto &p : pts) {
    PointN q;
    q.x = static_cast<float>(p[0] - origin.x());
    q.y = static_cast<float>(p[1] - origin.y());
    q.z = static_cast<float>(p[2] - origin.z());
    cloud->push_back(q);
  }
  cloud->width = static_cast<uint32_t>(cloud->size());
  cloud->height = 1;
  pcl::NormalEstimation<PointN, PointN> ne;
  ne.setInputCloud(cloud);
  ne.setSearchMethod(std::make_shared<pcl::search::KdTree<PointN>>());
  ne.setKSearch(
      std::min<int>(kNormalNeighbours, static_cast<int>(cloud->size())));
  ne.compute(*cloud); // fills normal_x/y/z and curvature, keeps x/y/z
  cloud->is_dense = false;
  return cloud;
}

/// Point-to-plane linear least squares (PCL's
/// TransformationEstimationPointToPlaneLLS linearisation) whose solve
/// DROPS the directions the correspondences do not constrain, instead of
/// inverting a singular matrix (PCL's ATA.inverse(), which returns inf/NaN
/// for a bare wall and lets near-singular directions amplify noise into a
/// slide). The rotation unknowns are scaled by @p lever_m (the cloud's RMS
/// radius) so that all six are in metres and their eigenvalues compare.
class ConstrainedPointToPlane
    : public pcl::registration::TransformationEstimation<PointN, PointN,
                                                         float> {
    public:
  using Base =
      pcl::registration::TransformationEstimation<PointN, PointN, float>;
  using Matrix4 = Base::Matrix4;

  explicit ConstrainedPointToPlane(double lever_m)
      : lever_(std::max(lever_m, 1e-3)) {}

  void estimateRigidTransformation(const CloudN &src, const CloudN &tgt,
                                   Matrix4 &out) const override {
    pcl::ConstCloudIterator<PointN> s(src), t(tgt);
    estimate(s, t, out);
  }
  void estimateRigidTransformation(const CloudN &src,
                                   const pcl::Indices &indices_src,
                                   const CloudN &tgt,
                                   Matrix4 &out) const override {
    pcl::ConstCloudIterator<PointN> s(src, indices_src), t(tgt, indices_src);
    estimate(s, t, out);
  }
  void estimateRigidTransformation(const CloudN &src,
                                   const pcl::Indices &indices_src,
                                   const CloudN &tgt,
                                   const pcl::Indices &indices_tgt,
                                   Matrix4 &out) const override {
    pcl::ConstCloudIterator<PointN> s(src, indices_src), t(tgt, indices_tgt);
    estimate(s, t, out);
  }
  void estimateRigidTransformation(const CloudN &src, const CloudN &tgt,
                                   const pcl::Correspondences &corrs,
                                   Matrix4 &out) const override {
    pcl::ConstCloudIterator<PointN> s(src, corrs, true), t(tgt, corrs, false);
    estimate(s, t, out);
  }

  /// Unconstrained directions of the last solve (0..6).
  int unconstrained_dofs() const { return unconstrained_; }

    private:
  void estimate(pcl::ConstCloudIterator<PointN> &s,
                pcl::ConstCloudIterator<PointN> &t, Matrix4 &out) const {
    using Vector6d = Eigen::Matrix<double, 6, 1>;
    using Matrix6d = Eigen::Matrix<double, 6, 6>;
    Matrix6d ata = Matrix6d::Zero();
    Vector6d atb = Vector6d::Zero();
    for (; s.isValid() && t.isValid(); ++s, ++t) {
      const Eigen::Vector3d p = s->getVector3fMap().cast<double>();
      const Eigen::Vector3d q = t->getVector3fMap().cast<double>();
      const Eigen::Vector3d n = t->getNormalVector3fMap().cast<double>();
      if (!p.allFinite() || !q.allFinite() || !n.allFinite())
        continue;
      Vector6d row;
      row.head<3>() = p.cross(n) / lever_; // d(n·(R p))/d(angle·lever)
      row.tail<3>() = n;
      ata.noalias() += row * row.transpose();
      atb.noalias() += row * n.dot(q - p);
    }
    Eigen::SelfAdjointEigenSolver<Matrix6d> eig(ata);
    const Vector6d lambda = eig.eigenvalues(); // ascending
    const double cutoff = kUnconstrainedEigenRatio * lambda[5];
    Vector6d x = Vector6d::Zero();
    int dropped = 0;
    for (int k = 0; k < 6; ++k) {
      if (!(lambda[5] > 0.0) || lambda[k] <= cutoff) {
        ++dropped;
        continue;
      }
      const auto v = eig.eigenvectors().col(k);
      x += v * (v.dot(atb) / lambda[k]);
    }
    unconstrained_ = dropped;
    x.head<3>() /= lever_;
    // PCL's convention: R = Rz(x2) * Ry(x1) * Rx(x0), then the translation.
    Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
    m.topLeftCorner<3, 3>() =
        (Eigen::AngleAxisd(x[2], Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(x[1], Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(x[0], Eigen::Vector3d::UnitX()))
            .toRotationMatrix();
    m.topRightCorner<3, 1>() = x.tail<3>();
    out = m.cast<float>();
  }

  double lever_;
  mutable int unconstrained_ = 0;
};

/// Overlap measures of @p src (moved by @p m) against @p tree's cloud.
struct Overlap {
  double fitness = 0;         ///< RMS NN distance over all points
  double inlier_fraction = 0; ///< share within the gate
  /// RMS point-to-plane distance over those (to the neighbour's tangent
  /// plane): the surfaces' disagreement, free of the sampling spacing.
  double inlier_rms = 0;
};

Overlap overlap(const CloudN &src, const CloudN &dst,
                const pcl::search::KdTree<PointN> &tree,
                const Eigen::Matrix4d &m, double gate_m) {
  Overlap o;
  if (src.empty())
    return o;
  double all = 0, in = 0;
  int n_in = 0;
  pcl::Indices idx(1);
  std::vector<float> d2(1);
  const double gate2 = gate_m * gate_m;
  for (const auto &p : src) {
    const Eigen::Vector3d q =
        m.topLeftCorner<3, 3>() * p.getVector3fMap().cast<double>() +
        m.topRightCorner<3, 1>();
    PointN pt;
    pt.getVector3fMap() = q.cast<float>();
    if (tree.nearestKSearch(pt, 1, idx, d2) < 1)
      continue;
    all += d2[0];
    if (d2[0] <= gate2) {
      const PointN &t = dst[static_cast<std::size_t>(idx[0])];
      const Eigen::Vector3d n = t.getNormalVector3fMap().cast<double>();
      const double r = n.allFinite()
                           ? n.dot(q - t.getVector3fMap().cast<double>())
                           : std::sqrt(static_cast<double>(d2[0]));
      in += r * r;
      ++n_in;
    }
  }
  const auto n = static_cast<double>(src.size());
  o.fitness = std::sqrt(all / n);
  o.inlier_fraction = n_in / n;
  o.inlier_rms = n_in > 0 ? std::sqrt(in / n_in) : 0.0;
  return o;
}

/// Rotation angle of @p m (degrees).
double rotation_deg_of(const Eigen::Matrix4d &m) {
  const double tr = m(0, 0) + m(1, 1) + m(2, 2);
  return std::acos(std::clamp((tr - 1.0) / 2.0, -1.0, 1.0)) * 180.0 / M_PI;
}

/// The stored colour image as 8-bit BGR, or empty with @p why set when the
/// frame has none or it is in a format the overlay cannot show.
cv::Mat color_bgr(const ProjectDB &db, int node_id, std::string &why) {
  const cv::Mat img = db.sensor_frame_image(node_id);
  if (img.empty()) {
    why = "no stored colour image";
    return {};
  }
  if (img.depth() != CV_8U) {
    why = "colour image is not 8-bit (cv depth " + std::to_string(img.depth()) +
          ")";
    return {};
  }
  cv::Mat bgr;
  switch (img.channels()) {
  case 3:
    return img;
  case 1:
    cv::cvtColor(img, bgr, cv::COLOR_GRAY2BGR);
    return bgr;
  case 4:
    cv::cvtColor(img, bgr, cv::COLOR_BGRA2BGR);
    return bgr;
  default:
    why = "colour image has " + std::to_string(img.channels()) + " channels";
    return {};
  }
}

/// Back-project @p node_id's depth into world space; with @p color (8-bit
/// BGR) also sample each point's RGB into @p colors.
std::vector<std::array<float, 3>>
back_project(const ProjectDB &db, int node_id, const FramePairIcpOptions &o,
             const cv::Mat *color,
             std::vector<std::array<std::uint8_t, 3>> *colors,
             bool in_world = true) {
  const cv::Mat depth16 = db.sensor_frame_depth(node_id);
  if (depth16.empty())
    throw std::runtime_error("frame " + std::to_string(node_id) +
                             " has no stored depth image");
  if (in_world && !db.has_sensor_frame_pose(node_id))
    throw std::runtime_error("frame " + std::to_string(node_id) +
                             " has no stored pose");

  const auto pose_arr = in_world
                            ? db.sensor_frame_pose(node_id)
                            : std::array<double, 16>{1, 0, 0, 0, 0, 1, 0, 0,
                                                     0, 0, 1, 0, 0, 0, 0, 1};
  const auto intr = db.sensor_frame_intrinsics(node_id);

  cv::Mat depth_f;
  depth16.convertTo(depth_f, CV_32FC1, 1.0 / 1000.0);

  // Scale the intrinsics to the depth resolution (as reconstruct.cpp does).
  const double sx = static_cast<double>(depth_f.cols) / std::max(1, intr.width);
  const double sy =
      static_cast<double>(depth_f.rows) / std::max(1, intr.height);
  const float fx = static_cast<float>(intr.fx * sx);
  const float fy = static_cast<float>(intr.fy * sy);
  const float cx = static_cast<float>(intr.cx * sx);
  const float cy = static_cast<float>(intr.cy * sy);

  const Eigen::Matrix4f world_tf =
      Eigen::Map<const RowMajor4d>(pose_arr.data()).cast<float>();
  const Eigen::Matrix4f local_tf =
      Eigen::Map<const RowMajor4d>(intr.local_transform.data()).cast<float>();
  const Eigen::Matrix4f tf = world_tf * local_tf;

  const int step = std::max(1, o.pixel_step);
  std::vector<std::array<float, 3>> pts;
  pts.reserve(static_cast<size_t>(depth_f.rows / step + 1) *
              static_cast<size_t>(depth_f.cols / step + 1));
  if (color)
    colors->reserve(pts.capacity());
  for (int v = 0; v < depth_f.rows; v += step) {
    const float *row = depth_f.ptr<float>(v);
    for (int u = 0; u < depth_f.cols; u += step) {
      const float z = row[u];
      if (!(z >= o.min_depth_m && z <= o.max_depth_m))
        continue;
      const float x = (static_cast<float>(u) - cx) * z / fx;
      const float y = (static_cast<float>(v) - cy) * z / fy;
      const Eigen::Vector4f p = tf * Eigen::Vector4f(x, y, z, 1.0f);
      pts.push_back({p[0], p[1], p[2]});
      if (color) {
        // Depth pixel -> colour pixel (the two may differ in resolution),
        // as reconstruct.cpp maps it.
        const int col_u =
            std::clamp(u * color->cols / depth_f.cols, 0, color->cols - 1);
        const int col_v =
            std::clamp(v * color->rows / depth_f.rows, 0, color->rows - 1);
        const cv::Vec3b &bgr = color->at<cv::Vec3b>(col_v, col_u);
        colors->push_back({bgr[2], bgr[1], bgr[0]});
      }
    }
  }
  if (pts.empty())
    throw std::runtime_error(
        "frame " + std::to_string(node_id) +
        " produced an empty cloud (no depth return between " +
        std::to_string(o.min_depth_m) + " and " +
        std::to_string(o.max_depth_m) + " m)");
  return pts;
}

} // namespace

std::vector<std::array<float, 3>>
frame_world_points(const ProjectDB &db, int node_id,
                   const FramePairIcpOptions &o) {
  return back_project(db, node_id, o, nullptr, nullptr);
}

std::vector<std::array<float, 3>>
frame_pose_frame_points(const ProjectDB &db, int node_id,
                        const FramePairIcpOptions &o) {
  return back_project(db, node_id, o, nullptr, nullptr, /*in_world=*/false);
}

FrameWorldCloud frame_world_colored_points(const ProjectDB &db, int node_id,
                                           const FramePairIcpOptions &o) {
  std::string why;
  const cv::Mat color = color_bgr(db, node_id, why);
  FrameWorldCloud out;
  // Throws, as frame_world_points() does, for a frame without depth or pose.
  out.points = back_project(db, node_id, o, color.empty() ? nullptr : &color,
                            &out.colors);
  if (color.empty())
    core::warn("frame {}: {} back-projected points have no colour ({}); "
               "they keep the tint",
               node_id, out.points.size(), why);
  return out;
}

FramePairIcpResult refine_frame_pair_icp(const ProjectDB &db, int from_id,
                                         int to_id,
                                         const FramePairIcpOptions &opts) {
  if (from_id == to_id)
    throw std::invalid_argument("refine_frame_pair_icp: 'from' and 'to' are "
                                "the same frame (" +
                                std::to_string(from_id) + ")");
  const auto src_pts = frame_world_points(db, from_id, opts);
  const auto dst_pts = frame_world_points(db, to_id, opts);

  // Work about the "to" cloud's centroid: world coordinates can be tens of
  // metres from the origin, which costs float precision in the clouds and
  // conditioning in the rotation unknowns.
  Eigen::Vector3d origin = Eigen::Vector3d::Zero();
  for (const auto &p : dst_pts)
    origin += Eigen::Vector3d(p[0], p[1], p[2]);
  origin /= static_cast<double>(dst_pts.size());
  Eigen::Matrix4d to_centred = Eigen::Matrix4d::Identity();
  to_centred.topRightCorner<3, 1>() = -origin;
  const Eigen::Matrix4d from_centred = to_centred.inverse();

  auto src = centred_cloud_with_normals(src_pts, origin);
  auto dst = centred_cloud_with_normals(dst_pts, origin);
  double lever = 0;
  for (const auto &p : *dst)
    lever += p.getVector3fMap().cast<double>().squaredNorm();
  lever = std::sqrt(lever / static_cast<double>(dst->size()));

  const Eigen::Matrix4d seed_world =
      opts.initial_guess ? Eigen::Matrix4d(Eigen::Map<const RowMajor4d>(
                               opts.initial_guess->data()))
                         : Eigen::Matrix4d::Identity();
  const Eigen::Matrix4d seed = to_centred * seed_world * from_centred;

  auto estimator = std::make_shared<ConstrainedPointToPlane>(lever);
  pcl::IterativeClosestPointWithNormals<PointN, PointN> icp;
  icp.setTransformationEstimation(estimator);
  icp.setInputSource(src);
  icp.setInputTarget(dst);
  icp.setMaximumIterations(opts.max_iterations);
  icp.setMaxCorrespondenceDistance(opts.max_correspondence_m);
  icp.setTransformationEpsilon(1e-6);
  // PCL compares this ABSOLUTE epsilon with the change of the mean squared
  // correspondence distance (m^2): 1e-4 stopped ICP while a weakly
  // constrained direction still crept 1 cm per iteration. Converge on the
  // step size (1 mm) instead; max_iterations bounds the loop.
  icp.setEuclideanFitnessEpsilon(1e-10);
  CloudN aligned;
  icp.align(aligned, seed.cast<float>());

  const Eigen::Matrix4d centred_delta =
      icp.getFinalTransformation().cast<double>();
  const Eigen::Matrix4d delta = from_centred * centred_delta * to_centred;

  FramePairIcpResult result;
  result.converged = icp.hasConverged() && delta.allFinite();
  result.unconstrained_dofs = estimator->unconstrained_dofs();
  result.source_points = static_cast<int>(src->size());
  result.target_points = static_cast<int>(dst->size());

  pcl::search::KdTree<PointN> tree;
  tree.setInputCloud(dst);
  const Overlap at_seed =
      overlap(*src, *dst, tree, seed, opts.inlier_threshold_m);
  const Overlap at_end =
      overlap(*src, *dst, tree, centred_delta, opts.inlier_threshold_m);
  result.fitness = at_end.fitness;
  result.inlier_fraction = at_end.inlier_fraction;
  result.inlier_rms_m = at_end.inlier_rms;
  result.seed_inlier_fraction = at_seed.inlier_fraction;
  result.seed_inlier_rms_m = at_seed.inlier_rms;

  // Relative pose: T_to^{-1} * T_delta * T_from (from-camera -> to-camera).
  const auto from_arr = db.sensor_frame_pose(from_id);
  const auto to_arr = db.sensor_frame_pose(to_id);
  const Eigen::Matrix4d t_from = Eigen::Map<const RowMajor4d>(from_arr.data());
  const Eigen::Matrix4d t_to = Eigen::Map<const RowMajor4d>(to_arr.data());
  const Eigen::Matrix4d rel = t_to.inverse() * delta * t_from;

  // The "from" camera's optical centre in the world (pose * local origin),
  // and how far the correction moves it.
  {
    const auto intr = db.sensor_frame_intrinsics(from_id);
    const Eigen::Matrix4d local =
        Eigen::Map<const RowMajor4d>(intr.local_transform.data());
    const Eigen::Vector4d c = t_from * local * Eigen::Vector4d(0, 0, 0, 1);
    result.source_center_shift_m = (delta * c - c).head<3>().norm();
    result.rotation_deg = rotation_deg_of(delta);
    // Measured where the seed put the camera: ICP's own move.
    const Eigen::Vector4d seeded_c = seed_world * c;
    const Eigen::Matrix4d move = delta * seed_world.inverse();
    result.seed_shift_m = (move * seeded_c - seeded_c).head<3>().norm();
    result.seed_rotation_deg = rotation_deg_of(move);
  }
  result.worse_than_seed = icp_worse_than_seed(result, opts);
  if (result.worse_than_seed)
    core::warn("ICP of frame {} onto {} moved it {:.3f} m / {:.2f} deg from "
               "its start and fits worse there: {:.1f}% -> {:.1f}% of points "
               "within {:.3f} m, inlier RMS {:.4f} -> {:.4f} m; keep the "
               "start pose",
               from_id, to_id, result.seed_shift_m, result.seed_rotation_deg,
               100.0 * result.seed_inlier_fraction,
               100.0 * result.inlier_fraction, opts.inlier_threshold_m,
               result.seed_inlier_rms_m, result.inlier_rms_m);
  if (result.unconstrained_dofs > 0)
    core::debug("ICP of frame {} onto {}: {} of 6 directions unconstrained "
                "by the surfaces (left unmoved)",
                from_id, to_id, result.unconstrained_dofs);

  const Eigen::Matrix4d edge = rel.inverse();
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j) {
      result.relative_pose[static_cast<size_t>(i * 4 + j)] = rel(i, j);
      result.world_delta[static_cast<size_t>(i * 4 + j)] = delta(i, j);
      result.edge_transform[static_cast<size_t>(i * 4 + j)] = edge(i, j);
    }
  return result;
}

bool icp_worse_than_seed(const FramePairIcpResult &r,
                         const FramePairIcpOptions &o) {
  const bool far = r.seed_shift_m > o.inlier_threshold_m ||
                   r.seed_rotation_deg > kIcpSeedDriftDeg;
  const bool worse =
      r.inlier_fraction < r.seed_inlier_fraction ||
      r.inlier_rms_m > r.seed_inlier_rms_m * (1.0 + kIcpSeedRmsTolerance);
  return far && worse;
}

std::array<double, 36> icp_edge_information(const FramePairIcpResult &result) {
  const double st = std::max(result.fitness, 0.01);
  const double sr = std::max(result.fitness / 2.0, 0.005);
  std::array<double, 36> info{};
  for (int k = 0; k < 3; ++k) {
    info[static_cast<size_t>(k * 6 + k)] = 1.0 / (sr * sr);
    info[static_cast<size_t>((k + 3) * 6 + (k + 3))] = 1.0 / (st * st);
  }
  return info;
}

} // namespace reusex::slam
