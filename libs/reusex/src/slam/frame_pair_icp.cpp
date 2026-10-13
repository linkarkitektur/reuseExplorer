// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Depth-cloud ICP between two stored sensor frames (#465). Moved here from
// the web GUI's ICP callback (now ruxd's own src/icp.cpp, in its own repo)
// so that callback and the Qt client's pair strip share one implementation.

#include "slam/frame_pair_icp.hpp"

#include "core/ProjectDB.hpp"
#include "core/SensorIntrinsics.hpp"

#include <Eigen/Core>
#include <Eigen/LU>

#include <opencv2/core.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/search/kdtree.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace reusex::slam {
namespace {

using PointT = pcl::PointXYZ;
using CloudT = pcl::PointCloud<PointT>;
using RowMajor4d = Eigen::Matrix<double, 4, 4, Eigen::RowMajor>;

/// Back-project a stored depth frame into a world-space cloud. Depth is
/// CV_16UC1 in millimetres; the result is in metres.
CloudT::Ptr depth_to_world_cloud(const ProjectDB &db, int node_id,
                                 const FramePairIcpOptions &o) {
  const cv::Mat depth16 = db.sensor_frame_depth(node_id);
  if (depth16.empty())
    throw std::runtime_error("frame " + std::to_string(node_id) +
                             " has no stored depth image");
  if (!db.has_sensor_frame_pose(node_id))
    throw std::runtime_error("frame " + std::to_string(node_id) +
                             " has no stored pose");

  const auto pose_arr = db.sensor_frame_pose(node_id);
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
  auto cloud = std::make_shared<CloudT>();
  cloud->reserve(static_cast<size_t>(depth_f.rows / step + 1) *
                 static_cast<size_t>(depth_f.cols / step + 1));
  for (int v = 0; v < depth_f.rows; v += step) {
    const float *row = depth_f.ptr<float>(v);
    for (int u = 0; u < depth_f.cols; u += step) {
      const float z = row[u];
      if (!(z >= o.min_depth_m && z <= o.max_depth_m))
        continue;
      const float x = (static_cast<float>(u) - cx) * z / fx;
      const float y = (static_cast<float>(v) - cy) * z / fy;
      const Eigen::Vector4f p = tf * Eigen::Vector4f(x, y, z, 1.0f);
      cloud->emplace_back(p[0], p[1], p[2]);
    }
  }
  cloud->is_dense = true;
  cloud->width = static_cast<uint32_t>(cloud->size());
  cloud->height = 1;
  if (cloud->empty())
    throw std::runtime_error(
        "frame " + std::to_string(node_id) +
        " produced an empty cloud (no depth return between " +
        std::to_string(o.min_depth_m) + " and " +
        std::to_string(o.max_depth_m) + " m)");
  return cloud;
}

} // namespace

FramePairIcpResult refine_frame_pair_icp(const ProjectDB &db, int from_id,
                                         int to_id,
                                         const FramePairIcpOptions &opts) {
  if (from_id == to_id)
    throw std::invalid_argument("refine_frame_pair_icp: 'from' and 'to' are "
                                "the same frame (" +
                                std::to_string(from_id) + ")");
  auto src = depth_to_world_cloud(db, from_id, opts);
  auto dst = depth_to_world_cloud(db, to_id, opts);

  pcl::IterativeClosestPoint<PointT, PointT> icp;
  icp.setInputSource(src);
  icp.setInputTarget(dst);
  icp.setMaximumIterations(opts.max_iterations);
  icp.setMaxCorrespondenceDistance(opts.max_correspondence_m);
  icp.setTransformationEpsilon(1e-6);
  icp.setEuclideanFitnessEpsilon(1e-4);

  CloudT aligned;
  if (opts.initial_guess) {
    const Eigen::Matrix4f guess =
        Eigen::Map<const RowMajor4d>(opts.initial_guess->data()).cast<float>();
    icp.align(aligned, guess);
  } else {
    icp.align(aligned);
  }

  const Eigen::Matrix4d delta = icp.getFinalTransformation().cast<double>();
  FramePairIcpResult result;
  result.converged = icp.hasConverged();
  // getFitnessScore() is a mean squared distance; report RMS.
  result.fitness = std::sqrt(std::max(0.0, icp.getFitnessScore()));
  result.source_points = static_cast<int>(src->size());
  result.target_points = static_cast<int>(dst->size());

  // Relative pose: T_to^{-1} * T_delta * T_from (from-camera -> to-camera).
  const auto from_arr = db.sensor_frame_pose(from_id);
  const auto to_arr = db.sensor_frame_pose(to_id);
  const Eigen::Matrix4d t_from = Eigen::Map<const RowMajor4d>(from_arr.data());
  const Eigen::Matrix4d t_to = Eigen::Map<const RowMajor4d>(to_arr.data());
  const Eigen::Matrix4d rel = t_to.inverse() * delta * t_from;

  // Inlier fraction: aligned source points within the gate of the target.
  if (!aligned.empty()) {
    pcl::search::KdTree<PointT> tree;
    tree.setInputCloud(dst);
    const float sq = opts.inlier_threshold_m * opts.inlier_threshold_m;
    int inliers = 0;
    pcl::Indices idx(1);
    std::vector<float> d2(1);
    for (const auto &pt : aligned)
      if (tree.nearestKSearch(pt, 1, idx, d2) > 0 && d2[0] <= sq)
        ++inliers;
    result.inlier_fraction =
        static_cast<double>(inliers) / static_cast<double>(src->size());
  }

  // The "from" camera's optical centre in the world (pose * local origin),
  // and how far the correction moves it.
  {
    const auto intr = db.sensor_frame_intrinsics(from_id);
    const Eigen::Matrix4d local =
        Eigen::Map<const RowMajor4d>(intr.local_transform.data());
    const Eigen::Vector4d c = t_from * local * Eigen::Vector4d(0, 0, 0, 1);
    result.source_center_shift_m = (delta * c - c).head<3>().norm();
    const double tr = delta(0, 0) + delta(1, 1) + delta(2, 2);
    result.rotation_deg =
        std::acos(std::clamp((tr - 1.0) / 2.0, -1.0, 1.0)) * 180.0 / M_PI;
  }

  const Eigen::Matrix4d edge = rel.inverse();
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j) {
      result.relative_pose[static_cast<size_t>(i * 4 + j)] = rel(i, j);
      result.world_delta[static_cast<size_t>(i * 4 + j)] = delta(i, j);
      result.edge_transform[static_cast<size_t>(i * 4 + j)] = edge(i, j);
    }
  return result;
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
