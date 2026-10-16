// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "segmentation/surfel_extraction.hpp"
#include "core/ProjectDB.hpp"
#include "core/SensorIntrinsics.hpp"
#include "core/logging.hpp"
#include "geometry/transform_utils.hpp"
#include "segmentation/depth_filters.hpp"

#include <opencv2/core.hpp>

#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/voxel_grid.h>

#include <algorithm>

namespace reusex::geometry {

std::optional<FrameSurfels>
extract_frame_surfels(const ProjectDB &db, int node_id,
                      const SurfelExtractionParams &params) {
  cv::Mat color = db.sensor_frame_image(node_id);
  cv::Mat depth16 = db.sensor_frame_depth(node_id); // CV_16UC1 mm
  cv::Mat confidence = db.sensor_frame_confidence(node_id);
  auto intr = db.sensor_frame_intrinsics(node_id);

  if (color.empty() || depth16.empty()) {
    core::debug("Node {}: missing color or depth, skipping surfel extraction",
                node_id);
    return std::nullopt;
  }

  // `world_pose` is the SEED the registration optimisers start from, and both
  // of them write their answer straight back to the database. Seeding from
  // `sensor_frame_pose()`'s identity fallback would put a frame at the origin
  // and let JPR/PlaneGraph drag the rest of the graph toward it; an all-zero
  // transform seeds a singular world_pose outright (#336).
  if (!db.has_sensor_frame_pose(node_id)) {
    core::debug("Node {}: no usable stored pose, skipping surfel extraction",
                node_id);
    return std::nullopt;
  }
  auto pose = db.sensor_frame_pose(node_id);
  if (intr.width <= 0 || intr.height <= 0) {
    core::warn("Node {}: intrinsics have invalid dimensions {}x{}", node_id,
               intr.width, intr.height);
    return std::nullopt;
  }

  // Convert depth to float meters and apply the same filters as reconstruction.
  cv::Mat depth_f;
  depth16.convertTo(depth_f, CV_32FC1, 1.0 / 1000.0);
  if (params.apply_depth_filters) {
    apply_depth_discontinuity_filter(depth_f, confidence, 0.5f);
    apply_ray_consistency_filter(depth_f, confidence, 0.2f);
  }

  // Scale intrinsics to depth resolution (mirrors reconstruct_point_clouds).
  const double scale_x =
      static_cast<double>(depth_f.cols) / std::max(1, intr.width);
  const double scale_y =
      static_cast<double>(depth_f.rows) / std::max(1, intr.height);
  const float fx = static_cast<float>(intr.fx * scale_x);
  const float fy = static_cast<float>(intr.fy * scale_y);
  const float cx = static_cast<float>(intr.cx * scale_x);
  const float cy = static_cast<float>(intr.cy * scale_y);
  const int step = std::max(1, params.sampling_factor);
  const float min_d = params.min_distance;
  const float max_d = params.max_distance;
  const int conf_thresh = params.confidence_threshold;

  auto points = std::make_shared<Cloud>();
  const size_t estimated =
      static_cast<size_t>((depth_f.rows / step) * (depth_f.cols / step));
  points->reserve(estimated);

  // Back-project into the OPTICAL frame (no world/local transform applied).
  for (int v = 0; v < depth_f.rows; v += step) {
    const float *depth_row = depth_f.ptr<float>(v);
    const uchar *conf_row =
        confidence.empty() ? nullptr : confidence.ptr<uchar>(v);
    for (int u = 0; u < depth_f.cols; u += step) {
      const float z = depth_row[u];
      if (z <= 0.0f || z < min_d || z > max_d)
        continue;
      if (conf_row && conf_row[u] < conf_thresh)
        continue;

      PointT pt;
      pt.x = (static_cast<float>(u) - cx) * z / fx;
      pt.y = (static_cast<float>(v) - cy) * z / fy;
      pt.z = z;

      int col_u = u * color.cols / depth_f.cols;
      int col_v = v * color.rows / depth_f.rows;
      col_u = std::clamp(col_u, 0, color.cols - 1);
      col_v = std::clamp(col_v, 0, color.rows - 1);
      const auto &pixel = color.at<cv::Vec3b>(col_v, col_u);
      pt.b = pixel[0];
      pt.g = pixel[1];
      pt.r = pixel[2];
      points->push_back(pt);
    }
  }

  if (points->size() < 16) {
    core::debug("Node {}: only {} surfels, skipping", node_id, points->size());
    return std::nullopt;
  }

  // Optional per-frame voxel downsample to bound surfel count.
  if (params.voxel_size > 0.0f) {
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(points);
    vg.setLeafSize(params.voxel_size, params.voxel_size, params.voxel_size);
    auto ds = std::make_shared<Cloud>();
    vg.filter(*ds);
    if (ds->size() >= 16)
      points = ds;
  }

  // Estimate normals in the optical frame, viewpoint at origin (camera center)
  // so they consistently face the camera.
  pcl::NormalEstimationOMP<PointT, NormalT> ne;
  ne.setInputCloud(points);
  ne.setRadiusSearch(params.normal_radius);
  ne.setViewPoint(0.0f, 0.0f, 0.0f);
  auto normals = std::make_shared<CloudN>();
  ne.compute(*normals);

  FrameSurfels frame;
  frame.node_id = node_id;
  frame.points = points;
  frame.normals = normals;
  // world_pose = worldTf * localTf  (optical -> world).
  frame.world_pose = to_affine(pose) * to_affine(intr.local_transform);
  return frame;
}

} // namespace reusex::geometry
