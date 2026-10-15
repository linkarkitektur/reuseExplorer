// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "vision/project.hpp"
#include "core/ProjectDB.hpp"
#include "core/SensorIntrinsics.hpp"
#include "core/label_semantics.hpp"
#include "core/logging.hpp"
#include "core/processing_observer.hpp"
#include "core/visual_observer.hpp"
#include "types.hpp"

#include <range/v3/view/iota.hpp>

#include <fmt/std.h>

#include <rtabmap/core/CameraModel.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/util2d.h>
#include <rtabmap/core/util3d.h>

#include <pcl/common/io.h> // pcl::copyPointCloud

#include <algorithm>
#include <cmath>
#include <limits>
#include <opencv2/opencv.hpp>
#include <stdexcept>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include <Eigen/Core>

#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>

using namespace rtabmap;

namespace reusex::vision {

namespace {

/// @brief Create an rtabmap::CameraModel from ProjectDB SensorIntrinsics
inline rtabmap::CameraModel createCameraModelFromIntrinsics(
    const reusex::core::SensorIntrinsics &intrinsics,
    const cv::Size &targetSize) {
  // Build 3x3 intrinsic matrix K
  cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
  K.at<double>(0, 0) = intrinsics.fx;
  K.at<double>(1, 1) = intrinsics.fy;
  K.at<double>(0, 2) = intrinsics.cx;
  K.at<double>(1, 2) = intrinsics.cy;

  // Convert local_transform to rtabmap::Transform (expects 3x4 matrix)
  // NOTE: Data is stored row-major, so we must use RowMajor mapping
  using Matrix4dRM = Eigen::Matrix<double, 4, 4, Eigen::RowMajor>;
  Matrix4dRM local_eigen =
      Eigen::Map<const Matrix4dRM>(intrinsics.local_transform.data());
  Eigen::Matrix4f local_float = local_eigen.cast<float>();
  cv::Mat local_mat_4x4;
  cv::eigen2cv(local_float, local_mat_4x4);
  // Extract first 3 rows for rtabmap::Transform (3x4 format)
  cv::Mat local_mat = local_mat_4x4(cv::Rect(0, 0, 4, 3));
  rtabmap::Transform localTransform(local_mat);

  // Create CameraModel (no distortion, no rectification)
  cv::Mat D;                              // Empty distortion
  cv::Mat R = cv::Mat::eye(3, 3, CV_64F); // No rotation
  cv::Mat P; // Empty projection matrix (will be computed from K)

  cv::Size imageSize(intrinsics.width, intrinsics.height);
  rtabmap::CameraModel cm("ProjectDB", imageSize, K, D, R, P, localTransform);

  // Scale to target size if needed
  if (targetSize != imageSize) {
    const double scaleX =
        static_cast<double>(targetSize.width) / imageSize.width;
    const double scaleY =
        static_cast<double>(targetSize.height) / imageSize.height;

    cv::Mat K_scaled = K.clone();
    K_scaled.at<double>(0, 0) *= scaleX; // fx
    K_scaled.at<double>(1, 1) *= scaleY; // fy
    K_scaled.at<double>(0, 2) *= scaleX; // cx
    K_scaled.at<double>(1, 2) *= scaleY; // cy

    return rtabmap::CameraModel("ProjectDB", targetSize, K_scaled, D, R, P,
                                localTransform);
  }

  return cm;
}

} // namespace

std::string project_labels_parameters(const ProjectLabelsOptions &options) {
  // Floats go through fmt's shortest round-trip for their own type, so 0.15f
  // is logged as 0.15, not 0.15000000596046448 (as create/stage_bridge.hpp).
  nlohmann::json j = nlohmann::json::object();
  j["max_depth_m"] =
      nlohmann::json::parse(fmt::format("{}", options.max_depth_m));
  j["depth_tolerance_m"] =
      nlohmann::json::parse(fmt::format("{}", options.depth_tolerance_m));
  j[reusex::core::kLabelEncodingKey] = reusex::core::kLabelEncodingApiPlusOne;
  return j.dump();
}

auto project(const ProjectDB &db, CloudConstPtr cloud,
             const ProjectLabelsOptions &options) -> CloudLPtr {
  reusex::trace("calling project");

  if (!std::isfinite(options.max_depth_m) || options.max_depth_m <= 0.0f)
    throw std::invalid_argument(fmt::format(
        "project (projecting_labels): max_depth_m must be > 0, got {}",
        options.max_depth_m));
  if (!std::isfinite(options.depth_tolerance_m) ||
      options.depth_tolerance_m < 0.0f)
    throw std::invalid_argument(fmt::format(
        "project (projecting_labels): depth_tolerance_m must be >= 0, got {}",
        options.depth_tolerance_m));

  // Validate the input before anything downstream can see it (STANDARDS §5).
  // Without this, an empty cloud reaches
  // rtabmap::util3d::projectCloudToCamera() and dies on an RTABMap-internal
  // UASSERT ("Condition (!laserScan->empty()) not met!") that names neither
  // this stage nor the empty input.
  if (!cloud) {
    throw std::runtime_error(
        "project (projecting_labels): input cloud pointer is null");
  }
  if (cloud->empty()) {
    throw std::runtime_error(
        "project (projecting_labels): input cloud is empty (0 points) — run "
        "`rux create clouds` before `rux create project`");
  }

  // Create output labeled point cloud
  CloudLPtr labels(new CloudL);
  labels->points.resize(cloud->points.size());
  labels->width = cloud->width;
  labels->height = cloud->height;
  labels->is_dense = cloud->is_dense;

  // Per-point (point label, frame count) votes. Within one frame each point
  // is visited by exactly one thread, so the per-point vectors need no lock;
  // the final majority_label() is order-independent, so neither frame order
  // nor thread count changes the result (STANDARDS §6).
  std::vector<std::vector<std::pair<uint32_t, int>>> votes(
      cloud->points.size());

  reusex::info("Loading sensor frames from ProjectDB ...");
  reusex::core::stopwatch timer;
  auto frameIds = db.sensor_frame_ids();
  reusex::debug("Loaded {} sensor frames in {:.3f}s", frameIds.size(), timer);
  timer.reset();

  // Degenerate but recoverable: nothing to project from, so every one of the
  // cloud's points stays unlabeled. Say so with the numbers (STANDARDS §5)
  // rather than returning a silently blank CloudL.
  if (frameIds.empty()) {
    reusex::warn("No sensor frames in project — all {} points of the input "
                 "cloud will stay unlabeled",
                 cloud->size());
  }

  // Frames that actually carried a segmentation image; used for the
  // end-of-stage diagnostic below.
  size_t framesWithLabels = 0;
  // Frames that carried a segmentation image but no usable stored pose, so
  // their labels could not be placed in the world (#336).
  size_t framesSkippedNoPose = 0;

  /* Dicription of the different coordinate frames
   *
   * Frames:
   *  - World: RTAB-Map odometry frame
   *  - Base: RTAB-Map local frame (node pose)
   *  - Camera: RTAB-Map camera frame
   *  - Image: Image plane
   *
   * Transforms:
   *  - pose: Base to World
   *  - cm.localTransform(): Camera to Base
   *  - cm.K(): Image to Camera
   *
   *  - ?: RTABMap to PCL
   */

  {
    auto observer = std::make_shared<reusex::core::ProgressObserver>(
        reusex::core::Stage::projecting_labels, frameIds.size());
    for (size_t i = 0; i < frameIds.size(); ++i) {
      int id = frameIds[i];
      reusex::trace("Processing node {}/{}", i + 1, frameIds.size());

      // --- Fetch label image
      cv::Mat labeledImage = db.segmentation_image(id); // CV_32S
      if (labeledImage.empty()) {
        reusex::warn("No segmentation for node {}, skipping", id);
        ++(*observer);
        continue;
      }

      // A frame with no usable stored pose would be projected through
      // `sensor_frame_pose()`'s identity fallback: its 2D labels would be cast
      // onto whichever cloud points happen to sit in front of the world
      // origin, writing confident semantic labels onto the wrong geometry
      // (#336). Skipping keeps those points unlabeled, which is recoverable.
      if (!db.has_sensor_frame_pose(id)) {
        reusex::debug("Node {} has no usable stored pose, skipping projection",
                      id);
        ++framesSkippedNoPose;
        ++(*observer);
        continue;
      }

      ++framesWithLabels;

      // --- Fetch sensor data from ProjectDB
      auto pose_array = db.sensor_frame_pose(id); // std::array<double, 16>
      auto intrinsics = db.sensor_frame_intrinsics(id); // SensorIntrinsics

      // Convert pose to rtabmap::Transform (expects 3x4 matrix)
      // NOTE: Data is stored row-major (see MEMORY.md), so we must use RowMajor
      // mapping
      using Matrix4dRM = Eigen::Matrix<double, 4, 4, Eigen::RowMajor>;
      Matrix4dRM pose_eigen = Eigen::Map<const Matrix4dRM>(pose_array.data());
      Eigen::Matrix4f pose_float = pose_eigen.cast<float>();
      cv::Mat pose_mat_4x4;
      cv::eigen2cv(pose_float, pose_mat_4x4);
      // Extract first 3 rows for rtabmap::Transform (3x4 format)
      cv::Mat pose_mat = pose_mat_4x4(cv::Rect(0, 0, 4, 3));
      rtabmap::Transform pose(pose_mat);

      // Convert local_transform to rtabmap::Transform (expects 3x4 matrix)
      // NOTE: Data is stored row-major, so we must use RowMajor mapping
      Matrix4dRM local_eigen =
          Eigen::Map<const Matrix4dRM>(intrinsics.local_transform.data());
      Eigen::Matrix4f local_float = local_eigen.cast<float>();
      cv::Mat local_mat_4x4;
      cv::eigen2cv(local_float, local_mat_4x4);
      // Extract first 3 rows for rtabmap::Transform (3x4 format)
      cv::Mat local_mat = local_mat_4x4(cv::Rect(0, 0, 4, 3));
      rtabmap::Transform t_cam(local_mat);

      // Create scaled camera model for projection
      rtabmap::CameraModel cm = createCameraModelFromIntrinsics(
          intrinsics, cv::Size(intrinsics.width / 16, intrinsics.height / 16));

      auto t_local = pose.inverse();

      // Observer may be null when running headless — guard before use.
      if (auto *visual_observer = reusex::core::get_visual_observer()) {
        auto camera_pose = (pose * cm.localTransform().inverse()).toEigen3f();
        visual_observer->viewer_add_camera_frustum(
            fmt::format("camera_{}", id), cm.fx(), cm.fy(),
            cm.imageSize().width, cm.imageSize().height,
            Eigen::Affine3f(camera_pose),
            reusex::core::Stage::projecting_labels, static_cast<int>(i));
      }

      pcl::Indices indices;
      indices.resize(cloud->points.size());
      std::iota(indices.begin(), indices.end(), 0);

      std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud_lf(
          new pcl::PointCloud<pcl::PointXYZ>);
      pcl::copyPointCloud(*cloud, *cloud_lf);
      cloud_lf = rtabmap::util3d::transformPointCloud(
          cloud_lf, t_local); // Mover cloud to link frame

      // --- Build Z-buffer from frustum points and fill holes
      cv::Mat zbuffer = rtabmap::util3d::projectCloudToCamera(
          cm.imageSize() /*labeledImage.size()*/, cm.K_raw(), cloud_lf, t_cam);

      reusex::trace("Filling Z-buffer holes for node {}", id);
      // rtabmap::util2d::fillDepthHoles(zbuffer, 1, 0.50f);
      // rtabmap::util2d::fillRegisteredDepthHoles(zbuffer, true, false, false);
      rtabmap::util3d::fillProjectedCloudHoles(zbuffer, true, true);

      cv::resize(zbuffer, zbuffer, labeledImage.size());

      // Scale camera model to match label image resolution
      cm = createCameraModelFromIntrinsics(intrinsics, labeledImage.size());
      {
        reusex::trace("Assigning labels for node {}", id);
        // Assign labels to the points in the point cloud
        auto fx = cm.fx();
        auto fy = cm.fy();
        auto cx = cm.cx();
        auto cy = cm.cy();
        auto h = cm.imageHeight();
        auto w = cm.imageWidth();
        const float max_depth = options.max_depth_m;
        const float tolerance = options.depth_tolerance_m;
#pragma omp parallel for shared(votes, cloud, indices, labeledImage, zbuffer)  \
    firstprivate(t_cam, fx, fy, cx, cy, w, h, max_depth, tolerance)
        for (int j = 0; j < static_cast<int>(indices.size()); ++j) {
          auto idx = indices[j];

          auto pt_cam = rtabmap::util3d::transformPoint(cloud_lf->points[idx],
                                                        t_cam.inverse());
          if (pt_cam.z <= 0.0f || pt_cam.z > max_depth)
            continue;

          auto inv_z = 1.0 / static_cast<double>(pt_cam.z);

          const double dx = (fx * static_cast<double>(pt_cam.x)) * inv_z + cx;
          const double dy = (fy * static_cast<double>(pt_cam.y)) * inv_z + cy;

          const int px = static_cast<int>(std::round(dx));
          const int py = static_cast<int>(std::round(dy));
          if (px < 0 || py < 0 || px >= w || py >= h)
            continue;

          const auto zb = zbuffer.at<float>(py, px);
          const auto z = static_cast<float>(pt_cam.z);
          if (!std::isfinite(zb) || std::abs(z - zb) > tolerance)
            continue;

          // labeledImage is API CV_32S (-1 = bg) -> point label (0 =
          // unlabeled). Background casts no vote: it never overwrites a class.
          const uint32_t label =
              reusex::core::api_to_point_label(labeledImage.at<int>(py, px));
          if (label == reusex::core::kUnlabeled)
            continue;
          auto &point_votes = votes[idx];
          auto it = std::find_if(
              point_votes.begin(), point_votes.end(),
              [label](const auto &vote) { return vote.first == label; });
          if (it != point_votes.end())
            ++it->second;
          else
            point_votes.emplace_back(label, 1);
        }
      } // end assign labels

      ++(*observer);
    }
  }

  size_t labelledPoints = 0;
#pragma omp parallel for reduction(+ : labelledPoints)
  for (int i = 0; i < static_cast<int>(votes.size()); ++i) {
    labels->points[i].label = reusex::core::majority_label(votes[i]);
    if (labels->points[i].label != reusex::core::kUnlabeled)
      ++labelledPoints;
  }

  // Degenerate output is not an error — annotation may simply not have run —
  // but it must not be silent (STANDARDS §5).
  if (framesSkippedNoPose > 0) {
    reusex::warn("Skipped {} of {} sensor frames that had a segmentation image "
                 "but no usable stored pose; their labels were not projected",
                 framesSkippedNoPose, frameIds.size());
  }

  if (!frameIds.empty() && framesWithLabels == 0) {
    reusex::warn("None of the {} sensor frames contributed labels ({} had no "
                 "segmentation image, {} had a segmentation image but no "
                 "usable stored pose) — all {} points stay unlabeled; run "
                 "`rux create annotate` first",
                 frameIds.size(),
                 frameIds.size() - framesWithLabels - framesSkippedNoPose,
                 framesSkippedNoPose, cloud->size());
  }

  if (framesWithLabels > 0 && labelledPoints == 0)
    reusex::warn("{} sensor frames contributed labels but none of the {} "
                 "points received a class vote (max depth {} m, depth "
                 "tolerance {} m) — all points stay unlabeled",
                 framesWithLabels, cloud->size(), options.max_depth_m,
                 options.depth_tolerance_m);

  reusex::info("Projected labels from {}/{} sensor frames onto {}/{} points "
               "by majority vote in {:.3f}s",
               framesWithLabels, frameIds.size(), labelledPoints, cloud->size(),
               timer);

  return labels;
}

} // namespace reusex::vision
