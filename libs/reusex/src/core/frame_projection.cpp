// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "core/frame_projection.hpp"

#include "instance_points.hpp"

#include "core/ProjectDB.hpp"
#include "core/instance_evidence.hpp"
#include "core/logging.hpp"

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace reusex::core {

std::optional<ProjectedBox>
project_points_to_box(const std::vector<Eigen::Vector3f> &world_points,
                      const std::array<double, 16> &pose,
                      const SensorIntrinsics &intrinsics, int image_width,
                      int image_height) {
  if (world_points.empty() || image_width <= 0 || image_height <= 0)
    return std::nullopt;
  if (intrinsics.fx <= 0.0 || intrinsics.fy <= 0.0 || intrinsics.width <= 0 ||
      intrinsics.height <= 0)
    return std::nullopt;

  // Row-major 4x4 world<-base pose and base<-camera local transform.
  using Matrix4dRM = Eigen::Matrix<double, 4, 4, Eigen::RowMajor>;
  Eigen::Matrix4d base_to_world = Eigen::Map<const Matrix4dRM>(pose.data());
  Eigen::Matrix4d cam_to_base =
      Eigen::Map<const Matrix4dRM>(intrinsics.local_transform.data());

  // world -> camera = (base_to_world * cam_to_base)^-1
  Eigen::Matrix4d cam_to_world = base_to_world * cam_to_base;
  Eigen::Matrix4d world_to_cam = cam_to_world.inverse();

  // Scale intrinsics from their native resolution to the requested image size.
  const double sx = static_cast<double>(image_width) / intrinsics.width;
  const double sy = static_cast<double>(image_height) / intrinsics.height;
  const double fx = intrinsics.fx * sx;
  const double fy = intrinsics.fy * sy;
  const double cx = intrinsics.cx * sx;
  const double cy = intrinsics.cy * sy;

  int x_min = image_width, y_min = image_height, x_max = -1, y_max = -1;
  int in_bounds = 0;

  for (const auto &p : world_points) {
    Eigen::Vector4d pw(p.x(), p.y(), p.z(), 1.0);
    Eigen::Vector4d pc = world_to_cam * pw;
    const double z = pc.z();
    if (!(z > 0.0)) // behind camera or degenerate
      continue;

    const double u = fx * (pc.x() / z) + cx;
    const double v = fy * (pc.y() / z) + cy;
    const int px = static_cast<int>(std::lround(u));
    const int py = static_cast<int>(std::lround(v));
    if (px < 0 || py < 0 || px >= image_width || py >= image_height)
      continue;

    ++in_bounds;
    x_min = std::min(x_min, px);
    y_min = std::min(y_min, py);
    x_max = std::max(x_max, px);
    y_max = std::max(y_max, py);
  }

  if (in_bounds == 0)
    return std::nullopt;

  ProjectedBox box;
  box.x_min = x_min;
  box.y_min = y_min;
  box.x_max = x_max;
  box.y_max = y_max;
  box.in_bounds = in_bounds;
  return box;
}

std::optional<ViewRegion>
project_instance_bbox(const ProjectDB &db, int frame_id,
                      const std::vector<Eigen::Vector3f> &points,
                      double padding) {
  if (points.empty() || !db.has_sensor_frame_pose(frame_id))
    return std::nullopt;
  const auto intr = db.sensor_frame_intrinsics(frame_id);
  if (intr.width <= 0 || intr.height <= 0)
    return std::nullopt;
  const auto box = project_points_to_box(points, db.sensor_frame_pose(frame_id),
                                         intr, intr.width, intr.height);
  if (!box)
    return std::nullopt;
  const double w = intr.width;
  const double h = intr.height;
  return pad_region({box->x_min / w, box->y_min / h,
                     (box->x_max - box->x_min + 1) / w,
                     (box->y_max - box->y_min + 1) / h},
                    padding);
}

std::map<std::uint32_t, std::vector<Eigen::Vector3f>>
instance_points(const ProjectDB &db, std::string_view label_cloud,
                std::string_view positions_cloud) {
  return detail::load_instance_points(db, label_cloud, positions_cloud,
                                      "instance_points");
}

ResourceView complete_view_region(const ProjectDB &db, ResourceView view,
                                  std::string_view label_cloud,
                                  std::uint32_t instance_id,
                                  std::string_view positions_cloud) {
  // A frame that does not exist is the caller's error to report; leave it.
  if (view.kind != ViewKind::frame || view.region ||
      !db.has_sensor_frame(view.id))
    return view;
  std::map<std::uint32_t, std::vector<Eigen::Vector3f>> all;
  try {
    all = instance_points(db, label_cloud, positions_cloud);
  } catch (const std::invalid_argument &e) {
    reusex::warn("frame view {} of instance {}: whole image, cannot place the "
                 "instance ({})",
                 view.id, instance_id, e.what());
    return view;
  } catch (const CloudMisalignedError &e) {
    reusex::warn("frame view {} of instance {}: whole image, cannot place the "
                 "instance ({})",
                 view.id, instance_id, e.what());
    return view;
  }
  const auto it = all.find(instance_id);
  if (it == all.end()) {
    reusex::info("frame view {}: instance {} of '{}' has no finite points; "
                 "whole image",
                 view.id, instance_id, label_cloud);
    return view;
  }
  view.region = project_instance_bbox(db, view.id, it->second);
  if (!view.region)
    reusex::info("frame view {}: instance {} of '{}' ({} points) does not "
                 "project into the frame; whole image",
                 view.id, instance_id, label_cloud, it->second.size());
  return view;
}

std::map<std::uint32_t, std::vector<Eigen::Vector3f>>
detail::load_instance_points(const ProjectDB &db, std::string_view label_cloud,
                             std::string_view positions_cloud,
                             std::string_view caller) {
  const std::string who(caller);
  if (!db.has_point_cloud(label_cloud))
    throw std::invalid_argument(who + ": no point cloud '" +
                                std::string(label_cloud) + "'");
  if (!db.has_point_cloud(positions_cloud))
    throw std::invalid_argument(who + ": no positions cloud '" +
                                std::string(positions_cloud) + "'");
  const auto positions = db.point_cloud_xyzrgb(positions_cloud);
  const auto labels = db.point_cloud_label(label_cloud);
  if (!positions || !labels)
    throw std::runtime_error(who + ": could not load '" +
                             std::string(positions_cloud) + "' or '" +
                             std::string(label_cloud) + "'");
  if (positions->size() != labels->size())
    throw CloudMisalignedError(
        "instance-label cloud '" + std::string(label_cloud) + "' has " +
        std::to_string(labels->size()) + " points but base cloud '" +
        std::string(positions_cloud) + "' has " +
        std::to_string(positions->size()) + " — they are not index-aligned");
  std::map<std::uint32_t, std::vector<Eigen::Vector3f>> out;
  for (std::size_t i = 0; i < labels->size(); ++i) {
    const auto label = labels->points[i].label;
    if (label == 0)
      continue; // unlabeled (STANDARDS §3)
    const auto &p = positions->points[i];
    // One NaN must not poison an average or a bbox.
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
      continue;
    out[label].emplace_back(p.x, p.y, p.z);
  }
  return out;
}

} // namespace reusex::core
