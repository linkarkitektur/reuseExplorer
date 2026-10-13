// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

/// Where an instance lands in a sensor frame: the pinhole bbox of a point set
/// (moved here from vision/instance_crop, #373) and the padded, normalised
/// region the resource view record stores (spec 2026-10-13-resource-view-
/// record §2). In core because sync_survey and apply_mask_selection need it
/// and core may not link vision (STANDARDS §1); vision re-exports it.

#include "reusex/core/SensorIntrinsics.hpp"
#include "reusex/core/resource_view.hpp"

#include <Eigen/Core>

#include <array>
#include <cstdint>
#include <map>
#include <optional>
#include <string_view>
#include <vector>

namespace reusex {
class ProjectDB;
}

namespace reusex::core {

/// A projected 2D bounding box (inclusive pixel bounds), with the count of
/// points that landed inside the image.
struct ProjectedBox {
  int x_min = 0, y_min = 0, x_max = 0, y_max = 0;
  int in_bounds = 0;
};

/// Project @p world_points through @p pose (row-major 4x4 base-to-world) and
/// @p intrinsics (scaled from their native size to @p image_width x
/// @p image_height); the bbox of the points in front of the camera and inside
/// the image. nullopt when none is, for empty input or degenerate intrinsics.
std::optional<ProjectedBox>
project_points_to_box(const std::vector<Eigen::Vector3f> &world_points,
                      const std::array<double, 16> &pose,
                      const SensorIntrinsics &intrinsics, int image_width,
                      int image_height);

/// The region of frame @p frame_id that shows @p points: their bbox in the
/// frame's intrinsics resolution, normalised to 0..1 and padded by
/// @p padding per side (pad_region). nullopt when the frame has no usable
/// pose (has_sensor_frame_pose), degenerate intrinsics, or no point projects
/// inside it. No occlusion test: occluded points of the instance widen it.
std::optional<ViewRegion>
project_instance_bbox(const ProjectDB &db, int frame_id,
                      const std::vector<Eigen::Vector3f> &points,
                      double padding = kViewRegionPadding);

/// The finite points of every instance of @p label_cloud, positions taken
/// from the index-aligned @p positions_cloud. Label 0 (unlabeled) skipped.
/// @throws std::invalid_argument when either cloud is not in the project,
///         CloudMisalignedError (instance_evidence.hpp) when they differ in
///         size, std::runtime_error when a cloud cannot be loaded.
std::map<std::uint32_t, std::vector<Eigen::Vector3f>>
instance_points(const ProjectDB &db, std::string_view label_cloud,
                std::string_view positions_cloud = "cloud");

} // namespace reusex::core
