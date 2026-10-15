// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "reusex/types/point_types.hpp"

#include <string>

namespace reusex {
class ProjectDB;
}

namespace reusex::vision {

/// Tunables of project() — the single source of the defaults `rux create
/// project` mirrors (STANDARDS §4).
struct ProjectLabelsOptions {
  /// Points farther than this from a frame's camera (metres, along the
  /// optical axis) take no label from that frame. Typical: 3-7 m for RGB-D
  /// depth cameras, 10-30 m for LiDAR.
  float max_depth_m = 7.0f;
  /// Z-buffer agreement (metres) for a point to count as visible in a frame;
  /// a point farther than this behind the frame's z-buffer is occluded.
  float depth_tolerance_m = 0.15f;
};

/**
 * @brief Project the per-frame segmentation labels stored in @p db onto
 * @p cloud by multi-view majority vote.
 *
 * For each sensor frame with a segmentation image and a usable pose, this
 * builds a z-buffer of the cloud and casts a vote, for every point visible in
 * that frame, for the class at its pixel. A point's label is
 * core::majority_label() of its votes: background never votes (so it never
 * overwrites a class), and an exact tie goes to the lowest class. The result
 * is independent of frame order and thread count.
 *
 * @return One point label per cloud point (API class + 1, 0 = unlabeled).
 * @throws std::runtime_error on a null or empty cloud.
 * @throws std::invalid_argument if `max_depth_m <= 0` or
 *         `depth_tolerance_m < 0` (or either is not finite).
 */
auto project(const ProjectDB &db, CloudConstPtr cloud,
             const ProjectLabelsOptions &options = {}) -> CloudLPtr;

/// The `parameters` JSON of the semantic cloud project() produces: the
/// options plus `"label_encoding": "api_plus_one"` (STANDARDS §3.1).
std::string project_labels_parameters(const ProjectLabelsOptions &options);

} // namespace reusex::vision
