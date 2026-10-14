// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

/// Turn a 2D mask drawn on one sensor frame into a labelled 3D selection and a
/// survey part (Segmentering view, spec B2).
///
/// Lives in `core` (not `vision`) for the same reason `frame_visibility` does:
/// the GUI server (`ruxd_api_lib`) links `reusex_core` but not `reusex_vision`,
/// and the existing batch projector `vision::project()` drags in RTABMap. The
/// projection here is a plain pinhole model plus an occlusion test against the
/// frame's own depth image — no z-buffer, no RTABMap, no PCL in this header.

#include "reusex/core/ProjectDB.hpp"
#include "reusex/core/resource_view.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace cv {
class Mat;
} // namespace cv

namespace reusex::core {

/// A frame cannot be projected (no usable pose, no depth image, no base
/// cloud, degenerate intrinsics) or a selection is unusable (no points).
/// The GUI maps this to 422: the request was well formed, the project state
/// does not allow it.
class MaskSelectionError : public std::runtime_error {
    public:
  using std::runtime_error::runtime_error;
};

/// Tunables for `project_frame_mask()` (STANDARDS §4).
struct MaskProjectionOptions {
  /// Base cloud the mask selects from (index space of the result).
  std::string cloud = "cloud";
  /// A point is visible when `|z - depth(u,v)| <= depth_tolerance` (metres),
  /// where `depth(u,v)` is the frame's own depth image at the projection.
  double depth_tolerance = 0.10;
  /// Points farther than this from the camera (metres) are ignored; past it a
  /// consumer depth sensor's readings are too noisy to trust. `<= 0` disables
  /// the limit.
  double max_depth = 8.0;
};

/**
 * @brief Indices of the base-cloud points that the mask covers in one frame.
 *
 * Every point of `opts.cloud` is placed in the camera optical frame with
 * `world -> camera = (pose * local_transform)^-1` (the `visible_frames()` /
 * `vision::project()` convention) and projected with the frame's pinhole
 * intrinsics. Pixel coordinates are computed in the intrinsics' own
 * `width x height` and rescaled to the mask's and the depth image's sizes, so
 * the mask may be the colour image's size (a saved segmentation image) or any
 * other. A point is selected when it is in front of the camera, within
 * `max_depth`, lands inside the image on a non-zero mask pixel, and passes the
 * occlusion test against the frame's depth image (CV_16U millimetres or CV_32F
 * metres; a pixel with no depth reading rejects the point — it cannot be
 * confirmed visible).
 *
 * Zero hits is not an error (the mask may cover only background beyond the
 * cloud); it logs a `warn` with the counts that explain it (STANDARDS §5).
 *
 * @param mask Single-channel mask, non-zero = selected (CV_8U or CV_32S).
 * @return Selected point indices, ascending.
 * @throws MaskSelectionError when the frame has no usable stored pose, no
 *         depth image, degenerate intrinsics, or the base cloud is missing.
 * @throws std::invalid_argument when the mask is empty or multi-channel.
 */
std::vector<std::size_t>
project_frame_mask(const ProjectDB &db, int frame_id, const cv::Mat &mask,
                   const MaskProjectionOptions &opts = {});

/// Normalised, unpadded bbox [x, y, w, h] of the non-zero pixels of a
/// single-channel CV_8U or CV_32S mask; nullopt when none is set.
/// @throws std::invalid_argument for a multi-channel or other-depth mask.
std::optional<ViewRegion> mask_region(const cv::Mat &mask);

/// Tunables for `apply_mask_selection()`.
struct MaskSelectionOptions {
  /// Base cloud the indices refer to; its size sizes a missing label cloud.
  std::string base_cloud = "cloud";
  std::string semantic_cloud = "labels"; // pipeline kDefaultSemanticCloud
  std::string instance_cloud = std::string(kDefaultInstanceCloud);
  /// Optional rooms cloud: when present and index-aligned, the new part gets
  /// the room most of its points fall in (as `sync_survey` does).
  std::string rooms_cloud = "rooms";
  /// Survey type for the new part. Unset: for a class that already existed,
  /// the type `sync_survey` would use for it (same `semantic_class`); else a
  /// type named exactly `class_name`; else a new type named `class_name`. A
  /// class created by this call never matches by `semantic_class` — a type
  /// carrying that id is a leftover from an earlier `labels` generation.
  /// Rejected (`ReviewStatus::rejected`) types are skipped by both matches:
  /// when only a rejected type matches, a new type is created instead of
  /// reviving it. Set to a rejected type, the call throws MaskSelectionError.
  std::optional<int64_t> type_id;
  /// Recorded in the pipeline log entry; `-1` when the selection did not come
  /// from a frame.
  int frame_id = -1;
  /// The mask's bbox in frame_id's image (mask_region()), unpadded. The new
  /// part's view is frame_id with this region padded 15 % per side. Unset
  /// with frame_id >= 0: the selected points' bbox in that frame is used.
  /// Validated up front (std::invalid_argument, nothing written).
  std::optional<ViewRegion> view_region;
};

/// What `apply_mask_selection()` created.
struct MaskSelectionResult {
  std::string resource_code; ///< The new survey part, e.g. "RX-042".
  int64_t type_id = 0;       ///< Its survey type.
  bool type_created = false; ///< True when the type was created here.
  uint32_t label_id = 0;     ///< Semantic class (CloudL value, >= 1).
  bool label_created = false;
  uint32_t instance_id = 0; ///< New instance (CloudL value, >= 1).
  std::string instance_guid;
  int point_count = 0;
  /// Instances that gave up points to the new one, with their new counts.
  std::vector<std::pair<uint32_t, int>> shrunk_instances;
  /// The new part's view (frame_id + region); nullopt without a frame.
  std::optional<ResourceView> view;
};

/**
 * @brief Label the selected points as `class_name`, make them one new
 *        instance, and file a survey part for it — in one transaction.
 *
 * - Semantic class: the id whose `label_definitions(semantic_cloud)` name is
 *   exactly `class_name`, else a new id (one past the largest used). The
 *   semantic cloud is created all-zero (cloud-sized) when absent.
 * - Instance: a new id one past the largest used, assigned to the selected
 *   points (taking them from whatever instance held them), an `instances` row
 *   with a fresh GUID, and an `SM<cls>-<id> (<n>p)` label definition. The
 *   instances that lost points get their row and definition counts updated;
 *   one left with no points keeps its row (its material link and survey part
 *   stay intact) and is reported with a `warn`.
 * - Survey part: linked to the new instance (so `sync_survey` will not add a
 *   second one), in the type chosen per `MaskSelectionOptions::type_id`.
 * - View: with `frame_id >= 0` naming an existing frame, the part's view
 *   (see MaskSelectionOptions::view_region); an unknown frame is logged at
 *   warn and the part gets none.
 * - A `pipeline_log` entry (`segment_resource`) records the parameters.
 *
 * Existing instance rows are never deleted and re-inserted, so material links
 * (`instance_materials` cascades on that delete) survive. Rewritten clouds
 * keep the `stage`/`parameters` they were produced with; a cloud created here
 * is saved with stage `segment_resource`. All reads happen inside the write
 * transaction (BEGIN IMMEDIATE), so a concurrent writer cannot be overwritten
 * from stale copies.
 *
 * @throws std::invalid_argument for an empty/blank `class_name`, an index
 *         out of range, or an invalid `opts.view_region`.
 * @throws MaskSelectionError for an empty selection, a missing base cloud, or
 *         a label cloud whose size differs from the base cloud's.
 * @throws MaskSelectionError when `opts.type_id` names a rejected type.
 * @throws std::out_of_range when `opts.type_id` names no survey type.
 */
MaskSelectionResult
apply_mask_selection(ProjectDB &db, const std::vector<std::size_t> &indices,
                     const std::string &class_name,
                     const MaskSelectionOptions &opts = {});

} // namespace reusex::core
