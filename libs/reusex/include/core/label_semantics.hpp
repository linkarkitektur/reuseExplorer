// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <algorithm>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>

/// @file label_semantics.hpp
/// @brief Single source of truth for label encodings across the pipeline.
///
/// Three layers encode labels differently (see docs/STANDARDS.md §3.1):
///
/// - segmentation_images storage (PNG): `CV_16U`, `0` = background, a class
///   is stored as `api + 1` (`1..65535`, so API class <= 65534).
/// - ProjectDB segmentation API: `CV_32S`, `-1` = background, classes
///   `0..N-1` (the annotate prompt order).
/// - In-memory point labels (pcl::Label / CloudL): `uint32_t`, `0` =
///   unlabeled, classes `1..N` (= API class + 1). Semantic label clouds carry
///   `"label_encoding": "api_plus_one"` in their parameters JSON and key their
///   label_definitions by point label.
///
/// Conversions between layers MUST go through the helpers here; no inline
/// offset arithmetic (`label + 1`, `label - 1`) or raw casts are allowed at
/// call sites. Writing a label ≥ 65535 to storage throws instead of wrapping.

namespace reusex::core {

/// Point-cloud convention: `0` means "unlabeled" in every CloudL (labels,
/// planes, rooms, instances). Valid point labels start at `1`.
inline constexpr uint32_t kUnlabeled = 0;

/// ProjectDB segmentation API convention: `-1` marks background/unlabeled in
/// the CV_32S label images returned by ProjectDB.
inline constexpr int kBackgroundApi = -1;

/// Largest label that can be stored in the CV_16U PNG layer. Storage encodes
/// `label + 1`, so the maximum encodable value before wrapping 16 bits is
/// `65535 - 1 = 65534`.
inline constexpr uint16_t kMaxStorableLabel = 65534;

/// @return true if @p label denotes a valid (non-unlabeled) point label.
inline constexpr bool is_valid_label(uint32_t label) noexcept {
  return label != kUnlabeled;
}

/// @return true if @p label is outside the point-label contract: above
/// INT32_MAX, i.e. a negative API label (typically -1, the background or an
/// uninitialised room) that reached a uint32 point cloud and wrapped to
/// 0xFFFFFFFF (STANDARDS §3.1). Such a value is neither unlabeled nor a class.
inline constexpr bool is_out_of_contract_label(uint32_t label) noexcept {
  return label > 0x7FFFFFFFu;
}

/// Convert a point label to a zero-based vector index, replacing the
/// error-prone bare `label - 1` idiom used to index parallel arrays such as
/// `plane_normals`.
///
/// @throws std::out_of_range if @p label is `kUnlabeled` (0), which has no
/// associated index.
inline size_t label_to_index(uint32_t label) {
  if (!is_valid_label(label))
    throw std::out_of_range(
        "label_to_index: label 0 is unlabeled and has no index");
  return static_cast<size_t>(label) - 1;
}

// ── Scalar conversions ────────────────────────────────────────────────────

/// API (CV_32S, -1 = background) → point label (uint32_t, 0 = unlabeled).
///
/// Background (`kBackgroundApi`) maps to `kUnlabeled`; a class id `c >= 0`
/// maps to `c + 1`. API class 0 (the first prompt class) is a real class and
/// must not collide with `kUnlabeled` (STANDARDS §3.1, decision D8).
///
/// @throws std::out_of_range if @p api_label is negative but not the
/// background sentinel.
inline uint32_t api_to_point_label(int api_label) {
  if (api_label == kBackgroundApi)
    return kUnlabeled;
  if (api_label < 0)
    throw std::out_of_range("api_to_point_label: negative label " +
                            std::to_string(api_label) +
                            " is not the background sentinel (-1)");
  return static_cast<uint32_t>(api_label) + 1u;
}

/// Point label (uint32_t, 0 = unlabeled) → API (CV_32S, -1 = background).
/// Inverse of api_to_point_label(): `kUnlabeled` maps to `kBackgroundApi`,
/// point label `p >= 1` to API class `p - 1`.
///
/// @throws std::out_of_range if @p point_label is out of contract
/// (is_out_of_contract_label(), e.g. a wrapped -1).
inline int point_label_to_api(uint32_t point_label) {
  if (point_label == kUnlabeled)
    return kBackgroundApi;
  if (is_out_of_contract_label(point_label))
    throw std::out_of_range("point_label_to_api: label " +
                            std::to_string(point_label) +
                            " is outside the point-label contract");
  return static_cast<int>(point_label - 1u);
}

/// Key in a semantic label cloud's `parameters` JSON that records how its
/// point labels relate to API class ids. Every writer of a semantic CloudL
/// (one whose labels come from segmentation images / annotate classes) sets
/// it to kLabelEncodingApiPlusOne; the schema v29 migration shifts stored
/// semantic clouds lacking it. Structural label clouds (planes, rooms,
/// instances) carry their own 1-based ids and never set it.
inline constexpr const char *kLabelEncodingKey = "label_encoding";

/// Value of kLabelEncodingKey: point label = API class + 1, label
/// definitions keyed by point label.
inline constexpr const char *kLabelEncodingApiPlusOne = "api_plus_one";

/// API label (CV_32S, -1 = background) → storage value (CV_16U, 0 = bg,
/// `label + 1`).
///
/// @throws std::out_of_range if @p api_label < -1, or if @p api_label exceeds
/// kMaxStorableLabel (would wrap the 16-bit storage layer).
inline uint16_t api_to_storage(int api_label) {
  if (api_label < kBackgroundApi)
    throw std::out_of_range("api_to_storage: label " +
                            std::to_string(api_label) + " is below -1");
  if (api_label > static_cast<int>(kMaxStorableLabel))
    throw std::out_of_range(
        "api_to_storage: label " + std::to_string(api_label) +
        " exceeds maximum storable label " +
        std::to_string(static_cast<int>(kMaxStorableLabel)) +
        " (would wrap CV_16U storage)");
  return static_cast<uint16_t>(api_label + 1);
}

/// Storage value (CV_16U, 0 = bg, `label + 1`) → API label (CV_32S,
/// -1 = background).
inline int storage_to_api(uint16_t storage_value) {
  return static_cast<int>(storage_value) - 1;
}

// ── Whole-cv::Mat converters ──────────────────────────────────────────────

/// Convert a storage label image (CV_16U, 0 = bg) to an API label image
/// (CV_32S, -1 = background) by subtracting the +1 offset.
///
/// @throws std::invalid_argument if @p storage is not CV_16U (empty is
/// passed through as empty).
inline cv::Mat storage_mat_to_api(const cv::Mat &storage) {
  if (storage.empty())
    return cv::Mat();
  if (storage.type() != CV_16UC1)
    throw std::invalid_argument(
        "storage_mat_to_api: expected CV_16UC1 storage image");
  cv::Mat api;
  storage.convertTo(api, CV_32S);
  api -= 1;
  return api;
}

/// Convert an API label image (CV_32S, -1 = background) to a storage label
/// image (CV_16U, 0 = bg) by adding the +1 offset.
///
/// @throws std::invalid_argument if @p api is not CV_32S.
/// @throws std::out_of_range if any label is < -1 or > kMaxStorableLabel
/// (would wrap the CV_16U storage layer).
inline cv::Mat api_mat_to_storage(const cv::Mat &api) {
  if (api.empty())
    throw std::invalid_argument("api_mat_to_storage: empty label image");
  if (api.type() != CV_32SC1)
    throw std::invalid_argument(
        "api_mat_to_storage: expected CV_32SC1 API image");

  double min_val = 0.0;
  double max_val = 0.0;
  cv::minMaxLoc(api, &min_val, &max_val);
  if (min_val < static_cast<double>(kBackgroundApi))
    throw std::out_of_range("api_mat_to_storage: label " +
                            std::to_string(static_cast<int>(min_val)) +
                            " is below -1");
  if (max_val > static_cast<double>(kMaxStorableLabel))
    throw std::out_of_range(
        "api_mat_to_storage: label " +
        std::to_string(static_cast<int>(max_val)) +
        " exceeds maximum storable label " +
        std::to_string(static_cast<int>(kMaxStorableLabel)) +
        " (would wrap CV_16U storage)");

  cv::Mat offset = api + 1;
  cv::Mat storage;
  offset.convertTo(storage, CV_16U);
  return storage;
}

// ── Voting ────────────────────────────────────────────────────────────────

/// The label that wins a vote, given (point label, count) pairs.
///
/// The one vote rule of the semantic stages (`rux create project` across
/// frames, `rux create clouds` inside a voxel): `kUnlabeled` never votes, so
/// background never overwrites a class; entries with the same label are
/// summed; the highest total wins and an exact tie goes to the lowest label
/// (STANDARDS §6: the result does not depend on the order of @p counts).
/// Entries with a count <= 0 are ignored.
///
/// @return the winning label, or `kUnlabeled` when no class voted.
inline uint32_t
majority_label(const std::vector<std::pair<uint32_t, int>> &counts) {
  std::vector<std::pair<uint32_t, long long>> totals;
  totals.reserve(counts.size());
  for (const auto &[label, count] : counts) {
    if (label == kUnlabeled || count <= 0)
      continue;
    totals.emplace_back(label, count);
  }
  std::sort(totals.begin(), totals.end());

  uint32_t best_label = kUnlabeled;
  long long best_total = 0;
  for (size_t i = 0; i < totals.size();) {
    const uint32_t label = totals[i].first;
    long long total = 0;
    for (; i < totals.size() && totals[i].first == label; ++i)
      total += totals[i].second;
    // Ascending label order + strict '>' keeps the lowest label on a tie.
    if (total > best_total) {
      best_total = total;
      best_label = label;
    }
  }
  return best_label;
}

} // namespace reusex::core
