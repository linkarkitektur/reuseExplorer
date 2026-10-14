// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// RTABMap graph links -> pose_graph_edges (spec 2026-10-13 §4.2). Kept free
// of rtabmap headers (they stay confined to io/rtabmap.cpp), so the mapping
// is unit-testable without an RTABMap database.

#include <reusex/core/ProjectDB.hpp>

#include <array>
#include <map>
#include <optional>

namespace reusex::io {

/// Mirrors rtabmap::Link::Type (rtabmap 0.23, Link.h); io/rtabmap.cpp
/// static_asserts every value.
namespace rtabmap_link_type {
inline constexpr int neighbor = 0;
inline constexpr int global_closure = 1;
inline constexpr int local_space_closure = 2;
inline constexpr int local_time_closure = 3;
inline constexpr int user_closure = 4;
inline constexpr int virtual_closure = 5;
inline constexpr int neighbor_merged = 6;
inline constexpr int pose_prior = 7;
inline constexpr int landmark = 8;
inline constexpr int gravity = 9;
} // namespace rtabmap_link_type

struct RtabmapLink {
  int from = 0; ///< RTABMap node id (before the import's id offset)
  int to = 0;
  int type = 0; ///< rtabmap_link_type
  /// pose(to) = pose(from) * transform, base frame, row-major 4x4.
  std::array<double, 16> transform{};
  /// RTABMap order (x, y, z, roll, pitch, yaw), row-major 6x6.
  std::array<double, 36> information{};
};

enum class RtabmapLinkSkip {
  none,
  unsupported_type,      ///< virtual / landmark / gravity / pose prior / other
  self_link,             ///< from == to
  endpoint_not_imported, ///< a node without a colour image was not imported
  invalid_transform,     ///< null or non-rigid link transform
};

struct RtabmapLinkConversion {
  std::optional<ProjectDB::PoseGraphEdge> edge;
  RtabmapLinkSkip skip = RtabmapLinkSkip::none;
};

/// RTABMap's 3x4 row-major float pose -> row-major 4x4 doubles. The pose
/// write path and the link path share it, so both use one convention.
std::array<double, 16> rtabmap_transform_to_array(const float *data3x4);

/// (x, y, z, roll, pitch, yaw) -> GTSAM (rx, ry, rz, tx, ty, tz).
std::array<double, 36>
rtabmap_information_to_gtsam(const std::array<double, 36> &rtabmap_order);

/// Map one link. Neighbor / NeighborMerged -> "odometry"; Global /
/// LocalSpace / LocalTime / User closures -> "loop_closure"; everything
/// else is skipped. Node ids get @p id_offset added. @p imported_poses holds
/// the stored transform of every imported node, keyed by RTABMap id; it
/// feeds seed_disagreement_*. The direction is kept as given.
RtabmapLinkConversion convert_rtabmap_link(
    const RtabmapLink &link, int id_offset,
    const std::map<int, std::array<double, 16>> &imported_poses);

} // namespace reusex::io
