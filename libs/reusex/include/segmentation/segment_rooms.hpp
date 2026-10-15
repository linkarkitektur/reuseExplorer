// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "reusex/core/logging.hpp"
#include "reusex/core/processing_observer.hpp"
#include "reusex/types/point_types.hpp"
#include "reusex/utils/fmt_formatter.hpp"

#include <pcl/community_clustering.hpp>

#include <fmt/format.h>

#include <pcl/common/pca.h>
#include <pcl/correspondence.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/io/auto_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

#include <atomic>

namespace reusex::geometry {
struct SegmentRoomsOptions {
  IndicesConstPtr filter = nullptr; // Optional filter to limit processing

  float grid_size = 0.5F;
  float resolution = 1.0F;
  float beta = 0.01F;
  // Finite Leiden iteration bound. STANDARDS §5 forbids unbounded loops in
  // long-running stages; the previous -1 ("until convergence") could spin
  // indefinitely on pathological graphs. 100 iterations is well past the
  // point where Leiden modularity plateaus for room-scale graphs.
  int max_iter = 100;

  /// Minimum room size, as the plane surface [m^2] its sample points cover
  /// (one sample per grid_size^2). Leiden makes every visibility-graph vertex
  /// that sees nothing — a point under a desk, on an exterior face — a
  /// community of its own; communities below this size are not rooms. Their
  /// samples join the room they see most, or, seeing none, take their label
  /// from nearby points like the unsampled ones (reusex-3vc). The default,
  /// 5 m^2, is a third of the floor, ceiling and walls of a 1 x 1.5 m WC; 0
  /// keeps every community.
  float min_room_area = 5.0F;

  // --- Label propagation to non-sampled points (distance-bounded k-NN) ---
  // Room clustering runs on a uniformly sampled subset; the remaining points
  // inherit a room label from their sampled neighbours. A distance-bounded
  // k-NN majority vote replaces the old unchecked 1-NN so that a single
  // misclassified seed cannot smear across a whole region, and points with no
  // labelled neighbour inside the radius stay kUnlabeled (0) instead of
  // silently copying a far-away label.
  int propagate_k = 5;               ///< Neighbours polled per missing point.
  float propagate_max_radius = 0.5F; ///< Max search radius (meters); no label
                                     ///< beyond this stays unlabeled.

  // Optional cancellation flag. Caller retains ownership and must keep this
  // alive for the full duration of the segment_rooms(...) call.
  const std::atomic_bool *cancel_token = nullptr;
};

/// Propagate room labels from a labelled subset to a set of missing points
/// using a distance-bounded k-NN majority vote.
///
/// For each index in @p missing_indices the function polls up to @p k nearest
/// neighbours drawn from @p sampled_indices, discards any farther than
/// @p max_radius, and assigns the majority room label among the remainder.
/// Ties break deterministically towards the smallest room label. Points with
/// no labelled neighbour inside the radius keep their current label (expected
/// to be reusex::core::kUnlabeled == 0) and are counted.
///
/// @param cloud           Full point cloud (positions used for the search).
/// @param labels          In/out labels; sampled_indices must already hold the
///                        seed room labels, missing points are written here.
/// @param sampled_indices Indices whose labels act as propagation seeds.
/// @param missing_indices Indices to assign a label to.
/// @param k               Neighbours polled per missing point (clamped to >=1).
/// @param max_radius      Maximum neighbour distance in meters.
/// @return Number of missing points that remained unlabeled.
auto propagate_room_labels(CloudConstPtr cloud, CloudLPtr labels,
                           IndicesConstPtr sampled_indices,
                           IndicesConstPtr missing_indices, int k,
                           float max_radius) -> size_t;

auto segment_rooms_impl(CloudConstPtr cloud, CloudNConstPtr normals,
                        CloudLConstPtr planes,
                        const SegmentRoomsOptions &options) -> CloudLPtr;
auto segment_rooms(CloudConstPtr cloud, CloudNConstPtr normals,
                   CloudLConstPtr planes,
                   const SegmentRoomsOptions &options = SegmentRoomsOptions{})
    -> CloudLPtr;

} // namespace reusex::geometry
