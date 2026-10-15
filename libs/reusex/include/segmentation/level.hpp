// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

/// @file
/// Storey detection and the measured vertical (solid-model workstream, W1.1).
///
/// From the plane segmentation of one scan: fit the floor and ceiling of each
/// storey by iterated least squares, take the vertical as the normalised
/// bisector of their normals (the sum of every fitted floor/ceiling normal, so
/// for one storey exactly the bisector), and express each storey's floor and
/// ceiling in one level frame whose origin is the storey-0 floor
/// (`geometry/level_frame.hpp`). Storeys come from a 1D histogram of
/// horizontal-plane points along the vertical.
///
/// Pure algorithm: no ProjectDB, no logging of results beyond diagnostics.
/// The stage that stores the rows is W1.3.

#include "reusex/core/solid_model.hpp"
#include "reusex/types/point_types.hpp"

#include <array>
#include <vector>

namespace reusex::geometry {

struct LevelOptions {
  double horizontal_cos = 0.9848; ///< |n.z| above this = horizontal (10 deg)
  double inlier_m = 0.02;         ///< iterated LSQ inlier band [m]
  int max_fit_iterations = 10;    ///< LSQ refits per surface (bounded)
  double hist_bin_m = 0.05;       ///< Z histogram bin [m]
  double peak_min_share = 0.25;   ///< a bin is a level if >= this * max bin
  /// Surfaces closer than this along the vertical are one level (a slab:
  /// ceiling_i below, floor_{i+1} above) [m].
  double slab_merge_m = 0.6;
  /// Consecutive levels closer than this are not a storey [m].
  double min_storey_height_m = 2.0;
  double default_height_m = 2.6; ///< used when no ceiling level exists [m]
};

struct StoreysResult {
  /// idx ascending; z_floor / z_ceiling in the shared level frame (storey-0
  /// floor at z = 0); every row carries the same level_transform. Never
  /// empty.
  std::vector<core::StoreyRecord> storeys;
  std::array<double, 3> vertical{0, 0, 1}; ///< world up as measured
  int horizontal_planes = 0; ///< planes with |n.z| > horizontal_cos
  int levels = 0;            ///< levels found (slab pairs count once)
};

/// Detect storeys in one scan.
///
/// @param cloud           The scan (per point).
/// @param plane_labels    Plane label per point, index-aligned with @p cloud;
///                        0 = no plane (STANDARDS §3).
/// @param plane_centroids Per plane, indexed by label_to_index(label).
/// @param plane_normals   Per plane, aligned with @p plane_centroids. Sign is
///                        ignored; world z is assumed roughly up.
/// @throws std::invalid_argument when cloud/planes sizes disagree, a label
///         has no plane entry, or no plane exists.
///
/// A scan with no horizontal plane, one level, or no ceiling still yields one
/// storey row; the reason is logged and recorded in the row's `metadata`.
StoreysResult detect_storeys(const Cloud &cloud, const CloudL &plane_labels,
                             const CloudLoc &plane_centroids,
                             const CloudN &plane_normals,
                             const LevelOptions &opt = {});

} // namespace reusex::geometry
