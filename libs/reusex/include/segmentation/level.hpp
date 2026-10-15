// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

/// @file
/// Storey detection and the measured vertical (solid-model workstream, W1.1).
///
/// From the plane segmentation of one scan: group the horizontal planes into
/// surfaces by height along the vertical, fit each storey's floor and ceiling
/// by iterated least squares on its surface's own points, take the vertical
/// as the normalised bisector of their normals (the sum of every fitted
/// floor/ceiling normal, so for one storey exactly the bisector), and express
/// each storey's floor and ceiling in a level frame whose origin is the
/// storey-0 floor (`geometry/level_frame.hpp`).
///
/// Floor or ceiling: when the stored plane normals face their sensors
/// (docs/CONTRACTS.md, `create clouds`; reusex-0pi), an up-facing plane was
/// seen from above (a floor, a table) and a down-facing one from below (a
/// ceiling). The storey walk then takes the floor over a slab as the
/// strongest up-facing surface above the ceiling below it, however few
/// points it has against the ground floor. Normals from an older cloud all
/// face the world origin; the sign is then ignored and the surfaces are read
/// by height alone (StoreysResult::normal_sign_used).
///
/// Drift: pose drift can leave the storeys of one scan rotated against each
/// other. With LevelOptions::per_storey_frame each storey whose own
/// floor/ceiling bisector is more than storey_frame_tol_deg off the shared
/// vertical gets its own level frame (rotation from its own vertical), placed
/// so that its floor centroid keeps its shared-frame level coordinates: the
/// floor stays at z_floor (the floor centroid's height above the storey-0
/// floor centroid along the shared vertical), level x/y agree with the
/// shared frame's there, and level (0, 0, z_floor) stays a world point at
/// that storey's floor height for GUID matching and the solid-model
/// benchmark. z_ceiling is measured on the frame's vertical through the
/// floor centroid.
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
  /// A horizontal plane more than this off the measured vertical (a sloped
  /// ceiling, a ramp) is not a floor/ceiling candidate [deg].
  double surface_max_tilt_deg = 3.0;
  double inlier_m = 0.02;      ///< iterated LSQ inlier band [m]
  int max_fit_iterations = 10; ///< LSQ refits per surface (bounded)
  /// Height bin [m]: a horizontal plane within two bins of a stronger
  /// plane's height (and, with the normal sign in use, facing the same way)
  /// joins that plane's surface.
  double hist_bin_m = 0.05;
  /// A surface counts as a level when it holds at least this share of the
  /// points of the strongest surface (of the same facing, with the normal
  /// sign in use). A storey's ceiling needs this share of its floor's
  /// points; the floor over a slab needs none.
  double peak_min_share = 0.25;
  /// Surfaces closer than this along the vertical are one level (a slab:
  /// ceiling_i below, floor_{i+1} above) [m].
  double slab_merge_m = 0.6;
  /// Consecutive levels closer than this are not a storey [m].
  double min_storey_height_m = 2.0;
  double default_height_m = 2.6; ///< used when no ceiling level exists [m]
  /// Read the sign of the stored plane normals as floor (up) / ceiling
  /// (down) when it carries that information (see the file comment).
  bool use_normal_sign = true;
  /// Give a storey its own level frame when its vertical disagrees with the
  /// shared one by more than storey_frame_tol_deg (drifted multi-storey
  /// scans). Off: every row carries the one shared frame. On by default:
  /// on house2 (two storeys 1.0 deg apart) it cut the point-weighted tilt
  /// of the horizontal planes against their storey's vertical from
  /// 0.75 / 0.60 deg to 0.47 / 0.48 deg (reusex-z1v.3.5); a scan whose
  /// storeys agree keeps one frame.
  bool per_storey_frame = true;
  double storey_frame_tol_deg = 0.25; ///< [deg], see per_storey_frame
};

struct StoreysResult {
  /// idx ascending; z_floor / z_ceiling in the row's own level_transform
  /// (storey-0 floor at z = 0). Rows share one level_transform unless
  /// per_storey_frame gave a drifted storey its own. Never empty.
  std::vector<core::StoreyRecord> storeys;
  std::array<double, 3> vertical{0, 0, 1}; ///< shared world up as measured
  int horizontal_planes = 0; ///< planes with |n.z| > horizontal_cos
  int levels = 0;            ///< levels found (slab pairs count once)
  /// The plane normals' sign told floors from ceilings (see file comment).
  bool normal_sign_used = false;
  /// Largest angle between a storey's own vertical and the shared one [deg]
  /// (measured whether or not per_storey_frame applied it).
  double max_storey_tilt_deg = 0.0;
  int own_frames = 0; ///< storeys given their own level frame
};

/// Detect storeys in one scan.
///
/// @param cloud           The scan (per point).
/// @param plane_labels    Plane label per point, index-aligned with @p cloud;
///                        0 = no plane (STANDARDS §3).
/// @param plane_centroids Per plane, indexed by label_to_index(label).
/// @param plane_normals   Per plane, aligned with @p plane_centroids. World
///                        z is assumed roughly up; the sign is read as
///                        floor/ceiling only as described above.
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
