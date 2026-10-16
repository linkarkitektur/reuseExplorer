// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

/// @file
/// Rooms from a clutter-free height band (reusex-rqn.2, W4.1 reusex-z1v.6.1):
/// a per-storey 2D occupancy grid in the storey's level frame, the exterior
/// flooded from the grid border, and the interior partitioned into rooms by a
/// watershed of the free-space distance transform, so rooms are cut where
/// the free space narrows to less than 2 x core_radius (a door, a gap in a
/// glass partition between two mullions).
///
/// Evidence, per storey and cell:
///  - wall: at least wall_min_points points with |n_z| < wall_max_abs_nz
///    between floor + band_bottom and min(floor + band_top, ceiling -
///    ceiling_margin). The band sits above desks, screens and parapets and
///    below the ceiling clutter, so furniture does not cut rooms.
///  - footprint: a point with |n_z| > horizontal_min_abs_nz below floor +
///    floor_max_height (floors, both levels of a split level, desk tops) or
///    above floor + ceiling_min_height (ceilings), closed by footprint_close.
///
/// Measured in docs/research/solid-model-ideas-lab.md §4.1 against the
/// reference plans: room IoU NewOffice 0.579 (Leiden) -> 0.663, house1
/// 0.602 -> 0.632, house2 0.469 -> 0.523. Known limits: a room narrower than
/// 2 x core_radius (a < 1 m WC stall) has no core and is flooded by its
/// neighbour; the footprint leaks through windows the sensor looked through;
/// sparse band walls (drift) merge rooms.
///
/// Determinism (STANDARDS §6): the grid is anchored to the level frame
/// origin (cell (c, r) covers x in [c, c+1) * cell_size), never to a
/// bounding-box minimum; the watershed breaks ties by insertion order over a
/// raster-order seeding; the result does not depend on point order.

#include "reusex/core/solid_model.hpp"
#include "reusex/types/point_types.hpp"

#include <atomic>
#include <cstdint>
#include <vector>

namespace reusex::geometry {

/// The `rooms` stage's `method` parameter: Leiden clustering of the plane
/// visibility graph (segment_rooms.hpp) or this file's band watershed.
inline constexpr const char *kRoomsMethodLeiden = "leiden";
inline constexpr const char *kRoomsMethodBand = "band";
/// Default since the maintainer decision of 2026-10-16 (reusex-icg): band
/// beats Leiden on room IoU on all three measured references. Band needs the
/// storeys table; when the default resolves to band and the project has no
/// storeys rows, the `rooms` stage falls back to Leiden with a warning
/// instead of refusing (see `run_rooms` in `pipeline/stages.cpp`) — an
/// *explicit* `--method band` without storeys still refuses, as before.
inline constexpr const char *kDefaultRoomsMethod = kRoomsMethodBand;

struct BandRoomsOptions {
  /// Grid cell edge [m].
  double cell_size = 0.05;
  /// Wall band, heights above the storey floor [m]. The top is clamped to
  /// ceiling - ceiling_margin.
  double band_bottom = 1.2;
  double band_top = 2.3;
  double ceiling_margin = 0.15;
  /// A wall point's normal is at most this far from horizontal (|n_z|).
  double wall_max_abs_nz = 0.3;
  /// Points a cell needs in the band to block it.
  int wall_min_points = 3;
  /// Footprint: horizontal points (|n_z| above this) ...
  double horizontal_min_abs_nz = 0.8;
  /// ... below floor + floor_max_height (floor, split level, desk tops) ...
  double floor_max_height = 0.9;
  /// ... or above floor + ceiling_min_height (ceiling).
  double ceiling_min_height = 1.9;
  /// Points within this margin below the floor / above the ceiling still
  /// belong to the storey [m].
  double storey_margin = 0.3;
  /// Morphological closing of the footprint (disk radius) [m].
  double footprint_close = 0.4;
  /// Wall cells are dilated by this (disk radius) before the distance
  /// transform [m].
  double wall_dilate = 0.05;
  /// Room cores are cells farther than this from any wall or the exterior
  /// [m]. Rooms are cut where the free space is narrower than twice this:
  /// a door (< 1 m) cuts, a 1 m corridor does not seed a room of its own.
  double core_radius = 0.5;
  /// Cores smaller than this are noise, not rooms [m^2].
  double min_core_area = 0.15;
  /// Regions smaller than this after the watershed are dropped [m^2].
  double min_room_area = 1.0;
  /// Points take the room of the nearest room cell when they lie within the
  /// interior grown by this margin (walls, the outer wall faces) [m]; points
  /// farther out stay unlabeled.
  double assign_margin = 0.2;
  /// Empty cells around the storey's points on every side [m].
  double grid_padding = 1.0;

  /// Optional cancellation flag, checked between storeys. Caller keeps it
  /// alive for the call.
  const std::atomic_bool *cancel_token = nullptr;
};

/// One storey's occupancy grid. Row-major, `width` columns (level x) by
/// `height` rows (level y); cell (c, r) covers level x in
/// [(col0 + c) * cell_size, (col0 + c + 1) * cell_size), likewise y with
/// row0, so the grid is anchored to the level frame origin.
struct RoomGrid {
  int storey_idx = 0;
  double cell_size = 0.05;
  int col0 = 0;
  int row0 = 0;
  int width = 0;
  int height = 0;

  /// Input evidence (1 = set): band wall cells and raw footprint cells.
  std::vector<uint8_t> wall;
  std::vector<uint8_t> footprint;

  /// Outputs of partition_room_grid():
  /// footprint closed and with every hole the exterior flood cannot reach
  /// filled (furniture shadows, wall cells inside the building).
  std::vector<uint8_t> interior;
  /// Free-space distance [m] (0 on walls and outside).
  std::vector<float> clearance;
  /// Room id per cell, 0 = none (wall, exterior, dropped region).
  std::vector<int32_t> labels;

  int cores = 0;              ///< cores seeded (after the min_core_area filter)
  int rooms = 0;              ///< rooms kept
  int dropped_regions = 0;    ///< regions below min_room_area
  double interior_area = 0.0; ///< [m^2]

  [[nodiscard]] size_t index(int col, int row) const {
    return static_cast<size_t>(row) * static_cast<size_t>(width) +
           static_cast<size_t>(col);
  }
  /// An empty grid of @p w x @p h cells (evidence zeroed).
  void resize(int w, int h);
};

/// The 2D part: from `wall` and `footprint`, fill `interior`, `clearance`
/// and `labels`. Room ids are first_label, first_label + 1, ... in the
/// raster order of their cores. Returns the number of rooms.
int partition_room_grid(RoomGrid &grid, const BandRoomsOptions &options,
                        int first_label = 1);

struct BandRoomsResult {
  /// Per point room label (0 = unlabeled), index-aligned with the cloud.
  CloudLPtr labels;
  /// One grid per storey, in storey order.
  std::vector<RoomGrid> grids;
  int rooms = 0;
  size_t unlabeled_points = 0;
};

/// Rooms of every storey. @p normals are the per-point normals ("normals"
/// cloud, world frame), index-aligned with @p cloud. @p storeys give each
/// storey's level frame and floor / ceiling height (rows of the storeys
/// stage). @p evidence, when set, limits the points that rasterise
/// evidence; every point is still labelled. A point in the height range of
/// several storeys takes the lowest one's room. Room ids run on across
/// storeys.
auto segment_band_rooms(const Cloud &cloud, const CloudN &normals,
                        const std::vector<core::StoreyRecord> &storeys,
                        const BandRoomsOptions &options = {},
                        const pcl::Indices *evidence = nullptr)
    -> BandRoomsResult;

} // namespace reusex::geometry
