// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

/// @file
/// Floor sub-levels within one storey (split levels, reusex-546 / house1).
///
/// A split-level house has part of a storey's floor raised (house1: the
/// north block 0.75 m above the south block and the wing). detect_storeys()
/// keeps such a step inside ONE storey -- it is below min_storey_height_m,
/// and the storey's floor is the lowest qualifying level -- which is also how
/// the reference format represents it (one storey; per-space z_floor and
/// per-wall z_base, docs/design/solid-model-json.md) and how IFC models a
/// split level whose halves share one storey's ceiling structure: an
/// IfcBuildingStorey whose IfcSpace / IfcSlab elements sit at their own
/// elevations. This file finds the raised parts on a storey's plan grid:
///
///  1. Per block (block_m), the floor is the lowest horizontal surface seen
///  there:
///     the lowest height with at least cell_min_points horizontal points
///     (|n_z| > horizontal_min_abs_nz) within cell_bin_m, among those
///     between the storey floor - cell_bin_m and floor + max_step_m.
///  2. Blocks whose floor lies at least min_step_m above the storey floor are
///     clustered by height (bins of cell_bin_m, strongest first, cells within
///     level_tol_m of the cluster's median); a cluster is a sub-level where
///     it holds a seed -- what is left after a morphological opening of
///     open_radius_m -- of at least min_area_m2. A desk, a bench or a
///     kitchen counter top is narrower than the opening, and the floor is
///     usually seen beneath and around it, so it is no level. The level's
///     region is every cell of the cluster connected to such a seed, so a
///     narrow corridor of the raised block stays with it.
///  3. Blocks without floor evidence (walls, cupboards, unseen corners) take
///     the level of the nearest cell that has one, within fill_radius_m;
///     beyond that, the storey floor.
///
/// Only levels above the storey floor are looked for: detect_storeys() takes
/// the lowest level holding peak_min_share of the strongest floor's points
/// as the floor, so a lower part of a split level is the storey floor unless
/// it is a small sunken pit. A storey without a raised part gets a map that is
/// the storey floor in every cell, exactly, so nothing downstream changes.
///
/// Deterministic (STANDARDS §6): the result depends on the points' positions
/// only, not on their order; clusters are visited strongest first, ties by
/// height.

#include <Eigen/Core>

#include <cstdint>
#include <vector>

namespace reusex::geometry {

struct SplitLevelOptions {
  /// A point is horizontal (a floor candidate) above this |n_z|.
  double horizontal_min_abs_nz = 0.8;
  /// A sub-level lies at least this far above the storey floor [m]...
  double min_step_m = 0.3;
  /// ...and at most this far (a step of a storey height is a storey) [m].
  double max_step_m = 1.5;
  /// The floor is measured per block of this edge [m] (a whole number of
  /// grid cells), so that a block holds a few points at scan density (a
  /// 5 cm cell holds about one).
  double block_m = 0.2;
  /// Height bin of the per-block floor and of the level clusters [m].
  double cell_bin_m = 0.05;
  /// Horizontal points a block needs within one bin to have a floor there.
  int cell_min_points = 3;
  /// A cell belongs to a level when its floor is within this of the level's
  /// height [m].
  double level_tol_m = 0.1;
  /// Opening radius of a level's seed [m]: narrower parts (desks, counters,
  /// beds) do not seed a level.
  double open_radius_m = 0.5;
  /// Least seed area of a level [m^2].
  double min_area_m2 = 4.0;
  /// Cells without floor evidence take the level of the nearest cell with
  /// one within this distance [m]; farther, the storey floor.
  double fill_radius_m = 1.0;
};

/// One floor level of a storey, in its level frame.
struct FloorLevel {
  double z = 0.0;       ///< floor height [m]
  double area_m2 = 0.0; ///< blocks where that floor is seen [m^2]
  bool base = false;    ///< the storey's own floor (z = StoreyRecord::z_floor)
};

/// The local floor of every cell of a storey's plan grid.
struct FloorLevelMap {
  /// Per cell (row-major, width x height of the grid), the floor height of
  /// the level the cell belongs to [m]; the storey floor where there is no
  /// sub-level.
  std::vector<float> floor_z;
  /// The storey floor first (base), then the sub-levels by height.
  std::vector<FloorLevel> levels;
  int evidence_cells = 0; ///< blocks with a floor of their own
  int filled_cells = 0;   ///< blocks given the level of a neighbour

  /// More than the storey's own floor.
  [[nodiscard]] bool split() const { return levels.size() > 1; }
};

/// The floor levels of one storey on its plan grid: cell (c, r) covers level
/// x in [col0 + c, col0 + c + 1) * cell_size and likewise y with row0.
///
/// @param points  Level-frame positions of the storey's points.
/// @param abs_nz  |n_z| in the level frame per point (aligned with
///                @p points).
/// @param use     Optional per-point mask (aligned with @p points; empty =
///                all points count).
/// @param z_floor The storey's floor (StoreyRecord::z_floor).
/// @throws std::invalid_argument when the sizes disagree or the grid is
///         empty.
FloorLevelMap floor_level_map(const std::vector<Eigen::Vector3d> &points,
                              const std::vector<double> &abs_nz,
                              const std::vector<uint8_t> &use, double z_floor,
                              int col0, int row0, int width, int height,
                              double cell_size,
                              const SplitLevelOptions &options = {});

} // namespace reusex::geometry
