// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

/// @file
/// Openings (doors, windows, plain openings) on wall faces (solid-model WP5,
/// reusex-z1v.7). Sub-steps as free, unit-testable functions:
///
///  1. face_raster(): per wall, an (s, z) raster of 10 cm cells over its
///     face; a cell is open when no wall point lies in the wall's slab there
///     AND nothing stands within OpeningOptions::front_m in front of the face
///     on a side it was seen (the occluder check: wall missing behind a cabinet
///     is not an opening) (W5.1).
///  2. fit_openings(): morphological opening of the cells that are not
///     solid, connected components, a rectangle fit per component, kept when
///     big, full and seen-through enough and framed by wall on both sides
///     (W5.1).
///
/// Everything is in the level frame of the wall's storey
/// (core::StoreyRecord::level_transform): z up, s along the wall axis from
/// (x0, y0), t across it (+t = the axis direction turned +90 deg).
/// Inputs are records and clouds, never segmentation types (STANDARDS §1).
///
/// Deterministic (STANDARDS §6): the raster is anchored at the wall's start
/// and the storey floor, cells and components are visited in index order,
/// every iteration is bounded.

#include "reusex/core/solid_model.hpp"

#include <Eigen/Core>

#include <cstdint>
#include <vector>

namespace reusex::geometry {

/// Every tunable of the openings stage (STANDARDS §4).
struct OpeningOptions {
  // --- W5.1 face raster ---------------------------------------------------
  /// Side of the (s, z) cells [m]; s is anchored at the wall's start, z at
  /// the storey floor + floor_clear_m.
  double cell_m = 0.10;
  /// The wall's slab is |t| <= thickness / 2 + this [m].
  double slab_margin_m = 0.05;
  /// Half-thickness used for a wall whose thickness is unknown (<= 0) [m].
  double unknown_half_thickness_m = 0.10;
  /// The occluder check looks this far in front of each face [m]: a point
  /// there (beyond the slab) hides the wall behind it, so the cell is
  /// occluded, not open.
  double front_m = 1.2;
  /// Points within this of the storey floor / ceiling are neither wall nor
  /// occluder (the floor runs on through a doorway, the ceiling over every
  /// face); the raster spans the storey height between them [m].
  double floor_clear_m = 0.05;
  double ceiling_clear_m = 0.10;
  /// A cell is solid with at least this many slab points.
  int min_wall_points = 1;
  /// A cell is occluded with at least this many points in front of it.
  int min_occluder_points = 2;

  // --- W5.1 rectangle fit -------------------------------------------------
  /// Radius of the square structuring element of the morphological opening
  /// of the open cells [cells] (0 = none).
  int morph_radius = 1;
  /// A rectangle is kept when at least this wide and tall [m]...
  double min_width_m = 0.5;
  double min_height_m = 0.4;
  /// ...at least this share of its cells is not solid (open or occluded:
  /// an open cell is seen through, an occluded one unknown -- a desk below
  /// a window, an open door leaf)...
  double min_fill = 0.55;
  /// ...at least this share is open (seen through): a stretch of face that
  /// is only hidden (wall missing behind a cabinet) is no opening...
  double min_open_share = 0.25;
  /// ...and the cells just beyond each side along the wall are solid over at
  /// least this share of its rows, an occluded cell counting half (jambs: a
  /// rectangle running off the end of the wall, or into unseen face, is not
  /// framed).
  double min_jamb_share = 0.3;
  /// A rectangle that reaches the top of the raster but not the bottom and
  /// is less than top_band_max_m tall is a band of face the sensor never saw
  /// (the top of a wall it was too close to look up at), not an opening:
  /// dropped when set. Taller ones are windows whose head meets the ceiling.
  bool drop_top_bands = true;
  double top_band_max_m = 0.8;
  /// A rectangle standing on the floor lower than this is the gap under a
  /// radiator, a desk or a shelf, not an opening [m].
  double min_floor_height_m = 1.2;
};

/// What one cell of a face raster saw.
enum class FaceCellState : std::uint8_t {
  solid,    ///< wall points in the slab
  open,     ///< no wall points, nothing in front on a seen side
  occluded, ///< no wall points, something in front hides the face
};

/// The (s, z) raster of one wall face.
struct FaceRaster {
  double cell = 0.10;  ///< cell side [m]
  double z0 = 0.0;     ///< level-frame z of the bottom edge of row 0 [m]
  double length = 0.0; ///< wall axis length [m]
  double half_thickness = 0.10; ///< the slab's half-width before the margin
  int ns = 0, nz = 0;           ///< columns (along s), rows (along z)
  /// Row-major, cells[iz * ns + is]; column is covers [is, is + 1) * cell.
  std::vector<FaceCellState> cells;
  std::vector<int> wall_points;     ///< slab points per cell
  std::vector<int> occluder_points; ///< points in front, per cell

  [[nodiscard]] FaceCellState at(int is, int iz) const {
    return cells[static_cast<std::size_t>(iz) * static_cast<std::size_t>(ns) +
                 static_cast<std::size_t>(is)];
  }
};

/// The raster of @p wall's face from @p points (level frame of the wall's
/// storey). Columns run from the wall's start (x0, y0) over its length, rows
/// from @p z_floor + floor_clear_m to @p z_ceiling - ceiling_clear_m. A
/// point with 0 <= s < length in that height range counts as wall evidence
/// when |t| <= half thickness + slab_margin_m, and as an occluder when
/// further out but within front_m of the face, on a side the wall was seen
/// from (WallRecord::face_a_points > 0 for +t, face_b_points for -t; both
/// when neither is known).
///
/// @throws std::invalid_argument for a zero-length axis, a non-positive
///         cell, or z_ceiling - ceiling_clear_m <= z_floor + floor_clear_m.
FaceRaster face_raster(const std::vector<Eigen::Vector3d> &points,
                       const core::WallRecord &wall, double z_floor,
                       double z_ceiling, const OpeningOptions &opt = {});

/// A rectangle of open face.
struct OpeningRect {
  double s_start = 0.0, s_end = 0.0; ///< along the axis from (x0, y0) [m]
  double z_sill = 0.0, z_head = 0.0; ///< level frame [m]
  double fill = 0.0;       ///< non-solid (open or occluded) share of cells
  double open_share = 0.0; ///< open share of its cells
  int open_cells = 0;
  /// Solid share (occluded cells counting half) of the column just before
  /// s_start / just after s_end over the rectangle's rows (0 at the wall's
  /// end).
  double jamb_start = 0.0, jamb_end = 0.0;
  bool at_floor = false;   ///< reaches the bottom row (z_sill = the floor)
  bool at_ceiling = false; ///< reaches the top row (z_head = the ceiling)
};

/// The openings of one face (W5.1): the cells that are not solid (open or
/// occluded), morphologically opened
/// (OpeningOptions::morph_radius), split into 4-connected components; per
/// component its bounding box, trimmed edge by edge while an edge row or
/// column is less than half open; kept with min_width_m, min_height_m,
/// min_fill, min_open_share, min_jamb_share and min_floor_height_m (and
/// drop_top_bands / top_band_max_m). A rectangle on the
/// bottom row has its sill on @p z_floor, one on the top row its head on
/// @p z_ceiling. Sorted by s_start, then z_sill.
std::vector<OpeningRect> fit_openings(const FaceRaster &raster, double z_floor,
                                      double z_ceiling,
                                      const OpeningOptions &opt = {});

} // namespace reusex::geometry
