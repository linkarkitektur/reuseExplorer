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
///  3. merge_mullions(), classify_opening(): one opening split by a mullion
///     is merged; door / window / floor-to-ceiling opening by sill and
///     height (W5.2).
///  4. detect_openings(): 1-3 on every wall of a storey plus the evidence --
///     semantic labels and instances on the rectangle, glass images, the
///     camera path walking through it (trajectory_crossings()), the open
///     door leaf (door_leaf()) for state and swing -- as core records
///     (W5.2).
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
#include <functional>
#include <string>
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

  /// Measure the floor and ceiling in front of each wall (face_levels())
  /// instead of taking the storey's: a scan whose rooms drifted apart
  /// vertically has its floor tens of centimetres off the storey's level in
  /// some rooms (house1: 0.65 m), which would put every door's sill there.
  bool local_levels = true;
  /// A rectangle whose sill is at most this above the floor stands on it
  /// (only a threshold, or floor noise, is below it) [m].
  double sill_snap_m = 0.15;
  /// face_levels() looks this far above the storey floor / below its
  /// ceiling (and half as far beyond them) for the local level [m]...
  double level_search_m = 1.0;
  /// ...and takes the densest 2 cm layer with at least this many points.
  int min_level_points = 50;

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
  // --- W5.2 classification -------------------------------------------------
  /// A door's sill is below this above the floor, its height in
  /// [door_min_height_m, door_max_height_m] [m]...
  double door_max_sill_m = 0.3;
  double door_min_height_m = 1.7;
  double door_max_height_m = 2.4;
  /// ...a low-silled rectangle whose head is within this of the ceiling is a
  /// floor-to-ceiling `opening`; everything else is a `window` [m].
  double full_height_tol_m = 0.15;
  /// Two rectangles of one face at most this far apart along the wall,
  /// whose sills and heads agree within mullion_max_dz_m, are one opening
  /// split by a mullion (or a door leaf edge) [m].
  double mullion_max_gap_m = 0.25;
  double mullion_max_dz_m = 0.15;

  // --- W5.2 evidence -------------------------------------------------------
  /// Points within this beyond the slab count as on the face for the
  /// semantic evidence (a window frame, a door leaf standing in it) [m].
  double evidence_band_m = 0.3;
  /// A semantic class supports an opening when at least this share of the
  /// labelled points on its rectangle carry it...
  double min_class_share = 0.2;
  /// ...and an instance when it has at least this many points there and
  /// that is at least min_class_share of the labelled points.
  int min_instance_points = 20;
  /// Glass (glass_confidence_images) supports a window when at least this
  /// share of its rectangle is suppressed as glass.
  double min_glass_share = 0.3;
  /// Camera path crossings (the operator walked through: a door, research
  /// §4.4): steps longer than this are jumps, not walks [m]...
  double crossing_max_step_m = 1.5;
  /// ...a crossing within this of an opening's extent supports it [m]...
  double crossing_tol_m = 0.25;
  /// ...a rectangle the path crosses whose sill is at most this above the
  /// floor reaches the floor (nobody walks through a window: its lower part
  /// was hidden by a leaf or a chair) and is classified again [m]...
  double crossing_door_max_sill_m = 1.0;
  /// ...and a camera belongs to the storey when its height is within
  /// (z_floor - below, z_ceiling + above) [m].
  double crossing_storey_below_m = 0.5;
  double crossing_storey_above_m = 0.3;

  // --- W5.2 door state -----------------------------------------------------
  /// Leaf points: in the quarter-cylinder a leaf of the door's width sweeps
  /// on either side, at least this far above the sill and below the head [m]
  double leaf_margin_m = 0.2;
  /// A leaf: at least this many points in one leaf_bin_deg angular bin
  /// about a hinge, between leaf_min_radius_share and 1 of the width out.
  int min_leaf_points = 30;
  double leaf_bin_deg = 10.0;
  double leaf_min_radius_share = 0.3;
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
/// when neither is known). A one-sided wall of unknown thickness (thickness
/// <= 0, points on one face only) has its axis on its seen face
/// (reusex-v8p): its slab is taken 2 x unknown_half_thickness_m deep behind
/// that face, as face_levels() and door_leaf() do.
///
/// @throws std::invalid_argument for a zero-length axis, a non-positive
///         cell, or z_ceiling - ceiling_clear_m <= z_floor + floor_clear_m.
FaceRaster face_raster(const std::vector<Eigen::Vector3d> &points,
                       const core::WallRecord &wall, double z_floor,
                       double z_ceiling, const OpeningOptions &opt = {});

/// The floor and ceiling in front of a wall (W5.1).
struct FaceLevels {
  double z_floor = 0.0, z_ceiling = 0.0;
  bool floor_measured = false, ceiling_measured = false;
};

/// The floor and ceiling heights in front of @p wall: of the points of
/// @p points (level frame) beyond its slab but within front_m of it, over
/// its length, the lowest 2 cm layer within level_search_m above
/// @p z_floor (half that below it) holding min_level_points and 30 % of the
/// densest one, and likewise the highest below @p z_ceiling; the storey's value
/// where none is found (or with OpeningOptions::local_levels off).
FaceLevels face_levels(const std::vector<Eigen::Vector3d> &points,
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

/// Merge rectangles of one face split by a mullion (W5.2): neighbours along
/// the wall at most mullion_max_gap_m apart whose sills and heads agree
/// within mullion_max_dz_m become their union (fill = the cell-weighted
/// mean, the outer jambs, at_floor / at_ceiling of either); so do
/// rectangles stacked over the same stretch (ends within mullion_max_dz_m)
/// at most mullion_max_gap_m apart vertically (a transom); a rectangle
/// lying over half inside a bigger one is dropped. Output sorted by
/// s_start.
std::vector<OpeningRect> merge_mullions(std::vector<OpeningRect> rects,
                                        const OpeningOptions &opt = {});

/// The kind of an opening from its sill and head (W5.2): a sill within
/// door_max_sill_m of @p z_floor with the head within full_height_tol_m of
/// @p z_ceiling is a floor-to-ceiling "opening"; with a height in
/// [door_min_height_m, door_max_height_m] a "door"; anything else a
/// "window".
std::string classify_opening(const OpeningRect &rect, double z_floor,
                             double z_ceiling, const OpeningOptions &opt = {});

/// What a semantic class says a point is, for opening evidence.
enum class OpeningClass : std::uint8_t { none, door, window, glass };

/// An instance of the `instances` cloud, for opening evidence.
struct OpeningInstance {
  std::string guid;
  OpeningClass kind = OpeningClass::none;
};

/// The evidence of one storey, in its level frame.
struct OpeningEvidence {
  std::vector<Eigen::Vector3d> points; ///< every point of the storey
  /// Per point (index-aligned, or empty: no semantic labels): what its
  /// semantic class says.
  std::vector<OpeningClass> classes;
  /// Per point (index-aligned, or empty): index into @ref instances, -1 none.
  std::vector<int> instance;
  std::vector<OpeningInstance> instances;
  /// Camera centres in capture order (empty: no trajectory).
  std::vector<Eigen::Vector3d> trajectory;
  /// Share of an opening's rectangle the glass_confidence_images suppress as
  /// glass, -1 when no frame sees it (null: no images).
  std::function<double(const core::WallRecord &, const OpeningRect &)> glass;
};

/// Where the camera path crosses @p wall's axis: s of each crossing, for
/// steps no longer than crossing_max_step_m whose ends both lie in the
/// storey (W5.2; the idea of slam::trajectory_wall_crossings, research
/// §4.4: house1's 16 crossings all lie at reference openings).
std::vector<double>
trajectory_crossings(const std::vector<Eigen::Vector3d> &trajectory,
                     const core::WallRecord &wall, double z_floor,
                     double z_ceiling, const OpeningOptions &opt = {});

/// A door leaf found standing open (door_leaf()).
struct DoorLeaf {
  bool found = false;
  bool hinge_at_start = true; ///< hinged at s_start (else at s_end)
  /// The side it swings to: +1 = +t (the wall's side a, face_a_points),
  /// -1 = -t (side b).
  int side = 0;
  double angle_deg = 0.0; ///< from the wall face, 0..90
  int points = 0;
};

/// The open leaf of door @p rect on @p wall (W5.2): about each jamb and on
/// each side, the points (of @p points, level frame) in the quarter-cylinder
/// of radius = the door's width beyond the slab, between the sill and the
/// head (leaf_margin_m clear of both); their angle about the hinge binned by
/// leaf_bin_deg. The densest bin with min_leaf_points whose points span at
/// least half the radial range (a sheet running out from the hinge) and do
/// not run on past the width (a wall leaving the jamb) is the leaf.
DoorLeaf door_leaf(const std::vector<Eigen::Vector3d> &points,
                   const core::WallRecord &wall, const OpeningRect &rect,
                   const OpeningOptions &opt = {});

/// One opening found on a wall, with its evidence.
struct DetectedOpening {
  /// guid = the natural key "openings/<wall guid>/<s_start rounded 5 cm>";
  /// wall = the host's guid; kind, extent, state, is_external,
  /// source_instance_guid, confidence and metadata (the evidence) filled.
  core::OpeningRecord record;
  OpeningRect rect;
  double geometry_confidence = 0.0;
  double door_share = 0.0, window_share = 0.0, glass_label_share = 0.0;
  double glass_share = -1.0; ///< from the images, -1 none
  int crossings = 0;
  DoorLeaf leaf;
};

/// Steps 1-2 and the classification on every wall of @p storey (W5.1 +
/// W5.2): face raster, rectangles, mullions merged, kind, evidence (semantic
/// classes, the instance with most points on it -> source_instance_guid,
/// glass, camera crossings), door state and swing, is_external = the host
/// wall's (until spaces exist, W4). Confidence: from the geometry alone
/// (fill, jambs; at most 0.7), raised by each piece of evidence that agrees
/// with the kind. Walls of other storeys are skipped.
std::vector<DetectedOpening> detect_openings(
    const OpeningEvidence &evidence, const std::vector<core::WallRecord> &walls,
    const core::StoreyRecord &storey, const OpeningOptions &opt = {});

} // namespace reusex::geometry
