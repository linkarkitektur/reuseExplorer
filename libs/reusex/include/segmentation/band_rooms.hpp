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
/// neighbour unless walls and door cuts close it off (below); the footprint
/// leaks through windows the sensor looked through; sparse band walls
/// (drift) merge rooms where the walls table has no wall either.
///
/// The walls and openings tables, when the project has them, tighten the
/// partition (W4.1 + W5.4, reusex-z1v.6.1 / reusex-z1v.7.4; recommended
/// order walls -> openings -> rooms):
///  - wall graph: every wall of the storey is rasterised into the blocked
///    cells (BandRoomsOptions::use_wall_graph), so a wall the band saw too
///    sparsely (glass, drift) still separates rooms;
///  - doorway cuts: every door and floor-reaching opening of the openings
///    table closes its doorway (BandRoomsOptions::cut_at_doors), so rooms
///    are cut at doors however wide; a free-space component that no core
///    reaches but that is closed off this way (a WC stall) is a room of its
///    own;
///  - closed-off regions (reusex-ccz): a wall end the walls stage joined to
///    a wall the grid leaves out runs on to the wall it points at
///    (BandRoomsOptions::wall_graph_join_gap), and a region the wall graph
///    closes off that no room reached is a room of its own
///    (BandRoomsOptions::graph_enclosed_rooms): a stair hall whose risers
///    cut its band into slivers. house2: the stair hall is a room, room
///    count -2 -> -1, room IoU 0.595 -> 0.638; house1 and NewOffice
///    unchanged.
/// Without them the constriction watershed alone cuts the rooms, as before.
/// Measured with them (reusex-z1v.7.4, room IoU / count error against the
/// reference plans, walls -> openings -> rooms): NewOffice 0.544 / 0 ->
/// 0.694 / +7 (the glass-walled meeting room and the printer room are rooms
/// of their own; the WC stalls are too, which the reference draws as one
/// bathroom), house1 0.695 / -2 -> 0.771 / 0 (entrance hall and WC no longer
/// merge), house2 0.503 / -3 -> 0.507 / -1.
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

  // --- walls / openings tables (W4.1, W5.4) --------------------------------
  /// Rasterise the walls of the walls table (BandRoomsPlan::walls) into the
  /// blocked cells, each over its axis, half its measured thickness wide
  /// (at least wall_graph_half_width), except across its doorways unless
  /// cut_at_doors closes them.
  bool use_wall_graph = true;
  /// Close every door and floor-reaching opening of the openings table
  /// (kind door or opening) with a cut along its host wall's axis, running
  /// door_cut_overlap into the jambs: the rooms either side are separated
  /// at the door, however wide it is.
  bool cut_at_doors = true;
  /// Least half-width of a rasterised wall or door cut [m] (an unknown or
  /// one-sided wall's thickness; also at least 0.75 cells so the line is
  /// closed).
  double wall_graph_half_width = 0.05;
  /// A door cut runs this far past the door's ends along the wall [m].
  double door_cut_overlap = 0.15;
  /// Walls with evidence on less than this share of their face
  /// (WallRecord::seen_share; -1 = not measured, kept) are left out, with
  /// their doorways: a wall the scan barely shows is as likely a cabinet
  /// front or a drift artefact as a partition.
  double wall_graph_min_seen_share = 0.3;
  /// A wall of the graph whose end touches no other wall the grid keeps is
  /// drawn on, up to this far, to the first kept wall its axis meets at
  /// least 30 deg off parallel [m]. The walls stage may have joined that
  /// end to a wall the grid leaves out (wall_graph_min_seen_share: house2's
  /// stair side took the stair hall's west wall in a corner 0.4 m short of
  /// the hall's north wall, reusex-ccz), leaving a gap narrower than any
  /// door between two rooms. 0 disables.
  double wall_graph_join_gap = 0.6;
  /// A region the wall graph and door cuts close off (interior cells not in
  /// `graph`, 4-connected) that no core or coreless component reached is a
  /// room of its own, its free cells one id, when it holds min_room_area, is
  /// at least 2 x coreless_min_clearance wide between graph cells and opens
  /// onto the exterior on at most coreless_max_open_share of its boundary.
  /// A stair's risers cut its hall's band into slivers too narrow for a
  /// core and too small for a coreless room (house2, reusex-ccz); the walls
  /// stage still closes the hall off. false disables.
  bool graph_enclosed_rooms = true;
  /// A free-space component no core reaches (narrower than 2 x
  /// core_radius, closed off by walls and door cuts: a WC stall) is a room
  /// of its own when it is at least 2 x this wide somewhere and holds
  /// min_room_area [m]; 0 disables (it stays unlabeled, as before)...
  double coreless_min_clearance = 0.3;
  /// ...and when at most this share of its boundary opens onto the
  /// exterior (not the interior): a stall is closed by walls and its door
  /// cut, a footprint leak beyond a window that the wall graph cut off
  /// from its room is mostly open.
  double coreless_max_open_share = 0.25;
  /// Pockets (reusex-3r1): a region of at most pocket_max_area [m^2] that
  /// meets one neighbouring region through free space along at least
  /// pocket_min_contact_share of its boundary joins that neighbour (the
  /// larger one). Furniture or a column narrows the free space to less than
  /// 2 x core_radius, so the watershed seeds a second core in a corner of a
  /// room; a room behind a door meets its neighbour through the door only
  /// (a 0.8 m door of a 1.7 x 1.8 m WC is ~12 % of its boundary). Measured
  /// on NewOffice: a 1.38 m^2 corner of a meeting room, 44 %; the doors
  /// between rooms there and on house1, 3-12 %. 0 disables.
  double pocket_max_area = 4.0;
  double pocket_min_contact_share = 0.2;

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
  /// Cells of the rasterised wall graph and door cuts
  /// (rasterise_wall_graph()); blocked like `wall` but not dilated. Empty or
  /// all 0 without walls / openings tables.
  std::vector<uint8_t> graph;

  /// Outputs of partition_room_grid():
  /// footprint closed and with every hole the exterior flood cannot reach
  /// filled (furniture shadows, wall cells inside the building).
  std::vector<uint8_t> interior;
  /// Free-space distance [m] (0 on walls and outside).
  std::vector<float> clearance;
  /// Room id per cell, 0 = none (wall, exterior, dropped region).
  std::vector<int32_t> labels;

  int cores = 0;       ///< cores seeded (after the min_core_area filter)
  int coreless = 0;    ///< rooms from components no core reached
  int enclosed = 0;    ///< rooms from regions the wall graph closes off
  int graph_walls = 0; ///< walls rasterised (rasterise_wall_graph())
  int door_cuts = 0;   ///< doorways closed (rasterise_wall_graph())
  int joined_ends =
      0; ///< wall ends drawn on to a wall (rasterise_wall_graph())
  int merged_pockets = 0;     ///< regions merged as pockets (reusex-3r1)
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

/// The 2D part: from `wall`, `graph` and `footprint`, fill `interior`,
/// `clearance` and `labels`. Room ids are first_label, first_label + 1, ...
/// in the raster order of their cores, then of the coreless rooms
/// (BandRoomsOptions::coreless_min_clearance). Returns the number of rooms.
int partition_room_grid(RoomGrid &grid, const BandRoomsOptions &options,
                        int first_label = 1);

/// The walls and openings tables of a project, in their storeys' level
/// frames (rows of the walls and openings stages).
struct BandRoomsPlan {
  std::vector<core::WallRecord> walls;
  std::vector<core::OpeningRecord> openings;
};

/// Rasterise the walls of @p plan standing on the grid's storey
/// (WallRecord::storey == RoomGrid::storey_idx) into `grid.graph` (resized
/// to the grid, OR-ed): with use_wall_graph every wall over its axis
/// except its doorways, with cut_at_doors every doorway (door or opening,
/// by OpeningRecord::wall = WallRecord::guid) over its span plus
/// door_cut_overlap. With use_wall_graph, a wall end that touches no other
/// kept wall runs on up to wall_graph_join_gap to the first kept wall it
/// meets. Sets `graph_walls`, `door_cuts` and `joined_ends`; returns the
/// number of cells set. Deterministic: depends on the records, not their
/// order.
size_t rasterise_wall_graph(RoomGrid &grid, const BandRoomsPlan &plan,
                            const BandRoomsOptions &options);

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
/// storeys. @p plan, when set, is rasterised into each storey's grid
/// (rasterise_wall_graph()) before the partition.
auto segment_band_rooms(const Cloud &cloud, const CloudN &normals,
                        const std::vector<core::StoreyRecord> &storeys,
                        const BandRoomsOptions &options = {},
                        const pcl::Indices *evidence = nullptr,
                        const BandRoomsPlan *plan = nullptr) -> BandRoomsResult;

} // namespace reusex::geometry
