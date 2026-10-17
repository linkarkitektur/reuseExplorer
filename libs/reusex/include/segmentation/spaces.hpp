// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

/// @file
/// Space polygons from the band-rooms occupancy grid and the wall graph
/// (W4.2, reusex-z1v.6.2).
///
/// segment_band_rooms() (band_rooms.hpp) partitions each storey's grid into
/// room components; this file turns them into polygons that follow the
/// walls instead of the 5 cm raster:
///
///  1. Arrangement: a CGAL Arrangement_2 per storey of every wall's two faces
///     (axis +- thickness / 2; a one-sided wall, thickness -1, has its axis
///     on its seen face, so the axis itself), each run join_extension_m past
///     both ends so corners and tees close, plus an end cap across each
///     thick wall's ends, plus the simplified outline of the storey's
///     interior (RoomGrid::interior), which closes what the wall graph
///     leaves open. Dangling stubs (an extension that hits nothing) are
///     removed.
///  2. Labelling: the furniture pockets each room encloses are filled with
///     the room; then every bounded face counts the room cells whose
///     centres it holds. A face is *pure* to room L when L covers at least
///     pure_min_share of its area -- another room with at least
///     guest_min_share of its cells in it is its *guest* (a meeting room
///     the wall graph does not close off); *empty* when rooms cover at most
///     empty_max_share (a wall body, the courtyard); *mixed* otherwise (the
///     wall graph does not separate two rooms there). A face too small to
///     judge by its cells (below small_face_area, or thinner than one cell)
///     takes its longest neighbour's class, or is empty between two rooms.
///  3. Polygons: room L = (its pure faces minus their guests' cells) U (its
///     cells outside every pure or empty face but those it is a guest in),
///     so where the wall graph closes a room the polygon is the walls'
///     faces, and where it does not (no walls table at all, two rooms in
///     one face) the raster outline of the room fills in. A guest's cells
///     are grown by guest_grow_m to the walls first. The largest piece is
///     the space; holes below min_hole_area are filled; vertices are
///     snapped to 1 um.
///  4. Metrics: area (outer minus holes), perimeter (outer ring), volume
///     (area x storey height), compactness 4 pi A / P^2; kind: corridor when
///     the minimum-area bounding rectangle's aspect exceeds
///     corridor_min_aspect, open_plan when the room is at least
///     open_plan_min_area and encloses at least open_plan_min_holes
///     furniture pockets, else room.
///  5. Boundaries: every polygon edge that lies on a wall face (within
///     boundary_tolerance_m and boundary_max_angle_deg) is a space boundary
///     of that wall, side "a" when the space lies left of the wall's axis
///     seen from (x0, y0) to (x1, y1), "b" right; per (space, wall, side)
///     the s range is the hull of those edges projected on the axis.
///
/// Measured (reusex-z1v.6.2, room IoU / count error against the reference
/// plans; band rooms after walls -> openings, defaults): raster outline of
/// the same rooms NewOffice 0.764 / +1, house1 0.716 / 0, house2 0.580 / 0
/// -> polygonised 0.891 / +1, 0.801 / 0, 0.601 / 0.
///
/// Coordinates are in each storey's level frame, as the grid and the walls.
/// The room id of the grid is the space's room_label (the "rooms" cloud's
/// label when band rooms wrote it). Deterministic (STANDARDS §6): exact
/// constructions (Epeck), faces and rooms visited in a fixed order, rings
/// start at their lexicographically smallest vertex.

#include "reusex/core/solid_model.hpp"
#include "reusex/segmentation/band_rooms.hpp"

#include <vector>

namespace reusex::geometry {

struct SpacesOptions {
  /// Build the arrangement from the walls. Off: every space is its raster
  /// outline (the band-rooms grid polygonised as it is).
  bool use_walls = true;
  /// Insert the wall axes instead of the two faces.
  bool axes_only = false;
  /// Walls whose seen share is below this are left out (as
  /// BandRoomsOptions::wall_graph_min_seen_share; -1 = not measured, kept).
  double wall_min_seen_share = 0.3;
  /// Wall faces run this far past the wall's ends [m], so a corner or a tee
  /// with a detection gap still closes. Longer only splits rooms into more
  /// faces, which merge again; measured (room IoU, NewOffice / house1 /
  /// house2): 0.5 m 0.799 / 0.752 / 0.579, 2 m 0.844 / 0.796 / 0.606, 3 m
  /// 0.849 / 0.809 / 0.590, whole lines (100 m) 0.851 / 0.779 / 0.591.
  double join_extension_m = 2.0;
  /// Also insert the outline of the storey's interior (RoomGrid::interior,
  /// simplified to interior_simplify_m), so that rooms the wall graph does
  /// not close are faces bounded by it rather than the unbounded face.
  bool close_at_interior = true;
  double interior_simplify_m = 0.1;
  /// A face is pure to a room covering this share of its area...
  double pure_min_share = 0.5;
  /// Another room with at least this share of its cells in a pure face is
  /// its guest (a meeting room the wall graph does not close off): it keeps
  /// its raster cells there, and the face's room gets the rest.
  double guest_min_share = 0.5;
  /// A guest's cells are grown by this over unlabelled cells [m], so it
  /// reaches the walls of its host's face.
  double guest_grow_m = 0.15;
  /// A face that rooms cover at most this share of is empty (no space).
  double empty_max_share = 0.1;
  /// A face smaller than this [m^2] (a wall end poking past a corner) or
  /// thinner than one cell is too small to judge by its cells: it takes the
  /// class of the neighbour it shares the longest edge with.
  double small_face_area = 0.05;
  /// Pieces of a space other than its largest one are dropped; a warning
  /// names those above this area [m^2].
  double min_piece_area = 0.5;
  /// Holes smaller than this are filled [m^2].
  double min_hole_area = 0.25;
  /// A polygon edge lies on a wall face when both its ends are within this
  /// of the face line [m]...
  double boundary_tolerance_m = 0.05;
  /// ...it is at most this far from parallel to it [deg]...
  double boundary_max_angle_deg = 5.0;
  /// ...and overlaps the wall's run by at least this [m].
  double boundary_min_length_m = 0.1;
  /// Kind heuristics.
  double corridor_min_aspect = 4.0;
  double open_plan_min_area = 40.0;
  int open_plan_min_holes = 3;
  /// A furniture hole (an unlabelled pocket inside a room's raster) counts
  /// from this area [m^2].
  double open_plan_min_hole_area = 0.1;
};

/// What polygonise_spaces() did on one storey.
struct SpacesStoreyStats {
  int storey_idx = 0;
  int walls = 0;        ///< walls inserted
  int faces = 0;        ///< bounded faces of the arrangement
  int pure_faces = 0;   ///< faces given to one room
  int empty_faces = 0;  ///< faces with no room (wall bodies, outside)
  int mixed_faces = 0;  ///< faces the raster decides
  int sliver_faces = 0; ///< thinner than a cell, classed by a neighbour
  int rooms = 0;        ///< room components in the grid
  int spaces = 0;       ///< spaces produced
  int lost_rooms = 0;   ///< rooms with no polygon left (covered by others)
  /// Share of the spaces' total area that comes from pure faces (the rest
  /// is raster) [0, 1].
  double arrangement_share = 0.0;
};

struct SpacesResult {
  std::vector<core::SpaceRecord> spaces;
  std::vector<core::SpaceBoundaryRecord> boundaries;
  std::vector<SpacesStoreyStats> stats; ///< one per grid, in grid order
};

/// The spaces of every storey from its band-rooms grid (RoomGrid::labels,
/// the output of partition_room_grid()) and the walls standing on it
/// (WallRecord::storey == RoomGrid::storey_idx). @p storeys give z_floor /
/// z_ceiling for the volume (a grid without a matching storey gets volume
/// -1). Space guids are "space-<storey>-<room id>" until the spaces stage
/// (W4.3) gives them their stable keys; boundaries refer to those guids and
/// to WallRecord::guid.
auto polygonise_spaces(const std::vector<RoomGrid> &grids,
                       const std::vector<core::StoreyRecord> &storeys,
                       const std::vector<core::WallRecord> &walls,
                       const SpacesOptions &options = {}) -> SpacesResult;

} // namespace reusex::geometry
