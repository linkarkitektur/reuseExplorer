// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Typed solids from the typed building model (W6.1, reusex-z1v.8.1).
//
// build_solids() turns the storeys, walls, openings and spaces of a
// core::SolidModel into one closed solid per element:
//
//  - a wall is a box in its wall frame (axis, thickness, z_base, height)
//    minus a box per opening (CGAL corefinement, exact predicates), so a door
//    leaves a gap and a window a hole;
//  - a slab is a storey's floor polygon (the union of its spaces and wall
//    footprints, else the hull of its walls) extruded down from the floor by
//    the gap to the storey below (else a default thickness), plus a roof
//    slab over the top storey;
//  - a space is its polygon (with holes) extruded from its own z_floor to
//    its own z_ceiling.
//
// Split levels and differing ceilings (reusex-546): the spaces of a storey
// fall into floor levels (z_floor within level_tol_m of each other; a level
// within that of the storey floor IS the storey floor) and, on the top
// storey, ceiling levels likewise. Each floor level is a slab region of its
// own -- its spaces and the footprints of the walls standing on it, minus
// the regions of the higher levels -- whose top is the level's height and
// whose bottom is the storey's lowest slab bottom, so a raised part's slab
// fills down to the lower floor's slab and the step is closed whether or not
// a wall stands on it (the fill under a raised floor is not observed; the
// scan only sees its top). Roof regions are built per ceiling level the
// same way, the higher level taking the walls between them, each with the
// default thickness on its level. A detected wall (not locked, source
// "detected") runs from the floor level of the lowest space beside it to
// the ceiling level of the highest one (probed along both faces), so a wall
// between two levels covers both and every wall stands on its slab and
// meets its roof; a wall with no space beside it, or a locked / manual one,
// keeps its own z_base / height. A storey whose spaces all sit at its floor
// and ceiling (within level_tol_m) is built exactly as before.
//
// Each ElementSolid carries a triangle mesh in WORLD coordinates (each
// storey's level frame mapped back through the inverse of its
// level_transform) and the IFC-ready extrusion it was made from: a 2D
// profile, a depth and a placement (local -> world), plus the opening voids
// a wall had subtracted, so the IFC writer (W6.3) extrudes the same
// geometry without re-deriving it.

#include <reusex/core/solid_model.hpp>

#include <pcl/PolygonMesh.h>

#include <array>
#include <cstddef>
#include <string>
#include <string_view>
#include <vector>

namespace reusex::geometry {

/// What an element solid is.
enum class SolidType { wall, slab, space };

/// "wall" | "slab" | "space".
std::string_view to_string(SolidType type);

/// An extruded area solid (IfcExtrudedAreaSolid): a profile in the local XY
/// plane swept along local +Z by @ref depth, placed by @ref placement.
struct ExtrusionProfile {
  /// Outer ring, counter-clockwise, open (first point not repeated) [m].
  std::vector<core::PlanPoint> outer;
  /// Holes, clockwise, open [m].
  std::vector<std::vector<core::PlanPoint>> holes;
  double depth = 0.0; ///< extrusion along local +Z [m], > 0
  /// Local -> world, 4x4 row-major (rigid).
  std::array<double, 16> placement{1, 0, 0, 0, 0, 1, 0, 0,
                                   0, 0, 1, 0, 0, 0, 0, 1};
};

/// An opening a wall had subtracted (IfcOpeningElement): the box cut, as an
/// extrusion through the wall.
struct OpeningVoid {
  std::string guid; ///< core::OpeningRecord::guid
  std::string kind; ///< door | window | opening
  ExtrusionProfile profile;
};

/// One element's solid.
struct ElementSolid {
  SolidType type = SolidType::wall;
  /// The element it was built from: the wall's, the space's or (for a slab)
  /// the storey's guid.
  std::string source_guid;
  int storey = 0; ///< core::StoreyRecord::idx
  /// IFC predefined type: wall "STANDARD"; slab "FLOOR" | "ROOF"; space its
  /// kind (room | corridor | open_plan | unknown).
  std::string predefined;
  /// Slabs: which piece of the storey's floor (or roof) this is, numbered
  /// over its levels, lowest first, each level's pieces largest first.
  int part = 0;
  ExtrusionProfile profile;
  std::vector<OpeningVoid> voids; ///< walls only
  /// Triangles, outward-facing, in world coordinates (PointXYZ vertices).
  pcl::PolygonMesh mesh;
  bool closed = false;  ///< CGAL::is_closed() of the mesh
  double volume = 0.0;  ///< enclosed volume [m^3] (0 when not closed)
  int voids_failed = 0; ///< openings the corefinement could not subtract
};

/// How the solids are built.
struct SolidsOptions {
  /// Thickness given to a wall whose thickness is unknown (-1, its axis is
  /// the seen face): the box grows away from the space it faces, or about
  /// the axis when no space is on either side [m].
  double unknown_wall_thickness_m = 0.10;
  /// Extend a wall end that meets another wall (corner, tee) by half that
  /// wall's thickness, so the boxes meet at the corner.
  bool extend_at_joins = true;
  /// An opening box reaches this far past both faces of its wall, and past
  /// the wall's bottom / top when it touches them, so the cut goes through
  /// instead of leaving a coplanar sliver [m].
  double opening_overcut_m = 0.05;
  /// Slab thickness when there is no storey below (and for the roof) [m].
  double slab_thickness_m = 0.20;
  /// A gap to the storey below wider than this is not a slab (a split level,
  /// a missed storey): the default thickness is used instead [m].
  double max_slab_thickness_m = 1.0;
  /// A slab piece smaller than this is dropped [m^2].
  double slab_min_area_m2 = 0.5;
  /// A hole in a slab smaller than this is filled [m^2].
  double slab_min_hole_area_m2 = 1.0;
  /// Also build a roof slab over the top storey's ceiling.
  bool roof = true;
  /// Space floors (ceilings) within this of each other are one floor
  /// (ceiling) level, and a level within this of the storey's floor
  /// (ceiling) is that floor (ceiling) [m]. See the file comment.
  double level_tol_m = 0.15;
  /// Detected walls take their base / top from the spaces beside them.
  bool walls_follow_spaces = true;
};

/// What build_solids() made.
struct SolidsResult {
  std::vector<ElementSolid> walls, slabs, spaces;
  std::size_t walls_skipped = 0;    ///< zero-length axis or no height
  std::size_t spaces_skipped = 0;   ///< degenerate polygon or no height
  std::size_t openings_cut = 0;     ///< subtracted from their wall
  std::size_t openings_skipped = 0; ///< host missing, or outside the wall
  std::size_t openings_failed = 0;  ///< corefinement failed; wall left uncut
  std::size_t not_closed = 0;       ///< solids whose mesh is not closed
};

/// Build one solid per wall, slab piece and space of @p model.
/// Deterministic for a given model (STANDARDS §6): elements come out in the
/// model's order; slabs by storey, then floor level (lowest first), then
/// piece area (largest first); roof slabs likewise by ceiling level.
SolidsResult build_solids(const core::SolidModel &model,
                          const SolidsOptions &options = {});

} // namespace reusex::geometry
