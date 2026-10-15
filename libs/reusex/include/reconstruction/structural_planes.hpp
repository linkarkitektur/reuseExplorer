// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "reusex/types/eigen_types.hpp"
#include "reusex/types/point_types.hpp"

#include <Eigen/Core>

#include <cstddef>
#include <vector>

namespace reusex::geometry {

/**
 * @brief Options for picking the planes the cell complex is built from
 * (reusex-8l3).
 *
 * `create planes` keeps every planar surface of at least
 * `SegmentPlanesOptions::min_surface_area` (0.5 m^2): wall pieces, but also
 * door and window jambs, desk tops and cabinet sides. Those are real surfaces,
 * so they stay in the `planes` cloud. The cell complex is a different matter:
 * it extends every vertical plane into a line across the whole building and
 * cuts every horizontal plane into a storey section, so the projected cell
 * count is roughly (lines along x) x (lines along y) x (sections). On the
 * NewOffice scan, 88 vertical planes (plus 46 back faces `make_pairs` invents
 * for the unpaired ones) gave 3898 arrangement faces, and furniture tops
 * doubled the section count, for 35082 cells against a 5000-cell limit.
 *
 * Every threshold is in metres or square metres, measured on the inlier
 * points, so none of them depends on the scan's point density.
 */
struct StructuralPlaneOptions {
  /// Off: pass every plane through unchanged (the behaviour before 8l3).
  bool enabled = true;

  /// Vertical planes facing the same way whose offsets differ by less than
  /// this [m] lie on one wall line and become one arrangement line, wherever
  /// along the line they sit. That covers the piers between two windows, a
  /// window recess, or a wall scanned in pieces. Opposite-facing planes never
  /// merge: they are the two faces of a wall, which `make_pairs` pairs.
  double collinear_offset_m = 0.15;
  /// Angular tolerance [deg] for that same-line test. Small on purpose: the
  /// planes have been regularised and forced vertical by this point.
  double collinear_angle_deg = 5.0;

  /// A wall line (after merging) needs at least this much surface [m^2],
  /// counting the opposite face of the same wall, to become an arrangement
  /// line. Below it are jambs, reveals and
  /// furniture sides (0.2-1 m wide). Each one would cut the whole building,
  /// and none of them bounds a room.
  double min_wall_area_m2 = 1.5;
  /// The opposite face of a wall, if one lies within this distance [m]
  /// behind a wall line, adds its area to the line's support. `mesh()`
  /// overrides it with `MeshOptions::search_threshold`, the distance
  /// `make_pairs` pairs faces over.
  double wall_pair_max_m = 0.6;

  /// A horizontal plane that sits less than this [m] above a larger
  /// upward-facing plane under the same footprint is furniture: a desk top
  /// over the floor, or the underside of a desk or shelf. It is not a storey
  /// floor or ceiling, so it does not become a section. Storeys are taller
  /// than this, and a split level does not lie over the lower floor's
  /// footprint, so both survive.
  double min_storey_clearance_m = 1.8;
  /// Minimum fraction of a horizontal plane's footprint that has to lie over
  /// the lower plane before the clearance rule applies.
  double min_footprint_overlap = 0.25;

  /// Cell size [m] of the grid used to measure point density. A plane's area
  /// is its inlier count times cell^2 / (median points per occupied cell),
  /// so a sparse patch does not read as a large surface.
  double area_cell_m = 0.10;
  /// Cell size [m] of the xy grid used for the footprint-overlap test. It is
  /// coarser than area_cell_m, and the lower plane's footprint is dilated by
  /// one cell, so a floor seen only in gaps under a desk still counts.
  double footprint_cell_m = 0.25;
};

/// What select_structural_planes() did, for logging and tests.
struct StructuralPlaneReport {
  std::size_t vertical_in = 0;     ///< vertical planes considered
  std::size_t vertical_merged = 0; ///< absorbed into a collinear wall line
  std::size_t vertical_small = 0;  ///< lines dropped below min_wall_area_m2
  std::size_t horizontal_in = 0;   ///< horizontal planes considered
  std::size_t horizontal_furniture = 0; ///< dropped by the clearance rule
  std::size_t other = 0; ///< neither vertical nor horizontal, passed through
  std::size_t planes_out = 0; ///< planes left after selection
  /// Surface area one inlier point stands for [m^2], measured on the scan.
  double area_per_point_m2 = 0.0;
};

/**
 * @brief Reduce a plane set to the planes the cell complex should be built
 * from (reusex-8l3).
 *
 * Run it after regularisation, merging and `force_orthogonal_planes`, and
 * before `make_pairs`. It modifies `planes`, `inliers` and `centroids` in
 * place. A merged wall line keeps its best-supported member's plane and
 * centroid and takes the union of the members' inliers. Planes that are
 * neither vertical nor horizontal pass through unchanged. The output order
 * depends only on the input, so the result is deterministic.
 *
 * @throws std::runtime_error if the three containers differ in size.
 */
auto select_structural_planes(EigenVectorContainer<double, 4> &planes,
                              std::vector<IndicesPtr> &inliers,
                              EigenVectorContainer<double, 3> &centroids,
                              CloudConstPtr cloud,
                              const StructuralPlaneOptions &opt = {})
    -> StructuralPlaneReport;

} // namespace reusex::geometry
