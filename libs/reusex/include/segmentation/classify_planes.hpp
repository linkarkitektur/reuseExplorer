// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

/// @file
/// Plane classification: semantics x geometry (solid-model WP2, W2.1).
///
/// For every plane of the plane segmentation: its orientation against the
/// measured vertical, the majority semantic class of its inliers with the
/// vote share, a role (wall / floor / ceiling / slab / furniture / clutter /
/// unknown), the storey it belongs to, its area and its height range.
///
/// The role comes from the semantic classes through a core::ClassRoleMap when
/// they are trusted and the geometry does not contradict it; otherwise from
/// the geometric rules. Trusted: enough of the inliers are labelled
/// (min_labelled_share) and one role holds min_class_share of the labelled
/// votes, each point voting for its class's role -- so "cabinet" and "shelf"
/// votes add up to furniture (reusex-rqn.4). When no single role does but
/// furniture and clutter together do (a pier voted "column", "cabinet" and
/// some "wall"), the plane is the one of those two with more votes. The
/// record still carries the majority class and its own share. The rules
/// (Poux 2019/2020), which also run with no labels cloud at all. "Large" is
/// min_large_area_m2, "too big for furniture" is max_furniture_area_m2, the
/// storey's floor / ceiling heights come from its row (with no storeys: the
/// lowest large horizontal plane facing up, and the highest facing down at
/// least min_storey_height_m above it):
///  - floor: large horizontal plane facing up in the floor band, or too big
///    for furniture and in the lower half of the storey (a split level);
///  - ceiling: large horizontal plane facing down in the ceiling band, or
///    too big for furniture in the upper half (a lowered ceiling); a large
///    slanted plane facing down in the upper half (a pitched ceiling);
///  - wall: vertical, touching the floor or ceiling band (or the height of a
///    split-level floor), at least min(min_wall_height_m,
///    min_wall_height_share x the height from the floor it stands on to the
///    ceiling) tall;
///  - furniture: horizontal, facing up, table_min_m..table_max_m above the
///    floor and smaller than max_furniture_area_m2;
///  - clutter: any other horizontal or vertical plane;
///  - unknown: other slanted planes, planes outside every storey, no points.
/// The rules never produce "slab"; only the class map does.
///
/// "Facing" uses the sign of the plane normal, which faces the sensor that
/// saw the plane (CONTRACTS.md, `clouds`): a floor is seen from above, a
/// ceiling from below. Set `oriented_normals = false` for projects whose
/// clouds predate that (reusex-0pi: normals faced the world origin); the
/// rules then ignore the sign.
///
/// Heights are measured in the level frame (core::StoreyRecord::
/// level_transform) of the plane's storey; with no storeys, world z is up.
///
/// Pure algorithm, deterministic (STANDARDS §6): planes are visited in label
/// order, heights use exact order statistics, the area grid is anchored at
/// the world origin. No ProjectDB; the stage that stores the rows is W2.2.

#include "reusex/core/class_role_map.hpp"
#include "reusex/core/solid_model.hpp"
#include "reusex/types/point_types.hpp"

#include <cstdint>
#include <map>
#include <string>
#include <string_view>
#include <vector>

namespace reusex::geometry {

struct ClassifyPlanesOptions {
  /// |n . up| above this = horizontal (10 deg; LevelOptions::horizontal_cos).
  double horizontal_cos = 0.9848;
  /// |n . up| below this = vertical (80 deg from up); between = slanted.
  double vertical_cos = 0.1736;

  /// A floor/ceiling candidate covers at least this much area [m^2].
  double min_large_area_m2 = 1.0;
  /// Floor and ceiling bands: within this of the storey's floor / ceiling
  /// height [m]. Also the slack when assigning a plane to a storey.
  double band_m = 0.3;
  /// A wall's robust vertical extent is at least this [m]...
  double min_wall_height_m = 2.0;
  /// ...or this share of its storey's height, whichever is less (a low attic
  /// storey has walls shorter than 2 m).
  double min_wall_height_share = 0.75;
  /// A horizontal plane larger than this is structure (floor / ceiling),
  /// never furniture [m^2].
  double max_furniture_area_m2 = 6.0;
  /// Plane normals face the sensor (CONTRACTS.md); see the file comment.
  bool oriented_normals = true;
  /// Table-top heights above the floor [m].
  double table_min_m = 0.6;
  double table_max_m = 1.2;
  /// Without storeys: the ceiling is at least this far above the floor [m].
  double min_storey_height_m = 2.0;

  /// Side of the occupancy grid that measures a plane's area [m]. Set it at
  /// or above the cloud's point spacing, or gaps between points are lost.
  double area_cell_m = 0.1;
  /// Robust height range = these quantiles of the inlier heights (the
  /// rules); the record's z_min / z_max are the true extremes.
  double extent_quantile = 0.02;

  /// The semantic classes decide the role only if at least this share of the
  /// plane's inliers carries a label...
  double min_labelled_share = 0.1;
  /// ...and one role (furniture and clutter together, failing that) holds at
  /// least this share of those votes, each point voting for its class's role.
  double min_class_share = 0.5;
};

/// Who decided a plane's role.
enum class RoleSource {
  none,      ///< no points: role unknown
  semantic,  ///< the labelled points' majority role, via the class map
  rules,     ///< the geometric rules (no usable class)
  overruled, ///< the class gave a role the geometry contradicts; rules won
};

/// "none" | "semantic" | "rules" | "overruled": the plane_classes.role_source
/// text (core::PlaneClassRecord::role_source).
std::string_view to_string(RoleSource source) noexcept;

/// The semantic input: a label per point and the class names.
struct SemanticLabels {
  /// Point labels, index-aligned with the cloud; point label = API class + 1,
  /// 0 = unlabeled (STANDARDS §3.1, the "labels" cloud).
  const CloudL *labels = nullptr;
  /// Class name per point label (the semantic cloud's label_definitions,
  /// keyed by point label).
  std::map<uint32_t, std::string> class_names;
};

struct ClassifyPlanesResult {
  /// One record per plane label 1..N, in label order (index = label - 1).
  /// semantic_class is an API class id (point label - 1), -1 none.
  /// z_min / z_max in the level frame of the plane's storey (of the first
  /// storey when it has none; world z with no storeys); area in m^2.
  std::vector<core::PlaneClassRecord> planes;
  /// Aligned with @ref planes.
  std::vector<RoleSource> role_source;
  /// Planes per role, keyed by core::to_string(role).
  std::map<std::string, int> role_counts;
  int from_semantic = 0; ///< planes whose role came from the class map
  int overruled = 0;     ///< planes whose class role the geometry rejected
  /// Floor / ceiling heights the rules used when no storeys were given (the
  /// implicit storey); NaN when not found.
  double implicit_floor_z = 0.0, implicit_ceiling_z = 0.0;
};

/// Classify every plane of one scan.
///
/// @param cloud         The scan.
/// @param plane_labels  Plane label per point, index-aligned with @p cloud;
///                      0 = no plane.
/// @param plane_normals Normal per plane, indexed by label_to_index(label);
///                      sign ignored.
/// @param storeys       Stored storey rows (may be empty: one implicit storey
///                      from the planes, records get storey = -1).
/// @param semantic      Optional labels (labels == nullptr: rules only).
/// @param role_map      Class name -> role.
/// @throws std::invalid_argument when cloud / plane_labels / semantic sizes
///         disagree, a plane label has no normal, or an option is out of
///         range.
ClassifyPlanesResult classify_planes(
    const Cloud &cloud, const CloudL &plane_labels, const CloudN &plane_normals,
    const std::vector<core::StoreyRecord> &storeys,
    const SemanticLabels &semantic = {},
    const core::ClassRoleMap &role_map = core::default_class_role_map(),
    const ClassifyPlanesOptions &opt = {});

} // namespace reusex::geometry
