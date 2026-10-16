// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "reusex/reconstruction/CellComplex.hpp"
#include "reusex/reconstruction/Solidifier.hpp"
#include "reusex/reconstruction/structural_planes.hpp"
#include "reusex/types.hpp"

#include <pcl/PolygonMesh.h>
#include <pcl/common/common.h>

#include <array>
#include <memory>
#include <utility>
#include <vector>

namespace reusex::geometry {

/**
 * @brief Options for mesh generation.
 */
struct MeshOptions {
  // Defaults reconciled with the CLI in issue #217. The `rux create mesh` CLI
  // shipped with the tuned production values 0.60 / 0.25 (its footer examples
  // and workflow assume them); the old library defaults 10.0 / 0.05 were never
  // exercised through the CLI. Per docs/STANDARDS.md §4 the library is the
  // single source of truth, so the better CLI values were promoted here.
  float search_threshold = 0.60f;   ///< Search threshold for mesh generation
  float new_plane_offset = 0.25f;   ///< Offset for new plane creation
  IndicesConstPtr filter = nullptr; ///< Optional filter to limit processing

  /// MIP solver wall-clock time limit in seconds (issue #212). Guards against
  /// the solver hanging indefinitely on complex buildings.
  double time_limit_seconds = 120.0;

  /// Objective wall-weight passed to the Solidifier (was hardcoded, #212).
  double alpha = 0.04;

  /// Maximum cells the cell complex may generate before failing fast (#213).
  size_t max_cells = 5000;

  /// Solve the MIP per horizontal section when the arrangement is multi-storey
  /// and large enough, keeping multi-room buildings tractable (issue #226).
  bool sectioned = true;

  /// Cell-count threshold above which the sectioned solve engages (issue #226).
  size_t sectioned_threshold = 2000;

  /// MIP backend + fallback policy: auto (primary with HiGHS fallback), cuopt,
  /// or highs (issue #226).
  SolverChoice solver = SolverChoice::automatic;

  /// Size guard for `solver == automatic`: a (sub)problem over this many MIP
  /// variables runs on HiGHS instead of cuOpt, pre-emptively (reusex-81y:
  /// cuOpt SIGSEGVs on sections around ~3.3M variables).
  size_t cuopt_max_variables = 2'000'000;

  /// Which planes become arrangement lines and storey sections
  /// (reusex-8l3). See StructuralPlaneOptions.
  StructuralPlaneOptions structural{};

  /// Role of each input plane from the plane classification (the
  /// plane_classes table, `rux create classify`), index-aligned with the
  /// `planes` passed to mesh(). Planes whose role is furniture or clutter are
  /// left out before regularisation (reusex-z1v.4.3, see
  /// drop_planes_by_role()). Empty: no classification, every plane is used.
  std::vector<core::PlaneRole> plane_roles{};

  /// Storey-local arrangement (reusex-sba): give each horizontal section of
  /// the cell complex only the walls whose inlier z-range reaches into it
  /// (wall_z_ranges()), so one storey's walls no longer cut every other
  /// storey. Off: every wall spans every section (the behaviour before sba).
  bool storey_local = true;
  /// A wall reaches a section when its z-range overlaps it by this much [m]
  /// (by half the section for a thinner one, i.e. a slab).
  double section_overlap_m = CellComplex::default_min_section_overlap_m;

  /// World z of every storey floor and ceiling (reusex-z1v.3.4), from the
  /// storeys table through LevelFrame::world_z_at (pipeline::
  /// mesh_section_heights()). When non-empty, only horizontal planes within
  /// section_tolerance_m of one of these heights become section planes of
  /// the cell complex (select_section_planes()): table tops, shelves and the
  /// slab faces make_pairs invents no longer slice the building. Empty =
  /// every horizontal plane is a section (legacy).
  std::vector<double> section_heights{};
  double section_tolerance_m = 0.15;
};

/**
 * @brief The horizontal planes that become section planes (reusex-z1v.3.4).
 *
 * @param planes Plane coefficients (n . x + d = 0, unit n).
 * @param horizontals Indices into @p planes of the horizontal planes.
 * @param section_heights World z of the storey floors and ceilings.
 * @param tolerance_m A plane within this distance of a height is kept.
 * @return The horizontals within @p tolerance_m of a storey height, in input
 *   order. Returns @p horizontals unchanged when @p section_heights is empty,
 *   or (logged at warn) when fewer than two planes match: a stale storeys
 *   table must not silence the mesh stage.
 */
std::vector<size_t>
select_section_planes(const EigenVectorContainer<double, 4> &planes,
                      const std::vector<size_t> &horizontals,
                      const std::vector<double> &section_heights,
                      double tolerance_m);

/**
 * @brief World z-range of each wall plane, from its inliers (reusex-sba).
 *
 * For CellComplex's storey-local arrangement. A vertical plane's range is
 * the 2nd to 98th percentile of its inliers' z, so a few stray points do not
 * stretch a wall into the next storey. The two faces of a wall pair (@p
 * pairs, from make_pairs) share the union of their ranges, so a face
 * make_pairs invented (no inliers) takes its partner's. Planes that are not
 * vertical, and walls with no inliers on either face, get {NaN, NaN}: they
 * reach every section.
 *
 * @return One range per plane, index-aligned with @p planes.
 */
std::vector<std::array<double, 2>>
wall_z_ranges(const EigenVectorContainer<double, 4> &planes,
              const std::vector<IndicesPtr> &inliers, CloudConstPtr cloud,
              const std::vector<std::pair<size_t, size_t>> &pairs);

/**
 * @brief Generate a mesh from point cloud and geometric primitives.
 *
 * @param cloud Input point cloud.
 * @param normals Point cloud normals.
 * @param planes Detected plane coefficients.
 * @param centroids Plane centroids.
 * @param inliers Indices of points belonging to each plane.
 * @param rooms Room labels for points.
 * @param opt Mesh generation options (includes optional filter).
 * @return Generated polygon mesh.
 */
pcl::PolygonMeshPtr mesh(CloudConstPtr cloud, CloudNConstPtr normals,
                         EigenVectorContainer<double, 4> &planes,
                         EigenVectorContainer<double, 3> &centroids,
                         std::vector<IndicesPtr> &inliers, CloudLConstPtr rooms,
                         MeshOptions const opt = MeshOptions{});
} // namespace reusex::geometry
