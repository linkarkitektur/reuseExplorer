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
  /// Two floors or ceilings merge only when their patches' heights differ
  /// by less than this [m] (merge_planes' horizontal_distance_threshold,
  /// reusex-2kd): a split level's ceilings, or a flat ceiling beside a
  /// lower sloped one, stay apart. Walls keep the 0.5 m merge distance.
  double horizontal_merge_m = 0.3;
  /// A wall reaches a section when its z-range overlaps it by this much [m]
  /// (by half the section for a thinner one, i.e. a slab).
  double section_overlap_m = CellComplex::default_min_section_overlap_m;

  /// World z of every storey floor and ceiling (reusex-z1v.3.4), from the
  /// storeys table through LevelFrame::world_z_at (pipeline::
  /// mesh_section_heights()). When non-empty, only the horizontal planes
  /// inside the building become section planes of the cell complex
  /// (select_section_planes(), reusex-x8s): storey floors and ceilings,
  /// split levels and an attic ceiling under the roof stay; planes outside
  /// the building's height and the invented slab faces below its lowest
  /// floor and above its top ceiling go. Empty = every horizontal plane is
  /// a section (legacy).
  std::vector<double> section_heights{};
  double section_tolerance_m = 0.15;
};

/**
 * @brief The horizontal planes that become section planes (reusex-z1v.3.4,
 * reusex-x8s): the ones inside the building.
 *
 * The building's height is the span of the storey floors and ceilings and
 * of every plane z-range in @p z_ranges (walls and roof slopes measured from
 * their inliers, wall_z_ranges()), so an attic under the roof is inside
 * although the storeys table ends at the top storey's ceiling. A measured
 * horizontal (one with inliers) is kept when it lies within @p tolerance_m
 * of that span: storey floors and ceilings, split levels and the attic
 * ceiling stay; a plane outside it (NewOffice: tilted planes that
 * force_orthogonal_planes moved metres away from their own points) goes. A
 * face make_pairs invented (no inliers) is kept only between the lowest and
 * highest kept measured planes: the underside of the lowest floor and the
 * top of the highest ceiling are outside the building, a slab face between
 * two storeys is not.
 *
 * @param planes Plane coefficients (n . x + d = 0, unit n).
 * @param horizontals Indices into @p planes of the horizontal planes.
 * @param section_heights World z of the storey floors and ceilings.
 * @param tolerance_m Margin around the building's height.
 * @param z_ranges World z-range per plane, index-aligned with @p planes
 *   ({NaN, NaN}: none); may be empty.
 * @param inliers Inliers per plane, index-aligned with @p planes.
 * @return The kept horizontals, in input order. Returns @p horizontals
 *   unchanged when @p section_heights is empty, or (logged at warn) when
 *   fewer than two planes are kept: a stale storeys table must not silence
 *   the mesh stage.
 */
std::vector<size_t>
select_section_planes(const EigenVectorContainer<double, 4> &planes,
                      const std::vector<size_t> &horizontals,
                      const std::vector<double> &section_heights,
                      double tolerance_m,
                      const std::vector<std::array<double, 2>> &z_ranges,
                      const std::vector<IndicesPtr> &inliers);

/**
 * @brief World z-range of each wall plane, from its inliers (reusex-sba).
 *
 * For CellComplex's storey-local arrangement. A vertical plane's range
 * starts at the 2nd to 98th percentile of its inliers' z and follows the
 * sorted heights on outwards while consecutive ones are at most 0.15 m
 * apart: a stray point beyond a gap does not stretch a wall into the next
 * storey, but a stretch of the wall that reaches further than the rest is
 * kept however small its share (a long wall reaching down to a split
 * level's lower floor along one room only, reusex-7co; a wall's top up to
 * a ceiling higher than most of its points reach, reusex-2kd). With @p
 * storey_heights it follows them no further than 0.05 m past the storey
 * floor (ceiling) nearest to the percentile: a drifted scan's wall points
 * run on into the slab between two storeys (house2). The two faces of a
 * wall pair (@p pairs, from make_pairs) share the union of their ranges, so a
 * face make_pairs invented (no inliers) takes its partner's. Oblique planes
 * (roof slopes, reusex-e6z) get their range the same way, shared with their
 * invented back face. Floors and ceilings, and planes with no inliers on
 * either face, get {NaN, NaN}: a wall without one reaches every section.
 *
 * @param storey_heights World z of the storey floors and ceilings
 *   (MeshOptions::section_heights); empty: no such stop.
 * @return One range per plane, index-aligned with @p planes.
 */
std::vector<std::array<double, 2>>
wall_z_ranges(const EigenVectorContainer<double, 4> &planes,
              const std::vector<IndicesPtr> &inliers, CloudConstPtr cloud,
              const std::vector<std::pair<size_t, size_t>> &pairs,
              const std::vector<double> &storey_heights = {});

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
