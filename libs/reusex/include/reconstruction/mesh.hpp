// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "reusex/reconstruction/Solidifier.hpp"
#include "reusex/reconstruction/structural_planes.hpp"
#include "reusex/types.hpp"

#include <pcl/PolygonMesh.h>
#include <pcl/common/common.h>

#include <memory>

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
};

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
