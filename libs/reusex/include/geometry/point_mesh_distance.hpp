// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Point-to-mesh distances: the one definition shared by the solid benchmark's
// fit metrics (reconstruction/solid_metrics, fit_report) and the deviation
// layer of the scene builder (visualize/scene, reusex-8vx). Lives in
// geometry_common (Layer 1½) so both reach it without a peer edge.

#include "reusex/types/point_types.hpp"

#include <cstddef>
#include <vector>

namespace pcl {
struct PolygonMesh;
} // namespace pcl

namespace reusex::geometry {

/// Distance [m] from every point of @p cloud to the nearest triangle of
/// @p mesh, index-aligned with the cloud: NaN for a point with a non-finite
/// coordinate. A CGAL AABB tree over the mesh's faces; polygons with more
/// than three vertices are fanned, degenerate triangles skipped. The cloud and
/// the mesh must share a frame. Deterministic.
///
/// @throws std::invalid_argument when the mesh has no usable (non-degenerate)
///         triangle or a face references a vertex the mesh does not have.
std::vector<double> point_to_mesh_distances(const Cloud &cloud,
                                            const pcl::PolygonMesh &mesh);

/// Summary of a set of point-to-mesh distances.
struct DistanceSummary {
  std::size_t points = 0; ///< finite distances summarised
  double mean_m = 0.0;
  double median_m = 0.0; ///< nearest-rank
  double p95_m = 0.0;    ///< nearest-rank 95th percentile
  double max_m = 0.0;
  /// Share of points with distance <= the `within` threshold.
  double within_share = 0.0;
  /// Share of points with distance > the `beyond` threshold.
  double beyond_share = 0.0;
};

/// Summarise @p distances (NaN entries are ignored); @p within and @p beyond
/// are the thresholds of DistanceSummary::within_share / beyond_share. All
/// zero when no distance is finite.
DistanceSummary summarize_distances(const std::vector<double> &distances,
                                    double within, double beyond);

/// How much of a mesh the scan supports (the mesh-to-cloud direction that
/// point_to_mesh_distances() cannot see: a surface with no scan point near it,
/// e.g. a fake wall, leaves every point-to-mesh distance unchanged).
struct MeshSupport {
  double area_m2 = 0.0;             ///< total area of the usable triangles
  double unsupported_area_m2 = 0.0; ///< area farther than support_dist
  std::size_t samples = 0;          ///< surface samples tested
};

/// Sample every triangle of @p mesh (polygons fanned, degenerate triangles
/// skipped) with about one sample per @p spacing x @p spacing of its area, at
/// a fixed low-discrepancy pattern (R2 sequence), each sample carrying an
/// equal share of its triangle's area; a sample is unsupported when no
/// finite point of @p cloud lies within @p support_dist of it (a uniform
/// hash grid over the cloud). The cloud and the mesh must share a frame.
/// Deterministic.
///
/// @throws std::invalid_argument when @p support_dist or @p spacing is not
///         positive and finite, the cloud has no finite point, the mesh has
///         no usable triangle, or a face references a vertex the mesh does
///         not have.
MeshSupport mesh_support(const Cloud &cloud, const pcl::PolygonMesh &mesh,
                         double support_dist, double spacing);

} // namespace reusex::geometry
