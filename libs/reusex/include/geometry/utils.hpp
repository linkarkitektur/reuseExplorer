// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "reusex/core/logging.hpp"
#include "reusex/types.hpp"

#include <Eigen/Core>
#include <boost/functional/hash.hpp>
#include <pcl/ModelCoefficients.h>
#include <pcl/PolygonMesh.h>
#include <pcl/common/pca.h>
#include <range/v3/view/iota.hpp>
#include <range/v3/view/zip.hpp>

#include <stdexcept>

namespace reusex::geometry {

/**
 * @brief Calculate distance from a point to a plane.
 * @param plane Plane coefficients (nx, ny, nz, d) where n is the normal.
 * @param point 3D point coordinates.
 * @return Signed distance from point to plane.
 */
auto dist_plane_point(const Eigen::Vector4d &plane,
                      const Eigen::Vector3d &point) -> double;

/**
 * @brief Create pairs of opposite parallel planes.
 *
 * Finds pairs of planes with opposite normals within a distance threshold.
 * Creates new planes for unpaired planes.
 *
 * @param planes Plane coefficients.
 * @param inliers Indices of points belonging to each plane.
 * @param centroids Plane centroids.
 * @param threshold Maximum distance threshold for pairing. Default 0.6.
 * @param new_plane_offset Offset for creating new planes. Default 0.5.
 * @return Vector of plane index pairs.
 */
auto make_pairs(EigenVectorContainer<double, 4> &planes,
                std::vector<IndicesPtr> &inliers,
                EigenVectorContainer<double, 3> &centroids,
                const double threshold = 0.6,
                const double new_plane_offset = 0.5)
    -> std::vector<std::pair<size_t, size_t>>;

/**
 * @brief Force planes to be orthogonal to a reference direction.
 *
 * Adjusts plane normals to be either parallel or perpendicular to the up
 * vector. A snapped plane is then re-offset to pass through its own inlier
 * centroid: rotating the normal in place while keeping the old `d` would
 * instead pivot the plane about whichever point is closest to the origin,
 * which can move it metres away from the points it was fit to (reusex-8vj).
 *
 * @param planes Plane coefficients to adjust.
 * @param centroids Each plane's inlier centroid, index-aligned with @p
 * planes. A snapped plane's offset is recomputed through this point; a plane
 * left untouched (neither near-vertical nor near-horizontal) keeps its
 * original offset regardless of this value.
 * @param threshold Angular threshold for orthogonality. Default 0.1.
 * @param up Reference up vector. Default (0, 0, 1).
 * @return Adjusted plane coefficients.
 * @throws std::invalid_argument if `planes.size() != centroids.size()`.
 */
auto force_orthogonal_planes(EigenVectorContainer<double, 4> &planes,
                             const EigenVectorContainer<double, 3> &centroids,
                             const double threshold = 0.1,
                             const Eigen::Matrix<double, 3, 1> &up =
                                 Eigen::Matrix<double, 3, 1>(0, 0, 1))
    -> EigenVectorContainer<double, 4>;

/**
 * @brief Count number of inliers for a plane.
 * @param cloud Input point cloud.
 * @param plane Plane coefficients.
 * @param indices Point indices to check.
 * @param threshold Distance threshold for inliers. Default 0.2.
 * @return Number of inliers.
 */
auto compute_number_of_inliers(CloudConstPtr cloud,
                               Eigen::Vector4d const &plane,
                               IndicesConstPtr indices,
                               const float threshold = 0.2) -> size_t;

/**
 * @brief Merge similar planes based on angle and distance.
 *
 * Agglomerative clustering: a pair merges when it is coplanar (normal and
 * offset within the tolerances below) **and** spatially adjacent (inlier
 * bounding boxes overlap, or their gap is below the coplanarity tolerance).
 *
 * Planes whose normals face opposite ways (dot < 0) never merge: they are the
 * two faces of one wall, which make_pairs pairs, not a duplicate surface.
 *
 * There is no inlier-overlap gate. The `min_overlap` parameter this function
 * used to accept was removed in #325: measured on a real scan, honouring it at
 * its documented 0.8 default blocked legitimate split-wall merges (the very
 * regression #215 fixed), and any lower value changed nothing while doubling
 * the merge cost.
 *
 * @param planes_ Input plane coefficients.
 * @param inliers_ Inliers for each plane.
 * @param centroids_ Plane centroids.
 * @param cloud Point cloud.
 * @param angle_threshold Angular similarity threshold. Default 0.1.
 * @param distance_threshold Distance threshold for merging. Default 0.5.
 * @param horizontal_distance_threshold A pair of floors / ceilings (both
 *   |n_z| > 0.9) merges only when their centroids' heights differ by less
 *   than this [m]; <= 0 (default): no such gate. A split level's floors, or
 *   a ceiling beside a lower sloped one, are a few decimetres apart and must
 *   not merge, although the plane distance (measured at the midpoint of the
 *   centroids, where a slope nearly meets the flat ceiling it runs up to) is
 *   small (reusex-2kd).
 * @return Tuple of (merged planes, merged inliers, merged centroids).
 */
auto merge_planes(EigenVectorContainer<double, 4> const &planes_,
                  std::vector<IndicesPtr> const &inliers_,
                  EigenVectorContainer<double, 3> const &centroids_,
                  CloudConstPtr cloud, const double angle_threshold = 0.1,
                  const double distance_threshold = 0.5,
                  const double horizontal_distance_threshold = -1.0)
    -> std::tuple<EigenVectorContainer<double, 4>, std::vector<IndicesPtr>,
                  EigenVectorContainer<double, 3>>;

/**
 * @brief Separate planes into horizontal and vertical based on up vector.
 *
 * @param planes Plane coefficients.
 * @param up Reference up vector. Default (0, 0, 1).
 * @param epsilon Angular epsilon for classification. Default 0.1.
 * @return Tuple of (horizontal indices, vertical indices).
 */
auto separate_planes(const EigenVectorContainer<double, 4> &planes,
                     const Eigen::Vector3d &up = Eigen::Vector3d(0, 0, 1),
                     const double epsilon = 0.1)
    -> std::tuple<std::vector<size_t>, std::vector<size_t>>;

/**
 * @brief Compute the normal vector of a polygon.
 *
 * Calculates the normal vector of a polygon using the cross product sum method
 * (also known as Newell's method). The normal is automatically normalized.
 *
 * @tparam CloudPtr Point cloud pointer type (const or non-const, e.g.,
 * CloudLocConstPtr).
 * @param poly Polygon with vertex indices.
 * @param cloud Point cloud containing the vertices (read-only).
 * @return Normalized normal vector of the polygon.
 * @throws std::invalid_argument if polygon has fewer than 3 vertices.
 * @throws std::runtime_error if polygon is degenerate (zero normal magnitude).
 */
template <typename CloudPtr>
auto compute_polygon_normal(const pcl::Vertices &poly, const CloudPtr &cloud)
    -> Eigen::Vector3f {
  if (poly.vertices.size() < 3) {
    throw std::invalid_argument(
        "Polygon must have at least 3 vertices to compute a normal");
  }

  Eigen::Vector3f normal = Eigen::Vector3f::Zero();
  for (size_t i = 0; i < poly.vertices.size(); ++i) {
    const auto idx_c = poly.vertices[i];
    const auto idx_n = poly.vertices[(i + 1) % poly.vertices.size()];
    const auto &v_c = cloud->points[idx_c].getVector3fMap();
    const auto &v_n = cloud->points[idx_n].getVector3fMap();
    normal += v_c.cross(v_n);
  }

  const float magnitude = normal.norm();
  if (magnitude < 1e-10f) {
    throw std::runtime_error(
        "Cannot compute normal for degenerate polygon (zero magnitude)");
  }

  normal.normalize();
  return normal;
}

} // namespace reusex::geometry
