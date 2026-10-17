// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "geometry/utils.hpp"
#include "core/logging.hpp"
#include "utils/tolerances.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <numeric>

namespace reusex::geometry {

namespace {

/// Ensure a plane's normal is unit length. Plane coefficients (n, d) may drift
/// from |n| = 1 through repeated arithmetic; downstream distance/overlap tests
/// assume a normalized normal. Renormalizes in place (scaling d with n so the
/// plane is geometrically unchanged) and debug-logs when the drift exceeds
/// `1e-6` (STANDARDS §4: validate normals at module boundaries).
void validate_normalized(Eigen::Vector4d &plane, const char *where,
                         size_t index) {
  const double norm = plane.head<3>().norm();
  if (norm < reusex::kEpsilonLengthM) {
    reusex::warn("{}: plane {} has a degenerate (near-zero) normal (|n|={:g})",
                 where, index, norm);
    return;
  }
  if (std::abs(norm - 1.0) > 1e-6) {
    reusex::debug("{}: normalizing plane {} normal (|n|={:g})", where, index,
                  norm);
    plane /= norm;
  }
}

} // namespace

auto dist_plane_point(const Eigen::Vector4d &plane,
                      const Eigen::Vector3d &point) -> double {
  const double sq_norm = plane.head<3>().squaredNorm();
  // A normalized normal has squared norm ~1; a value below the relative
  // (dimensionless) epsilon means the "plane" is degenerate. Return +inf so
  // callers that compare against a finite threshold treat it as "infinitely
  // far" rather than dividing by ~0 and producing NaN/inf silently.
  if (sq_norm < reusex::kEpsilonRelative) {
    reusex::warn("dist_plane_point: degenerate plane normal (|n|^2={:g}); "
                 "returning +inf",
                 sq_norm);
    return std::numeric_limits<double>::infinity();
  }
  return (plane.head<3>().dot(point) + plane[3]) / sq_norm;
}

auto make_pairs(EigenVectorContainer<double, 4> &planes,
                std::vector<IndicesPtr> &inliers,
                EigenVectorContainer<double, 3> &centroids,
                const double threshold, const double new_plane_offset,
                const double max_crossed_gap)
    -> std::vector<std::pair<size_t, size_t>> {

  using Vector3 = Eigen::Vector3d;
  using Vector4 = Eigen::Vector4d;

  std::vector<std::pair<size_t, size_t>> pairs;
  std::vector<bool> paired(planes.size(), false);

  for (size_t i = 0; i < planes.size(); ++i) {
    if (paired[i])
      continue;

    auto &plane_i = planes[i];
    auto &centroid_i = centroids[i];

    size_t best_j = planes.size(); // invalid
    double best_diff = threshold;

    // Inline loop instead of ranges pipeline
    for (size_t j = i + 1; j < planes.size(); ++j) {
      if (paired[j])
        continue;

      auto &plane_j = planes[j];
      auto &centroid_j = centroids[j];

      // Opposite normals: dot ≈ -1. Compare via the subtended angle so the
      // tolerance is an actual angle (radians), not a hand-tuned dot slack.
      const double dot =
          std::clamp(plane_i.head<3>().dot(plane_j.head<3>()), -1.0, 1.0);
      constexpr double kOppositeAngleTol = 0.1; // rad (~5.7°), pairing slack
      if (std::abs(std::numbers::pi - std::acos(dot)) >= kOppositeAngleTol)
        continue; // not opposite normals

      const double dist_i = dist_plane_point(plane_i, centroid_j.head<3>());
      const double dist_j = dist_plane_point(plane_j, centroid_i.head<3>());
      const double diff = (std::abs(dist_i) + std::abs(dist_j)) * 0.5;

      if (diff < best_diff) {
        best_diff = diff;
        best_j = j;
      }
    }

    if (best_j != planes.size()) {
      // The two faces of one wall bound a slab of constant thickness. Their
      // fitted normals may differ by up to kOppositeAngleTol, and both planes
      // become arrangement lines across the whole building: a 4 degree skew
      // between NewOffice's 27 m facade and a short face of the same wall
      // widened the slab to ~1 m at the facade's far end, and the mesh
      // followed the skewed face out past the facade (reusex-bi6). Turn the
      // face with fewer inliers to be antiparallel to the better-supported
      // one, keeping it through its own centroid.
      const size_t keep =
          inliers[i]->size() >= inliers[best_j]->size() ? i : best_j;
      const size_t turn = keep == i ? best_j : i;
      const Vector3 n = -planes[keep].head<3>();
      planes[turn].head<3>() = n;
      planes[turn][3] = -n.dot(centroids[turn].head<3>());

      // A wall is the slab behind both of its faces (CellComplex gives a
      // cell a pair's wall id only there). When the two rooms' scans drifted
      // apart by more than the wall is thick, the faces cross: each lies in
      // front of the other, and no cell is behind both, so the pair can
      // never become a wall and the rooms on either side merge (house1's
      // partition i-n1, crossed by 0.09 m, reusex-wm0). Faces this close
      // whose normals point at each other cannot bound a gap a sensor
      // looked into: they are one wall, so swap their positions -- the slab
      // then spans the stretch where the two scans overlap, on the wall's
      // axis, and each face keeps its normal towards its own room. Farther
      // apart they stay where they were measured.
      const double crossed_by =
          planes[keep].head<3>().dot(centroids[turn].head<3>()) +
          planes[keep][3];
      if (crossed_by > 0.0 && crossed_by <= max_crossed_gap) {
        const Vector3 n_keep = planes[keep].head<3>();
        const Vector3 c_keep = centroids[keep].head<3>();
        planes[keep][3] = -n_keep.dot(centroids[turn].head<3>());
        planes[turn][3] = n_keep.dot(c_keep); // normal -n_keep
        reusex::debug("make_pairs: faces {} and {} cross by {:.3f} m (one "
                      "wall seen from two drifted scans): slab between them",
                      i, best_j, crossed_by);
      } else if (crossed_by > 0.0) {
        reusex::debug("make_pairs: faces {} and {} look at each other "
                      "{:.3f} m apart (over {:.2f} m): left where measured",
                      i, best_j, crossed_by, max_crossed_gap);
      }

      pairs.emplace_back(i, best_j);
      paired[best_j] = true;
    } else {
      // Create new plane in-place
      Vector4 new_plane = -plane_i;
      new_plane[3] -= new_plane_offset;

      auto p = centroid_i.head<3>();
      float dist = dist_plane_point(new_plane, p);

      Vector3 new_centroid = p - dist * new_plane.head<3>();

      planes.push_back(new_plane);
      inliers.push_back(IndicesPtr(new Indices{}));
      centroids.push_back(new_centroid);

      pairs.emplace_back(i, planes.size() - 1);
      paired.push_back(true);
    }
  }

  return pairs;
}

auto force_orthogonal_planes(EigenVectorContainer<double, 4> &planes,
                             const EigenVectorContainer<double, 3> &centroids,
                             const double threshold, const Eigen::Vector3d &up)
    -> EigenVectorContainer<double, 4> {
  reusex::trace("Force planes to be orthogonal");

  if (planes.size() != centroids.size())
    throw std::invalid_argument(
        fmt::format("force_orthogonal_planes: {} planes but {} centroids",
                    planes.size(), centroids.size()));

  // Validate normals are unit length at this module boundary (STANDARDS §4).
  for (size_t i = 0; i < planes.size(); ++i)
    validate_normalized(planes[i], "force_orthogonal_planes", i);

  for (size_t i = 0; i < planes.size(); ++i) {
    auto &plane = planes[i];
    auto normal = plane.head<3>(); // This is a block, no copy
    const double scalar = normal.dot(up);
    bool snapped = true;

    if (std::abs(scalar) < threshold) {
      // Wall: make normal orthogonal to 'up' in-place
      normal -= scalar * up; // modify the block directly
      normal.normalize();    // in-place normalization
    } else if (std::abs(scalar - 1.0) < threshold) {
      // Floor
      normal = up;
    } else if (std::abs(scalar + 1.0) < threshold) {
      // Ceiling
      normal = -up;
    } else {
      snapped = false;
      reusex::warn("Plane with normal ({:3f}, {:3f}, {:3f}) is not vertical or "
                   "horizontal",
                   normal.x(), normal.y(), normal.z());
    }

    // Snapping the normal rotates the plane; keeping the old `d` would pivot
    // it about the point nearest the origin instead of its own inliers,
    // moving it metres off the points it was fit to (reusex-8vj). Re-offset
    // through the inlier centroid so the snapped plane still passes through
    // it.
    if (snapped)
      plane[3] = -normal.dot(centroids[i]);
  }

  return planes;
}

auto compute_number_of_inliers(CloudConstPtr cloud,
                               Eigen::Vector4d const &plane,
                               IndicesConstPtr indices, const float threshold)
    -> size_t {
  const auto &n = plane.head<3>();
  const auto &d = plane[3];
  // If normal not guaranteed normalized, uncomment:
  // const float invNorm = 1.0f / n.norm();
  // const float scaledThreshold = threshold * invNorm;

  return std::ranges::count_if(*indices, [&](int idx) {
    const auto &p =
        cloud->points[idx].getVector3fMap().cast<double>(); // directly 3f map
    float dist = n.dot(p) + d;
    return std::abs(dist) < threshold; // or scaledThreshold
  });
}

namespace {

/// Axis-aligned bounding box of a plane's inlier points.
struct InlierBBox {
  Eigen::Vector3d lo{
      Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity())};
  Eigen::Vector3d hi{
      Eigen::Vector3d::Constant(-std::numeric_limits<double>::infinity())};
  bool empty{true};
};

InlierBBox inlier_bbox(CloudConstPtr cloud, const Indices &idx) {
  InlierBBox b;
  for (int i : idx) {
    const Eigen::Vector3d p = cloud->points[i].getVector3fMap().cast<double>();
    b.lo = b.lo.cwiseMin(p);
    b.hi = b.hi.cwiseMax(p);
    b.empty = false;
  }
  return b;
}

/// Signed per-axis gap between two AABBs: negative on an axis means the boxes
/// overlap on that axis. The largest component is the separation distance
/// (0 or negative = boxes touch/overlap). Used to decide spatial adjacency of
/// two coplanar inlier patches (issue #215: split walls scanned as disjoint
/// halves must still merge).
double bbox_gap(const InlierBBox &a, const InlierBBox &b) {
  if (a.empty || b.empty)
    return std::numeric_limits<double>::infinity();
  const Eigen::Vector3d gap =
      (a.lo - b.hi).cwiseMax(b.lo - a.hi); // per-axis separation
  return gap.maxCoeff();
}

/// Pairwise plane distance used by the agglomerative clustering: a weighted
/// combination of the normal-angle disagreement (normalized by the angle
/// tolerance) and the offset difference along the average normal (normalized by
/// the coplanarity tolerance). Both terms are dimensionless after
/// normalization, so a value < 1 means "within tolerance on both axes".
///
/// The metric is orientation-SENSITIVE: +n and -n are not the same direction.
/// Plane normals carry the side the surface was observed from, so two parallel
/// planes facing away from each other are the two faces of one wall (a pair for
/// make_pairs), never a duplicate. Opposite-facing planes (dot < 0) therefore
/// get an infinite distance (reusex-z1v.2.4).
double plane_distance(const Eigen::Vector4d &Pi, const Eigen::Vector3d &Ci,
                      const Eigen::Vector4d &Pj, const Eigen::Vector3d &Cj,
                      double angle_tol_rad, double coplanar_tol_m) {
  const Eigen::Vector3d ni = Pi.head<3>();
  const Eigen::Vector3d nj = Pj.head<3>();
  const double dot = std::clamp(ni.dot(nj), -1.0, 1.0);
  if (dot < 0.0)
    return std::numeric_limits<double>::infinity(); // opposite-facing
  const double angle = std::acos(dot);

  // Offset difference along the average normal: the perpendicular distance
  // between the two planes at their shared centroid.
  Eigen::Vector3d n_avg = ni + nj;
  const double n_avg_norm = n_avg.norm();
  double offset_diff;
  if (n_avg_norm < reusex::kEpsilonRelative) {
    offset_diff = std::numeric_limits<double>::infinity();
  } else {
    n_avg /= n_avg_norm;
    const Eigen::Vector3d midpoint = 0.5 * (Ci + Cj);
    const double di = n_avg.dot(midpoint - Ci);
    const double dj = n_avg.dot(midpoint - Cj);
    offset_diff = std::abs(di - dj);
  }

  const double a = angle / angle_tol_rad;
  const double o = offset_diff / coplanar_tol_m;
  return std::sqrt(a * a + o * o);
}

} // namespace

auto merge_planes(EigenVectorContainer<double, 4> const &planes_,
                  std::vector<IndicesPtr> const &inliers_,
                  EigenVectorContainer<double, 3> const &centroids_,
                  CloudConstPtr cloud, const double angle_threshold,
                  const double distance_threshold,
                  const double horizontal_distance_threshold)
    -> std::tuple<EigenVectorContainer<double, 4>, std::vector<IndicesPtr>,
                  EigenVectorContainer<double, 3>> {
  reusex::trace("Merge planes (agglomerative) with angle threshold {} and "
                "distance threshold {}",
                angle_threshold, distance_threshold);

  if (planes_.size() != inliers_.size() || planes_.size() != centroids_.size())
    throw std::runtime_error(fmt::format(
        "merge_planes: input sizes disagree (planes={}, inliers={}, "
        "centroids={})",
        planes_.size(), inliers_.size(), centroids_.size()));

  const size_t n = planes_.size();

  // ── Tolerances (issue #215/#216) ─────────────────────────────────────────
  // `angle_threshold` historically expresses the allowable `1 - cos(theta)`
  // slack; convert it to an angle so the distance metric works in radians. The
  // offset/adjacency tolerances derive from the shared coplanarity constant and
  // the caller-provided distance threshold (both in meters).
  const double angle_tol_rad =
      std::max(std::acos(std::clamp(1.0 - angle_threshold, -1.0, 1.0)),
               reusex::kEpsilonAngleRad);
  const double coplanar_tol_m =
      std::max(distance_threshold, reusex::kToleranceCoplanarM);
  // Two patches are spatially adjacent if their inlier boxes overlap or the gap
  // between them is below this. Split-wall halves scanned from two viewpoints
  // sit within a plane thickness of each other.
  const double adjacency_gap_m = coplanar_tol_m;
  // Two floors or ceilings merge only within this height difference: a split
  // level's two floors, or a ceiling beside a lower sloped one, stand apart
  // by a few decimetres and must stay two planes (reusex-2kd).
  const double horizontal_tol_m =
      horizontal_distance_threshold > 0.0
          ? std::max(horizontal_distance_threshold, reusex::kToleranceCoplanarM)
          : std::numeric_limits<double>::infinity();
  // |n_z| above this: a floor or ceiling, as in force_orthogonal_planes.
  constexpr double kHorizontalNz = 0.9;
  // A pair may merge only when its (dimensionless) plane distance is below
  // this.
  constexpr double kMergeDistanceThreshold = 1.0;

  // ── Mutable working clusters ─────────────────────────────────────────────
  struct Cluster {
    Eigen::Vector4d plane;
    IndicesPtr inliers;
    Eigen::Vector3d centroid;
    size_t weight; // inlier count, drives weighted centroid updates
    InlierBBox bbox;
    size_t origin; // smallest original index in the cluster (stable ordering)
    bool alive;
  };

  std::vector<Cluster> clusters;
  clusters.reserve(n);
  for (size_t i = 0; i < n; ++i) {
    Eigen::Vector4d plane = planes_[i];
    validate_normalized(plane, "merge_planes", i);
    Cluster c;
    c.plane = plane;
    c.inliers = std::make_shared<Indices>(*inliers_[i]);
    c.centroid = centroids_[i];
    c.weight = std::max<size_t>(inliers_[i]->size(), 1);
    c.bbox = inlier_bbox(cloud, *c.inliers);
    c.origin = i;
    c.alive = true;
    clusters.push_back(std::move(c));
  }

  // A pair is mergeable iff it is coplanar (plane distance below threshold) AND
  // spatially adjacent (inlier boxes overlap or gap below adjacency).
  //
  // There is deliberately no inlier-overlap gate (#325). The old greedy pass
  // took a `min_overlap` ratio defaulting to 0.8 — the mutual fraction of each
  // patch's inliers lying within tolerance of the other's plane — and #215
  // dropped it because it kept split walls apart. Reinstating it was measured
  // on the office scan (38 planes / 255,744 points): the gate blocks 2 of 16
  // merges at 0.8, including a 41,967-point wall pair whose plane distance is
  // 0.384 (well inside the merge tolerance) but whose mutual overlap is 0.588,
  // because a long patch necessarily deviates from a slightly-rotated
  // neighbour's plane at its far end. Below 0.8 the gate changes nothing and
  // merely doubles the merge cost (9.6 ms -> 19.1 ms), so no threshold buys
  // anything the coplanarity + adjacency test does not already decide.
  //
  // Opposite-facing planes are never mergeable (reusex-z1v.2.4): before this
  // gate, plane_distance treated +n and -n alike, so the two faces of a
  // 10-25 cm wall (offset well inside the 0.5 m tolerance) collapsed into one
  // mid-wall plane, and make_pairs then invented a back face new_plane_offset
  // behind it — moving the wall and giving it the wrong thickness.
  auto mergeable = [&](const Cluster &a, const Cluster &b, double &dist) {
    if (a.plane.head<3>().dot(b.plane.head<3>()) < 0.0) {
      dist = std::numeric_limits<double>::infinity();
      return false;
    }
    dist = plane_distance(a.plane, a.centroid, b.plane, b.centroid,
                          angle_tol_rad, coplanar_tol_m);
    // Two floors or ceilings: their patches' heights must agree too. The
    // plane distance alone is measured at the midpoint of the centroids, where
    // a sloped ceiling (or the refit of one) passes close to a flat one it
    // runs up to, metres away from either patch (house1, reusex-2kd).
    if (std::abs(a.plane[2]) > kHorizontalNz &&
        std::abs(b.plane[2]) > kHorizontalNz &&
        std::abs(a.centroid.z() - b.centroid.z()) >= horizontal_tol_m)
      return false;
    if (dist >= kMergeDistanceThreshold)
      return false;
    return bbox_gap(a.bbox, b.bbox) <= adjacency_gap_m;
  };

  pcl::PCA<PointT> pca;
  pca.setInputCloud(cloud);

  // ── Agglomerative loop: repeatedly merge the globally closest valid pair ──
  // O(n^3) worst case, but plane counts here are tens, not thousands.
  while (true) {
    double best_dist = kMergeDistanceThreshold;
    size_t best_a = n, best_b = n;

    for (size_t i = 0; i < clusters.size(); ++i) {
      if (!clusters[i].alive)
        continue;
      for (size_t j = i + 1; j < clusters.size(); ++j) {
        if (!clusters[j].alive)
          continue;
        double dist;
        if (!mergeable(clusters[i], clusters[j], dist))
          continue;
        // Deterministic tie-break: lower origin indices win.
        const bool better =
            dist < best_dist - 1e-15 ||
            (std::abs(dist - best_dist) <= 1e-15 &&
             (best_a == n ||
              std::tie(clusters[i].origin, clusters[j].origin) <
                  std::tie(clusters[best_a].origin, clusters[best_b].origin)));
        if (better) {
          best_dist = dist;
          best_a = i;
          best_b = j;
        }
      }
    }

    if (best_a == n)
      break; // no pair under threshold — done

    Cluster &a = clusters[best_a];
    Cluster &b = clusters[best_b];

    // Merge inliers.
    IndicesPtr merged(new Indices);
    merged->reserve(a.inliers->size() + b.inliers->size());
    merged->insert(merged->end(), a.inliers->begin(), a.inliers->end());
    merged->insert(merged->end(), b.inliers->begin(), b.inliers->end());

    // Refit the plane from the combined inliers via PCA (least-squares plane).
    // Fall back to an inlier-count-weighted blend if PCA is unavailable (too
    // few points).
    Eigen::Vector3d normal;
    Eigen::Vector3d centroid;
    if (merged->size() >= 3) {
      pca.setIndices(merged);
      normal = pca.getEigenVectors().col(2).cast<double>();
      centroid = pca.getMean().head<3>().cast<double>();
    } else {
      // Inlier-count-weighted centroid + normal (weights = cluster sizes).
      const double wa = static_cast<double>(a.weight);
      const double wb = static_cast<double>(b.weight);
      centroid = (wa * a.centroid + wb * b.centroid) / (wa + wb);
      Eigen::Vector3d nb = b.plane.head<3>();
      if (a.plane.head<3>().dot(nb) < 0)
        nb = -nb;
      normal = (wa * a.plane.head<3>() + wb * nb);
    }

    const double nnorm = normal.norm();
    if (nnorm < reusex::kEpsilonLengthM) {
      // Degenerate refit: keep the heavier cluster's plane.
      normal = (a.weight >= b.weight ? a.plane : b.plane).head<3>();
    } else {
      normal /= nnorm;
    }
    // Keep orientation consistent with the heavier cluster.
    const Eigen::Vector3d ref =
        (a.weight >= b.weight ? a.plane : b.plane).head<3>();
    if (normal.dot(ref) < 0)
      normal = -normal;

    a.plane.head<3>() = normal;
    a.plane[3] = -normal.dot(centroid);
    a.inliers = merged;
    a.centroid = centroid;
    a.weight = a.weight + b.weight;
    a.bbox = inlier_bbox(cloud, *merged);
    a.origin = std::min(a.origin, b.origin);
    b.alive = false;
  }

  // ── Collect surviving clusters, ordered deterministically by origin ──────
  std::vector<size_t> order;
  for (size_t i = 0; i < clusters.size(); ++i)
    if (clusters[i].alive)
      order.push_back(i);
  std::sort(order.begin(), order.end(), [&](size_t l, size_t r) {
    return clusters[l].origin < clusters[r].origin;
  });

  auto Pm = EigenVectorContainer<double, 4>();
  auto Im = std::vector<IndicesPtr>();
  auto Cm = EigenVectorContainer<double, 3>();
  Pm.reserve(order.size());
  Im.reserve(order.size());
  Cm.reserve(order.size());
  for (size_t i : order) {
    Pm.push_back(clusters[i].plane);
    Im.push_back(clusters[i].inliers);
    Cm.push_back(clusters[i].centroid);
  }

  reusex::debug("merge_planes: {} planes -> {} clusters", n, Pm.size());
  return std::make_tuple(Pm, Im, Cm);
}

auto separate_planes(const EigenVectorContainer<double, 4> &planes,
                     const Eigen::Vector3d &up, const double epsilon)
    -> std::tuple<std::vector<size_t>, std::vector<size_t>> {

  reusex::trace("Separate planes into vertical and horizontal planes");

  std::vector<size_t> vertical{};
  std::vector<size_t> horizontal{};

  for (size_t i = 0; i < planes.size(); ++i) {
    auto &plane = planes[i];
    const double dot_prod = plane.head<3>().dot(up);

    if (std::abs(dot_prod) < epsilon) // Wall
      vertical.push_back(i);
    else if (std::abs(dot_prod - 1.0) < epsilon) // Floor
      horizontal.push_back(i);
    else if (std::abs(dot_prod + 1.0) < epsilon) // Ceiling
      horizontal.push_back(i);
    else
      reusex::warn("Plane {} is not vertical or horizontal", i);
  }
  return std::make_tuple(vertical, horizontal);
}

} // namespace reusex::geometry
