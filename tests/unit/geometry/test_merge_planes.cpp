// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Unit tests for agglomerative plane merging (issue #215).
//
// merge_planes() clusters planes that are both coplanar (normal + offset within
// tolerance) AND spatially adjacent (inlier boxes overlap or gap below
// tolerance). It replaces the old greedy first-match pass whose 0.8 inlier
// overlap gate kept split walls apart. These tests pin the intended behavior:
//   - split wall (same plane, disjoint halves)      -> MERGES
//   - perpendicular planes                           -> NEVER merge
//   - parallel-but-offset walls (> tolerance apart)  -> NEVER merge
//   - coplanar+adjacent pair with low mutual inlier overlap -> MERGES (#325)
//   - the two faces of one thin wall (opposite normals)   -> NEVER merge
//   - result ordering is deterministic

#include <reusex/geometry/utils.hpp>
#include <reusex/types.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>

#include <cmath>
#include <numbers>

using namespace reusex;
using namespace reusex::geometry;

namespace {

/// Append a rectangular grid of points on the plane x = x0, spanning
/// y in [y0, y1] and z in [z0, z1], to `cloud`. Returns the inlier indices.
IndicesPtr add_x_wall_patch(CloudPtr &cloud, double x0, double y0, double y1,
                            double z0, double z1, double step = 0.05) {
  IndicesPtr idx(new Indices);
  for (double y = y0; y <= y1 + 1e-9; y += step) {
    for (double z = z0; z <= z1 + 1e-9; z += step) {
      PointT p;
      p.x = static_cast<float>(x0);
      p.y = static_cast<float>(y);
      p.z = static_cast<float>(z);
      idx->push_back(static_cast<int>(cloud->size()));
      cloud->push_back(p);
    }
  }
  return idx;
}

/// Append a patch lying on the plane through (x_hinge, ·, 0) tilted by
/// `tilt_rad` about the y axis: z = tan(tilt) * (x - x_hinge). Spans
/// x in [x0, x1] and y in [y0, y1].
IndicesPtr add_tilted_z_patch(CloudPtr &cloud, double x0, double x1, double y0,
                              double y1, double x_hinge, double tilt_rad,
                              double step = 0.5) {
  IndicesPtr idx(new Indices);
  const double slope = std::tan(tilt_rad);
  for (double x = x0; x <= x1 + 1e-9; x += step) {
    for (double y = y0; y <= y1 + 1e-9; y += step) {
      PointT p;
      p.x = static_cast<float>(x);
      p.y = static_cast<float>(y);
      p.z = static_cast<float>(slope * (x - x_hinge));
      idx->push_back(static_cast<int>(cloud->size()));
      cloud->push_back(p);
    }
  }
  return idx;
}

Eigen::Vector3d centroid_of(const CloudPtr &cloud, const IndicesPtr &idx) {
  Eigen::Vector3d c = Eigen::Vector3d::Zero();
  for (int i : *idx)
    c += cloud->points[i].getVector3fMap().cast<double>();
  return c / static_cast<double>(idx->size());
}

} // namespace

TEST_CASE("MergePlanes_SplitWallDisjointHalves_Merges",
          "[geometry][merge_planes]") {
  auto cloud = std::make_shared<Cloud>();

  // Two disjoint halves of the SAME wall x = 0: one at y in [0,2], the other at
  // y in [2.5,4.5]. Same plane, a small gap between the inlier patches.
  auto ia = add_x_wall_patch(cloud, 0.0, 0.0, 2.0, 0.0, 2.5);
  auto ib = add_x_wall_patch(cloud, 0.0, 2.5, 4.5, 0.0, 2.5);
  cloud->width = static_cast<uint32_t>(cloud->size());
  cloud->height = 1;

  EigenVectorContainer<double, 4> planes{Eigen::Vector4d(1, 0, 0, 0),
                                         Eigen::Vector4d(1, 0, 0, 0)};
  std::vector<IndicesPtr> inliers{ia, ib};
  EigenVectorContainer<double, 3> centroids{centroid_of(cloud, ia),
                                            centroid_of(cloud, ib)};

  auto [Pm, Im, Cm] = merge_planes(planes, inliers, centroids, cloud);

  REQUIRE(Pm.size() == 1);
  REQUIRE(Im.size() == 1);
  REQUIRE(Cm.size() == 1);
  // Merged inlier list is the union of both halves.
  REQUIRE(Im[0]->size() == ia->size() + ib->size());
  // Normal still points along +x, plane still at x = 0.
  Eigen::Vector3d n = Pm[0].head<3>().normalized();
  REQUIRE(std::abs(n.x()) == Catch::Approx(1.0).margin(1e-6));
  REQUIRE(std::abs(Pm[0][3] / Pm[0].head<3>().norm()) ==
          Catch::Approx(0.0).margin(1e-3));
}

TEST_CASE("MergePlanes_PerpendicularPlanes_NeverMerge",
          "[geometry][merge_planes]") {
  auto cloud = std::make_shared<Cloud>();

  // Wall x = 0 and wall y = 0 (perpendicular), meeting at the corner.
  auto ix = add_x_wall_patch(cloud, 0.0, 0.0, 3.0, 0.0, 2.5);
  IndicesPtr iy(new Indices);
  for (double x = 0.0; x <= 3.0 + 1e-9; x += 0.05)
    for (double z = 0.0; z <= 2.5 + 1e-9; z += 0.05) {
      PointT p;
      p.x = static_cast<float>(x);
      p.y = 0.0F;
      p.z = static_cast<float>(z);
      iy->push_back(static_cast<int>(cloud->size()));
      cloud->push_back(p);
    }
  cloud->width = static_cast<uint32_t>(cloud->size());
  cloud->height = 1;

  EigenVectorContainer<double, 4> planes{Eigen::Vector4d(1, 0, 0, 0),
                                         Eigen::Vector4d(0, 1, 0, 0)};
  std::vector<IndicesPtr> inliers{ix, iy};
  EigenVectorContainer<double, 3> centroids{centroid_of(cloud, ix),
                                            centroid_of(cloud, iy)};

  auto [Pm, Im, Cm] = merge_planes(planes, inliers, centroids, cloud);

  REQUIRE(Pm.size() == 2);
}

TEST_CASE("MergePlanes_ParallelOffsetWalls_NeverMerge",
          "[geometry][merge_planes]") {
  auto cloud = std::make_shared<Cloud>();

  // Two parallel walls x = 0 and x = 3 (3 m apart, far beyond tolerance).
  auto i0 = add_x_wall_patch(cloud, 0.0, 0.0, 3.0, 0.0, 2.5);
  auto i3 = add_x_wall_patch(cloud, 3.0, 0.0, 3.0, 0.0, 2.5);
  cloud->width = static_cast<uint32_t>(cloud->size());
  cloud->height = 1;

  EigenVectorContainer<double, 4> planes{Eigen::Vector4d(1, 0, 0, 0),
                                         Eigen::Vector4d(1, 0, 0, -3.0)};
  std::vector<IndicesPtr> inliers{i0, i3};
  EigenVectorContainer<double, 3> centroids{centroid_of(cloud, i0),
                                            centroid_of(cloud, i3)};

  auto [Pm, Im, Cm] = merge_planes(planes, inliers, centroids, cloud);

  REQUIRE(Pm.size() == 2);
}

TEST_CASE("MergePlanes_CoplanarAdjacentWithLowInlierOverlap_StillMerges",
          "[geometry][merge_planes]") {
  // Regression guard for #325: merge_planes has no inlier-overlap gate, and
  // must not grow one back. Two 20 m long patches, one flat (z = 0) and one
  // tilted 10 deg about y, share a hinge at x = 10 and sit 0.2 m apart in y.
  // They are coplanar within tolerance and spatially adjacent, so they merge —
  // even though only ~29 % of either patch's points lie within tolerance of the
  // other patch's plane (the far ends drift off it), which the removed
  // `min_overlap = 0.8` gate would have rejected.
  auto cloud = std::make_shared<Cloud>();

  constexpr double kTilt = 10.0 * std::numbers::pi / 180.0;
  auto flat = add_tilted_z_patch(cloud, 0.0, 20.0, 0.0, 1.0, 10.0, 0.0);
  auto tilted = add_tilted_z_patch(cloud, 0.0, 20.0, 1.2, 2.2, 10.0, kTilt);
  cloud->width = static_cast<uint32_t>(cloud->size());
  cloud->height = 1;

  EigenVectorContainer<double, 4> planes{
      Eigen::Vector4d(0, 0, 1, 0),
      Eigen::Vector4d(-std::sin(kTilt), 0, std::cos(kTilt),
                      10.0 * std::sin(kTilt))};
  std::vector<IndicesPtr> inliers{flat, tilted};
  EigenVectorContainer<double, 3> centroids{centroid_of(cloud, flat),
                                            centroid_of(cloud, tilted)};

  auto [Pm, Im, Cm] = merge_planes(planes, inliers, centroids, cloud);

  REQUIRE(Pm.size() == 1);
  REQUIRE(Im[0]->size() == flat->size() + tilted->size());
}

TEST_CASE("MergePlanes_RepeatedRuns_ProducesDeterministicOrdering",
          "[geometry][merge_planes]") {
  auto cloud = std::make_shared<Cloud>();

  // Three planes: two coplanar+adjacent halves of wall x=0, plus a distinct
  // wall x=3. The two halves must merge; the result must be stable across runs.
  auto ia = add_x_wall_patch(cloud, 0.0, 0.0, 2.0, 0.0, 2.5);
  auto ib = add_x_wall_patch(cloud, 0.0, 2.5, 4.5, 0.0, 2.5);
  auto ic = add_x_wall_patch(cloud, 3.0, 0.0, 4.5, 0.0, 2.5);
  cloud->width = static_cast<uint32_t>(cloud->size());
  cloud->height = 1;

  EigenVectorContainer<double, 4> planes{Eigen::Vector4d(1, 0, 0, 0),
                                         Eigen::Vector4d(1, 0, 0, 0),
                                         Eigen::Vector4d(1, 0, 0, -3.0)};
  std::vector<IndicesPtr> inliers{ia, ib, ic};
  EigenVectorContainer<double, 3> centroids{
      centroid_of(cloud, ia), centroid_of(cloud, ib), centroid_of(cloud, ic)};

  auto run = [&]() {
    auto [Pm, Im, Cm] = merge_planes(planes, inliers, centroids, cloud);
    std::vector<size_t> sizes;
    for (const auto &I : Im)
      sizes.push_back(I->size());
    return sizes;
  };

  auto first = run();
  REQUIRE(first.size() == 2); // halves merged, distant wall separate
  for (int rep = 0; rep < 3; ++rep)
    REQUIRE(run() == first); // identical output every run
}

namespace {

/// The two faces of one 0.15 m thick wall occupying x in [0, 0.15], scanned
/// from both sides. Normals are oriented toward the sensor (segment_planes
/// flips each plane normal to agree with the viewpoint-oriented point
/// normals), so they point away from each other: -x on the face at x = 0, +x
/// on the face at x = 0.15.
struct WallFaces {
  CloudPtr cloud = std::make_shared<Cloud>();
  EigenVectorContainer<double, 4> planes;
  std::vector<IndicesPtr> inliers;
  EigenVectorContainer<double, 3> centroids;
};

WallFaces two_faces_of_one_wall(double thickness = 0.15) {
  WallFaces w;
  auto near = add_x_wall_patch(w.cloud, 0.0, 0.0, 3.0, 0.0, 2.5);
  auto far = add_x_wall_patch(w.cloud, thickness, 0.0, 3.0, 0.0, 2.5);
  w.cloud->width = static_cast<uint32_t>(w.cloud->size());
  w.cloud->height = 1;
  w.planes = {Eigen::Vector4d(-1, 0, 0, 0),
              Eigen::Vector4d(1, 0, 0, -thickness)};
  w.inliers = {near, far};
  w.centroids = {centroid_of(w.cloud, near), centroid_of(w.cloud, far)};
  return w;
}

} // namespace

TEST_CASE("MergePlanes_TwoFacesOfThinWall_NeverMerge",
          "[geometry][merge_planes]") {
  // reusex-z1v.2.4: plane_distance treats +n and -n as the same direction, and
  // 0.15 m is well inside the 0.5 m offset tolerance, so the two faces of one
  // wall used to collapse into a single plane. They are a wall pair, not a
  // duplicate: opposite-facing planes must never merge.
  for (const double thickness : {0.10, 0.15, 0.25}) {
    CAPTURE(thickness);
    auto w = two_faces_of_one_wall(thickness);

    auto [Pm, Im, Cm] = merge_planes(w.planes, w.inliers, w.centroids, w.cloud);

    REQUIRE(Pm.size() == 2);
    REQUIRE(Im[0]->size() == w.inliers[0]->size());
    REQUIRE(Im[1]->size() == w.inliers[1]->size());
    // Each face keeps its own orientation.
    REQUIRE(Pm[0].head<3>().dot(w.planes[0].head<3>()) ==
            Catch::Approx(1.0).margin(1e-6));
    REQUIRE(Pm[1].head<3>().dot(w.planes[1].head<3>()) ==
            Catch::Approx(1.0).margin(1e-6));
  }
}

TEST_CASE("MergePlanes_TwoFacesOfThinWall_PairWithoutInventedBackFace",
          "[geometry][merge_planes][make_pairs]") {
  // Downstream of the merge (reconstruction/mesh.cpp): make_pairs must pair the
  // two measured faces with each other. Had they been merged, make_pairs would
  // find no opposite partner and invent a back face at new_plane_offset.
  auto w = two_faces_of_one_wall(0.15);

  auto [Pm, Im, Cm] = merge_planes(w.planes, w.inliers, w.centroids, w.cloud);
  REQUIRE(Pm.size() == 2);

  auto pairs = make_pairs(Pm, Im, Cm, /*threshold=*/0.60,
                          /*new_plane_offset=*/0.25);

  REQUIRE(Pm.size() == 2); // no invented plane appended
  REQUIRE(pairs.size() == 1);
  REQUIRE(pairs[0] == std::pair<size_t, size_t>{0, 1});
}
