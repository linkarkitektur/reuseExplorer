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
#include <memory>
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

TEST_CASE("MakePairs_SlightlySkewedFaces_PairBecomesParallelSlab",
          "[geometry][make_pairs]") {
  // A long, well-sampled wall face (x = 0, normal +x, 27 m) and a short face
  // of the same wall 0.4 m behind it, measured 4 degrees off (NewOffice: the
  // long-wing facade, 30k inliers, paired with a 6k-inlier face at the
  // kitchen). Both are wall-line candidates, so the pair's two planes become
  // arrangement lines across the whole building: left skewed, the slab
  // between them widens by 0.07 m per metre, reaching ~1 m at the far end of
  // the facade, and the mesh overshoots the facade there (reusex-bi6).
  // make_pairs must turn the weaker face (fewer inliers) to be exactly
  // antiparallel to the stronger one, keeping it through its own centroid.
  EigenVectorContainer<double, 4> planes;
  EigenVectorContainer<double, 3> centroids;
  std::vector<IndicesPtr> inliers;

  planes.emplace_back(1.0, 0.0, 0.0, 0.0); // x = 0, normal +x
  centroids.emplace_back(0.0, 13.5, 1.5);
  inliers.push_back(IndicesPtr(new Indices(3000, 0)));

  const double a = 4.0 * std::numbers::pi / 180.0;
  const Eigen::Vector3d n_b(-std::cos(a), std::sin(a), 0.0); // ~ -x, skewed
  const Eigen::Vector3d c_b(-0.4, 17.0, 1.5);
  planes.emplace_back(n_b.x(), n_b.y(), n_b.z(), -n_b.dot(c_b));
  centroids.push_back(c_b);
  inliers.push_back(IndicesPtr(new Indices(600, 0)));

  auto pairs = make_pairs(planes, inliers, centroids, /*threshold=*/0.60,
                          /*new_plane_offset=*/0.25);

  REQUIRE(planes.size() == 2);
  REQUIRE(pairs.size() == 1);
  // The strong face is untouched.
  CHECK(planes[0].isApprox(Eigen::Vector4d(1.0, 0.0, 0.0, 0.0)));
  // The weak face is antiparallel to it and still passes through its centroid.
  CHECK(planes[1].head<3>().dot(planes[0].head<3>()) ==
        Catch::Approx(-1.0).margin(1e-9));
  CHECK(planes[1].head<3>().dot(c_b) + planes[1][3] ==
        Catch::Approx(0.0).margin(1e-9));
}

TEST_CASE("MakePairs_CrossedFacesOfOneWall_SlabSpansTheirOverlap",
          "[geometry][make_pairs]") {
  // house1's partition i-n1 (reusex-wm0): the hall's face of the wall
  // (normal +x) was measured 0.09 m WEST of the entrance's face (normal -x):
  // the two rooms' scans drifted apart by more than the wall is thick, so the
  // faces cross and each one's front looks at the other. The cell complex
  // puts a wall cell where a cell is behind BOTH faces of a pair; for a
  // crossed pair there is no such place, so the mesh had no wall to put
  // between the two rooms and merged them. make_pairs must still pair the
  // faces, and turn them into a slab between the two measured positions,
  // each face keeping its normal (towards its own room).
  EigenVectorContainer<double, 4> planes;
  EigenVectorContainer<double, 3> centroids;
  std::vector<IndicesPtr> inliers;

  const Eigen::Vector3d c_east(1.85, -1.1, 0.0); // the hall's face, +x
  planes.emplace_back(1.0, 0.0, 0.0, -c_east.x());
  centroids.push_back(c_east);
  inliers.push_back(IndicesPtr(new Indices(1700, 0)));

  const Eigen::Vector3d c_west(1.94, -0.6, 0.0); // the entrance's face, -x
  planes.emplace_back(-1.0, 0.0, 0.0, c_west.x());
  centroids.push_back(c_west);
  inliers.push_back(IndicesPtr(new Indices(3000, 0)));

  auto pairs = make_pairs(planes, inliers, centroids, /*threshold=*/0.60,
                          /*new_plane_offset=*/0.25);

  REQUIRE(planes.size() == 2); // paired, no invented back face
  REQUIRE(pairs.size() == 1);
  // Normals unchanged: each face still faces its own room.
  CHECK(planes[0].head<3>().isApprox(Eigen::Vector3d(1, 0, 0)));
  CHECK(planes[1].head<3>().isApprox(Eigen::Vector3d(-1, 0, 0)));
  // A slab: a point between the measured positions is behind both faces.
  const Eigen::Vector3d mid(0.5 * (c_east.x() + c_west.x()), 0.0, 0.0);
  CHECK(dist_plane_point(planes[0], mid) < 0.0);
  CHECK(dist_plane_point(planes[1], mid) < 0.0);
  // It spans the stretch the two scans overlap: x in [1.85, 1.94].
  CHECK(-planes[0][3] == Catch::Approx(c_west.x()).margin(1e-9));
  CHECK(planes[1][3] == Catch::Approx(c_east.x()).margin(1e-9));
}

TEST_CASE("MakePairs_FacesLookingAtEachOtherAcrossAGap_StayWhereMeasured",
          "[geometry][make_pairs]") {
  // Two faces 0.3 m apart whose normals point at each other bound a gap a
  // sensor looked into (a shaft, a narrow passage), not one drifted wall
  // (walls' max_crossed_gap_m, 0.15 m): make_pairs must not move them onto
  // each other.
  EigenVectorContainer<double, 4> planes;
  EigenVectorContainer<double, 3> centroids;
  std::vector<IndicesPtr> inliers;
  planes.emplace_back(1.0, 0.0, 0.0, -1.0); // x = 1.0, normal +x
  centroids.emplace_back(1.0, 0.0, 0.0);
  inliers.push_back(IndicesPtr(new Indices(1000, 0)));
  planes.emplace_back(-1.0, 0.0, 0.0, 1.3); // x = 1.3, normal -x
  centroids.emplace_back(1.3, 0.0, 0.0);
  inliers.push_back(IndicesPtr(new Indices(2000, 0)));

  make_pairs(planes, inliers, centroids, /*threshold=*/0.60,
             /*new_plane_offset=*/0.25);

  CHECK(planes[0].isApprox(Eigen::Vector4d(1.0, 0.0, 0.0, -1.0)));
  CHECK(planes[1].isApprox(Eigen::Vector4d(-1.0, 0.0, 0.0, 1.3)));
}

TEST_CASE("MakePairs_FacesLookingAtEachOtherAcrossAGap_AreNoWall",
          "[geometry][make_pairs]") {
  // A pair is a wall: the slab behind both faces. Two faces whose fronts
  // look at each other across more than max_crossed_gap stay where measured,
  // so nothing lies behind both and the pair would be a wall nowhere
  // (house2's west-room partition and a north room's wall 0.23 m apart,
  // reusex-ro1). Each must get a pair that bounds a slab: here, with no
  // other candidate, an invented back face behind it.
  EigenVectorContainer<double, 4> planes;
  EigenVectorContainer<double, 3> centroids;
  std::vector<IndicesPtr> inliers;
  planes.emplace_back(-1.0, 0.0, 0.0, 3.0); // x = 3.0, normal -x
  centroids.emplace_back(3.0, 1.5, 1.25);
  inliers.push_back(IndicesPtr(new Indices(500, 0)));
  planes.emplace_back(1.0, 0.0, 0.0, -2.77); // x = 2.77, normal +x
  centroids.emplace_back(2.77, 4.5, 1.25);
  inliers.push_back(IndicesPtr(new Indices(800, 0)));

  auto pairs = make_pairs(planes, inliers, centroids, /*threshold=*/0.60,
                          /*new_plane_offset=*/0.25);

  REQUIRE(pairs.size() == 2);
  REQUIRE(planes.size() == 4);
  for (const auto &[a, b] : pairs) {
    // Some point lies behind both faces of every pair.
    const Eigen::Vector3d mid = 0.5 * (centroids[a] + centroids[b]);
    const Eigen::Vector3d probe(mid.x(), 0.0, 0.0);
    INFO("pair " << a << "," << b);
    CHECK(dist_plane_point(planes[a], probe) < 0.0);
    CHECK(dist_plane_point(planes[b], probe) < 0.0);
  }
  // Neither measured face moved.
  CHECK(planes[0].isApprox(Eigen::Vector4d(-1.0, 0.0, 0.0, 3.0)));
  CHECK(planes[1].isApprox(Eigen::Vector4d(1.0, 0.0, 0.0, -2.77)));
}

TEST_CASE("MakePairs_FacesOfTwoStoreysWalls_AreNoWall",
          "[geometry][make_pairs]") {
  // house2 (reusex-ro1): the ground floor's partition (normal -x, z 0..2.4)
  // and an upper-storey wall's face (normal +x, z 2.7..5.0, 4 degrees off)
  // lie back to back 0.2 m apart on one line. Without the cloud they pair,
  // and the weaker face is turned onto the other's angle; with it, their
  // heights do not overlap, so they are no wall and keep their angles.
  auto cloud = std::make_shared<Cloud>();
  auto face = [&](double x, double y0, double y1, double z0, double z1) {
    IndicesPtr idx(new Indices);
    for (double y = y0; y <= y1 + 1e-9; y += 0.1)
      for (double z = z0; z <= z1 + 1e-9; z += 0.1) {
        PointT p;
        p.x = static_cast<float>(x);
        p.y = static_cast<float>(y);
        p.z = static_cast<float>(z);
        idx->push_back(static_cast<int>(cloud->size()));
        cloud->push_back(p);
      }
    return idx;
  };
  const double a = 4.0 * std::numbers::pi / 180.0;
  auto setup = [&](EigenVectorContainer<double, 4> &planes,
                   EigenVectorContainer<double, 3> &centroids,
                   std::vector<IndicesPtr> &inliers) {
    planes.emplace_back(-1.0, 0.0, 0.0, 3.0); // x = 3.0, normal -x
    centroids.emplace_back(3.0, 2.5, 1.2);
    inliers.push_back(face(3.0, 1.0, 4.0, 0.0, 2.4));
    const Eigen::Vector3d n(std::cos(a), std::sin(a), 0.0);
    const Eigen::Vector3d c(3.2, 7.0, 3.85);
    planes.emplace_back(n.x(), n.y(), n.z(), -n.dot(c));
    centroids.push_back(c);
    inliers.push_back(face(3.2, 5.0, 9.0, 2.7, 5.0));
  };

  EigenVectorContainer<double, 4> planes;
  EigenVectorContainer<double, 3> centroids;
  std::vector<IndicesPtr> inliers;
  setup(planes, centroids, inliers);
  cloud->width = static_cast<uint32_t>(cloud->size());
  cloud->height = 1;

  SECTION("without the cloud they pair") {
    auto pairs = make_pairs(planes, inliers, centroids, 0.60, 0.25, 0.15);
    CHECK(pairs.size() == 1);
  }
  SECTION("with the cloud each gets its own back face") {
    const Eigen::Vector4d p0 = planes[0];
    auto pairs =
        make_pairs(planes, inliers, centroids, 0.60, 0.25, 0.15, cloud);
    CHECK(pairs.size() == 2);
    CHECK(planes.size() == 4);
    CHECK(planes[0].isApprox(p0)); // not turned onto the other's angle
  }
}

namespace {

/// Append a ceiling patch over x in [x0, x1], y in [y0, y1] at height
/// z(x) = z0 + rise_per_m * (x - x0); returns the inliers and sets `plane`
/// to the patch's plane with a downward normal.
IndicesPtr add_ceiling_patch(CloudPtr &cloud, double x0, double x1, double y0,
                             double y1, double z0, double rise_per_m,
                             Eigen::Vector4d &plane, double step = 0.1) {
  IndicesPtr idx(new Indices);
  for (double x = x0; x <= x1 + 1e-9; x += step)
    for (double y = y0; y <= y1 + 1e-9; y += step) {
      PointT p;
      p.x = static_cast<float>(x);
      p.y = static_cast<float>(y);
      p.z = static_cast<float>(z0 + rise_per_m * (x - x0));
      idx->push_back(static_cast<int>(cloud->size()));
      cloud->push_back(p);
    }
  Eigen::Vector3d n(rise_per_m, 0.0, -1.0);
  n.normalize();
  plane.head<3>() = n;
  plane[3] = -n.dot(Eigen::Vector3d(x0, y0, z0));
  return idx;
}

} // namespace

TEST_CASE("MergePlanes_CeilingBesideLowerSlopedOne_StaysApart",
          "[geometry][merge_planes]") {
  // house1 (reusex-2kd): a mono-pitch ceiling rising from 2.38 m to ~3.0 m
  // runs up to a flat ceiling at 3.09 m. Their plane distance, measured at
  // the midpoint of the centroids where the slope nearly reaches the flat
  // ceiling, is within the 0.5 m merge distance, so the two merged into one
  // ceiling between them (0.26 m below the flat one). With a horizontal
  // merge distance the patches' heights, 0.4 m apart, keep them apart.
  auto cloud = std::make_shared<Cloud>();
  Eigen::Vector4d slope_plane, flat_plane;
  auto is =
      add_ceiling_patch(cloud, 0.0, 5.0, 0.0, 4.0, 2.38,
                        std::tan(7.0 * std::numbers::pi / 180.0), slope_plane);
  auto iflat =
      add_ceiling_patch(cloud, 5.1, 10.0, 0.0, 4.0, 3.09, 0.0, flat_plane);
  cloud->width = static_cast<uint32_t>(cloud->size());
  cloud->height = 1;
  EigenVectorContainer<double, 4> planes{slope_plane, flat_plane};
  std::vector<IndicesPtr> inliers{is, iflat};
  EigenVectorContainer<double, 3> centroids{centroid_of(cloud, is),
                                            centroid_of(cloud, iflat)};

  SECTION("the default merge distance alone merges them") {
    auto [Pm, Im, Cm] = merge_planes(planes, inliers, centroids, cloud);
    CHECK(Pm.size() == 1);
  }
  SECTION("a horizontal merge distance keeps them apart") {
    auto [Pm, Im, Cm] =
        merge_planes(planes, inliers, centroids, cloud, 0.1, 0.5, 0.2);
    REQUIRE(Pm.size() == 2);
    CHECK(Im[1]->size() == iflat->size());
    CHECK(-Pm[1][3] / Pm[1][2] == Catch::Approx(3.09).margin(0.01));
  }
}

TEST_CASE("MergePlanes_CeilingPatchesAtOneHeight_StillMerge",
          "[geometry][merge_planes]") {
  // Two patches of one flat ceiling, 4 cm apart in height (drift between
  // two rooms' scans), still merge under the horizontal merge distance.
  auto cloud = std::make_shared<Cloud>();
  Eigen::Vector4d pa, pb;
  auto ia = add_ceiling_patch(cloud, 0.0, 4.0, 0.0, 4.0, 2.50, 0.0, pa);
  auto ib = add_ceiling_patch(cloud, 4.2, 8.0, 0.0, 4.0, 2.54, 0.0, pb);
  cloud->width = static_cast<uint32_t>(cloud->size());
  cloud->height = 1;
  EigenVectorContainer<double, 4> planes{pa, pb};
  std::vector<IndicesPtr> inliers{ia, ib};
  EigenVectorContainer<double, 3> centroids{centroid_of(cloud, ia),
                                            centroid_of(cloud, ib)};
  auto [Pm, Im, Cm] =
      merge_planes(planes, inliers, centroids, cloud, 0.1, 0.5, 0.2);
  CHECK(Pm.size() == 1);
}
