// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <reusex/geometry/point_mesh_distance.hpp>

#include <pcl/PolygonMesh.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cmath>
#include <stdexcept>

using Catch::Matchers::WithinAbs;
using reusex::geometry::mesh_support;
using reusex::geometry::point_to_mesh_distances;
using reusex::geometry::summarize_distances;

namespace {

/// The unit-free 2 x 2 quad z = 0, as one 4-gon (fanned by the helper).
pcl::PolygonMesh quad_mesh() {
  pcl::PointCloud<pcl::PointXYZ> verts;
  verts.push_back({0, 0, 0});
  verts.push_back({2, 0, 0});
  verts.push_back({2, 2, 0});
  verts.push_back({0, 2, 0});
  pcl::PolygonMesh mesh;
  pcl::toPCLPointCloud2(verts, mesh.cloud);
  pcl::Vertices quad;
  quad.vertices = {0, 1, 2, 3};
  mesh.polygons.push_back(quad);
  return mesh;
}

reusex::PointT at(float x, float y, float z) {
  reusex::PointT p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

} // namespace

TEST_CASE("PointToMeshDistances_IndexAlignedWithNaNForNonFinite",
          "[geometry][point_mesh_distance]") {
  reusex::Cloud cloud;
  cloud.push_back(at(1, 1, 0.03F));
  cloud.push_back(at(1, 1, -0.5F));
  cloud.push_back(at(std::nanf(""), 0, 0));
  cloud.push_back(at(3, 1, 0)); // 1 m beyond the quad's x = 2 edge
  const auto d = point_to_mesh_distances(cloud, quad_mesh());
  REQUIRE(d.size() == 4);
  CHECK_THAT(d[0], WithinAbs(0.03, 1e-6));
  CHECK_THAT(d[1], WithinAbs(0.5, 1e-6));
  CHECK(std::isnan(d[2]));
  CHECK_THAT(d[3], WithinAbs(1.0, 1e-6));
}

TEST_CASE("PointToMeshDistances_NoTriangle_Throws",
          "[geometry][point_mesh_distance]") {
  reusex::Cloud cloud;
  cloud.push_back(at(0, 0, 0));
  CHECK_THROWS_AS(point_to_mesh_distances(cloud, pcl::PolygonMesh{}),
                  std::invalid_argument);
}

TEST_CASE("SummarizeDistances_MeanP95AndThresholdShares",
          "[geometry][point_mesh_distance]") {
  // 100 distances 0.01 .. 1.00 plus one NaN (ignored).
  std::vector<double> d;
  for (int i = 1; i <= 100; ++i)
    d.push_back(0.01 * i);
  d.push_back(std::nan(""));
  const auto s = summarize_distances(d, 0.055, 0.205); // off-grid thresholds
  CHECK(s.points == 100);
  CHECK_THAT(s.mean_m, WithinAbs(0.505, 1e-9));
  CHECK_THAT(s.median_m, WithinAbs(0.50, 1e-9));
  CHECK_THAT(s.p95_m, WithinAbs(0.95, 1e-9));
  CHECK_THAT(s.max_m, WithinAbs(1.00, 1e-9));
  CHECK_THAT(s.within_share, WithinAbs(0.05, 1e-9));
  CHECK_THAT(s.beyond_share, WithinAbs(0.80, 1e-9));

  const auto empty = summarize_distances({std::nan("")}, 0.05, 0.2);
  CHECK(empty.points == 0);
}

TEST_CASE("MeshSupport_CloudOverHalfTheQuad_HalfTheAreaIsUnsupported",
          "[geometry][point_mesh_distance]") {
  // Points every 5 cm over x in [0, 1] only: the x > 1 half of the 2 x 2
  // quad has no scan point within 0.25 m except a 0.25 m strip at x = 1.
  reusex::Cloud cloud;
  for (int i = 0; i <= 20; ++i)
    for (int j = 0; j <= 40; ++j)
      cloud.push_back(at(0.05F * i, 0.05F * j, 0.02F));
  cloud.push_back(at(std::nanf(""), 0, 0)); // ignored
  const auto s = mesh_support(cloud, quad_mesh(), 0.25, 0.02);
  CHECK_THAT(s.area_m2, WithinAbs(4.0, 1e-9));
  // unsupported: x in (1.25, 2] = 0.75 of the 2 m width -> 1.5 m^2
  CHECK_THAT(s.unsupported_area_m2, WithinAbs(1.5, 0.03));
  CHECK(s.samples >= 9000); // ~ area / spacing^2

  // Deterministic: the same inputs give the same numbers.
  const auto t = mesh_support(cloud, quad_mesh(), 0.25, 0.02);
  CHECK(t.unsupported_area_m2 == s.unsupported_area_m2);
  CHECK(t.samples == s.samples);
}

TEST_CASE("MeshSupport_NoCloudNearTheMesh_EverythingUnsupported",
          "[geometry][point_mesh_distance]") {
  reusex::Cloud cloud;
  cloud.push_back(at(10, 10, 10));
  const auto s = mesh_support(cloud, quad_mesh(), 0.25, 0.1);
  CHECK_THAT(s.unsupported_area_m2, WithinAbs(4.0, 1e-9));
}

TEST_CASE("MeshSupport_BadInputs_Throw", "[geometry][point_mesh_distance]") {
  reusex::Cloud cloud;
  cloud.push_back(at(0, 0, 0));
  CHECK_THROWS_AS(mesh_support(cloud, pcl::PolygonMesh{}, 0.25, 0.05),
                  std::invalid_argument);
  CHECK_THROWS_AS(mesh_support(cloud, quad_mesh(), 0.0, 0.05),
                  std::invalid_argument);
  CHECK_THROWS_AS(mesh_support(cloud, quad_mesh(), 0.25, 0.0),
                  std::invalid_argument);
  reusex::Cloud nan_only;
  nan_only.push_back(at(std::nanf(""), 0, 0));
  CHECK_THROWS_AS(mesh_support(nan_only, quad_mesh(), 0.25, 0.05),
                  std::invalid_argument);
}
