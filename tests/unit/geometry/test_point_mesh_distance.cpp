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
