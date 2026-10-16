// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <reusex/geometry/utils.hpp>
#include <reusex/types.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>

using namespace reusex::geometry;
using Catch::Approx;

using namespace reusex;

TEST_CASE("ComputePolygonNormal_TriangleInXYPlane_ReturnsUnitZNormal",
          "[geometry][utils]") {
  // Create a simple triangle in the XY plane with normal pointing in +Z
  // direction
  CloudLocPtr cloud(new CloudLoc);
  cloud->points.resize(3);

  // Triangle vertices forming a right triangle
  cloud->points[0].x = 0.0f;
  cloud->points[0].y = 0.0f;
  cloud->points[0].z = 0.0f;

  cloud->points[1].x = 1.0f;
  cloud->points[1].y = 0.0f;
  cloud->points[1].z = 0.0f;

  cloud->points[2].x = 0.0f;
  cloud->points[2].y = 1.0f;
  cloud->points[2].z = 0.0f;

  pcl::Vertices poly;
  poly.vertices = {0, 1, 2};

  Eigen::Vector3f normal = geometry::compute_polygon_normal(poly, cloud);

  // The normal should point in the +Z direction (0, 0, 1)
  REQUIRE(normal.x() == Catch::Approx(0.0f).margin(1e-6));
  REQUIRE(normal.y() == Catch::Approx(0.0f).margin(1e-6));
  REQUIRE(normal.z() == Catch::Approx(1.0f).margin(1e-6));

  // Verify the normal is normalized
  REQUIRE(normal.norm() == Catch::Approx(1.0f).margin(1e-6));
}

TEST_CASE("ComputePolygonNormal_SquareInXYPlane_ReturnsUnitZNormal",
          "[geometry][utils]") {
  // Create a square in the XY plane
  CloudLocPtr cloud(new CloudLoc);
  cloud->points.resize(4);

  // Square vertices
  cloud->points[0].x = 0.0f;
  cloud->points[0].y = 0.0f;
  cloud->points[0].z = 0.0f;

  cloud->points[1].x = 2.0f;
  cloud->points[1].y = 0.0f;
  cloud->points[1].z = 0.0f;

  cloud->points[2].x = 2.0f;
  cloud->points[2].y = 2.0f;
  cloud->points[2].z = 0.0f;

  cloud->points[3].x = 0.0f;
  cloud->points[3].y = 2.0f;
  cloud->points[3].z = 0.0f;

  pcl::Vertices poly;
  poly.vertices = {0, 1, 2, 3};

  Eigen::Vector3f normal = geometry::compute_polygon_normal(poly, cloud);

  // The normal should point in the +Z direction (0, 0, 1)
  REQUIRE(normal.x() == Catch::Approx(0.0f).margin(1e-6));
  REQUIRE(normal.y() == Catch::Approx(0.0f).margin(1e-6));
  REQUIRE(normal.z() == Catch::Approx(1.0f).margin(1e-6));

  // Verify the normal is normalized
  REQUIRE(normal.norm() == Catch::Approx(1.0f).margin(1e-6));
}

TEST_CASE("ComputePolygonNormal_SquareInXZPlane_ReturnsUnitYNormal",
          "[geometry][utils]") {
  // Create a square in the XZ plane (vertical, normal pointing in +Y direction)
  CloudLocPtr cloud(new CloudLoc);
  cloud->points.resize(4);

  // Vertical square vertices
  cloud->points[0].x = 0.0f;
  cloud->points[0].y = 0.0f;
  cloud->points[0].z = 0.0f;

  cloud->points[1].x = 1.0f;
  cloud->points[1].y = 0.0f;
  cloud->points[1].z = 0.0f;

  cloud->points[2].x = 1.0f;
  cloud->points[2].y = 0.0f;
  cloud->points[2].z = 1.0f;

  cloud->points[3].x = 0.0f;
  cloud->points[3].y = 0.0f;
  cloud->points[3].z = 1.0f;

  pcl::Vertices poly;
  poly.vertices = {0, 1, 2, 3};

  Eigen::Vector3f normal = geometry::compute_polygon_normal(poly, cloud);

  // The normal should point in the -Y direction (0, -1, 0)
  // (depends on vertex winding order)
  REQUIRE(normal.x() == Catch::Approx(0.0f).margin(1e-6));
  REQUIRE(std::abs(normal.y()) == Catch::Approx(1.0f).margin(1e-6));
  REQUIRE(normal.z() == Catch::Approx(0.0f).margin(1e-6));

  // Verify the normal is normalized
  REQUIRE(normal.norm() == Catch::Approx(1.0f).margin(1e-6));
}

TEST_CASE("ComputePolygonNormal_ReversedWinding_ReturnsOppositeNormal",
          "[geometry][utils]") {
  // Create a triangle with reverse winding order
  CloudLocPtr cloud(new CloudLoc);
  cloud->points.resize(3);

  cloud->points[0].x = 0.0f;
  cloud->points[0].y = 0.0f;
  cloud->points[0].z = 0.0f;

  cloud->points[1].x = 1.0f;
  cloud->points[1].y = 0.0f;
  cloud->points[1].z = 0.0f;

  cloud->points[2].x = 0.0f;
  cloud->points[2].y = 1.0f;
  cloud->points[2].z = 0.0f;

  // Normal winding (CCW)
  pcl::Vertices poly_ccw;
  poly_ccw.vertices = {0, 1, 2};
  Eigen::Vector3f normal_ccw =
      geometry::compute_polygon_normal(poly_ccw, cloud);

  // Reverse winding (CW)
  pcl::Vertices poly_cw;
  poly_cw.vertices = {0, 2, 1};
  Eigen::Vector3f normal_cw = geometry::compute_polygon_normal(poly_cw, cloud);

  // Normals should be opposite
  REQUIRE(normal_ccw.x() == Catch::Approx(-normal_cw.x()).margin(1e-6));
  REQUIRE(normal_ccw.y() == Catch::Approx(-normal_cw.y()).margin(1e-6));
  REQUIRE(normal_ccw.z() == Catch::Approx(-normal_cw.z()).margin(1e-6));

  // Both should be normalized
  REQUIRE(normal_ccw.norm() == Catch::Approx(1.0f).margin(1e-6));
  REQUIRE(normal_cw.norm() == Catch::Approx(1.0f).margin(1e-6));
}

TEST_CASE("ComputePolygonNormal_TooFewVertices_ThrowsInvalidArgument",
          "[geometry][utils]") {
  CloudLocPtr cloud(new CloudLoc);
  cloud->points.resize(2);

  cloud->points[0].x = 0.0f;
  cloud->points[0].y = 0.0f;
  cloud->points[0].z = 0.0f;

  cloud->points[1].x = 1.0f;
  cloud->points[1].y = 0.0f;
  cloud->points[1].z = 0.0f;

  // Polygon with only 2 vertices
  pcl::Vertices poly;
  poly.vertices = {0, 1};

  REQUIRE_THROWS_AS(geometry::compute_polygon_normal(poly, cloud),
                    std::invalid_argument);
}

TEST_CASE("ComputePolygonNormal_CollinearVertices_ThrowsRuntimeError",
          "[geometry][utils]") {
  CloudLocPtr cloud(new CloudLoc);
  cloud->points.resize(3);

  // All three points are collinear
  cloud->points[0].x = 0.0f;
  cloud->points[0].y = 0.0f;
  cloud->points[0].z = 0.0f;

  cloud->points[1].x = 1.0f;
  cloud->points[1].y = 0.0f;
  cloud->points[1].z = 0.0f;

  cloud->points[2].x = 2.0f;
  cloud->points[2].y = 0.0f;
  cloud->points[2].z = 0.0f;

  pcl::Vertices poly;
  poly.vertices = {0, 1, 2};

  // Should throw because the normal has zero magnitude
  REQUIRE_THROWS_AS(geometry::compute_polygon_normal(poly, cloud),
                    std::runtime_error);
}

TEST_CASE("DistPlanePoint_VariousPointsAndPlanes_ReturnsSignedDistance",
          "[geometry][dist_plane_point]") {

  SECTION("Point on plane has zero distance") {
    // Plane: z = 0 (XY plane)
    Eigen::Vector4d plane(0.0, 0.0, 1.0, 0.0); // nx, ny, nz, d
    Eigen::Vector3d point(1.0, 2.0, 0.0);

    double distance = dist_plane_point(plane, point);
    REQUIRE(distance == Approx(0.0).margin(1e-10));
  }

  SECTION("Point above XY plane") {
    // Plane: z = 0
    Eigen::Vector4d plane(0.0, 0.0, 1.0, 0.0);
    Eigen::Vector3d point(0.0, 0.0, 5.0);

    double distance = dist_plane_point(plane, point);
    REQUIRE(distance == Approx(5.0));
  }

  SECTION("Point below XY plane") {
    // Plane: z = 0
    Eigen::Vector4d plane(0.0, 0.0, 1.0, 0.0);
    Eigen::Vector3d point(0.0, 0.0, -3.0);

    double distance = dist_plane_point(plane, point);
    REQUIRE(distance == Approx(-3.0));
  }

  SECTION("Distance to YZ plane") {
    // Plane: x = 0
    Eigen::Vector4d plane(1.0, 0.0, 0.0, 0.0);
    Eigen::Vector3d point(4.0, 0.0, 0.0);

    double distance = dist_plane_point(plane, point);
    REQUIRE(distance == Approx(4.0));
  }

  SECTION("Distance to XZ plane") {
    // Plane: y = 0
    Eigen::Vector4d plane(0.0, 1.0, 0.0, 0.0);
    Eigen::Vector3d point(0.0, -2.5, 0.0);

    double distance = dist_plane_point(plane, point);
    REQUIRE(distance == Approx(-2.5));
  }

  SECTION("Distance to offset plane") {
    // Plane: z = 3
    Eigen::Vector4d plane(0.0, 0.0, 1.0, -3.0);
    Eigen::Vector3d point(0.0, 0.0, 5.0);

    double distance = dist_plane_point(plane, point);
    REQUIRE(distance == Approx(2.0));
  }

  SECTION("Distance with non-unit normal") {
    // Plane with normal (2, 0, 0) and d = 0
    // NOTE: The dist_plane_point implementation uses squaredNorm() instead of
    // norm() This differs from the standard point-to-plane distance formula
    // which uses norm() The implementation divides by squared norm: distance =
    // (n·p + d) / ||n||²
    Eigen::Vector4d plane(2.0, 0.0, 0.0, 0.0);
    Eigen::Vector3d point(1.0, 0.0, 0.0);

    double distance = dist_plane_point(plane, point);
    // Distance = (2*1 + 0*0 + 0*0 + 0) / (4 + 0 + 0) = 2/4 = 0.5
    REQUIRE(distance == Approx(0.5).margin(1e-10));
  }

  SECTION("Distance to diagonal plane") {
    // Plane: x + y + z = 0 (passes through origin)
    // Normal is (1, 1, 1), d = 0
    Eigen::Vector4d plane(1.0, 1.0, 1.0, 0.0);
    Eigen::Vector3d point(1.0, 1.0, 1.0);

    double distance = dist_plane_point(plane, point);
    // Distance = (1 + 1 + 1 + 0) / (1 + 1 + 1) = 3 / 3 = 1.0
    REQUIRE(distance == Approx(1.0).margin(1e-10));
  }

  SECTION("Distance from origin to plane") {
    // Plane: x + y + z = 6
    Eigen::Vector4d plane(1.0, 1.0, 1.0, -6.0);
    Eigen::Vector3d point(0.0, 0.0, 0.0);

    double distance = dist_plane_point(plane, point);
    // Distance = (0 + 0 + 0 - 6) / (1 + 1 + 1) = -6 / 3 = -2.0
    REQUIRE(distance == Approx(-2.0).margin(1e-10));
  }

  SECTION("Symmetric points have opposite signed distances") {
    Eigen::Vector4d plane(0.0, 0.0, 1.0, 0.0);
    Eigen::Vector3d point1(0.0, 0.0, 3.0);
    Eigen::Vector3d point2(0.0, 0.0, -3.0);

    double dist1 = dist_plane_point(plane, point1);
    double dist2 = dist_plane_point(plane, point2);

    REQUIRE(dist1 == Approx(-dist2));
  }
}

TEST_CASE("DistPlanePoint_NormalizedNormalVectors_ReturnsSignedDistance",
          "[geometry][dist_plane_point][normalized]") {

  SECTION("Normalized normal vector") {
    // Plane with normalized normal (0, 0, 1) passing through z = 2
    Eigen::Vector4d plane(0.0, 0.0, 1.0, -2.0);
    Eigen::Vector3d point(5.0, 3.0, 7.0);

    double distance = dist_plane_point(plane, point);
    REQUIRE(distance == Approx(5.0));
  }

  SECTION("Another normalized plane") {
    // Plane with normal (1/sqrt(2), 1/sqrt(2), 0) and offset
    double sqrt2_inv = 1.0 / std::sqrt(2.0);
    Eigen::Vector4d plane(sqrt2_inv, sqrt2_inv, 0.0, 0.0);
    Eigen::Vector3d point(1.0, 1.0, 0.0);

    double distance = dist_plane_point(plane, point);
    // Distance = (1/sqrt(2) + 1/sqrt(2) + 0) / 1 = 2/sqrt(2) = sqrt(2)
    REQUIRE(distance == Approx(std::sqrt(2.0)));
  }
}

// ── force_orthogonal_planes (reusex-8vj) ───────────────────────────────────
//
// Snapping a tilted normal to vertical/horizontal rotates the plane. Keeping
// the old `d` pivots it about whichever point is closest to the origin
// instead of its own inliers, moving the plane metres away from the points
// it was fit to (NewOffice horizontals landed at -2.55, 6.42, 10.02 m against
// 0.5-1.5 m inliers). The fix recomputes `d` through the supplied centroid.

TEST_CASE("ForceOrthogonalPlanes_TiltedHorizontalAtHeight_StaysThroughCentroid",
          "[geometry][force_orthogonal_planes]") {
  // A near-horizontal plane (within the default 0.1 threshold of n_z = 1),
  // but tilted, with a centroid 10 m from the origin in x and 1 m up in z.
  // Fit so the plane actually passes through the centroid: n . c + d = 0.
  Eigen::Vector3d centroid(10.0, 0.0, 1.0);
  Eigen::Vector3d n(0.05, 0.0, std::sqrt(1.0 - 0.05 * 0.05));
  n.normalize();
  Eigen::Vector4d plane;
  plane.head<3>() = n;
  plane[3] = -n.dot(centroid);

  EigenVectorContainer<double, 4> planes{plane};
  EigenVectorContainer<double, 3> centroids{centroid};

  const auto result = force_orthogonal_planes(planes, centroids);

  REQUIRE(result.size() == 1);
  // Normal snapped to the up vector.
  CHECK(result[0].head<3>().isApprox(Eigen::Vector3d(0, 0, 1), 1e-9));
  // The snapped plane must still pass through its own centroid, not land at
  // whatever height the un-rotated offset implies (-10 m here, since the
  // original plane was closest to the origin at x=10, z≈-10*0.05=-0.5 plus
  // the tilt contribution — the regression this test guards against).
  CHECK(dist_plane_point(result[0], centroid) == Approx(0.0).margin(1e-9));
  CHECK(result[0][3] == Approx(-centroid.z()).margin(1e-9));
}

TEST_CASE("ForceOrthogonalPlanes_TiltedWallAwayFromOrigin_StaysThroughCentroid",
          "[geometry][force_orthogonal_planes]") {
  // A near-vertical plane tilted slightly from the Z axis, far from the
  // origin, must still pass through its own centroid after being squared to
  // a wall (orthogonal to 'up').
  Eigen::Vector3d centroid(5.0, 20.0, 1.5);
  Eigen::Vector3d n(1.0, 0.0, 0.05);
  n.normalize();
  Eigen::Vector4d plane;
  plane.head<3>() = n;
  plane[3] = -n.dot(centroid);

  EigenVectorContainer<double, 4> planes{plane};
  EigenVectorContainer<double, 3> centroids{centroid};

  const auto result = force_orthogonal_planes(planes, centroids);

  REQUIRE(result.size() == 1);
  CHECK(std::abs(result[0][2]) < 1e-9); // orthogonal to 'up'
  CHECK(dist_plane_point(result[0], centroid) == Approx(0.0).margin(1e-9));
}

TEST_CASE("ForceOrthogonalPlanes_AlreadyAxisAligned_OffsetUnchanged",
          "[geometry][force_orthogonal_planes]") {
  // A plane that is already exactly horizontal should pass through unchanged
  // (same normal, same offset, derived from the same centroid either way).
  Eigen::Vector3d centroid(3.0, -4.0, 2.0);
  Eigen::Vector4d plane(0.0, 0.0, 1.0, -2.0);

  EigenVectorContainer<double, 4> planes{plane};
  EigenVectorContainer<double, 3> centroids{centroid};

  const auto result = force_orthogonal_planes(planes, centroids);

  CHECK(result[0].head<3>().isApprox(Eigen::Vector3d(0, 0, 1), 1e-12));
  CHECK(result[0][3] == Approx(-2.0).margin(1e-12));
}

TEST_CASE("ForceOrthogonalPlanes_SizeMismatch_Throws",
          "[geometry][force_orthogonal_planes]") {
  EigenVectorContainer<double, 4> planes{Eigen::Vector4d(0, 0, 1, 0)};
  EigenVectorContainer<double, 3> centroids; // empty: mismatched size

  CHECK_THROWS_AS(force_orthogonal_planes(planes, centroids),
                  std::invalid_argument);
}
