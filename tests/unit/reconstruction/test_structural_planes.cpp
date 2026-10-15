// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Unit tests for select_structural_planes() (reusex-8l3): the step in
// `create mesh` that keeps the cell complex down to wall lines and storey
// sections. These tests pin what it keeps and what it drops:
//   - collinear same-facing wall pieces, far apart  -> one line
//   - the two faces of a thin wall                  -> both kept
//   - a jamb (small, perpendicular)                 -> dropped
//   - a small pier collinear with a facade          -> merged, kept
//   - a desk top / desk underside over the floor    -> dropped
//   - an upper storey floor, a split level          -> kept
//   - disabled                                      -> unchanged

#include <reusex/reconstruction/structural_planes.hpp>
#include <reusex/types.hpp>

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>

#include <stdexcept>

using namespace reusex;
using namespace reusex::geometry;

namespace {

struct Scene {
  CloudPtr cloud{new Cloud};
  EigenVectorContainer<double, 4> planes;
  std::vector<IndicesPtr> inliers;
  EigenVectorContainer<double, 3> centroids;

  /// Wall on x = x0 (normal +-x), spanning y in [y0, y1], z in [z0, z1].
  void x_wall(double x0, double sign, double y0, double y1, double z0,
              double z1, double step = 0.05) {
    IndicesPtr idx(new Indices);
    Eigen::Vector3d c = Eigen::Vector3d::Zero();
    for (double y = y0; y <= y1 + 1e-9; y += step)
      for (double z = z0; z <= z1 + 1e-9; z += step)
        add(idx, c, x0, y, z);
    push(Eigen::Vector4d(sign, 0, 0, -sign * x0), idx, c);
  }

  /// Wall on y = y0 (normal +-y), spanning x in [x0, x1], z in [z0, z1].
  void y_wall(double y0, double sign, double x0, double x1, double z0,
              double z1, double step = 0.05) {
    IndicesPtr idx(new Indices);
    Eigen::Vector3d c = Eigen::Vector3d::Zero();
    for (double x = x0; x <= x1 + 1e-9; x += step)
      for (double z = z0; z <= z1 + 1e-9; z += step)
        add(idx, c, x, y0, z);
    push(Eigen::Vector4d(0, sign, 0, -sign * y0), idx, c);
  }

  /// Horizontal patch at height h; sign +1 faces up (floor, desk top),
  /// -1 faces down (ceiling, desk underside).
  void slab(double h, double sign, double x0, double x1, double y0, double y1,
            double step = 0.05) {
    IndicesPtr idx(new Indices);
    Eigen::Vector3d c = Eigen::Vector3d::Zero();
    for (double x = x0; x <= x1 + 1e-9; x += step)
      for (double y = y0; y <= y1 + 1e-9; y += step)
        add(idx, c, x, y, h);
    push(Eigen::Vector4d(0, 0, sign, -sign * h), idx, c);
  }

    private:
  void add(IndicesPtr &idx, Eigen::Vector3d &c, double x, double y, double z) {
    PointT p;
    p.x = static_cast<float>(x);
    p.y = static_cast<float>(y);
    p.z = static_cast<float>(z);
    idx->push_back(static_cast<int>(cloud->size()));
    cloud->push_back(p);
    c += Eigen::Vector3d(x, y, z);
  }
  void push(const Eigen::Vector4d &plane, const IndicesPtr &idx,
            const Eigen::Vector3d &sum) {
    planes.push_back(plane);
    inliers.push_back(idx);
    centroids.push_back(sum / static_cast<double>(idx->size()));
  }
};

std::size_t count_kind(const EigenVectorContainer<double, 4> &planes,
                       bool vertical) {
  std::size_t n = 0;
  for (const auto &p : planes)
    if ((std::abs(p[2]) < 0.1) == vertical)
      ++n;
  return n;
}

bool has_height(const EigenVectorContainer<double, 4> &planes, double h) {
  for (const auto &p : planes)
    if (std::abs(p[2]) > 0.9 && std::abs(-p[3] / p[2] - h) < 1e-6)
      return true;
  return false;
}

} // namespace

TEST_CASE("structural planes: collinear wall pieces become one line",
          "[reconstruction][structural_planes]") {
  Scene s;
  s.x_wall(0.0, +1, 0.0, 3.0, 0.0, 2.5);   // 7.5 m^2
  s.x_wall(0.05, +1, 8.0, 10.0, 0.0, 2.5); // same line, 5 m along, 5 cm off
  const auto before = s.inliers[0]->size() + s.inliers[1]->size();

  const auto r =
      select_structural_planes(s.planes, s.inliers, s.centroids, s.cloud);
  REQUIRE(s.planes.size() == 1);
  CHECK(s.inliers[0]->size() == before);
  CHECK(r.vertical_in == 2);
  CHECK(r.vertical_merged == 1);
  // The best-supported piece keeps its plane.
  CHECK(s.planes[0][3] == 0.0);
}

TEST_CASE("structural planes: the two faces of a thin wall both stay",
          "[reconstruction][structural_planes]") {
  Scene s;
  s.x_wall(0.0, +1, 0.0, 4.0, 0.0, 2.5);
  s.x_wall(0.10, -1, 0.0, 4.0, 0.0, 2.5); // other face, 10 cm behind
  const auto r =
      select_structural_planes(s.planes, s.inliers, s.centroids, s.cloud);
  CHECK(s.planes.size() == 2);
  CHECK(r.vertical_merged == 0);
  CHECK(r.vertical_small == 0);
}

TEST_CASE("structural planes: a jamb is dropped, a collinear pier is kept",
          "[reconstruction][structural_planes]") {
  Scene s;
  s.x_wall(0.0, +1, 0.0, 3.0, 0.0, 2.5);  // facade piece, 7.5 m^2
  s.x_wall(0.02, +1, 4.0, 4.5, 0.0, 2.0); // pier, 1.0 m^2, same line
  s.y_wall(3.0, +1, 0.0, 0.25, 0.0, 2.0); // jamb, 0.5 m^2, perpendicular
  const auto r =
      select_structural_planes(s.planes, s.inliers, s.centroids, s.cloud);
  REQUIRE(s.planes.size() == 1);
  CHECK(std::abs(s.planes[0][0]) == 1.0); // the x wall line survives
  CHECK(r.vertical_merged == 1);
  CHECK(r.vertical_small == 1);
}

TEST_CASE("structural planes: a short thin wall counts both faces",
          "[reconstruction][structural_planes]") {
  // Each face of this 0.5 m stub is ~1.1 m^2, under the threshold, but the
  // wall it belongs to has twice that: both faces must stay (seen on house1,
  // where dropping them merged two rooms).
  Scene s;
  s.y_wall(1.0, +1, 0.0, 0.45, 0.0, 2.4);
  s.y_wall(1.1, -1, 0.0, 0.45, 0.0, 2.4);
  s.x_wall(5.0, +1, 0.0, 0.25, 0.0, 2.0); // a lone jamb still goes
  const auto r =
      select_structural_planes(s.planes, s.inliers, s.centroids, s.cloud);
  CHECK(s.planes.size() == 2);
  CHECK(r.vertical_small == 1);
}

TEST_CASE("structural planes: a lone small wall stays when the threshold "
          "allows it",
          "[reconstruction][structural_planes]") {
  Scene s;
  s.y_wall(3.0, +1, 0.0, 0.8, 0.0, 2.5); // 2.0 m^2 stub
  select_structural_planes(s.planes, s.inliers, s.centroids, s.cloud);
  CHECK(s.planes.size() == 1);
}

TEST_CASE("structural planes: furniture tops do not become storey sections",
          "[reconstruction][structural_planes]") {
  Scene s;
  s.slab(0.0, +1, 0.0, 10.0, 0.0, 8.0);  // floor
  s.slab(2.7, -1, 0.0, 10.0, 0.0, 8.0);  // ceiling
  s.slab(0.75, +1, 2.0, 4.0, 2.0, 3.0);  // desk top
  s.slab(0.72, -1, 2.0, 4.0, 2.0, 3.0);  // desk underside
  s.slab(3.0, +1, 0.0, 10.0, 0.0, 8.0);  // upper storey floor
  s.slab(0.6, +1, 12.0, 16.0, 0.0, 4.0); // split level beside the floor

  const auto r =
      select_structural_planes(s.planes, s.inliers, s.centroids, s.cloud);
  CHECK(r.horizontal_in == 6);
  CHECK(r.horizontal_furniture == 2);
  CHECK(s.planes.size() == 4);
  CHECK(has_height(s.planes, 0.0));
  CHECK(has_height(s.planes, 2.7));
  CHECK(has_height(s.planes, 3.0));
  CHECK(has_height(s.planes, 0.6));
  CHECK_FALSE(has_height(s.planes, 0.75));
  CHECK_FALSE(has_height(s.planes, 0.72));
}

TEST_CASE("structural planes: a large raised floor over a small one stays",
          "[reconstruction][structural_planes]") {
  // The clearance rule only drops a plane over a LARGER one: a big upper
  // floor stays even when a small cabinet top lies 1.5 m below it.
  Scene s;
  s.slab(0.0, +1, 0.0, 10.0, 0.0, 8.0);
  s.slab(1.5, +1, 2.0, 2.6, 2.0, 2.6); // cabinet top
  s.slab(3.0, +1, 0.0, 10.0, 0.0, 8.0);
  select_structural_planes(s.planes, s.inliers, s.centroids, s.cloud);
  CHECK(has_height(s.planes, 3.0));
  CHECK_FALSE(has_height(s.planes, 1.5));
}

TEST_CASE("structural planes: disabled leaves the planes alone",
          "[reconstruction][structural_planes]") {
  Scene s;
  s.x_wall(0.0, +1, 0.0, 3.0, 0.0, 2.5);
  s.y_wall(3.0, +1, 0.0, 0.25, 0.0, 2.0);
  s.slab(0.0, +1, 0.0, 10.0, 0.0, 8.0);
  s.slab(0.75, +1, 2.0, 4.0, 2.0, 3.0);
  StructuralPlaneOptions opt;
  opt.enabled = false;
  const auto r =
      select_structural_planes(s.planes, s.inliers, s.centroids, s.cloud, opt);
  CHECK(s.planes.size() == 4);
  CHECK(r.planes_out == 4);
  CHECK(count_kind(s.planes, true) == 2);
}

TEST_CASE("structural planes: mismatched inputs throw",
          "[reconstruction][structural_planes]") {
  Scene s;
  s.x_wall(0.0, +1, 0.0, 3.0, 0.0, 2.5);
  s.centroids.clear();
  CHECK_THROWS_AS(
      select_structural_planes(s.planes, s.inliers, s.centroids, s.cloud),
      std::runtime_error);
}
