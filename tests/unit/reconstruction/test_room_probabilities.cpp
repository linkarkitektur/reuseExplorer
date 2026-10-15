// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// CellComplex::compute_room_probabilities: the per-cell room evidence the
// Solidifier's data term is built from (reusex-bi6).

#include <reusex/reconstruction/CellComplex.hpp>

#include <catch2/catch_test_macros.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>

#include <array>
#include <cmath>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

using namespace reusex::geometry;

namespace {

using PlaneVec =
    std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>>;

struct Scan {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud{
      new pcl::PointCloud<pcl::PointXYZRGB>};
  pcl::PointCloud<pcl::Normal>::Ptr normals{new pcl::PointCloud<pcl::Normal>};
  pcl::PointCloud<pcl::Label>::Ptr labels{new pcl::PointCloud<pcl::Label>};

  void add(const Eigen::Vector3d &p, const Eigen::Vector3d &n,
           std::uint32_t label) {
    pcl::PointXYZRGB q;
    q.x = static_cast<float>(p.x());
    q.y = static_cast<float>(p.y());
    q.z = static_cast<float>(p.z());
    cloud->push_back(q);
    normals->push_back(pcl::Normal(static_cast<float>(n.x()),
                                   static_cast<float>(n.y()),
                                   static_cast<float>(n.z())));
    pcl::Label l;
    l.label = label;
    labels->push_back(l);
  }

  /// Sample the six faces of the box [lo, hi] at `step`; normals point
  /// inward when `inward`, outward otherwise.
  void add_box(const Eigen::Vector3d &lo, const Eigen::Vector3d &hi,
               double step, bool inward, std::uint32_t label) {
    for (int axis = 0; axis < 3; ++axis) {
      const int u = (axis + 1) % 3;
      const int v = (axis + 2) % 3;
      for (int side = 0; side < 2; ++side) {
        Eigen::Vector3d n = Eigen::Vector3d::Zero();
        n[axis] = side == 0 ? -1.0 : 1.0; // outward
        if (inward)
          n = -n;
        for (double a = lo[u]; a <= hi[u] + 1e-9; a += step)
          for (double b = lo[v]; b <= hi[v] + 1e-9; b += step) {
            Eigen::Vector3d p;
            p[axis] = side == 0 ? lo[axis] : hi[axis];
            p[u] = a;
            p[v] = b;
            add(p, n, label);
          }
      }
    }
  }
};

} // namespace

namespace {

/// The one bounded cell of a 4 x 3 x 2.5 m box room, with its room
/// probabilities computed from `scan`.
std::vector<double> box_room_probabilities(const Scan &scan) {
  PlaneVec planes;
  planes.emplace_back(1, 0, 0, 0);    // x = 0
  planes.emplace_back(1, 0, 0, -4);   // x = 4
  planes.emplace_back(0, 1, 0, 0);    // y = 0
  planes.emplace_back(0, 1, 0, -3);   // y = 3
  planes.emplace_back(0, 0, 1, 0);    // z = 0
  planes.emplace_back(0, 0, 1, -2.5); // z = 2.5
  std::vector<size_t> verticals{0, 1, 2, 3};
  std::vector<size_t> horizontals{4, 5};
  std::vector<std::pair<size_t, size_t>> pairs{{0, 1}, {2, 3}};
  auto cc = std::make_shared<CellComplex>(
      planes, verticals, horizontals, pairs, std::array<double, 2>{-1.0, -1.0},
      std::array<double, 2>{5.0, 4.0}, std::nullopt, 500);
  REQUIRE(cc->num_cells() == 1);

  cc->compute_room_probabilities<pcl::PointXYZRGB, pcl::Normal, pcl::Label>(
      scan.cloud, scan.normals, scan.labels);
  auto c_rp = cc->property_map<CellComplex::Vertex, std::vector<double>>(
      "c:room_probabilities");
  return c_rp[*cc->cells_begin()];
}

} // namespace

TEST_CASE("RoomProbabilities_EmptyRoom_ReadsAsTheRoom",
          "[geometry][cellcomplex][room_probabilities]") {
  // A 4 x 3 x 2.5 m room scanned on all six faces (room label 1, normals
  // into the room): every ray from inside hits a front face of room 1.
  Scan scan;
  scan.add_box({0, 0, 0}, {4, 3, 2.5}, 0.05, /*inward=*/true, 1);
  const auto p = box_room_probabilities(scan);
  REQUIRE(p.size() == 2);
  INFO("p_out = " << p[0] << ", p_room = " << p[1]);
  CHECK(p[1] > 0.9);
}

TEST_CASE("RoomProbabilities_ClutterAtCellCentre_DoesNotFlipRoomToOutside",
          "[geometry][cellcomplex][room_probabilities]") {
  // The same room with a 0.5 m closed object (a screen / lamp / shelf, as
  // on the NewOffice desk strips) exactly at the room cell's centroid, its
  // scanned surface facing out and assigned to the room, as `create rooms`
  // does with furniture. Casting every evidence ray from the single
  // centroid made all of them hit the object's back faces within a few cm,
  // so the cell read as "outside" (p_out = 1) and the MIP dropped it
  // although it is the room itself (reusex-bi6).
  Scan scan;
  scan.add_box({0, 0, 0}, {4, 3, 2.5}, 0.05, /*inward=*/true, 1);
  const Eigen::Vector3d centre(2.0, 1.5, 1.25);
  scan.add_box(centre - Eigen::Vector3d::Constant(0.25),
               centre + Eigen::Vector3d::Constant(0.25), 0.05,
               /*inward=*/false, 1);
  const auto p = box_room_probabilities(scan);
  REQUIRE(p.size() == 2);
  INFO("p_out = " << p[0] << ", p_room = " << p[1]);
  CHECK(p[1] > 0.6);
  CHECK(p[1] > p[0]);
}
