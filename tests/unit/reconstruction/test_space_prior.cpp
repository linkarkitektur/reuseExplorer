// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// apply_space_prior(): the stored spaces as evidence for the cells inside
// their footprint (reusex-6t4). House2's stair hall under the stair sees
// only back faces (the treads from below, the neighbouring rooms' walls
// from behind) and read p_out ~0.8 although the hall's space covers it.

#include <reusex/reconstruction/CellComplex.hpp>
#include <reusex/reconstruction/Solidifier.hpp>
#include <reusex/reconstruction/space_prior.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>

#include <array>
#include <cstdint>
#include <memory>
#include <stdexcept>
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

  /// One axis-aligned rectangle at p[axis] = c over [lo, hi] in the other
  /// two axes, its normal along `normal`.
  void add_rect(int axis, double c, const Eigen::Vector2d &lo,
                const Eigen::Vector2d &hi, const Eigen::Vector3d &normal,
                std::uint32_t label, double step = 0.05) {
    const int u = (axis + 1) % 3;
    const int v = (axis + 2) % 3;
    for (double a = lo.x(); a <= hi.x() + 1e-9; a += step)
      for (double b = lo.y(); b <= hi.y() + 1e-9; b += step) {
        Eigen::Vector3d p;
        p[axis] = c;
        p[u] = a;
        p[v] = b;
        add(p, normal, label);
      }
  }

  /// The six faces of [lo, hi], normals inward or outward.
  void add_box(const Eigen::Vector3d &lo, const Eigen::Vector3d &hi,
               bool inward, std::uint32_t label) {
    for (int axis = 0; axis < 3; ++axis) {
      const int u = (axis + 1) % 3;
      const int v = (axis + 2) % 3;
      for (int side = 0; side < 2; ++side) {
        Eigen::Vector3d n = Eigen::Vector3d::Zero();
        n[axis] = side == 0 ? -1.0 : 1.0; // outward
        if (inward)
          n = -n;
        add_rect(axis, side == 0 ? lo[axis] : hi[axis], {lo[u], lo[v]},
                 {hi[u], hi[v]}, n, label);
      }
    }
  }
};

/// The one bounded cell of a 4 x 3 x 2.5 m box, with its room
/// probabilities computed from `scan`.
std::shared_ptr<CellComplex> box_cell_complex(const Scan &scan) {
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
  return cc;
}

/// The probabilities of the box's one cell.
std::vector<double> probabilities(CellComplex &cc) {
  auto c_rp = cc.property_map<CellComplex::Vertex, std::vector<double>>(
      "c:room_probabilities");
  return c_rp[*cc.cells_begin()];
}

/// An occluded pocket of room 1 (label 1): its x = 0 wall and its floor are
/// scanned from inside, every other face is a neighbouring room's wall
/// (label 2) seen from behind, a 2 cm thick slab behind the pocket's own
/// unscanned face. Most rays from inside hit back faces: it reads outside.
Scan occluded_pocket() {
  Scan scan;
  scan.add_box({-0.02, -0.02, -0.02}, {4.02, 3.02, 2.52}, /*inward=*/false, 2);
  scan.add_rect(0, 0.0, {0.0, 0.0}, {3.0, 2.5}, {1, 0, 0}, 1);
  scan.add_rect(2, 0.0, {0.0, 0.0}, {4.0, 3.0}, {0, 0, 1}, 1);
  return scan;
}

SpaceFootprint rectangle_space(unsigned int room_label, double x0, double y0,
                               double x1, double y1, double z0 = 0.0,
                               double z1 = 2.5) {
  SpaceFootprint s;
  s.room_label = room_label;
  s.outer = {{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}};
  s.z_floor = z0;
  s.z_ceiling = z1;
  return s;
}

/// Index of a room label in the probability vector.
size_t index_of(const CellComplex &cc, unsigned int label) {
  for (size_t i = 0; i < cc.room_labels.size(); ++i)
    if (cc.room_labels[i] == label)
      return i + 1;
  FAIL("label " << label << " not in the rooms");
  return 0;
}

} // namespace

TEST_CASE("SpaceFootprint_Contains_OuterRingHolesAndStoreyHeight",
          "[geometry][space_prior]") {
  auto s = rectangle_space(1, 0, 0, 4, 3);
  s.holes = {{{1, 1}, {2, 1}, {2, 2}, {1, 2}}};
  CHECK(s.contains({0.5, 0.5, 1.0}));
  CHECK_FALSE(s.contains({1.5, 1.5, 1.0}));  // in the hole
  CHECK_FALSE(s.contains({4.5, 0.5, 1.0}));  // outside the ring
  CHECK_FALSE(s.contains({0.5, 0.5, 2.6}));  // above the ceiling
  CHECK_FALSE(s.contains({0.5, 0.5, -0.1})); // below the floor

  // The footprint is in its storey's level frame: a frame shifted up by
  // 3 m (an upper storey whose floor is level z = 0) takes world z 3..5.5.
  s.world_to_level = Eigen::Translation3d(0, 0, -3.0);
  CHECK(s.contains({0.5, 0.5, 4.0}));
  CHECK_FALSE(s.contains({0.5, 0.5, 1.0}));
}

TEST_CASE("SpacePrior_OccludedPocketInsideSpace_ReadsAsTheRoom",
          "[geometry][space_prior]") {
  // The rays call the pocket outside, but the space of room 1 covers it:
  // the blended probabilities favour room 1 and the Solidifier keeps it.
  auto cc = box_cell_complex(occluded_pocket());
  const size_t r1 = index_of(*cc, 1);
  const auto before = probabilities(*cc);
  INFO("before: p_out = " << before[0] << ", p_room1 = " << before[r1]);
  REQUIRE(before[0] > 0.6);
  REQUIRE(before[0] > before[r1]);

  const auto stats =
      apply_space_prior(*cc, {rectangle_space(1, 0, 0, 4, 3)}, 0.45);
  const auto after = probabilities(*cc);
  INFO("after: p_out = " << after[0] << ", p_room1 = " << after[r1]);
  CHECK(stats.spaces == 1);
  CHECK(stats.cells == 1);
  CHECK(stats.cells_flipped == 1);
  CHECK(stats.volume == Catch::Approx(30.0));
  CHECK(after[r1] > after[0]);
  double sum = 0.0;
  for (double p : after)
    sum += p;
  CHECK(sum == Catch::Approx(1.0));

  // No face is supported (no coverage computed): the cell's own faces are
  // on the bounding box, so no boundary cost; the room term decides.
  auto f_sp =
      cc->add_property_map<CellComplex::Vertex, double>("f:support_probability")
          .first;
  for (auto f = cc->faces_begin(); f != cc->faces_end(); ++f)
    f_sp[*f] = 1.0;
  Solidifier solidifier(cc);
  const auto result = solidifier.solve();
  REQUIRE(result.has_value());
  CHECK(result->first.at(*cc->cells_begin()) == static_cast<int>(r1));
}

TEST_CASE("SpacePrior_NoSpaceOrOtherPlaceOrHeight_KeepsTheRays",
          "[geometry][space_prior]") {
  auto cc = box_cell_complex(occluded_pocket());
  const auto before = probabilities(*cc);

  SECTION("no spaces (the spaces table is missing)") {
    const auto stats = apply_space_prior(*cc, {}, 0.45);
    CHECK(stats.cells == 0);
  }
  SECTION("weight 0") {
    apply_space_prior(*cc, {rectangle_space(1, 0, 0, 4, 3)}, 0.0);
  }
  SECTION("the space lies elsewhere in plan") {
    const auto stats =
        apply_space_prior(*cc, {rectangle_space(1, 10, 10, 14, 13)}, 0.45);
    CHECK(stats.cells == 0);
  }
  SECTION("the space is another storey's") {
    const auto stats = apply_space_prior(
        *cc, {rectangle_space(1, 0, 0, 4, 3, 3.0, 5.5)}, 0.45);
    CHECK(stats.cells == 0);
  }
  SECTION("the space names a room the rooms cloud does not have") {
    const auto stats =
        apply_space_prior(*cc, {rectangle_space(7, 0, 0, 4, 3)}, 0.45);
    CHECK(stats.spaces == 0);
    CHECK(stats.spaces_skipped == 1);
    CHECK(stats.cells == 0);
  }
  CHECK(probabilities(*cc) == before);
}

TEST_CASE("SpacePrior_IsEvidenceNotOverride", "[geometry][space_prior]") {
  // A cell every ray calls outside (all back faces: a wrong space drawn
  // over the slab behind a wall) keeps outside the largest probability
  // under any weight below 0.5.
  Scan scan;
  scan.add_box({-0.02, -0.02, -0.02}, {4.02, 3.02, 2.52}, /*inward=*/false, 2);
  scan.add_rect(0, -5.0, {0.0, 0.0}, {3.0, 2.5}, {1, 0, 0}, 1); // room 1
  auto cc = box_cell_complex(scan);
  REQUIRE(probabilities(*cc)[0] == Catch::Approx(1.0));
  const auto stats =
      apply_space_prior(*cc, {rectangle_space(1, 0, 0, 4, 3)}, 0.45);
  CHECK(stats.cells == 1);
  CHECK(stats.cells_flipped == 0);
  const auto p = probabilities(*cc);
  CHECK(p[0] == Catch::Approx(0.55));
  CHECK(p[index_of(*cc, 1)] == Catch::Approx(0.45));
}

TEST_CASE("SpacePrior_PartialCover_ScalesWithTheCoveredSamples",
          "[geometry][space_prior]") {
  // A space covering the cell's west half holds 4 of its 9 samples (the
  // west corners' midpoints; the centroid lies on its edge, outside).
  auto cc = box_cell_complex(occluded_pocket());
  const auto before = probabilities(*cc);
  const auto stats =
      apply_space_prior(*cc, {rectangle_space(1, -1, -1, 1.9, 4)}, 0.45);
  REQUIRE(stats.cells == 1);
  const double c = 4.0 / 9.0;
  CHECK(stats.volume == Catch::Approx(30.0 * c));
  CHECK(probabilities(*cc)[0] == Catch::Approx(before[0] * (1 - 0.45 * c)));
}

TEST_CASE("SpacePrior_WeightAboveOne_Throws", "[geometry][space_prior]") {
  auto cc = box_cell_complex(occluded_pocket());
  CHECK_THROWS_AS(apply_space_prior(*cc, {rectangle_space(1, 0, 0, 4, 3)}, 1.5),
                  std::invalid_argument);
}

TEST_CASE("SpacePrior_WallSlabCell_KeepsTheRays", "[geometry][space_prior]") {
  // A 0.2 m wall (a pair of faces at x = 1.9 / 2.1) splits a room-sized
  // box in two. A space polygon that runs across the wall (raster fill
  // where the wall graph did not close, or a misregistered wall) must not
  // pull the wall's slab into the room: that would merge the two rooms
  // through their wall and drop the wall from the envelope. Only the cells
  // either side take the prior.
  PlaneVec planes;
  planes.emplace_back(1, 0, 0, 0);    // 0: x = 0
  planes.emplace_back(1, 0, 0, -4);   // 1: x = 4
  planes.emplace_back(-1, 0, 0, 1.9); // 2: x = 1.9, facing -x
  planes.emplace_back(1, 0, 0, -2.1); // 3: x = 2.1, facing +x
  planes.emplace_back(0, 1, 0, 0);    // 4: y = 0
  planes.emplace_back(0, 1, 0, -3);   // 5: y = 3
  planes.emplace_back(0, 0, 1, 0);    // 6: z = 0
  planes.emplace_back(0, 0, 1, -2.5); // 7: z = 2.5
  std::vector<size_t> verticals{0, 1, 2, 3, 4, 5};
  std::vector<size_t> horizontals{6, 7};
  std::vector<std::pair<size_t, size_t>> pairs{{2, 3}};
  auto cc = std::make_shared<CellComplex>(
      planes, verticals, horizontals, pairs, std::array<double, 2>{-1.0, -1.0},
      std::array<double, 2>{5.0, 4.0}, std::nullopt, 500);

  cc->n_rooms = 1;
  cc->room_labels = {1};
  auto c_rp = cc->add_property_map<CellComplex::Vertex, std::vector<double>>(
                    "c:room_probabilities")
                  .first;
  std::vector<CellComplex::Vertex> slab, rooms;
  for (auto c = cc->cells_begin(); c != cc->cells_end(); ++c) {
    c_rp[*c] = {0.8, 0.2};
    const auto &p = (*cc)[*c].pos;
    if (p.x() < 0.0 || p.x() > 4.0 || p.y() < 0.0 || p.y() > 3.0)
      continue;
    (std::get<CellData>((*cc)[*c].data).wall_ids.empty() ? rooms : slab)
        .push_back(*c);
  }
  REQUIRE(slab.size() == 1);
  REQUIRE(rooms.size() == 2);

  const auto stats =
      apply_space_prior(*cc, {rectangle_space(1, 0, 0, 4, 3)}, 0.45);
  CHECK(stats.cells == 2);
  CHECK(c_rp[slab.front()] == std::vector<double>{0.8, 0.2});
  for (auto c : rooms)
    CHECK(c_rp[c][1] > c_rp[c][0]);
}

TEST_CASE("SpacePrior_RaysIntoTheOpen_StayOutside", "[geometry][space_prior]") {
  // A cell outside the building under a space polygon that leaks past the
  // walls (raster fill through a window): only the building's outer wall,
  // seen from behind, lies to one side; every other ray escapes the scan.
  // The escaped rays say outside whatever the space says: only the back
  // faces are re-weighed.
  Scan scan;
  scan.add_rect(0, 4.02, {0.0, 0.0}, {3.0, 2.5}, {-1, 0, 0}, 1);
  auto cc = box_cell_complex(scan);
  auto c_esc =
      cc->property_map<CellComplex::Vertex, double>("c:escape_probability");
  const double e = c_esc[*cc->cells_begin()];
  const auto before = probabilities(*cc);
  INFO("escape " << e << ", p_out " << before[0]);
  REQUIRE(e > 0.5);
  REQUIRE(e <= before[0] + 1e-12);

  apply_space_prior(*cc, {rectangle_space(1, 0, 0, 4, 3)}, 0.45);
  const auto after = probabilities(*cc);
  const double wc = 0.45;
  CHECK(after[0] == Catch::Approx((1 - wc) * before[0] + wc * e));
  CHECK(after[0] > after[index_of(*cc, 1)]);
}

TEST_CASE("RoomProbabilities_EscapeShare_IsPartOfOutside",
          "[geometry][space_prior][room_probabilities]") {
  // The occluded pocket's rays all hit something: none escapes.
  auto cc = box_cell_complex(occluded_pocket());
  auto c_esc =
      cc->property_map<CellComplex::Vertex, double>("c:escape_probability");
  CHECK(c_esc[*cc->cells_begin()] == Catch::Approx(0.0));
}
