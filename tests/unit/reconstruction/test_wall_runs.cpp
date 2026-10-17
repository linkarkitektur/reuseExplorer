// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Slab cells on their wall's measured run (reusex-9a8). CellComplex marks
// every cell between a pair's two faces as the wall's slab, along the
// pair's whole infinite line: in house2's stair hall such strips run
// through the pocket under the stair, and the space prior, which leaves a
// wall's slab alone, could not reach it. wall_runs() measures where the
// pair's points are along its axis; CellComplex::mark_wall_runs() keeps a
// pair in a cell's run_wall_ids only there. wall_ids (the MIP's slab) stay
// along the whole line.

#include <reusex/reconstruction/CellComplex.hpp>
#include <reusex/reconstruction/mesh.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>

#include <array>
#include <cmath>
#include <memory>
#include <set>
#include <stdexcept>
#include <utility>
#include <vector>

using namespace reusex::geometry;
using reusex::EigenVectorContainer;
using reusex::IndicesPtr;

namespace {

using PlaneVec =
    std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>>;

/// The cell whose centre lies nearest (x, y) in plan.
CellComplex::Vertex cell_at(const CellComplex &cc, double x, double y) {
  CellComplex::Vertex best{};
  double best_d = 1e300;
  for (auto c = cc.cells_begin(); c != cc.cells_end(); ++c) {
    const auto &p = cc[*c].pos;
    const double d = std::hypot(p.x() - x, p.y() - y);
    if (d < best_d) {
      best_d = d;
      best = *c;
    }
  }
  return best;
}

const std::set<int> &wall_ids(const CellComplex &cc, CellComplex::Vertex c) {
  return std::get<CellData>(cc[c].data).wall_ids;
}

const std::set<int> &run_ids(const CellComplex &cc, CellComplex::Vertex c) {
  return std::get<CellData>(cc[c].data).run_wall_ids;
}

/// A long room 0..10 x 0..4 m (single faces, no pairs), a line at x = 6
/// (another wall's face, so the strip is split there) and a 0.2 m wall
/// along y = 2 (faces y = 1.9 facing -y and y = 2.1 facing +y, pair 0).
std::shared_ptr<CellComplex> long_room_with_a_short_wall() {
  PlaneVec planes;
  planes.emplace_back(1, 0, 0, 0);    // 0: x = 0
  planes.emplace_back(1, 0, 0, -10);  // 1: x = 10
  planes.emplace_back(0, 1, 0, 0);    // 2: y = 0
  planes.emplace_back(0, 1, 0, -4);   // 3: y = 4
  planes.emplace_back(1, 0, 0, -6);   // 4: x = 6
  planes.emplace_back(0, -1, 0, 1.9); // 5: y = 1.9, facing -y
  planes.emplace_back(0, 1, 0, -2.1); // 6: y = 2.1, facing +y
  planes.emplace_back(0, 0, 1, 0);    // 7: z = 0
  planes.emplace_back(0, 0, 1, -2.5); // 8: z = 2.5
  std::vector<size_t> verticals{0, 1, 2, 3, 4, 5, 6};
  std::vector<size_t> horizontals{7, 8};
  std::vector<std::pair<size_t, size_t>> pairs{{5, 6}};
  return std::make_shared<CellComplex>(
      planes, verticals, horizontals, pairs, std::array<double, 2>{-1.0, -1.0},
      std::array<double, 2>{11.0, 5.0}, std::nullopt, 500);
}

} // namespace

TEST_CASE("WallRuns_ShortWallInALongRoom_OnlyItsRunIsTheWall",
          "[geometry][cellcomplex][wall_runs]") {
  // The wall runs from x = 0 to 3; beyond it the room is open. The strip
  // between its faces over x 0..6 holds the wall; the strip over 6..10 is
  // open floor on the wall's line. Both stay the MIP's slab (a room may
  // still end on the line there), only the first is on the run.
  auto cc = long_room_with_a_short_wall();
  const auto run_cell = cell_at(*cc, 3.0, 2.0);
  const auto open_cell = cell_at(*cc, 8.0, 2.0);
  REQUIRE(wall_ids(*cc, run_cell) == std::set<int>{0});
  REQUIRE(wall_ids(*cc, open_cell) == std::set<int>{0}); // the infinite line
  REQUIRE(run_ids(*cc, open_cell) == std::set<int>{0});  // until measured

  WallRun run;
  run.axis = {1.0, 0.0};
  run.spans = {{0.0, 3.0}};
  const auto stats = cc->mark_wall_runs({run}, 0.5);
  CHECK(run_ids(*cc, run_cell) == std::set<int>{0});
  CHECK(run_ids(*cc, open_cell).empty());
  CHECK(wall_ids(*cc, run_cell) == std::set<int>{0});
  CHECK(wall_ids(*cc, open_cell) == std::set<int>{0});
  CHECK(stats.pairs_measured == 1);
  CHECK(stats.cells_off_run == 1);
  CHECK(stats.volume_off_run == Catch::Approx(4.0 * 0.2 * 2.5));
  // The cells either side of the strip carry no wall id either way.
  CHECK(run_ids(*cc, cell_at(*cc, 8.0, 1.0)).empty());
  CHECK(run_ids(*cc, cell_at(*cc, 8.0, 3.0)).empty());
}

TEST_CASE("WallRuns_RunEndingNearALine_KeepsTheStripItReaches",
          "[geometry][cellcomplex][wall_runs]") {
  // A wall measured to x = 5.7 ends 0.3 m short of the line at x = 6 (a
  // corner its points stop short of): the margin reaches past the line,
  // so the strip beyond counts as the wall too.
  auto cc = long_room_with_a_short_wall();
  WallRun run;
  run.axis = {1.0, 0.0};
  run.spans = {{0.0, 5.7}};
  cc->mark_wall_runs({run}, 0.5);
  CHECK(run_ids(*cc, cell_at(*cc, 3.0, 2.0)) == std::set<int>{0});
  CHECK(run_ids(*cc, cell_at(*cc, 8.0, 2.0)) == std::set<int>{0});
}

TEST_CASE("WallRuns_NoRunOrNoAxis_KeepsTheWholeLine",
          "[geometry][cellcomplex][wall_runs]") {
  // A pair with no measured run (no runs given, or a floor/ceiling pair,
  // whose axis is zero) is the wall along its whole line, as before.
  auto cc = long_room_with_a_short_wall();
  SECTION("no runs") { CHECK(cc->mark_wall_runs({}, 0.5).cells_off_run == 0); }
  SECTION("zero axis") {
    WallRun run;
    run.spans = {{0.0, 3.0}};
    CHECK(cc->mark_wall_runs({run}, 0.5).cells_off_run == 0);
  }
  SECTION("no spans") {
    WallRun run;
    run.axis = {1.0, 0.0};
    CHECK(cc->mark_wall_runs({run}, 0.5).cells_off_run == 0);
  }
  CHECK(run_ids(*cc, cell_at(*cc, 8.0, 2.0)) == std::set<int>{0});
}

TEST_CASE("WallRuns_MoreRunsThanPairs_Throws",
          "[geometry][cellcomplex][wall_runs]") {
  auto cc = long_room_with_a_short_wall();
  CHECK_THROWS_AS(cc->mark_wall_runs({WallRun{}, WallRun{}}, 0.5),
                  std::invalid_argument);
}

TEST_CASE("WallRuns_PointsAlongTheAxis_SplitAtGapsWiderThanADoor",
          "[geometry][wall_runs]") {
  // A wall along x at y = 1.9 / 2.1, scanned over x 0..2, 3..5 (a 1 m
  // doorway, bridged) and 7.5..9 (a 2.5 m gap: open floor, not bridged).
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  auto face = std::make_shared<pcl::Indices>();
  auto back = std::make_shared<pcl::Indices>();
  auto add = [&](double x0, double x1, double y, pcl::Indices &idx) {
    for (double x = x0; x <= x1 + 1e-9; x += 0.05)
      for (double z = 0.1; z < 2.4; z += 0.5) {
        pcl::PointXYZRGB p;
        p.x = static_cast<float>(x);
        p.y = static_cast<float>(y);
        p.z = static_cast<float>(z);
        idx.push_back(static_cast<int>(cloud->size()));
        cloud->push_back(p);
      }
  };
  add(0.0, 2.0, 1.9, *face);
  add(3.0, 5.0, 1.9, *face);
  add(7.5, 9.0, 2.1, *back);
  EigenVectorContainer<double, 4> planes;
  planes.emplace_back(0, -1, 0, 1.9);
  planes.emplace_back(0, 1, 0, -2.1);
  planes.emplace_back(0, 0, 1, 0); // a floor and its invented underside
  planes.emplace_back(0, 0, -1, -0.2);
  std::vector<IndicesPtr> inliers{face, back, std::make_shared<pcl::Indices>(),
                                  std::make_shared<pcl::Indices>()};
  const std::vector<std::pair<size_t, size_t>> pairs{{0, 1}, {2, 3}};

  const auto runs = wall_runs(planes, inliers, cloud, pairs, 1.5);
  REQUIRE(runs.size() == 2);
  // The axis is the first face's normal turned +90 deg about z.
  CHECK(runs[0].axis.x() == Catch::Approx(1.0));
  CHECK(runs[0].axis.y() == Catch::Approx(0.0).margin(1e-12));
  REQUIRE(runs[0].spans.size() == 2);
  CHECK(runs[0].spans[0][0] == Catch::Approx(0.0).margin(1e-6));
  CHECK(runs[0].spans[0][1] == Catch::Approx(5.0).margin(1e-6));
  CHECK(runs[0].spans[1][0] == Catch::Approx(7.5).margin(1e-6));
  CHECK(runs[0].spans[1][1] == Catch::Approx(9.0).margin(1e-6));
  // Not a wall: no axis, the slab is not limited.
  CHECK(runs[1].axis.isZero());
  CHECK(runs[1].spans.empty());
}
