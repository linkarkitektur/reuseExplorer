// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Split levels (reusex-546): a storey whose floor steps up 0.85 m half way
// (house1's north block sits 0.75 m above its south block). The floor-level
// map finds the raised half, the band-rooms wall band follows each cell's own
// floor, and every space gets its own measured floor and ceiling.

#include <reusex/geometry/transform_utils.hpp>
#include <reusex/segmentation/band_rooms.hpp>
#include <reusex/segmentation/spaces.hpp>
#include <reusex/segmentation/split_level.hpp>
#include <reusex/types/point_types.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <map>
#include <vector>

using namespace reusex;
using namespace reusex::geometry;
using Catch::Matchers::WithinAbs;

namespace {

constexpr double kStep = 0.85;   // the split
constexpr double kLowCeil = 2.7; // ceiling over the lower half
constexpr double kHighCeil = 3.4;

/// Points and normals in the level frame (identity pose).
struct Scan {
  double step = 0.05;
  std::vector<Eigen::Vector3d> pts;
  std::vector<Eigen::Vector3d> nrm;

  void horizontal(double x0, double x1, double y0, double y1, double z,
                  bool down = false) {
    for (double x = x0; x <= x1 + 1e-9; x += step)
      for (double y = y0; y <= y1 + 1e-9; y += step) {
        pts.emplace_back(x, y, z);
        nrm.emplace_back(0, 0, down ? -1 : 1);
      }
  }
  void wall_x(double x, double y0, double y1, double z0, double z1) {
    for (double y = y0; y <= y1 + 1e-9; y += step)
      for (double z = z0; z <= z1 + 1e-9; z += step) {
        pts.emplace_back(x, y, z);
        nrm.emplace_back(1, 0, 0);
      }
  }
  void wall_y(double y, double x0, double x1, double z0, double z1) {
    for (double x = x0; x <= x1 + 1e-9; x += step)
      for (double z = z0; z <= z1 + 1e-9; z += step) {
        pts.emplace_back(x, y, z);
        nrm.emplace_back(0, 1, 0);
      }
  }

  /// Two levels side by side, 4 m deep: the lower half x in [0, 5] (floor
  /// 0, ceiling 2.7), the upper half x in [5, 10] (floor 0.85, ceiling
  /// 3.4), a full-height wall between them at x = 5.
  static Scan two_levels() {
    Scan s;
    s.horizontal(0.05, 4.95, 0.05, 3.95, 0.0);
    s.horizontal(0.05, 4.95, 0.05, 3.95, kLowCeil, true);
    s.horizontal(5.05, 9.95, 0.05, 3.95, kStep);
    s.horizontal(5.05, 9.95, 0.05, 3.95, kHighCeil, true);
    s.wall_x(0.0, 0.0, 4.0, 0.0, kLowCeil);
    s.wall_x(10.0, 0.0, 4.0, kStep, kHighCeil);
    for (const double y : {0.0, 4.0}) {
      s.wall_y(y, 0.0, 5.0, 0.0, kLowCeil);
      s.wall_y(y, 5.0, 10.0, kStep, kHighCeil);
    }
    s.wall_x(5.0, 0.0, 4.0, 0.0, kHighCeil);
    return s;
  }

  [[nodiscard]] std::pair<Cloud, CloudN> build() const {
    Cloud c;
    CloudN n;
    for (size_t i = 0; i < pts.size(); ++i) {
      PointT p;
      p.x = static_cast<float>(pts[i].x());
      p.y = static_cast<float>(pts[i].y());
      p.z = static_cast<float>(pts[i].z());
      c.push_back(p);
      NormalT q;
      q.normal_x = static_cast<float>(nrm[i].x());
      q.normal_y = static_cast<float>(nrm[i].y());
      q.normal_z = static_cast<float>(nrm[i].z());
      n.push_back(q);
    }
    return {c, n};
  }

  [[nodiscard]] std::vector<double> abs_nz() const {
    std::vector<double> out;
    for (const auto &v : nrm)
      out.push_back(std::abs(v.z()));
    return out;
  }
};

core::StoreyRecord storey() {
  core::StoreyRecord s;
  s.idx = 0;
  s.z_floor = 0.0;
  s.z_ceiling = kHighCeil;
  s.level_transform = to_array16(Eigen::Affine3d::Identity());
  return s;
}

/// The grid: x in [-1, 11), y in [-1, 5) at 5 cm.
struct Grid {
  int col0 = -20, row0 = -20, width = 240, height = 120;
  double cell = 0.05;
  [[nodiscard]] size_t at(double x, double y) const {
    const int c = static_cast<int>(std::floor(x / cell)) - col0;
    const int r = static_cast<int>(std::floor(y / cell)) - row0;
    return static_cast<size_t>(r) * static_cast<size_t>(width) +
           static_cast<size_t>(c);
  }
};

/// Room label of the floor point nearest level (x, y) at height z.
uint32_t room_at(const Scan &s, const CloudL &labels, double x, double y,
                 double z) {
  size_t best = 0;
  double d = 1e9;
  for (size_t i = 0; i < s.pts.size(); ++i) {
    const double e = (s.pts[i] - Eigen::Vector3d(x, y, z)).norm();
    if (e < d) {
      d = e;
      best = i;
    }
  }
  return labels[best].label;
}

} // namespace

TEST_CASE("SplitLevel_TwoLevelBox_FindsBothFloors",
          "[segmentation][split_level]") {
  const Scan s = Scan::two_levels();
  const Grid g;
  const auto map = floor_level_map(s.pts, s.abs_nz(), {}, 0.0, g.col0, g.row0,
                                   g.width, g.height, g.cell);
  REQUIRE(map.split());
  REQUIRE(map.levels.size() == 2);
  CHECK(map.levels[0].base);
  CHECK(map.levels[0].z == 0.0);
  CHECK_FALSE(map.levels[1].base);
  CHECK_THAT(map.levels[1].z, WithinAbs(kStep, 0.02));
  // Each half is ~20 m^2 (5 x 4), the fill reaching the walls around it.
  CHECK_THAT(map.levels[1].area_m2, WithinAbs(20.0, 3.0));
  CHECK_THAT(map.levels[0].area_m2, WithinAbs(20.0, 3.0));
  CHECK_THAT(map.floor_z[g.at(7.0, 2.0)], WithinAbs(kStep, 0.02));
  CHECK(map.floor_z[g.at(2.0, 2.0)] == 0.0F);
  // The outer wall of the raised half stands on the raised floor.
  CHECK_THAT(map.floor_z[g.at(10.0, 2.0)], WithinAbs(kStep, 0.02));
  CHECK(map.floor_z[g.at(0.0, 2.0)] == 0.0F);
}

TEST_CASE("SplitLevel_DesksAtTheStepHeight_AreNoLevel",
          "[segmentation][split_level]") {
  // One flat office, 8 x 6 m, with four 1.6 x 0.8 m desks at 0.75 m and the
  // floor unseen beneath them: desk tops are not a raised floor.
  Scan s;
  struct Desk {
    double x0, y0;
  };
  const std::vector<Desk> desks = {
      {1.0, 1.0}, {1.0, 3.0}, {4.0, 1.0}, {4.0, 3.0}};
  const auto under_desk = [&](double x, double y) {
    return std::any_of(desks.begin(), desks.end(), [&](const Desk &d) {
      return x >= d.x0 && x <= d.x0 + 1.6 && y >= d.y0 && y <= d.y0 + 0.8;
    });
  };
  for (double x = 0.05; x <= 7.95 + 1e-9; x += s.step)
    for (double y = 0.05; y <= 5.95 + 1e-9; y += s.step)
      if (!under_desk(x, y)) {
        s.pts.emplace_back(x, y, 0.0);
        s.nrm.emplace_back(0, 0, 1);
      }
  for (const auto &d : desks)
    s.horizontal(d.x0, d.x0 + 1.6, d.y0, d.y0 + 0.8, 0.75);
  s.horizontal(0.05, 7.95, 0.05, 5.95, 2.6, true);
  const Grid g;
  const auto map = floor_level_map(s.pts, s.abs_nz(), {}, 0.0, g.col0, g.row0,
                                   g.width, g.height, g.cell);
  CHECK_FALSE(map.split());
  REQUIRE(map.levels.size() == 1);
  CHECK(std::all_of(map.floor_z.begin(), map.floor_z.end(),
                    [](float z) { return z == 0.0F; }));
}

TEST_CASE("SplitLevel_FloorLevelMap_IgnoresPointOrder",
          "[segmentation][split_level]") {
  Scan s = Scan::two_levels();
  const Grid g;
  const auto a = floor_level_map(s.pts, s.abs_nz(), {}, 0.0, g.col0, g.row0,
                                 g.width, g.height, g.cell);
  std::reverse(s.pts.begin(), s.pts.end());
  std::reverse(s.nrm.begin(), s.nrm.end());
  const auto b = floor_level_map(s.pts, s.abs_nz(), {}, 0.0, g.col0, g.row0,
                                 g.width, g.height, g.cell);
  CHECK(a.floor_z == b.floor_z);
  REQUIRE(a.levels.size() == b.levels.size());
  CHECK(a.levels[1].z == b.levels[1].z);
}

TEST_CASE("SplitLevel_BandRooms_WallBandFollowsTheLocalFloor",
          "[segmentation][split_level][band_rooms]") {
  // A 1.0 m high counter row across the raised half (x = 7.5, y 0.6-3.4):
  // above the raised floor it is furniture, below the band. Measured from
  // the storey floor the band (1.2-2.3 m) would catch its top 0.65 m and the
  // 0.6 m gaps at its ends would cut the raised half in two.
  Scan s = Scan::two_levels();
  s.wall_x(7.5, 0.6, 3.4, kStep, kStep + 1.0);
  const auto [cloud, normals] = s.build();

  BandRoomsOptions o;
  o.use_wall_graph = false;
  o.cut_at_doors = false;
  const auto split = segment_band_rooms(cloud, normals, {storey()}, o);
  REQUIRE(split.grids.size() == 1);
  CHECK(split.grids[0].floor_levels.size() == 2);
  REQUIRE_FALSE(split.grids[0].floor_z.empty());
  const auto &l = *split.labels;
  const uint32_t low = room_at(s, l, 2.5, 2.0, 0.0);
  const uint32_t west = room_at(s, l, 6.2, 2.0, kStep);
  const uint32_t east = room_at(s, l, 9.0, 2.0, kStep);
  CHECK(low > 0);
  CHECK(west > 0);
  CHECK(west == east);
  CHECK(low != west);

  // Without split levels the counter row cuts the raised half.
  o.split_levels = false;
  const auto flat = segment_band_rooms(cloud, normals, {storey()}, o);
  CHECK(flat.grids[0].floor_z.empty());
  CHECK(room_at(s, *flat.labels, 6.2, 2.0, kStep) !=
        room_at(s, *flat.labels, 9.0, 2.0, kStep));
}

TEST_CASE("SplitLevel_FlatStorey_BandRoomsUnchanged",
          "[segmentation][split_level][band_rooms]") {
  // No raised part: the grids and labels are the same with and without
  // split levels.
  Scan s;
  s.horizontal(0.05, 5.95, 0.05, 3.95, 0.0);
  s.horizontal(0.05, 5.95, 0.05, 3.95, 2.6, true);
  s.wall_x(0.0, 0.0, 4.0, 0.0, 2.6);
  s.wall_x(6.0, 0.0, 4.0, 0.0, 2.6);
  s.wall_y(0.0, 0.0, 6.0, 0.0, 2.6);
  s.wall_y(4.0, 0.0, 6.0, 0.0, 2.6);
  s.wall_x(3.0, 0.0, 1.6, 0.0, 2.6);
  s.wall_x(3.0, 2.4, 4.0, 0.0, 2.6);
  const auto [cloud, normals] = s.build();
  core::StoreyRecord st = storey();
  st.z_ceiling = 2.6;
  BandRoomsOptions o;
  const auto a = segment_band_rooms(cloud, normals, {st}, o);
  o.split_levels = false;
  const auto b = segment_band_rooms(cloud, normals, {st}, o);
  CHECK(a.grids[0].floor_z.empty());
  CHECK(a.grids[0].floor_levels.size() == 1);
  CHECK(a.grids[0].labels == b.grids[0].labels);
  CHECK(a.grids[0].wall == b.grids[0].wall);
  CHECK(a.grids[0].footprint == b.grids[0].footprint);
}

TEST_CASE("SplitLevel_Spaces_TakeTheirOwnFloorAndCeiling",
          "[segmentation][split_level][spaces]") {
  const Scan s = Scan::two_levels();
  const auto [cloud, normals] = s.build();
  BandRoomsOptions o;
  o.use_wall_graph = false;
  o.cut_at_doors = false;
  const auto rooms = segment_band_rooms(cloud, normals, {storey()}, o);
  auto sp = polygonise_spaces(rooms.grids, {storey()}, {});
  REQUIRE(sp.spaces.size() == 2);
  // Before measuring, both carry the storey's floor and ceiling.
  for (const auto &x : sp.spaces) {
    CHECK(x.z_floor == 0.0);
    CHECK(x.z_ceiling == kHighCeil);
  }
  const auto stats =
      measure_space_heights(sp.spaces, rooms.grids, {storey()}, cloud, normals);
  CHECK(stats.floors_measured == 2);
  CHECK(stats.ceilings_measured == 2);
  std::sort(sp.spaces.begin(), sp.spaces.end(),
            [](const auto &a, const auto &b) { return a.z_floor < b.z_floor; });
  const auto &low = sp.spaces[0];
  const auto &high = sp.spaces[1];
  CHECK_THAT(low.z_floor, WithinAbs(0.0, 0.01));
  CHECK_THAT(low.z_ceiling, WithinAbs(kLowCeil, 0.01));
  CHECK_THAT(high.z_floor, WithinAbs(kStep, 0.01));
  CHECK_THAT(high.z_ceiling, WithinAbs(kHighCeil, 0.01));
  for (const auto *x : {&low, &high}) {
    CHECK_THAT(x->volume,
               WithinAbs(x->area * (x->z_ceiling - x->z_floor), 1e-9));
    CHECK(x->metadata.find("\"z_floor_cells\":") != std::string::npos);
  }

  // Off: the storey's heights stay.
  auto off = polygonise_spaces(rooms.grids, {storey()}, {});
  SpacesOptions so;
  so.measure_heights = false;
  measure_space_heights(off.spaces, rooms.grids, {storey()}, cloud, normals,
                        so);
  for (const auto &x : off.spaces) {
    CHECK(x.z_floor == 0.0);
    CHECK(x.z_ceiling == kHighCeil);
  }
}
