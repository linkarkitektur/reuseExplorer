// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Rooms from a clutter-free height band (reusex-rqn.2, W4.1 reusex-z1v.6.1):
// the 2D partition on synthetic occupancy grids, then the full per-storey
// path on synthetic scans.

#include <reusex/geometry/transform_utils.hpp>
#include <reusex/segmentation/band_rooms.hpp>
#include <reusex/types/point_types.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <map>
#include <random>
#include <set>
#include <string>
#include <vector>

using namespace reusex;
using namespace reusex::geometry;
using Catch::Approx;

namespace {

// ---------------------------------------------------------------------------
// Grid-level helpers: a grid in metres, cell 5 cm, origin at level (0, 0).
// ---------------------------------------------------------------------------

struct Plan {
  RoomGrid g;
  double cell = 0.05;

  /// A grid covering level x in [x0, x1), y in [y0, y1).
  Plan(double x0, double x1, double y0, double y1) {
    g.cell_size = cell;
    g.col0 = static_cast<int>(std::floor(x0 / cell));
    g.row0 = static_cast<int>(std::floor(y0 / cell));
    g.resize(static_cast<int>(std::ceil((x1 - x0) / cell)),
             static_cast<int>(std::ceil((y1 - y0) / cell)));
  }

  [[nodiscard]] int col(double x) const {
    return static_cast<int>(std::floor(x / cell)) - g.col0;
  }
  [[nodiscard]] int row(double y) const {
    return static_cast<int>(std::floor(y / cell)) - g.row0;
  }

  void fill(std::vector<uint8_t> &m, double x0, double x1, double y0, double y1,
            uint8_t v = 1) {
    for (int r = std::max(0, row(y0)); r < std::min(g.height, row(y1)); ++r)
      for (int c = std::max(0, col(x0)); c < std::min(g.width, col(x1)); ++c)
        m[g.index(c, r)] = v;
  }
  void floor(double x0, double x1, double y0, double y1) {
    fill(g.footprint, x0, x1, y0, y1);
  }
  /// A 10 cm wall from (x0, y0) to (x1, y1), axis-aligned.
  void wall(double x0, double x1, double y0, double y1) {
    fill(g.wall, x0 - 0.05, x1 + 0.05, y0 - 0.05, y1 + 0.05);
  }

  [[nodiscard]] int32_t at(double x, double y) const {
    return g.labels[g.index(col(x), row(y))];
  }
};

std::set<int32_t> room_ids(const RoomGrid &g) {
  std::set<int32_t> s;
  for (int32_t l : g.labels)
    if (l > 0)
      s.insert(l);
  return s;
}

double room_area(const RoomGrid &g, int32_t id) {
  return static_cast<double>(std::count(g.labels.begin(), g.labels.end(), id)) *
         g.cell_size * g.cell_size;
}

// ---------------------------------------------------------------------------
// Point-level helpers: patches in the level frame, moved to world by a pose.
// ---------------------------------------------------------------------------

struct Scan {
  Eigen::Affine3d pose = Eigen::Affine3d::Identity();
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
  /// A storey slab: floor at z0 and ceiling at z0 + h over the rectangle.
  void slab(double x0, double x1, double y0, double y1, double z0 = 0.0,
            double h = 2.6) {
    horizontal(x0, x1, y0, y1, z0);
    horizontal(x0, x1, y0, y1, z0 + h, true);
  }
  /// Outer walls of a rectangle, full height.
  void box(double x0, double x1, double y0, double y1, double z0 = 0.0,
           double h = 2.6) {
    wall_x(x0, y0, y1, z0, z0 + h);
    wall_x(x1, y0, y1, z0, z0 + h);
    wall_y(y0, x0, x1, z0, z0 + h);
    wall_y(y1, x0, x1, z0, z0 + h);
  }

  [[nodiscard]] std::pair<Cloud, CloudN> build() const {
    Cloud c;
    CloudN n;
    for (size_t i = 0; i < pts.size(); ++i) {
      const Eigen::Vector3d w = pose * pts[i];
      const Eigen::Vector3d wn = pose.linear() * nrm[i];
      PointT p;
      p.x = static_cast<float>(w.x());
      p.y = static_cast<float>(w.y());
      p.z = static_cast<float>(w.z());
      c.push_back(p);
      NormalT q;
      q.normal_x = static_cast<float>(wn.x());
      q.normal_y = static_cast<float>(wn.y());
      q.normal_z = static_cast<float>(wn.z());
      n.push_back(q);
    }
    return {c, n};
  }

  [[nodiscard]] core::StoreyRecord storey(int idx = 0, double zf = 0.0,
                                          double zc = 2.6) const {
    core::StoreyRecord s;
    s.idx = idx;
    s.z_floor = zf;
    s.z_ceiling = zc;
    s.level_transform = to_array16(pose.inverse());
    return s;
  }

  /// Index of the first point within 1 cm of level (x, y, z).
  [[nodiscard]] size_t near(double x, double y, double z) const {
    for (size_t i = 0; i < pts.size(); ++i)
      if ((pts[i] - Eigen::Vector3d(x, y, z)).norm() < 0.03)
        return i;
    FAIL("no point near (" << x << ", " << y << ", " << z << ")");
    return 0;
  }
};

/// Majority room of the floor points (z = 0) inside a level rectangle.
uint32_t floor_room(const Scan &s, const CloudL &labels, double x0, double x1,
                    double y0, double y1) {
  std::map<uint32_t, int> votes;
  for (size_t i = 0; i < s.pts.size(); ++i) {
    const auto &p = s.pts[i];
    if (std::abs(p.z()) < 1e-6 && p.x() > x0 && p.x() < x1 && p.y() > y0 &&
        p.y() < y1)
      ++votes[labels[i].label];
  }
  uint32_t best = 0;
  int n = -1;
  for (auto [l, c] : votes)
    if (c > n) {
      best = l;
      n = c;
    }
  return best;
}

} // namespace

// ===========================================================================
// 2D partition on synthetic occupancy grids
// ===========================================================================

TEST_CASE("BandRooms_Grid_TwoRoomsJoinedByADoor_AreTwoRooms",
          "[segmentation][band_rooms]") {
  // Two 4 x 4 m rooms side by side, the shared wall at x = 4 has a 0.9 m door.
  Plan p(-1, 10, -1, 5);
  p.floor(0, 8, 0, 4);
  p.wall(0, 0, 0, 4);
  p.wall(8, 8, 0, 4);
  p.wall(0, 8, 0, 0);
  p.wall(0, 8, 4, 4);
  p.wall(4, 4, 0, 1.5);
  p.wall(4, 4, 2.4, 4);

  const int n = partition_room_grid(p.g, BandRoomsOptions{});
  CHECK(n == 2);
  CHECK(p.g.rooms == 2);
  const int32_t a = p.at(2, 2);
  const int32_t b = p.at(6, 2);
  CHECK(a > 0);
  CHECK(b > 0);
  CHECK(a != b);
  // Each room keeps nearly all of its 16 m^2 (minus the dilated walls).
  CHECK(room_area(p.g, a) > 13.0);
  CHECK(room_area(p.g, b) > 13.0);
  // The cut lies in the door: no cell of room a east of the wall.
  for (int r = 0; r < p.g.height; ++r)
    for (int c = p.col(4.2); c < p.g.width; ++c)
      CHECK(p.g.labels[p.g.index(c, r)] != a);
  // The exterior stays empty.
  CHECK(p.at(-0.5, -0.5) == 0);
  CHECK(p.at(9.5, 2) == 0);
}

TEST_CASE("BandRooms_Grid_FurnitureHolesInTheFootprint_AreBridged",
          "[segmentation][band_rooms]") {
  // One 5 x 4 m room whose floor evidence has holes (desk shadows) and a
  // notch at the wall; the exterior flood cannot reach the holes.
  Plan p(-1, 6, -1, 5);
  p.floor(0, 5, 0, 4);
  p.fill(p.g.footprint, 1, 2, 1, 2, 0);
  p.fill(p.g.footprint, 3, 4.2, 2.5, 3.2, 0);
  p.wall(0, 0, 0, 4);
  p.wall(5, 5, 0, 4);
  p.wall(0, 5, 0, 0);
  p.wall(0, 5, 4, 4);

  CHECK(partition_room_grid(p.g, BandRoomsOptions{}) == 1);
  CHECK(p.g.interior[p.g.index(p.col(1.5), p.row(1.5))] == 1);
  CHECK(p.at(1.5, 1.5) == p.at(4.5, 0.5));
  CHECK(p.at(3.5, 2.8) == p.at(4.5, 0.5));
  CHECK(p.g.interior_area == Approx(5.0 * 4.0).margin(1.0));
}

TEST_CASE("BandRooms_Grid_NarrowWcStalls_AreFloodedByTheCorridor",
          "[segmentation][band_rooms]") {
  // Known limit (ideas-lab §4.1): a stall narrower than 2 x core_radius has
  // no core, so the watershed floods it from the corridor. In practice the
  // bar is higher than 1 m: the core must also clear min_core_area, so with
  // 10 cm partitions a 2 m deep stall needs about 1.5 m of width; a 1.6 m
  // stall keeps its own room. Corridor y in [0, 1.6], stalls north of it,
  // 2 m deep, each with a 0.6 m door.
  Plan p(-1, 7, -1, 4.6);
  p.floor(0, 6, 0, 3.6);
  p.wall(0, 0, 0, 3.6);
  p.wall(6, 6, 0, 3.6);
  p.wall(0, 6, 0, 0);
  p.wall(0, 6, 3.6, 3.6);
  // Stall partitions at x = 0.9, 1.8, 2.7 (three 0.9 m stalls) and 4.3
  // (one 1.6 m stall from 2.7 to 4.3); the rest of the north side is open.
  for (double x : {0.9, 1.8, 2.7, 4.3})
    p.wall(x, x, 1.6, 3.6);
  // Stall fronts at y = 1.6 with a 0.6 m door each.
  p.wall(0, 0.15, 1.6, 1.6);
  p.wall(0.75, 1.05, 1.6, 1.6);
  p.wall(1.65, 1.95, 1.6, 1.6);
  p.wall(2.55, 2.85, 1.6, 1.6);
  p.wall(3.45, 4.3, 1.6, 1.6);

  partition_room_grid(p.g, BandRoomsOptions{});
  const int32_t corridor = p.at(3, 0.8);
  REQUIRE(corridor > 0);
  // The 0.9 m stalls carry no room of their own.
  for (double x : {0.45, 1.35, 2.25}) {
    const int32_t s = p.at(x, 2.6);
    CHECK((s == 0 || s == corridor));
  }
  // The 1.6 m stall does.
  const int32_t wide = p.at(3.5, 2.6);
  CHECK(wide > 0);
  CHECK(wide != corridor);
}

TEST_CASE("BandRooms_Grid_PocketBehindAPost_MergesIntoItsRoom",
          "[segmentation][band_rooms]") {
  // reusex-3r1: a 5 x 1.9 m room with one free-standing post (a column, a
  // tall cabinet end) 1.6 m from its east wall. The free space either side of
  // the post is narrower than 2 x core_radius, so the east end gets a core of
  // its own and the watershed splits it off -- but it meets the rest of the
  // room through free space along much of its boundary, not through a door.
  Plan p(-1, 6, -1, 3);
  p.floor(0, 5, 0, 1.9);
  p.wall(0, 0, 0, 1.9);
  p.wall(5, 5, 0, 1.9);
  p.wall(0, 5, 0, 0);
  p.wall(0, 5, 1.9, 1.9);
  p.wall(3.4, 3.4, 0.9, 1.0);

  // Without the pocket merge the east end is a room of its own.
  BandRoomsOptions off;
  off.pocket_min_contact_share = 0.0;
  RoomGrid split = p.g;
  CHECK(partition_room_grid(split, off) == 2);

  CHECK(partition_room_grid(p.g, BandRoomsOptions{}) == 1);
  CHECK(p.g.merged_pockets == 1);
  CHECK(p.at(4.4, 0.5) == p.at(1.5, 1.0));
  CHECK(p.at(4.4, 0.5) > 0);
}

TEST_CASE("BandRooms_Grid_SmallRoomBehindADoor_IsNoPocket",
          "[segmentation][band_rooms]") {
  // A 1.7 x 1.8 m room (a WC) off a 4 m hall through a 0.8 m door: it meets
  // the hall through the door only, a small share of its boundary.
  Plan p(-1, 7, -1, 3);
  p.floor(0, 5.8, 0, 1.9);
  p.wall(0, 0, 0, 1.9);
  p.wall(5.8, 5.8, 0, 1.9);
  p.wall(0, 5.8, 0, 0);
  p.wall(0, 5.8, 1.9, 1.9);
  p.wall(4.0, 4.0, 0, 0.55);
  p.wall(4.0, 4.0, 1.35, 1.9);

  CHECK(partition_room_grid(p.g, BandRoomsOptions{}) == 2);
  CHECK(p.g.merged_pockets == 0);
  CHECK(p.at(4.9, 1.0) != p.at(2.0, 1.0));
}

TEST_CASE("BandRooms_Grid_SmallRegionsAndCoresAreDropped",
          "[segmentation][band_rooms]") {
  // A 0.9 x 0.9 m closet is below min_room_area (1 m^2) and has no core.
  Plan p(-1, 3, -1, 3);
  p.floor(0, 0.9, 0, 0.9);
  p.wall(0, 0, 0, 0.9);
  p.wall(0.9, 0.9, 0, 0.9);
  p.wall(0, 0.9, 0, 0);
  p.wall(0, 0.9, 0.9, 0.9);
  CHECK(partition_room_grid(p.g, BandRoomsOptions{}) == 0);
  CHECK(room_ids(p.g).empty());
}

TEST_CASE("BandRooms_Grid_IsDeterministic", "[segmentation][band_rooms]") {
  Plan p(-1, 10, -1, 5);
  p.floor(0, 8, 0, 4);
  p.wall(0, 8, 0, 0);
  p.wall(0, 8, 4, 4);
  p.wall(4, 4, 0, 1.5);
  p.wall(4, 4, 2.4, 4);
  RoomGrid a = p.g;
  RoomGrid b = p.g;
  partition_room_grid(a, BandRoomsOptions{}, 7);
  partition_room_grid(b, BandRoomsOptions{}, 7);
  CHECK(a.labels == b.labels);
  CHECK(a.clearance == b.clearance);
  // Ids start at first_label.
  CHECK(*room_ids(a).begin() == 7);
}

// ===========================================================================
// Full path on synthetic scans
// ===========================================================================

TEST_CASE("BandRooms_Scan_TwoRoomsAndADoor", "[segmentation][band_rooms]") {
  Scan s;
  s.slab(0, 8, 0, 4);
  s.box(0, 8, 0, 4);
  s.wall_x(4, 0, 1.5, 0, 2.6);
  s.wall_x(4, 2.4, 4, 0, 2.6);
  s.wall_x(4, 1.5, 2.4, 2.1, 2.6); // lintel above the door
  auto [cloud, normals] = s.build();

  const auto r = segment_band_rooms(cloud, normals, {s.storey()});
  REQUIRE(r.labels);
  REQUIRE(r.labels->size() == cloud.size());
  CHECK(r.rooms == 2);
  REQUIRE(r.grids.size() == 1);
  const uint32_t a = floor_room(s, *r.labels, 0.5, 3.5, 0.5, 3.5);
  const uint32_t b = floor_room(s, *r.labels, 4.5, 7.5, 0.5, 3.5);
  CHECK(a > 0);
  CHECK(b > 0);
  CHECK(a != b);
  // Wall and ceiling points take a room too (no unlabeled scan point here).
  CHECK((*r.labels)[s.near(0, 2, 1.5)].label == a);
  CHECK((*r.labels)[s.near(6, 2, 2.6)].label == b);
  CHECK(r.unlabeled_points == 0);
}

TEST_CASE("BandRooms_Scan_OpenLPlanWithDeskClutter_IsOneRoom",
          "[segmentation][band_rooms]") {
  // L-shaped open office: [0,10]x[0,4] plus [0,4]x[4,9]. Desk rows with
  // 1.15 m screens cross both legs, leaving 0.6 m aisles: below the band.
  Scan s;
  s.slab(0, 10, 0, 4);
  s.slab(0, 4, 4.05, 9);
  s.wall_y(0, 0, 10, 0, 2.6);
  s.wall_x(10, 0, 4, 0, 2.6);
  s.wall_y(4, 4, 10, 0, 2.6);
  s.wall_x(4, 4, 9, 0, 2.6);
  s.wall_y(9, 0, 4, 0, 2.6);
  s.wall_x(0, 0, 9, 0, 2.6);
  for (double x : {3.0, 6.0, 8.0}) { // screens across the east leg
    s.wall_x(x, 0.6, 3.4, 0, 1.15);
    s.horizontal(x - 0.4, x + 0.4, 0.6, 3.4, 0.75); // desk tops
  }
  for (double y : {5.5, 7.5}) // screens across the north leg
    s.wall_y(y, 0.6, 3.4, 0, 1.15);
  auto [cloud, normals] = s.build();

  const auto r = segment_band_rooms(cloud, normals, {s.storey()});
  CHECK(r.rooms == 1);
  const uint32_t a = floor_room(s, *r.labels, 8.5, 9.5, 0.5, 3.5);
  CHECK(a > 0);
  CHECK(floor_room(s, *r.labels, 0.5, 3.5, 8, 8.9) == a);

  // Control: a band reaching down to the desks cuts the office at the
  // screen rows (the clutter the band is there to avoid).
  BandRoomsOptions low;
  low.band_bottom = 0.3;
  const auto rl = segment_band_rooms(cloud, normals, {s.storey()}, low);
  CHECK(rl.rooms > 1);
}

TEST_CASE("BandRooms_Scan_GlassPartitionOnMullions_SeparatesTheRoom",
          "[segmentation][band_rooms]") {
  // A 10 x 6 m office; a meeting room [6,10]x[0,4] behind a glass partition:
  // the glass returns no points, only 5 cm mullions every 0.9 m (full
  // height) and a 1 m door. reusex-k89: visibility sees through it.
  Scan s;
  s.slab(0, 10, 0, 6);
  s.box(0, 10, 0, 6);
  auto mullion_x = [&](double x, double y) {
    s.wall_x(x, y - 0.025, y + 0.025, 0, 2.6);
  };
  auto mullion_y = [&](double y, double x) {
    s.wall_y(y, x - 0.025, x + 0.025, 0, 2.6);
  };
  // West glass face x = 6, y in [0, 4]: mullions every 0.9 m.
  for (double y = 0.0; y <= 4.0 + 1e-9; y += 0.9)
    mullion_x(6, y);
  // North glass face y = 4, x in [6, 10], with a door at x in [7.5, 8.5].
  for (double x : {6.0, 6.75, 7.5, 8.5, 9.25, 10.0})
    mullion_y(4, x);
  auto [cloud, normals] = s.build();

  const auto r = segment_band_rooms(cloud, normals, {s.storey()});
  CHECK(r.rooms == 2);
  const uint32_t meeting = floor_room(s, *r.labels, 6.5, 9.5, 0.5, 3.5);
  const uint32_t office = floor_room(s, *r.labels, 0.5, 5.5, 0.5, 5.5);
  CHECK(meeting > 0);
  CHECK(office > 0);
  CHECK(meeting != office);
  // The cut follows the glass: the office's floor points do not reach more
  // than one mullion spacing into the meeting room.
  int leaked = 0;
  int inside = 0;
  for (size_t i = 0; i < s.pts.size(); ++i) {
    const auto &p = s.pts[i];
    if (std::abs(p.z()) > 1e-6 || p.x() < 6.2 || p.y() > 3.8)
      continue;
    ++inside;
    leaked += (*r.labels)[i].label == office ? 1 : 0;
  }
  CHECK(leaked < inside / 10);
}

TEST_CASE("BandRooms_Scan_FollowsTheLevelFrame_AndIgnoresPointOrder",
          "[segmentation][band_rooms]") {
  Scan s;
  s.slab(0, 8, 0, 4);
  s.box(0, 8, 0, 4);
  s.wall_x(4, 0, 1.5, 0, 2.6);
  s.wall_x(4, 2.4, 4, 0, 2.6);
  // Tilted 2 deg and rotated 30 deg about z, far from the origin.
  s.pose = Eigen::Translation3d(512.3, -77.1, 41.0) *
           Eigen::AngleAxisd(0.52, Eigen::Vector3d::UnitZ()) *
           Eigen::AngleAxisd(0.035, Eigen::Vector3d::UnitX());
  auto [cloud, normals] = s.build();
  const auto r = segment_band_rooms(cloud, normals, {s.storey()});
  CHECK(r.rooms == 2);

  // Same scan, points shuffled: every point keeps its room.
  std::vector<size_t> perm(cloud.size());
  for (size_t i = 0; i < perm.size(); ++i)
    perm[i] = i;
  std::mt19937 rng(3);
  std::shuffle(perm.begin(), perm.end(), rng);
  Cloud c2;
  CloudN n2;
  for (size_t i : perm) {
    c2.push_back(cloud[i]);
    n2.push_back(normals[i]);
  }
  const auto r2 = segment_band_rooms(c2, n2, {s.storey()});
  REQUIRE(r2.rooms == r.rooms);
  for (size_t k = 0; k < perm.size(); ++k)
    REQUIRE((*r2.labels)[k].label == (*r.labels)[perm[k]].label);
}

TEST_CASE("BandRooms_Scan_TwoStoreys_NumberRoomsAcross",
          "[segmentation][band_rooms]") {
  Scan s;
  s.slab(0, 6, 0, 4, 0.0);
  s.box(0, 6, 0, 4, 0.0);
  s.slab(0, 6, 0, 4, 3.0);
  s.box(0, 6, 0, 4, 3.0);
  s.wall_x(3, 0, 1.5, 3.0, 5.6); // storey 1 has two rooms
  s.wall_x(3, 2.4, 4, 3.0, 5.6);
  auto [cloud, normals] = s.build();
  const auto r = segment_band_rooms(
      cloud, normals, {s.storey(0, 0.0, 2.6), s.storey(1, 3.0, 5.6)});
  CHECK(r.rooms == 3);
  REQUIRE(r.grids.size() == 2);
  CHECK(r.grids[0].rooms == 1);
  CHECK(r.grids[1].rooms == 2);
  const uint32_t down = floor_room(s, *r.labels, 0.5, 5.5, 0.5, 3.5);
  CHECK(down == 1);
  // Upper floor points (z = 3) are not counted by floor_room; check one.
  const uint32_t up_w = (*r.labels)[s.near(1.5, 2.0, 3.0)].label;
  const uint32_t up_e = (*r.labels)[s.near(4.5, 2.0, 3.0)].label;
  CHECK(up_w >= 2);
  CHECK(up_e >= 2);
  CHECK(up_w != up_e);
}

TEST_CASE("BandRooms_Scan_PointsAboveTheTopCeiling_TakeTheRoomBelow",
          "[segmentation][band_rooms]") {
  // An attic over a storey whose detected ceiling is the collar beam: its
  // points must not stay unlabeled, or the mesh stage drops the volume.
  Scan s;
  s.slab(0, 8, 0, 4);
  s.box(0, 8, 0, 4);
  s.wall_x(4, 0, 1.5, 0, 2.6);
  s.wall_x(4, 2.4, 4, 0, 2.6);
  s.horizontal(1, 3, 1, 3, 3.8, true);    // attic ceiling, room west
  s.horizontal(5, 7, 1, 3, 3.8, true);    // attic ceiling, room east
  s.horizontal(9.5, 10, 1, 3, 3.8, true); // outside the building
  auto [cloud, normals] = s.build();
  const auto r = segment_band_rooms(cloud, normals, {s.storey()});
  REQUIRE(r.rooms == 2);
  const uint32_t west = (*r.labels)[s.near(2, 2, 0)].label;
  const uint32_t east = (*r.labels)[s.near(6, 2, 0)].label;
  CHECK((*r.labels)[s.near(2, 2, 3.8)].label == west);
  CHECK((*r.labels)[s.near(6, 2, 3.8)].label == east);
  CHECK((*r.labels)[s.near(9.75, 2, 3.8)].label == 0);
  // The lower storey of two does not take the upper storey's points.
  Scan t;
  t.slab(0, 6, 0, 4, 0.0);
  t.box(0, 6, 0, 4, 0.0);
  t.slab(0, 6, 0, 4, 3.0);
  t.box(0, 6, 0, 4, 3.0);
  auto [c2, n2] = t.build();
  const auto r2 = segment_band_rooms(
      c2, n2, {t.storey(0, 0.0, 2.6), t.storey(1, 3.0, 5.6)});
  CHECK((*r2.labels)[t.near(3, 2, 3.0)].label == 2);
}

TEST_CASE("BandRooms_Scan_NoStoreys_Throws", "[segmentation][band_rooms]") {
  Scan s;
  s.slab(0, 4, 0, 4);
  auto [cloud, normals] = s.build();
  CHECK_THROWS_AS(segment_band_rooms(cloud, normals, {}),
                  std::invalid_argument);
  CloudN short_normals;
  CHECK_THROWS_AS(segment_band_rooms(cloud, short_normals, {s.storey()}),
                  std::invalid_argument);
}

// ===========================================================================
// Wall graph and doorway cuts from the walls / openings tables
// (W4.1 reusex-z1v.6.1, W5.4 reusex-z1v.7.4)
// ===========================================================================

namespace {

core::WallRecord wall_rec(const std::string &guid, double x0, double y0,
                          double x1, double y1, double thickness = 0.1,
                          int storey = 0) {
  core::WallRecord w;
  w.guid = guid;
  w.storey = storey;
  w.x0 = x0;
  w.y0 = y0;
  w.x1 = x1;
  w.y1 = y1;
  w.height = 2.6;
  w.thickness = thickness;
  return w;
}

core::OpeningRecord opening_rec(const std::string &wall, const char *kind,
                                double s0, double s1, double sill = 0.0,
                                double head = 2.1) {
  core::OpeningRecord o;
  o.guid = wall + "/" + kind + "/" + std::to_string(s0);
  o.wall = wall;
  o.kind = kind;
  o.s_start = s0;
  o.s_end = s1;
  o.z_sill = sill;
  o.z_head = head;
  return o;
}

/// Two 4 x 4 m rooms side by side, the shared wall at x = 4 with a
/// 1.6 m wide doorway (y 1.2 .. 2.8): wider than 2 x core_radius, so the
/// constriction watershed alone sees one room.
Plan wide_door_plan() {
  Plan p(-1, 10, -1, 5);
  p.floor(0, 8, 0, 4);
  p.wall(0, 0, 0, 4);
  p.wall(8, 8, 0, 4);
  p.wall(0, 8, 0, 0);
  p.wall(0, 8, 4, 4);
  p.wall(4, 4, 0, 1.2);
  p.wall(4, 4, 2.8, 4);
  return p;
}

/// The walls table of wide_door_plan(): the outline and the partition,
/// bridged over the doorway as the walls stage leaves it.
BandRoomsPlan wide_door_tables(const char *kind = "door") {
  BandRoomsPlan t;
  t.walls = {wall_rec("s", 0, 0, 8, 0), wall_rec("n", 0, 4, 8, 4),
             wall_rec("w", 0, 0, 0, 4), wall_rec("e", 8, 0, 8, 4),
             wall_rec("p", 4, 0, 4, 4)};
  t.openings = {opening_rec("p", kind, 1.2, 2.8)};
  return t;
}

} // namespace

TEST_CASE("BandRooms_Grid_WideDoorway_WithoutTables_IsOneRoom",
          "[segmentation][band_rooms][doors]") {
  // The control: what the constriction cut alone does with a 1.6 m door.
  Plan p = wide_door_plan();
  CHECK(partition_room_grid(p.g, BandRoomsOptions{}) == 1);
}

TEST_CASE("BandRooms_Grid_WideDoorway_FromTheOpeningsTable_CutsTheRooms",
          "[segmentation][band_rooms][doors]") {
  // W5.4 acceptance: the door of the openings table splits the two rooms,
  // with the door cut alone (no wall graph) and with the whole graph.
  for (const bool graph : {false, true}) {
    INFO("use_wall_graph = " << graph);
    Plan p = wide_door_plan();
    BandRoomsOptions o;
    o.use_wall_graph = graph;
    const size_t cells = rasterise_wall_graph(p.g, wide_door_tables(), o);
    CHECK(cells > 0);
    CHECK(p.g.door_cuts == 1);
    CHECK(p.g.graph_walls == (graph ? 5 : 0));
    CHECK(partition_room_grid(p.g, o) == 2);
    const int32_t a = p.at(2, 2);
    const int32_t b = p.at(6, 2);
    CHECK(a > 0);
    CHECK(b > 0);
    CHECK(a != b);
    // The cut lies in the door, and each room keeps nearly all its area.
    CHECK(room_area(p.g, a) > 13.0);
    CHECK(room_area(p.g, b) > 13.0);
    for (int r = p.row(0.2); r < p.row(3.8); ++r)
      for (int c = p.col(4.2); c < p.col(7.8); ++c)
        CHECK(p.g.labels[p.g.index(c, r)] != a);
  }
}

TEST_CASE("BandRooms_Grid_FloorReachingOpening_CutsButWindowDoesNot",
          "[segmentation][band_rooms][doors]") {
  {
    Plan p = wide_door_plan();
    BandRoomsOptions o;
    rasterise_wall_graph(p.g, wide_door_tables("opening"), o);
    CHECK(p.g.door_cuts == 1);
    CHECK(partition_room_grid(p.g, o) == 2);
  }
  {
    // A "window" in the gap is no doorway; without the wall graph nothing
    // closes the gap.
    Plan p = wide_door_plan();
    BandRoomsOptions o;
    o.use_wall_graph = false;
    rasterise_wall_graph(p.g, wide_door_tables("window"), o);
    CHECK(p.g.door_cuts == 0);
    CHECK(partition_room_grid(p.g, o) == 1);
  }
}

TEST_CASE("BandRooms_Grid_DoorCutsOff_LeavesTheDoorwayOpen",
          "[segmentation][band_rooms][doors]") {
  // The constriction fallback: with cut_at_doors off the wall graph skips
  // the doorway, so the 1.6 m door joins the rooms as before.
  Plan p = wide_door_plan();
  BandRoomsOptions o;
  o.cut_at_doors = false;
  rasterise_wall_graph(p.g, wide_door_tables(), o);
  CHECK(p.g.door_cuts == 0);
  CHECK(p.g.graph_walls == 5);
  CHECK(p.g.graph[p.g.index(p.col(4.0), p.row(2.0))] == 0);
  CHECK(p.g.graph[p.g.index(p.col(4.0), p.row(0.6))] == 1);
  CHECK(partition_room_grid(p.g, o) == 1);
}

TEST_CASE("BandRooms_Grid_WallGraph_ClosesAPartitionTheBandMissed",
          "[segmentation][band_rooms][walls]") {
  // W4.1: a glass partition at x = 4 the band saw nothing of, with a 0.9 m
  // door in it. Without the walls table one room; with it the wall graph
  // blocks the glass and the door (cut or constriction) splits the rooms.
  auto plan = [] {
    Plan p(-1, 10, -1, 5);
    p.floor(0, 8, 0, 4);
    p.wall(0, 0, 0, 4);
    p.wall(8, 8, 0, 4);
    p.wall(0, 8, 0, 0);
    p.wall(0, 8, 4, 4);
    return p;
  };
  BandRoomsPlan t;
  t.walls = {wall_rec("p", 4, 0, 4, 4, -1.0)}; // one-sided, unknown thickness
  t.openings = {opening_rec("p", "door", 1.5, 2.4)};
  {
    Plan p = plan();
    CHECK(partition_room_grid(p.g, BandRoomsOptions{}) == 1);
  }
  for (const bool cut : {true, false}) {
    INFO("cut_at_doors = " << cut);
    Plan p = plan();
    BandRoomsOptions o;
    o.cut_at_doors = cut;
    rasterise_wall_graph(p.g, t, o);
    CHECK(partition_room_grid(p.g, o) == 2);
    CHECK(p.at(2, 2) != p.at(6, 2));
  }
}

TEST_CASE("BandRooms_Grid_WcStallsClosedByDoorCuts_KeepTheirOwnRooms",
          "[segmentation][band_rooms][doors]") {
  // The NarrowWcStalls plan above with its walls table: 0.9 m stalls have
  // no core, but closed off by their door cuts each is a free-space
  // component of its own, so each is a room (coreless).
  Plan p(-1, 7, -1, 4.6);
  p.floor(0, 6, 0, 3.6);
  p.wall(0, 0, 0, 3.6);
  p.wall(6, 6, 0, 3.6);
  p.wall(0, 6, 0, 0);
  p.wall(0, 6, 3.6, 3.6);
  for (double x : {0.9, 1.8, 2.7, 4.3})
    p.wall(x, x, 1.6, 3.6);
  p.wall(0, 0.15, 1.6, 1.6);
  p.wall(0.75, 1.05, 1.6, 1.6);
  p.wall(1.65, 1.95, 1.6, 1.6);
  p.wall(2.55, 2.85, 1.6, 1.6);
  p.wall(3.45, 4.3, 1.6, 1.6);

  BandRoomsPlan t;
  t.walls.push_back(wall_rec("front", 0, 1.6, 4.3, 1.6));
  for (double x : {0.9, 1.8, 2.7, 4.3})
    t.walls.push_back(
        wall_rec("part" + std::to_string(int(x * 10)), x, 1.6, x, 3.6));
  for (double s : {0.15, 1.05, 1.95, 2.85})
    t.openings.push_back(opening_rec("front", "door", s, s + 0.6));

  BandRoomsOptions o;
  rasterise_wall_graph(p.g, t, o);
  CHECK(p.g.door_cuts == 4);
  partition_room_grid(p.g, o);
  const int32_t corridor = p.at(3, 0.8);
  REQUIRE(corridor > 0);
  std::set<int32_t> stalls;
  for (double x : {0.45, 1.35, 2.25, 3.5}) {
    const int32_t s = p.at(x, 2.6);
    CHECK(s > 0);
    CHECK(s != corridor);
    stalls.insert(s);
  }
  CHECK(stalls.size() == 4);
  CHECK(p.g.coreless == 3);
  CHECK(p.g.rooms == 5);

  // coreless_min_clearance = 0 keeps the old behaviour: no room.
  BandRoomsOptions off = o;
  off.coreless_min_clearance = 0.0;
  partition_room_grid(p.g, off);
  CHECK(p.g.coreless == 0);
  CHECK(p.at(0.45, 2.6) == 0);
}

TEST_CASE("BandRooms_Grid_WallGraph_SkipsOtherStoreys_AndIsOrderFree",
          "[segmentation][band_rooms][walls]") {
  Plan p = wide_door_plan();
  BandRoomsPlan t = wide_door_tables();
  t.walls.push_back(wall_rec("up", 2, 0, 2, 4, 0.1, /*storey=*/1));
  BandRoomsOptions o;
  RoomGrid a = p.g;
  rasterise_wall_graph(a, t, o);
  CHECK(a.graph_walls == 5);
  CHECK(a.graph[a.index(p.col(2.0), p.row(2.0))] == 0);

  BandRoomsPlan r = t;
  std::reverse(r.walls.begin(), r.walls.end());
  std::reverse(r.openings.begin(), r.openings.end());
  RoomGrid b = p.g;
  rasterise_wall_graph(b, r, o);
  CHECK(a.graph == b.graph);
  partition_room_grid(a, o, 3);
  partition_room_grid(b, o, 3);
  CHECK(a.labels == b.labels);
}

TEST_CASE("BandRooms_Scan_WideDoorFromTheOpeningsTable_TwoRooms",
          "[segmentation][band_rooms][doors]") {
  // The full path: a 1.6 m doorway, in a tilted, rotated frame. Its lintel
  // sits above the wall band (a lintel in the band closes the doorway by
  // itself, like a wall).
  Scan s;
  s.slab(0, 8, 0, 4);
  s.box(0, 8, 0, 4);
  s.wall_x(4, 0, 1.2, 0, 2.6);
  s.wall_x(4, 2.8, 4, 0, 2.6);
  s.wall_x(4, 1.2, 2.8, 2.45, 2.6);
  s.pose = Eigen::Translation3d(12.3, -7.1, 4.0) *
           Eigen::AngleAxisd(0.52, Eigen::Vector3d::UnitZ()) *
           Eigen::AngleAxisd(0.035, Eigen::Vector3d::UnitX());
  auto [cloud, normals] = s.build();
  CHECK(segment_band_rooms(cloud, normals, {s.storey()}).rooms == 1);

  const BandRoomsPlan t = wide_door_tables();
  const auto r =
      segment_band_rooms(cloud, normals, {s.storey()}, {}, nullptr, &t);
  CHECK(r.rooms == 2);
  REQUIRE(r.grids.size() == 1);
  CHECK(r.grids[0].door_cuts == 1);
  const uint32_t a = floor_room(s, *r.labels, 0.5, 3.5, 0.5, 3.5);
  const uint32_t b = floor_room(s, *r.labels, 4.5, 7.5, 0.5, 3.5);
  CHECK(a > 0);
  CHECK(b > 0);
  CHECK(a != b);
  CHECK(r.unlabeled_points == 0);
}

TEST_CASE("BandRooms_Grid_FootprintLeakCutOffByTheWallGraph_IsNoRoom",
          "[segmentation][band_rooms][walls]") {
  // A 4 x 4 m room whose footprint leaks 0.8 m out through a window in its
  // east wall (the sensor looked through the glass). The wall graph closes
  // the window; the leak beyond it has no core and opens onto the exterior,
  // so it is no room (it stays with the room, through the point assignment).
  Plan p(-1, 7, -1, 5);
  p.floor(0, 4, 0, 4);
  p.floor(4, 4.8, 0.5, 3.5);
  p.wall(0, 0, 0, 4);
  p.wall(0, 4, 0, 0);
  p.wall(0, 4, 4, 4);
  p.wall(4, 4, 0, 0.5);
  p.wall(4, 4, 3.5, 4);
  BandRoomsPlan t;
  t.walls = {wall_rec("e", 4, 0, 4, 4)};
  t.openings = {opening_rec("e", "window", 0.5, 3.5, 0.9, 2.1)};
  BandRoomsOptions o;
  rasterise_wall_graph(p.g, t, o);
  CHECK(partition_room_grid(p.g, o) == 1);
  CHECK(p.g.coreless == 0);
  CHECK(p.at(4.4, 2) == 0);
}

TEST_CASE("BandRooms_Grid_BarelySeenWall_IsLeftOutWithItsDoors",
          "[segmentation][band_rooms][walls]") {
  // A wall the scan shows on 10 % of its face (a cabinet front, a drift
  // artefact) with an "opening" over most of it must not cut a room.
  Plan p(-1, 6, -1, 5);
  p.floor(0, 5, 0, 4);
  p.wall(0, 0, 0, 4);
  p.wall(5, 5, 0, 4);
  p.wall(0, 5, 0, 0);
  p.wall(0, 5, 4, 4);
  BandRoomsPlan t;
  t.walls = {wall_rec("ghost", 2.5, 0, 2.5, 4)};
  t.walls[0].seen_share = 0.1;
  t.openings = {opening_rec("ghost", "opening", 0.3, 3.7, 0.0, 2.6)};
  BandRoomsOptions o;
  rasterise_wall_graph(p.g, t, o);
  CHECK(p.g.graph_walls == 0);
  CHECK(p.g.door_cuts == 0);
  CHECK(partition_room_grid(p.g, o) == 1);
  // Seen enough, it is a wall: two rooms.
  t.walls[0].seen_share = 0.5;
  rasterise_wall_graph(p.g, t, o);
  CHECK(p.g.graph_walls == 1);
  CHECK(partition_room_grid(p.g, o) == 2);
}
