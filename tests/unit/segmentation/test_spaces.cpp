// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Space polygons from the band-rooms grid and the wall graph (W4.2,
// reusex-z1v.6.2): synthetic grids with hand-set room labels and walls.

#include <reusex/segmentation/spaces.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <map>
#include <random>
#include <string>
#include <utility>
#include <vector>

using namespace reusex;
using namespace reusex::geometry;
using Catch::Approx;

namespace {

/// A 5 cm grid over level x, y in [x0, x1) x [y0, y1), storey 0.
struct Grid {
  RoomGrid g;
  double cell = 0.05;

  Grid(double x0, double x1, double y0, double y1) {
    g.cell_size = cell;
    g.col0 = static_cast<int>(std::floor(x0 / cell));
    g.row0 = static_cast<int>(std::floor(y0 / cell));
    g.resize(static_cast<int>(std::ceil((x1 - x0) / cell)),
             static_cast<int>(std::ceil((y1 - y0) / cell)));
  }

  /// Label the cells whose centre lies in [x0, x1) x [y0, y1).
  void fill(double x0, double x1, double y0, double y1, int32_t label) {
    for (int r = 0; r < g.height; ++r)
      for (int c = 0; c < g.width; ++c) {
        const double x = (g.col0 + c + 0.5) * cell;
        const double y = (g.row0 + r + 0.5) * cell;
        if (x >= x0 && x < x1 && y >= y0 && y < y1)
          g.labels[g.index(c, r)] = label;
      }
  }
};

core::WallRecord wall(const std::string &guid, double x0, double y0, double x1,
                      double y1, double thickness = 0.2) {
  core::WallRecord w;
  w.guid = guid;
  w.storey = 0;
  w.x0 = x0;
  w.y0 = y0;
  w.x1 = x1;
  w.y1 = y1;
  w.height = 2.6;
  w.thickness = thickness;
  w.seen_share = 0.9;
  return w;
}

core::StoreyRecord storey0() {
  core::StoreyRecord s;
  s.idx = 0;
  s.z_floor = 0.0;
  s.z_ceiling = 2.5;
  return s;
}

double ring_area(const std::vector<core::PlanPoint> &ring) {
  double a = 0.0;
  for (size_t i = 0; i < ring.size(); ++i) {
    const auto &p = ring[i];
    const auto &q = ring[(i + 1) % ring.size()];
    a += p[0] * q[1] - q[0] * p[1];
  }
  return 0.5 * a;
}

/// The L: (0,0) (6,0) (6,3) (3,3) (3,6) (0,6), 27 m^2. Walls 0.2 m thick
/// with their inner faces on the L, listed counter-clockwise so the room is
/// left of every axis (side "a").
std::vector<core::WallRecord> l_walls() {
  return {
      wall("south", -0.1, -0.1, 6.1, -0.1), wall("east", 6.1, -0.1, 6.1, 3.1),
      wall("notch_h", 6.1, 3.1, 2.9, 3.1),  wall("notch_v", 3.1, 2.9, 3.1, 6.1),
      wall("north", 3.1, 6.1, -0.1, 6.1),   wall("west", -0.1, 6.1, -0.1, -0.1),
  };
}

/// The L's room cells: one cell short of every face (walls are dilated in
/// the band grid), with two furniture pockets left unlabelled.
Grid l_grid() {
  Grid G(-1.0, 7.0, -1.0, 7.0);
  G.fill(0.05, 5.95, 0.05, 2.95, 1);
  G.fill(0.05, 2.95, 2.95, 5.95, 1);
  G.fill(1.0, 1.6, 1.0, 1.5, 0); // desk
  G.fill(4.0, 4.4, 1.5, 2.5, 0); // cabinet
  return G;
}

const core::SpaceBoundaryRecord *find_boundary(const SpacesResult &r,
                                               const std::string &space,
                                               const std::string &wall,
                                               const std::string &side) {
  for (const auto &b : r.boundaries)
    if (b.space == space && b.wall == wall && b.side == side)
      return &b;
  return nullptr;
}

} // namespace

TEST_CASE("Spaces_LShapedRoom_IsOneSixVertexPolygonOnTheWallFaces",
          "[segmentation][spaces]") {
  auto G = l_grid();
  const auto r = polygonise_spaces({G.g}, {storey0()}, l_walls());

  REQUIRE(r.spaces.size() == 1);
  const auto &s = r.spaces[0];
  CHECK(s.storey == 0);
  CHECK(s.room_label == 1);
  for (const auto &p : s.polygon.outer)
    UNSCOPED_INFO(p[0] << ", " << p[1]);
  CHECK(s.polygon.outer.size() == 6);      // an L, not its bounding rectangle
  CHECK(s.polygon.holes.empty());          // furniture pockets are not holes
  CHECK(ring_area(s.polygon.outer) > 0.0); // counter-clockwise
  CHECK(s.area == Approx(27.0).epsilon(0.01));
  CHECK(std::abs(ring_area(s.polygon.outer)) == Approx(27.0).epsilon(0.01));
  CHECK(s.perimeter == Approx(24.0).epsilon(0.01));
  CHECK(s.volume == Approx(27.0 * 2.5).epsilon(0.01));
  CHECK(s.compactness ==
        Approx(4.0 * M_PI * 27.0 / (24.0 * 24.0)).epsilon(0.01));
  CHECK(s.kind == "room");

  // Every vertex is a corner of the L.
  const std::vector<core::PlanPoint> corners = {{0, 0}, {6, 0}, {6, 3},
                                                {3, 3}, {3, 6}, {0, 6}};
  for (const auto &p : s.polygon.outer) {
    const bool on_corner =
        std::any_of(corners.begin(), corners.end(), [&](const auto &c) {
          return std::abs(c[0] - p[0]) < 1e-6 && std::abs(c[1] - p[1]) < 1e-6;
        });
    CHECK(on_corner);
  }

  // One boundary per wall, on the room's side, over the inner face.
  REQUIRE(r.boundaries.size() == 6);
  for (const auto &w : l_walls()) {
    INFO(w.guid);
    const auto *b = find_boundary(r, s.guid, w.guid, "a");
    REQUIRE(b != nullptr);
    CHECK(b->s_start == Approx(0.1).margin(1e-6));
  }
  CHECK(find_boundary(r, s.guid, "south", "a")->s_end ==
        Approx(6.1).margin(1e-6));
  CHECK(find_boundary(r, s.guid, "east", "a")->s_end ==
        Approx(3.1).margin(1e-6));
  CHECK(find_boundary(r, s.guid, "notch_h", "a")->s_end ==
        Approx(3.1).margin(1e-6));

  REQUIRE(r.stats.size() == 1);
  CHECK(r.stats[0].spaces == 1);
  CHECK(r.stats[0].arrangement_share == Approx(1.0));
}

TEST_CASE("Spaces_ReversedWall_BoundsTheRoomOnSideB",
          "[segmentation][spaces]") {
  auto G = l_grid();
  auto walls = l_walls();
  std::swap(walls[0].x0, walls[0].x1); // south now runs east -> west
  const auto r = polygonise_spaces({G.g}, {storey0()}, walls);
  REQUIRE(r.spaces.size() == 1);
  const auto *b = find_boundary(r, r.spaces[0].guid, "south", "b");
  REQUIRE(b != nullptr);
  CHECK(b->s_start == Approx(0.1).margin(1e-6));
  CHECK(b->s_end == Approx(6.1).margin(1e-6));
  CHECK(find_boundary(r, r.spaces[0].guid, "south", "a") == nullptr);
}

TEST_CASE("Spaces_WithoutWalls_AreTheRasterOutline", "[segmentation][spaces]") {
  auto G = l_grid();
  const auto r = polygonise_spaces({G.g}, {storey0()}, {});
  REQUIRE(r.spaces.size() == 1);
  const auto &s = r.spaces[0];
  // The room's cells: 5.9 x 2.9 + 2.9 x 3.0, its furniture pockets
  // filled (they belong to the room, not holes in it).
  const double cells = 5.9 * 2.9 + 2.9 * 3.0;
  CHECK(std::abs(ring_area(s.polygon.outer)) == Approx(cells).epsilon(1e-6));
  CHECK(s.polygon.holes.empty());
  CHECK(s.area == Approx(cells).epsilon(1e-6));
  CHECK(s.polygon.outer.size() == 6);
  CHECK(r.boundaries.empty());
  CHECK(r.stats[0].arrangement_share == Approx(0.0));
}

TEST_CASE("Spaces_TwoRoomsAndAPartition_ShareTheWallFromBothSides",
          "[segmentation][spaces]") {
  // Two 4 x 3 rooms side by side, a 0.1 m partition at x = 4.
  Grid G(-1.0, 9.0, -1.0, 4.0);
  G.fill(0.05, 3.9, 0.05, 2.95, 1);
  G.fill(4.1, 7.95, 0.05, 2.95, 2);
  const std::vector<core::WallRecord> walls = {
      wall("s", -0.1, -0.1, 8.1, -0.1),     wall("e", 8.1, -0.1, 8.1, 3.1),
      wall("n", 8.1, 3.1, -0.1, 3.1),       wall("w", -0.1, 3.1, -0.1, -0.1),
      wall("mid", 4.0, 0.0, 4.0, 3.0, 0.1), // ends on the faces: no gap
  };
  const auto r = polygonise_spaces({G.g}, {storey0()}, walls);
  REQUIRE(r.spaces.size() == 2);
  const auto &a = r.spaces[0];
  const auto &b = r.spaces[1];
  CHECK(a.room_label == 1);
  CHECK(b.room_label == 2);
  CHECK(a.area == Approx(3.95 * 3.0).epsilon(0.001));
  CHECK(b.area == Approx(3.95 * 3.0).epsilon(0.001));
  CHECK(a.polygon.outer.size() == 4);
  CHECK(b.polygon.outer.size() == 4);
  // "mid" runs +y: room 1 (west) is left of it, room 2 right.
  const auto *ba = find_boundary(r, a.guid, "mid", "a");
  const auto *bb = find_boundary(r, b.guid, "mid", "b");
  REQUIRE(ba != nullptr);
  REQUIRE(bb != nullptr);
  CHECK(ba->s_start == Approx(0.0).margin(1e-6));
  CHECK(ba->s_end == Approx(3.0).margin(1e-6));
  CHECK(find_boundary(r, a.guid, "mid", "b") == nullptr);
  CHECK(find_boundary(r, a.guid, "e", "a") == nullptr);
  CHECK(find_boundary(r, b.guid, "w", "a") == nullptr);
}

TEST_CASE("Spaces_MissingPartition_FallsBackToTheRasterBetweenTheRooms",
          "[segmentation][spaces]") {
  // As above, but the walls table has no partition: the one face holds two
  // rooms, so the raster decides between them and both stay.
  Grid G(-1.0, 9.0, -1.0, 4.0);
  G.fill(0.05, 3.9, 0.05, 2.95, 1);
  G.fill(4.1, 7.95, 0.05, 2.95, 2);
  const std::vector<core::WallRecord> walls = {
      wall("s", -0.1, -0.1, 8.1, -0.1),
      wall("e", 8.1, -0.1, 8.1, 3.1),
      wall("n", 8.1, 3.1, -0.1, 3.1),
      wall("w", -0.1, 3.1, -0.1, -0.1),
  };
  const auto r = polygonise_spaces({G.g}, {storey0()}, walls);
  REQUIRE(r.spaces.size() == 2);
  CHECK(r.stats[0].mixed_faces == 1);
  CHECK(r.spaces[0].area == Approx(3.85 * 2.9).epsilon(1e-6));
  CHECK(r.spaces[1].area == Approx(3.85 * 2.9).epsilon(1e-6));
}

TEST_CASE("Spaces_TeeWithAGap_ClosesByExtension", "[segmentation][spaces]") {
  // The partition stops 0.3 m short of the north wall (a detection gap):
  // extended, it still closes the two rooms.
  Grid G(-1.0, 9.0, -1.0, 4.0);
  G.fill(0.05, 3.9, 0.05, 2.95, 1);
  G.fill(4.1, 7.95, 0.05, 2.95, 2);
  const std::vector<core::WallRecord> walls = {
      wall("s", -0.1, -0.1, 8.1, -0.1),     wall("e", 8.1, -0.1, 8.1, 3.1),
      wall("n", 8.1, 3.1, -0.1, 3.1),       wall("w", -0.1, 3.1, -0.1, -0.1),
      wall("mid", 4.0, 0.0, 4.0, 2.7, 0.1),
  };
  const auto r = polygonise_spaces({G.g}, {storey0()}, walls);
  REQUIRE(r.spaces.size() == 2);
  CHECK(r.spaces[0].polygon.outer.size() == 4);
  CHECK(r.spaces[0].area == Approx(3.95 * 3.0).epsilon(0.001));
  CHECK(r.stats[0].mixed_faces == 0);
}

TEST_CASE("Spaces_Corridor_IsKindCorridor", "[segmentation][spaces]") {
  Grid G(-1.0, 13.0, -1.0, 3.0);
  G.fill(0.05, 11.95, 0.05, 1.45, 1);
  const std::vector<core::WallRecord> walls = {
      wall("s", -0.1, -0.1, 12.1, -0.1),
      wall("e", 12.1, -0.1, 12.1, 1.6),
      wall("n", 12.1, 1.6, -0.1, 1.6),
      wall("w", -0.1, 1.6, -0.1, -0.1),
  };
  const auto r = polygonise_spaces({G.g}, {storey0()}, walls);
  REQUIRE(r.spaces.size() == 1);
  CHECK(r.spaces[0].kind == "corridor");
  CHECK(r.spaces[0].area == Approx(12.0 * 1.5).epsilon(0.001));
}

TEST_CASE("Spaces_OpenPlanWithFurniture_IsKindOpenPlan",
          "[segmentation][spaces]") {
  Grid G(-1.0, 11.0, -1.0, 9.0);
  G.fill(0.05, 9.95, 0.05, 7.95, 1);
  for (int k = 0; k < 4; ++k) // four desk islands
    G.fill(1.0 + 2.0 * k, 1.8 + 2.0 * k, 3.0, 4.0, 0);
  const std::vector<core::WallRecord> walls = {
      wall("s", -0.1, -0.1, 10.1, -0.1),
      wall("e", 10.1, -0.1, 10.1, 8.1),
      wall("n", 10.1, 8.1, -0.1, 8.1),
      wall("w", -0.1, 8.1, -0.1, -0.1),
  };
  const auto r = polygonise_spaces({G.g}, {storey0()}, walls);
  REQUIRE(r.spaces.size() == 1);
  CHECK(r.spaces[0].kind == "open_plan");
  CHECK(r.spaces[0].polygon.holes.empty());
  CHECK(r.spaces[0].area == Approx(80.0).epsilon(0.001));
}

TEST_CASE("Spaces_WallOrder_DoesNotChangeTheResult", "[segmentation][spaces]") {
  auto G = l_grid();
  auto walls = l_walls();
  walls.push_back(wall("stub", 1.5, 4.0, 1.5, 5.0, 0.1)); // a free-standing
  const auto ref = polygonise_spaces({G.g}, {storey0()}, walls);
  std::mt19937 rng(7);
  for (int k = 0; k < 3; ++k) {
    std::shuffle(walls.begin(), walls.end(), rng);
    const auto r = polygonise_spaces({G.g}, {storey0()}, walls);
    REQUIRE(r.spaces.size() == ref.spaces.size());
    for (size_t i = 0; i < r.spaces.size(); ++i) {
      CHECK(r.spaces[i].polygon.outer == ref.spaces[i].polygon.outer);
      CHECK(r.spaces[i].polygon.holes == ref.spaces[i].polygon.holes);
    }
    REQUIRE(r.boundaries.size() == ref.boundaries.size());
    for (size_t i = 0; i < r.boundaries.size(); ++i) {
      CHECK(r.boundaries[i].wall == ref.boundaries[i].wall);
      CHECK(r.boundaries[i].s_start == ref.boundaries[i].s_start);
    }
  }
}

TEST_CASE("Spaces_OneSidedWall_UsesItsAxisAsTheFace",
          "[segmentation][spaces]") {
  // External walls seen from inside only: thickness -1, axis on the face.
  Grid G(-1.0, 6.0, -1.0, 5.0);
  G.fill(0.05, 4.95, 0.05, 3.95, 1);
  const std::vector<core::WallRecord> walls = {
      wall("s", 0.0, 0.0, 5.0, 0.0, -1.0),
      wall("e", 5.0, 0.0, 5.0, 4.0, -1.0),
      wall("n", 5.0, 4.0, 0.0, 4.0, -1.0),
      wall("w", 0.0, 4.0, 0.0, 0.0, -1.0),
  };
  const auto r = polygonise_spaces({G.g}, {storey0()}, walls);
  REQUIRE(r.spaces.size() == 1);
  CHECK(r.spaces[0].area == Approx(20.0).epsilon(1e-6));
  CHECK(r.spaces[0].polygon.outer.size() == 4);
  REQUIRE(r.boundaries.size() == 4);
  for (const auto &b : r.boundaries) {
    CHECK(b.side == "a");
    CHECK(b.s_start == Approx(0.0).margin(1e-6));
  }
}

TEST_CASE("Spaces_RoomInsideAnotherRoomsFace_KeepsItsCells",
          "[segmentation][spaces]") {
  // A 10 x 8 room whose south-west 3 x 3 corner the band grid cut off as a
  // room of its own (clutter, no wall in the walls table): one face, two
  // rooms. The corner room is a guest of the big one's face.
  Grid G(-1.0, 11.0, -1.0, 9.0);
  G.fill(0.05, 9.95, 0.05, 7.95, 1);
  G.fill(0.0, 3.05, 0.0, 3.05, 0); // the clutter line, unlabelled
  G.fill(0.05, 2.95, 0.05, 2.95, 2);
  const std::vector<core::WallRecord> walls = {
      wall("s", -0.1, -0.1, 10.1, -0.1),
      wall("e", 10.1, -0.1, 10.1, 8.1),
      wall("n", 10.1, 8.1, -0.1, 8.1),
      wall("w", -0.1, 8.1, -0.1, -0.1),
  };
  const auto r = polygonise_spaces({G.g}, {storey0()}, walls);
  REQUIRE(r.spaces.size() == 2);
  const auto &big = r.spaces[0];
  const auto &corner = r.spaces[1];
  // The guest grows over the unlabelled line to the walls: 3.05 x 3.05.
  CHECK(corner.area == Approx(3.05 * 3.05).epsilon(0.01));
  CHECK(corner.polygon.outer.size() == 4);
  CHECK(big.area == Approx(80.0 - 3.05 * 3.05).epsilon(0.01));
  CHECK(big.polygon.outer.size() == 6);
  CHECK(big.polygon.holes.empty());
  CHECK(find_boundary(r, corner.guid, "s", "a") != nullptr);
  CHECK(find_boundary(r, corner.guid, "w", "a") != nullptr);
  CHECK(find_boundary(r, corner.guid, "n", "a") == nullptr);
}

TEST_CASE("Spaces_MissingExteriorWall_ClosedByTheInterior",
          "[segmentation][spaces]") {
  // Walls on three sides; the east one was not detected. The storey's
  // interior outline closes the face, so the room is still a face with
  // straight edges, not its raster.
  Grid G(-1.0, 8.0, -1.0, 5.0);
  G.fill(0.05, 5.95, 0.05, 3.95, 1);
  for (int r = 0; r < G.g.height; ++r)
    for (int c = 0; c < G.g.width; ++c) {
      const double x = (G.g.col0 + c + 0.5) * G.cell;
      const double y = (G.g.row0 + r + 0.5) * G.cell;
      G.g.interior[G.g.index(c, r)] =
          x > -0.2 && x < 6.2 && y > -0.2 && y < 4.2 ? 1 : 0;
    }
  const std::vector<core::WallRecord> walls = {
      wall("s", -0.1, -0.1, 6.1, -0.1),
      wall("n", 6.1, 4.1, -0.1, 4.1),
      wall("w", -0.1, 4.1, -0.1, -0.1),
  };
  const auto r = polygonise_spaces({G.g}, {storey0()}, walls);
  REQUIRE(r.spaces.size() == 1);
  CHECK(r.spaces[0].polygon.outer.size() == 4);
  CHECK(r.spaces[0].area == Approx(6.2 * 4.0).epsilon(0.01));
  CHECK(r.stats[0].arrangement_share == Approx(1.0).margin(1e-6));

  SpacesOptions open;
  open.close_at_interior = false;
  const auto raster = polygonise_spaces({G.g}, {storey0()}, walls, open);
  REQUIRE(raster.spaces.size() == 1);
  CHECK(raster.stats[0].arrangement_share < 0.5);
}

namespace {

/// Interior mask of @p G: the cells whose centre @p in holds.
template <typename In> void set_interior(Grid &G, const In &in) {
  for (int r = 0; r < G.g.height; ++r)
    for (int c = 0; c < G.g.width; ++c) {
      const double x = (G.g.col0 + c + 0.5) * G.cell;
      const double y = (G.g.row0 + r + 0.5) * G.cell;
      G.g.interior[G.g.index(c, r)] = in(x, y) ? 1 : 0;
    }
}

} // namespace

TEST_CASE("Spaces_OffsetExteriorRuns_AreBridgedSoTheLeakStaysOut",
          "[segmentation][spaces]") {
  // reusex-ajf: the east wall comes as two parallel runs 0.4 m apart (a
  // doubled sheet / drift) that meet end to end at y = 4. Their extensions
  // run side by side and never cross, so the strip between them joins the
  // room to the footprint leak beyond the window in the northern run, and
  // only the interior outline closes the face. A bridge across the two
  // runs' ends keeps the leak out of the room.
  Grid G(-1.0, 10.0, -1.0, 11.0);
  G.fill(0.05, 6.35, 0.05, 4.0, 1);
  G.fill(0.05, 5.95, 4.0, 9.95, 1);
  set_interior(G, [](double x, double y) {
    return (x > -0.1 && x < 6.45 && y > -0.1 && y < 4.05) ||
           (x > -0.1 && x < 6.05 && y > 4.0 && y < 10.1) ||
           (x > 6.0 && x < 8.0 && y > 5.0 && y < 8.0); // the leak
  });
  const std::vector<core::WallRecord> walls = {
      wall("s", 0.0, 0.0, 6.4, 0.0, -1.0),
      wall("e1", 6.4, 0.0, 6.4, 4.0, -1.0),
      wall("e2", 6.0, 4.05, 6.0, 10.0, -1.0),
      wall("n", 6.0, 10.0, 0.0, 10.0, -1.0),
      wall("w", 0.0, 10.0, 0.0, 0.0, -1.0),
  };
  const auto r = polygonise_spaces({G.g}, {storey0()}, walls);
  REQUIRE(r.spaces.size() == 1);
  // 6.4 x 4.025 + 6.0 x 5.975 with the bridge half way between the ends.
  CHECK(r.spaces[0].area == Approx(61.61).epsilon(0.005));
  CHECK(r.stats[0].arrangement_share == Approx(1.0).margin(1e-6));

  // The bridges depend on the geometry, not on the GUIDs (new on every
  // fresh walls run): renamed so the walls sort the other way round.
  auto renamed = walls;
  for (size_t i = 0; i < renamed.size(); ++i)
    renamed[i].guid = std::string(1, static_cast<char>('z' - i));
  const auto again = polygonise_spaces({G.g}, {storey0()}, renamed);
  REQUIRE(again.spaces.size() == 1);
  CHECK(again.stats[0].bridges == r.stats[0].bridges);
  CHECK(again.spaces[0].polygon.outer == r.spaces[0].polygon.outer);

  SpacesOptions unbridged;
  unbridged.bridge_max_offset_m = 0.0;
  const auto leak = polygonise_spaces({G.g}, {storey0()}, walls, unbridged);
  REQUIRE(leak.spaces.size() == 1);
  CHECK(leak.spaces[0].area > 65.0); // the leak is in
}

TEST_CASE("Spaces_LeaksBeyondAnExternalWall_AreExterior",
          "[segmentation][spaces]") {
  // reusex-e0k: the footprint leaks through two windows in the external
  // east wall. One leak was absorbed into the room (its cells carry the
  // room's label), the other is a room of its own. Both lie beyond the
  // external wall's outer face, so neither is part of a space.
  Grid G(-1.0, 10.0, -1.0, 6.0);
  G.fill(0.05, 5.95, 0.05, 3.95, 1);
  G.fill(6.05, 7.5, 0.5, 1.5, 1); // absorbed leak
  G.fill(6.05, 8.0, 2.2, 3.6, 2); // a leak of its own
  set_interior(G, [](double x, double y) {
    return (x > -0.1 && x < 6.1 && y > -0.1 && y < 4.1) ||
           (x >= 6.0 && x < 7.55 && y > 0.45 && y < 1.55) ||
           (x >= 6.0 && x < 8.05 && y > 2.15 && y < 3.65);
  });
  std::vector<core::WallRecord> walls = {
      wall("s", 0.0, 0.0, 6.0, 0.0, -1.0),
      wall("e", 6.0, 0.0, 6.0, 4.0, -1.0),
      wall("n", 6.0, 4.0, 0.0, 4.0, -1.0),
      wall("w", 0.0, 4.0, 0.0, 0.0, -1.0),
  };
  for (auto &w : walls)
    w.is_external = true;
  const auto r = polygonise_spaces({G.g}, {storey0()}, walls);
  REQUIRE(r.spaces.size() == 1);
  CHECK(r.spaces[0].room_label == 1);
  CHECK(r.spaces[0].area == Approx(24.0).epsilon(0.005));
  CHECK(r.stats[0].exterior_rooms == 1);

  // Without the external flag the leaks stay (the regression this guards).
  for (auto &w : walls)
    w.is_external = false;
  const auto leak = polygonise_spaces({G.g}, {storey0()}, walls);
  CHECK(leak.spaces.size() == 2);
}

TEST_CASE("Spaces_ExternalWallWithRoomsOnBothSides_IsLeftAlone",
          "[segmentation][spaces]") {
  // A wall flagged external by mistake with rooms of the same size either
  // side: which side is outside is not clear, so nothing is cut.
  Grid G(-1.0, 13.0, -1.0, 5.0);
  G.fill(0.05, 5.95, 0.05, 3.95, 1);
  G.fill(6.05, 11.95, 0.05, 3.95, 2);
  std::vector<core::WallRecord> walls = {
      wall("s", 0.0, 0.0, 12.0, 0.0, -1.0),
      wall("mid", 6.0, 0.0, 6.0, 4.0, -1.0),
      wall("n", 12.0, 4.0, 0.0, 4.0, -1.0),
      wall("w", 0.0, 4.0, 0.0, 0.0, -1.0),
      wall("e", 12.0, 0.0, 12.0, 4.0, -1.0),
  };
  walls[1].is_external = true;
  const auto r = polygonise_spaces({G.g}, {storey0()}, walls);
  REQUIRE(r.spaces.size() == 2);
  CHECK(r.stats[0].exterior_rooms == 0);
}
