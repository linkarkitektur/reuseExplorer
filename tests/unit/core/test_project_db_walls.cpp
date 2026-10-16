// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// The walls and wall_joins tables (schema v33, reusex-z1v.5.5):
// core::WallRecord / WallJoinRecord rows with deterministic GUIDs and
// source/locked semantics.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/guid.hpp>
#include <reusex/core/solid_model.hpp>
#include <reusex/io/solid_model_json.hpp>

#include "../../support/temp_path.hpp"

#include <stdexcept>
#include <string>
#include <vector>

using Catch::Matchers::WithinAbs;
using reusex::ProjectDB;
using reusex::core::WallJoinRecord;
using reusex::core::WallRecord;
using reusex::test_support::TempPath;

namespace {

WallRecord wall(const ProjectDB &db, const std::string &key, int storey,
                double x0, double y0, double x1, double y1) {
  WallRecord w;
  w.guid = reusex::core::deterministic_guid(db.project_guid(), key);
  w.storey = storey;
  w.x0 = x0;
  w.y0 = y0;
  w.x1 = x1;
  w.y1 = y1;
  w.z_base = 0.0;
  w.height = 2.6;
  w.thickness = 0.12;
  w.thickness_ci_lo = 0.11;
  w.thickness_ci_hi = 0.13;
  w.lean_deg = 0.4;
  w.is_external = true;
  w.assumed = false;
  w.seen_share = 0.8;
  w.face_a_points = 1200;
  w.face_b_points = 900;
  w.co_visibility = 0.7;
  w.axis_family = 1;
  w.confidence = 0.5;
  w.metadata = R"({"low_seen_share":false})";
  return w;
}

/// An L of three walls: a-b corner, b-c tee.
struct Fixture {
  std::vector<WallRecord> walls;
  std::vector<WallJoinRecord> joins;
};

Fixture three_walls(const ProjectDB &db) {
  Fixture f;
  f.walls = {wall(db, "walls/0/a", 0, 0, 0, 4, 0),
             wall(db, "walls/0/b", 0, 4, 0, 4, 3),
             wall(db, "walls/1/c", 1, 2, 0, 2, 3)};
  f.walls[1].assumed = true;
  f.walls[1].thickness_ci_lo = f.walls[1].thickness_ci_hi = -1.0;
  f.joins = {{f.walls[0].guid, f.walls[1].guid, "corner", "end", "start"},
             {f.walls[2].guid, f.walls[0].guid, "tee", "start", "path"}};
  return f;
}

} // namespace

TEST_CASE("Walls_FreshProject_IsEmpty", "[projectdb][walls]") {
  TempPath tmp("test_walls_empty");
  ProjectDB db(tmp.path);
  CHECK_FALSE(db.has_walls());
  CHECK(db.walls().empty());
  CHECK(db.wall_joins().empty());
  CHECK_FALSE(db.wall_by_guid("nope").has_value());
}

TEST_CASE("Walls_SaveAndLoad_RoundTripsEveryField", "[projectdb][walls]") {
  TempPath tmp("test_walls_roundtrip");
  ProjectDB db(tmp.path);
  const int run = db.log_pipeline_start("walls");
  const auto f = three_walls(db);
  CHECK(db.save_walls(f.walls, f.joins, run) == 3);
  REQUIRE(db.has_walls());

  const auto rows = db.walls();
  REQUIRE(rows.size() == 3);
  // storey ascending, then insertion order
  CHECK(rows[0].guid == f.walls[0].guid);
  CHECK(rows[2].guid == f.walls[2].guid);
  const auto &w = rows[0];
  const auto &in = f.walls[0];
  CHECK(w.id > 0);
  CHECK(w.storey == 0);
  CHECK_THAT(w.x1, WithinAbs(4.0, 1e-12));
  CHECK_THAT(w.height, WithinAbs(2.6, 1e-12));
  CHECK_THAT(w.thickness, WithinAbs(in.thickness, 1e-12));
  CHECK_THAT(w.thickness_ci_lo, WithinAbs(in.thickness_ci_lo, 1e-12));
  CHECK_THAT(w.thickness_ci_hi, WithinAbs(in.thickness_ci_hi, 1e-12));
  CHECK_THAT(w.lean_deg, WithinAbs(0.4, 1e-12));
  CHECK(w.is_external);
  CHECK_FALSE(w.assumed);
  CHECK_THAT(w.seen_share, WithinAbs(0.8, 1e-12));
  CHECK(w.face_a_points == 1200);
  CHECK(w.face_b_points == 900);
  CHECK_THAT(w.co_visibility, WithinAbs(0.7, 1e-12));
  CHECK(w.axis_family == 1);
  CHECK(w.source == "detected");
  CHECK_FALSE(w.locked);
  CHECK_THAT(w.confidence, WithinAbs(0.5, 1e-12));
  CHECK(w.metadata == in.metadata);
  CHECK(rows[1].assumed);

  const auto by_guid = db.wall_by_guid(f.walls[1].guid);
  REQUIRE(by_guid.has_value());
  CHECK(by_guid->assumed);

  const auto joins = db.wall_joins();
  REQUIRE(joins.size() == 2);
  CHECK(joins[0].wall_a == f.walls[0].guid);
  CHECK(joins[0].wall_b == f.walls[1].guid);
  CHECK(joins[0].kind == "corner");
  CHECK(joins[0].at_a == "end");
  CHECK(joins[0].at_b == "start");
  CHECK(joins[1].kind == "tee");
  CHECK(joins[1].at_b == "path");
}

TEST_CASE("Walls_Save_ReplacesUnlockedRowsOnly", "[projectdb][walls]") {
  TempPath tmp("test_walls_locked");
  ProjectDB db(tmp.path);
  auto f = three_walls(db);
  f.walls[0].locked = true;
  f.walls[0].source = "manual";
  f.walls[0].thickness = 0.3;
  db.save_walls(f.walls, f.joins);

  // A re-run: wall a re-detected (same guid, other thickness), b gone, d new.
  auto again = three_walls(db);
  auto d = wall(db, "walls/0/d", 0, 0, 3, 4, 3);
  std::vector<WallRecord> fresh{again.walls[0], d};
  std::vector<WallJoinRecord> joins{
      {again.walls[0].guid, d.guid, "corner", "start", "start"}};
  CHECK(db.save_walls(fresh, joins) == 1); // a is locked: skipped

  const auto rows = db.walls();
  REQUIRE(rows.size() == 2);
  const auto a = db.wall_by_guid(f.walls[0].guid);
  REQUIRE(a.has_value());
  CHECK(a->locked);
  CHECK(a->source == "manual");
  CHECK_THAT(a->thickness, WithinAbs(0.3, 1e-12)); // the locked row wins
  CHECK(db.wall_by_guid(d.guid).has_value());
  CHECK_FALSE(db.wall_by_guid(f.walls[1].guid).has_value());
  // the old joins went with their unlocked walls; the new one to a stands
  const auto stored = db.wall_joins();
  REQUIRE(stored.size() == 1);
  CHECK(stored[0].wall_a == f.walls[0].guid);
  CHECK(stored[0].wall_b == d.guid);
}

TEST_CASE("Walls_Save_KeepsAJoinBetweenTwoLockedWalls", "[projectdb][walls]") {
  TempPath tmp("test_walls_locked_join");
  ProjectDB db(tmp.path);
  auto f = three_walls(db);
  f.walls[0].locked = f.walls[1].locked = true;
  db.save_walls(f.walls, f.joins);
  // A re-run: the a-b join between the two locked walls is kept (not
  // duplicated); the c-a tee went with unlocked c and comes back with it.
  db.save_walls(f.walls, f.joins);
  const auto joins = db.wall_joins();
  REQUIRE(joins.size() == 2);
  CHECK(joins[0].kind == "corner");
  CHECK(joins[1].kind == "tee");
  // Without joins in the re-run, the locked pair's join still stands.
  db.save_walls(f.walls, {});
  REQUIRE(db.wall_joins().size() == 1);
  CHECK(db.wall_joins()[0].kind == "corner");
}

TEST_CASE("Walls_Save_RejectsInvalidRecords", "[projectdb][walls]") {
  TempPath tmp("test_walls_invalid");
  ProjectDB db(tmp.path);
  const auto f = three_walls(db);

  auto no_guid = f.walls;
  no_guid[0].guid.clear();
  CHECK_THROWS_AS(db.save_walls(no_guid, {}), std::invalid_argument);
  auto bad_source = f.walls;
  bad_source[0].source = "guessed";
  CHECK_THROWS_AS(db.save_walls(bad_source, {}), std::invalid_argument);
  auto dup = f.walls;
  dup[1].guid = dup[0].guid;
  CHECK_THROWS_AS(db.save_walls(dup, {}), std::invalid_argument);

  auto bad_kind = f.joins;
  bad_kind[0].kind = "glued";
  CHECK_THROWS_AS(db.save_walls(f.walls, bad_kind), std::invalid_argument);
  auto bad_end = f.joins;
  bad_end[0].at_a = "middle";
  CHECK_THROWS_AS(db.save_walls(f.walls, bad_end), std::invalid_argument);
  auto unknown = f.joins;
  unknown[0].wall_b = "no-such-wall";
  CHECK_THROWS_AS(db.save_walls(f.walls, unknown), std::invalid_argument);
  auto self = f.joins;
  self[0].wall_b = self[0].wall_a;
  CHECK_THROWS_AS(db.save_walls(f.walls, self), std::invalid_argument);

  // nothing was written by any refused call
  CHECK_FALSE(db.has_walls());
}

TEST_CASE("Walls_Delete_RemovesLockedRowsAndJoins", "[projectdb][walls]") {
  TempPath tmp("test_walls_delete");
  ProjectDB db(tmp.path);
  auto f = three_walls(db);
  f.walls[0].locked = true;
  db.save_walls(f.walls, f.joins);
  db.delete_walls();
  CHECK_FALSE(db.has_walls());
  CHECK(db.wall_joins().empty());
}

TEST_CASE("Walls_ReadOnlyProject_RefusesWrites", "[projectdb][walls]") {
  TempPath tmp("test_walls_ro");
  {
    ProjectDB db(tmp.path);
  }
  ProjectDB ro(tmp.path, /*readOnly=*/true);
  CHECK_THROWS_AS(ro.save_walls(three_walls(ro).walls, {}), std::runtime_error);
}

TEST_CASE("LoadSolidModel_ReadsStoredWalls", "[projectdb][walls][io]") {
  TempPath tmp("test_walls_load_model");
  ProjectDB db(tmp.path);
  {
    const auto m = reusex::io::load_solid_model(db);
    CHECK(m.walls.empty());
    CHECK_FALSE(m.has_walls()); // never ran: n/a
  }
  const auto f = three_walls(db);
  db.save_walls(f.walls, f.joins);
  const auto m = reusex::io::load_solid_model(db);
  CHECK(m.walls.size() == 3);
  CHECK(m.wall_joins.size() == 2);
  CHECK(m.has_walls());
}
