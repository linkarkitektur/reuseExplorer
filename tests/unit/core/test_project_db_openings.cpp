// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// The openings table (schema v34, reusex-z1v.7.3): save / load / locked rows
// / validation, and io::load_solid_model filling openings.

#include <catch2/catch_test_macros.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/guid.hpp>
#include <reusex/io/solid_model_json.hpp>

#include "../../support/temp_path.hpp"

#include <stdexcept>
#include <string>
#include <vector>

using reusex::ProjectDB;
using reusex::core::OpeningRecord;
using reusex::core::WallRecord;
using reusex::test_support::TempPath;

namespace {

WallRecord host(const ProjectDB &db) {
  WallRecord w;
  w.guid = reusex::core::deterministic_guid(db.project_guid(), "walls/0/a");
  w.x1 = 5.0;
  w.height = 2.6;
  w.thickness = 0.12;
  return w;
}

OpeningRecord opening(const ProjectDB &db, const std::string &key,
                      const std::string &wall, double s0, double s1) {
  OpeningRecord o;
  o.guid = reusex::core::deterministic_guid(db.project_guid(), key);
  o.wall = wall;
  o.kind = "door";
  o.s_start = s0;
  o.s_end = s1;
  o.z_sill = 0.0;
  o.z_head = 2.05;
  o.state = "open";
  o.is_external = true;
  o.source_instance_guid = "inst-1";
  o.confidence = 0.8;
  o.metadata = R"({"fill":0.9})";
  return o;
}

} // namespace

TEST_CASE("Openings_FreshProject_IsEmpty", "[projectdb][openings]") {
  TempPath tmp("test_openings_empty");
  ProjectDB db(tmp.path);
  CHECK(db.openings().empty());
  CHECK_FALSE(db.has_openings());
  CHECK_FALSE(db.opening_by_guid("nope").has_value());
}

TEST_CASE("Openings_SaveAndLoad_RoundTripsEveryField",
          "[projectdb][openings]") {
  TempPath tmp("test_openings_roundtrip");
  ProjectDB db(tmp.path);
  const auto w = host(db);
  auto a = opening(db, "openings/a/1.00", w.guid, 1.0, 1.9);
  auto b = opening(db, "openings/a/3.00", w.guid, 3.0, 4.2);
  b.kind = "window";
  b.state = "unknown";
  b.z_sill = 0.9;
  b.is_external = false;
  b.source_instance_guid.clear();
  b.locked = true;
  b.source = "manual";
  CHECK(db.save_openings({b, a}, 7) == 2);
  CHECK(db.has_openings());
  const auto rows = db.openings();
  REQUIRE(rows.size() == 2);
  // By host, then s_start.
  const auto &r = rows[0];
  CHECK(r.id > 0);
  CHECK(r.guid == a.guid);
  CHECK(r.wall == w.guid);
  CHECK(r.kind == "door");
  CHECK(r.s_start == 1.0);
  CHECK(r.s_end == 1.9);
  CHECK(r.z_sill == 0.0);
  CHECK(r.z_head == 2.05);
  CHECK(r.state == "open");
  CHECK(r.is_external);
  CHECK(r.source_instance_guid == "inst-1");
  CHECK(r.source == "detected");
  CHECK_FALSE(r.locked);
  CHECK(r.confidence == 0.8);
  CHECK(r.metadata == R"({"fill":0.9})");
  CHECK(rows[1].kind == "window");
  CHECK(rows[1].locked);
  CHECK(db.opening_by_guid(b.guid)->z_sill == 0.9);
}

TEST_CASE("Openings_Save_ReplacesUnlockedRowsOnly", "[projectdb][openings]") {
  TempPath tmp("test_openings_locked");
  ProjectDB db(tmp.path);
  const auto w = host(db);
  auto keep = opening(db, "k", w.guid, 1.0, 1.9);
  keep.locked = true;
  keep.source = "manual";
  const auto gone = opening(db, "g", w.guid, 3.0, 3.9);
  db.save_openings({keep, gone});

  // A re-run: the locked one comes back moved (skipped: the stored row
  // wins), the unlocked one is not found again, a new one is.
  auto moved = keep;
  moved.locked = false;
  moved.source = "detected";
  moved.s_start = 1.2;
  const auto fresh = opening(db, "n", w.guid, 4.0, 4.8);
  CHECK(db.save_openings({moved, fresh}) == 1);
  const auto rows = db.openings();
  REQUIRE(rows.size() == 2);
  CHECK(rows[0].guid == keep.guid);
  CHECK(rows[0].s_start == 1.0);
  CHECK(rows[0].locked);
  CHECK(rows[1].guid == fresh.guid);
}

TEST_CASE("Openings_Save_RejectsInvalidRecords", "[projectdb][openings]") {
  TempPath tmp("test_openings_invalid");
  ProjectDB db(tmp.path);
  const auto w = host(db);
  const auto ok = opening(db, "a", w.guid, 1.0, 1.9);
  auto bad = ok;
  bad.guid.clear();
  CHECK_THROWS_AS(db.save_openings({bad}), std::invalid_argument);
  bad = ok;
  bad.wall.clear();
  CHECK_THROWS_AS(db.save_openings({bad}), std::invalid_argument);
  bad = ok;
  bad.kind = "hatch";
  CHECK_THROWS_AS(db.save_openings({bad}), std::invalid_argument);
  bad = ok;
  bad.state = "ajar";
  CHECK_THROWS_AS(db.save_openings({bad}), std::invalid_argument);
  bad = ok;
  bad.source = "guessed";
  CHECK_THROWS_AS(db.save_openings({bad}), std::invalid_argument);
  bad = ok;
  bad.s_end = bad.s_start;
  CHECK_THROWS_AS(db.save_openings({bad}), std::invalid_argument);
  CHECK_THROWS_AS(db.save_openings({ok, ok}), std::invalid_argument);
  // A refused call leaves the table as it was.
  db.save_openings({ok});
  CHECK_THROWS_AS(db.save_openings({bad}), std::invalid_argument);
  CHECK(db.openings().size() == 1);
}

TEST_CASE("Openings_Delete_AndReadOnly", "[projectdb][openings]") {
  TempPath tmp("test_openings_delete");
  {
    ProjectDB db(tmp.path);
    auto o = opening(db, "a", host(db).guid, 1.0, 1.9);
    o.locked = true;
    db.save_openings({o});
  }
  {
    ProjectDB ro(tmp.path, /*readOnly=*/true);
    CHECK(ro.openings().size() == 1);
    CHECK_THROWS_AS(ro.save_openings({}), std::runtime_error);
    CHECK_THROWS_AS(ro.delete_openings(), std::runtime_error);
  }
  ProjectDB db(tmp.path);
  db.delete_openings();
  CHECK_FALSE(db.has_openings());
}

TEST_CASE("LoadSolidModel_ReadsOpenings_SkipsOrphans",
          "[projectdb][openings][io]") {
  TempPath tmp("test_openings_load_model");
  ProjectDB db(tmp.path);
  reusex::core::StoreyRecord s;
  s.guid = reusex::core::deterministic_guid(db.project_guid(), "storeys/z0");
  s.z_ceiling = 2.6;
  db.save_storeys({s});
  const auto w = host(db);
  db.save_walls({w}, {});
  {
    const auto m = reusex::io::load_solid_model(db);
    CHECK(m.openings.empty());
    CHECK_FALSE(m.has_openings());
  }
  db.save_openings({opening(db, "a", w.guid, 1.0, 1.9),
                    opening(db, "b", "a-wall-that-is-gone", 1.0, 1.9)});
  const auto m = reusex::io::load_solid_model(db);
  REQUIRE(m.openings.size() == 1);
  CHECK(m.openings[0].wall == w.guid);
  CHECK(m.has_openings());
}
