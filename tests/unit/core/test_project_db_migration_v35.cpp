// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Schema v34 -> v35 (solid-model WP4, reusex-z1v.6.3): the spaces and
// space_boundaries tables. A fresh project is rolled back to the exact v34
// shape with raw SQL (no spaces tables), then reopened.

#include <catch2/catch_test_macros.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/guid.hpp>

#include "../../support/temp_path.hpp"

#include <sqlite3.h>

#include <filesystem>
#include <stdexcept>
#include <string>

using reusex::ProjectDB;
using reusex::test_support::TempPath;
namespace fs = std::filesystem;

namespace {

void exec_raw(const fs::path &path, const std::string &sql) {
  sqlite3 *raw = nullptr;
  REQUIRE(sqlite3_open(path.string().c_str(), &raw) == SQLITE_OK);
  char *err = nullptr;
  const int rc = sqlite3_exec(raw, sql.c_str(), nullptr, nullptr, &err);
  INFO((err ? err : ""));
  sqlite3_free(err);
  sqlite3_close(raw);
  REQUIRE(rc == SQLITE_OK);
}

bool table_exists(const fs::path &path, const char *table) {
  sqlite3 *raw = nullptr;
  REQUIRE(sqlite3_open(path.string().c_str(), &raw) == SQLITE_OK);
  sqlite3_stmt *s = nullptr;
  sqlite3_prepare_v2(
      raw, "SELECT 1 FROM sqlite_master WHERE type='table' AND name=?;", -1, &s,
      nullptr);
  sqlite3_bind_text(s, 1, table, -1, SQLITE_STATIC);
  const bool found = sqlite3_step(s) == SQLITE_ROW;
  sqlite3_finalize(s);
  sqlite3_close(raw);
  return found;
}

reusex::core::SpaceRecord one_space(const ProjectDB &db) {
  reusex::core::SpaceRecord s;
  s.guid = reusex::core::deterministic_guid(db.project_guid(), "spaces/0/1,1");
  s.kind = "room";
  s.polygon.outer = {{0, 0}, {2, 0}, {2, 2}, {0, 2}};
  return s;
}

} // namespace

TEST_CASE("MigrationV35_FreshProject_HasSpaces",
          "[projectdb][migration][spaces]") {
  TempPath tmp("test_migration_v35_fresh");
  {
    ProjectDB db(tmp.path);
    CHECK(db.schema_version() == ProjectDB::latest_schema_version());
    CHECK(ProjectDB::latest_schema_version() >= 35);
    CHECK_FALSE(db.has_spaces());
  }
  CHECK(table_exists(tmp.path, "spaces"));
  CHECK(table_exists(tmp.path, "space_boundaries"));
}

TEST_CASE("MigrationV35_FromV34_AddsTablesKeepsData",
          "[projectdb][migration][spaces]") {
  TempPath tmp("test_migration_v35_upgrade");
  std::string guid;
  {
    ProjectDB db(tmp.path);
    guid = db.project_guid();
    reusex::core::WallRecord w;
    w.guid = reusex::core::deterministic_guid(guid, "walls/0/0,0,4,0");
    w.x1 = 4.0;
    db.save_walls({w}, {});
    reusex::core::OpeningRecord o;
    o.guid = reusex::core::deterministic_guid(guid, "openings/w/1");
    o.wall = w.guid;
    o.s_start = 1.0;
    o.s_end = 2.0;
    o.z_head = 2.0;
    db.save_openings({o});
  }
  exec_raw(tmp.path, R"(
    DROP TABLE space_boundaries;
    DROP TABLE spaces;
    DELETE FROM schema_version WHERE version >= 35;
  )");
  REQUIRE_FALSE(table_exists(tmp.path, "spaces"));

  {
    // Read-only: no migration, spaces read as empty, writes refused.
    ProjectDB ro(tmp.path, /*readOnly=*/true);
    CHECK(ro.schema_version() == 34);
    CHECK(ro.spaces().empty());
    CHECK(ro.space_boundaries().empty());
    CHECK_FALSE(ro.has_spaces());
    CHECK_THROWS_AS(ro.save_spaces({one_space(ro)}, {}), std::runtime_error);
  }
  {
    ProjectDB db(tmp.path);
    CHECK(db.schema_version() == ProjectDB::latest_schema_version());
    CHECK(db.project_guid() == guid);
    CHECK(db.walls().size() == 1); // earlier data untouched
    CHECK(db.openings().size() == 1);
    CHECK(db.save_spaces({one_space(db)}, {}) == 1);
  }
  ProjectDB again(tmp.path);
  REQUIRE(again.spaces().size() == 1);
  CHECK(again.spaces()[0].polygon.outer.size() == 4);
}
