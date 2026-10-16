// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Schema v32 -> v33 (solid-model WP3, reusex-z1v.5.5): the walls and
// wall_joins tables. A fresh project is rolled back to the exact v32 shape
// with raw SQL (no walls / wall_joins), then reopened.

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

void roll_back_to_v32(const fs::path &path) {
  exec_raw(path, R"(
    DROP TABLE wall_joins;
    DROP TABLE walls;
    DELETE FROM schema_version WHERE version >= 33;
  )");
}

reusex::core::WallRecord one_wall(const ProjectDB &db) {
  reusex::core::WallRecord w;
  w.guid =
      reusex::core::deterministic_guid(db.project_guid(), "walls/0/0,0,4,0");
  w.x1 = 4.0;
  w.height = 2.6;
  w.thickness = 0.2;
  w.assumed = true;
  return w;
}

} // namespace

TEST_CASE("MigrationV33_FreshProject_HasWalls",
          "[projectdb][migration][walls]") {
  TempPath tmp("test_migration_v33_fresh");
  {
    ProjectDB db(tmp.path);
    CHECK(db.schema_version() == ProjectDB::latest_schema_version());
    CHECK(ProjectDB::latest_schema_version() >= 33);
    CHECK_FALSE(db.has_walls());
  }
  CHECK(table_exists(tmp.path, "walls"));
  CHECK(table_exists(tmp.path, "wall_joins"));
}

TEST_CASE("MigrationV33_FromV32_AddsTablesKeepsData",
          "[projectdb][migration][walls]") {
  TempPath tmp("test_migration_v33_upgrade");
  std::string guid;
  {
    ProjectDB db(tmp.path);
    guid = db.project_guid();
    reusex::core::StoreyRecord s;
    s.guid = reusex::core::deterministic_guid(guid, "storeys/z0");
    s.z_ceiling = 2.6;
    db.save_storeys({s});
  }
  roll_back_to_v32(tmp.path);
  REQUIRE_FALSE(table_exists(tmp.path, "walls"));
  REQUIRE_FALSE(table_exists(tmp.path, "wall_joins"));

  {
    // Read-only: no migration, walls read as empty, writes refused.
    ProjectDB ro(tmp.path, /*readOnly=*/true);
    CHECK(ro.schema_version() == 32);
    CHECK(ro.walls().empty());
    CHECK(ro.wall_joins().empty());
    CHECK_FALSE(ro.has_walls());
    CHECK_THROWS_AS(ro.save_walls({one_wall(ro)}, {}), std::runtime_error);
  }

  {
    ProjectDB db(tmp.path);
    CHECK(db.schema_version() == ProjectDB::latest_schema_version());
    CHECK(db.project_guid() == guid); // earlier data untouched
    CHECK(db.storeys().size() == 1);
    CHECK(db.save_walls({one_wall(db)}, {}) == 1);
  }
  ProjectDB again(tmp.path);
  REQUIRE(again.walls().size() == 1);
  CHECK(again.walls()[0].assumed);
}
