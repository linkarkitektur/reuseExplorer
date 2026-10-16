// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Schema v33 -> v34 (solid-model WP5, reusex-z1v.7.3): the openings table.
// A fresh project is rolled back to the exact v33 shape with raw SQL (no
// openings), then reopened.

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

reusex::core::OpeningRecord one_opening(const ProjectDB &db) {
  reusex::core::OpeningRecord o;
  o.guid = reusex::core::deterministic_guid(db.project_guid(), "openings/w/1");
  o.wall = "w";
  o.kind = "window";
  o.s_start = 1.0;
  o.s_end = 2.0;
  o.z_sill = 0.9;
  o.z_head = 2.1;
  return o;
}

} // namespace

TEST_CASE("MigrationV34_FreshProject_HasOpenings",
          "[projectdb][migration][openings]") {
  TempPath tmp("test_migration_v34_fresh");
  {
    ProjectDB db(tmp.path);
    CHECK(db.schema_version() == ProjectDB::latest_schema_version());
    CHECK(ProjectDB::latest_schema_version() >= 34);
    CHECK_FALSE(db.has_openings());
  }
  CHECK(table_exists(tmp.path, "openings"));
}

TEST_CASE("MigrationV34_FromV33_AddsTableKeepsData",
          "[projectdb][migration][openings]") {
  TempPath tmp("test_migration_v34_upgrade");
  std::string guid;
  {
    ProjectDB db(tmp.path);
    guid = db.project_guid();
    reusex::core::WallRecord w;
    w.guid = reusex::core::deterministic_guid(guid, "walls/0/0,0,4,0");
    w.x1 = 4.0;
    db.save_walls({w}, {});
  }
  exec_raw(tmp.path, R"(
    DROP TABLE openings;
    DELETE FROM schema_version WHERE version >= 34;
  )");
  REQUIRE_FALSE(table_exists(tmp.path, "openings"));

  {
    // Read-only: no migration, openings read as empty, writes refused.
    ProjectDB ro(tmp.path, /*readOnly=*/true);
    CHECK(ro.schema_version() == 33);
    CHECK(ro.openings().empty());
    CHECK_FALSE(ro.has_openings());
    CHECK_THROWS_AS(ro.save_openings({one_opening(ro)}), std::runtime_error);
  }
  {
    ProjectDB db(tmp.path);
    CHECK(db.schema_version() == ProjectDB::latest_schema_version());
    CHECK(db.project_guid() == guid);
    CHECK(db.walls().size() == 1); // earlier data untouched
    CHECK(db.save_openings({one_opening(db)}) == 1);
  }
  ProjectDB again(tmp.path);
  REQUIRE(again.openings().size() == 1);
  CHECK(again.openings()[0].kind == "window");
}
