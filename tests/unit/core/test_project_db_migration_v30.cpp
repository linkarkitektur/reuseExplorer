// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Schema v29 -> v30 (solid-model WP1, reusex-z1v.3.2): the storeys table and
// the project_guid setting. A fresh project is rolled back to the exact v29
// shape with raw SQL (no storeys table, no project_guid), then reopened.

#include <catch2/catch_test_macros.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/guid.hpp>

#include "../../support/temp_path.hpp"

#include <sqlite3.h>

#include <filesystem>
#include <optional>
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

void roll_back_to_v29(const fs::path &path) {
  exec_raw(path, R"(
    DROP TABLE storeys;
    DELETE FROM project_settings WHERE key = 'project_guid';
    DELETE FROM schema_version WHERE version >= 30;
  )");
}

} // namespace

TEST_CASE("MigrationV30_FreshProject_HasStoreysAndProjectGuid",
          "[projectdb][migration][storeys]") {
  TempPath tmp("test_migration_v30_fresh");
  {
    ProjectDB db(tmp.path);
    CHECK(db.schema_version() == ProjectDB::latest_schema_version());
    CHECK(ProjectDB::latest_schema_version() >= 30);
    CHECK(db.project_guid().size() == 36);
    CHECK_FALSE(db.has_storeys());
  }
  CHECK(table_exists(tmp.path, "storeys"));
}

TEST_CASE("MigrationV30_FromV29_AddsTableAndMintsGuid",
          "[projectdb][migration][storeys]") {
  TempPath tmp("test_migration_v30_upgrade");
  {
    ProjectDB db(tmp.path);
    db.set_setting("other", "kept");
  }
  roll_back_to_v29(tmp.path);
  REQUIRE_FALSE(table_exists(tmp.path, "storeys"));

  {
    // Read-only: no migration, storeys read as empty, no GUID to derive from.
    ProjectDB ro(tmp.path, /*readOnly=*/true);
    CHECK(ro.schema_version() == 29);
    CHECK(ro.storeys().empty());
    CHECK_FALSE(ro.has_storeys());
    CHECK_THROWS_AS(ro.project_guid(), std::runtime_error);
  }

  std::string guid;
  {
    ProjectDB db(tmp.path);
    CHECK(db.schema_version() == ProjectDB::latest_schema_version());
    guid = db.project_guid();
    CHECK(guid.size() == 36);
    CHECK(db.setting("other") == std::optional<std::string>("kept"));
    reusex::core::StoreyRecord s;
    s.guid = reusex::core::deterministic_guid(guid, "storeys/0");
    s.z_ceiling = 2.6;
    CHECK(db.save_storeys({s}) == 1);
  }
  ProjectDB again(tmp.path);
  CHECK(again.project_guid() == guid); // minted once, not per open
  CHECK(again.storeys().size() == 1);
}
