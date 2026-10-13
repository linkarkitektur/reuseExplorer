// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
// Schema v25 -> v26 (Kortlægning fixes spec A3): the survey tombstone table
// survey_dismissed_instances. A fresh project is rolled back to v25 with raw
// SQL, then reopened read-write so migrateToV26 runs on real v25 data.

#include <catch2/catch_test_macros.hpp>

#include <core/ProjectDB.hpp>

#include "../../support/survey_fixture.hpp"
#include "../../support/temp_path.hpp"

#include <sqlite3.h>

#include <filesystem>
#include <string>
#include <vector>

using reusex::ProjectDB;
namespace fs = std::filesystem;
using namespace reusex::test_support;

namespace {
struct TempDB : reusex::test_support::TempPath {
  TempDB() : reusex::test_support::TempPath("test_migration_v26") {}
};

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

void roll_back_to_v25(const fs::path &path) {
  exec_raw(path, "DROP TABLE survey_dismissed_instances;"
                 "DELETE FROM schema_version WHERE version >= 26;");
}

bool table_exists(const fs::path &path, const char *name) {
  sqlite3 *raw = nullptr;
  REQUIRE(sqlite3_open(path.string().c_str(), &raw) == SQLITE_OK);
  sqlite3_stmt *s = nullptr;
  sqlite3_prepare_v2(raw,
                     "SELECT 1 FROM sqlite_master WHERE type='table' AND "
                     "name=?;",
                     -1, &s, nullptr);
  sqlite3_bind_text(s, 1, name, -1, SQLITE_STATIC);
  const bool found = sqlite3_step(s) == SQLITE_ROW;
  sqlite3_finalize(s);
  sqlite3_close(raw);
  return found;
}
} // namespace

TEST_CASE("MigrationV26_FreshProject_HasTombstoneTable",
          "[ProjectDB][migration]") {
  TempDB tmp;
  {
    ProjectDB db(tmp.path);
    CHECK(db.schema_version() == ProjectDB::latest_schema_version());
    CHECK(db.dismissed_instances().empty());
    db.dismiss_instance("g-1");
    db.dismiss_instance("g-1"); // idempotent
    db.dismiss_instance("g-0");
    CHECK(db.is_instance_dismissed("g-1"));
    CHECK_FALSE(db.is_instance_dismissed("g-2"));
    CHECK(db.dismissed_instances() == std::vector<std::string>{"g-0", "g-1"});
  }
  CHECK(table_exists(tmp.path, "survey_dismissed_instances"));
}

TEST_CASE("MigrationV26_FromV25_KeepsSurveyData_AddsTable",
          "[ProjectDB][migration]") {
  TempDB tmp;
  {
    ProjectDB db(tmp.path);
    make_instance_cloud(db, 2);
    const auto t = make_type(db, "Døre");
    make_part(db, "RX-001", t, 1u);
    make_part(db, "RX-002", t);
  }
  roll_back_to_v25(tmp.path);
  REQUIRE_FALSE(table_exists(tmp.path, "survey_dismissed_instances"));
  {
    // A read-only open of the v25 project does not migrate, and the
    // tombstone readers degrade to "nothing dismissed".
    ProjectDB ro(tmp.path, /*readOnly=*/true);
    CHECK(ro.schema_version() == 25);
    CHECK_FALSE(ro.is_instance_dismissed("guid-inst-1"));
    CHECK(ro.dismissed_instances().empty());
  }
  ProjectDB db(tmp.path);
  CHECK(db.schema_version() == ProjectDB::latest_schema_version());
  CHECK(table_exists(tmp.path, "survey_dismissed_instances"));
  CHECK(db.survey_parts().size() == 2);
  CHECK(db.survey_part("RX-001")->instance_guid ==
        std::optional<std::string>("guid-inst-1"));
  db.dismiss_instance("guid-inst-1");
  CHECK(db.is_instance_dismissed("guid-inst-1"));
}
