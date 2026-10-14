// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
// Schema v26 -> v27 (resource view record): six nullable survey_parts.view_*
// columns with a CHECK on the invariant. A fresh project is rolled back to
// v26 with raw SQL, then reopened read-write so migrateToV27 runs on real
// v26 data, and read-only to check a stale project still reads.

#include <catch2/catch_test_macros.hpp>

#include <core/ProjectDB.hpp>

#include "../../support/survey_fixture.hpp"
#include "../../support/temp_path.hpp"

#include <sqlite3.h>

#include <filesystem>
#include <string>

using reusex::ProjectDB;
namespace fs = std::filesystem;
using namespace reusex::test_support;

namespace {
struct TempDB : TempPath {
  TempDB() : TempPath("test_migration_v27") {}
};

int exec_rc(const fs::path &path, const std::string &sql) {
  sqlite3 *raw = nullptr;
  REQUIRE(sqlite3_open(path.string().c_str(), &raw) == SQLITE_OK);
  const int rc = sqlite3_exec(raw, sql.c_str(), nullptr, nullptr, nullptr);
  sqlite3_close(raw);
  return rc;
}

/// view_h first: its CHECK names the other view columns, and SQLite refuses
/// to drop a column another column's CHECK still names.
void roll_back_to_v26(const fs::path &path) {
  REQUIRE(exec_rc(path, "ALTER TABLE survey_parts DROP COLUMN view_h;"
                        "ALTER TABLE survey_parts DROP COLUMN view_w;"
                        "ALTER TABLE survey_parts DROP COLUMN view_y;"
                        "ALTER TABLE survey_parts DROP COLUMN view_x;"
                        "ALTER TABLE survey_parts DROP COLUMN view_id;"
                        "ALTER TABLE survey_parts DROP COLUMN view_kind;"
                        "DELETE FROM schema_version WHERE version >= 27;") ==
          SQLITE_OK);
}
} // namespace

TEST_CASE("MigrationV27_FreshProject_HasViewColumnsAndCheck",
          "[ProjectDB][migration]") {
  TempDB tmp;
  {
    ProjectDB db(tmp.path);
    CHECK(db.schema_version() == ProjectDB::latest_schema_version());
    CHECK(ProjectDB::latest_schema_version() >= 27);
    const auto t = make_type(db, "Døre");
    make_part(db, "RX-001", t);
    CHECK_FALSE(db.survey_part("RX-001")->view);
  }
  // The invariant holds in the schema, not only in the setter.
  CHECK(exec_rc(tmp.path, "UPDATE survey_parts SET view_kind = 'frame';") !=
        SQLITE_OK); // kind without id
  CHECK(exec_rc(tmp.path, "UPDATE survey_parts SET view_kind = 'photo', "
                          "view_id = 1;") != SQLITE_OK);
  CHECK(exec_rc(tmp.path,
                "UPDATE survey_parts SET view_kind = 'frame', "
                "view_id = 1, view_x = 0.1;") != SQLITE_OK); // partial region
  CHECK(exec_rc(tmp.path,
                "UPDATE survey_parts SET view_kind = 'frame', view_id = 1, "
                "view_x = 0, view_y = 0.6, view_w = 0.5, view_h = 0.5;") !=
        SQLITE_OK); // y + h > 1
  CHECK(exec_rc(tmp.path,
                "UPDATE survey_parts SET view_kind = 'frame', view_id = 1, "
                "view_x = 0.1, view_y = 0.1, view_w = 0.5, view_h = 0.5;") ==
        SQLITE_OK);
}

TEST_CASE("MigrationV27_FromV26_KeepsPartsWithNullViews",
          "[ProjectDB][migration]") {
  TempDB tmp;
  {
    ProjectDB db(tmp.path);
    make_instance_cloud(db, 1);
    const auto t = make_type(db, "Døre");
    make_part(db, "RX-001", t, 1u, 3.0);
  }
  roll_back_to_v26(tmp.path);
  {
    ProjectDB db(tmp.path);
    CHECK(db.schema_version() == ProjectDB::latest_schema_version());
    const auto p = db.survey_part("RX-001");
    REQUIRE(p);
    CHECK(p->quantity == 3.0);
    CHECK(p->instance_guid == "guid-inst-1");
    CHECK_FALSE(p->view);
  }
}

TEST_CASE("MigrationV27_ReadOnlyV26_ReadsWithoutViews",
          "[ProjectDB][migration]") {
  TempDB tmp;
  {
    ProjectDB db(tmp.path);
    const auto t = make_type(db, "Døre");
    make_part(db, "RX-001", t);
  }
  roll_back_to_v26(tmp.path);
  ProjectDB db(tmp.path, /*readOnly=*/true);
  CHECK(db.schema_version() == 26);
  const auto parts = db.survey_parts();
  REQUIRE(parts.size() == 1);
  CHECK_FALSE(parts[0].view);
}
