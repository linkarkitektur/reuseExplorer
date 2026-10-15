// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Schema v30 -> v31 (solid-model WP2, reusex-z1v.4.2): the plane_classes
// table. A fresh project is rolled back to the exact v30 shape with raw SQL
// (no plane_classes table), then reopened.

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

void roll_back_to_v30(const fs::path &path) {
  exec_raw(path, R"(
    DROP TABLE plane_classes;
    DELETE FROM schema_version WHERE version >= 31;
  )");
}

reusex::core::PlaneClassRecord one_plane(const ProjectDB &db) {
  reusex::core::PlaneClassRecord r;
  r.plane_label = 1;
  r.guid = reusex::core::deterministic_guid(db.project_guid(),
                                            "plane_classes/planes/1");
  r.orientation = "horizontal";
  r.role = "floor";
  r.role_source = "rules";
  return r;
}

void seed_planes(ProjectDB &db) {
  reusex::CloudL labels;
  reusex::LabelT l;
  l.label = 1;
  labels.push_back(l);
  db.save_point_cloud("planes", labels, "segment_planes", "{}");
}

} // namespace

TEST_CASE("MigrationV31_FreshProject_HasPlaneClasses",
          "[projectdb][migration][plane_classes]") {
  TempPath tmp("test_migration_v31_fresh");
  {
    ProjectDB db(tmp.path);
    CHECK(db.schema_version() == ProjectDB::latest_schema_version());
    CHECK(ProjectDB::latest_schema_version() >= 31);
    CHECK_FALSE(db.has_plane_classes());
  }
  CHECK(table_exists(tmp.path, "plane_classes"));
}

TEST_CASE("MigrationV31_FromV30_AddsTableKeepsData",
          "[projectdb][migration][plane_classes]") {
  TempPath tmp("test_migration_v31_upgrade");
  std::string guid;
  {
    ProjectDB db(tmp.path);
    guid = db.project_guid();
    seed_planes(db);
    reusex::core::StoreyRecord s;
    s.guid = reusex::core::deterministic_guid(guid, "storeys/z0");
    s.z_ceiling = 2.6;
    db.save_storeys({s});
  }
  roll_back_to_v30(tmp.path);
  REQUIRE_FALSE(table_exists(tmp.path, "plane_classes"));

  {
    // Read-only: no migration, plane classes read as empty, writes refused.
    ProjectDB ro(tmp.path, /*readOnly=*/true);
    CHECK(ro.schema_version() == 30);
    CHECK(ro.plane_classes().empty());
    CHECK_FALSE(ro.has_plane_classes());
  }

  {
    ProjectDB db(tmp.path);
    CHECK(db.schema_version() == ProjectDB::latest_schema_version());
    CHECK(db.project_guid() == guid); // v30 data untouched
    CHECK(db.storeys().size() == 1);
    CHECK(db.save_plane_classes("planes", {one_plane(db)}) == 1);
  }
  ProjectDB again(tmp.path);
  REQUIRE(again.plane_classes().size() == 1);
  CHECK(again.plane_classes()[0].role == "floor");
}

TEST_CASE("PlaneClasses_PreV31ReadWriteSave_IsRefusedClearly",
          "[projectdb][migration][plane_classes]") {
  // A v30 project opened read-only cannot gain the table; saving says why.
  TempPath tmp("test_migration_v31_ro_save");
  {
    ProjectDB db(tmp.path);
    seed_planes(db);
  }
  roll_back_to_v30(tmp.path);
  ProjectDB ro(tmp.path, /*readOnly=*/true);
  CHECK_THROWS_AS(ro.save_plane_classes("planes", {one_plane(ro)}),
                  std::runtime_error);
}
