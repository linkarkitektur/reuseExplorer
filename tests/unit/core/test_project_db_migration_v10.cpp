// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <catch2/catch_test_macros.hpp>

#include <core/ProjectDB.hpp>

#include "../../support/temp_path.hpp"

#include <opencv2/core.hpp>
#include <sqlite3.h>

#include <filesystem>
#include <string>

using namespace reusex;
namespace fs = std::filesystem;

namespace {

// Keeps the original "test_migration_v10" filename prefix while delegating
// uniqueness/cleanup to the shared helper.
struct TempPath : reusex::test_support::TempPath {
  TempPath() : reusex::test_support::TempPath("test_migration_v10") {}
};

void exec(sqlite3 *db, const char *sql) {
  char *err = nullptr;
  int rc = sqlite3_exec(db, sql, nullptr, nullptr, &err);
  std::string msg = err ? err : "";
  sqlite3_free(err);
  REQUIRE(rc == SQLITE_OK);
}

// Build a minimal schema-version-9 database directly with sqlite3, containing:
//  - one instance-label cloud ("instances") with label_definitions
//  - two passports (one referenced, one that will be orphaned)
//  - two instance_materials links: one valid, one dangling (missing passport)
void buildV9Fixture(const fs::path &path) {
  sqlite3 *db = nullptr;
  REQUIRE(sqlite3_open(path.string().c_str(), &db) == SQLITE_OK);

  exec(db, "CREATE TABLE schema_version (version INTEGER NOT NULL, "
           "applied_at TEXT, description TEXT);");
  // Mark as v9 so ProjectDB runs only migrateToV10.
  exec(db, "INSERT INTO schema_version (version, description) VALUES "
           "(9, 'test fixture');");

  exec(db, "CREATE TABLE point_clouds (id INTEGER PRIMARY KEY AUTOINCREMENT, "
           "name TEXT UNIQUE, point_type TEXT, point_count INTEGER, "
           "point_step INTEGER, width INTEGER, height INTEGER, stage TEXT, "
           "parameters TEXT);");
  exec(db, "INSERT INTO point_clouds (id, name, point_type, point_count, "
           "point_step, width, height, stage, parameters) VALUES "
           "(1, 'instances', 'Label', 4, 4, 4, 1, 'segment_instances', '');");

  exec(db, "CREATE TABLE label_definitions (cloud_id INTEGER, label_id "
           "INTEGER, name TEXT, PRIMARY KEY (cloud_id, label_id));");
  exec(db, "INSERT INTO label_definitions VALUES (1, 1, 'SM3-1 (2p)');");
  exec(db, "INSERT INTO label_definitions VALUES (1, 2, 'SM3-2 (2p)');");

  exec(db, "CREATE TABLE material_passports (id TEXT PRIMARY KEY, project_id "
           "TEXT, document_guid TEXT UNIQUE, created_at TEXT, revised_at TEXT, "
           "version_number TEXT, version_date TEXT);");
  exec(db, "INSERT INTO material_passports (id, document_guid, created_at) "
           "VALUES ('p1', 'mat-valid', '2025-01-01');");

  // v9 instance_materials: no FKs.
  exec(db,
       "CREATE TABLE instance_materials (cloud_id INTEGER, instance_id "
       "INTEGER, material_guid TEXT, PRIMARY KEY (cloud_id, instance_id));");
  // Valid link (instance 1 -> existing passport).
  exec(db, "INSERT INTO instance_materials VALUES (1, 1, 'mat-valid');");
  // Orphan link (instance 2 -> passport that does not exist).
  exec(db, "INSERT INTO instance_materials VALUES (2, 2, 'mat-missing');");

  sqlite3_close(db);
}

// Roll a freshly-created (v14) database back to v13 by dropping the
// glass_confidence_images table and rewinding the schema_version row.
// Read-only opens never migrate, so this is what a v13 project on disk
// looks like to today's reader.
void downgrade_to_v13(const fs::path &path) {
  sqlite3 *db = nullptr;
  REQUIRE(sqlite3_open(path.string().c_str(), &db) == SQLITE_OK);
  char *err = nullptr;
  const int rc = sqlite3_exec(db,
                              "DROP TABLE IF EXISTS pose_graph_edges;"
                              "DROP TABLE IF EXISTS glass_confidence_images;"
                              "DELETE FROM schema_version WHERE version >= 14;",
                              nullptr, nullptr, &err);
  sqlite3_free(err);
  sqlite3_close(db);
  REQUIRE(rc == SQLITE_OK);
}

} // namespace

TEST_CASE("ProjectDbSchemaVersion_FreshMigrationDatabase_IsLatest",
          "[projectdb][migration]") {
  TempPath tmp;
  ProjectDB db(tmp.path);
  REQUIRE(db.schema_version() == ProjectDB::latest_schema_version());
}

TEST_CASE("ProjectDb_PreV13ProjectReadWriteOpen_MigratesToV14_"
          "GlassConfidenceImagesTableExistsAndCascades",
          "[projectdb][migration]") {
  TempPath tmp;
  {
    ProjectDB fresh(tmp.path); // creates v14
  }
  downgrade_to_v13(tmp.path);

  // Read-only open must not migrate; table is still absent.
  {
    ProjectDB ro(tmp.path, /*readOnly=*/true);
    REQUIRE(ro.schema_version() == 13);
    // hasGlassConfidenceImage returns false when the table is absent.
    CHECK_FALSE(ro.has_glass_confidence_image(1));
  }

  // Read-write open triggers migrateToV14.
  {
    ProjectDB db(tmp.path);
    REQUIRE(db.schema_version() == ProjectDB::latest_schema_version());

    // Insert a sensor frame + glass confidence image.
    cv::Mat color(4, 4, CV_8UC3, cv::Scalar(0, 0, 0));
    db.save_sensor_frame(99, color);
    cv::Mat conf(4, 4, CV_8U, cv::Scalar(255));
    db.save_glass_confidence_image(99, conf);
    REQUIRE(db.has_glass_confidence_image(99));
  }

  // Re-open and verify the image survived across close/reopen.
  {
    ProjectDB db(tmp.path);
    REQUIRE(db.has_glass_confidence_image(99));

    // Delete the parent sensor frame via raw sqlite3; the FK CASCADE must
    // remove the glass_confidence_images row automatically.
    sqlite3 *raw = nullptr;
    REQUIRE(sqlite3_open(tmp.path.string().c_str(), &raw) == SQLITE_OK);
    exec(raw, "PRAGMA foreign_keys = ON;");
    exec(raw, "DELETE FROM sensor_frames WHERE node_id = 99;");
    sqlite3_close(raw);
  }

  // Reopen and confirm the cascade fired.
  {
    ProjectDB db(tmp.path);
    CHECK_FALSE(db.has_glass_confidence_image(99));
  }
}

TEST_CASE("ProjectDbMigrationV9ToV10_OrphanAndValidInstanceMaterialLinks_"
          "DropsOrphanKeepsValid",
          "[projectdb][migration]") {
  TempPath tmp;
  buildV9Fixture(tmp.path);

  // Opening triggers migrateToV10 (and every later migration).
  ProjectDB db(tmp.path);
  REQUIRE(db.schema_version() >= 10);

  // Instances were backfilled from label_definitions.
  auto instances = db.instances("instances");
  REQUIRE(instances.size() == 2);
  // Semantic class parsed from "SM3-..." naming by v10, then re-based to the
  // point-label numbering (API class + 1) by v29, definitions included.
  for (const auto &r : instances)
    REQUIRE(r.semantic_class == 4);
  REQUIRE(db.label_definitions("instances").at(1) == "SM4-1 (2p)");

  // The valid link survived; the orphan (missing passport) was dropped.
  auto links = db.instance_materials("instances");
  REQUIRE(links.size() == 1);
  REQUIRE(links.count(1) == 1);
  REQUIRE(links.at(1) == "mat-valid");
  REQUIRE(links.count(2) == 0);
}
