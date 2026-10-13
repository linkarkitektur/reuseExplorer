// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Schema v26 -> v27 (pose-graph editor spec §3): pose_graph_edges gains the
// measurement, information, provenance, status and metric columns plus a
// UNIQUE(from, to, type) key on canonical (from < to) rows;
// sensor_frames.initial_transform and project_settings appear. A fresh project
// is rolled back to the exact v26 shape with raw SQL, seeded with the kinds of
// rows `rux optimize` and ruxd wrote under v15..v26, then reopened.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <reusex/core/ProjectDB.hpp>

#include "../../support/temp_path.hpp"

#include <sqlite3.h>

#include <cmath>
#include <filesystem>
#include <string>

using Catch::Matchers::WithinAbs;
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

bool column_exists(const fs::path &path, const char *table, const char *col) {
  sqlite3 *raw = nullptr;
  REQUIRE(sqlite3_open(path.string().c_str(), &raw) == SQLITE_OK);
  sqlite3_stmt *s = nullptr;
  const std::string sql = std::string("PRAGMA table_info(") + table + ");";
  sqlite3_prepare_v2(raw, sql.c_str(), -1, &s, nullptr);
  bool found = false;
  while (sqlite3_step(s) == SQLITE_ROW)
    if (std::string(
            reinterpret_cast<const char *>(sqlite3_column_text(s, 1))) == col)
      found = true;
  sqlite3_finalize(s);
  sqlite3_close(raw);
  return found;
}

/// The exact v15..v26 pose_graph_edges DDL plus legacy rows: a duplicate in
/// reverse direction, a self-loop, a reversed loop closure pair and a
/// second type on the same pair.
void roll_back_to_v26_with_legacy_rows(const fs::path &path) {
  exec_raw(path, R"(
    DROP TABLE pose_graph_edges;
    CREATE TABLE pose_graph_edges (
      id           INTEGER PRIMARY KEY,
      from_node_id INTEGER NOT NULL,
      to_node_id   INTEGER NOT NULL,
      edge_type    TEXT    NOT NULL,
      residual     REAL    NOT NULL,
      weight       REAL
    );
    INSERT INTO pose_graph_edges VALUES (1, 1, 2, 'odometry',     0.1, 100.0);
    INSERT INTO pose_graph_edges VALUES (2, 2, 1, 'odometry',     0.2,  50.0);
    INSERT INTO pose_graph_edges VALUES (3, 9, 5, 'loop_closure', 0.5,  NULL);
    INSERT INTO pose_graph_edges VALUES (4, 5, 9, 'loop_closure', 0.6,  25.0);
    INSERT INTO pose_graph_edges VALUES (5, 3, 3, 'loop_closure', 0.7,  25.0);
    INSERT INTO pose_graph_edges VALUES (6, 1, 2, 'loop_closure', 0.8,  25.0);
    DROP TABLE project_settings;
    ALTER TABLE sensor_frames DROP COLUMN initial_transform;
    DELETE FROM schema_version WHERE version >= 27;
  )");
}

} // namespace

TEST_CASE("MigrationV27_FreshProject_HasNewColumnsAndTables",
          "[projectdb][migration][pose_graph]") {
  TempPath tmp("test_migration_v27_fresh");
  {
    ProjectDB db(tmp.path);
    CHECK(db.schema_version() == 27);
    CHECK(ProjectDB::latest_schema_version() == 27);
    CHECK(db.list_pose_graph_edges().empty());
  }
  for (const char *col :
       {"transform", "information", "source", "status", "created_at",
        "seed_disagreement_m", "seed_disagreement_deg", "deviation_m",
        "deviation_deg"}) {
    INFO(col);
    CHECK(column_exists(tmp.path, "pose_graph_edges", col));
  }
  CHECK(column_exists(tmp.path, "sensor_frames", "initial_transform"));
  CHECK(column_exists(tmp.path, "project_settings", "value"));
}

TEST_CASE("MigrationV27_ReadOnlyV26_DegradesToLegacyRows",
          "[projectdb][migration][pose_graph]") {
  TempPath tmp("test_migration_v27_ro");
  {
    ProjectDB db(tmp.path);
  }
  roll_back_to_v26_with_legacy_rows(tmp.path);

  ProjectDB ro(tmp.path, /*readOnly=*/true);
  REQUIRE(ro.schema_version() == 26);
  const auto rows = ro.list_pose_graph_edges(); // must not throw
  REQUIRE(rows.size() == 6);
  CHECK(rows[0].source == "legacy");
  CHECK(rows[0].status == "auto");
  CHECK_FALSE(rows[0].transform.has_value());
  CHECK(rows[0].id == 1);
  CHECK(ro.has_pose_graph());
}

TEST_CASE("MigrationV27_FromV26_DeduplicatesCanonicalisesAndKeepsLowestId",
          "[projectdb][migration][pose_graph]") {
  TempPath tmp("test_migration_v27_dedup");
  {
    ProjectDB db(tmp.path);
  }
  roll_back_to_v26_with_legacy_rows(tmp.path);

  ProjectDB db(tmp.path);
  REQUIRE(db.schema_version() == 27);
  const auto rows = db.list_pose_graph_edges();
  // (1,2,odometry) from ids 1+2 -> id 1; (5,9,loop_closure) from ids 3+4 ->
  // id 3; self-loop id 5 dropped; (1,2,loop_closure) id 6 kept.
  REQUIRE(rows.size() == 3);
  CHECK(rows[0].id == 1);
  CHECK(rows[0].from_node_id == 1);
  CHECK(rows[0].to_node_id == 2);
  CHECK(rows[0].edge_type == "odometry");
  CHECK_THAT(rows[0].residual, WithinAbs(0.1, 1e-12));
  CHECK(rows[1].id == 3);
  CHECK(rows[1].from_node_id == 5);
  CHECK(rows[1].to_node_id == 9);
  CHECK_THAT(rows[1].residual, WithinAbs(0.5, 1e-12));
  CHECK(std::isnan(rows[1].weight));
  CHECK(rows[2].id == 6);
  for (const auto &r : rows) {
    CHECK(r.source == "legacy");
    CHECK(r.status == "auto");
    CHECK_FALSE(r.transform.has_value());
    CHECK_FALSE(r.information.has_value());
    CHECK_FALSE(r.deviation_m.has_value());
  }
  CHECK(column_exists(tmp.path, "sensor_frames", "initial_transform"));
}

TEST_CASE("MigrationV27_UniqueKey_RejectsRawDuplicate",
          "[projectdb][migration][pose_graph]") {
  TempPath tmp("test_migration_v27_unique");
  {
    ProjectDB db(tmp.path);
  }
  exec_raw(tmp.path, "INSERT INTO pose_graph_edges (from_node_id, to_node_id, "
                     "edge_type, source, status) VALUES (1, 2, 'odometry', "
                     "'manual', 'auto');");
  sqlite3 *raw = nullptr;
  REQUIRE(sqlite3_open(tmp.path.string().c_str(), &raw) == SQLITE_OK);
  const int dup = sqlite3_exec(
      raw,
      "INSERT INTO pose_graph_edges (from_node_id, to_node_id, edge_type, "
      "source, status) VALUES (1, 2, 'odometry', 'manual', 'auto');",
      nullptr, nullptr, nullptr);
  const int reversed = sqlite3_exec(
      raw,
      "INSERT INTO pose_graph_edges (from_node_id, to_node_id, edge_type, "
      "source, status) VALUES (2, 1, 'loop_closure', 'manual', 'auto');",
      nullptr, nullptr, nullptr);
  sqlite3_close(raw);
  CHECK(dup == SQLITE_CONSTRAINT);
  CHECK(reversed == SQLITE_CONSTRAINT); // CHECK (from_node_id < to_node_id)
}

TEST_CASE("PoseGraphEdge_LookupEitherDirection_FindsCanonicalRow",
          "[projectdb][pose_graph]") {
  TempPath tmp("test_pose_graph_lookup");
  {
    ProjectDB db(tmp.path);
  }
  exec_raw(tmp.path, "INSERT INTO pose_graph_edges (from_node_id, to_node_id, "
                     "edge_type, source, status) VALUES (4, 8, 'loop_closure', "
                     "'orb', 'confirmed');");
  ProjectDB db(tmp.path, /*readOnly=*/true);
  const auto a = db.pose_graph_edge(4, 8, "loop_closure");
  const auto b = db.pose_graph_edge(8, 4, "loop_closure");
  REQUIRE(a.has_value());
  REQUIRE(b.has_value());
  CHECK(a->id == b->id);
  CHECK(a->status == "confirmed");
  CHECK(a->source == "orb");
  CHECK_FALSE(db.pose_graph_edge(4, 8, "odometry").has_value());
}
