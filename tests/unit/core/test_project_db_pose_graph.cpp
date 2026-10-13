// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Pose-graph edge storage in ProjectDB (schema v15, #265).
//
// Two properties are pinned:
//  1. migration: a pre-v15 fixture migrates cleanly (the table appears and
//     is empty);
//  2. upsert/status/metrics API: edges round-trip through upsert, status,
//     metric and purge calls.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <core/ProjectDB.hpp>
#include <reusex/core/pose_graph.hpp>

#include <Eigen/Geometry>

#include "../../support/temp_path.hpp"

#include <sqlite3.h>

#include <array>
#include <cmath>
#include <filesystem>
#include <limits>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

using reusex::ProjectDB;
namespace fs = std::filesystem;
using Catch::Approx;
using reusex::test_support::TempPath;

namespace {

/// A pre-v27 style edge: key, residual and weight only (designated
/// initialisers would still trip -Wmissing-field-initializers).
ProjectDB::PoseGraphEdge legacy_edge(int from, int to, std::string type,
                                     double residual, double weight) {
  ProjectDB::PoseGraphEdge e;
  e.from_node_id = from;
  e.to_node_id = to;
  e.edge_type = std::move(type);
  e.residual = residual;
  e.weight = weight;
  return e;
}

void exec(sqlite3 *db, const char *sql) {
  char *err = nullptr;
  int rc = sqlite3_exec(db, sql, nullptr, nullptr, &err);
  std::string msg = err ? err : "";
  sqlite3_free(err);
  REQUIRE(rc == SQLITE_OK);
}

/// Minimal v14 fixture: schema_version table stamped at 14, no
/// pose_graph_edges.
void buildV14Fixture(const fs::path &path) {
  sqlite3 *db = nullptr;
  REQUIRE(sqlite3_open(path.string().c_str(), &db) == SQLITE_OK);

  exec(db, "PRAGMA journal_mode=WAL;");
  exec(db, "CREATE TABLE schema_version (version INTEGER NOT NULL, "
           "applied_at TEXT, description TEXT);");
  exec(db, "INSERT INTO schema_version (version, description) VALUES "
           "(14, 'test fixture v14');");

  // Minimal tables that ProjectDB expects to find when it opens:
  exec(db, "CREATE TABLE point_clouds (id INTEGER PRIMARY KEY AUTOINCREMENT, "
           "name TEXT UNIQUE, point_type TEXT, point_count INTEGER, "
           "point_step INTEGER, width INTEGER, height INTEGER, stage TEXT, "
           "parameters TEXT);");
  exec(db,
       "CREATE TABLE point_cloud_data (id INTEGER PRIMARY KEY AUTOINCREMENT, "
       "cloud_id INTEGER, data BLOB, chunk_index INTEGER);");
  exec(db,
       "CREATE TABLE label_definitions (cloud_id INTEGER, label_id INTEGER, "
       "name TEXT, PRIMARY KEY (cloud_id, label_id));");
  exec(db, "CREATE TABLE meshes (id INTEGER PRIMARY KEY AUTOINCREMENT, "
           "name TEXT UNIQUE, format TEXT, data BLOB, vertex_count INTEGER, "
           "face_count INTEGER, stage TEXT, parameters TEXT);");
  exec(db, "CREATE TABLE sensor_frames (node_id INTEGER PRIMARY KEY, "
           "color_image BLOB, depth_image BLOB, confidence_image BLOB, "
           "transform BLOB, fx REAL, fy REAL, cx REAL, cy REAL, "
           "camera_model TEXT, timestamp REAL, local_transform BLOB);");
  exec(db,
       "CREATE TABLE pipeline_log (id INTEGER PRIMARY KEY AUTOINCREMENT, "
       "stage TEXT, started_at TEXT DEFAULT CURRENT_TIMESTAMP, "
       "finished_at TEXT, success INTEGER, error_msg TEXT, parameters TEXT);");
  exec(db, "CREATE TABLE instances (id INTEGER PRIMARY KEY AUTOINCREMENT, "
           "cloud_id INTEGER REFERENCES point_clouds(id) ON DELETE CASCADE, "
           "label_id INTEGER, point_count INTEGER, centroid_x REAL, "
           "centroid_y REAL, centroid_z REAL, bounding_box BLOB);");
  exec(db, "CREATE TABLE segmentation_images (node_id INTEGER PRIMARY KEY "
           "REFERENCES sensor_frames(node_id) ON DELETE CASCADE, "
           "label_image BLOB NOT NULL);");
  exec(db,
       "CREATE TABLE panoramic_images (id INTEGER PRIMARY KEY AUTOINCREMENT, "
       "filename TEXT, image BLOB, node_id INTEGER, pose BLOB, "
       "timestamp REAL);");
  exec(
      db,
      "CREATE TABLE building_components (id INTEGER PRIMARY KEY AUTOINCREMENT, "
      "name TEXT UNIQUE, type TEXT, parent_id INTEGER, "
      "vertex_data BLOB, confidence REAL, guid TEXT, metadata TEXT, notes "
      "TEXT);");
  exec(db,
       "CREATE TABLE material_passports (id INTEGER PRIMARY KEY AUTOINCREMENT, "
       "document_guid TEXT UNIQUE NOT NULL, created_at TEXT, "
       "schema_version INTEGER DEFAULT 1);");
  exec(db,
       "CREATE TABLE instance_materials (id INTEGER PRIMARY KEY AUTOINCREMENT, "
       "cloud_name TEXT NOT NULL, label_id INTEGER NOT NULL, "
       "material_guid TEXT NOT NULL);");
  exec(db,
       "CREATE TABLE gaussian_splats (id INTEGER PRIMARY KEY AUTOINCREMENT, "
       "name TEXT UNIQUE, format TEXT, gaussian_count INTEGER, "
       "sh_degree INTEGER, byte_size INTEGER, created_at TEXT, "
       "stage TEXT, parameters TEXT);");
  exec(
      db,
      "CREATE TABLE gaussian_splat_data (id INTEGER PRIMARY KEY AUTOINCREMENT, "
      "splat_id INTEGER, chunk_index INTEGER, data BLOB);");
  exec(db,
       "CREATE TABLE mesh_texture_data (id INTEGER PRIMARY KEY AUTOINCREMENT, "
       "mesh_id INTEGER, tex_name TEXT, format TEXT, width INTEGER, "
       "height INTEGER, image_data BLOB);");
  exec(db, "CREATE TABLE projects (id INTEGER PRIMARY KEY AUTOINCREMENT, "
           "name TEXT, address TEXT, year_of_construction INTEGER, "
           "survey_date TEXT, organisation TEXT, description TEXT);");
  exec(db, "CREATE TABLE property_definitions (id INTEGER PRIMARY KEY "
           "AUTOINCREMENT, "
           "document_guid TEXT NOT NULL, property_name TEXT NOT NULL, "
           "property_type TEXT, unit TEXT, description TEXT);");
  exec(db,
       "CREATE TABLE property_values (id INTEGER PRIMARY KEY AUTOINCREMENT, "
       "document_guid TEXT NOT NULL, property_name TEXT NOT NULL, "
       "value TEXT);");
  exec(db,
       "CREATE TABLE transaction_log (id INTEGER PRIMARY KEY AUTOINCREMENT, "
       "document_guid TEXT NOT NULL, transaction_type TEXT NOT NULL, "
       "transaction_date TEXT, quantity REAL, unit TEXT, notes TEXT, "
       "document_reference TEXT);");
  exec(db, "CREATE TABLE glass_confidence_images (node_id INTEGER PRIMARY KEY "
           "REFERENCES sensor_frames(node_id) ON DELETE CASCADE, "
           "confidence_image BLOB NOT NULL);");

  sqlite3_close(db);
}

} // namespace

TEST_CASE("pose_graph migration v14->v15: table created, initially empty",
          "[projectdb][migration][pose_graph]") {
  TempPath tmp("test_pose_graph_migration");
  buildV14Fixture(tmp.path);

  ProjectDB db(tmp.path); // triggers migration
  CHECK(db.list_pose_graph_edges().empty());
  CHECK_FALSE(db.has_pose_graph());
}

namespace {

namespace pg = reusex::core::pose_graph;

std::array<double, 16> pose_rt(double yaw, double x, double y, double z) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.topLeftCorner<3, 3>() =
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  T(0, 3) = x;
  T(1, 3) = y;
  T(2, 3) = z;
  return pg::row_major_from_matrix(T);
}

ProjectDB::PoseGraphEdge measured_edge(int from, int to, std::string type) {
  ProjectDB::PoseGraphEdge e;
  e.from_node_id = from;
  e.to_node_id = to;
  e.edge_type = std::move(type);
  e.transform = pose_rt(0.3, 1.0, -2.0, 0.5);
  e.information =
      pg::row_major_from_information(pg::information_from_sigmas(0.05, 0.10));
  e.source = "orb";
  e.status = "auto";
  e.seed_disagreement_m = 0.25;
  e.seed_disagreement_deg = 3.0;
  return e;
}

} // namespace

TEST_CASE("PoseGraphUpsert_NewEdge_RoundTripsEveryField",
          "[projectdb][pose_graph]") {
  TempPath tmp("test_pose_graph_upsert");
  ProjectDB db(tmp.path);
  const auto in = measured_edge(3, 7, "loop_closure");
  REQUIRE(db.upsert_pose_graph_edge(in));

  const auto out = db.list_pose_graph_edges();
  REQUIRE(out.size() == 1);
  const auto &e = out[0];
  CHECK(e.id > 0);
  CHECK(e.from_node_id == 3);
  CHECK(e.to_node_id == 7);
  CHECK(e.edge_type == "loop_closure");
  CHECK(std::isnan(e.residual));    // never solved
  CHECK(e.weight == Approx(100.0)); // 1 / 0.10^2, from information
  REQUIRE(e.transform.has_value());
  CHECK(*e.transform == *in.transform);
  REQUIRE(e.information.has_value());
  CHECK(*e.information == *in.information);
  CHECK(e.source == "orb");
  CHECK(e.status == "auto");
  CHECK(e.created_at.size() == 20); // 2026-10-13T12:34:56Z
  CHECK(e.created_at.back() == 'Z');
  CHECK(e.seed_disagreement_m == Approx(0.25));
  CHECK_FALSE(e.deviation_m.has_value());

  // A second upsert of the same key with an empty created_at (the normal
  // caller path) must keep the first write's created_at, not reset it.
  auto again = measured_edge(3, 7, "loop_closure");
  REQUIRE(again.created_at.empty());
  REQUIRE(db.upsert_pose_graph_edge(again));
  const auto out2 = db.list_pose_graph_edges();
  REQUIRE(out2.size() == 1);
  CHECK(out2[0].created_at == e.created_at);
}

TEST_CASE("PoseGraphUpsert_ReversedDirection_StoresCanonicalInverse",
          "[projectdb][pose_graph]") {
  TempPath tmp("test_pose_graph_canonical");
  ProjectDB db(tmp.path);
  auto in = measured_edge(9, 4, "loop_closure");
  pg::Matrix6d info = pg::Matrix6d::Identity() * 50.0;
  info(0, 4) = info(4, 0) = 3.0; // a coupled, non-diagonal information
  in.information = pg::row_major_from_information(info);
  REQUIRE(db.upsert_pose_graph_edge(in));

  const auto e = db.pose_graph_edge(4, 9, "loop_closure");
  REQUIRE(e.has_value());
  CHECK(e->from_node_id == 4);
  CHECK(e->to_node_id == 9);
  const Eigen::Matrix4d T = pg::matrix_from_row_major(*in.transform);
  CHECK((pg::matrix_from_row_major(*e->transform) - T.inverse()).norm() <
        1e-12);
  const pg::Matrix6d expected = pg::reverse_edge_information(info, T);
  CHECK((pg::information_from_row_major(*e->information) - expected).norm() <
        1e-9);

  // The same pair, given forwards, replaces it rather than adding a row.
  auto fwd = measured_edge(4, 9, "loop_closure");
  REQUIRE(db.upsert_pose_graph_edge(fwd));
  REQUIRE(db.list_pose_graph_edges().size() == 1);
  CHECK(*db.pose_graph_edge(4, 9, "loop_closure")->transform == *fwd.transform);
}

TEST_CASE("PoseGraphUpsert_OverTombstone_IsRefusedUnlessOverwritten",
          "[projectdb][pose_graph]") {
  TempPath tmp("test_pose_graph_tombstone");
  ProjectDB db(tmp.path);
  REQUIRE(db.upsert_pose_graph_edge(measured_edge(1, 5, "loop_closure")));
  REQUIRE(db.set_pose_graph_edge_status(5, 1, "loop_closure", "rejected"));

  auto again = measured_edge(1, 5, "loop_closure");
  again.transform = pose_rt(0.0, 9.0, 9.0, 9.0);
  CHECK_FALSE(db.upsert_pose_graph_edge(again));
  auto row = db.pose_graph_edge(1, 5, "loop_closure");
  CHECK(row->status == "rejected");
  CHECK((*row->transform)[3] != 9.0); // untouched

  again.status = "confirmed";
  CHECK(db.upsert_pose_graph_edge(again, /*overwrite_rejected=*/true));
  row = db.pose_graph_edge(1, 5, "loop_closure");
  CHECK(row->status == "confirmed");
  CHECK((*row->transform)[3] == 9.0);
}

TEST_CASE("PoseGraphStatus_UnknownEdgeOrStatus_IsReported",
          "[projectdb][pose_graph]") {
  TempPath tmp("test_pose_graph_status");
  ProjectDB db(tmp.path);
  CHECK_FALSE(db.set_pose_graph_edge_status(1, 2, "odometry", "rejected"));
  REQUIRE(db.upsert_pose_graph_edge(measured_edge(1, 2, "odometry")));
  CHECK_THROWS_AS(db.set_pose_graph_edge_status(1, 2, "odometry", "deleted"),
                  std::invalid_argument);
  CHECK(db.set_pose_graph_edge_status(1, 2, "odometry", "confirmed"));
  CHECK(db.pose_graph_edge(1, 2, "odometry")->status == "confirmed");
}

TEST_CASE("PoseGraphMetrics_Update_WritesAndClears",
          "[projectdb][pose_graph]") {
  TempPath tmp("test_pose_graph_metrics");
  ProjectDB db(tmp.path);
  REQUIRE(db.upsert_pose_graph_edge(measured_edge(1, 2, "odometry")));
  REQUIRE(db.upsert_pose_graph_edge(measured_edge(2, 6, "loop_closure")));

  std::vector<ProjectDB::PoseGraphEdgeMetrics> m(2);
  m[0] = {1, 2, "odometry", 0.5, 0.01, 0.2, 0.0, 0.0};
  m[1] = {6, 2, "loop_closure", std::nullopt, 0.4, 5.0, 0.3, 4.0};
  CHECK(db.update_pose_graph_edge_metrics(m) == 2);

  const auto a = db.pose_graph_edge(1, 2, "odometry");
  CHECK(a->residual == Approx(0.5));
  CHECK(a->deviation_m == Approx(0.01));
  const auto b = db.pose_graph_edge(2, 6, "loop_closure");
  CHECK(std::isnan(b->residual));
  CHECK(b->deviation_deg == Approx(5.0));
  CHECK(b->seed_disagreement_m == Approx(0.3));
  CHECK(b->status == "auto"); // metrics never touch status
}

TEST_CASE("PoseGraphPurgeBySource_KeepsTombstonesAndOtherSources",
          "[projectdb][pose_graph]") {
  TempPath tmp("test_pose_graph_purge");
  ProjectDB db(tmp.path);
  auto a = measured_edge(1, 2, "odometry");
  a.source = "odometry_init";
  auto b = measured_edge(2, 3, "odometry");
  b.source = "odometry_init";
  auto c = measured_edge(3, 4, "odometry");
  c.source = "rtabmap";
  REQUIRE(db.upsert_pose_graph_edge(a));
  REQUIRE(db.upsert_pose_graph_edge(b));
  REQUIRE(db.upsert_pose_graph_edge(c));
  REQUIRE(db.set_pose_graph_edge_status(2, 3, "odometry", "rejected"));

  CHECK(db.delete_pose_graph_edges_from_source("odometry_init") == 1);
  const auto rows = db.list_pose_graph_edges();
  REQUIRE(rows.size() == 2);
  CHECK(db.pose_graph_edge(2, 3, "odometry")->status == "rejected");
  CHECK(db.pose_graph_edge(3, 4, "odometry").has_value());
}

TEST_CASE("PoseGraphUpsert_InvalidInput_Throws", "[projectdb][pose_graph]") {
  TempPath tmp("test_pose_graph_invalid");
  ProjectDB db(tmp.path);
  auto self = measured_edge(4, 4, "loop_closure");
  CHECK_THROWS_AS(db.upsert_pose_graph_edge(self), std::invalid_argument);
  auto status = measured_edge(1, 2, "loop_closure");
  status.status = "maybe";
  CHECK_THROWS_AS(db.upsert_pose_graph_edge(status), std::invalid_argument);
  auto broken = measured_edge(1, 2, "loop_closure");
  broken.transform = std::array<double, 16>{}; // all zero: not rigid
  CHECK_THROWS_AS(db.upsert_pose_graph_edge(broken), std::invalid_argument);
  auto asym = measured_edge(1, 2, "loop_closure");
  (*asym.information)[1] = 7.0;
  CHECK_THROWS_AS(db.upsert_pose_graph_edge(asym), std::invalid_argument);
  auto no_source = measured_edge(1, 2, "loop_closure");
  no_source.source.clear();
  CHECK_THROWS_AS(db.upsert_pose_graph_edge(no_source), std::invalid_argument);
  CHECK(db.list_pose_graph_edges().empty());
}

TEST_CASE("PoseGraphUpsert_ReadOnlyProject_Throws", "[projectdb][pose_graph]") {
  TempPath tmp("test_pose_graph_ro");
  {
    ProjectDB db(tmp.path);
  }
  ProjectDB ro(tmp.path, /*readOnly=*/true);
  CHECK_THROWS_AS(ro.upsert_pose_graph_edge(measured_edge(1, 2, "odometry")),
                  std::runtime_error);
}

TEST_CASE("PoseGraphUpsert_WithoutMeasurement_KeepsStoredMeasurement",
          "[projectdb][pose_graph]") {
  // A measurement-less upsert onto an existing row (the Qt EdgeEditor's
  // "add", ruxd's POST /posegraph/edges) is a status change: the stored
  // measurement, its metrics and its source survive (final review I1).
  TempPath tmp("test_pose_graph_upsert_keep");
  ProjectDB db(tmp.path);
  auto stored = measured_edge(2, 6, "odometry");
  stored.source = "rtabmap";
  REQUIRE(db.upsert_pose_graph_edge(stored));
  ProjectDB::PoseGraphEdgeMetrics m;
  m.from_node_id = 2;
  m.to_node_id = 6;
  m.edge_type = "odometry";
  m.residual = 0.7;
  m.deviation_m = 0.02;
  m.deviation_deg = 0.4;
  m.seed_disagreement_m = 0.25;
  m.seed_disagreement_deg = 3.0;
  REQUIRE(db.update_pose_graph_edge_metrics(std::span(&m, 1)) == 1);
  const auto before = *db.pose_graph_edge(2, 6, "odometry");

  ProjectDB::PoseGraphEdge confirm = legacy_edge(6, 2, "odometry", 9.0, 1.0);
  confirm.source = "manual";
  confirm.status = "confirmed";
  REQUIRE(db.upsert_pose_graph_edge(confirm, /*overwrite_rejected=*/true));
  auto row = db.pose_graph_edge(2, 6, "odometry");
  REQUIRE(row.has_value());
  CHECK(row->status == "confirmed");
  CHECK(row->source == "rtabmap");
  REQUIRE(row->transform.has_value());
  CHECK(*row->transform == *before.transform);
  REQUIRE(row->information.has_value());
  CHECK(*row->information == *before.information);
  CHECK(row->weight == Approx(before.weight));
  CHECK(row->residual == Approx(0.7));
  CHECK(row->deviation_m == Approx(0.02));
  CHECK(row->seed_disagreement_m == Approx(0.25));

  // "auto" without a measurement never downgrades a confirmed row ...
  ProjectDB::PoseGraphEdge plain = legacy_edge(2, 6, "odometry", 0.0, 0.0);
  REQUIRE(db.upsert_pose_graph_edge(plain, /*overwrite_rejected=*/true));
  CHECK(db.pose_graph_edge(2, 6, "odometry")->status == "confirmed");
  // ... but un-rejects a tombstone when the caller asks for that.
  REQUIRE(db.set_pose_graph_edge_status(2, 6, "odometry", "rejected"));
  CHECK_FALSE(db.upsert_pose_graph_edge(plain));
  CHECK(db.pose_graph_edge(2, 6, "odometry")->status == "rejected");
  REQUIRE(db.upsert_pose_graph_edge(plain, /*overwrite_rejected=*/true));
  row = db.pose_graph_edge(2, 6, "odometry");
  CHECK(row->status == "auto");
  CHECK(*row->transform == *before.transform);

  // A new key without a measurement is still inserted as given.
  REQUIRE(db.upsert_pose_graph_edge(legacy_edge(1, 9, "loop_closure", 0, 4)));
  row = db.pose_graph_edge(1, 9, "loop_closure");
  REQUIRE(row.has_value());
  CHECK(row->source == "manual");
  CHECK_FALSE(row->transform.has_value());

  // An upsert WITH a measurement still replaces everything, as before.
  auto replace = measured_edge(2, 6, "odometry");
  replace.transform = pose_rt(0.0, 4.0, 0.0, 0.0);
  replace.status = "auto";
  REQUIRE(db.upsert_pose_graph_edge(replace));
  row = db.pose_graph_edge(2, 6, "odometry");
  CHECK(row->source == "orb");
  CHECK(row->status == "auto");
  CHECK((*row->transform)[3] == 4.0);
}

#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
TEST_CASE("PoseGraphDeprecatedAdd_OverStoredEdge_KeepsMeasurementAndConfirmed",
          "[projectdb][pose_graph]") {
  TempPath tmp("test_pose_graph_deprecated_add_keep");
  ProjectDB db(tmp.path);
  auto stored = measured_edge(4, 11, "loop_closure");
  stored.source = "rtabmap";
  REQUIRE(db.upsert_pose_graph_edge(stored));
  REQUIRE(db.set_pose_graph_edge_status(4, 11, "loop_closure", "confirmed"));

  // ruxd's add: no measurement, default status "auto".
  const int id =
      db.add_pose_graph_edge(legacy_edge(11, 4, "loop_closure", 0.1, 1.0));
  CHECK(id > 0);
  auto row = db.pose_graph_edge(4, 11, "loop_closure");
  REQUIRE(row.has_value());
  CHECK(row->id == id);
  CHECK(row->status == "confirmed");
  CHECK(row->source == "rtabmap");
  REQUIRE(row->transform.has_value());
  CHECK(*row->transform == *stored.transform);

  // Even an add that carries a measurement does not downgrade confirmed.
  auto measured = measured_edge(4, 11, "loop_closure");
  measured.transform = pose_rt(0.0, 3.0, 0.0, 0.0);
  CHECK(db.add_pose_graph_edge(measured) == id);
  row = db.pose_graph_edge(4, 11, "loop_closure");
  CHECK(row->status == "confirmed");
  CHECK((*row->transform)[3] == 3.0);
}

TEST_CASE("PoseGraphDeprecatedWrappers_KeepRuxdWorking",
          "[projectdb][pose_graph]") {
  TempPath tmp("test_pose_graph_deprecated");
  ProjectDB db(tmp.path);
  const auto e = legacy_edge(8, 3, "loop_closure", 0.4, 25.0);
  const int id = db.add_pose_graph_edge(e);
  CHECK(id > 0);
  const auto row = db.pose_graph_edge(3, 8, "loop_closure");
  REQUIRE(row.has_value());
  CHECK(row->id == id);
  CHECK(row->source == "manual");
  // "delete" is now a reject: the row stays, as a tombstone.
  CHECK(db.delete_pose_graph_edges(8, 3, "loop_closure") == 1);
  CHECK(db.pose_graph_edge(3, 8, "loop_closure")->status == "rejected");
  // ... and add over it un-rejects (the old add always wrote a row).
  CHECK(db.add_pose_graph_edge(e) == id);
  CHECK(db.pose_graph_edge(3, 8, "loop_closure")->status == "auto");
  // Empty type rejects every type on the pair.
  REQUIRE(db.upsert_pose_graph_edge(legacy_edge(3, 8, "odometry", 0.1, 1.0)));
  CHECK(db.delete_pose_graph_edges(3, 8) == 2);
}
#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif
