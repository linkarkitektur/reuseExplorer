// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
// Schema v31 -> v32: sample_part_links, samples linked to specific survey
// parts. A fresh project is rolled back to v31 with raw SQL, then reopened
// read-write so migrateToV32 runs on real v31 data (plane_classes included).

#include <catch2/catch_test_macros.hpp>

#include <core/ProjectDB.hpp>
#include <core/guid.hpp>

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
  TempDB() : reusex::test_support::TempPath("test_migration_v32") {}
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

void roll_back_to_v31(const fs::path &path) {
  exec_raw(path, "DROP TABLE sample_part_links;"
                 "DELETE FROM schema_version WHERE version >= 32;");
}

// One plane_classes row (v31) on a one-point "planes" label cloud, so the
// upgrade runs over a project that already carries v31 data.
void seed_plane_class(ProjectDB &db) {
  reusex::CloudL labels;
  reusex::LabelT l;
  l.label = 1;
  labels.push_back(l);
  db.save_point_cloud("planes", labels, "segment_planes", "{}");
  reusex::core::PlaneClassRecord r;
  r.plane_label = 1;
  r.guid = reusex::core::deterministic_guid(db.project_guid(),
                                            "plane_classes/planes/1");
  r.orientation = "horizontal";
  r.role = "floor";
  r.role_source = "rules";
  REQUIRE(db.save_plane_classes("planes", {r}) == 1);
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

TEST_CASE("MigrationV32_FreshProject_HasSamplePartLinks",
          "[ProjectDB][migration][samples]") {
  TempDB tmp;
  {
    ProjectDB db(tmp.path);
    CHECK(db.schema_version() == ProjectDB::latest_schema_version());
    CHECK(ProjectDB::latest_schema_version() >= 32);
  }
  CHECK(table_exists(tmp.path, "sample_part_links"));
}

TEST_CASE("MigrationV32_FromV31_KeepsSamples_AddsTable",
          "[ProjectDB][migration][samples]") {
  TempDB tmp;
  int64_t type_id = 0;
  int64_t sample_id = 0;
  {
    ProjectDB db(tmp.path);
    type_id = make_type(db, "Fuger");
    make_part(db, "RX-001", type_id);
    sample_id = db.add_sample("PCB", "Fuge", {type_id}).id;
    seed_plane_class(db);
  }
  roll_back_to_v31(tmp.path);
  REQUIRE_FALSE(table_exists(tmp.path, "sample_part_links"));
  {
    // A read-only open of the v31 project does not migrate; samples read
    // with no part links and the type lookup still works.
    ProjectDB ro(tmp.path, /*readOnly=*/true);
    CHECK(ro.schema_version() == 31);
    CHECK(ro.plane_classes().size() == 1);
    const auto s = ro.sample(sample_id);
    REQUIRE(s);
    CHECK(s->part_codes.empty());
    CHECK(ro.samples_for_type(type_id).size() == 1);
  }
  ProjectDB db(tmp.path);
  CHECK(db.schema_version() == ProjectDB::latest_schema_version());
  CHECK(table_exists(tmp.path, "sample_part_links"));
  REQUIRE(db.plane_classes().size() == 1); // v31 data untouched
  CHECK(db.plane_classes()[0].role == "floor");
  CHECK(db.sample(sample_id)->type_ids == std::vector<int64_t>{type_id});
  db.set_sample_part_links(sample_id, {"RX-001"});
  CHECK(db.sample(sample_id)->part_codes == std::vector<std::string>{"RX-001"});
}
