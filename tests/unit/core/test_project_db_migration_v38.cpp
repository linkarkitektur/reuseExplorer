// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Schema v37 -> v38 (reusex-ufcm.8): report_assets. Two branches each
// shipped a v37 before they were merged — survey mass (kept as v37) and
// report_assets (renumbered to v38). A project migrated by either one alone
// is stamped v37 but lacks the other half; reopening must leave both.

#include <catch2/catch_test_macros.hpp>

#include <reusex/core/ProjectDB.hpp>

#include "../../support/temp_path.hpp"

#include <sqlite3.h>

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

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

const std::vector<std::uint8_t> kPng = {0x89, 'P', 'N', 'G', 1, 2, 3};

void check_both_halves(const fs::path &path) {
  ProjectDB db(path);
  CHECK(db.schema_version() == ProjectDB::latest_schema_version());
  CHECK(db.has_survey_mass_columns());
  db.set_report_asset("cover", kPng, "image/png");
  const auto a = db.report_asset("cover");
  REQUIRE(a);
  CHECK(a->first == kPng);
  CHECK(table_exists(path, "report_assets"));
  CHECK(table_exists(path, "buildups"));
  CHECK(table_exists(path, "value_provenance"));
}

} // namespace

TEST_CASE("MigrationV38_FreshProject_HasReportAssets",
          "[projectdb][migration][report]") {
  TempPath tmp("test_migration_v38_fresh");
  CHECK(ProjectDB::latest_schema_version() >= 38);
  check_both_halves(tmp.path);
}

TEST_CASE("MigrationV38_SurveyMassOnlyV37_AddsReportAssets",
          "[projectdb][migration][report]") {
  TempPath tmp("test_migration_v38_mass_v37");
  {
    ProjectDB db(tmp.path);
  }
  // The survey-mass branch's v37: no report_assets.
  exec_raw(tmp.path, R"(
    DROP TABLE report_assets;
    DELETE FROM schema_version WHERE version >= 38;
  )");
  {
    ProjectDB ro(tmp.path, /*readOnly=*/true);
    CHECK(ro.schema_version() == 37);
    CHECK_FALSE(ro.report_asset("cover"));
  }
  check_both_halves(tmp.path);
}

TEST_CASE("MigrationV38_ReportAssetsOnlyV37_AddsSurveyMass",
          "[projectdb][migration][report]") {
  TempPath tmp("test_migration_v38_report_v37");
  {
    ProjectDB db(tmp.path);
  }
  // The report-preview branch's v37: report_assets, no survey mass.
  exec_raw(tmp.path, R"(
    DROP INDEX idx_survey_parts_element;
    ALTER TABLE survey_parts DROP COLUMN element_guid;
    ALTER TABLE survey_parts DROP COLUMN geom_quantity;
    ALTER TABLE survey_parts DROP COLUMN geom_unit;
    ALTER TABLE survey_parts DROP COLUMN geom_source;
    ALTER TABLE survey_types DROP COLUMN buildup_id;
    ALTER TABLE survey_types DROP COLUMN element_category;
    DROP TABLE buildups;
    DROP TABLE value_provenance;
    DELETE FROM schema_version WHERE version >= 37;
    INSERT INTO schema_version (version, description)
      VALUES (37, 'report_assets: uploaded report images (cover)');
  )");
  {
    ProjectDB ro(tmp.path, /*readOnly=*/true);
    CHECK(ro.schema_version() == 37);
    CHECK_FALSE(ro.has_survey_mass_columns());
  }
  check_both_halves(tmp.path);
}
