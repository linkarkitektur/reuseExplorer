// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// survey_parts.dismissed (schema v36, reusex-co6): a part marked as an error
// stays listed but counts toward neither its type's quantity nor room
// coverage nor its type's miljøstatus, reaches no delivered output (CSV
// export, PDF report data), survives sync_survey, and can be restored.

#include <catch2/catch_test_macros.hpp>

#include <core/ProjectDB.hpp>
#include <core/report_generator.hpp>
#include <core/resource_export.hpp>
#include <core/resource_templates.hpp>
#include <core/resources.hpp>
#include <core/survey_service.hpp>

#include "../../support/temp_path.hpp"

#include <nlohmann/json.hpp>
#include <pcl/point_types.h>
#include <sqlite3.h>

#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <iterator>
#include <stdexcept>
#include <string>

using reusex::ProjectDB;
using reusex::test_support::TempDir;
using reusex::test_support::TempPath;
using namespace reusex::core;
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

int64_t add_type(ProjectDB &db) {
  ProjectDB::SurveyTypeRecord t;
  t.name = "Betonsøjler, bærende";
  t.mass_t = 58.0;
  return db.add_survey_type(t).id;
}

void add_part(ProjectDB &db, const std::string &code, int64_t type_id,
              double quantity, bool dismissed = false) {
  ProjectDB::SurveyPartRecord p;
  p.code = code;
  p.type_id = type_id;
  p.quantity = quantity;
  p.dismissed = dismissed;
  db.add_survey_part(p);
}

reusex::CloudL label_cloud(std::initializer_list<std::uint32_t> labels) {
  reusex::CloudL c;
  for (auto l : labels) {
    pcl::Label p;
    p.label = l;
    c.push_back(p);
  }
  return c;
}

/// Instances 1,2 (class 3), 3 (class 5); instance 1 lies in room 4, 2 and 3
/// in room 6.
void seed_scan(ProjectDB &db) {
  db.save_point_cloud("instances", label_cloud({1, 1, 2, 2, 3, 0}), "test",
                      "{}");
  db.save_instances("instances",
                    {{1, "g1", 3, 2}, {2, "g2", 3, 2}, {3, "g3", 5, 1}});
  db.save_point_cloud("labels", label_cloud({3, 3, 3, 3, 5, 0}), "test", "{}");
  db.save_label_definitions("labels", {{3, "window"}, {5, "door"}});
  db.save_point_cloud("rooms", label_cloud({4, 4, 6, 6, 6, 0}), "test", "{}");
  db.save_label_definitions("rooms", {{4, "Office Zone"}});
}

} // namespace

TEST_CASE("SurveyPartDismissed_MigrationFromV35_DefaultsToFalse",
          "[projectdb][migration][survey]") {
  TempPath tmp("test_migration_v36_dismissed");
  {
    ProjectDB db(tmp.path);
    CHECK(ProjectDB::latest_schema_version() >= 36);
    add_part(db, "RX-001", add_type(db), 4.0);
  }
  exec_raw(tmp.path, R"(
    ALTER TABLE survey_parts DROP COLUMN dismissed;
    DELETE FROM schema_version WHERE version >= 36;
  )");
  {
    // Read-only: no migration; the part reads as not dismissed and a
    // dismissal is refused rather than silently dropped.
    ProjectDB ro(tmp.path, /*readOnly=*/true);
    CHECK(ro.schema_version() == 35);
    REQUIRE(ro.survey_part("RX-001"));
    CHECK_FALSE(ro.survey_part("RX-001")->dismissed);
  }
  {
    ProjectDB db(tmp.path);
    CHECK(db.schema_version() == ProjectDB::latest_schema_version());
    const auto p = db.survey_part("RX-001");
    REQUIRE(p);
    CHECK_FALSE(p->dismissed);
    CHECK(p->quantity == 4.0); // existing row untouched
    ProjectDB::SurveyPartPatch patch;
    patch.dismissed = true;
    CHECK(db.update_survey_part("RX-001", patch).dismissed);
  }
}

TEST_CASE("SurveyPartDismissed_PatchRoundTrip", "[projectdb][survey]") {
  TempPath tmp("test_survey_part_dismissed_patch");
  {
    ProjectDB db(tmp.path);
    const auto type = add_type(db);
    add_part(db, "RX-001", type, 2.0);
    add_part(db, "RX-002", type, 3.0, /*dismissed=*/true);
    CHECK_FALSE(db.survey_part("RX-001")->dismissed);
    CHECK(db.survey_part("RX-002")->dismissed);

    ProjectDB::SurveyPartPatch dismiss;
    dismiss.dismissed = true;
    const auto out = db.update_survey_part("RX-001", dismiss);
    CHECK(out.dismissed);
    CHECK(out.quantity == 2.0);

    // A patch that does not name dismissed leaves it alone.
    ProjectDB::SurveyPartPatch note;
    note.note = "revnet";
    CHECK(db.update_survey_part("RX-001", note).dismissed);
  }
  {
    ProjectDB db(tmp.path);
    CHECK(db.survey_part("RX-001")->dismissed); // persisted
    ProjectDB::SurveyPartPatch restore;
    restore.dismissed = false;
    CHECK_FALSE(db.update_survey_part("RX-001", restore).dismissed);
    // Still listed either way.
    CHECK(db.survey_parts().size() == 2);
  }
}

TEST_CASE("SurveyPartDismissed_ExcludedFromQuantityAndTotals",
          "[survey][service]") {
  TempPath tmp("test_survey_part_dismissed_totals");
  ProjectDB db(tmp.path);
  const auto type = add_type(db);
  add_part(db, "RX-001", type, 18.0);
  add_part(db, "RX-002", type, 6.0);
  add_part(db, "RX-003", type, 100.0, /*dismissed=*/true);
  CHECK(type_quantity(db, type) == 24.0);

  // Redistribution scales only the counted parts; the dismissed one keeps
  // its quantity but is still returned.
  const auto parts = set_type_quantity(db, type, 30.0);
  REQUIRE(parts.size() == 3);
  CHECK(db.survey_part("RX-001")->quantity == 22.5);
  CHECK(db.survey_part("RX-002")->quantity == 7.5);
  CHECK(db.survey_part("RX-003")->quantity == 100.0);
  CHECK(type_quantity(db, type) == 30.0);

  // The PDF report's survey rows sum the same way.
  ProjectDB::SurveyTypePatch approve;
  approve.review_status = ReviewStatus::approved;
  db.update_survey_type(type, approve);
  const auto rows = report_survey_rows(db);
  REQUIRE(rows.size() == 1);
  CHECK(rows[0].quantity == 30.0);

  // A type whose parts are all dismissed has nothing to distribute over.
  const auto only_dismissed = add_type(db);
  add_part(db, "RX-004", only_dismissed, 5.0, /*dismissed=*/true);
  CHECK(type_quantity(db, only_dismissed) == 0.0);
  CHECK_THROWS_AS(set_type_quantity(db, only_dismissed, 1.0),
                  std::invalid_argument);
}

TEST_CASE("SurveyPartDismissed_RoomsWithoutParts", "[survey][service]") {
  TempPath tmp("test_survey_part_dismissed_rooms");
  ProjectDB db(tmp.path);
  CHECK(rooms_without_parts(db).empty()); // no rooms cloud
  seed_scan(db);
  sync_survey(db);
  CHECK(rooms_without_parts(db).empty());
  ProjectDB::SurveyPartPatch dismiss;
  dismiss.dismissed = true;
  db.update_survey_part("RX-001", dismiss); // the only part in room 4
  CHECK(rooms_without_parts(db) == std::vector<std::string>{"Office Zone"});
  db.update_survey_part("RX-002", dismiss); // room 6 still has RX-003
  CHECK(rooms_without_parts(db) == std::vector<std::string>{"Office Zone"});
  db.update_survey_part("RX-003", dismiss);
  CHECK(rooms_without_parts(db) ==
        std::vector<std::string>{"Office Zone", "Rum 6"});
}

TEST_CASE("SurveyPartDismissed_SyncLeavesItDismissed", "[survey][sync]") {
  TempPath tmp("test_survey_part_dismissed_sync");
  ProjectDB db(tmp.path);
  seed_scan(db);
  REQUIRE(sync_survey(db).parts_created == 3);
  ProjectDB::SurveyPartPatch dismiss;
  dismiss.dismissed = true;
  db.update_survey_part("RX-002", dismiss);

  const auto r = sync_survey(db);
  CHECK(r.parts_created == 0);
  CHECK(r.parts_existing == 3);
  CHECK(r.parts_dismissed ==
        0); // that counter is for deleted parts' tombstones
  REQUIRE(db.survey_parts().size() == 3);
  CHECK(db.survey_part("RX-002")->dismissed);
  CHECK(db.survey_part("RX-002")->instance_id == 2u);
  CHECK_FALSE(db.survey_part("RX-001")->dismissed);
}

TEST_CASE("SurveyPartDismissed_ListedWithFlag_ButNotExported",
          "[survey][resources][csv][pdf]") {
  TempPath tmp("test_survey_part_dismissed_export");
  ProjectDB db(tmp.path);
  const auto type = add_type(db);
  add_part(db, "RX-001", type, 1.0);
  add_part(db, "RX-002", type, 2.0, /*dismissed=*/true);

  const auto listed = list_resources(db);
  REQUIRE(listed.size() == 2); // the GUI lists it for "Gendan"
  CHECK_FALSE(listed[0].dismissed);
  CHECK(listed[1].dismissed);
  CHECK(resource(db, "RX-002").dismissed);

  TemplateInput in;
  in.name = "Kort";
  in.members = nlohmann::json::parse(R"([{"key":"sys:quantity"}])");
  in.csv = nlohmann::json::parse(R"({"encoding":"utf-8"})");
  const auto id = create_template(db, in).record.id;
  CHECK(export_resources_csv(db, id) == "Kode;Mængde\r\nRX-001;1\r\n");
  const auto section = resource_report_section(db, id);
  REQUIRE(section.tables.size() == 1);
  REQUIRE(section.tables[0].rows.size() == 1);
  CHECK(section.tables[0].rows[0][0].find("RX-001") != std::string::npos);
}

TEST_CASE("SurveyPartDismissed_ReportData_LeavesOutItsPassport",
          "[survey][report][dump]") {
  TempPath tmp("test_survey_part_dismissed_report");
  ProjectDB db(tmp.path);
  const auto type = add_type(db);
  add_part(db, "RX-001", type, 1.0);
  add_part(db, "RX-002", type, 2.0, /*dismissed=*/true);
  std::string kept, dropped;
  {
    ProjectDB::Transaction tx(db);
    kept = db.ensure_resource_passport("RX-001");
    dropped = db.ensure_resource_passport("RX-002");
    tx.commit();
  }
  REQUIRE(kept != dropped);
  ProjectDB::SurveyTypePatch approve;
  approve.review_status = ReviewStatus::approved;
  db.update_survey_type(type, approve);

  TempDir out("test_survey_part_dismissed_report_dir");
  reusex::dump_report_data(db, {}, out.path);
  std::ifstream f(out.path / "data.json");
  const auto data =
      nlohmann::json::parse(std::string(std::istreambuf_iterator<char>(f), {}));
  std::vector<std::string> guids;
  for (const auto &m : data.at("materials"))
    guids.push_back(m.at("guid").get<std::string>());
  CHECK(guids == std::vector<std::string>{kept});
  const auto &rows = data.at("survey").at("rows");
  REQUIRE(rows.size() == 1);
  CHECK(rows[0].at("quantity").get<std::string>().rfind("1 ", 0) == 0); // not 3

  // Restored, the part is delivered again.
  ProjectDB::SurveyPartPatch restore;
  restore.dismissed = false;
  db.update_survey_part("RX-002", restore);
  TempDir again("test_survey_part_dismissed_report_dir");
  reusex::dump_report_data(db, {}, again.path);
  std::ifstream g(again.path / "data.json");
  const auto data2 =
      nlohmann::json::parse(std::string(std::istreambuf_iterator<char>(g), {}));
  CHECK(data2.at("materials").size() == 2);
}

TEST_CASE("SurveyPartDismissed_PartLinkedSample_BearsOnNothing",
          "[survey][service][sample_part_links]") {
  TempPath tmp("test_survey_part_dismissed_samples");
  ProjectDB db(tmp.path);
  const auto type = add_type(db);
  add_part(db, "RX-001", type, 1.0);
  db.add_sample("Bly i maling", "", {}, SampleStage::udtaget, {"RX-001"});
  CHECK(db.samples_for_type(type).size() == 1);
  CHECK(environment_status_of(db, type) == EnvironmentStatus::afventer);

  ProjectDB::SurveyPartPatch dismiss;
  dismiss.dismissed = true;
  db.update_survey_part("RX-001", dismiss);
  CHECK(db.samples_for_type(type).empty());
  CHECK(environment_status_of(db, type) == EnvironmentStatus::ren_screening);
  CHECK(environment_statuses(db).at(type) == EnvironmentStatus::ren_screening);
  // The link itself is kept, so restoring brings the sample back.
  CHECK(db.samples().at(0).part_codes == std::vector<std::string>{"RX-001"});
  dismiss.dismissed = false;
  db.update_survey_part("RX-001", dismiss);
  CHECK(environment_statuses(db).at(type) == EnvironmentStatus::afventer);
}
