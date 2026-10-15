// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Schema v28 -> v29 (decision D8, reusex-z1v.2.1): point label = API class + 1.
// Before v29 a semantic label cloud stored the API class id unchanged, so API
// class 0 collided with kUnlabeled. The migration shifts every stored
// semantic cloud that lacks `parameters.label_encoding = "api_plus_one"` by
// +1 (0 stays unlabeled), shifts its label_definitions keys, stamps the key,
// and follows the semantic class numbers that reference it (instances rows,
// "SM<cls>-" instance definitions, survey types). Structural clouds (planes,
// rooms, instances) keep their ids. A fresh project is written with the
// pre-v29 layout through the public API, its v29 row deleted, then reopened.

#include <catch2/catch_test_macros.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/label_semantics.hpp>
#include <reusex/core/survey.hpp>
#include <reusex/types/point_types.hpp>

#include "../../support/temp_path.hpp"

#include <nlohmann/json.hpp>
#include <sqlite3.h>

#include <algorithm>
#include <filesystem>
#include <map>
#include <string>
#include <vector>

using reusex::CloudL;
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

void roll_back_to_v28(const fs::path &path) {
  exec_raw(path, "DELETE FROM schema_version WHERE version >= 29;");
}

CloudL label_cloud(const std::vector<uint32_t> &values) {
  CloudL c;
  for (auto v : values) {
    pcl::Label l;
    l.label = v;
    c.push_back(l);
  }
  return c;
}

std::vector<uint32_t> values_of(const ProjectDB &db, const std::string &name) {
  std::vector<uint32_t> out;
  const auto cloud = db.point_cloud_label(name); // keep the cloud alive
  for (const auto &l : cloud->points)
    out.push_back(l.label);
  return out;
}

std::string encoding_of(const ProjectDB &db, const std::string &name) {
  const auto params =
      nlohmann::json::parse(db.point_cloud_provenance(name).second);
  return params.value(reusex::core::kLabelEncodingKey, std::string());
}

constexpr uint32_t kWrapped = 0xFFFFFFFFu; // a -1 that reached a CloudL

} // namespace

TEST_CASE("MigrationV29_OldSemanticCloud_ShiftsLabelsAndDefinitions",
          "[projectdb][migration][label_semantics]") {
  TempPath tmp("test_migration_v29_shift");
  {
    ProjectDB db(tmp.path);
    // Pre-v29 `rux create clouds` output: API ids stored verbatim (class 0
    // already lost in 0), definitions keyed by API id from annotate.
    db.save_point_cloud("labels", label_cloud({0, 1, 2, 0, 5, kWrapped}),
                        "reconstruct", R"({"resolution":0.05})");
    db.save_label_definitions("labels",
                              {{0, "wall"}, {1, "floor"}, {2, "door"}});
    // Structural clouds: not semantic, must not move.
    db.save_point_cloud("planes", label_cloud({0, 1, 2, 0, 5, 1}),
                        "segment_planes", "{}");
    db.save_point_cloud("rooms", label_cloud({1, 1, 2, 0, 2, 1}),
                        "segment_rooms", "{}");
    db.save_label_definitions("rooms", {{1, "Stue"}, {2, "Gang"}});
    db.save_point_cloud("instances", label_cloud({0, 1, 2, 0, 3, 0}),
                        "segment_instances", "{}");
    db.save_instances(
        "instances",
        {{1, "guid-1", 1, 1}, {2, "guid-2", 2, 1}, {3, "guid-3", 5, 1}});
    db.save_label_definitions(
        "instances", {{1, "SM1-1 (1p)"}, {2, "SM2-2 (1p)"}, {3, "SM5-3 (1p)"}});
    reusex::ProjectDB::SurveyTypeRecord door;
    door.name = "door";
    door.semantic_class = 2;
    db.add_survey_type(door);
    reusex::ProjectDB::SurveyTypeRecord manual;
    manual.name = "Manuel";
    manual.semantic_class = reusex::core::kManualSemanticClass;
    db.add_survey_type(manual);
  }
  roll_back_to_v28(tmp.path);

  ProjectDB db(tmp.path);
  REQUIRE(db.schema_version() == ProjectDB::latest_schema_version());
  REQUIRE(ProjectDB::latest_schema_version() >= 29);

  CHECK(values_of(db, "labels") ==
        std::vector<uint32_t>{0, 2, 3, 0, 6, kWrapped});
  CHECK(db.label_definitions("labels") ==
        std::map<int, std::string>{{1, "wall"}, {2, "floor"}, {3, "door"}});
  CHECK(encoding_of(db, "labels") == reusex::core::kLabelEncodingApiPlusOne);
  // The rest of the parameters survive.
  CHECK(nlohmann::json::parse(db.point_cloud_provenance("labels").second)
            .value("resolution", 0.0) == 0.05);
  CHECK(db.point_cloud_provenance("labels").first == "reconstruct");

  CHECK(values_of(db, "planes") == std::vector<uint32_t>{0, 1, 2, 0, 5, 1});
  CHECK(values_of(db, "rooms") == std::vector<uint32_t>{1, 1, 2, 0, 2, 1});
  CHECK(db.label_definitions("rooms") ==
        std::map<int, std::string>{{1, "Stue"}, {2, "Gang"}});
  CHECK(values_of(db, "instances") == std::vector<uint32_t>{0, 1, 2, 0, 3, 0});
  CHECK(encoding_of(db, "planes").empty());
  CHECK(encoding_of(db, "instances").empty());

  // Semantic class references follow the shifted semantic cloud.
  auto inst = db.instances("instances");
  std::sort(inst.begin(), inst.end(), [](const auto &a, const auto &b) {
    return a.instance_id < b.instance_id;
  });
  REQUIRE(inst.size() == 3);
  CHECK(inst[0].semantic_class == 2);
  CHECK(inst[1].semantic_class == 3);
  CHECK(inst[2].semantic_class == 6);
  CHECK(inst[0].guid == "guid-1");
  CHECK(db.label_definitions("instances") ==
        std::map<int, std::string>{
            {1, "SM2-1 (1p)"}, {2, "SM3-2 (1p)"}, {3, "SM6-3 (1p)"}});
  for (const auto &t : db.survey_types()) {
    if (t.name == "door")
      CHECK(t.semantic_class == 3);
    if (t.name == "Manuel")
      CHECK(t.semantic_class == reusex::core::kManualSemanticClass);
  }
}

TEST_CASE("MigrationV29_AlreadyEncodedCloud_IsNotShiftedAgain",
          "[projectdb][migration][label_semantics]") {
  TempPath tmp("test_migration_v29_encoded");
  {
    ProjectDB db(tmp.path);
    db.save_point_cloud("labels", label_cloud({0, 1, 2, 3}), "reconstruct",
                        R"({"label_encoding":"api_plus_one"})");
    db.save_label_definitions("labels",
                              {{1, "wall"}, {2, "floor"}, {3, "door"}});
    db.save_point_cloud("instances", label_cloud({0, 1, 1, 2}),
                        "segment_instances", "{}");
    db.save_instances("instances", {{1, "guid-1", 2, 2}, {2, "guid-2", 3, 1}});
    db.save_label_definitions("instances",
                              {{1, "SM2-1 (2p)"}, {2, "SM3-2 (1p)"}});
  }
  roll_back_to_v28(tmp.path);

  {
    ProjectDB db(tmp.path);
    REQUIRE(db.schema_version() == ProjectDB::latest_schema_version());
    CHECK(values_of(db, "labels") == std::vector<uint32_t>{0, 1, 2, 3});
    CHECK(db.label_definitions("labels") ==
          std::map<int, std::string>{{1, "wall"}, {2, "floor"}, {3, "door"}});
    auto inst = db.instances("instances");
    std::sort(inst.begin(), inst.end(), [](const auto &a, const auto &b) {
      return a.instance_id < b.instance_id;
    });
    REQUIRE(inst.size() == 2);
    CHECK(inst[0].semantic_class == 2);
    CHECK(inst[1].semantic_class == 3);
    CHECK(db.label_definitions("instances").at(1) == "SM2-1 (2p)");
  }

  // Reopening a migrated project runs nothing again.
  ProjectDB again(tmp.path);
  CHECK(values_of(again, "labels") == std::vector<uint32_t>{0, 1, 2, 3});
}

TEST_CASE("MigrationV29_ProjectWithoutSemanticCloud_LeavesClassesAlone",
          "[projectdb][migration][label_semantics]") {
  // A project whose instances came from mask -> resource on a semantic cloud
  // that no longer exists has nothing to re-base its class numbers on.
  TempPath tmp("test_migration_v29_none");
  {
    ProjectDB db(tmp.path);
    db.save_point_cloud("instances", label_cloud({0, 1}), "segment_instances",
                        "{}");
    db.save_instances("instances", {{1, "guid-1", 4, 1}});
    db.save_label_definitions("instances", {{1, "SM4-1 (1p)"}});
  }
  roll_back_to_v28(tmp.path);

  ProjectDB db(tmp.path);
  REQUIRE(db.schema_version() == ProjectDB::latest_schema_version());
  CHECK(db.instances("instances").at(0).semantic_class == 4);
  CHECK(db.label_definitions("instances").at(1) == "SM4-1 (1p)");
}

TEST_CASE("MigrationV29_FreshProject_IsAtLatestVersion",
          "[projectdb][migration][label_semantics]") {
  TempPath tmp("test_migration_v29_fresh");
  ProjectDB db(tmp.path);
  CHECK(db.schema_version() == ProjectDB::latest_schema_version());
  CHECK(ProjectDB::latest_schema_version() == 29);
}
