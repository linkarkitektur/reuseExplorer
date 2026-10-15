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

#include <reusex/core/MaterialPassport.hpp>
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

constexpr uint32_t kWrapped = 0xFFFFFFFFu;  // a -1 that reached a CloudL
constexpr uint32_t kMaxInt32 = 0x7FFFFFFFu; // +1 would leave the contract

void add_passport(ProjectDB &db, const std::string &guid,
                  const std::string &designation) {
  reusex::core::MaterialPassport p;
  p.metadata.document_guid = guid;
  p.metadata.creation_date = "2025-01-01T00:00:00Z";
  p.metadata.version_number = "1.0.0";
  p.description.designation = designation;
  db.add_material_passport(p, "test-project");
}

std::vector<ProjectDB::InstanceRecord> sorted_instances(const ProjectDB &db,
                                                        const char *cloud) {
  auto inst = db.instances(cloud);
  std::sort(inst.begin(), inst.end(), [](const auto &a, const auto &b) {
    return a.instance_id < b.instance_id;
  });
  return inst;
}

} // namespace

TEST_CASE("MigrationV29_OldSemanticCloud_ShiftsLabelsAndDefinitions",
          "[projectdb][migration][label_semantics]") {
  TempPath tmp("test_migration_v29_shift");
  {
    ProjectDB db(tmp.path);
    // Pre-v29 `rux create clouds` output: API ids stored verbatim (class 0
    // already lost in 0), definitions keyed by API id from annotate.
    db.save_point_cloud("labels",
                        label_cloud({0, 1, 2, 0, 5, kWrapped, kMaxInt32}),
                        "reconstruct", R"({"resolution":0.05})");
    db.save_label_definitions("labels",
                              {{0, "wall"}, {1, "floor"}, {2, "door"}});
    // Structural clouds: not semantic, must not move.
    db.save_point_cloud("planes", label_cloud({0, 1, 2, 0, 5, 1, 1}),
                        "segment_planes", "{}");
    db.save_point_cloud("rooms", label_cloud({1, 1, 2, 0, 2, 1, 1}),
                        "segment_rooms", "{}");
    db.save_label_definitions("rooms", {{1, "Stue"}, {2, "Gang"}});
    db.save_point_cloud("instances", label_cloud({0, 1, 2, 0, 3, 0, 0}),
                        "segment_instances", "{}");
    db.save_instances(
        "instances",
        {{1, "guid-1", 1, 1}, {2, "guid-2", 2, 1}, {3, "guid-3", 5, 1}});
    db.save_label_definitions(
        "instances", {{1, "SM1-1 (1p)"}, {2, "SM2-2 (1p)"}, {3, "SM5-3 (1p)"}});
    // Survey types: a user-named one, sync_survey's generated fallback name,
    // and the manual sentinel.
    ProjectDB::SurveyTypeRecord door;
    door.name = "door";
    door.semantic_class = 2;
    db.add_survey_type(door);
    ProjectDB::SurveyTypeRecord generated;
    generated.name = "Klasse 5";
    generated.semantic_class = 5;
    db.add_survey_type(generated);
    ProjectDB::SurveyTypeRecord renamed;
    renamed.name = "Klasse 5 (vinduer)"; // user-edited: kept
    renamed.semantic_class = 5;
    db.add_survey_type(renamed);
    ProjectDB::SurveyTypeRecord manual;
    manual.name = "Manuel";
    manual.semantic_class = reusex::core::kManualSemanticClass;
    db.add_survey_type(manual);
    // `rux create materials`' "class N" designation for an unnamed class, and
    // a user-written one.
    add_passport(db, "mat-3", "class 5");
    db.set_instance_material("instances", 3, "mat-3");
    add_passport(db, "mat-2", "Dør, eg");
    db.set_instance_material("instances", 2, "mat-2");
  }
  roll_back_to_v28(tmp.path);

  ProjectDB db(tmp.path);
  REQUIRE(db.schema_version() == ProjectDB::latest_schema_version());
  REQUIRE(ProjectDB::latest_schema_version() >= 29);

  // 0, a wrapped -1 and a value that would wrap stay as they are.
  CHECK(values_of(db, "labels") ==
        std::vector<uint32_t>{0, 2, 3, 0, 6, kWrapped, kMaxInt32});
  CHECK(db.label_definitions("labels") ==
        std::map<int, std::string>{{1, "wall"}, {2, "floor"}, {3, "door"}});
  CHECK(encoding_of(db, "labels") == reusex::core::kLabelEncodingApiPlusOne);
  // The rest of the parameters survive.
  CHECK(nlohmann::json::parse(db.point_cloud_provenance("labels").second)
            .value("resolution", 0.0) == 0.05);
  CHECK(db.point_cloud_provenance("labels").first == "reconstruct");

  CHECK(values_of(db, "planes") == std::vector<uint32_t>{0, 1, 2, 0, 5, 1, 1});
  CHECK(values_of(db, "rooms") == std::vector<uint32_t>{1, 1, 2, 0, 2, 1, 1});
  CHECK(db.label_definitions("rooms") ==
        std::map<int, std::string>{{1, "Stue"}, {2, "Gang"}});
  CHECK(values_of(db, "instances") ==
        std::vector<uint32_t>{0, 1, 2, 0, 3, 0, 0});
  CHECK(encoding_of(db, "planes").empty());
  CHECK(encoding_of(db, "instances").empty());

  // Semantic class references follow the shifted semantic cloud.
  auto inst = sorted_instances(db, "instances");
  REQUIRE(inst.size() == 3);
  CHECK(inst[0].semantic_class == 2);
  CHECK(inst[1].semantic_class == 3);
  CHECK(inst[2].semantic_class == 6);
  CHECK(inst[0].guid == "guid-1");
  CHECK(db.label_definitions("instances") ==
        std::map<int, std::string>{
            {1, "SM2-1 (1p)"}, {2, "SM3-2 (1p)"}, {3, "SM6-3 (1p)"}});
  std::map<std::string, int> types;
  for (const auto &t : db.survey_types())
    types[t.name] = t.semantic_class;
  CHECK(types == std::map<std::string, int>{
                     {"door", 3},
                     {"Klasse 6", 6},
                     {"Klasse 5 (vinduer)", 6},
                     {"Manuel", reusex::core::kManualSemanticClass}});
  CHECK(db.material_passport("mat-3").description.designation == "class 6");
  CHECK(db.material_passport("mat-2").description.designation == "Dør, eg");
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
  }
  roll_back_to_v28(tmp.path);

  {
    ProjectDB db(tmp.path);
    REQUIRE(db.schema_version() == ProjectDB::latest_schema_version());
    CHECK(values_of(db, "labels") == std::vector<uint32_t>{0, 1, 2, 3});
    CHECK(db.label_definitions("labels") ==
          std::map<int, std::string>{{1, "wall"}, {2, "floor"}, {3, "door"}});
  }

  // Reopening a migrated project runs nothing again.
  ProjectDB again(tmp.path);
  CHECK(values_of(again, "labels") == std::vector<uint32_t>{0, 1, 2, 3});
}

TEST_CASE("MigrationV29_ProjectWithoutSemanticCloud_StillRebasesClasses",
          "[projectdb][migration][label_semantics]") {
  // The stored class numbers were always the old (API-equal) numbering, so
  // they move even when the semantic cloud they came from is gone. The
  // instance cloud here has no `instances` rows of its own (pre-v10 /
  // sync_downsample'd), and its SM definitions are rewritten all the same.
  TempPath tmp("test_migration_v29_none");
  {
    ProjectDB db(tmp.path);
    db.save_point_cloud("instances", label_cloud({0, 1}), "segment_instances",
                        "{}");
    db.save_instances("instances", {{1, "guid-1", 4, 1}});
    db.save_label_definitions("instances", {{1, "SM4-1 (1p)"}});
    db.save_point_cloud("old_instances", label_cloud({0, 1}), "sync_downsample",
                        "{}");
    db.save_label_definitions("old_instances",
                              {{1, "SM4-1 (1p)"}, {2, "SMx-2"}, {3, "wall"}});
    ProjectDB::SurveyTypeRecord t;
    t.name = "Klasse 4";
    t.semantic_class = 4;
    db.add_survey_type(t);
  }
  roll_back_to_v28(tmp.path);

  ProjectDB db(tmp.path);
  REQUIRE(db.schema_version() == ProjectDB::latest_schema_version());
  CHECK(db.instances("instances").at(0).semantic_class == 5);
  CHECK(db.label_definitions("instances").at(1) == "SM5-1 (1p)");
  CHECK(
      db.label_definitions("old_instances") ==
      std::map<int, std::string>{{1, "SM5-1 (1p)"}, {2, "SMx-2"}, {3, "wall"}});
  REQUIRE(db.survey_types().size() == 1);
  CHECK(db.survey_types().at(0).name == "Klasse 5");
  CHECK(db.survey_types().at(0).semantic_class == 5);
}

TEST_CASE("MigrationV29_DefaultNameAnyStage_IsSemantic",
          "[projectdb][migration][label_semantics]") {
  // "labels" is the default semantic cloud whichever stage last wrote it.
  for (const char *stage : {"sync_downsample", "segment_resource", ""}) {
    INFO("stage '" << stage << "'");
    TempPath tmp(std::string("test_migration_v29_stage_") + stage);
    {
      ProjectDB db(tmp.path);
      db.save_point_cloud("labels", label_cloud({0, 1, 2}), stage, "");
      db.save_label_definitions("labels", {{1, "floor"}});
    }
    roll_back_to_v28(tmp.path);
    ProjectDB db(tmp.path);
    CHECK(values_of(db, "labels") == std::vector<uint32_t>{0, 2, 3});
    CHECK(db.label_definitions("labels") ==
          std::map<int, std::string>{{2, "floor"}});
    CHECK(encoding_of(db, "labels") == reusex::core::kLabelEncodingApiPlusOne);
  }
}

TEST_CASE("MigrationV29_ValueStraddlingChunkBoundary_IsShiftedWhole",
          "[projectdb][migration][label_semantics]") {
  // Real chunks are 256 MiB; the boundary is forced by splitting the stored
  // blob after 6 bytes, i.e. through the middle of the second label.
  TempPath tmp("test_migration_v29_chunks");
  {
    ProjectDB db(tmp.path);
    db.save_point_cloud("labels", label_cloud({7, 0x00010203u, 9, 0}),
                        "reconstruct", "");
  }
  exec_raw(tmp.path, R"(
    INSERT INTO point_cloud_data (cloud_id, chunk_index, data)
      SELECT cloud_id, 1, substr(data, 7) FROM point_cloud_data
      WHERE chunk_index = 0 AND cloud_id =
        (SELECT id FROM point_clouds WHERE name = 'labels');
    UPDATE point_cloud_data SET data = substr(data, 1, 6)
      WHERE chunk_index = 0 AND cloud_id =
        (SELECT id FROM point_clouds WHERE name = 'labels');
  )");
  {
    ProjectDB db(tmp.path, /*readOnly=*/true);
    REQUIRE(values_of(db, "labels") ==
            std::vector<uint32_t>{7, 0x00010203u, 9, 0});
  }
  roll_back_to_v28(tmp.path);

  ProjectDB db(tmp.path);
  CHECK(values_of(db, "labels") ==
        std::vector<uint32_t>{8, 0x00010204u, 10, 0});
}

TEST_CASE("MigrationV29_ThrowMidway_RollsBackEverything",
          "[projectdb][migration][label_semantics]") {
  // A first semantic cloud is shifted, then a corrupt one (5 bytes, not a
  // whole label) throws: the savepoint must undo the first shift too, and
  // the project must stay at v28.
  TempPath tmp("test_migration_v29_rollback");
  {
    ProjectDB db(tmp.path);
    db.save_point_cloud("a_semantic", label_cloud({1, 2}), "reconstruct", "");
    db.save_point_cloud("labels", label_cloud({1, 2}), "project_labels", "");
  }
  exec_raw(tmp.path, R"(
    UPDATE point_cloud_data SET data = substr(data, 1, 5)
      WHERE cloud_id = (SELECT id FROM point_clouds WHERE name = 'labels');
  )");
  roll_back_to_v28(tmp.path);

  REQUIRE_THROWS(ProjectDB(tmp.path));

  ProjectDB ro(tmp.path, /*readOnly=*/true);
  CHECK(ro.schema_version() == 28);
  CHECK(values_of(ro, "a_semantic") == std::vector<uint32_t>{1, 2});
  CHECK(ro.point_cloud_provenance("a_semantic").second.empty());
}

TEST_CASE("MigrationV29_FreshProject_IsAtLatestVersion",
          "[projectdb][migration][label_semantics]") {
  TempPath tmp("test_migration_v29_fresh");
  ProjectDB db(tmp.path);
  CHECK(db.schema_version() == ProjectDB::latest_schema_version());
  CHECK(ProjectDB::latest_schema_version() >= 29);
}
