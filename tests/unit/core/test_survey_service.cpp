// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
// Survey operations that need a project: the sample approval gate, quantity
// redistribution across parts, totals, and the checked sample edit.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <core/ProjectDB.hpp>
#include <core/SensorIntrinsics.hpp>
#include <core/resource_view.hpp>
#include <core/survey_service.hpp>

#include "../../support/temp_path.hpp"

#include <opencv2/core.hpp>
#include <pcl/point_types.h>

#include <algorithm>
#include <stdexcept>

using reusex::ProjectDB;
using namespace reusex::core;

namespace {
struct TempDB : reusex::test_support::TempPath {
  TempDB() : reusex::test_support::TempPath("test_survey_service") {}
};
int64_t type_with_parts(ProjectDB &db, std::vector<double> quantities) {
  ProjectDB::SurveyTypeRecord t;
  t.name = "Betonsøjler, bærende";
  t.mass_t = 58.0;
  const auto id = db.add_survey_type(t).id;
  int n = db.max_survey_part_number();
  for (double q : quantities)
    db.add_survey_part({part_code(++n),
                        id,
                        std::nullopt,
                        std::nullopt,
                        std::nullopt,
                        "",
                        q,
                        false,
                        "",
                        {},
                        {}});
  return id;
}
} // namespace

TEST_CASE("SetReviewStatus_PendingSample_BlocksApproval_NotRejection",
          "[survey][service]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto id = type_with_parts(db, {1});
  const auto s = db.add_sample("PCB i fugemasse", "");
  db.set_sample_links(s.id, {id});
  CHECK(environment_status_of(db, id) == EnvironmentStatus::afventer);
  CHECK_THROWS_AS(set_review_status(db, id, ReviewStatus::approved),
                  SamplePendingError);
  CHECK(db.survey_type(id)->review_status == ReviewStatus::queue);
  CHECK(set_review_status(db, id, ReviewStatus::rejected).review_status ==
        ReviewStatus::rejected);

  ProjectDB::SamplePatch answered;
  answered.stage = SampleStage::svar;
  answered.result = SampleResult::ren;
  update_sample_checked(db, s.id, answered);
  CHECK(environment_status_of(db, id) == EnvironmentStatus::ren_proevesvar);
  CHECK(set_review_status(db, id, ReviewStatus::approved).review_status ==
        ReviewStatus::approved);
}

TEST_CASE("SetTypeQuantity_RedistributesAcrossParts", "[survey][service]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto id = type_with_parts(db, {18, 6});
  const auto parts = set_type_quantity(db, id, 30);
  REQUIRE(parts.size() == 2);
  CHECK(parts[0].quantity == 22.5);
  CHECK(parts[1].quantity == 7.5);
  const auto empty = type_with_parts(db, {});
  CHECK_THROWS_AS(set_type_quantity(db, empty, 5), std::invalid_argument);
  CHECK_THROWS_AS(set_type_quantity(db, id, -1), std::invalid_argument);
}

TEST_CASE("UpdateSampleChecked_ResultBeforeAnswer_Throws",
          "[survey][service]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto s = db.add_sample("x", "");
  ProjectDB::SamplePatch p;
  p.result = SampleResult::ren;
  CHECK_THROWS_AS(update_sample_checked(db, s.id, p), std::invalid_argument);
  CHECK(db.sample(s.id)->result == SampleResult::none);
}

TEST_CASE("TypeTotals_CarryEnvironment", "[survey][service]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto a = type_with_parts(db, {1});
  type_with_parts(db, {1});
  const auto s = db.add_sample("x", "");
  db.set_sample_links(s.id, {a});
  const auto totals = type_totals(db);
  REQUIRE(totals.size() == 2);
  CHECK(totals[0].environment == EnvironmentStatus::afventer);
  CHECK(totals[1].environment == EnvironmentStatus::ren_screening);
  CHECK(totals[0].mass_t == 58.0);
}

namespace {
reusex::CloudL label_cloud(std::initializer_list<std::uint32_t> labels) {
  reusex::CloudL c;
  for (auto l : labels) {
    pcl::Label p;
    p.label = l;
    c.push_back(p);
  }
  return c;
}
/// Instances 1,2 (class 3 = window), 3 (class 5 = door); rooms 4 and 6.
void seed_scan(ProjectDB &db, bool with_rooms = true) {
  db.save_point_cloud("instances", label_cloud({1, 1, 2, 2, 3, 0}), "test",
                      "{}");
  db.save_instances("instances",
                    {{1, "g1", 3, 2}, {2, "g2", 3, 2}, {3, "g3", 5, 1}});
  db.save_point_cloud("labels", label_cloud({3, 3, 3, 3, 5, 0}), "test", "{}");
  db.save_label_definitions("labels", {{3, "window"}, {5, "door"}});
  if (with_rooms) {
    db.save_point_cloud("rooms", label_cloud({4, 4, 6, 6, 6, 0}), "test", "{}");
    db.save_label_definitions("rooms", {{4, "Office Zone"}});
  }
}
} // namespace

TEST_CASE("SyncSurvey_CreatesTypePerClass_PartPerInstance_WithRooms",
          "[survey][sync]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  seed_scan(db);
  const auto r = sync_survey(db);
  CHECK(r.types_created == 2);
  CHECK(r.parts_created == 3);
  CHECK(r.rooms_assigned);
  const auto types = db.survey_types();
  REQUIRE(types.size() == 2);
  CHECK(types[0].name == "window");
  CHECK(types[0].semantic_class == 3);
  const auto parts = db.survey_parts();
  REQUIRE(parts.size() == 3);
  CHECK(parts[0].code == "RX-001");
  CHECK(parts[0].instance_id == 1u);
  CHECK(parts[0].room_name == "Office Zone");
  CHECK(parts[1].room_id == 6u);
  CHECK(parts[1].room_name == "Rum 6");
  CHECK(parts[2].type_id == types[1].id);
}

TEST_CASE("SyncSurvey_Rerun_PreservesUserEdits", "[survey][sync]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  seed_scan(db);
  sync_survey(db);
  const auto types = db.survey_types();
  ProjectDB::SurveyTypePatch rename;
  rename.name = "Vinduespartier, aluminium";
  db.update_survey_type(types[0].id, rename);
  ProjectDB::SurveyPartPatch move;
  move.type_id = types[1].id;
  move.quantity = 26;
  db.update_survey_part("RX-001", move);

  const auto r = sync_survey(db);
  CHECK(r.types_created == 0);
  CHECK(r.parts_created == 0);
  CHECK(r.parts_existing == 3);
  CHECK(db.survey_type(types[0].id)->name == "Vinduespartier, aluminium");
  CHECK(db.survey_part("RX-001")->type_id == types[1].id);
  CHECK(db.survey_part("RX-001")->quantity == 26);
}

TEST_CASE("SyncSurvey_NoInstanceCloud_ThrowsNamingTheStage", "[survey][sync]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  try {
    sync_survey(db);
    FAIL("expected a throw");
  } catch (const std::runtime_error &e) {
    CHECK(std::string(e.what()).find("rux create instances") !=
          std::string::npos);
  }
}

TEST_CASE("SyncSurvey_WithoutRooms_StillCreatesParts", "[survey][sync]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  seed_scan(db, /*with_rooms=*/false);
  const auto r = sync_survey(db);
  CHECK(r.parts_created == 3);
  CHECK_FALSE(r.rooms_assigned);
  CHECK(db.survey_parts()[0].room_name.empty());
}

TEST_CASE("SyncSurvey_InstanceGuidReplaced_ReportsOrphan_AndCreatesNewPart",
          "[survey][sync]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  seed_scan(db);
  sync_survey(db);
  REQUIRE(db.survey_part("RX-001").has_value());
  REQUIRE(db.survey_part("RX-001")->instance_guid == "g1");

  // `rux create instances` (or --clear) re-ran without carrying instance 1's
  // guid over: same instance_id, new guid. RX-001's instance_guid ("g1") no
  // longer resolves to any instances row.
  db.save_instances("instances",
                    {{1, "g1b", 3, 2}, {2, "g2", 3, 2}, {3, "g3", 5, 1}});

  const auto r = sync_survey(db);
  CHECK(r.parts_orphaned == 1);
  CHECK(r.orphaned_codes == std::vector<std::string>{"RX-001"});
  CHECK(r.parts_created == 1); // the "new" instance (guid g1b)

  const auto orphan = db.survey_part("RX-001");
  REQUIRE(orphan.has_value());
  CHECK(orphan->instance_guid == "g1");
  CHECK_FALSE(orphan->instance_id.has_value());
  CHECK_FALSE(orphan->cloud_name.has_value());
}

TEST_CASE("SyncSurvey_RestoresLostInstanceLinks_NeverOverwrites",
          "[survey][sync]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  seed_scan(db);
  sync_survey(db);
  std::string own1, own2;
  {
    ProjectDB::Transaction tx(db);
    own1 = db.ensure_resource_passport("RX-001");
    own2 = db.ensure_resource_passport("RX-002");
    tx.commit();
  }
  REQUIRE(db.instance_material_guid("instances", 1) == own1);
  // A `rux create instances` re-run: same guids, but save_instances
  // cascade-deletes every instance_materials row of the cloud.
  db.save_instances("instances",
                    {{1, "g1", 3, 2}, {2, "g2", 3, 2}, {3, "g3", 5, 1}});
  REQUIRE_FALSE(db.instance_material_guid("instances", 1).has_value());
  // Instance 2 meanwhile got linked to another passport: left alone.
  db.set_instance_material("instances", 2, own1);

  const auto r = sync_survey(db);
  CHECK(r.links_restored == 1); // RX-001; RX-003 owns no passport
  CHECK(db.instance_material_guid("instances", 1) == own1);
  CHECK(db.instance_material_guid("instances", 2) == own1);
  CHECK_FALSE(db.instance_material_guid("instances", 3).has_value());
  CHECK(db.survey_part("RX-002")->material_guid == own2);
  CHECK(sync_survey(db).links_restored == 0);
}

TEST_CASE("SyncSurvey_ManualType_NeverMatched_UnclassifiedGetsOwnType",
          "[survey][sync]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  // An unclassified instance (class -1) plus a manually created type, which
  // must use kManualSemanticClass so sync_survey's seeding never matches it.
  db.save_point_cloud("instances", label_cloud({1, 1, 0}), "test", "{}");
  db.save_instances("instances", {{1, "g1", -1, 2}});
  ProjectDB::SurveyTypeRecord manual;
  manual.name = "Manuelt tilføjet type";
  manual.semantic_class = kManualSemanticClass;
  db.add_survey_type(manual);

  const auto r = sync_survey(db);
  CHECK(r.types_created == 1); // "Uklassificeret", not the manual type
  CHECK(r.parts_created == 1);
  const auto types = db.survey_types();
  REQUIRE(types.size() == 2);
  const auto manual_type =
      std::find_if(types.begin(), types.end(), [](const auto &t) {
        return t.name == "Manuelt tilføjet type";
      });
  REQUIRE(manual_type != types.end());
  for (const auto &p : db.survey_parts())
    CHECK(p.type_id != manual_type->id);
  const auto unclassified =
      std::find_if(types.begin(), types.end(),
                   [](const auto &t) { return t.name == "Uklassificeret"; });
  REQUIRE(unclassified != types.end());
  CHECK(unclassified->semantic_class == -1);
}

// Regression: an instance cloud written before schema v10 has its labels and
// "SM{class}-{id} (Np)" definitions but no `instances` rows — the v10
// migration backfilled rows only for clouds that had material links. Sync
// used to iterate the empty table, create nothing and report "no instances"
// while the cloud held 155 of them (NewOffice, 2026-10-02).
TEST_CASE("SyncSurvey_LegacyCloudWithoutInstanceRows_BackfillsThem",
          "[survey][sync]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  db.save_point_cloud("instances", label_cloud({1, 1, 2, 2, 3, 0}), "test",
                      "{}");
  // 4294967295 is the pre-#214 unlabeled semantic value wrapped to uint32.
  db.save_label_definitions(
      "instances",
      {{1, "SM3-1 (2p)"}, {2, "SM3-2 (2p)"}, {3, "SM4294967295-3 (1p)"}});
  db.save_point_cloud("labels", label_cloud({3, 3, 3, 3, 0, 0}), "test", "{}");
  db.save_label_definitions("labels", {{3, "window"}});
  db.save_point_cloud("rooms", label_cloud({4, 4, 4, 6, 6, 0}), "test", "{}");
  REQUIRE(db.instances("instances").empty());

  const auto r = sync_survey(db);
  CHECK(r.instances_backfilled == 3);
  CHECK(r.instances_seen == 3);
  CHECK(r.types_created == 2); // window + Uklassificeret
  CHECK(r.parts_created == 3);
  CHECK(r.rooms_assigned);

  const auto rows = db.instances("instances");
  REQUIRE(rows.size() == 3);
  CHECK(rows[0].semantic_class == 3);
  CHECK(rows[0].point_count == 2);
  CHECK_FALSE(rows[0].guid.empty());
  CHECK(rows[2].semantic_class == -1);
  CHECK(rows[2].point_count == 1);
  const auto parts = db.survey_parts();
  REQUIRE(parts.size() == 3);
  CHECK(parts[0].instance_guid == rows[0].guid);
  CHECK(parts[1].room_id == 4u); // instance 2: points 2,3 → rooms 4,6 tie → 4

  // A rerun finds the rows it wrote: nothing is backfilled or duplicated.
  const auto again = sync_survey(db);
  CHECK(again.instances_backfilled == 0);
  CHECK(again.instances_seen == 3);
  CHECK(again.parts_created == 0);
  CHECK(again.parts_existing == 3);
}

TEST_CASE("SyncSurvey_LegacyCloudWithoutDefinitions_BackfillsUnclassified",
          "[survey][sync]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  db.save_point_cloud("instances", label_cloud({7, 7, 0}), "test", "{}");
  const auto r = sync_survey(db);
  CHECK(r.instances_backfilled == 1);
  CHECK(r.parts_created == 1);
  REQUIRE(db.survey_types().size() == 1);
  CHECK(db.survey_types()[0].name == "Uklassificeret");
  CHECK(db.instances("instances")[0].instance_id == 7u);
}

TEST_CASE("SyncSurvey_EmptyInstanceCloud_ReportsZeroSeen", "[survey][sync]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  db.save_point_cloud("instances", label_cloud({0, 0}), "test", "{}");
  const auto r = sync_survey(db);
  CHECK(r.instances_seen == 0);
  CHECK(r.instances_backfilled == 0);
  CHECK(r.parts_created == 0);
  CHECK(db.instances("instances").empty());
}

namespace {
reusex::core::SensorIntrinsics pinhole128() {
  reusex::core::SensorIntrinsics i;
  i.fx = i.fy = 100.0;
  i.cx = i.cy = 64.0;
  i.width = i.height = 128;
  return i;
}
/// A frame at the origin looking down +z, no depth (frustum test only).
void posed_frame(ProjectDB &db, int id) {
  db.save_sensor_frame(id, cv::Mat(128, 128, CV_8UC3, cv::Scalar(40, 90, 160)),
                       cv::Mat(), cv::Mat(),
                       {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1},
                       pinhole128(), static_cast<double>(id), -1);
}
/// Instance 1 in front of the camera (two points at z = 2), instance 2
/// behind every camera. Both class 3.
void seed_seen_scan(ProjectDB &db, bool with_positions = true) {
  reusex::Cloud positions;
  reusex::CloudL labels;
  auto add = [&](float x, float y, float z, std::uint32_t label) {
    pcl::PointXYZRGB p;
    p.x = x;
    p.y = y;
    p.z = z;
    positions.push_back(p);
    pcl::Label l;
    l.label = label;
    labels.push_back(l);
  };
  add(-0.1F, 0.0F, 2.0F, 1);
  add(0.1F, 0.0F, 2.0F, 1);
  add(0.0F, 0.0F, -10.0F, 2);
  if (with_positions)
    db.save_point_cloud("cloud", positions, "test", "{}");
  db.save_point_cloud("instances", labels, "test", "{}");
  db.save_instances("instances", {{1, "g1", 3, 2}, {2, "g2", 3, 1}});
}
} // namespace

TEST_CASE("SyncSurvey_AttachesBestFrameView_BackfillsAndNeverOverwrites",
          "[survey][sync][view]") {
  using Catch::Matchers::WithinAbs;
  TempDB tmp;
  ProjectDB db(tmp.path);
  seed_seen_scan(db);

  // No sensor frames yet: parts are created, no view anywhere.
  const auto first = sync_survey(db);
  CHECK(first.parts_created == 2);
  CHECK(first.views_attached == 0);
  CHECK(first.views_unavailable == 2);
  CHECK_FALSE(db.survey_part("RX-001")->view);

  // Frames arrive later (an old project): the next sync backfills.
  posed_frame(db, 1);
  const auto second = sync_survey(db);
  CHECK(second.parts_created == 0);
  CHECK(second.views_attached == 1);
  CHECK(second.views_unavailable == 1); // instance 2 is seen by no frame
  const auto view = db.survey_part("RX-001")->view;
  REQUIRE(view);
  CHECK(view->kind == ViewKind::frame);
  CHECK(view->id == 1);
  REQUIRE(view->region);
  CHECK_THAT((*view->region)[0], WithinAbs(57.35 / 128.0, 1e-9));
  CHECK_THAT((*view->region)[2], WithinAbs(14.3 / 128.0, 1e-9));
  CHECK_FALSE(db.survey_part("RX-002")->view);

  // The user's choice survives a rerun.
  const ResourceView chosen{ViewKind::frame, 1, std::nullopt};
  db.set_survey_part_view("RX-001", chosen);
  const auto third = sync_survey(db);
  CHECK(third.views_attached == 0);
  CHECK(db.survey_part("RX-001")->view == chosen);
}

TEST_CASE("SyncSurvey_FramesButNoPositionsCloud_SucceedsWithoutViews",
          "[survey][sync][view]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  seed_seen_scan(db, /*with_positions=*/false);
  posed_frame(db, 1);
  const auto r = sync_survey(db);
  CHECK(r.parts_created == 2);
  CHECK(r.views_attached == 0);
  CHECK(r.views_unavailable == 2);
}

TEST_CASE("SyncSurvey_MisalignedPositionsCloud_SucceedsWithoutViews",
          "[survey][sync][view]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  seed_seen_scan(db);
  reusex::Cloud shorter; // 1 point vs. the instance cloud's 3
  pcl::PointXYZRGB p;
  p.x = p.y = 0.0F;
  p.z = 2.0F;
  shorter.push_back(p);
  db.save_point_cloud("cloud", shorter, "test", "{}");
  posed_frame(db, 1);
  const auto r = sync_survey(db);
  CHECK(r.parts_created == 2);
  CHECK(r.views_attached == 0);
  CHECK(r.views_unavailable == 2);
  CHECK_FALSE(db.survey_part("RX-001")->view);
}
