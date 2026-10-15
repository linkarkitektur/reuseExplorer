// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Mask -> cloud selection -> instance + survey part (Segmentering, spec B2).
//
// Projection fixture: a pinhole camera (fx=fy=100, 128x128, principal point
// at the centre, identity local transform) at the world origin looking down
// +z. Its depth image is HALF the intrinsics' size (64x64, CV_16U mm) and
// reads 2 m everywhere — a wall at z=2. The mask is TWICE the intrinsics' size
// (256x256) and covers intrinsics pixels [50, 90) in u and v. Both scale
// factors are deliberate: a projector that forgot to rescale either image
// would sample the wrong pixel and the expected set would change.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <core/MaterialPassport.hpp>
#include <core/ProjectDB.hpp>
#include <core/SensorIntrinsics.hpp>
#include <core/mask_selection.hpp>
#include <core/resource_view.hpp>
#include <core/survey_service.hpp>
#include <types.hpp>

#include "../../support/temp_path.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <array>
#include <map>
#include <vector>

using namespace reusex;
using reusex::core::apply_mask_selection;
using reusex::core::MaskSelectionError;
using reusex::core::MaskSelectionOptions;
using reusex::core::project_frame_mask;
using reusex::test_support::TempPath;

namespace {

reusex::core::SensorIntrinsics intrinsics() {
  reusex::core::SensorIntrinsics i;
  i.fx = i.fy = 100.0;
  i.cx = i.cy = 64.0;
  i.width = i.height = 128;
  return i;
}

constexpr std::array<double, 16> kIdentity = {1, 0, 0, 0, 0, 1, 0, 0,
                                              0, 0, 1, 0, 0, 0, 0, 1};

void save_frame(ProjectDB &db, int id, bool with_depth = true) {
  cv::Mat color(128, 128, CV_8UC3, cv::Scalar(10, 20, 30));
  cv::Mat depth;
  if (with_depth)
    depth = cv::Mat(64, 64, CV_16UC1, cv::Scalar(2000));
  db.save_sensor_frame(id, color, depth, cv::Mat(), kIdentity, intrinsics(),
                       1.0, -1);
}

/// 256x256 mask selecting intrinsics pixels [50, 90) x [50, 90).
cv::Mat make_mask() {
  cv::Mat m(256, 256, CV_8UC1, cv::Scalar(0));
  cv::rectangle(m, cv::Rect(100, 100, 80, 80), cv::Scalar(255), cv::FILLED);
  return m;
}

Cloud make_cloud(const std::vector<std::array<float, 3>> &xyz) {
  Cloud c;
  for (const auto &p : xyz) {
    PointT pt;
    pt.x = p[0];
    pt.y = p[1];
    pt.z = p[2];
    c.push_back(pt);
  }
  return c;
}

/// n points along x at z=2; geometry is irrelevant to apply_mask_selection.
Cloud line_cloud(std::size_t n) {
  std::vector<std::array<float, 3>> xyz;
  for (std::size_t i = 0; i < n; ++i)
    xyz.push_back({0.01f * static_cast<float>(i), 0.f, 2.f});
  return make_cloud(xyz);
}

CloudL label_cloud(const std::vector<uint32_t> &labels) {
  CloudL c;
  for (auto l : labels) {
    pcl::Label p;
    p.label = l;
    c.push_back(p);
  }
  return c;
}

} // namespace

TEST_CASE("project_frame_mask selects visible in-mask points only",
          "[core][mask_selection]") {
  const TempPath tmp("mask_projection");
  ProjectDB db(tmp.path);
  save_frame(db, 1);
  db.save_point_cloud("cloud", make_cloud({
                                   {0.0f, 0.0f, 2.0f},  // 0 centre, on wall
                                   {0.0f, 0.0f, 3.0f},  // 1 behind the wall
                                   {0.1f, 0.0f, 2.0f},  // 2 u=69, in mask
                                   {5.0f, 0.0f, 2.0f},  // 3 out of bounds
                                   {0.0f, 0.0f, -1.0f}, // 4 behind camera
                                   {-0.5f, 0.0f, 2.0f}, // 5 u=39, off mask
                                   {0.0f, 0.0f, 2.05f}, // 6 within 0.10 m
                                   {0.0f, 0.0f, 1.85f}, // 7 0.15 m in front
                               }));

  const auto hits = project_frame_mask(db, 1, make_mask());
  CHECK(hits == std::vector<std::size_t>{0, 2, 6});

  SECTION("tolerance and max_depth are honoured") {
    reusex::core::MaskProjectionOptions o;
    o.depth_tolerance = 0.2; // now admits point 7 (0.15 m off)
    CHECK(project_frame_mask(db, 1, make_mask(), o) ==
          std::vector<std::size_t>{0, 2, 6, 7});
    o.max_depth = 1.9; // only point 7 (z=1.85) is near enough
    CHECK(project_frame_mask(db, 1, make_mask(), o) ==
          std::vector<std::size_t>{7});
  }

  SECTION("a CV_32S mask works like a CV_8U one") {
    cv::Mat m32;
    make_mask().convertTo(m32, CV_32S);
    CHECK(project_frame_mask(db, 1, m32) == hits);
  }

  SECTION("an empty-covering mask yields no hits, not an error") {
    cv::Mat none(256, 256, CV_8UC1, cv::Scalar(0));
    CHECK(project_frame_mask(db, 1, none).empty());
  }
}

TEST_CASE("project_frame_mask refuses frames it cannot project",
          "[core][mask_selection]") {
  const TempPath tmp("mask_projection_errors");
  ProjectDB db(tmp.path);
  db.save_point_cloud("cloud", make_cloud({{0, 0, 2}}));

  SECTION("no such frame") {
    CHECK_THROWS_AS(project_frame_mask(db, 99, make_mask()),
                    MaskSelectionError);
  }
  SECTION("no pose") {
    db.save_sensor_frame(2, cv::Mat(128, 128, CV_8UC3, cv::Scalar(0)));
    CHECK_THROWS_AS(project_frame_mask(db, 2, make_mask()), MaskSelectionError);
  }
  SECTION("no depth") {
    save_frame(db, 3, /*with_depth=*/false);
    CHECK_THROWS_AS(project_frame_mask(db, 3, make_mask()), MaskSelectionError);
  }
  SECTION("empty mask image") {
    save_frame(db, 4);
    CHECK_THROWS_AS(project_frame_mask(db, 4, cv::Mat()),
                    std::invalid_argument);
  }
  SECTION("no base cloud") {
    save_frame(db, 5);
    reusex::core::MaskProjectionOptions o;
    o.cloud = "missing";
    CHECK_THROWS_AS(project_frame_mask(db, 5, make_mask(), o),
                    MaskSelectionError);
  }
}

TEST_CASE("apply_mask_selection on a project with no label clouds",
          "[core][mask_selection]") {
  const TempPath tmp("mask_apply_fresh");
  ProjectDB db(tmp.path);
  db.save_point_cloud("cloud", line_cloud(10));

  MaskSelectionOptions opts;
  opts.frame_id = 7;
  const auto r = apply_mask_selection(db, {3, 1, 2, 2}, "  Dør ", opts);

  CHECK(r.label_id == 1);
  CHECK(r.label_created);
  CHECK(r.instance_id == 1);
  CHECK(r.point_count == 3); // duplicates collapse
  CHECK_FALSE(r.instance_guid.empty());
  CHECK(r.shrunk_instances.empty());
  CHECK(r.resource_code == "RX-001");
  CHECK(r.type_created);

  const auto labels = db.point_cloud_label("labels");
  REQUIRE(labels->size() == 10);
  CHECK((*labels)[0].label == 0);
  CHECK((*labels)[1].label == 1);
  CHECK((*labels)[3].label == 1);
  CHECK(db.label_definitions("labels") ==
        std::map<int, std::string>{{1, "Dør"}});
  // A semantic cloud created here carries the encoding marker (STANDARDS
  // §3.1); the instance cloud is structural and does not.
  CHECK(db.point_cloud_provenance("labels").second ==
        R"({"label_encoding":"api_plus_one"})");
  CHECK(db.point_cloud_provenance("instances").second.empty());

  const auto inst = db.point_cloud_label("instances");
  REQUIRE(inst->size() == 10);
  CHECK((*inst)[2].label == 1);
  CHECK((*inst)[4].label == 0);
  const auto rows = db.instances("instances");
  REQUIRE(rows.size() == 1);
  CHECK(rows[0].guid == r.instance_guid);
  CHECK(rows[0].semantic_class == 1);
  CHECK(rows[0].point_count == 3);
  CHECK(db.label_definitions("instances").at(1) == "SM1-1 (3p)");

  const auto type = db.survey_type(r.type_id);
  REQUIRE(type);
  CHECK(type->name == "Dør");
  CHECK(type->semantic_class == 1);
  const auto part = db.survey_part(r.resource_code);
  REQUIRE(part);
  CHECK(part->type_id == r.type_id);
  CHECK(part->instance_guid == r.instance_guid);
  CHECK(part->instance_id == 1u);
  CHECK(part->cloud_name == "instances");

  // The part is already filed: sync_survey must not add a second one.
  const auto sync = reusex::core::sync_survey(db);
  CHECK(sync.parts_created == 0);
  CHECK(sync.parts_existing == 1);
  CHECK(db.survey_parts().size() == 1);

  // Logged.
  const auto log = db.pipeline_log();
  REQUIRE_FALSE(log.empty());
  CHECK(log.front().stage == "segment_resource");
  CHECK(log.front().status == "success");
  CHECK_THAT(log.front().parameters,
             Catch::Matchers::ContainsSubstring("\"frame_id\":7"));
}

namespace {
ProjectDB::SurveyTypePatch rejected_patch() {
  ProjectDB::SurveyTypePatch patch;
  patch.review_status = core::ReviewStatus::rejected;
  return patch;
}
} // namespace

TEST_CASE("apply_mask_selection on a project with labels and instances",
          "[core][mask_selection]") {
  const TempPath tmp("mask_apply_existing");
  ProjectDB db(tmp.path);
  db.save_point_cloud("cloud", line_cloud(10));
  // Semantic: 0-4 wall (1), 5-9 door (2).
  db.save_point_cloud("labels", label_cloud({1, 1, 1, 1, 1, 2, 2, 2, 2, 2}));
  db.save_label_definitions("labels", {{1, "Væg"}, {2, "Dør"}});
  // Instances: 0-4 -> 1 (wall), 5-9 -> 2 (door).
  db.save_point_cloud("instances", label_cloud({1, 1, 1, 1, 1, 2, 2, 2, 2, 2}));
  db.save_instances("instances",
                    {{1, "guid-wall", 1, 5}, {2, "guid-door", 2, 5}});
  db.save_label_definitions("instances",
                            {{1, "SM1-1 (5p)"}, {2, "SM2-2 (5p)"}});
  // A material link on the wall instance must survive (a save_instances()
  // rewrite would cascade it away).
  reusex::core::MaterialPassport p;
  p.metadata.document_guid = "passport-wall";
  p.metadata.creation_date = "2025-01-01T00:00:00Z";
  p.metadata.version_number = "1.0.0";
  db.add_material_passport(p, "test-project");
  db.set_instance_material("instances", 1, "passport-wall");
  // The type sync_survey would file class 2 under.
  ProjectDB::SurveyTypeRecord doors;
  doors.name = "Døre (fra scan)";
  doors.semantic_class = 2;
  const auto door_type = db.add_survey_type(doors).id;

  const auto r = apply_mask_selection(db, {3, 4, 5}, "Dør");

  CHECK(r.label_id == 2); // found by name
  CHECK_FALSE(r.label_created);
  CHECK(r.instance_id == 3);
  CHECK(r.type_id == door_type); // the class's existing type
  CHECK_FALSE(r.type_created);
  CHECK(r.shrunk_instances ==
        std::vector<std::pair<uint32_t, int>>{{1, 3}, {2, 4}});

  const auto labels = db.point_cloud_label("labels");
  CHECK((*labels)[3].label == 2);
  CHECK((*labels)[2].label == 1);
  CHECK(db.label_definitions("labels").size() == 2);

  const auto rows = db.instances("instances");
  REQUIRE(rows.size() == 3);
  CHECK(rows[0].guid == "guid-wall");
  CHECK(rows[0].point_count == 3);
  CHECK(rows[1].guid == "guid-door");
  CHECK(rows[1].point_count == 4);
  CHECK(rows[2].instance_id == 3);
  CHECK(rows[2].point_count == 3);
  CHECK(rows[2].semantic_class == 2);
  const auto defs = db.label_definitions("instances");
  CHECK(defs.at(1) == "SM1-1 (3p)");
  CHECK(defs.at(2) == "SM2-2 (4p)");
  CHECK(defs.at(3) == "SM2-3 (3p)");
  CHECK(db.instance_material_guid("instances", 1) == "passport-wall");

  SECTION("a new class gets the next id and a new type") {
    const auto r2 = apply_mask_selection(db, {0}, "Vindue");
    CHECK(r2.label_id == 3);
    CHECK(r2.label_created);
    CHECK(r2.instance_id == 4);
    CHECK(r2.type_created);
    CHECK(db.survey_type(r2.type_id)->name == "Vindue");
    CHECK(r2.resource_code == "RX-002");
  }
  SECTION("an explicit type_id wins") {
    ProjectDB::SurveyTypeRecord other;
    other.name = "Andet";
    const auto other_id = db.add_survey_type(other).id;
    MaskSelectionOptions o;
    o.type_id = other_id;
    const auto r2 = apply_mask_selection(db, {9}, "Dør", o);
    CHECK(r2.type_id == other_id);
    CHECK_FALSE(r2.type_created);
  }
  SECTION("an existing type matched by name") {
    ProjectDB::SurveyTypeRecord named;
    named.name = "Radiator";
    const auto named_id = db.add_survey_type(named).id;
    CHECK(apply_mask_selection(db, {8}, "Radiator").type_id == named_id);
  }
  SECTION("a rejected type is never picked: a new type is created") {
    // Reject the class's type and add a rejected one matching by name: both
    // automatic matches must skip them.
    db.update_survey_type(door_type, rejected_patch());
    ProjectDB::SurveyTypeRecord named;
    named.name = "Dør";
    named.review_status = core::ReviewStatus::rejected;
    const auto named_id = db.add_survey_type(named).id;
    const auto types_before = db.survey_types().size();

    const auto r2 = apply_mask_selection(db, {9}, "Dør");
    CHECK(r2.type_created);
    CHECK(r2.type_id != door_type);
    CHECK(r2.type_id != named_id);
    CHECK(db.survey_types().size() == types_before + 1);
    const auto created = db.survey_type(r2.type_id);
    REQUIRE(created);
    CHECK(created->name == "Dør");
    CHECK(created->review_status == core::ReviewStatus::queue);
    // The rejected types stay rejected.
    CHECK(db.survey_type(door_type)->review_status ==
          core::ReviewStatus::rejected);
  }
  SECTION("a rejected type as type_id is refused and writes nothing") {
    db.update_survey_type(door_type, rejected_patch());
    const auto parts_before = db.survey_parts().size();
    MaskSelectionOptions o;
    o.type_id = door_type;
    CHECK_THROWS_AS(apply_mask_selection(db, {9}, "Dør", o),
                    MaskSelectionError);
    CHECK(db.survey_parts().size() == parts_before);
    CHECK(db.instances("instances").size() == 3);
  }
}

TEST_CASE("apply_mask_selection rejects bad input and writes nothing",
          "[core][mask_selection]") {
  const TempPath tmp("mask_apply_errors");
  ProjectDB db(tmp.path);
  db.save_point_cloud("cloud", line_cloud(4));

  CHECK_THROWS_AS(apply_mask_selection(db, {0}, "  "), std::invalid_argument);
  CHECK_THROWS_AS(apply_mask_selection(db, {}, "Dør"), MaskSelectionError);
  CHECK_THROWS_AS(apply_mask_selection(db, {4}, "Dør"), std::invalid_argument);
  MaskSelectionOptions o;
  o.type_id = 4242;
  CHECK_THROWS_AS(apply_mask_selection(db, {0}, "Dør", o), std::out_of_range);
  db.save_point_cloud("labels", label_cloud({0, 0})); // out of sync
  CHECK_THROWS_AS(apply_mask_selection(db, {0}, "Dør"), MaskSelectionError);

  CHECK_FALSE(db.has_point_cloud("instances"));
  CHECK(db.survey_parts().empty());
  CHECK(db.survey_types().empty());
}

TEST_CASE("apply_mask_selection keeps cloud provenance and ignores stale "
          "class types",
          "[core][mask_selection]") {
  const TempPath tmp("mask_apply_provenance");
  ProjectDB db(tmp.path);
  db.save_point_cloud("cloud", line_cloud(4));
  db.save_point_cloud("labels", label_cloud({1, 1, 0, 0}), "reconstruct",
                      R"({"storage_order":"morton_10bit","voxel":0.05})");
  db.save_label_definitions("labels", {{1, "Væg"}});
  // A type left over from an earlier `labels` generation whose class id is
  // the one the next new class will get (2).
  ProjectDB::SurveyTypeRecord stale;
  stale.name = "Lampe";
  stale.semantic_class = 2;
  const auto stale_id = db.add_survey_type(stale).id;

  const auto r = apply_mask_selection(db, {2, 3}, "Vindue");
  CHECK(r.label_id == 2);
  CHECK(r.label_created);
  CHECK(r.type_id != stale_id); // not filed under "Lampe"
  CHECK(r.type_created);
  CHECK(db.survey_type(r.type_id)->name == "Vindue");

  const auto [stage, params] = db.point_cloud_provenance("labels");
  CHECK(stage == "reconstruct");
  CHECK(params == R"({"storage_order":"morton_10bit","voxel":0.05})");
  CHECK(db.point_cloud_storage_order("labels") == "morton_10bit");
  // A cloud created by the edit is attributed to it.
  CHECK(db.point_cloud_provenance("instances").first == "segment_resource");
}

TEST_CASE("MaskRegion_Rectangle_NormalisedUnpaddedBbox",
          "[core][mask_selection]") {
  using Catch::Matchers::WithinAbs;
  const auto r =
      reusex::core::mask_region(make_mask()); // 256², rect (100,100,80,80)
  REQUIRE(r);
  CHECK_THAT((*r)[0], WithinAbs(100.0 / 256.0, 1e-12));
  CHECK_THAT((*r)[1], WithinAbs(100.0 / 256.0, 1e-12));
  CHECK_THAT((*r)[2], WithinAbs(80.0 / 256.0, 1e-12));
  CHECK_THAT((*r)[3], WithinAbs(80.0 / 256.0, 1e-12));
  cv::Mat labels(4, 4, CV_32S, cv::Scalar(0));
  labels.at<int>(2, 1) = 7;
  CHECK(reusex::core::mask_region(labels) ==
        reusex::core::ViewRegion{0.25, 0.5, 0.25, 0.25});
  CHECK_FALSE(reusex::core::mask_region(cv::Mat(4, 4, CV_8UC1, cv::Scalar(0))));
  CHECK_THROWS_AS(
      reusex::core::mask_region(cv::Mat(4, 4, CV_8UC3, cv::Scalar(1, 1, 1))),
      std::invalid_argument);
}

TEST_CASE("ApplyMaskSelection_WithFrame_SetsPaddedView",
          "[core][mask_selection]") {
  using Catch::Matchers::WithinAbs;
  using reusex::core::ViewKind;
  const TempPath tmp("mask_apply_view");
  ProjectDB db(tmp.path);
  db.save_point_cloud("cloud", line_cloud(10));
  save_frame(db, 1);

  SECTION("from the mask region the caller passes") {
    MaskSelectionOptions opts;
    opts.frame_id = 1;
    opts.view_region = reusex::core::ViewRegion{0.375, 0.375, 0.25, 0.25};
    const auto r = apply_mask_selection(db, {0, 2}, "Dør", opts);
    REQUIRE(r.view);
    CHECK(r.view->kind == ViewKind::frame);
    CHECK(r.view->id == 1);
    REQUIRE(r.view->region);
    CHECK_THAT((*r.view->region)[0], WithinAbs(0.3375, 1e-12));
    CHECK_THAT((*r.view->region)[2], WithinAbs(0.325, 1e-12));
    CHECK(db.survey_part(r.resource_code)->view == r.view);
  }
  SECTION("without a region, from the selection projected into the frame") {
    MaskSelectionOptions opts;
    opts.frame_id = 1;
    // Points 0 and 2 (x = 0, 0.02 at z = 2): u = 64, 65; v = 64.
    const auto r = apply_mask_selection(db, {0, 2}, "Dør", opts);
    REQUIRE(r.view);
    REQUIRE(r.view->region);
    CHECK_THAT((*r.view->region)[0], WithinAbs(63.7 / 128.0, 1e-6));
    CHECK_THAT((*r.view->region)[2], WithinAbs(2.6 / 128.0, 1e-6));
  }
  SECTION("no frame, or an unknown one: no view, part still created") {
    MaskSelectionOptions none;
    CHECK_FALSE(apply_mask_selection(db, {0}, "Dør", none).view);
    MaskSelectionOptions missing;
    missing.frame_id = 99;
    const auto r = apply_mask_selection(db, {1}, "Dør", missing);
    CHECK_FALSE(r.view);
    CHECK_FALSE(db.survey_part(r.resource_code)->view);
  }
  SECTION("an invalid region is refused before anything is written") {
    MaskSelectionOptions bad;
    bad.frame_id = 1;
    bad.view_region = reusex::core::ViewRegion{0.1, 0.1, 0.0, 0.2};
    CHECK_THROWS_AS(apply_mask_selection(db, {0}, "Dør", bad),
                    std::invalid_argument);
    CHECK(db.survey_parts().empty());
  }
}
