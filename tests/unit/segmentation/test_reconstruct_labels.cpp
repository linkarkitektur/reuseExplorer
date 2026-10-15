// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// reusex-z1v.2.1: API class 0 (the first annotate prompt class) must survive
// `rux create clouds` as point label 1 and become an instance. It used to map
// to 0 == kUnlabeled, so segment_instances skipped the whole class.

#include <catch2/catch_test_macros.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/label_semantics.hpp>
#include <reusex/segmentation/reconstruct.hpp>
#include <reusex/segmentation/segment_instances.hpp>

#include "../../support/pose_fixture.hpp"
#include "../../support/temp_path.hpp"

#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>

#include <map>
#include <string>
#include <vector>

using namespace reusex;
using namespace reusex::test_support;

namespace {

constexpr int kW = 64;
constexpr int kH = 48;

geometry::ReconstructionParams test_params() {
  geometry::ReconstructionParams p;
  p.resolution = 0.05f;
  p.min_distance = 0.0f;
  p.max_distance = 4.0f;
  p.sampling_factor = 1;
  p.confidence_threshold = 2;
  return p;
}

} // namespace

TEST_CASE("ReconstructPointClouds_ApiClassZero_BecomesPointLabelOneAndInstance",
          "[segmentation][reconstruct][label_semantics]") {
  TempPath tmp("test_reconstruct_class_zero");
  ProjectDB db(tmp.path);
  db.save_sensor_frame(
      1, make_color(kW, kH), make_depth(kW, kH, 1.5), cv::Mat(),
      translation_pose(0.0, 0.0, 0.0),
      make_intrinsics(64.0, 64.0, kW / 2.0, kH / 2.0, kW, kH, identity16()));
  // Every pixel is API class 0, the first prompt class ("wall").
  db.save_segmentation_image(1, cv::Mat(kH, kW, CV_32S, cv::Scalar(0)));
  const int log = db.log_pipeline_start("annotate_class_map",
                                        R"({"0":"wall","1":"floor"})");
  db.log_pipeline_end(log, true);

  geometry::reconstruct_point_clouds(db, test_params());

  REQUIRE(db.has_point_cloud("labels"));
  auto labels = db.point_cloud_label("labels");
  REQUIRE_FALSE(labels->empty());
  size_t first_class = 0;
  for (const auto &l : *labels) {
    REQUIRE((l.label == 1u || l.label == core::kUnlabeled));
    if (l.label == 1u)
      ++first_class;
  }
  CHECK(first_class == labels->size());

  // Definitions are keyed by point label, and the cloud says so.
  CHECK(db.label_definitions("labels") ==
        std::map<int, std::string>{{1, "wall"}, {2, "floor"}});
  const auto params =
      nlohmann::json::parse(db.point_cloud_provenance("labels").second);
  CHECK(params.value(core::kLabelEncodingKey, std::string()) ==
        core::kLabelEncodingApiPlusOne);
  CHECK(params.value("storage_order", std::string()) == "morton_10bit_bitrev");

  geometry::SegmentInstancesRequest req;
  req.cloud = db.point_cloud_xyzrgb("cloud");
  req.semantic_labels = labels;
  req.min_cluster_size = 10;
  const auto result = geometry::segment_instances(req);
  REQUIRE_FALSE(result.instance_to_semantic.empty());
  for (const auto &[inst, cls] : result.instance_to_semantic)
    CHECK(cls == core::api_to_point_label(0));
}

// ── Voxel vote (reusex-z1v.2.2) ───────────────────────────────────────────
// Frames below share one pose, so every voxel collects one vote per frame for
// the same surface. The vote ignores kUnlabeled (a background frame never
// overwrites a class) and breaks an exact tie towards the lowest label; it
// used to count unlabeled as a candidate and break ties in unordered_map
// iteration order.

namespace {

/// Frames 1..N at the identity pose, frame i labelled uniformly with
/// @p api_classes[i] (kBackgroundApi for an all-background frame).
void save_uniform_frames(ProjectDB &db, const std::vector<int> &api_classes) {
  int id = 1;
  for (int api : api_classes) {
    db.save_sensor_frame(
        id, make_color(kW, kH), make_depth(kW, kH, 1.5), cv::Mat(),
        translation_pose(0.0, 0.0, 0.0),
        make_intrinsics(64.0, 64.0, kW / 2.0, kH / 2.0, kW, kH, identity16()));
    db.save_segmentation_image(id, cv::Mat(kH, kW, CV_32S, cv::Scalar(api)));
    ++id;
  }
}

/// Every stored label must be @p expected.
void require_all_labels(const ProjectDB &db, uint32_t expected) {
  REQUIRE(db.has_point_cloud("labels"));
  auto labels = db.point_cloud_label("labels");
  REQUIRE_FALSE(labels->empty());
  for (const auto &l : *labels)
    REQUIRE(l.label == expected);
}

} // namespace

TEST_CASE("ReconstructPointClouds_ClassAgainstTwoBackgroundFrames_ClassWins",
          "[segmentation][reconstruct][label_semantics]") {
  TempPath tmp("test_reconstruct_vote_bg");
  ProjectDB db(tmp.path);
  save_uniform_frames(db, {0, core::kBackgroundApi, core::kBackgroundApi});

  geometry::reconstruct_point_clouds(db, test_params());

  require_all_labels(db, core::api_to_point_label(0));
}

TEST_CASE("ReconstructPointClouds_ExactTie_LowestLabelWins",
          "[segmentation][reconstruct][label_semantics]") {
  TempPath tmp("test_reconstruct_vote_tie");
  ProjectDB db(tmp.path);
  save_uniform_frames(db, {6, 2});

  geometry::reconstruct_point_clouds(db, test_params());

  require_all_labels(db, core::api_to_point_label(2));
}

TEST_CASE("ReconstructPointClouds_TwoAgainstOne_MajorityWins",
          "[segmentation][reconstruct][label_semantics]") {
  TempPath tmp("test_reconstruct_vote_majority");
  ProjectDB db(tmp.path);
  save_uniform_frames(db, {1, 5, 5});

  geometry::reconstruct_point_clouds(db, test_params());

  require_all_labels(db, core::api_to_point_label(5));
}

TEST_CASE("ReconstructPointClouds_SeveralAnnotateRuns_LatestClassMapWins",
          "[segmentation][reconstruct][label_semantics]") {
  TempPath tmp("test_reconstruct_latest_class_map");
  ProjectDB db(tmp.path);
  save_uniform_frames(db, {0});
  const int old_run =
      db.log_pipeline_start("annotate_class_map", R"({"0":"door"})");
  db.log_pipeline_end(old_run, true);
  const int new_run = db.log_pipeline_start("annotate_class_map",
                                            R"({"0":"wall","1":"floor"})");
  db.log_pipeline_end(new_run, true);

  geometry::reconstruct_point_clouds(db, test_params());

  CHECK(db.label_definitions("labels") ==
        std::map<int, std::string>{{1, "wall"}, {2, "floor"}});
}
