// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <catch2/catch_test_macros.hpp>

#include <core/ProjectDB.hpp>
#include <reusex/segmentation/downsample.hpp>
#include <reusex/segmentation/sync_downsample.hpp>
#include <types.hpp>

#include "../../support/synthetic_scene.hpp"
#include "../../support/temp_path.hpp"

#include <filesystem>
#include <string>
#include <unordered_map>

using namespace reusex;
using namespace reusex::geometry;
namespace fs = std::filesystem;

namespace {

struct TempDB : reusex::test_support::TempPath {
  TempDB() : TempPath("test_sync_downsample") {}
};

// A tiny cloud with two well-separated clusters (leaf=10 => two buckets), so
// bucket membership is unambiguous and float-precision safe.
CloudPtr two_cluster_cloud() {
  auto c = std::make_shared<Cloud>();
  const std::array<std::array<float, 3>, 4> a = {{{15.5f, 15.5f, 15.5f},
                                                  {15.1f, 15.3f, 15.4f},
                                                  {15.9f, 15.7f, 15.6f},
                                                  {15.4f, 15.5f, 15.2f}}};
  const std::array<std::array<float, 3>, 4> b = {{{37.3f, 37.3f, 37.3f},
                                                  {36.9f, 37.1f, 37.2f},
                                                  {37.7f, 37.5f, 37.4f},
                                                  {37.2f, 37.3f, 37.0f}}};
  for (const auto &p : a) {
    PointT pt;
    pt.x = p[0];
    pt.y = p[1];
    pt.z = p[2];
    pt.r = pt.g = pt.b = 10;
    c->push_back(pt);
  }
  for (const auto &p : b) {
    PointT pt;
    pt.x = p[0];
    pt.y = p[1];
    pt.z = p[2];
    pt.r = pt.g = pt.b = 20;
    c->push_back(pt);
  }
  c->width = static_cast<uint32_t>(c->size());
  c->height = 1;
  return c;
}

} // namespace

// ── Label majority-vote overload ────────────────────────────────────────

TEST_CASE("DownsampleLabels_MixedBucketWithMajority_AssignsMajorityLabel",
          "[sync_downsample]") {
  auto cloud = two_cluster_cloud();
  auto a = voxel_assignment(*cloud, 10.0f);
  REQUIRE(a.bucket_count == 2);

  // Bucket A (rows 0..3): labels {7,7,7,3} -> majority 7
  // Bucket B (rows 4..7): labels {2,2,5,5} -> tie -> lowest label 2
  CloudL labels;
  const std::array<uint32_t, 8> vals = {7, 7, 7, 3, 2, 2, 5, 5};
  for (uint32_t v : vals) {
    LabelT l;
    l.label = v;
    labels.push_back(l);
  }

  auto out = downsample(labels, a);
  REQUIRE(out);
  REQUIRE(out->size() == 2);

  // Map bucket id -> expected majority using the primary centroids for order.
  auto down = downsample(*cloud, a);
  size_t aRow = (*down)[0].x < 25.0f ? 0 : 1;
  size_t bRow = 1 - aRow;

  REQUIRE((*out)[aRow].label == 7u);
  REQUIRE((*out)[bRow].label == 2u); // deterministic tie-break: lowest label
}

TEST_CASE("DownsampleLabels_TiedBucketLabels_BreaksTieToLowestLabel",
          "[sync_downsample]") {
  auto cloud = two_cluster_cloud();
  auto a = voxel_assignment(*cloud, 10.0f);

  // Both buckets are perfect ties among distinct labels.
  CloudL labels;
  const std::array<uint32_t, 8> vals = {9, 4, 4, 9, 8, 8, 1, 1};
  for (uint32_t v : vals) {
    LabelT l;
    l.label = v;
    labels.push_back(l);
  }
  auto out = downsample(labels, a);
  REQUIRE(out->size() == 2);
  // Bucket A {9,4,4,9} -> tie {4,9} -> 4 ; Bucket B {8,8,1,1} -> tie {1,8} -> 1
  auto down = downsample(*cloud, a);
  size_t aRow = (*down)[0].x < 25.0f ? 0 : 1;
  size_t bRow = 1 - aRow;
  REQUIRE((*out)[aRow].label == 4u);
  REQUIRE((*out)[bRow].label == 1u);
}

TEST_CASE("DownsampleLabels_MismatchedSize_Throws", "[sync_downsample]") {
  auto cloud = two_cluster_cloud();
  auto a = voxel_assignment(*cloud, 10.0f);
  CloudL labels;
  LabelT l;
  l.label = 1;
  labels.push_back(l);
  REQUIRE_THROWS_AS(downsample(labels, a), std::invalid_argument);
}

// ── Synchronized project-level downsample ───────────────────────────────

TEST_CASE("SyncDownsample_MultipleSiblingClouds_KeepsAllSiblingsIndexAligned",
          "[sync_downsample]") {
  TempDB tmp;
  ProjectDB db(tmp.path);

  auto scene = reusex::test_support::make_room();
  const size_t n = scene.cloud->size();
  REQUIRE(scene.normals->size() == n);
  REQUIRE(scene.labels->size() == n);

  db.save_point_cloud("cloud", *scene.cloud, "import");
  db.save_point_cloud("normals", *scene.normals, "import");
  db.save_point_cloud("labels", *scene.labels, "import",
                      R"({"label_encoding":"api_plus_one"})");
  db.save_label_definitions("labels", {{1, "floor"},
                                       {2, "ceiling"},
                                       {3, "wall"},
                                       {4, "wall"},
                                       {5, "wall"},
                                       {6, "wall"}});

  // Compute the ground-truth expectation independently.
  auto assignment = voxel_assignment(*scene.cloud, 0.10f);
  auto expected_labels = downsample(*scene.labels, assignment);

  SyncDownsampleOptions opts;
  opts.leaf_size = 0.10f;
  auto result = sync_downsample(db, "cloud", opts);

  // Primary + two siblings written.
  REQUIRE(result.written_clouds.size() == 3);
  REQUIRE(result.desynced_clouds.empty());

  auto out_cloud = db.point_cloud_xyzrgb("cloud");
  auto out_normals = db.point_cloud_normal("normals");
  auto out_labels = db.point_cloud_label("labels");
  REQUIRE(out_cloud);
  REQUIRE(out_normals);
  REQUIRE(out_labels);

  const size_t m = out_cloud->size();
  REQUIRE(m == assignment.bucket_count);
  REQUIRE(out_normals->size() == m);
  REQUIRE(out_labels->size() == m);
  REQUIRE(expected_labels->size() == m);

  // Every downsampled label must match the independently-computed vote.
  for (size_t i = 0; i < m; ++i)
    REQUIRE((*out_labels)[i].label == (*expected_labels)[i].label);

  // Label definitions carried over.
  auto defs = db.label_definitions("labels");
  REQUIRE(defs.size() == 6);
  REQUIRE(defs.at(1) == "floor");
  // ...and so does the semantic cloud's label encoding (STANDARDS §3.1).
  CHECK(db.point_cloud_provenance("labels").second.find(
            R"("label_encoding":"api_plus_one")") != std::string::npos);

  // Lineage recorded.
  auto log = db.pipeline_log();
  bool found = false;
  for (const auto &e : log)
    if (e.stage == "sync_downsample" && e.status == "success")
      found = true;
  REQUIRE(found);
}

TEST_CASE("SyncDownsample_OnlyPrimaryWithoutForceDesync_"
          "ThrowsAndLeavesSiblingsUntouched",
          "[sync_downsample]") {
  TempDB tmp;
  ProjectDB db(tmp.path);

  auto scene = reusex::test_support::make_room();
  db.save_point_cloud("cloud", *scene.cloud, "import");
  db.save_point_cloud("normals", *scene.normals, "import");
  db.save_point_cloud("labels", *scene.labels, "import");

  SyncDownsampleOptions opts;
  opts.leaf_size = 0.10f;
  opts.only_primary = true;
  // No force_desync -> must throw and leave siblings untouched.
  REQUIRE_THROWS_AS(sync_downsample(db, "cloud", opts), std::runtime_error);

  // Siblings still at their original (full) size.
  REQUIRE(db.point_cloud_normal("normals")->size() == scene.normals->size());
  REQUIRE(db.point_cloud_label("labels")->size() == scene.labels->size());
}

TEST_CASE("SyncDownsample_OnlyPrimaryWithForceDesync_LeavesSiblingsStale",
          "[sync_downsample]") {
  TempDB tmp;
  ProjectDB db(tmp.path);

  auto scene = reusex::test_support::make_room();
  const size_t sib_n = scene.normals->size();
  db.save_point_cloud("cloud", *scene.cloud, "import");
  db.save_point_cloud("normals", *scene.normals, "import");

  SyncDownsampleOptions opts;
  opts.leaf_size = 0.10f;
  opts.only_primary = true;
  opts.force_desync = true;
  auto result = sync_downsample(db, "cloud", opts);

  REQUIRE(result.written_clouds.size() == 1); // only primary
  REQUIRE(result.desynced_clouds.size() == 1);
  REQUIRE(result.desynced_clouds[0] == "normals");

  // Primary shrank; sibling deliberately left stale at original size.
  REQUIRE(db.point_cloud_xyzrgb("cloud")->size() < sib_n);
  REQUIRE(db.point_cloud_normal("normals")->size() == sib_n);
}

TEST_CASE("SyncDownsample_MissingOrWrongTypePrimary_Throws",
          "[sync_downsample]") {
  TempDB tmp;
  ProjectDB db(tmp.path);

  SyncDownsampleOptions opts;
  opts.leaf_size = 0.10f;
  REQUIRE_THROWS_AS(sync_downsample(db, "cloud", opts), std::invalid_argument);
}
