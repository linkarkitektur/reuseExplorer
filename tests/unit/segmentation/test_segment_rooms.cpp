// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include "../../support/synthetic_scene.hpp"

#include <reusex/core/label_semantics.hpp>
#include <reusex/reconstruction/mesh.hpp>
#include <reusex/segmentation/reconstruct.hpp>
#include <reusex/segmentation/segment_instances.hpp>
#include <reusex/segmentation/segment_planes.hpp>
#include <reusex/segmentation/segment_rooms.hpp>
#include <reusex/types.hpp>

#include <catch2/catch_test_macros.hpp>

#include <set>
#include <utility>
#include <vector>

using namespace reusex;
using namespace reusex::geometry;

namespace {

// Build a labels cloud sized to `cloud`, all unlabeled (0).
CloudLPtr make_unlabeled(CloudConstPtr cloud) {
  CloudLPtr labels(new CloudL);
  labels->resize(cloud->size());
  for (auto &p : labels->points)
    p.label = core::kUnlabeled;
  return labels;
}

PointT pt(float x, float y, float z) {
  PointT p;
  p.x = x;
  p.y = y;
  p.z = z;
  p.r = p.g = p.b = 255;
  return p;
}

} // namespace

TEST_CASE("PropagateRoomLabels_WithinRadiusMajorityVote_AssignsMajorityLabel",
          "[geometry][segment_rooms][propagation]") {
  // Seeds: a tight cluster of room-1 points around the origin (plus one
  // stray room-2 seed) and a missing point sitting inside the cluster.
  CloudPtr cloud(new Cloud);
  // 0..3 seeds (room 1), 4 seed (room 2, the "misclassified" outlier).
  cloud->push_back(pt(0.00F, 0.0F, 0.0F));
  cloud->push_back(pt(0.05F, 0.0F, 0.0F));
  cloud->push_back(pt(0.00F, 0.05F, 0.0F));
  cloud->push_back(pt(0.05F, 0.05F, 0.0F));
  cloud->push_back(pt(0.02F, 0.02F, 0.0F)); // stray room-2 seed inside cluster
  // 5: the missing point, close to all seeds.
  cloud->push_back(pt(0.025F, 0.025F, 0.0F));

  IndicesPtr sampled(new Indices{0, 1, 2, 3, 4});
  IndicesPtr missing(new Indices{5});

  auto labels = make_unlabeled(cloud);
  labels->points[0].label = 1;
  labels->points[1].label = 1;
  labels->points[2].label = 1;
  labels->points[3].label = 1;
  labels->points[4].label = 2; // outlier

  const size_t unprop =
      propagate_room_labels(cloud, labels, sampled, missing, 5, 0.5F);

  // Majority (four room-1 vs one room-2) wins: a single misclassified seed
  // does NOT smear into the missing point.
  CHECK(unprop == 0);
  CHECK(labels->points[5].label == 1U);
}

TEST_CASE("PropagateRoomLabels_PointBeyondRadius_StaysUnlabeled",
          "[geometry][segment_rooms][propagation]") {
  CloudPtr cloud(new Cloud);
  cloud->push_back(pt(0.0F, 0.0F, 0.0F)); // seed, room 1
  cloud->push_back(pt(0.1F, 0.0F, 0.0F)); // seed, room 1
  // Missing point 10 m away -> outside any sensible radius.
  cloud->push_back(pt(10.0F, 0.0F, 0.0F));

  IndicesPtr sampled(new Indices{0, 1});
  IndicesPtr missing(new Indices{2});

  auto labels = make_unlabeled(cloud);
  labels->points[0].label = 1;
  labels->points[1].label = 1;

  const size_t unprop =
      propagate_room_labels(cloud, labels, sampled, missing, 5, 0.5F);

  // No labelled neighbour within 0.5 m: stays kUnlabeled and is counted.
  CHECK(unprop == 1);
  CHECK(labels->points[2].label == core::kUnlabeled);
}

TEST_CASE("PropagateRoomLabels_TiedEquidistantSeeds_BreaksTieToSmallestLabel",
          "[geometry][segment_rooms][propagation]") {
  // Two seeds equidistant from the missing point, different labels (3 and 7).
  // The tie must break towards the smaller label (3), deterministically.
  CloudPtr cloud(new Cloud);
  cloud->push_back(pt(-0.1F, 0.0F, 0.0F)); // seed, room 7
  cloud->push_back(pt(0.1F, 0.0F, 0.0F));  // seed, room 3
  cloud->push_back(pt(0.0F, 0.0F, 0.0F));  // missing, exactly between

  IndicesPtr sampled(new Indices{0, 1});
  IndicesPtr missing(new Indices{2});

  auto run_once = [&]() {
    auto labels = make_unlabeled(cloud);
    labels->points[0].label = 7;
    labels->points[1].label = 3;
    const size_t unprop =
        propagate_room_labels(cloud, labels, sampled, missing, 5, 0.5F);
    CHECK(unprop == 0);
    return labels->points[2].label;
  };

  const uint32_t first = run_once();
  CHECK(first == 3U); // smallest label wins the tie
  // Determinism: repeated runs give identical output.
  CHECK(run_once() == first);
}

TEST_CASE(
    "PropagateRoomLabels_NearestSeedUnlabeled_IgnoresSentinelAndUsesNextSeed",
    "[geometry][segment_rooms][propagation]") {
  // Nearest seed is itself unlabeled; the majority vote must ignore it and
  // fall back to the labelled seed farther away (still in radius).
  CloudPtr cloud(new Cloud);
  cloud->push_back(pt(0.01F, 0.0F, 0.0F)); // closest seed, UNLABELED
  cloud->push_back(pt(0.20F, 0.0F, 0.0F)); // labelled seed, room 4
  cloud->push_back(pt(0.0F, 0.0F, 0.0F));  // missing

  IndicesPtr sampled(new Indices{0, 1});
  IndicesPtr missing(new Indices{2});

  auto labels = make_unlabeled(cloud);
  labels->points[0].label = core::kUnlabeled; // explicit
  labels->points[1].label = 4;

  const size_t unprop =
      propagate_room_labels(cloud, labels, sampled, missing, 5, 0.5F);

  CHECK(unprop == 0);
  CHECK(labels->points[2].label == 4U);
}

// ─────────────────────────────────────────────────────────────────────────
// Small-community dissolution (reusex-3vc).
//
// Leiden turns every isolated vertex of the visibility graph into its own
// community, and segment_rooms used to report each one as a room: NewOffice
// gave 2334 "rooms" of which 1785 were single sample points. Communities
// below the minimum room size are dissolved into the community they see most.
// ─────────────────────────────────────────────────────────────────────────

TEST_CASE("DissolveSmallCommunities_SmallCommunitySeesBigOne_JoinsMostSeen",
          "[geometry][segment_rooms][dissolve]") {
  // Community 0: vertices 0..3 (kept, size 4). Community 1: vertices 4..7
  // (kept). Community 2: vertex 8 alone, with three edges into community 0
  // and one into community 1.
  const std::vector<int> membership{0, 0, 0, 0, 1, 1, 1, 1, 2};
  const std::vector<std::pair<int, int>> edges{{0, 1}, {1, 2}, {2, 3}, {4, 5},
                                               {5, 6}, {6, 7}, {8, 0}, {8, 1},
                                               {8, 2}, {8, 4}};
  const std::vector<double> weights(edges.size(), 1.0);

  const auto out =
      pcl::dissolveSmallCommunities(9, edges, weights, membership, 2);

  REQUIRE(out.size() == 9);
  CHECK(out[8] == out[0]);
  CHECK(out[0] == 0);
  CHECK(out[4] == 1);
}

TEST_CASE("DissolveSmallCommunities_ChainOfSmallVertices_ReachesKeptCommunity",
          "[geometry][segment_rooms][dissolve]") {
  // Vertex 4 only sees vertex 5, which sees the kept community {0..3}. Both
  // small singletons must end up in it (two passes).
  const std::vector<int> membership{0, 0, 0, 0, 1, 2};
  const std::vector<std::pair<int, int>> edges{
      {0, 1}, {1, 2}, {2, 3}, {4, 5}, {5, 3}};
  const std::vector<double> weights(edges.size(), 1.0);

  const auto out =
      pcl::dissolveSmallCommunities(6, edges, weights, membership, 3);

  CHECK(out[4] == 0);
  CHECK(out[5] == 0);
}

TEST_CASE("DissolveSmallCommunities_IsolatedVertex_StaysUnassigned",
          "[geometry][segment_rooms][dissolve]") {
  const std::vector<int> membership{0, 0, 0, 1};
  const std::vector<std::pair<int, int>> edges{{0, 1}, {1, 2}};
  const std::vector<double> weights(edges.size(), 1.0);

  const auto out =
      pcl::dissolveSmallCommunities(4, edges, weights, membership, 2);

  CHECK(out[0] == 0);
  CHECK(out[3] == pcl::kNoCommunity);
}

TEST_CASE("DissolveSmallCommunities_DroppedIdsInBetween_RenumbersCompactly",
          "[geometry][segment_rooms][dissolve]") {
  // Communities 0 and 2 are kept, 1 is dropped: the result uses ids 0 and 1
  // in the original order, so room labels stay dense.
  const std::vector<int> membership{0, 0, 1, 2, 2};
  const std::vector<std::pair<int, int>> edges{{0, 1}, {3, 4}};
  const std::vector<double> weights(edges.size(), 1.0);

  const auto out =
      pcl::dissolveSmallCommunities(5, edges, weights, membership, 2);

  CHECK(out == std::vector<int>{0, 0, pcl::kNoCommunity, 1, 1});
}

TEST_CASE("DissolveSmallCommunities_EqualPull_BreaksTieToSmallestCommunity",
          "[geometry][segment_rooms][dissolve]") {
  const std::vector<int> membership{0, 0, 1, 1, 2};
  const std::vector<std::pair<int, int>> edges{{0, 1}, {2, 3}, {4, 3}, {4, 0}};
  const std::vector<double> weights(edges.size(), 1.0);

  const auto out =
      pcl::dissolveSmallCommunities(5, edges, weights, membership, 2);

  CHECK(out[4] == 0);
}

TEST_CASE("DissolveSmallCommunities_MinSizeOne_KeepsEveryCommunity",
          "[geometry][segment_rooms][dissolve]") {
  const std::vector<int> membership{0, 1, 2};
  const std::vector<std::pair<int, int>> edges{};
  const std::vector<double> weights{};

  const auto out =
      pcl::dissolveSmallCommunities(3, edges, weights, membership, 1);

  CHECK(out == membership);
}

namespace {

// A closed 4 x 3 x 2.5 m room plus a 1 x 1 m patch standing 6 m away and
// facing away from it, so the patch's samples see only each other.
test_support::SyntheticScene room_with_stray_patch(uint32_t &patch_label) {
  auto scene = test_support::make_room();
  std::mt19937 gen(7);
  patch_label = 7;
  test_support::add_patch(scene, {10.0F, 1.0F, 0.5F}, {0.0F, 1.0F, 0.0F},
                          {0.0F, 0.0F, 1.0F}, {1.0F, 0.0F, 0.0F}, 0.05F, 0.005F,
                          patch_label, gen);
  scene.cloud->width = static_cast<uint32_t>(scene.cloud->size());
  scene.normals->width = static_cast<uint32_t>(scene.normals->size());
  scene.labels->width = static_cast<uint32_t>(scene.labels->size());
  return scene;
}

} // namespace

TEST_CASE("SegmentRooms_StrayPatchSeesNothingElse_DoesNotBecomeARoom",
          "[geometry][segment_rooms][dissolve]") {
  uint32_t patch_label = 0;
  const auto scene = room_with_stray_patch(patch_label);

  const auto rooms = segment_rooms(scene.cloud, scene.normals, scene.labels);
  REQUIRE(rooms->size() == scene.cloud->size());

  std::set<uint32_t> room_labels;
  std::set<uint32_t> patch_room_labels;
  for (size_t i = 0; i < rooms->size(); ++i) {
    const uint32_t r = rooms->points[i].label;
    if (scene.labels->points[i].label == patch_label) {
      if (r != core::kUnlabeled)
        patch_room_labels.insert(r);
    } else if (r != core::kUnlabeled) {
      room_labels.insert(r);
    }
  }
  // The box is a room; the 1 m^2 patch is far below the minimum room size
  // and sees no room, so its points stay unlabeled instead of forming one.
  CHECK_FALSE(room_labels.empty());
  CHECK(patch_room_labels.empty());
}

TEST_CASE("SegmentRooms_MinRoomAreaZero_KeepsTinyCommunitiesAsRooms",
          "[geometry][segment_rooms][dissolve]") {
  uint32_t patch_label = 0;
  const auto scene = room_with_stray_patch(patch_label);

  SegmentRoomsOptions options;
  options.min_room_area = 0.0F;
  const auto rooms =
      segment_rooms(scene.cloud, scene.normals, scene.labels, options);

  size_t labelled_patch_points = 0;
  for (size_t i = 0; i < rooms->size(); ++i)
    if (scene.labels->points[i].label == patch_label &&
        rooms->points[i].label != core::kUnlabeled)
      ++labelled_patch_points;
  // Without a minimum the patch is its own community, as before the fix.
  CHECK(labelled_patch_points > 0);
}

// ─────────────────────────────────────────────────────────────────────────
// Single-source-of-defaults contract (docs/STANDARDS.md §4).
//
// The CLI option structs in apps/rux/include/create/*.hpp initialise their
// fields FROM these library structs (e.g.
// `SubcommandSegRoomsOptions::resolution =
//  reusex::geometry::SegmentRoomsOptions{}.resolution`), so the CLI can never
// disagree with the library by construction. This test pins the *library*
// side — the single source — so any accidental drift of a library default is
// caught, and documents the values that issue #217 reconciled.
// ─────────────────────────────────────────────────────────────────────────
TEST_CASE("LibraryOptionDefaults_AllPipelineStages_MatchDocumentedValues",
          "[geometry][defaults][STANDARDS]") {
  SECTION("SegmentRoomsOptions") {
    SegmentRoomsOptions o;
    CHECK(o.resolution == 1.0F);
    CHECK(o.beta == 0.01F);
    // Leiden bound is now finite (issue #217): must NOT be negative.
    CHECK(o.max_iter == 100);
    CHECK(o.max_iter > 0);
    CHECK(o.grid_size == 0.5F);
    CHECK(o.propagate_k == 5);
    CHECK(o.propagate_max_radius == 0.5F);
    CHECK(o.min_room_area == 5.0F);
  }

  SECTION("SegmentPlanesOptions") {
    SegmentPlanesOptions o;
    CHECK(o.angle_threshold == 25.0F);
    CHECK(o.plane_dist_threshold == 0.07F);
    CHECK(o.min_inliers == 1000);
    CHECK(o.radius == 0.5F);
    CHECK(o.interval_0 == 16.0F);
    CHECK(o.interval_factor == 1.5F);
  }

  SECTION("MeshOptions (search/offset reconciled to tuned CLI values)") {
    MeshOptions o;
    CHECK(o.search_threshold == 0.60F);
    CHECK(o.new_plane_offset == 0.25F);
  }

  SECTION("ReconstructionParams") {
    ReconstructionParams o;
    CHECK(o.resolution == 0.05F);
    CHECK(o.min_distance == 0.0F);
    CHECK(o.max_distance == 4.0F);
    CHECK(o.sampling_factor == 4);
    CHECK(o.confidence_threshold == 2);
  }

  SECTION("SegmentInstancesRequest") {
    SegmentInstancesRequest o;
    CHECK(o.cluster_tolerance == 0.5F);
    CHECK(o.min_cluster_size == 50);
    CHECK(o.max_cluster_size == 1000000);
  }
}
