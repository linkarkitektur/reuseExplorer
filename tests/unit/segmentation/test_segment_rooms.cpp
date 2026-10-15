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

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <map>
#include <random>
#include <set>
#include <tuple>
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
    CHECK(o.merge_affinity == 0.08F);
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

// ─────────────────────────────────────────────────────────────────────────
// One room per real space (reusex-5l7).
//
// NewOffice's open office came out as six rooms: modularity splits a long or
// L-shaped space whose far ends do not see each other, and desk screens cut
// the lower sight lines. The mesh stage then put a full-height wall between
// every pair of room labels. Rooms that see each other about as well as they
// see themselves are merged; rooms joined by a door are not.
// ─────────────────────────────────────────────────────────────────────────

namespace {

/// Two groups of @p n vertices, each a clique; every vertex i of group 0 sees
/// vertex i + j of group 1 for j < @p cross_per_vertex.
void two_cliques(int n, int cross_per_vertex,
                 std::vector<std::pair<int, int>> &edges,
                 std::vector<int> &membership) {
  edges.clear();
  membership.assign(static_cast<std::size_t>(2 * n), 0);
  for (int g = 0; g < 2; ++g)
    for (int i = 0; i < n; ++i) {
      membership[static_cast<std::size_t>(g * n + i)] = g;
      for (int j = i + 1; j < n; ++j)
        edges.emplace_back(g * n + i, g * n + j);
    }
  for (int i = 0; i < n; ++i)
    for (int j = 0; j < cross_per_vertex; ++j)
      edges.emplace_back(i, n + (i + j) % n);
}

} // namespace

TEST_CASE("MergeCommunitiesByVisibility_DenselyCrossVisible_Merge",
          "[geometry][segment_rooms][merge]") {
  // Each half sees half of the other: cross density 0.5 vs internal 1.0.
  std::vector<std::pair<int, int>> edges;
  std::vector<int> membership;
  two_cliques(10, 5, edges, membership);
  const std::vector<double> weights(edges.size(), 1.0);

  std::vector<pcl::CommunityMerge> log;
  const auto out = pcl::mergeCommunitiesByVisibility(
      membership.size(), edges, weights, membership, 0.3, &log);
  CHECK(out == std::vector<int>(20, 0));
  REQUIRE_FALSE(log.empty());
  CHECK(log.back().merged);
  CHECK(log.back().affinity == Catch::Approx(0.5));
}

TEST_CASE("MergeCommunitiesByVisibility_SeenOnlyThroughADoor_StaySeparate",
          "[geometry][segment_rooms][merge]") {
  // One sight line per vertex: cross density 0.1 vs internal 1.0.
  std::vector<std::pair<int, int>> edges;
  std::vector<int> membership;
  two_cliques(10, 1, edges, membership);
  const std::vector<double> weights(edges.size(), 1.0);

  const auto out = pcl::mergeCommunitiesByVisibility(membership.size(), edges,
                                                     weights, membership, 0.3);
  CHECK(out == membership);
}

TEST_CASE("MergeCommunitiesByVisibility_Disabled_ReturnsInput",
          "[geometry][segment_rooms][merge]") {
  std::vector<std::pair<int, int>> edges;
  std::vector<int> membership;
  two_cliques(6, 6, edges, membership);
  const std::vector<double> weights(edges.size(), 1.0);

  CHECK(pcl::mergeCommunitiesByVisibility(membership.size(), edges, weights,
                                          membership, 0.0) == membership);
}

TEST_CASE("MergeCommunitiesByVisibility_ChainOfMerges_RenumbersAndKeepsNone",
          "[geometry][segment_rooms][merge]") {
  // Communities 0 and 2 see each other densely, 1 is apart, vertex 9 has no
  // community. Result: {0, 2} -> 0, 1 -> 1, the unassigned vertex untouched.
  const std::vector<int> membership{0, 0, 0, 1, 1,
                                    1, 2, 2, 2, pcl::kNoCommunity};
  std::vector<std::pair<int, int>> edges{
      {0, 1}, {0, 2}, {1, 2}, {3, 4}, {3, 5}, {4, 5}, {6, 7}, {6, 8},
      {7, 8}, {0, 6}, {1, 7}, {2, 8}, {0, 7}, {1, 8}, {5, 9}};
  const std::vector<double> weights(edges.size(), 1.0);

  const auto out = pcl::mergeCommunitiesByVisibility(membership.size(), edges,
                                                     weights, membership, 0.3);
  CHECK(out == std::vector<int>{0, 0, 0, 1, 1, 1, 0, 0, 0, pcl::kNoCommunity});
}

namespace {

/// Builds an axis-aligned interior: every surface a separate plane label.
struct InteriorBuilder {
  test_support::SyntheticScene scene{std::make_shared<Cloud>(),
                                     std::make_shared<CloudN>(),
                                     std::make_shared<CloudL>()};
  std::mt19937 gen{11};
  uint32_t next_label = 1;
  float spacing = 0.1F;

  /// Rectangle origin + u + v facing @p normal (into the room).
  void patch(const Eigen::Vector3f &origin, const Eigen::Vector3f &u,
             const Eigen::Vector3f &v, const Eigen::Vector3f &normal) {
    test_support::add_patch(scene, origin, u, v, normal, spacing, 0.005F,
                            next_label++, gen);
  }
  /// Floor and ceiling of the box [x0, x1] x [y0, y1], height h.
  void slab(float x0, float y0, float x1, float y1, float h) {
    patch({x0, y0, 0.0F}, {x1 - x0, 0, 0}, {0, y1 - y0, 0}, {0, 0, 1});
    patch({x0, y0, h}, {x1 - x0, 0, 0}, {0, y1 - y0, 0}, {0, 0, -1});
  }
  /// Wall face x = const over y in [y0, y1], z in [z0, z1].
  void wall_x(float x, float y0, float y1, float z0, float z1, float nx) {
    patch({x, y0, z0}, {0, y1 - y0, 0}, {0, 0, z1 - z0}, {nx, 0, 0});
  }
  /// Wall face y = const over x in [x0, x1], z in [z0, z1].
  void wall_y(float y, float x0, float x1, float z0, float z1, float ny) {
    patch({x0, y, z0}, {x1 - x0, 0, 0}, {0, 0, z1 - z0}, {0, ny, 0});
  }
  /// A desk: a 0.75 m top over [x0, x1] x [y0, y1] and a 1.3 m screen along
  /// its long middle line, both faces scanned.
  void desk(float x0, float y0, float x1, float y1) {
    patch({x0, y0, 0.75F}, {x1 - x0, 0, 0}, {0, y1 - y0, 0}, {0, 0, 1});
    if (x1 - x0 < y1 - y0) {
      const float xm = 0.5F * (x0 + x1);
      wall_x(xm - 0.02F, y0, y1, 0.75F, 1.3F, -1.0F);
      wall_x(xm + 0.02F, y0, y1, 0.75F, 1.3F, 1.0F);
    } else {
      const float ym = 0.5F * (y0 + y1);
      wall_y(ym - 0.02F, x0, x1, 0.75F, 1.3F, -1.0F);
      wall_y(ym + 0.02F, x0, x1, 0.75F, 1.3F, 1.0F);
    }
  }
  test_support::SyntheticScene finish() {
    scene.cloud->width = static_cast<uint32_t>(scene.cloud->size());
    scene.cloud->height = 1;
    scene.normals->width = static_cast<uint32_t>(scene.normals->size());
    scene.normals->height = 1;
    scene.labels->width = static_cast<uint32_t>(scene.labels->size());
    scene.labels->height = 1;
    return scene;
  }
};

/// Room labels of the floor points (z < 0.05, normal up) inside the box
/// [x0, x1] x [y0, y1]: label -> count, unlabeled excluded.
std::map<uint32_t, std::size_t>
floor_rooms(const test_support::SyntheticScene &scene, const CloudL &rooms,
            float x0, float y0, float x1, float y1) {
  std::map<uint32_t, std::size_t> out;
  for (std::size_t i = 0; i < rooms.size(); ++i) {
    const auto &p = scene.cloud->points[i];
    if (p.z > 0.05F || scene.normals->points[i].normal_z < 0.9F)
      continue;
    if (p.x < x0 || p.x > x1 || p.y < y0 || p.y > y1)
      continue;
    if (rooms.points[i].label != core::kUnlabeled)
      ++out[rooms.points[i].label];
  }
  return out;
}

/// The most frequent label and its share of the counted points.
std::pair<uint32_t, double>
majority(const std::map<uint32_t, std::size_t> &votes) {
  std::size_t total = 0;
  std::size_t best = 0;
  uint32_t label = core::kUnlabeled;
  for (const auto &[l, c] : votes) {
    total += c;
    if (c > best) {
      best = c;
      label = l;
    }
  }
  return {label, total == 0 ? 0.0 : static_cast<double>(best) / total};
}

/// An open L-shaped office, 12 m along each arm, 4 m wide, 2.7 m high, with
/// rows of desks and screens across both arms.
test_support::SyntheticScene open_l_office() {
  InteriorBuilder b;
  const float h = 2.7F;
  b.slab(0, 0, 12, 4, h); // arm along x
  b.slab(0, 4, 4, 12, h); // arm along y
  b.wall_y(0, 0, 12, 0, h, 1);
  b.wall_x(12, 0, 4, 0, h, -1);
  b.wall_y(4, 4, 12, 0, h, -1);
  b.wall_x(4, 4, 12, 0, h, -1);
  b.wall_y(12, 0, 4, 0, h, -1);
  b.wall_x(0, 0, 12, 0, h, 1);
  // Desk rows across the x arm (screens across the arm's axis) ...
  for (const float x : {5.0F, 7.5F, 10.0F}) {
    b.desk(x - 0.8F, 0.3F, x + 0.8F, 1.7F);
    b.desk(x - 0.8F, 2.3F, x + 0.8F, 3.7F);
  }
  // ... and across the y arm.
  for (const float y : {6.0F, 8.5F, 11.0F}) {
    b.desk(0.3F, y - 0.8F, 1.7F, y + 0.8F);
    b.desk(2.3F, y - 0.8F, 3.7F, y + 0.8F);
  }
  return b.finish();
}

/// Two 5 x 4 m rooms side by side, a 0.2 m wall between them with a
/// 0.9 x 2.1 m door.
test_support::SyntheticScene two_rooms_with_door() {
  InteriorBuilder b;
  const float h = 2.7F;
  const float xa = 5.0F; // room A: x in [0, 5]
  const float xb = 5.2F; // room B: x in [5.2, 10.2]
  const float xe = 10.2F;
  b.slab(0, 0, xa, 4, h);
  b.slab(xb, 0, xe, 4, h);
  b.wall_x(0, 0, 4, 0, h, 1);
  b.wall_x(xe, 0, 4, 0, h, -1);
  for (const auto &[x0, x1] : {std::pair{0.0F, xa}, std::pair{xb, xe}}) {
    b.wall_y(0, x0, x1, 0, h, 1);
    b.wall_y(4, x0, x1, 0, h, -1);
  }
  for (const auto &[x, nx] : {std::pair{xa, -1.0F}, std::pair{xb, 1.0F}}) {
    b.wall_x(x, 0, 1.5F, 0, h, nx);
    b.wall_x(x, 2.4F, 4, 0, h, nx);
    b.wall_x(x, 1.5F, 2.4F, 2.1F, h, nx); // lintel
  }
  return b.finish();
}

} // namespace

TEST_CASE("SegmentRooms_OpenLShapedOfficeWithDeskRows_StaysOneRoom",
          "[geometry][segment_rooms][merge]") {
  const auto scene = open_l_office();
  const auto rooms = segment_rooms(scene.cloud, scene.normals, scene.labels);

  // Every part of the office's floor belongs to the same room.
  const auto [label, share] =
      majority(floor_rooms(scene, *rooms, 0, 0, 12, 12));
  INFO("floor majority room " << label << " share " << share);
  CHECK(share >= 0.95);
  for (const auto &[x0, y0, x1, y1] :
       {std::tuple{0.0F, 0.0F, 4.0F, 4.0F}, std::tuple{8.0F, 0.0F, 12.0F, 4.0F},
        std::tuple{0.0F, 8.0F, 4.0F, 12.0F}}) {
    const auto [part_label, part_share] =
        majority(floor_rooms(scene, *rooms, x0, y0, x1, y1));
    INFO("part " << x0 << "," << y0 << " room " << part_label);
    CHECK(part_label == label);
  }
}

TEST_CASE("SegmentRooms_TwoRoomsJoinedByADoor_AreTwoRooms",
          "[geometry][segment_rooms][merge]") {
  const auto scene = two_rooms_with_door();
  const auto rooms = segment_rooms(scene.cloud, scene.normals, scene.labels);

  const auto [a, share_a] = majority(floor_rooms(scene, *rooms, 0, 0, 5, 4));
  const auto [b, share_b] =
      majority(floor_rooms(scene, *rooms, 5.2F, 0, 10.2F, 4));
  CHECK(a != b);
  CHECK(share_a >= 0.9);
  CHECK(share_b >= 0.9);
}
