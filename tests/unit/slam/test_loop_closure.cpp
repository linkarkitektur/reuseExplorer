// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Unit tests for the wide-baseline loop-closure front-end (issue #225, P2).
//
// The two pieces that decide whether a loop edge is real — the RANSAC 3D-3D
// relative-pose estimator and the Pairwise Consistency Maximization filter that
// rejects perceptual-aliasing false positives — are exercised directly through
// the `detail` test hooks in slam/LoopClosure.hpp (same precedent as
// `plane_factor_consistent_residual()` for the plane-graph back-end). Both are
// testable without a database or RGB-D fixtures: correspondences and poses are
// constructed by hand.
//
// PCM matters most: it is the stated safety mechanism that makes `--loop-trust`
// (a relaxed GNC-TLS inlier threshold on loop edges) defensible, so a
// regression there would silently remove the only guard against a false loop
// warping the trajectory.
//
// detect_loop_edges itself needs stored sensor frames, so only its degenerate
// input paths (which return before touching the database) are covered here.
// Fixed seeds throughout (docs/STANDARDS.md §6).

#include <reusex/core/ProjectDB.hpp>
#include <reusex/slam/LoopClosure.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "../../support/temp_path.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <random>
#include <vector>

namespace fs = std::filesystem;

using namespace reusex;
using namespace reusex::geometry;
using Catch::Matchers::WithinAbs;

namespace {

/// A rigid transform from an axis-angle rotation and a translation.
Eigen::Matrix4d rigid(double angle, const Eigen::Vector3d &axis,
                      const Eigen::Vector3d &t) {
  Eigen::Matrix4d M = Eigen::Matrix4d::Identity();
  M.block<3, 3>(0, 0) =
      Eigen::AngleAxisd(angle, axis.normalized()).toRotationMatrix();
  M.block<3, 1>(0, 3) = t;
  return M;
}

/// A planar pose: yaw about +z at (x, y, 0). Used to build a synthetic seed
/// trajectory for the PCM tests.
Eigen::Matrix4d planar_pose(double x, double y, double yaw) {
  return rigid(yaw, Eigen::Vector3d::UnitZ(), Eigen::Vector3d(x, y, 0.0));
}

/// A loop edge carrying the given measurement, with enough inliers that the
/// pcm_max_edges cap never reorders it away.
LoopEdge make_edge(int i, int j, const Eigen::Matrix4d &T_ij, int inliers) {
  LoopEdge e;
  e.i = i;
  e.j = j;
  e.T_ij = T_ij;
  e.inliers = inliers;
  return e;
}

/// Temp project database path that auto-cleans.
struct TempDB : reusex::test_support::TempPath {
  TempDB() : TempPath("test_loop_closure") {}
};

} // namespace

// ---------------------------------------------------------------------------
// RANSAC 3D-3D relative pose
// ---------------------------------------------------------------------------

TEST_CASE("RansacRigid_CorrespondencesWithOutliers_RecoversTrueTransform",
          "[loop_closure][slam][ransac]") {
  // 60 exact correspondences under a known transform, plus 20 gross outliers
  // (offset by ~5 m, far beyond any inlier threshold). RANSAC must find the
  // true transform and label exactly the 60 good correspondences as inliers.
  const Eigen::Matrix4d truth =
      rigid(0.35, Eigen::Vector3d(0.2, -0.5, 1.0), {0.4, -0.2, 1.1});
  const Eigen::Matrix3d R = truth.block<3, 3>(0, 0);
  const Eigen::Vector3d t = truth.block<3, 1>(0, 3);

  const int n_in = 60, n_out = 20;
  std::mt19937 gen(7);
  std::uniform_real_distribution<double> u(-2.0, 2.0);

  Eigen::Matrix3Xd src(3, n_in + n_out), dst(3, n_in + n_out);
  for (int k = 0; k < n_in + n_out; ++k) {
    const Eigen::Vector3d p(u(gen), u(gen), u(gen));
    src.col(k) = p;
    dst.col(k) = R * p + t;
    if (k >= n_in) // corrupt the tail: a mismatched correspondence
      dst.col(k) += Eigen::Vector3d(3.0, -2.5, 4.0);
  }

  LoopClosureOptions opt;
  opt.ransac_inlier_dist = 0.02f; // correspondences are exact
  opt.ransac_iterations = 500;

  std::mt19937 rng(opt.seed);
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  const std::vector<int> inliers = detail::ransac_rigid(src, dst, opt, rng, T);

  INFO("inliers=" << inliers.size() << " (expected " << n_in << ")");
  REQUIRE(static_cast<int>(inliers.size()) == n_in);
  // Only the uncorrupted correspondences may be inliers.
  for (int idx : inliers)
    REQUIRE(idx < n_in);
  // And the refit on the full inlier set must reproduce the truth exactly.
  REQUIRE_THAT((T - truth).norm(), WithinAbs(0.0, 1e-9));
}

TEST_CASE("RansacRigid_TooFewCorrespondences_ReturnsNoInliers",
          "[loop_closure][slam][ransac]") {
  // Degenerate input: fewer than the 3 correspondences a rigid fit needs. Must
  // return no inliers and leave the caller's transform untouched (rather than
  // returning a fabricated pose).
  LoopClosureOptions opt;
  std::mt19937 rng(opt.seed);

  for (int n : {0, 1, 2}) {
    Eigen::Matrix3Xd src(3, n), dst(3, n);
    for (int k = 0; k < n; ++k) {
      src.col(k) = Eigen::Vector3d(k, 0.0, 0.0);
      dst.col(k) = Eigen::Vector3d(k, 1.0, 0.0);
    }
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    const std::vector<int> inliers =
        detail::ransac_rigid(src, dst, opt, rng, T);
    INFO("n=" << n);
    REQUIRE(inliers.empty());
    REQUIRE((T - Eigen::Matrix4d::Identity()).norm() == 0.0);
  }
}

// ---------------------------------------------------------------------------
// Pairwise Consistency Maximization
// ---------------------------------------------------------------------------

TEST_CASE("PcmFilter_MixedConsistentAndInconsistentEdges_KeepsOnlyConsistent",
          "[loop_closure][slam][pcm]") {
  // A synthetic 6-frame seed trajectory. An edge whose measurement equals the
  // seed relative pose closes the PCM cycle exactly (identity), so any set of
  // such edges is mutually consistent; an edge with a bogus measurement does
  // not close with any of them.
  std::vector<Eigen::Matrix4d> seed;
  for (int k = 0; k < 6; ++k)
    seed.push_back(planar_pose(k * 1.0, 0.2 * k, 0.15 * k));

  auto seed_rel = [&](int i, int j) {
    return Eigen::Matrix4d(seed[i].inverse() * seed[j]);
  };

  SECTION("a translation-inconsistent edge is rejected") {
    std::vector<LoopEdge> edges{
        make_edge(0, 3, seed_rel(0, 3), 100),
        make_edge(1, 4, seed_rel(1, 4), 95),
        make_edge(2, 5, seed_rel(2, 5), 90),
    };
    // 1 m off — well beyond pcm_trans_threshold (0.30 m).
    Eigen::Matrix4d bogus = seed_rel(0, 5);
    bogus.block<3, 1>(0, 3) += Eigen::Vector3d(1.0, 0.0, 0.0);
    edges.push_back(make_edge(0, 5, bogus, 85));

    LoopClosureOptions opt;
    opt.pcm_max_edges = 0; // no cap: all four edges enter the O(M^2) test

    const std::vector<LoopEdge> kept = detail::pcm_filter(edges, seed, opt);

    INFO("kept " << kept.size() << " of " << edges.size());
    REQUIRE(kept.size() == 3);
    for (const auto &e : kept)
      REQUIRE_FALSE((e.i == 0 && e.j == 5)); // the bogus edge is gone
  }

  SECTION("a rotation-inconsistent edge is rejected") {
    std::vector<LoopEdge> edges{
        make_edge(0, 3, seed_rel(0, 3), 100),
        make_edge(1, 4, seed_rel(1, 4), 95),
        make_edge(2, 5, seed_rel(2, 5), 90),
    };
    // 0.4 rad of spurious rotation on an otherwise seed-consistent
    // measurement: the cycle no longer closes rotationally (0.4 rad is beyond
    // pcm_rot_threshold = 0.15 rad), so the edge must be voted out.
    const Eigen::Matrix4d bogus =
        seed_rel(0, 5) *
        rigid(0.4, Eigen::Vector3d::UnitY(), Eigen::Vector3d::Zero());
    edges.push_back(make_edge(0, 5, bogus, 85));

    LoopClosureOptions opt;
    opt.pcm_max_edges = 0;

    const std::vector<LoopEdge> kept = detail::pcm_filter(edges, seed, opt);
    REQUIRE(kept.size() == 3);
    for (const auto &e : kept)
      REQUIRE_FALSE((e.i == 0 && e.j == 5));
  }

  SECTION("a fully consistent set is kept whole") {
    // Control: the same machinery must not discard anything when every edge
    // agrees, otherwise the "rejected" result above would prove nothing.
    const std::vector<LoopEdge> edges{
        make_edge(0, 3, seed_rel(0, 3), 100),
        make_edge(1, 4, seed_rel(1, 4), 95),
        make_edge(2, 5, seed_rel(2, 5), 90),
        make_edge(0, 5, seed_rel(0, 5), 85),
    };
    LoopClosureOptions opt;
    opt.pcm_max_edges = 0;

    const std::vector<LoopEdge> kept = detail::pcm_filter(edges, seed, opt);
    REQUIRE(kept.size() == edges.size());
  }

  SECTION("two or fewer edges are returned unchanged") {
    // Documented behaviour: a pair cannot outvote itself, so PCM abstains.
    // detect_loop_edges relies on this (it only calls PCM above 2 edges).
    Eigen::Matrix4d bogus = seed_rel(0, 5);
    bogus.block<3, 1>(0, 3) += Eigen::Vector3d(1.0, 0.0, 0.0);
    const std::vector<LoopEdge> edges{make_edge(0, 3, seed_rel(0, 3), 100),
                                      make_edge(0, 5, bogus, 85)};
    LoopClosureOptions opt;
    opt.pcm_max_edges = 0;

    REQUIRE(detail::pcm_filter(edges, seed, opt).size() == 2);
    REQUIRE(detail::pcm_filter({}, seed, opt).empty());
  }

  SECTION("pcm_max_edges caps the set to the highest-inlier edges") {
    std::vector<LoopEdge> edges{
        make_edge(0, 3, seed_rel(0, 3), 10),
        make_edge(1, 4, seed_rel(1, 4), 200),
        make_edge(2, 5, seed_rel(2, 5), 150),
        make_edge(0, 5, seed_rel(0, 5), 20),
    };
    LoopClosureOptions opt;
    opt.pcm_max_edges = 2; // <= 2 edges survive the cap -> returned unchanged

    const std::vector<LoopEdge> kept = detail::pcm_filter(edges, seed, opt);
    REQUIRE(kept.size() == 2);
    REQUIRE(kept[0].inliers == 200);
    REQUIRE(kept[1].inliers == 150);
  }
}

// ---------------------------------------------------------------------------
// filter_consistent_loop_edges — the public entry point optimize_sensor_poses
// uses to PCM-filter the UNION of internally-detected and external
// (--loop-edges) edges. Before this existed, external edges reached the graph
// with no consistency filter at all, which is what made --loop-trust unsafe.
// ---------------------------------------------------------------------------

TEST_CASE("FilterConsistentLoopEdges_MixedInternalAndExternalEdges_"
          "RejectsInconsistentEdge",
          "[loop_closure][slam][pcm]") {
  std::vector<Eigen::Matrix4d> seed;
  for (int k = 0; k < 6; ++k)
    seed.push_back(planar_pose(k * 1.0, 0.2 * k, 0.15 * k));

  auto seed_rel = [&](int i, int j) {
    return Eigen::Matrix4d(seed[i].inverse() * seed[j]);
  };

  // Three "internal" edges that agree with the seed, plus one "external" edge
  // whose measurement is 1 m off — the aliasing false positive the review
  // flagged as unfiltered.
  std::vector<LoopEdge> unioned{
      make_edge(0, 3, seed_rel(0, 3), 100),
      make_edge(1, 4, seed_rel(1, 4), 95),
      make_edge(2, 5, seed_rel(2, 5), 90),
  };
  Eigen::Matrix4d bogus = seed_rel(0, 5);
  bogus.block<3, 1>(0, 3) += Eigen::Vector3d(1.0, 0.0, 0.0);
  unioned.push_back(make_edge(0, 5, bogus, 85));

  LoopClosureOptions opt;
  opt.pcm_max_edges = 0;

  const std::vector<LoopEdge> kept =
      filter_consistent_loop_edges(unioned, seed, opt);

  REQUIRE(kept.size() == 3);
  for (const auto &e : kept)
    REQUIRE_FALSE((e.i == 0 && e.j == 5));

  // The public wrapper must be exactly the detail:: implementation, not a
  // second copy that could drift from it.
  const std::vector<LoopEdge> via_detail =
      detail::pcm_filter(unioned, seed, opt);
  REQUIRE(kept.size() == via_detail.size());
}

// ---------------------------------------------------------------------------
// detect_loop_edges degenerate inputs
// ---------------------------------------------------------------------------

TEST_CASE("DetectLoopEdges_DegenerateInputs_ReturnsEmptyWithoutCrash",
          "[loop_closure][slam][degenerate]") {
  // These paths must return an empty edge set without crashing and without
  // touching the database — a caller that hands over a single frame, no frames,
  // or mismatched arrays gets nothing rather than an exception or a bad read.
  TempDB tmp;
  ProjectDB db(tmp.path);

  LoopClosureOptions opt;
  opt.enable = true;
  opt.min_frame_gap = 1;

  SECTION("no frames") {
    LoopClosureResult stats;
    REQUIRE(detect_loop_edges(db, {}, {}, opt, &stats).empty());
    REQUIRE(stats.edges == 0);
    REQUIRE(stats.candidates == 0);
  }

  SECTION("a single frame cannot form a pair") {
    const std::vector<int> nodes{1};
    const std::vector<Eigen::Matrix4d> poses{Eigen::Matrix4d::Identity()};
    REQUIRE(detect_loop_edges(db, nodes, poses, opt, nullptr).empty());
  }

  SECTION("node_ids / seed_poses size mismatch") {
    const std::vector<int> nodes{1, 2, 3};
    const std::vector<Eigen::Matrix4d> poses{Eigen::Matrix4d::Identity(),
                                             Eigen::Matrix4d::Identity()};
    LoopClosureResult stats;
    REQUIRE(detect_loop_edges(db, nodes, poses, opt, &stats).empty());
    REQUIRE(stats.edges == 0);
  }

  SECTION("frames that carry no stored imagery yield no edges") {
    // Enough frames to pass the size guard, but the database holds no sensor
    // frames at all: feature extraction yields nothing for every frame, so the
    // candidate loop must degrade gracefully to zero edges.
    const std::vector<int> nodes{1, 2, 3};
    const std::vector<Eigen::Matrix4d> poses(3, Eigen::Matrix4d::Identity());
    LoopClosureResult stats;
    REQUIRE(detect_loop_edges(db, nodes, poses, opt, &stats).empty());
    REQUIRE(stats.edges == 0);
    REQUIRE(stats.total_inliers == 0);
  }
}

// ---------------------------------------------------------------------------
// PCM with fixed inliers (consistent_loop_edge_indices)
// ---------------------------------------------------------------------------

TEST_CASE("ConsistentLoopEdgeIndices_FixedEdge_OverridesALargerAliasingClique",
          "[loop_closure][slam][pcm]") {
  // Six frames 1 m apart on x. True loops (0,4), (1,5) agree with the seed;
  // three aliasing edges (0,3), (1,4), (2,5) are each 2 m off in the same
  // way, so they agree with EACH OTHER and outvote the true pair — unless a
  // true edge is fixed (a user-confirmed stored edge, spec §4.3).
  std::vector<Eigen::Matrix4d> seed;
  for (int k = 0; k < 6; ++k)
    seed.push_back(planar_pose(1.0 * k, 0.0, 0.0));
  auto rel = [&](int i, int j) {
    return Eigen::Matrix4d(seed[i].inverse() * seed[j]);
  };
  Eigen::Matrix4d off = Eigen::Matrix4d::Identity();
  off(0, 3) = 2.0;
  const std::vector<LoopEdge> edges = {make_edge(0, 4, rel(0, 4), 100),
                                       make_edge(1, 5, rel(1, 5), 100),
                                       make_edge(0, 3, rel(0, 3) * off, 100),
                                       make_edge(1, 4, rel(1, 4) * off, 100),
                                       make_edge(2, 5, rel(2, 5) * off, 100)};
  const LoopClosureOptions opt;

  const auto free = consistent_loop_edge_indices(edges, seed, opt, {});
  CHECK(free == std::vector<size_t>{2, 3, 4});

  const auto fixed = consistent_loop_edge_indices(
      edges, seed, opt, {true, false, false, false, false});
  CHECK(fixed == std::vector<size_t>{0, 1});
}

TEST_CASE("ConsistentLoopEdgeIndices_FixedEdgesSurviveTheCap",
          "[loop_closure][slam][pcm]") {
  std::vector<Eigen::Matrix4d> seed;
  for (int k = 0; k < 6; ++k)
    seed.push_back(planar_pose(1.0 * k, 0.0, 0.0));
  auto rel = [&](int i, int j) {
    return Eigen::Matrix4d(seed[i].inverse() * seed[j]);
  };
  const std::vector<LoopEdge> edges = {make_edge(0, 4, rel(0, 4), 1),
                                       make_edge(1, 5, rel(1, 5), 500),
                                       make_edge(0, 5, rel(0, 5), 400)};
  LoopClosureOptions opt;
  opt.pcm_max_edges = 1; // would keep only the 500-inlier edge
  const auto kept =
      consistent_loop_edge_indices(edges, seed, opt, {true, false, false});
  CHECK(std::find(kept.begin(), kept.end(), size_t{0}) != kept.end());
}

TEST_CASE("ConsistentLoopEdgeIndices_SingleCandidateInconsistentWithFixed_"
          "IsRejected",
          "[loop_closure][slam][pcm]") {
  // Two edges only: without a fixed edge the "a pair cannot outvote itself"
  // rule keeps both, but a candidate that contradicts a fixed (confirmed)
  // edge is rejected even below the clique-search threshold (ruling R9).
  std::vector<Eigen::Matrix4d> seed;
  for (int k = 0; k < 6; ++k)
    seed.push_back(planar_pose(1.0 * k, 0.0, 0.0));
  auto rel = [&](int i, int j) {
    return Eigen::Matrix4d(seed[i].inverse() * seed[j]);
  };
  Eigen::Matrix4d off = Eigen::Matrix4d::Identity();
  off(0, 3) = 2.0;
  const std::vector<LoopEdge> edges = {make_edge(0, 4, rel(0, 4), 100),
                                       make_edge(0, 3, rel(0, 3) * off, 100)};
  const LoopClosureOptions opt;

  CHECK(consistent_loop_edge_indices(edges, seed, opt, {}) ==
        std::vector<size_t>{0, 1});
  CHECK(consistent_loop_edge_indices(edges, seed, opt, {true, false}) ==
        std::vector<size_t>{0});
  // A consistent single candidate is still kept alongside the fixed edge.
  const std::vector<LoopEdge> agree = {make_edge(0, 4, rel(0, 4), 100),
                                       make_edge(1, 5, rel(1, 5), 100)};
  CHECK(consistent_loop_edge_indices(agree, seed, opt, {true, false}) ==
        std::vector<size_t>{0, 1});
}
