// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Unit tests for the noise-floor acceptance gate of `rux align 360`
// (reusex-bx2.4).
//
// docs/research/panorama-alignment-failure-modes.md showed that the per-slice
// PnP of a panorama whose 2D<->3D pairings were randomly SHUFFLED still reaches
// 0-17 inliers, comfortably above the old `min_slice_correspondences = 6`
// gate, so pure noise could seed an "aligned" pose. The fix measures that null
// distribution per panorama (the same slices, the same correspondence counts,
// the world points permuted) and requires the real best slice to beat it by a
// margin. These tests pin both directions on synthetic correspondences:
//
//   - pure-noise correspondences (random pairs, or a genuine set shuffled)
//     are rejected;
//   - a genuine slice (a known pose, half its correspondences junk) is
//     accepted, and the resected pose is the true one.
//
// Includes the module-private `slam/panorama_features.hpp` directly — same
// arrangement as test_panorama_features_pose.cpp (STANDARDS §2).

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <reusex/slam/PanoramaAlignment.hpp>

#include "slam/panorama_features.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <random>
#include <string>
#include <vector>

using namespace reusex::geometry;
using namespace reusex::geometry::pano_detail;
using Catch::Matchers::WithinAbs;

namespace {

constexpr int kSlicePx = 1008; // the production slice size
constexpr int kSlices = 4;     // 4 x 90 deg around the equator

/// The virtual pinhole of slice @p i: 90 deg FOV, yawed i * 90 deg about the
/// panorama's y (down) axis — the geometry overlapping_views() produces.
SlicePnPInput empty_slice(int i) {
  SlicePnPInput s;
  const double f = 0.5 * kSlicePx; // tan(45 deg) = 1
  s.K << f, 0, 0.5 * kSlicePx, 0, f, 0.5 * kSlicePx, 0, 0, 1;
  s.R_pano_from_view =
      Eigen::AngleAxisd(i * M_PI / 2.0, Eigen::Vector3d::UnitY())
          .toRotationMatrix();
  return s;
}

/// A junk correspondence: a random slice pixel paired with a random world
/// point in a 12 m box around the origin — what the Lowe ratio test lets
/// through from slices no candidate frame sees.
void add_junk(SlicePnPInput &s, int n, std::mt19937 &rng) {
  std::uniform_real_distribution<double> px(0.0, kSlicePx);
  std::uniform_real_distribution<double> w(-6.0, 6.0);
  for (int k = 0; k < n; ++k) {
    s.pixels.emplace_back(static_cast<float>(px(rng)),
                          static_cast<float>(px(rng)));
    s.world.emplace_back(w(rng), w(rng), w(rng));
  }
}

/// Clustered junk, closer to what the matcher really emits than uniform
/// pairs: the slice keypoints bunch on a few textured blobs and the frame
/// points they mis-match to bunch on the few surfaces the candidate frames
/// see. A random pose that maps one world blob onto one pixel blob then
/// "explains" a dozen pairs by chance, which is how pure noise beat the old
/// fixed gate on NewOffice (best-slice 0-17 inliers under the shuffle).
void add_clustered_junk(SlicePnPInput &s, int n, std::mt19937 &rng) {
  constexpr int kBlobs = 3;
  std::uniform_real_distribution<double> centre_px(150.0, kSlicePx - 150.0);
  std::uniform_real_distribution<double> centre_w(-4.0, 4.0);
  std::normal_distribution<double> spread_px(0.0, 25.0);
  std::normal_distribution<double> spread_w(0.0, 0.15);
  std::uniform_int_distribution<int> pick(0, kBlobs - 1);
  std::vector<Eigen::Vector2d> pc;
  std::vector<Eigen::Vector3d> wc;
  for (int b = 0; b < kBlobs; ++b) {
    pc.emplace_back(centre_px(rng), centre_px(rng));
    wc.emplace_back(centre_w(rng), centre_w(rng), centre_w(rng));
  }
  for (int k = 0; k < n; ++k) {
    const Eigen::Vector2d &p = pc[pick(rng)];
    const Eigen::Vector3d &w = wc[pick(rng)];
    s.pixels.emplace_back(static_cast<float>(p.x() + spread_px(rng)),
                          static_cast<float>(p.y() + spread_px(rng)));
    s.world.emplace_back(w.x() + spread_w(rng), w.y() + spread_w(rng),
                         w.z() + spread_w(rng));
  }
}

struct Pose {
  Eigen::Matrix3d Q; ///< pano_from_world
  Eigen::Vector3d c; ///< panorama centre in world
};

Pose true_pose() {
  return {Eigen::AngleAxisd(0.4, Eigen::Vector3d(0.2, 1.0, 0.1).normalized())
              .toRotationMatrix(),
          Eigen::Vector3d(1.5, -0.3, 2.0)};
}

/// Genuine correspondences seen by slice @p s under @p pose: a pixel, a depth
/// of 2-6 m, back-projected to the world point it observes (plus 0.5 px of
/// pixel noise).
void add_genuine(SlicePnPInput &s, const Pose &pose, int n, std::mt19937 &rng) {
  std::uniform_real_distribution<double> px(20.0, kSlicePx - 20.0);
  std::uniform_real_distribution<double> depth(2.0, 6.0);
  std::normal_distribution<double> noise(0.0, 0.5);
  for (int k = 0; k < n; ++k) {
    const double u = px(rng), v = px(rng);
    const Eigen::Vector3d ray = s.K.inverse() * Eigen::Vector3d(u, v, 1.0);
    const Eigen::Vector3d p_view = depth(rng) * ray;
    const Eigen::Vector3d p_pano = s.R_pano_from_view * p_view;
    s.world.push_back(pose.Q.transpose() * p_pano + pose.c);
    s.pixels.emplace_back(static_cast<float>(u + noise(rng)),
                          static_cast<float>(v + noise(rng)));
  }
}

/// One genuine slice (100 true + 100 junk) among three junk-only slices.
std::vector<SlicePnPInput> genuine_scene(unsigned seed = 7) {
  std::mt19937 rng(seed);
  std::vector<SlicePnPInput> slices;
  for (int i = 0; i < kSlices; ++i) {
    slices.push_back(empty_slice(i));
    if (i == 1)
      add_genuine(slices.back(), true_pose(), 100, rng);
    add_clustered_junk(slices.back(), i == 1 ? 100 : 400, rng);
  }
  return slices;
}

/// The research null model applied once: each slice's world points permuted
/// among that slice's own correspondences. Same counts, same pixel and world
/// distributions, no geometry.
std::vector<SlicePnPInput> shuffled(std::vector<SlicePnPInput> slices,
                                    unsigned seed) {
  std::mt19937 rng(seed);
  for (auto &s : slices)
    std::shuffle(s.world.begin(), s.world.end(), rng);
  return slices;
}

SlicePnPOptions pnp_options() {
  const PanoramaAlignmentOptions lib; // gate defaults come from the library
  SlicePnPOptions o;
  o.min_correspondences = lib.min_slice_correspondences;
  o.ransac_iterations = lib.ransac_iterations;
  o.ransac_reproj_px = lib.ransac_reproj_px;
  return o;
}

bool accepted(const std::vector<SlicePnPInput> &slices, int *best = nullptr,
              int *needed = nullptr) {
  const PanoramaAlignmentOptions lib;
  const SliceResection r = resect_best_slice(slices, pnp_options());
  const int null_best = shuffled_best_slice_inliers(slices, pnp_options(),
                                                    lib.null_trials, lib.seed);
  const int threshold = noise_floor_threshold(null_best, lib.null_margin,
                                              lib.min_slice_correspondences);
  if (best)
    *best = r.best_inliers;
  if (needed)
    *needed = threshold;
  return r.best_slice >= 0 && r.best_inliers >= threshold;
}

} // namespace

TEST_CASE("PanoramaNoiseFloor_GenuineSlice_IsAcceptedWithTruePose",
          "[slam][panorama][align][noise_floor]") {
  const auto slices = genuine_scene();
  const SliceResection r = resect_best_slice(slices, pnp_options());
  REQUIRE(r.best_slice == 1);
  CHECK(r.best_inliers >= 90);
  REQUIRE(r.slice_inliers.size() == slices.size());

  // the seed pose is the true one: centre = -Q^T t
  const Pose truth = true_pose();
  const Eigen::Vector3d centre = -r.Q.transpose() * r.t;
  CHECK((centre - truth.c).norm() < 0.05);
  const Eigen::AngleAxisd dR(r.Q * truth.Q.transpose());
  CHECK(std::abs(dR.angle()) < 0.01);

  int best = 0, needed = 0;
  CHECK(accepted(slices, &best, &needed));
  INFO("best " << best << " needed " << needed);
  CHECK(best >= needed);
}

TEST_CASE("PanoramaNoiseFloor_ShuffledGenuineSlice_IsRejected",
          "[slam][panorama][align][noise_floor]") {
  // The research null itself: the genuine scene with its pairings shuffled.
  for (unsigned seed : {1u, 2u, 3u, 4u, 5u}) {
    int best = 0, needed = 0;
    const bool ok = accepted(shuffled(genuine_scene(), seed), &best, &needed);
    INFO("shuffle seed " << seed << ": best " << best << " needed " << needed);
    CHECK_FALSE(ok);
  }
}

TEST_CASE("PanoramaNoiseFloor_ClusteredJunk_BeatsOldGateButIsRejected",
          "[slam][panorama][align][noise_floor]") {
  const PanoramaAlignmentOptions lib;
  for (unsigned seed : {11u, 12u, 13u, 14u, 15u}) {
    std::mt19937 rng(seed);
    std::vector<SlicePnPInput> slices;
    for (int i = 0; i < kSlices; ++i) {
      slices.push_back(empty_slice(i));
      add_clustered_junk(slices.back(), 400, rng);
    }
    int best = 0, needed = 0;
    const bool ok = accepted(slices, &best, &needed);
    INFO("seed " << seed << ": best " << best << " needed " << needed);
    // PnP-RANSAC on pure noise still returns a "pose" with a dozen inliers:
    // the old fixed gate (min_slice_correspondences) would have taken it...
    CHECK(best >= lib.min_slice_correspondences);
    // ...the per-panorama noise floor does not.
    CHECK_FALSE(ok);
  }
}

TEST_CASE("PanoramaNoiseFloor_RepeatedJunkPairs_AreRejected",
          "[slam][panorama][align][noise_floor]") {
  // One wrong match repeated across 8 near-identical candidate frames: the
  // same slice pixel paired with (almost) the same world point 8 times. A
  // random pose that explains one pair explains all 8, so the null must move
  // such runs together or it underestimates the floor.
  const PanoramaAlignmentOptions lib;
  for (unsigned seed : {31u, 32u, 33u}) {
    std::mt19937 rng(seed);
    std::normal_distribution<double> jitter_px(0.0, 0.3);
    std::normal_distribution<double> jitter_w(0.0, 0.01);
    std::vector<SlicePnPInput> slices;
    for (int i = 0; i < kSlices; ++i) {
      SlicePnPInput base = empty_slice(i);
      add_clustered_junk(base, 60, rng);
      SlicePnPInput s = empty_slice(i);
      for (size_t k = 0; k < base.pixels.size(); ++k)
        for (int f = 0; f < 8; ++f) {
          s.pixels.emplace_back(
              base.pixels[k].x + static_cast<float>(jitter_px(rng)),
              base.pixels[k].y + static_cast<float>(jitter_px(rng)));
          s.world.push_back(base.world[k] + Eigen::Vector3d(jitter_w(rng),
                                                            jitter_w(rng),
                                                            jitter_w(rng)));
        }
      slices.push_back(std::move(s));
    }
    int best = 0, needed = 0;
    const bool ok = accepted(slices, &best, &needed);
    INFO("seed " << seed << ": best " << best << " needed " << needed);
    CHECK(best >= 2 * lib.min_slice_correspondences);
    CHECK_FALSE(ok);
  }
}

TEST_CASE("PanoramaNoiseFloor_UniformRandomPairs_AreRejected",
          "[slam][panorama][align][noise_floor]") {
  std::mt19937 rng(21);
  std::vector<SlicePnPInput> slices;
  for (int i = 0; i < kSlices; ++i) {
    slices.push_back(empty_slice(i));
    add_junk(slices.back(), 400, rng);
  }
  CHECK_FALSE(accepted(slices));
}

TEST_CASE("PanoramaNoiseFloor_ShuffledNull_IsDeterministic",
          "[slam][panorama][align][noise_floor]") {
  const auto slices = genuine_scene();
  const int a = shuffled_best_slice_inliers(slices, pnp_options(), 3, 42);
  const int b = shuffled_best_slice_inliers(slices, pnp_options(), 3, 42);
  CHECK(a == b);
  // and the null is far below the genuine slice's support
  CHECK(a < 40);
}

TEST_CASE("PanoramaNoiseFloor_TooFewCorrespondences_NoPnP",
          "[slam][panorama][align][noise_floor]") {
  std::vector<SlicePnPInput> slices{empty_slice(0), empty_slice(1)};
  std::mt19937 rng(3);
  add_junk(slices[1], 3, rng); // below min_correspondences
  const SliceResection r = resect_best_slice(slices, pnp_options());
  CHECK(r.best_slice == -1);
  CHECK(r.best_inliers == 0);
  CHECK(r.slice_inliers == std::vector<int>{-1, -1});
  CHECK(shuffled_best_slice_inliers(slices, pnp_options(), 3, 42) == 0);
  CHECK(shuffled_best_slice_inliers({}, pnp_options(), 3, 42) == 0);
}

TEST_CASE("PanoramaNoiseFloor_Threshold_BeatsMarginTimesNull",
          "[slam][panorama][align][noise_floor]") {
  // strictly more than margin x null, never below the PnP minimum
  CHECK(noise_floor_threshold(10, 1.5, 6) == 16);
  CHECK(noise_floor_threshold(17, 1.5, 6) == 26);
  CHECK(noise_floor_threshold(0, 1.5, 6) == 6);
  CHECK(noise_floor_threshold(3, 1.5, 6) == 6);
}

TEST_CASE("PanoramaAlignmentSummary_NamesGateAndNumbers",
          "[slam][panorama][align][noise_floor]") {
  PanoramaAlignmentResult r;
  r.outcome = PanoramaAlignmentOutcome::below_noise_floor;
  r.diagnostics.best_slice_inliers = 13;
  r.diagnostics.null_best_slice_inliers = 15;
  r.diagnostics.noise_floor_threshold = 23;
  const std::string s = panorama_alignment_summary(r);
  CHECK(s.find("below_noise_floor") != std::string::npos);
  CHECK(s.find("13") != std::string::npos);
  CHECK(s.find("15") != std::string::npos);
  CHECK(s.find("23") != std::string::npos);

  PanoramaAlignmentResult g;
  g.outcome = PanoramaAlignmentOutcome::correction_gate;
  g.diagnostics.delta_from_seed_m = 2.49;
  g.diagnostics.max_correction_m = 1.5;
  const std::string gs = panorama_alignment_summary(g);
  CHECK(gs.find("correction_gate") != std::string::npos);
  CHECK(gs.find("2.49") != std::string::npos);
  CHECK(gs.find("1.50") != std::string::npos);
}
