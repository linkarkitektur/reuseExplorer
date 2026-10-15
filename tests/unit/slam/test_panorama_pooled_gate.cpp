// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Unit tests for the null-calibrated pooled-inlier gate of `rux align 360`
// (reusex-bx2.8).
//
// The pooled gate counts the correspondences of the WHOLE sphere that the
// best slice's pose explains. Its fixed `min_inliers = 40` was set on the
// old pool, which counted each panorama keypoint once per overlapping slice
// and once per near-identical candidate frame. After reusex-bx2.5 deduplicated
// the pool, panoramas with a clear best-slice signal (NewOffice 007 and 012,
// 3.8x and 5.1x their noise floor) stop at 30-36 pooled inliers. These tests
// pin the replacement: the pooled count must beat the pooled count the
// shuffled null reaches on the same panorama, by a margin.
//
// Includes the module-private `slam/panorama_features.hpp` directly — same
// arrangement as test_panorama_noise_floor.cpp (STANDARDS §2).

#include <catch2/catch_test_macros.hpp>

#include <reusex/slam/PanoramaAlignment.hpp>

#include "slam/panorama_features.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

using namespace reusex::geometry;
using namespace reusex::geometry::pano_detail;

namespace {

constexpr int kSlicePx = 1008;
constexpr int kSlices = 4;

SlicePnPInput empty_slice(int i) {
  SlicePnPInput s;
  const double f = 0.5 * kSlicePx;
  s.K << f, 0, 0.5 * kSlicePx, 0, f, 0.5 * kSlicePx, 0, 0, 1;
  s.R_pano_from_view =
      Eigen::AngleAxisd(i * M_PI / 2.0, Eigen::Vector3d::UnitY())
          .toRotationMatrix();
  return s;
}

/// Clustered junk (see test_panorama_noise_floor.cpp): pixels and world
/// points bunched on a few blobs, which is what lets a random pose explain a
/// dozen pairs by chance.
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
  Eigen::Vector3d c; ///< centre in world
};

Pose true_pose() {
  return {Eigen::AngleAxisd(0.4, Eigen::Vector3d(0.2, 1.0, 0.1).normalized())
              .toRotationMatrix(),
          Eigen::Vector3d(1.5, -0.3, 2.0)};
}

void add_genuine(SlicePnPInput &s, const Pose &pose, int n, std::mt19937 &rng) {
  std::uniform_real_distribution<double> px(20.0, kSlicePx - 20.0);
  std::uniform_real_distribution<double> depth(2.0, 6.0);
  std::normal_distribution<double> noise(0.0, 0.5);
  for (int k = 0; k < n; ++k) {
    const double u = px(rng), v = px(rng);
    const Eigen::Vector3d ray = s.K.inverse() * Eigen::Vector3d(u, v, 1.0);
    const Eigen::Vector3d p_pano = s.R_pano_from_view * (depth(rng) * ray);
    s.world.push_back(pose.Q.transpose() * p_pano + pose.c);
    s.pixels.emplace_back(static_cast<float>(u + noise(rng)),
                          static_cast<float>(v + noise(rng)));
  }
}

/// NewOffice 007 / 012 in miniature: a best slice with a clear signal
/// (@p best genuine among junk) and a few genuine matches in each other
/// slice, so the pooled support is real but below the old fixed gate.
std::vector<SlicePnPInput> thin_scene(unsigned seed, int best = 22,
                                      int others = 4) {
  std::mt19937 rng(seed);
  std::vector<SlicePnPInput> slices;
  for (int i = 0; i < kSlices; ++i) {
    slices.push_back(empty_slice(i));
    add_genuine(slices.back(), true_pose(), i == 1 ? best : others, rng);
    add_clustered_junk(slices.back(), i == 1 ? 60 : 100, rng);
  }
  return slices;
}

std::vector<SlicePnPInput> junk_scene(unsigned seed) {
  std::mt19937 rng(seed);
  std::vector<SlicePnPInput> slices;
  for (int i = 0; i < kSlices; ++i) {
    slices.push_back(empty_slice(i));
    add_clustered_junk(slices.back(), 300, rng);
  }
  return slices;
}

SlicePnPOptions pnp_options() {
  const PanoramaAlignmentOptions lib;
  SlicePnPOptions o;
  o.min_correspondences = lib.min_slice_correspondences;
  o.ransac_iterations = lib.ransac_iterations;
  o.ransac_reproj_px = lib.ransac_reproj_px;
  o.usac = lib.usac;
  o.usac_seed = static_cast<int>(lib.seed);
  return o;
}

/// The bearing refinement as align_panorama() runs it on the pool: the
/// angular gate from the slice reprojection tolerance, relaxed 2x.
BearingRefineOptions refine_options() {
  const PanoramaAlignmentOptions lib;
  BearingRefineOptions ro;
  ro.ang_gate = 2.0 * std::atan(lib.ransac_reproj_px / (0.5 * kSlicePx));
  ro.iterations = lib.refine_iterations;
  ro.min_inliers = 1;
  return ro;
}

/// The real pooled support: the best slice's pose applied to every slice's
/// correspondences and refined, as align_panorama() does.
struct Pooled {
  int initial = 0;
  int final = 0;
  Eigen::Vector3d centre = Eigen::Vector3d::Zero();
};
Pooled real_pooled(const std::vector<SlicePnPInput> &slices) {
  Pooled out;
  const SliceResection r = resect_best_slice(slices, pnp_options());
  if (r.best_slice < 0)
    return out;
  std::vector<Eigen::Vector3d> pts, brg;
  for (const auto &s : slices)
    for (size_t k = 0; k < s.pixels.size(); ++k) {
      pts.push_back(s.world[k]);
      brg.push_back(slice_bearing(s.K, s.R_pano_from_view, s.pixels[k]));
    }
  Eigen::Matrix3d Q = r.Q;
  Eigen::Vector3d t = r.t;
  out.final = static_cast<int>(
      refine_bearing_pose(pts, brg, refine_options(), Q, t, &out.initial)
          .size());
  out.centre = -Q.transpose() * t;
  return out;
}

NullFloor null_floor(const std::vector<SlicePnPInput> &slices) {
  const PanoramaAlignmentOptions lib;
  const BearingRefineOptions ro = refine_options();
  return shuffled_null_floor(slices, pnp_options(), lib.null_trials, lib.seed,
                             &ro);
}

} // namespace

TEST_CASE("PanoramaPooledGate_NullFloor_MatchesBestSliceNull",
          "[slam][panorama][align][pooled_gate]") {
  // The pooled null extends the best-slice null without changing it: the
  // same permutations in the same order, so the best-slice noise floor (and
  // every diagnostics file measured with it) is unchanged.
  const PanoramaAlignmentOptions lib;
  for (unsigned seed : {1u, 2u, 3u}) {
    const auto slices = thin_scene(seed);
    const NullFloor n = null_floor(slices);
    CHECK(n.best_slice_inliers ==
          shuffled_best_slice_inliers(slices, pnp_options(), lib.null_trials,
                                      lib.seed));
    // a pooled count includes its own slice's inliers
    CHECK(n.pooled_initial >= 0);
    CHECK(n.pooled_final >= 0);
  }
}

TEST_CASE("PanoramaPooledGate_NullFloor_IsDeterministic",
          "[slam][panorama][align][pooled_gate]") {
  const auto slices = thin_scene(4);
  const NullFloor a = null_floor(slices);
  const NullFloor b = null_floor(slices);
  CHECK(a.best_slice_inliers == b.best_slice_inliers);
  CHECK(a.pooled_initial == b.pooled_initial);
  CHECK(a.pooled_final == b.pooled_final);
}

TEST_CASE("PanoramaPooledGate_ThinGenuineSignal_PassesBelowOldFixedGate",
          "[slam][panorama][align][pooled_gate]") {
  // Real but thin support, as 007 / 012 after the deduplicated pool: fewer
  // pooled inliers than the old fixed gate of 40, yet far above what the
  // shuffled null reaches on the same panorama.
  const PanoramaAlignmentOptions lib;
  for (unsigned seed : {1u, 2u, 3u, 4u, 5u}) {
    const auto slices = thin_scene(seed);
    const Pooled real = real_pooled(slices);
    const NullFloor n = null_floor(slices);
    const int need_initial = noise_floor_threshold(
        n.pooled_initial, lib.pooled_null_margin, lib.min_inliers);
    const int need_final = noise_floor_threshold(
        n.pooled_final, lib.pooled_null_margin, lib.min_inliers);
    INFO("seed " << seed << ": real " << real.initial << " -> " << real.final
                 << ", null " << n.pooled_initial << " -> " << n.pooled_final
                 << ", needs " << need_initial << " / " << need_final);
    CHECK(real.final < 40); // the old gate would have rejected it
    CHECK(real.initial >= need_initial);
    CHECK(real.final >= need_final);
    CHECK((real.centre - true_pose().c).norm() < 0.1);
  }
}

TEST_CASE("PanoramaPooledGate_ClusteredJunk_IsRejected",
          "[slam][panorama][align][pooled_gate]") {
  // Pure noise must not pass the calibrated pooled gate, whatever its
  // best slice managed.
  const PanoramaAlignmentOptions lib;
  for (unsigned seed : {11u, 12u, 13u, 14u, 15u}) {
    const auto slices = junk_scene(seed);
    const Pooled real = real_pooled(slices);
    const NullFloor n = null_floor(slices);
    const int need_final = noise_floor_threshold(
        n.pooled_final, lib.pooled_null_margin, lib.min_inliers);
    INFO("seed " << seed << ": real " << real.initial << " -> " << real.final
                 << ", null " << n.pooled_initial << " -> " << n.pooled_final
                 << ", needs " << need_final);
    CHECK(real.final < need_final);
  }
}

TEST_CASE("PanoramaPooledGate_NoPooledOptions_SkipsPooledNull",
          "[slam][panorama][align][pooled_gate]") {
  const PanoramaAlignmentOptions lib;
  const auto slices = thin_scene(1);
  const NullFloor n = shuffled_null_floor(slices, pnp_options(),
                                          lib.null_trials, lib.seed, nullptr);
  CHECK(n.pooled_initial == -1);
  CHECK(n.pooled_final == -1);
  CHECK(n.best_slice_inliers ==
        shuffled_best_slice_inliers(slices, pnp_options(), lib.null_trials,
                                    lib.seed));
}
