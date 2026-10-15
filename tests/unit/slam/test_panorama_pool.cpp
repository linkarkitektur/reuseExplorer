// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Unit tests for the correspondence pool of `rux align 360` (reusex-bx2.5).
//
// docs/research/panorama-alignment-failure-modes.md: the pool was ~95 % junk
// by construction. Every slice keypoint picked its 1-NN in every frame (40-50
// % passed the Lowe test even in slices no frame sees), overlapping slices put
// the same panorama keypoint in twice, the nadir (operator, hand, stick) gave
// 10 % of the pool and no inliers, and EPnP RANSAC at 500 iterations could not
// sample a 1-5 % inlier rate. These tests pin the pieces of the rebuilt pool
// on synthetic data:
//
//   - each direction of the sphere is owned by one slice, so a keypoint seen
//     by two overlapping slices enters the pool once;
//   - the nadir/zenith masks drop the bearings around straight down/up;
//   - frame-driven matching tests the ratio against a competitor elsewhere on
//     the sphere (not ORB's duplicate detection of the same corner);
//   - a panorama keypoint matched by several frames counts once;
//   - USAC PnP finds a pose a 500-iteration EPnP RANSAC misses.
//
// Includes the module-private `slam/panorama_features.hpp` directly — same
// arrangement as test_panorama_features_pose.cpp (STANDARDS §2).

#include <catch2/catch_test_macros.hpp>

#include <reusex/geometry/EquirectProjection.hpp>

#include "slam/panorama_features.hpp"

#include <Eigen/Geometry>

#include <opencv2/core.hpp>

#include <cmath>
#include <random>
#include <set>
#include <vector>

using namespace reusex::geometry;
using namespace reusex::geometry::pano_detail;

namespace {

constexpr double kDeg = M_PI / 180.0;

/// The production slice geometry: 8 x 90 deg around the equator plus the two
/// pole caps, as overlapping_views() renders them (on a blank equirect).
std::vector<SliceKeypoints> production_slices(int tile = 200) {
  const cv::Mat equirect(64, 128, CV_8UC3, cv::Scalar::all(0));
  std::vector<SliceKeypoints> out;
  for (const auto &v : overlapping_views(equirect, 8, 90.0, tile)) {
    SliceKeypoints s;
    s.K = v.K;
    s.R_pano_from_view = v.R_pano_from_view;
    s.size = cv::Size(tile, tile);
    out.push_back(std::move(s));
  }
  return out;
}

/// Projection of panorama bearing @p b into slice @p s, if it lands inside.
bool project(const SliceKeypoints &s, const Eigen::Vector3d &b,
             cv::Point2f *px) {
  const Eigen::Vector3d d = s.R_pano_from_view.transpose() * b;
  if (d.z() <= 1e-9)
    return false;
  const double u = s.K(0, 0) * d.x() / d.z() + s.K(0, 2);
  const double v = s.K(1, 1) * d.y() / d.z() + s.K(1, 2);
  if (u < 0 || v < 0 || u >= s.size.width || v >= s.size.height)
    return false;
  *px = cv::Point2f(static_cast<float>(u), static_cast<float>(v));
  return true;
}

Eigen::Vector3d random_bearing(std::mt19937 &rng) {
  std::normal_distribution<double> n(0.0, 1.0);
  return Eigen::Vector3d(n(rng), n(rng), n(rng)).normalized();
}

/// A 32-byte ORB-shaped descriptor whose first 4 bytes encode @p id.
cv::Mat id_descriptor(int id, std::mt19937 &rng) {
  cv::Mat d(1, 32, CV_8U);
  for (int c = 0; c < 32; ++c)
    d.at<uchar>(0, c) = static_cast<uchar>(rng() & 0xff);
  for (int c = 0; c < 4; ++c)
    d.at<uchar>(0, c) = static_cast<uchar>((id >> (8 * c)) & 0xff);
  return d;
}

int descriptor_id(const cv::Mat &row) {
  int id = 0;
  for (int c = 0; c < 4; ++c)
    id |= static_cast<int>(row.at<uchar>(0, c)) << (8 * c);
  return id;
}

/// Flip @p n distinct bits of descriptor row @p d (bytes 4.. only, so the
/// encoded id survives).
cv::Mat flip_bits(const cv::Mat &d, int n, std::mt19937 &rng) {
  cv::Mat out = d.clone();
  std::set<int> used;
  while (static_cast<int>(used.size()) < n) {
    const int bit = 32 + static_cast<int>(rng() % (28 * 8));
    if (!used.insert(bit).second)
      continue;
    out.at<uchar>(0, bit / 8) ^= static_cast<uchar>(1u << (bit % 8));
  }
  return out;
}

/// Scatter @p n random bearings into every slice that sees them; returns how
/// many bearings at least one slice sees.
int scatter(std::vector<SliceKeypoints> &slices, int n, std::mt19937 &rng) {
  int seen = 0;
  for (int id = 0; id < n; ++id) {
    const Eigen::Vector3d b = random_bearing(rng);
    const cv::Mat d = id_descriptor(id, rng);
    bool any = false;
    for (auto &s : slices) {
      cv::Point2f px;
      if (!project(s, b, &px))
        continue;
      s.pixels.push_back(px);
      s.descriptors.push_back(d);
      any = true;
    }
    seen += any ? 1 : 0;
  }
  return seen;
}

SphereKeypoints sphere_from(const std::vector<Eigen::Vector3d> &bearings,
                            const std::vector<cv::Mat> &desc) {
  SphereKeypoints s;
  for (size_t k = 0; k < bearings.size(); ++k) {
    s.pixels.emplace_back(0.0f, 0.0f);
    s.slice.push_back(0);
    s.bearings.push_back(bearings[k].normalized());
    s.descriptors.push_back(desc[k]);
  }
  return s;
}

Eigen::Vector3d at(double az_deg, double el_deg) {
  // panorama frame: z forward, x right, y down
  return Eigen::Vector3d(std::cos(el_deg * kDeg) * std::sin(az_deg * kDeg),
                         -std::sin(el_deg * kDeg),
                         std::cos(el_deg * kDeg) * std::cos(az_deg * kDeg));
}

} // namespace

TEST_CASE("PanoramaPool_OwningSlice_IsTheNearestAxisThatSeesTheBearing",
          "[slam][panorama][align][pool]") {
  const auto slices = production_slices();
  REQUIRE(slices.size() == 10);
  // every slice owns its own optical axis
  for (int i = 0; i < 10; ++i)
    CHECK(owning_slice(slices, slices[i].R_pano_from_view.col(2)) == i);
  // a bearing 10 deg off slice 3's axis towards slice 4 is still slice 3's,
  // and 30 deg off it is slice 4's (axes are 45 deg apart)
  const Eigen::Vector3d a3 = slices[3].R_pano_from_view.col(2);
  const Eigen::Vector3d a4 = slices[4].R_pano_from_view.col(2);
  const Eigen::Vector3d axis = a3.cross(a4).normalized();
  CHECK(owning_slice(slices, Eigen::AngleAxisd(10 * kDeg, axis) * a3) == 3);
  CHECK(owning_slice(slices, Eigen::AngleAxisd(30 * kDeg, axis) * a3) == 4);
}

TEST_CASE("PanoramaPool_OverlappingSlices_EnterEachKeypointOnce",
          "[slam][panorama][align][pool]") {
  std::mt19937 rng(5);
  auto slices = production_slices();
  const int seen = scatter(slices, 3000, rng);
  size_t raw = 0;
  for (const auto &s : slices)
    raw += s.pixels.size();
  // 8 x 90 deg around the equator overlaps 45 deg: many keypoints are in two
  REQUIRE(raw > static_cast<size_t>(seen) * 5 / 4);

  SpherePoolOptions keep_all;
  keep_all.dedupe_overlap = false;
  CHECK(pool_sphere_keypoints(slices, keep_all).bearings.size() == raw);

  const SphereKeypoints pool = pool_sphere_keypoints(slices, {});
  REQUIRE(static_cast<int>(pool.bearings.size()) == seen);
  CHECK(pool.dropped_overlap == static_cast<int>(raw) - seen);
  std::set<int> ids;
  for (int k = 0; k < pool.descriptors.rows; ++k) {
    ids.insert(descriptor_id(pool.descriptors.row(k)));
    // the keypoint sits in its owning slice, and its bearing is the one the
    // slice pixel back-projects to
    CHECK(owning_slice(slices, pool.bearings[k]) == pool.slice[k]);
    const auto &s = slices[pool.slice[k]];
    CHECK((slice_bearing(s.K, s.R_pano_from_view, pool.pixels[k]) -
           pool.bearings[k])
              .norm() < 1e-9);
  }
  CHECK(static_cast<int>(ids.size()) == seen);
}

TEST_CASE("PanoramaPool_NadirAndZenithMasks_DropThePoles",
          "[slam][panorama][align][pool]") {
  std::mt19937 rng(9);
  auto slices = production_slices();
  scatter(slices, 4000, rng);
  const SphereKeypoints all = pool_sphere_keypoints(slices, {});

  SpherePoolOptions nadir;
  nadir.nadir_mask_deg = 40.0;
  const SphereKeypoints no_nadir = pool_sphere_keypoints(slices, nadir);
  const Eigen::Vector3d down(0, 1, 0);
  int expected_dropped = 0;
  for (const auto &b : all.bearings)
    expected_dropped +=
        std::acos(std::clamp(b.dot(down), -1.0, 1.0)) < 40.0 * kDeg ? 1 : 0;
  REQUIRE(expected_dropped > 0);
  CHECK(no_nadir.dropped_pole == expected_dropped);
  CHECK(no_nadir.bearings.size() + expected_dropped == all.bearings.size());
  for (const auto &b : no_nadir.bearings)
    CHECK(std::acos(std::clamp(b.dot(down), -1.0, 1.0)) >= 40.0 * kDeg);
  // the down cap (slice 9) has nothing left; the up cap is untouched
  int down_cap = 0, up_cap_all = 0, up_cap = 0;
  for (int s : no_nadir.slice) {
    down_cap += s == 9;
    up_cap += s == 8;
  }
  for (int s : all.slice)
    up_cap_all += s == 8;
  CHECK(down_cap > 0); // the cap owns a little beyond 45 deg from down
  CHECK(up_cap == up_cap_all);
  // a mask wider than the cap's field of view (54.7 deg to its corners)
  // empties it
  SpherePoolOptions wide;
  wide.nadir_mask_deg = 55.0;
  for (int s : pool_sphere_keypoints(slices, wide).slice)
    CHECK(s != 9);

  SpherePoolOptions zenith;
  zenith.zenith_mask_deg = 40.0;
  const SphereKeypoints no_zenith = pool_sphere_keypoints(slices, zenith);
  for (const auto &b : no_zenith.bearings)
    CHECK(std::acos(std::clamp(-b.y(), -1.0, 1.0)) >= 40.0 * kDeg);
}

TEST_CASE("PanoramaPool_FrameToSphere_RatioAgainstADistinctPlace",
          "[slam][panorama][align][pool]") {
  std::mt19937 rng(17);
  std::vector<Eigen::Vector3d> b;
  std::vector<cv::Mat> d;
  // 0: a unique corner
  b.push_back(at(0, 0));
  d.push_back(id_descriptor(0, rng));
  // 1, 2: ORB's duplicate detection of one corner (another pyramid level):
  // identical descriptors 0.2 deg apart
  b.push_back(at(90, 10));
  d.push_back(id_descriptor(1, rng));
  b.push_back(at(90.2, 10));
  d.push_back(d.back().clone());
  // 3, 4: repetitive texture — identical descriptors 60 deg apart
  b.push_back(at(180, 0));
  d.push_back(id_descriptor(3, rng));
  b.push_back(at(240, 0));
  d.push_back(d.back().clone());
  // background: random descriptors elsewhere on the sphere
  for (int k = 5; k < 400; ++k) {
    b.push_back(random_bearing(rng));
    d.push_back(id_descriptor(k, rng));
  }
  const SphereKeypoints sphere = sphere_from(b, d);

  cv::Mat frame;
  frame.push_back(flip_bits(d[0], 6, rng)); // -> 0
  frame.push_back(flip_bits(d[1], 6, rng)); // -> 1 or 2, kept
  frame.push_back(flip_bits(d[3], 6, rng)); // ambiguous, dropped
  const auto m = match_frame_to_sphere(frame, sphere, 0.85f, 2.0);
  REQUIRE(m.size() == 2);
  CHECK(m[0].frame_kp == 0);
  CHECK(m[0].sphere_kp == 0);
  CHECK(m[1].frame_kp == 1);
  CHECK((m[1].sphere_kp == 1 || m[1].sphere_kp == 2));

  // Without the distinct-place rule (distinct_deg = 0) the duplicate
  // detection of corner 1 is its own competitor and the match is lost.
  const auto strict = match_frame_to_sphere(frame, sphere, 0.85f, 0.0);
  CHECK(strict.size() == 1);
}

TEST_CASE("PanoramaPool_SphereToFrame_IsTheLegacyPerKeypointRatio",
          "[slam][panorama][align][pool]") {
  std::mt19937 rng(23);
  std::vector<Eigen::Vector3d> b;
  std::vector<cv::Mat> d;
  for (int k = 0; k < 50; ++k) {
    b.push_back(random_bearing(rng));
    d.push_back(id_descriptor(k, rng));
  }
  const SphereKeypoints sphere = sphere_from(b, d);
  cv::Mat frame;
  for (int k = 0; k < 10; ++k)
    frame.push_back(flip_bits(d[k], 4, rng));
  for (int k = 0; k < 200; ++k)
    frame.push_back(id_descriptor(1000 + k, rng));
  const auto m = match_sphere_to_frame(frame, sphere, 0.85f);
  // every planted sphere keypoint finds its frame copy
  int planted = 0;
  for (const auto &x : m)
    if (x.sphere_kp < 10) {
      CHECK(x.frame_kp == x.sphere_kp);
      ++planted;
    }
  CHECK(planted == 10);
}

TEST_CASE("PanoramaPool_OnePerSphereKeypoint_KeepsLowestHamming",
          "[slam][panorama][align][pool]") {
  // sphere keypoint 7 matched by three frames, 2 by two, 5 once
  const std::vector<int> kp{7, 2, 7, 5, 2, 7};
  const std::vector<float> ham{30, 20, 12, 40, 20, 12};
  // 7: entries 2 and 5 tie at 12 -> the earlier (2); 2: 1 and 4 tie -> 1
  CHECK(one_per_sphere_keypoint(kp, ham) == std::vector<int>{1, 2, 3});
  CHECK(one_per_sphere_keypoint({}, {}).empty());
}

TEST_CASE("PanoramaPool_UsacPnP_FindsAPoseAtAFewPercentInliers",
          "[slam][panorama][align][pool]") {
  // One slice, 60 genuine correspondences among 540 junk (10 %), the rate
  // of NewOffice's best slices. A 5-point EPnP sample is all-inlier with
  // probability 1e-5, so 500 iterations find it 0.5 % of the time; USAC's
  // 3-point samples (1e-3) with local optimisation find it.
  std::mt19937 rng(41);
  SlicePnPInput s;
  const double f = 504.0;
  s.K << f, 0, 504.0, 0, f, 504.0, 0, 0, 1;
  const Eigen::Matrix3d Q =
      Eigen::AngleAxisd(0.3, Eigen::Vector3d(0.1, 1.0, 0.2).normalized())
          .toRotationMatrix();
  const Eigen::Vector3d c(0.5, -0.2, 1.0);
  std::uniform_real_distribution<double> px(20.0, 988.0), depth(2.0, 6.0),
      w(-6.0, 6.0);
  std::normal_distribution<double> noise(0.0, 0.5);
  for (int k = 0; k < 60; ++k) {
    const double u = px(rng), v = px(rng);
    const Eigen::Vector3d ray = s.K.inverse() * Eigen::Vector3d(u, v, 1.0);
    s.world.push_back(Q.transpose() * (depth(rng) * ray) + c);
    s.pixels.emplace_back(static_cast<float>(u + noise(rng)),
                          static_cast<float>(v + noise(rng)));
  }
  for (int k = 0; k < 540; ++k) {
    s.pixels.emplace_back(static_cast<float>(px(rng)),
                          static_cast<float>(px(rng)));
    s.world.emplace_back(w(rng), w(rng), w(rng));
  }
  SlicePnPOptions o;
  o.usac = true;
  o.ransac_iterations = 5000;
  o.usac_seed = 42;
  const SliceResection r = resect_best_slice({s}, o);
  REQUIRE(r.best_slice == 0);
  CHECK(r.best_inliers >= 55);
  const Eigen::Vector3d centre = -r.Q.transpose() * r.t;
  CHECK((centre - c).norm() < 0.1);
  CHECK(std::abs(Eigen::AngleAxisd(r.Q * Q.transpose()).angle()) < 0.02);
  // deterministic under the same seed
  CHECK(resect_best_slice({s}, o).best_inliers == r.best_inliers);
}
