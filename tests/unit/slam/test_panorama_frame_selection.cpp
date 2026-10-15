// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Unit tests for candidate-frame selection in `rux align 360`
// (reusex-bx2.6).
//
// The candidates used to be the seed frame and its nearest neighbours by id.
// On NewOffice those are often motion-blurred, and for a third of the
// panoramas they look within 3-29 deg of one direction, so the few real
// matches came from one frame and the resected centre was poorly constrained
// in translation (docs/research/panorama-alignment-failure-modes.md). These
// tests pin the replacement: a sharpness measure that ranks a blurred frame
// below a sharp one, and a greedy choice that drops blurred frames and
// spreads the rest over viewing directions.
//
// Includes the module-private `slam/panorama_features.hpp` directly — same
// arrangement as test_panorama_pool.cpp (STANDARDS §2).

#include <catch2/catch_test_macros.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/slam/PanoramaAlignment.hpp>

#include "slam/panorama_features.hpp"

#include "../../support/pose_fixture.hpp"
#include "../../support/temp_path.hpp"

#include <Eigen/Geometry>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <random>
#include <set>
#include <vector>

using namespace reusex::geometry::pano_detail;

namespace {

constexpr double kDeg = M_PI / 180.0;

/// A textured grey image: random rectangles of random grey levels.
cv::Mat texture(int w, int h, unsigned seed) {
  std::mt19937 rng(seed);
  cv::Mat img(h, w, CV_8UC1, cv::Scalar(128));
  std::uniform_int_distribution<int> x(0, w - 1), y(0, h - 1), g(0, 255);
  for (int k = 0; k < 400; ++k) {
    const cv::Point a(x(rng), y(rng)), b(x(rng), y(rng));
    cv::rectangle(img, a, b, cv::Scalar(g(rng)), cv::FILLED);
  }
  return img;
}

/// Unit viewing direction at @p yaw_deg about the world y axis.
Eigen::Vector3d dir(double yaw_deg) {
  return Eigen::Vector3d(std::sin(yaw_deg * kDeg), 0.0,
                         std::cos(yaw_deg * kDeg));
}

FrameChoice frame(int id, double dt, double sharpness, double yaw_deg) {
  FrameChoice f;
  f.node_id = id;
  f.dt_s = dt;
  f.sharpness = sharpness;
  f.view_dir = dir(yaw_deg);
  return f;
}

std::vector<int> ids(const std::vector<FrameChoice> &frames,
                     const std::vector<int> &picked) {
  std::vector<int> out;
  for (int i : picked)
    out.push_back(frames[i].node_id);
  return out;
}

} // namespace

TEST_CASE("PanoramaFrameSelection_Sharpness_RanksBlurBelowSharp",
          "[slam][panorama][align][frame_selection]") {
  const cv::Mat sharp = texture(640, 480, 1);
  cv::Mat blurred;
  cv::GaussianBlur(sharp, blurred, cv::Size(0, 0), 4.0);
  const double s = image_sharpness(sharp);
  const double b = image_sharpness(blurred);
  INFO("sharp " << s << " blurred " << b);
  CHECK(s > 4.0 * b);
  // a flat image has none
  CHECK(image_sharpness(cv::Mat(480, 640, CV_8UC1, cv::Scalar(90))) == 0.0);
  // measured at a fixed working width, so resolution does not dominate
  cv::Mat big;
  cv::resize(sharp, big, cv::Size(1280, 960), 0, 0, cv::INTER_NEAREST);
  const double sb = image_sharpness(big);
  CHECK(sb > 0.5 * s);
  CHECK(sb < 2.0 * s);
  // colour input is accepted and deterministic
  cv::Mat bgr;
  cv::cvtColor(sharp, bgr, cv::COLOR_GRAY2BGR);
  CHECK(image_sharpness(bgr) == image_sharpness(bgr));
}

TEST_CASE("PanoramaFrameSelection_DropsBlurredFrames",
          "[slam][panorama][align][frame_selection]") {
  // The seed (dt 0) is blurred; a sharp frame 3 s earlier is not (NewOffice
  // 010: seed 3022 blurred, 3017 sharp and carrying every inlier).
  const std::vector<FrameChoice> frames{
      frame(3022, 0.0, 10.0, 0.0), frame(3017, -3.3, 100.0, 5.0),
      frame(3023, 0.7, 12.0, 2.0), frame(3016, -4.0, 90.0, 60.0)};
  FrameSelectOptions o;
  o.max_frames = 4;
  o.min_sharpness_ratio = 0.5;
  o.view_spread_deg = 30.0;
  const auto picked = ids(frames, select_candidate_frames(frames, o));
  CHECK(picked == std::vector<int>{3017, 3016});
}

TEST_CASE("PanoramaFrameSelection_SpreadsOverViewDirections",
          "[slam][panorama][align][frame_selection]") {
  // Five sharp frames looking the same way, two slightly less sharp ones
  // looking 90 and 180 deg away: a pick of 3 takes one from each direction.
  std::vector<FrameChoice> frames;
  for (int k = 0; k < 5; ++k)
    frames.push_back(frame(100 + k, 0.5 * k, 100.0 - k, 2.0 * k));
  frames.push_back(frame(200, 3.0, 80.0, 90.0));
  frames.push_back(frame(300, -3.0, 70.0, 180.0));
  FrameSelectOptions o;
  o.max_frames = 3;
  o.min_sharpness_ratio = 0.5;
  o.view_spread_deg = 30.0;
  const auto picked = ids(frames, select_candidate_frames(frames, o));
  CHECK(picked == std::vector<int>{100, 200, 300});

  // with more room than directions, the sharpest repeats fill up the rest
  o.max_frames = 5;
  const auto more = ids(frames, select_candidate_frames(frames, o));
  REQUIRE(more.size() == 5);
  CHECK(std::vector<int>(more.begin(), more.begin() + 3) ==
        std::vector<int>{100, 200, 300});
  CHECK(std::set<int>(more.begin() + 3, more.end()) == std::set<int>{101, 102});
}

TEST_CASE("PanoramaFrameSelection_TiesAreDeterministic",
          "[slam][panorama][align][frame_selection]") {
  // Featureless frames (sharpness 0 everywhere, the same direction): the
  // order falls back to |dt|, then node id, so the seed comes first.
  const std::vector<FrameChoice> frames{frame(1, -1.0, 0.0, 0.0),
                                        frame(2, 0.0, 0.0, 0.0),
                                        frame(3, 1.0, 0.0, 0.0)};
  FrameSelectOptions o;
  o.max_frames = 2;
  o.min_sharpness_ratio = 0.5;
  o.view_spread_deg = 30.0;
  CHECK(ids(frames, select_candidate_frames(frames, o)) ==
        std::vector<int>{2, 1});
  CHECK(select_candidate_frames({}, o).empty());
}

TEST_CASE("AlignPanorama_SelectFrames_TakesSharpNearbyFramesInTheWindow",
          "[slam][panorama][align][frame_selection]") {
  // Seed 10 is blurred; 11 and 12 are sharp; 13 is sharp but 5 m away;
  // 14 is sharp but 30 s later. Only 11 and 12 are worth matching.
  using namespace reusex;
  using namespace reusex::test_support;
  constexpr int kW = 160, kH = 120;
  TempPath tmp("test_pano_select_frames");
  ProjectDB db(tmp.path);
  const cv::Mat sharp = make_checker(kW, kH);
  cv::Mat blurred;
  cv::GaussianBlur(sharp, blurred, cv::Size(0, 0), 3.0);
  const auto intr = make_intrinsics(80.0, 80.0, kW / 2.0, kH / 2.0, kW, kH);
  const cv::Mat depth = make_depth(kW, kH, 2.0);
  auto add = [&](int id, const cv::Mat &img, double x, double ts) {
    db.save_sensor_frame(id, img, depth, cv::Mat(), translation_pose(x, 0, 0),
                         intr, ts);
  };
  add(10, blurred, 0.0, 100.0);
  add(11, sharp, 0.2, 101.0);
  add(12, sharp, -0.3, 98.5);
  add(13, sharp, 5.0, 102.0);
  add(14, sharp, 0.1, 130.0);
  std::vector<uint8_t> jpg;
  cv::imencode(".jpg", cv::Mat(128, 256, CV_8UC3, cv::Scalar(120, 120, 120)),
               jpg);
  db.save_panoramic_image("pano.jpg", jpg, 100.0, 10);
  const int pano = db.list_panoramic_images().front().id;

  geometry::PanoramaAlignmentOptions opt;
  opt.n_yaw = 4;
  opt.slice = 64;
  opt.select_frames = true;
  opt.select_window_s = 10.0;
  opt.select_max_dist_m = 1.5;
  opt.min_sharpness_ratio = 0.5;
  const auto r = geometry::align_panorama(db, pano, 10, opt);
  const auto &d = r.diagnostics;
  CHECK(d.frames_considered == 3); // 10, 11, 12: 13 too far, 14 too late
  std::set<int> chosen;
  for (const auto &f : d.frames) {
    chosen.insert(f.node_id);
    CHECK(f.sharpness > 0.0);
  }
  CHECK(chosen == std::set<int>{11, 12});

  // by id (the old way) the blurred seed comes first
  opt.select_frames = false;
  const auto by_id = geometry::align_panorama(db, pano, 10, opt);
  REQUIRE_FALSE(by_id.diagnostics.frames.empty());
  CHECK(by_id.diagnostics.frames[0].node_id == 10);
  CHECK(by_id.diagnostics.frames_considered == 0);
}
