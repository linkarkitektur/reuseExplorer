// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Unit tests for the per-panorama diagnostics of `rux align 360`
// (reusex-bx2.1). A rejected panorama used to report only "insufficient
// matches", whichever gate actually rejected it; these tests pin that every
// early exit names its own outcome, and that the per-slice / per-frame
// funnel counters are filled in on the way there.

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/SensorIntrinsics.hpp>
#include <reusex/slam/PanoramaAlignment.hpp>

#include <catch2/catch_test_macros.hpp>

#include "../../support/temp_path.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <nlohmann/json.hpp>

#include <string>
#include <vector>

using namespace reusex;
using namespace reusex::geometry;

namespace {

std::vector<uint8_t> encode_jpeg(const cv::Mat &img) {
  std::vector<uint8_t> buf;
  cv::imencode(".jpg", img, buf);
  return buf;
}

/// One featureless sensor frame (flat colour, 2 m depth, identity pose).
void add_flat_frame(ProjectDB &db, int node_id) {
  core::SensorIntrinsics intr;
  intr.fx = 100.0;
  intr.fy = 100.0;
  intr.cx = 64.0;
  intr.cy = 48.0;
  intr.width = 128;
  intr.height = 96;
  const cv::Mat color(96, 128, CV_8UC3, cv::Scalar(90, 90, 90));
  const cv::Mat depth(96, 128, CV_16UC1, cv::Scalar(2000)); // 2 m
  const std::array<double, 16> pose = {1, 0, 0, 0, 0, 1, 0, 0,
                                       0, 0, 1, 0, 0, 0, 0, 1};
  db.save_sensor_frame(node_id, color, depth, cv::Mat(), pose, intr,
                       100.0 + node_id);
}

PanoramaAlignmentOptions small_options() {
  PanoramaAlignmentOptions opt;
  opt.n_yaw = 4;
  opt.slice = 64; // keep the test fast: tiny slices of a tiny equirect
  return opt;
}

} // namespace

TEST_CASE("AlignPanorama_UndecodableImage_ReportsNoImage",
          "[slam][panorama][align]") {
  test_support::TempPath tmp{"reusex_pano_align_diag_noimg", ".rux"};
  ProjectDB db(tmp.path);
  add_flat_frame(db, 1);
  db.save_panoramic_image("bad.jpg", std::vector<uint8_t>{1, 2, 3}, 101.0, 1);
  const int pano = db.list_panoramic_images().front().id;

  const auto r = align_panorama(db, pano, 1, small_options());
  CHECK_FALSE(r.aligned);
  CHECK(r.outcome == PanoramaAlignmentOutcome::no_image);
}

TEST_CASE("AlignPanorama_SeedNodeAbsent_ReportsSeedNotFound",
          "[slam][panorama][align]") {
  test_support::TempPath tmp{"reusex_pano_align_diag_noseed", ".rux"};
  ProjectDB db(tmp.path);
  add_flat_frame(db, 1);
  db.save_panoramic_image(
      "pano.jpg",
      encode_jpeg(cv::Mat(128, 256, CV_8UC3, cv::Scalar(120, 120, 120))), 101.0,
      1);
  const int pano = db.list_panoramic_images().front().id;

  const auto r = align_panorama(db, pano, /*seed_node_id=*/77, small_options());
  CHECK_FALSE(r.aligned);
  CHECK(r.outcome == PanoramaAlignmentOutcome::seed_not_found);
}

TEST_CASE("AlignPanorama_FeaturelessInputs_ReportsNoSlicePnpWithFunnelCounts",
          "[slam][panorama][align]") {
  test_support::TempPath tmp{"reusex_pano_align_diag_flat", ".rux"};
  ProjectDB db(tmp.path);
  add_flat_frame(db, 1);
  add_flat_frame(db, 2);
  add_flat_frame(db, 3);
  db.save_panoramic_image(
      "pano.jpg",
      encode_jpeg(cv::Mat(128, 256, CV_8UC3, cv::Scalar(120, 120, 120))), 102.0,
      2);
  const int pano = db.list_panoramic_images().front().id;

  PanoramaAlignmentOptions opt = small_options();
  opt.max_candidates = 2;
  const auto r = align_panorama(db, pano, 2, opt);

  CHECK_FALSE(r.aligned);
  CHECK(r.outcome == PanoramaAlignmentOutcome::no_slice_pnp);

  const auto &d = r.diagnostics;
  // n_yaw equator slices + the two pole caps
  REQUIRE(d.slices.size() == static_cast<size_t>(opt.n_yaw + 2));
  CHECK(d.slices[1].yaw_deg == 90.0);
  CHECK(d.slices[opt.n_yaw].pitch_deg == 90.0);
  // the candidate cap holds: the seed first, then its nearest neighbour
  REQUIRE(d.frames.size() == 2);
  CHECK(d.frames[0].node_id == 2);
  CHECK(d.frames[0].dt_from_seed_s == 0.0);
  // featureless images: nothing survives any stage of the funnel
  for (const auto &s : d.slices) {
    CHECK(s.ratio_matches == 0);
    CHECK(s.correspondences == 0);
    CHECK(s.pnp_inliers == -1);
  }
  CHECK(d.correspondences_total == 0);
  CHECK(d.best_slice == -1);
}

TEST_CASE("PanoramaAlignmentOutcome_ToString_NamesEveryOutcome",
          "[slam][panorama][align]") {
  CHECK(std::string(to_string(PanoramaAlignmentOutcome::aligned)) == "aligned");
  CHECK(std::string(to_string(PanoramaAlignmentOutcome::no_slice_pnp)) ==
        "no_slice_pnp");
  CHECK(std::string(to_string(PanoramaAlignmentOutcome::upside_down)) ==
        "upside_down");
  CHECK(std::string(
            to_string(PanoramaAlignmentOutcome::too_few_pooled_inliers)) ==
        "too_few_pooled_inliers");
  CHECK(std::string(to_string(PanoramaAlignmentOutcome::correction_gate)) ==
        "correction_gate");
}

TEST_CASE("PanoramaAlignmentDiagnosticsJson_RejectedResult_CarriesOutcome",
          "[slam][panorama][align]") {
  PanoramaAlignmentResult r;
  r.pano_id = 4;
  r.seed_node = 9;
  r.outcome = PanoramaAlignmentOutcome::correction_gate;
  r.diagnostics.slices.resize(3);
  r.diagnostics.slices[0].ratio_matches = 12;
  r.diagnostics.final_inliers = 30;
  r.diagnostics.delta_from_seed_m = 7.5;

  const auto j = nlohmann::json::parse(panorama_alignment_diagnostics_json(r));
  CHECK(j.at("pano_id") == 4);
  CHECK(j.at("outcome") == "correction_gate");
  CHECK(j.at("aligned") == false);
  CHECK(j.at("slices").size() == 3);
  CHECK(j.at("slices")[0].at("ratio_matches") == 12);
  CHECK(j.at("final_inliers") == 30);
  CHECK(j.at("delta_from_seed_m") == 7.5);
}
