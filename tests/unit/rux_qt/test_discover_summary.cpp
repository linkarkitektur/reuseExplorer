// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

// Rediger's status after "Find kanter" reads the graph_discover stage's
// result summary (pipeline/optimize_stage.hpp). The last case runs the real
// stage so a change to that summary's wording fails here, not silently in
// the GUI.

#include <catch2/catch_test_macros.hpp>

#include <rux_qt/workspace_logic.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/pipeline/optimize_stage.hpp>

#include "../../support/pose_fixture.hpp"
#include "../../support/temp_path.hpp"

#include <nlohmann/json.hpp>

#include <fstream>
#include <string>

using namespace rux::qt;

TEST_CASE("DiscoverSummary_ReadsEveryCount", "[rux_qt][discover]") {
  const auto c = parse_discover_summary(
      "40 proposed, 12 matched, 3 PCM-rejected, 2 over the PCM cap, 4 "
      "already stored, 1 rejected before, 5 added, 6 stored rows given a "
      "measurement");
  REQUIRE(c);
  CHECK_FALSE(c->dry_run);
  CHECK(c->odometry_initialised == 0);
  CHECK(c->proposed == 40);
  CHECK(c->matched == 12);
  CHECK(c->pcm_rejected == 3);
  CHECK(c->pcm_capped == 2);
  CHECK(c->skipped_existing == 4);
  CHECK(c->skipped_rejected == 1);
  CHECK(c->added == 5);
  CHECK(c->filled == 6);
}

TEST_CASE("DiscoverSummary_DryRunAndInitialisedOdometry",
          "[rux_qt][discover]") {
  const auto c = parse_discover_summary(
      "dry run: no odometry stored: initialised 1234 odometry edges first; 0 "
      "proposed, 0 matched, 0 PCM-rejected, 0 over the PCM cap, 0 already "
      "stored, 0 rejected before, 0 added, 0 stored rows given a measurement");
  REQUIRE(c);
  CHECK(c->dry_run);
  CHECK(c->odometry_initialised == 1234);
  CHECK(c->added == 0);
}

TEST_CASE("DiscoverSummary_OtherText_IsNotACount", "[rux_qt][discover]") {
  CHECK_FALSE(parse_discover_summary(""));
  CHECK_FALSE(parse_discover_summary("12 frames, 3 landmarks, 1.0 -> 0.5 "
                                     "error, max shift 0.0100 m"));
  // A count missing, or one not a number, reads as no summary at all.
  CHECK_FALSE(parse_discover_summary(
      "40 proposed, 12 matched, 3 PCM-rejected, 2 over the PCM cap"));
  CHECK_FALSE(parse_discover_summary(
      "x proposed, 12 matched, 3 PCM-rejected, 2 over the PCM cap, 4 "
      "already stored, 1 rejected before, 5 added, 6 stored rows given a "
      "measurement"));
}

TEST_CASE("DiscoverSummary_ReadsTheRealStageResult", "[rux_qt][discover]") {
  using namespace reusex::pipeline;
  using namespace reusex::test_support;
  TempPath project("test_qt_discover_summary");
  TempPath edges("test_qt_discover_summary", ".json");
  {
    reusex::ProjectDB db(project.path);
    for (int k = 1; k <= 8; ++k)
      db.save_sensor_frame(k, make_color(8, 8), make_depth(8, 8, 1.0),
                           cv::Mat(), translation_pose(0.5 * k, 0, 0),
                           make_intrinsics(8, 8, 4, 4, 8, 8), k);
  }
  // One file edge 1 -> 8 that disagrees with the seed (3.5 m) by 1 m, so the
  // seed-disagreement gate keeps it (as test_optimize_stage.cpp).
  nlohmann::json e = {{"node_i", 1},
                      {"node_j", 8},
                      {"sigma_rot", 0.05},
                      {"sigma_trans", 0.1},
                      {"inliers", 100}};
  e["T_ij"] = {1, 0, 0, 2.5, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  nlohmann::json j = {{"schema", "reusex.loop_edges.v1"},
                      {"producer", "test"},
                      {"edges", nlohmann::json::array({e})}};
  std::ofstream(edges.path) << j.dump();

  StageContext ctx;
  ctx.project = project.path;
  ctx.stage = JobStage::graph_discover;
  ctx.parameters = nlohmann::json{{"loop_closure", false},
                                  {"loop_edges_file", edges.path.string()}}
                       .dump();
  const StageResult r = stage_executor_with_optimize()(ctx);
  INFO(r.message);
  REQUIRE(r.ok);
  const auto c = parse_discover_summary(r.message);
  REQUIRE(c);
  CHECK_FALSE(c->dry_run);
  CHECK(c->odometry_initialised == 7);
  CHECK(c->added == 1);
  CHECK(c->pcm_rejected == 0);
}
