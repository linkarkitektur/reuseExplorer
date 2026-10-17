// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

// pipeline::stage_executor_with_optimize (header-only, pipeline/
// optimize_stage.hpp): the executor the Qt client and ruxd's web GUI both hand
// their job runners. The light binary links reusex_pipeline and reusex_slam,
// which is exactly what the header needs.

#include <catch2/catch_test_macros.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/pipeline/optimize_stage.hpp>

#include "../../support/pose_fixture.hpp"
#include "../../support/temp_path.hpp"

#include <nlohmann/json.hpp>

#include <fstream>
#include <string>

using namespace reusex::pipeline;
using namespace reusex::test_support;

TEST_CASE("OptimizeStage_BadParameters_RefusedAsInvalid",
          "[pipeline][optimize_stage]") {
  TempPath project("test_optimize_stage_bad");
  {
    reusex::ProjectDB db(project.path);
  }

  const auto exec = stage_executor_with_optimize();
  StageContext ctx;
  ctx.project = project.path;
  ctx.stage = JobStage::optimize;

  for (const std::string params :
       {"[1]", "{oops", R"({"min_observations":"seven"})"}) {
    ctx.parameters = params;
    const StageResult r = exec(ctx);
    INFO(params << " -> " << r.message);
    CHECK_FALSE(r.ok);
    CHECK(r.invalid_input);
    CHECK_FALSE(r.message.empty());
  }
}

TEST_CASE("OptimizeStage_OtherStages_GoToTheDefaultExecutor",
          "[pipeline][optimize_stage]") {
  TempPath project("test_optimize_stage_delegate");
  {
    reusex::ProjectDB db(project.path);
  }

  StageContext ctx;
  ctx.project = project.path;
  ctx.stage = JobStage::planes; // no cloud: the default executor refuses
  const StageResult via_optimize_exec = stage_executor_with_optimize()(ctx);
  const StageResult via_default = default_stage_executor()(ctx);
  CHECK(via_optimize_exec.ok == via_default.ok);
  CHECK(via_optimize_exec.invalid_input == via_default.invalid_input);
  CHECK_FALSE(via_optimize_exec.ok);
}

TEST_CASE("GraphDiscoverStage_BadParameters_RefusedAsInvalid",
          "[pipeline][optimize_stage]") {
  TempPath project("test_discover_stage_bad");
  {
    reusex::ProjectDB db(project.path);
  }
  StageContext ctx;
  ctx.project = project.path;
  ctx.stage = JobStage::graph_discover;
  for (const std::string params :
       {"[1]", R"({"loop_proposal":"psychic"})",
        R"({"loop_closure":false})"}) { // last: no source selected
    ctx.parameters = params;
    const StageResult r = stage_executor_with_optimize()(ctx);
    INFO(params << " -> " << r.message);
    CHECK_FALSE(r.ok);
    CHECK(r.invalid_input);
  }
}

TEST_CASE("GraphDiscoverStage_DefaultExecutor_Refuses",
          "[pipeline][optimize_stage]") {
  TempPath project("test_discover_stage_default");
  {
    reusex::ProjectDB db(project.path);
  }
  StageContext ctx;
  ctx.project = project.path;
  ctx.stage = JobStage::graph_discover;
  CHECK_FALSE(default_stage_executor()(ctx).ok);
}

TEST_CASE("GraphDiscoverStage_NoGraph_ReportsTheOdometryItInitialised",
          "[pipeline][optimize_stage]") {
  TempPath project("test_discover_stage_init");
  TempPath edges("test_discover_stage_init", ".json");
  {
    reusex::ProjectDB db(project.path);
    for (int k = 1; k <= 8; ++k)
      db.save_sensor_frame(k, make_color(8, 8), make_depth(8, 8, 1.0),
                           cv::Mat(), translation_pose(0.5 * k, 0, 0),
                           make_intrinsics(8, 8, 4, 4, 8, 8), k);
  }
  // One file edge 1 -> 8 that disagrees with the seed (3.5 m) by 1 m, so the
  // seed-disagreement gate keeps it.
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
  ctx.job_id = "job-discover-7";
  const StageResult r = stage_executor_with_optimize()(ctx);
  INFO(r.message);
  REQUIRE(r.ok);
  CHECK(r.message.find("initialised 7 odometry edges") != std::string::npos);
  REQUIRE(r.outputs.size() == 1);
  CHECK(r.outputs[0].name == "pose_graph_edges");
  CHECK(r.outputs[0].count == 8); // 7 odometry + 1 loop edge

  // #464 follow-up (reusex-xlg.5): stage_executor_with_optimize() used to
  // bypass run_stage() entirely for optimize/graph_discover, so a Qt
  // Pipeline/Rediger or ruxd run of either stage left no trace in
  // pipeline_log (the GUI's history view). This must write the same kind of
  // row run_stage() writes for every other stage, job id folded in.
  reusex::ProjectDB readback(project.path, /*readOnly=*/true);
  const auto log = readback.pipeline_log();
  REQUIRE(log.size() == 1);
  CHECK(log.front().stage == "graph_discover");
  CHECK(log.front().status == "success");
  CHECK_FALSE(log.front().finished_at.empty());
  const auto params = nlohmann::json::parse(log.front().parameters);
  CHECK(params.at("job_id") == "job-discover-7");
  CHECK(params.at("loop_closure") == false);
}

TEST_CASE("OptimizeStage_MissingSensorFrames_RefusedAsInvalid",
          "[pipeline][optimize_stage]") {
  // reusex-xlg.15: stage_executor_with_optimize() used to run
  // run_optimize_stage() without the stage's documented input contract
  // (core::PipelineStage::optimize needs >= 2 sensor_frames with depth),
  // unlike run_stage()'s dispatch path. An empty project used to come back
  // as a misleading "success: poses left unchanged" instead of a clear
  // refusal.
  TempPath project("test_optimize_stage_missing_inputs");
  {
    reusex::ProjectDB db(project.path);
  }

  StageContext ctx;
  ctx.project = project.path;
  ctx.stage = JobStage::optimize;

  const StageResult r = stage_executor_with_optimize()(ctx);
  INFO(r.message);
  CHECK_FALSE(r.ok);
  CHECK(r.invalid_input);
  CHECK(r.message.find("stage inputs not satisfied") != std::string::npos);
}

TEST_CASE("GraphDiscoverStage_MissingSensorFrames_RefusedAsInvalid",
          "[pipeline][optimize_stage]") {
  // reusex-xlg.15, graph_discover side: the contract needs >= 2
  // sensor_frames; an empty project must be refused up front rather than
  // reaching slam::discover_loop_edges() with nothing to match.
  TempPath project("test_discover_stage_missing_inputs");
  {
    reusex::ProjectDB db(project.path);
  }

  StageContext ctx;
  ctx.project = project.path;
  ctx.stage = JobStage::graph_discover;

  const StageResult r = stage_executor_with_optimize()(ctx);
  INFO(r.message);
  CHECK_FALSE(r.ok);
  CHECK(r.invalid_input);
  CHECK(r.message.find("stage inputs not satisfied") != std::string::npos);
}

TEST_CASE("OptimizeStage_InvalidInput_RecordsPipelineLogEntry",
          "[pipeline][optimize_stage]") {
  TempPath project("test_optimize_stage_log_invalid");
  {
    // Enough sensor_frames (with depth) to satisfy the stage's input
    // contract, so this test exercises the bad-PARAMETER path specifically,
    // distinct from OptimizeStage_MissingSensorFrames_RefusedAsInvalid above.
    reusex::ProjectDB db(project.path);
    for (int k = 1; k <= 2; ++k)
      db.save_sensor_frame(k, make_color(8, 8), make_depth(8, 8, 1.0),
                           cv::Mat(), translation_pose(0.5 * k, 0, 0),
                           make_intrinsics(8, 8, 4, 4, 8, 8), k);
  }

  StageContext ctx;
  ctx.project = project.path;
  ctx.stage = JobStage::optimize;
  // Well-formed JSON object, but a type the optimize-parameters reader
  // rejects: reaches run_optimize_stage() (so a log row IS opened), which
  // returns StageResult::invalid() before touching any scan data.
  ctx.parameters = R"({"min_observations":"seven"})";
  ctx.job_id = "job-optimize-9";

  const StageResult r = stage_executor_with_optimize()(ctx);
  CHECK_FALSE(r.ok);
  CHECK(r.invalid_input);
  CHECK_FALSE(r.message.empty());

  reusex::ProjectDB readback(project.path, /*readOnly=*/true);
  const auto log = readback.pipeline_log();
  REQUIRE(log.size() == 1);
  // The CLI-compatible name (rux optimize has always written this), so the
  // GUI and CLI paths do not fill pipeline_log with two names for the same
  // operation.
  CHECK(log.front().stage == "pose_optimization_plane_graph");
  CHECK(log.front().status == "failed");
  CHECK_FALSE(log.front().error_msg.empty());
  const auto params = nlohmann::json::parse(log.front().parameters);
  CHECK(params.at("job_id") == "job-optimize-9");
}
