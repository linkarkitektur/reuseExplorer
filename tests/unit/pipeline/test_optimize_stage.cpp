// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

// pipeline::stage_executor_with_optimize (header-only, pipeline/
// optimize_stage.hpp): the executor the Qt client and ruxd's web GUI both hand
// their job runners. The light binary links reusex_pipeline and reusex_slam,
// which is exactly what the header needs.

#include <catch2/catch_test_macros.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/pipeline/optimize_stage.hpp>

#include "../../support/temp_path.hpp"

#include <string>

using namespace reusex::pipeline;
using reusex::test_support::TempPath;

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
