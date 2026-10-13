// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// `rux graph ...` subcommands parsed and run through the real CLI11 setup.

#include <catch2/catch_test_macros.hpp>

#include <create/stage_bridge.hpp>
#include <global-params.hpp>
#include <graph.hpp>
#include <graph/discover.hpp>
#include <optimize.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/slam/graph_discover.hpp>

#include "../../support/pose_fixture.hpp"
#include "../../support/temp_path.hpp"

#include <CLI/CLI.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

using namespace reusex::test_support;

namespace {

/// Parse and run `rux -p <project> <args...>`; true when it succeeded.
bool run_rux(const std::filesystem::path &project,
             std::vector<std::string> args) {
  CLI::App app{"rux"};
  auto global = std::make_shared<RuxOptions>();
  app.add_option("-p,--project", global->project_db);
  setup_subcommand_graph(app, global);
  args.insert(args.begin(), {"-p", project.string()});
  std::vector<std::string> reversed(args.rbegin(), args.rend());
  try {
    app.parse(reversed);
  } catch (const CLI::RuntimeError &) {
    return false;
  }
  return true;
}

void make_frames(const std::filesystem::path &path, int n) {
  reusex::ProjectDB db(path);
  for (int k = 1; k <= n; ++k)
    db.save_sensor_frame(k, make_color(8, 8), make_depth(8, 8, 1.0), cv::Mat(),
                         translation_pose(0.1 * k, 0, 0),
                         make_intrinsics(8, 8, 4, 4, 8, 8), k);
}

} // namespace

TEST_CASE("GraphCli_Init_RefusesASecondInitWithoutForce",
          "[rux_app][cli][graph]") {
  TempPath tmp("test_graph_cli_init");
  make_frames(tmp.path, 4);
  CHECK(run_rux(tmp.path, {"graph", "init"}));
  CHECK_FALSE(run_rux(tmp.path, {"graph", "init"}));
  CHECK(run_rux(tmp.path, {"graph", "init", "--force"}));
  reusex::ProjectDB db(tmp.path, /*readOnly=*/true);
  CHECK(db.list_pose_graph_edges().size() == 3);
  const auto log = db.pipeline_log();
  CHECK(std::count_if(log.begin(), log.end(), [](const auto &e) {
          return e.stage == "graph_init" && e.status == "success";
        }) == 2);
}

TEST_CASE("GraphCli_Init_NeedsTwoFrames", "[rux_app][cli][graph]") {
  TempPath tmp("test_graph_cli_init_empty");
  make_frames(tmp.path, 1);
  CHECK_FALSE(run_rux(tmp.path, {"graph", "init"}));
}

TEST_CASE("GraphCli_DiscoverFlagDefaults_MatchTheLibrary",
          "[rux_app][cli][graph]") {
  // STANDARDS §4: the CLI defaults mirror GraphDiscoverOptions{} through its
  // memberwise operator== (nested LoopClosure/PanoramaLoop options included).
  CHECK(graph_discover_options(SubcommandGraphDiscoverOptions{}) ==
        reusex::slam::GraphDiscoverOptions{});
}

TEST_CASE("GraphCli_EveryFlagMovedFromOptimize_ParsesOnDiscover",
          "[rux_app][cli][graph]") {
  for (const auto &[flag, takes_value] : optimize_moved_flags()) {
    if (flag == "--loop-closure")
      continue; // ORB is on by default in discover; its switch is
                // --no-loop-closure
    INFO(flag);
    CLI::App app{"rux"};
    auto global = std::make_shared<RuxOptions>();
    app.add_option("-p,--project", global->project_db);
    setup_subcommand_graph(app, global);
    std::string stage;
    rux::set_stage_params_sink(
        [&](std::string_view s, const std::string &) { stage = s; });
    std::vector<std::string> args = {"graph", "discover", flag};
    if (takes_value)
      args.push_back(flag == "--loop-proposal" ? "spatial" : "1");
    std::vector<std::string> reversed(args.rbegin(), args.rend());
    CHECK_NOTHROW(app.parse(reversed));
    rux::set_stage_params_sink({});
    CHECK(stage == "graph_discover");
  }
}

TEST_CASE("GraphCli_Prune_RequiresACriterionAndRejects",
          "[rux_app][cli][graph]") {
  TempPath tmp("test_graph_cli_prune");
  make_frames(tmp.path, 4);
  CHECK(run_rux(tmp.path, {"graph", "init"}));
  CHECK_FALSE(run_rux(tmp.path, {"graph", "prune"}));
  CHECK(run_rux(tmp.path, {"graph", "prune", "--pair", "2,3", "--dry-run"}));
  {
    reusex::ProjectDB db(tmp.path, /*readOnly=*/true);
    CHECK(db.pose_graph_edge(2, 3, "odometry")->status == "auto");
  }
  CHECK(run_rux(tmp.path, {"graph", "prune", "--pair", "3,2"}));
  reusex::ProjectDB db(tmp.path, /*readOnly=*/true);
  CHECK(db.pose_graph_edge(2, 3, "odometry")->status == "rejected");
  CHECK_FALSE(run_rux(tmp.path, {"graph", "prune", "--pair", "nonsense"}));
}
