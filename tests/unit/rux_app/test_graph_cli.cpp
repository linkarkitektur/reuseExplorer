// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// `rux graph ...` subcommands parsed and run through the real CLI11 setup.

#include <catch2/catch_test_macros.hpp>

#include <global-params.hpp>
#include <graph.hpp>

#include <reusex/core/ProjectDB.hpp>

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
