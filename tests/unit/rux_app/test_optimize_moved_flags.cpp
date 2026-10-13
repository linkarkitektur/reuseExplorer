// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// The 2026-10-13 CLI break (spec D5): loop-edge discovery flags moved from
// `rux optimize` to `rux graph discover`; `optimize` refuses them with a
// pointer instead of "unknown option" or, worse, silently ignoring them.

#include <catch2/catch_test_macros.hpp>

#include <create/stage_bridge.hpp>
#include <global-params.hpp>
#include <optimize.hpp>

#include <CLI/CLI.hpp>
#include <spdlog/spdlog.h>

#include <memory>
#include <string>
#include <vector>

namespace {

bool parse_optimize(const std::vector<std::string> &args, bool &captured) {
  // The subcommand logs through spdlog's default logger, which a GuiLaunch
  // test earlier in the same binary may have shut down (spdlog::shutdown()
  // leaves it null). Run against a sink-less logger and restore afterwards.
  const auto previous_logger = spdlog::default_logger();
  if (!previous_logger)
    spdlog::set_default_logger(
        std::make_shared<spdlog::logger>("test_optimize_moved_flags"));
  CLI::App app{"rux"};
  auto global = std::make_shared<RuxOptions>();
  app.add_option("-p,--project", global->project_db);
  setup_subcommand_optimize(app, global);
  captured = false;
  rux::set_stage_params_sink(
      [&](std::string_view, const std::string &) { captured = true; });
  std::vector<std::string> reversed(args.rbegin(), args.rend());
  bool refused = false;
  try {
    app.parse(reversed);
  } catch (const CLI::RuntimeError &) {
    refused = true;
  }
  rux::set_stage_params_sink({});
  if (!previous_logger)
    spdlog::set_default_logger(nullptr);
  return refused;
}

} // namespace

TEST_CASE("OptimizeMovedFlags_EveryMovedFlag_IsRefused",
          "[rux_app][cli][optimize]") {
  REQUIRE(optimize_moved_flags().size() == 24);
  for (const auto &[flag, takes_value] : optimize_moved_flags()) {
    INFO(flag);
    std::vector<std::string> args = {"optimize", flag};
    if (takes_value)
      args.push_back("1");
    bool captured = false;
    CHECK(parse_optimize(args, captured));
    CHECK_FALSE(captured); // refused before the stage parameters were built
  }
}

TEST_CASE("OptimizeMovedFlags_LoopTrustStaysOnOptimize",
          "[rux_app][cli][optimize]") {
  bool captured = false;
  CHECK_FALSE(parse_optimize(
      {"optimize", "--loop-trust", "--loop-trust-inlier-cost", "150"},
      captured));
  CHECK(captured);
}

TEST_CASE("OptimizeMovedFlags_RefusalMessage_PointsAtGraphDiscover",
          "[rux_app][cli][optimize]") {
  for (const auto &[flag, takes_value] : optimize_moved_flags()) {
    INFO(flag);
    const std::string msg = optimize_moved_flag_message(flag);
    CHECK(msg.find("rux graph discover") != std::string::npos);
    CHECK(msg.find(flag) != std::string::npos);
  }
}

TEST_CASE("OptimizeMovedFlags_LoopClosureRefusal_SaysItIsOnByDefault",
          "[rux_app][cli][optimize]") {
  // `rux graph discover` runs ORB loop closure by default and has no
  // --loop-closure flag: the refusal must not suggest one.
  const std::string msg = optimize_moved_flag_message("--loop-closure");
  CHECK(msg.find("rux graph discover --loop-closure") == std::string::npos);
  CHECK(msg.find("on by default") != std::string::npos);
}
