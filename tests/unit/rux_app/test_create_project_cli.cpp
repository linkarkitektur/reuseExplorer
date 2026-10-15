// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// `rux create project` flags (reusex-z1v.2.2) round-trip into the stage's
// parameters, and with no flags the stage runs at the library's
// vision::ProjectLabelsOptions defaults (STANDARDS §4: the CLI never defines a
// second default). Parsed through the real CLI11 setup; the stage-params sink
// (create/stage_bridge.hpp) keeps the subcommand from opening a project.

#include <catch2/catch_test_macros.hpp>

#include <create.hpp>
#include <create/stage_bridge.hpp>
#include <global-params.hpp>

#include <reusex/core/label_semantics.hpp>
#include <reusex/vision/project.hpp>

#include <CLI/CLI.hpp>
#include <nlohmann/json.hpp>

#include <memory>
#include <string>
#include <vector>

using nlohmann::json;

namespace {

struct Captured {
  std::string stage;
  json params;
};

Captured parse_create_project(std::vector<std::string> args) {
  CLI::App app{"rux"};
  auto global = std::make_shared<RuxOptions>();
  app.add_option("-p,--project", global->project_db);
  setup_subcommand_create(app, global);

  Captured got;
  rux::set_stage_params_sink(
      [&](std::string_view stage, const std::string &params) {
        got.stage = std::string(stage);
        got.params = json::parse(params);
      });
  args.insert(args.begin(), {"-p", "p.rux", "create", "project"});
  std::vector<std::string> reversed(args.rbegin(), args.rend());
  try {
    app.parse(reversed);
  } catch (const CLI::ParseError &e) {
    rux::set_stage_params_sink({});
    FAIL("CLI rejected the command: " << e.what());
  }
  rux::set_stage_params_sink({});
  return got;
}

} // namespace

TEST_CASE("CreateProjectCli_NoFlags_UsesLibraryDefaults",
          "[rux_app][cli][project]") {
  const auto got = parse_create_project({});
  REQUIRE(got.stage == "project");

  // Byte-identical to what the library writes for its own defaults.
  CHECK(got.params == json::parse(reusex::vision::project_labels_parameters(
                          reusex::vision::ProjectLabelsOptions{})));
  CHECK(got.params.at(reusex::core::kLabelEncodingKey) ==
        reusex::core::kLabelEncodingApiPlusOne);
}

TEST_CASE("CreateProjectCli_Flags_ReachTheStageUnchanged",
          "[rux_app][cli][project]") {
  const auto got =
      parse_create_project({"--max-depth", "4.5", "--depth-tolerance", "0.25"});
  REQUIRE(got.stage == "project");

  reusex::vision::ProjectLabelsOptions expected;
  expected.max_depth_m = 4.5F;
  expected.depth_tolerance_m = 0.25F;
  CHECK(got.params ==
        json::parse(reusex::vision::project_labels_parameters(expected)));
}

TEST_CASE("CreateProjectCli_NonPositiveMaxDepth_IsRejected",
          "[rux_app][cli][project]") {
  CLI::App app{"rux"};
  auto global = std::make_shared<RuxOptions>();
  setup_subcommand_create(app, global);
  rux::set_stage_params_sink([](std::string_view, const std::string &) {});
  std::vector<std::string> args = {"0", "--max-depth", "project", "create"};
  CHECK_THROWS_AS(app.parse(args), CLI::ValidationError);
  rux::set_stage_params_sink({});
}
