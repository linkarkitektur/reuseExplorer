// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// `rux analyze solid` parsed and run through the real CLI11 setup
// (reusex-z1v.2.5).

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <analyze.hpp>
#include <global-params.hpp>

#include <reusex/core/ProjectDB.hpp>

#include "../../support/temp_path.hpp"

#include <CLI/CLI.hpp>
#include <nlohmann/json.hpp>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

using namespace reusex::test_support;
using Catch::Matchers::WithinAbs;

namespace {

std::filesystem::path fixture(const char *name) {
  return std::filesystem::path(__FILE__)
             .parent_path()
             .parent_path()
             .parent_path() /
         "fixtures" / "solid_model" / name;
}

/// Parse and run `rux -p <project> analyze solid <args...>`.
bool run_rux(const std::filesystem::path &project,
             std::vector<std::string> args) {
  CLI::App app{"rux"};
  auto global = std::make_shared<RuxOptions>();
  app.add_option("-p,--project", global->project_db);
  setup_subcommand_analyze(app, global);
  args.insert(args.begin(), {"-p", project.string(), "analyze", "solid"});
  std::vector<std::string> reversed(args.rbegin(), args.rend());
  try {
    app.parse(reversed);
  } catch (const CLI::Error &) {
    return false;
  }
  return true;
}

nlohmann::json read_json(const std::filesystem::path &p) {
  std::ifstream in(p);
  return nlohmann::json::parse(in);
}

} // namespace

TEST_CASE("AnalyzeSolidCli_ProjectWithoutSolidModelTables_ReportsAllNa",
          "[rux_app][cli][analyze]") {
  TempPath project("test_analyze_solid_project");
  {
    reusex::ProjectDB db(project.path);
  }
  TempPath out("test_analyze_solid_out");
  REQUIRE(run_rux(project.path,
                  {"--reference", fixture("two_rooms.plan.json").string(),
                   "--json", "-o", out.path.string()}));
  const auto j = read_json(out.path);
  for (const char *k :
       {"room_count_error", "room_iou_mean", "wall_position_rms_m",
        "wall_thickness_err_mean_m", "wall_thickness_err_p95_m", "wall_recall",
        "wall_precision", "opening_recall", "opening_precision", "fit_mean_m",
        "fit_within_share", "seen_share_mean", "assumed_share"}) {
    INFO(k);
    CHECK(j.at(k).is_null());
  }
}

TEST_CASE("AnalyzeSolidCli_SyntheticDetectionAgainstReference_ScoresIt",
          "[rux_app][cli][analyze]") {
  TempPath out("test_analyze_solid_model_out");
  // --model needs no project: a path that does not exist is fine.
  TempPath no_project("test_analyze_solid_absent");
  REQUIRE(run_rux(no_project.path,
                  {"--reference", fixture("two_rooms.plan.json").string(),
                   "--model", fixture("two_rooms.detected.json").string(), "-o",
                   out.path.string()}));
  const auto j = read_json(out.path);
  CHECK(j.at("room_count_error") == 0);
  CHECK_THAT(j.at("wall_recall").get<double>(), WithinAbs(6.0 / 7.0, 1e-9));
  CHECK_THAT(j.at("wall_precision").get<double>(), WithinAbs(6.0 / 7.0, 1e-9));
  CHECK_THAT(j.at("wall_position_rms_m").get<double>(),
             WithinAbs(std::sqrt(0.0112 / 12.0), 1e-9));
  CHECK_THAT(j.at("wall_thickness_err_mean_m").get<double>(),
             WithinAbs(0.07 / 6.0, 1e-9));
  CHECK_THAT(j.at("wall_thickness_err_p95_m").get<double>(),
             WithinAbs(0.03, 1e-9));
  CHECK_THAT(j.at("opening_recall").get<double>(), WithinAbs(2.0 / 3.0, 1e-9));
  CHECK_THAT(j.at("opening_precision").get<double>(),
             WithinAbs(2.0 / 3.0, 1e-9));
  // West: L-shaped reference vs a rectangle swallowing the closet; east: a
  // rectangle fully inside the reference.
  const double west = 15.8484 / (16.04 + 17.6148 - 15.8484);
  const double east = 17.02 / 17.86;
  CHECK_THAT(j.at("room_iou_mean").get<double>(),
             WithinAbs((west + east) / 2.0, 1e-6));
  CHECK_THAT(j.at("seen_share_mean").get<double>(),
             WithinAbs(4.65 / 7.0, 1e-9));
  CHECK_THAT(j.at("assumed_share").get<double>(), WithinAbs(2.0 / 7.0, 1e-9));
  CHECK(j.at("fit_mean_m").is_null());
}

TEST_CASE("AnalyzeSolidCli_InvalidReference_Fails", "[rux_app][cli][analyze]") {
  TempPath project("test_analyze_solid_bad_ref_project");
  {
    reusex::ProjectDB db(project.path);
  }
  TempPath bad("test_analyze_solid_bad_ref");
  {
    std::ofstream f(bad.path);
    f << R"({"model_version": 1, "walls": [{"axis": [[0, 0], [0, 0]]}]})";
  }
  CHECK_FALSE(run_rux(project.path, {"--reference", bad.path.string()}));
}

TEST_CASE("AnalyzeSolidCli_NoProjectAndNoModel_Fails",
          "[rux_app][cli][analyze]") {
  TempPath no_project("test_analyze_solid_no_project");
  CHECK_FALSE(
      run_rux(no_project.path,
              {"--reference", fixture("two_rooms.plan.json").string()}));
}
