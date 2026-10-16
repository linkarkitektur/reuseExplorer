// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// `rux edit plan --import / --export` parsed and run through the real CLI11
// setup (reusex-z1v.2.6).

#include <catch2/catch_test_macros.hpp>

#include <edit.hpp>
#include <global-params.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/io/solid_model_json.hpp>

#include "../../support/source_dir.hpp"
#include "../../support/temp_path.hpp"

#include <CLI/CLI.hpp>

#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

using namespace reusex::test_support;

namespace {

std::filesystem::path fixture(const char *name) {
  return reusex::test_support::source_dir() / "tests" / "fixtures" /
         "solid_model" / name;
}

/// Parse and run `rux -p <project> edit plan <args...>`; the exit code.
int rux_exit(const std::filesystem::path &project,
             std::vector<std::string> args) {
  CLI::App app{"rux"};
  auto global = std::make_shared<RuxOptions>();
  app.add_option("-p,--project", global->project_db);
  setup_subcommand_edit(app, global);
  args.insert(args.begin(), {"-p", project.string(), "edit", "plan"});
  std::vector<std::string> reversed(args.rbegin(), args.rend());
  try {
    app.parse(reversed);
  } catch (const CLI::Error &e) {
    return e.get_exit_code() == 0 ? -1 : e.get_exit_code();
  }
  return 0;
}

} // namespace

TEST_CASE("EditPlan_DryRunImportExport_IsTheImportedModel",
          "[rux_app][edit_plan]") {
  TempPath project("test_edit_plan_dry");
  TempPath out("test_edit_plan_dry_out", ".json");
  REQUIRE(rux_exit(project.path,
                   {"--import", fixture("two_rooms.plan.json").string(),
                    "--dry-run", "--export", out.path.string()}) == 0);
  // --dry-run never opens (or creates) the project.
  CHECK_FALSE(std::filesystem::exists(project.path));

  auto expected =
      reusex::io::solid_model_from_file(fixture("two_rooms.plan.json"));
  reusex::io::mark_imported(expected);
  const auto written = reusex::io::solid_model_from_file(out.path);
  CHECK(reusex::io::solid_model_to_json(written) ==
        reusex::io::solid_model_to_json(expected));
  for (const auto &w : written.walls)
    CHECK((w.source == "imported" && w.locked));

  // The written file is a fixed point: importing it again changes nothing.
  TempPath again("test_edit_plan_dry_again", ".json");
  REQUIRE(rux_exit(project.path, {"--import", out.path.string(), "--dry-run",
                                  "--export", again.path.string()}) == 0);
  CHECK(reusex::io::solid_model_to_json(reusex::io::solid_model_from_file(
            again.path)) == reusex::io::solid_model_to_json(written));
}

TEST_CASE("EditPlan_YamlImportJsonExport_SameModelEitherExtension",
          "[rux_app][edit_plan]") {
  // reusex-yf4.1: --import and --export pick JSON or YAML by extension.
  TempPath project("test_edit_plan_yaml");
  TempPath yaml_out("test_edit_plan_yaml_out", ".yaml");
  REQUIRE(rux_exit(project.path,
                   {"--import", fixture("two_rooms.plan.json").string(),
                    "--dry-run", "--export", yaml_out.path.string()}) == 0);
  CHECK_FALSE(std::filesystem::exists(project.path)); // --dry-run

  auto expected =
      reusex::io::solid_model_from_file(fixture("two_rooms.plan.json"));
  reusex::io::mark_imported(expected);
  const auto written_yaml = reusex::io::solid_model_from_file(yaml_out.path);
  CHECK(reusex::io::solid_model_to_json(written_yaml) ==
        reusex::io::solid_model_to_json(expected));

  // Import the YAML this just wrote and export it back out as JSON: the same
  // model either way round, through the real CLI parsing both times.
  TempPath json_out("test_edit_plan_yaml_roundtrip", ".json");
  REQUIRE(
      rux_exit(project.path, {"--import", yaml_out.path.string(), "--dry-run",
                              "--export", json_out.path.string()}) == 0);
  CHECK(reusex::io::solid_model_to_json(reusex::io::solid_model_from_file(
            json_out.path)) == reusex::io::solid_model_to_json(written_yaml));
}

TEST_CASE("EditPlan_InvalidPlan_IsAnInvalidArgument", "[rux_app][edit_plan]") {
  TempPath project("test_edit_plan_invalid");
  TempPath bad("test_edit_plan_bad", ".json");
  {
    std::ofstream o(bad.path);
    o << R"({"model_version": 1, "walls": [{"guid": "w1", "axs": []}]})";
  }
  CHECK(rux_exit(project.path, {"--import", bad.path.string(), "--dry-run"}) ==
        3);
}

TEST_CASE("EditPlan_ImportIntoAProjectWithoutTables_FailsLoudly",
          "[rux_app][edit_plan]") {
  TempPath project("test_edit_plan_notables");
  {
    reusex::ProjectDB db(project.path);
  }
  // Nothing can be stored before the solid-model tables exist: a silent
  // success would leave the user thinking the reference is in the project.
  CHECK(rux_exit(project.path,
                 {"--import", fixture("two_rooms.plan.json").string()}) == 4);
}

TEST_CASE("EditPlan_Export_WritesTheProjectModel", "[rux_app][edit_plan]") {
  TempPath project("test_edit_plan_export");
  TempPath out("test_edit_plan_export_out", ".json");
  {
    reusex::ProjectDB db(project.path);
  }
  REQUIRE(rux_exit(project.path, {"--export", out.path.string()}) == 0);
  const auto m = reusex::io::solid_model_from_file(out.path);
  CHECK_FALSE(m.has_walls()); // no table yet: nothing provided
}

TEST_CASE("EditPlan_NeedsImportOrExport", "[rux_app][edit_plan]") {
  TempPath project("test_edit_plan_none");
  CHECK(rux_exit(project.path, {}) != 0);
  CHECK(rux_exit(project.path, {"--dry-run"}) != 0);
}
