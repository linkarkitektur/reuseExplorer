// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// reusex-z1v.2.3 (W0.3): `create mesh -a/--angle` and `-l/--distance` parsed
// angle_threshold/distance_threshold, but reusex::geometry::MeshOptions has
// no matching fields and mesh.cpp never forwarded them anywhere — they were
// silently ignored. Removed rather than wired (bead's "removal preferred").

#include <catch2/catch_test_macros.hpp>

#include <create/mesh.hpp>
#include <create/stage_bridge.hpp>
#include <global-params.hpp>

#include <CLI/CLI.hpp>

#include <memory>
#include <string>
#include <vector>

namespace {

bool parse_mesh(const std::vector<std::string> &args) {
  CLI::App app{"rux"};
  auto global = std::make_shared<RuxOptions>();
  app.add_option("-p,--project", global->project_db);
  setup_subcommand_create_mesh(app, global);

  // Capture the stage parameters instead of actually opening a project and
  // running the solver.
  rux::set_stage_params_sink([](std::string_view, const std::string &) {});

  std::vector<std::string> reversed(args.rbegin(), args.rend());
  bool refused = false;
  try {
    app.parse(reversed);
  } catch (const CLI::ParseError &) {
    refused = true;
  }

  rux::set_stage_params_sink({});
  return refused;
}

} // namespace

TEST_CASE("CreateMeshFlags_AngleAndDistance_AreGone", "[rux_app][cli][mesh]") {
  CHECK(parse_mesh({"mesh", "-a", "15"}));
  CHECK(parse_mesh({"mesh", "--angle", "15"}));
  CHECK(parse_mesh({"mesh", "-l", "0.3"}));
  CHECK(parse_mesh({"mesh", "--distance", "0.3"}));
}

TEST_CASE("CreateMeshFlags_OtherFlags_StillParse", "[rux_app][cli][mesh]") {
  CHECK_FALSE(parse_mesh({"mesh", "--threshold", "0.5"}));
  CHECK_FALSE(parse_mesh({"mesh", "--offset", "0.2"}));
  CHECK_FALSE(parse_mesh({"mesh", "-f", "rooms == 2"}));
}

TEST_CASE("CreateMeshFlags_HelpText_HasNoAngleOrDistance",
          "[rux_app][cli][mesh]") {
  CLI::App app{"rux"};
  auto global = std::make_shared<RuxOptions>();
  app.add_option("-p,--project", global->project_db);
  setup_subcommand_create_mesh(app, global);

  CLI::App *mesh_cmd = app.get_subcommand("mesh");
  REQUIRE(mesh_cmd != nullptr);
  const std::string help = mesh_cmd->help();
  CHECK(help.find("--angle") == std::string::npos);
  CHECK(help.find("--distance") == std::string::npos);
  CHECK(help.find("-a 15") == std::string::npos); // footer example line gone
  CHECK(help.find("Angle threshold") == std::string::npos);
}
