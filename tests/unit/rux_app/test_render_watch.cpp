// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// `rux render --plan <gt> --watch` (reusex-yf4.2): the headless half of the
// ground-truth authoring loop, driven one pass at a time -- no sleeping.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <global-params.hpp>
#include <plan_watch.hpp>
#include <render.hpp>
#include <render_watch.hpp>

#include <reusex/core/ProjectDB.hpp>

#include "../../support/temp_path.hpp"

#include <CLI/CLI.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

using namespace reusex::test_support;
using Catch::Matchers::ContainsSubstring;
namespace fs = std::filesystem;

namespace {

/// One wall face y = 0, x in [0, 4], z in [0, 2.5] (5 cm grid) as 'cloud'.
void make_project(const fs::path &path) {
  reusex::Cloud cloud;
  for (double x = 0.0; x <= 4.0 + 1e-9; x += 0.05)
    for (double z = 0.0; z <= 2.5 + 1e-9; z += 0.05) {
      pcl::PointXYZRGB p;
      p.x = static_cast<float>(x);
      p.z = static_cast<float>(z);
      p.r = p.g = p.b = 200;
      cloud.push_back(p);
    }
  reusex::ProjectDB db(path);
  db.save_point_cloud("cloud", cloud);
}

/// A one-wall plan whose wall axis runs along y = @p y.
std::string plan_yaml(double y) {
  return "model_version: 1\n"
         "walls:\n"
         "  - guid: south\n"
         "    storey: 0\n"
         "    axis: [[0, " +
         std::to_string(y) + "], [4, " + std::to_string(y) +
         "]]\n"
         "    height: 2.5\n";
}

void write(const fs::path &p, const std::string &text) {
  std::ofstream(p, std::ios::trunc) << text;
}

SubcommandRenderOptions watch_options(const fs::path &plan,
                                      const fs::path &out) {
  SubcommandRenderOptions opt;
  // What setup_subcommand_render() would fill in, at a small size.
  opt.view = "plan";
  opt.layers = "cloud";
  opt.size = "320x240";
  opt.cloud_name = "cloud";
  opt.mesh_name = "mesh";
  opt.point_size = 2.0;
  opt.orbit_elevation_deg = 25.0;
  opt.section_thickness = 0.1;
  opt.mesh_style = "auto";
  opt.deviation_within_m = 0.02;
  opt.deviation_uncovered_m = 0.1;
  opt.plan = plan;
  opt.output = out;
  opt.watch = true;
  return opt;
}

} // namespace

TEST_CASE("PlanFile keeps the last good plan when an edit does not parse",
          "[rux_app][render][watch]") {
  TempDir dir("plan_file");
  const fs::path plan = dir.path / "gt.yaml";
  write(plan, plan_yaml(0.0));
  rux::PlanFile file(plan);
  CHECK_FALSE(file.model());
  REQUIRE(file.reload());
  CHECK(file.generation() == 1);
  REQUIRE(file.model()->walls.size() == 1);

  // A zero-length axis: the strict reader names the element.
  write(plan, "model_version: 1\nwalls:\n  - guid: south\n    axis: [[1, 1], "
              "[1, 1]]\n");
  CHECK_FALSE(file.reload());
  CHECK_THAT(file.error(), ContainsSubstring("walls[0]"));
  CHECK_THAT(file.error(), ContainsSubstring("south"));
  REQUIRE(file.model()); // still the last good one
  CHECK(file.model()->walls[0].y0 == 0.0);
  CHECK(file.generation() == 1);

  write(plan, plan_yaml(0.5));
  REQUIRE(file.reload());
  CHECK(file.error().empty());
  CHECK(file.model()->walls[0].y0 == 0.5);
  CHECK(file.generation() == 2);
}

TEST_CASE("RenderWatchSession re-renders and re-scores on each good save",
          "[rux_app][render][watch]") {
  TempDir dir("render_watch");
  const fs::path project = dir.path / "p.rux";
  const fs::path plan = dir.path / "gt.yaml";
  const fs::path out = dir.path / "out" / "plan.png";
  make_project(project);
  write(plan, plan_yaml(0.0));

  RuxOptions global;
  global.project_db = project;
  rux::RenderWatchSession session(watch_options(plan, out), global);
  CHECK(session.watched_paths() == std::vector<fs::path>{plan});

  // Pass 0: the wall sits on the face -- fully supported.
  const auto p0 = session.pass(true, true);
  CHECK(p0.index == 0);
  CHECK(p0.plan_ok);
  CHECK_THAT(p0.report, ContainsSubstring("ok: 0 storeys, 1 wall"));
  CHECK_THAT(p0.report, ContainsSubstring("wall support          mean 1.00"));
  CHECK_THAT(p0.report, ContainsSubstring("0 of 1 walls weak"));
  CHECK_THAT(p0.report, ContainsSubstring("wall recall"));
  CHECK_THAT(p0.report, ContainsSubstring("the project provides no walls"));
  const bool gl = p0.rendered;
  if (gl) {
    REQUIRE(p0.written.size() == 1);
    CHECK(fs::exists(out));
  } else {
    WARN("No offscreen GL here; render not checked: " << p0.render_error);
  }

  // An edit that does not parse: last good plan kept, nothing re-rendered.
  write(plan, "model_version: 1\nwalls:\n  - guid: south\n    axis: [[1, 1], "
              "[1, 1]]\n");
  const auto p1 = session.pass(true, false);
  CHECK(p1.index == 1);
  CHECK_FALSE(p1.plan_ok);
  CHECK_THAT(p1.plan_error, ContainsSubstring("walls[0] (guid 'south')"));
  CHECK_THAT(p1.report, ContainsSubstring("INVALID: "));
  CHECK_THAT(p1.report, ContainsSubstring(
                            "walls[0] (guid 'south'): axis has zero length"));
  CHECK_THAT(p1.report,
             ContainsSubstring("keeping the last good plan (pass 0)"));
  CHECK_FALSE(p1.rendered);
  CHECK_THAT(p1.report, ContainsSubstring("images                unchanged"));

  // The wall moved 12 cm off its face: support drops, the offset shows.
  write(plan, plan_yaml(0.12));
  const auto p2 = session.pass(true, false);
  CHECK(p2.plan_ok);
  CHECK(p2.rendered == gl);
  CHECK_THAT(p2.report, ContainsSubstring("wall support          mean 0.00"));
  CHECK_THAT(p2.report, ContainsSubstring("1 of 1 walls weak"));
  CHECK_THAT(p2.report, ContainsSubstring("weak wall south"));
  CHECK_THAT(p2.report, ContainsSubstring("offset -120 mm"));

  // Deterministic: the same inputs print the same block (but its header).
  const auto p3 = session.pass(true, true);
  const auto body = [](const std::string &s) { return s.substr(s.find('\n')); };
  CHECK(body(p3.report) == body(p2.report));
}

TEST_CASE("RenderWatchSession watches the project too with --watch-project",
          "[rux_app][render][watch]") {
  TempDir dir("render_watch");
  const fs::path project = dir.path / "p.rux";
  const fs::path plan = dir.path / "gt.yaml";
  auto opt = watch_options(plan, dir.path / "x.png");
  opt.watch_project = true;
  RuxOptions global;
  global.project_db = project;
  rux::RenderWatchSession session(opt, global);
  const auto paths = session.watched_paths();
  REQUIRE(paths.size() == 3);
  CHECK(paths[1] == project);
  CHECK(paths[2] == fs::path(project.string() + "-wal"));
}

TEST_CASE("rux render --watch --watch-cycles 0 renders once and returns",
          "[rux_app][render][watch][cli]") {
  TempDir dir("render_watch_cli");
  const fs::path project = dir.path / "p.rux";
  const fs::path plan = dir.path / "gt.yaml";
  make_project(project);
  write(plan, plan_yaml(0.0));

  CLI::App app{"rux"};
  auto global = std::make_shared<RuxOptions>();
  app.add_option("-p,--project", global->project_db);
  setup_subcommand_render(app, global);
  std::vector<std::string> args{"-p",
                                project.string(),
                                "render",
                                "--plan",
                                plan.string(),
                                "--watch",
                                "--watch-cycles",
                                "0",
                                "--size",
                                "160x120",
                                "-o",
                                (dir.path / "o.png").string()};
  std::vector<std::string> reversed(args.rbegin(), args.rend());
  // Runs the real callback: one pass, then returns (no Ctrl-C needed).
  REQUIRE_NOTHROW(app.parse(reversed));

  // --watch-project without --watch is refused.
  CLI::App app2{"rux"};
  auto global2 = std::make_shared<RuxOptions>();
  setup_subcommand_render(app2, global2);
  app2.get_subcommand("render")->callback([] {});
  std::vector<std::string> bad{"--watch-project", "render"};
  CHECK_THROWS_AS(app2.parse(bad), CLI::ParseError);
}
