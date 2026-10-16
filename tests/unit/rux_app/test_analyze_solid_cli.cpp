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
#include <reusex/io/solid_model_json.hpp>

#include "../../support/temp_path.hpp"

#include <CLI/CLI.hpp>
#include <nlohmann/json.hpp>
#include <pcl/PolygonMesh.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

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

/// Parse and run `rux -p <project> analyze solid <args...>`; the exit code.
int rux_exit(const std::filesystem::path &project,
             std::vector<std::string> args) {
  CLI::App app{"rux"};
  auto global = std::make_shared<RuxOptions>();
  app.add_option("-p,--project", global->project_db);
  setup_subcommand_analyze(app, global);
  args.insert(args.begin(), {"-p", project.string(), "analyze", "solid"});
  std::vector<std::string> reversed(args.rbegin(), args.rend());
  try {
    app.parse(reversed);
  } catch (const CLI::Error &e) {
    return e.get_exit_code() == 0 ? -1 : e.get_exit_code();
  }
  return 0;
}

bool run_rux(const std::filesystem::path &project,
             std::vector<std::string> args) {
  return rux_exit(project, std::move(args)) == 0;
}

/// A project with a 2 x 2 m floor quad as mesh 'mesh' and two cloud points
/// 1 cm and 5 cm above it as cloud 'cloud'. @p degenerate makes every mesh
/// face collinear, so the fit cannot be computed.
void make_fit_project(const std::filesystem::path &path, bool degenerate) {
  pcl::PointCloud<pcl::PointXYZ> verts;
  verts.push_back({0, 0, 0});
  verts.push_back({2, 0, 0});
  verts.push_back(degenerate ? pcl::PointXYZ{1, 0, 0} : pcl::PointXYZ{2, 2, 0});
  verts.push_back(degenerate ? pcl::PointXYZ{0.5, 0, 0}
                             : pcl::PointXYZ{0, 2, 0});
  pcl::PolygonMesh mesh;
  pcl::toPCLPointCloud2(verts, mesh.cloud);
  pcl::Vertices quad;
  quad.vertices = {0, 1, 2, 3};
  mesh.polygons.push_back(quad);

  reusex::Cloud cloud;
  pcl::PointXYZRGB p;
  p.x = 1.0f;
  p.y = 1.0f;
  p.z = 0.01f;
  cloud.push_back(p);
  p.z = 0.05f;
  cloud.push_back(p);

  reusex::ProjectDB db(path);
  db.save_point_cloud("cloud", cloud);
  db.save_mesh("mesh", mesh);
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
        "wall_precision", "opening_recall", "opening_precision",
        "opening_iou_mean", "fit_mean_m", "fit_within_share", "mesh_area_m2",
        "mesh_unsupported_share", "seen_share_mean", "assumed_share"}) {
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
  // d3 and d4 carry assumed thicknesses and are not scored: d1 2 cm, d2 3 cm,
  // d5 2 cm, d6 0.
  CHECK_THAT(j.at("wall_thickness_err_mean_m").get<double>(),
             WithinAbs(0.07 / 4.0, 1e-9));
  CHECK(j.at("counts").at("thickness_assumed_excluded") == 2);
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

TEST_CASE("AnalyzeSolidCli_YamlReferenceAndModel_SameReportAsJson",
          "[rux_app][cli][analyze]") {
  // reusex-yf4.1: --reference / --model pick JSON or YAML by extension.
  TempPath ref_yaml("test_analyze_solid_ref", ".yaml");
  reusex::io::solid_model_to_file(
      reusex::io::solid_model_from_file(fixture("two_rooms.plan.json")),
      ref_yaml.path);
  TempPath model_yaml("test_analyze_solid_model", ".yml");
  reusex::io::solid_model_to_file(
      reusex::io::solid_model_from_file(fixture("two_rooms.detected.json")),
      model_yaml.path);

  TempPath out_json("test_analyze_solid_via_json");
  TempPath no_project("test_analyze_solid_yaml_absent");
  REQUIRE(run_rux(no_project.path,
                  {"--reference", fixture("two_rooms.plan.json").string(),
                   "--model", fixture("two_rooms.detected.json").string(), "-o",
                   out_json.path.string()}));
  TempPath out_yaml("test_analyze_solid_via_yaml");
  REQUIRE(run_rux(no_project.path,
                  {"--reference", ref_yaml.path.string(), "--model",
                   model_yaml.path.string(), "-o", out_yaml.path.string()}));
  CHECK(read_json(out_json.path) == read_json(out_yaml.path));
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

TEST_CASE("AnalyzeSolidCli_InvalidReference_ExitsWithInvalidArgument",
          "[rux_app][cli][analyze]") {
  TempPath project("test_analyze_solid_exit_project");
  {
    reusex::ProjectDB db(project.path);
  }
  TempPath bad("test_analyze_solid_exit_ref");
  {
    std::ofstream f(bad.path);
    f << R"({"model_version": 1, "units": "ft"})";
  }
  // RuxError::INVALID_ARGUMENT (-3) maps to exit code 3.
  CHECK(rux_exit(project.path, {"--reference", bad.path.string()}) == 3);
}

TEST_CASE("AnalyzeSolidCli_ProjectMeshAndCloud_ReportsFitWithItsSource",
          "[rux_app][cli][analyze]") {
  TempPath project("test_analyze_solid_fit_project");
  make_fit_project(project.path, /*degenerate=*/false);
  TempPath out("test_analyze_solid_fit_out");
  REQUIRE(run_rux(project.path,
                  {"--reference", fixture("two_rooms.plan.json").string(), "-o",
                   out.path.string()}));
  const auto j = read_json(out.path);
  CHECK_THAT(j.at("fit_mean_m").get<double>(), WithinAbs(0.03, 1e-6));
  CHECK_THAT(j.at("fit_within_share").get<double>(), WithinAbs(0.5, 1e-12));
  // Both scan points sit over the quad's centre: only a ~0.25 m disc around
  // them (pi * 0.25^2 of the 4 m^2) is supported at the default distance.
  CHECK_THAT(j.at("mesh_area_m2").get<double>(), WithinAbs(4.0, 1e-9));
  CHECK_THAT(j.at("mesh_unsupported_share").get<double>(),
             WithinAbs(1.0 - 0.196 / 4.0, 0.01));
  REQUIRE(j.at("fit_source").is_string());
  CHECK(j.at("fit_source").get<std::string>().find("mesh 'mesh'") !=
        std::string::npos);
}

TEST_CASE("AnalyzeSolidCli_SupportDist_WidensWhatCountsAsSupported",
          "[rux_app][cli][analyze]") {
  TempPath project("test_analyze_solid_support_project");
  make_fit_project(project.path, /*degenerate=*/false);
  TempPath out("test_analyze_solid_support_out");
  // Every point of the 2 x 2 quad is within 1.5 m of the scan points at its
  // centre (the corners are sqrt(2) m away).
  REQUIRE(run_rux(project.path,
                  {"--reference", fixture("two_rooms.plan.json").string(),
                   "--support-dist", "1.5", "-o", out.path.string()}));
  CHECK_THAT(read_json(out.path).at("mesh_unsupported_share").get<double>(),
             WithinAbs(0.0, 1e-12));
}

TEST_CASE("AnalyzeSolidCli_FitCannotBeComputed_ReportIsStillWritten",
          "[rux_app][cli][analyze]") {
  TempPath project("test_analyze_solid_fit_fail_project");
  make_fit_project(project.path, /*degenerate=*/true);
  TempPath out("test_analyze_solid_fit_fail_out");
  REQUIRE(run_rux(project.path,
                  {"--reference", fixture("two_rooms.plan.json").string(), "-o",
                   out.path.string()}));
  const auto j = read_json(out.path);
  CHECK(j.at("fit_mean_m").is_null());
  CHECK(j.at("fit_source").is_null());
}

TEST_CASE("AnalyzeSolidCli_ModelFile_FitsTheProjectMeshOnlyWhenAsked",
          "[rux_app][cli][analyze]") {
  // A baseline model file and the project's mesh are different things: the
  // fit must not appear next to the baseline's metrics unless asked for.
  TempPath project("test_analyze_solid_model_fit_project");
  make_fit_project(project.path, /*degenerate=*/false);
  const std::vector<std::string> base = {
      "--reference", fixture("two_rooms.plan.json").string(), "--model",
      fixture("two_rooms.detected.json").string()};

  TempPath out1("test_analyze_solid_model_fit_out1");
  auto args = base;
  args.insert(args.end(), {"-o", out1.path.string()});
  REQUIRE(run_rux(project.path, args));
  CHECK(read_json(out1.path).at("fit_mean_m").is_null());

  TempPath out2("test_analyze_solid_model_fit_out2");
  args = base;
  args.insert(args.end(), {"--mesh", "mesh", "-o", out2.path.string()});
  REQUIRE(run_rux(project.path, args));
  const auto j = read_json(out2.path);
  CHECK_FALSE(j.at("fit_mean_m").is_null());
  CHECK(j.at("fit_source").is_string());
}
