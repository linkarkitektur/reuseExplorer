// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// `rux analyze drift` parsed and run through the real CLI11 setup
// (reusex-rcv.1.1): the JSON report reaches -o with every field.

#include <catch2/catch_test_macros.hpp>

#include <analyze.hpp>
#include <global-params.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/SensorIntrinsics.hpp>

#include "../../support/source_dir.hpp"
#include "../../support/temp_path.hpp"

#include <CLI/CLI.hpp>
#include <Eigen/Core>
#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>

#include <array>
#include <cmath>
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

int rux_exit(const std::filesystem::path &project,
             std::vector<std::string> args) {
  CLI::App app{"rux"};
  auto global = std::make_shared<RuxOptions>();
  app.add_option("-p,--project", global->project_db);
  setup_subcommand_analyze(app, global);
  args.insert(args.begin(), {"-p", project.string(), "analyze", "drift"});
  std::vector<std::string> reversed(args.rbegin(), args.rend());
  try {
    app.parse(reversed);
  } catch (const CLI::Error &e) {
    return e.get_exit_code() == 0 ? -1 : e.get_exit_code();
  }
  return 0;
}

/// Two passes (t = 0.. and t = 100..) of a camera at the centre of a
/// 4 x 4 x 2.6 m box, panning a full turn each: depth ray-cast, no noise.
void make_project(const std::filesystem::path &path) {
  constexpr int w = 64, h = 48, n = 16;
  reusex::core::SensorIntrinsics k;
  k.fx = k.fy = w / 2.0;
  k.cx = w / 2.0;
  k.cy = h / 2.0;
  k.width = w;
  k.height = h;
  reusex::ProjectDB db(path, /*readOnly=*/false);
  const cv::Mat color(h, w, CV_8UC3, cv::Scalar(128, 128, 128));
  const cv::Mat conf(h, w, CV_8UC1, cv::Scalar(255));
  for (int f = 0; f < 2 * n; ++f) {
    const double yaw = 2.0 * M_PI * (f % n) / n;
    const Eigen::Vector3d z(std::cos(yaw), std::sin(yaw), 0);
    const Eigen::Vector3d x(std::sin(yaw), -std::cos(yaw), 0);
    const Eigen::Vector3d y(0, 0, -1);
    const Eigen::Vector3d o(0, 0, 1.4);
    cv::Mat depth(h, w, CV_16UC1, cv::Scalar(0));
    for (int v = 0; v < h; ++v)
      for (int u = 0; u < w; ++u) {
        const Eigen::Vector3d d =
            x * (u - k.cx) / k.fx + y * (v - k.cy) / k.fy + z;
        double t = 1e9;
        for (int a = 0; a < 3; ++a)
          for (double b : {a == 2 ? 0.0 : -2.0, a == 2 ? 2.6 : 2.0})
            if (std::abs(d[a]) > 1e-12 && (b - o[a]) / d[a] > 0)
              t = std::min(t, (b - o[a]) / d[a]);
        depth.at<std::uint16_t>(v, u) =
            static_cast<std::uint16_t>(std::lround(t * 1000.0));
      }
    std::array<double, 16> pose{x[0], y[0], z[0], o[0], x[1], y[1], z[1], o[1],
                                x[2], y[2], z[2], o[2], 0,    0,    0,    1};
    db.save_sensor_frame(f + 1, color, depth, conf, pose, k,
                         (f < n ? 0.0 : 100.0) + (f % n));
  }
}

} // namespace

TEST_CASE("AnalyzeDriftCli_WritesFullJsonReport", "[rux_app][drift]") {
  const TempPath project("drift_cli", ".rux");
  const TempPath out("drift_cli_out", ".json");
  make_project(project.path);

  REQUIRE(
      rux_exit(project.path, {"--segments", "2", "--depth-stride", "4",
                              "--plan", fixture("two_rooms.plan.json").string(),
                              "-o", out.path.string()}) == 0);
  std::ifstream in(out.path);
  REQUIRE(in);
  const auto j = nlohmann::json::parse(in);
  CHECK(j["frames_used"] == 32);
  CHECK(j["timestamps"] == true);
  CHECK(j["segments"].size() == 2);
  CHECK(j["pairs"].size() == 2);
  CHECK(j["suggestions"].size() == 2);
  CHECK(j["suggestions"][0]["delta"].size() == 16);
  CHECK(j["residual_median_m"].is_number());
  CHECK(j["noise_floor_m"].is_number());
  CHECK(j["crossings"].is_object());
  CHECK(j["warnings"].is_array());
}

TEST_CASE("AnalyzeDriftCli_MissingProject_Fails", "[rux_app][drift]") {
  const TempPath project("drift_cli_missing", ".rux");
  CHECK(rux_exit(project.path, {}) != 0);
}
