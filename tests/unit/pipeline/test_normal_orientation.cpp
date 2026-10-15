// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// reusex-0se: the clouds stage records that its normals face the sensor
// ("normal_orientation": "sensor"), and the planes stage carries that over to
// plane_normals, so a later stage can tell them from legacy (origin-facing)
// normals.

#include <catch2/catch_test_macros.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/normal_orientation.hpp>
#include <reusex/pipeline/stages.hpp>

#include "../../support/pose_fixture.hpp"
#include "../../support/temp_path.hpp"

#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>

#include <optional>
#include <string>

using namespace reusex;
using namespace reusex::test_support;

namespace {

std::optional<std::string> orientation(const ProjectDB &db, const char *name) {
  return core::normal_orientation_of(db.point_cloud_provenance(name).second);
}

pipeline::StageResult run(ProjectDB &db, const TempPath &project,
                          pipeline::JobStage stage, std::string params) {
  pipeline::StageContext ctx;
  ctx.project = project.path;
  ctx.stage = stage;
  ctx.parameters = std::move(params);
  return pipeline::run_stage(db, ctx);
}

} // namespace

TEST_CASE("NormalOrientationOf_ParsesTheKeyAndToleratesJunk",
          "[core][normal_orientation]") {
  CHECK(core::normal_orientation_of(R"({"normal_orientation":"sensor"})") ==
        std::optional<std::string>("sensor"));
  CHECK_FALSE(core::normal_orientation_of(R"({"resolution":0.05})"));
  CHECK_FALSE(core::normal_orientation_of(""));
  CHECK_FALSE(core::normal_orientation_of("not json"));
  CHECK_FALSE(core::normal_orientation_of("[1,2]"));
  CHECK_FALSE(core::normal_orientation_of(R"({"normal_orientation":1})"));
}

TEST_CASE("CloudsAndPlanesStages_StampSensorNormalOrientation",
          "[pipeline][stages][normal_orientation]") {
  TempPath project("test_normal_orientation_stamp");
  ProjectDB db(project.path);
  constexpr int kW = 64, kH = 48;
  db.save_sensor_frame(
      1, make_color(kW, kH), make_depth(kW, kH, 1.5), cv::Mat(),
      translation_pose(0.0, 0.0, 0.0),
      make_intrinsics(64.0, 64.0, kW / 2.0, kH / 2.0, kW, kH, identity16()));

  const auto clouds = run(db, project, pipeline::JobStage::clouds,
                          R"({"resolution":0.05,"min_distance":0.0,)"
                          R"("max_distance":4.0,"sampling_factor":1,)"
                          R"("confidence_threshold":2})");
  INFO(clouds.message);
  REQUIRE(clouds.ok);
  CHECK(orientation(db, "cloud") == std::optional<std::string>("sensor"));
  CHECK(orientation(db, "normals") == std::optional<std::string>("sensor"));

  const auto planes =
      run(db, project, pipeline::JobStage::planes, R"({"min_inliers":20})");
  INFO(planes.message);
  REQUIRE(planes.ok);
  CHECK(orientation(db, "plane_normals") ==
        std::optional<std::string>("sensor"));
  // The stage's own parameters are still there next to the marker.
  const auto params =
      nlohmann::json::parse(db.point_cloud_provenance("plane_normals").second);
  CHECK(params.value("min_inliers", 0) == 20);
}

TEST_CASE("PlanesStage_LegacyNormals_PlaneNormalsCarryNoMarker",
          "[pipeline][stages][normal_orientation]") {
  TempPath project("test_normal_orientation_legacy");
  ProjectDB db(project.path);
  // A cloud as a pre-0se `rux create clouds` left it: no marker.
  Cloud cloud;
  CloudN normals;
  for (int u = 0; u < 30; ++u)
    for (int v = 0; v < 30; ++v) {
      PointT p;
      p.x = 0.05f * float(u);
      p.y = 0.05f * float(v);
      p.z = 0.0f;
      cloud.push_back(p);
      NormalT n;
      n.normal_z = -1.0f; // faces the origin from above: legacy
      normals.push_back(n);
    }
  db.save_point_cloud("cloud", cloud, "reconstruct", R"({"resolution":0.05})");
  db.save_point_cloud("normals", normals, "reconstruct",
                      R"({"resolution":0.05})");

  const auto planes =
      run(db, project, pipeline::JobStage::planes, R"({"min_inliers":20})");
  INFO(planes.message);
  REQUIRE(planes.ok);
  CHECK_FALSE(orientation(db, "plane_normals"));
}
