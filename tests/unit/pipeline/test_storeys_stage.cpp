// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// The `storeys` pipeline stage (reusex-z1v.3.3): cloud + planes -> the
// storeys table, through pipeline::run_stage (the path `rux create storeys`
// and the Qt Pipeline page share).

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/guid.hpp>
#include <reusex/core/stage_contract.hpp>
#include <reusex/core/storey_identity.hpp>
#include <reusex/pipeline/stage_parameters.hpp>
#include <reusex/pipeline/stages.hpp>
#include <reusex/segmentation/level.hpp>
#include <reusex/types/point_types.hpp>

#include "../../support/temp_path.hpp"

#include <nlohmann/json.hpp>

#include <sqlite3.h>

#include <string>
#include <vector>

using namespace reusex;
using namespace reusex::pipeline;
using Catch::Approx;
using reusex::test_support::TempPath;

namespace {

/// Two axis-aligned storeys (floor 0 / ceiling 2.7, floor 3.0 / ceiling 5.8)
/// with walls, written as the planes stage would.
void seed_two_storeys(ProjectDB &db, bool with_basement = false) {
  Cloud cloud;
  CloudL labels;
  CloudLoc centroids;
  CloudN normals;
  uint32_t label = 0;
  auto patch = [&](auto &&point_at, int nu, int nv, double nx, double ny,
                   double nz) {
    ++label;
    double sx = 0, sy = 0, sz = 0;
    for (int u = 0; u <= nu; ++u)
      for (int v = 0; v <= nv; ++v) {
        const auto [x, y, z] = point_at(u, v);
        PointT p;
        p.x = float(x);
        p.y = float(y);
        p.z = float(z);
        cloud.push_back(p);
        LabelT l;
        l.label = label;
        labels.push_back(l);
        sx += x;
        sy += y;
        sz += z;
      }
    const double n = double((nu + 1) * (nv + 1));
    LocT c;
    c.x = float(sx / n);
    c.y = float(sy / n);
    c.z = float(sz / n);
    centroids.push_back(c);
    NormalT nn;
    nn.normal_x = float(nx);
    nn.normal_y = float(ny);
    nn.normal_z = float(nz);
    normals.push_back(nn);
  };
  auto horizontal = [&](double z, double nz) {
    patch([z](int u,
              int v) { return std::array<double, 3>{0.2 * u, 0.2 * v, z}; },
          40, 30, 0, 0, nz);
  };
  auto wall = [&](double x, double z0, double z1) {
    patch(
        [x, z0](int u, int v) {
          return std::array<double, 3>{x, 0.2 * u, z0 + 0.2 * v};
        },
        30, int((z1 - z0) / 0.2), 1, 0, 0);
  };
  if (with_basement) { // floor -3.0, ceiling -0.3 (a 0.3 m slab below 0)
    horizontal(-3.0, 1);
    horizontal(-0.3, -1);
    wall(0, -2.9, -0.4);
    wall(8, -2.9, -0.4);
  }
  horizontal(0.0, 1);
  horizontal(2.7, -1);
  wall(0, 0.1, 2.6);
  wall(8, 0.1, 2.6);
  horizontal(3.0, 1);
  horizontal(5.8, -1);
  wall(0, 3.1, 5.7);
  wall(8, 3.1, 5.7);

  db.save_point_cloud("cloud", cloud, "cloud_reconstruction");
  db.save_point_cloud("planes", labels, "segment_planes");
  db.save_point_cloud("plane_centroids", centroids, "segment_planes");
  db.save_point_cloud("plane_normals", normals, "segment_planes");
}

/// storeys.run_id of every row, idx ascending (not in StoreyRecord).
std::vector<int> run_ids(const TempPath &project) {
  sqlite3 *raw = nullptr;
  REQUIRE(sqlite3_open(project.path.string().c_str(), &raw) == SQLITE_OK);
  sqlite3_stmt *st = nullptr;
  sqlite3_prepare_v2(raw, "SELECT run_id FROM storeys ORDER BY idx;", -1, &st,
                     nullptr);
  std::vector<int> ids;
  while (sqlite3_step(st) == SQLITE_ROW)
    ids.push_back(sqlite3_column_int(st, 0));
  sqlite3_finalize(st);
  sqlite3_close(raw);
  return ids;
}

StageResult run(ProjectDB &db, const TempPath &project,
                std::string parameters = {}) {
  StageContext ctx;
  ctx.project = project.path;
  ctx.stage = JobStage::storeys;
  ctx.parameters = std::move(parameters);
  return run_stage(db, ctx);
}

} // namespace

TEST_CASE("StoreysStage_Names_AndContract", "[pipeline][stages][storeys]") {
  CHECK(to_string(JobStage::storeys) == "storeys");
  CHECK(pipeline_log_name(JobStage::storeys) == "storeys");
  CHECK(parse_job_stage("storeys") == JobStage::storeys);
  const auto &c = core::stage_contract(core::PipelineStage::storeys);
  CHECK(c.command == "rux create storeys");
  REQUIRE(c.outputs.size() == 1);
  CHECK(c.outputs[0] == "storeys");
  CHECK(core::producing_stage("storeys") == core::PipelineStage::storeys);
}

TEST_CASE("StoreysStage_Parameters_AreLevelOptionsDefaults",
          "[pipeline][stages][storeys]") {
  const geometry::LevelOptions d{};
  const auto &params = stage_parameters(JobStage::storeys);
  auto value = [&](const char *key) -> ParameterDefault {
    for (const auto &p : params)
      if (p.key == key)
        return p.default_value;
    FAIL("no parameter " << key);
    return {};
  };
  CHECK(params.size() == 9);
  CHECK(std::get<double>(value("horizontal_cos")) == d.horizontal_cos);
  CHECK(std::get<double>(value("inlier_m")) == d.inlier_m);
  CHECK(std::get<long long>(value("max_fit_iterations")) ==
        d.max_fit_iterations);
  CHECK(std::get<double>(value("hist_bin_m")) == d.hist_bin_m);
  CHECK(std::get<double>(value("peak_min_share")) == d.peak_min_share);
  CHECK(std::get<double>(value("slab_merge_m")) == d.slab_merge_m);
  CHECK(std::get<double>(value("min_storey_height_m")) ==
        d.min_storey_height_m);
  CHECK(std::get<double>(value("default_height_m")) == d.default_height_m);
  CHECK(std::get<double>(value("guid_match_m")) ==
        core::StoreyIdentityOptions{}.match_m);
}

TEST_CASE("StoreysStage_EmptyProject_IsRefusedAsInvalidInput",
          "[pipeline][stages][storeys]") {
  TempPath project("test_storeys_stage_empty");
  ProjectDB db(project.path);
  const auto r = run(db, project);
  CHECK_FALSE(r.ok);
  CHECK(r.invalid_input);
  CHECK(r.message.find("planes") != std::string::npos);
  CHECK_FALSE(db.has_storeys());
}

TEST_CASE("StoreysStage_TwoStoreys_WritesRowsWithStableGuids",
          "[pipeline][stages][storeys]") {
  TempPath project("test_storeys_stage_two");
  ProjectDB db(project.path);
  seed_two_storeys(db);

  const auto r = run(db, project);
  INFO(r.message);
  REQUIRE(r.ok);
  REQUIRE(r.outputs.size() == 1);
  CHECK(r.outputs[0].kind == "table");
  CHECK(r.outputs[0].name == "storeys");
  CHECK(r.outputs[0].count == 2);

  const auto rows = db.storeys();
  REQUIRE(rows.size() == 2);
  CHECK(rows[0].z_floor == Approx(0.0).margin(1e-6));
  CHECK(rows[0].z_ceiling == Approx(2.7).margin(0.02));
  CHECK(rows[1].z_floor == Approx(3.0).margin(0.02));
  CHECK(rows[1].z_ceiling == Approx(5.8).margin(0.02));
  // First run: nothing stored to match, so both GUIDs are minted from the
  // floor height (core/storey_identity.hpp).
  CHECK(rows[0].guid ==
        core::deterministic_guid(db.project_guid(), "storeys/z0"));
  CHECK(rows[1].guid ==
        core::deterministic_guid(db.project_guid(), "storeys/z300"));
  for (const auto &s : rows) {
    CHECK(s.source == "detected");
    CHECK_FALSE(s.name.empty());
  }

  const auto log = db.pipeline_log(1);
  REQUIRE(log.size() == 1);
  CHECK(log[0].stage == "storeys");
  CHECK(log[0].status == "success");
  CHECK(run_ids(project) == std::vector<int>{log[0].id, log[0].id});

  // A re-run with the same input keeps every GUID.
  REQUIRE(run(db, project).ok);
  const auto again = db.storeys();
  REQUIRE(again.size() == 2);
  CHECK(again[0].guid == rows[0].guid);
  CHECK(again[1].guid == rows[1].guid);
}

TEST_CASE("StoreysStage_Parameters_ReachDetectStoreys",
          "[pipeline][stages][storeys]") {
  TempPath project("test_storeys_stage_params");
  ProjectDB db(project.path);
  seed_two_storeys(db);
  // A 6 m minimum storey height merges both storeys into one.
  const auto r = run(db, project, R"({"min_storey_height_m": 6.0})");
  INFO(r.message);
  REQUIRE(r.ok);
  CHECK(db.storeys().size() == 1);
  const auto log = db.pipeline_log(1);
  REQUIRE(log.size() == 1);
  CHECK(nlohmann::json::parse(log[0].parameters)["min_storey_height_m"] == 6.0);
}

TEST_CASE("StoreysStage_NewLowerStorey_KeepsTheOldStoreysGuids",
          "[pipeline][stages][storeys]") {
  TempPath project("test_storeys_stage_basement");
  std::vector<core::StoreyRecord> before;
  {
    ProjectDB db(project.path);
    seed_two_storeys(db);
    REQUIRE(run(db, project).ok);
    before = db.storeys();
    REQUIRE(before.size() == 2);
    // Re-scan finds a basement: every storey moves up one idx and the level
    // frame's origin moves 3 m down.
    seed_two_storeys(db, /*with_basement=*/true);
    const auto r = run(db, project);
    INFO(r.message);
    REQUIRE(r.ok);
  }
  ProjectDB db(project.path);
  const auto after = db.storeys();
  REQUIRE(after.size() == 3);
  CHECK(after[1].guid == before[0].guid);
  CHECK(after[2].guid == before[1].guid);
  CHECK(after[0].guid != before[0].guid);
  CHECK(after[0].guid != before[1].guid);
  CHECK(after[1].z_floor == Approx(3.0).margin(0.02));
}
