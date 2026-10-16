// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// `rooms` with `method = band` (reusex-rqn.2) through pipeline::run_stage,
// the path `rux create rooms --method band` and the Qt Pipeline page share.

#include <catch2/catch_test_macros.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/pipeline/stages.hpp>
#include <reusex/types/point_types.hpp>

#include "../../support/temp_path.hpp"

#include <set>
#include <string>

using namespace reusex;
using namespace reusex::pipeline;
using reusex::test_support::TempPath;

namespace {

/// Two 4 x 4 m rooms joined by a 0.9 m door, one storey (floor 0, ceiling
/// 2.6), as clouds + planes + storeys leave it. One dummy plane is enough
/// for the rooms contract; the band method does not read planes.
void seed_two_rooms(ProjectDB &db, bool with_storeys = true) {
  Cloud cloud;
  CloudN normals;
  CloudL planes;
  auto add = [&](double x, double y, double z, float nx, float ny, float nz) {
    PointT p;
    p.x = float(x);
    p.y = float(y);
    p.z = float(z);
    cloud.push_back(p);
    NormalT n;
    n.normal_x = nx;
    n.normal_y = ny;
    n.normal_z = nz;
    normals.push_back(n);
    LabelT l;
    l.label = 1;
    planes.push_back(l);
  };
  const double s = 0.05;
  for (double x = 0; x <= 8 + 1e-9; x += s)
    for (double y = 0; y <= 4 + 1e-9; y += s) {
      add(x, y, 0.0, 0, 0, 1);
      add(x, y, 2.6, 0, 0, -1);
    }
  auto wall_x = [&](double x, double y0, double y1) {
    for (double y = y0; y <= y1 + 1e-9; y += s)
      for (double z = 0; z <= 2.6 + 1e-9; z += s)
        add(x, y, z, 1, 0, 0);
  };
  auto wall_y = [&](double y, double x0, double x1) {
    for (double x = x0; x <= x1 + 1e-9; x += s)
      for (double z = 0; z <= 2.6 + 1e-9; z += s)
        add(x, y, z, 0, 1, 0);
  };
  wall_x(0, 0, 4);
  wall_x(8, 0, 4);
  wall_y(0, 0, 8);
  wall_y(4, 0, 8);
  wall_x(4, 0, 1.5);
  wall_x(4, 2.4, 4);
  db.save_point_cloud("cloud", cloud);
  db.save_point_cloud("normals", normals);
  db.save_point_cloud("planes", planes);
  pcl::PointCloud<pcl::PointXYZ> centroids;
  centroids.push_back(pcl::PointXYZ(4, 2, 0));
  db.save_point_cloud("plane_centroids", centroids);
  CloudN plane_normals;
  plane_normals.push_back(normals[0]);
  db.save_point_cloud("plane_normals", plane_normals);
  if (with_storeys) {
    core::StoreyRecord st;
    st.guid = "storey-0";
    st.z_floor = 0.0;
    st.z_ceiling = 2.6;
    db.save_storeys({st});
  }
}

StageResult run(ProjectDB &db, const TempPath &project,
                const std::string &params) {
  StageContext ctx;
  ctx.project = project.path;
  ctx.stage = JobStage::rooms;
  ctx.parameters = params;
  return run_stage(db, ctx);
}

} // namespace

TEST_CASE("BandRoomsStage_TwoRooms_WritesTheRoomsCloud",
          "[pipeline][stages][band_rooms]") {
  TempPath project("test_band_rooms_stage_two");
  ProjectDB db(project.path);
  seed_two_rooms(db);
  const auto r = run(db, project, R"({"method": "band"})");
  INFO(r.message);
  REQUIRE(r.ok);
  auto rooms = db.point_cloud_label("rooms");
  REQUIRE(rooms->size() == db.point_cloud_xyzrgb("cloud")->size());
  std::set<uint32_t> ids;
  for (const auto &l : rooms->points)
    if (l.label > 0)
      ids.insert(l.label);
  CHECK(ids == std::set<uint32_t>{1, 2});
}

TEST_CASE("BandRoomsStage_NoStoreys_IsRefusedAsInvalidInput",
          "[pipeline][stages][band_rooms]") {
  TempPath project("test_band_rooms_stage_no_storeys");
  ProjectDB db(project.path);
  seed_two_rooms(db, /*with_storeys=*/false);
  const auto r = run(db, project, R"({"method": "band"})");
  CHECK_FALSE(r.ok);
  CHECK(r.invalid_input);
  CHECK(r.message.find("rux create storeys") != std::string::npos);
  CHECK_FALSE(db.has_point_cloud("rooms"));
}

TEST_CASE("BandRoomsStage_UnknownMethod_IsRefusedAsInvalidInput",
          "[pipeline][stages][band_rooms]") {
  TempPath project("test_band_rooms_stage_unknown");
  ProjectDB db(project.path);
  seed_two_rooms(db);
  const auto r = run(db, project, R"({"method": "voronoi"})");
  CHECK_FALSE(r.ok);
  CHECK(r.invalid_input);
  CHECK(r.message.find("voronoi") != std::string::npos);
}
