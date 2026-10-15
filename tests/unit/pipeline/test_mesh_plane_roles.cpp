// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// pipeline::mesh_plane_roles (reusex-z1v.4.3): the plane_classes rows of the
// "planes" cloud -> MeshOptions::plane_roles, index-aligned with the planes
// io::getPlanes() returns, and empty whenever the rows do not describe those
// planes.

#include <catch2/catch_test_macros.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/guid.hpp>
#include <reusex/core/solid_model.hpp>
#include <reusex/pipeline/mesh_parameters.hpp>

#include "../../support/temp_path.hpp"

#include <memory>
#include <string>
#include <vector>

using reusex::IndicesPtr;
using reusex::ProjectDB;
using reusex::core::PlaneClassRecord;
using reusex::core::PlaneRole;
using reusex::pipeline::mesh_plane_roles;
using reusex::test_support::TempPath;

namespace {

/// Three planes with 2, 1 and 3 points (labels 1, 2, 3).
void seed_planes_cloud(ProjectDB &db) {
  reusex::CloudL labels;
  for (uint32_t l : {1u, 1u, 2u, 3u, 3u, 3u}) {
    reusex::LabelT p;
    p.label = l;
    labels.push_back(p);
  }
  db.save_point_cloud("planes", labels, "segment_planes", "{}");
}

std::vector<IndicesPtr> inliers_of(std::vector<int> counts) {
  std::vector<IndicesPtr> out;
  int next = 0;
  for (int c : counts) {
    auto idx = std::make_shared<reusex::Indices>();
    for (int i = 0; i < c; ++i)
      idx->push_back(next++);
    out.push_back(idx);
  }
  return out;
}

PlaneClassRecord row(const ProjectDB &db, int label, const char *role,
                     int points) {
  PlaneClassRecord r;
  r.plane_label = label;
  r.guid = reusex::core::deterministic_guid(
      db.project_guid(), "plane_classes/planes/" + std::to_string(label));
  r.orientation = "horizontal";
  r.role = role;
  r.role_source = "rules";
  r.point_count = points;
  return r;
}

} // namespace

TEST_CASE("MeshPlaneRoles_NoClassification_IsEmpty",
          "[pipeline][mesh][plane_roles]") {
  TempPath tmp("test_mesh_plane_roles_none");
  ProjectDB db(tmp.path);
  seed_planes_cloud(db);
  CHECK(mesh_plane_roles(db, inliers_of({2, 1, 3})).empty());
}

TEST_CASE("MeshPlaneRoles_Rows_AreIndexedByPlaneLabel",
          "[pipeline][mesh][plane_roles]") {
  TempPath tmp("test_mesh_plane_roles_rows");
  ProjectDB db(tmp.path);
  seed_planes_cloud(db);
  // Plane 2 has no row: it stays (unknown).
  db.save_plane_classes("planes",
                        {row(db, 1, "floor", 2), row(db, 3, "furniture", 3)});
  const auto roles = mesh_plane_roles(db, inliers_of({2, 1, 3}));
  REQUIRE(roles.size() == 3);
  CHECK(roles[0] == PlaneRole::floor);
  CHECK(roles[1] == PlaneRole::unknown);
  CHECK(roles[2] == PlaneRole::furniture);
}

TEST_CASE("MeshPlaneRoles_StaleClassification_IsIgnored",
          "[pipeline][mesh][plane_roles]") {
  TempPath tmp("test_mesh_plane_roles_stale");
  ProjectDB db(tmp.path);
  seed_planes_cloud(db);
  db.save_plane_classes("planes",
                        {row(db, 1, "floor", 2), row(db, 3, "furniture", 3)});
  // `create planes` ran again: plane 3 now has 4 points.
  CHECK(mesh_plane_roles(db, inliers_of({2, 1, 4})).empty());
  // ... or there are fewer planes than the rows name.
  CHECK(mesh_plane_roles(db, inliers_of({2, 1})).empty());
}
