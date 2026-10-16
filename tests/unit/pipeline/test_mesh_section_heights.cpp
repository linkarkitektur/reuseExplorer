// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// pipeline::mesh_section_heights / mesh_options_for (reusex-z1v.3.4): the
// storeys table -> MeshOptions::section_heights, in world z through each
// storey's own level frame; nothing without storeys or when switched off.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/guid.hpp>
#include <reusex/core/solid_model.hpp>
#include <reusex/pipeline/mesh_parameters.hpp>

#include "../../support/temp_path.hpp"

#include <string>
#include <vector>

using Catch::Approx;
using reusex::ProjectDB;
using reusex::core::StoreyRecord;
using reusex::pipeline::mesh_options_for;
using reusex::pipeline::mesh_section_heights;
using reusex::pipeline::MeshStageOptions;
using reusex::test_support::TempPath;

namespace {

/// A storey whose level frame puts world z = level z + lift.
StoreyRecord storey(const ProjectDB &db, int idx, double z_floor,
                    double z_ceiling, double lift) {
  StoreyRecord s;
  s.idx = idx;
  s.guid = reusex::core::deterministic_guid(db.project_guid(),
                                            "storeys/" + std::to_string(idx));
  s.name = "Etage " + std::to_string(idx);
  s.z_floor = z_floor;
  s.z_ceiling = z_ceiling;
  s.level_transform = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, -lift, 0, 0, 0, 1};
  return s;
}

} // namespace

TEST_CASE("MeshSectionHeights_NoStoreys_IsEmpty",
          "[pipeline][mesh][storey_sections]") {
  TempPath tmp("test_mesh_section_heights_none");
  ProjectDB db(tmp.path);
  CHECK(mesh_section_heights(db).empty());
  MeshStageOptions stage;
  stage.use_storey_sections = true;
  CHECK(mesh_options_for(db, stage, {}).section_heights.empty());
}

TEST_CASE("MeshSectionHeights_Storeys_FloorsAndCeilingsInWorldZ",
          "[pipeline][mesh][storey_sections]") {
  TempPath tmp("test_mesh_section_heights_storeys");
  ProjectDB db(tmp.path);
  // Storey 1 in its own frame, as `create storeys` writes a drifted upper
  // floor: its floor is at level z 0, 2.8 m up in world z.
  db.save_storeys(
      {storey(db, 0, 0.0, 2.57, 1.5), storey(db, 1, 0.0, 2.13, 1.5 + 2.8)});
  const auto h = mesh_section_heights(db);
  REQUIRE(h.size() == 4);
  CHECK(h[0] == Approx(1.5));
  CHECK(h[1] == Approx(4.07));
  CHECK(h[2] == Approx(4.3));
  CHECK(h[3] == Approx(6.43));

  MeshStageOptions stage; // on by default (reusex-x8s)
  CHECK(mesh_options_for(db, stage, {}).section_heights == h);
  stage.use_storey_sections = false;
  CHECK(mesh_options_for(db, stage, {}).section_heights.empty());
}
