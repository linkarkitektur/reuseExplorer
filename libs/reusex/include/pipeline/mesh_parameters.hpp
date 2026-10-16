// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// The mesh stage's parameters -> MeshOptions, in ONE place.
//
// `rux create mesh` drives the solver itself (it also feeds the interactive
// viewer), while the GUI and the Qt client run the mesh stage through
// run_stage(). Both read their options here, from the same JSON keys
// (stage_parameters(JobStage::mesh)), so a CLI line and a GUI run with the
// same parameters solve the same problem.

#include <reusex/reconstruction/mesh.hpp>

#include <string>
#include <string_view>
#include <vector>

namespace reusex {
class ProjectDB;
}

namespace reusex::pipeline {

struct MeshStageOptions {
  geometry::MeshOptions options;
  std::string output_name = "mesh";
  /// Leave out the planes `rux create classify` called furniture or clutter
  /// (key "plane_classes"; reusex-z1v.4.3). Has no effect on a project
  /// without plane_classes rows for the "planes" cloud.
  ///
  /// Off by default: measured 2026-10-15 (reusex-vwa) it lowered the fit on
  /// house1 (the 12 clutter planes it drops include short hallway walls:
  /// mean deviation 21.4 -> 28.4 cm, uncovered 28.7% -> 37.1%) and on
  /// NewOffice it left the uncovered share unchanged (36.9% -> 37.3%) while
  /// adding a third section (5020 cells, over the 5000 default limit).
  bool use_plane_classes = false;
  /// Only storey floors and ceilings become section planes of the cell
  /// complex when the project has a storeys table (key "storey_sections";
  /// reusex-z1v.3.4): MeshOptions::section_heights from
  /// mesh_section_heights(). No effect without storeys.
  ///
  /// Off by default: measured 2026-10-16 (reusex-sba) it drops planes the
  /// storeys table does not describe but the mesh needs. On house1 it left
  /// out the split-level floor (mean deviation 12.2 -> 16.6 cm, unsupported
  /// mesh 24.6% -> 34.8% against storey-local walls alone); on house2 it cut
  /// the attic above the top storey's ceiling off the mesh (P95 deviation
  /// 86 -> 127 cm). It does make NewOffice fit max_cells (16659 -> 1851
  /// cells) by dropping horizontals at 6-10 m and over the ceiling.
  bool use_storey_sections = false;
};

/// Read the mesh stage's JSON parameters ("" = defaults). Unknown keys are
/// ignored.
/// @throws std::invalid_argument for a non-object JSON or an unknown solver.
MeshStageOptions mesh_options_from_parameters(std::string_view parameters);

/// The plane_classes role of each plane of the "planes" cloud, index-aligned
/// with @p inliers as io::getPlanes() returns them (index = plane label - 1),
/// for MeshOptions::plane_roles. A plane without a row is `unknown` (kept).
///
/// Empty (every plane is used) when the project has no plane_classes rows,
/// or when the rows do not describe these planes: a row's plane label is out
/// of range or its point count differs from the plane's inlier count, which
/// means `create planes` ran again after `create classify`. That case is
/// logged at warn, naming the first mismatch.
std::vector<core::PlaneRole>
mesh_plane_roles(const ProjectDB &db, const std::vector<IndicesPtr> &inliers);

/// World z of every storey floor and ceiling in the storeys table, for
/// MeshOptions::section_heights (reusex-z1v.3.4): each storey's z_floor and
/// z_ceiling through its own level frame (LevelFrame::world_z_at). Empty
/// when the project has no storeys.
std::vector<double> mesh_section_heights(const ProjectDB &db);

/// The mesh stage's options for @p db: MeshOptions::plane_roles and
/// MeshOptions::section_heights filled from the project as @p stage asks.
/// `rux create mesh` and the pipeline's mesh stage both go through here.
geometry::MeshOptions mesh_options_for(const ProjectDB &db,
                                       const MeshStageOptions &stage,
                                       const std::vector<IndicesPtr> &inliers);

} // namespace reusex::pipeline
