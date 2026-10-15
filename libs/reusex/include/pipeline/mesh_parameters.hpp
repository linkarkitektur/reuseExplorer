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

} // namespace reusex::pipeline
