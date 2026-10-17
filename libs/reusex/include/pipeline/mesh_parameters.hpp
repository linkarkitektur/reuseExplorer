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
  /// Leave out the planes `rux create classify` called furniture, and the
  /// horizontal ones it called clutter (key "plane_classes";
  /// reusex-z1v.4.3); vertical clutter goes on to the structural filter
  /// (geometry::drop_planes_by_role(), reusex-rtx). Has no effect on a
  /// project without plane_classes rows for the "planes" cloud.
  ///
  /// On by default since reusex-rtx (2026-10-17, scripts/bench-solid.sh on
  /// house1 / house2 / NewOffice, mean fit / unsupported mesh share,
  /// off -> on): house1 12.9 -> 12.9 cm / 26.5% -> 25.4%, house2 26.0 ->
  /// 26.5 cm / 25.3% -> 25.4% (off itself varies 26.0..26.5 cm between
  /// runs), NewOffice 23.3 -> 22.5 cm / 8.6% -> 8.3%. Dropping vertical
  /// clutter as well (reusex-vwa) had cost house1 4.7 cm and house2 63 cm:
  /// the height rule calls hallway walls seen to door height clutter.
  bool use_plane_classes = true;
  /// Only the horizontal planes inside the building become section planes
  /// of the cell complex when the project has a storeys table (key
  /// "storey_sections"; reusex-z1v.3.4, reusex-x8s): MeshOptions::
  /// section_heights from mesh_section_heights(), see
  /// geometry::select_section_planes(). Storey floors and ceilings, split
  /// levels and an attic ceiling under the roof stay; planes outside the
  /// building's height and the invented slab faces under its lowest floor
  /// and over its top ceiling go. No effect without storeys.
  ///
  /// On by default since 2026-10-16 (reusex-x8s). Measured against off (same
  /// build): NewOffice fits max_cells (16659 > 5000 -> 1851 cells; its
  /// horizontals at -2.5 and 6-10 m are tilted planes force_orthogonal_planes
  /// moved out of the building); house2 4699 -> 4136 cells, mean deviation
  /// 28.8 -> 26.2 cm, unsupported 26.3% -> 26.2%, and the attic fits the
  /// cell budget for every wall (reusex-e6z); house1 959 -> 521 cells, mean
  /// 12.4 -> 12.2 cm, unsupported 24.7% -> 24.6%, split level kept. The
  /// first rule (only planes near a storey height, reusex-sba) dropped
  /// house1's split level and house2's attic and stayed off.
  bool use_storey_sections = true;
  /// The stored spaces as a room prior for the cells inside their
  /// footprints (key "space_prior"; reusex-6t4): MeshOptions::spaces from
  /// mesh_space_footprints(), see geometry::apply_space_prior(). No effect
  /// without a spaces table.
  ///
  /// On by default since reusex-6t4 (2026-10-17, scripts/bench-solid.sh,
  /// off -> on at weight 0.3): house2 fit 21.2 -> 20.4 cm, unsupported mesh
  /// 21.8% -> 21.4%; house1 identical; NewOffice fit 19.2 -> 19.2 cm,
  /// unsupported 11.0% -> 10.9%.
  bool use_space_prior = true;
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

/// The walls table in world coordinates, for MeshOptions::walls
/// (reusex-72a): each wall's axis and base through its storey's level frame
/// (LevelFrame::to_world), its top at base + height. A wall whose storey is
/// not in the storeys table is left out (logged at warn). Empty when the
/// project has no walls.
std::vector<geometry::WallEvidence> mesh_wall_evidence(const ProjectDB &db);

/// The spaces table as footprints in their storeys' level frames, for
/// MeshOptions::spaces (reusex-6t4): each space's polygon, its room_label
/// and its storey's frame, floor and ceiling. A space without a room label
/// or whose storey is not in the storeys table is left out (logged at
/// warn). Empty when the project has no spaces.
std::vector<geometry::SpaceFootprint>
mesh_space_footprints(const ProjectDB &db);

/// The mesh stage's options for @p db: MeshOptions::plane_roles,
/// MeshOptions::section_heights and (with the section heights, which it is
/// read against) MeshOptions::walls filled from the project as @p stage
/// asks.
/// `rux create mesh` and the pipeline's mesh stage both go through here.
geometry::MeshOptions mesh_options_for(const ProjectDB &db,
                                       const MeshStageOptions &stage,
                                       const std::vector<IndicesPtr> &inliers);

} // namespace reusex::pipeline
