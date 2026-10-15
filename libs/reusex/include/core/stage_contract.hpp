// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// The single, machine-readable source of truth for what each pipeline stage
// consumes and produces in a `.rux` project (#246).
//
// This table used to exist three times: as a hand-written `switch` in
// `core::check_stage_inputs()`, as
// `rux::validation::validate_*_prerequisites()` in the CLI, and as prose in
// docs/CONTRACTS.md. The three could — and did — disagree, so `rux validate
// --stage mesh` could pass while `rux create mesh` refused. Everything now
// reads THIS table:
//
//   core::check_stage_inputs()   interprets it against a ProjectDB
//   pipeline::run_stage()        refuses through check_stage_inputs()
//   apps/rux                     gates every subcommand through it
//   docs/CONTRACTS.md            is a prose mirror, asserted by a unit test
//
// Layering (STANDARDS §1): the table is pure data — enums and names — with no
// PCL/CGAL/ProjectDB in it, so it sits at the lowest layer every consumer can
// reach. `pipeline` is Layer 4 and could not be read from `core`; `utils` would
// divorce it from the ProjectDB naming semantics it describes. Hence `core`.

#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace reusex::core {

/// Pipeline stages that have an input contract. Declared in pipeline order:
/// a stage's inputs may only be produced by a *strictly earlier* stage, which
/// is asserted by tests/unit/core/test_stage_contract.cpp.
enum class PipelineStage {
  import,         ///< extract raw sensor frames from a scan
  graph_init,     ///< snapshot initial poses + odometry edges (rux graph init)
  graph_discover, ///< find loop edges into the pose graph (rux graph discover)
  optimize,       ///< refine per-frame sensor poses (a.k.a. register)
  clouds,         ///< back-project sensor frames into "cloud" + "normals"
  annotate,       ///< ML semantic labelling of the stored frames
  project,        ///< project 2D semantic labels onto the 3D cloud
  planes,         ///< detect planar surfaces
  rooms,          ///< partition into rooms
  instances,      ///< separate semantic labels into spatial instances
  storeys,        ///< storeys and the measured vertical (level frame)
  classify,       ///< the role of each plane (wall, floor, ceiling, ...)
  mesh,           ///< generate the reconstructed mesh
  texture,        ///< texture-map the reconstructed mesh
  windows,        ///< derive window building components
  materials,      ///< create one material passport per instance + link it
  survey,         ///< fill the Ressourcekortlægning from the instances table
  attributes,     ///< VLM-derived per-material annotations (description + kv)
  // `gsplat` consumes `cloud` + `sensor_frames`, so anywhere after `clouds`
  // would satisfy the ordering invariant. It sits last because it is a LEAF:
  // it produces a `splat` nothing else consumes, so putting it mid-table would
  // read as if the reconstruction spine ran through it. Enumerator values are
  // not persisted anywhere; only the declaration order (the ordering
  // invariant above) matters.
  gsplat, ///< train a 3D Gaussian splat and store it in the project
};

/// What kind of thing a named artifact is inside a `.rux` project.
enum class ArtifactKind {
  point_cloud,    ///< a named `ProjectDB` point cloud
  mesh,           ///< a row in the `meshes` table
  gaussian_splat, ///< a row in the `gaussian_splats` table
  table,          ///< a relational table (`sensor_frames`, …)
};

/// Which index-alignment class an artifact belongs to.
///
/// This is a property of the ARTIFACT, not of the stage that consumes it, so
/// no stage can accidentally declare the per-plane `plane_centroids` to be
/// index-aligned with the per-point `cloud` (STANDARDS §3.2).
enum class Alignment {
  none,      ///< not index-aligned with anything (tables, meshes)
  per_point, ///< one entry per point of `cloud`
  per_plane, ///< one entry per detected plane
};

/// One named cloud / table / mesh the pipeline traffics in.
struct Artifact {
  std::string_view name;
  ArtifactKind kind = ArtifactKind::point_cloud;
  Alignment alignment = Alignment::none;
  std::string_view description;
};

/// One prerequisite of a stage.
struct StageInput {
  /// Artifact names, satisfied when ANY of them is present. More than one
  /// entry only for genuinely interchangeable inputs (the `instances` stage
  /// accepts `labels` or, failing that, the geometric `planes`).
  std::vector<std::string_view> any_of;
  /// Minimum row count, for `ArtifactKind::table` inputs only.
  int min_rows = 1;
  /// Minimum number of sensor frames that must carry depth. Only meaningful
  /// for the `sensor_frames` table; 0 disables the check. Pose refinement is
  /// depth-driven and silently produces nothing without it.
  int min_depth_frames = 0;
};

/// The full input/output contract of one pipeline stage.
struct StageContract {
  PipelineStage stage;
  /// Canonical lower-case name — the `rux validate --stage` token.
  std::string_view name;
  /// Additional accepted names (`register` for `optimize`).
  std::vector<std::string_view> aliases;
  /// The command that runs this stage, used to build resolution hints.
  std::string_view command;
  /// One-line description for help text.
  std::string_view summary;
  std::vector<StageInput> inputs;
  /// Artifacts this stage writes INTO the project. Used to derive resolution
  /// hints and to assert the pipeline forms a DAG.
  std::vector<std::string_view> outputs;
  /// Artifacts this stage writes OUTSIDE the project, described in prose
  /// (e.g. "a .ply at the path given by -o/--out"). Empty for every stage
  /// whose whole output lives in the `.rux`.
  ///
  /// Kept deliberately separate from `outputs` rather than modelled as another
  /// `ArtifactKind`, because nothing in the project can ever depend on it: a
  /// file on disk has no name `ProjectDB` can be asked about, so it is
  /// invisible to `producing_stage()`, to the DAG assertion, and to
  /// `check_stage_inputs()`. Encoding it as an artifact would let a future
  /// stage declare it as an input, and the checker would have no way to answer
  /// whether it is present. It is documentation with a machine-readable home,
  /// not a dependency edge — but it is what lets the "every stage produces
  /// something" invariant stay true for a stage with an empty `outputs`.
  std::string_view external_outputs = {};
};

/// Every stage contract, in pipeline order.
const std::vector<StageContract> &stage_contracts();

/// The contract of one stage. Throws std::logic_error for an enumerator with
/// no table row — a programming error, never a runtime condition.
const StageContract &stage_contract(PipelineStage stage);

/// Every artifact the contracts refer to.
const std::vector<Artifact> &pipeline_artifacts();

/// Look up an artifact by name, or nullptr when it is not a pipeline artifact.
const Artifact *find_artifact(std::string_view name);

/// The earliest stage that writes @p artifact, or nullopt when nothing does
/// (an externally sourced artifact). `optimize` rewrites `sensor_frames` in
/// place, so the earliest producer — `import` — is the one reported.
std::optional<PipelineStage> producing_stage(std::string_view artifact);

/// Parse a stage name (e.g. "mesh", "register") to a PipelineStage.
/// Returns std::nullopt for an unknown name.
std::optional<PipelineStage> parse_pipeline_stage(std::string_view name);

/// Canonical lower-case name of a stage (the primary CLI token).
std::string_view to_string(PipelineStage stage);

/// All stage names accepted by parse_pipeline_stage(), for help text.
std::vector<std::string> pipeline_stage_names();

} // namespace reusex::core
