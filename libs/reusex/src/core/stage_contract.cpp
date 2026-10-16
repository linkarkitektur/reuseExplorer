// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "reusex/core/stage_contract.hpp"

#include <algorithm>
#include <stdexcept>

namespace reusex::core {
namespace {

// ── The artifact registry ──────────────────────────────────────────────────
// Every named cloud / table / mesh a stage may consume or produce, with the
// index-alignment class it belongs to. Stages below refer to these by name;
// a name not listed here is rejected by the table-integrity unit test.
const std::vector<Artifact> &artifacts() {
  static const std::vector<Artifact> table = {
      {"sensor_frames", ArtifactKind::table, Alignment::none,
       "per-frame color/depth/confidence/pose/intrinsics rows"},
      {"segmentation_images", ArtifactKind::table, Alignment::none,
       "per-frame 2D semantic label images"},
      {"pose_graph_edges", ArtifactKind::table, Alignment::none,
       "the stored pose graph: odometry and loop edges with measurements, "
       "status and solve metrics (schema v28)"},
      {"cloud", ArtifactKind::point_cloud, Alignment::per_point,
       "the fused scan cloud (PointXYZRGB)"},
      {"normals", ArtifactKind::point_cloud, Alignment::per_point,
       "per-point surface normals (Normal)"},
      {"labels", ArtifactKind::point_cloud, Alignment::per_point,
       "per-point semantic class labels (Label)"},
      {"planes", ArtifactKind::point_cloud, Alignment::per_point,
       "per-point plane labels (Label)"},
      {"plane_centroids", ArtifactKind::point_cloud, Alignment::per_plane,
       "one centroid per detected plane (PointXYZ)"},
      {"plane_normals", ArtifactKind::point_cloud, Alignment::per_plane,
       "one normal per detected plane (Normal)"},
      {"rooms", ArtifactKind::point_cloud, Alignment::per_point,
       "per-point room labels (Label)"},
      {"instances", ArtifactKind::point_cloud, Alignment::per_point,
       "per-point instance labels (Label), plus the `instances` table"},
      {"storeys", ArtifactKind::table, Alignment::none,
       "storeys: floor/ceiling heights in the level frame and the "
       "world->level transform (schema v30)"},
      {"plane_classes", ArtifactKind::table, Alignment::none,
       "plane_classes: orientation, majority class, role, storey and extent "
       "of each plane of `planes` (schema v31)"},
      {"walls", ArtifactKind::table, Alignment::none,
       "walls: axis, height, measured thickness with its CI, lean and "
       "evidence of each wall, in its storey's level frame (schema v33)"},
      {"wall_joins", ArtifactKind::table, Alignment::none,
       "wall_joins: corner / tee / collinear joins between walls "
       "(schema v33)"},
      {"mesh", ArtifactKind::mesh, Alignment::none,
       "the reconstructed watertight mesh"},
      {"textured_mesh", ArtifactKind::mesh, Alignment::none,
       "the texture-mapped mesh"},
      {"building_components", ArtifactKind::table, Alignment::none,
       "derived building components (windows, …)"},
      {"instance_materials", ArtifactKind::table, Alignment::none,
       "instance→material-passport links + the material passports"},
      {"survey_types", ArtifactKind::table, Alignment::none,
       "Ressourcekortlægning survey types (one per semantic class)"},
      {"survey_parts", ArtifactKind::table, Alignment::none,
       "Ressourcekortlægning survey parts (one bygningsdel per instance)"},
      {"material_annotations", ArtifactKind::table, Alignment::none,
       "per-material VLM annotations (description + key/value attributes)"},
      {"splat", ArtifactKind::gaussian_splat, Alignment::none,
       "a trained 3D Gaussian Splatting model"},
  };
  return table;
}

// ── The stage table ────────────────────────────────────────────────────────
// Order is pipeline order and is load-bearing: an input may only be produced
// by a strictly earlier stage (asserted in test_stage_contract.cpp), which is
// also what makes the resolution-hint recursion terminate.
const std::vector<StageContract> &contracts() {
  static const std::vector<StageContract> table = {
      {PipelineStage::import,
       "import",
       {},
       "rux import rtabmap <scan.db>",
       "extract raw sensor frames from an external scan",
       // Reads an external scan file, so it has no in-project prerequisite.
       {},
       // RTABMap links become pose_graph_edges (rtabmap only).
       {"sensor_frames", "pose_graph_edges"}},

      {PipelineStage::graph_init,
       "graph_init",
       {},
       "rux graph init",
       "snapshot the initial poses and freeze odometry edges between "
       "consecutive frames",
       {{{"sensor_frames"}, /*min_rows=*/2}},
       {"pose_graph_edges"}},

      {PipelineStage::graph_discover,
       "graph_discover",
       {},
       "rux graph discover",
       "find wide-baseline loop edges and store them in the pose graph",
       {{{"sensor_frames"}, /*min_rows=*/2}},
       {"pose_graph_edges"}},

      {PipelineStage::optimize,
       "optimize",
       {"register"},
       "rux optimize",
       "refine the stored per-frame sensor poses",
       // Both pose refiners are depth-driven and need two frames to have any
       // relative pose to refine at all.
       {{{"sensor_frames"}, /*min_rows=*/2, /*min_depth_frames=*/2}},
       {"sensor_frames", "pose_graph_edges"}},

      {PipelineStage::clouds,
       "clouds",
       {},
       "rux create clouds",
       "back-project depth frames into a fused cloud",
       {{{"sensor_frames"}}},
       {"cloud", "normals"}},

      {PipelineStage::annotate,
       "annotate",
       {},
       "rux create annotate -n <model>",
       "ML semantic labelling of the stored frames",
       {{{"sensor_frames"}}},
       {"segmentation_images"}},

      {PipelineStage::project,
       "project",
       {},
       "rux create project",
       "project 2D semantic labels onto the 3D cloud",
       {{{"cloud"}}, {{"segmentation_images"}}},
       {"labels"}},

      {PipelineStage::planes,
       "planes",
       {},
       "rux create planes",
       "detect planar surfaces",
       {{{"cloud"}}, {{"normals"}}},
       {"planes", "plane_centroids", "plane_normals"}},

      {PipelineStage::storeys,
       "storeys",
       {},
       "rux create storeys",
       "detect storeys and the measured vertical (level frame)",
       // Horizontal planes give the floor/ceiling fits and the height
       // histogram; `cloud` + `planes` carry their inlier points.
       {{{"cloud"}}, {{"planes"}}, {{"plane_centroids"}}, {{"plane_normals"}}},
       {"storeys"}},

      {PipelineStage::rooms,
       "rooms",
       {},
       "rux create rooms",
       "partition the plane graph into rooms",
       // `normals` and `storeys` are inputs of `method = band` only (checked
       // by the stage, which says to run `rux create storeys`), so they are
       // not declared; `storeys` sits earlier in this table all the same.
       {{{"cloud"}}, {{"planes"}}, {{"plane_centroids"}}, {{"plane_normals"}}},
       {"rooms"}},

      {PipelineStage::instances,
       "instances",
       {},
       "rux create instances",
       "split semantic labels into spatial instances",
       // `labels` is the default semantic input; the geometric `planes` cloud
       // is accepted as a fallback so the check stays useful on a project that
       // was never annotated.
       {{{"cloud"}}, {{"labels", "planes"}}},
       {"instances"}},

      {PipelineStage::classify,
       "classify",
       {},
       "rux create classify",
       "classify each plane: wall, floor, ceiling, furniture, ...",
       // Plane inliers (`cloud` + `planes`) and normals give orientation,
       // area and heights; the storeys give the level frame and the floor /
       // ceiling bands. `labels` is an optional enrichment (the semantic
       // vote), so it is not declared, as for `survey`.
       {{{"cloud"}}, {{"planes"}}, {{"plane_normals"}}, {{"storeys"}}},
       {"plane_classes"}},

      {PipelineStage::walls,
       "walls",
       {},
       "rux create walls",
       "walls with a measured thickness and the joins between them",
       // Wall evidence = the inliers (`cloud` + `planes`) of the planes whose
       // role is wall, with their normals, in each storey's level frame.
       {{{"cloud"}},
        {{"planes"}},
        {{"plane_normals"}},
        {{"plane_classes"}},
        {{"storeys"}}},
       {"walls", "wall_joins"}},

      {PipelineStage::mesh,
       "mesh",
       {},
       "rux create mesh",
       "solidify the cell complex into a room-partitioned mesh",
       {{{"cloud"}},
        {{"normals"}},
        {{"rooms"}},
        {{"planes"}},
        {{"plane_centroids"}},
        {{"plane_normals"}}},
       {"mesh"}},

      {PipelineStage::texture,
       "texture",
       {},
       "rux create texture",
       "texture-map the reconstructed mesh",
       {{{"mesh"}}, {{"cloud"}}, {{"sensor_frames"}}},
       {"textured_mesh"}},

      {PipelineStage::windows,
       "windows",
       {},
       "rux create windows",
       "derive window building components",
       {{{"cloud"}}, {{"labels"}}, {{"instances"}}, {{"mesh"}}},
       {"building_components"}},

      {PipelineStage::materials,
       "materials",
       {},
       "rux create materials",
       "create one material passport per instance and link it",
       // Needs the instance-label cloud (one passport per instance) — the
       // `instances` table it links against is produced with that cloud.
       {{{"instances"}}},
       {"instance_materials"}},

      {PipelineStage::survey,
       "survey",
       {},
       "rux create survey",
       "fill the Ressourcekortlægning from the instances table",
       // One survey type per semantic class and one part per instance, so it
       // needs only the instance-label cloud — the same prerequisite as
       // `materials`. `rooms`/`labels` are optional enrichments (room
       // assignment / type naming) sync_survey degrades gracefully without,
       // so they are not declared as inputs here.
       {{{"instances"}}},
       {"survey_types", "survey_parts"}},

      {PipelineStage::attributes,
       "attributes",
       {},
       "rux create attributes",
       "describe each material with a vision-language model",
       // Iterates material passports, so it needs the material links
       // (`instance_materials`, produced by `create materials`); to crop a
       // material it also needs the fused cloud (the instances' 3D points), the
       // instance-label cloud (which points belong to which instance) and the
       // posed sensor frames (the 2D view). All are present after `materials`.
       {{{"cloud"}},
        {{"instances"}},
        {{"instance_materials"}},
        {{"sensor_frames"}}},
       {"material_annotations"}},

      // `reusex_gsplat` only exists under WITH_CUDA + the vendored rasterizer,
      // while `reusex_core` is built unconditionally — but that is not a
      // problem for this row, because the table is pure data (enums and
      // string_views; see the layering note in the header). Nothing here names
      // a gsplat type, and the interpreter only asks ProjectDB for a named
      // cloud and the `sensor_frames` row count. The entry therefore compiles
      // and validates identically in a CPU-only build: `rux validate --stage
      // gsplat` answers even where `rux create gsplat` is not compiled in,
      // which is the useful behaviour — you can check a project on the laptop
      // that captured it and train on the machine with the GPU.
      {PipelineStage::gsplat,
       "gsplat",
       {},
       "rux create gsplat",
       "train a 3D Gaussian splat from the fused cloud and posed frames",
       // Two frames is the structural floor, not a quality bar: a radiance
       // field fitted to a single view is just that view, so there is nothing
       // to optimize. It matches the `optimize` row's reasoning. The real
       // requirement — dense multi-view overlap over a contiguous capture
       // segment — is a judgement about the *content* of the frames that no
       // row count can express, so the subcommand's footer carries it instead.
       // No `min_depth_frames`: the geometry comes from the seed cloud and the
       // trainer only needs color + pose, so a depth-less frame is still a
       // usable training view.
       {{{"cloud"}}, {{"sensor_frames"}, /*min_rows=*/2}},
       // (360 panoramas are an *optional* extra source of views
       // (`--use-panoramas`), never a prerequisite, so they are deliberately
       // absent from the inputs above.)
       //
       // `splat` is the contract's declared name; `--name` overrides it, the
       // same way `--seed-cloud` overrides `cloud`.
       //
       // Since #322 the trained model goes INTO the project, so this row has a
       // real `outputs` and an EMPTY `external_outputs` — it was the one stage
       // with the opposite pair. `-o/--out` still writes a .ply and
       // `--render-dir` still writes PNGs, but those are exports and
       // diagnostics documented in `rux create gsplat --help`, not products
       // the project's state depends on. Declaring them here as well would
       // give a reader two places to look for the same answer, which is
       // exactly what `external_outputs` exists to avoid.
       {"splat"},
       ""},
  };
  return table;
}

} // namespace

const std::vector<StageContract> &stage_contracts() { return contracts(); }

const std::vector<Artifact> &pipeline_artifacts() { return artifacts(); }

const StageContract &stage_contract(PipelineStage stage) {
  for (const auto &entry : contracts())
    if (entry.stage == stage)
      return entry;
  // Unreachable for any enumerator. A new enumerator without a table row is a
  // programming error, so fail loudly rather than silently validating nothing.
  throw std::logic_error(
      "reusex::core: PipelineStage missing from the stage contract table");
}

const Artifact *find_artifact(std::string_view name) {
  for (const auto &entry : artifacts())
    if (entry.name == name)
      return &entry;
  return nullptr;
}

std::optional<PipelineStage> producing_stage(std::string_view artifact) {
  for (const auto &entry : contracts())
    if (std::find(entry.outputs.begin(), entry.outputs.end(), artifact) !=
        entry.outputs.end())
      return entry.stage;
  return std::nullopt;
}

std::optional<PipelineStage> parse_pipeline_stage(std::string_view name) {
  for (const auto &entry : contracts()) {
    if (entry.name == name)
      return entry.stage;
    if (std::find(entry.aliases.begin(), entry.aliases.end(), name) !=
        entry.aliases.end())
      return entry.stage;
  }
  return std::nullopt;
}

std::string_view to_string(PipelineStage stage) {
  return stage_contract(stage).name;
}

std::vector<std::string> pipeline_stage_names() {
  std::vector<std::string> names;
  for (const auto &entry : contracts()) {
    names.emplace_back(entry.name);
    for (const auto &alias : entry.aliases)
      names.emplace_back(alias);
  }
  return names;
}

} // namespace reusex::core
