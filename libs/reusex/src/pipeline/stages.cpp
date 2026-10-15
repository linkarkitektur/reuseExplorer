// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "reusex/pipeline/stages.hpp"
#include "reusex/core/label_encoding.hpp"
#include "reusex/pipeline/mesh_parameters.hpp"
#include "reusex/pipeline/stage_parameters.hpp"
#include "reusex/pipeline/tile_index.hpp"

#include "reusex/core/ProjectDB.hpp"
#include "reusex/core/filter_expression.hpp"
#include "reusex/core/guid.hpp"
#include "reusex/core/label_semantics.hpp"
#include "reusex/core/logging.hpp"
#include "reusex/core/validate.hpp"
#include "reusex/io/reusex.hpp"
#include "reusex/reconstruction/mesh.hpp"
#include "reusex/segmentation/reconcile_instances.hpp"
#include "reusex/segmentation/reconstruct.hpp"
#include "reusex/segmentation/segment_instances.hpp"
#include "reusex/segmentation/segment_planes.hpp"
#include "reusex/segmentation/segment_rooms.hpp"
#include "reusex/types/point_types.hpp"

#include <fmt/format.h>
#include <fmt/ranges.h>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <exception>
#include <map>
#include <set>
#include <stdexcept>

namespace reusex::pipeline {
namespace {

using json = nlohmann::json;

struct StageName {
  JobStage stage;
  std::string_view name;
  /// Value written to `pipeline_log.stage`. Matches what the CLI subcommands
  /// have always written, so the GUI and CLI paths do not fill one column with
  /// two different names for the same operation.
  std::string_view log_name;
  core::PipelineStage contract;
  bool cancellable;
};

// Single source of truth for stage identity. The `name` column is the token
// used in `ruxd`'s HTTP contract (docs/gui/openapi.yaml, in its own repo) and
// `contract` selects the input check run before the stage executes
// (docs/CONTRACTS.md).
constexpr std::array<StageName, 8> kStages{{
    {JobStage::clouds, "clouds", "cloud_reconstruction",
     core::PipelineStage::clouds, false},
    {JobStage::planes, "planes", "segment_planes", core::PipelineStage::planes,
     true},
    {JobStage::rooms, "rooms", "segment_rooms", core::PipelineStage::rooms,
     true},
    {JobStage::instances, "instances", "segment_instances",
     core::PipelineStage::instances, true},
    // Not cancellable: the HiGHS/cuOpt MIP solver has no cooperative cancel
    // hook; the time_limit_seconds parameter is the correct escape hatch for
    // large scenes. The job remains in `running` until the solver returns.
    {JobStage::mesh, "mesh", "mesh_generation", core::PipelineStage::mesh,
     false},
    // Not cancellable: the GTSAM solver has no cooperative cancel hook; the
    // job runs to completion. Matches `pose_optimization_plane_graph`, the log
    // name rux optimize has always written, for history-view join
    // compatibility.
    {JobStage::optimize, "optimize", "pose_optimization_plane_graph",
     core::PipelineStage::optimize, false},
    // ML semantic labelling. Cancellable: a batch run iterates frames, so a
    // cancel request can take effect between frames the way it does for
    // planes/rooms/instances. `run_stage()` validates the
    // core::PipelineStage::annotate contract (sensor_frames) before
    // dispatch(), exactly like every other stage; dispatch() itself always
    // refuses (see the comment there) because reusex_pipeline must not link
    // reusex_vision (STANDARDS §1). A caller that wants this stage to
    // actually run supplies its own executor, the way the GUI's custom
    // executor already does for optimize (#464).
    {JobStage::annotate, "annotate", "annotate", core::PipelineStage::annotate,
     true},
    // Not cancellable: ORB matching has no cooperative cancel hook. Like
    // optimize it needs reusex_slam, so dispatch() refuses it and the
    // header-only pipeline/optimize_stage.hpp executor runs it (#464).
    {JobStage::graph_discover, "graph_discover", "graph_discover",
     core::PipelineStage::graph_discover, false},
}};

const StageName &descriptor(JobStage stage) {
  for (const auto &entry : kStages)
    if (entry.stage == stage)
      return entry;
  // Unreachable for any enumerator; a new enumerator without a table row is a
  // programming error, so fail loudly rather than silently mis-dispatching.
  throw std::logic_error("reusex::pipeline: JobStage missing from stage table");
}

/// Parse the parameter blob. Empty string yields an empty object.
json parse_parameters(const std::string &parameters) {
  if (parameters.empty())
    return json::object();
  json parsed = json::parse(parameters, nullptr, /*allow_exceptions=*/true);
  if (!parsed.is_object())
    throw std::runtime_error("stage parameters must be a JSON object");
  return parsed;
}

template <typename T>
T param_or(const json &params, const char *key, T fallback) {
  auto it = params.find(key);
  if (it == params.end() || it->is_null())
    return fallback;
  return it->get<T>();
}

/// The parameter blob written to `pipeline_log`, with the driving job id
/// folded in under "job_id" when there is one.
std::string logged_parameters(const json &params, const std::string &job_id) {
  if (job_id.empty())
    return params.empty() ? std::string{} : params.dump();
  json annotated = params;
  annotated["job_id"] = job_id;
  return annotated.dump();
}

/// Run the stage's documented input contract and fail before doing any work
/// when a prerequisite is missing (STANDARDS §5: fail fast, loudly, with the
/// reason). Warnings are logged but do not block.
std::optional<std::string>
check_inputs(const ProjectDB &db, core::PipelineStage contract,
             const core::ArtifactOverrides &overrides) {
  std::vector<core::ValidationIssue> issues;
  core::check_stage_inputs(db, contract, issues, overrides);

  std::vector<std::string> errors;
  for (const auto &issue : issues) {
    if (issue.severity != core::ValidationSeverity::error) {
      warn("stage input warning [{}]: {}", issue.check, issue.message);
      continue;
    }
    errors.push_back(fmt::format("{}: {}", issue.check, issue.message));
    // The derived resolution hint (#246) goes to the log rather than into the
    // refusal message: an HTTP client gets a one-line reason, but whoever
    // reads the worker log still sees which commands would fix it.
    if (!issue.hint.empty())
      info("stage input resolution: {}", issue.hint);
  }
  if (errors.empty())
    return std::nullopt;
  return fmt::format("stage inputs not satisfied — {}",
                     fmt::join(errors, "; "));
}

/// Which contract artifact names this run has been pointed away from.
///
/// Not a second contract: the table still says the `instances` stage consumes
/// a semantic label cloud, this only says which cloud the caller's parameters
/// named. Without it, `semantic_cloud: "foo"` would be checked against
/// `labels` and refused (or, worse, accepted) for the wrong cloud.
core::ArtifactOverrides contract_overrides(JobStage stage, const json &params) {
  core::ArtifactOverrides overrides;
  if (stage == JobStage::instances) {
    const auto semantic = param_or<std::string>(params, "semantic_cloud", "");
    if (!semantic.empty())
      overrides["labels"] = semantic;
  }
  return overrides;
}

/// Resolve the optional `"filter"` parameter into the point indices a stage
/// should restrict itself to.
///
/// Mirrors the `filter` field of the segmentation option structs, so it is a
/// parameter like any other rather than a side channel. Returns a message when
/// the expression is unusable so the caller can refuse the run as invalid
/// input instead of letting it surface as a mid-run failure.
std::optional<std::string> resolve_filter(const json &params, ProjectDB &db,
                                          size_t cloud_size,
                                          IndicesConstPtr &out) {
  const auto expression = param_or<std::string>(params, "filter", "");
  if (expression.empty())
    return std::nullopt;
  try {
    out = core::evaluate_filter_expression(expression, db, cloud_size);
  } catch (const std::exception &e) {
    return fmt::format("filter evaluation failed: {}", e.what());
  }
  return std::nullopt;
}

// --- individual stage bodies ----------------------------------------------
// These are the ONLY implementations of these stages. `rux create <stage>` and
// the GUI/daemon job runners both reach them through run_stage(); the CLI's
// former copies were deleted in #284, so a fix applied here cannot miss a
// second copy. Anything a front end still owns (flag parsing, exit codes,
// prerequisite hints) stays in the front end.

StageResult run_clouds(ProjectDB &db, const StageContext &ctx,
                       const json &params) {
  geometry::ReconstructionParams p;
  p.resolution = param_or(params, "resolution", p.resolution);
  p.min_distance = param_or(params, "min_distance", p.min_distance);
  p.max_distance = param_or(params, "max_distance", p.max_distance);
  p.sampling_factor = param_or(params, "sampling_factor", p.sampling_factor);
  p.confidence_threshold =
      param_or(params, "confidence_threshold", p.confidence_threshold);
  p.glass_filter = param_or(params, "glass_filter", p.glass_filter);
  p.glass_threshold = param_or(params, "glass_threshold", p.glass_threshold);

  geometry::reconstruct_point_clouds(db, p);

  const auto cloud = db.point_cloud_xyzrgb("cloud");
  auto summary = fmt::format("reconstructed {} points at {:.3f} m resolution",
                             cloud ? cloud->size() : 0, p.resolution);

  // reconstruct_point_clouds() returns void and writes "cloud", "normals" and
  // (when the frames carried semantic labels) "labels" from one filtered index
  // set, each only when non-empty. The only count in hand is the one read back
  // above; "normals" is claimed on presence because reconstruct and the E57
  // importer are its only writers and it is filtered by the same indices as
  // "cloud", but its size is left at -1 rather than loading the whole cloud
  // back for a number nothing needs. "labels" is deliberately NOT claimed:
  // `rux create project` writes the same cloud name, so finding it afterwards
  // is no evidence that this run produced it.
  std::vector<StageArtifact> outputs;
  if (cloud)
    outputs.push_back({"cloud", "cloud", static_cast<int64_t>(cloud->size())});
  if (db.has_point_cloud("normals"))
    outputs.push_back({"cloud", "normals", -1});

  // The spatial tile index the web GUI streams the cloud by (#395). Built
  // here, not by a front end, so `rux create clouds` and a GUI-submitted job
  // leave the same project behind.
  if (const auto bytes = build_tile_index(db, "cloud"); bytes > 0)
    info("built spatial tile index for 'cloud' ({} bytes)", bytes);

  // A cancel that arrived mid-run could not stop this stage, and the output IS
  // written. Reporting "cancelled" would be a lie that makes the user think
  // the project is untouched, so report the success that actually happened and
  // say the cancel was too late.
  if (ctx.is_cancelled()) {
    warn("cancel requested, but the clouds stage cannot be interrupted; it ran "
         "to completion and its output was saved");
    summary += " (cancel requested too late; the stage cannot be interrupted, "
               "so it completed and saved its output)";
  }
  return StageResult::success(std::move(summary), std::move(outputs));
}

StageResult run_planes(ProjectDB &db, const StageContext &ctx,
                       const json &params) {
  geometry::SegmentPlanesOptions options;
  options.angle_threshold =
      param_or(params, "angle_threshold", options.angle_threshold);
  options.plane_dist_threshold =
      param_or(params, "plane_dist_threshold", options.plane_dist_threshold);
  options.min_inliers = param_or(params, "min_inliers", options.min_inliers);
  options.radius = param_or(params, "radius", options.radius);
  options.interval_0 = param_or(params, "interval_0", options.interval_0);
  options.interval_factor =
      param_or(params, "interval_factor", options.interval_factor);
  options.adaptive = param_or(params, "adaptive", options.adaptive);
  options.noise_seed = param_or(params, "noise_seed", options.noise_seed);
  // An explicitly supplied threshold pins that parameter, bypassing adaptive
  // derivation for it only — mirroring `rux create planes -d/-m` (#214).
  if (params.contains("plane_dist_threshold"))
    options.plane_dist_threshold_override = options.plane_dist_threshold;
  if (params.contains("min_inliers"))
    options.min_inliers_override = options.min_inliers;
  options.cancel_token = ctx.cancel_token;

  auto cloud = db.point_cloud_xyzrgb("cloud");
  auto normals = db.point_cloud_normal("normals");

  if (auto problem = resolve_filter(params, db, cloud->size(), options.filter))
    return StageResult::invalid(*problem);

  auto [labels, centroids, plane_normals] =
      geometry::segment_planes(cloud, normals, options);

  if (ctx.is_cancelled())
    return StageResult::cancel("plane segmentation cancelled");

  db.save_point_cloud("planes", *labels, "segment_planes", ctx.parameters);
  db.save_point_cloud("plane_centroids", *centroids, "segment_planes",
                      ctx.parameters);
  db.save_point_cloud("plane_normals", *plane_normals, "segment_planes",
                      ctx.parameters);

  return StageResult::success(
      fmt::format("detected {} plane(s)", centroids->size()),
      {{"cloud", "planes", static_cast<int64_t>(labels->size())},
       {"cloud", "plane_centroids", static_cast<int64_t>(centroids->size())},
       {"cloud", "plane_normals",
        static_cast<int64_t>(plane_normals->size())}});
}

StageResult run_rooms(ProjectDB &db, const StageContext &ctx,
                      const json &params) {
  geometry::SegmentRoomsOptions options;
  options.grid_size = param_or(params, "grid_size", options.grid_size);
  options.resolution = param_or(params, "resolution", options.resolution);
  options.beta = param_or(params, "beta", options.beta);
  options.max_iter = param_or(params, "max_iter", options.max_iter);
  options.propagate_k = param_or(params, "propagate_k", options.propagate_k);
  options.propagate_max_radius =
      param_or(params, "propagate_max_radius", options.propagate_max_radius);
  options.cancel_token = ctx.cancel_token;

  auto cloud = db.point_cloud_xyzrgb("cloud");
  auto planes = db.point_cloud_label("planes");
  auto plane_normals = db.point_cloud_normal("plane_normals");

  // "cloud" and "planes" are index-aligned by contract (STANDARDS §3.2); a
  // mismatch means one of them was regenerated without the other, and the
  // per-point normal expansion below would read past the end.
  if (planes->size() != cloud->size())
    return StageResult::invalid(
        fmt::format("Point cloud size mismatch: cloud={}, planes={}",
                    cloud->size(), planes->size()));

  // Expand per-plane normals to per-point normals. Plane labels are 1-based
  // (0 = unlabeled), so plane_normals is indexed via label_to_index
  // (STANDARDS §3).
  CloudNPtr normals(new CloudN);
  normals->resize(planes->size());
  normals->width = planes->width;
  normals->height = planes->height;
  for (size_t i = 0; i < planes->points.size(); ++i) {
    const uint32_t label = planes->points[i].label;
    if (!core::is_valid_label(label))
      continue;
    normals->points[i] = plane_normals->points[core::label_to_index(label)];
  }

  if (auto problem = resolve_filter(params, db, cloud->size(), options.filter))
    return StageResult::invalid(*problem);

  auto labels = geometry::segment_rooms(cloud, normals, planes, options);

  if (ctx.is_cancelled())
    return StageResult::cancel("room segmentation cancelled");

  db.save_point_cloud("rooms", *labels, "segment_rooms", ctx.parameters);

  std::set<uint32_t> unique;
  for (const auto &point : labels->points)
    if (core::is_valid_label(point.label))
      unique.insert(point.label);
  // `count` is points written, not rooms found — the room count is in the
  // summary. One cloud was written, so one artifact is reported.
  return StageResult::success(
      fmt::format("segmented {} room(s)", unique.size()),
      {{"cloud", "rooms", static_cast<int64_t>(labels->size())}});
}

StageResult run_instances(ProjectDB &db, const StageContext &ctx,
                          const json &params) {
  const auto semantic_cloud = param_or<std::string>(
      params, "semantic_cloud", std::string(kDefaultSemanticCloud));
  const auto output_cloud = param_or<std::string>(
      params, "output_cloud", std::string(kDefaultInstanceCloud));

  geometry::SegmentInstancesRequest request;
  request.cloud = db.point_cloud_xyzrgb("cloud");
  request.semantic_labels = db.point_cloud_label(semantic_cloud);
  core::warn_if_unstamped_semantic_cloud(db, semantic_cloud);
  request.cluster_tolerance =
      param_or(params, "cluster_tolerance", request.cluster_tolerance);
  request.min_cluster_size =
      param_or(params, "min_cluster_size", request.min_cluster_size);
  request.max_cluster_size =
      param_or(params, "max_cluster_size", request.max_cluster_size);
  if (auto it = params.find("labels"); it != params.end() && it->is_array())
    request.labels_to_process = it->get<std::set<uint32_t>>();
  request.cancel_token = ctx.cancel_token;

  if (!request.cloud || !request.semantic_labels)
    return StageResult::failure(
        fmt::format("could not load 'cloud' and '{}'", semantic_cloud));

  auto result = geometry::segment_instances(request);

  if (ctx.is_cancelled())
    return StageResult::cancel("instance segmentation cancelled");

  // Stable identity (#207): match the new instances against the previous ones
  // BEFORE overwriting the output cloud, so GUIDs and material links survive
  // regeneration. Skipping this would silently orphan every material passport.
  CloudLPtr old_labels;
  std::vector<ProjectDB::InstanceRecord> old_instances;
  std::map<int, std::string> old_links;
  if (db.has_point_cloud(output_cloud)) {
    try {
      old_labels = db.point_cloud_label(output_cloud);
      old_instances = db.instances(output_cloud);
      old_links = db.instance_materials(output_cloud);
    } catch (const std::exception &e) {
      warn("could not load previous instance state for '{}': {}", output_cloud,
           e.what());
    }
  }

  std::vector<geometry::PriorInstance> prior;
  prior.reserve(old_instances.size());
  for (const auto &old : old_instances)
    prior.push_back({old.instance_id, old.semantic_class, old.guid});

  auto reconcile = geometry::reconcile_instance_identities(
      old_labels.get(), *result.instance_labels, prior,
      result.instance_to_semantic, result.instance_sizes,
      []() { return core::generate_guid(); });

  db.save_point_cloud(output_cloud, *result.instance_labels,
                      "segment_instances", ctx.parameters);

  std::vector<ProjectDB::InstanceRecord> records;
  records.reserve(reconcile.instances.size());
  for (const auto &inst : reconcile.instances)
    records.push_back(
        {inst.instance_id, inst.guid, inst.semantic_class, inst.point_count});
  db.save_instances(output_cloud, records);

  std::map<int, std::string> definitions;
  for (const auto &[instance_id, semantic_class] :
       result.instance_to_semantic) {
    const size_t size = result.instance_sizes.at(instance_id);
    definitions[static_cast<int>(instance_id)] =
        fmt::format("SM{}-{} ({}p)", semantic_class, instance_id, size);
  }
  db.save_label_definitions(output_cloud, definitions);
  info("Saved {} instance definitions", definitions.size());

  // Re-create the instance->material links that the instances-row DELETE
  // cascaded away, for every instance that carried its GUID over.
  std::map<uint32_t, uint32_t> old_to_new;
  for (const auto &inst : reconcile.instances)
    if (inst.carried_over)
      old_to_new[inst.matched_old_id] = inst.instance_id;

  size_t carried = 0;
  size_t dropped = 0;
  for (const auto &[old_id, material_guid] : old_links) {
    auto it = old_to_new.find(static_cast<uint32_t>(old_id));
    if (it == old_to_new.end()) {
      ++dropped;
      warn("dropping material link: old instance {} has no match in the "
           "regenerated cloud; passport {} kept but unlinked",
           old_id, material_guid);
      continue;
    }
    try {
      db.set_instance_material(output_cloud, static_cast<int>(it->second),
                               material_guid);
      ++carried;
    } catch (const std::exception &e) {
      ++dropped;
      warn("could not re-link material {} to instance {}: {}", material_guid,
           it->second, e.what());
    }
  }
  // Loud reconciliation report (STANDARDS §3.2, §5). An instance that lost its
  // identity takes its material passport's link with it, so every orphan is
  // named rather than folded into a count.
  if (old_labels && !old_instances.empty()) {
    info("Instance identity: {} carried over, {} fresh GUID(s)",
         reconcile.matched_count, reconcile.fresh_count);
    if (!reconcile.orphaned_old.empty()) {
      warn("{} previous instance(s) had no match in the regenerated cloud:",
           reconcile.orphaned_old.size());
      for (const auto &orphan : reconcile.orphaned_old)
        warn("  - old instance {} (class {}, guid {}, best overlap {:.0f}%)",
             orphan.instance_id, orphan.semantic_class, orphan.guid,
             orphan.best_overlap * 100.0);
    }
  }
  if (!old_links.empty())
    info("Material links: {} carried over, {} dropped", carried, dropped);

  size_t labeled_points = 0;
  for (const auto &label : *result.instance_labels)
    if (core::is_valid_label(label.label))
      ++labeled_points;

  const size_t cloud_size = request.cloud->size();
  info("Summary:");
  info("  Total instances: {}", result.instance_to_semantic.size());
  info("  Labeled points: {}/{} ({:.1f}%)", labeled_points, cloud_size,
       cloud_size ? 100.0 * labeled_points / cloud_size : 0.0);

  std::map<uint32_t, std::vector<uint32_t>> semantic_to_instances;
  for (const auto &[instance_id, semantic_class] : result.instance_to_semantic)
    semantic_to_instances[semantic_class].push_back(instance_id);

  info("Per-class breakdown:");
  for (const auto &[semantic_class, instances] : semantic_to_instances) {
    size_t total_points = 0;
    for (const uint32_t instance_id : instances)
      total_points += result.instance_sizes.at(instance_id);
    info("  Semantic class {}: {} instances, {} points", semantic_class,
         instances.size(), total_points);
  }

  // Two artifacts: the per-point instance label cloud, and the instances table
  // keyed by that cloud's name (`GET /api/v1/instances/{cloud}`). The label
  // definitions written above belong to the cloud and are served with it, so
  // they are not a third artifact.
  return StageResult::success(
      fmt::format("{} instance(s): {} carried over, {} fresh",
                  reconcile.instances.size(), reconcile.matched_count,
                  reconcile.fresh_count),
      {{"cloud", output_cloud,
        static_cast<int64_t>(result.instance_labels->size())},
       {"table", output_cloud, static_cast<int64_t>(records.size())}});
}

StageResult run_mesh(ProjectDB &db, const StageContext & /*ctx*/,
                     const json &params) {
  MeshStageOptions parsed;
  try {
    parsed = mesh_options_from_parameters(params.dump());
  } catch (const std::invalid_argument &e) {
    return StageResult::invalid(e.what());
  }
  const std::string &output_name = parsed.output_name;
  geometry::MeshOptions &options = parsed.options;

  auto cloud = db.point_cloud_xyzrgb("cloud");
  auto normals = db.point_cloud_normal("normals");
  auto rooms = db.point_cloud_label("rooms");
  auto plane_labels = db.point_cloud_label("planes");
  auto plane_centroids = db.point_cloud_xyz("plane_centroids");
  auto plane_normals = db.point_cloud_normal("plane_normals");

  // Same "filter" parameter every other stage reads (resolve_filter above):
  // `rux create mesh -f ...` and a GUI/Qt run restrict to the same points.
  if (auto problem = resolve_filter(params, db, cloud->size(), options.filter))
    return StageResult::invalid(*problem);

  auto [planes, centroids, inliers] =
      reusex::io::getPlanes(plane_labels, plane_normals, plane_centroids);

  pcl::PolygonMeshPtr mesh_result = geometry::mesh(
      cloud, normals, planes, centroids, inliers, rooms, options);

  const auto vertex_count = static_cast<int64_t>(mesh_result->cloud.width) *
                            mesh_result->cloud.height;
  const auto face_count = static_cast<int64_t>(mesh_result->polygons.size());

  if (vertex_count == 0 || face_count == 0)
    return StageResult::failure(
        "mesh generation produced an empty mesh (0 vertices or faces); "
        "the MIP solver may have selected no cells — try widening the plane "
        "filter, lowering the search threshold, or re-running "
        "'rux create planes/rooms'");

  db.save_mesh(output_name, *mesh_result, "mesh_generation");

  return StageResult::success(
      fmt::format("mesh '{}' saved ({} vertices, {} faces)", output_name,
                  vertex_count, face_count),
      {{"mesh", output_name, vertex_count}});
}

StageResult dispatch(ProjectDB &db, const StageContext &ctx,
                     const json &params) {
  switch (ctx.stage) {
  case JobStage::clouds:
    return run_clouds(db, ctx, params);
  case JobStage::planes:
    return run_planes(db, ctx, params);
  case JobStage::rooms:
    return run_rooms(db, ctx, params);
  case JobStage::instances:
    return run_instances(db, ctx, params);
  case JobStage::mesh:
    return run_mesh(db, ctx, params);
  case JobStage::optimize:
    // Optimize requires reusex_slam (GTSAM), which reusex_pipeline does not
    // link. The GUI wires a custom StageExecutor from rux_lib that intercepts
    // this stage before dispatch() is reached. This path is the safe fallback
    // for any caller that forgot to provide the custom executor.
    return StageResult::failure(
        "the optimize stage requires the slam executor; register one via "
        "ServerOptions::stage_executor (see #464)");
  case JobStage::annotate:
    // ML inference (SAM3/YOLO) requires reusex_vision, which reusex_pipeline
    // does not link (STANDARDS §1) — the same reasoning that keeps
    // reusex_slam out for `optimize` above. Unlike optimize this is refused
    // as invalid input rather than a failure: there is no "wrong input",
    // just no in-process way to do the work, so a caller is meant to supply
    // its own executor (the ruxd worker executor, docs/design/gpu-worker.md
    // W0, ruxd repo) rather than retry the default one.
    return StageResult::invalid(
        "annotate needs an executor with a vision backend");
  case JobStage::graph_discover:
    return StageResult::failure(
        "the graph_discover stage requires the slam executor "
        "(pipeline::stage_executor_with_optimize, see #464)");
  }
  return StageResult::failure("unknown stage");
}

} // namespace

// --- mesh parameters ------------------------------------------------------

MeshStageOptions mesh_options_from_parameters(std::string_view parameters) {
  json params = json::object();
  if (!parameters.empty()) {
    params = json::parse(parameters, nullptr, /*allow_exceptions=*/false);
    if (params.is_discarded() || !params.is_object())
      throw std::invalid_argument("mesh parameters must be a JSON object");
  }
  MeshStageOptions out;
  geometry::MeshOptions &options = out.options;
  out.output_name = param_or<std::string>(params, "output_name", "mesh");
  options.search_threshold = static_cast<float>(
      param_or(params, "search_threshold",
               static_cast<double>(options.search_threshold)));
  options.new_plane_offset = static_cast<float>(
      param_or(params, "new_plane_offset",
               static_cast<double>(options.new_plane_offset)));
  options.time_limit_seconds =
      param_or(params, "time_limit_seconds", options.time_limit_seconds);
  options.alpha = param_or(params, "alpha", options.alpha);
  options.max_cells = param_or(params, "max_cells", options.max_cells);
  options.sectioned = param_or(params, "sectioned", options.sectioned);
  options.sectioned_threshold =
      param_or(params, "sectioned_threshold", options.sectioned_threshold);
  const auto solver_str = param_or<std::string>(params, "solver", "auto");
  try {
    options.solver = geometry::parse_solver_choice(solver_str);
  } catch (const std::exception &e) {
    throw std::invalid_argument(
        fmt::format("invalid solver '{}': {}", solver_str, e.what()));
  }
  return out;
}

// --- StageResult ----------------------------------------------------------

StageResult StageResult::success(std::string message) {
  return StageResult{true, false, false, std::move(message), {}};
}

StageResult StageResult::success(std::string message,
                                 std::vector<StageArtifact> outputs) {
  return StageResult{true, false, false, std::move(message),
                     std::move(outputs)};
}

StageResult StageResult::failure(std::string message) {
  return StageResult{false, false, false, std::move(message), {}};
}

StageResult StageResult::invalid(std::string message) {
  return StageResult{false, false, true, std::move(message), {}};
}

StageResult StageResult::cancel(std::string message) {
  return StageResult{false, true, false, std::move(message), {}};
}

// --- StageContext ---------------------------------------------------------

bool StageContext::is_cancelled() const noexcept {
  return cancel_token != nullptr &&
         cancel_token->load(std::memory_order_acquire);
}

// --- Stage naming ---------------------------------------------------------

std::string_view to_string(JobStage stage) { return descriptor(stage).name; }

std::optional<JobStage> parse_job_stage(std::string_view name) {
  for (const auto &entry : kStages)
    if (entry.name == name)
      return entry.stage;
  return std::nullopt;
}

std::vector<std::string> job_stage_names() {
  std::vector<std::string> names;
  names.reserve(kStages.size());
  for (const auto &entry : kStages)
    names.emplace_back(entry.name);
  return names;
}

std::string_view pipeline_log_name(JobStage stage) {
  return descriptor(stage).log_name;
}

bool stage_supports_cancellation(JobStage stage) {
  return descriptor(stage).cancellable;
}

// --- Execution ------------------------------------------------------------

StageResult run_stage(ProjectDB &db, const StageContext &ctx) {
  const auto &desc = descriptor(ctx.stage);
  int log_id = -1;

  // Parameters are parsed before anything is recorded: a blob that is not even
  // JSON describes no attempt worth logging, and it is the caller's mistake
  // rather than the stage's failure.
  json params;
  try {
    params = parse_parameters(ctx.parameters);
  } catch (const std::exception &e) {
    error("stage '{}' refused: invalid parameters: {}", desc.name, e.what());
    return StageResult::invalid(
        fmt::format("invalid parameters: {}", e.what()));
  }

  try {
    // Open the log row BEFORE validating inputs, so a run that is refused for
    // an unsatisfied contract still leaves a durable record of the attempt and
    // its reason. `pipeline_log` is what the GUI's history view reads
    // (`ruxd`'s docs/gui/openapi.yaml, in its own repo), and a rejection that
    // vanishes without trace is exactly the silent failure STANDARDS §5
    // forbids.
    //
    // The job id is folded into the logged parameters (rather than needing a
    // schema migration) so the durable history can be joined back to the job
    // that caused it — the recovery path the contract documents for a client
    // that reconnects after a restart.
    log_id = db.log_pipeline_start(desc.log_name,
                                   logged_parameters(params, ctx.job_id));

    if (auto problem = check_inputs(db, desc.contract,
                                    contract_overrides(ctx.stage, params))) {
      error("stage '{}' refused: {}", desc.name, *problem);
      db.log_pipeline_end(log_id, false, *problem);
      return StageResult::invalid(*problem);
    }

    core::stopwatch watch;
    StageResult result = dispatch(db, ctx, params);
    const double elapsed = watch.elapsed();

    if (result.cancelled) {
      warn("stage '{}' cancelled after {:.2f}s: {}", desc.name, elapsed,
           result.message);
      db.log_pipeline_end(log_id, ProjectDB::PipelineOutcome::cancelled,
                          result.message);
    } else if (result.ok) {
      info("stage '{}' finished in {:.2f}s: {}", desc.name, elapsed,
           result.message);
      db.log_pipeline_end(log_id, true);
    } else if (result.invalid_input) {
      error("stage '{}' refused: {}", desc.name, result.message);
      db.log_pipeline_end(log_id, false, result.message);
    } else {
      error("stage '{}' failed after {:.2f}s: {}", desc.name, elapsed,
            result.message);
      db.log_pipeline_end(log_id, false, result.message);
    }
    return result;

  } catch (const std::exception &e) {
    // A cancel token tripped inside the algorithm may surface as an exception;
    // report it as a cancellation rather than a failure so the job status
    // matches what the user asked for.
    const bool cancelled = ctx.is_cancelled();
    const std::string message =
        cancelled ? fmt::format("cancelled ({})", e.what()) : e.what();
    if (cancelled)
      warn("stage '{}' cancelled: {}", desc.name, e.what());
    else
      error("stage '{}' failed: {}", desc.name, e.what());

    if (log_id >= 0) {
      try {
        db.log_pipeline_end(log_id,
                            cancelled ? ProjectDB::PipelineOutcome::cancelled
                                      : ProjectDB::PipelineOutcome::failed,
                            message);
      } catch (const std::exception &nested) {
        error("could not close pipeline_log row {}: {}", log_id, nested.what());
      }
    }
    return cancelled ? StageResult::cancel(message)
                     : StageResult::failure(message);
  }
}

StageResult run_stage(const StageContext &ctx) {
  try {
    ProjectDB db(ctx.project, /*readOnly=*/false);
    return run_stage(db, ctx);
  } catch (const std::exception &e) {
    error("could not open project '{}': {}", ctx.project.string(), e.what());
    return StageResult::failure(fmt::format("could not open project '{}': {}",
                                            ctx.project.string(), e.what()));
  }
}

StageExecutor default_stage_executor() {
  return [](const StageContext &ctx) { return run_stage(ctx); };
}

} // namespace reusex::pipeline
