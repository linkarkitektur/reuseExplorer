// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "reusex/pipeline/stages.hpp"
#include "reusex/core/label_encoding.hpp"
#include "reusex/pipeline/mesh_parameters.hpp"
#include "reusex/pipeline/opening_parameters.hpp"
#include "reusex/pipeline/solid_parameters.hpp"
#include "reusex/pipeline/space_parameters.hpp"
#include "reusex/pipeline/stage_parameters.hpp"
#include "reusex/pipeline/tile_index.hpp"
#include "reusex/pipeline/wall_parameters.hpp"

#include "reusex/core/ProjectDB.hpp"
#include "reusex/core/SensorIntrinsics.hpp"
#include "reusex/core/class_role_map.hpp"
#include "reusex/core/filter_expression.hpp"
#include "reusex/core/guid.hpp"
#include "reusex/core/label_semantics.hpp"
#include "reusex/core/logging.hpp"
#include "reusex/core/normal_orientation.hpp"
#include "reusex/core/opening_identity.hpp"
#include "reusex/core/space_identity.hpp"
#include "reusex/core/storey_identity.hpp"
#include "reusex/core/validate.hpp"
#include "reusex/core/wall_identity.hpp"
#include "reusex/geometry/level_frame.hpp"
#include "reusex/geometry/transform_utils.hpp"
#include "reusex/io/reusex.hpp"
#include "reusex/io/solid_model_json.hpp"
#include "reusex/reconstruction/mesh.hpp"
#include "reusex/reconstruction/openings.hpp"
#include "reusex/reconstruction/solids.hpp"
#include "reusex/reconstruction/walls.hpp"
#include "reusex/segmentation/band_rooms.hpp"
#include "reusex/segmentation/classify_planes.hpp"
#include "reusex/segmentation/level.hpp"
#include "reusex/segmentation/reconcile_instances.hpp"
#include "reusex/segmentation/reconstruct.hpp"
#include "reusex/segmentation/segment_instances.hpp"
#include "reusex/segmentation/segment_planes.hpp"
#include "reusex/segmentation/segment_rooms.hpp"
#include "reusex/segmentation/spaces.hpp"
#include "reusex/types/point_types.hpp"

#include <fmt/format.h>
#include <fmt/ranges.h>
#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <pcl/PolygonMesh.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
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
constexpr std::array<StageName, 14> kStages{{
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
    // Not cancellable: one histogram and a few bounded least-squares fits
    // over the horizontal planes, seconds even on a large scan.
    {JobStage::storeys, "storeys", "storeys", core::PipelineStage::storeys,
     false},
    // Not cancellable: one pass over the plane inliers and a sort per plane.
    {JobStage::classify, "classify", "classify", core::PipelineStage::classify,
     false},
    // Not cancellable: per storey, histograms and bounded fits over the wall
    // points; about a second on NewOffice (1.5 M points).
    {JobStage::walls, "walls", "walls", core::PipelineStage::walls, false},
    // Not cancellable: per wall, a raster over the points near it; a few
    // seconds on NewOffice (88 walls, 1.5 M points).
    {JobStage::openings, "openings", "openings", core::PipelineStage::openings,
     false},
    // Not cancellable between storeys only: the band grid is rebuilt
    // (cancellable inside segment_band_rooms) and one CGAL arrangement per
    // storey is labelled; a few seconds on NewOffice.
    {JobStage::spaces, "spaces", "spaces", core::PipelineStage::spaces, true},
    // Not cancellable: one box and a corefinement per opening per wall, a
    // polygon union per storey; about a second on NewOffice.
    {JobStage::solids, "solids", "solids", core::PipelineStage::solids, false},
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
  // The plane normals take their sign from the point normals, so they face
  // the sensor exactly when those do: carry the clouds stage's marker over
  // (reusex-0se). Legacy normals have none, and neither do their planes.
  std::string normals_params = ctx.parameters;
  if (const auto orientation = core::normal_orientation_of(
          db.point_cloud_provenance("normals").second)) {
    json p = params;
    p[core::kNormalOrientationKey] = *orientation;
    normals_params = p.dump();
  }
  db.save_point_cloud("plane_normals", *plane_normals, "segment_planes",
                      normals_params);

  return StageResult::success(
      fmt::format("detected {} plane(s)", centroids->size()),
      {{"cloud", "planes", static_cast<int64_t>(labels->size())},
       {"cloud", "plane_centroids", static_cast<int64_t>(centroids->size())},
       {"cloud", "plane_normals",
        static_cast<int64_t>(plane_normals->size())}});
}

/// `rooms` with `method = band`: rooms from the clutter-free height band
/// (segmentation/band_rooms.hpp, reusex-rqn.2). Needs the storeys table for
/// the level frame and the floor / ceiling heights, and the per-point
/// "normals" cloud.
geometry::BandRoomsOptions band_rooms_options(const json &params) {
  geometry::BandRoomsOptions o;
  o.cell_size = param_or(params, "band_cell_size", o.cell_size);
  o.band_bottom = param_or(params, "band_bottom", o.band_bottom);
  o.band_top = param_or(params, "band_top", o.band_top);
  o.core_radius = param_or(params, "band_core_radius", o.core_radius);
  o.min_room_area = param_or(params, "band_min_room_area", o.min_room_area);
  o.wall_min_points =
      param_or(params, "band_wall_min_points", o.wall_min_points);
  o.footprint_close =
      param_or(params, "band_footprint_close", o.footprint_close);
  o.use_wall_graph = param_or(params, "band_use_walls", o.use_wall_graph);
  o.cut_at_doors = param_or(params, "band_cut_at_doors", o.cut_at_doors);
  o.wall_graph_min_seen_share =
      param_or(params, "band_wall_min_seen_share", o.wall_graph_min_seen_share);
  return o;
}

/// The walls / openings tables the band rooms are cut by (W4.1, W5.4), as
/// @p o asks for them.
geometry::BandRoomsPlan band_rooms_plan(const ProjectDB &db,
                                        const geometry::BandRoomsOptions &o) {
  geometry::BandRoomsPlan plan;
  if (o.use_wall_graph || o.cut_at_doors) {
    plan.walls = db.walls();
    if (o.cut_at_doors)
      plan.openings = db.openings();
  }
  return plan;
}

/// Polygonise band rooms (W4.2) and give the spaces their stable GUIDs
/// against the stored spaces (W4.3): the spaces stage and the rooms
/// --spaces-json debug path share this.
geometry::SpacesResult polygonise_with_guids(
    const ProjectDB &db, const std::vector<geometry::RoomGrid> &grids,
    const std::vector<core::StoreyRecord> &storeys,
    const std::vector<core::WallRecord> &walls, const SpaceStageOptions &so,
    core::SpaceIdentityResult *ids = nullptr) {
  auto sp = geometry::polygonise_spaces(grids, storeys, walls, so.spaces);
  const auto r = core::assign_space_guids(sp.spaces, sp.boundaries, db.spaces(),
                                          db.project_guid(), so.identity);
  if (ids)
    *ids = r;
  return sp;
}

StageResult run_band_rooms(ProjectDB &db, const StageContext &ctx,
                           const json &params) {
  geometry::BandRoomsOptions o = band_rooms_options(params);
  o.cancel_token = ctx.cancel_token;
  if (!(o.band_top > o.band_bottom))
    return StageResult::invalid(
        fmt::format("band_top ({} m) must lie above band_bottom ({} m)",
                    o.band_top, o.band_bottom));

  const auto storeys = db.storeys();
  if (storeys.empty())
    return StageResult::invalid(
        "rooms --method band needs the storeys table (level frame, floor "
        "and ceiling heights); run `rux create storeys` first");
  if (!db.has_point_cloud("normals"))
    return StageResult::invalid(
        "rooms --method band needs the 'normals' cloud; run `rux create "
        "clouds` first");

  auto cloud = db.point_cloud_xyzrgb("cloud");
  auto normals = db.point_cloud_normal("normals");
  if (normals->size() != cloud->size())
    return StageResult::invalid(
        fmt::format("Point cloud size mismatch: cloud={}, normals={}",
                    cloud->size(), normals->size()));

  IndicesConstPtr filter;
  if (auto problem = resolve_filter(params, db, cloud->size(), filter))
    return StageResult::invalid(*problem);

  // The walls and openings tables, when the walls / openings stages have
  // run (recommended order walls -> openings -> rooms): the wall graph
  // blocks the grid and the doors cut the rooms (W4.1, W5.4). Optional: the
  // constriction watershed alone cuts the rooms without them.
  const geometry::BandRoomsPlan plan = band_rooms_plan(db, o);
  if (plan.walls.empty())
    info("rooms: no walls table (run `rux create walls` and `rux create "
         "openings` before rooms to cut rooms at walls and doors); band "
         "evidence only");
  else if (o.cut_at_doors && plan.openings.empty())
    info("rooms: {} wall(s) but no openings table (run `rux create "
         "openings` before rooms to cut rooms at doors); constriction cuts "
         "only",
         plan.walls.size());

  auto r = geometry::segment_band_rooms(*cloud, *normals, storeys, o,
                                        filter ? filter.get() : nullptr,
                                        plan.walls.empty() ? nullptr : &plan);
  if (ctx.is_cancelled())
    return StageResult::cancel("room segmentation cancelled");

  db.save_point_cloud("rooms", *r.labels, "segment_rooms", ctx.parameters);

  // Debug path (W4.2, reusex-z1v.6.2): the rooms as space polygons against
  // the wall graph, written as a solid-model document for `rux analyze
  // solid --model` without storing them. The spaces stage (W4.3) stores
  // the same spaces, through the same code and with the same GUIDs.
  const auto spaces_json =
      param_or<std::string>(params, "spaces_json", std::string());
  if (!spaces_json.empty()) {
    SpaceStageOptions so;
    so.spaces.use_walls = !param_or(params, "spaces_raster", false);
    so.spaces.wall_min_seen_share = o.wall_graph_min_seen_share;
    core::SolidModel model;
    model.name = "band rooms as spaces (debug)";
    model.description = so.spaces.use_walls
                            ? "rux create rooms --spaces-json: band rooms "
                              "polygonised against the wall graph (W4.2)"
                            : "rux create rooms --spaces-json "
                              "--spaces-raster: band rooms' raster outlines";
    model.storeys = storeys;
    model.walls = plan.walls.empty() ? db.walls() : plan.walls;
    auto sp = polygonise_with_guids(db, r.grids, storeys, model.walls, so);
    model.spaces = std::move(sp.spaces);
    model.space_boundaries = std::move(sp.boundaries);
    model.provided.spaces = true;
    io::solid_model_to_file(model, spaces_json);
    info("rooms: wrote {} space(s) and {} space boundar(y/ies) to {}",
         model.spaces.size(), model.space_boundaries.size(), spaces_json);
  }

  int graph_walls = 0;
  int door_cuts = 0;
  for (const auto &g : r.grids) {
    graph_walls += g.graph_walls;
    door_cuts += g.door_cuts;
  }
  return StageResult::success(
      fmt::format("segmented {} room(s) in {} storey(s) from the {}-{} m "
                  "band, {} wall(s) and {} door cut(s) from the walls / "
                  "openings tables; {} of {} points unlabeled",
                  r.rooms, r.grids.size(), o.band_bottom, o.band_top,
                  graph_walls, door_cuts, r.unlabeled_points, cloud->size()),
      {{"cloud", "rooms", static_cast<int64_t>(r.labels->size())}});
}

StageResult run_rooms(ProjectDB &db, const StageContext &ctx,
                      const json &params) {
  const bool method_specified =
      params.contains("method") && !params.at("method").is_null();
  auto method = param_or<std::string>(
      params, "method", std::string(geometry::kDefaultRoomsMethod));
  if (method != geometry::kRoomsMethodBand &&
      method != geometry::kRoomsMethodLeiden)
    return StageResult::invalid(
        fmt::format("unknown rooms method '{}' (expected '{}' or '{}')", method,
                    geometry::kRoomsMethodLeiden, geometry::kRoomsMethodBand));

  // Band is the default (reusex-icg), but it needs the storeys table. An
  // explicit `--method band` without storeys still refuses below
  // (run_band_rooms) — that is a user asking for something the project
  // cannot do yet. The *default* degrades instead of breaking every
  // `clouds -> planes -> rooms` pipeline that has not run `create storeys`:
  // fall back to Leiden, with a warning naming the reason and both fixes.
  std::string fallback_note;
  if (method == geometry::kRoomsMethodBand && !method_specified &&
      db.storeys().empty()) {
    warn("rooms: default method 'band' needs the storeys table, but the "
         "project has none; falling back to 'leiden' for this run. Run `rux "
         "create storeys` first to use band, or pass --method leiden to "
         "silence this warning.");
    method = geometry::kRoomsMethodLeiden;
    fallback_note =
        " (method defaulted to band, but the project has no storeys; fell "
        "back to leiden - run `rux create storeys` or pass --method leiden)";
  }

  if (method == geometry::kRoomsMethodBand)
    return run_band_rooms(db, ctx, params);

  geometry::SegmentRoomsOptions options;
  options.grid_size = param_or(params, "grid_size", options.grid_size);
  options.resolution = param_or(params, "resolution", options.resolution);
  options.beta = param_or(params, "beta", options.beta);
  options.max_iter = param_or(params, "max_iter", options.max_iter);
  options.propagate_k = param_or(params, "propagate_k", options.propagate_k);
  options.propagate_max_radius =
      param_or(params, "propagate_max_radius", options.propagate_max_radius);
  options.min_room_area =
      param_or(params, "min_room_area", options.min_room_area);
  options.merge_affinity =
      param_or(params, "merge_affinity", options.merge_affinity);
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
      fmt::format("segmented {} room(s){}", unique.size(), fallback_note),
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
  // parsed.options is `options`, filter included: this adds the plane roles
  // and storey section heights the project provides.
  options = mesh_options_for(db, parsed, inliers);

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

StageResult run_storeys(ProjectDB &db, const json &params, int log_id) {
  geometry::LevelOptions o;
  o.horizontal_cos = param_or(params, "horizontal_cos", o.horizontal_cos);
  o.surface_max_tilt_deg =
      param_or(params, "surface_max_tilt_deg", o.surface_max_tilt_deg);
  o.inlier_m = param_or(params, "inlier_m", o.inlier_m);
  o.max_fit_iterations =
      param_or(params, "max_fit_iterations", o.max_fit_iterations);
  o.hist_bin_m = param_or(params, "hist_bin_m", o.hist_bin_m);
  o.peak_min_share = param_or(params, "peak_min_share", o.peak_min_share);
  o.slab_merge_m = param_or(params, "slab_merge_m", o.slab_merge_m);
  o.min_storey_height_m =
      param_or(params, "min_storey_height_m", o.min_storey_height_m);
  o.default_height_m = param_or(params, "default_height_m", o.default_height_m);
  o.use_normal_sign = param_or(params, "use_normal_sign", o.use_normal_sign);
  o.per_storey_frame = param_or(params, "per_storey_frame", o.per_storey_frame);
  o.storey_frame_tol_deg =
      param_or(params, "storey_frame_tol_deg", o.storey_frame_tol_deg);
  core::StoreyIdentityOptions identity;
  identity.match_m = param_or(params, "guid_match_m", identity.match_m);

  const auto cloud = db.point_cloud_xyzrgb("cloud");
  const auto planes = db.point_cloud_label("planes");
  const auto centroids = db.point_cloud_xyz("plane_centroids");
  const auto normals = db.point_cloud_normal("plane_normals");

  geometry::StoreysResult r;
  try {
    r = geometry::detect_storeys(*cloud, *planes, *centroids, *normals, o);
  } catch (const std::invalid_argument &e) {
    // Mismatched cloud/planes sizes or a scan with no plane: the input, not
    // the stage, is at fault.
    return StageResult::invalid(e.what());
  }

  // Stable identity (reusex-z1v, STANDARDS §6): a re-detected storey keeps
  // the GUID of the stored storey whose floor it matches in the new level
  // frame, so walls/spaces referring to it, and a locked (hand-edited) row,
  // survive a re-run that renumbers the storeys. Only an unmatched storey
  // gets a fresh deterministic GUID (core/storey_identity.hpp).
  const auto stored = db.storeys();
  const auto ids =
      core::assign_storey_guids(r.storeys, stored, db.project_guid(), identity);
  for (auto &s : r.storeys)
    if (s.name.empty())
      s.name = fmt::format("Etage {}", s.idx);
  info("storeys: {} matched a stored storey (within {} m), {} new GUID(s)",
       ids.matched, identity.match_m, ids.minted);

  // Locked rows survive the re-run in THEIR frame and height range; say so
  // when that no longer agrees with this run (docs/CONTRACTS.md: mixed
  // frames are possible only through locked rows).
  for (const auto &c : core::locked_storey_conflicts(stored, r.storeys)) {
    if (c.frame_differs)
      warn("storeys: locked storey {} ({}) keeps its own level frame, "
           "{:.3f} deg / {:.3f} m away from this run's",
           c.idx, c.guid, c.frame_angle_deg, c.frame_shift_m);
    if (!c.overlapping_idx.empty())
      warn("storeys: locked storey {} ({}) overlaps the height range of "
           "new storey(s) [{}]",
           c.idx, c.guid, fmt::join(c.overlapping_idx, ", "));
  }

  const size_t written = db.save_storeys(r.storeys, log_id);
  const auto rows = db.storeys();
  const auto locked_kept = static_cast<size_t>(std::count_if(
      rows.begin(), rows.end(), [](const auto &s) { return s.locked; }));
  if (written < r.storeys.size())
    warn("storeys: {} of {} detected storey(s) not written: a locked row "
         "holds their GUID or idx",
         r.storeys.size() - written, r.storeys.size());

  const double tilt_deg =
      std::acos(std::clamp(r.vertical[2], -1.0, 1.0)) * 180.0 / M_PI;
  return StageResult::success(
      fmt::format("{} storey(s) from {} level(s) of {} horizontal plane(s); "
                  "vertical {:.3f} deg off world z; storey verticals up to "
                  "{:.3f} deg off it, {} storey(s) in their own "
                  "frame; floor/ceiling by {}; {} row(s) written, {} locked "
                  "row(s) kept",
                  r.storeys.size(), r.levels, r.horizontal_planes, tilt_deg,
                  r.max_storey_tilt_deg, r.own_frames,
                  r.normal_sign_used ? "normal sign" : "height", written,
                  locked_kept),
      {{"table", "storeys", static_cast<int64_t>(written)}});
}

StageResult run_classify(ProjectDB &db, const json &params, int log_id) {
  geometry::ClassifyPlanesOptions o;
  o.horizontal_cos = param_or(params, "horizontal_cos", o.horizontal_cos);
  o.vertical_cos = param_or(params, "vertical_cos", o.vertical_cos);
  o.min_large_area_m2 =
      param_or(params, "min_large_area_m2", o.min_large_area_m2);
  o.band_m = param_or(params, "band_m", o.band_m);
  o.min_wall_height_m =
      param_or(params, "min_wall_height_m", o.min_wall_height_m);
  o.min_wall_height_share =
      param_or(params, "min_wall_height_share", o.min_wall_height_share);
  o.max_furniture_area_m2 =
      param_or(params, "max_furniture_area_m2", o.max_furniture_area_m2);
  o.table_min_m = param_or(params, "table_min_m", o.table_min_m);
  o.table_max_m = param_or(params, "table_max_m", o.table_max_m);
  o.min_storey_height_m =
      param_or(params, "min_storey_height_m", o.min_storey_height_m);
  o.area_cell_m = param_or(params, "area_cell_m", o.area_cell_m);
  o.extent_quantile = param_or(params, "extent_quantile", o.extent_quantile);
  o.min_labelled_share =
      param_or(params, "min_labelled_share", o.min_labelled_share);
  o.min_class_share = param_or(params, "min_class_share", o.min_class_share);
  o.min_floating_wall_run_m =
      param_or(params, "min_floating_wall_run_m", o.min_floating_wall_run_m);

  // The class -> role map: a file path (rux create classify --role-map), an
  // inline role-map object (a job built by hand), or the built-in map.
  core::ClassRoleMap role_map = core::default_class_role_map();
  std::string role_map_source = "built-in";
  if (const auto it = params.find("role_map"); it != params.end()) {
    try {
      if (it->is_string() && !it->get<std::string>().empty()) {
        role_map_source = it->get<std::string>();
        role_map = core::load_class_role_map(role_map_source);
      } else if (it->is_object()) {
        role_map_source = "inline";
        role_map = core::parse_class_role_map(it->dump());
      } else if (!it->is_string() && !it->is_null()) {
        return StageResult::invalid(
            "role_map must be a role-map file path or a role-map object");
      }
    } catch (const std::exception &e) {
      return StageResult::invalid(fmt::format("role_map: {}", e.what()));
    }
  }

  // Which way the plane normals face (reusex-0se): sensor-facing normals let
  // the rules tell a floor (seen from above) from a ceiling (seen from
  // below); legacy ones faced the world origin and their sign means nothing.
  auto orientation = core::normal_orientation_of(
      db.point_cloud_provenance("plane_normals").second);
  if (!orientation && db.has_point_cloud("normals"))
    orientation = core::normal_orientation_of(
        db.point_cloud_provenance("normals").second);
  o.oriented_normals =
      orientation && *orientation == core::kNormalOrientationSensor;
  if (o.oriented_normals)
    info("classify: plane normals face their sensor; floors and ceilings "
         "are told apart by the normal's sign");
  else
    warn("classify: the clouds carry no normal_orientation marker (made "
         "before reusex-0se, or imported), so which way a plane faces is "
         "ignored; re-run `rux create clouds` and `rux create planes` to use "
         "it");

  const auto cloud = db.point_cloud_xyzrgb("cloud");
  const auto planes = db.point_cloud_label("planes");
  const auto normals = db.point_cloud_normal("plane_normals");
  const auto storeys = db.storeys();

  // The semantic vote is optional: a project that was never annotated is
  // classified by the rules alone.
  geometry::SemanticLabels semantic;
  CloudLPtr labels;
  if (db.has_point_cloud(kDefaultSemanticCloud)) {
    labels = db.point_cloud_label(kDefaultSemanticCloud);
    if (labels && labels->size() == cloud->size()) {
      semantic.labels = labels.get();
      for (const auto &[label, name] :
           db.label_definitions(kDefaultSemanticCloud))
        if (label >= 0)
          semantic.class_names[static_cast<uint32_t>(label)] = name;
    } else {
      warn("classify: the '{}' cloud has {} points but 'cloud' has {}; "
           "classifying without semantic labels",
           kDefaultSemanticCloud, labels ? labels->size() : 0, cloud->size());
    }
  }

  geometry::ClassifyPlanesResult r;
  try {
    r = geometry::classify_planes(*cloud, *planes, *normals, storeys, semantic,
                                  role_map, o);
  } catch (const std::invalid_argument &e) {
    return StageResult::invalid(e.what());
  }
  if (r.planes.empty())
    warn("classify: 'plane_normals' holds no plane, so there is nothing to "
         "classify; run `rux create planes` first");

  const auto project_guid = db.project_guid();
  const std::string metadata =
      json{{"oriented_normals", o.oriented_normals}}.dump();
  int outside = 0;
  for (auto &p : r.planes) {
    p.guid = core::deterministic_guid(
        project_guid, fmt::format("plane_classes/planes/{}", p.plane_label));
    p.metadata = metadata;
    if (p.storey < 0 && p.point_count > 0)
      ++outside;
  }
  if (outside > 0)
    info("classify: {} plane(s) lie outside every storey (storey -1, e.g. a "
         "roof above the top storey's ceiling)",
         outside);

  // A locked row survives the re-run as it was. If the plane segmentation
  // changed under it, the label may now mean another plane: say so.
  for (const auto &stored : db.plane_classes("planes")) {
    if (!stored.locked)
      continue;
    const auto it =
        std::find_if(r.planes.begin(), r.planes.end(), [&](const auto &p) {
          return p.plane_label == stored.plane_label;
        });
    if (it == r.planes.end())
      warn("classify: locked plane {} no longer exists in 'planes' (kept)",
           stored.plane_label);
    else if (it->point_count != stored.point_count)
      warn("classify: locked plane {} had {} points, the planes cloud now "
           "has {} (re-segmented since it was locked?)",
           stored.plane_label, stored.point_count, it->point_count);
  }

  const size_t written = db.save_plane_classes("planes", r.planes, log_id);
  const auto rows = db.plane_classes("planes");
  const auto locked_kept = static_cast<size_t>(std::count_if(
      rows.begin(), rows.end(), [](const auto &p) { return p.locked; }));

  std::vector<std::string> counts;
  for (const auto &[role, n] : r.role_counts)
    counts.push_back(fmt::format("{} {}", n, role));
  return StageResult::success(
      fmt::format("{} plane(s): {}; {} from the class map ({}), {} overruled "
                  "by geometry; normals {}; {} row(s) written, {} locked "
                  "row(s) kept",
                  r.planes.size(), fmt::join(counts, ", "), r.from_semantic,
                  role_map_source, r.overruled,
                  o.oriented_normals ? "sensor-facing" : "unoriented (legacy)",
                  written, locked_kept),
      {{"table", "plane_classes", static_cast<int64_t>(written)}});
}

StageResult run_walls(ProjectDB &db, const json &params, int log_id) {
  WallStageOptions o;
  try {
    o = wall_options_from_parameters(params.dump());
  } catch (const std::invalid_argument &e) {
    return StageResult::invalid(e.what());
  }

  // Which way the plane normals face (reusex-0se): sensor-facing normals
  // keep the two faces of a thin wall in separate offset profiles; legacy
  // ones faced the world origin and their sign means nothing.
  auto orientation = core::normal_orientation_of(
      db.point_cloud_provenance("plane_normals").second);
  if (!orientation && db.has_point_cloud("normals"))
    orientation = core::normal_orientation_of(
        db.point_cloud_provenance("normals").second);
  o.walls.use_normal_sign =
      orientation && *orientation == core::kNormalOrientationSensor;
  if (o.walls.use_normal_sign)
    info("walls: plane normals face their sensor; each side of a wall gets "
         "its own offset profile");
  else
    warn("walls: the clouds carry no normal_orientation marker (made before "
         "reusex-0se, or imported), so which side a face was seen from is "
         "unknown and thin walls may merge into one face; re-run `rux create "
         "clouds` and `rux create planes` to use it");

  const auto storeys = db.storeys();
  const auto classes = db.plane_classes("planes");
  if (storeys.empty())
    return StageResult::invalid(
        "walls: the project has no storeys; run `rux create storeys`");
  if (classes.empty())
    return StageResult::invalid("walls: the planes have no plane_classes; run "
                                "`rux create classify`");

  const auto cloud = db.point_cloud_xyzrgb("cloud");
  const auto planes = db.point_cloud_label("planes");
  const auto normals = db.point_cloud_normal("plane_normals");

  // The roles must describe these planes: a planes re-run after classify
  // renumbers them, and a stale "wall" label would pick another plane's
  // points as wall evidence.
  {
    std::map<int, int> inliers;
    for (const auto &l : planes->points)
      if (l.label >= 1)
        ++inliers[static_cast<int>(l.label)];
    for (const auto &c : classes) {
      const auto it = inliers.find(c.plane_label);
      const int have = it == inliers.end() ? 0 : it->second;
      if (have != c.point_count)
        return StageResult::invalid(fmt::format(
            "walls: plane_classes says plane {} has {} point(s) but 'planes' "
            "gives it {}: the classification is stale (planes ran again "
            "after classify); re-run `rux create classify`",
            c.plane_label, c.point_count, have));
    }
  }

  // The capture path: evidence that a wide gap between collinear walls is
  // a door (reusex-3e9).
  std::vector<Eigen::Vector3d> path;
  for (const int id : db.sensor_frame_ids()) {
    if (!db.has_sensor_frame_pose(id))
      continue;
    const Eigen::Affine3d world =
        geometry::to_affine(db.sensor_frame_pose(id)) *
        geometry::to_affine(db.sensor_frame_intrinsics(id).local_transform);
    path.push_back(world.translation());
  }

  // Closed doors are part of their wall's line (reusex-mc6): door-class
  // planes are wall support. The class names come from the semantic cloud
  // (keyed by point label = API class + 1).
  std::set<int> door_planes;
  if (db.has_point_cloud(kDefaultSemanticCloud)) {
    std::map<int, std::string> names;
    for (const auto &[label, name] :
         db.label_definitions(kDefaultSemanticCloud))
      if (label >= 1)
        names[core::point_label_to_api(static_cast<uint32_t>(label))] = name;
    door_planes = geometry::door_support_planes(classes, names);
    if (!door_planes.empty())
      info("walls: {} door plane(s) support their wall's line",
           door_planes.size());
  }

  std::vector<core::WallRecord> walls;
  std::vector<core::WallJoinRecord> joins;
  int families = 0, faces = 0, pairs = 0, low_seen = 0;
  for (const auto &st : storeys) {
    geometry::WallsResult r;
    try {
      r = geometry::detect_walls(*cloud, *planes, *normals, classes, st,
                                 o.walls, path, door_planes);
    } catch (const std::invalid_argument &e) {
      return StageResult::invalid(e.what());
    }
    auto [w, j] = geometry::wall_records(r.graph, st);
    info("walls: storey {}: {} axis famil{}, {} face(s), {} pair(s) ({} "
         "rejected), {} wall(s), {} join(s), {} wide doorway(s) bridged; "
         "not walls: {} window reveal(s), {} free-standing (columns, "
         "cupboards, desk screens)",
         st.idx, r.families.size(), r.families.size() == 1 ? "y" : "ies",
         r.faces.size(), r.pairing.pairs.size(), r.pairing.rejected, w.size(),
         j.size(), r.graph.door_bridges, r.graph.reveals,
         r.graph.free_standing);
    families += static_cast<int>(r.families.size());
    faces += static_cast<int>(r.faces.size());
    pairs += static_cast<int>(r.pairing.pairs.size());
    low_seen += r.graph.low_seen;
    walls.insert(walls.end(), w.begin(), w.end());
    joins.insert(joins.end(), j.begin(), j.end());
  }
  if (walls.empty())
    warn("walls: no wall on {} storey(s): {} axis families, {} faces, {} "
         "pairs over {} plane class(es) ({} wall role); check `rux get "
         "plane_classes` for wall planes",
         storeys.size(), families, faces, pairs, classes.size(),
         std::count_if(classes.begin(), classes.end(),
                       [](const auto &c) { return c.role == "wall"; }));

  // Stable identity (reusex-z1v, STANDARDS §6): a re-detected wall keeps the
  // GUID of the stored wall it lies on, so openings/spaces referring to it
  // and a locked (hand-edited) row survive a re-run; only an unmatched wall
  // gets a fresh GUID from its 5 cm-rounded axis key (core/wall_identity.hpp).
  const auto stored = db.walls();
  core::WallIdentityResult ids;
  try {
    ids = core::assign_wall_guids(walls, joins, stored, db.project_guid(),
                                  o.identity);
  } catch (const std::invalid_argument &e) {
    return StageResult::failure(e.what());
  }
  info("walls: {} matched a stored wall ({} locked, kept as stored), {} new "
       "GUID(s)",
       ids.matched, ids.locked_matched, ids.minted);

  const size_t written = db.save_walls(walls, joins, log_id);
  const auto rows = db.walls();
  const auto stored_joins = db.wall_joins();
  const auto locked_kept = static_cast<size_t>(std::count_if(
      rows.begin(), rows.end(), [](const auto &w) { return w.locked; }));

  const auto count = [&](auto pred) {
    return std::count_if(walls.begin(), walls.end(), pred);
  };
  const auto kind_count = [&](std::string_view kind) {
    return std::count_if(joins.begin(), joins.end(),
                         [&](const auto &j) { return j.kind == kind; });
  };
  return StageResult::success(
      fmt::format("{} wall(s) on {} storey(s): {} measured, {} assumed "
                  "(one side seen), {} external, {} seen under {:.0f} %; {} "
                  "corner, {} tee, {} collinear join(s); normals {}; {} "
                  "row(s) written, {} locked row(s) kept",
                  walls.size(), storeys.size(),
                  count([](const auto &w) { return !w.assumed; }),
                  count([](const auto &w) { return w.assumed; }),
                  count([](const auto &w) { return w.is_external; }), low_seen,
                  100.0 * o.walls.min_seen_share, kind_count("corner"),
                  kind_count("tee"), kind_count("collinear"),
                  o.walls.use_normal_sign ? "sensor-facing"
                                          : "unoriented (legacy)",
                  written, locked_kept),
      {{"table", "walls", static_cast<int64_t>(written)},
       {"table", "wall_joins", static_cast<int64_t>(stored_joins.size())}});
}

namespace {

/// What a semantic class name says for opening evidence: glass first (a
/// "glass partition", "glass door"), then door ("door", "door frame"), then
/// window.
geometry::OpeningClass opening_class_of(std::string name) {
  std::transform(name.begin(), name.end(), name.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  if (name.find("glass") != std::string::npos)
    return geometry::OpeningClass::glass;
  if (name.find("door") != std::string::npos)
    return geometry::OpeningClass::door;
  if (name.find("window") != std::string::npos)
    return geometry::OpeningClass::window;
  return geometry::OpeningClass::none;
}

/// One posed frame with a glass confidence image, for the glass evidence.
struct GlassFrame {
  Eigen::Affine3d world_to_optical;
  double fx, fy, cx, cy; ///< scaled to @ref image
  cv::Mat image;         ///< CV_8U, 0 = glass, 255 = trust depth
};

/// Share of @p rect's face (a 4 x 4 grid of samples in the wall's axis
/// plane, taken to world through @p storey) that the frames seeing it mark
/// as glass (< 128); -1 when no frame sees any sample.
double glass_share(const std::vector<GlassFrame> &frames,
                   const core::StoreyRecord &storey,
                   const core::WallRecord &wall,
                   const geometry::OpeningRect &rect) {
  const Eigen::Affine3d level_to_world =
      geometry::to_affine(storey.level_transform).inverse();
  const Eigen::Vector2d a(wall.x0, wall.y0), b(wall.x1, wall.y1);
  const double len = (b - a).norm();
  if (!(len > 0.0))
    return -1.0;
  const Eigen::Vector2d d = (b - a) / len;
  int seen = 0, glass = 0;
  for (int i = 0; i < 4; ++i)
    for (int k = 0; k < 4; ++k) {
      const double s =
          rect.s_start + (i + 0.5) / 4.0 * (rect.s_end - rect.s_start);
      const double z =
          rect.z_sill + (k + 0.5) / 4.0 * (rect.z_head - rect.z_sill);
      const Eigen::Vector2d q = a + s * d;
      const Eigen::Vector3d w =
          level_to_world * Eigen::Vector3d(q.x(), q.y(), z);
      for (const auto &f : frames) {
        const Eigen::Vector3d c = f.world_to_optical * w;
        if (c.z() < 0.3 || c.z() > 6.0)
          continue;
        const double u = f.fx * c.x() / c.z() + f.cx;
        const double v = f.fy * c.y() / c.z() + f.cy;
        if (u < 0 || v < 0 || u >= f.image.cols || v >= f.image.rows)
          continue;
        ++seen;
        if (f.image.at<std::uint8_t>(static_cast<int>(v), static_cast<int>(u)) <
            128)
          ++glass;
      }
    }
  return seen > 0 ? static_cast<double>(glass) / seen : -1.0;
}

} // namespace

StageResult run_openings(ProjectDB &db, const json &params, int log_id) {
  OpeningStageOptions o;
  try {
    o = opening_options_from_parameters(params.dump());
  } catch (const std::invalid_argument &e) {
    return StageResult::invalid(e.what());
  }

  const auto storeys = db.storeys();
  const auto walls = db.walls();
  if (storeys.empty())
    return StageResult::invalid(
        "openings: the project has no storeys; run `rux create storeys`");
  if (walls.empty())
    return StageResult::invalid(
        "openings: the project has no walls; run `rux create walls`");
  const auto cloud = db.point_cloud_xyzrgb("cloud");
  const std::size_t n = cloud->size();

  // Semantic classes (optional): what each point's class says.
  std::vector<geometry::OpeningClass> classes;
  std::map<int, geometry::OpeningClass> class_of_label;
  if (db.has_point_cloud(kDefaultSemanticCloud)) {
    for (const auto &[label, name] :
         db.label_definitions(kDefaultSemanticCloud))
      class_of_label[label] = opening_class_of(name);
    const auto labels = db.point_cloud_label(kDefaultSemanticCloud);
    if (labels && labels->size() == n) {
      classes.assign(n, geometry::OpeningClass::none);
      for (std::size_t i = 0; i < n; ++i)
        if (auto it =
                class_of_label.find(static_cast<int>(labels->points[i].label));
            it != class_of_label.end())
          classes[i] = it->second;
    } else {
      warn("openings: the '{}' cloud has {} points but 'cloud' has {}; no "
           "semantic evidence",
           kDefaultSemanticCloud, labels ? labels->size() : 0, n);
    }
  }
  // Instances (optional): the instance each point belongs to.
  std::vector<int> instance;
  std::vector<geometry::OpeningInstance> instances;
  if (db.has_point_cloud("instances")) {
    const auto ic = db.point_cloud_label("instances");
    const auto rows = db.instances("instances");
    if (ic && ic->size() == n && !rows.empty()) {
      std::map<uint32_t, int> index;
      for (const auto &r : rows) {
        index[r.instance_id] = static_cast<int>(instances.size());
        const auto it = class_of_label.find(r.semantic_class);
        instances.push_back({r.guid, it == class_of_label.end()
                                         ? geometry::OpeningClass::none
                                         : it->second});
      }
      instance.assign(n, -1);
      for (std::size_t i = 0; i < n; ++i)
        if (auto it = index.find(ic->points[i].label); it != index.end())
          instance[i] = it->second;
    } else {
      warn("openings: the 'instances' cloud has {} points ({} rows) but "
           "'cloud' has {}; no instance evidence (re-run `rux create "
           "instances`)",
           ic ? ic->size() : 0, rows.size(), n);
    }
  }
  // The camera path (frames in capture order) and the glass images.
  std::vector<Eigen::Vector3d> path;
  std::vector<GlassFrame> glass_frames;
  const auto glass_ids = db.glass_confidence_image_ids();
  const std::set<int> glass_set(glass_ids.begin(), glass_ids.end());
  // At most ~300 glass frames, evenly over the capture (memory: the images
  // are kept at a quarter of their size).
  const std::size_t glass_step =
      std::max<std::size_t>(1, glass_ids.size() / 300);
  std::size_t glass_seen = 0;
  for (const int id : db.sensor_frame_ids()) {
    if (!db.has_sensor_frame_pose(id))
      continue;
    const auto intr = db.sensor_frame_intrinsics(id);
    const Eigen::Affine3d world =
        geometry::to_affine(db.sensor_frame_pose(id)) *
        geometry::to_affine(intr.local_transform);
    path.push_back(world.translation());
    if (!glass_set.count(id) || glass_seen++ % glass_step != 0)
      continue;
    cv::Mat img = db.glass_confidence_image(id);
    if (img.empty() || intr.width <= 0 || intr.height <= 0)
      continue;
    cv::resize(img, img, cv::Size(), 0.25, 0.25, cv::INTER_NEAREST);
    GlassFrame g;
    g.world_to_optical = world.inverse();
    g.fx = intr.fx * img.cols / intr.width;
    g.fy = intr.fy * img.rows / intr.height;
    g.cx = intr.cx * img.cols / intr.width;
    g.cy = intr.cy * img.rows / intr.height;
    g.image = img;
    glass_frames.push_back(std::move(g));
  }
  info("openings: evidence: {} semantic labels, {} instances, {} camera "
       "positions, {} glass frames",
       classes.empty() ? "no" : "with", instances.size(), path.size(),
       glass_frames.size());

  std::vector<core::OpeningRecord> found;
  std::map<std::string, int> kinds;
  int open_doors = 0, crossed = 0, with_instance = 0;
  for (const auto &st : storeys) {
    const Eigen::Affine3d T = geometry::to_affine(st.level_transform);
    geometry::OpeningEvidence ev;
    std::vector<std::size_t> keep;
    for (std::size_t i = 0; i < n; ++i) {
      const auto &p = cloud->points[i];
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
        continue;
      const Eigen::Vector3d q = T * Eigen::Vector3d(p.x, p.y, p.z);
      if (q.z() < st.z_floor - 0.2 || q.z() > st.z_ceiling + 0.2)
        continue;
      ev.points.push_back(q);
      keep.push_back(i);
    }
    if (!classes.empty())
      for (const auto i : keep)
        ev.classes.push_back(classes[i]);
    if (!instance.empty()) {
      for (const auto i : keep)
        ev.instance.push_back(instance[i]);
      ev.instances = instances;
    }
    for (const auto &c : path)
      ev.trajectory.push_back(T * c);
    if (!glass_frames.empty())
      ev.glass = [&, st](const core::WallRecord &w,
                         const geometry::OpeningRect &r) {
        return glass_share(glass_frames, st, w, r);
      };
    std::vector<geometry::DetectedOpening> det;
    try {
      det = geometry::detect_openings(ev, walls, st, o.openings);
    } catch (const std::invalid_argument &e) {
      return StageResult::invalid(e.what());
    }
    int on_storey = 0;
    for (auto &d : det) {
      ++kinds[d.record.kind];
      open_doors += d.record.state == "open" ? 1 : 0;
      crossed += d.crossings > 0 ? 1 : 0;
      with_instance += d.record.source_instance_guid.empty() ? 0 : 1;
      found.push_back(std::move(d.record));
      ++on_storey;
    }
    info("openings: storey {}: {} point(s), {} opening(s)", st.idx,
         ev.points.size(), on_storey);
  }
  if (found.empty())
    warn("openings: no opening in {} wall(s) on {} storey(s) over {} points; "
         "check the walls (`rux render --layers cloud,walls`) and the "
         "occluder reach (--front {})",
         walls.size(), storeys.size(), n, o.openings.front_m);

  // Stable identity: a re-detected opening keeps the GUID of the stored one
  // in the same wall (core/opening_identity.hpp); walls keep theirs across a
  // walls re-run, so a locked (hand-fixed) opening survives both.
  core::OpeningIdentityResult ids;
  try {
    ids = core::assign_opening_guids(found, db.openings(), db.project_guid(),
                                     o.identity);
  } catch (const std::invalid_argument &e) {
    return StageResult::failure(e.what());
  }
  info("openings: {} matched a stored opening ({} locked, kept as stored), "
       "{} new GUID(s)",
       ids.matched, ids.locked_matched, ids.minted);

  const size_t written = db.save_openings(found, log_id);
  const auto rows = db.openings();
  const auto locked_kept = static_cast<size_t>(std::count_if(
      rows.begin(), rows.end(), [](const auto &r) { return r.locked; }));
  return StageResult::success(
      fmt::format("{} opening(s) in {} wall(s): {} door(s) ({} open), {} "
                  "window(s), {} opening(s); {} crossed by the camera path, "
                  "{} with an instance; {} row(s) written, {} locked row(s) "
                  "kept",
                  found.size(), walls.size(), kinds["door"], open_doors,
                  kinds["window"], kinds["opening"], crossed, with_instance,
                  written, locked_kept),
      {{"table", "openings", static_cast<int64_t>(rows.size())}});
}

/// `spaces` (W4.3, reusex-z1v.6.3): the band rooms of the `rooms` cloud as
/// space polygons against the wall graph, stored with stable GUIDs. The
/// band grid is rebuilt with the parameters `rooms` was saved with and must
/// reproduce its labels, so a space's room_label is the cloud's.
StageResult run_spaces(ProjectDB &db, const StageContext &ctx,
                       const json &params, int log_id) {
  SpaceStageOptions so;
  try {
    so = space_options_from_parameters(params.dump());
  } catch (const std::invalid_argument &e) {
    return StageResult::invalid(e.what());
  }
  const auto storeys = db.storeys();
  if (storeys.empty())
    return StageResult::invalid(
        "spaces: the project has no storeys; run `rux create storeys`");

  // The rooms cloud's own parameters: method and band options.
  json rooms_params = json::object();
  try {
    const auto [rooms_stage, rooms_json] = db.point_cloud_provenance("rooms");
    (void)rooms_stage;
    if (!rooms_json.empty())
      rooms_params = parse_parameters(rooms_json);
  } catch (const std::exception &e) {
    return StageResult::invalid(fmt::format(
        "spaces: cannot read the parameters of the 'rooms' cloud ({}); "
        "re-run `rux create rooms`",
        e.what()));
  }
  const auto method = param_or<std::string>(
      rooms_params, "method", std::string(geometry::kDefaultRoomsMethod));
  if (method != geometry::kRoomsMethodBand)
    return StageResult::invalid(fmt::format(
        "spaces: the 'rooms' cloud was made with --method {}; spaces are "
        "polygonised from band rooms -- run `rux create rooms --method band`",
        method));

  geometry::BandRoomsOptions o = band_rooms_options(rooms_params);
  o.cancel_token = ctx.cancel_token;
  auto cloud = db.point_cloud_xyzrgb("cloud");
  auto normals = db.point_cloud_normal("normals");
  auto stored = db.point_cloud_label("rooms");
  if (normals->size() != cloud->size() || stored->size() != cloud->size())
    return StageResult::invalid(fmt::format(
        "spaces: point cloud size mismatch: cloud={}, normals={}, rooms={}; "
        "re-run `rux create rooms`",
        cloud->size(), normals->size(), stored->size()));
  IndicesConstPtr filter;
  if (auto problem = resolve_filter(rooms_params, db, cloud->size(), filter))
    return StageResult::invalid("spaces: the rooms cloud's " + *problem);

  const geometry::BandRoomsPlan plan = band_rooms_plan(db, o);
  auto r = geometry::segment_band_rooms(*cloud, *normals, storeys, o,
                                        filter ? filter.get() : nullptr,
                                        plan.walls.empty() ? nullptr : &plan);
  if (ctx.is_cancelled())
    return StageResult::cancel("spaces cancelled");

  // The rebuilt grid must be the one the rooms cloud came from.
  size_t differ = 0;
  for (size_t i = 0; i < cloud->size(); ++i)
    if ((*r.labels)[i].label != (*stored)[i].label)
      ++differ;
  if (differ > 0)
    return StageResult::invalid(fmt::format(
        "spaces: the 'rooms' cloud no longer matches its band grid: {} of {} "
        "points ({:.2f} %) get another room when the grid is rebuilt from "
        "today's storeys / walls / openings ({} wall(s), {} opening(s)); "
        "re-run `rux create rooms`",
        differ, cloud->size(),
        100.0 * static_cast<double>(differ) /
            static_cast<double>(std::max<size_t>(cloud->size(), 1)),
        plan.walls.size(), plan.openings.size()));

  const auto walls = db.walls();
  if (so.spaces.use_walls && walls.empty())
    warn("spaces: no walls table (run `rux create walls` first); every space "
         "is the raster outline of its room");
  core::SpaceIdentityResult ids;
  geometry::SpacesResult sp;
  try {
    sp = polygonise_with_guids(db, r.grids, storeys, walls, so, &ids);
  } catch (const std::invalid_argument &e) {
    return StageResult::failure(e.what());
  }
  // Each space's own floor and ceiling (split levels, sloped ceilings;
  // reusex-546).
  try {
    geometry::measure_space_heights(sp.spaces, r.grids, storeys, *cloud,
                                    *normals, so.spaces);
  } catch (const std::invalid_argument &e) {
    return StageResult::failure(e.what());
  }
  info("spaces: {} matched a stored space ({} locked, kept as stored), {} "
       "new GUID(s)",
       ids.matched, ids.locked_matched, ids.minted);
  int lost = 0;
  double arrangement = 0.0, total = 0.0;
  for (const auto &st : sp.stats)
    lost += st.lost_rooms;
  for (const auto &s : sp.spaces) {
    total += s.area;
    arrangement += std::max(s.confidence, 0.0) * s.area;
  }
  if (sp.spaces.empty())
    warn("spaces: no space from {} room(s) on {} storey(s) ({} wall(s), {} "
         "lost to other rooms); is the rooms cloud empty?",
         r.rooms, r.grids.size(), walls.size(), lost);

  const size_t written = db.save_spaces(sp.spaces, sp.boundaries, log_id);
  const auto rows = db.spaces();
  const auto locked_kept = static_cast<size_t>(std::count_if(
      rows.begin(), rows.end(), [](const auto &row) { return row.locked; }));
  std::map<std::string, int> kinds;
  for (const auto &s : sp.spaces)
    ++kinds[s.kind];
  return StageResult::success(
      fmt::format("{} space(s) from {} room(s) on {} storey(s): {} room(s), "
                  "{} corridor(s), {} open plan; {:.1f} m2, {:.0f} % on wall "
                  "faces; {} boundar(y/ies); {} room(s) lost; {} row(s) "
                  "written, {} locked row(s) kept",
                  sp.spaces.size(), r.rooms, r.grids.size(), kinds["room"],
                  kinds["corridor"], kinds["open_plan"], total,
                  total > 0.0 ? 100.0 * arrangement / total : 0.0,
                  sp.boundaries.size(), lost, written, locked_kept),
      {{"table", "spaces", static_cast<int64_t>(rows.size())},
       {"table", "space_boundaries",
        static_cast<int64_t>(db.space_boundaries().size())}});
}

json profile_json(const geometry::ExtrusionProfile &p) {
  return {{"outer", p.outer},
          {"holes", p.holes},
          {"depth", p.depth},
          {"placement", p.placement}};
}

/// Concatenate @p elements into one mesh, and the per-element mapping (face
/// and vertex ranges, type, source GUID, the IFC-ready profile) into
/// @p index.
pcl::PolygonMesh
concat_solids(const std::vector<geometry::ElementSolid> &elements,
              json &index) {
  pcl::PointCloud<pcl::PointXYZ> all;
  pcl::PolygonMesh out;
  index = json::array();
  for (const auto &e : elements) {
    pcl::PointCloud<pcl::PointXYZ> pts;
    pcl::fromPCLPointCloud2(e.mesh.cloud, pts);
    const auto v0 = static_cast<std::uint32_t>(all.size());
    const auto f0 = out.polygons.size();
    all += pts;
    for (const auto &f : e.mesh.polygons) {
      pcl::Vertices g;
      for (const auto v : f.vertices)
        g.vertices.push_back(v + v0);
      out.polygons.push_back(std::move(g));
    }
    json voids = json::array();
    for (const auto &v : e.voids)
      voids.push_back({{"guid", v.guid},
                       {"kind", v.kind},
                       {"profile", profile_json(v.profile)}});
    index.push_back({{"guid", e.source_guid},
                     {"type", geometry::to_string(e.type)},
                     {"storey", e.storey},
                     {"predefined", e.predefined},
                     {"part", e.part},
                     {"closed", e.closed},
                     {"volume", e.volume},
                     {"faces", {f0, e.mesh.polygons.size()}},
                     {"vertices", {v0, pts.size()}},
                     {"profile", profile_json(e.profile)},
                     {"voids", std::move(voids)},
                     {"voids_failed", e.voids_failed}});
  }
  pcl::toPCLPointCloud2(all, out.cloud);
  return out;
}

/// `solids` (W6.1, reusex-z1v.8.1): the typed model (storeys, walls,
/// openings, spaces) as one closed solid per element, stored as the meshes
/// solid_walls / solid_slabs / solid_spaces. Each mesh's `parameters` holds
/// the stage parameters and the element index (concat_solids), so a reader
/// can split the mesh back into its elements without a schema change.
StageResult run_solids(ProjectDB &db, const json &params) {
  SolidStageOptions so;
  try {
    so = solid_options_from_parameters(params.dump());
  } catch (const std::invalid_argument &e) {
    return StageResult::invalid(e.what());
  }
  const core::SolidModel model = io::load_solid_model(db);
  if (model.storeys.empty())
    return StageResult::invalid(
        "solids: the project has no storeys; run `rux create storeys`");
  if (model.walls.empty())
    return StageResult::invalid(
        "solids: the project has no walls; run `rux create walls`");
  if (model.spaces.empty())
    warn("solids: no spaces table (run `rux create spaces` first): no space "
         "volumes, and each storey's slab is the hull of its walls");

  const auto r = geometry::build_solids(model, so.solids);
  if (r.openings_failed > 0)
    warn("solids: {} of {} opening(s) could not be subtracted from their wall "
         "(the corefinement failed); those walls are left uncut there",
         r.openings_failed, model.openings.size());
  if (r.openings_skipped > 0)
    warn("solids: {} of {} opening(s) lie outside their wall's box or name a "
         "wall the walls table does not hold; not cut",
         r.openings_skipped, model.openings.size());
  if (r.not_closed > 0)
    warn("solids: {} solid(s) are not closed (a degenerate or self-touching "
         "polygon); they are stored as they are",
         r.not_closed);
  if (r.walls_skipped > 0 || r.spaces_skipped > 0)
    warn("solids: {} wall(s) and {} space(s) skipped (zero length, area or "
         "height)",
         r.walls_skipped, r.spaces_skipped);

  std::vector<StageArtifact> outputs;
  const std::string stage_params = solid_parameters_json(so);
  const auto store = [&](std::string_view name,
                         const std::vector<geometry::ElementSolid> &elements,
                         std::string_view type) {
    if (elements.empty()) {
      if (db.delete_mesh(name))
        warn("solids: no {} solid; the stale '{}' mesh was removed", type,
             name);
      else
        warn("solids: no {} solid; no '{}' mesh written", type, name);
      return;
    }
    json index;
    const pcl::PolygonMesh mesh = concat_solids(elements, index);
    const json meta = {{"solids_version", 1},
                       {"type", type},
                       {"parameters", json::parse(stage_params)},
                       {"elements", std::move(index)}};
    db.save_mesh(name, mesh, "solids", meta.dump());
    outputs.push_back({"mesh", std::string(name),
                       static_cast<int64_t>(mesh.polygons.size())});
  };
  store(kSolidWallsMesh, r.walls, "wall");
  store(kSolidSlabsMesh, r.slabs, "slab");
  store(kSolidSpacesMesh, r.spaces, "space");
  if (outputs.empty())
    return StageResult::failure(fmt::format(
        "solids: no solid from {} wall(s), {} space(s) on {} storey(s)",
        model.walls.size(), model.spaces.size(), model.storeys.size()));

  double wall_volume = 0.0;
  for (const auto &w : r.walls)
    wall_volume += w.volume;
  return StageResult::success(
      fmt::format("{} wall solid(s) ({:.1f} m3; {} opening(s) cut, {} "
                  "failed, {} skipped), {} slab(s), {} space volume(s); {} "
                  "not closed",
                  r.walls.size(), wall_volume, r.openings_cut,
                  r.openings_failed, r.openings_skipped, r.slabs.size(),
                  r.spaces.size(), r.not_closed),
      std::move(outputs));
}

StageResult dispatch(ProjectDB &db, const StageContext &ctx, const json &params,
                     int log_id) {
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
  case JobStage::storeys:
    return run_storeys(db, params, log_id);
  case JobStage::classify:
    return run_classify(db, params, log_id);
  case JobStage::walls:
    return run_walls(db, params, log_id);
  case JobStage::openings:
    return run_openings(db, params, log_id);
  case JobStage::spaces:
    return run_spaces(db, ctx, params, log_id);
  case JobStage::solids:
    return run_solids(db, params);
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
  auto &st = options.structural;
  st.enabled = param_or(params, "structural_filter", st.enabled);
  st.min_wall_area_m2 =
      param_or(params, "structural_min_wall_area", st.min_wall_area_m2);
  st.collinear_offset_m =
      param_or(params, "structural_collinear_offset", st.collinear_offset_m);
  st.min_storey_clearance_m = param_or(params, "structural_storey_clearance",
                                       st.min_storey_clearance_m);
  st.min_footprint_overlap = param_or(params, "structural_footprint_overlap",
                                      st.min_footprint_overlap);
  out.use_plane_classes =
      param_or(params, "plane_classes", out.use_plane_classes);
  out.use_storey_sections =
      param_or(params, "storey_sections", out.use_storey_sections);
  out.use_space_prior = param_or(params, "space_prior", out.use_space_prior);
  options.space_prior_weight =
      param_or(params, "space_prior_weight", options.space_prior_weight);
  if (!(options.space_prior_weight >= 0.0 && options.space_prior_weight < 1.0))
    throw std::invalid_argument(fmt::format(
        "space_prior_weight {} is outside [0, 1)", options.space_prior_weight));
  options.storey_local = param_or(params, "storey_local", options.storey_local);
  const auto solver_str = param_or<std::string>(params, "solver", "auto");
  try {
    options.solver = geometry::parse_solver_choice(solver_str);
  } catch (const std::exception &e) {
    throw std::invalid_argument(
        fmt::format("invalid solver '{}': {}", solver_str, e.what()));
  }
  options.cuopt_max_variables =
      param_or(params, "cuopt_max_variables", options.cuopt_max_variables);
  return out;
}

std::vector<core::PlaneRole>
mesh_plane_roles(const ProjectDB &db, const std::vector<IndicesPtr> &inliers) {
  if (!db.has_plane_classes("planes"))
    return {};
  const auto rows = db.plane_classes("planes");
  std::vector<core::PlaneRole> roles(inliers.size(), core::PlaneRole::unknown);
  for (const auto &row : rows) {
    const auto n = inliers.size();
    if (row.plane_label < 1 || static_cast<size_t>(row.plane_label) > n) {
      warn("mesh: plane_classes has a row for plane {} but 'planes' has {} "
           "plane(s); the classification is stale, so every plane is used. "
           "Re-run `rux create classify`",
           row.plane_label, n);
      return {};
    }
    const auto idx = static_cast<size_t>(row.plane_label - 1);
    const auto have = inliers[idx] ? inliers[idx]->size() : 0;
    if (static_cast<size_t>(row.point_count) != have) {
      warn("mesh: plane_classes says plane {} has {} point(s) but 'planes' "
           "gives it {}; the classification is stale, so every plane is "
           "used. Re-run `rux create classify`",
           row.plane_label, row.point_count, have);
      return {};
    }
    roles[idx] = core::plane_role_from_string(row.role).value_or(
        core::PlaneRole::unknown);
  }
  return roles;
}

std::vector<double> mesh_section_heights(const ProjectDB &db) {
  std::vector<double> heights;
  if (!db.has_storeys())
    return heights;
  for (const auto &s : db.storeys()) {
    const auto frame = geometry::LevelFrame::from_record(s);
    heights.push_back(frame.world_z_at(s.z_floor));
    heights.push_back(frame.world_z_at(s.z_ceiling));
  }
  return heights;
}

std::vector<geometry::WallEvidence> mesh_wall_evidence(const ProjectDB &db) {
  std::vector<geometry::WallEvidence> out;
  const auto walls = db.walls();
  if (walls.empty())
    return out;
  std::map<int, geometry::LevelFrame> frames;
  for (const auto &s : db.storeys())
    frames.emplace(s.idx, geometry::LevelFrame::from_record(s));
  size_t orphans = 0;
  for (const auto &w : walls) {
    const auto it = frames.find(w.storey);
    if (it == frames.end()) {
      ++orphans;
      continue;
    }
    const auto &f = it->second;
    const Eigen::Vector3d a = f.to_world({w.x0, w.y0, w.z_base});
    const Eigen::Vector3d b = f.to_world({w.x1, w.y1, w.z_base});
    geometry::WallEvidence e;
    e.a = a.head<2>();
    e.b = b.head<2>();
    e.z_lo = std::min(a.z(), b.z());
    e.z_hi = std::max(a.z(), b.z()) + w.height;
    e.thickness = w.thickness;
    out.push_back(e);
  }
  if (orphans > 0)
    warn("mesh: {} of {} wall(s) stand on a storey the storeys table does not "
         "have; left out of the wall evidence. Re-run `rux create walls`",
         orphans, walls.size());
  return out;
}

std::vector<geometry::SpaceFootprint>
mesh_space_footprints(const ProjectDB &db) {
  std::vector<geometry::SpaceFootprint> out;
  if (!db.has_spaces())
    return out;
  const auto spaces = db.spaces();
  std::map<int, core::StoreyRecord> storeys;
  for (const auto &s : db.storeys())
    storeys.emplace(s.idx, s);
  size_t orphans = 0, unlabelled = 0;
  for (const auto &sp : spaces) {
    if (sp.room_label <= 0) {
      ++unlabelled;
      continue;
    }
    const auto it = storeys.find(sp.storey);
    if (it == storeys.end()) {
      ++orphans;
      continue;
    }
    geometry::SpaceFootprint f;
    f.room_label = static_cast<unsigned int>(sp.room_label);
    for (const auto &p : sp.polygon.outer)
      f.outer.emplace_back(p[0], p[1]);
    for (const auto &h : sp.polygon.holes) {
      auto &ring = f.holes.emplace_back();
      for (const auto &p : h)
        ring.emplace_back(p[0], p[1]);
    }
    f.world_to_level =
        geometry::LevelFrame::from_record(it->second).world_to_level;
    f.z_floor = it->second.z_floor;
    f.z_ceiling = it->second.z_ceiling;
    out.push_back(std::move(f));
  }
  if (orphans + unlabelled > 0)
    warn("mesh: {} of {} space(s) left out of the space prior ({} on a "
         "storey the storeys table does not have, {} without a room "
         "label). Re-run `rux create spaces`",
         orphans + unlabelled, spaces.size(), orphans, unlabelled);
  return out;
}

geometry::MeshOptions mesh_options_for(const ProjectDB &db,
                                       const MeshStageOptions &stage,
                                       const std::vector<IndicesPtr> &inliers) {
  geometry::MeshOptions options = stage.options;
  if (stage.use_plane_classes)
    options.plane_roles = mesh_plane_roles(db, inliers);
  if (stage.use_storey_sections) {
    options.section_heights = mesh_section_heights(db);
    if (!options.section_heights.empty())
      info("mesh: section planes from the storeys table, storey floor/ceiling "
           "heights [{:.2f}] m",
           fmt::join(options.section_heights, ", "));
    if (!options.section_heights.empty()) {
      options.walls = mesh_wall_evidence(db);
      if (!options.walls.empty())
        info("mesh: {} wall(s) from the walls table back short wall lines",
             options.walls.size());
    }
  }
  if (stage.use_space_prior) {
    options.spaces = mesh_space_footprints(db);
    if (!options.spaces.empty())
      info("mesh: {} space(s) from the spaces table as a room prior",
           options.spaces.size());
  }
  return options;
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

StageResult run_logged_stage(ProjectDB &db, const StageContext &ctx,
                             std::string_view name, std::string_view log_name,
                             const std::function<StageResult()> &body) {
  return run_logged_stage(db, ctx, name, log_name,
                          [&body](int /*log_id*/) { return body(); });
}

StageResult run_logged_stage(ProjectDB &db, const StageContext &ctx,
                             std::string_view name, std::string_view log_name,
                             const std::function<StageResult(int)> &body) {
  int log_id = -1;

  // Parameters are parsed before anything is recorded: a blob that is not even
  // JSON describes no attempt worth logging, and it is the caller's mistake
  // rather than the stage's failure.
  json params;
  try {
    params = parse_parameters(ctx.parameters);
  } catch (const std::exception &e) {
    error("stage '{}' refused: invalid parameters: {}", name, e.what());
    return StageResult::invalid(
        fmt::format("invalid parameters: {}", e.what()));
  }

  try {
    // Open the log row BEFORE running `body`, so a run that is refused (for
    // an unsatisfied contract, bad parameters the stage itself rejects, or
    // anything else `body` decides) still leaves a durable record of the
    // attempt and its reason. `pipeline_log` is what the GUI's history view
    // reads (`ruxd`'s docs/gui/openapi.yaml, in its own repo), and a
    // rejection that vanishes without trace is exactly the silent failure
    // STANDARDS §5 forbids.
    //
    // The job id is folded into the logged parameters (rather than needing a
    // schema migration) so the durable history can be joined back to the job
    // that caused it — the recovery path the contract documents for a client
    // that reconnects after a restart.
    log_id =
        db.log_pipeline_start(log_name, logged_parameters(params, ctx.job_id));

    core::stopwatch watch;
    StageResult result = body(log_id);
    const double elapsed = watch.elapsed();

    if (result.cancelled) {
      warn("stage '{}' cancelled after {:.2f}s: {}", name, elapsed,
           result.message);
      db.log_pipeline_end(log_id, ProjectDB::PipelineOutcome::cancelled,
                          result.message);
    } else if (result.ok) {
      info("stage '{}' finished in {:.2f}s: {}", name, elapsed, result.message);
      db.log_pipeline_end(log_id, true);
    } else if (result.invalid_input) {
      error("stage '{}' refused: {}", name, result.message);
      db.log_pipeline_end(log_id, false, result.message);
    } else {
      error("stage '{}' failed after {:.2f}s: {}", name, elapsed,
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
      warn("stage '{}' cancelled: {}", name, e.what());
    else
      error("stage '{}' failed: {}", name, e.what());

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

StageResult run_stage(ProjectDB &db, const StageContext &ctx) {
  const auto &desc = descriptor(ctx.stage);
  return run_logged_stage(
      db, ctx, desc.name, desc.log_name, [&](int log_id) -> StageResult {
        // body() only runs once run_logged_stage has already parsed
        // ctx.parameters successfully (that parse is what gated opening the log
        // row), so this cannot throw; it is a second parse, not re-validation,
        // done here rather than threading nlohmann::json through
        // run_logged_stage's public signature (STANDARDS §2:
        // pipeline/stages.hpp stays free of the nlohmann dependency).
        const json params = parse_parameters(ctx.parameters);
        if (auto problem = check_inputs(db, desc.contract,
                                        contract_overrides(ctx.stage, params)))
          return StageResult::invalid(*problem);
        return dispatch(db, ctx, params, log_id);
      });
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
