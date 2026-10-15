// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "utils/parse_number.hpp"
#include <reusex/pipeline/stage_parameters.hpp>

#include <reusex/core/storey_identity.hpp>
#include <reusex/reconstruction/mesh.hpp>
#include <reusex/segmentation/classify_planes.hpp>
#include <reusex/segmentation/level.hpp>
#include <reusex/segmentation/reconstruct.hpp>
#include <reusex/segmentation/segment_instances.hpp>
#include <reusex/segmentation/segment_planes.hpp>
#include <reusex/segmentation/segment_rooms.hpp>

#include <fmt/format.h>

#include <stdexcept>

namespace reusex::pipeline {
namespace {

/// Widen a `float` option-struct field to the `double` the wire carries.
///
/// A plain `static_cast<double>(0.05F)` is 0.05000000074505806, which is what
/// a JSON encoder would faithfully print into a form field. Going through the
/// shortest round-trip decimal representation of the *float* gives back the
/// 0.05 the struct was written with, without inventing precision.
double widen(float value) {
  // fmt always writes '.', so the parse must not follow LC_NUMERIC either.
  return reusex::utils::to_double(fmt::format("{}", value));
}

/// Every default below comes from a default-constructed option struct, so the
/// literals live in exactly one place — the struct — and this table cannot
/// drift away from what `run_stage()` actually falls back to.
/// @param default_value  taken as `float` because every fractional option in
///        the four structs is a `float`; passing a double here would quietly
///        admit a default this table cannot round-trip.
ParameterDescriptor num(std::string key, std::string label,
                        std::string description, float default_value,
                        std::optional<double> minimum = std::nullopt,
                        std::optional<double> maximum = std::nullopt) {
  return ParameterDescriptor{std::move(key),
                             ParameterType::number,
                             std::move(label),
                             std::move(description),
                             widen(default_value),
                             minimum,
                             maximum,
                             false};
}

/// A fractional option whose struct field is a `double` (LevelOptions):
/// no float widening to undo, the default is carried as is.
ParameterDescriptor dnum(std::string key, std::string label,
                         std::string description, double default_value,
                         std::optional<double> minimum = std::nullopt,
                         std::optional<double> maximum = std::nullopt) {
  return ParameterDescriptor{std::move(key),   ParameterType::number,
                             std::move(label), std::move(description),
                             default_value,    minimum,
                             maximum,          false};
}

ParameterDescriptor integer(std::string key, std::string label,
                            std::string description, long long default_value,
                            std::optional<double> minimum = std::nullopt,
                            std::optional<double> maximum = std::nullopt) {
  return ParameterDescriptor{std::move(key),   ParameterType::integer,
                             std::move(label), std::move(description),
                             default_value,    minimum,
                             maximum,          false};
}

ParameterDescriptor boolean(std::string key, std::string label,
                            std::string description, bool default_value) {
  return ParameterDescriptor{std::move(key),   ParameterType::boolean,
                             std::move(label), std::move(description),
                             default_value,    std::nullopt,
                             std::nullopt,     false};
}

ParameterDescriptor text(std::string key, std::string label,
                         std::string description, std::string default_value) {
  return ParameterDescriptor{std::move(key),
                             ParameterType::string,
                             std::move(label),
                             std::move(description),
                             std::move(default_value),
                             std::nullopt,
                             std::nullopt,
                             false};
}

/// A parameter with no neutral value: absent means "do not restrict".
ParameterDescriptor optional_of(ParameterType type, std::string key,
                                std::string label, std::string description) {
  return ParameterDescriptor{
      std::move(key),   type,         std::move(label), std::move(description),
      std::monostate{}, std::nullopt, std::nullopt,     false};
}

/// A parameter with no default that a job must give (non-empty for a list).
ParameterDescriptor required_of(ParameterType type, std::string key,
                                std::string label, std::string description) {
  ParameterDescriptor d = optional_of(type, std::move(key), std::move(label),
                                      std::move(description));
  d.required = true;
  return d;
}

const std::vector<ParameterDescriptor> &clouds_parameters() {
  static const std::vector<ParameterDescriptor> table = [] {
    const geometry::ReconstructionParams d{};
    return std::vector<ParameterDescriptor>{
        num("resolution", "Voxel size [m]",
            "Voxel grid resolution used to downsample the fused cloud.",
            d.resolution),
        num("min_distance", "Minimum depth [m]",
            "Depth samples closer than this are discarded.", d.min_distance),
        num("max_distance", "Maximum depth [m]",
            "Depth samples farther than this are discarded.", d.max_distance),
        integer("sampling_factor", "Pixel subsampling",
                "Keep every Nth pixel of each depth frame.", d.sampling_factor),
        integer("confidence_threshold", "Minimum confidence",
                "Discard depth samples below this confidence level.",
                d.confidence_threshold),
    };
  }();
  return table;
}

const std::vector<ParameterDescriptor> &planes_parameters() {
  static const std::vector<ParameterDescriptor> table = [] {
    const geometry::SegmentPlanesOptions d{};
    std::vector<ParameterDescriptor> table{
        num("angle_threshold", "Angle threshold [deg]",
            "Maximum normal deviation for a point to join a plane.",
            d.angle_threshold, 0.0, 365.0),
        num("plane_dist_threshold", "Distance threshold [m]",
            "Maximum point-to-plane distance. Sending this key at all pins the "
            "threshold and switches off adaptive derivation for it.",
            d.plane_dist_threshold, 0.0, 1.0),
        integer("min_inliers", "Minimum cluster size",
                "Smallest accepted plane, in points. Sending this key at all "
                "pins it and switches off adaptive derivation for it.",
                d.min_inliers, 3.0, 1000000.0),
        num("radius", "Region-growing radius [m]",
            "Neighbourhood radius used while growing a plane.", d.radius, 0.0,
            5.0),
        num("interval_0", "Initial refit interval",
            "Initial interval between "
            "plane refits.",
            d.interval_0, 1.0, 10000.0),
        num("interval_factor", "Refit interval factor",
            "Multiplier applied to the refit interval after each refit.",
            d.interval_factor, 1.0, 10.0),
        boolean("adaptive", "Adaptive thresholds",
                "Derive the distance threshold (~3 sigma) and the minimum "
                "cluster size from measured cloud noise. Explicit values still "
                "win per parameter.",
                d.adaptive),
        integer("noise_seed", "Noise-estimator seed",
                "Deterministic seed for the noise estimator (STANDARDS §6).",
                static_cast<long long>(d.noise_seed), 0.0),
        optional_of(ParameterType::string, "filter", "Point filter",
                    "Filter expression restricting which points are "
                    "segmented. Empty means the whole cloud."),
    };
    // #214: these two are pinned by presence, not by value.
    for (auto &parameter : table)
      if (parameter.key == "plane_dist_threshold" ||
          parameter.key == "min_inliers")
        parameter.presence_sensitive = true;
    return table;
  }();
  return table;
}

const std::vector<ParameterDescriptor> &rooms_parameters() {
  static const std::vector<ParameterDescriptor> table = [] {
    const geometry::SegmentRoomsOptions d{};
    return std::vector<ParameterDescriptor>{
        num("grid_size", "Grid size [m]",
            "Spatial discretisation used to build the plane graph.",
            d.grid_size, 0.01, 10.0),
        num("resolution", "Leiden resolution",
            "Higher values produce more, smaller rooms.", d.resolution, 0.0,
            10.0),
        num("beta", "Leiden beta",
            "Randomness of the refinement phase; lower is more deterministic.",
            d.beta, 0.0, 1.0),
        integer("max_iter", "Maximum iterations",
                "Finite bound on the Leiden iteration count.", d.max_iter, -1.0,
                1000.0),
        integer("propagate_k", "Propagation neighbours",
                "Neighbours polled per point when propagating room labels off "
                "the sampled subset.",
                d.propagate_k, 1.0),
        num("propagate_max_radius", "Propagation radius [m]",
            "Points with no room label inside this radius stay unlabeled.",
            d.propagate_max_radius, 0.0, 10.0),
        num("min_room_area", "Minimum room area [m^2]",
            "Plane surface a room's sample points must cover; smaller "
            "communities join the room they see most. 0 keeps all.",
            d.min_room_area, 0.0, 1000.0),
        optional_of(ParameterType::string, "filter", "Point filter",
                    "Filter expression restricting which points are "
                    "partitioned. Empty means the whole cloud."),
    };
  }();
  return table;
}

const std::vector<ParameterDescriptor> &instances_parameters() {
  static const std::vector<ParameterDescriptor> table = [] {
    const geometry::SegmentInstancesRequest d{};
    return std::vector<ParameterDescriptor>{
        text("semantic_cloud", "Semantic cloud",
             "Name of the semantic-label cloud to split into instances.",
             std::string(kDefaultSemanticCloud)),
        text("output_cloud", "Output cloud",
             "Name of the instance-label cloud to write.",
             std::string(kDefaultInstanceCloud)),
        num("cluster_tolerance", "Cluster tolerance [m]",
            "Euclidean distance threshold separating two instances.",
            d.cluster_tolerance, 0.01, 5.0),
        integer("min_cluster_size", "Minimum instance size",
                "Smallest accepted instance, in points.", d.min_cluster_size,
                1.0, 100000.0),
        integer("max_cluster_size", "Maximum instance size",
                "Largest accepted instance, in points.", d.max_cluster_size,
                10.0, 10000000.0),
        optional_of(ParameterType::integer_list, "labels", "Semantic labels",
                    "Restrict clustering to these semantic point labels "
                    "(point label = API class + 1, so the first annotate "
                    "class is 1). Empty means every label above 0."),
    };
  }();
  return table;
}

const std::vector<ParameterDescriptor> &mesh_parameters() {
  static const std::vector<ParameterDescriptor> table = [] {
    const geometry::MeshOptions d{};
    return std::vector<ParameterDescriptor>{
        text("solver", "MIP solver",
             "MIP backend: 'auto' uses the primary solver (cuOpt on GPU if "
             "available) and falls back to HiGHS CPU on error or OOM; "
             "'highs' forces the CPU solver; 'cuopt' forces the GPU solver. "
             "This job cannot be cancelled once the solver is running — set "
             "time_limit_seconds to bound wall-clock time on large scenes.",
             "auto"),
        num("time_limit_seconds", "Time limit [s]",
            "MIP solver wall-clock time limit. The solver returns its best "
            "solution found so far when this limit expires, guarding against "
            "indefinite hangs on complex buildings.",
            static_cast<float>(d.time_limit_seconds), 1.0, 36000.0),
        text("output_name", "Output mesh name",
             "Name the mesh is saved under in the ProjectDB.", "mesh"),
        num("search_threshold", "Search threshold [m]",
            "Maximum distance for pairing nearby planes.", d.search_threshold,
            0.01, 10.0),
        num("new_plane_offset", "New-plane offset [m]",
            "Offset distance used when synthesising a closing plane.",
            d.new_plane_offset, 0.01, 1.0),
        num("alpha", "Wall-complexity weight",
            "Weight on the wall/complexity term of the MIP objective.",
            static_cast<float>(d.alpha), 0.0, 100.0),
        integer("max_cells", "Max cells",
                "Cell-complex cell count above which the stage fails fast "
                "with a diagnostic instead of attempting an intractable MIP.",
                static_cast<long long>(d.max_cells), 1.0, 1000000.0),
        boolean("sectioned", "Sectioned solve",
                "Solve the MIP per horizontal section (storey) when the "
                "arrangement is large, keeping multi-room buildings tractable.",
                d.sectioned),
        integer("sectioned_threshold", "Sectioned threshold",
                "Engage the sectioned solve only when the cell complex has "
                "at least this many cells.",
                static_cast<long long>(d.sectioned_threshold), 1.0, 1000000.0),
        optional_of(ParameterType::string, "filter", "Point filter",
                    "Filter expression restricting which points are meshed. "
                    "Empty means the whole cloud."),
    };
  }();
  return table;
}

const std::vector<ParameterDescriptor> &optimize_parameters() {
  // Defaults mirror SubcommandOptimizeOptions in apps/rux/include/optimize.hpp
  // (STANDARDS §4). Only the knobs most likely to be changed from the GUI are
  // exposed here; the full CLI surface is available via `rux optimize --help`.
  // Deliberately not including <reusex/slam/PlaneGraphOptimizer.hpp>: the slam
  // module links GTSAM which must not enter reusex_pipeline (see #464 layering
  // note in stages.cpp). Default values are hard-coded to match the header.
  static const std::vector<ParameterDescriptor> table{
      integer("min_observations", "Min landmark observations",
              "A plane landmark must be seen by at least this many sensor "
              "frames to enter the factor graph.",
              4, 1.0, 100.0),
      integer("assoc_rounds", "Association rounds",
              "Number of associate / optimize / refit iterations (1 = "
              "one-shot). More rounds let landmarks re-form on improved poses.",
              2, 1.0, 10.0),
      boolean("no_gnc", "Disable GNC",
              "Use plain Levenberg-Marquardt without Graduated Non-Convexity. "
              "GNC down-weights outlier plane associations; disabling it is "
              "faster but less robust.",
              false),
      boolean("dry_run", "Dry run",
              "Compute and report statistics without writing optimised poses "
              "back to the project.",
              false),
  };
  return table;
}

const std::vector<ParameterDescriptor> &annotate_parameters() {
  // No option-struct home: vision::AnnotationConfig (vision/annotate.hpp)
  // owns prompts/confidence/skip_annotated, but reusex_pipeline must not link
  // reusex_vision (STANDARDS §1) — the same reasoning dispatch() in
  // stages.cpp gives for refusing to run the stage by default. So, like
  // optimize_parameters() above for reusex_slam, the defaults are hard-coded
  // here rather than read from the struct; keep them in sync with
  // AnnotationConfig by hand.
  //
  // "frames" and "placement" are not AnnotationConfig fields at all — they
  // are job-routing parameters a caller's own executor reads before handing
  // the rest to vision::annotate() for the frames it selected (the gpu-worker
  // design, docs/design/gpu-worker.md §3, ruxd repo).
  static const std::vector<ParameterDescriptor> table{
      required_of(ParameterType::string_list, "prompts", "Prompts",
                  "The prompts to segment — text concept/class names, e.g. "
                  "[\"wall\",\"window\"]. Required and non-empty: SAM3 has "
                  "no default class set to fall back on."),
      optional_of(ParameterType::integer_list, "frames", "Frames",
                  "Sensor frame ids to annotate. A {\"from\":n,\"to\":m} "
                  "range object is also accepted in place of an explicit "
                  "id list. Absent means every stored frame."),
      num("confidence", "Confidence threshold",
          "Minimum detection confidence kept in the output label map.", 0.5F,
          0.0, 1.0),
      boolean("skip_annotated", "Skip already-annotated frames",
              "Leave frames that already have a stored segmentation image "
              "untouched instead of re-running inference on them.",
              false),
      text("placement", "Placement",
           "Where inference runs: 'auto' uses a capable worker when one is "
           "online and falls back to the local backend otherwise, 'worker' "
           "requires a worker and fails without one, 'local' always runs "
           "in-process.",
           "auto"),
  };
  return table;
}

const std::vector<ParameterDescriptor> &graph_discover_parameters() {
  // Defaults mirror slam::GraphDiscoverOptions (slam/graph_discover.hpp).
  // reusex_pipeline must not link reusex_slam (#464), so they are hard-coded
  // here and pinned by tests/unit/slam/test_discover_parameters.cpp.
  static const std::vector<ParameterDescriptor> table{
      boolean("loop_closure", "ORB loop closure",
              "Match temporally distant frame pairs (ORB features + depth, "
              "RANSAC) and propose loop edges.",
              true),
      boolean("use_panoramas", "Panorama loop edges",
              "Derive loop edges from the project's 360 panoramas (#236).",
              false),
      text("loop_edges_file", "External edge file",
           "Path to a reusex.loop_edges.v1 JSON file from an out-of-process "
           "matcher. Empty means none.",
           ""),
      text("loop_proposal", "Candidate proposal",
           "How candidate pairs are proposed: auto, appearance, spatial or "
           "exhaustive.",
           "auto"),
      integer("loop_min_frame_gap", "Min frame gap",
              "Skip pairs closer than this in frame index (odometry covers "
              "them).",
              50, 1.0, 100000.0),
      integer("loop_min_inliers", "Min RANSAC inliers",
              "Reject an ORB pair below this many RANSAC inliers.", 60, 3.0,
              100000.0),
      boolean("pcm", "Consistency filter (PCM)",
              "Keep only loop edges consistent with each other and with the "
              "stored active edges; confirmed edges are fixed inliers.",
              true),
      boolean("dry_run", "Dry run",
              "Report what would be added without writing edges.", false),
  };
  return table;
}

const std::vector<ParameterDescriptor> &storeys_parameters() {
  // Keys are the LevelOptions field names plus guid_match_m
  // (core::StoreyIdentityOptions::match_m); bounds mirror the CLI's
  // CLI::Range checks (apps/rux/src/create/storeys.cpp).
  static const std::vector<ParameterDescriptor> table = [] {
    const geometry::LevelOptions d{};
    return std::vector<ParameterDescriptor>{
        dnum("horizontal_cos", "Horizontal cosine",
             "A plane is horizontal when |n.z| exceeds this cosine.",
             d.horizontal_cos, 0.5, 1.0),
        dnum("surface_max_tilt_deg", "Surface maximum tilt [deg]",
             "A horizontal plane more than this off the measured vertical "
             "(a sloped ceiling, a ramp) is not a floor/ceiling candidate.",
             d.surface_max_tilt_deg, 0.1, 30.0),
        dnum("inlier_m", "Fit inlier band [m]",
             "Inlier band of the iterated floor/ceiling least-squares fit.",
             d.inlier_m, 0.001, 0.5),
        integer("max_fit_iterations", "Max fit iterations",
                "Refits per floor/ceiling surface (bounded).",
                d.max_fit_iterations, 1.0, 100.0),
        dnum("hist_bin_m", "Height bin [m]",
             "Horizontal planes within two bins of a stronger plane's "
             "height form one surface.",
             d.hist_bin_m, 0.005, 1.0),
        dnum("peak_min_share", "Surface minimum share",
             "A surface is a level when it holds at least this share of "
             "the strongest surface's points.",
             d.peak_min_share, 0.01, 1.0),
        dnum("slab_merge_m", "Slab merge [m]",
             "Surfaces closer than this along the vertical are one level "
             "(a slab: a ceiling below, a floor above).",
             d.slab_merge_m, 0.0, 3.0),
        dnum("min_storey_height_m", "Minimum storey height [m]",
             "Consecutive levels closer than this are not a storey.",
             d.min_storey_height_m, 0.5, 20.0),
        dnum("default_height_m", "Default storey height [m]",
             "Storey height used when no ceiling level exists.",
             d.default_height_m, 0.5, 20.0),
        boolean("use_normal_sign", "Use normal sign",
                "Tell floors (up-facing) from ceilings (down-facing) by the "
                "sensor-oriented plane normals when they carry that.",
                d.use_normal_sign),
        boolean("per_storey_frame", "Per-storey frame",
                "Give a storey whose vertical disagrees with the shared one "
                "its own level frame (drifted multi-storey scans).",
                d.per_storey_frame),
        dnum("storey_frame_tol_deg", "Storey frame tolerance [deg]",
             "A storey gets its own frame when its vertical is more than "
             "this off the shared one.",
             d.storey_frame_tol_deg, 0.0, 10.0),
        dnum("guid_match_m", "Identity match [m]",
             "A re-detected storey keeps the GUID of the stored storey whose "
             "floor is within this distance (in the new level frame).",
             core::StoreyIdentityOptions{}.match_m, 0.0, 5.0),
    };
  }();
  return table;
}

const std::vector<ParameterDescriptor> &classify_parameters() {
  // Keys are the ClassifyPlanesOptions field names plus role_map; bounds
  // mirror the CLI's CLI::Range checks (apps/rux/src/create/classify.cpp).
  // oriented_normals is not a parameter: the stage reads it from the clouds'
  // provenance (reusex-0se).
  static const std::vector<ParameterDescriptor> table = [] {
    const geometry::ClassifyPlanesOptions d{};
    return std::vector<ParameterDescriptor>{
        dnum("horizontal_cos", "Horizontal cosine",
             "A plane is horizontal when |n.up| exceeds this cosine.",
             d.horizontal_cos, 0.5, 1.0),
        dnum("vertical_cos", "Vertical cosine",
             "A plane is vertical when |n.up| is below this cosine; between "
             "the two it is slanted.",
             d.vertical_cos, 0.0, 0.5),
        dnum("min_large_area_m2", "Minimum floor/ceiling area [m^2]",
             "A floor or ceiling candidate covers at least this much area.",
             d.min_large_area_m2, 0.0, 1000.0),
        dnum("band_m", "Floor/ceiling band [m]",
             "Within this of the storey's floor / ceiling height.", d.band_m,
             0.0, 2.0),
        dnum("min_wall_height_m", "Minimum wall height [m]",
             "A wall's robust vertical extent is at least this ...",
             d.min_wall_height_m, 0.0, 20.0),
        dnum("min_wall_height_share", "Minimum wall height share",
             "... or this share of its storey's height, whichever is less.",
             d.min_wall_height_share, 0.0, 1.0),
        dnum("max_furniture_area_m2", "Maximum furniture area [m^2]",
             "A horizontal plane larger than this is structure, never "
             "furniture.",
             d.max_furniture_area_m2, 0.0, 1000.0),
        dnum("table_min_m", "Lowest table top [m]",
             "Table-top heights above the floor start here.", d.table_min_m,
             0.0, 3.0),
        dnum("table_max_m", "Highest table top [m]",
             "Table-top heights above the floor end here.", d.table_max_m, 0.0,
             3.0),
        dnum("min_storey_height_m", "Minimum storey height [m]",
             "Without storeys: the ceiling is at least this far above the "
             "floor.",
             d.min_storey_height_m, 0.5, 20.0),
        dnum("area_cell_m", "Area grid cell [m]",
             "Side of the occupancy grid measuring a plane's area; at or "
             "above the cloud's point spacing.",
             d.area_cell_m, 0.005, 1.0),
        dnum("extent_quantile", "Extent quantile",
             "Robust height range = these quantiles of the inlier heights.",
             d.extent_quantile, 0.0, 0.49),
        dnum("min_labelled_share", "Minimum labelled share",
             "The semantic class decides only if this share of the plane's "
             "inliers carries a label ...",
             d.min_labelled_share, 0.0, 1.0),
        dnum("min_class_share", "Minimum class share",
             "... and the majority class holds this share of those votes.",
             d.min_class_share, 0.0, 1.0),
        text("role_map", "Role map file",
             "Class -> role map JSON file (core/class_role_map.hpp); empty = "
             "the built-in map.",
             ""),
    };
  }();
  return table;
}

} // namespace

std::string_view to_string(ParameterType type) {
  switch (type) {
  case ParameterType::number:
    return "number";
  case ParameterType::integer:
    return "integer";
  case ParameterType::boolean:
    return "boolean";
  case ParameterType::string:
    return "string";
  case ParameterType::integer_list:
    return "integer_list";
  case ParameterType::string_list:
    return "string_list";
  }
  throw std::logic_error("unhandled ParameterType");
}

const std::vector<ParameterDescriptor> &stage_parameters(JobStage stage) {
  switch (stage) {
  case JobStage::clouds:
    return clouds_parameters();
  case JobStage::planes:
    return planes_parameters();
  case JobStage::rooms:
    return rooms_parameters();
  case JobStage::instances:
    return instances_parameters();
  case JobStage::mesh:
    return mesh_parameters();
  case JobStage::optimize:
    return optimize_parameters();
  case JobStage::annotate:
    return annotate_parameters();
  case JobStage::graph_discover:
    return graph_discover_parameters();
  case JobStage::storeys:
    return storeys_parameters();
  case JobStage::classify:
    return classify_parameters();
  }
  throw std::logic_error("unhandled JobStage in stage_parameters()");
}

} // namespace reusex::pipeline
