// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "optimize.hpp"
#include "create/stage_bridge.hpp"
#include "exit_status.hpp"
#include "stage_prerequisites.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/slam/PlaneGraphOptimizer.hpp>
#include <reusex/slam/optimize_parameters.hpp>

#include <fmt/format.h>
#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

const std::vector<std::pair<std::string, bool>> &optimize_moved_flags() {
  static const std::vector<std::pair<std::string, bool>> flags = {
      {"--loop-closure", false},
      {"--loop-proposal", true},
      {"--loop-min-frame-gap", true},
      {"--loop-max-distance", true},
      {"--loop-max-view-angle", true},
      {"--loop-max-candidates", true},
      {"--loop-min-inliers", true},
      {"--loop-max-features", true},
      {"--loop-ratio-test", true},
      {"--loop-ransac-inlier-dist", true},
      {"--loop-max-seed-disagreement", true},
      {"--loop-min-seed-disagreement", true},
      {"--loop-min-seed-disagreement-fraction", true},
      {"--loop-no-pcm", false},
      {"--use-panoramas", false},
      {"--pano-max-frames", true},
      {"--pano-min-inliers", true},
      {"--pano-max-edges", true},
      {"--pano-slices", true},
      {"--pano-max-features", true},
      {"--pano-max-distance", true},
      {"--loop-edges", true},
      {"--loop-edges-min-disagreement", true},
      {"--loop-edges-min-disagreement-fraction", true},
  };
  return flags;
}

std::string optimize_moved_flag_message(const std::string &flag) {
  // Discover runs ORB loop closure unless --no-loop-closure: it has no
  // --loop-closure flag to suggest.
  if (flag == "--loop-closure")
    return "'--loop-closure' moved to 'rux graph discover', where ORB loop "
           "closure is on by default: rux optimize now solves the stored pose "
           "graph. Run 'rux graph discover' first, then 'rux optimize'.";
  return fmt::format("'{}' moved to 'rux graph discover': rux optimize now "
                     "solves the stored pose graph. Run 'rux graph discover "
                     "{} ...' first, then 'rux optimize'.",
                     flag, flag);
}

void setup_subcommand_optimize(CLI::App &app,
                               std::shared_ptr<RuxOptions> global_opt) {
  auto opt = std::make_shared<SubcommandOptimizeOptions>();
  auto *sub = app.add_subcommand(
      "optimize",
      "Globally optimize per-frame poses via a plane-landmark pose graph");

  sub->footer(R"(
DESCRIPTION:
  Globally optimizes the stored per-frame sensor poses of a single scan using a
  plane-landmark pose graph (issue #225, P1). Unlike 'rux register' — which
  polishes poses locally with frame-to-frame point-to-plane residuals and
  provably saturates because pairwise energy cannot express global consistency —
  this back-end changes the objective to *shared global plane landmarks*:

    1. detect the dominant planes each frame sees (RANSAC on its surfels),
    2. associate those observations across frames into persistent
       OrientedPlane3 landmarks (normal-angle + offset gating, min observations),
    3. build a GTSAM factor graph (per-frame Pose3 + odometry BetweenFactors from
       the seed poses + one OrientedPlane3Factor per observation + a soft prior
       on the first pose for gauge), and
    4. solve it with Graduated Non-Convexity (GNC) so a handful of bad plane
       associations are down-weighted rather than corrupting the solution.

  The solve always starts from the initial poses (sensor_frames.
  initial_transform, snapshotted by 'rux graph init' or 'rux import rtabmap'),
  so re-running is repeatable. Odometry and loop edges come from the stored
  pose graph: 'rux graph discover' adds loop edges, 'rux graph prune' rejects
  bad ones. A project without odometry edges is initialised automatically.

  Plane factors are weighted by their inlier support (a wall fit from thousands
  of surfels pulls harder than a small patch); disable with
  --no-plane-inlier-weight. Wide-baseline loop edges are found by
  'rux graph discover'.

EXAMPLES:
  rux optimize                        # Defaults (GNC on, inlier-weighted planes)
  rux optimize --dry-run              # Report statistics without writing
  rux graph discover && rux optimize  # Add ORB loop edges, then solve
  rux graph discover --no-loop-closure --loop-edges edges.json \
    && rux optimize --loop-trust      # External learned-matcher edges
  rux optimize --min-observations 3 --assoc-distance 0.15
  rux optimize --no-gnc               # Plain Levenberg-Marquardt (no robustness)

WORKFLOW:
  1. rux import rtabmap scan.db       # Import sensor frames
  2. rux graph discover; rux optimize   # Loop edges, then global pose optimization
  3. rux create clouds                # Regenerate clouds with optimized poses

NOTES:
  - Requires at least 2 sensor frames with depth (run 'rux import' first)
  - Run 'rux create clouds' afterwards to rebuild the merged point cloud
  - When too few plane landmarks are found the poses are left unchanged
)");

  // --- Per-frame plane detection ---
  sub->add_option("--max-planes-per-frame", opt->max_planes_per_frame,
                  "Keep at most this many planes per frame")
      ->default_val(opt->max_planes_per_frame);
  sub->add_option("--min-plane-inliers", opt->min_plane_inliers,
                  "Reject planes with fewer supporting surfels")
      ->default_val(opt->min_plane_inliers);
  sub->add_option("--ransac-distance", opt->ransac_distance,
                  "Inlier point-to-plane distance (m)")
      ->default_val(opt->ransac_distance);
  sub->add_option("--ransac-normal-angle", opt->ransac_normal_angle,
                  "Inlier normal agreement threshold (deg)")
      ->default_val(opt->ransac_normal_angle);
  sub->add_option("--ransac-iterations", opt->ransac_iterations,
                  "RANSAC hypotheses per plane")
      ->default_val(opt->ransac_iterations);

  // --- Cross-frame association ---
  sub->add_option(
         "--assoc-normal-angle", opt->assoc_normal_angle,
         "Max normal angle to merge observations into a landmark (deg)")
      ->default_val(opt->assoc_normal_angle);
  sub->add_option("--assoc-distance", opt->assoc_distance,
                  "Max plane-offset difference to merge (m)")
      ->default_val(opt->assoc_distance);
  sub->add_option("--min-observations", opt->min_observations,
                  "Landmark must be seen by at least this many frames")
      ->default_val(opt->min_observations);
  sub->add_option("--assoc-overlap-margin", opt->assoc_overlap_margin,
                  "In-plane overlap slack for merging observations (m, <=0 "
                  "disables the overlap gate)")
      ->default_val(opt->assoc_overlap_margin);
  sub->add_option("--min-landmark-spread-ratio", opt->min_landmark_spread_ratio,
                  "Reject landmarks whose observation centroids are more "
                  "collinear than this SV ratio (<=0 disables)")
      ->default_val(opt->min_landmark_spread_ratio);

  // --- Alternating rounds ---
  sub->add_option("--assoc-rounds", opt->assoc_rounds,
                  "Associate/optimize/refit rounds (1 = one-shot)")
      ->default_val(opt->assoc_rounds);
  sub->add_option("--assoc-round-tol", opt->assoc_round_tol,
                  "Stop rounds when the round's max pose shift drops below (m)")
      ->default_val(opt->assoc_round_tol);

  // --- Factor-graph noise ---
  sub->add_option("--odometry-sigma-rot", opt->odometry_sigma_rot,
                  "Odometry rotation std (rad)")
      ->default_val(opt->odometry_sigma_rot);
  sub->add_option("--odometry-sigma-trans", opt->odometry_sigma_trans,
                  "Odometry translation std (m)")
      ->default_val(opt->odometry_sigma_trans);
  sub->add_option("--underconstrained-odom-scale",
                  opt->underconstrained_odom_scale,
                  "Odometry sigma multiplier for frames that span < 2 plane "
                  "normal directions (<1 tightens; 1 disables the guard)")
      ->default_val(opt->underconstrained_odom_scale);
  sub->add_option(
         "--odometry-noise", opt->odometry_noise,
         "Per-edge odometry noise model: fixed (default; every edge gets "
         "--odometry-sigma-*) or motion (sigma proportional to that edge's own "
         "seed motion, normalised by the run's median so the odometry chain's "
         "overall authority against the plane factors is unchanged). Note the "
         "measured result of issue #225: loosening odometry makes absolute GT "
         "accuracy monotonically WORSE on drifting captures, so 'motion' is a "
         "redistribution probe, not a drift fix")
      ->default_val(opt->odometry_noise)
      ->check(CLI::IsMember({"fixed", "motion"}));
  sub->add_option("--odometry-weight-min", opt->odometry_weight_min,
                  "Min per-edge odometry sigma scale (--odometry-noise motion)")
      ->default_val(opt->odometry_weight_min);
  sub->add_option("--odometry-weight-max", opt->odometry_weight_max,
                  "Max per-edge odometry sigma scale (--odometry-noise motion)")
      ->default_val(opt->odometry_weight_max);
  sub->add_flag("--odometry-robust", opt->odometry_robust,
                "Let GNC down-weight individual odometry factors instead of "
                "registering every one as a known inlier. Measured on the #225 "
                "ARKitScenes GT scans: it does not recover the regression, "
                "because smooth drift has no single wrong edge to demote");
  sub->add_option("--odometry-gnc-inlier-cost", opt->odometry_gnc_inlier_cost,
                  "GNC-TLS inlier threshold for odometry factors under "
                  "--odometry-robust (chi2(6, 0.99)/2 = 8.41)")
      ->default_val(opt->odometry_gnc_inlier_cost);
  sub->add_option("--plane-sigma-normal", opt->plane_sigma_normal,
                  "Plane-normal measurement std (rad)")
      ->default_val(opt->plane_sigma_normal);
  sub->add_option("--plane-sigma-distance", opt->plane_sigma_distance,
                  "Plane-distance measurement std (m)")
      ->default_val(opt->plane_sigma_distance);
  sub->add_option(
         "--plane-noise", opt->plane_noise,
         "Per-observation plane-factor noise model: inliers (default; one "
         "sqrt(median_N/N) scalar on both channels), fit (sigmas from each "
         "detection's residual RMS, support and in-plane extent, weighting the "
         "normal and distance channels separately), or uniform (no weighting). "
         "Use 'fit' on captures whose poses DRIFT — measured better than "
         "'inliers' on every metric of all three ARKitScenes GT scans, but "
         "worse on a non-drifting laser-GT scan, so it is not the default. "
         "Pair it with --plane-weight-min 0.10 --plane-weight-max 15. "
         "--plane-sigma-normal/-distance always describe a MEDIAN-quality "
         "observation, so switching models redistributes authority between "
         "observations without changing the plane term's overall weight")
      ->default_val(opt->plane_noise)
      ->check(CLI::IsMember({"uniform", "inliers", "fit"}));
  sub->add_flag("--no-plane-inlier-weight", opt->no_plane_inlier_weight,
                "Deprecated spelling of --plane-noise uniform (kept so the "
                "configurations recorded in issue #225 stay reproducible); "
                "overrides --plane-noise when given");
  sub->add_option("--plane-weight-min", opt->plane_weight_min,
                  "Min plane-factor sigma scale (most authority a single "
                  "observation may earn). Defaults are tuned for --plane-noise "
                  "fit; the legacy 'inliers' model was tuned at 0.5")
      ->default_val(opt->plane_weight_min);
  sub->add_option(
         "--plane-weight-max", opt->plane_weight_max,
         "Max plane-factor sigma scale (least authority). Defaults are "
         "tuned for --plane-noise fit; the legacy 'inliers' model was "
         "tuned at 3.0. If most observations hit the clamp, the clamp "
         "— not the fit quality — is setting the weights (warned about "
         "at run time); widen the range")
      ->default_val(opt->plane_weight_max);
  sub->add_option(
         "--plane-sigma-scale", opt->plane_sigma_scale,
         "Global multiplier on BOTH plane sigmas — the plane term's authority "
         "against the odometry chain. The plane term's weight in the objective "
         "goes as 1/scale^2, so >1 weakens it and <1 strengthens it; 1.0 is "
         "the shipped calibration (bit-identical to omitting the flag). Use it "
         "to sweep plane-term weight in one dimension (#225 §9)")
      ->default_val(opt->plane_sigma_scale)
      ->check(CLI::PositiveNumber);
  sub->add_flag("--no-plane-factors", opt->no_plane_factors,
                "Build the pose graph WITHOUT plane factors (the "
                "--plane-sigma-scale -> infinity limit, taken exactly): "
                "odometry + the frame-0 gauge prior only. Planes are still "
                "detected and associated for reporting. The solve then cannot "
                "move the seed trajectory, which makes this the 'plane term "
                "off' endpoint of a weight sweep");
  sub->add_option("--prior-sigma-rot", opt->prior_sigma_rot,
                  "First-pose gauge prior rotation std (rad)")
      ->default_val(opt->prior_sigma_rot);
  sub->add_option("--prior-sigma-trans", opt->prior_sigma_trans,
                  "First-pose gauge prior translation std (m)")
      ->default_val(opt->prior_sigma_trans);

  // --- Solver ---
  sub->add_flag("--no-gnc", opt->no_gnc,
                "Disable Graduated Non-Convexity (use plain LM)");
  sub->add_option("--gnc-inlier-cost", opt->gnc_inlier_cost,
                  "GNC TLS inlier threshold on factor error (0.5*chi^2)")
      ->default_val(opt->gnc_inlier_cost);
  sub->add_option("--iterations", opt->iterations,
                  "Levenberg-Marquardt (inner) iteration cap")
      ->default_val(opt->iterations);
  sub->add_option("--seed", opt->seed, "RANSAC seed (determinism)")
      ->default_val(opt->seed);

  // --- Loop-edge solver handling ---
  sub->add_flag(
      "--loop-trust", opt->loop_trust,
      "Trust loop edges: give them their own relaxed GNC-TLS inlier "
      "threshold (--loop-trust-inlier-cost) instead of --gnc-inlier-cost, so "
      "a genuine large-drift correction is not discarded as an outlier "
      "(Huber kernel under --no-gnc)");
  sub->add_option("--loop-trust-inlier-cost", opt->loop_trust_inlier_cost,
                  "GNC-TLS inlier threshold for --loop-trust loop edges "
                  "(0.5 * whitened squared residual)")
      ->default_val(opt->loop_trust_inlier_cost);
  sub->add_option("--loop-default-sigma-rot", opt->loop_default_sigma_rot,
                  "Rotation std (rad) of a stored loop edge without an "
                  "information matrix")
      ->default_val(opt->loop_default_sigma_rot);
  sub->add_option("--loop-default-sigma-trans", opt->loop_default_sigma_trans,
                  "Translation std (m) of a stored loop edge without an "
                  "information matrix")
      ->default_val(opt->loop_default_sigma_trans);

  // --- Manhattan-world prior (reusex-rqn.1) ---
  sub->add_flag(
      "--manhattan", opt->manhattan,
      "Manhattan-world prior: tie every vertical plane landmark to one of two "
      "dominant horizontal axes (estimated jointly) and every floor/ceiling "
      "landmark to the up axis. Makes rotational (yaw/tilt) drift between "
      "rooms observable without loop closure. Walls more than "
      "--manhattan-max-angle off the grid are left alone");
  sub->add_option("--manhattan-sigma", opt->manhattan_sigma_deg,
                  "Std of a Manhattan factor's normal residual (deg)")
      ->default_val(opt->manhattan_sigma_deg)
      ->check(CLI::PositiveNumber);
  sub->add_option("--manhattan-max-angle", opt->manhattan_max_angle_deg,
                  "Gate and GNC inlier threshold (deg): landmarks further "
                  "than this from the grid get no Manhattan factor. Must "
                  "exceed the drift to correct and stay below the building's "
                  "real off-grid angles")
      ->default_val(opt->manhattan_max_angle_deg)
      ->check(CLI::Range(0.5f, 45.0f));
  sub->add_option("--manhattan-level-max-angle",
                  opt->manhattan_level_max_angle_deg,
                  "Gate (deg) for tying floor/ceiling landmarks to up; keep "
                  "it below real ceiling slopes")
      ->default_val(opt->manhattan_level_max_angle_deg)
      ->check(CLI::Range(0.1f, 45.0f));
  sub->add_option("--manhattan-up", opt->manhattan_up,
                  "Up axis of the Manhattan grid: z (the pipeline's "
                  "convention), x, y, or auto (the axis the camera positions "
                  "spread least along)")
      ->default_val(opt->manhattan_up)
      ->check(CLI::IsMember({"auto", "x", "y", "z"}));

  // --- Flags that moved to `rux graph discover` (2026-10-13) ---
  // Registered hidden so a stale script gets a pointer instead of
  // "unknown option" (spec D5).
  auto moved_sink = std::make_shared<std::string>();
  auto moved =
      std::make_shared<std::vector<std::pair<std::string, CLI::Option *>>>();
  for (const auto &[flag, takes_value] : optimize_moved_flags()) {
    CLI::Option *o =
        takes_value ? sub->add_option(flag, *moved_sink) : sub->add_flag(flag);
    o->group(""); // hidden from --help
    moved->emplace_back(flag, o);
  }

  // --- Surfel extraction ---
  sub->add_option("--surfel-voxel", opt->surfel_voxel,
                  "Per-frame voxel downsample for surfels (m, <=0 disables)")
      ->default_val(opt->surfel_voxel);
  sub->add_option("--min-distance", opt->min_distance, "Minimum depth (m)")
      ->default_val(opt->min_distance);
  sub->add_option("--max-distance", opt->max_distance, "Maximum depth (m)")
      ->default_val(opt->max_distance);
  sub->add_option("--sampling-factor", opt->sampling_factor,
                  "Per-frame pixel subsampling factor")
      ->default_val(opt->sampling_factor);
  sub->add_option("--confidence", opt->confidence_threshold,
                  "Minimum confidence threshold")
      ->default_val(opt->confidence_threshold);

  sub->add_flag("--dry-run", opt->dry_run,
                "Compute and report statistics without writing poses back");

  sub->callback([opt, global_opt, moved, moved_sink]() {
    for (const auto &[flag, o] : *moved)
      if (o->count() > 0) {
        spdlog::error("{}", optimize_moved_flag_message(flag));
        rux::finish(RuxError::INVALID_ARGUMENT);
      }
    spdlog::trace("calling run_subcommand_optimize");
    if (rux::capture_stage_params("optimize", optimize_stage_parameters(*opt)))
      return;
    rux::finish(run_subcommand_optimize(*opt, *global_opt));
  });
}

std::string optimize_stage_parameters(SubcommandOptimizeOptions const &opt) {
  // The subset of the flags the in-process optimize stage reads (the GUI's
  // and the Qt client's runs; pipeline/stage_parameters.cpp).
  return rux::StageParams()
      .set("min_observations", opt.min_observations)
      .set("assoc_rounds", opt.assoc_rounds)
      .set("no_gnc", opt.no_gnc)
      .set("manhattan", opt.manhattan)
      .set("manhattan_sigma_deg", opt.manhattan_sigma_deg)
      .set("manhattan_max_angle_deg", opt.manhattan_max_angle_deg)
      .set("manhattan_level_max_angle_deg", opt.manhattan_level_max_angle_deg)
      .set("dry_run", opt.dry_run)
      .dump();
}

reusex::geometry::PlaneGraphOptions
plane_graph_options(SubcommandOptimizeOptions const &opt) {
  reusex::geometry::PlaneGraphOptions options;
  options.max_planes_per_frame = opt.max_planes_per_frame;
  options.min_plane_inliers = opt.min_plane_inliers;
  options.ransac_distance = opt.ransac_distance;
  options.ransac_normal_angle = opt.ransac_normal_angle;
  options.ransac_iterations = opt.ransac_iterations;
  options.assoc_normal_angle = opt.assoc_normal_angle;
  options.assoc_distance = opt.assoc_distance;
  options.min_landmark_observations = opt.min_observations;
  options.assoc_overlap_margin = opt.assoc_overlap_margin;
  options.min_landmark_spread_ratio = opt.min_landmark_spread_ratio;
  options.assoc_rounds = opt.assoc_rounds;
  options.assoc_round_tol = opt.assoc_round_tol;
  options.odometry_sigma_rot = opt.odometry_sigma_rot;
  options.odometry_sigma_trans = opt.odometry_sigma_trans;
  options.underconstrained_odom_scale = opt.underconstrained_odom_scale;
  options.plane_sigma_normal = opt.plane_sigma_normal;
  options.plane_sigma_distance = opt.plane_sigma_distance;
  options.plane_noise_model =
      opt.no_plane_inlier_weight
          ? reusex::geometry::PlaneNoiseModel::uniform
          : (opt.plane_noise == "uniform"
                 ? reusex::geometry::PlaneNoiseModel::uniform
                 : (opt.plane_noise == "inliers"
                        ? reusex::geometry::PlaneNoiseModel::inlier_count
                        : reusex::geometry::PlaneNoiseModel::fit_geometry));
  options.odometry_noise_model =
      opt.odometry_noise == "motion"
          ? reusex::geometry::OdometryNoiseModel::motion
          : reusex::geometry::OdometryNoiseModel::fixed;
  options.odometry_weight_min = opt.odometry_weight_min;
  options.odometry_weight_max = opt.odometry_weight_max;
  options.odometry_robust = opt.odometry_robust;
  options.odometry_gnc_inlier_cost = opt.odometry_gnc_inlier_cost;
  options.plane_weight_min = opt.plane_weight_min;
  options.plane_weight_max = opt.plane_weight_max;
  options.plane_sigma_scale = opt.plane_sigma_scale;
  options.use_plane_factors = !opt.no_plane_factors;
  options.prior_sigma_rot = opt.prior_sigma_rot;
  options.prior_sigma_trans = opt.prior_sigma_trans;
  options.use_gnc = !opt.no_gnc;
  options.gnc_inlier_cost = opt.gnc_inlier_cost;
  options.max_iterations = opt.iterations;
  options.seed = opt.seed;
  options.surfel.min_distance = opt.min_distance;
  options.surfel.max_distance = opt.max_distance;
  options.surfel.sampling_factor = opt.sampling_factor;
  options.surfel.confidence_threshold = opt.confidence_threshold;
  options.surfel.voxel_size = opt.surfel_voxel;
  options.loop_edges_trusted = opt.loop_trust;
  options.loop_trust_inlier_cost = opt.loop_trust_inlier_cost;
  options.loop_default_sigma_rot = opt.loop_default_sigma_rot;
  options.loop_default_sigma_trans = opt.loop_default_sigma_trans;
  options.manhattan = opt.manhattan;
  options.manhattan_sigma_deg = opt.manhattan_sigma_deg;
  options.manhattan_max_angle_deg = opt.manhattan_max_angle_deg;
  options.manhattan_level_max_angle_deg = opt.manhattan_level_max_angle_deg;
  options.manhattan_up =
      opt.manhattan_up == "x"
          ? reusex::geometry::ManhattanUp::x
          : (opt.manhattan_up == "y"
                 ? reusex::geometry::ManhattanUp::y
                 : (opt.manhattan_up == "z"
                        ? reusex::geometry::ManhattanUp::z
                        : reusex::geometry::ManhattanUp::automatic));
  return options;
}

int run_subcommand_optimize(SubcommandOptimizeOptions const &opt,
                            const RuxOptions &global_opt) {
  fs::path project_path = global_opt.project_db;
  spdlog::info("Optimizing sensor poses in: {}", project_path.string());

  try {
    // std::clamp is UB when lo > hi, and the plane-factor weighting clamps
    // into [plane_weight_min, plane_weight_max]. Reject the inverted range
    // here instead of letting it reach the library.
    if (opt.plane_weight_min > opt.plane_weight_max) {
      spdlog::error("--plane-weight-min ({}) must be <= --plane-weight-max "
                    "({})",
                    opt.plane_weight_min, opt.plane_weight_max);
      spdlog::info("Resolution: pass a non-inverted range, e.g. "
                   "--plane-weight-min 0.5 --plane-weight-max 3.0");
      return RuxError::INVALID_ARGUMENT;
    }

    reusex::ProjectDB db(project_path);

    // Reuse the register prerequisites (>= 2 sensor frames carrying depth).
    if (int rc = rux::check_stage_prerequisites(
            db, reusex::core::PipelineStage::optimize);
        rc != RuxError::SUCCESS)
      return rc;

    // Flags -> options, then the stage parameters through the same reader
    // the in-process optimize stage uses (Qt client, ruxd): the two paths
    // cannot read min_observations / assoc_rounds / no_gnc / dry_run apart.
    reusex::geometry::PlaneGraphOptions options = plane_graph_options(opt);
    bool dry_run = opt.dry_run;
    reusex::geometry::apply_optimize_parameters(options, dry_run,
                                                optimize_stage_parameters(opt));

    int logId = db.log_pipeline_start(
        "pose_optimization_plane_graph",
        fmt::format(
            R"({{"min_observations":{},"assoc_rounds":{},"no_gnc":{},"assoc_normal_angle":{},"assoc_distance":{},"max_planes_per_frame":{},"min_plane_inliers":{},"plane_noise":"{}","use_gnc":{},"iterations":{},"seed":{},"dry_run":{},"loop_trust":{}}})",
            opt.min_observations, opt.assoc_rounds, opt.no_gnc,
            opt.assoc_normal_angle, opt.assoc_distance,
            opt.max_planes_per_frame, opt.min_plane_inliers,
            opt.no_plane_inlier_weight ? "uniform" : opt.plane_noise,
            !opt.no_gnc, opt.iterations, opt.seed, dry_run, opt.loop_trust));

    auto result = reusex::geometry::optimize_sensor_poses(db, options, dry_run);

    db.log_pipeline_end(logId, true);

    spdlog::info("Plane-graph optimization: {} frames, {} plane detections, "
                 "{} landmarks, {} plane factors",
                 result.frames, result.planes_detected, result.landmarks,
                 result.plane_factors);
    spdlog::info("Factor-graph error {:.4f} -> {:.4f}, max pose shift {:.4f} m",
                 result.initial_error, result.final_error,
                 result.max_pose_shift);
    spdlog::info("Pose graph: {} stored edges in the solve{}{}",
                 result.graph_edges,
                 result.bridge_edges > 0
                     ? fmt::format(", {} bridging edges from the initial "
                                   "poses",
                                   result.bridge_edges)
                     : std::string(),
                 result.graph_initialised
                     ? " (odometry initialised from the current poses)"
                     : "");
    if (result.median_fit_sigma_normal > 0.0)
      spdlog::info(
          "Plane measurement noise: median fit sigma {:.5f} rad / {:.5f} m; "
          "{} strong and {} weak observations hit the sigma-scale clamp",
          result.median_fit_sigma_normal, result.median_fit_sigma_distance,
          result.plane_noise_clamped_low, result.plane_noise_clamped_high);
    if (result.loop_edges > 0)
      spdlog::info("Loop closure: {} wide-baseline edges added to the graph",
                   result.loop_edges);

    // Mirrors the library guard (PlaneGraphOptimizer::optimize): the poses are
    // only left untouched when there is NEITHER a landmark NOR a loop edge to
    // constrain them. With loop edges but no landmarks the graph was solved and
    // the poses WERE written, so this must not short-circuit.
    if (result.landmarks == 0 && result.loop_edges == 0) {
      spdlog::warn("No plane landmarks reached the minimum observation count "
                   "and no loop edges were added; poses were left unchanged. "
                   "Try lowering --min-observations or relaxing "
                   "--assoc-distance / --assoc-normal-angle.");
      return RuxError::SUCCESS;
    }

    if (!result.converged) {
      spdlog::error("Factor-graph optimizer failed to produce a solution; "
                    "poses were left unchanged (see log for details)");
      return RuxError::GENERIC;
    }

    if (opt.dry_run)
      spdlog::info("Dry run: poses were not modified");
    else
      spdlog::info("Optimized poses written. Run 'rux create clouds' to "
                   "regenerate the merged point cloud.");

    return RuxError::SUCCESS;

  } catch (const std::exception &e) {
    spdlog::error("Pose optimization failed: {}", e.what());
    return RuxError::GENERIC;
  }
}
