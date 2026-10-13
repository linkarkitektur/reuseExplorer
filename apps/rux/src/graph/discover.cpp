// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "graph/discover.hpp"
#include "create/stage_bridge.hpp"
#include "exit_status.hpp"
#include "stage_prerequisites.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/stage_contract.hpp>
#include <reusex/slam/discover_parameters.hpp>

#include <spdlog/spdlog.h>

#include <exception>
#include <stdexcept>

void setup_subcommand_graph_discover(CLI::App &parent,
                                     std::shared_ptr<RuxOptions> global_opt) {
  auto opt = std::make_shared<SubcommandGraphDiscoverOptions>();
  auto *sub = parent.add_subcommand(
      "discover", "Find wide-baseline loop edges and add them to the graph");
  sub->footer(R"(
DESCRIPTION:
  Runs the loop-edge front-ends seeded from the INITIAL poses — ORB frame-pair
  matching (on by default), 360-panorama resections (--use-panoramas) and an
  external matcher file (--loop-edges) — then skips every frame pair the
  stored graph already decides (an existing edge, or one you rejected), keeps
  the largest set consistent with the stored edges (PCM; confirmed edges are
  fixed inliers) and stores the survivors with status 'auto'. Run
  'rux optimize' afterwards to use them.

EXAMPLES:
  rux graph discover                               # ORB loop closure
  rux graph discover --use-panoramas               # ...plus panorama edges
  rux graph discover --no-loop-closure --loop-edges edges.json
  rux graph discover --dry-run                     # report only
)");

  sub->add_flag("--no-loop-closure", opt->no_loop_closure,
                "Do not run ORB frame-pair loop detection");
  sub->add_option("--loop-proposal", opt->loop_proposal,
                  "Loop-candidate proposal: auto (exhaustive on small scans, "
                  "else appearance), appearance, spatial or exhaustive")
      ->default_val(opt->loop_proposal)
      ->check(CLI::IsMember({"auto", "appearance", "spatial", "exhaustive"}));
  sub->add_option("--loop-min-frame-gap", opt->loop_min_frame_gap,
                  "Skip pairs closer than this in frame index")
      ->default_val(opt->loop_min_frame_gap);
  sub->add_option("--loop-max-distance", opt->loop_max_distance,
                  "Max seed camera-centre distance to propose a loop pair (m)")
      ->default_val(opt->loop_max_distance);
  sub->add_option("--loop-max-view-angle", opt->loop_max_view_angle,
                  "Max viewing-direction angle to propose a loop pair (deg)")
      ->default_val(opt->loop_max_view_angle);
  sub->add_option("--loop-max-candidates", opt->loop_max_candidates,
                  "Max loop candidates per frame (nearest first)")
      ->default_val(opt->loop_max_candidates);
  sub->add_option("--loop-min-inliers", opt->loop_min_inliers,
                  "Reject a loop edge below this many RANSAC inliers")
      ->default_val(opt->loop_min_inliers);
  sub->add_option("--loop-max-features", opt->loop_max_features,
                  "ORB features per frame for loop matching")
      ->default_val(opt->loop_max_features);
  sub->add_option("--loop-ratio-test", opt->loop_ratio_test,
                  "Lowe ratio threshold for loop descriptor matches")
      ->default_val(opt->loop_ratio_test);
  sub->add_option("--loop-ransac-inlier-dist", opt->loop_ransac_inlier_dist,
                  "3D-3D RANSAC inlier threshold for loop relative pose (m)")
      ->default_val(opt->loop_ransac_inlier_dist);
  sub->add_option("--loop-max-seed-disagreement",
                  opt->loop_max_seed_disagreement,
                  "Reject a loop edge whose translation disagrees with the "
                  "seed by more than this (m, <=0 disables)")
      ->default_val(opt->loop_max_seed_disagreement);
  sub->add_option("--loop-min-seed-disagreement",
                  opt->loop_min_seed_disagreement,
                  "Absolute floor (m) of the lower seed-disagreement gate")
      ->default_val(opt->loop_min_seed_disagreement);
  sub->add_option("--loop-min-seed-disagreement-fraction",
                  opt->loop_min_seed_disagreement_fraction,
                  "Scale-relative part of that gate (fraction of the "
                  "trajectory extent, #339)")
      ->default_val(opt->loop_min_seed_disagreement_fraction);
  sub->add_flag("--loop-no-pcm", opt->loop_no_pcm,
                "Disable pairwise-consistency (PCM) filtering");
  sub->add_flag("--use-panoramas", opt->use_panoramas,
                "Derive loop edges from the project's 360 panoramas (#236)");
  sub->add_option("--pano-max-frames", opt->pano_max_frames,
                  "Max sensor frames resected against each panorama")
      ->default_val(opt->pano_max_frames);
  sub->add_option("--pano-min-inliers", opt->pano_min_inliers,
                  "Accept a per-frame resection above this many PnP inliers")
      ->default_val(opt->pano_min_inliers);
  sub->add_option("--pano-max-edges", opt->pano_max_edges,
                  "Max loop edges contributed per panorama")
      ->default_val(opt->pano_max_edges);
  sub->add_option("--pano-slices", opt->pano_n_yaw,
                  "Perspective slices rendered around the panorama equator")
      ->default_val(opt->pano_n_yaw);
  sub->add_option("--pano-max-features", opt->pano_max_features,
                  "ORB features per panorama slice / sensor frame")
      ->default_val(opt->pano_max_features);
  sub->add_option("--pano-max-distance", opt->pano_max_distance,
                  "Reject a resection placing the panorama further than this "
                  "from the frame (m, <=0 disables)")
      ->default_val(opt->pano_max_distance);
  sub->add_option("--loop-edges", opt->loop_edges_file,
                  "Externally computed loop edges (reusex.loop_edges.v1 JSON)");
  sub->add_option("--loop-edges-min-disagreement",
                  opt->loop_edges_min_disagreement,
                  "Drop file edges agreeing with the seed within this (m); "
                  "absolute floor of the gate")
      ->default_val(opt->loop_edges_min_disagreement);
  sub->add_option("--loop-edges-min-disagreement-fraction",
                  opt->loop_edges_min_disagreement_fraction,
                  "Scale-relative part of the file-edge gate (#339)")
      ->default_val(opt->loop_edges_min_disagreement_fraction);
  sub->add_option("--seed", opt->seed, "RANSAC seed (determinism)")
      ->default_val(opt->seed);
  sub->add_flag("--dry-run", opt->dry_run,
                "Report what would be added without writing edges");

  sub->callback([opt, global_opt]() {
    if (rux::capture_stage_params("graph_discover",
                                  graph_discover_stage_parameters(*opt)))
      return;
    rux::finish(run_subcommand_graph_discover(*opt, *global_opt));
  });
}

std::string
graph_discover_stage_parameters(const SubcommandGraphDiscoverOptions &opt) {
  // The subset the in-process stage reads (pipeline/stage_parameters.cpp).
  return rux::StageParams()
      .set("loop_closure", !opt.no_loop_closure)
      .set("use_panoramas", opt.use_panoramas)
      .set("loop_edges_file", opt.loop_edges_file)
      .set("loop_proposal", opt.loop_proposal)
      .set("loop_min_frame_gap", opt.loop_min_frame_gap)
      .set("loop_min_inliers", opt.loop_min_inliers)
      .set("pcm", !opt.loop_no_pcm)
      .set("dry_run", opt.dry_run)
      .dump();
}

reusex::slam::GraphDiscoverOptions
graph_discover_options(const SubcommandGraphDiscoverOptions &opt) {
  reusex::slam::GraphDiscoverOptions o;
  auto &lc = o.candidates.loop_closure;
  lc.enable = !opt.no_loop_closure;
  lc.proposal = reusex::slam::parse_loop_proposal(opt.loop_proposal);
  lc.min_frame_gap = opt.loop_min_frame_gap;
  lc.max_candidate_distance = opt.loop_max_distance;
  lc.max_view_angle = opt.loop_max_view_angle;
  lc.max_candidates_per_frame = opt.loop_max_candidates;
  lc.min_match_inliers = opt.loop_min_inliers;
  lc.max_features = opt.loop_max_features;
  lc.ratio_test = opt.loop_ratio_test;
  lc.ransac_inlier_dist = opt.loop_ransac_inlier_dist;
  lc.max_seed_disagreement = opt.loop_max_seed_disagreement;
  lc.min_seed_disagreement = opt.loop_min_seed_disagreement;
  lc.min_seed_disagreement_fraction = opt.loop_min_seed_disagreement_fraction;
  lc.pcm = !opt.loop_no_pcm;
  lc.seed = opt.seed;
  auto &pl = o.candidates.panorama_loops;
  pl.enable = opt.use_panoramas;
  pl.max_frames = opt.pano_max_frames;
  pl.min_frame_inliers = opt.pano_min_inliers;
  pl.max_edges_per_panorama = opt.pano_max_edges;
  pl.n_yaw = opt.pano_n_yaw;
  pl.max_features = opt.pano_max_features;
  pl.max_pano_distance = opt.pano_max_distance;
  pl.seed = opt.seed;
  o.candidates.loop_edges_file = opt.loop_edges_file;
  o.candidates.loop_edges_min_seed_disagreement =
      opt.loop_edges_min_disagreement;
  o.candidates.loop_edges_min_seed_disagreement_fraction =
      opt.loop_edges_min_disagreement_fraction;
  o.dry_run = opt.dry_run;
  return o;
}

int run_subcommand_graph_discover(const SubcommandGraphDiscoverOptions &opt,
                                  const RuxOptions &global_opt) {
  try {
    reusex::ProjectDB db(global_opt.project_db);
    if (int rc = rux::check_stage_prerequisites(
            db, reusex::core::PipelineStage::graph_discover);
        rc != RuxError::SUCCESS)
      return rc;
    // Flags -> options, then the stage JSON through the reader the GUI's
    // in-process stage uses: the two paths cannot read the keys apart.
    auto options = graph_discover_options(opt);
    const std::string params = graph_discover_stage_parameters(opt);
    reusex::slam::apply_discover_parameters(options, params);
    const int logId = db.log_pipeline_start("graph_discover", params);
    try {
      const auto r = reusex::slam::discover_loop_edges(db, options);
      db.log_pipeline_end(logId, true);
      if (r.initialised_graph)
        spdlog::info("No odometry was stored: initialised {} odometry edges "
                     "first (as 'rux graph init')",
                     r.odometry_edges_written);
      spdlog::info("Loop edges{}: {} proposed, {} matched, {} PCM-rejected, "
                   "{} over the PCM cap, {} already stored, {} rejected "
                   "before, {} added",
                   opt.dry_run ? " (dry run)" : "", r.proposed, r.candidates,
                   r.pcm_rejected, r.pcm_capped, r.skipped_existing,
                   r.skipped_rejected, r.added);
      if (!opt.dry_run && r.added > 0)
        spdlog::info("Run 'rux optimize' to solve with the new edges.");
      return RuxError::SUCCESS;
    } catch (const std::exception &e) {
      db.log_pipeline_end(logId, false, e.what());
      throw;
    }
  } catch (const std::invalid_argument &e) {
    spdlog::error("graph discover: {}", e.what());
    return RuxError::INVALID_ARGUMENT;
  } catch (const std::exception &e) {
    spdlog::error("graph discover failed: {}", e.what());
    return RuxError::GENERIC;
  }
}
