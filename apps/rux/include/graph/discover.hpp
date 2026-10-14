// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "../global-params.hpp"

// Included to MIRROR the library defaults (docs/STANDARDS.md §4).
#include <reusex/slam/graph_discover.hpp>

#include <CLI/CLI.hpp>
#include <memory>
#include <string>

namespace graph_discover_defaults {
inline const reusex::slam::GraphDiscoverOptions kOptions{};
inline const auto &kLoop = kOptions.candidates.loop_closure;
inline const auto &kPano = kOptions.candidates.panorama_loops;
} // namespace graph_discover_defaults

/// `rux graph discover` flags; defaults mirror slam::GraphDiscoverOptions.
struct SubcommandGraphDiscoverOptions {
  bool no_loop_closure = !graph_discover_defaults::kLoop.enable;
  std::string loop_proposal = "auto";
  int loop_min_frame_gap = graph_discover_defaults::kLoop.min_frame_gap;
  float loop_max_distance =
      graph_discover_defaults::kLoop.max_candidate_distance;
  float loop_max_view_angle = graph_discover_defaults::kLoop.max_view_angle;
  int loop_max_candidates =
      graph_discover_defaults::kLoop.max_candidates_per_frame;
  int loop_min_inliers = graph_discover_defaults::kLoop.min_match_inliers;
  int loop_max_features = graph_discover_defaults::kLoop.max_features;
  float loop_ratio_test = graph_discover_defaults::kLoop.ratio_test;
  float loop_ransac_inlier_dist =
      graph_discover_defaults::kLoop.ransac_inlier_dist;
  float loop_max_seed_disagreement =
      graph_discover_defaults::kLoop.max_seed_disagreement;
  float loop_min_seed_disagreement =
      graph_discover_defaults::kLoop.min_seed_disagreement;
  float loop_min_seed_disagreement_fraction =
      graph_discover_defaults::kLoop.min_seed_disagreement_fraction;
  bool loop_no_pcm = !graph_discover_defaults::kLoop.pcm;
  bool use_panoramas = graph_discover_defaults::kPano.enable;
  int pano_max_frames = graph_discover_defaults::kPano.max_frames;
  int pano_min_inliers = graph_discover_defaults::kPano.min_frame_inliers;
  int pano_max_edges = graph_discover_defaults::kPano.max_edges_per_panorama;
  int pano_n_yaw = graph_discover_defaults::kPano.n_yaw;
  int pano_max_features = graph_discover_defaults::kPano.max_features;
  double pano_max_distance = graph_discover_defaults::kPano.max_pano_distance;
  std::string loop_edges_file =
      graph_discover_defaults::kOptions.candidates.loop_edges_file;
  double loop_edges_min_disagreement =
      graph_discover_defaults::kOptions.candidates
          .loop_edges_min_seed_disagreement;
  double loop_edges_min_disagreement_fraction =
      graph_discover_defaults::kOptions.candidates
          .loop_edges_min_seed_disagreement_fraction;
  unsigned seed = graph_discover_defaults::kLoop.seed;
  bool dry_run = graph_discover_defaults::kOptions.dry_run;
};

/// Every flag -> the library options (what `rux graph discover` runs with).
reusex::slam::GraphDiscoverOptions
graph_discover_options(const SubcommandGraphDiscoverOptions &opt);

/// The flags the in-process graph_discover stage reads, as its JSON.
std::string
graph_discover_stage_parameters(const SubcommandGraphDiscoverOptions &opt);

void setup_subcommand_graph_discover(CLI::App &parent,
                                     std::shared_ptr<RuxOptions> global_opt);
int run_subcommand_graph_discover(const SubcommandGraphDiscoverOptions &opt,
                                  const RuxOptions &global_opt);
