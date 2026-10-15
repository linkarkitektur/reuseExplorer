// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "../global-params.hpp"

#include <CLI/CLI.hpp>
#include <memory>
#include <string>

struct SubcommandAlignPanoramaOptions {
  bool dry_run = false;   ///< compute + report only, do not write poses
  bool overwrite = false; ///< re-align panoramas that are already aligned
  bool fix_translation =
      false;                   ///< fix centre to seed pose, solve rotation only
  int min_inliers = 40;        ///< accept an alignment above this many inliers
  int window = 12;             ///< candidate frames on each side of the seed
  int max_candidates = 8;      ///< cap candidate frames matched per panorama
  int n_yaw = 8;               ///< perspective slices around the equator
  double fov = 90.0;           ///< per-slice horizontal FOV (deg)
  double max_correction = 5.0; ///< reject corrections beyond this (m); 0=off
  std::string figures_dir; ///< if set, dump alignment comparison figures here
  int figures_limit = 6;   ///< cap number of panoramas figured
  /// if set, write every panorama's matching funnel (outcome, per-slice and
  /// per-frame counts) to this JSON file
  std::string diagnostics_path;
  /// also include every depth-backed correspondence in the diagnostics file
  bool diagnostics_correspondences = false;
};

void setup_subcommand_align_panorama(CLI::App &app,
                                     std::shared_ptr<RuxOptions> global_opt);
int run_subcommand_align_panorama(SubcommandAlignPanoramaOptions const &opt,
                                  const RuxOptions &global_opt);
