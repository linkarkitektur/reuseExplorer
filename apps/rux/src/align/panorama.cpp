// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "align/panorama.hpp"
#include "exit_status.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/slam/PanoramaAlignment.hpp>

#include <spdlog/fmt/std.h>
#include <spdlog/spdlog.h>

#include <cmath>
#include <exception>
#include <filesystem>
#include <fstream>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

void setup_subcommand_align_panorama(CLI::App &app,
                                     std::shared_ptr<RuxOptions> global_opt) {
  auto opt = std::make_shared<SubcommandAlignPanoramaOptions>();
  auto *sub = app.add_subcommand(
      "360", "Align 360 panoramas to the scan (content-based pose refinement)");

  sub->footer(R"(
DESCRIPTION:
  Refines each panorama's 6-DoF pose from image content instead of the EXIF
  timestamp placement. Virtual pinhole slices of the equirect are ORB-matched
  against nearby sensor frames, the matches are lifted to metric 3D via the
  frames' depth, and the panorama pose is resected with PnP + RANSAC.

EXAMPLES:
  rux align 360                    # align all panoramas, write refined poses
  rux align 360 --dry-run          # report inliers / RMS / correction only
  rux align 360 --overwrite        # re-align panoramas already aligned
  rux align 360 --fix-translation  # rotation-only (recommended when EXIF reliable)
  rux -vv align 360 --dry-run --diagnostics diag.json
                                   # why each panorama aligned or not

WORKFLOW:
  1. rux import rtabmap scan.db     # sensor frames (with timestamps + depth)
  2. rux import 360 ./360/          # panoramas, seeded by timestamp
  3. rux align 360                  # refine panorama poses from content

NOTES:
  - Requires panoramas linked to a sensor frame (node_id) with depth.
  - A panorama that fails to align keeps its timestamp placement; the log
    names the gate that rejected it and its numbers (no_slice_pnp,
    below_noise_floor, upside_down, too_few_pooled_inliers,
    refinement_collapsed, correction_gate, ...).
    -vv logs the matching funnel per panorama, -vvv per slice and frame.
  - Acceptance is evidence-based: the best slice's PnP support must beat the
    same panorama's shuffled-correspondence null by --null-margin
    (below_noise_floor otherwise), and the centre must stay within
    --max-correction of the seed frame, the hand-held position prior for a
    360 camera carried next to the scanner. Raise or disable (0) it for a
    camera that was placed elsewhere.
  - --fix-translation is recommended when EXIF timestamps are reliable.
    The panorama centre is fixed to the seed frame position; only the 3-DOF
    rotation is solved. This avoids ill-conditioned translation drift when
    the matched frames span a narrow angular cone.
)");

  sub->add_flag("--dry-run", opt->dry_run,
                "Compute and report only; do not write poses");
  sub->add_flag("--overwrite", opt->overwrite,
                "Re-align panoramas that are already aligned");
  auto &ao = opt->align; // flags bind to the library struct (STANDARDS §4)
  sub->add_flag(
      "--fix-translation", ao.fix_translation,
      "Fix the panorama centre to the seed frame's stored position and solve "
      "for rotation only. Recommended when EXIF timestamps are reliable.");
  sub->add_option("--min-inliers", ao.min_inliers,
                  "Accept an alignment above this many pooled inliers")
      ->check(CLI::NonNegativeNumber)
      ->capture_default_str();
  sub->add_option("--window", ao.candidate_window,
                  "Candidate sensor frames on each side of the seed")
      ->check(CLI::NonNegativeNumber)
      ->capture_default_str();
  sub->add_option("--max-candidates", ao.max_candidates,
                  "Cap candidate frames matched per panorama")
      ->check(CLI::PositiveNumber)
      ->capture_default_str();
  sub->add_option("--n-yaw", ao.n_yaw, "Perspective slices around the equator")
      ->check(CLI::Range(1, 64))
      ->capture_default_str();
  sub->add_option("--fov", ao.fov_deg, "Per-slice horizontal FOV (deg)")
      ->check(CLI::Range(1.0, 179.0))
      ->capture_default_str();
  sub->add_option("--slice", ao.slice, "Perspective slice size (px, square)")
      ->check(CLI::Range(64, 8192))
      ->capture_default_str();
  sub->add_option("--max-features", ao.max_features, "ORB features per image")
      ->check(CLI::PositiveNumber)
      ->capture_default_str();
  sub->add_option("--ratio-test", ao.ratio_test, "Lowe ratio threshold")
      ->check(CLI::Range(0.0f, 1.0f))
      ->capture_default_str();
  sub->add_option("--max-depth", ao.max_depth,
                  "Ignore frame keypoints deeper than this (m)")
      ->check(CLI::PositiveNumber)
      ->capture_default_str();
  sub->add_option("--ransac-iterations", ao.ransac_iterations,
                  "solvePnPRansac iterations per slice")
      ->check(CLI::PositiveNumber)
      ->capture_default_str();
  sub->add_option("--null-trials", ao.null_trials,
                  "Shuffled-correspondence trials that measure each "
                  "panorama's noise floor (0 disables the gate)")
      ->check(CLI::NonNegativeNumber)
      ->capture_default_str();
  sub->add_option("--null-margin", ao.null_margin,
                  "The best slice must beat the noise floor by this factor")
      ->check(CLI::Range(1.0, 100.0))
      ->capture_default_str();
  sub->add_option("--max-correction", ao.max_correction_m,
                  "Hand-held position prior: reject a centre further than "
                  "this from the seed frame, in m (0 disables)")
      ->capture_default_str();
  sub->add_option("--seed", ao.seed, "Seed of the noise-floor permutations")
      ->capture_default_str();
  sub->add_option("--figures", opt->figures_dir,
                  "Dump alignment comparison figures to this directory");
  sub->add_option("--figures-limit", opt->figures_limit,
                  "Cap number of panoramas figured")
      ->capture_default_str();
  sub->add_option("--diagnostics", opt->diagnostics_path,
                  "Write each panorama's outcome and matching funnel "
                  "(per-slice / per-frame counts) to this JSON file");
  sub->add_flag("--diagnostics-correspondences",
                opt->diagnostics_correspondences,
                "Also write every depth-backed correspondence to the "
                "--diagnostics file");

  sub->callback([opt, global_opt]() {
    spdlog::trace("calling run_subcommand_align_panorama");
    rux::finish(run_subcommand_align_panorama(*opt, *global_opt));
  });
}

int run_subcommand_align_panorama(SubcommandAlignPanoramaOptions const &opt,
                                  const RuxOptions &global_opt) {
  try {
    fs::path project_path = global_opt.project_db;
    spdlog::info("Aligning 360 panoramas in project: {}",
                 project_path.string());

    reusex::ProjectDB db(project_path);

    reusex::geometry::PanoramaAlignmentOptions ao = opt.align;
    ao.collect_correspondences =
        !opt.diagnostics_path.empty() && opt.diagnostics_correspondences;
    if (opt.diagnostics_correspondences && opt.diagnostics_path.empty())
      spdlog::warn("--diagnostics-correspondences has no effect without "
                   "--diagnostics");

    // Slices are spread evenly over 360 deg; if their FOVs do not add up the
    // equator is sampled with gaps and matchable features get dropped.
    if (ao.n_yaw * ao.fov_deg < 360.0)
      spdlog::warn("--n-yaw {} x --fov {} deg covers only {:.0f} deg of the "
                   "equator; slices leave gaps and matches will be missed",
                   ao.n_yaw, ao.fov_deg, ao.n_yaw * ao.fov_deg);

    auto panos = db.list_panoramic_images();
    if (panos.empty()) {
      spdlog::warn("No panoramas in project. Nothing to align.");
      return RuxError::SUCCESS;
    }

    int log_id = db.log_pipeline_start(
        "align_360", fmt::format(R"({{"panoramas":{}}})", panos.size()));

    int aligned = 0, skipped = 0, failed = 0, figured = 0;
    double sum_delta = 0.0;
    std::map<std::string, int> by_outcome;
    std::vector<std::string> diagnostics;

    try {
      for (const auto &pano : panos) {
        if (pano.node_id < 0) {
          spdlog::info("{}: no linked sensor frame, skipping", pano.filename);
          ++skipped;
          continue;
        }
        if (pano.has_pose && !opt.overwrite) {
          spdlog::info("{}: already aligned (use --overwrite to redo)",
                       pano.filename);
          ++skipped;
          continue;
        }

        // Enable the correspondence figure for the first N panoramas figured.
        if (!opt.figures_dir.empty() && figured < opt.figures_limit) {
          ao.debug_dir = opt.figures_dir;
          ao.debug_name = fs::path(pano.filename).replace_extension().string();
        } else {
          ao.debug_dir.clear();
        }

        auto r =
            reusex::geometry::align_panorama(db, pano.id, pano.node_id, ao);
        ++by_outcome[reusex::geometry::to_string(r.outcome)];
        if (!opt.diagnostics_path.empty())
          diagnostics.push_back(
              reusex::geometry::panorama_alignment_diagnostics_json(r));
        if (!r.aligned) {
          // names the gate and the numbers that decided it (STANDARDS §5)
          spdlog::warn("{}: not aligned, keeps its timestamp placement — {}",
                       pano.filename,
                       reusex::geometry::panorama_alignment_summary(r));
          ++failed;
          continue;
        }

        if (r.translation_fixed) {
          spdlog::info(
              "{}: aligned (rotation-only) — {} inliers, {:.3f} deg RMS, "
              "translation fixed (seed node position){}",
              pano.filename, r.inliers, r.rms_deg,
              opt.dry_run ? " (dry-run)" : "");
        } else {
          spdlog::info("{}: aligned — {} inliers, {:.3f} deg RMS, {:.3f} m "
                       "correction{}",
                       pano.filename, r.inliers, r.rms_deg, r.delta_from_seed_m,
                       opt.dry_run ? " (dry-run)" : "");
        }
        sum_delta += r.delta_from_seed_m;
        ++aligned;
        if (!ao.debug_dir.empty())
          ++figured;

        if (!opt.dry_run)
          db.save_panorama_pose(pano.id, r.pose, r.inliers, r.rms_deg);
      }
      db.log_pipeline_end(log_id, true);
    } catch (...) {
      db.log_pipeline_end(log_id, false, "align_360 failed");
      throw;
    }

    spdlog::info("Alignment complete: {} aligned, {} skipped, {} failed"
                 "{}. Mean correction {:.3f} m",
                 aligned, skipped, failed,
                 opt.dry_run ? " (dry-run, no poses written)" : "",
                 aligned > 0 ? sum_delta / aligned : 0.0);
    for (const auto &[outcome, n] : by_outcome)
      spdlog::info("  {}: {}", outcome, n);
    if (aligned == 0 && failed > 0)
      spdlog::warn("No panorama aligned: all {} attempted kept their "
                   "timestamp placement (gates per panorama above)",
                   failed);

    if (!opt.diagnostics_path.empty()) {
      std::ofstream out(opt.diagnostics_path);
      if (!out)
        throw std::runtime_error("cannot write diagnostics file " +
                                 opt.diagnostics_path);
      out << "[\n";
      for (size_t i = 0; i < diagnostics.size(); ++i)
        out << diagnostics[i] << (i + 1 < diagnostics.size() ? ",\n" : "\n");
      out << "]\n";
      spdlog::info("Wrote alignment diagnostics for {} panoramas to {}",
                   diagnostics.size(), opt.diagnostics_path);
    }

    return RuxError::SUCCESS;

  } catch (const std::exception &e) {
    spdlog::error("Panorama alignment failed: {}", e.what());
    return RuxError::GENERIC;
  }
}
