// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "edit/perturb_poses.hpp"
#include "exit_status.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/slam/perturb_poses.hpp>

#include <spdlog/spdlog.h>

#include <stdexcept>

void setup_subcommand_edit_perturb_poses(
    CLI::App &app, std::shared_ptr<RuxOptions> global_opt) {
  auto opt = std::make_shared<SubcommandEditPerturbPosesOptions>();
  auto *sub = app.add_subcommand(
      "perturb-poses",
      "Inject seeded, reproducible synthetic drift into the stored poses");

  sub->footer(R"(
DESCRIPTION:
  Overwrites every stored sensor-frame pose with a drifted version of itself,
  so a capture that has absolute ground truth but no drift can adjudicate a
  pose-refinement stage (issue #338, #221 Tier 2).

  The three ARKitScenes benchmark scans carry a GT mesh but their seeds are
  already accurate to 15.7-18.6 mm, so the correct behaviour of every pose
  stage on them is to do nothing — which makes them unable to measure whether
  a pose stage works. This command supplies the missing half: it perturbs the
  RELATIVE pose chain and integrates, the way real SLAM drift accumulates, and
  leaves the GT mesh (and frame 0) untouched so the injected deformation is
  exactly what a solver is being asked to undo.

  The drift model is a bias term (an Ornstein-Uhlenbeck process over distance
  travelled — gyro bias, scale error: the systematic, curved part) plus a
  sqrt(distance) random walk. Its overall amplitude is solved so that the
  realised median relative-pose disagreement over temporally distant frame
  pairs equals --target-drift-ratio * --drift-scale times the trajectory's
  extent. The 0.20 default puts 2-9x the measured loop-edge error and ~8x the
  50 mm F-score threshold of drift on the ARKitScenes scans. The office scan's
  own regime is 0.80 (14.455 m over an 18.01 m extent,
  registration-improvements.md §9.5) and is reachable with --drift-scale 4,
  but applied to a 1.8 m room scan it tumbles the worst frame by 180 deg and
  stops being drift; the command warns past 60 deg.

  DESTRUCTIVE. The poses are overwritten in place. Run it on a COPY of a
  project, never on an original capture; --yes is required to confirm.

EXAMPLES:
  rux -p copy.rux edit perturb-poses --dry-run
      Report the trajectory's extent, path length and frame count without
      writing anything.

  rux -p copy.rux edit perturb-poses --seed 1 --drift-scale 1.0 --yes
      The default regime: 20% of the trajectory extent, realisation 1.

  rux -p copy.rux edit perturb-poses --seed 1 --drift-scale 0.25 --yes
      A mild capture: still several times the 50 mm F-score threshold.

  rux -p copy.rux edit perturb-poses --seed 1 --drift-scale 1.0 --mode yaw --yes
      Heading drift only (reusex-xka): roll, pitch and camera height stay on
      the seed, as on a capture whose IMU keeps gravity honest.

  rux -p copy.rux edit perturb-poses --seed 1 --drift-scale 4 --yes
      The office scan's own (pathological) 80%-of-extent regime.

NOTES:
  - Deterministic: same project + same --seed + same options => identical
    poses, bit for bit (docs/STANDARDS.md §6). Vary --seed for an ensemble.
  - --drift-scale 0 is a no-op that leaves the poses bit-identical; it is the
    guard proving the harness itself injects nothing.
  - Frames with no stored pose are left untouched and reported.
)");

  sub->add_option("--seed", opt->seed, "RNG seed; fixes the realisation")
      ->default_val(opt->seed);
  sub->add_option("--drift-scale", opt->drift_scale,
                  "Multiplier on the target drift ratio (0 = no-op)")
      ->default_val(opt->drift_scale)
      ->check(CLI::NonNegativeNumber);
  sub->add_option("--target-drift-ratio", opt->target_drift_ratio,
                  "Target median distant-pair disagreement as a fraction of "
                  "the trajectory's extent (0.80 = the office scan's measured "
                  "drift regime)")
      ->default_val(opt->target_drift_ratio);
  sub->add_option("--min-frame-gap", opt->min_frame_gap,
                  "Frame gap defining a 'temporally distant' pair for the "
                  "drift measurement")
      ->default_val(opt->min_frame_gap);
  sub->add_option("--rot-bias-gain", opt->rot_bias_gain,
                  "Systematic rotational error per metre travelled (rad/m)")
      ->default_val(opt->rot_bias_gain);
  sub->add_option("--rot-walk-gain", opt->rot_walk_gain,
                  "Rotational random walk (rad per sqrt(m))")
      ->default_val(opt->rot_walk_gain);
  sub->add_option("--trans-bias-gain", opt->trans_bias_gain,
                  "Systematic translational (scale) error per step")
      ->default_val(opt->trans_bias_gain);
  sub->add_option("--trans-walk-gain", opt->trans_walk_gain,
                  "Translational random walk (m per sqrt(m))")
      ->default_val(opt->trans_walk_gain);
  sub->add_option("--bias-correlation-length", opt->bias_correlation_length,
                  "Correlation length of the bias process (m)")
      ->default_val(opt->bias_correlation_length);
  sub->add_option("--mode", opt->mode,
                  "Drift axes: 'full' (all six DoF, the #338 model; "
                  "tilt-dominated on a single room) or 'yaw' (heading about "
                  "world z + horizontal translation only; tilt and height "
                  "stay on the seed, the houses' failure mode)")
      ->default_val(opt->mode)
      ->check(CLI::IsMember({"full", "yaw"}));
  sub->add_flag("--dry-run", opt->dry_run,
                "Measure and report without writing any pose back");
  sub->add_flag("--yes", opt->yes,
                "Confirm that this project is a disposable copy — the poses "
                "are overwritten in place");

  sub->callback([opt, global_opt]() {
    spdlog::trace("calling run_subcommand_edit_perturb_poses");
    rux::finish(run_subcommand_edit_perturb_poses(*opt, *global_opt));
  });
}

int run_subcommand_edit_perturb_poses(
    SubcommandEditPerturbPosesOptions const &opt,
    const RuxOptions &global_opt) {
  fs::path project_path = global_opt.project_db;

  if (!opt.dry_run && !opt.yes) {
    spdlog::error(
        "perturb-poses overwrites every stored pose in '{}'. Pass --yes to "
        "confirm this is a disposable copy, or --dry-run to only measure.",
        project_path.string());
    return RuxError::INVALID_ARGUMENT;
  }

  try {
    reusex::ProjectDB db(project_path);

    reusex::geometry::PoseDriftOptions dopt;
    dopt.seed = opt.seed;
    dopt.drift_scale = opt.drift_scale;
    dopt.target_drift_ratio = opt.target_drift_ratio;
    dopt.min_frame_gap = opt.min_frame_gap;
    dopt.rot_bias_gain = opt.rot_bias_gain;
    dopt.rot_walk_gain = opt.rot_walk_gain;
    dopt.trans_bias_gain = opt.trans_bias_gain;
    dopt.trans_walk_gain = opt.trans_walk_gain;
    dopt.bias_correlation_length = opt.bias_correlation_length;
    dopt.mode = opt.mode == "yaw" ? reusex::geometry::DriftMode::yaw
                                  : reusex::geometry::DriftMode::full;

    spdlog::info("Perturbing poses in {} (seed {}, drift-scale {}, mode {})",
                 project_path.string(), opt.seed, opt.drift_scale, opt.mode);

    const auto r =
        reusex::geometry::perturb_sensor_poses(db, dopt, opt.dry_run);

    spdlog::info("Frames: {}   extent: {:.3f} m   path: {:.3f} m", r.frames,
                 r.trajectory_extent, r.path_length);
    spdlog::info("Drift ratio: {:.4f} (target {:.4f}){}   median distant-pair "
                 "disagreement: {:.4f} m",
                 r.drift_ratio, opt.target_drift_ratio * opt.drift_scale,
                 r.calibrated ? "" : "  [NOT CALIBRATED]",
                 r.median_pair_disagreement);
    spdlog::info("Pose error: median {:.4f} m   max {:.4f} m   final {:.4f} m  "
                 " max rotation {:.3f} deg",
                 r.median_position_error, r.max_position_error,
                 r.final_position_error, r.max_rotation_error);
    if (!r.calibrated && opt.drift_scale > 0.0)
      spdlog::warn("Amplitude search did not reach the requested drift ratio — "
                   "the reported ratio is what was actually applied");
    if (opt.dry_run)
      spdlog::info("Dry run: no poses were written");

    return RuxError::SUCCESS;

  } catch (const std::invalid_argument &e) {
    spdlog::error("perturb-poses failed: {}", e.what());
    return RuxError::INVALID_ARGUMENT;
  } catch (const std::exception &e) {
    spdlog::error("perturb-poses failed: {}", e.what());
    return RuxError::GENERIC;
  }
}
