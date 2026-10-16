// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "analyze/poses.hpp"
#include "exit_status.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/io/trajectory.hpp>

#include <fmt/format.h>
#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <fstream>
#include <iostream>
#include <stdexcept>

namespace {

using reusex::slam::PoseMetricsOptions;

PoseMetricsOptions::Align align_from_string(const std::string &s) {
  if (s == "none")
    return PoseMetricsOptions::Align::none;
  if (s == "se3")
    return PoseMetricsOptions::Align::se3;
  if (s == "sim3")
    return PoseMetricsOptions::Align::sim3;
  // CLI::IsMember already restricts --align to these three values.
  throw std::invalid_argument("align_from_string: unknown alignment '" + s +
                              "'");
}

} // namespace

std::string format_pose_metrics_summary(const reusex::slam::PoseMetrics &m) {
  return fmt::format(
      "matched poses        {}\n"
      "ATE RMSE / median / max   {:.4g} m / {:.4g} m / {:.4g} m\n"
      "ATE rotation RMSE     {:.4g} deg\n"
      "RPE (1 delta step)    {:.4g} m / {:.4g} deg\n"
      "RPE (path length)     {:.4g} m / {:.4g} deg\n"
      "alignment scale       {:.6g}\n"
      "segment ATE RMSE [m]  {}\n",
      m.matched, m.ate_rmse_m, m.ate_median_m, m.ate_max_m, m.rot_rmse_deg,
      m.rpe_trans_frames_m, m.rpe_rot_frames_deg, m.rpe_trans_m, m.rpe_rot_deg,
      m.scale, [&] {
        std::string s;
        for (double v : m.segment_ate_m) {
          if (!s.empty())
            s += ", ";
          s += fmt::format("{:.4g}", v);
        }
        return s;
      }());
}

namespace {

nlohmann::json to_json(const reusex::slam::PoseMetrics &m,
                       const SubcommandAnalyzePosesOptions &opt) {
  nlohmann::json j;
  j["matched"] = m.matched;
  j["ate_rmse_m"] = m.ate_rmse_m;
  j["ate_median_m"] = m.ate_median_m;
  j["ate_max_m"] = m.ate_max_m;
  j["rot_rmse_deg"] = m.rot_rmse_deg;
  j["rpe_trans_m"] = m.rpe_trans_m;
  j["rpe_rot_deg"] = m.rpe_rot_deg;
  j["rpe_trans_frames_m"] = m.rpe_trans_frames_m;
  j["rpe_rot_frames_deg"] = m.rpe_rot_frames_deg;
  j["segment_ate_m"] = m.segment_ate_m;
  j["scale"] = m.scale;
  j["options"] = {{"align", opt.align},
                  {"rpe_distance_m", opt.metrics.rpe_distance_m},
                  {"rpe_frames", opt.metrics.rpe_frames},
                  {"segments", opt.metrics.segments}};
  return j;
}

} // namespace

void setup_subcommand_analyze_poses(CLI::App &app,
                                    std::shared_ptr<RuxOptions> global_opt) {
  auto opt = std::make_shared<SubcommandAnalyzePosesOptions>();
  auto *sub = app.add_subcommand(
      "poses", "Score a trajectory (or the project's own poses) against a "
               "reference trajectory");

  sub->footer(R"(
DESCRIPTION:
  Compares an estimated trajectory against a reference, both TUM format
  (rux export trajectory), and reports:

  - ATE: RMSE / median / max translation error and RMS rotation error, over
    poses matched by timestamp (--align none by default, since a reference
    frozen from the same project shares its gauge; se3/sim3 run a Umeyama
    alignment first when it does not)
  - RPE: relative pose error at a fixed matched-index gap (--rpe-frames) and
    at a fixed path length travelled (--rpe-distance), which need no
    alignment (a shared global offset cancels)
  - per-segment ATE RMSE (--segments contiguous chunks of the matched
    sequence), under the one whole-trajectory alignment

  With no --estimate, the project's own current poses (or, with --initial,
  initial_transform) are exported in memory and scored, mirroring
  'rux export trajectory' without writing a file.

EXAMPLES:
  rux -p scan.rux analyze poses --reference ref.tum
  rux -p scan.rux analyze poses --reference ref.tum --estimate other.tum
  rux -p scan.rux analyze poses --reference seed.tum --align se3
  rux -p scan.rux analyze poses --reference ref.tum --json -o report.json

NOTES:
  - Trajectories are matched by timestamp, not node id (docs comment in
    core/trajectory.hpp): a TUM file carries only a timestamp column, so two
    trajectories can only be matched by that column's value. A project's own
    export and 'rux edit perturb-poses' both leave timestamps untouched, so
    this always matches the frames you expect.
  - A metric is 0 when fewer than 2 poses matched; a warning on stderr says
    why (STANDARDS §5).
)");

  sub->add_option("--reference", opt->reference_path,
                  "Reference trajectory (TUM)")
      ->required()
      ->check(CLI::ExistingFile);
  sub->add_option("--estimate", opt->estimate_path,
                  "Estimated trajectory (TUM); default: the project's own "
                  "current poses")
      ->check(CLI::ExistingFile);
  sub->add_flag("--initial", opt->initial,
                "With no --estimate, score initial_transform instead of the "
                "current pose");
  sub->add_flag("--json", opt->json, "Print the JSON report to stdout");
  sub->add_option("-o,--output", opt->output_path,
                  "Also write the JSON report to this file");

  sub->add_option("--align", opt->align,
                  "Alignment before ATE: none (default, shared gauge), se3 "
                  "(rigid Umeyama), sim3 (+ scale)")
      ->default_val(opt->align)
      ->check(CLI::IsMember({"none", "se3", "sim3"}));
  sub->add_option("--rpe-distance", opt->metrics.rpe_distance_m,
                  "RPE delta as a path length travelled [m]")
      ->default_val(opt->metrics.rpe_distance_m)
      ->check(CLI::PositiveNumber);
  sub->add_option("--rpe-frames", opt->metrics.rpe_frames,
                  "RPE delta as a matched-index gap")
      ->default_val(opt->metrics.rpe_frames)
      ->check(CLI::PositiveNumber);
  sub->add_option("--segments", opt->metrics.segments,
                  "Number of contiguous chunks for the per-segment ATE")
      ->default_val(opt->metrics.segments)
      ->check(CLI::PositiveNumber);

  sub->callback([opt, global_opt]() {
    spdlog::trace("calling analyze poses subcommand");
    rux::finish(run_subcommand_analyze_poses(*opt, *global_opt));
  });
}

int run_subcommand_analyze_poses(SubcommandAnalyzePosesOptions const &opt,
                                 const RuxOptions &global_opt) {
  using namespace reusex;
  try {
    core::Trajectory reference;
    try {
      reference = io::read_tum(opt.reference_path);
    } catch (const std::exception &e) {
      spdlog::error("Reference trajectory '{}': {}", opt.reference_path,
                    e.what());
      return RuxError::INVALID_ARGUMENT;
    }

    core::Trajectory estimate;
    if (!opt.estimate_path.empty()) {
      try {
        estimate = io::read_tum(opt.estimate_path);
      } catch (const std::exception &e) {
        spdlog::error("Estimate trajectory '{}': {}", opt.estimate_path,
                      e.what());
        return RuxError::INVALID_ARGUMENT;
      }
    } else {
      if (!fs::exists(global_opt.project_db)) {
        spdlog::error(
            "Project '{}' not found (pass -p, or --estimate a TUM file)",
            global_opt.project_db.string());
        return RuxError::INVALID_ARGUMENT;
      }
      ProjectDB db(global_opt.project_db, /*readOnly=*/true);
      estimate = io::trajectory_from_project(db, opt.initial);
    }

    reusex::slam::PoseMetricsOptions metrics_opt = opt.metrics;
    metrics_opt.align = align_from_string(opt.align);

    reusex::slam::PoseMetrics metrics;
    try {
      metrics =
          reusex::slam::compare_trajectories(estimate, reference, metrics_opt);
    } catch (const std::invalid_argument &e) {
      spdlog::error("analyze poses: invalid option: {}", e.what());
      return RuxError::INVALID_ARGUMENT;
    }

    const nlohmann::json j = to_json(metrics, opt);
    if (opt.json)
      std::cout << j.dump(2) << '\n';
    else
      std::cout << format_pose_metrics_summary(metrics);
    if (!opt.output_path.empty()) {
      std::ofstream out(opt.output_path);
      if (!out) {
        spdlog::error("Cannot write to '{}'", opt.output_path);
        return RuxError::IO;
      }
      out << j.dump(2) << '\n';
      spdlog::info("Report written to {}", opt.output_path);
    }
    return RuxError::SUCCESS;

  } catch (const std::exception &e) {
    spdlog::error("Pose analysis failed: {}", e.what());
    return RuxError::GENERIC;
  }
}
