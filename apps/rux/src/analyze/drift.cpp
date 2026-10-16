// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "analyze/drift.hpp"
#include "exit_status.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/solid_model.hpp>
#include <reusex/io/solid_model_json.hpp>

#include <fmt/format.h>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <optional>
#include <tuple>
#include <vector>

namespace fs = std::filesystem;

namespace {

std::string mm(const std::optional<double> &v) {
  return v ? fmt::format("{:.1f} mm", *v * 1000.0) : std::string("n/a");
}

} // namespace

std::string format_drift_summary(const reusex::slam::DriftReport &r, int top) {
  std::string s = fmt::format(
      "poses                 {} ({} of {} frames sampled, {} used, {} "
      "points; passes split by {})\n"
      "noise floor           {} (p90 {})\n"
      "pass-to-pass          {} (p90 {}){}\n"
      "frames                median {} over {} frames with a revisit; {} "
      "above the flag\n"
      "halves                within {} ({} pairs), across {} ({} pairs)\n",
      r.pose_source, r.frames_sampled, r.frames_total, r.frames_used, r.points,
      r.timestamps ? "time" : "frame order", mm(r.noise_floor_m),
      mm(r.noise_floor_p90_m), mm(r.residual_median_m), mm(r.residual_p90_m),
      r.noise_floor_m && r.residual_median_m && *r.noise_floor_m > 0
          ? fmt::format("  = {:.1f}x the floor",
                        *r.residual_median_m / *r.noise_floor_m)
          : std::string(),
      mm(r.frame_residual_median_m), r.revisit_frames, r.frames_flagged,
      mm(r.halves.within_m), r.halves.within_pairs, mm(r.halves.across_m),
      r.halves.across_pairs);

  auto pairs = r.pairs;
  std::sort(pairs.begin(), pairs.end(), [](const auto &a, const auto &b) {
    return a.residual_m != b.residual_m
               ? a.residual_m > b.residual_m
               : std::tie(a.a, a.b) < std::tie(b.a, b.b);
  });
  auto seg_nodes = [&](int i) {
    for (const auto &g : r.segments)
      if (g.index == i)
        return fmt::format("{}-{}", g.first_node, g.last_node);
    return std::string("?");
  };
  s += "worst segment pairs   (segment a into b)\n";
  for (int i = 0; i < std::min<int>(top, static_cast<int>(pairs.size())); ++i)
    s += fmt::format("  {:>2} ({:>11}) into {:>2} ({:>11})  {:6.1f} mm  {} "
                     "points\n",
                     pairs[static_cast<std::size_t>(i)].a,
                     seg_nodes(pairs[static_cast<std::size_t>(i)].a),
                     pairs[static_cast<std::size_t>(i)].b,
                     seg_nodes(pairs[static_cast<std::size_t>(i)].b),
                     pairs[static_cast<std::size_t>(i)].residual_m * 1000.0,
                     pairs[static_cast<std::size_t>(i)].points);

  auto sug = r.suggestions;
  std::sort(sug.begin(), sug.end(), [](const auto &a, const auto &b) {
    const double ga = a.before_m - a.after_m, gb = b.before_m - b.after_m;
    return ga != gb ? ga > gb : a.segment < b.segment;
  });
  s += "segment suggestions   (rigid move against all other segments, by "
       "gain)\n";
  for (const auto &g : sug)
    s += fmt::format("  {:>2} nodes {:>11}  {:5.1f} -> {:5.1f} mm  move "
                     "{:.3f} m (centroid {:.3f} m), {:.2f} deg (yaw {:+.2f})\n",
                     g.segment, fmt::format("{}-{}", g.first_node, g.last_node),
                     g.before_m * 1000.0, g.after_m * 1000.0, g.shift_m,
                     g.centroid_shift_m, g.rotation_deg, g.yaw_deg);

  if (r.crossings) {
    s += fmt::format("crossings             {} of {} plan-wall crossings are "
                     "through a wall ({} steps)\n",
                     r.crossings->through_wall(), r.crossings->crossings.size(),
                     r.crossings->steps);
    for (const auto &x : r.crossings->crossings)
      if (!x.at_opening)
        s += fmt::format("  {} -> {}  wall {} (storey {}) at s = {:.2f} m\n",
                         x.from_node, x.to_node, x.wall_guid, x.storey, x.s_m);
  }
  for (const auto &d : r.doubled_walls)
    s += fmt::format("doubled walls         storey {}: {} of {} patches "
                     "two-layered ({:.2f}), separation median {}\n",
                     d.storey, d.bimodal, d.patches, d.bimodal_share,
                     mm(d.separation_median_m));
  for (const auto &w : r.warnings)
    s += "warning: " + w + "\n";
  return s;
}

void setup_subcommand_analyze_drift(CLI::App &app,
                                    std::shared_ptr<RuxOptions> global_opt) {
  auto opt = std::make_shared<SubcommandAnalyzeDriftOptions>();
  auto *sub = app.add_subcommand(
      "drift", "Find where the stored poses disagree (pass-to-pass drift)");

  sub->footer(R"(
DESCRIPTION:
  Back-projects the stored depth of every sampled sensor frame with its pose
  and measures, GT-free, where the capture disagrees with itself
  (docs/research/visual-drift-loop.md §3):

  - pass-to-pass residual: each point against the nearest point of a frame
    captured more than --gap-s apart (point-to-plane), next to the same
    measure against frames 1-3 apart: the sensor noise floor.
  - the segment x segment matrix (--segments equal time segments, each pair
    matched only against each other) and its half-vs-half summary: a 1-2 deg
    pass break that the pooled numbers cannot see shows here.
  - per-segment suggestions: each segment rigidly aligned (robust
    point-to-plane ICP) to all the others, with the residual before/after.
  - a per-cell map of the residual (the drift heatmap, JSON "cells").
  - with --plan: camera steps that cross a plan wall away from its openings
    (drift OR a plan error: a place to look).
  - doubled-wall patches on the fused cloud (needs storeys: the plan's or
    the project's).

EXAMPLES:
  rux -p scan.rux analyze drift
  rux -p scan.rux analyze drift --plan plan.yaml --json -o drift.json
  rux -p scan.rux analyze drift --initial --frame-step 4 --depth-stride 4

NOTES:
  - Read-only. The residuals depend on the sampling density: compare runs
    only at the same --frame-step and --depth-stride.
  - A capture without timestamps splits passes by node order
    (--gap-frames).
  - "delta" in a suggestion is a world-frame move: new pose = delta * pose
    for every frame of the segment.
)");

  auto &d = opt->diag;
  sub->add_option("--plan", opt->plan_path,
                  "Reference plan (solid-model JSON or YAML) for the "
                  "through-wall crossings")
      ->check(CLI::ExistingFile);
  sub->add_flag("--initial", opt->initial,
                "Use the initial poses (initial_transform) instead of the "
                "current ones");
  sub->add_option("--segments", d.segments, "Equal time segments")
      ->default_val(d.segments)
      ->check(CLI::Range(1, 1000));
  sub->add_option("--frame-step", d.frame_step, "Use every Nth frame")
      ->default_val(d.frame_step)
      ->check(CLI::PositiveNumber);
  sub->add_option("--depth-stride", d.depth_stride,
                  "Sample every Nth depth pixel")
      ->default_val(d.depth_stride)
      ->check(CLI::PositiveNumber);
  sub->add_option("--max-depth", d.max_depth_m, "Drop farther returns [m]")
      ->default_val(d.max_depth_m)
      ->check(CLI::PositiveNumber);
  sub->add_option("--gap-s", d.gap_s,
                  "Frames this far apart in time are different passes [s]")
      ->default_val(d.gap_s)
      ->check(CLI::PositiveNumber);
  sub->add_option("--gap-frames", d.gap_frames,
                  "... or this far apart in node order (no timestamps)")
      ->default_val(d.gap_frames)
      ->check(CLI::PositiveNumber);
  sub->add_option("--match-radius", d.match_radius_m,
                  "Segment-pair nearest-neighbour gate [m]")
      ->default_val(d.match_radius_m)
      ->check(CLI::PositiveNumber);
  sub->add_option("--icp-max-corr", d.icp_max_corr_m,
                  "Segment ICP correspondence gate [m]")
      ->default_val(d.icp_max_corr_m)
      ->check(CLI::PositiveNumber);
  sub->add_option("--huber", d.huber_m, "Segment ICP Huber threshold [m]")
      ->default_val(d.huber_m)
      ->check(CLI::PositiveNumber);
  sub->add_option("--cell", d.cell_m, "Heatmap cell size [m]")
      ->default_val(d.cell_m)
      ->check(CLI::PositiveNumber);
  sub->add_flag("--no-doubled-walls", "Skip the doubled-wall patches");
  sub->add_option("--top", opt->top, "Rows of the worst-pairs table")
      ->default_val(opt->top)
      ->check(CLI::NonNegativeNumber);
  sub->add_flag("--json", opt->json, "Print the JSON report to stdout");
  sub->add_option("-o,--output", opt->output_path,
                  "Also write the JSON report to this file");

  sub->callback([opt, global_opt, sub]() {
    spdlog::trace("calling analyze drift subcommand");
    opt->diag.doubled_walls = sub->count("--no-doubled-walls") == 0;
    opt->diag.pose_source = opt->initial
                                ? reusex::slam::DriftPoseSource::initial
                                : reusex::slam::DriftPoseSource::current;
    rux::finish(run_subcommand_analyze_drift(*opt, *global_opt));
  });
}

int run_subcommand_analyze_drift(SubcommandAnalyzeDriftOptions const &opt,
                                 const RuxOptions &global_opt) {
  using namespace reusex;
  std::optional<core::SolidModel> plan;
  if (!opt.plan_path.empty()) {
    try {
      plan = io::solid_model_from_file(opt.plan_path);
    } catch (const std::exception &e) {
      spdlog::error("Plan '{}': {}", opt.plan_path, e.what());
      return RuxError::INVALID_ARGUMENT;
    }
  }
  if (!fs::exists(global_opt.project_db)) {
    spdlog::error("Project '{}' not found (pass -p)",
                  global_opt.project_db.string());
    return RuxError::INVALID_ARGUMENT;
  }
  try {
    const ProjectDB db(global_opt.project_db, /*readOnly=*/true);
    const auto report =
        slam::diagnose_drift(db, opt.diag, plan ? &*plan : nullptr);
    const std::string json = slam::drift_report_to_json(report);
    if (opt.json)
      std::cout << json << '\n';
    else
      std::cout << format_drift_summary(report, opt.top);
    if (!opt.output_path.empty()) {
      std::ofstream out(opt.output_path);
      if (!out) {
        spdlog::error("Cannot write to '{}'", opt.output_path);
        return RuxError::IO;
      }
      out << json << '\n';
    }
  } catch (const std::invalid_argument &e) {
    spdlog::error("analyze drift: {}", e.what());
    return RuxError::INVALID_ARGUMENT;
  } catch (const std::exception &e) {
    spdlog::error("analyze drift: {}", e.what());
    return RuxError::GENERIC;
  }
  return RuxError::SUCCESS;
}
