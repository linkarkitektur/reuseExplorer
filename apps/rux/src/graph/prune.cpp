// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "graph/prune.hpp"
#include "exit_status.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/slam/graph_prune.hpp>

#include <fmt/format.h>
#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>
#include <exception>
#include <stdexcept>
#include <utility>

namespace {

std::pair<int, int> parse_pair(const std::string &text) {
  const auto comma = text.find(',');
  if (comma == std::string::npos)
    throw std::invalid_argument("--pair expects 'A,B', got '" + text + "'");
  try {
    return {std::stoi(text.substr(0, comma)),
            std::stoi(text.substr(comma + 1))};
  } catch (const std::exception &) {
    throw std::invalid_argument("--pair expects 'A,B', got '" + text + "'");
  }
}

std::string fmt_metric(double v) {
  return std::isfinite(v) ? fmt::format("{:.3f}", v) : std::string("-");
}

} // namespace

void setup_subcommand_graph_prune(CLI::App &parent,
                                  std::shared_ptr<RuxOptions> global_opt) {
  auto opt = std::make_shared<SubcommandGraphPruneOptions>();
  auto *sub = parent.add_subcommand(
      "prune", "Reject pose-graph edges by residual, deviation, type, source");
  sub->footer(R"(
DESCRIPTION:
  Marks matching edges 'rejected'. Rejected edges stay in the project as
  tombstones: 'rux optimize' ignores them and 'rux graph discover' never adds
  them again. Criteria AND together; at least one is required. Confirmed
  edges are spared unless --include-confirmed.

EXAMPLES:
  rux graph prune --residual-above 50 --type loop_closure --dry-run
  rux graph prune --deviation-above 0.5
  rux graph prune --pair 412,980
)");
  sub->add_option("--residual-above", opt->residual_above,
                  "Residual (0.5 x whitened chi^2) greater than this");
  sub->add_option("--deviation-above", opt->deviation_above,
                  "Deviation from the solved poses greater than this (m)");
  sub->add_option("--deviation-deg-above", opt->deviation_deg_above,
                  "Rotational deviation greater than this (deg)");
  sub->add_option("--type", opt->type,
                  "Edge type: odometry, loop_closure or panorama");
  sub->add_option("--source", opt->source,
                  "Edge source: rtabmap, odometry_init, orb, panorama, file, "
                  "icp, manual or legacy");
  sub->add_option("--pair", opt->pair, "Frame pair 'A,B' (either order)");
  sub->add_flag("--include-confirmed", opt->include_confirmed,
                "Also reject confirmed edges");
  sub->add_flag("--dry-run", opt->dry_run,
                "List the matching edges without rejecting them");
  sub->callback([opt, global_opt]() {
    rux::finish(run_subcommand_graph_prune(*opt, *global_opt));
  });
}

int run_subcommand_graph_prune(const SubcommandGraphPruneOptions &opt,
                               const RuxOptions &global_opt) {
  try {
    reusex::slam::PruneFilter f;
    f.residual_above = opt.residual_above;
    f.deviation_above_m = opt.deviation_above;
    f.deviation_above_deg = opt.deviation_deg_above;
    if (!opt.type.empty())
      f.edge_type = opt.type;
    if (!opt.source.empty())
      f.source = opt.source;
    if (!opt.pair.empty())
      f.pair = parse_pair(opt.pair);
    f.include_confirmed = opt.include_confirmed;
    f.dry_run = opt.dry_run;

    reusex::ProjectDB db(global_opt.project_db);
    nlohmann::json params = {{"dry_run", opt.dry_run},
                             {"include_confirmed", opt.include_confirmed}};
    const int logId = db.log_pipeline_start("graph_prune", params.dump());
    try {
      const auto r = reusex::slam::prune_pose_graph(db, f);
      db.log_pipeline_end(logId, true);
      for (const auto &e : r.matched)
        spdlog::info("  {:>6} {:>6}  {:<13} {:<14} {:<9} residual {} "
                     "deviation {} m",
                     e.from_node_id, e.to_node_id, e.edge_type, e.source,
                     e.status, fmt_metric(e.residual),
                     fmt_metric(e.deviation_m.value_or(NAN)));
      spdlog::info("{} edges matched; {} rejected{}", r.matched.size(),
                   r.rejected, opt.dry_run ? " (dry run)" : "");
      return RuxError::SUCCESS;
    } catch (const std::exception &e) {
      db.log_pipeline_end(logId, false, e.what());
      throw;
    }
  } catch (const std::invalid_argument &e) {
    spdlog::error("graph prune: {}", e.what());
    return RuxError::INVALID_ARGUMENT;
  } catch (const std::exception &e) {
    spdlog::error("graph prune failed: {}", e.what());
    return RuxError::GENERIC;
  }
}
