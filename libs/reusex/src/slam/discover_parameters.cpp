// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "slam/discover_parameters.hpp"

#include <nlohmann/json.hpp>

#include <stdexcept>
#include <string>

namespace reusex::slam {

geometry::LoopProposal parse_loop_proposal(std::string_view name) {
  using geometry::LoopProposal;
  if (name == "auto")
    return LoopProposal::automatic;
  if (name == "spatial")
    return LoopProposal::spatial;
  if (name == "appearance")
    return LoopProposal::appearance;
  if (name == "exhaustive")
    return LoopProposal::exhaustive;
  throw std::invalid_argument("unknown loop_proposal '" + std::string(name) +
                              "' (auto, spatial, appearance, exhaustive)");
}

std::string_view to_string(geometry::LoopProposal proposal) {
  using geometry::LoopProposal;
  switch (proposal) {
  case LoopProposal::automatic:
    return "auto";
  case LoopProposal::spatial:
    return "spatial";
  case LoopProposal::appearance:
    return "appearance";
  case LoopProposal::exhaustive:
    return "exhaustive";
  }
  return "auto";
}

void apply_discover_parameters(GraphDiscoverOptions &options,
                               std::string_view parameters) {
  nlohmann::json params = nlohmann::json::object();
  if (!parameters.empty()) {
    params = nlohmann::json::parse(parameters, nullptr,
                                   /*allow_exceptions=*/false);
    if (params.is_discarded() || !params.is_object())
      throw std::invalid_argument(
          "graph_discover parameters must be a JSON object");
  }
  auto get = [&](const char *key, auto fallback) {
    auto it = params.find(key);
    if (it == params.end() || it->is_null())
      return fallback;
    try {
      return it->template get<decltype(fallback)>();
    } catch (const nlohmann::json::exception &e) {
      throw std::invalid_argument(std::string("graph_discover parameter '") +
                                  key + "': " + e.what());
    }
  };
  auto &c = options.candidates;
  c.loop_closure.enable = get("loop_closure", c.loop_closure.enable);
  c.panorama_loops.enable = get("use_panoramas", c.panorama_loops.enable);
  c.loop_edges_file = get("loop_edges_file", c.loop_edges_file);
  const std::string proposal =
      get("loop_proposal", std::string(to_string(c.loop_closure.proposal)));
  c.loop_closure.proposal = parse_loop_proposal(proposal);
  c.loop_closure.min_frame_gap =
      get("loop_min_frame_gap", c.loop_closure.min_frame_gap);
  c.loop_closure.min_match_inliers =
      get("loop_min_inliers", c.loop_closure.min_match_inliers);
  c.loop_closure.pcm = get("pcm", c.loop_closure.pcm);
  options.dry_run = get("dry_run", options.dry_run);
}

} // namespace reusex::slam
