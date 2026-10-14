// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// The graph_discover stage's JSON parameters -> GraphDiscoverOptions, in ONE
// place (mirrors slam/optimize_parameters.hpp): `rux graph discover`, the Qt
// client's in-process stage and ruxd read them through this function.

#include <reusex/slam/graph_discover.hpp>

#include <string_view>

namespace reusex::slam {

/// Keys: loop_closure, use_panoramas, loop_edges_file, loop_proposal,
/// loop_min_frame_gap, loop_min_inliers, pcm, dry_run. Absent or null keys
/// keep the value already in @p options; unknown keys are ignored.
/// @throws std::invalid_argument when @p parameters is not a JSON object, a
///         known key has the wrong type, or loop_proposal is unknown.
void apply_discover_parameters(GraphDiscoverOptions &options,
                               std::string_view parameters);

/// "auto" | "spatial" | "appearance" | "exhaustive".
/// @throws std::invalid_argument for anything else.
geometry::LoopProposal parse_loop_proposal(std::string_view name);
std::string_view to_string(geometry::LoopProposal proposal);

} // namespace reusex::slam
