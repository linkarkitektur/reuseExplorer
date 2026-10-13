// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "global-params.hpp"

#include <CLI/CLI.hpp>
#include <memory>

/// `rux graph` — edit the stored pose graph (spec 2026-10-13 §4.3).
void setup_subcommand_graph(CLI::App &app,
                            std::shared_ptr<RuxOptions> global_opt);
