// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <map>
#include <string>

/// @file annotate_class_map.hpp
/// @brief Label definitions of a semantic cloud, from the latest
/// `create annotate` run (STANDARDS §3.1).

namespace reusex {
class ProjectDB;
}

namespace reusex::core {

/// Class names of the LATEST `annotate_class_map` pipeline-log entry (the
/// highest log id), re-keyed from API class id to point label (API + 1) — the
/// `label_definitions` a semantic cloud written from the current
/// segmentation images should carry.
///
/// Empty when no annotate run logged a class map, or when the latest entry
/// cannot be parsed (logged at warn); an older run's names are never used in
/// its place, since they may belong to a different prompt list.
std::map<int, std::string> annotated_label_definitions(const ProjectDB &db);

} // namespace reusex::core
