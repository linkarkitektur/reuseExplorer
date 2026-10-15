// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <string>
#include <string_view>

/// @file label_encoding.hpp
/// @brief Which label clouds are semantic, and the `label_encoding` stamp
/// they carry (STANDARDS §3.1, schema v29, decision D8).
///
/// A semantic label cloud stores point label = API class + 1 and records it as
/// `"label_encoding": "api_plus_one"` in its `parameters` JSON. A semantic
/// cloud without the stamp in a v29+ project was written by a pre-v29 binary
/// after the migration ran: its class ids are probably one too low. Readers
/// warn (warn_if_unstamped_semantic_cloud()) and `rux validate` flags it
/// (check_unstamped_semantic_clouds() in core/validate.hpp).

namespace reusex {
class ProjectDB;
}

namespace reusex::core {

/// The v29 rule: a Label cloud is semantic when a semantic stage wrote it
/// (`reconstruct`, `project_labels`) or it has the default semantic name
/// `labels` — unless a structural stage (`segment_planes`, `segment_rooms`,
/// `segment_instances`) wrote it. Imported label fields (`import_ply`) are
/// not semantic by this rule: their encoding is the source's.
bool is_semantic_label_cloud(std::string_view name, std::string_view stage);

/// @return true when @p params_json is a JSON object whose
/// `label_encoding` is `api_plus_one`. Empty or unparsable JSON is false.
bool has_api_plus_one_encoding(std::string_view params_json);

/// @p params_json with `label_encoding` set to `api_plus_one`. Empty or
/// non-object input yields `{"label_encoding":"api_plus_one"}`.
std::string stamp_label_encoding(std::string_view params_json);

/// @return true when @p cloud — a Label cloud the caller reads as semantic —
/// lacks the stamp in a project at schema >= 29. False for a missing cloud,
/// another point type, a cloud a structural stage wrote, or an older (not yet
/// migrated, e.g. opened read-only) project.
bool is_unstamped_semantic_cloud(const ProjectDB &db, std::string_view cloud);

/// Call where a cloud is read as the semantic cloud: logs a warning naming
/// the cloud, its stage and the consequence when is_unstamped_semantic_cloud().
/// @return is_unstamped_semantic_cloud(db, cloud).
bool warn_if_unstamped_semantic_cloud(const ProjectDB &db,
                                      std::string_view cloud);

} // namespace reusex::core
