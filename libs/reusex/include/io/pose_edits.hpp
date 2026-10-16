// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// The pose-edits document, reusex.pose_edits.v1 (reusex-rcv.1.2): reader and
// writer of core::PoseEdits as JSON (tools) or YAML (hand editing, comments
// allowed). Format: docs/design/pose-edits.md, schema
// docs/design/pose-edits.schema.json.
//
// The YAML front end converts to a JSON value first and runs the same strict
// validator, so both formats fail with the same message for the same
// mistake. Every message names the edit ("edit 'm-late-pass'", or
// "edits[3]" when it has no id) and the field.

#include <reusex/core/pose_edits.hpp>

#include <filesystem>
#include <string>
#include <string_view>

namespace reusex::io {

/// The `schema` value this build reads and writes.
inline constexpr std::string_view kPoseEditsSchema = "reusex.pose_edits.v1";

/// Parse a JSON document.
/// @throws std::runtime_error "pose edits: <where>: <what>" on any error: an
///         unknown key or kind, a missing field, a wrong type, a non-finite
///         number, a non-rigid matrix, an unknown DoF name, a duplicate id,
///         a node range with first > last.
core::PoseEdits pose_edits_from_json(std::string_view text);

/// Parse a YAML document (see io/solid_model_json.hpp for how plain scalars
/// are typed); the same validator as pose_edits_from_json.
core::PoseEdits pose_edits_from_yaml(std::string_view text);

/// Serialise: fixed key order, every field of each edit's kind, so
/// `from_json(to_json(e)) == e` exactly.
std::string pose_edits_to_json(const core::PoseEdits &edits, int indent = 2);

/// Serialise as block YAML (same key order; comments are not kept).
std::string pose_edits_to_yaml(const core::PoseEdits &edits);

/// @throws std::runtime_error when @p edits names a project_guid that is not
///         @p project_guid (an empty guid on either side is not checked).
void check_pose_edits_project(const core::PoseEdits &edits,
                              std::string_view project_guid);

/// Read @p path: YAML for `.yaml` / `.yml` (case-insensitive), JSON
/// otherwise; then check_pose_edits_project() against
/// @p expected_project_guid when that is non-empty.
/// @throws std::runtime_error naming the path or the offending edit.
core::PoseEdits read_pose_edits(const std::filesystem::path &path,
                                std::string_view expected_project_guid = {});

/// Write @p edits to @p path (YAML or JSON by extension), creating parent
/// directories.
/// @throws std::runtime_error when the file cannot be written.
void write_pose_edits(const core::PoseEdits &edits,
                      const std::filesystem::path &path);

} // namespace reusex::io
