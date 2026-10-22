// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

/// Resources
/// (resources/templates design doc §4, removed 2026-10-14, see git history): a
/// resource is one survey part; its values are addressed by key id
/// (core/resource_keys.hpp) and routed by scope — the type, the part, or
/// the part's own passport, created on the first passport write.

#include "reusex/core/ProjectDB.hpp"
#include "reusex/core/resource_keys.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace reusex::core {

struct ResourceValue {
  std::string key;
  std::optional<std::string> value; ///< nullopt: not set
};
struct Resource {
  std::string code;
  int64_t type_id = 0;
  bool manual = false; ///< added by hand: no instance behind it
  std::vector<ResourceValue> values;
  /// The part's view record (schema v27), filled whatever `keys` selects —
  /// the GUI needs it even when a template leaves sys:view out.
  std::optional<ResourceView> view{};
  /// The part is marked as an error (survey_parts.dismissed, schema v36).
  /// Still listed so the GUI can restore it, but kept out of every delivered
  /// output (CSV export, PDF report). Kept LAST for aggregate initializers.
  bool dismissed = false;
};
/// The value stored under @p key in @p r, or nullopt (absent or unset).
std::optional<std::string> value_of(const Resource &r, std::string_view key);

/// One per survey part, by code, dismissed parts included (see
/// Resource::dismissed). With @p keys, `values` holds exactly those
/// keys in that order (null when unset or unknown); without, every
/// built-in key plus each catalogue key the part's passport stores, in
/// catalogue order. Passport fields with no catalogue key are skipped, and
/// stored blanks ("", "[]", a leksikon TriState's "unknown") read as unset.
std::vector<Resource> list_resources(
    const ProjectDB &db,
    const std::optional<std::vector<std::string>> &keys = std::nullopt);
/// @throws std::out_of_range when @p code is unknown.
Resource
resource(const ProjectDB &db, std::string_view code,
         const std::optional<std::vector<std::string>> &keys = std::nullopt);

struct ResourceWrite {
  std::string key;
  std::optional<std::string> value; ///< nullopt clears
};
struct ResourcePatchResult {
  Resource resource;
  /// The type's other parts, when a type-scoped key was written (they
  /// changed too); empty otherwise.
  std::vector<Resource> siblings;
};
/// Validate every write (normalise_value), then apply them in one
/// transaction: type-scoped keys to the survey type, part-scoped built-ins
/// to the part, leksikon/column keys to the part's passport (created on
/// the first non-null write). Clearing an absent value succeeds. A frame
/// `sys:view` without a region, on an instance-backed part, is stored with
/// the instance's padded region in that frame (complete_view_region). A
/// resource keeps its image once it has one: clearing a set `sys:view` is
/// refused (KeyValueError); clearing an already-absent one stays a no-op
/// success — only changing it is allowed.
/// @throws KeyValueError (nothing written) — also when one key appears
///         twice in @p writes, or `sys:view` is cleared while set —,
///         std::out_of_range (no part), std::runtime_error (a cloud cannot
///         be loaded while filling a frame view's region).
ResourcePatchResult patch_resource(
    ProjectDB &db, std::string_view code,
    const std::vector<ResourceWrite> &writes,
    const std::optional<std::vector<std::string>> &keys = std::nullopt);

/// The leksikon field a hand-added resource's `name` is stored under.
inline constexpr std::string_view kDesignationField = "designation";
/// A manual part with the next RX code; @p name becomes its designation.
/// @throws std::out_of_range (no type), std::invalid_argument (empty name).
Resource create_resource(ProjectDB &db, int64_t type_id,
                         const std::optional<std::string> &name = std::nullopt);

/// Delete a part — manual or instance-backed — in one transaction.
/// An instance-backed part's instance guid is tombstoned
/// (ProjectDB::dismiss_instance) so sync_survey does not re-create it, and
/// its instance's passport link is dropped when it points at the part's
/// passport. The passport is then deleted unless something else (another
/// part or instance) still links it; a kept passport is logged at warn.
/// @throws std::out_of_range for an unknown code.
void delete_resource(ProjectDB &db, std::string_view code);
/// Delete a survey type and every part in it, with delete_resource's rules
/// for each part (tombstones, passports), in one transaction. Its sample
/// links cascade.
/// @throws std::out_of_range for an unknown type.
struct SurveyTypeDeletion {
  std::size_t parts_deleted = 0;
  std::size_t instances_dismissed = 0; ///< scan-backed parts, now tombstoned
};
SurveyTypeDeletion delete_survey_type(ProjectDB &db, int64_t type_id);

/// Sparse update of a user column definition.
struct ColumnPatch {
  std::optional<std::string> name, type;
  std::optional<std::vector<std::string>> options;
  std::optional<int> sort_order, width;
};
/// Add a user column (def.id is ignored and returned filled in).
/// @throws NameConflictError when a user column or a leksikon field already
///         has that name, or any passport already stores values under it
///         (left by a deleted column — never inherited silently),
///         std::invalid_argument for an empty name.
ProjectDB::PropertyDefinition create_column(ProjectDB &db,
                                            ProjectDB::PropertyDefinition def);
/// Delete a user column and, in the same transaction, every value stored
/// under its name (logged at warn with the count), so the name can be used
/// again. @throws std::out_of_range (no column).
void delete_column(ProjectDB &db, const std::string &id);
/// A rename moves the stored values in the same transaction.
/// @throws std::out_of_range (no column), NameConflictError (name taken, or
///         any passport already stores values under it — even when this
///         column holds none), std::invalid_argument (empty).
ProjectDB::PropertyDefinition
update_column(ProjectDB &db, const std::string &id, const ColumnPatch &patch);

} // namespace reusex::core
