// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

/// The resource key catalogue (resources/templates design doc §4.3, removed
/// 2026-10-14, see git history): every key a resource
/// (survey part) can carry, and the ONE mapping from a key id to where its
/// value is stored. The API and the frontend only see key ids; the storage
/// field names (leksikon name_en, column display name) stay in here.

#include "reusex/core/ProjectDB.hpp"
#include "reusex/core/materialepas_traits.hpp"
#include "reusex/core/resource_view.hpp"

#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace reusex::core {

/// Where a key's value lives.
enum class KeySource { builtin, leksikon, column };
/// What a write changes: the survey type (all its parts) or the one part.
enum class KeyScope { type, part };
std::string_view to_string(KeyScope); // "type" | "part"

inline constexpr std::string_view kBuiltinCategory = "Kortlægning";
inline constexpr std::string_view kColumnCategory = "Egne felter";
/// The resource view record key (schema v27, core/resource_view.hpp).
inline constexpr std::string_view kViewKey = "sys:view";

struct ResourceKey {
  std::string id; ///< "sys:<name>" | "lex:<leksikon_guid>" | "col:<column id>"
  std::string label;
  std::string category;
  KeyScope scope = KeyScope::part;
  /// text | number | enum | multiselect | boolean | date. A multiselect
  /// value is a JSON array of option strings (the MaterialEPAS storage
  /// form), e.g. ["concrete","steel"].
  std::string data_type = "text";
  std::string unit;                 ///< "" when none
  std::vector<std::string> options; ///< enum / multiselect choices
  bool editable = true;
  KeySource source = KeySource::builtin;
  /// Storage name: the sys name ("quantity"), the leksikon field name_en
  /// ("width_mm") or the user column's display name. Never on the wire.
  std::string field;
  /// Number keys that only take whole numbers (leksikon Integer fields).
  bool integer = false;
};

/// One top-level leksikon property, in leksikon (MaterialEPAS section) order.
struct LeksikonField {
  std::string guid; ///< leksikon GUID == property_definitions.leksikon_guid
  std::string field_name; ///< == property_definitions.name_en
  std::string category;   ///< == property_definitions.category
  traits::PropertyType type = traits::PropertyType::String;
};
/// Nested object-array fields (dangerous substances, emissions) are skipped:
/// they are rows, not one value.
std::vector<LeksikonField> leksikon_fields();
/// Distinct categories of leksikon_fields(), in order.
std::vector<std::string> leksikon_categories();

std::vector<ResourceKey>
builtin_keys(); ///< the 12 sys: keys, spec order (sys:view last)
std::vector<ResourceKey> leksikon_keys();
ResourceKey column_key(const ProjectDB::PropertyDefinition &def);
/// Built-in, then leksikon, then user columns (in the given order) — the
/// "catalogue order" template categories expand in.
std::vector<ResourceKey>
key_catalogue(const std::vector<ProjectDB::PropertyDefinition> &columns);
std::vector<ResourceKey> key_catalogue(const ProjectDB &db);
/// nullptr when @p id is not in @p catalogue.
const ResourceKey *find_key(const std::vector<ResourceKey> &catalogue,
                            std::string_view id);

/// A write the catalogue refuses: unknown key, read-only key, or a value the
/// key's type does not accept. key() names the key for the 400 body.
class KeyValueError : public std::invalid_argument {
    public:
  KeyValueError(std::string key, const std::string &message)
      : std::invalid_argument(message), key_(std::move(key)) {}
  const std::string &key() const noexcept { return key_; }

    private:
  std::string key_;
};

/// Finite number with '.' or one ',' as decimal separator; surrounding
/// blanks and a leading '+' are allowed. nullopt otherwise.
std::optional<double> parse_number(std::string_view text);
/// Shortest round-trip text: 290.0 -> "290", 0.1 -> "0.1".
std::string format_number(double v);

/// Validate @p value for @p key and return it normalised for storage
/// (numbers with '.', booleans "true"/"false", multiselect arrays compact
/// JSON); nullopt means clear. "" clears any key except a built-in text key
/// (stored as ""); an empty multiselect array "[]" clears too.
/// a view (sys:view) must be a parse_view-able JSON object, stored canonical
/// (view_to_json).
/// @throws KeyValueError when the key is read-only, the value does not fit
///         its data_type/options (a multiselect wants a JSON array of
///         distinct options; a leksikon whole number must fit an int),
///         sys:quantity is negative, sys:name or sys:unit is empty, or a
///         clear hits sys:name/quantity/unit/treatment/starred.
std::optional<std::string>
normalise_value(const ResourceKey &key,
                const std::optional<std::string> &value);

} // namespace reusex::core
