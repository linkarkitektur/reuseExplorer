// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "reusex/core/resource_keys.hpp"

#include "reusex/core/materialepas_enums.hpp"
#include "reusex/core/materialepas_json_export.hpp"
#include "reusex/core/survey.hpp"

#include <fmt/format.h>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <cctype>
#include <charconv>
#include <cmath>
#include <limits>
#include <set>

namespace reusex::core {
namespace {
using traits::PropertyType;

/// The property_definitions.category ProjectDB files each section under
/// (ProjectDB.cpp, ensureAllPropertyDefinitions). The test
/// ResourceKeys_LeksikonCategories_MatchPropertyDefinitions pins the two.
const char *category_of(const traits::PropertyDescriptor *props) {
  using namespace traits;
  if (props == PropertyTraits<Owner>::properties())
    return "Owner";
  if (props == PropertyTraits<ConstructionItemDescription>::properties())
    return "Description";
  if (props == PropertyTraits<ProductInformation>::properties())
    return "Product";
  if (props == PropertyTraits<Certifications>::properties())
    return "Certifications";
  if (props == PropertyTraits<Dimensions>::properties())
    return "Dimensions";
  if (props == PropertyTraits<Condition>::properties())
    return "Condition";
  if (props == PropertyTraits<Pollution>::properties())
    return "Pollution";
  if (props == PropertyTraits<EnvironmentalPotential>::properties())
    return "Environmental";
  if (props == PropertyTraits<FireProperties>::properties())
    return "Fire";
  if (props == PropertyTraits<History>::properties())
    return "History";
  return nullptr;
}

std::string humanize(std::string_view field) {
  std::string out(field);
  std::replace(out.begin(), out.end(), '_', ' ');
  if (!out.empty())
    out[0] =
        static_cast<char>(std::toupper(static_cast<unsigned char>(out[0])));
  return out;
}

std::string unit_of(std::string_view field) {
  auto ends = [&](std::string_view s) {
    return field.size() >= s.size() &&
           field.substr(field.size() - s.size()) == s;
  };
  if (ends("_mm"))
    return "mm";
  if (ends("_m3"))
    return "m³";
  if (ends("_m2"))
    return "m²";
  if (ends("_kg"))
    return "kg";
  return "";
}

bool is_iso_date(std::string_view v) {
  if (v.size() != 10 || v[4] != '-' || v[7] != '-')
    return false;
  for (std::size_t i : {0, 1, 2, 3, 5, 6, 8, 9})
    if (!std::isdigit(static_cast<unsigned char>(v[i])))
      return false;
  const int month = (v[5] - '0') * 10 + (v[6] - '0');
  const int day = (v[8] - '0') * 10 + (v[9] - '0');
  return month >= 1 && month <= 12 && day >= 1 && day <= 31;
}

std::vector<std::string> material_options() {
  std::vector<std::string> out;
  for (const auto name : material_names())
    out.emplace_back(name);
  return out;
}

/// A multiselect value: a JSON array of distinct option strings, returned
/// compact; an empty array is nullopt (a clear).
std::optional<std::string> normalise_multiselect(const ResourceKey &key,
                                                 const std::string &v) {
  const auto j = nlohmann::json::parse(v, nullptr, /*allow_exceptions=*/false);
  auto refuse = [&](const std::string &why) {
    return KeyValueError(key.id,
                         "'" + key.id + "' " + why + ", got '" + v + "'");
  };
  if (j.is_discarded() || !j.is_array())
    throw refuse("must be a JSON array of strings");
  std::set<std::string> seen;
  for (const auto &item : j) {
    if (!item.is_string())
      throw refuse("must be a JSON array of strings");
    const auto &s = item.get_ref<const std::string &>();
    if (std::find(key.options.begin(), key.options.end(), s) ==
        key.options.end())
      throw KeyValueError(key.id, "'" + key.id + "' has no value '" + s + "'");
    if (!seen.insert(s).second)
      throw KeyValueError(key.id, "'" + key.id + "' lists '" + s + "' twice");
  }
  if (j.empty())
    return std::nullopt;
  return j.dump();
}

/// Built-in keys whose column is NOT NULL with no meaningful empty value.
/// R-P7: "" written to one of these on a text key is a 400-class rejection,
/// same as a missing (nullopt) write — not a silent clear.
bool clearable(const ResourceKey &k) {
  static const std::set<std::string, std::less<>> required{
      "sys:name", "sys:quantity", "sys:unit", "sys:treatment", "sys:starred"};
  return required.find(k.id) == required.end();
}
} // namespace

std::string_view to_string(KeyScope s) {
  return s == KeyScope::type ? "type" : "part";
}

std::vector<LeksikonField> leksikon_fields() {
  std::vector<LeksikonField> out;
  for (const auto &sd : json_export::section_descriptors()) {
    const char *category = category_of(sd.properties);
    if (!category)
      throw std::logic_error(std::string("leksikon section '") + sd.name_en +
                             "' has no property_definitions category");
    for (std::size_t i = 0; i < sd.property_count; ++i) {
      const auto &p = sd.properties[i];
      if (p.type == PropertyType::ObjectArray)
        continue;
      out.push_back({p.leksikon_guid, p.field_name, category, p.type});
    }
  }
  return out;
}

std::vector<std::string> leksikon_categories() {
  std::vector<std::string> out;
  for (const auto &f : leksikon_fields())
    if (std::find(out.begin(), out.end(), f.category) == out.end())
      out.push_back(f.category);
  return out;
}

std::vector<ResourceKey> builtin_keys() {
  auto k = [](const char *name, const char *label, KeyScope scope,
              const char *type, bool editable = true) {
    ResourceKey key;
    key.id = std::string("sys:") + name;
    key.label = label;
    key.category = std::string(kBuiltinCategory);
    key.scope = scope;
    key.data_type = type;
    key.editable = editable;
    key.source = KeySource::builtin;
    key.field = name;
    return key;
  };
  std::vector<ResourceKey> out{
      k("name", "Betegnelse", KeyScope::type, "text"),
      k("quantity", "Mængde", KeyScope::part, "number"),
      k("unit", "Enhed", KeyScope::type, "text"),
      k("eak", "EAK", KeyScope::type, "text"),
      k("bim7aa", "BIM7AA", KeyScope::type, "text"),
      k("treatment", "Behandling", KeyScope::type, "enum"),
      k("environment", "Miljøstatus", KeyScope::type, "enum", false),
      k("room", "Rum", KeyScope::part, "text"),
      k("mass_t", "Tons", KeyScope::type, "number"),
      k("note", "Note", KeyScope::part, "text"),
      k("starred", "Vigtig", KeyScope::part, "boolean"),
      k("view", "Ressourcebillede", KeyScope::part, "view"),
  };
  for (auto t :
       {Treatment::bevaring, Treatment::genbrug, Treatment::genanvendelse,
        Treatment::nyttiggoerelse, Treatment::bortskaffelse})
    out[5].options.emplace_back(to_string(t));
  for (auto e :
       {EnvironmentStatus::ren_screening, EnvironmentStatus::afventer,
        EnvironmentStatus::forurenet, EnvironmentStatus::ren_proevesvar})
    out[6].options.emplace_back(to_string(e));
  out[8].unit = "t";
  return out;
}

std::vector<ResourceKey> leksikon_keys() {
  std::vector<ResourceKey> out;
  for (const auto &f : leksikon_fields()) {
    ResourceKey k;
    k.id = "lex:" + f.guid;
    k.label = humanize(f.field_name);
    k.category = f.category;
    k.scope = KeyScope::part;
    k.source = KeySource::leksikon;
    k.field = f.field_name;
    k.unit = unit_of(f.field_name);
    switch (f.type) {
    case PropertyType::Integer:
      k.data_type = "number";
      k.integer = true;
      break;
    case PropertyType::Double:
      k.data_type = "number";
      break;
    case PropertyType::Boolean:
      k.data_type = "boolean";
      break;
    case PropertyType::TriState:
      k.data_type = "enum";
      k.options = {"yes", "no", "unknown"};
      break;
    case PropertyType::EnumValue: // the Material enum (Deserializer default)
      k.data_type = "enum";
      k.options = material_options();
      break;
    case PropertyType::EnumArray: // std::vector<Material>, a JSON array
      k.data_type = "multiselect";
      k.options = material_options();
      break;
    case PropertyType::StringArray:
      // Free-form lists (image paths, documents): nothing to validate an
      // entry against yet, so read-only until an editor exists for them.
      k.data_type = "text";
      k.editable = false;
      break;
    default: // String
      k.data_type = "text";
      break;
    }
    out.push_back(std::move(k));
  }
  return out;
}

ResourceKey column_key(const ProjectDB::PropertyDefinition &def) {
  ResourceKey k;
  k.id = "col:" + def.id;
  k.label = def.name;
  k.category = std::string(kColumnCategory);
  k.scope = KeyScope::part;
  k.source = KeySource::column;
  k.field = def.name;
  if (def.type == "number" || def.type == "date" || def.type == "boolean")
    k.data_type = def.type;
  else if (def.type == "select") {
    k.data_type = "enum";
    k.options = def.options;
  } else
    k.data_type = "text"; // text, multiselect, url
  return k;
}

std::vector<ResourceKey>
key_catalogue(const std::vector<ProjectDB::PropertyDefinition> &columns) {
  auto out = builtin_keys();
  for (auto &k : leksikon_keys())
    out.push_back(std::move(k));
  for (const auto &d : columns)
    out.push_back(column_key(d));
  return out;
}

std::vector<ResourceKey> key_catalogue(const ProjectDB &db) {
  return key_catalogue(db.list_property_definitions());
}

const ResourceKey *find_key(const std::vector<ResourceKey> &catalogue,
                            std::string_view id) {
  for (const auto &k : catalogue)
    if (k.id == id)
      return &k;
  return nullptr;
}

std::optional<double> parse_number(std::string_view text) {
  const auto b = text.find_first_not_of(" \t");
  if (b == std::string_view::npos)
    return std::nullopt;
  std::string s(text.substr(b, text.find_last_not_of(" \t") - b + 1));
  if (s.find('.') == std::string::npos) {
    const auto c = s.find(',');
    if (c != std::string::npos && s.find(',', c + 1) == std::string::npos)
      s[c] = '.';
  }
  if (!s.empty() && s[0] == '+')
    s.erase(0, 1); // from_chars rejects a leading '+'
  double v = 0.0;
  const auto [ptr, ec] = std::from_chars(s.data(), s.data() + s.size(), v);
  if (ec != std::errc{} || ptr != s.data() + s.size() || !std::isfinite(v))
    return std::nullopt;
  return v;
}

std::string format_number(double v) { return fmt::format("{}", v); }

std::optional<std::string>
normalise_value(const ResourceKey &key,
                const std::optional<std::string> &value) {
  if (!key.editable)
    throw KeyValueError(key.id, "'" + key.id + "' is read-only");
  // "" clears every key but a built-in text one, whose NOT NULL column
  // stores "" (sys:note, sys:room, ...).
  const bool blank =
      !value || (value->empty() &&
                 (key.data_type != "text" || key.source != KeySource::builtin));
  if (blank) {
    if (!clearable(key))
      throw KeyValueError(key.id, "'" + key.id + "' cannot be cleared");
    return std::nullopt;
  }
  const std::string &v = *value;
  if (key.data_type == "number") {
    const auto n = parse_number(v);
    if (!n)
      throw KeyValueError(key.id,
                          "'" + key.id + "' must be a number, got '" + v + "'");
    if (key.integer && std::floor(*n) != *n)
      throw KeyValueError(
          key.id, "'" + key.id + "' must be a whole number, got '" + v + "'");
    // Leksikon Integer fields are read back as int (std::from_chars).
    if (key.integer &&
        (*n < static_cast<double>(std::numeric_limits<int>::min()) ||
         *n > static_cast<double>(std::numeric_limits<int>::max())))
      throw KeyValueError(key.id,
                          "'" + key.id + "' is out of range, got '" + v + "'");
    if (key.id == "sys:quantity" && *n < 0.0)
      throw KeyValueError(key.id, "'sys:quantity' must be a number >= 0");
    return format_number(*n);
  }
  if (key.data_type == "boolean") {
    if (v == "true" || v == "false")
      return v;
    throw KeyValueError(key.id,
                        "'" + key.id + "' must be \"true\" or \"false\"");
  }
  if (key.data_type == "enum") {
    if (std::find(key.options.begin(), key.options.end(), v) !=
        key.options.end())
      return v;
    throw KeyValueError(key.id, "'" + key.id + "' has no value '" + v + "'");
  }
  if (key.data_type == "multiselect")
    return normalise_multiselect(key, v);
  if (key.data_type == "view") {
    try {
      return view_to_json(parse_view(v));
    } catch (const std::invalid_argument &e) {
      throw KeyValueError(key.id,
                          "'" + key.id + "' " + e.what() + ", got '" + v + "'");
    }
  }
  if (key.data_type == "date") {
    if (is_iso_date(v))
      return v;
    throw KeyValueError(
        key.id, "'" + key.id + "' must be a date YYYY-MM-DD, got '" + v + "'");
  }
  // key.data_type == "text" here (the other types all return above). An
  // empty string on a non-clearable text key (sys:name, sys:unit) is a
  // 400-class rejection, same as a missing value (R-P7) — not a silent
  // clear, since blank only short-circuited above for non-text types.
  if (v.empty() && !clearable(key))
    throw KeyValueError(key.id, "'" + key.id + "' cannot be cleared");
  return v;
}

} // namespace reusex::core
