// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "reusex/core/buildups.hpp"

#include "reusex/core/ProjectDB.hpp"
#include "reusex/core/buildups_default.hpp"
#include "reusex/core/logging.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <cctype>
#include <map>
#include <set>
#include <stdexcept>
#include <utility>

namespace reusex::core {

namespace {
using json = nlohmann::json;

constexpr std::array<std::pair<BuildupCategory, std::string_view>, 4>
    kCategories{{{BuildupCategory::ydervaeg, "ydervaeg"},
                 {BuildupCategory::indervaeg, "indervaeg"},
                 {BuildupCategory::daek, "daek"},
                 {BuildupCategory::tag, "tag"}}};

std::optional<double> opt_number(const json &j, const char *key,
                                 std::string_view where) {
  if (!j.contains(key) || j[key].is_null())
    return std::nullopt;
  if (!j[key].is_number())
    throw std::invalid_argument(std::string(where) + ": '" + key +
                                "' must be a number");
  const double v = j[key].get<double>();
  if (v < 0.0)
    throw std::invalid_argument(std::string(where) + ": '" + key +
                                "' must not be negative");
  return v;
}

std::string opt_string(const json &j, const char *key) {
  return j.contains(key) && j[key].is_string() ? j[key].get<std::string>()
                                               : std::string{};
}

BuildupLayer parse_layer(const json &j, std::string_view where) {
  if (!j.is_object())
    throw std::invalid_argument(std::string(where) + ": a layer is an object");
  BuildupLayer l;
  l.material = opt_string(j, "material");
  l.t = opt_number(j, "t", where);
  l.rho = opt_number(j, "rho", where);
  l.kg_m2 = opt_number(j, "kg_m2", where);
  l.eak = opt_string(j, "eak");
  l.name = opt_string(j, "name");
  if (!l.kg_m2 && !l.rho)
    throw std::invalid_argument(std::string(where) + ": layer '" + l.material +
                                "' needs kg_m2 or rho");
  return l;
}

json layer_json(const BuildupLayer &l) {
  json j = {{"material", l.material}, {"eak", l.eak}};
  if (l.t)
    j["t"] = *l.t;
  if (l.rho)
    j["rho"] = *l.rho;
  if (l.kg_m2)
    j["kg_m2"] = *l.kg_m2;
  if (!l.name.empty())
    j["name"] = l.name;
  return j;
}

Buildup parse_buildup_object(const json &j) {
  if (!j.is_object())
    throw std::invalid_argument("buildup: expected a JSON object");
  Buildup b;
  b.id = opt_string(j, "id");
  b.name = opt_string(j, "name");
  if (b.id.empty())
    throw std::invalid_argument("buildup: 'id' is required");
  if (b.name.empty())
    throw std::invalid_argument("buildup '" + b.id + "': 'name' is required");
  const auto cat = buildup_category_from_string(opt_string(j, "category"));
  if (!cat)
    throw std::invalid_argument(
        "buildup '" + b.id +
        "': 'category' must be ydervaeg, indervaeg, daek or tag");
  b.category = *cat;
  if (!j.contains("main"))
    throw std::invalid_argument("buildup '" + b.id + "': 'main' is required");
  const std::string where = "buildup '" + b.id + "'";
  b.main = parse_layer(j["main"], where);
  if (j.contains("others") && !j["others"].is_null()) {
    if (!j["others"].is_array())
      throw std::invalid_argument(where + ": 'others' must be an array");
    for (const auto &l : j["others"])
      b.others.push_back(parse_layer(l, where));
  }
  b.note = opt_string(j, "note");
  return b;
}

std::string trimmed(std::string_view s) {
  const auto a = s.find_first_not_of(" \t\r\n");
  if (a == std::string_view::npos)
    return {};
  const auto e = s.find_last_not_of(" \t\r\n");
  return std::string(s.substr(a, e - a + 1));
}

/// ASCII lower case plus the Danish capitals (UTF-8 Æ Ø Å -> æ ø å).
std::string lower_da(std::string_view s) {
  std::string out;
  for (std::size_t i = 0; i < s.size(); ++i) {
    const auto c = static_cast<unsigned char>(s[i]);
    if (c == 0xC3 && i + 1 < s.size()) {
      auto d = static_cast<unsigned char>(s[i + 1]);
      if (d == 0x86 || d == 0x98 || d == 0x85) // Æ Ø Å
        d = static_cast<unsigned char>(d + 0x20);
      out += static_cast<char>(c);
      out += static_cast<char>(d);
      ++i;
      continue;
    }
    out += static_cast<char>(std::tolower(c));
  }
  return out;
}

/// Words of @p s: runs of letters/digits (any byte >= 0x80 counts as a
/// letter, so æ ø å stay inside a word).
std::vector<std::string> words_of(const std::string &s) {
  std::vector<std::string> out;
  std::string cur;
  for (const char ch : s) {
    const auto c = static_cast<unsigned char>(ch);
    if (std::isalnum(c) || c >= 0x80) {
      cur += ch;
    } else if (!cur.empty()) {
      out.push_back(std::move(cur));
      cur.clear();
    }
  }
  if (!cur.empty())
    out.push_back(std::move(cur));
  return out;
}
} // namespace

std::string_view to_string(BuildupCategory c) noexcept {
  for (const auto &[k, v] : kCategories)
    if (k == c)
      return v;
  return "ydervaeg";
}

std::optional<BuildupCategory>
buildup_category_from_string(std::string_view s) {
  for (const auto &[k, v] : kCategories)
    if (v == s)
      return k;
  return std::nullopt;
}

Buildup parse_buildup(std::string_view json_text) {
  json j;
  try {
    j = json::parse(json_text);
  } catch (const json::parse_error &e) {
    throw std::invalid_argument(std::string("buildup: malformed JSON: ") +
                                e.what());
  }
  return parse_buildup_object(j);
}

std::vector<Buildup> parse_buildup_library(std::string_view json_text) {
  json j;
  try {
    j = json::parse(json_text);
  } catch (const json::parse_error &e) {
    throw std::invalid_argument(
        std::string("buildup library: malformed JSON: ") + e.what());
  }
  if (!j.is_object() || !j.contains("buildups") || !j["buildups"].is_array())
    throw std::invalid_argument(
        "buildup library: expected {\"version\": 1, \"buildups\": [...]}");
  if (j.contains("version") && j["version"] != 1)
    throw std::invalid_argument("buildup library: unsupported version " +
                                j["version"].dump());
  std::vector<Buildup> out;
  std::set<std::string> ids;
  for (const auto &e : j["buildups"]) {
    auto b = parse_buildup_object(e);
    if (!ids.insert(b.id).second)
      throw std::invalid_argument("buildup library: duplicate id '" + b.id +
                                  "'");
    out.push_back(std::move(b));
  }
  return out;
}

std::string to_json(const Buildup &b) {
  json others = json::array();
  for (const auto &l : b.others)
    others.push_back(layer_json(l));
  json j = {{"id", b.id},
            {"category", to_string(b.category)},
            {"name", b.name},
            {"main", layer_json(b.main)},
            {"others", std::move(others)}};
  if (!b.note.empty())
    j["note"] = b.note;
  return j.dump();
}

const std::vector<Buildup> &default_buildups() {
  static const std::vector<Buildup> lib =
      parse_buildup_library(detail::kDefaultBuildupsJson);
  return lib;
}

std::vector<Buildup> buildup_library(const ProjectDB &db) {
  std::vector<Buildup> out = default_buildups();
  std::map<std::string, Buildup> extra;
  for (const auto &row : db.project_buildups()) {
    Buildup b;
    try {
      b = parse_buildup(row.definition);
    } catch (const std::invalid_argument &e) {
      reusex::warn("buildup_library: project build-up '{}' is not valid and "
                   "is skipped: {}",
                   row.id, e.what());
      continue;
    }
    b.id = row.id; // the row's key wins over an id inside the JSON
    b.origin = "project";
    const auto it = std::find_if(
        out.begin(), out.end(), [&](const Buildup &d) { return d.id == b.id; });
    if (it != out.end())
      *it = std::move(b);
    else
      extra.emplace(b.id, std::move(b));
  }
  for (auto &[id, b] : extra)
    out.push_back(std::move(b));
  return out;
}

std::optional<Buildup> find_buildup(const std::vector<Buildup> &library,
                                    std::string_view id) {
  for (const auto &b : library)
    if (b.id == id)
      return b;
  return std::nullopt;
}

std::optional<Buildup> find_buildup_by_name(const std::vector<Buildup> &library,
                                            std::string_view name) {
  const std::string n = trimmed(name);
  if (n.empty())
    return std::nullopt;
  for (const auto &b : library)
    if (b.name == n)
      return b;
  return std::nullopt;
}

double layer_tonnes(const BuildupLayer &layer, double quantity,
                    std::string_view unit) {
  if (layer.kg_m2)
    return quantity * *layer.kg_m2 / 1000.0;
  const double rho = layer.rho.value_or(0.0);
  if (unit == kUnitVolume)
    return quantity * rho / 1000.0;
  return quantity * layer.t.value_or(0.0) * rho / 1000.0;
}

double layered_thickness(const Buildup &b) {
  double sum = 0.0;
  const auto add = [&](const BuildupLayer &l) {
    if (!l.kg_m2 && l.t && *l.t < 1.0)
      sum += *l.t;
  };
  add(b.main);
  for (const auto &l : b.others)
    add(l);
  return sum;
}

std::optional<BuildupCategory> category_of(std::string_view type_name,
                                           std::string_view bim7aa_code) {
  struct Rule {
    BuildupCategory cat;
    std::vector<std::string_view> contains, starts;
    std::string_view bim;
  };
  // Most specific first, as the frontend's CATEGORY_RULES: "Tagpap" is tag,
  // "Etagedæk" is dæk (it contains "dæk"; "tag" only matches a word start).
  static const std::vector<Rule> rules = {
      {BuildupCategory::indervaeg,
       {"indervæg", "skillevæg", "gipsvæg"},
       {},
       "22"},
      {BuildupCategory::ydervaeg, {"ydervæg", "facade", "mur"}, {}, "21"},
      {BuildupCategory::daek, {"dæk", "gulv"}, {}, "23"},
      {BuildupCategory::tag, {}, {"tag"}, "27"},
  };
  const auto words = words_of(lower_da(type_name));
  for (const auto &r : rules) {
    for (const auto &w : words) {
      for (const auto k : r.contains)
        if (w.find(k) != std::string::npos)
          return r.cat;
      for (const auto k : r.starts)
        if (w.rfind(k, 0) == 0)
          return r.cat;
    }
  }
  const std::string code = trimmed(bim7aa_code);
  if (code.size() >= 2 && std::isdigit(static_cast<unsigned char>(code[0])) &&
      std::isdigit(static_cast<unsigned char>(code[1])) &&
      (code.size() == 2 ||
       !std::isalnum(static_cast<unsigned char>(code[2])))) {
    const std::string_view group(code.data(), 2);
    for (const auto &r : rules)
      if (r.bim == group)
        return r.cat;
  }
  return std::nullopt;
}

} // namespace reusex::core
