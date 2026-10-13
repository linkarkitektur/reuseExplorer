// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "reusex/core/resource_export.hpp"

#include "reusex/core/survey.hpp"

#include <algorithm>
#include <map>
#include <set>

namespace reusex::core {
namespace {
std::vector<ResourceKey> columns_of(const std::vector<std::string> &ids,
                                    const std::vector<ResourceKey> &catalogue) {
  std::vector<ResourceKey> out;
  for (const auto &id : ids)
    if (const auto *k = find_key(catalogue, id))
      out.push_back(*k);
  return out;
}
} // namespace

std::string display_value(const ResourceKey &key,
                          const std::optional<std::string> &value) {
  if (!value)
    return {};
  if (key.id == "sys:treatment")
    if (const auto t = treatment_from_string(*value))
      return std::string(treatment_label_da(*t));
  if (key.id == "sys:environment")
    for (auto e :
         {EnvironmentStatus::ren_screening, EnvironmentStatus::afventer,
          EnvironmentStatus::forurenet, EnvironmentStatus::ren_proevesvar})
      if (to_string(e) == *value)
        return std::string(environment_label_da(e));
  if (key.data_type == "boolean") {
    if (*value == "true")
      return "Ja";
    if (*value == "false")
      return "Nej";
  }
  if (key.id == kViewKey) {
    try {
      return view_display(parse_view(*value));
    } catch (const std::invalid_argument &) {
      return *value; // not ours to hide; show what is stored
    }
  }
  return *value;
}

std::string csv_cell(std::string_view value, std::string_view delimiter) {
  std::string s(value);
  if (!s.empty() && (s[0] == '=' || s[0] == '+' || s[0] == '-' || s[0] == '@' ||
                     s[0] == '\t' || s[0] == '\r'))
    s.insert(s.begin(), '\'');
  const bool quote = s.find(delimiter) != std::string::npos ||
                     s.find_first_of("\"\n\r") != std::string::npos;
  if (!quote)
    return s;
  std::string out = "\"";
  for (char c : s)
    out += c == '"' ? std::string("\"\"") : std::string(1, c);
  return out + "\"";
}

std::vector<std::string> csv_labels(const std::vector<ResourceKey> &columns) {
  std::map<std::string, std::size_t> uses{{std::string(kCsvCodeLabel), 1}};
  for (const auto &k : columns)
    ++uses[k.label];
  std::vector<std::string> out;
  for (const auto &k : columns)
    out.push_back(uses[k.label] > 1 ? k.label + " (" + k.category + ")"
                                    : k.label);
  return out;
}

std::string build_resource_csv(const std::vector<ResourceKey> &columns,
                               const std::vector<Resource> &rows,
                               const CsvOptions &o) {
  const bool by_key = o.header == "key";
  std::string out = o.encoding == "utf-8-bom" ? "\xEF\xBB\xBF" : "";
  out += csv_cell(by_key ? kCsvCodeKey : kCsvCodeLabel, o.delimiter);
  const auto labels = by_key ? std::vector<std::string>{} : csv_labels(columns);
  for (std::size_t i = 0; i < columns.size(); ++i)
    out +=
        o.delimiter + csv_cell(by_key ? columns[i].id : labels[i], o.delimiter);
  out += "\r\n";
  for (const auto &r : rows) {
    out += csv_cell(r.code, o.delimiter);
    for (const auto &k : columns) {
      const auto v = value_of(r, k.id);
      out +=
          o.delimiter +
          csv_cell(by_key ? v.value_or("") : display_value(k, v), o.delimiter);
    }
    out += "\r\n";
  }
  return out;
}

std::string export_resources_csv(const ProjectDB &db, int64_t template_id) {
  const auto view = template_view(db, template_id);
  const auto columns = columns_of(view.resolved.keys, key_catalogue(db));
  return build_resource_csv(columns, list_resources(db, view.resolved.keys),
                            view.csv);
}

std::vector<ResourceTable>
resource_tables(const std::vector<ResourceKey> &columns,
                const std::vector<Resource> &rows, std::size_t max_columns) {
  if (max_columns < 2)
    throw std::invalid_argument("resource_tables: max_columns must leave room "
                                "for Betegnelse and one more column");
  if (rows.empty())
    return {};
  std::vector<const ResourceKey *> others;
  for (const auto &k : columns)
    // sys:name leads every table; sys:view is an image — the report shows
    // the picture (sub-project C), never "Foto #12" in a table cell.
    if (k.id != "sys:name" && k.id != kViewKey)
      others.push_back(&k);
  const std::size_t per = max_columns - 1;
  std::vector<ResourceTable> out;
  for (std::size_t first = 0;; first += per) {
    const std::size_t last = std::min(first + per, others.size());
    ResourceTable t;
    t.headers.push_back("Betegnelse");
    for (std::size_t i = first; i < last; ++i)
      t.headers.push_back(others[i]->label);
    for (const auto &r : rows) {
      std::vector<std::string> cells{value_of(r, "sys:name").value_or("") +
                                     " · " + r.code};
      for (std::size_t i = first; i < last; ++i)
        cells.push_back(display_value(*others[i], value_of(r, others[i]->id)));
      t.rows.push_back(std::move(cells));
    }
    out.push_back(std::move(t));
    if (last >= others.size())
      break;
  }
  return out;
}

ResourceReportSection resource_report_section(const ProjectDB &db,
                                              int64_t template_id) {
  const auto view = template_view(db, template_id);
  const auto columns = columns_of(view.resolved.keys, key_catalogue(db));
  std::vector<std::string> keys{"sys:name"};
  for (const auto &id : view.resolved.keys)
    if (id != "sys:name")
      keys.push_back(id);
  std::set<int64_t> rejected;
  for (const auto &t : db.survey_types())
    if (t.review_status == ReviewStatus::rejected)
      rejected.insert(t.id);
  std::vector<Resource> rows;
  for (auto &r : list_resources(db, keys))
    if (rejected.count(r.type_id) == 0)
      rows.push_back(std::move(r));
  return {view.record.name, resource_tables(columns, rows)};
}

} // namespace reusex::core
