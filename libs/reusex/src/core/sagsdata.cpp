// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <reusex/core/sagsdata.hpp>

#include <reusex/core/ProjectDB.hpp>

#include <algorithm>
#include <cmath>
#include <regex>
#include <stdexcept>

namespace reusex::core {

const std::vector<std::string> &sagsdata_fields() {
  static const std::vector<std::string> fields{"bbr_number",
                                               "cadastral",
                                               "client",
                                               "client_contact",
                                               "coordinator",
                                               "municipality",
                                               "demolition_start",
                                               "demolished_floor_area_m2",
                                               "screening_approved_at"};
  return fields;
}

bool is_sagsdata_field(std::string_view field) {
  const auto &f = sagsdata_fields();
  return std::find(f.begin(), f.end(), field) != f.end();
}

std::string sagsdata_setting_key(std::string_view project_id,
                                 std::string_view field) {
  return std::string("sagsdata/").append(project_id).append("/").append(field);
}

std::optional<std::string> sagsdata_value(const ProjectDB &db,
                                          std::string_view project_id,
                                          std::string_view field) {
  auto value = db.setting(sagsdata_setting_key(project_id, field));
  if (!value || value->empty())
    return std::nullopt;
  return value;
}

nlohmann::json sagsdata_json(const ProjectDB &db, std::string_view project_id) {
  nlohmann::json out = nlohmann::json::object();
  for (const auto &field : sagsdata_fields()) {
    const auto value = sagsdata_value(db, project_id, field);
    if (!value) {
      out[field] = nullptr;
    } else if (field == "demolished_floor_area_m2") {
      // Stored as JSON text so the number round-trips exactly.
      const auto n =
          nlohmann::json::parse(*value, nullptr, /*allow_exceptions=*/false);
      out[field] = n.is_number() ? n : nlohmann::json(nullptr);
    } else {
      out[field] = *value;
    }
  }
  return out;
}

bool is_iso_date(std::string_view s) {
  if (s.size() != 10 || s[4] != '-' || s[7] != '-')
    return false;
  for (std::size_t i = 0; i < s.size(); ++i)
    if (i != 4 && i != 7 && (s[i] < '0' || s[i] > '9'))
      return false;
  const int y = std::stoi(std::string(s.substr(0, 4)));
  const int m = std::stoi(std::string(s.substr(5, 2)));
  const int d = std::stoi(std::string(s.substr(8, 2)));
  if (m < 1 || m > 12 || d < 1)
    return false;
  static constexpr int kDays[] = {31, 28, 31, 30, 31, 30,
                                  31, 31, 30, 31, 30, 31};
  const bool leap = (y % 4 == 0 && y % 100 != 0) || y % 400 == 0;
  return d <= kDays[m - 1] + (m == 2 && leap ? 1 : 0);
}

bool is_iso_date_time(std::string_view s) {
  static const std::regex re(
      R"(^\d{4}-\d{2}-\d{2}T([01]\d|2[0-3]):[0-5]\d(:[0-5]\d(\.\d+)?)?(Z|[+-]([01]\d|2[0-3]):[0-5]\d)$)");
  return s.size() > 10 && is_iso_date(s.substr(0, 10)) &&
         std::regex_match(s.begin(), s.end(), re);
}

std::string sagsdata_stored_value(std::string_view field,
                                  const nlohmann::json &value) {
  const std::string name(field);
  if (!is_sagsdata_field(field))
    throw std::invalid_argument("'" + name + "' is not a Sagsdata field");
  if (value.is_null())
    return {};
  if (field == "demolished_floor_area_m2") {
    if (!value.is_number() || !std::isfinite(value.get<double>()) ||
        value.get<double>() < 0.0)
      throw std::invalid_argument("'" + name +
                                  "' must be a non-negative number or null");
    return value.dump();
  }
  if (!value.is_string())
    throw std::invalid_argument("'" + name + "' must be a string or null");
  auto stored = value.get<std::string>();
  if (stored.empty())
    return stored;
  if (field == "demolition_start" && !is_iso_date(stored))
    throw std::invalid_argument("'" + name +
                                "' must be a date (YYYY-MM-DD) or null");
  if (field == "screening_approved_at" && !is_iso_date(stored) &&
      !is_iso_date_time(stored))
    throw std::invalid_argument("'" + name +
                                "' must be a date (YYYY-MM-DD), an ISO 8601 "
                                "date-time or null");
  return stored;
}

std::string danish_date(std::string_view s) {
  if (s.size() >= 10 && is_iso_date(s.substr(0, 10)))
    return std::string(s.substr(8, 2)) + "." + std::string(s.substr(5, 2)) +
           "." + std::string(s.substr(0, 4));
  return std::string(s);
}

std::string primary_project_id(const ProjectDB &db) {
  const auto ids = db.list_project_ids();
  return ids.empty() ? std::string() : ids.front();
}

} // namespace reusex::core
