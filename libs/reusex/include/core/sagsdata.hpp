// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

/// The BEK 496 Sagsdata fields of a project (bygherre, BBR, matrikel, ...)
/// beyond ProjectDB::ProjectMetadata. They live in project_settings, one key
/// per field, `sagsdata/<project id>/<field>`; "" (or no row) means unset.
///
/// One owner for the key scheme, the validation and the wire form, so ruxd's
/// /projects API and the PDF report (write_report_data) cannot drift.

#include <nlohmann/json.hpp>

#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace reusex {
class ProjectDB;
}

namespace reusex::core {

/// The field names, in form order: bbr_number, cadastral, client,
/// client_contact, coordinator, municipality, demolition_start,
/// demolished_floor_area_m2, screening_approved_at.
const std::vector<std::string> &sagsdata_fields();

/// True when @p field is one of sagsdata_fields().
bool is_sagsdata_field(std::string_view field);

/// "sagsdata/<project_id>/<field>".
std::string sagsdata_setting_key(std::string_view project_id,
                                 std::string_view field);

/// The stored text of one field; nullopt when unset (no row, or "").
std::optional<std::string> sagsdata_value(const ProjectDB &db,
                                          std::string_view project_id,
                                          std::string_view field);

/// Every field as JSON: a string, `demolished_floor_area_m2` as a number,
/// null when unset (an unparsable stored area reads as null).
nlohmann::json sagsdata_json(const ProjectDB &db, std::string_view project_id);

/// The text to store for @p field given its JSON value: "" for null (clears).
/// `demolished_floor_area_m2` must be a finite non-negative number (stored as
/// its JSON text, so it round-trips exactly); `demolition_start` a date
/// (YYYY-MM-DD); `screening_approved_at` a date or an ISO 8601 date-time;
/// every other field a string. An empty string also clears.
/// @throws std::invalid_argument naming the field and the rule, or an
///         unknown field.
std::string sagsdata_stored_value(std::string_view field,
                                  const nlohmann::json &value);

/// YYYY-MM-DD with a real calendar day.
bool is_iso_date(std::string_view s);
/// A date, `T`, `hh:mm[:ss[.frac]]`, then `Z` or `±hh:mm`.
bool is_iso_date_time(std::string_view s);

/// "21.09.2026" for an ISO date or a timestamp starting with one; @p s
/// unchanged otherwise.
std::string danish_date(std::string_view s);

/// The project the report and the GUI treat as "the" project: the first of
/// list_project_ids(), or "" when there is none.
std::string primary_project_id(const ProjectDB &db);

} // namespace reusex::core
