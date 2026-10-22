// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

/// Resources out through a template (resources/templates design doc §6.3,
/// removed 2026-10-14, see git history): the backend CSV
/// (GET /resources/export.csv) and the PDF report's Ressourcetabel.

#include "reusex/core/ProjectDB.hpp"
#include "reusex/core/resource_keys.hpp"
#include "reusex/core/resource_templates.hpp"
#include "reusex/core/resources.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace reusex::core {

/// A value as a reader sees it: treatment and miljøstatus as their Danish
/// labels, booleans as Ja/Nej, unset as "".
std::string display_value(const ResourceKey &key,
                          const std::optional<std::string> &value);
/// One CSV cell: a cell starting with = + - @ TAB or CR gets a leading '
/// (formula injection), then RFC 4180 quoting when it holds the delimiter,
/// a quote, CR or LF.
std::string csv_cell(std::string_view value, std::string_view delimiter);
/// The leading resource-code column of the CSV: its header in label mode
/// and in key mode.
inline constexpr std::string_view kCsvCodeLabel = "Kode";
inline constexpr std::string_view kCsvCodeKey = "code";
/// The CSV header labels of @p columns: each key's label, with
/// " (<category>)" appended to every label that occurs more than once
/// (counting the "Kode" column), so no two headers read the same.
std::vector<std::string> csv_labels(const std::vector<ResourceKey> &columns);
/// Header + one CRLF-terminated row per resource, led by the resource code.
/// header "label" writes csv_labels() and display values; "key" writes key
/// ids and raw values. A "utf-8-bom" encoding prefixes the UTF-8 byte-order
/// mark.
std::string build_resource_csv(const std::vector<ResourceKey> &columns,
                               const std::vector<Resource> &rows,
                               const CsvOptions &options);
/// The CSV for one template: its resolved keys, every non-dismissed resource
/// by code, its CSV options. @throws std::out_of_range when the template is
/// unknown.
std::string export_resources_csv(const ProjectDB &db, int64_t template_id);

struct ResourceTable {
  std::vector<std::string> headers;
  std::vector<std::vector<std::string>> rows;
};
inline constexpr std::size_t kResourceTableMaxColumns = 8;
/// Wrap @p columns into tables of at most @p max_columns, each led by a
/// "Betegnelse" column whose cell is "<sys:name> · <code>" (sys:name is not
/// repeated among the others). No rows: no tables.
std::vector<ResourceTable>
resource_tables(const std::vector<ResourceKey> &columns,
                const std::vector<Resource> &rows,
                std::size_t max_columns = kResourceTableMaxColumns);
struct ResourceReportSection {
  std::string template_name;
  std::vector<ResourceTable> tables;
};
/// The PDF's Ressourcetabel for one template: every non-dismissed resource
/// whose type is not rejected, by code. @throws std::out_of_range for an
/// unknown template.
ResourceReportSection resource_report_section(const ProjectDB &db,
                                              int64_t template_id);

} // namespace reusex::core
