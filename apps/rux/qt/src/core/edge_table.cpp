// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <rux_qt/edge_table.hpp>

#include <algorithm>
#include <cmath>
#include <tuple>

namespace rux::qt {
namespace {
auto key_tuple(const EdgeView &v) {
  return std::tie(v.edge.key.from, v.edge.key.to, v.edge.key.type);
}
} // namespace

std::string edge_column_title(EdgeColumn c) {
  switch (c) {
  case EdgeColumn::pair:
    return "PAR";
  case EdgeColumn::type:
    return "TYPE";
  case EdgeColumn::source:
    return "KILDE";
  case EdgeColumn::status:
    return "STATUS";
  case EdgeColumn::residual:
    return "RESIDUAL";
  case EdgeColumn::deviation_m:
    return "AFV. M";
  case EdgeColumn::deviation_deg:
    return "AFV. °";
  case EdgeColumn::seed_m:
    return "FRØ M";
  case EdgeColumn::seed_deg:
    return "FRØ °";
  }
  return {};
}

bool edge_column_numeric(EdgeColumn c) {
  return c == EdgeColumn::residual || c == EdgeColumn::deviation_m ||
         c == EdgeColumn::deviation_deg || c == EdgeColumn::seed_m ||
         c == EdgeColumn::seed_deg;
}

std::optional<double> edge_column_value(const EdgeView &v, EdgeColumn c) {
  switch (c) {
  case EdgeColumn::residual:
    return edge_metric(v, EdgeColourMode::residual);
  case EdgeColumn::deviation_m:
    return v.edge.deviation_m;
  case EdgeColumn::deviation_deg:
    return v.edge.deviation_deg;
  case EdgeColumn::seed_m:
    return v.edge.seed_disagreement_m;
  case EdgeColumn::seed_deg:
    return v.edge.seed_disagreement_deg;
  default:
    return std::nullopt;
  }
}

std::string edge_status_da(std::string_view s) {
  if (s == "confirmed")
    return "Bekræftet";
  if (s == "rejected")
    return "Afvist";
  return "Auto";
}

std::string edge_source_da(std::string_view s) {
  if (s == "rtabmap")
    return "RTABMap";
  if (s == "odometry_init")
    return "Odometri (init)";
  if (s == "orb")
    return "ORB";
  if (s == "panorama")
    return "Panorama";
  if (s == "file")
    return "Fil";
  if (s == "icp")
    return "ICP";
  if (s == "manual")
    return "Manuel";
  if (s == "legacy")
    return "Ældre";
  if (s.empty())
    return "—";
  return std::string(s);
}

std::string edge_cell_text(const EdgeView &v, EdgeColumn c) {
  switch (c) {
  case EdgeColumn::pair:
    return std::to_string(v.edge.key.from) + " → " +
           std::to_string(v.edge.key.to);
  case EdgeColumn::type:
    return edge_type_da(v.edge.key.type);
  case EdgeColumn::source:
    return edge_source_da(v.edge.source);
  case EdgeColumn::status:
    return edge_status_da(v.edge.status) +
           (v.pending != EdgeView::Pending::none ? " · ikke gemt" : "");
  default:
    break;
  }
  const auto value = edge_column_value(v, c);
  if (!value || !std::isfinite(*value))
    return "—";
  const int decimals =
      (c == EdgeColumn::deviation_deg || c == EdgeColumn::seed_deg) ? 1 : 2;
  return format_decimal_da(*value, decimals);
}

bool edge_matches(const EdgeView &v, const EdgeFilter &f) {
  if (!f.type.empty() && v.edge.key.type != f.type)
    return false;
  if (!f.status.empty())
    return v.edge.status == f.status;
  return f.show_rejected || v.edge.status != "rejected";
}

void sort_edges(std::vector<EdgeView> &rows, EdgeColumn column,
                bool descending) {
  std::stable_sort(rows.begin(), rows.end(),
                   [&](const EdgeView &a, const EdgeView &b) {
                     if (edge_column_numeric(column)) {
                       const auto va = edge_column_value(a, column),
                                  vb = edge_column_value(b, column);
                       const bool ha = va && std::isfinite(*va),
                                  hb = vb && std::isfinite(*vb);
                       if (ha != hb)
                         return ha; // missing last, whatever the direction
                       if (ha && *va != *vb)
                         return descending ? *va > *vb : *va < *vb;
                       return key_tuple(a) < key_tuple(b);
                     }
                     if (column != EdgeColumn::pair) {
                       const auto ta = edge_cell_text(a, column),
                                  tb = edge_cell_text(b, column);
                       if (ta != tb)
                         return descending ? ta > tb : ta < tb;
                       return key_tuple(a) < key_tuple(b);
                     }
                     return descending ? key_tuple(b) < key_tuple(a)
                                       : key_tuple(a) < key_tuple(b);
                   });
}

EdgeColumn default_sort_column(EdgeColourMode mode) {
  switch (mode) {
  case EdgeColourMode::deviation:
    return EdgeColumn::deviation_m;
  case EdgeColourMode::seed:
    return EdgeColumn::seed_m;
  default:
    return EdgeColumn::residual;
  }
}

std::vector<EdgeView> edge_table(const std::vector<EdgeView> &all,
                                 const EdgeFilter &f, EdgeColumn column,
                                 bool descending) {
  std::vector<EdgeView> out;
  for (const auto &v : all)
    if (edge_matches(v, f))
      out.push_back(v);
  sort_edges(out, column, descending);
  return out;
}

} // namespace rux::qt
