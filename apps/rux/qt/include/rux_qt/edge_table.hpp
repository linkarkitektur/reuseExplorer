// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// The Rediger edge list's data, Qt-free (spec §5.2): columns, the Danish
// cell text, sorting with missing values always last (so "most suspicious
// first" never starts with a page of dashes), and the type/status filter.

#include <rux_qt/database_logic.hpp>
#include <rux_qt/edge_colouring.hpp>

#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace rux::qt {

enum class EdgeColumn {
  pair = 0,
  type,
  source,
  status,
  residual,
  deviation_m,
  deviation_deg,
  seed_m,
  seed_deg
};
inline constexpr int kEdgeColumnCount = 9;

/// Upper-case header text (QSS header rules drop QFont::AllUppercase).
std::string edge_column_title(EdgeColumn c);
bool edge_column_numeric(EdgeColumn c);
/// The sort value of a numeric column; nullopt when missing.
std::optional<double> edge_column_value(const EdgeView &v, EdgeColumn c);
std::string edge_cell_text(const EdgeView &v, EdgeColumn c);
/// The edge list's cell: as edge_cell_text, but STATUS without the
/// " · ikke gemt" suffix (the widest prose column) and PAR prefixed "• "
/// for a staged edge, a cue that does not rely on the row's colour.
std::string edge_list_cell_text(const EdgeView &v, EdgeColumn c);
/// Hidden until the user shows them (the list's header menu): the angle
/// columns, so the rest fits Rediger's left pane at 1440 px.
std::vector<EdgeColumn> default_hidden_edge_columns();
std::string edge_status_da(std::string_view status);
std::string edge_source_da(std::string_view source);

struct EdgeFilter {
  std::string type;   ///< empty = every type
  std::string status; ///< empty = every non-rejected (+ rejected if shown)
  bool show_rejected = false;
};
bool edge_matches(const EdgeView &v, const EdgeFilter &f);
/// Stable: ties by (from, to, type); missing numeric values last.
void sort_edges(std::vector<EdgeView> &rows, EdgeColumn column,
                bool descending);
/// The column a colour mode's suspicious edges sort to the top by.
EdgeColumn default_sort_column(EdgeColourMode mode);
std::vector<EdgeView> edge_table(const std::vector<EdgeView> &all,
                                 const EdgeFilter &f, EdgeColumn column,
                                 bool descending);

} // namespace rux::qt
