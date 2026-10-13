// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

// The Rediger edge list (spec §5.2): columns, Danish cells, sort with
// missing values last in both directions, type/status filter.

#include <catch2/catch_test_macros.hpp>

#include <rux_qt/edge_table.hpp>

#include <vector>

using namespace rux::qt;

namespace {
EdgeView row(int from, int to, const char *type, const char *status,
             std::optional<double> dev = std::nullopt, double residual = 0.0) {
  EdgeView v;
  v.edge.key = {from, to, type};
  v.edge.status = status;
  v.edge.residual = residual;
  v.edge.deviation_m = dev;
  v.stored_status = status;
  return v;
}
std::vector<int> froms(const std::vector<EdgeView> &rows) {
  std::vector<int> out;
  for (const auto &r : rows)
    out.push_back(r.edge.key.from);
  return out;
}
} // namespace

TEST_CASE("EdgeTable_SortsMissingLastInBothDirections", "[rux_qt][edgetable]") {
  std::vector<EdgeView> rows = {row(1, 2, "odometry", "auto", 0.3),
                                row(2, 3, "odometry", "auto"),
                                row(3, 4, "loop_closure", "auto", 1.2),
                                row(4, 5, "loop_closure", "auto", 0.3)};
  sort_edges(rows, EdgeColumn::deviation_m, /*descending=*/true);
  CHECK(froms(rows) == std::vector<int>{3, 1, 4, 2}); // tie: by key
  sort_edges(rows, EdgeColumn::deviation_m, /*descending=*/false);
  CHECK(froms(rows) == std::vector<int>{1, 4, 3, 2});
  sort_edges(rows, EdgeColumn::pair, false);
  CHECK(froms(rows) == std::vector<int>{1, 2, 3, 4});
}

TEST_CASE("EdgeTable_StagedMeasurement_HasNoResidual", "[rux_qt][edgetable]") {
  auto v = row(1, 2, "loop_closure", "confirmed", std::nullopt, 9.0);
  v.pending = EdgeView::Pending::added;
  CHECK_FALSE(edge_column_value(v, EdgeColumn::residual).has_value());
  CHECK(edge_cell_text(v, EdgeColumn::residual) == "—");
  CHECK(edge_cell_text(v, EdgeColumn::status) == "Bekræftet · ikke gemt");
}

TEST_CASE("EdgeTable_CellsAreDanish", "[rux_qt][edgetable]") {
  auto v = row(412, 980, "loop_closure", "auto", 0.4567, 12.346);
  v.edge.deviation_deg = 3.21;
  v.edge.source = "orb";
  CHECK(edge_cell_text(v, EdgeColumn::pair) == "412 → 980");
  CHECK(edge_cell_text(v, EdgeColumn::type) == "Løkkelukning");
  CHECK(edge_cell_text(v, EdgeColumn::source) == "ORB");
  CHECK(edge_cell_text(v, EdgeColumn::status) == "Auto");
  CHECK(edge_cell_text(v, EdgeColumn::residual) == "12,35");
  CHECK(edge_cell_text(v, EdgeColumn::deviation_m) == "0,46");
  CHECK(edge_cell_text(v, EdgeColumn::deviation_deg) == "3,2");
  CHECK(edge_cell_text(v, EdgeColumn::seed_m) == "—");
  CHECK(edge_source_da("") == "—");
  CHECK(edge_source_da("odometry_init") == "Odometri (init)");
  CHECK(edge_column_title(EdgeColumn::seed_deg) == "FRØ °");
}

TEST_CASE("EdgeTable_Filter_TypeStatusAndRejected", "[rux_qt][edgetable]") {
  const std::vector<EdgeView> all = {
      row(1, 2, "odometry", "auto"), row(2, 3, "loop_closure", "confirmed"),
      row(3, 4, "loop_closure", "rejected"), row(4, 5, "panorama", "auto")};
  EdgeFilter f;
  CHECK(edge_table(all, f, EdgeColumn::pair, false).size() ==
        3); // rejected hidden
  f.show_rejected = true;
  CHECK(edge_table(all, f, EdgeColumn::pair, false).size() == 4);
  f = {};
  f.type = "loop_closure";
  CHECK(froms(edge_table(all, f, EdgeColumn::pair, false)) ==
        std::vector<int>{2});
  f.status = "rejected"; // asking for rejected shows them regardless
  CHECK(froms(edge_table(all, f, EdgeColumn::pair, false)) ==
        std::vector<int>{3});
}

TEST_CASE("EdgeTable_DefaultSortFollowsTheColourMode", "[rux_qt][edgetable]") {
  CHECK(default_sort_column(EdgeColourMode::residual) == EdgeColumn::residual);
  CHECK(default_sort_column(EdgeColourMode::deviation) ==
        EdgeColumn::deviation_m);
  CHECK(default_sort_column(EdgeColourMode::seed) == EdgeColumn::seed_m);
  CHECK(default_sort_column(EdgeColourMode::type) == EdgeColumn::residual);
}
