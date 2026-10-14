// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

// Edge colouring of the Rediger canvas (spec §5.2): the fixed χ²-based
// residual scale, percentile scales for the metric modes, categorical
// tokens, and the legend built from what is drawn.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <rux_qt/edge_colouring.hpp>

#include <cmath>
#include <vector>

using namespace rux::qt;
using Catch::Approx;

namespace {
EdgeView view(int from, int to, const char *type = "loop_closure",
              const char *status = "auto") {
  EdgeView v;
  v.edge.key = {from, to, type};
  v.edge.status = status;
  v.stored_status = status;
  return v;
}
EdgeView with_residual(double r) {
  auto v = view(1, 2);
  v.edge.residual = r;
  return v;
}
EdgeView with_deviation(double m) {
  auto v = view(1, 2);
  v.edge.deviation_m = m;
  return v;
}
} // namespace

TEST_CASE("EdgeColour_ResidualScale_IsTheChiSquareThreshold",
          "[rux_qt][colour]") {
  CHECK(kResidualGood == Approx(6.29579).epsilon(1e-5));
  CHECK(kResidualBad == Approx(31.479).epsilon(1e-4));
  const auto s = colour_scale(EdgeColourMode::residual, {});
  CHECK(s.valid);
  CHECK(s.fixed);
  const auto style = [&](double r) {
    return edge_style(with_residual(r), EdgeColourMode::residual, s, {}, 8);
  };
  CHECK(style(1.0).ramp == 0); // well inside the 95 % gate
  CHECK(style(kResidualGood).ramp == 0);
  CHECK(style(kResidualBad).ramp == kRampBuckets - 1);
  CHECK(style(1000.0).ramp == kRampBuckets - 1);
  const int mid = style((kResidualGood + kResidualBad) / 2).ramp;
  CHECK(mid >= 7);
  CHECK(mid <= 8);
}

TEST_CASE("EdgeColour_StagedMeasurement_HasNoResidualYet", "[rux_qt][colour]") {
  auto v = with_residual(50.0);
  v.pending = EdgeView::Pending::added;
  const auto st = edge_style(v, EdgeColourMode::residual,
                             colour_scale(EdgeColourMode::residual, {}), {}, 8);
  CHECK(st.ramp == -1);
  CHECK(st.token == "--color-text-faint");
  CHECK(st.dashed);
}

TEST_CASE("Percentile_EdgeCases", "[rux_qt][colour]") {
  std::vector<double> v;
  for (int i = 1; i <= 100; ++i)
    v.push_back(i);
  CHECK(percentile(v, 0.05) == Approx(5.95));
  CHECK(percentile(v, 0.95) == Approx(95.05));
  CHECK(percentile({7.0}, 0.05) == Approx(7.0));
  CHECK(std::isnan(percentile({}, 0.5)));
  CHECK(percentile({3.0, 1.0, 2.0}, 0.5) == Approx(2.0)); // unsorted input
}

TEST_CASE("EdgeColour_MetricModes_UseThe5To95Percentile", "[rux_qt][colour]") {
  std::vector<EdgeView> drawn;
  for (int i = 1; i <= 100; ++i)
    drawn.push_back(with_deviation(i * 0.01));
  const auto s = colour_scale(EdgeColourMode::deviation, drawn);
  REQUIRE(s.valid);
  CHECK_FALSE(s.fixed);
  CHECK(s.lo == Approx(0.0595));
  CHECK(s.hi == Approx(0.9505));
  CHECK(edge_style(with_deviation(0.01), EdgeColourMode::deviation, s, {}, 8)
            .ramp == 0);
  CHECK(edge_style(with_deviation(2.0), EdgeColourMode::deviation, s, {}, 8)
            .ramp == kRampBuckets - 1);
}

TEST_CASE("EdgeColour_MetricModes_WithoutValues_AreNeutral",
          "[rux_qt][colour]") {
  // Legacy rows / never optimized: no metric anywhere.
  const std::vector<EdgeView> drawn = {view(1, 2), view(2, 3, "odometry")};
  const auto s = colour_scale(EdgeColourMode::seed, drawn);
  CHECK_FALSE(s.valid);
  const auto st = edge_style(drawn[0], EdgeColourMode::seed, s, {}, 8);
  CHECK(st.ramp == -1);
  CHECK(st.token == "--color-text-faint");
  const auto legend = build_legend(EdgeColourMode::seed, drawn, false, 8);
  CHECK(legend.continuous);
  CHECK(legend.missing == 2);
  CHECK_FALSE(legend.scale.valid);
  // One value: lo == hi, every edge at the start of the ramp.
  const std::vector<EdgeView> one = {with_deviation(0.3)};
  const auto s1 = colour_scale(EdgeColourMode::deviation, one);
  CHECK(s1.valid);
  CHECK(edge_style(one[0], EdgeColourMode::deviation, s1, {}, 8).ramp == 0);
}

TEST_CASE("EdgeColour_Type_UnknownTypesGetTheirOwnSwatch", "[rux_qt][colour]") {
  const std::vector<EdgeView> all = {view(1, 2, "odometry"), view(2, 3),
                                     view(3, 4, "panorama"), view(4, 5, "imu"),
                                     view(5, 6, "gravity")};
  const auto unknown = unknown_edge_types(all);
  REQUIRE(unknown == std::vector<std::string>{"gravity", "imu"});
  const ColourScale none;
  CHECK(edge_style(all[0], EdgeColourMode::type, none, unknown, 8).token ==
        "--color-text-faint");
  CHECK_FALSE(edge_style(all[0], EdgeColourMode::type, none, unknown, 8).wide);
  CHECK(edge_style(all[1], EdgeColourMode::type, none, unknown, 8).token ==
        "--color-accent");
  CHECK(edge_style(all[2], EdgeColourMode::type, none, unknown, 8).token ==
        "--color-star");
  CHECK(edge_style(all[3], EdgeColourMode::type, none, unknown, 8).token ==
        "--label-1");
  CHECK(edge_style(all[4], EdgeColourMode::type, none, unknown, 8).token ==
        "--label-0");
  const auto legend = build_legend(EdgeColourMode::type, all, false, 8);
  REQUIRE(legend.entries.size() == 5);
  CHECK(legend.entries[0].label == "Odometri");
  CHECK(legend.entries[3].label == "gravity");
  CHECK(legend.entries[3].count == 1);
}

TEST_CASE("EdgeColour_Status_AndRejectedHiddenByDefault", "[rux_qt][colour]") {
  std::vector<EdgeView> all = {view(1, 2),
                               view(2, 3, "loop_closure", "confirmed"),
                               view(3, 4, "loop_closure", "rejected"),
                               view(4, 5, "loop_closure", "rejected")};
  CHECK_FALSE(edge_drawn(all[2], false));
  CHECK(edge_drawn(all[2], true));
  const auto hidden = build_legend(EdgeColourMode::status, all, false, 8);
  CHECK(hidden.hidden_rejected == 2);
  REQUIRE(hidden.entries.size() == 2); // only what is drawn
  CHECK(hidden.entries[0].label == "Auto");
  CHECK(hidden.entries[1].label == "Bekræftet");
  const auto shown = build_legend(EdgeColourMode::status, all, true, 8);
  REQUIRE(shown.entries.size() == 3);
  CHECK(shown.entries[2].label == "Afvist");
  CHECK(shown.entries[2].count == 2);
  CHECK(shown.entries[2].dashed);
  const auto st = edge_style(all[2], EdgeColourMode::status, {}, {}, 8);
  CHECK(st.token == "--color-status-failed");
  CHECK(st.dashed);
}

TEST_CASE("EdgeColour_Legend_CountsStagedEdges", "[rux_qt][colour]") {
  auto staged = view(7, 8);
  staged.pending = EdgeView::Pending::added;
  const auto legend =
      build_legend(EdgeColourMode::type, {view(1, 2), staged}, false, 8);
  CHECK(legend.staged == 1);
  CHECK(legend.entries.back().label == "Ikke gemt");
  CHECK(legend.entries.back().dashed);
  // Staged edges keep their mode colour (dashed): the legend shows the
  // pattern in a neutral ink, never the selected-edge highlight's colour.
  CHECK(legend.entries.back().token == "--color-on-chrome-muted");
}

TEST_CASE("ColourMode_KeysRoundTrip", "[rux_qt][colour]") {
  for (auto m : colour_modes())
    CHECK(parse_colour_mode(colour_mode_key(m)) == m);
  CHECK(colour_mode_da(EdgeColourMode::seed) == "Frøafvigelse (m)");
  CHECK_FALSE(parse_colour_mode("rainbow").has_value());
}

TEST_CASE("EdgeColour_DrawOrder_WorstOnTop", "[rux_qt][colour]") {
  // A bogus loop closure (the reddest bucket) must be painted over the
  // normal ones, and odometry hairlines under every wide stroke.
  EdgeStyle hair{"--color-text-faint", -1, false, false};
  EdgeStyle missing{"--color-text-faint", -1, false, true};
  EdgeStyle good{"", 2, false, true};
  EdgeStyle bad{"", kRampBuckets - 1, false, true};
  EdgeStyle dashed{"--color-accent", -1, true, true};
  CHECK(edge_draw_order(hair) < edge_draw_order(missing));
  CHECK(edge_draw_order(missing) < edge_draw_order(good));
  CHECK(edge_draw_order(good) < edge_draw_order(bad));
  CHECK(edge_draw_order(bad) < edge_draw_order(dashed));
  CHECK(edge_draw_order(good) ==
        edge_draw_order(EdgeStyle{"", 2, false, true}));
}
