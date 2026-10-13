// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <rux_qt/edge_colouring.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>

namespace rux::qt {
namespace {

bool known_type(const std::string &t) {
  return t == "odometry" || t == "loop_closure" || t == "panorama";
}

std::string type_token(const std::string &type,
                       const std::vector<std::string> &unknown, int slots) {
  if (type == "odometry")
    return "--color-text-faint";
  if (type == "loop_closure")
    return "--color-accent";
  if (type == "panorama")
    return "--color-star";
  const auto it = std::find(unknown.begin(), unknown.end(), type);
  const auto i = it == unknown.end() ? 0 : it - unknown.begin();
  const int slot = static_cast<int>(i) % std::max(slots, 1);
  return "--label-" + std::to_string(slot); // token-lint: allow (--label-N)
}

std::string status_token(const std::string &status) {
  if (status == "confirmed")
    return "--color-status-succeeded";
  if (status == "rejected")
    return "--color-status-failed";
  return "--color-on-chrome-muted";
}

std::string status_da(const std::string &status) {
  if (status == "confirmed")
    return "Bekræftet";
  if (status == "rejected")
    return "Afvist";
  return "Auto";
}

} // namespace

const std::vector<EdgeColourMode> &colour_modes() {
  static const std::vector<EdgeColourMode> m = {
      EdgeColourMode::type, EdgeColourMode::status, EdgeColourMode::residual,
      EdgeColourMode::deviation, EdgeColourMode::seed};
  return m;
}

std::string_view colour_mode_key(EdgeColourMode m) {
  switch (m) {
  case EdgeColourMode::type:
    return "type";
  case EdgeColourMode::status:
    return "status";
  case EdgeColourMode::residual:
    return "residual";
  case EdgeColourMode::deviation:
    return "deviation";
  case EdgeColourMode::seed:
    return "seed";
  }
  return "type";
}

std::optional<EdgeColourMode> parse_colour_mode(std::string_view key) {
  for (auto m : colour_modes())
    if (colour_mode_key(m) == key)
      return m;
  return std::nullopt;
}

std::string colour_mode_da(EdgeColourMode m) {
  switch (m) {
  case EdgeColourMode::type:
    return "Type";
  case EdgeColourMode::status:
    return "Status";
  case EdgeColourMode::residual:
    return "Residual";
  case EdgeColourMode::deviation:
    return "Afvigelse (m)";
  case EdgeColourMode::seed:
    return "Frøafvigelse (m)";
  }
  return "Type";
}

std::optional<double> edge_metric(const EdgeView &v, EdgeColourMode mode) {
  switch (mode) {
  case EdgeColourMode::residual:
    if (v.pending == EdgeView::Pending::added ||
        v.pending == EdgeView::Pending::replaced)
      return std::nullopt;
    return v.edge.residual;
  case EdgeColourMode::deviation:
    return v.edge.deviation_m;
  case EdgeColourMode::seed:
    return v.edge.seed_disagreement_m;
  default:
    return std::nullopt;
  }
}

double percentile(std::vector<double> v, double p) {
  if (v.empty())
    return std::numeric_limits<double>::quiet_NaN();
  std::sort(v.begin(), v.end());
  const double pos =
      std::clamp(p, 0.0, 1.0) * static_cast<double>(v.size() - 1);
  const auto i = static_cast<std::size_t>(std::floor(pos));
  const double frac = pos - static_cast<double>(i);
  return i + 1 < v.size() ? v[i] + frac * (v[i + 1] - v[i]) : v[i];
}

ColourScale colour_scale(EdgeColourMode mode,
                         const std::vector<EdgeView> &drawn) {
  ColourScale s;
  if (mode == EdgeColourMode::residual) {
    s.lo = kResidualGood;
    s.hi = kResidualBad;
    s.fixed = true;
    s.valid = true;
    return s;
  }
  if (mode != EdgeColourMode::deviation && mode != EdgeColourMode::seed)
    return s;
  std::vector<double> values;
  for (const auto &v : drawn)
    if (auto m = edge_metric(v, mode); m && std::isfinite(*m))
      values.push_back(*m);
  if (values.empty())
    return s;
  s.lo = percentile(values, 0.05);
  s.hi = percentile(values, 0.95);
  s.valid = true;
  return s;
}

double ramp_position(double value, const ColourScale &s) {
  if (!s.valid || !std::isfinite(value))
    return 0.0;
  if (!(s.hi > s.lo))
    return value > s.hi ? 1.0 : 0.0;
  return std::clamp((value - s.lo) / (s.hi - s.lo), 0.0, 1.0);
}

int ramp_bucket(double t) {
  return std::clamp(static_cast<int>(t * kRampBuckets), 0, kRampBuckets - 1);
}

bool edge_drawn(const EdgeView &v, bool show_rejected) {
  return show_rejected || v.edge.status != "rejected";
}

std::vector<std::string>
unknown_edge_types(const std::vector<EdgeView> &edges) {
  std::set<std::string> s;
  for (const auto &v : edges)
    if (!known_type(v.edge.key.type))
      s.insert(v.edge.key.type);
  return {s.begin(), s.end()};
}

EdgeStyle edge_style(const EdgeView &v, EdgeColourMode mode,
                     const ColourScale &scale,
                     const std::vector<std::string> &unknown, int slots) {
  EdgeStyle s;
  s.wide = v.edge.key.type != "odometry";
  s.dashed =
      v.pending != EdgeView::Pending::none || v.edge.status == "rejected";
  switch (mode) {
  case EdgeColourMode::type:
    s.token = type_token(v.edge.key.type, unknown, slots);
    break;
  case EdgeColourMode::status:
    s.token = status_token(v.edge.status);
    break;
  default:
    if (auto m = edge_metric(v, mode); m && scale.valid && std::isfinite(*m))
      s.ramp = ramp_bucket(ramp_position(*m, scale));
    else
      s.token = "--color-text-faint";
  }
  return s;
}

EdgeLegend build_legend(EdgeColourMode mode, const std::vector<EdgeView> &all,
                        bool show_rejected, int slots) {
  EdgeLegend l;
  std::vector<EdgeView> drawn;
  for (const auto &v : all) {
    if (!edge_drawn(v, show_rejected)) {
      ++l.hidden_rejected;
      continue;
    }
    drawn.push_back(v);
    l.staged += v.pending != EdgeView::Pending::none;
  }
  auto count_if = [&](auto pred) {
    return static_cast<int>(std::count_if(drawn.begin(), drawn.end(), pred));
  };
  if (mode == EdgeColourMode::type) {
    const auto unknown = unknown_edge_types(drawn);
    std::vector<std::string> order = {"odometry", "loop_closure", "panorama"};
    order.insert(order.end(), unknown.begin(), unknown.end());
    for (const auto &t : order)
      if (int n =
              count_if([&](const EdgeView &v) { return v.edge.key.type == t; }))
        l.entries.push_back(
            {type_token(t, unknown, slots), edge_type_da(t), n, false});
  } else if (mode == EdgeColourMode::status) {
    for (const std::string st : {"auto", "confirmed", "rejected"})
      if (int n =
              count_if([&](const EdgeView &v) { return v.edge.status == st; }))
        l.entries.push_back(
            {status_token(st), status_da(st), n, st == "rejected"});
  } else {
    l.continuous = true;
    l.scale = colour_scale(mode, drawn);
    l.unit = mode == EdgeColourMode::residual ? "" : "m";
    l.missing = count_if([&](const EdgeView &v) {
      const auto m = edge_metric(v, mode);
      return !m || !std::isfinite(*m);
    });
  }
  if (l.staged > 0)
    l.entries.push_back({"--color-on-chrome", "Ikke gemt", l.staged, true});
  return l;
}

} // namespace rux::qt
