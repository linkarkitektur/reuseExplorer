// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// How the Rediger canvas colours an edge (spec §5.2), Qt-free: by type,
// status, residual (a FIXED χ² scale), deviation or seed disagreement (the
// 5th-95th percentile of the drawn edges), and the legend built from what
// is drawn. Colours are token NAMES; the canvas resolves them through the
// theme and mixes the three ramp tokens for continuous modes.

#include <rux_qt/database_logic.hpp>

#include <array>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace rux::qt {

enum class EdgeColourMode { type, status, residual, deviation, seed };

/// Combo order.
const std::vector<EdgeColourMode> &colour_modes();
std::string_view colour_mode_key(EdgeColourMode mode); ///< QSettings value
std::optional<EdgeColourMode> parse_colour_mode(std::string_view key);
/// "Type", "Status", "Residual", "Afvigelse (m)", "Frøafvigelse (m)".
std::string colour_mode_da(EdgeColourMode mode);

/// χ²₆ at 0.95 (a 6-DoF Pose3 factor). residual is 0.5·χ², so the gate is
/// half of it.
inline constexpr double kChi2Dof6P95 = 12.591587243743977;
inline constexpr double kResidualGood = kChi2Dof6P95 / 2.0;
inline constexpr double kResidualBad = 5.0 * kResidualGood;
inline constexpr int kRampBuckets = 16;
/// good -> middle -> bad.
inline constexpr std::array<std::string_view, 3> kRampTokens = {
    "--color-status-succeeded", "--color-star", "--color-status-failed"};

/// The value @p mode colours by: residual (none for a staged measurement),
/// deviation_m, seed_disagreement_m; nullopt for type/status or when absent.
std::optional<double> edge_metric(const EdgeView &v, EdgeColourMode mode);

/// Linear-interpolated percentile, @p p in [0, 1]; NaN for no values.
double percentile(std::vector<double> values, double p);

struct ColourScale {
  double lo = 0.0;
  double hi = 0.0;
  bool fixed = false; ///< the residual's χ² scale
  bool valid = false; ///< false: nothing to scale (no values, or categorical)
};
ColourScale colour_scale(EdgeColourMode mode,
                         const std::vector<EdgeView> &drawn);
/// Position of @p value on @p scale in [0, 1] (0 when lo == hi or invalid).
double ramp_position(double value, const ColourScale &scale);
int ramp_bucket(double t); ///< [0, kRampBuckets)

/// Rejected edges are drawn only when the toggle shows them.
bool edge_drawn(const EdgeView &v, bool show_rejected);
/// Types other than odometry / loop_closure / panorama, sorted.
std::vector<std::string> unknown_edge_types(const std::vector<EdgeView> &edges);

struct EdgeStyle {
  std::string token;   ///< categorical colour token; empty when @ref ramp set
  int ramp = -1;       ///< ramp bucket, or -1
  bool dashed = false; ///< staged, or rejected (shown)
  bool wide = true;    ///< false: an odometry hairline
};
EdgeStyle edge_style(const EdgeView &v, EdgeColourMode mode,
                     const ColourScale &scale,
                     const std::vector<std::string> &unknown_types,
                     int label_slots);

struct LegendEntry {
  std::string token;
  std::string label;
  int count = 0;
  bool dashed = false;
};
struct EdgeLegend {
  std::vector<LegendEntry> entries; ///< categorical buckets (+ "Ikke gemt")
  bool continuous = false;          ///< a colour bar from scale.lo to hi
  ColourScale scale;
  std::string unit; ///< "m", or "" for the residual
  int missing = 0;  ///< drawn edges without the metric
  int hidden_rejected = 0;
  int staged = 0;
};
EdgeLegend build_legend(EdgeColourMode mode, const std::vector<EdgeView> &all,
                        bool show_rejected, int label_slots);

} // namespace rux::qt
