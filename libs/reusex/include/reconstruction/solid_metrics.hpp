// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Solid-model benchmark metrics (reusex-z1v.2.5): score a detected typed
// building model against a reference plan in the same format
// (core/solid_model.hpp, io/solid_model_json.hpp), plus the point-to-solid fit
// of the scan against a mesh. Backs `rux analyze solid` and
// scripts/bench-solid.sh.
//
// Every metric is optional: it is n/a (nullopt, JSON null) when either model
// does not provide the element kind it needs (SolidModel::has_*), so the
// report runs on a project before any solid-model stage exists and each work
// package adds its own line as its table lands.

#include "reusex/core/solid_model.hpp"
#include "reusex/types/point_types.hpp"

#include <cstddef>
#include <optional>
#include <string>

namespace pcl {
struct PolygonMesh;
} // namespace pcl

namespace reusex::geometry {

/// Matching tolerances for compare_solid_models() and fit_report().
struct SolidMetricsOptions {
  /// Two wall axes are candidates for one wall when their directions differ
  /// by at most this much (undirected) [deg].
  double wall_match_angle_deg = 10.0;
  /// ... and the detected axis' midpoint lies within this distance of the
  /// reference axis line [m]. Also bounds how far a detected opening may sit
  /// from the reference opening's host wall.
  double wall_match_dist_m = 0.5;
  /// A detected opening matches a reference opening when their rectangles in
  /// the reference wall's (s, z) face coordinates have at least this IoU.
  double opening_match_iou = 0.5;
  /// A scan point "fits" the solid when it is within this distance [m].
  double fit_within_m = 0.02;
};

/// The benchmark report. Lower is better for errors, RMS and fit distances;
/// higher for IoU, recall, precision and the within-share.
struct SolidModelReport {
  /// detected spaces - reference spaces (signed).
  std::optional<int> room_count_error;
  /// Mean over reference spaces of the IoU with the detected space it was
  /// matched to (one-to-one, greedy by IoU); an unmatched reference space
  /// counts 0.
  std::optional<double> room_iou_mean;
  /// RMS of the matched detected walls' endpoint distances to their reference
  /// wall's axis line [m].
  std::optional<double> wall_position_rms_m;
  /// |detected - reference| thickness over matched walls whose thickness is
  /// known on both sides: mean and nearest-rank P95 [m].
  std::optional<double> wall_thickness_err_mean_m;
  std::optional<double> wall_thickness_err_p95_m;
  std::optional<double> wall_recall;    ///< matched / reference walls
  std::optional<double> wall_precision; ///< matched / detected walls
  std::optional<double> opening_recall;
  std::optional<double> opening_precision;
  /// Point-to-solid fit (fit_report): mean distance [m], and the share of
  /// points within SolidMetricsOptions::fit_within_m.
  std::optional<double> fit_mean_m;
  std::optional<double> fit_within_share;
  /// Mean WallRecord::seen_share over detected walls that report one.
  std::optional<double> seen_share_mean;
  /// Share of detected walls whose thickness was assumed, not measured.
  std::optional<double> assumed_share;

  /// Element counts behind the ratios (0 when the kind is not provided).
  std::size_t reference_walls = 0, detected_walls = 0, matched_walls = 0;
  std::size_t reference_spaces = 0, detected_spaces = 0, matched_spaces = 0;
  std::size_t reference_openings = 0, detected_openings = 0,
              matched_openings = 0;
  std::size_t fit_points = 0;

  /// Every metric as a key; null for each nullopt. Counts under "counts".
  [[nodiscard]] std::string to_json(int indent = 2) const;
};

/// Score @p detected against @p reference.
///
/// Walls are compared storey by storey (WallRecord::storey), in the level
/// frame both models are expressed in. Matching is one-to-one and greedy in a
/// fixed order (best score first, ties by index), so the report is
/// deterministic.
///
/// @throws std::invalid_argument when a space polygon is not simple, or an
///         opening names a host wall its model does not contain.
SolidModelReport compare_solid_models(const core::SolidModel &detected,
                                      const core::SolidModel &reference,
                                      const SolidMetricsOptions &opt = {});

/// Fill the fit metrics of @p report: distance from every finite point of
/// @p cloud to the nearest triangle of @p mesh (a CGAL AABB tree over the
/// mesh's faces; polygons with more than three vertices are fanned). The
/// cloud and the mesh must share a frame.
///
/// @throws std::invalid_argument when the mesh has no usable triangle or the
///         cloud no finite point.
void fit_report(const Cloud &cloud, const pcl::PolygonMesh &mesh,
                const SolidMetricsOptions &opt, SolidModelReport &report);

} // namespace reusex::geometry
