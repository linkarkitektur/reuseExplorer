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
  /// ... and the two axes overlap along the reference axis by at least this
  /// fraction of their combined extent (1D IoU along the reference axis), so
  /// a short fragment lying on the line cannot take the place of the full
  /// wall. Candidates that pass are ranked by midpoint distance.
  double wall_match_min_overlap = 0.5;
  /// A detected opening matches a reference opening when their rectangles in
  /// the reference wall's (s, z) face coordinates have at least this IoU.
  double opening_match_iou = 0.5;
  /// When both models carry storey records, a detected storey is compared
  /// with the reference storey whose floor is nearest in world z, if within
  /// this distance [m] (see compare_solid_models()).
  double storey_match_dist_m = 1.0;
  /// A scan point "fits" the solid when it is within this distance [m].
  double fit_within_m = 0.02;
  /// A point of the solid mesh is "supported" when a scan point lies within
  /// this distance [m]. 0.25 m ignores the few-cm offsets of a fitted plane
  /// but not a surface the scan never saw (a fake wall between rooms):
  /// on NewOffice it separates the over-split open office's mesh (0.20
  /// unsupported) from the one built on merged rooms (0.12) best of
  /// 0.05-0.5 m (reusex-rqn, docs/research/solid-model-ideas-lab.md).
  double support_dist_m = 0.25;
  /// About one support sample per this x this of mesh area [m].
  double support_spacing_m = 0.05;
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
  /// wall's axis line [m]. Face to face when one of the two is drawn on one
  /// face (thickness unknown or assumed) and the other has measured faces:
  /// the distance to the nearer of those faces (reusex-v8p).
  std::optional<double> wall_position_rms_m;
  /// |detected - reference| thickness over matched walls whose thickness is
  /// known (> 0) and measured (not WallRecord::assumed) on both sides: mean
  /// and nearest-rank P95 [m].
  std::optional<double> wall_thickness_err_mean_m;
  std::optional<double> wall_thickness_err_p95_m;
  std::optional<double> wall_recall;    ///< matched / reference walls
  std::optional<double> wall_precision; ///< matched / detected walls
  std::optional<double> opening_recall;
  std::optional<double> opening_precision;
  /// Mean over reference openings of the IoU with the detected opening it is
  /// paired with one-to-one (greedy by IoU, any overlap, no
  /// opening_match_iou threshold); an unpaired reference opening counts 0.
  /// Shows how close the misses of opening_recall came.
  std::optional<double> opening_iou_mean;
  /// Point-to-solid fit (fit_report): mean distance [m], and the share of
  /// points within SolidMetricsOptions::fit_within_m.
  std::optional<double> fit_mean_m;
  std::optional<double> fit_within_share;
  /// Solid-to-scan support (fit_report): the mesh's area [m^2], and the share
  /// of it with no scan point within SolidMetricsOptions::support_dist_m --
  /// surfaces the scan does not show, which the point-to-solid fit cannot see.
  std::optional<double> mesh_area_m2;
  std::optional<double> mesh_unsupported_share;
  /// Mean WallRecord::seen_share over detected walls that report one.
  std::optional<double> seen_share_mean;
  /// Share of detected walls flagged WallRecord::assumed (thickness
  /// defaulted, not measured). The record has no "unknown" state, so a
  /// detector that never sets the flag reports 0, not n/a.
  std::optional<double> assumed_share;

  /// Where the fit metrics came from ("project '<path>': cloud 'c' vs mesh
  /// 'm'"), empty when they were not computed. Set by the caller.
  std::string fit_source;

  /// Element counts behind the ratios (0 when the kind is not provided).
  std::size_t reference_walls = 0, detected_walls = 0, matched_walls = 0;
  std::size_t reference_spaces = 0, detected_spaces = 0, matched_spaces = 0;
  std::size_t reference_openings = 0, detected_openings = 0,
              matched_openings = 0;
  std::size_t fit_points = 0;
  std::size_t support_samples = 0; ///< mesh samples behind the support share
  /// Matched wall pairs scored for thickness, and pairs left out because one
  /// side's thickness was assumed.
  std::size_t thickness_pairs = 0, thickness_assumed_excluded = 0;
  /// Detected walls, spaces and openings on storeys the reference does not
  /// cover, left out of every metric (precision, room count, ...).
  std::size_t detected_outside_reference = 0;

  /// Every metric as a key; null for each nullopt; "fit_source" (null when
  /// empty). Counts under "counts".
  [[nodiscard]] std::string to_json(int indent = 2) const;
};

/// Score @p detected against @p reference.
///
/// Walls, spaces and openings (through their host wall) are compared storey
/// by storey, in world: each detected element is taken out of its storey's
/// level frame (the inverse of StoreyRecord::level_transform) and into the
/// level frame of the reference storey it is compared with, so two models
/// whose level frames differ (a reference authored in world x/y with its
/// floor at z = 0, a detection whose origin is a floor point of that run)
/// still line up. A storey without a record has the identity frame.
///
/// Storey correspondence: when both models carry storey records, a detected
/// storey corresponds to the reference storey whose floor (z_floor, taken to
/// world) is nearest in world z, within
/// SolidMetricsOptions::storey_match_dist_m, one-to-one, closest first;
/// otherwise storeys correspond by idx. Detected elements on a storey with
/// no corresponding reference storey, or on one the reference does not
/// cover (neither in its `storeys` nor on any of its walls or spaces), are
/// left out and counted in SolidModelReport::detected_outside_reference;
/// when the reference names no storey at all, nothing is left out.
/// Matching is one-to-one and greedy in a fixed order (best score first,
/// ties by index), so the report is deterministic.
///
/// @throws std::invalid_argument when a wall axis has zero length, a space
///         polygon is not a valid polygon with holes (non-simple or
///         zero-area ring), or an opening names a host wall its model does
///         not contain (io::solid_model_from_json already rejects all of
///         these; the checks guard models built in memory), or a storey's
///         level_transform is singular or not finite.
SolidModelReport compare_solid_models(const core::SolidModel &detected,
                                      const core::SolidModel &reference,
                                      const SolidMetricsOptions &opt = {});

/// Fill the fit metrics of @p report: distance from every finite point of
/// @p cloud to the nearest triangle of @p mesh (a CGAL AABB tree over the
/// mesh's faces; polygons with more than three vertices are fanned), and the
/// reverse direction, the share of the mesh's area with no cloud point within
/// SolidMetricsOptions::support_dist_m (geometry::mesh_support). The cloud
/// and the mesh must share a frame.
///
/// @throws std::invalid_argument when the mesh has no usable (non-degenerate)
///         triangle, a face references a vertex the mesh does not have, or
///         the cloud has no finite point. @p report is left unchanged then.
void fit_report(const Cloud &cloud, const pcl::PolygonMesh &mesh,
                const SolidMetricsOptions &opt, SolidModelReport &report);

} // namespace reusex::geometry
