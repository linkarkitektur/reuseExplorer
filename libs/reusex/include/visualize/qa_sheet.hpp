// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// The mesh-QA contact sheet (reusex-8vx, `rux render --qa`): one image that
// shows a mesh against the cloud it was built from — a plan cut, the four
// elevations, a few vertical sections and a deviation plan — so a mesh change
// is checked from every side, not from one flattering angle.

#include "reusex/visualize/render_view.hpp"

#include <opencv2/core/mat.hpp>

#include <array>
#include <string>
#include <vector>

namespace reusex::visualize {

/// What render_qa_sheet() draws.
struct QaSheetOptions {
  /// Every panel starts from this: cloud and mesh names, point size, the cut
  /// height of the plan panels, the deviation thresholds, an optional plan
  /// overlay, the yaw. Its layers, view, section and size are set per panel.
  RenderOptions base;
  /// The vertical sections to show; empty = kDefaultQaSections through the
  /// scene's plan extent (see default_qa_sections()).
  std::vector<SectionSpec> sections;
  /// Size of one panel [px].
  int panel_width = 960;
  int panel_height = 720;
  /// Panels per row of the sheet.
  int columns = 3;
  /// Framing quantile of every panel (RenderOptions::frame_quantile): stray
  /// points beyond the building do not shrink it.
  double frame_quantile = 0.002;
};

/// One panel of the sheet.
struct QaPanel {
  std::string name;  ///< file-name-safe key, e.g. "plan", "elev_front"
  std::string title; ///< caption drawn on the sheet
  cv::Mat image;     ///< BGR, panel_width x panel_height
};

/// The sheet and what went into it.
struct QaSheet {
  cv::Mat sheet; ///< BGR: a caption bar over a grid of every panel
  std::vector<QaPanel> panels;
  /// The deviation panel's summary (all points, and points on a segmented
  /// plane when the project has `planes`).
  std::optional<DeviationStats> deviation;
  std::optional<DeviationStats> deviation_planar;
};

/// The sections the sheet shows when QaSheetOptions::sections is empty, as
/// world segments square to the display frame (the view yaw @p yaw_deg, so
/// the cuts follow the building's axes as the plan shows them). Positions are
/// quantiles of the points, not of the box, so they land where the building
/// is (an L or U footprint leaves most of its box empty): one at the median
/// display u, running +v, and two at the 1/3 and 2/3 quantiles of display v,
/// running +u. Each line spans @p box (display frame, u v z) plus the slab
/// thickness. End points are rounded to the centimetre.
/// @p u / @p v are the points' display coordinates (reordered in place).
std::vector<SectionSpec>
default_qa_sections(std::vector<double> &u, std::vector<double> &v,
                    const std::array<double, 3> &box_min,
                    const std::array<double, 3> &box_max, double yaw_deg,
                    double thickness);

/// Render the mesh-QA contact sheet of @p db: the plan cut with the mesh
/// overlaid and its section line, the front / back / left / right
/// elevations of cloud + mesh, the vertical sections (slab of cloud, the
/// mesh's section line) and the deviation plan (cloud coloured by distance to
/// the mesh, with legend and summary). Deterministic (STANDARDS §6).
///
/// @throws OffscreenGlUnavailable without offscreen OpenGL, and
///         std::runtime_error when the project lacks the cloud or the mesh, or
///         the options are invalid (see render_view()).
QaSheet render_qa_sheet(const ProjectDB &db, const QaSheetOptions &opts);

} // namespace reusex::visualize
