// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// The drift-loop render layers (reusex-rcv.1.4): what Layer::passes,
// Layer::graph and Layer::drift draw, as plain data, and the helpers they
// share with a caller.
//
//  - passes: every point coloured by the time segment of the frame it came
//    from, so a doubled wall shows as two colours. The stored clouds carry no
//    per-point provenance, so the points come from preview_cloud(): the depth
//    frames back-projected on the fly (also what `rux optimize --watch`
//    previews a candidate solve with).
//  - graph: the camera trajectory (coloured by the same segments), the
//    stored pose-graph edges coloured by their solve residual on the fixed
//    chi-square scale of the Rediger GraphPanel, node-id labels along the
//    path, and pose-edit pins.
//  - drift: the pass-to-pass residual cells of `rux analyze drift`
//    (slam::diagnose_drift) as a heatmap, one per storey. The layer reads a
//    DriftHeatmap, a POD the caller fills from an in-memory DriftReport or
//    from the report's JSON (drift_heatmap_from_json()), so reusex_visualize
//    does not link reusex_slam.
//
// Header hygiene (STANDARDS §2): Eigen + the core PODs only; no VTK, no
// ProjectDB definition.

#include "reusex/core/frame_depth_cache.hpp"
#include "reusex/core/pose_edits.hpp"

#include <Eigen/Core>

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace reusex {
class ProjectDB;
}

namespace reusex::visualize {

struct LabelPalette;

// ── Passes ──────────────────────────────────────────────────────────────────

/// The time segment of sampled frame @p k of @p n, in [0, segments): equal
/// frame counts, the rule slam::diagnose_drift() cuts its segments by, so a
/// pass colour is the same segment index the drift report names.
int pass_segment(std::size_t k, std::size_t n, int segments);

/// The colour of segment @p segment: `--label-<segment mod size>` of
/// @p palette, so a pass has one colour in `rux render`, `rux view` and the
/// Qt client. Segments beyond the palette size repeat colours (warned by the
/// passes layer).
std::array<std::uint8_t, 3> pass_color(const LabelPalette &palette,
                                       int segment);

/// How the passes layer back-projects the frames (preview_cloud()).
struct PreviewCloudOptions {
  int segments = 8;         ///< time segments (one colour each)
  int frame_step = 1;       ///< use every Nth sensor frame (node order)
  int depth_stride = 8;     ///< sample every Nth depth pixel
  double min_depth_m = 0.2; ///< drop closer returns
  double max_depth_m = 4.0; ///< drop farther returns
  /// Keep one point per voxel AND segment (0 = keep every point): thins the
  /// overlap of one pass without merging two passes' surfaces.
  double voxel_m = 0.02;
};

/// One segment of a preview cloud: its node range and point count.
struct PassSegment {
  int index = 0;
  std::int64_t first_node = 0, last_node = 0;
  std::size_t frames = 0, points = 0;
};

/// Back-projected depth with per-point provenance.
struct PreviewCloud {
  std::vector<Eigen::Vector3f> points; ///< world [m]
  /// Per point: index into @ref node_ids (the sampled frame it came from).
  std::vector<std::uint32_t> frame;
  std::vector<std::int64_t> node_ids;   ///< sampled frames, node order
  std::vector<int> frame_segment;       ///< per sampled frame
  std::vector<PassSegment> segments;    ///< every segment with a frame
  std::size_t frames_without_depth = 0; ///< sampled, posed, no usable depth
  std::size_t frames_without_pose = 0;  ///< sampled, no pose
};

/// Back-project the sensor frames of @p db into one world cloud that
/// remembers each point's frame (the passes layer, `optimize --watch`).
///
/// @param poses optional world poses replacing the stored ones (a candidate
///        solve), index-aligned with db.sensor_frame_ids() (all frames,
///        before frame_step); the camera sits at pose * local_transform.
/// Deterministic (STANDARDS §6): frames in node order, pixels in raster
/// order, the first point of a (voxel, segment) kept.
/// @param depth_cache optional sampled depth of @p db's frames
///        (core::cache_frame_depth() with preview_depth_sampling(opts)): the
///        frames are back-projected from it instead of read from @p db, with
///        a bit-identical result (reusex-rcv.1.17).
/// @throws std::invalid_argument for a @p poses of the wrong size, a
///         @p depth_cache made with other sampling or for another frame
///         count, or options out of range; std::runtime_error when no frame
///         yields a point.
PreviewCloud preview_cloud(const ProjectDB &db,
                           const PreviewCloudOptions &opts = {},
                           std::span<const Eigen::Matrix4d> poses = {},
                           const core::FrameDepthCache *depth_cache = nullptr);

/// The depth sampling preview_cloud() reads with @p opts: the
/// core::cache_frame_depth() argument of a cache it accepts.
core::DepthSampling preview_depth_sampling(const PreviewCloudOptions &opts);

// ── Graph ───────────────────────────────────────────────────────────────────

/// chi-square(6 DoF) at 0.95: the Rediger GraphPanel's fixed residual scale.
/// A stored residual is 0.5 * chi-square, so an edge is "good" up to half of
/// it and "bad" at five times that (rux_qt/edge_colouring.hpp).
inline constexpr double kChi2Dof6P95 = 12.591587243743977;

/// What the graph layer draws.
struct GraphLayerOptions {
  /// Edge residual (0.5 chi-square) drawn green up to this ...
  double residual_good = kChi2Dof6P95 / 2.0;
  /// ... and vermillion at this; reddish purple beyond.
  double residual_bad = 5.0 * kChi2Dof6P95 / 2.0;
  /// Draw odometry edges too (they run along the trajectory).
  bool odometry_edges = false;
  /// Draw rejected edges too.
  bool rejected_edges = false;
  /// Label every Nth posed node with its id along the path (0 = none).
  int label_every = 20;
  /// Height of a node label [m] (0 = derived from the scene: 1.2 % of the
  /// trajectory's plan extent, at least 0.15 m).
  double label_height_m = 0.0;
  double trajectory_width = 2.0; ///< [px]
  double edge_width = 2.5;       ///< [px]
  /// Pin colour of a pose edit (`--label-4`, blue).
  std::array<std::uint8_t, 3> pin_rgb{0x00, 0x72, 0xb2};
  /// Edge colour when the edge has no residual (never solved).
  std::array<std::uint8_t, 3> unsolved_rgb{0x9a, 0xa0, 0xa6};
};

/// One pose-graph edge as the graph layer draws it: by default the stored
/// rows (ProjectDB::list_pose_graph_edges()); a caller with a candidate solve
/// in memory (`rux optimize --watch`, reusex-rcv.1.18) hands in the same
/// edges with that solve's residuals (RenderOptions::graph_edges).
struct GraphLayerEdge {
  std::int64_t from_node_id = 0, to_node_id = 0;
  std::string edge_type;       ///< "odometry" | "loop_closure" | ...
  std::string status = "auto"; ///< "auto" | "confirmed" | "rejected"
  /// 0.5 x whitened squared residual; NaN = not solved (drawn grey).
  double residual = std::numeric_limits<double>::quiet_NaN();
};

/// The colour of an edge with residual @p r: green at 0, yellow at
/// @p good, vermillion at @p bad, reddish purple beyond (the deviation ramp,
/// deviation_color()); the unsolved grey for NaN.
std::array<std::uint8_t, 3> residual_color(double r,
                                           const GraphLayerOptions &opts);

// ── Drift heatmap ───────────────────────────────────────────────────────────

/// A storey the heatmap's cells are keyed by (slam::DriftStorey).
struct DriftHeatStorey {
  int idx = 0;
  std::string name;
  double z_floor = 0.0, z_ceiling = 0.0; ///< level frame [m]
  /// World -> level frame, row-major.
  std::array<double, 16> level_transform{1, 0, 0, 0, 0, 1, 0, 0,
                                         0, 0, 1, 0, 0, 0, 0, 1};
};

/// One heat cell (slam::DriftCell).
struct DriftHeatCell {
  int storey = -1;         ///< DriftHeatStorey::idx; -1 = world cell
  double x = 0.0, y = 0.0; ///< centre, the storey's level frame or world
  double z_m = 0.0;        ///< low z of its points (world cells are drawn here)
  double residual_m = 0.0; ///< median pass-to-pass residual
  std::size_t points = 0;
};

/// The drift layer's input: the cells of a drift report.
struct DriftHeatmap {
  double cell_m = 0.5;
  std::optional<double> noise_floor_m; ///< for the legend
  std::optional<double> residual_median_m;
  std::vector<DriftHeatStorey> storeys; ///< empty = world cells
  std::vector<DriftHeatCell> cells;
};

/// Parse the JSON `rux analyze drift -o` writes (slam::drift_report_to_json;
/// a report without "storeys" / "cell_m" reads as world cells of 0.5 m).
/// @throws std::invalid_argument when @p json is not such a report.
DriftHeatmap drift_heatmap_from_json(std::string_view json);

/// What the drift layer draws.
struct DriftLayerOptions {
  /// A cell this good is green (about twice the sensor noise floor of the
  /// captures in docs/research/visual-drift-loop.md, 4-8 mm) ...
  double good_m = 0.015;
  /// ... vermillion at this; reddish purple beyond (a pass break: the
  /// top few percent of house2's cells).
  double bad_m = 0.050;
  /// Draw only this storey's cells; std::nullopt = every storey (a plan view
  /// with a cut picks the storey the cut lies in, render_view()).
  std::optional<int> storey;
  /// Cell opacity (the cloud under it shows through).
  double opacity = 0.75;
  /// A storey cell is drawn this far above its storey's floor [m].
  double lift_m = 0.03;
};

/// The colour of a cell with median residual @p r (the deviation ramp).
std::array<std::uint8_t, 3> drift_color(double r,
                                        const DriftLayerOptions &opts);

// ── Legends ─────────────────────────────────────────────────────────────────

/// What the drift-loop layers drew, for a legend (SceneInfo::drift_legend).
struct DriftLegendInfo {
  /// passes (and the trajectory's colours): one entry per segment.
  struct Pass {
    int index = 0;
    std::int64_t first_node = 0, last_node = 0;
    std::array<std::uint8_t, 3> rgb{0, 0, 0};
    std::size_t points = 0;
  };
  std::vector<Pass> passes;
  bool passes_drawn = false;
  std::size_t pass_points = 0;

  bool graph_drawn = false;
  std::size_t graph_nodes = 0;
  std::size_t graph_edges = 0;     ///< edges drawn
  std::size_t graph_edges_bad = 0; ///< residual >= residual_bad
  std::size_t graph_edges_unsolved = 0;
  std::size_t graph_pins = 0; ///< pose-edit pins
  GraphLayerOptions graph;

  bool drift_drawn = false;
  std::size_t drift_cells = 0;     ///< cells drawn
  std::optional<int> drift_storey; ///< the storey shown; nullopt = all
  std::string drift_storey_name;
  double drift_max_m = 0.0; ///< worst cell drawn
  std::optional<double> noise_floor_m;
  DriftLayerOptions drift;
};

} // namespace reusex::visualize
