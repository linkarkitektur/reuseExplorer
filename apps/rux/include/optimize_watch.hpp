// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// `rux optimize --edits e.yaml --watch` (reusex-rcv.1.5): the pose loop.
// Render once, then on every save of the edits file re-solve the stored pose
// graph from the initial poses with the edits as factors (in memory: the
// project is written only with --apply), back-project a fresh preview cloud
// with the candidate poses, re-render the drift-loop panels and print a
// metrics block whose suggestions paste straight into the edits file.
//
// The loop is a thin shell around OptimizeWatchSession::pass(), which tests
// drive directly, without sleeping (like render_watch.hpp).

#include "global-params.hpp"
#include "optimize.hpp"

#include <reusex/core/pose_edits.hpp>
#include <reusex/core/solid_model.hpp>
#include <reusex/slam/PlaneGraphOptimizer.hpp>
#include <reusex/visualize/drift_layers.hpp>

#include <Eigen/Core>

#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace rux {

/// The pose change between two pose sets (translation of the pose origin).
struct PoseChange {
  std::size_t nodes = 0;
  double median_m = 0.0, p90_m = 0.0, max_m = 0.0;
  std::int64_t max_node = -1;
  double max_rot_deg = 0.0;
};

/// reusex-sb2: how far the nodes NO edit names moved against the solve
/// without edits. A GT-free residual can improve because an edit shears the
/// whole trajectory; this is the number that shows it.
struct ShearIndicator {
  std::size_t untouched = 0; ///< nodes outside every edit's node set
  PoseChange change;         ///< of those, vs the no-edit solve
  std::size_t flagged = 0;   ///< moved more than shear_flag_m
  /// Per drift segment with no edited node: the largest move.
  struct Segment {
    int index = 0;
    std::int64_t first_node = 0, last_node = 0;
    double max_m = 0.0;
  };
  std::vector<Segment> segments;
};

/// What one pass of the loop did and measured.
struct OptimizeWatchPass {
  int index = 0;            ///< 0 = the initial pass
  bool edits_read = false;  ///< the edits file was (re)read this pass
  bool edits_ok = false;    ///< ... accepted (parsed AND lowered)
  std::string edits_error;  ///< the reader's / lowering's error otherwise
  bool solved = false;      ///< a new solve is on screen
  int shown_solve = -1;     ///< the pass whose solve the block describes
  bool rendered = false;    ///< images were (re)written
  std::string render_error; ///< why rendering failed, if it did
  std::vector<std::filesystem::path> written;
  bool applied = false; ///< poses written to the project (--apply)
  /// Candidate poses (sensor_frames.transform convention), index-aligned
  /// with ProjectDB::sensor_frame_ids(); frames outside the solve keep their
  /// stored pose.
  std::vector<Eigen::Matrix4d> poses;
  std::optional<double> cross_segment_median_m;
  ShearIndicator shear;
  /// The suggestions as a ready-to-paste YAML list (items of `edits:`).
  std::string suggestions_yaml;
  /// The same, one ready-to-paste item (with its comment line) per edit.
  std::vector<std::string> suggestion_items;
  std::string report; ///< the block printed to stdout
  std::string json;   ///< the same block for agents (--json)
  double seconds = 0.0;
};

class OptimizeWatchSession {
    public:
  OptimizeWatchSession(SubcommandOptimizeOptions opt, RuxOptions global_opt);
  ~OptimizeWatchSession();
  OptimizeWatchSession(const OptimizeWatchSession &) = delete;
  OptimizeWatchSession &operator=(const OptimizeWatchSession &) = delete;

  /// One pass: re-read the edits when @p edits_changed (a file that does not
  /// parse or lower keeps the last good solve on screen and nothing is
  /// re-solved), re-solve, diagnose, render, report. The first pass loads
  /// the graph and always solves (without edits when the file is not good
  /// yet). Never throws: every failure is reported in the result.
  OptimizeWatchPass pass(bool edits_changed);

  /// The files the loop watches: the edits file.
  [[nodiscard]] std::vector<std::filesystem::path> watched_paths() const;

    private:
  struct State;
  std::unique_ptr<State> state_;
};

/// The `--watch` loop: pass(true), then one pass per save until SIGINT /
/// SIGTERM or --watch-cycles.
int run_optimize_watch(SubcommandOptimizeOptions const &opt,
                       const RuxOptions &global_opt);

/// Ask a running watch loop to stop, as SIGINT does.
void request_optimize_watch_stop();

/// Statistics of |t(a_k) - t(b_k)| over @p ids (nodes with both poses).
PoseChange pose_change(const std::vector<std::int64_t> &ids,
                       const std::vector<Eigen::Matrix4d> &a,
                       const std::vector<Eigen::Matrix4d> &b,
                       const std::vector<bool> *mask = nullptr);

/// Node ids an edit set names (anchor node, move / on_plan_wall range,
/// relative and same_surface sides; not reject_edge), sorted, among @p ids.
std::vector<std::int64_t> edited_nodes(const reusex::core::PoseEdits &edits,
                                       const std::vector<std::int64_t> &ids);

/// The rigid delta D (about the world origin) that best moves the poses
/// @p from onto @p to (each pose contributes its origin and the tips of its
/// unit axes; Kabsch, no scale). A segment ICP correction measured on a
/// candidate solve, D_c * candidate, becomes the `move` delta on the
/// initial poses the solve seeds from: fit(initial -> D_c * candidate).
Eigen::Matrix4d fit_rigid_delta(const std::vector<Eigen::Matrix4d> &from,
                                const std::vector<Eigen::Matrix4d> &to);

/// reusex-rcv.1.18: the pose-graph edges of @p graph as the graph layer
/// draws them, coloured by the candidate solve @p result (its per-edge
/// residuals, PoseGraphEdgeRecord::tag = index in graph.edges) instead of
/// the residuals last written to the project. An edge that did not enter
/// the solve has none (NaN, drawn grey). @p stored_status: the status of
/// each stored row (ProjectDB::list_pose_graph_edges() order, which
/// graph.edges follows); edges beyond it (odometry derived in memory) are
/// "auto".
std::vector<reusex::visualize::GraphLayerEdge>
candidate_graph_edges(const reusex::geometry::StoredPoseGraph &graph,
                      const std::vector<std::string> &stored_status,
                      const reusex::geometry::PlaneGraphResult &result);

/// Split the node ids @p nodes (node order) into runs of one storey: each
/// camera position (@p positions, world) goes to the storey whose
/// [z_floor - 0.5, z_ceiling + 0.3] band (level frame) contains it, else the
/// nearest band. Runs shorter than @p min_run frames are merged into the
/// previous run. Without storeys: one run. Returns [first, last, storey].
struct StoreyRun {
  std::int64_t first = 0, last = 0;
  int storey = -1;
  std::size_t frames = 0;
};
std::vector<StoreyRun>
storey_runs(const std::vector<std::int64_t> &nodes,
            const std::vector<Eigen::Vector3d> &positions,
            const std::vector<reusex::core::StoreyRecord> &storeys,
            std::size_t min_run = 3);

} // namespace rux
