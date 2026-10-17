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
//
// The reference (reusex-rcv.1.21 / .24, calibration in
// docs/research/visual-drift-loop.md §9): the no-edit FULL solve -- what
// plain `rux optimize` gives -- is solved once at load. Every block compares
// the candidate with it, the shear indicator and the shear guards measure
// against it, and `--fast` cycles solve on top of it (fast_graph_at()), so
// an edit corrects the plain-optimize trajectory instead of the drifted
// odometry.

#include "global-params.hpp"
#include "optimize.hpp"

#include <reusex/core/pose_edits.hpp>
#include <reusex/core/solid_model.hpp>
#include <reusex/slam/PlaneGraphOptimizer.hpp>
#include <reusex/slam/drift_diagnostics.hpp>
#include <reusex/visualize/drift_layers.hpp>

#include <Eigen/Core>

#include <cstdint>
#include <filesystem>
#include <map>
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

/// reusex-sb2 / reusex-rcv.1.22: how the nodes NO edit names moved against
/// the reference (the no-edit FULL solve, i.e. plain `rux optimize`).
///
/// Movement alone is not shear. Drift accumulates from the gauge outward, so
/// a correct move of segment k carries the tail after it rigidly
/// (downstream) and spreads its correction over the links between the gauge
/// and k (upstream); both are expected. Shear -- the regression-221 failure
/// -- is an untouched part moved to where it fits the other passes WORSE
/// than in the reference. So a segment no edit names counts as sheared only
/// when it moved more than the flag AND its cross-segment residual got worse
/// than the reference's by more than max(1 mm, 10%). Without residuals (no
/// drift report) the upstream movement is flagged, conservatively.
struct ShearIndicator {
  std::size_t untouched = 0; ///< nodes outside every edit's node set
  PoseChange change;         ///< of those, vs the reference
  /// Untouched nodes that moved more than the flag, by side of the last
  /// edited node: before it (towards the gauge) / after it (carried tail).
  std::size_t upstream_moved = 0, downstream_moved = 0;
  std::size_t flagged = 0; ///< moved nodes of the sheared segments
  /// Per drift segment with no edited node.
  struct Segment {
    int index = 0;
    std::int64_t first_node = 0, last_node = 0;
    double max_m = 0.0; ///< its largest move vs the reference
    /// Median cross-segment residual of its pairs, candidate / reference.
    std::optional<double> residual_m, reference_residual_m;
    bool downstream = false; ///< after the last edited node
    bool sheared = false;    ///< moved > flag and fits worse
  };
  std::vector<Segment> segments;
};

/// The input of shear_indicator(): one drift segment and its median
/// cross-segment residual on the candidate and on the reference.
struct ShearSegmentFit {
  int index = 0;
  std::int64_t first_node = 0, last_node = 0;
  std::optional<double> residual_m, reference_residual_m;
};

/// The shear indicator of a candidate @p poses against the @p reference
/// poses (both index-aligned with @p ids); @p untouched marks the nodes no
/// edit names. See ShearIndicator for the rule.
ShearIndicator shear_indicator(const std::vector<std::int64_t> &ids,
                               const std::vector<Eigen::Matrix4d> &poses,
                               const std::vector<Eigen::Matrix4d> &reference,
                               const std::vector<bool> &untouched,
                               const std::vector<ShearSegmentFit> &segments,
                               double flag_m);

/// Median residual of the cross-segment pairs (a != b) that involve each
/// segment of @p report, by segment index.
std::map<int, double>
segment_cross_residuals(const reusex::slam::DriftReport &report);

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
  /// reusex-rcv.1.24: the same numbers for the reference (the no-edit full
  /// solve, what plain `rux optimize` gives), measured once at load.
  std::optional<double> reference_cross_segment_median_m;
  std::optional<double> reference_pass_to_pass_m;
  /// The candidate against the reference: whether its cross-segment median
  /// is lower (nullopt without both numbers).
  std::optional<bool> beats_reference;
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

/// reusex-9ge: how much of a move's rigid delta a solve realised.
///
/// A move's target is D * initial for every node of its block, so the
/// block's RMS residual mixes two things: how far the block as a whole
/// fell short of D (the rigid part) and how the block bent away from its
/// initial shape (the rest). Under `--fast` the graph measures plain
/// optimize's relative poses, not the initial ones, so a block bends 0.5-3
/// cm / 0.2-1.5 deg however well it is moved -- no rigid move of the
/// initial poses can remove that. Measured on the PR #74 calibration
/// (41069051-full-d1.0 / -d0.25), every suggested move realised 95-100% of
/// its delta while its RMS read 2-8 sigma.
struct MoveRealisation {
  int nodes = 0;
  /// |D| at the block's centroid (its initial positions) and D's angle.
  double move_trans_m = 0.0;
  double move_rot_deg = 0.0;
  /// The target vs the rigid fit A of the block (initial -> solved): |D c
  /// - A c| at the centroid c, and the angle of R_D^T R_A.
  double remaining_trans_m = 0.0;
  double remaining_rot_deg = 0.0;
  /// 1 - remaining / move: 1 all of it, 0 none, < 0 pushed the other way.
  double share_trans = 1.0;
  double share_rot = 1.0;
  /// RMS of solved vs A * initial over the block: its bend, not judged.
  double bend_trans_m = 0.0;
  double bend_rot_deg = 0.0;
};
/// @p initial / @p solved: the block's poses (the same nodes, in order);
/// @p delta: the move's delta in the poses' frame (about the world origin).
MoveRealisation move_realisation(const std::vector<Eigen::Matrix4d> &initial,
                                 const std::vector<Eigen::Matrix4d> &solved,
                                 const Eigen::Matrix4d &delta);
/// "realised 99% / 98% of 0.306 m / 17.51 deg, 2.2 mm / 0.32 deg from
/// target, block bent 15.1 mm / 0.45 deg": the edit line's summary.
std::string describe_realisation(const MoveRealisation &m);

/// reusex-rcv.1.28: whether a solve honoured one pose edit.
///
/// GNC's verdict alone is not enough: its threshold is per factor, and a
/// move of n nodes lowers to n priors at sigma * sqrt(n) each, so a block
/// can end 12 cm / 6.6 deg off its target with every factor still a GNC
/// inlier. An anchor, move or relative edit is honoured only when GNC kept
/// it AND its RMS residual over its factors is within kHonouredSigmas of the
/// edit's own sigma, in translation and in rotation. same_surface /
/// on_plan_wall: GNC only.
///
/// reusex-9ge: an all-DoF move with a @p realisation is judged by its rigid
/// part instead: honoured when, in translation and in rotation, it realised
/// at least kHonouredMoveShare of its delta OR ended within kHonouredSigmas
/// of its target (a small move). Its bend is reported, not judged. NOT
/// HONOURED then means the graph fought the move off.
inline constexpr double kHonouredSigmas = 2.0;
inline constexpr double kHonouredMoveShare = 0.8;
struct EditVerdict {
  bool honoured = true;
  std::string why; ///< empty when honoured
};
/// @p edit: the edit the residual's tag names (nullptr: GNC only).
EditVerdict judge_edit(const reusex::geometry::EditResidual &residual,
                       const reusex::core::PoseEdit *edit,
                       const MoveRealisation *realisation = nullptr);

/// reusex-rcv.1.28: the gain a suggested move predicted for its segment
/// (segment ICP into the rest), read back from the note the loop wrote on
/// it ("cycle N: seg S (a-b) into the rest X -> Y mm ..."). The note travels
/// with the edits file, so the rule below holds across processes: the
/// calibration harness and an agent run one `--watch-cycles 0` process per
/// cycle, with no session memory between them.
struct MovePrediction {
  int segment = -1;
  double before_m = 0.0, after_m = 0.0;
};
/// nullopt: not a move, or its note carries no prediction (hand-written).
std::optional<MovePrediction>
move_prediction(const reusex::core::PoseEdit &edit);

/// A move counts as realised once its segment's residual fell by at least
/// this share of the predicted gain.
inline constexpr double kRealisedShare = 0.5;

/// Whether the move @p e realised its predicted gain: its segment's
/// residual now (@p segment_now, by segment index) is at most
/// before - kRealisedShare * (before - after). nullopt when there is no
/// prediction or no residual for the segment.
std::optional<bool> move_realised(const reusex::core::PoseEdit &e,
                                  const std::map<int, double> &segment_now);

/// reusex-rcv.1.28: why a suggestion that would replace the move
/// @p replaced must not be offered, or nullopt when it may be. It is held
/// back while @p replaced is a suggested move whose predicted gain did not
/// materialise (move_realised() == false): replacing it with another move
/// of the same segment, predicting another gain, is the REPLACES ping-pong
/// (41069051-full-d0.25: m-2565-2884 -> -2 -> m-2565-2884, each predicting
/// 8-9 -> 5 mm, none realised). The way on is to drop the move, not to
/// replace it.
std::optional<std::string>
blocked_replacement(const reusex::core::PoseEdit &replaced,
                    const std::map<int, double> &segment_now);

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

/// reusex-rcv.1.21: the graph the `--fast` cycles solve. Seeded at
/// @p reference (one pose per graph frame, `transform` convention: the
/// no-edit full solve), every edge that enters a solve measuring the
/// reference's relative pose, surfels dropped (no plane factors). Its
/// no-edit solve is the reference itself; an edit bends the reference's
/// shape instead of the drifted odometry's, so the shear guards and the
/// final full solve no longer pull the trajectory back to the drift.
reusex::geometry::StoredPoseGraph
fast_graph_at(const reusex::geometry::StoredPoseGraph &graph,
              const std::vector<Eigen::Matrix4d> &reference);

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
