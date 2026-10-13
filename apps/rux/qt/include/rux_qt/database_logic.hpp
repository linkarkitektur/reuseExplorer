// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// The Qt-free half of the Database workspace (Stream Q, Q2): the A/B frame
// pair and its keyboard, the pending pose-graph edits that "Gem ændringer"
// writes in one transaction, the table viewer's paging, and the small
// formatters (sizes, blob kinds, poses, edge types, write errors) the widgets
// show. Standard library only, unit-tested in the light binary.

#include <array>
#include <cstdint>
#include <limits>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace rux::qt {

// ----------------------------------------------------------- frame pair --

/// The two frames being compared, as indices into the sorted frame ids. A is
/// the frame being browsed; B is the one it is compared with. Both are always
/// valid when there are frames; with one frame, A and B are the same.
class FramePair {
    public:
  /// Sorted, de-duplicated frame ids. A goes to the first frame and B to the
  /// second (or the first, with only one).
  void reset(std::vector<int> ids);

  bool empty() const { return ids_.empty(); }
  int size() const { return static_cast<int>(ids_.size()); }
  const std::vector<int> &ids() const { return ids_; }

  int a_index() const { return a_; }
  int b_index() const { return b_; }
  /// -1 when empty.
  int a_id() const;
  int b_id() const;

  /// Move by @p delta frames, clamped to the ends. True if it moved.
  bool step_a(int delta);
  bool step_b(int delta);
  bool set_a_index(int index);
  bool set_b_index(int index);
  /// Select by frame id; false (and no change) when the id is not a frame.
  bool set_a_id(int id);
  bool set_b_id(int id);

  /// Index of frame @p id, or -1.
  int index_of(int id) const;
  /// The index whose id is nearest @p id (ties go to the lower id); -1 when
  /// empty. For jumping to "the first frame of scan N" or a typed id.
  int nearest_index(int id) const;

    private:
  static bool set(int &slot, int index, int size);
  std::vector<int> ids_;
  int a_ = -1;
  int b_ = -1;
};

/// What a key press in the frame browser does.
enum class BrowserKey { none, a_prev, a_next, b_prev, b_next };
enum class ArrowKey { left, right, other };

/// ←/→ move A, Shift+←/→ move B. Nothing while a text field or spin box has
/// the focus — its own cursor keys win.
BrowserKey browser_key(ArrowKey key, bool shift, bool text_focus);

/// How far a browser key moves: 1, or 10 with Ctrl (Page-like jumps).
int browser_step(bool ctrl);

// ------------------------------------------------------- pending edits --

/// One pose-graph edge as the editor sees it.
struct EdgeKey {
  int from = 0;
  int to = 0;
  std::string type; ///< "odometry" | "loop_closure" | "panorama"
  bool operator==(const EdgeKey &o) const {
    return from == o.from && to == o.to && type == o.type;
  }
};

struct EdgeRecord {
  EdgeKey key;
  /// 0.5 x whitened squared residual after the last solve; NaN = never
  /// solved (a staged measurement has none until optimize runs).
  double residual = std::numeric_limits<double>::quiet_NaN();
  /// 1/σ_trans²; NaN when unknown (a manual edge without ICP).
  double weight = std::numeric_limits<double>::quiet_NaN();
  /// T_from^-1 · T_to in the sensor_frames.transform convention, row-major.
  std::optional<std::array<double, 16>> transform;
  /// 6x6 information, GTSAM order (rx, ry, rz, tx, ty, tz); nullopt = the
  /// optimizer's per-type default.
  std::optional<std::array<double, 36>> information;
  std::string source; ///< rtabmap | odometry_init | orb | … ; "" unknown
  std::string status = "auto"; ///< auto | confirmed | rejected
  std::optional<double> seed_disagreement_m, seed_disagreement_deg;
  std::optional<double> deviation_m, deviation_deg;
};

/// Field-wise equality where two NaN weights/residuals are equal.
bool same_record(const EdgeRecord &a, const EdgeRecord &b);

/// One edge with its pending state, as the canvas, the list and the pair
/// strip show it.
struct EdgeView {
  enum class Pending {
    none,     ///< as stored
    added,    ///< a new key, staged
    replaced, ///< a new measurement for a stored key, staged
    status,   ///< a staged status change of a stored key
  };
  EdgeRecord edge; ///< effective: pending applied
  Pending pending = Pending::none;
  std::string stored_status; ///< status in the file; empty for a new key
  bool reversed = false;     ///< seen from (a, b), it is stored b -> a
};

/// Pose-graph edits staged in memory until "Gem ændringer". The base is the
/// file; at most one pending op per key. A user "delete" is a rejection
/// (spec D4: a tombstone that discover and optimize respect).
///
/// Keys match in either direction ((min(from, to), max(from, to), type), the
/// store's canonical key): staging 5 -> 1 over a stored 1 -> 5 acts on that
/// edge, and the store's upsert inverts the measurement on save.
class PendingEdgeEdits {
    public:
  enum class Result {
    added,      ///< a new edge is staged
    replaced,   ///< a new measurement for an existing / staged key
    revived,    ///< a new measurement over a rejected edge (overwrites it)
    confirmed,  ///< a status change to confirmed is staged (also: an add
                ///< without a transform over a stored edge)
    rejected,   ///< a status change to rejected is staged
    unrejected, ///< a rejected edge goes back to auto
    unstaged,   ///< it cancelled a pending op: the file state shows again
    unchanged,  ///< already in that state
    not_found,  ///< no such edge
    invalid,    ///< from == to, an unknown type, or a weight <= 0
  };

  struct Op {
    enum class Kind { upsert, set_status };
    Kind kind = Kind::upsert;
    EdgeRecord edge; ///< upsert: the record; set_status: key + status
    bool overwrite_rejected = false;
  };

  void set_base(std::vector<EdgeRecord> edges); ///< clears pending
  const std::vector<EdgeRecord> &base() const { return base_; }

  /// Stage a measurement (always status "confirmed"; residual and metrics
  /// cleared — optimize writes them). Over a stored key WITHOUT a transform
  /// the store keeps the stored measurement and only confirms it: that is
  /// Result::confirmed (or unchanged when it is confirmed already).
  Result add(const EdgeRecord &edge);
  Result confirm(const EdgeKey &key);
  Result reject(const EdgeKey &key);
  Result unreject(const EdgeKey &key);
  /// Drop whatever is pending on @p key.
  Result revert(const EdgeKey &key);
  void discard() { pending_.clear(); }

  bool empty() const { return pending_.empty(); }
  int count() const { return static_cast<int>(pending_.size()); }
  /// Status changes first, then measurements, each in staging order.
  std::vector<Op> ops() const;

  /// Every edge, base order then new keys in staging order.
  std::vector<EdgeView> effective() const;
  /// effective() between @p a and @p b, either direction.
  std::vector<EdgeView> between(int a, int b) const;
  /// Non-rejected effective edges at @p frame (a key counted once).
  int degree(int frame) const;
  std::map<int, int> degrees() const;

  /// After a successful save of @p saved (a snapshot of ops()): fold it into
  /// the base; ops staged or changed during the save stay pending.
  void commit_saved(const std::vector<Op> &saved);

    private:
  const EdgeRecord *stored(const EdgeKey &k) const;
  int pending_index(const EdgeKey &k) const;
  Result stage_status(const EdgeKey &k, const std::string &status,
                      Result on_change);
  std::vector<EdgeRecord> base_;
  std::vector<Op> pending_;
};

/// The banner's line: "2 nye eller målte kanter, 1 afvist, 1 bekræftet".
std::string pending_summary_da(const std::vector<PendingEdgeEdits::Op> &ops);

/// True for the three stored edge types.
bool is_edge_type(std::string_view type);
/// Danish name of an edge type ("Odometri", "Løkkelukning", "Panorama").
std::string edge_type_da(std::string_view type);

/// A weight for a manual edge from an ICP fit: the information weight the
/// store uses (1 / sigma^2) with sigma the RMS fitness, floored at 1 cm so a
/// perfect synthetic fit cannot claim infinite certainty.
double weight_from_icp_fitness(double rms_m);

// --------------------------------------------------------------- paging --

/// Lazy paging of a table model (QAbstractTableModel::fetchMore).
struct Paging {
  std::int64_t total = 0;  ///< rows in the table
  std::int64_t loaded = 0; ///< rows fetched so far
  std::int64_t page = 200; ///< rows per fetch

  bool can_fetch_more() const { return loaded < total; }
  /// Rows the next fetch asks for (0 when done).
  std::int64_t next_count() const;
  /// Rows to fetch so that row @p row is loaded (0 if it already is).
  std::int64_t count_to_reach(std::int64_t row) const;
};

// ----------------------------------------------------------- formatting --

/// A byte size the Danish way: "834 B", "33,4 kB", "1,2 MB", "3,40 GB".
std::string format_bytes_da(std::uint64_t bytes);

/// What a blob is, from its first bytes: "PNG", "JPEG", "PLY", "PDF", "WebP",
/// "GZIP", "ZIP", "JSON"… or empty when unknown.
std::string sniff_blob(std::string_view head);

/// What a known blob column holds, in Danish ("Farvebillede", "Pose 4×4"),
/// or empty for an unknown column.
std::string blob_column_meaning(std::string_view table,
                                std::string_view column);

/// The text a table cell shows for a blob: meaning, format and size, e.g.
/// "Farvebillede · JPEG · 172,7 kB". A 128-byte `transform` is "Pose 4×4".
std::string describe_blob(std::string_view table, std::string_view column,
                          std::string_view head, std::uint64_t size);

/// Translation and rotation of a row-major 4x4 pose.
struct PoseSummary {
  std::array<double, 3> t{}; ///< translation (m)
  double yaw_deg = 0;        ///< about z
  double pitch_deg = 0;      ///< about y
  double roll_deg = 0;       ///< about x
  double angle_deg = 0;      ///< total rotation angle
};
PoseSummary summarize_pose(const std::array<double, 16> &m);

/// How far apart two poses are: translation distance (m) and the rotation
/// angle between them (degrees).
struct PoseDelta {
  double distance_m = 0;
  double angle_deg = 0;
};
PoseDelta pose_delta(const std::array<double, 16> &a,
                     const std::array<double, 16> &b);

/// A decimal number with a Danish comma and @p decimals digits.
std::string format_decimal_da(double v, int decimals);

/// A table row's title from its primary-key column(s): "node_id 1177" for a
/// single-column key, or every key column joined with " · " for a composite
/// one (cloud_chunks' cloud_id+chunk_index, instance_materials'
/// material_guid+key, …) — using only the first column would show the same
/// title for every chunk of the same cloud. @p key_columns is
/// {column name, displayed value} for each primary-key column, in column
/// order; empty when the table has none (the caller falls back to "Row N").
std::string row_key_title(
    const std::vector<std::pair<std::string, std::string>> &key_columns);

// --------------------------------------------------------- write errors --

enum class WriteErrorKind { read_only, locked, other };

/// Map a ProjectDB / sqlite exception from a save to a kind.
WriteErrorKind classify_write_error(std::string_view what);
/// The Danish message shown in the pending-edits banner for @p kind.
std::string write_error_da(WriteErrorKind kind);

} // namespace rux::qt
