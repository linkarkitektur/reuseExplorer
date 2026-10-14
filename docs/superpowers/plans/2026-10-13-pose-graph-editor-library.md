<!--
SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen

SPDX-License-Identifier: GPL-3.0-or-later
-->

# Pose-Graph Editor — Library, CLI and Pipeline (P1–P3) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the stored `pose_graph_edges` table the source of truth for pose optimisation: schema v27 with measurements, information, provenance and status; `rux graph init|discover|prune`; a repeatable `rux optimize` that solves the stored graph; RTABMap links imported as edges; and a `graph_discover` pipeline stage.

**Architecture:** `core` owns the schema, the `ProjectDB` edge API and a header-only SE(3)/vocabulary helper (`core/pose_graph.hpp`) that `io` and `slam` share. `slam` owns every algorithm: graph init, the stored-graph optimize glue, discovery (the existing ORB / panorama / file front-ends, moved out of optimize) with PCM over stored ∪ new edges, and pruning. The GTSAM optimizer gains a general edge input (`optimize_graph`) and keeps `optimize()` as a thin legacy wrapper, so the old behaviour is reproducible by construction. The CLI and the header-only pipeline executor stay thin.

**Tech Stack:** C++20, SQLite3, Eigen, GTSAM (inside `slam/*.cpp` only), PCL ICP, RTABMap (inside `io/rtabmap.cpp` only), CLI11, nlohmann_json, Catch2 v3, Qt6 (only the small `graph_discover` wiring in Task 17).

**Spec:** [`docs/superpowers/specs/2026-10-13-pose-graph-editor-design.md`](../specs/2026-10-13-pose-graph-editor-design.md) — phases P1, P2, P3. P4–P6 (Qt Rediger workspace, pair panel, panorama preview) are planned separately and consume the section "Interfaces for the Qt plan" below.

## Global Constraints

- `LATEST_SCHEMA_VERSION` 26 → 27 (`libs/reusex/src/core/ProjectDB.cpp:410`).
- `pose_graph_edges.transform`: BLOB, 16 × f64, row-major; measurement `T_ij = T_from⁻¹ · T_to` in the **`sensor_frames.transform` convention** (the stored world pose, not the optical frame).
- `pose_graph_edges.information`: BLOB, 36 × f64, row-major, 6×6 information in **GTSAM Pose3 tangent order (rx, ry, rz, tx, ty, tz)**. NULL → per-type default σ from `PlaneGraphOptions`.
- `source` ∈ `rtabmap | odometry_init | orb | panorama | file | icp | manual | legacy`.
- `status` ∈ `auto | confirmed | rejected`, default `'auto'`.
- `created_at`: ISO-8601 UTC (`strftime('%Y-%m-%dT%H:%M:%SZ','now')`, the format `survey_dismissed_instances` already uses).
- Metric columns: `seed_disagreement_m`, `seed_disagreement_deg`, `deviation_m`, `deviation_deg` (REAL, nullable).
- `edge_type` ∈ `odometry | loop_closure | panorama`; `residual` = 0.5 × whitened χ² after the solve (written by `optimize`); `weight` = 1/σ_trans² (derived from `information` when present).
- `UNIQUE(from_node_id, to_node_id, edge_type)`; the migration de-duplicates first and keeps the lowest `id`. Migrated v15 rows: `source='legacy'`, `status='auto'`, `transform=NULL`.
- `sensor_frames.initial_transform`: nullable BLOB, same layout as `transform`.
- `CREATE TABLE project_settings (key TEXT PRIMARY KEY, value TEXT NOT NULL)`; first key `view.yaw_deg`.
- `add_pose_graph_edge` / `delete_pose_graph_edges` stay as `[[deprecated]]` wrappers (`delete` → reject) — `ruxd` builds against the installed package.
- Moved CLI flags (`--loop-closure`, `--loop-*` except `--loop-trust*`, `--use-panoramas`, `--pano-*`, `--loop-edges*`) fail on `rux optimize` with a "moved to `rux graph discover`" message.
- STANDARDS §1: `core` must not include `slam`/`geometry` headers or link GTSAM; `reusex_pipeline` must not link `reusex_slam` (stage code that needs slam lives in the header-only `pipeline/optimize_stage.hpp`); `io` must not link `slam`.
- STANDARDS §2: no GTSAM include in any public header.
- STANDARDS §4: every CLI default mirrors the library options struct (enforced by `operator==` tests).
- STANDARDS §5: every skipped/degenerate input is logged at `warn` **with the count and the remedy**.
- STANDARDS §6: deterministic — no unordered iteration into results; RANSAC seeds stay fixed.
- Tests: Catch2 v3; temp files only via `tests/support/temp_path.hpp` (`reusex::test_support::TempPath`).
- New files carry SPDX headers (`// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen` / `// SPDX-License-Identifier: GPL-3.0-or-later`); `#pragma once`; snake_case functions, PascalCase types, trailing-underscore members.
- Commits end with:
  ```
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_018dLKtE8qid8y5QVF9b2yri
  ```
- Build/test from the worktree root `/home/mephisto/repos/ReUseX__worktrees/gui-graph-editor` (its `build/` is configured: Ninja, Release, `WITH_CUDA=ON`, `BUILD_QT_CLIENT=ON`). **Never build or touch the main checkout's `build/`** — it is a shared benchmark resource.

## Review Focus

1. **A v26 project opened read-only** (the Qt client and `rux info` open read-only and never migrate) — `list_pose_graph_edges()`, `pose_graph_edge()`, `initial_pose()`, `has_initial_pose()` and `setting()` must degrade to legacy defaults / `nullopt`, never throw "no such column". Test: Task 2 Step 1 (`MigrationV27_ReadOnlyV26_DegradesToLegacyRows`), Task 4 Step 1.
2. **An edge supplied with `from > to`** (RTABMap stores loop links newest→oldest; a GUI user may pick B before A) — must be stored canonically (`from < to`) with the inverted measurement and the adjoint-mapped information, and must collide with the same pair given the other way round instead of creating a second factor. Test: Task 3 Step 1 (`PoseGraphUpsert_ReversedDirection_StoresCanonicalInverse`).
3. **A stored graph that is disconnected for the solver** (a frame yields no surfels, a user rejected an odometry edge, two scans in one project) — the solve must stay well-posed: bridge consecutive frames in different components with an odometry edge from the initial poses (not persisted), and `warn` with the count. Test: Task 10 Step 1 (`OptimizeStoredGraph_RejectedOdometry_IsBridgedAndWarned`).
4. **A project carrying legacy v15 rows from earlier `rux optimize` runs** (including `odometry` rows with NULL transform on consecutive keys) — `optimize` must still auto-initialise (only edges *with* a transform count as usable odometry), and `graph init` must overwrite those legacy rows rather than treat them as existing. Test: Task 7 Step 1 (`InitPoseGraph_LegacyNullTransformOdometry_IsReplaced`), Task 10 Step 1.
5. **A foreign pose writer between init and optimize** (`rux register`, then `rux optimize`) — optimize seeds from `initial_transform`, so the register result is silently discarded; the user expects chaining. `optimize` must `warn` naming `rux graph init --force` when `pipeline_log` shows a `pose_refinement_jpr` run newer than the newest `import`/`graph_init`/`pose_optimization_plane_graph` run and it did not just initialise the graph. Test: Task 10 Step 1 (`OptimizeStoredGraph_RegisterAfterInit_WarnsToReinit`).

---

## Interfaces for the Qt plan

Everything below is final after Task 17. Header paths are as installed (`<reusex/...>`).

### `core/pose_graph.hpp` (header-only, namespace `reusex::core::pose_graph`)

```cpp
inline constexpr std::string_view kTypeOdometry = "odometry";
inline constexpr std::string_view kTypeLoopClosure = "loop_closure";
inline constexpr std::string_view kTypePanorama = "panorama";
inline constexpr std::string_view kStatusAuto = "auto";
inline constexpr std::string_view kStatusConfirmed = "confirmed";
inline constexpr std::string_view kStatusRejected = "rejected";
inline constexpr std::string_view kSourceRtabmap = "rtabmap";
inline constexpr std::string_view kSourceOdometryInit = "odometry_init";
inline constexpr std::string_view kSourceOrb = "orb";
inline constexpr std::string_view kSourcePanorama = "panorama";
inline constexpr std::string_view kSourceFile = "file";
inline constexpr std::string_view kSourceIcp = "icp";
inline constexpr std::string_view kSourceManual = "manual";
inline constexpr std::string_view kSourceLegacy = "legacy";
inline constexpr std::string_view kSettingViewYawDeg = "view.yaw_deg";

using Matrix6d = Eigen::Matrix<double, 6, 6>;
bool is_valid_status(std::string_view status);
Eigen::Matrix4d matrix_from_row_major(const std::array<double, 16> &a);
std::array<double, 16> row_major_from_matrix(const Eigen::Matrix4d &m);
Matrix6d information_from_row_major(const std::array<double, 36> &a);
std::array<double, 36> row_major_from_information(const Matrix6d &m);
bool is_rigid_transform(const std::array<double, 16> &a);
bool is_valid_information(const std::array<double, 36> &a);
Matrix6d pose3_adjoint(const Eigen::Matrix4d &T);                 // GTSAM order
Matrix6d reverse_edge_information(const Matrix6d &info_ij, const Eigen::Matrix4d &T_ij);
Matrix6d information_from_sigmas(double sigma_rot, double sigma_trans);
struct TransformError { double translation_m = 0.0; double rotation_deg = 0.0; };
/// E = T_ij⁻¹ · (P_i⁻¹ · P_j): |t(E)| and angle(R(E)).
TransformError transform_error(const Eigen::Matrix4d &T_ij,
                               const Eigen::Matrix4d &P_i,
                               const Eigen::Matrix4d &P_j);
```

### `core/ProjectDB.hpp` (nested in `reusex::ProjectDB`)

```cpp
struct PoseGraphEdge {
  int from_node_id = 0;                 // always < to_node_id when read back
  int to_node_id = 0;
  std::string edge_type;                // "odometry" | "loop_closure" | "panorama" (others possible from ruxd)
  double residual = std::numeric_limits<double>::quiet_NaN(); // NULL <-> NaN
  double weight = std::numeric_limits<double>::quiet_NaN();   // NULL <-> NaN; derived from information on write
  std::optional<std::array<double, 16>> transform;   // T_from^-1 * T_to, sensor_frames.transform convention
  std::optional<std::array<double, 36>> information; // GTSAM order (rx,ry,rz,tx,ty,tz)
  std::string source = "manual";
  std::string status = "auto";
  std::string created_at;                // "" on write -> now (ISO-8601 UTC)
  std::optional<double> seed_disagreement_m;
  std::optional<double> seed_disagreement_deg;
  std::optional<double> deviation_m;
  std::optional<double> deviation_deg;
  int64_t id = 0;                        // row id; read-only (ignored on write)
};

struct PoseGraphEdgeMetrics {
  int from_node_id = 0;                  // either direction; canonicalised
  int to_node_id = 0;
  std::string edge_type;
  std::optional<double> residual;        // nullopt -> NULL
  std::optional<double> deviation_m;
  std::optional<double> deviation_deg;
  std::optional<double> seed_disagreement_m;
  std::optional<double> seed_disagreement_deg;
};

std::vector<PoseGraphEdge> list_pose_graph_edges() const;   // ORDER BY id; v26 read-only -> legacy rows
std::optional<PoseGraphEdge> pose_graph_edge(int from, int to, std::string_view edge_type) const; // either direction
bool has_pose_graph() const;
/// Insert, or replace the row with the same canonical key. from > to is
/// canonicalised (transform inverted, information mapped). Returns false and
/// writes nothing when the existing row is "rejected" and !overwrite_rejected.
/// Throws std::invalid_argument on from == to, an invalid status, an empty
/// source/edge_type, a non-rigid transform or a non-finite/asymmetric
/// information; std::runtime_error on a read-only project.
bool upsert_pose_graph_edge(const PoseGraphEdge &edge, bool overwrite_rejected = false);
/// false when no such row; throws std::invalid_argument on an invalid status.
bool set_pose_graph_edge_status(int from, int to, std::string_view edge_type, std::string_view status);
/// One SAVEPOINT; returns rows updated. Nests inside ProjectDB::Transaction.
int update_pose_graph_edge_metrics(std::span<const PoseGraphEdgeMetrics> metrics);
/// Deletes non-rejected rows with this source (tombstones stay); returns count.
int delete_pose_graph_edges_from_source(std::string_view source);
[[deprecated("use upsert_pose_graph_edge")]] int add_pose_graph_edge(const PoseGraphEdge &edge);
[[deprecated("use set_pose_graph_edge_status(..., \"rejected\")")]]
int delete_pose_graph_edges(int from, int to, std::string_view edge_type = "");

void set_initial_pose(int node_id, const std::array<double, 16> &pose); // throws if no frame / not rigid
std::optional<std::array<double, 16>> initial_pose(int node_id) const;  // nullopt: NULL, missing column, not rigid
bool has_initial_pose(int node_id) const;

std::optional<std::string> setting(std::string_view key) const;         // nullopt on pre-v27 / missing key
void set_setting(std::string_view key, std::string_view value);
```

Notes for the Qt client:
- `upsert_pose_graph_edge`, `set_pose_graph_edge_status` and `update_pose_graph_edge_metrics` may be called inside a `ProjectDB::Transaction` (they use single statements or a SAVEPOINT). `slam::init_pose_graph`, `slam::discover_loop_edges` and `slam::prune_pose_graph` open their own transaction and must **not** be called inside one.
- Rejected edges stay in `list_pose_graph_edges()`; filter on `status`.

### `slam/frame_pair_icp.hpp` (namespace `reusex::slam`, Task 6)

```cpp
struct FramePairIcpOptions {
  int pixel_step = 4; float min_depth_m = 0.3f; float max_depth_m = 4.0f;
  int max_iterations = 50; double max_correspondence_m = 0.5; float inlier_threshold_m = 0.05f;
  /// World-space correction applied to the "from" cloud before ICP starts
  /// (row-major 4x4); the result's world_delta INCLUDES it. nullopt = identity.
  std::optional<std::array<double, 16>> initial_world_delta;
};
struct FramePairIcpResult {
  std::array<double, 16> relative_pose;  // unchanged: T_to^-1 * delta * T_from
  std::array<double, 16> world_delta;    // unchanged (now includes the initial guess)
  /// pose_graph_edges.transform for the edge (from_id -> to_id):
  /// (delta * T_from)^-1 * T_to == relative_pose^-1.
  std::array<double, 16> edge_transform;
  double source_center_shift_m, rotation_deg, fitness, inlier_fraction;
  bool converged; int source_points, target_points;
};
FramePairIcpResult refine_frame_pair_icp(const ProjectDB &db, int from_id, int to_id,
                                         const FramePairIcpOptions &opts = {});
/// Diagonal GTSAM-order information for an ICP edge:
/// sigma_t = max(fitness, 0.01) m, sigma_r = max(fitness / 2.0, 0.005) rad.
std::array<double, 36> icp_edge_information(const FramePairIcpResult &result);
```

For a pair panel where B carries a manual world-space offset ΔT: call
`refine_frame_pair_icp(db, B, A, {.initial_world_delta = ΔT})` and store the edge
`(from=B, to=A, transform=edge_transform, information=icp_edge_information(r), source="icp", status="confirmed")`;
`upsert_pose_graph_edge` canonicalises the direction.

### `slam/pose_graph_conventions.hpp` (header-only, namespace `reusex::slam`, Task 5)

```cpp
Eigen::Matrix4d stored_to_optical(const Eigen::Matrix4d &T_ij, const Eigen::Matrix4d &local_i, const Eigen::Matrix4d &local_j);
Eigen::Matrix4d optical_to_stored(const Eigen::Matrix4d &T_ij_optical, const Eigen::Matrix4d &local_i, const Eigen::Matrix4d &local_j);
core::pose_graph::Matrix6d information_stored_to_optical(const core::pose_graph::Matrix6d &info, const Eigen::Matrix4d &local_j);
core::pose_graph::Matrix6d information_optical_to_stored(const core::pose_graph::Matrix6d &info, const Eigen::Matrix4d &local_j);
```

### `slam/pose_graph.hpp` (namespace `reusex::slam`, Task 7)

```cpp
class PoseGraphExistsError : public std::runtime_error { public: using std::runtime_error::runtime_error; };
struct InitPoseGraphOptions { bool force = false; };
struct InitPoseGraphResult {
  int frames = 0; int snapshots_written = 0; int odometry_edges_written = 0;
  int odometry_edges_removed = 0; int pairs_with_existing_odometry = 0; int scans = 0;
};
int count_usable_odometry_edges(const std::vector<ProjectDB::PoseGraphEdge> &edges); // odometry, not rejected, transform set
std::vector<ProjectDB::PoseGraphEdge> odometry_init_edges(const ProjectDB &db, bool from_current_transform);
InitPoseGraphResult init_pose_graph(ProjectDB &db, const InitPoseGraphOptions &options = {});
```

### `slam/graph_discover.hpp`, `slam/discover_parameters.hpp`, `slam/graph_prune.hpp` (Tasks 15–17)

```cpp
struct GraphDiscoverOptions {
  LoopCandidateOptions candidates{.loop_closure = {.enable = true}}; // ORB on by default
  bool dry_run = false;
  bool operator==(const GraphDiscoverOptions &) const = default;
};
struct GraphDiscoverResult {
  int frames = 0; int proposed = 0; int candidates = 0; int pcm_rejected = 0;
  int stored_inconsistent = 0; int skipped_existing = 0; int skipped_rejected = 0;
  int added = 0; int added_orb = 0; int added_panorama = 0; int added_file = 0;
  bool initialised_graph = false;
};
GraphDiscoverResult discover_loop_edges(ProjectDB &db, const GraphDiscoverOptions &options);

void apply_discover_parameters(GraphDiscoverOptions &options, std::string_view parameters);
geometry::LoopProposal parse_loop_proposal(std::string_view name);  // throws std::invalid_argument
std::string_view to_string(geometry::LoopProposal proposal);

struct PruneFilter {
  std::optional<double> residual_above, deviation_above_m, deviation_above_deg;
  std::optional<std::string> edge_type, source;
  std::optional<std::pair<int, int>> pair;
  bool include_confirmed = false;
  bool dry_run = false;
};
struct PruneResult { std::vector<ProjectDB::PoseGraphEdge> matched; int rejected = 0; };
PruneResult prune_pose_graph(ProjectDB &db, const PruneFilter &filter);  // std::invalid_argument when no criterion
```

### Pipeline (Task 17)

- `reusex::pipeline::JobStage::graph_discover` (appended after `annotate`): wire token `"graph_discover"`, `pipeline_log_name` `"graph_discover"`, contract `core::PipelineStage::graph_discover`, not cancellable. Executed by `pipeline::stage_executor_with_optimize()` (`<reusex/pipeline/optimize_stage.hpp>`); the default executor refuses it with `StageResult::failure`.
- JSON parameters (`pipeline::stage_parameters(JobStage::graph_discover)`):

| key | type | default | bounds |
|---|---|---|---|
| `loop_closure` | boolean | `true` | |
| `use_panoramas` | boolean | `false` | |
| `loop_edges_file` | string | `""` | |
| `loop_proposal` | string | `"auto"` (`auto\|spatial\|appearance\|exhaustive`) | |
| `loop_min_frame_gap` | integer | `50` | 1 … 100000 |
| `loop_min_inliers` | integer | `60` | 3 … 100000 |
| `pcm` | boolean | `true` | |
| `dry_run` | boolean | `false` | |

- Success outputs: `{"table", "pose_graph_edges", added}`; `run_optimize_stage` now also reports `{"table", "pose_graph_edges", <edges with metrics written>}`.
- CLI equivalent for "Kopiér som rux-kommando": `rux graph discover` (`rux_qt/cli_command` stage spec `{"graph_discover", {"graph", "discover"}, …}`; flags `--no-loop-closure`, `--use-panoramas`, `--loop-edges`, `--loop-proposal`, `--loop-min-frame-gap`, `--loop-min-inliers`, `--loop-no-pcm`, `--dry-run`).
- Settings key for the view yaw (P4): `core::pose_graph::kSettingViewYawDeg` (`"view.yaw_deg"`), value a decimal string in degrees.

---

## File Structure

| Path | Status | Responsibility |
|---|---|---|
| `libs/reusex/include/core/pose_graph.hpp` | create | Vocabulary constants + header-only SE(3)/information helpers shared by core, io, slam |
| `libs/reusex/include/core/ProjectDB.hpp` | modify | `PoseGraphEdge`, `PoseGraphEdgeMetrics`, edge/initial-pose/settings API |
| `libs/reusex/src/core/ProjectDB.cpp` | modify | `migrateToV27`, edge/initial-pose/settings implementation |
| `libs/reusex/include/slam/pose_graph_conventions.hpp` | create | stored ↔ optical conversion of edge transforms and information (header-only) |
| `libs/reusex/include/slam/frame_pair_icp.hpp`, `src/slam/frame_pair_icp.cpp` | modify | ICP initial guess, `edge_transform`, `icp_edge_information` |
| `libs/reusex/include/slam/pose_graph.hpp`, `src/slam/pose_graph.cpp` | create | `init_pose_graph`, `odometry_init_edges`, `count_usable_odometry_edges` |
| `libs/reusex/include/slam/loop_candidates.hpp`, `src/slam/loop_candidates.cpp` | create | `LoopCandidateOptions` + `collect_loop_candidates` (front-ends moved out of optimize) |
| `libs/reusex/include/slam/PlaneGraphOptimizer.hpp`, `src/slam/PlaneGraphOptimizer.cpp` | modify | `GraphEdge`, `optimize_graph`, `legacy_graph_edges`, loop default σ, edge tags |
| `libs/reusex/src/slam/optimize_sensor_poses.cpp` | modify (rewrite) | Solve the stored graph; seeds from `initial_transform`; bridging; metrics |
| `libs/reusex/src/slam/perturb_poses.cpp` | modify | Re-snapshot initial poses / re-derive `odometry_init` after drift synthesis |
| `libs/reusex/include/io/rtabmap_links.hpp`, `src/io/rtabmap_links.cpp` | create | Pure RTABMap-link → `PoseGraphEdge` conversion (no rtabmap include) |
| `libs/reusex/src/io/rtabmap.cpp` | modify | Store links, write `initial_transform` |
| `libs/reusex/include/slam/LoopClosure.hpp`, `src/slam/LoopClosure.cpp` | modify | PCM with fixed inliers (`consistent_loop_edge_indices`) |
| `libs/reusex/include/slam/graph_discover.hpp`, `src/slam/graph_discover.cpp` | create | `discover_loop_edges` |
| `libs/reusex/include/slam/graph_prune.hpp`, `src/slam/graph_prune.cpp` | create | `prune_pose_graph` |
| `libs/reusex/include/slam/discover_parameters.hpp`, `src/slam/discover_parameters.cpp` | create | JSON → `GraphDiscoverOptions` |
| `libs/reusex/include/core/stage_contract.hpp`, `src/core/stage_contract.cpp` | modify | `graph_init`, `graph_discover` contracts; `pose_graph_edges` artifact |
| `libs/reusex/include/pipeline/stages.hpp`, `src/pipeline/stages.cpp`, `src/pipeline/stage_parameters.cpp`, `include/pipeline/optimize_stage.hpp` | modify | `JobStage::graph_discover` |
| `apps/rux/include/graph.hpp`, `apps/rux/src/graph.cpp` | create | `rux graph` group |
| `apps/rux/include/graph/{init,discover,prune}.hpp`, `apps/rux/src/graph/{init,discover,prune}.cpp` | create | thin subcommands |
| `apps/rux/include/optimize.hpp`, `apps/rux/src/optimize.cpp` | modify | flag removal + moved-flag rejection, loop default σ flags |
| `apps/rux/src/rux.cpp` | modify | register `graph` |
| `apps/rux/qt/src/EdgeEditor.cpp`, `apps/rux/qt/gallery/shell_pages.cpp` | modify | migrate off removed/deprecated edge calls |
| `apps/rux/qt/src/pipeline_ui.cpp`, `apps/rux/qt/src/PipelineWorkspace.cpp`, `apps/rux/qt/src/core/cli_command.cpp` | modify | `graph_discover` stage names, labels, CLI mapping |
| `docs/CONTRACTS.md`, `docs/DIRECTION.md`, `CLAUDE.md`, `README.md` | modify | docs (STANDARDS §9.9) |
| `scripts/bench-*.sh`, `scripts/create_pseudo_gt.sh`, `scripts/spatial_loop_validate.sh`, `tools/loop_edges/README.md` | modify | `optimize --loop-*` → `graph discover` + `optimize` |

Tests (all new files under `tests/unit/…`, picked up by the glob): `core/test_pose_graph_math.cpp`, `core/test_project_db_migration_v27.cpp`, `core/test_project_db_initial_pose_settings.cpp`, `slam/test_pose_graph_conventions.cpp`, `slam/test_pose_graph_init.cpp`, `slam/test_loop_candidates.cpp`, `slam/test_optimize_stored_graph.cpp`, `slam/test_graph_discover.cpp`, `slam/test_graph_prune.cpp`, `slam/test_discover_parameters.cpp`, `io/test_rtabmap_links.cpp`, `rux_app/test_optimize_moved_flags.cpp`, `rux_app/test_graph_cli.cpp`; modified: `core/test_project_db_pose_graph.cpp`, `core/test_project_db_migration_v26.cpp`, `slam/test_frame_pair_icp.cpp`, `slam/test_plane_graph_optimizer.cpp`, `slam/test_seed_disagreement_gate.cpp`, `slam/test_loop_closure.cpp`, `slam/test_perturb_poses.cpp`, `pipeline/*`, `rux_app/test_cli_command_roundtrip.cpp`, `rux_qt/test_edge_save_locked.cpp`.

## Build and test commands (used by every task)

```bash
# build the light unit-test binary (core, slam, io, pipeline, rux_qt tests)
nix develop -c cmake --build build --target reusex_unit_tests -j$(nproc)
# run one test case or tag
nix develop -c build/tests/reusex_unit_tests "<TEST_CASE name or [tag]>"
# heavy binary (rux_app tests: CLI round-trip, moved flags, graph CLI)
nix develop -c cmake --build build --target reusex_unit_tests_vision -j$(nproc)
nix develop -c build/tests/reusex_unit_tests_vision "[rux_app]"
# rux itself
nix develop -c cmake --build build --target rux -j$(nproc)
```

If `build/tests/reusex_unit_tests_vision` does not exist after the build, run `ls build/tests` and use the binary name listed there that contains `vision` (the target is `reusex_unit_tests_vision`, `tests/CMakeLists.txt`).

---

## Task 0: Baseline

**Files:** none.

- [ ] **Step 1: Confirm a clean start and a green baseline**

```bash
cd /home/mephisto/repos/ReUseX__worktrees/gui-graph-editor
git status --short            # expect only " M shell.nix" (pre-existing, not ours: never commit it)
nix develop -c cmake --build build -j$(nproc)
nix develop -c bash -c 'cd build && ctest --output-on-failure --parallel $(nproc) -E reusex_package_consumer'
```

Expected: build succeeds; ctest all pass (record any pre-existing failure names in the task notes so later tasks are not blamed for them).

- [ ] **Step 2: Record the baseline commit for the benchmark in Task 19**

```bash
S=/tmp/claude-1001/-home-mephisto-repos-ReUseX--worktrees-gui-graph-editor/b9a0a5e9-d5a5-48c6-adfb-71b8bc180a4a/scratchpad
mkdir -p $S && git rev-parse HEAD > $S/pose-graph-baseline-sha.txt && cat $S/pose-graph-baseline-sha.txt
```

Also note the SHA in the task notes; Task 19 builds it as the benchmark baseline.

---

# Phase P1 — schema v27, `ProjectDB` API, conventions helper

## Task 1: `core/pose_graph.hpp` — vocabulary and SE(3) helpers

**Files:**
- Create: `libs/reusex/include/core/pose_graph.hpp`
- Test: `tests/unit/core/test_pose_graph_math.cpp`

**Interfaces:**
- Consumes: Eigen only.
- Produces: everything listed under `core/pose_graph.hpp` in "Interfaces for the Qt plan".

- [ ] **Step 1: Write the failing test**

Create `tests/unit/core/test_pose_graph_math.cpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// core/pose_graph.hpp: the conventions every pose_graph_edges writer shares
// (spec §3.1): row-major blobs, GTSAM (rot, trans) tangent order, and the
// information of a reversed edge.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <reusex/core/pose_graph.hpp>

#include <Eigen/Geometry>

#include <array>
#include <cmath>
#include <limits>

namespace pg = reusex::core::pose_graph;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {

Eigen::Matrix4d make_pose(double angle, const Eigen::Vector3d &axis,
                          const Eigen::Vector3d &t) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.topLeftCorner<3, 3>() =
      Eigen::AngleAxisd(angle, axis.normalized()).toRotationMatrix();
  T.topRightCorner<3, 1>() = t;
  return T;
}

/// First-order log of a near-identity transform, GTSAM order (rot, trans).
Eigen::Matrix<double, 6, 1> small_log(const Eigen::Matrix4d &M) {
  const Eigen::AngleAxisd aa(Eigen::Matrix3d(M.topLeftCorner<3, 3>()));
  Eigen::Matrix<double, 6, 1> v;
  v.head<3>() = aa.angle() * aa.axis();
  v.tail<3>() = M.topRightCorner<3, 1>();
  return v;
}

pg::Matrix6d spd_information() {
  pg::Matrix6d A;
  A << 4, 1, 0, 0, 0.5, 0, //
      1, 5, 0, 0, 0, 0,    //
      0, 0, 6, 0.2, 0, 0,  //
      0, 0, 0.2, 90, 3, 0, //
      0.5, 0, 0, 3, 80, 1, //
      0, 0, 0, 0, 1, 70;
  return 0.5 * (A + A.transpose()) + 10.0 * pg::Matrix6d::Identity();
}

} // namespace

TEST_CASE("PoseGraphMath_RowMajorBlobs_RoundTrip", "[core][pose_graph]") {
  const Eigen::Matrix4d T = make_pose(0.3, {1, 2, 3}, {4, -5, 6});
  const auto a = pg::row_major_from_matrix(T);
  CHECK(a[3] == 4.0); // row 0, col 3: translation x
  CHECK(pg::matrix_from_row_major(a) == T);

  const pg::Matrix6d I = spd_information();
  CHECK(pg::information_from_row_major(pg::row_major_from_information(I)) ==
        I);
}

TEST_CASE("PoseGraphMath_IsRigidTransform_RejectsBrokenBlobs",
          "[core][pose_graph]") {
  CHECK(pg::is_rigid_transform(
      pg::row_major_from_matrix(make_pose(1.0, {0, 0, 1}, {1, 2, 3}))));
  std::array<double, 16> zero{};
  CHECK_FALSE(pg::is_rigid_transform(zero));
  auto nan = pg::row_major_from_matrix(Eigen::Matrix4d::Identity());
  nan[7] = std::numeric_limits<double>::quiet_NaN();
  CHECK_FALSE(pg::is_rigid_transform(nan));
  auto reflection = pg::row_major_from_matrix(Eigen::Matrix4d::Identity());
  reflection[0] = -1.0; // det(R) = -1
  CHECK_FALSE(pg::is_rigid_transform(reflection));
  auto scaled = pg::row_major_from_matrix(Eigen::Matrix4d::Identity());
  scaled[0] = scaled[5] = scaled[10] = 2.0; // not orthonormal
  CHECK_FALSE(pg::is_rigid_transform(scaled));
}

TEST_CASE("PoseGraphMath_IsValidInformation_RejectsAsymmetricOrNonFinite",
          "[core][pose_graph]") {
  CHECK(pg::is_valid_information(
      pg::row_major_from_information(spd_information())));
  auto bad = pg::row_major_from_information(spd_information());
  bad[1] += 1.0; // (0,1) != (1,0)
  CHECK_FALSE(pg::is_valid_information(bad));
  bad = pg::row_major_from_information(spd_information());
  bad[0] = std::numeric_limits<double>::infinity();
  CHECK_FALSE(pg::is_valid_information(bad));
}

TEST_CASE("PoseGraphMath_ReverseEdgeInformation_PreservesChiSquare",
          "[core][pose_graph]") {
  // X_j = X_i * T_ij * D with a small D: the forward error is ~log(D); the
  // reversed edge (T_ji = T_ij^-1) sees the same disagreement from j's side.
  // The chi-square of the two must agree to first order.
  const Eigen::Matrix4d T_ij = make_pose(0.7, {0.2, 1, 0.3}, {1.5, -0.4, 2.0});
  const Eigen::Matrix4d X_i = make_pose(-0.4, {1, 0, 1}, {3, 1, -2});
  const Eigen::Matrix4d D = make_pose(2e-4, {0.3, -1, 0.5}, {1e-4, -2e-4, 3e-4});
  const Eigen::Matrix4d X_j = X_i * T_ij * D;
  const pg::Matrix6d L = spd_information();

  const auto e_ij = small_log(T_ij.inverse() * X_i.inverse() * X_j);
  const Eigen::Matrix4d T_ji = T_ij.inverse();
  const auto e_ji = small_log(T_ji.inverse() * X_j.inverse() * X_i);
  const pg::Matrix6d L_ji = pg::reverse_edge_information(L, T_ij);

  const double chi_ij = e_ij.dot(L * e_ij);
  const double chi_ji = e_ji.dot(L_ji * e_ji);
  REQUIRE(chi_ij > 0.0);
  CHECK_THAT(chi_ji, WithinRel(chi_ij, 1e-3));
}

TEST_CASE("PoseGraphMath_ReverseTwice_IsIdentity", "[core][pose_graph]") {
  const Eigen::Matrix4d T = make_pose(1.1, {1, 1, 0}, {0.5, 2, -1});
  const pg::Matrix6d L = spd_information();
  const pg::Matrix6d back =
      pg::reverse_edge_information(pg::reverse_edge_information(L, T),
                                   T.inverse());
  CHECK((back - L).norm() < 1e-9 * L.norm());
}

TEST_CASE("PoseGraphMath_InformationFromSigmas_IsDiagonalInGtsamOrder",
          "[core][pose_graph]") {
  const pg::Matrix6d L = pg::information_from_sigmas(0.1, 0.5);
  CHECK_THAT(L(0, 0), WithinRel(100.0, 1e-12)); // rotation first
  CHECK_THAT(L(5, 5), WithinRel(4.0, 1e-12));   // translation last
  CHECK(L(0, 5) == 0.0);
}

TEST_CASE("PoseGraphMath_TransformError_IsZeroForAgreeingPoses",
          "[core][pose_graph]") {
  const Eigen::Matrix4d P_i = make_pose(0.4, {0, 0, 1}, {1, 2, 0});
  const Eigen::Matrix4d T = make_pose(0.2, {0, 1, 0}, {0.5, 0, 0});
  const auto ok = pg::transform_error(T, P_i, P_i * T);
  CHECK_THAT(ok.translation_m, WithinAbs(0.0, 1e-12));
  CHECK_THAT(ok.rotation_deg, WithinAbs(0.0, 1e-6));
  const Eigen::Matrix4d off = make_pose(0.0, {0, 0, 1}, {0, 0.3, 0});
  const auto bad = pg::transform_error(T, P_i, P_i * T * off);
  CHECK_THAT(bad.translation_m, WithinAbs(0.3, 1e-12));
}

TEST_CASE("PoseGraphMath_Vocabulary_StatusValidation", "[core][pose_graph]") {
  CHECK(pg::is_valid_status("auto"));
  CHECK(pg::is_valid_status("confirmed"));
  CHECK(pg::is_valid_status("rejected"));
  CHECK_FALSE(pg::is_valid_status("deleted"));
  CHECK_FALSE(pg::is_valid_status(""));
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `nix develop -c cmake --build build --target reusex_unit_tests -j$(nproc)`
Expected: FAIL — `reusex/core/pose_graph.hpp: No such file or directory`.

- [ ] **Step 3: Write the header**

Create `libs/reusex/include/core/pose_graph.hpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Vocabulary and SE(3) helpers for the stored pose graph (`pose_graph_edges`,
// schema v27). Header-only and Eigen-only so core (ProjectDB), io (the RTABMap
// importer) and slam share ONE definition of the conventions (STANDARDS §1:
// core may not reach slam or geometry_common).
//
// Conventions (spec §3.1):
//  * transform   T_ij = T_from^-1 * T_to in the sensor_frames.transform
//                convention, 16 doubles row-major.
//  * information 6x6 in GTSAM Pose3 tangent order (rx, ry, rz, tx, ty, tz),
//                36 doubles row-major.

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <string_view>

namespace reusex::core::pose_graph {

inline constexpr std::string_view kTypeOdometry = "odometry";
inline constexpr std::string_view kTypeLoopClosure = "loop_closure";
inline constexpr std::string_view kTypePanorama = "panorama";

inline constexpr std::string_view kStatusAuto = "auto";
inline constexpr std::string_view kStatusConfirmed = "confirmed";
inline constexpr std::string_view kStatusRejected = "rejected";

inline constexpr std::string_view kSourceRtabmap = "rtabmap";
inline constexpr std::string_view kSourceOdometryInit = "odometry_init";
inline constexpr std::string_view kSourceOrb = "orb";
inline constexpr std::string_view kSourcePanorama = "panorama";
inline constexpr std::string_view kSourceFile = "file";
inline constexpr std::string_view kSourceIcp = "icp";
inline constexpr std::string_view kSourceManual = "manual";
inline constexpr std::string_view kSourceLegacy = "legacy";

/// project_settings key of the persistent display-only view yaw (spec D7).
inline constexpr std::string_view kSettingViewYawDeg = "view.yaw_deg";

using Matrix6d = Eigen::Matrix<double, 6, 6>;

inline bool is_valid_status(std::string_view status) {
  return status == kStatusAuto || status == kStatusConfirmed ||
         status == kStatusRejected;
}

inline Eigen::Matrix4d matrix_from_row_major(const std::array<double, 16> &a) {
  Eigen::Matrix4d m;
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c)
      m(r, c) = a[static_cast<size_t>(r * 4 + c)];
  return m;
}

inline std::array<double, 16> row_major_from_matrix(const Eigen::Matrix4d &m) {
  std::array<double, 16> a{};
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c)
      a[static_cast<size_t>(r * 4 + c)] = m(r, c);
  return a;
}

inline Matrix6d information_from_row_major(const std::array<double, 36> &a) {
  Matrix6d m;
  for (int r = 0; r < 6; ++r)
    for (int c = 0; c < 6; ++c)
      m(r, c) = a[static_cast<size_t>(r * 6 + c)];
  return m;
}

inline std::array<double, 36> row_major_from_information(const Matrix6d &m) {
  std::array<double, 36> a{};
  for (int r = 0; r < 6; ++r)
    for (int c = 0; c < 6; ++c)
      a[static_cast<size_t>(r * 6 + c)] = m(r, c);
  return a;
}

/// Finite, bottom row [0 0 0 1], orthonormal rotation block with det > 0.
/// Tolerances admit float-precision imports (RTABMap stores floats).
inline bool is_rigid_transform(const std::array<double, 16> &a) {
  for (double v : a)
    if (!std::isfinite(v))
      return false;
  if (std::abs(a[12]) > 1e-6 || std::abs(a[13]) > 1e-6 ||
      std::abs(a[14]) > 1e-6 || std::abs(a[15] - 1.0) > 1e-6)
    return false;
  const Eigen::Matrix3d R = matrix_from_row_major(a).topLeftCorner<3, 3>();
  return (R.transpose() * R - Eigen::Matrix3d::Identity()).norm() < 1e-3 &&
         R.determinant() > 0.0;
}

/// Finite and symmetric (relative tolerance 1e-9).
inline bool is_valid_information(const std::array<double, 36> &a) {
  for (double v : a)
    if (!std::isfinite(v))
      return false;
  const Matrix6d m = information_from_row_major(a);
  return (m - m.transpose()).norm() <= 1e-9 * std::max(1.0, m.norm());
}

inline Eigen::Matrix3d skew(const Eigen::Vector3d &v) {
  Eigen::Matrix3d s;
  s << 0, -v.z(), v.y(), //
      v.z(), 0, -v.x(),  //
      -v.y(), v.x(), 0;
  return s;
}

/// Adjoint of T in GTSAM's (rotation, translation) tangent order:
/// [[R, 0], [t^ R, R]].
inline Matrix6d pose3_adjoint(const Eigen::Matrix4d &T) {
  const Eigen::Matrix3d R = T.topLeftCorner<3, 3>();
  const Eigen::Vector3d t = T.topRightCorner<3, 1>();
  Matrix6d A = Matrix6d::Zero();
  A.topLeftCorner<3, 3>() = R;
  A.bottomLeftCorner<3, 3>() = skew(t) * R;
  A.bottomRightCorner<3, 3>() = R;
  return A;
}

/// Information of the reversed edge j -> i. The BetweenFactor error of the
/// reversed measurement is e_ji = -Ad(T_ij) e_ij, so
/// L_ji = Ad(T_ji)^T L_ij Ad(T_ji) with T_ji = T_ij^-1. Symmetrised.
inline Matrix6d reverse_edge_information(const Matrix6d &info_ij,
                                         const Eigen::Matrix4d &T_ij) {
  const Matrix6d A = pose3_adjoint(T_ij.inverse());
  const Matrix6d out = A.transpose() * info_ij * A;
  return 0.5 * (out + out.transpose());
}

/// Diagonal information diag(1/sr^2 x3, 1/st^2 x3).
inline Matrix6d information_from_sigmas(double sigma_rot, double sigma_trans) {
  Matrix6d m = Matrix6d::Zero();
  for (int k = 0; k < 3; ++k) {
    m(k, k) = 1.0 / (sigma_rot * sigma_rot);
    m(k + 3, k + 3) = 1.0 / (sigma_trans * sigma_trans);
  }
  return m;
}

struct TransformError {
  double translation_m = 0.0;
  double rotation_deg = 0.0;
};

/// How far poses P_i, P_j disagree with the measurement T_ij:
/// E = T_ij^-1 * (P_i^-1 * P_j); returns |t(E)| and the rotation angle of E.
/// Used for seed_disagreement_* (P = initial poses) and deviation_*
/// (P = solved poses).
inline TransformError transform_error(const Eigen::Matrix4d &T_ij,
                                      const Eigen::Matrix4d &P_i,
                                      const Eigen::Matrix4d &P_j) {
  const Eigen::Matrix4d E = T_ij.inverse() * (P_i.inverse() * P_j);
  const double c =
      std::clamp((E.topLeftCorner<3, 3>().trace() - 1.0) / 2.0, -1.0, 1.0);
  return {E.topRightCorner<3, 1>().norm(), std::acos(c) * 180.0 / M_PI};
}

} // namespace reusex::core::pose_graph
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `nix develop -c cmake --build build --target reusex_unit_tests -j$(nproc) && nix develop -c build/tests/reusex_unit_tests "[pose_graph]"`
Expected: PASS (all 8 test cases).

- [ ] **Step 5: Commit**

```bash
git add libs/reusex/include/core/pose_graph.hpp tests/unit/core/test_pose_graph_math.cpp
git commit -m "core: pose-graph vocabulary and SE(3) helpers (schema v27 prep)

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_018dLKtE8qid8y5QVF9b2yri"
```

---

## Task 2: Schema v27 migration and the extended `PoseGraphEdge` (read side)

**Files:**
- Modify: `libs/reusex/include/core/ProjectDB.hpp:591-639` (pose-graph block)
- Modify: `libs/reusex/src/core/ProjectDB.cpp` — `LATEST_SCHEMA_VERSION` (:410), member flags (:405-407), configure (:527-528), `runMigrations` (after :815), new `migrateToV27` (after `migrateToV26`, :2421-2436), `listPoseGraphEdges` (:1639-1665), `savePoseGraphEdges` (:1600-1637)
- Modify: `tests/unit/core/test_project_db_migration_v26.cpp:67-68,101`
- Create: `tests/unit/core/test_project_db_migration_v27.cpp`

**Interfaces:**
- Consumes: `core/pose_graph.hpp` (Task 1).
- Produces: `ProjectDB::PoseGraphEdge` (final layout, see Interfaces section), `list_pose_graph_edges()`, `pose_graph_edge(int, int, std::string_view)`, `ProjectDB::latest_schema_version() == 27`.

- [ ] **Step 1: Write the failing migration test**

Create `tests/unit/core/test_project_db_migration_v27.cpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Schema v26 -> v27 (pose-graph editor spec §3): pose_graph_edges gains the
// measurement, information, provenance, status and metric columns plus a
// UNIQUE(from, to, type) key on canonical (from < to) rows;
// sensor_frames.initial_transform and project_settings appear. A fresh project
// is rolled back to the exact v26 shape with raw SQL, seeded with the kinds of
// rows `rux optimize` and ruxd wrote under v15..v26, then reopened.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <reusex/core/ProjectDB.hpp>

#include "../../support/temp_path.hpp"

#include <sqlite3.h>

#include <cmath>
#include <filesystem>
#include <string>

using reusex::ProjectDB;
using reusex::test_support::TempPath;
using Catch::Matchers::WithinAbs;
namespace fs = std::filesystem;

namespace {

void exec_raw(const fs::path &path, const std::string &sql) {
  sqlite3 *raw = nullptr;
  REQUIRE(sqlite3_open(path.string().c_str(), &raw) == SQLITE_OK);
  char *err = nullptr;
  const int rc = sqlite3_exec(raw, sql.c_str(), nullptr, nullptr, &err);
  INFO((err ? err : ""));
  sqlite3_free(err);
  sqlite3_close(raw);
  REQUIRE(rc == SQLITE_OK);
}

bool column_exists(const fs::path &path, const char *table, const char *col) {
  sqlite3 *raw = nullptr;
  REQUIRE(sqlite3_open(path.string().c_str(), &raw) == SQLITE_OK);
  sqlite3_stmt *s = nullptr;
  const std::string sql = std::string("PRAGMA table_info(") + table + ");";
  sqlite3_prepare_v2(raw, sql.c_str(), -1, &s, nullptr);
  bool found = false;
  while (sqlite3_step(s) == SQLITE_ROW)
    if (std::string(reinterpret_cast<const char *>(sqlite3_column_text(s, 1))) ==
        col)
      found = true;
  sqlite3_finalize(s);
  sqlite3_close(raw);
  return found;
}

/// The exact v15..v26 pose_graph_edges DDL plus legacy rows: a duplicate in
/// reverse direction, a self-loop, a reversed loop closure pair and a
/// second type on the same pair.
void roll_back_to_v26_with_legacy_rows(const fs::path &path) {
  exec_raw(path, R"(
    DROP TABLE pose_graph_edges;
    CREATE TABLE pose_graph_edges (
      id           INTEGER PRIMARY KEY,
      from_node_id INTEGER NOT NULL,
      to_node_id   INTEGER NOT NULL,
      edge_type    TEXT    NOT NULL,
      residual     REAL    NOT NULL,
      weight       REAL
    );
    INSERT INTO pose_graph_edges VALUES (1, 1, 2, 'odometry',     0.1, 100.0);
    INSERT INTO pose_graph_edges VALUES (2, 2, 1, 'odometry',     0.2,  50.0);
    INSERT INTO pose_graph_edges VALUES (3, 9, 5, 'loop_closure', 0.5,  NULL);
    INSERT INTO pose_graph_edges VALUES (4, 5, 9, 'loop_closure', 0.6,  25.0);
    INSERT INTO pose_graph_edges VALUES (5, 3, 3, 'loop_closure', 0.7,  25.0);
    INSERT INTO pose_graph_edges VALUES (6, 1, 2, 'loop_closure', 0.8,  25.0);
    DROP TABLE project_settings;
    ALTER TABLE sensor_frames DROP COLUMN initial_transform;
    DELETE FROM schema_version WHERE version >= 27;
  )");
}

} // namespace

TEST_CASE("MigrationV27_FreshProject_HasNewColumnsAndTables",
          "[projectdb][migration][pose_graph]") {
  TempPath tmp("test_migration_v27_fresh");
  {
    ProjectDB db(tmp.path);
    CHECK(db.schema_version() == 27);
    CHECK(ProjectDB::latest_schema_version() == 27);
    CHECK(db.list_pose_graph_edges().empty());
  }
  for (const char *col :
       {"transform", "information", "source", "status", "created_at",
        "seed_disagreement_m", "seed_disagreement_deg", "deviation_m",
        "deviation_deg"}) {
    INFO(col);
    CHECK(column_exists(tmp.path, "pose_graph_edges", col));
  }
  CHECK(column_exists(tmp.path, "sensor_frames", "initial_transform"));
  CHECK(column_exists(tmp.path, "project_settings", "value"));
}

TEST_CASE("MigrationV27_ReadOnlyV26_DegradesToLegacyRows",
          "[projectdb][migration][pose_graph]") {
  TempPath tmp("test_migration_v27_ro");
  { ProjectDB db(tmp.path); }
  roll_back_to_v26_with_legacy_rows(tmp.path);

  ProjectDB ro(tmp.path, /*readOnly=*/true);
  REQUIRE(ro.schema_version() == 26);
  const auto rows = ro.list_pose_graph_edges(); // must not throw
  REQUIRE(rows.size() == 6);
  CHECK(rows[0].source == "legacy");
  CHECK(rows[0].status == "auto");
  CHECK_FALSE(rows[0].transform.has_value());
  CHECK(rows[0].id == 1);
  CHECK(ro.has_pose_graph());
}

TEST_CASE("MigrationV27_FromV26_DeduplicatesCanonicalisesAndKeepsLowestId",
          "[projectdb][migration][pose_graph]") {
  TempPath tmp("test_migration_v27_dedup");
  { ProjectDB db(tmp.path); }
  roll_back_to_v26_with_legacy_rows(tmp.path);

  ProjectDB db(tmp.path);
  REQUIRE(db.schema_version() == 27);
  const auto rows = db.list_pose_graph_edges();
  // (1,2,odometry) from ids 1+2 -> id 1; (5,9,loop_closure) from ids 3+4 ->
  // id 3; self-loop id 5 dropped; (1,2,loop_closure) id 6 kept.
  REQUIRE(rows.size() == 3);
  CHECK(rows[0].id == 1);
  CHECK(rows[0].from_node_id == 1);
  CHECK(rows[0].to_node_id == 2);
  CHECK(rows[0].edge_type == "odometry");
  CHECK_THAT(rows[0].residual, WithinAbs(0.1, 1e-12));
  CHECK(rows[1].id == 3);
  CHECK(rows[1].from_node_id == 5);
  CHECK(rows[1].to_node_id == 9);
  CHECK_THAT(rows[1].residual, WithinAbs(0.5, 1e-12));
  CHECK(std::isnan(rows[1].weight));
  CHECK(rows[2].id == 6);
  for (const auto &r : rows) {
    CHECK(r.source == "legacy");
    CHECK(r.status == "auto");
    CHECK_FALSE(r.transform.has_value());
    CHECK_FALSE(r.information.has_value());
    CHECK_FALSE(r.deviation_m.has_value());
  }
  CHECK(column_exists(tmp.path, "sensor_frames", "initial_transform"));
}

TEST_CASE("MigrationV27_UniqueKey_RejectsRawDuplicate",
          "[projectdb][migration][pose_graph]") {
  TempPath tmp("test_migration_v27_unique");
  { ProjectDB db(tmp.path); }
  exec_raw(tmp.path, "INSERT INTO pose_graph_edges (from_node_id, to_node_id, "
                     "edge_type, source, status) VALUES (1, 2, 'odometry', "
                     "'manual', 'auto');");
  sqlite3 *raw = nullptr;
  REQUIRE(sqlite3_open(tmp.path.string().c_str(), &raw) == SQLITE_OK);
  const int dup = sqlite3_exec(
      raw,
      "INSERT INTO pose_graph_edges (from_node_id, to_node_id, edge_type, "
      "source, status) VALUES (1, 2, 'odometry', 'manual', 'auto');",
      nullptr, nullptr, nullptr);
  const int reversed = sqlite3_exec(
      raw,
      "INSERT INTO pose_graph_edges (from_node_id, to_node_id, edge_type, "
      "source, status) VALUES (2, 1, 'loop_closure', 'manual', 'auto');",
      nullptr, nullptr, nullptr);
  sqlite3_close(raw);
  CHECK(dup == SQLITE_CONSTRAINT);
  CHECK(reversed == SQLITE_CONSTRAINT); // CHECK (from_node_id < to_node_id)
}

TEST_CASE("PoseGraphEdge_LookupEitherDirection_FindsCanonicalRow",
          "[projectdb][pose_graph]") {
  TempPath tmp("test_pose_graph_lookup");
  { ProjectDB db(tmp.path); }
  exec_raw(tmp.path, "INSERT INTO pose_graph_edges (from_node_id, to_node_id, "
                     "edge_type, source, status) VALUES (4, 8, 'loop_closure', "
                     "'orb', 'confirmed');");
  ProjectDB db(tmp.path, /*readOnly=*/true);
  const auto a = db.pose_graph_edge(4, 8, "loop_closure");
  const auto b = db.pose_graph_edge(8, 4, "loop_closure");
  REQUIRE(a.has_value());
  REQUIRE(b.has_value());
  CHECK(a->id == b->id);
  CHECK(a->status == "confirmed");
  CHECK(a->source == "orb");
  CHECK_FALSE(db.pose_graph_edge(4, 8, "odometry").has_value());
}
```

Edit `tests/unit/core/test_project_db_migration_v26.cpp`: replace line 67-68

```cpp
    CHECK(db.schema_version() == 26);
    CHECK(ProjectDB::latest_schema_version() == 26);
```

with

```cpp
    CHECK(db.schema_version() == ProjectDB::latest_schema_version());
    CHECK(ProjectDB::latest_schema_version() >= 26);
```

and line 101 `CHECK(db.schema_version() == 26);` with `CHECK(db.schema_version() == ProjectDB::latest_schema_version());`.

- [ ] **Step 2: Run test to verify it fails**

Run: `nix develop -c cmake --build build --target reusex_unit_tests -j$(nproc)`
Expected: compile FAIL — `no member named 'pose_graph_edge'` / `'id' in PoseGraphEdge`.

- [ ] **Step 3: Extend the header**

In `libs/reusex/include/core/ProjectDB.hpp` add `#include <limits>` and `#include <span>` to the include block (:15-24). Replace the whole block from `// --- Pose Graph ---` (:591) up to and including the declaration of `has_pose_graph()` (:622) with:

```cpp
  // --- Pose Graph ---
  //
  // The stored pose graph is the source of truth for `rux optimize` (schema
  // v27, docs/superpowers/specs/2026-10-13-pose-graph-editor-design.md §3).
  // Rows are canonical: from_node_id < to_node_id, UNIQUE(from, to, type).
  // Node positions are read from `sensor_frames` (their current world poses,
  // and the snapshot in `initial_transform`), so there is no nodes table.

  /// One edge stored in `pose_graph_edges`. The first five members keep their
  /// pre-v27 order so `{from, to, type, residual, weight}` still initialises.
  struct PoseGraphEdge {
    int from_node_id = 0;
    int to_node_id = 0;
    /// "odometry" | "loop_closure" | "panorama" (other strings are stored
    /// as given; ruxd may write them).
    std::string edge_type;
    /// 0.5 x whitened squared residual after the last solve (GTSAM
    /// convention). NaN <-> NULL (never solved).
    double residual = std::numeric_limits<double>::quiet_NaN();
    /// 1 / sigma_trans^2. Derived from `information` on write when that is
    /// present. NaN <-> NULL.
    double weight = std::numeric_limits<double>::quiet_NaN();
    /// Measurement T_from^-1 * T_to in the sensor_frames.transform
    /// convention, row-major. nullopt for migrated v15 rows.
    std::optional<std::array<double, 16>> transform;
    /// 6x6 information, GTSAM Pose3 order (rx, ry, rz, tx, ty, tz),
    /// row-major. nullopt: the per-type default sigma of PlaneGraphOptions.
    std::optional<std::array<double, 36>> information;
    /// rtabmap | odometry_init | orb | panorama | file | icp | manual | legacy
    std::string source = "manual";
    /// auto | confirmed | rejected
    std::string status = "auto";
    /// ISO-8601 UTC; "" on write means "now".
    std::string created_at;
    std::optional<double> seed_disagreement_m;
    std::optional<double> seed_disagreement_deg;
    std::optional<double> deviation_m;
    std::optional<double> deviation_deg;
    /// Row id, filled by the readers; ignored on write.
    int64_t id = 0;
  };

  /// Transitional (removed in Task 10 together with its only caller, the
  /// pre-v27 optimize): delete every row and insert @p edges as legacy rows.
  void save_pose_graph_edges(const std::vector<PoseGraphEdge> &edges);

  /// All stored edges (every status), ordered by id. Empty when the table
  /// does not exist. On a pre-v27 project opened read-only the rows carry
  /// source "legacy", status "auto" and no measurement.
  std::vector<PoseGraphEdge> list_pose_graph_edges() const;

  /// The edge with key (from, to, type), given in either direction.
  std::optional<PoseGraphEdge> pose_graph_edge(int from_node_id,
                                               int to_node_id,
                                               std::string_view edge_type) const;

  /// True when the pose graph table exists and has at least one row.
  bool has_pose_graph() const;
```

Leave `add_pose_graph_edge` / `delete_pose_graph_edges` declarations as they are for now (Task 3 replaces them).

- [ ] **Step 4: Implement the migration and the readers**

In `libs/reusex/src/core/ProjectDB.cpp`:

1. Add `#include "reusex/core/pose_graph.hpp"` next to the other `reusex/core` includes at the top.
2. `:410` → `static constexpr int LATEST_SCHEMA_VERSION = 27;`
3. After `bool hasDismissedTable = false;` (:407) add:

```cpp
  /// pose_graph_edges has the v27 columns. Same lifetime rule as above.
  bool hasPoseGraphV27 = false;
  /// sensor_frames.initial_transform exists (v27).
  bool hasInitialTransform = false;
  /// project_settings exists (v27).
  bool hasSettingsTable = false;
```

4. After `hasDismissedTable = tableExists("survey_dismissed_instances");` (:528) add:

```cpp
    hasPoseGraphV27 = columnExists("pose_graph_edges", "status");
    hasInitialTransform = columnExists("sensor_frames", "initial_transform");
    hasSettingsTable = tableExists("project_settings");
```

5. In `runMigrations`, after the `if (current < 26) { migrateToV26(); }` block add:

```cpp
    if (current < 27) {
      migrateToV27();
    }
```

6. After `migrateToV26()` (ends :2436) add:

```cpp
  void migrateToV27() {
    reusex::info("Migrating database to schema version 27");
    // Pose-graph editor (spec §3): the stored graph becomes the source of
    // truth for `rux optimize`. pose_graph_edges is REBUILT rather than
    // ALTERed: v15 declared `residual REAL NOT NULL` (an unsolved edge has no
    // residual) and SQLite cannot add the UNIQUE / CHECK constraints the
    // tombstones match on. Rows are canonicalised to from < to, self-loops
    // dropped, and duplicates collapsed onto the lowest id (SQLite's
    // single-MIN() aggregate returns the bare columns of that row).
    Savepoint sp(db, "migrate_v27");
    if (tableExists("pose_graph_edges") &&
        !columnExists("pose_graph_edges", "status")) {
      execOrThrow(R"(
        CREATE TABLE pose_graph_edges_v27 (
          id                    INTEGER PRIMARY KEY,
          from_node_id          INTEGER NOT NULL,
          to_node_id            INTEGER NOT NULL,
          edge_type             TEXT    NOT NULL,
          residual              REAL,
          weight                REAL,
          transform             BLOB,
          information           BLOB,
          source                TEXT    NOT NULL DEFAULT 'legacy',
          status                TEXT    NOT NULL DEFAULT 'auto'
            CHECK (status IN ('auto', 'confirmed', 'rejected')),
          created_at            TEXT,
          seed_disagreement_m   REAL,
          seed_disagreement_deg REAL,
          deviation_m           REAL,
          deviation_deg         REAL,
          CHECK (from_node_id < to_node_id),
          UNIQUE (from_node_id, to_node_id, edge_type)
        );
        INSERT INTO pose_graph_edges_v27
          (id, from_node_id, to_node_id, edge_type, residual, weight,
           source, status)
        SELECT MIN(id), MIN(from_node_id, to_node_id),
               MAX(from_node_id, to_node_id), edge_type, residual, weight,
               'legacy', 'auto'
        FROM pose_graph_edges
        WHERE from_node_id <> to_node_id
        GROUP BY MIN(from_node_id, to_node_id), MAX(from_node_id, to_node_id),
                 edge_type;
        DROP TABLE pose_graph_edges;
        ALTER TABLE pose_graph_edges_v27 RENAME TO pose_graph_edges;
      )");
    }
    if (tableExists("sensor_frames") &&
        !columnExists("sensor_frames", "initial_transform"))
      execOrThrow("ALTER TABLE sensor_frames ADD COLUMN initial_transform "
                  "BLOB;");
    execOrThrow(R"(
      CREATE TABLE IF NOT EXISTS project_settings (
        key   TEXT PRIMARY KEY,
        value TEXT NOT NULL
      );
    )");
    insertSchemaVersion(27, "Stored pose graph: edge measurements, "
                            "information, provenance, status and metrics; "
                            "sensor_frames.initial_transform; "
                            "project_settings");
    sp.release();
    reusex::info("Migration to schema version 27 complete");
  }
```

7. Replace `listPoseGraphEdges()` (:1639-1665) with a version that handles both shapes, and add the single-edge lookup. Put the row reader right above them:

```cpp
  static std::optional<double> optionalDouble(sqlite3_stmt *s, int col) {
    if (sqlite3_column_type(s, col) == SQLITE_NULL)
      return std::nullopt;
    return sqlite3_column_double(s, col);
  }

  template <size_t N>
  static std::optional<std::array<double, N>> optionalBlob(sqlite3_stmt *s,
                                                           int col) {
    const void *blob = sqlite3_column_blob(s, col);
    if (!blob ||
        sqlite3_column_bytes(s, col) != static_cast<int>(N * sizeof(double)))
      return std::nullopt;
    std::array<double, N> out{};
    std::memcpy(out.data(), blob, N * sizeof(double));
    return out;
  }

  static std::string textOr(sqlite3_stmt *s, int col, const char *fallback) {
    const auto *t = reinterpret_cast<const char *>(sqlite3_column_text(s, col));
    return t ? std::string(t) : std::string(fallback);
  }

  /// Column list matching readPoseGraphEdge(); the legacy list serves a
  /// pre-v27 project opened read-only.
  const char *poseGraphColumns() const {
    return hasPoseGraphV27
               ? "id, from_node_id, to_node_id, edge_type, residual, weight, "
                 "transform, information, source, status, created_at, "
                 "seed_disagreement_m, seed_disagreement_deg, deviation_m, "
                 "deviation_deg"
               : "id, from_node_id, to_node_id, edge_type, residual, weight";
  }

  ProjectDB::PoseGraphEdge readPoseGraphEdge(sqlite3_stmt *s) const {
    ProjectDB::PoseGraphEdge e;
    e.id = sqlite3_column_int64(s, 0);
    e.from_node_id = sqlite3_column_int(s, 1);
    e.to_node_id = sqlite3_column_int(s, 2);
    e.edge_type = textOr(s, 3, "");
    e.residual = optionalDouble(s, 4).value_or(
        std::numeric_limits<double>::quiet_NaN());
    e.weight = optionalDouble(s, 5).value_or(
        std::numeric_limits<double>::quiet_NaN());
    if (!hasPoseGraphV27) {
      e.source = std::string(core::pose_graph::kSourceLegacy);
      e.status = std::string(core::pose_graph::kStatusAuto);
      return e;
    }
    e.transform = optionalBlob<16>(s, 6);
    e.information = optionalBlob<36>(s, 7);
    e.source = textOr(s, 8, "legacy");
    e.status = textOr(s, 9, "auto");
    e.created_at = textOr(s, 10, "");
    e.seed_disagreement_m = optionalDouble(s, 11);
    e.seed_disagreement_deg = optionalDouble(s, 12);
    e.deviation_m = optionalDouble(s, 13);
    e.deviation_deg = optionalDouble(s, 14);
    return e;
  }

  std::vector<ProjectDB::PoseGraphEdge> listPoseGraphEdges() const {
    if (!tableExists("pose_graph_edges"))
      return {};
    const std::string sel = std::string("SELECT ") + poseGraphColumns() +
                            " FROM pose_graph_edges ORDER BY id;";
    sqlite3_stmt *stmt =
        prepare_or_throw(db, sel.c_str(), "list_pose_graph_edges");
    StmtGuard guard(stmt);
    std::vector<ProjectDB::PoseGraphEdge> out;
    int rc = SQLITE_ROW;
    while ((rc = sqlite3_step(stmt)) == SQLITE_ROW)
      out.push_back(readPoseGraphEdge(stmt));
    if (rc != SQLITE_DONE)
      throw std::runtime_error("list_pose_graph_edges: " +
                               std::string(sqlite3_errmsg(db)));
    return out;
  }

  std::optional<ProjectDB::PoseGraphEdge>
  poseGraphEdge(int from, int to, std::string_view type) const {
    if (!tableExists("pose_graph_edges"))
      return std::nullopt;
    // Pre-v27 rows are not canonical: match both orders there.
    const std::string sel =
        std::string("SELECT ") + poseGraphColumns() +
        " FROM pose_graph_edges WHERE edge_type = ?3 AND "
        "((from_node_id = ?1 AND to_node_id = ?2) OR "
        "(from_node_id = ?2 AND to_node_id = ?1)) ORDER BY id LIMIT 1;";
    sqlite3_stmt *stmt = prepare_or_throw(db, sel.c_str(), "pose_graph_edge");
    StmtGuard guard(stmt);
    sqlite3_bind_int(stmt, 1, from);
    sqlite3_bind_int(stmt, 2, to);
    const std::string t(type);
    sqlite3_bind_text(stmt, 3, t.c_str(), -1, SQLITE_TRANSIENT);
    const int rc = sqlite3_step(stmt);
    if (rc == SQLITE_ROW)
      return readPoseGraphEdge(stmt);
    if (rc != SQLITE_DONE)
      throw std::runtime_error("pose_graph_edge: " +
                               std::string(sqlite3_errmsg(db)));
    return std::nullopt;
  }
```

(`prepare_or_throw` and `StmtGuard` are the file's existing helpers, `:141` and the `StmtGuard` used throughout.)

8. Replace `savePoseGraphEdges` (:1600-1637) — transitional until Task 10 removes it with its only caller — by a v27-compatible version:

```cpp
  void savePoseGraphEdges(const std::vector<ProjectDB::PoseGraphEdge> &edges) {
    checkWritable();
    execOrThrow("BEGIN TRANSACTION;");
    try {
      execOrThrow("DELETE FROM pose_graph_edges;");
      const char *ins = R"(
        INSERT INTO pose_graph_edges
          (from_node_id, to_node_id, edge_type, residual, weight, source,
           status)
        VALUES (MIN(?1, ?2), MAX(?1, ?2), ?3, ?4, ?5, 'legacy', 'auto')
        ON CONFLICT (from_node_id, to_node_id, edge_type) DO NOTHING;
      )";
      sqlite3_stmt *stmt = prepare_or_throw(db, ins, "save_pose_graph_edges");
      StmtGuard guard(stmt);
      for (const auto &e : edges) {
        if (e.from_node_id == e.to_node_id)
          continue; // the v27 CHECK (from < to) forbids self-loops
        sqlite3_reset(stmt);
        sqlite3_bind_int(stmt, 1, e.from_node_id);
        sqlite3_bind_int(stmt, 2, e.to_node_id);
        sqlite3_bind_text(stmt, 3, e.edge_type.c_str(), -1, SQLITE_TRANSIENT);
        if (std::isfinite(e.residual))
          sqlite3_bind_double(stmt, 4, e.residual);
        else
          sqlite3_bind_null(stmt, 4);
        if (std::isfinite(e.weight))
          sqlite3_bind_double(stmt, 5, e.weight);
        else
          sqlite3_bind_null(stmt, 5);
        if (sqlite3_step(stmt) != SQLITE_DONE)
          throw std::runtime_error("pose_graph_edges insert failed: " +
                                   std::string(sqlite3_errmsg(db)));
      }
      execOrThrow("COMMIT;");
    } catch (...) {
      sqlite3_exec(db, "ROLLBACK;", nullptr, nullptr, nullptr);
      throw;
    }
  }
```

9. In the public section (`// --- Pose Graph ---`, :8412) add:

```cpp
std::optional<ProjectDB::PoseGraphEdge>
ProjectDB::pose_graph_edge(int from_node_id, int to_node_id,
                           std::string_view edge_type) const {
  return impl_->poseGraphEdge(from_node_id, to_node_id, edge_type);
}
```

(`has_pose_graph()` is unchanged; `addPoseGraphEdge` keeps compiling for now because its INSERT names only v15 columns, all of which still exist and `source`/`status` have defaults — Task 3 replaces it.)

- [ ] **Step 5: Run tests to verify they pass**

Run: `nix develop -c cmake --build build --target reusex_unit_tests -j$(nproc) && nix develop -c build/tests/reusex_unit_tests "[migration]"` then `... "[pose_graph]"`.
Expected: PASS, including the pre-existing `pose_graph migration v14->v15` and `save replaces previous run atomically` cases (still served by the transitional `save_pose_graph_edges`), and `MigrationV26_*`.

- [ ] **Step 6: Commit**

```bash
git add libs/reusex/include/core/ProjectDB.hpp libs/reusex/src/core/ProjectDB.cpp \
        tests/unit/core/test_project_db_migration_v27.cpp tests/unit/core/test_project_db_migration_v26.cpp
git commit -m "core: schema v27 — stored pose-graph columns, initial_transform, project_settings

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_018dLKtE8qid8y5QVF9b2yri"
```

---

## Task 3: Pose-graph write API (upsert, status, metrics, purge, deprecated wrappers)

**Files:**
- Modify: `libs/reusex/include/core/ProjectDB.hpp` (pose-graph block)
- Modify: `libs/reusex/src/core/ProjectDB.cpp` (`addPoseGraphEdge`/`deletePoseGraphEdges` :1680-1735 replaced; public wrappers :8422-8428)
- Modify: `apps/rux/qt/src/EdgeEditor.cpp:106-119,211-214`
- Modify: `tests/unit/rux_qt/test_edge_save_locked.cpp:28-44`
- Modify: `tests/unit/core/test_project_db_pose_graph.cpp` (append cases)

**Interfaces:**
- Consumes: `core::pose_graph` helpers (Task 1); `PoseGraphEdge` (Task 2).
- Produces: `upsert_pose_graph_edge`, `set_pose_graph_edge_status`, `update_pose_graph_edge_metrics`, `delete_pose_graph_edges_from_source`, `PoseGraphEdgeMetrics`, deprecated `add_pose_graph_edge`/`delete_pose_graph_edges` (signatures exactly as in "Interfaces for the Qt plan").

- [ ] **Step 1: Write the failing tests**

Append to `tests/unit/core/test_project_db_pose_graph.cpp` (add `#include <reusex/core/pose_graph.hpp>`, `#include <Eigen/Geometry>`, `#include <array>`, `#include <span>`, `#include <vector>` to its includes):

```cpp
namespace {

namespace pg = reusex::core::pose_graph;

std::array<double, 16> pose_rt(double yaw, double x, double y, double z) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.topLeftCorner<3, 3>() =
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  T(0, 3) = x;
  T(1, 3) = y;
  T(2, 3) = z;
  return pg::row_major_from_matrix(T);
}

ProjectDB::PoseGraphEdge measured_edge(int from, int to, std::string type) {
  ProjectDB::PoseGraphEdge e;
  e.from_node_id = from;
  e.to_node_id = to;
  e.edge_type = std::move(type);
  e.transform = pose_rt(0.3, 1.0, -2.0, 0.5);
  e.information = pg::row_major_from_information(
      pg::information_from_sigmas(0.05, 0.10));
  e.source = "orb";
  e.status = "auto";
  e.seed_disagreement_m = 0.25;
  e.seed_disagreement_deg = 3.0;
  return e;
}

} // namespace

TEST_CASE("PoseGraphUpsert_NewEdge_RoundTripsEveryField",
          "[projectdb][pose_graph]") {
  TempPath tmp("test_pose_graph_upsert");
  ProjectDB db(tmp.path);
  const auto in = measured_edge(3, 7, "loop_closure");
  REQUIRE(db.upsert_pose_graph_edge(in));

  const auto out = db.list_pose_graph_edges();
  REQUIRE(out.size() == 1);
  const auto &e = out[0];
  CHECK(e.id > 0);
  CHECK(e.from_node_id == 3);
  CHECK(e.to_node_id == 7);
  CHECK(e.edge_type == "loop_closure");
  CHECK(std::isnan(e.residual)); // never solved
  CHECK(e.weight == Approx(100.0)); // 1 / 0.10^2, from information
  REQUIRE(e.transform.has_value());
  CHECK(*e.transform == *in.transform);
  REQUIRE(e.information.has_value());
  CHECK(*e.information == *in.information);
  CHECK(e.source == "orb");
  CHECK(e.status == "auto");
  CHECK(e.created_at.size() == 20); // 2026-10-13T12:34:56Z
  CHECK(e.created_at.back() == 'Z');
  CHECK(e.seed_disagreement_m == Approx(0.25));
  CHECK_FALSE(e.deviation_m.has_value());
}

TEST_CASE("PoseGraphUpsert_ReversedDirection_StoresCanonicalInverse",
          "[projectdb][pose_graph]") {
  TempPath tmp("test_pose_graph_canonical");
  ProjectDB db(tmp.path);
  auto in = measured_edge(9, 4, "loop_closure");
  pg::Matrix6d info = pg::Matrix6d::Identity() * 50.0;
  info(0, 4) = info(4, 0) = 3.0; // a coupled, non-diagonal information
  in.information = pg::row_major_from_information(info);
  REQUIRE(db.upsert_pose_graph_edge(in));

  const auto e = db.pose_graph_edge(4, 9, "loop_closure");
  REQUIRE(e.has_value());
  CHECK(e->from_node_id == 4);
  CHECK(e->to_node_id == 9);
  const Eigen::Matrix4d T = pg::matrix_from_row_major(*in.transform);
  CHECK((pg::matrix_from_row_major(*e->transform) - T.inverse()).norm() <
        1e-12);
  const pg::Matrix6d expected = pg::reverse_edge_information(info, T);
  CHECK((pg::information_from_row_major(*e->information) - expected).norm() <
        1e-9);

  // The same pair, given forwards, replaces it rather than adding a row.
  auto fwd = measured_edge(4, 9, "loop_closure");
  REQUIRE(db.upsert_pose_graph_edge(fwd));
  REQUIRE(db.list_pose_graph_edges().size() == 1);
  CHECK(*db.pose_graph_edge(4, 9, "loop_closure")->transform ==
        *fwd.transform);
}

TEST_CASE("PoseGraphUpsert_OverTombstone_IsRefusedUnlessOverwritten",
          "[projectdb][pose_graph]") {
  TempPath tmp("test_pose_graph_tombstone");
  ProjectDB db(tmp.path);
  REQUIRE(db.upsert_pose_graph_edge(measured_edge(1, 5, "loop_closure")));
  REQUIRE(db.set_pose_graph_edge_status(5, 1, "loop_closure", "rejected"));

  auto again = measured_edge(1, 5, "loop_closure");
  again.transform = pose_rt(0.0, 9.0, 9.0, 9.0);
  CHECK_FALSE(db.upsert_pose_graph_edge(again));
  auto row = db.pose_graph_edge(1, 5, "loop_closure");
  CHECK(row->status == "rejected");
  CHECK((*row->transform)[3] != 9.0); // untouched

  again.status = "confirmed";
  CHECK(db.upsert_pose_graph_edge(again, /*overwrite_rejected=*/true));
  row = db.pose_graph_edge(1, 5, "loop_closure");
  CHECK(row->status == "confirmed");
  CHECK((*row->transform)[3] == 9.0);
}

TEST_CASE("PoseGraphStatus_UnknownEdgeOrStatus_IsReported",
          "[projectdb][pose_graph]") {
  TempPath tmp("test_pose_graph_status");
  ProjectDB db(tmp.path);
  CHECK_FALSE(db.set_pose_graph_edge_status(1, 2, "odometry", "rejected"));
  REQUIRE(db.upsert_pose_graph_edge(measured_edge(1, 2, "odometry")));
  CHECK_THROWS_AS(db.set_pose_graph_edge_status(1, 2, "odometry", "deleted"),
                  std::invalid_argument);
  CHECK(db.set_pose_graph_edge_status(1, 2, "odometry", "confirmed"));
  CHECK(db.pose_graph_edge(1, 2, "odometry")->status == "confirmed");
}

TEST_CASE("PoseGraphMetrics_Update_WritesAndClears",
          "[projectdb][pose_graph]") {
  TempPath tmp("test_pose_graph_metrics");
  ProjectDB db(tmp.path);
  REQUIRE(db.upsert_pose_graph_edge(measured_edge(1, 2, "odometry")));
  REQUIRE(db.upsert_pose_graph_edge(measured_edge(2, 6, "loop_closure")));

  std::vector<ProjectDB::PoseGraphEdgeMetrics> m(2);
  m[0] = {1, 2, "odometry", 0.5, 0.01, 0.2, 0.0, 0.0};
  m[1] = {6, 2, "loop_closure", std::nullopt, 0.4, 5.0, 0.3, 4.0};
  CHECK(db.update_pose_graph_edge_metrics(m) == 2);

  const auto a = db.pose_graph_edge(1, 2, "odometry");
  CHECK(a->residual == Approx(0.5));
  CHECK(a->deviation_m == Approx(0.01));
  const auto b = db.pose_graph_edge(2, 6, "loop_closure");
  CHECK(std::isnan(b->residual));
  CHECK(b->deviation_deg == Approx(5.0));
  CHECK(b->seed_disagreement_m == Approx(0.3));
  CHECK(b->status == "auto"); // metrics never touch status
}

TEST_CASE("PoseGraphPurgeBySource_KeepsTombstonesAndOtherSources",
          "[projectdb][pose_graph]") {
  TempPath tmp("test_pose_graph_purge");
  ProjectDB db(tmp.path);
  auto a = measured_edge(1, 2, "odometry");
  a.source = "odometry_init";
  auto b = measured_edge(2, 3, "odometry");
  b.source = "odometry_init";
  auto c = measured_edge(3, 4, "odometry");
  c.source = "rtabmap";
  REQUIRE(db.upsert_pose_graph_edge(a));
  REQUIRE(db.upsert_pose_graph_edge(b));
  REQUIRE(db.upsert_pose_graph_edge(c));
  REQUIRE(db.set_pose_graph_edge_status(2, 3, "odometry", "rejected"));

  CHECK(db.delete_pose_graph_edges_from_source("odometry_init") == 1);
  const auto rows = db.list_pose_graph_edges();
  REQUIRE(rows.size() == 2);
  CHECK(db.pose_graph_edge(2, 3, "odometry")->status == "rejected");
  CHECK(db.pose_graph_edge(3, 4, "odometry").has_value());
}

TEST_CASE("PoseGraphUpsert_InvalidInput_Throws", "[projectdb][pose_graph]") {
  TempPath tmp("test_pose_graph_invalid");
  ProjectDB db(tmp.path);
  auto self = measured_edge(4, 4, "loop_closure");
  CHECK_THROWS_AS(db.upsert_pose_graph_edge(self), std::invalid_argument);
  auto status = measured_edge(1, 2, "loop_closure");
  status.status = "maybe";
  CHECK_THROWS_AS(db.upsert_pose_graph_edge(status), std::invalid_argument);
  auto broken = measured_edge(1, 2, "loop_closure");
  broken.transform = std::array<double, 16>{}; // all zero: not rigid
  CHECK_THROWS_AS(db.upsert_pose_graph_edge(broken), std::invalid_argument);
  auto asym = measured_edge(1, 2, "loop_closure");
  (*asym.information)[1] = 7.0;
  CHECK_THROWS_AS(db.upsert_pose_graph_edge(asym), std::invalid_argument);
  auto no_source = measured_edge(1, 2, "loop_closure");
  no_source.source.clear();
  CHECK_THROWS_AS(db.upsert_pose_graph_edge(no_source), std::invalid_argument);
  CHECK(db.list_pose_graph_edges().empty());
}

TEST_CASE("PoseGraphUpsert_ReadOnlyProject_Throws", "[projectdb][pose_graph]") {
  TempPath tmp("test_pose_graph_ro");
  { ProjectDB db(tmp.path); }
  ProjectDB ro(tmp.path, /*readOnly=*/true);
  CHECK_THROWS_AS(ro.upsert_pose_graph_edge(measured_edge(1, 2, "odometry")),
                  std::runtime_error);
}

#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
TEST_CASE("PoseGraphDeprecatedWrappers_KeepRuxdWorking",
          "[projectdb][pose_graph]") {
  TempPath tmp("test_pose_graph_deprecated");
  ProjectDB db(tmp.path);
  ProjectDB::PoseGraphEdge e{8, 3, "loop_closure", 0.4, 25.0};
  const int id = db.add_pose_graph_edge(e);
  CHECK(id > 0);
  const auto row = db.pose_graph_edge(3, 8, "loop_closure");
  REQUIRE(row.has_value());
  CHECK(row->id == id);
  CHECK(row->source == "manual");
  // "delete" is now a reject: the row stays, as a tombstone.
  CHECK(db.delete_pose_graph_edges(8, 3, "loop_closure") == 1);
  CHECK(db.pose_graph_edge(3, 8, "loop_closure")->status == "rejected");
  // ... and add over it un-rejects (the old add always wrote a row).
  CHECK(db.add_pose_graph_edge(e) == id);
  CHECK(db.pose_graph_edge(3, 8, "loop_closure")->status == "auto");
  // Empty type rejects every type on the pair.
  REQUIRE(db.upsert_pose_graph_edge({3, 8, "odometry", 0.1, 1.0}));
  CHECK(db.delete_pose_graph_edges(3, 8) == 2);
}
#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif
```

- [ ] **Step 2: Run test to verify it fails**

Run: `nix develop -c cmake --build build --target reusex_unit_tests -j$(nproc)`
Expected: compile FAIL — `no member named 'upsert_pose_graph_edge'`.

- [ ] **Step 3: Declare the write API**

In `ProjectDB.hpp`, right after `has_pose_graph()` (Task 2) replace the old `add_pose_graph_edge` / `delete_pose_graph_edges` declarations with:

```cpp
  /// Per-edge solve metrics, written by `rux optimize`. The key may be given
  /// in either direction. nullopt writes NULL.
  struct PoseGraphEdgeMetrics {
    int from_node_id = 0;
    int to_node_id = 0;
    std::string edge_type;
    std::optional<double> residual;
    std::optional<double> deviation_m;
    std::optional<double> deviation_deg;
    std::optional<double> seed_disagreement_m;
    std::optional<double> seed_disagreement_deg;
  };

  /// Insert @p edge, or replace every column of the row with the same
  /// canonical key (from < to, edge_type). An edge given with from > to is
  /// canonicalised: transform inverted, information mapped through the
  /// adjoint (core::pose_graph::reverse_edge_information); metric magnitudes
  /// are kept as given. `weight` is derived from `information` when present.
  ///
  /// @returns false — and writes nothing — when the stored row is "rejected"
  ///          and @p overwrite_rejected is false (tombstones survive
  ///          `rux graph discover`).
  /// @throws std::invalid_argument for from == to, an empty edge_type or
  ///         source, an invalid status, a non-rigid transform or an invalid
  ///         information; std::runtime_error on a read-only project.
  bool upsert_pose_graph_edge(const PoseGraphEdge &edge,
                              bool overwrite_rejected = false);

  /// Set "auto" | "confirmed" | "rejected" on the edge (either direction).
  /// @returns false when no such edge exists.
  /// @throws std::invalid_argument for any other status string.
  bool set_pose_graph_edge_status(int from_node_id, int to_node_id,
                                  std::string_view edge_type,
                                  std::string_view status);

  /// Write residual / deviation / seed-disagreement for many edges inside one
  /// SAVEPOINT (nests in a ProjectDB::Transaction). Unknown keys are skipped.
  /// @returns the number of rows updated.
  int update_pose_graph_edge_metrics(
      std::span<const PoseGraphEdgeMetrics> metrics);

  /// Delete every NON-rejected row with this source; tombstones stay.
  /// @returns the number of rows deleted.
  int delete_pose_graph_edges_from_source(std::string_view source);

  /// Superseded by upsert_pose_graph_edge (kept for ruxd, which builds against
  /// the installed package). Upserts with overwrite_rejected = true and
  /// source "manual" when the edge has none; returns the row id.
  [[deprecated("use upsert_pose_graph_edge")]]
  int add_pose_graph_edge(const PoseGraphEdge &edge);

  /// Superseded by set_pose_graph_edge_status: marks the matching edges
  /// "rejected" (every type on the pair when @p edge_type is empty) instead
  /// of deleting them. @returns the number of rows marked.
  [[deprecated("use set_pose_graph_edge_status(..., \"rejected\")")]]
  int delete_pose_graph_edges(int from_node_id, int to_node_id,
                              std::string_view edge_type = "");
```

- [ ] **Step 4: Implement**

In `ProjectDB.cpp`, delete `addPoseGraphEdge` and `deletePoseGraphEdges` (:1680-1735) and add in their place:

```cpp
  void requireV27PoseGraph(const char *who) const {
    if (!hasPoseGraphV27)
      throw std::runtime_error(std::string(who) +
                               ": project schema predates v27; open it "
                               "read-write once to migrate");
  }

  static void bindOptionalDouble(sqlite3_stmt *s, int idx,
                                 const std::optional<double> &v) {
    if (v && std::isfinite(*v))
      sqlite3_bind_double(s, idx, *v);
    else
      sqlite3_bind_null(s, idx);
  }

  static void bindNanAsNull(sqlite3_stmt *s, int idx, double v) {
    if (std::isfinite(v))
      sqlite3_bind_double(s, idx, v);
    else
      sqlite3_bind_null(s, idx);
  }

  bool upsertPoseGraphEdge(ProjectDB::PoseGraphEdge e, bool overwriteRejected) {
    namespace pg = core::pose_graph;
    checkWritable();
    requireV27PoseGraph("upsert_pose_graph_edge");
    if (e.from_node_id == e.to_node_id)
      throw std::invalid_argument("upsert_pose_graph_edge: self-loop on node " +
                                  std::to_string(e.from_node_id));
    if (e.edge_type.empty())
      throw std::invalid_argument("upsert_pose_graph_edge: empty edge_type");
    if (e.source.empty())
      throw std::invalid_argument("upsert_pose_graph_edge: empty source");
    if (!pg::is_valid_status(e.status))
      throw std::invalid_argument("upsert_pose_graph_edge: invalid status '" +
                                  e.status + "'");
    if (e.transform && !pg::is_rigid_transform(*e.transform))
      throw std::invalid_argument(
          "upsert_pose_graph_edge: transform is not a rigid 4x4 pose");
    if (e.information && !pg::is_valid_information(*e.information))
      throw std::invalid_argument(
          "upsert_pose_graph_edge: information is not a finite symmetric 6x6");

    if (e.from_node_id > e.to_node_id) {
      std::swap(e.from_node_id, e.to_node_id);
      if (e.transform) {
        const Eigen::Matrix4d T = pg::matrix_from_row_major(*e.transform);
        if (e.information)
          e.information = pg::row_major_from_information(
              pg::reverse_edge_information(
                  pg::information_from_row_major(*e.information), T));
        e.transform = pg::row_major_from_matrix(T.inverse());
      }
    }
    if (e.information) {
      const auto &a = *e.information;
      e.weight = (a[21] + a[28] + a[35]) / 3.0; // mean translational precision
    }

    const char *sql = R"(
      INSERT INTO pose_graph_edges
        (from_node_id, to_node_id, edge_type, residual, weight, transform,
         information, source, status, created_at, seed_disagreement_m,
         seed_disagreement_deg, deviation_m, deviation_deg)
      VALUES (?1, ?2, ?3, ?4, ?5, ?6, ?7, ?8, ?9,
              COALESCE(NULLIF(?10, ''),
                       strftime('%Y-%m-%dT%H:%M:%SZ', 'now')),
              ?11, ?12, ?13, ?14)
      ON CONFLICT (from_node_id, to_node_id, edge_type) DO UPDATE SET
        residual = excluded.residual, weight = excluded.weight,
        transform = excluded.transform, information = excluded.information,
        source = excluded.source, status = excluded.status,
        created_at = excluded.created_at,
        seed_disagreement_m = excluded.seed_disagreement_m,
        seed_disagreement_deg = excluded.seed_disagreement_deg,
        deviation_m = excluded.deviation_m,
        deviation_deg = excluded.deviation_deg
      WHERE pose_graph_edges.status <> 'rejected' OR ?15 = 1;
    )";
    sqlite3_stmt *s = prepare_or_throw(db, sql, "upsert_pose_graph_edge");
    StmtGuard guard(s);
    sqlite3_bind_int(s, 1, e.from_node_id);
    sqlite3_bind_int(s, 2, e.to_node_id);
    sqlite3_bind_text(s, 3, e.edge_type.c_str(), -1, SQLITE_TRANSIENT);
    bindNanAsNull(s, 4, e.residual);
    bindNanAsNull(s, 5, e.weight);
    if (e.transform)
      sqlite3_bind_blob(s, 6, e.transform->data(), 16 * sizeof(double),
                        SQLITE_TRANSIENT);
    else
      sqlite3_bind_null(s, 6);
    if (e.information)
      sqlite3_bind_blob(s, 7, e.information->data(), 36 * sizeof(double),
                        SQLITE_TRANSIENT);
    else
      sqlite3_bind_null(s, 7);
    sqlite3_bind_text(s, 8, e.source.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(s, 9, e.status.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(s, 10, e.created_at.c_str(), -1, SQLITE_TRANSIENT);
    bindOptionalDouble(s, 11, e.seed_disagreement_m);
    bindOptionalDouble(s, 12, e.seed_disagreement_deg);
    bindOptionalDouble(s, 13, e.deviation_m);
    bindOptionalDouble(s, 14, e.deviation_deg);
    sqlite3_bind_int(s, 15, overwriteRejected ? 1 : 0);
    if (sqlite3_step(s) != SQLITE_DONE)
      throw std::runtime_error("upsert_pose_graph_edge: " +
                               std::string(sqlite3_errmsg(db)));
    return sqlite3_changes(db) > 0;
  }

  bool setPoseGraphEdgeStatus(int from, int to, std::string_view type,
                              std::string_view status) {
    checkWritable();
    requireV27PoseGraph("set_pose_graph_edge_status");
    if (!core::pose_graph::is_valid_status(status))
      throw std::invalid_argument("set_pose_graph_edge_status: invalid status '" +
                                  std::string(status) + "'");
    sqlite3_stmt *s = prepare_or_throw(
        db,
        "UPDATE pose_graph_edges SET status = ?4 WHERE from_node_id = "
        "MIN(?1, ?2) AND to_node_id = MAX(?1, ?2) AND edge_type = ?3;",
        "set_pose_graph_edge_status");
    StmtGuard guard(s);
    sqlite3_bind_int(s, 1, from);
    sqlite3_bind_int(s, 2, to);
    const std::string t(type), st(status);
    sqlite3_bind_text(s, 3, t.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(s, 4, st.c_str(), -1, SQLITE_TRANSIENT);
    if (sqlite3_step(s) != SQLITE_DONE)
      throw std::runtime_error("set_pose_graph_edge_status: " +
                               std::string(sqlite3_errmsg(db)));
    return sqlite3_changes(db) > 0;
  }

  int updatePoseGraphEdgeMetrics(
      std::span<const ProjectDB::PoseGraphEdgeMetrics> metrics) {
    checkWritable();
    requireV27PoseGraph("update_pose_graph_edge_metrics");
    Savepoint sp(db, "pose_graph_metrics");
    sqlite3_stmt *s = prepare_or_throw(
        db,
        "UPDATE pose_graph_edges SET residual = ?4, deviation_m = ?5, "
        "deviation_deg = ?6, seed_disagreement_m = ?7, "
        "seed_disagreement_deg = ?8 WHERE from_node_id = MIN(?1, ?2) AND "
        "to_node_id = MAX(?1, ?2) AND edge_type = ?3;",
        "update_pose_graph_edge_metrics");
    StmtGuard guard(s);
    int updated = 0;
    for (const auto &m : metrics) {
      sqlite3_reset(s);
      sqlite3_clear_bindings(s);
      sqlite3_bind_int(s, 1, m.from_node_id);
      sqlite3_bind_int(s, 2, m.to_node_id);
      sqlite3_bind_text(s, 3, m.edge_type.c_str(), -1, SQLITE_TRANSIENT);
      bindOptionalDouble(s, 4, m.residual);
      bindOptionalDouble(s, 5, m.deviation_m);
      bindOptionalDouble(s, 6, m.deviation_deg);
      bindOptionalDouble(s, 7, m.seed_disagreement_m);
      bindOptionalDouble(s, 8, m.seed_disagreement_deg);
      if (sqlite3_step(s) != SQLITE_DONE)
        throw std::runtime_error("update_pose_graph_edge_metrics: " +
                                 std::string(sqlite3_errmsg(db)));
      updated += sqlite3_changes(db);
    }
    sp.release();
    return updated;
  }

  int deletePoseGraphEdgesFromSource(std::string_view source) {
    checkWritable();
    requireV27PoseGraph("delete_pose_graph_edges_from_source");
    sqlite3_stmt *s = prepare_or_throw(
        db,
        "DELETE FROM pose_graph_edges WHERE source = ?1 AND status <> "
        "'rejected';",
        "delete_pose_graph_edges_from_source");
    StmtGuard guard(s);
    const std::string src(source);
    sqlite3_bind_text(s, 1, src.c_str(), -1, SQLITE_TRANSIENT);
    if (sqlite3_step(s) != SQLITE_DONE)
      throw std::runtime_error("delete_pose_graph_edges_from_source: " +
                               std::string(sqlite3_errmsg(db)));
    return sqlite3_changes(db);
  }
```

Add `#include <Eigen/Geometry>` and `#include <span>` to `ProjectDB.cpp` if not already present. Replace the public wrappers (:8422-8428) with:

```cpp
bool ProjectDB::upsert_pose_graph_edge(const PoseGraphEdge &edge,
                                       bool overwrite_rejected) {
  return impl_->upsertPoseGraphEdge(edge, overwrite_rejected);
}

bool ProjectDB::set_pose_graph_edge_status(int from_node_id, int to_node_id,
                                           std::string_view edge_type,
                                           std::string_view status) {
  return impl_->setPoseGraphEdgeStatus(from_node_id, to_node_id, edge_type,
                                       status);
}

int ProjectDB::update_pose_graph_edge_metrics(
    std::span<const PoseGraphEdgeMetrics> metrics) {
  return impl_->updatePoseGraphEdgeMetrics(metrics);
}

int ProjectDB::delete_pose_graph_edges_from_source(std::string_view source) {
  return impl_->deletePoseGraphEdgesFromSource(source);
}

int ProjectDB::add_pose_graph_edge(const PoseGraphEdge &edge) {
  PoseGraphEdge e = edge;
  if (e.source.empty())
    e.source = std::string(core::pose_graph::kSourceManual);
  if (!impl_->upsertPoseGraphEdge(e, /*overwriteRejected=*/true))
    return -1;
  const auto row = pose_graph_edge(e.from_node_id, e.to_node_id, e.edge_type);
  return row ? static_cast<int>(row->id) : -1;
}

int ProjectDB::delete_pose_graph_edges(int from_node_id, int to_node_id,
                                       std::string_view edge_type) {
  if (!edge_type.empty())
    return set_pose_graph_edge_status(from_node_id, to_node_id, edge_type,
                                      core::pose_graph::kStatusRejected)
               ? 1
               : 0;
  int n = 0;
  for (const auto &e : list_pose_graph_edges())
    if (std::min(e.from_node_id, e.to_node_id) ==
            std::min(from_node_id, to_node_id) &&
        std::max(e.from_node_id, e.to_node_id) ==
            std::max(from_node_id, to_node_id))
      n += set_pose_graph_edge_status(e.from_node_id, e.to_node_id,
                                      e.edge_type,
                                      core::pose_graph::kStatusRejected)
               ? 1
               : 0;
  return n;
}
```

If the compiler warns about the deprecated declarations being *defined* (GCC does not; Clang may), wrap the two definitions in the same `#pragma GCC diagnostic ignored "-Wdeprecated-declarations"` push/pop used in the test.

- [ ] **Step 5: Migrate the in-tree callers off the deprecated calls**

`apps/rux/qt/src/EdgeEditor.cpp:106-119` becomes:

```cpp
        for (const auto &op : snapshot) {
          const auto &e = op.edge;
          if (op.kind == PendingEdgeEdits::Op::Kind::remove) {
            db.set_pose_graph_edge_status(
                e.key.from, e.key.to, e.key.type,
                reusex::core::pose_graph::kStatusRejected);
          } else {
            reusex::ProjectDB::PoseGraphEdge row;
            row.from_node_id = e.key.from;
            row.to_node_id = e.key.to;
            row.edge_type = e.key.type;
            row.residual = e.residual;
            row.weight = e.weight;
            row.source = std::string(reusex::core::pose_graph::kSourceManual);
            row.status =
                std::string(reusex::core::pose_graph::kStatusConfirmed);
            db.upsert_pose_graph_edge(row, /*overwrite_rejected=*/true);
          }
        }
```

and in `EdgeEditor::reload()` (:211) skip tombstones so a "removed" edge stays removed on screen until P4 draws rejected edges:

```cpp
      for (const auto &e : db->list_pose_graph_edges()) {
        if (e.status == reusex::core::pose_graph::kStatusRejected)
          continue;
        base.push_back({{e.from_node_id, e.to_node_id, e.edge_type},
                        e.residual,
                        e.weight});
      }
```

Add `#include <reusex/core/pose_graph.hpp>` to `EdgeEditor.cpp`. Apply the identical replacement to `apply_ops()` in `tests/unit/rux_qt/test_edge_save_locked.cpp:28-44` (its assertions only count rows after an add, so they stay as they are; the read-only case still fails in `checkWritable()` with the same message).

- [ ] **Step 6: Run tests to verify they pass**

Run: `nix develop -c cmake --build build --target reusex_unit_tests -j$(nproc) && nix develop -c build/tests/reusex_unit_tests "[pose_graph]"` and `... "[rux_qt]"`.
Expected: PASS. Then `nix develop -c cmake --build build --target rux -j$(nproc)` builds with no new warnings (`grep -i deprecat` on the build output is empty).

- [ ] **Step 7: Commit**

```bash
git add libs/reusex/include/core/ProjectDB.hpp libs/reusex/src/core/ProjectDB.cpp \
        apps/rux/qt/src/EdgeEditor.cpp tests/unit/rux_qt/test_edge_save_locked.cpp \
        tests/unit/core/test_project_db_pose_graph.cpp
git commit -m "core: pose-graph upsert/status/metrics API; add/delete become deprecated wrappers

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_018dLKtE8qid8y5QVF9b2yri"
```

---

## Task 4: Initial poses and project settings

**Files:**
- Modify: `libs/reusex/include/core/ProjectDB.hpp` (after `has_sensor_frame_pose`, and a new `// --- Project settings ---` block)
- Modify: `libs/reusex/src/core/ProjectDB.cpp` (Impl helpers next to `readSensorFramePoseBlob` :2916; public wrappers next to `sensor_frame_pose` :6945)
- Create: `tests/unit/core/test_project_db_initial_pose_settings.cpp`

**Interfaces:**
- Consumes: `core::pose_graph::is_rigid_transform` (Task 1), `hasInitialTransform`/`hasSettingsTable` flags (Task 2).
- Produces: `set_initial_pose`, `initial_pose`, `has_initial_pose`, `setting`, `set_setting`.

- [ ] **Step 1: Write the failing test**

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// sensor_frames.initial_transform (the snapshot `rux optimize` seeds from) and
// project_settings (spec §3.2, §3.3).

#include <catch2/catch_test_macros.hpp>

#include <reusex/core/ProjectDB.hpp>

#include "../../support/pose_fixture.hpp"
#include "../../support/temp_path.hpp"

#include <sqlite3.h>

#include <stdexcept>

using reusex::ProjectDB;
using namespace reusex::test_support;

namespace {
void seed(ProjectDB &db, int id, std::array<double, 16> pose) {
  db.save_sensor_frame(id, make_color(8, 8), make_depth(8, 8, 1.0), cv::Mat(),
                       pose, make_intrinsics(8, 8, 4, 4, 8, 8), 0.0);
}
} // namespace

TEST_CASE("InitialPose_SetAndRead_IsIndependentOfTransform",
          "[projectdb][initial_pose]") {
  TempPath tmp("test_initial_pose");
  ProjectDB db(tmp.path);
  seed(db, 3, translation_pose(1, 2, 3));
  CHECK_FALSE(db.has_initial_pose(3));
  CHECK_FALSE(db.initial_pose(3).has_value());

  db.set_initial_pose(3, translation_pose(7, 8, 9));
  REQUIRE(db.has_initial_pose(3));
  CHECK((*db.initial_pose(3))[3] == 7.0);
  CHECK(db.sensor_frame_pose(3)[3] == 1.0); // transform untouched

  db.update_sensor_frame_pose(3, translation_pose(4, 4, 4));
  CHECK((*db.initial_pose(3))[3] == 7.0); // and vice versa
}

TEST_CASE("InitialPose_InvalidInput_Throws", "[projectdb][initial_pose]") {
  TempPath tmp("test_initial_pose_invalid");
  ProjectDB db(tmp.path);
  seed(db, 3, identity16());
  CHECK_THROWS_AS(db.set_initial_pose(99, identity16()), std::runtime_error);
  CHECK_THROWS_AS(db.set_initial_pose(3, zero_pose()), std::invalid_argument);
  CHECK_THROWS_AS(db.set_initial_pose(3, nan_translation_pose()),
                  std::invalid_argument);
}

TEST_CASE("InitialPose_ReadOnlyV26_ReadsAsAbsent", "[projectdb][initial_pose]") {
  TempPath tmp("test_initial_pose_v26");
  {
    ProjectDB db(tmp.path);
    seed(db, 3, identity16());
  }
  sqlite3 *raw = nullptr;
  REQUIRE(sqlite3_open(tmp.path.string().c_str(), &raw) == SQLITE_OK);
  REQUIRE(sqlite3_exec(raw,
                       "ALTER TABLE sensor_frames DROP COLUMN "
                       "initial_transform; DROP TABLE project_settings; "
                       "DELETE FROM schema_version WHERE version >= 27;",
                       nullptr, nullptr, nullptr) == SQLITE_OK);
  sqlite3_close(raw);
  ProjectDB ro(tmp.path, /*readOnly=*/true);
  CHECK_FALSE(ro.has_initial_pose(3));
  CHECK_FALSE(ro.initial_pose(3).has_value());
  CHECK_FALSE(ro.setting("view.yaw_deg").has_value());
}

TEST_CASE("ProjectSettings_RoundTripAndOverwrite", "[projectdb][settings]") {
  TempPath tmp("test_project_settings");
  {
    ProjectDB db(tmp.path);
    CHECK_FALSE(db.setting("view.yaw_deg").has_value());
    db.set_setting("view.yaw_deg", "12.5");
    db.set_setting("view.yaw_deg", "-30");
    db.set_setting("other", "x");
  }
  ProjectDB db(tmp.path, /*readOnly=*/true);
  CHECK(db.setting("view.yaw_deg") == std::optional<std::string>("-30"));
  CHECK(db.setting("other") == std::optional<std::string>("x"));
  CHECK_THROWS_AS(db.set_setting("k", "v"), std::runtime_error);
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `nix develop -c cmake --build build --target reusex_unit_tests -j$(nproc)`
Expected: compile FAIL — `no member named 'set_initial_pose'`.

- [ ] **Step 3: Declare**

In `ProjectDB.hpp` after `bool has_sensor_frame_pose(int nodeId) const;` add:

```cpp
  /// Snapshot of the pose `rux optimize` seeds from (schema v27,
  /// sensor_frames.initial_transform). Written by `rux graph init` and
  /// `rux import rtabmap`; never by optimize.
  /// @throws std::invalid_argument if @p pose is not a rigid transform;
  ///         std::runtime_error if there is no such frame or the project is
  ///         read-only / pre-v27.
  void set_initial_pose(int nodeId, const std::array<double, 16> &pose);

  /// nullopt when NULL, not a rigid transform, the frame is missing, or the
  /// project predates v27 (a read-only open never migrates).
  std::optional<std::array<double, 16>> initial_pose(int nodeId) const;

  bool has_initial_pose(int nodeId) const;
```

and before `// --- Scans` add:

```cpp
  // --- Project settings (schema v27) ---

  /// nullopt for an unknown key or a pre-v27 project.
  std::optional<std::string> setting(std::string_view key) const;
  /// Insert or replace. @throws std::runtime_error on a read-only project.
  void set_setting(std::string_view key, std::string_view value);
```

- [ ] **Step 4: Implement**

Impl helpers (after `hasSensorFramePose`, ~:2987):

```cpp
  void setInitialPose(int nodeId, const std::array<double, 16> &pose) {
    checkWritable();
    if (!hasInitialTransform)
      throw std::runtime_error("set_initial_pose: project schema predates "
                               "v27; open it read-write once to migrate");
    if (!core::pose_graph::is_rigid_transform(pose))
      throw std::invalid_argument("set_initial_pose: frame " +
                                  std::to_string(nodeId) +
                                  ": not a rigid transform");
    sqlite3_stmt *s = prepare_or_throw(
        db, "UPDATE sensor_frames SET initial_transform = ? WHERE node_id = ?;",
        "set_initial_pose");
    StmtGuard guard(s);
    sqlite3_bind_blob(s, 1, pose.data(), 16 * sizeof(double), SQLITE_TRANSIENT);
    sqlite3_bind_int(s, 2, nodeId);
    if (sqlite3_step(s) != SQLITE_DONE)
      throw std::runtime_error("set_initial_pose: " +
                               std::string(sqlite3_errmsg(db)));
    if (sqlite3_changes(db) == 0)
      throw std::runtime_error("set_initial_pose: no sensor frame with "
                               "node_id " +
                               std::to_string(nodeId));
  }

  std::optional<std::array<double, 16>> initialPose(int nodeId) const {
    if (!hasInitialTransform)
      return std::nullopt;
    sqlite3_stmt *s = prepare_or_throw(
        db, "SELECT initial_transform FROM sensor_frames WHERE node_id = ?;",
        "initial_pose");
    StmtGuard guard(s);
    sqlite3_bind_int(s, 1, nodeId);
    if (sqlite3_step(s) != SQLITE_ROW)
      return std::nullopt;
    auto pose = optionalBlob<16>(s, 0);
    if (!pose || !core::pose_graph::is_rigid_transform(*pose))
      return std::nullopt;
    return pose;
  }

  std::optional<std::string> getSetting(std::string_view key) const {
    if (!hasSettingsTable)
      return std::nullopt;
    sqlite3_stmt *s = prepare_or_throw(
        db, "SELECT value FROM project_settings WHERE key = ?;", "setting");
    StmtGuard guard(s);
    const std::string k(key);
    sqlite3_bind_text(s, 1, k.c_str(), -1, SQLITE_TRANSIENT);
    if (sqlite3_step(s) != SQLITE_ROW)
      return std::nullopt;
    return textOr(s, 0, "");
  }

  void putSetting(std::string_view key, std::string_view value) {
    checkWritable();
    if (!hasSettingsTable)
      throw std::runtime_error("set_setting: project schema predates v27");
    sqlite3_stmt *s = prepare_or_throw(
        db,
        "INSERT INTO project_settings (key, value) VALUES (?1, ?2) "
        "ON CONFLICT (key) DO UPDATE SET value = excluded.value;",
        "set_setting");
    StmtGuard guard(s);
    const std::string k(key), v(value);
    sqlite3_bind_text(s, 1, k.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(s, 2, v.c_str(), -1, SQLITE_TRANSIENT);
    if (sqlite3_step(s) != SQLITE_DONE)
      throw std::runtime_error("set_setting: " +
                               std::string(sqlite3_errmsg(db)));
  }
```

(`optionalBlob`/`textOr` are the static helpers from Task 2; if they were placed after this point in the class, move them up — class members are visible anyway.) Public wrappers after `has_sensor_frame_pose` (:6951):

```cpp
void ProjectDB::set_initial_pose(int nodeId,
                                 const std::array<double, 16> &pose) {
  impl_->setInitialPose(nodeId, pose);
}
std::optional<std::array<double, 16>>
ProjectDB::initial_pose(int nodeId) const {
  return impl_->initialPose(nodeId);
}
bool ProjectDB::has_initial_pose(int nodeId) const {
  return impl_->initialPose(nodeId).has_value();
}
std::optional<std::string> ProjectDB::setting(std::string_view key) const {
  return impl_->getSetting(key);
}
void ProjectDB::set_setting(std::string_view key, std::string_view value) {
  impl_->putSetting(key, value);
}
```

- [ ] **Step 5: Run tests to verify they pass**

Run: `nix develop -c cmake --build build --target reusex_unit_tests -j$(nproc) && nix develop -c build/tests/reusex_unit_tests "[initial_pose],[settings]"`
Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add libs/reusex/include/core/ProjectDB.hpp libs/reusex/src/core/ProjectDB.cpp \
        tests/unit/core/test_project_db_initial_pose_settings.cpp
git commit -m "core: sensor_frames.initial_transform and project_settings API

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_018dLKtE8qid8y5QVF9b2yri"
```

---

## Task 5: `slam/pose_graph_conventions.hpp` — stored ↔ optical

**Files:**
- Create: `libs/reusex/include/slam/pose_graph_conventions.hpp`
- Test: `tests/unit/slam/test_pose_graph_conventions.cpp`

**Interfaces:**
- Consumes: `core::pose_graph::{Matrix6d, pose3_adjoint}` (Task 1).
- Produces: `slam::stored_to_optical`, `optical_to_stored`, `information_stored_to_optical`, `information_optical_to_stored` (signatures in "Interfaces for the Qt plan").

Background: the optimizer works on optical-frame poses `world_pose = worldTf · localTf` (`segmentation/surfel_extraction.cpp:138`). A stored measurement `T` (worldTf convention) becomes `L_i⁻¹ · T · L_j` in optical coordinates. The optical BetweenFactor error is `Ad(L_j⁻¹) · ξ_stored`, so information maps as `Λ_opt = Ad(L_j)ᵀ Λ Ad(L_j)`.

- [ ] **Step 1: Write the failing test**

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Stored (sensor_frames.transform) <-> optical (optimizer) edge conventions,
// spec §4.1.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <reusex/slam/pose_graph_conventions.hpp>

#include <Eigen/Geometry>

namespace pg = reusex::core::pose_graph;
using namespace reusex::slam;
using Catch::Matchers::WithinRel;

namespace {

Eigen::Matrix4d make_pose(double angle, const Eigen::Vector3d &axis,
                          const Eigen::Vector3d &t) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.topLeftCorner<3, 3>() =
      Eigen::AngleAxisd(angle, axis.normalized()).toRotationMatrix();
  T.topRightCorner<3, 1>() = t;
  return T;
}

Eigen::Matrix<double, 6, 1> small_log(const Eigen::Matrix4d &M) {
  const Eigen::AngleAxisd aa(Eigen::Matrix3d(M.topLeftCorner<3, 3>()));
  Eigen::Matrix<double, 6, 1> v;
  v.head<3>() = aa.angle() * aa.axis();
  v.tail<3>() = M.topRightCorner<3, 1>();
  return v;
}

// An iPad-like local transform: base -> optical rotation plus a lever arm.
const Eigen::Matrix4d kLi = make_pose(M_PI / 2, {1, -1, 1}, {0.02, -0.05, 0.01});
const Eigen::Matrix4d kLj = make_pose(M_PI / 2, {1, -1, 1}, {0.02, -0.05, 0.01});

} // namespace

TEST_CASE("PoseGraphConventions_Transform_RoundTrips", "[slam][pose_graph]") {
  const Eigen::Matrix4d T = make_pose(0.4, {0, 0, 1}, {1.0, 0.2, -0.3});
  const Eigen::Matrix4d opt = stored_to_optical(T, kLi, kLj);
  CHECK((optical_to_stored(opt, kLi, kLj) - T).norm() < 1e-12);
}

TEST_CASE("PoseGraphConventions_Transform_MatchesOpticalPoses",
          "[slam][pose_graph]") {
  // Stored poses W, optical poses W*L: the optical measurement must equal
  // (W_i L_i)^-1 (W_j L_j).
  const Eigen::Matrix4d W_i = make_pose(0.3, {0, 1, 0}, {2, 1, 0});
  const Eigen::Matrix4d W_j = make_pose(-0.2, {1, 0, 0}, {3, -1, 0.5});
  const Eigen::Matrix4d T = W_i.inverse() * W_j;
  const Eigen::Matrix4d expected = (W_i * kLi).inverse() * (W_j * kLj);
  CHECK((stored_to_optical(T, kLi, kLj) - expected).norm() < 1e-12);
}

TEST_CASE("PoseGraphConventions_Information_PreservesChiSquare",
          "[slam][pose_graph]") {
  const Eigen::Matrix4d T = make_pose(0.6, {1, 2, 0}, {1.0, -0.5, 0.2});
  const Eigen::Matrix4d W_i = make_pose(0.1, {0, 0, 1}, {0.5, 0.5, 0});
  const Eigen::Matrix4d D = make_pose(1e-4, {1, 0, 1}, {2e-4, -1e-4, 1e-4});
  const Eigen::Matrix4d W_j = W_i * T * D;
  pg::Matrix6d L = pg::information_from_sigmas(0.02, 0.05);
  L(1, 3) = L(3, 1) = 40.0;

  const auto e_stored = small_log(T.inverse() * W_i.inverse() * W_j);
  const Eigen::Matrix4d T_opt = stored_to_optical(T, kLi, kLj);
  const auto e_opt =
      small_log(T_opt.inverse() * (W_i * kLi).inverse() * (W_j * kLj));
  const pg::Matrix6d L_opt = information_stored_to_optical(L, kLj);
  CHECK_THAT(e_opt.dot(L_opt * e_opt),
             WithinRel(e_stored.dot(L * e_stored), 1e-3));
  CHECK((information_optical_to_stored(L_opt, kLj) - L).norm() <
        1e-9 * L.norm());
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `nix develop -c cmake --build build --target reusex_unit_tests -j$(nproc)`
Expected: FAIL — header not found.

- [ ] **Step 3: Write the header**

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// The one place stored pose-graph measurements (sensor_frames.transform
// convention, spec §3.1) are converted to and from the optimizer's optical
// convention (world_pose = worldTf * localTf, surfel_extraction.hpp).
//
//   T_optical = L_i^-1 * T_stored * L_j
//   e_optical = Ad(L_j^-1) * e_stored  =>  Info_opt = Ad(L_j)^T Info Ad(L_j)
//
// Header-only and GTSAM-free (STANDARDS §2).

#include <reusex/core/pose_graph.hpp>

#include <Eigen/Core>

namespace reusex::slam {

inline Eigen::Matrix4d stored_to_optical(const Eigen::Matrix4d &T_ij,
                                         const Eigen::Matrix4d &local_i,
                                         const Eigen::Matrix4d &local_j) {
  return local_i.inverse() * T_ij * local_j;
}

inline Eigen::Matrix4d optical_to_stored(const Eigen::Matrix4d &T_ij_optical,
                                         const Eigen::Matrix4d &local_i,
                                         const Eigen::Matrix4d &local_j) {
  return local_i * T_ij_optical * local_j.inverse();
}

inline core::pose_graph::Matrix6d
information_stored_to_optical(const core::pose_graph::Matrix6d &info,
                              const Eigen::Matrix4d &local_j) {
  const core::pose_graph::Matrix6d A = core::pose_graph::pose3_adjoint(local_j);
  const core::pose_graph::Matrix6d out = A.transpose() * info * A;
  return 0.5 * (out + out.transpose());
}

inline core::pose_graph::Matrix6d
information_optical_to_stored(const core::pose_graph::Matrix6d &info,
                              const Eigen::Matrix4d &local_j) {
  const core::pose_graph::Matrix6d A =
      core::pose_graph::pose3_adjoint(local_j.inverse());
  const core::pose_graph::Matrix6d out = A.transpose() * info * A;
  return 0.5 * (out + out.transpose());
}

} // namespace reusex::slam
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `nix develop -c cmake --build build --target reusex_unit_tests -j$(nproc) && nix develop -c build/tests/reusex_unit_tests "[slam][pose_graph]"`
Expected: PASS (3 cases).

- [ ] **Step 5: Commit**

```bash
git add libs/reusex/include/slam/pose_graph_conventions.hpp tests/unit/slam/test_pose_graph_conventions.cpp
git commit -m "slam: stored<->optical pose-graph edge conventions

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_018dLKtE8qid8y5QVF9b2yri"
```

---

## Task 6: ICP initial guess, edge transform and ICP edge information (for P5)

**Files:**
- Modify: `libs/reusex/include/slam/frame_pair_icp.hpp`
- Modify: `libs/reusex/src/slam/frame_pair_icp.cpp:96-160`
- Test: `tests/unit/slam/test_frame_pair_icp.cpp` (append)

**Interfaces:**
- Consumes: nothing new.
- Produces: `FramePairIcpOptions::initial_world_delta`, `FramePairIcpResult::edge_transform`, `icp_edge_information(const FramePairIcpResult&)`.

- [ ] **Step 1: Write the failing tests**

Append to `tests/unit/slam/test_frame_pair_icp.cpp`:

```cpp
TEST_CASE("FramePairIcp_InitialGuess_IsTheStartingPoint", "[slam][icp]") {
  // Both frames at the identity pose; "from" sees a plane at 1.0 m, "to" at
  // 1.1 m, so the correct world correction of "from" is +0.1 m in z. With a
  // 2 cm correspondence window ICP cannot find it from identity, but starts
  // right on it from the guess.
  const TempPath project("frame_pair_icp_guess");
  const auto intr = make_intr();
  {
    reusex::ProjectDB db(project.path, /*readOnly=*/false);
    db.save_sensor_frame(1, dummy_color(), flat_depth(1000), full_confidence(),
                         identity_pose(), intr, 0.0);
    db.save_sensor_frame(2, dummy_color(), flat_depth(1100), full_confidence(),
                         identity_pose(), intr, 1.0);
  }
  reusex::ProjectDB db(project.path, /*readOnly=*/true);

  reusex::slam::FramePairIcpOptions o;
  o.max_correspondence_m = 0.02;
  o.max_iterations = 5;
  const auto no_guess = reusex::slam::refine_frame_pair_icp(db, 1, 2, o);
  CHECK(std::abs(no_guess.world_delta[11]) < 0.01);

  o.initial_world_delta = std::array<double, 16>{1, 0, 0, 0,   0, 1, 0, 0,
                                                 0, 0, 1, 0.1, 0, 0, 0, 1};
  const auto guessed = reusex::slam::refine_frame_pair_icp(db, 1, 2, o);
  CHECK(guessed.converged);
  CHECK(std::abs(guessed.world_delta[11] - 0.1) < 0.01);
  CHECK(guessed.fitness < 0.01);
  // edge (1 -> 2) = (delta * T_1)^-1 * T_2 = translate(0, 0, -0.1)
  CHECK(std::abs(guessed.edge_transform[11] + 0.1) < 0.01);
}

TEST_CASE("FramePairIcp_EdgeTransform_IsInverseOfRelativePose",
          "[slam][icp]") {
  const TempPath project("frame_pair_icp_edge_transform");
  const auto intr = make_intr();
  {
    reusex::ProjectDB db(project.path, /*readOnly=*/false);
    db.save_sensor_frame(1, dummy_color(), flat_depth(1000), full_confidence(),
                         identity_pose(), intr, 0.0);
    db.save_sensor_frame(2, dummy_color(), flat_depth(1000), full_confidence(),
                         translate_pose(0.15), intr, 1.0);
  }
  reusex::ProjectDB db(project.path, /*readOnly=*/true);
  const auto r = reusex::slam::refine_frame_pair_icp(db, 1, 2);
  Eigen::Matrix4d rel, edge;
  for (int i = 0; i < 16; ++i) {
    rel(i / 4, i % 4) = r.relative_pose[static_cast<size_t>(i)];
    edge(i / 4, i % 4) = r.edge_transform[static_cast<size_t>(i)];
  }
  CHECK((edge - rel.inverse()).norm() < 1e-9);

  const auto info = reusex::slam::icp_edge_information(r);
  for (int k = 0; k < 6; ++k)
    CHECK(info[static_cast<size_t>(k * 6 + k)] > 0.0);
  CHECK(info[1] == 0.0); // diagonal
  // translation precision = 1 / max(fitness, 0.01)^2
  const double st = std::max(r.fitness, 0.01);
  CHECK(info[21] == Catch::Approx(1.0 / (st * st)));
}
```

Add `#include <Eigen/Core>` and `#include <algorithm>` to the test's includes.

- [ ] **Step 2: Run test to verify it fails**

Run: `nix develop -c cmake --build build --target reusex_unit_tests -j$(nproc)`
Expected: compile FAIL — `no member named 'initial_world_delta'`.

- [ ] **Step 3: Implement**

Header: add `#include <optional>`; append to `FramePairIcpOptions`:

```cpp
  /// World-space correction applied to the "from" cloud before ICP starts
  /// (row-major 4x4), e.g. a manual alignment from the pair panel. The
  /// result's world_delta INCLUDES it. nullopt = identity (seeded by the
  /// stored poses alone).
  std::optional<std::array<double, 16>> initial_world_delta;
```

append to `FramePairIcpResult`:

```cpp
  /// pose_graph_edges.transform for the edge (from_id -> to_id) in the
  /// sensor_frames.transform convention: (world_delta * T_from)^-1 * T_to,
  /// i.e. relative_pose^-1.
  std::array<double, 16> edge_transform{1, 0, 0, 0, 0, 1, 0, 0,
                                        0, 0, 1, 0, 0, 0, 0, 1};
```

and after `refine_frame_pair_icp`:

```cpp
/// Diagonal 6x6 information (GTSAM order rx, ry, rz, tx, ty, tz, row-major)
/// for a pose-graph edge measured by ICP: sigma_t = max(fitness, 0.01) m and
/// sigma_r = max(fitness / 2.0, 0.005) rad (the RMS point error over a ~2 m
/// lever arm).
std::array<double, 36> icp_edge_information(const FramePairIcpResult &result);
```

In `frame_pair_icp.cpp`, replace `CloudT aligned; icp.align(aligned);` with:

```cpp
  CloudT aligned;
  if (opts.initial_world_delta) {
    const Eigen::Matrix4f guess =
        Eigen::Map<const RowMajor4d>(opts.initial_world_delta->data())
            .cast<float>();
    icp.align(aligned, guess);
  } else {
    icp.align(aligned);
  }
```

and inside the final `for` loop that copies `rel`/`delta`, also fill the edge transform:

```cpp
  const Eigen::Matrix4d edge = rel.inverse();
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j) {
      result.relative_pose[static_cast<size_t>(i * 4 + j)] = rel(i, j);
      result.world_delta[static_cast<size_t>(i * 4 + j)] = delta(i, j);
      result.edge_transform[static_cast<size_t>(i * 4 + j)] = edge(i, j);
    }
```

Append at namespace scope:

```cpp
std::array<double, 36> icp_edge_information(const FramePairIcpResult &result) {
  const double st = std::max(result.fitness, 0.01);
  const double sr = std::max(result.fitness / 2.0, 0.005);
  std::array<double, 36> info{};
  for (int k = 0; k < 3; ++k) {
    info[static_cast<size_t>(k * 6 + k)] = 1.0 / (sr * sr);
    info[static_cast<size_t>((k + 3) * 6 + (k + 3))] = 1.0 / (st * st);
  }
  return info;
}
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `nix develop -c cmake --build build --target reusex_unit_tests -j$(nproc) && nix develop -c build/tests/reusex_unit_tests "[icp]"`
Expected: PASS (5 old + 2 new). If `InitialGuess` fails only on `no_guess.world_delta[11]` because PCL found correspondences at the plane borders, lower `o.max_correspondence_m` to `0.01`.

- [ ] **Step 5: Commit**

```bash
git add libs/reusex/include/slam/frame_pair_icp.hpp libs/reusex/src/slam/frame_pair_icp.cpp tests/unit/slam/test_frame_pair_icp.cpp
git commit -m "slam: ICP initial guess, edge_transform and icp_edge_information

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_018dLKtE8qid8y5QVF9b2yri"
```

---

# Phase P2 — `graph init`, stored-graph optimize, CLI break, RTABMap links, docs

## Task 7: `slam::init_pose_graph`

**Files:**
- Create: `libs/reusex/include/slam/pose_graph.hpp`, `libs/reusex/src/slam/pose_graph.cpp`
- Test: `tests/unit/slam/test_pose_graph_init.cpp`

**Interfaces:**
- Consumes: `ProjectDB::{upsert_pose_graph_edge, delete_pose_graph_edges_from_source, list_pose_graph_edges, set_initial_pose, initial_pose, has_initial_pose, sensor_frame_ids_with_scan, sensor_frame_pose, has_sensor_frame_pose}`, `ProjectDB::Transaction`, `core::pose_graph` helpers.
- Produces: `slam::PoseGraphExistsError`, `InitPoseGraphOptions`, `InitPoseGraphResult`, `count_usable_odometry_edges`, `odometry_init_edges`, `init_pose_graph` (exact signatures in "Interfaces for the Qt plan").

Rules (spec §4.3 + resolved ambiguities):
- Usable odometry = `edge_type == "odometry"`, `status != "rejected"`, transform present. Legacy NULL-transform rows do not count and are overwritten.
- Without `force`: refuse (throw `PoseGraphExistsError`) when usable odometry exists; snapshot `initial_transform` only for frames that lack one.
- With `force`: delete non-rejected `odometry_init` rows, re-snapshot every frame whose current transform is rigid, write `odometry_init` edges for consecutive pairs that have no other odometry row with a measurement and no odometry tombstone. Loop edges and tombstones are never touched.
- Consecutive = consecutive node ids **within one scan** (`sensor_frame_ids_with_scan`); scans are not chained (the optimizer's connectivity bridge, Task 10, keeps a multi-scan solve well-posed).
- `information` is NULL (→ `odometry_sigma_*` at solve time, so the `--odometry-sigma-*` flags keep working); `seed_disagreement_*` = 0.

- [ ] **Step 1: Write the failing test**

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// slam::init_pose_graph — `rux graph init` (spec §4.3, D3).

#include <catch2/catch_test_macros.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/pose_graph.hpp>
#include <reusex/slam/pose_graph.hpp>

#include "../../support/pose_fixture.hpp"
#include "../../support/temp_path.hpp"

#include <Eigen/Geometry>

using reusex::ProjectDB;
using namespace reusex::test_support;
namespace pg = reusex::core::pose_graph;
namespace slam = reusex::slam;

namespace {

std::array<double, 16> pose_k(int k) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.topLeftCorner<3, 3>() =
      Eigen::AngleAxisd(0.1 * k, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  T(0, 3) = 0.3 * k;
  T(1, 3) = 0.05 * k * k;
  return pg::row_major_from_matrix(T);
}

void seed(ProjectDB &db, int id, const std::array<double, 16> &pose,
          int scan_id = -1) {
  db.save_sensor_frame(id, make_color(8, 8), make_depth(8, 8, 1.0), cv::Mat(),
                       pose, make_intrinsics(8, 8, 4, 4, 8, 8), id, scan_id);
}

Eigen::Matrix4d rel(const ProjectDB &db, int a, int b) {
  return pg::matrix_from_row_major(db.sensor_frame_pose(a)).inverse() *
         pg::matrix_from_row_major(db.sensor_frame_pose(b));
}

} // namespace

TEST_CASE("InitPoseGraph_FreshProject_WritesOdometryChainAndSnapshots",
          "[slam][pose_graph][init]") {
  TempPath tmp("test_graph_init_fresh");
  ProjectDB db(tmp.path);
  for (int k = 1; k <= 5; ++k)
    seed(db, k, pose_k(k));

  const auto r = slam::init_pose_graph(db);
  CHECK(r.frames == 5);
  CHECK(r.snapshots_written == 5);
  CHECK(r.odometry_edges_written == 4);
  CHECK(r.scans == 1);

  const auto edges = db.list_pose_graph_edges();
  REQUIRE(edges.size() == 4);
  for (int k = 1; k <= 4; ++k) {
    INFO("edge " << k);
    const auto e = db.pose_graph_edge(k, k + 1, "odometry");
    REQUIRE(e.has_value());
    CHECK(e->source == "odometry_init");
    CHECK(e->status == "auto");
    CHECK_FALSE(e->information.has_value());
    CHECK(*e->seed_disagreement_m == 0.0);
    CHECK((pg::matrix_from_row_major(*e->transform) - rel(db, k, k + 1))
              .norm() < 1e-12);
  }
  for (int k = 1; k <= 5; ++k)
    CHECK(*db.initial_pose(k) == db.sensor_frame_pose(k));
  CHECK(slam::count_usable_odometry_edges(edges) == 4);
}

TEST_CASE("InitPoseGraph_Twice_RefusesWithoutForce",
          "[slam][pose_graph][init]") {
  TempPath tmp("test_graph_init_twice");
  ProjectDB db(tmp.path);
  for (int k = 1; k <= 3; ++k)
    seed(db, k, pose_k(k));
  slam::init_pose_graph(db);
  CHECK_THROWS_AS(slam::init_pose_graph(db), slam::PoseGraphExistsError);
}

TEST_CASE("InitPoseGraph_Force_ReplacesOnlyInitEdges",
          "[slam][pose_graph][init]") {
  TempPath tmp("test_graph_init_force");
  ProjectDB db(tmp.path);
  for (int k = 1; k <= 5; ++k)
    seed(db, k, pose_k(k));
  slam::init_pose_graph(db);
  const auto old_23 = *db.pose_graph_edge(2, 3, "odometry")->transform;

  ProjectDB::PoseGraphEdge loop;
  loop.from_node_id = 1;
  loop.to_node_id = 5;
  loop.edge_type = "loop_closure";
  loop.transform = pg::row_major_from_matrix(rel(db, 1, 5));
  loop.source = "manual";
  loop.status = "confirmed";
  REQUIRE(db.upsert_pose_graph_edge(loop));
  REQUIRE(db.set_pose_graph_edge_status(2, 3, "odometry", "rejected"));

  // Move every frame differently (register/perturb), then re-init.
  for (int k = 1; k <= 5; ++k)
    db.update_sensor_frame_pose(k, pose_k(k + 10));
  const auto r = slam::init_pose_graph(db, {.force = true});
  CHECK(r.odometry_edges_removed == 3); // (1,2), (3,4), (4,5); (2,3) is a tombstone
  CHECK(r.snapshots_written == 5);
  CHECK(r.odometry_edges_written == 3);
  CHECK(r.pairs_with_existing_odometry == 1);

  CHECK(*db.initial_pose(3) == pose_k(13));
  CHECK((pg::matrix_from_row_major(
             *db.pose_graph_edge(1, 2, "odometry")->transform) -
         rel(db, 1, 2))
            .norm() < 1e-12);
  const auto tomb = db.pose_graph_edge(2, 3, "odometry");
  CHECK(tomb->status == "rejected");
  CHECK(*tomb->transform == old_23);
  CHECK(db.pose_graph_edge(1, 5, "loop_closure")->status == "confirmed");
}

TEST_CASE("InitPoseGraph_TwoScans_DoesNotChainAcrossScans",
          "[slam][pose_graph][init]") {
  TempPath tmp("test_graph_init_scans");
  ProjectDB db(tmp.path);
  const auto a = db.create_scan("/data/scan_a.db");
  const auto b = db.create_scan("/data/scan_b.db");
  for (int k = 1; k <= 3; ++k)
    seed(db, k, pose_k(k), a.id);
  for (int k = 4; k <= 5; ++k)
    seed(db, k, pose_k(k), b.id);

  const auto r = slam::init_pose_graph(db);
  CHECK(r.scans == 2);
  CHECK(r.odometry_edges_written == 3);
  CHECK(db.pose_graph_edge(1, 2, "odometry").has_value());
  CHECK(db.pose_graph_edge(2, 3, "odometry").has_value());
  CHECK_FALSE(db.pose_graph_edge(3, 4, "odometry").has_value());
  CHECK(db.pose_graph_edge(4, 5, "odometry").has_value());
}

TEST_CASE("InitPoseGraph_PoselessFrame_IsSkipped", "[slam][pose_graph][init]") {
  TempPath tmp("test_graph_init_poseless");
  {
    ProjectDB db(tmp.path);
    for (int k = 1; k <= 4; ++k)
      seed(db, k, pose_k(k));
  }
  clear_sensor_frame_pose(tmp.path, 3);
  ProjectDB db(tmp.path);
  const auto r = slam::init_pose_graph(db);
  CHECK(r.frames == 3);
  CHECK(db.pose_graph_edge(1, 2, "odometry").has_value());
  CHECK(db.pose_graph_edge(2, 4, "odometry").has_value());
  CHECK_FALSE(db.has_initial_pose(3));
}

TEST_CASE("InitPoseGraph_LegacyNullTransformOdometry_IsReplaced",
          "[slam][pose_graph][init]") {
  // Review focus 4: rows left by a pre-v27 `rux optimize` must neither block
  // init nor survive it.
  TempPath tmp("test_graph_init_legacy");
  ProjectDB db(tmp.path);
  for (int k = 1; k <= 3; ++k)
    seed(db, k, pose_k(k));
  ProjectDB::PoseGraphEdge legacy{1, 2, "odometry", 0.1, 100.0};
  legacy.source = "legacy";
  REQUIRE(db.upsert_pose_graph_edge(legacy));
  REQUIRE(slam::count_usable_odometry_edges(db.list_pose_graph_edges()) == 0);

  slam::init_pose_graph(db); // must not throw PoseGraphExistsError
  const auto e = db.pose_graph_edge(1, 2, "odometry");
  CHECK(e->source == "odometry_init");
  CHECK(e->transform.has_value());
  CHECK(db.list_pose_graph_edges().size() == 2);
}

TEST_CASE("OdometryInitEdges_FromCurrentTransform_IgnoresTheSnapshot",
          "[slam][pose_graph][init]") {
  TempPath tmp("test_graph_init_plan");
  ProjectDB db(tmp.path);
  for (int k = 1; k <= 2; ++k)
    seed(db, k, pose_k(k));
  db.set_initial_pose(2, pose_k(7));
  const auto from_initial = slam::odometry_init_edges(db, false);
  const auto from_current = slam::odometry_init_edges(db, true);
  REQUIRE(from_initial.size() == 1);
  REQUIRE(from_current.size() == 1);
  CHECK((pg::matrix_from_row_major(*from_current[0].transform) - rel(db, 1, 2))
            .norm() < 1e-12);
  CHECK((pg::matrix_from_row_major(*from_initial[0].transform) -
         rel(db, 1, 2))
            .norm() > 1e-3);
  CHECK(db.list_pose_graph_edges().empty()); // planning writes nothing
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `nix develop -c cmake --build build --target reusex_unit_tests -j$(nproc)`
Expected: FAIL — `reusex/slam/pose_graph.hpp: No such file or directory`.

- [ ] **Step 3: Write the header**

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// `rux graph init` (spec 2026-10-13 §4.3, D3): snapshot the initial poses
// `rux optimize` seeds from and freeze odometry edges between consecutive
// frames of each scan. RTABMap imports get both from the importer instead.

#include <reusex/core/ProjectDB.hpp>

#include <stdexcept>
#include <vector>

namespace reusex::slam {

/// init_pose_graph() without `force` on a graph that already has usable
/// odometry. The CLI maps it to INVALID_ARGUMENT.
class PoseGraphExistsError : public std::runtime_error {
    public:
  using std::runtime_error::runtime_error;
};

struct InitPoseGraphOptions {
  /// Replace the `odometry_init` edges and re-snapshot every initial pose
  /// from the current transforms. Loop edges, other odometry sources and
  /// tombstones are left alone.
  bool force = false;
};

struct InitPoseGraphResult {
  int frames = 0;                  ///< posed frames considered
  int snapshots_written = 0;       ///< initial_transform rows written
  int odometry_edges_written = 0;  ///< odometry_init edges upserted
  int odometry_edges_removed = 0;  ///< odometry_init rows deleted (force)
  int pairs_with_existing_odometry = 0; ///< pairs left to a stored edge/tombstone
  int scans = 0;                   ///< scan groups chained independently
};

/// Odometry edges that can enter a solve: type "odometry", not rejected,
/// with a measurement. Legacy NULL-transform rows do not count.
int count_usable_odometry_edges(
    const std::vector<ProjectDB::PoseGraphEdge> &edges);

/// The `odometry_init` edges init_pose_graph() would write — one per
/// consecutive pair of posed frames within a scan — computed from the
/// initial poses (falling back to `transform` per frame), or from the
/// current transforms when @p from_current_transform. Writes nothing.
std::vector<ProjectDB::PoseGraphEdge>
odometry_init_edges(const ProjectDB &db, bool from_current_transform);

/// Opens its own ProjectDB::Transaction: do not call inside one.
/// @throws PoseGraphExistsError when usable odometry exists and !force.
InitPoseGraphResult init_pose_graph(ProjectDB &db,
                                    const InitPoseGraphOptions &options = {});

} // namespace reusex::slam
```

- [ ] **Step 4: Write the implementation**

`libs/reusex/src/slam/pose_graph.cpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "slam/pose_graph.hpp"

#include "core/logging.hpp"
#include "core/pose_graph.hpp"

#include <fmt/format.h>

#include <algorithm>
#include <map>
#include <set>
#include <utility>

namespace reusex::slam {
namespace {

namespace pg = core::pose_graph;

bool has_rigid_current_pose(const ProjectDB &db, int id) {
  return db.has_sensor_frame_pose(id) &&
         pg::is_rigid_transform(db.sensor_frame_pose(id));
}

/// Posed frames grouped by scan, each group in node-id order (the order
/// sensor_frame_ids_with_scan() returns).
std::map<int, std::vector<int>> posed_frames_by_scan(const ProjectDB &db,
                                                     bool from_current,
                                                     int *skipped = nullptr) {
  std::map<int, std::vector<int>> groups;
  int skip = 0;
  for (const auto &[id, scan] : db.sensor_frame_ids_with_scan()) {
    const bool posed = from_current
                           ? has_rigid_current_pose(db, id)
                           : (db.has_initial_pose(id) ||
                              has_rigid_current_pose(db, id));
    if (!posed) {
      ++skip;
      continue;
    }
    groups[scan].push_back(id);
  }
  if (skipped)
    *skipped = skip;
  return groups;
}

Eigen::Matrix4d seed_pose(const ProjectDB &db, int id, bool from_current) {
  if (!from_current)
    if (auto p = db.initial_pose(id))
      return pg::matrix_from_row_major(*p);
  return pg::matrix_from_row_major(db.sensor_frame_pose(id));
}

} // namespace

int count_usable_odometry_edges(
    const std::vector<ProjectDB::PoseGraphEdge> &edges) {
  return static_cast<int>(std::count_if(
      edges.begin(), edges.end(), [](const ProjectDB::PoseGraphEdge &e) {
        return e.edge_type == pg::kTypeOdometry &&
               e.status != pg::kStatusRejected && e.transform.has_value();
      }));
}

std::vector<ProjectDB::PoseGraphEdge>
odometry_init_edges(const ProjectDB &db, bool from_current_transform) {
  std::vector<ProjectDB::PoseGraphEdge> out;
  for (const auto &[scan, ids] :
       posed_frames_by_scan(db, from_current_transform)) {
    for (size_t k = 0; k + 1 < ids.size(); ++k) {
      const Eigen::Matrix4d A = seed_pose(db, ids[k], from_current_transform);
      const Eigen::Matrix4d B =
          seed_pose(db, ids[k + 1], from_current_transform);
      ProjectDB::PoseGraphEdge e;
      e.from_node_id = ids[k];
      e.to_node_id = ids[k + 1];
      e.edge_type = std::string(pg::kTypeOdometry);
      e.transform = pg::row_major_from_matrix(A.inverse() * B);
      e.source = std::string(pg::kSourceOdometryInit);
      e.status = std::string(pg::kStatusAuto);
      e.seed_disagreement_m = 0.0;
      e.seed_disagreement_deg = 0.0;
      out.push_back(std::move(e));
    }
  }
  return out;
}

InitPoseGraphResult init_pose_graph(ProjectDB &db,
                                    const InitPoseGraphOptions &options) {
  InitPoseGraphResult r;
  const int usable = count_usable_odometry_edges(db.list_pose_graph_edges());
  if (!options.force && usable > 0)
    throw PoseGraphExistsError(fmt::format(
        "the stored pose graph already has {} odometry edges; pass --force "
        "('rux graph init --force') to re-derive the odometry_init edges and "
        "re-snapshot the initial poses from the current poses",
        usable));

  ProjectDB::Transaction tx(db);
  if (options.force)
    r.odometry_edges_removed =
        db.delete_pose_graph_edges_from_source(pg::kSourceOdometryInit);

  int skipped = 0;
  const auto groups = posed_frames_by_scan(db, /*from_current=*/false, &skipped);
  r.scans = static_cast<int>(groups.size());
  for (const auto &[scan, ids] : groups) {
    r.frames += static_cast<int>(ids.size());
    for (int id : ids)
      if ((options.force || !db.has_initial_pose(id)) &&
          has_rigid_current_pose(db, id)) {
        db.set_initial_pose(id, db.sensor_frame_pose(id));
        ++r.snapshots_written;
      }
  }

  // A pair already covered by an odometry measurement (rtabmap) or an
  // odometry tombstone keeps it; legacy NULL-transform rows are overwritten.
  std::set<std::pair<int, int>> taken;
  for (const auto &e : db.list_pose_graph_edges())
    if (e.edge_type == pg::kTypeOdometry &&
        (e.transform.has_value() || e.status == pg::kStatusRejected))
      taken.emplace(e.from_node_id, e.to_node_id);
  for (const auto &e : odometry_init_edges(db, /*from_current=*/false)) {
    if (taken.count({e.from_node_id, e.to_node_id}) > 0 ||
        !db.upsert_pose_graph_edge(e)) {
      ++r.pairs_with_existing_odometry;
      continue;
    }
    ++r.odometry_edges_written;
  }
  tx.commit();

  if (skipped > 0)
    core::warn("GraphInit: {} sensor frames carry no usable pose (missing or "
               "non-rigid transform) and were left out of the odometry chain",
               skipped);
  core::info("GraphInit: {} frames in {} scan(s): {} initial poses "
             "snapshotted, {} odometry edges written ({} pairs kept an "
             "existing edge), {} odometry_init edges replaced",
             r.frames, r.scans, r.snapshots_written, r.odometry_edges_written,
             r.pairs_with_existing_odometry, r.odometry_edges_removed);
  return r;
}

} // namespace reusex::slam
```

- [ ] **Step 5: Run tests to verify they pass**

Run: `nix develop -c cmake --build build --target reusex_unit_tests -j$(nproc) && nix develop -c build/tests/reusex_unit_tests "[init]"`
Expected: PASS (7 cases).

- [ ] **Step 6: Commit**

```bash
git add libs/reusex/include/slam/pose_graph.hpp libs/reusex/src/slam/pose_graph.cpp tests/unit/slam/test_pose_graph_init.cpp
git commit -m "slam: init_pose_graph — initial-pose snapshot and odometry_init edges

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_018dLKtE8qid8y5QVF9b2yri"
```

---

## Task 8: Loop front-ends leave `optimize` (`collect_loop_candidates`) and the CLI break

**Files:**
- Create: `libs/reusex/include/slam/loop_candidates.hpp`, `libs/reusex/src/slam/loop_candidates.cpp`
- Modify: `libs/reusex/include/slam/PlaneGraphOptimizer.hpp` (remove the `loop_closure`, `panorama_loops`, `loop_edges_file`, `loop_edges_min_seed_disagreement`, `loop_edges_min_seed_disagreement_fraction` members and their comments, :350-434; drop `#include "reusex/slam/PanoramaLoopEdges.hpp"`; reword the `operator==` comment to "every field above and every field of `surfel`")
- Modify: `libs/reusex/src/slam/optimize_sensor_poses.cpp:59-279,288-293,342-366` (delete the `want_loops` block and the panorama attribution; call `optimizer.optimize(frames)`)
- Modify: `libs/reusex/include/slam/LoopClosure.hpp:62-64` and `PanoramaLoopEdges.hpp:73` (`enable` doc: "opt in with `rux graph discover`")
- Modify: `apps/rux/include/optimize.hpp:69-123`, `apps/rux/src/optimize.cpp` (flags :228-367, mapping :457-493, footer :26-69, log JSON :531-542, loop log :561-564)
- Modify: `tests/unit/slam/test_plane_graph_optimizer.cpp:856-863`, `tests/unit/slam/test_seed_disagreement_gate.cpp:33,173-209`, `tests/unit/rux_app/test_cli_command_roundtrip.cpp:360-361`
- Create: `tests/unit/slam/test_loop_candidates.cpp`, `tests/unit/rux_app/test_optimize_moved_flags.cpp`

**Interfaces:**
- Consumes: `geometry::{LoopClosureOptions, PanoramaLoopOptions, LoopEdge, detect_loop_edges, detect_panorama_loop_edges, load_loop_edges, trajectory_extent, seed_disagreement_gate}`.
- Produces:

```cpp
namespace reusex::slam {
struct LoopCandidateOptions {
  geometry::LoopClosureOptions loop_closure;
  geometry::PanoramaLoopOptions panorama_loops;
  std::string loop_edges_file;
  double loop_edges_min_seed_disagreement = 0.0;
  double loop_edges_min_seed_disagreement_fraction = 0.0278;
  bool operator==(const LoopCandidateOptions &) const = default;
};
struct LoopCandidate { geometry::LoopEdge edge; std::string source; }; // "orb"|"panorama"|"file"
struct LoopCandidateReport {
  int proposed = 0; int orb = 0; int panorama = 0; int file_loaded = 0; int file = 0;
  int dropped_duplicate = 0; int dropped_gate = 0;
};
std::vector<LoopCandidate> collect_loop_candidates(ProjectDB &db, const std::vector<int> &node_ids,
    const std::vector<Eigen::Matrix4d> &seed_poses, const LoopCandidateOptions &options,
    LoopCandidateReport *report = nullptr);
}
// apps/rux/include/optimize.hpp
const std::vector<std::pair<std::string, bool>> &optimize_moved_flags(); // {flag, takes_value}
```

PCM is **not** applied here (only inside `detect_loop_edges`, unchanged): discovery runs PCM over stored ∪ new edges (Task 15). Between this task and Task 10, `rux optimize` runs with no loop edges at all — intended; the stored graph supplies them from Task 10 on.

- [ ] **Step 1: Write the failing tests**

`tests/unit/slam/test_loop_candidates.cpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// slam::collect_loop_candidates — the ORB / panorama / file front-ends moved
// out of optimize_sensor_poses (spec §4.3). The file source runs without
// images, so it carries the unit test; ORB and panorama keep their own tests.

#include <catch2/catch_test_macros.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/slam/loop_candidates.hpp>

#include "../../support/temp_path.hpp"

#include <Eigen/Geometry>
#include <nlohmann/json.hpp>

#include <fstream>

using reusex::ProjectDB;
using reusex::test_support::TempPath;
namespace slam = reusex::slam;

namespace {

std::vector<Eigen::Matrix4d> seeds_along_x(int n) {
  std::vector<Eigen::Matrix4d> s;
  for (int k = 0; k < n; ++k) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T(0, 3) = 0.1 * k;
    s.push_back(T);
  }
  return s;
}

nlohmann::json edge_json(int ni, int nj, const Eigen::Matrix4d &T) {
  nlohmann::json e;
  e["node_i"] = ni;
  e["node_j"] = nj;
  std::vector<double> t;
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c)
      t.push_back(T(r, c));
  e["T_ij"] = t;
  e["sigma_rot"] = 0.05;
  e["sigma_trans"] = 0.1;
  e["inliers"] = 100;
  return e;
}

} // namespace

TEST_CASE("LoopCandidates_FileSource_GatesAndTagsEdges",
          "[slam][loop_candidates]") {
  TempPath project("test_loop_candidates");
  TempPath file("test_loop_candidates", ".json");
  ProjectDB db(project.path);
  const std::vector<int> ids = {10, 11, 12, 13, 14, 15, 16, 17};
  const auto seeds = seeds_along_x(8);

  Eigen::Matrix4d off = Eigen::Matrix4d::Identity();
  off(0, 3) = 0.5;
  nlohmann::json j;
  j["schema"] = "reusex.loop_edges.v1";
  j["producer"] = "test";
  j["edges"] = {edge_json(10, 17, seeds[0].inverse() * seeds[7]),       // agrees
                edge_json(11, 16, seeds[1].inverse() * seeds[6] * off)}; // 0.5 m off
  std::ofstream(file.path) << j.dump();

  slam::LoopCandidateOptions o;
  o.loop_closure.enable = false;
  o.loop_edges_file = file.path.string();
  o.loop_edges_min_seed_disagreement = 0.2;
  o.loop_edges_min_seed_disagreement_fraction = 0.0;

  slam::LoopCandidateReport rep;
  const auto c = slam::collect_loop_candidates(db, ids, seeds, o, &rep);
  REQUIRE(c.size() == 1);
  CHECK(c[0].source == "file");
  CHECK(c[0].edge.i == 1);
  CHECK(c[0].edge.j == 6);
  CHECK(rep.file_loaded == 2);
  CHECK(rep.dropped_gate == 1);
  CHECK(rep.file == 1);
}

TEST_CASE("LoopCandidates_NoSourceEnabled_ReturnsNothing",
          "[slam][loop_candidates]") {
  TempPath project("test_loop_candidates_none");
  ProjectDB db(project.path);
  slam::LoopCandidateOptions o; // loop_closure.enable defaults to false here
  CHECK(slam::collect_loop_candidates(db, {1, 2}, seeds_along_x(2), o).empty());
}

TEST_CASE("LoopCandidateOptions_OperatorEquals_IsMemberwiseThroughNestedStructs",
          "[slam][loop_candidates][parity]") {
  const slam::LoopCandidateOptions a;
  slam::LoopCandidateOptions b = a;
  CHECK(a == b);
  SECTION("a loop_closure field") {
    b.loop_closure.vocab_size = a.loop_closure.vocab_size + 1;
    CHECK_FALSE(a == b);
  }
  SECTION("a panorama_loops field") {
    b.panorama_loops.refine_iterations = a.panorama_loops.refine_iterations + 1;
    CHECK_FALSE(a == b);
  }
  SECTION("the file gate") {
    b.loop_edges_min_seed_disagreement_fraction = 0.5;
    CHECK_FALSE(a == b);
  }
}
```

`tests/unit/rux_app/test_optimize_moved_flags.cpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// The 2026-10-13 CLI break (spec D5): loop-edge discovery flags moved from
// `rux optimize` to `rux graph discover`; `optimize` refuses them with a
// pointer instead of "unknown option" or, worse, silently ignoring them.

#include <catch2/catch_test_macros.hpp>

#include <create/stage_bridge.hpp>
#include <global-params.hpp>
#include <optimize.hpp>

#include <CLI/CLI.hpp>

#include <memory>
#include <string>
#include <vector>

namespace {

bool parse_optimize(const std::vector<std::string> &args, bool &captured) {
  CLI::App app{"rux"};
  auto global = std::make_shared<RuxOptions>();
  app.add_option("-p,--project", global->project_db);
  setup_subcommand_optimize(app, global);
  captured = false;
  rux::set_stage_params_sink(
      [&](std::string_view, const std::string &) { captured = true; });
  std::vector<std::string> reversed(args.rbegin(), args.rend());
  bool refused = false;
  try {
    app.parse(reversed);
  } catch (const CLI::RuntimeError &) {
    refused = true;
  }
  rux::set_stage_params_sink({});
  return refused;
}

} // namespace

TEST_CASE("OptimizeMovedFlags_EveryMovedFlag_IsRefused",
          "[rux_app][cli][optimize]") {
  REQUIRE(optimize_moved_flags().size() == 24);
  for (const auto &[flag, takes_value] : optimize_moved_flags()) {
    INFO(flag);
    std::vector<std::string> args = {"optimize", flag};
    if (takes_value)
      args.push_back("1");
    bool captured = false;
    CHECK(parse_optimize(args, captured));
    CHECK_FALSE(captured); // refused before the stage parameters were built
  }
}

TEST_CASE("OptimizeMovedFlags_LoopTrustStaysOnOptimize",
          "[rux_app][cli][optimize]") {
  bool captured = false;
  CHECK_FALSE(parse_optimize(
      {"optimize", "--loop-trust", "--loop-trust-inlier-cost", "150"},
      captured));
  CHECK(captured);
}
```

Edit `tests/unit/slam/test_plane_graph_optimizer.cpp`: delete the two `SECTION`s "a loop_closure field the old hand list never covered" and "a panorama_loops field the old hand list never covered" (:855-863) — they moved to `LoopCandidateOptions_OperatorEquals_*`.

Edit `tests/unit/slam/test_seed_disagreement_gate.cpp`: add `#include <reusex/slam/loop_candidates.hpp>`, replace `using reusex::geometry::PlaneGraphOptions;` (:33) by `using reusex::slam::LoopCandidateOptions;`, and in the three `PlaneGraphOptions_*` test cases (:170-209) replace `PlaneGraphOptions opt` / `const PlaneGraphOptions opt` by `LoopCandidateOptions opt` / `const LoopCandidateOptions opt` (test names stay).

Edit `tests/unit/rux_app/test_cli_command_roundtrip.cpp`: delete the two lines `INFO(describe("loop_closure", ...));` and `INFO(describe("panorama_loops", ...));` (:360-361).

- [ ] **Step 2: Run tests to verify they fail**

Run: `nix develop -c cmake --build build --target reusex_unit_tests -j$(nproc)`
Expected: FAIL — `reusex/slam/loop_candidates.hpp: No such file or directory`.

- [ ] **Step 3: Write `loop_candidates.hpp`**

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// The wide-baseline loop-edge FRONT-ENDS (ORB frame pairs #225 P2, 360
// panoramas #236, external matcher files), moved out of
// optimize_sensor_poses when the stored pose graph became the source of truth
// (spec 2026-10-13 §4.3). `rux graph discover` calls this, filters against
// the stored graph and runs PCM over stored ∪ new edges.

#include <reusex/slam/LoopClosure.hpp>
#include <reusex/slam/PanoramaLoopEdges.hpp>

#include <Eigen/Core>

#include <string>
#include <vector>

namespace reusex {
class ProjectDB;
}

namespace reusex::slam {

struct LoopCandidateOptions {
  /// ORB frame-pair detection (`enable` defaults to false here; the
  /// discover options turn it on by default).
  geometry::LoopClosureOptions loop_closure;
  /// Panorama-derived edges (#236), gated by `loop_closure`'s
  /// min_frame_gap / seed-disagreement knobs.
  geometry::PanoramaLoopOptions panorama_loops;
  /// "reusex.loop_edges.v1" JSON from an out-of-process matcher; "" = none.
  std::string loop_edges_file;
  /// Absolute floor (m) of the seed-disagreement gate for file edges; the
  /// gate is max(fraction * trajectory_extent, floor) (#339).
  double loop_edges_min_seed_disagreement = 0.0;
  /// Scale-relative part of that gate; 0.0278 == 0.50 m on the 18.01 m
  /// office scan.
  double loop_edges_min_seed_disagreement_fraction = 0.0278;

  /// Memberwise through the nested option structs (their own `= default`).
  bool operator==(const LoopCandidateOptions &) const = default;
};

struct LoopCandidate {
  geometry::LoopEdge edge; ///< i/j index node_ids; T_ij optical convention
  std::string source;      ///< "orb" | "panorama" | "file"
};

struct LoopCandidateReport {
  int proposed = 0;          ///< pairs the front-ends tried / file edges loaded
  int orb = 0;               ///< ORB edges returned (after detect's own PCM)
  int panorama = 0;          ///< panorama edges kept after pair dedup
  int file_loaded = 0;       ///< valid edges in the file
  int file = 0;              ///< file edges kept after gate + dedup
  int dropped_duplicate = 0; ///< pair already proposed by an earlier source
  int dropped_gate = 0;      ///< file edges agreeing with the seed
};

/// Run the enabled front-ends over @p node_ids (DB node ids, index order)
/// with @p seed_poses (OPTICAL convention: initial pose * local transform).
/// One edge per frame pair: ORB wins over panorama wins over file. No PCM
/// across sources here.
std::vector<LoopCandidate>
collect_loop_candidates(ProjectDB &db, const std::vector<int> &node_ids,
                        const std::vector<Eigen::Matrix4d> &seed_poses,
                        const LoopCandidateOptions &options,
                        LoopCandidateReport *report = nullptr);

} // namespace reusex::slam
```

- [ ] **Step 4: Write `loop_candidates.cpp` (moved from `optimize_sensor_poses.cpp`, log lines kept verbatim for `scripts/bench-loop-edges.sh`)**

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "slam/loop_candidates.hpp"

#include "core/ProjectDB.hpp"
#include "core/logging.hpp"

#include <algorithm>
#include <set>
#include <utility>

namespace reusex::slam {
namespace {

std::pair<int, int> pair_key(const geometry::LoopEdge &e) {
  return {std::min(e.i, e.j), std::max(e.i, e.j)};
}

} // namespace

std::vector<LoopCandidate>
collect_loop_candidates(ProjectDB &db, const std::vector<int> &node_ids,
                        const std::vector<Eigen::Matrix4d> &seed_poses,
                        const LoopCandidateOptions &options,
                        LoopCandidateReport *report) {
  using namespace reusex::geometry;
  LoopCandidateReport rep;
  std::vector<LoopCandidate> out;
  // One factor per pair (two factors would multiply the information): the
  // first source to propose a pair wins, in ORB > panorama > file order —
  // ORB is a direct RANSAC on this scan's own depth.
  std::set<std::pair<int, int>> taken;

  if (options.loop_closure.enable) {
    LoopClosureResult lc;
    auto edges = detect_loop_edges(db, node_ids, seed_poses,
                                   options.loop_closure, &lc);
    core::info("PlaneGraph: loop closure proposed {} edges from {} matched "
               "candidates",
               lc.edges, lc.candidates);
    rep.proposed += lc.candidates;
    for (auto &e : edges) {
      taken.insert(pair_key(e));
      out.push_back({std::move(e), "orb"});
      ++rep.orb;
    }
  }

  if (options.panorama_loops.enable) {
    PanoramaLoopResult pl;
    // #339: resolve the scale-relative gate here so every source is judged
    // against one gate computed from one extent.
    LoopClosureOptions pano_gates = options.loop_closure;
    pano_gates.min_seed_disagreement = static_cast<float>(
        seed_disagreement_gate(trajectory_extent(seed_poses),
                               pano_gates.min_seed_disagreement_fraction,
                               pano_gates.min_seed_disagreement));
    pano_gates.min_seed_disagreement_fraction = 0.0f;
    auto pano = detect_panorama_loop_edges(db, node_ids, seed_poses,
                                           options.panorama_loops, pano_gates,
                                           &pl);
    if (pl.panoramas == 0)
      core::warn("PlaneGraph: --use-panoramas requested but this project "
                 "contains 0 panoramic images ({} sensor frames); no "
                 "panorama loop edges will be added. Import 360 imagery "
                 "with 'rux import 360' first.",
                 node_ids.size());
    rep.proposed += pl.proposed;
    int dropped_dup = 0;
    for (auto &e : pano) {
      if (!taken.insert(pair_key(e)).second) {
        ++dropped_dup;
        continue;
      }
      out.push_back({std::move(e), "panorama"});
      ++rep.panorama;
    }
    rep.dropped_duplicate += dropped_dup;
    if (dropped_dup > 0)
      core::info("PlaneGraph: dropped {} panorama loop edges duplicating an "
                 "internally-detected pair (avoids double factors)",
                 dropped_dup);
    core::info("PlaneGraph: {} panorama loop edges kept from {}/{} panoramas "
               "({} frame resections, {} pairs proposed; dropped {} gap / "
               "{} seed-gate / {} per-panorama cap)",
               rep.panorama, pl.panoramas_matched, pl.panoramas,
               pl.frames_resected, pl.proposed, pl.dropped_gap,
               pl.dropped_seed_gate, pl.dropped_cap);
  }

  if (!options.loop_edges_file.empty()) {
    LoopClosureResult ext;
    auto file_edges = load_loop_edges(options.loop_edges_file, node_ids, &ext);
    core::info("PlaneGraph: {} external loop edges loaded from '{}'",
               ext.edges, options.loop_edges_file);
    rep.file_loaded = ext.edges;
    rep.proposed += ext.edges;

    // An external edge agreeing with the seed carries no drift information
    // and only injects matcher noise (honka laser-GT 0.7958 -> 0.62); #339
    // scales the gate with the capture.
    const double extent = trajectory_extent(seed_poses);
    const double gate = seed_disagreement_gate(
        extent, options.loop_edges_min_seed_disagreement_fraction,
        options.loop_edges_min_seed_disagreement);
    core::info("PlaneGraph: trajectory extent {:.2f} m -> external "
               "seed-disagreement gate {:.3f} m (fraction {:.5f}, floor "
               "{:.3f} m)",
               extent, gate, options.loop_edges_min_seed_disagreement_fraction,
               options.loop_edges_min_seed_disagreement);

    int dropped = 0, dropped_dup = 0;
    for (auto &e : file_edges) {
      if (gate > 0.0 && e.i >= 0 && e.j >= 0) {
        const Eigen::Matrix4d rel_seed =
            seed_poses[e.i].inverse() * seed_poses[e.j];
        const double disagreement =
            (rel_seed.block<3, 1>(0, 3) - e.T_ij.block<3, 1>(0, 3)).norm();
        if (disagreement < gate) {
          ++dropped;
          continue;
        }
      }
      if (!taken.insert(pair_key(e)).second) {
        ++dropped_dup;
        continue;
      }
      out.push_back({std::move(e), "file"});
      ++rep.file;
    }
    rep.dropped_gate += dropped;
    rep.dropped_duplicate += dropped_dup;
    if (dropped > 0)
      core::info("PlaneGraph: dropped {} external loop edges that agree with "
                 "the seed within {:.3f} m (non-informative)",
                 dropped, gate);
    if (dropped_dup > 0)
      core::info("PlaneGraph: dropped {} external loop edges duplicating an "
                 "internally-detected pair (avoids double factors)",
                 dropped_dup);
    core::info("PlaneGraph: {} external loop edges kept after gating",
               rep.file);
  }

  if (report)
    *report = rep;
  return out;
}

} // namespace reusex::slam
```

- [ ] **Step 5: Remove the moved fields and the in-optimize front-ends**

1. `PlaneGraphOptimizer.hpp`: delete the members `loop_closure`, `panorama_loops`, `loop_edges_file`, `loop_edges_min_seed_disagreement`, `loop_edges_min_seed_disagreement_fraction` with their comment blocks; delete `#include "reusex/slam/PanoramaLoopEdges.hpp"`. Keep `loop_edges_trusted` and `loop_trust_inlier_cost`. Update the `operator==` comment to "Memberwise, covering every field above AND every field of `surfel`".
2. `optimize_sensor_poses.cpp`: delete everything from the `// P2: assemble wide-baseline loop edges` comment through the end of the `if (want_loops) { ... }` block (:59-279); replace `PlaneGraphResult result = optimizer.optimize(frames, loop_edges);` with `PlaneGraphResult result = optimizer.optimize(frames);`; delete the panorama-attribution loop after it (:288-293) and the `panorama_pairs` relabelling inside the persistence block (keep `std::string type = e.type;`). Remove the now-unused `#include <set>`.
3. In `LoopClosure.hpp` change the `enable` comment to `///< off by default here; on by default in `rux graph discover`` and in `PanoramaLoopEdges.hpp:73` change "opt in with `rux optimize --loop-closure --use-panoramas`" to "opt in with `rux graph discover --use-panoramas`".

- [ ] **Step 6: The CLI break**

`apps/rux/include/optimize.hpp`: delete the members from `// Wide-baseline loop closure (P2...` through `double loop_edges_min_disagreement_fraction = 0.0278;` (:69-123) and add back only

```cpp
  // Loop-edge solver handling (the edges themselves come from the stored
  // graph — `rux graph discover`). Defaults mirror PlaneGraphOptions.
  bool loop_trust = false;
  float loop_trust_inlier_cost =
      reusex::geometry::PlaneGraphOptions{}.loop_trust_inlier_cost;
```

and after the function declarations:

```cpp
/// Flags that moved to `rux graph discover` on 2026-10-13, with whether they
/// take a value. `rux optimize` registers them hidden and refuses them.
const std::vector<std::pair<std::string, bool>> &optimize_moved_flags();
```

(add `#include <utility>` and `#include <vector>`).

`apps/rux/src/optimize.cpp`:
- Delete the option registrations from `// --- Wide-baseline loop closure (P2) ---` (:228) to the end of `--loop-edges-min-disagreement-fraction` (:367), then re-add only:

```cpp
  // --- Loop-edge solver handling ---
  sub->add_flag(
      "--loop-trust", opt->loop_trust,
      "Trust loop edges: give them their own relaxed GNC-TLS inlier "
      "threshold (--loop-trust-inlier-cost) instead of --gnc-inlier-cost, so "
      "a genuine large-drift correction is not discarded as an outlier "
      "(Huber kernel under --no-gnc)");
  sub->add_option("--loop-trust-inlier-cost", opt->loop_trust_inlier_cost,
                  "GNC-TLS inlier threshold for --loop-trust loop edges "
                  "(0.5 * whitened squared residual)")
      ->default_val(opt->loop_trust_inlier_cost);

  // --- Flags that moved to `rux graph discover` (2026-10-13) ---
  // Registered hidden so a stale script gets a pointer instead of
  // "unknown option" (spec D5).
  auto moved_sink = std::make_shared<std::string>();
  auto moved =
      std::make_shared<std::vector<std::pair<std::string, CLI::Option *>>>();
  for (const auto &[flag, takes_value] : optimize_moved_flags()) {
    CLI::Option *o = takes_value ? sub->add_option(flag, *moved_sink)
                                 : sub->add_flag(flag);
    o->group(""); // hidden from --help
    moved->emplace_back(flag, o);
  }
```

- Replace the callback with:

```cpp
  sub->callback([opt, global_opt, moved, moved_sink]() {
    for (const auto &[flag, o] : *moved)
      if (o->count() > 0) {
        spdlog::error("'{}' moved to 'rux graph discover': rux optimize now "
                      "solves the stored pose graph. Run 'rux graph discover "
                      "{} ...' first, then 'rux optimize'.",
                      flag, flag);
        rux::finish(RuxError::INVALID_ARGUMENT);
      }
    spdlog::trace("calling run_subcommand_optimize");
    if (rux::capture_stage_params("optimize", optimize_stage_parameters(*opt)))
      return;
    rux::finish(run_subcommand_optimize(*opt, *global_opt));
  });
```

- Add at file scope (before `setup_subcommand_optimize`):

```cpp
const std::vector<std::pair<std::string, bool>> &optimize_moved_flags() {
  static const std::vector<std::pair<std::string, bool>> flags = {
      {"--loop-closure", false},
      {"--loop-proposal", true},
      {"--loop-min-frame-gap", true},
      {"--loop-max-distance", true},
      {"--loop-max-view-angle", true},
      {"--loop-max-candidates", true},
      {"--loop-min-inliers", true},
      {"--loop-max-features", true},
      {"--loop-ratio-test", true},
      {"--loop-ransac-inlier-dist", true},
      {"--loop-max-seed-disagreement", true},
      {"--loop-min-seed-disagreement", true},
      {"--loop-min-seed-disagreement-fraction", true},
      {"--loop-no-pcm", false},
      {"--use-panoramas", false},
      {"--pano-max-frames", true},
      {"--pano-min-inliers", true},
      {"--pano-max-edges", true},
      {"--pano-slices", true},
      {"--pano-max-features", true},
      {"--pano-max-distance", true},
      {"--loop-edges", true},
      {"--loop-edges-min-disagreement", true},
      {"--loop-edges-min-disagreement-fraction", true},
  };
  return flags;
}
```

- In `plane_graph_options()` delete every `options.loop_closure.*`, `options.loop_edges_file`, `options.loop_edges_min_seed_disagreement*` and `options.panorama_loops.*` line (:457-493 except the two below) so only these remain from that block:

```cpp
  options.loop_edges_trusted = opt.loop_trust;
  options.loop_trust_inlier_cost = opt.loop_trust_inlier_cost;
```

- Replace the `log_pipeline_start` JSON (:531-542) with:

```cpp
    int logId = db.log_pipeline_start(
        "pose_optimization_plane_graph",
        fmt::format(
            R"({{"min_observations":{},"assoc_rounds":{},"no_gnc":{},"assoc_normal_angle":{},"assoc_distance":{},"max_planes_per_frame":{},"min_plane_inliers":{},"plane_noise":"{}","use_gnc":{},"iterations":{},"seed":{},"dry_run":{},"loop_trust":{}}})",
            opt.min_observations, opt.assoc_rounds, opt.no_gnc,
            opt.assoc_normal_angle, opt.assoc_distance,
            opt.max_planes_per_frame, opt.min_plane_inliers,
            opt.no_plane_inlier_weight ? "uniform" : opt.plane_noise,
            !opt.no_gnc, opt.iterations, opt.seed, dry_run, opt.loop_trust));
```

- Replace the `if (opt.loop_closure || ...)` loop-log (:561-564) with

```cpp
    if (result.loop_edges > 0)
      spdlog::info("Loop closure: {} wide-baseline edges added to the graph",
                   result.loop_edges);
```

  (the text "Loop closure: N wide-baseline edges added" is parsed by `scripts/bench-loop-edges.sh`).
- Footer: replace the paragraph "Optimized poses overwrite the stored transforms in place — re-import the RTABMap database to recover the originals, or use --dry-run to preview." with

```
  The solve always starts from the initial poses (sensor_frames.
  initial_transform, snapshotted by 'rux graph init' or 'rux import rtabmap'),
  so re-running is repeatable. Odometry and loop edges come from the stored
  pose graph: 'rux graph discover' adds loop edges, 'rux graph prune' rejects
  bad ones. A project without odometry edges is initialised automatically.
```

  replace the last sentence of the next paragraph ("Optionally, wide-baseline loop-closure edges (P2) can be detected ... via --loop-closure.") with "Wide-baseline loop edges are found by 'rux graph discover'.", and the EXAMPLES block with

```
EXAMPLES:
  rux optimize                        # Defaults (GNC on, inlier-weighted planes)
  rux optimize --dry-run              # Report statistics without writing
  rux graph discover && rux optimize  # Add ORB loop edges, then solve
  rux graph discover --no-loop-closure --loop-edges edges.json \
    && rux optimize --loop-trust      # External learned-matcher edges
  rux optimize --min-observations 3 --assoc-distance 0.15
  rux optimize --no-gnc               # Plain Levenberg-Marquardt (no robustness)
```

  and the WORKFLOW block's step 2 with `2. rux graph discover; rux optimize   # Loop edges, then global pose optimization`.

- [ ] **Step 7: Run tests to verify they pass**

```bash
nix develop -c cmake --build build --target reusex_unit_tests reusex_unit_tests_vision rux -j$(nproc)
nix develop -c build/tests/reusex_unit_tests "[loop_candidates],[gate],[plane_graph]"
nix develop -c build/tests/reusex_unit_tests_vision "[rux_app]"
```

Expected: PASS, including `CliCommandRoundTrip_OptimizeFlagDefaults_MatchTheStageBase` and both `OptimizeMovedFlags_*` cases. `grep -rn "loop_edges_file\|panorama_loops\|\.loop_closure\b" libs apps --include=*.cpp --include=*.hpp | grep -v loop_candidates` lists nothing that still reads them from `PlaneGraphOptions`.

- [ ] **Step 8: Commit**

```bash
git add libs/reusex/include/slam/loop_candidates.hpp libs/reusex/src/slam/loop_candidates.cpp \
        libs/reusex/include/slam/PlaneGraphOptimizer.hpp libs/reusex/include/slam/LoopClosure.hpp \
        libs/reusex/include/slam/PanoramaLoopEdges.hpp libs/reusex/src/slam/optimize_sensor_poses.cpp \
        apps/rux/include/optimize.hpp apps/rux/src/optimize.cpp \
        tests/unit/slam/test_loop_candidates.cpp tests/unit/slam/test_plane_graph_optimizer.cpp \
        tests/unit/slam/test_seed_disagreement_gate.cpp tests/unit/rux_app/test_cli_command_roundtrip.cpp \
        tests/unit/rux_app/test_optimize_moved_flags.cpp
git commit -m "slam,rux: loop front-ends leave optimize; optimize refuses the moved flags

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_018dLKtE8qid8y5QVF9b2yri"
```

---

## Task 9: `PlaneGraphOptimizer::optimize_graph` — general edges with information

**Files:**
- Modify: `libs/reusex/include/slam/PlaneGraphOptimizer.hpp` (new `GraphEdge`, `PlaneGraphOptions::loop_default_sigma_*`, `PoseGraphEdgeRecord::tag`, `PlaneGraphResult` reporting fields, `optimize_graph`, `legacy_graph_edges`)
- Modify: `libs/reusex/src/slam/PlaneGraphOptimizer.cpp:403-405,671,700-860,984`
- Modify: `apps/rux/include/optimize.hpp`, `apps/rux/src/optimize.cpp` (mirror the two new options)
- Test: `tests/unit/slam/test_plane_graph_optimizer.cpp` (append)

**Interfaces:**
- Consumes: `core::pose_graph::information_from_sigmas` (tests only).
- Produces:

```cpp
namespace reusex::geometry {
struct GraphEdge {
  enum class Kind { odometry, loop };
  int i = -1; int j = -1;
  Eigen::Matrix4d T_ij = Eigen::Matrix4d::Identity();          // optical convention
  std::optional<Eigen::Matrix<double, 6, 6>> information;     // GTSAM order
  Kind kind = Kind::odometry;
  bool known_inlier = false;
  int tag = -1;
};
struct PoseGraphEdgeRecord { int i; int j; std::string type; double residual; double weight; int tag = -1; };
struct PlaneGraphResult { ...; int graph_edges = 0; int bridge_edges = 0; bool graph_initialised = false; int edges_with_metrics = 0; };
// PlaneGraphOptions gains:
float loop_default_sigma_rot = 0.05f;   // rad
float loop_default_sigma_trans = 0.10f; // m
PlaneGraphResult PlaneGraphOptimizer::optimize_graph(std::vector<FrameSurfels> &frames,
                                                     const std::vector<GraphEdge> &edges) const;
std::vector<GraphEdge> legacy_graph_edges(const std::vector<FrameSurfels> &frames,
                                          const std::vector<LoopEdge> &loop_edges);
}
```

Noise rules (the old behaviour is the `information == nullopt` odometry branch, unchanged):
- odometry, no information: `odometry_sigma_* × s × motion scale` (motion statistics over the no-information odometry edges only), `s = underconstrained_odom_scale` when either endpoint is under-constrained.
- odometry, information: `Λ / s²` (motion model not applied).
- loop, no information: `loop_default_sigma_*`.
- loop, information: `Λ`.
- Diagonal information → `Diagonal::Precisions`, otherwise `Gaussian::Information`.
- `known_inlier`: GNC may not demote it (goes into `trusted_factors`; no Huber under `--no-gnc`). Odometry follows `odometry_robust` unless `known_inlier`.
- `optimize(frames, loop_edges)` becomes `optimize_graph(frames, legacy_graph_edges(frames, loop_edges))` — consecutive seed odometry first, then the loop edges with `information_from_sigmas` — so every existing optimizer test is the regression test for the refactor.

- [ ] **Step 1: Write the failing tests**

Append to `tests/unit/slam/test_plane_graph_optimizer.cpp` (add `#include <reusex/core/pose_graph.hpp>` and `#include <catch2/catch_approx.hpp>`):

```cpp
TEST_CASE("PlaneGraphOptimizer_OptimizeGraphWithLegacyEdges_EqualsOptimize",
          "[plane_graph][optimize][stored_graph]") {
  auto a = drifted_corner_frames();
  auto b = drifted_corner_frames();
  const PlaneGraphOptions o = test_options();
  const auto ra = PlaneGraphOptimizer(o).optimize(a);
  const auto rb =
      PlaneGraphOptimizer(o).optimize_graph(b, legacy_graph_edges(b, {}));
  REQUIRE(ra.converged);
  REQUIRE(rb.converged);
  CHECK(ra.landmarks == rb.landmarks);
  CHECK(ra.edges.size() == rb.edges.size());
  for (size_t i = 0; i < a.size(); ++i)
    CHECK((a[i].world_pose.matrix() - b[i].world_pose.matrix()).norm() <
          1e-12);
}

TEST_CASE("PlaneGraphOptimizer_StoredOdometryMeasurement_OverridesTheSeed",
          "[plane_graph][optimize][stored_graph]") {
  // Frame 1's seed is drifted, but the stored odometry says the true relative
  // pose is the identity. With plane factors off and a tight explicit
  // information on that edge, the solve must follow the stored measurement —
  // the seed only initialises.
  std::vector<FrameSurfels> frames;
  for (int i = 0; i < 2; ++i)
    frames.push_back(make_corner_frame(i));
  frames[1].world_pose = drift(0.05, {1, 0, 0}, {0.05, 0.0, 0.0});

  PlaneGraphOptions o = test_options();
  o.use_plane_factors = false;
  o.use_gnc = false;
  GraphEdge e;
  e.i = 0;
  e.j = 1;
  e.T_ij = Eigen::Matrix4d::Identity();
  e.information = reusex::core::pose_graph::information_from_sigmas(1e-3, 1e-3);
  e.kind = GraphEdge::Kind::odometry;
  e.tag = 7;

  const auto res = PlaneGraphOptimizer(o).optimize_graph(frames, {e});
  REQUIRE(res.converged);
  CHECK(pose_trans_error(frames[1].world_pose, Eigen::Affine3d::Identity()) <
        1e-3);
  REQUIRE(res.edges.size() == 1);
  CHECK(res.edges[0].tag == 7);
  CHECK(res.edges[0].type == "odometry");
  CHECK(res.edges[0].weight == Catch::Approx(1e6));
}

TEST_CASE("PlaneGraphOptimizer_KnownInlierLoopEdge_SurvivesGnc",
          "[plane_graph][optimize][stored_graph]") {
  // The setup of CorrectLoopEdgeNoPlaneFactors, but WITH GNC: an ordinary
  // loop edge whose residual at the drifted seed looks like an outlier is
  // down-weighted; a user-confirmed (known-inlier) one is not.
  auto run = [](bool known) {
    std::vector<FrameSurfels> frames;
    for (int i = 0; i < 2; ++i)
      frames.push_back(make_corner_frame(i));
    frames[1].world_pose = drift(0.05, {1, 0, 0}, {0.05, 0.0, 0.0});
    PlaneGraphOptions o = test_options();
    o.min_landmark_observations = 10; // 0 landmarks
    o.use_gnc = true;
    auto edges = legacy_graph_edges(frames, {});
    GraphEdge loop;
    loop.i = 0;
    loop.j = 1;
    loop.T_ij = Eigen::Matrix4d::Identity();
    loop.information =
        reusex::core::pose_graph::information_from_sigmas(0.01, 0.01);
    loop.kind = GraphEdge::Kind::loop;
    loop.known_inlier = known;
    edges.push_back(loop);
    const auto res = PlaneGraphOptimizer(o).optimize_graph(frames, edges);
    REQUIRE(res.converged);
    REQUIRE(res.loop_edges == 1);
    return pose_trans_error(frames[1].world_pose, Eigen::Affine3d::Identity());
  };
  const double seed_err = 0.05;
  const double known = run(true);
  const double free = run(false);
  CHECK(known < 0.5 * seed_err);
  CHECK(known < free);
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `nix develop -c cmake --build build --target reusex_unit_tests -j$(nproc)`
Expected: compile FAIL — `'GraphEdge' was not declared`.

- [ ] **Step 3: Extend the header**

In `PlaneGraphOptimizer.hpp` add `#include <Eigen/Core>` and `#include <optional>`. After `loop_trust_inlier_cost` add:

```cpp
  /// Noise of a loop edge that carries no information matrix (a stored edge
  /// with NULL `information`, spec §3.1). Mirrors the LoopClosureOptions
  /// base sigmas.
  float loop_default_sigma_rot = 0.05f;   ///< rad
  float loop_default_sigma_trans = 0.10f; ///< m
```

Before `struct PoseGraphEdgeRecord` add:

```cpp
/// One edge of the (stored) pose graph handed to the solver, already in the
/// optimizer's OPTICAL convention (slam/pose_graph_conventions.hpp). Indices
/// are positions in the frames vector.
struct GraphEdge {
  enum class Kind { odometry, loop };
  int i = -1;
  int j = -1;
  /// pose(i)^-1 * pose(j), optical convention.
  Eigen::Matrix4d T_ij = Eigen::Matrix4d::Identity();
  /// 6x6, GTSAM order (rx, ry, rz, tx, ty, tz). nullopt: odometry uses
  /// odometry_sigma_* (x observability guard x motion model), loop uses
  /// loop_default_sigma_*.
  std::optional<Eigen::Matrix<double, 6, 6>> information;
  Kind kind = Kind::odometry;
  /// GNC may never demote this factor (a user-confirmed edge).
  bool known_inlier = false;
  /// Caller's tag, echoed in PoseGraphEdgeRecord::tag; -1 = none.
  int tag = -1;
};
```

In `PoseGraphEdgeRecord` append `int tag = -1; ///< GraphEdge::tag of the edge`. In `PlaneGraphResult` append:

```cpp
  /// Stored-graph bookkeeping (optimize_sensor_poses).
  int graph_edges = 0;  ///< stored edges that entered the solve
  int bridge_edges = 0; ///< connectivity bridges from the initial poses
  bool graph_initialised = false; ///< the run created the odometry edges
  int edges_with_metrics = 0;     ///< stored rows whose metrics were written
```

In `class PlaneGraphOptimizer` add after `optimize`:

```cpp
  /// Solve with an explicit edge list instead of consecutive-seed odometry:
  /// odometry AND loop factors all come from @p edges (out-of-range or
  /// self-loop edges are skipped). The gauge prior still pins frame 0 at its
  /// world_pose. optimize() == optimize_graph(frames,
  /// legacy_graph_edges(frames, loop_edges)).
  PlaneGraphResult optimize_graph(std::vector<FrameSurfels> &frames,
                                  const std::vector<GraphEdge> &edges) const;
```

and after the class:

```cpp
/// The graph optimize() has always built: odometry between consecutive frames
/// measured from their world_pose (no information: default sigmas), then each
/// loop edge with information_from_sigmas(sigma_rot, sigma_trans).
std::vector<GraphEdge> legacy_graph_edges(const std::vector<FrameSurfels> &frames,
                                          const std::vector<LoopEdge> &loop_edges);
```

- [ ] **Step 4: Implement in `PlaneGraphOptimizer.cpp`**

1. Add includes `#include "core/pose_graph.hpp"` and `#include <algorithm>`; in the anonymous namespace add:

```cpp
gtsam::Pose3 to_pose3(const Eigen::Matrix4d &T) {
  return gtsam::Pose3(gtsam::Rot3(T.block<3, 3>(0, 0)),
                      gtsam::Point3(T.block<3, 1>(0, 3)));
}

gtsam::SharedNoiseModel information_noise(const Eigen::Matrix<double, 6, 6> &I) {
  if (I.isDiagonal(1e-12))
    return gtsam::noiseModel::Diagonal::Precisions(gtsam::Vector(I.diagonal()));
  return gtsam::noiseModel::Gaussian::Information(I);
}
```

2. Rename the definition `PlaneGraphOptimizer::optimize(std::vector<FrameSurfels> &frames, const std::vector<LoopEdge> &loop_edges) const` (:403-405) to `PlaneGraphOptimizer::optimize_graph(std::vector<FrameSurfels> &frames, const std::vector<GraphEdge> &edges) const`. Right after `const int N = ...` and the `N < 2` check add:

```cpp
  auto in_range = [N](const GraphEdge &e) {
    return e.i >= 0 && e.j >= 0 && e.i < N && e.j < N && e.i != e.j;
  };
  const bool has_loop_edges =
      std::any_of(edges.begin(), edges.end(), [](const GraphEdge &e) {
        return e.kind == GraphEdge::Kind::loop;
      });
```

3. `if (kept == 0 && loop_edges.empty())` (:671) → `if (kept == 0 && !has_loop_edges)`.
4. `struct EdgeRec` gains `int tag;`.
5. Replace the whole odometry block (from `// Odometry from consecutive SEED poses` :750 to the closing `}` of that block :810) with:

```cpp
    // Odometry factors, one per odometry-kind edge. Edges without an
    // information matrix keep the historical weighting: odometry_sigma_*,
    // tightened on under-constrained frames, optionally motion-scaled. A
    // stored information is the measurement's own noise (only the
    // observability guard applies to it).
    {
      const double base_r = options_.odometry_sigma_rot;
      const double base_t = options_.odometry_sigma_trans;
      const double scale = options_.underconstrained_odom_scale;

      std::vector<int> motion_slot(edges.size(), -1);
      std::vector<OdometryEdgeMotion> motion;
      for (size_t k = 0; k < edges.size(); ++k) {
        const auto &e = edges[k];
        if (e.kind != GraphEdge::Kind::odometry || e.information ||
            !in_range(e))
          continue;
        const gtsam::Pose3 rel = to_pose3(e.T_ij);
        motion_slot[k] = static_cast<int>(motion.size());
        motion.push_back({rel.translation().norm(),
                          gtsam::Rot3::Logmap(rel.rotation()).norm()});
      }
      std::vector<OdometrySigmaScale> motion_scale;
      if (options_.odometry_noise_model == OdometryNoiseModel::motion &&
          !motion.empty()) {
        motion_scale = odometry_motion_scales(
            motion, static_cast<double>(options_.odometry_weight_min),
            static_cast<double>(options_.odometry_weight_max));
        std::vector<double> mt, mr;
        for (const auto &m : motion) {
          mt.push_back(m.translation);
          mr.push_back(m.rotation);
        }
        core::info("PlaneGraph: motion-scaled odometry noise over {} edges "
                   "(median motion {:.4f} m / {:.4f} rad)",
                   motion.size(), median_of(mt), median_of(mr));
      }

      for (size_t k = 0; k < edges.size(); ++k) {
        const auto &e = edges[k];
        if (e.kind != GraphEdge::Kind::odometry || !in_range(e))
          continue;
        const double s =
            (underconstrained[e.i] || underconstrained[e.j]) ? scale : 1.0;
        gtsam::SharedNoiseModel noise;
        double weight = 0.0;
        if (e.information) {
          const Eigen::Matrix<double, 6, 6> I = *e.information / (s * s);
          noise = information_noise(I);
          weight = I.diagonal().tail<3>().mean();
        } else {
          const int slot = motion_slot[k];
          const double mr =
              motion_scale.empty() ? 1.0 : motion_scale[slot].rotation;
          const double mt =
              motion_scale.empty() ? 1.0 : motion_scale[slot].translation;
          const double sr = base_r * s * mr;
          const double st = base_t * s * mt;
          noise = gtsam::noiseModel::Diagonal::Sigmas(
              (gtsam::Vector(6) << sr, sr, sr, st, st, st).finished());
          weight = 1.0 / (st * st);
        }
        graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
            X(e.i), X(e.j), to_pose3(e.T_ij), noise);
        // Odometry is a KNOWN INLIER by default: GNC may not demote it. With
        // --odometry-robust it is left out of the trusted set so GNC can
        // (the #225 "grossly-wrong seed edge" probe) — unless the user
        // confirmed the edge.
        if (options_.odometry_robust && !e.known_inlier)
          odom_factors.push_back(graph.size() - 1);
        else
          trusted_factors.push_back(graph.size() - 1);
        edge_recs.push_back(
            {e.i, e.j, "odometry", graph.size() - 1, weight, e.tag});
      }
    }
```

6. Replace the loop-edge `for` (:834-859, keep the long comment above it) with:

```cpp
    int loop_edge_count = 0;
    for (const auto &e : edges) {
      if (e.kind != GraphEdge::Kind::loop || !in_range(e))
        continue;
      gtsam::SharedNoiseModel base_noise;
      double weight = 0.0;
      if (e.information) {
        base_noise = information_noise(*e.information);
        weight = e.information->diagonal().tail<3>().mean();
      } else {
        const double sr = options_.loop_default_sigma_rot;
        const double st = options_.loop_default_sigma_trans;
        base_noise = gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(6) << sr, sr, sr, st, st, st).finished());
        weight = 1.0 / (st * st);
      }
      gtsam::SharedNoiseModel noise = base_noise;
      if (options_.loop_edges_trusted && !options_.use_gnc && !e.known_inlier)
        noise = gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Huber::Create(1.345), base_noise);
      graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
          X(e.i), X(e.j), to_pose3(e.T_ij), noise);
      if (e.known_inlier)
        trusted_factors.push_back(graph.size() - 1);
      else if (options_.loop_edges_trusted)
        loop_factors.push_back(graph.size() - 1);
      edge_recs.push_back(
          {e.i, e.j, "loop_closure", graph.size() - 1, weight, e.tag});
      ++loop_edge_count;
    }
    result.loop_edges = loop_edge_count;
```

7. `result.edges.push_back({rec.i, rec.j, rec.type, res, rec.weight});` (:984) → `result.edges.push_back({rec.i, rec.j, rec.type, res, rec.weight, rec.tag});`
8. After `optimize_graph`'s closing brace add:

```cpp
PlaneGraphResult
PlaneGraphOptimizer::optimize(std::vector<FrameSurfels> &frames,
                              const std::vector<LoopEdge> &loop_edges) const {
  return optimize_graph(frames, legacy_graph_edges(frames, loop_edges));
}

std::vector<GraphEdge> legacy_graph_edges(const std::vector<FrameSurfels> &frames,
                                          const std::vector<LoopEdge> &loop_edges) {
  std::vector<GraphEdge> edges;
  edges.reserve(frames.size() + loop_edges.size());
  for (size_t i = 0; i + 1 < frames.size(); ++i) {
    GraphEdge e;
    e.i = static_cast<int>(i);
    e.j = static_cast<int>(i + 1);
    e.T_ij = frames[i].world_pose.matrix().inverse() *
             frames[i + 1].world_pose.matrix();
    edges.push_back(e);
  }
  for (const auto &l : loop_edges) {
    GraphEdge e;
    e.i = l.i;
    e.j = l.j;
    e.T_ij = l.T_ij;
    e.information =
        core::pose_graph::information_from_sigmas(l.sigma_rot, l.sigma_trans);
    e.kind = GraphEdge::Kind::loop;
    edges.push_back(e);
  }
  return edges;
}
```

- [ ] **Step 5: Mirror the new options on the CLI (STANDARDS §4)**

`optimize.hpp`, next to `loop_trust_inlier_cost`:

```cpp
  float loop_default_sigma_rot =
      reusex::geometry::PlaneGraphOptions{}.loop_default_sigma_rot;
  float loop_default_sigma_trans =
      reusex::geometry::PlaneGraphOptions{}.loop_default_sigma_trans;
```

`optimize.cpp`, after `--loop-trust-inlier-cost`:

```cpp
  sub->add_option("--loop-default-sigma-rot", opt->loop_default_sigma_rot,
                  "Rotation std (rad) of a stored loop edge without an "
                  "information matrix")
      ->default_val(opt->loop_default_sigma_rot);
  sub->add_option("--loop-default-sigma-trans", opt->loop_default_sigma_trans,
                  "Translation std (m) of a stored loop edge without an "
                  "information matrix")
      ->default_val(opt->loop_default_sigma_trans);
```

and in `plane_graph_options()`:

```cpp
  options.loop_default_sigma_rot = opt.loop_default_sigma_rot;
  options.loop_default_sigma_trans = opt.loop_default_sigma_trans;
```

- [ ] **Step 6: Run tests to verify they pass**

```bash
nix develop -c cmake --build build --target reusex_unit_tests reusex_unit_tests_vision -j$(nproc)
nix develop -c build/tests/reusex_unit_tests "[plane_graph]"
nix develop -c build/tests/reusex_unit_tests_vision "[rux_app]"
```

Expected: PASS — every pre-existing `[plane_graph]` case (they now run through `optimize_graph`) plus the three new ones, and `OptimizeFlagDefaults_MatchTheStageBase`.

- [ ] **Step 7: Commit**

```bash
git add libs/reusex/include/slam/PlaneGraphOptimizer.hpp libs/reusex/src/slam/PlaneGraphOptimizer.cpp \
        apps/rux/include/optimize.hpp apps/rux/src/optimize.cpp tests/unit/slam/test_plane_graph_optimizer.cpp
git commit -m "slam: optimize_graph — stored edges with information, known inliers and tags

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_018dLKtE8qid8y5QVF9b2yri"
```

---

## Task 10: `optimize` solves the stored graph

**Files:**
- Modify (rewrite): `libs/reusex/src/slam/optimize_sensor_poses.cpp`
- Modify: `libs/reusex/include/slam/PlaneGraphOptimizer.hpp:513-530` (doc of `optimize_sensor_poses`)
- Modify: `libs/reusex/include/pipeline/optimize_stage.hpp:63-67` (artifact)
- Modify: `libs/reusex/include/core/ProjectDB.hpp`, `libs/reusex/src/core/ProjectDB.cpp` (remove `save_pose_graph_edges` / `savePoseGraphEdges`)
- Modify: `apps/rux/qt/gallery/shell_pages.cpp:314-328`
- Modify: `apps/rux/src/optimize.cpp` (report line)
- Modify: `tests/unit/core/test_project_db_pose_graph.cpp` (drop the two `save_pose_graph_edges` cases)
- Create: `tests/unit/slam/test_optimize_stored_graph.cpp`

**Interfaces:**
- Consumes: Tasks 2–9 (`list_pose_graph_edges`, `initial_pose`, `update_pose_graph_edge_metrics`, `slam::init_pose_graph`, `slam::odometry_init_edges`, `slam::count_usable_odometry_edges`, `slam::stored_to_optical`, `slam::information_stored_to_optical`, `optimize_graph`, `GraphEdge`, `core::pose_graph::transform_error`).
- Produces: `geometry::optimize_sensor_poses(ProjectDB&, const PlaneGraphOptions&, bool dry_run)` — same signature, new semantics (spec §4.3 steps 1–4); `PlaneGraphResult::{graph_edges, bridge_edges, graph_initialised, edges_with_metrics}` filled.

- [ ] **Step 1: Write the failing tests**

`tests/unit/slam/test_optimize_stored_graph.cpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// `rux optimize` on the stored pose graph (spec 2026-10-13 §4.3): seeds from
// initial_transform, never replaces edges, writes per-edge metrics, and is
// repeatable. Frames look at one wall 1.5 m away so they yield surfels and a
// plane landmark.

#include <catch2/catch_test_macros.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/logging.hpp>
#include <reusex/core/pose_graph.hpp>
#include <reusex/slam/PlaneGraphOptimizer.hpp>
#include <reusex/slam/pose_graph.hpp>

#include "../../support/pose_fixture.hpp"
#include "../../support/temp_path.hpp"

#include <cmath>
#include <string>
#include <vector>

using reusex::ProjectDB;
using namespace reusex::test_support;
namespace pg = reusex::core::pose_graph;
namespace geo = reusex::geometry;
namespace slam = reusex::slam;

namespace {

constexpr int kW = 64;
constexpr int kH = 48;
constexpr int kFrames = 5;

void seed_wall_frame(ProjectDB &db, int id, const std::array<double, 16> &pose) {
  db.save_sensor_frame(id, make_color(kW, kH), make_depth(kW, kH, 1.5),
                       cv::Mat(), pose,
                       make_intrinsics(64.0, 64.0, kW / 2.0, kH / 2.0, kW, kH),
                       static_cast<double>(id));
}

std::array<double, 16> truth(int k) { return translation_pose(0.1 * k, 0, 0); }

geo::PlaneGraphOptions wall_options() {
  geo::PlaneGraphOptions o;
  o.surfel.min_distance = 0.0f;
  o.surfel.max_distance = 4.0f;
  o.surfel.sampling_factor = 2;
  o.surfel.confidence_threshold = 2;
  o.surfel.normal_radius = 0.1f;
  o.surfel.voxel_size = 0.0f;
  o.surfel.apply_depth_filters = false;
  o.min_plane_inliers = 50;
  o.min_landmark_observations = 2;
  o.min_landmark_spread_ratio = 0.0f;
  o.assoc_overlap_margin = 2.0f;
  o.assoc_rounds = 1;
  return o;
}

void make_project(ProjectDB &db) {
  for (int k = 0; k < kFrames; ++k)
    seed_wall_frame(db, k, truth(k));
}

double dist(const std::array<double, 16> &a, const std::array<double, 16> &b) {
  return (pg::matrix_from_row_major(a) - pg::matrix_from_row_major(b)).norm();
}

/// Captures `warn` lines from the library for the duration of a test.
class LogCapture {
    public:
  LogCapture() : previous_(reusex::core::get_log_level()) {
    reusex::core::set_log_level(reusex::core::LogLevel::info);
    reusex::core::set_log_handler(
        [this](reusex::core::LogLevel level, std::string_view message) {
          if (level == reusex::core::LogLevel::warn)
            warnings_.emplace_back(message);
        });
  }
  ~LogCapture() {
    reusex::core::reset_log_handler();
    reusex::core::set_log_level(previous_);
  }
  bool saw(std::string_view needle) const {
    for (const auto &w : warnings_)
      if (w.find(needle) != std::string::npos)
        return true;
    return false;
  }

    private:
  reusex::core::LogLevel previous_;
  std::vector<std::string> warnings_;
};

} // namespace

TEST_CASE("OptimizeStoredGraph_SeedsFromInitialPoses_NotFromTransform",
          "[slam][optimize][stored_graph]") {
  TempPath tmp("test_optimize_stored_seed");
  ProjectDB db(tmp.path);
  make_project(db);
  slam::init_pose_graph(db); // initial = truth, odometry from truth
  for (int k = 0; k < kFrames; ++k)
    db.update_sensor_frame_pose(k, translation_pose(5.0 + k, 3.0, 0.0));

  auto o = wall_options();
  o.use_plane_factors = false; // odometry + gauge: the solve IS the seed
  const auto r = geo::optimize_sensor_poses(db, o);
  REQUIRE(r.converged);
  for (int k = 0; k < kFrames; ++k) {
    INFO("frame " << k);
    CHECK(dist(db.sensor_frame_pose(k), truth(k)) < 1e-6);
    CHECK(*db.initial_pose(k) == truth(k)); // never written by optimize
  }
}


TEST_CASE("OptimizeStoredGraph_RunTwice_IsRepeatable",
          "[slam][optimize][stored_graph]") {
  TempPath tmp("test_optimize_stored_repeat");
  ProjectDB db(tmp.path);
  make_project(db);
  slam::init_pose_graph(db); // odometry measured on the true trajectory
  // Seed frame 3 off the wall so the solve has work to do.
  db.set_initial_pose(3, translation_pose(0.3, 0.0, 0.05));
  const auto o = wall_options();

  const auto r1 = geo::optimize_sensor_poses(db, o);
  REQUIRE(r1.converged);
  std::vector<std::array<double, 16>> first;
  for (int k = 0; k < kFrames; ++k)
    first.push_back(db.sensor_frame_pose(k));
  const auto edges1 = db.list_pose_graph_edges();

  const auto r2 = geo::optimize_sensor_poses(db, o);
  REQUIRE(r2.converged);
  for (int k = 0; k < kFrames; ++k) {
    INFO("frame " << k);
    CHECK(dist(db.sensor_frame_pose(k), first[static_cast<size_t>(k)]) < 1e-9);
  }
  const auto edges2 = db.list_pose_graph_edges();
  REQUIRE(edges2.size() == edges1.size());
  for (size_t i = 0; i < edges2.size(); ++i) {
    CHECK(edges2[i].id == edges1[i].id);
    CHECK(edges2[i].status == edges1[i].status);
    CHECK(edges2[i].transform == edges1[i].transform);
    CHECK(std::isfinite(edges2[i].residual));
    CHECK(edges2[i].deviation_m.has_value());
    CHECK(edges2[i].seed_disagreement_m.has_value());
  }
  CHECK(r1.edges_with_metrics == kFrames - 1);
}

TEST_CASE("OptimizeStoredGraph_NeverRemovesEdgesOrChangesStatus",
          "[slam][optimize][stored_graph]") {
  LogCapture log;
  TempPath tmp("test_optimize_stored_keep");
  ProjectDB db(tmp.path);
  make_project(db);
  slam::init_pose_graph(db);
  auto loop = [&](int a, int b, const char *status, bool measured) {
    ProjectDB::PoseGraphEdge e;
    e.from_node_id = a;
    e.to_node_id = b;
    e.edge_type = "loop_closure";
    if (measured)
      e.transform = pg::row_major_from_matrix(
          pg::matrix_from_row_major(truth(a)).inverse() *
          pg::matrix_from_row_major(truth(b)));
    e.source = measured ? "manual" : "legacy";
    e.status = status;
    REQUIRE(db.upsert_pose_graph_edge(e));
  };
  loop(0, 4, "confirmed", true);
  loop(1, 3, "auto", true);
  REQUIRE(db.set_pose_graph_edge_status(1, 3, "loop_closure", "rejected"));
  loop(0, 2, "auto", false);

  auto o = wall_options();
  o.use_plane_factors = false;
  const auto r = geo::optimize_sensor_poses(db, o);
  REQUIRE(r.converged);
  CHECK(r.graph_edges == kFrames); // 4 odometry + the confirmed loop
  CHECK(db.list_pose_graph_edges().size() == 7);

  const auto confirmed = db.pose_graph_edge(0, 4, "loop_closure");
  const auto rejected = db.pose_graph_edge(1, 3, "loop_closure");
  const auto legacy = db.pose_graph_edge(0, 2, "loop_closure");
  CHECK(confirmed->status == "confirmed");
  CHECK(std::isfinite(confirmed->residual));
  CHECK(rejected->status == "rejected");
  CHECK(std::isnan(rejected->residual));
  CHECK(rejected->deviation_m.has_value()); // still measured against the solve
  CHECK(legacy->status == "auto");
  CHECK(std::isnan(legacy->residual));
  CHECK_FALSE(legacy->deviation_m.has_value());
  CHECK(log.saw("carry no measurement"));
}

TEST_CASE("OptimizeStoredGraph_RejectedOdometry_IsBridgedAndWarned",
          "[slam][optimize][stored_graph]") {
  // Review focus 3.
  LogCapture log;
  TempPath tmp("test_optimize_stored_bridge");
  ProjectDB db(tmp.path);
  make_project(db);
  slam::init_pose_graph(db);
  REQUIRE(db.set_pose_graph_edge_status(2, 3, "odometry", "rejected"));

  auto o = wall_options();
  o.use_plane_factors = false;
  const auto r = geo::optimize_sensor_poses(db, o);
  REQUIRE(r.converged);
  CHECK(r.bridge_edges == 1);
  CHECK(log.saw("bridged"));
  CHECK(db.pose_graph_edge(2, 3, "odometry")->status == "rejected");
  for (int k = 0; k < kFrames; ++k)
    CHECK(dist(db.sensor_frame_pose(k), truth(k)) < 1e-6);
}

TEST_CASE("OptimizeStoredGraph_RegisterAfterInit_WarnsToReinit",
          "[slam][optimize][stored_graph]") {
  // Review focus 5.
  TempPath tmp("test_optimize_stored_register");
  ProjectDB db(tmp.path);
  make_project(db);
  slam::init_pose_graph(db);
  db.log_pipeline_end(db.log_pipeline_start("graph_init", "{}"), true);
  auto o = wall_options();
  o.use_plane_factors = false;

  SECTION("a register run newer than the init warns") {
    db.log_pipeline_end(db.log_pipeline_start("pose_refinement_jpr", "{}"),
                        true);
    LogCapture log;
    REQUIRE(geo::optimize_sensor_poses(db, o).converged);
    CHECK(log.saw("'rux register' ran after"));
  }
  SECTION("no register run, no warning") {
    LogCapture log;
    REQUIRE(geo::optimize_sensor_poses(db, o).converged);
    CHECK_FALSE(log.saw("'rux register' ran after"));
  }
}

TEST_CASE("OptimizeStoredGraph_NoGraph_AutoInitialisesButDryRunWritesNothing",
          "[slam][optimize][stored_graph]") {
  TempPath tmp("test_optimize_stored_autoinit");
  ProjectDB db(tmp.path);
  make_project(db);
  auto o = wall_options();
  o.use_plane_factors = false;

  const auto dry = geo::optimize_sensor_poses(db, o, /*dry_run=*/true);
  CHECK(dry.converged);
  CHECK(db.list_pose_graph_edges().empty());
  CHECK_FALSE(db.has_initial_pose(0));

  const auto r = geo::optimize_sensor_poses(db, o);
  REQUIRE(r.converged);
  CHECK(r.graph_initialised);
  const auto edges = db.list_pose_graph_edges();
  CHECK(slam::count_usable_odometry_edges(edges) == kFrames - 1);
  for (const auto &e : edges)
    CHECK(e.source == "odometry_init");
  CHECK(db.has_initial_pose(0));
}

TEST_CASE("OptimizeStoredGraph_LegacyOdometryRows_StillAutoInitialise",
          "[slam][optimize][stored_graph]") {
  // Review focus 4: a project carrying rows from a pre-v27 optimize.
  TempPath tmp("test_optimize_stored_legacy");
  ProjectDB db(tmp.path);
  make_project(db);
  for (int k = 0; k < 2; ++k) {
    ProjectDB::PoseGraphEdge legacy{k, k + 1, "odometry", 0.1, 100.0};
    legacy.source = "legacy";
    REQUIRE(db.upsert_pose_graph_edge(legacy));
  }
  auto o = wall_options();
  o.use_plane_factors = false;
  const auto r = geo::optimize_sensor_poses(db, o);
  REQUIRE(r.converged);
  CHECK(r.graph_initialised);
  const auto e = db.pose_graph_edge(0, 1, "odometry");
  CHECK(e->source == "odometry_init");
  CHECK(e->transform.has_value());
}
```

If every case fails at `REQUIRE(r.converged)` with "no planes detected" in the log, the fixture yields too few surfels: lower `o.min_plane_inliers` in `wall_options()` to 30 and re-run before touching the implementation.

- [ ] **Step 2: Run test to verify it fails**

Run: `nix develop -c cmake --build build --target reusex_unit_tests -j$(nproc) && nix develop -c build/tests/reusex_unit_tests "[stored_graph]"`
Expected: FAIL — e.g. `SeedsFromInitialPoses` finds the poses at the garbage transforms (the old code seeds from `transform`), `edges_with_metrics` is 0.

- [ ] **Step 3: Rewrite `optimize_sensor_poses.cpp`**

Replace the whole file with:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Database glue for the plane-landmark pose-graph back-end: solve the STORED
// pose graph (docs/superpowers/specs/2026-10-13-pose-graph-editor-design.md
// §4.3). Seeds come from sensor_frames.initial_transform, so re-running gives
// the same answer; every active stored edge (auto + confirmed, with a
// measurement) becomes one BetweenFactor; poses are written to `transform`
// and per-edge metrics back to pose_graph_edges. The edge set itself is never
// replaced. Kept separate from PlaneGraphOptimizer.cpp so the solver core
// stays free of any database dependency.

#include "core/ProjectDB.hpp"
#include "core/SensorIntrinsics.hpp"
#include "core/logging.hpp"
#include "core/pose_graph.hpp"
#include "slam/PlaneGraphOptimizer.hpp"
#include "slam/pose_graph.hpp"
#include "slam/pose_graph_conventions.hpp"

#include <Eigen/Geometry>

#include <fmt/format.h>

#include <algorithm>
#include <initializer_list>
#include <map>
#include <numeric>
#include <stdexcept>
#include <string_view>
#include <utility>
#include <vector>

namespace reusex::geometry {
namespace {

namespace pg = core::pose_graph;

/// Union-find over frame indices, for the connectivity bridge.
class Components {
    public:
  explicit Components(int n) : parent_(static_cast<size_t>(n)) {
    std::iota(parent_.begin(), parent_.end(), 0);
  }
  int find(int x) {
    while (parent_[static_cast<size_t>(x)] != x) {
      parent_[static_cast<size_t>(x)] =
          parent_[static_cast<size_t>(parent_[static_cast<size_t>(x)])];
      x = parent_[static_cast<size_t>(x)];
    }
    return x;
  }
  void unite(int a, int b) {
    a = find(a);
    b = find(b);
    if (a != b)
      parent_[static_cast<size_t>(std::max(a, b))] = std::min(a, b);
  }

    private:
  std::vector<int> parent_;
};

/// Newest successful pipeline_log id among @p stages, 0 when none.
int newest_success(const std::vector<ProjectDB::PipelineLogEntry> &log,
                   std::initializer_list<std::string_view> stages) {
  int newest = 0;
  for (const auto &e : log)
    if (e.status == "success" &&
        std::find(stages.begin(), stages.end(), e.stage) != stages.end())
      newest = std::max(newest, e.id);
  return newest;
}

} // namespace

PlaneGraphResult optimize_sensor_poses(ProjectDB &db,
                                       const PlaneGraphOptions &options,
                                       bool dry_run) {
  // --- 0. The stored graph; initialise it when it has no usable odometry --
  auto stored = db.list_pose_graph_edges();
  bool initialised = false;
  if (slam::count_usable_odometry_edges(stored) == 0) {
    if (dry_run) {
      auto planned = slam::odometry_init_edges(db, /*from_current=*/false);
      core::info("PlaneGraph dry-run: the stored pose graph has no odometry "
                 "edges; using {} derived from the current poses in memory "
                 "(nothing written)",
                 planned.size());
      stored.insert(stored.end(), planned.begin(), planned.end());
    } else {
      const auto init = slam::init_pose_graph(db);
      core::info("PlaneGraph: the stored pose graph had no odometry edges; "
                 "initialised {} from the current poses and snapshotted {} "
                 "initial poses (as 'rux graph init')",
                 init.odometry_edges_written, init.snapshots_written);
      stored = db.list_pose_graph_edges();
      initialised = true;
    }
  }

  // Review focus 5: a pose refiner that ran after the graph was set up.
  if (!initialised) {
    const auto log = db.pipeline_log();
    if (newest_success(log, {"pose_refinement_jpr"}) >
        newest_success(log, {"import", "graph_init",
                             "pose_optimization_plane_graph"}))
      core::warn("PlaneGraph: 'rux register' ran after the pose graph was "
                 "initialised; optimize seeds from the stored initial poses "
                 "and ignores its refinement. Run 'rux graph init --force' "
                 "first to optimise on top of it.");
  }

  // --- 1. Frames: surfels (optical frame, pose-independent) + seeds -------
  auto frameIds = db.sensor_frame_ids();
  std::sort(frameIds.begin(), frameIds.end()); // temporal ordering

  std::vector<FrameSurfels> frames;
  std::vector<Eigen::Matrix4d> local;       // parallel to frames
  std::vector<Eigen::Matrix4d> seed_stored; // initial pose, worldTf convention
  int fallback = 0;
  for (int id : frameIds) {
    auto fs = extract_frame_surfels(db, id, options.surfel);
    if (!fs)
      continue;
    std::array<double, 16> seed{};
    if (auto p = db.initial_pose(id)) {
      seed = *p;
    } else {
      seed = db.sensor_frame_pose(id);
      ++fallback;
    }
    const Eigen::Matrix4d L =
        pg::matrix_from_row_major(db.sensor_frame_intrinsics(id).local_transform);
    const Eigen::Matrix4d S = pg::matrix_from_row_major(seed);
    fs->world_pose.matrix() = S * L; // optical -> world, as extraction does
    local.push_back(L);
    seed_stored.push_back(S);
    frames.push_back(std::move(*fs));
  }

  if (frames.size() < frameIds.size())
    core::warn("PlaneGraph: {} of {} sensor frames yielded no surfels "
               "(missing colour/depth, invalid intrinsics, no usable stored "
               "pose, or too few points); optimizing the remaining {}",
               frameIds.size() - frames.size(), frameIds.size(), frames.size());
  if (fallback > 0)
    core::warn("PlaneGraph: {} of {} frames have no initial pose "
               "(sensor_frames.initial_transform); seeded from their current "
               "transform. Run 'rux graph init --force' to snapshot them.",
               fallback, frames.size());
  if (frames.size() < 2)
    throw std::runtime_error(fmt::format(
        "PlaneGraph: fewer than 2 usable sensor frames with depth/pose ({} of "
        "{} frames yielded surfels)",
        frames.size(), frameIds.size()));

  // --- 2. Active stored edges -> solver edges (optical convention) --------
  std::map<int, int> index_of;
  for (size_t k = 0; k < frames.size(); ++k)
    index_of[frames[k].node_id] = static_cast<int>(k);

  std::vector<GraphEdge> edges;
  int rejected = 0, no_measurement = 0, missing_frame = 0;
  for (size_t k = 0; k < stored.size(); ++k) {
    const auto &s = stored[k];
    if (s.status == pg::kStatusRejected) {
      ++rejected;
      continue;
    }
    if (!s.transform) {
      ++no_measurement;
      continue;
    }
    const auto a = index_of.find(s.from_node_id);
    const auto b = index_of.find(s.to_node_id);
    if (a == index_of.end() || b == index_of.end()) {
      ++missing_frame;
      continue;
    }
    GraphEdge e;
    e.i = a->second;
    e.j = b->second;
    e.T_ij = slam::stored_to_optical(pg::matrix_from_row_major(*s.transform),
                                     local[static_cast<size_t>(e.i)],
                                     local[static_cast<size_t>(e.j)]);
    if (s.information)
      e.information = slam::information_stored_to_optical(
          pg::information_from_row_major(*s.information),
          local[static_cast<size_t>(e.j)]);
    e.kind = s.edge_type == pg::kTypeOdometry ? GraphEdge::Kind::odometry
                                              : GraphEdge::Kind::loop;
    e.known_inlier =
        e.kind == GraphEdge::Kind::loop && s.status == pg::kStatusConfirmed;
    e.tag = static_cast<int>(k);
    edges.push_back(std::move(e));
  }
  if (no_measurement > 0)
    core::warn("PlaneGraph: {} stored edges carry no measurement (pre-v27 "
               "'legacy' rows) and were skipped; re-create them with 'rux "
               "graph discover' or 'rux graph init --force'",
               no_measurement);
  if (missing_frame > 0)
    core::warn("PlaneGraph: {} stored edges reference a frame that yielded "
               "no surfels or has no pose and were skipped",
               missing_frame);
  if (rejected > 0)
    core::info("PlaneGraph: {} rejected edges left out of the solve",
               rejected);
  const int graph_edges = static_cast<int>(edges.size());

  // --- 3. Connectivity bridge (spec review focus 3) ------------------------
  Components comp(static_cast<int>(frames.size()));
  for (const auto &e : edges)
    comp.unite(e.i, e.j);
  int bridged = 0;
  for (int k = 0; k + 1 < static_cast<int>(frames.size()); ++k) {
    if (comp.find(k) == comp.find(k + 1))
      continue;
    GraphEdge e;
    e.i = k;
    e.j = k + 1;
    e.T_ij = frames[static_cast<size_t>(k)].world_pose.matrix().inverse() *
             frames[static_cast<size_t>(k + 1)].world_pose.matrix();
    edges.push_back(e);
    comp.unite(k, k + 1);
    ++bridged;
  }
  if (bridged > 0)
    core::warn("PlaneGraph: the stored graph splits into {} disconnected "
               "parts over the usable frames (frames without surfels, "
               "rejected odometry, or scan boundaries); bridged them with {} "
               "odometry edges from the initial poses (not stored)",
               bridged + 1, bridged);

  // --- 4. Solve -------------------------------------------------------------
  PlaneGraphOptimizer optimizer(options);
  PlaneGraphResult result = optimizer.optimize_graph(frames, edges);
  result.graph_edges = graph_edges;
  result.bridge_edges = bridged;
  result.graph_initialised = initialised;

  if (dry_run) {
    core::info("PlaneGraph dry-run: poses NOT written back");
    return result;
  }
  if (result.landmarks == 0 && result.loop_edges == 0) {
    core::warn(
        "PlaneGraph: no landmarks or loop edges — poses NOT written back");
    return result;
  }
  if (!result.converged) {
    core::error("PlaneGraph: optimization did not converge — poses NOT "
                "written back (docs/STANDARDS.md §5: no silent failure)");
    return result;
  }

  // --- 5. Poses (transform only) and per-edge metrics, one transaction ----
  // worldTf = world_pose * localTf^-1 (the stored `transform` convention).
  std::vector<Eigen::Matrix4d> solved(frames.size());
  for (size_t k = 0; k < frames.size(); ++k)
    solved[k] = frames[k].world_pose.matrix() * local[k].inverse();

  std::map<int, double> residual_of; // stored index -> residual
  for (const auto &rec : result.edges)
    if (rec.tag >= 0)
      residual_of[rec.tag] = rec.residual;

  std::vector<ProjectDB::PoseGraphEdgeMetrics> metrics;
  metrics.reserve(stored.size());
  for (size_t k = 0; k < stored.size(); ++k) {
    const auto &s = stored[k];
    ProjectDB::PoseGraphEdgeMetrics m;
    m.from_node_id = s.from_node_id;
    m.to_node_id = s.to_node_id;
    m.edge_type = s.edge_type;
    if (auto it = residual_of.find(static_cast<int>(k));
        it != residual_of.end())
      m.residual = it->second;
    const auto a = index_of.find(s.from_node_id);
    const auto b = index_of.find(s.to_node_id);
    if (s.transform && a != index_of.end() && b != index_of.end()) {
      const Eigen::Matrix4d T = pg::matrix_from_row_major(*s.transform);
      const auto i = static_cast<size_t>(a->second);
      const auto j = static_cast<size_t>(b->second);
      const auto dev = pg::transform_error(T, solved[i], solved[j]);
      const auto seed = pg::transform_error(T, seed_stored[i], seed_stored[j]);
      m.deviation_m = dev.translation_m;
      m.deviation_deg = dev.rotation_deg;
      m.seed_disagreement_m = seed.translation_m;
      m.seed_disagreement_deg = seed.rotation_deg;
    }
    metrics.push_back(std::move(m));
  }

  ProjectDB::Transaction tx(db);
  for (size_t k = 0; k < frames.size(); ++k)
    db.update_sensor_frame_pose(frames[k].node_id,
                                pg::row_major_from_matrix(solved[k]));
  result.edges_with_metrics = db.update_pose_graph_edge_metrics(metrics);
  tx.commit();
  core::info("PlaneGraph: wrote {} optimized poses and metrics for {} stored "
             "pose-graph edges",
             frames.size(), result.edges_with_metrics);
  return result;
}

} // namespace reusex::geometry
```

- [ ] **Step 4: Update the declaration's doc, the stage, the gallery and remove `save_pose_graph_edges`**

1. `PlaneGraphOptimizer.hpp` — replace the doc comment of `optimize_sensor_poses` (:513-526) with:

```cpp
/// High-level entry point: solve the STORED pose graph of @p db
/// (pose_graph_edges, schema v27). Seeds every frame from
/// sensor_frames.initial_transform (falling back to `transform` with a
/// warning), adds one factor per active stored edge (auto + confirmed, with a
/// measurement; confirmed loop edges are GNC known inliers), bridges a
/// disconnected graph from the initial poses, and — unless @p dry_run —
/// writes the optimized world poses to `transform` (never to
/// initial_transform) and residual / deviation / seed-disagreement back to
/// every stored edge. The edge set is never replaced. When the graph has no
/// usable odometry, slam::init_pose_graph() runs first (in memory only under
/// @p dry_run).
```

2. `pipeline/optimize_stage.hpp` — replace the final success outputs line (:66-67) with:

```cpp
        // `sensor_frames` holds the optimized poses; `pose_graph_edges` the
        // per-edge metrics (and the odometry, when this run initialised it).
        {{"table", "sensor_frames", static_cast<int64_t>(result.frames)},
         {"table", "pose_graph_edges",
          static_cast<int64_t>(result.edges_with_metrics)}});
```

3. `ProjectDB.hpp`/`.cpp`: delete the `save_pose_graph_edges` declaration (Task 2's transitional one), `Impl::savePoseGraphEdges`, and `ProjectDB::save_pose_graph_edges`. In `tests/unit/core/test_project_db_pose_graph.cpp` delete the two cases `pose_graph round trip: save and list` and `pose_graph save replaces previous run atomically`, and update the file's header comment list to "1. migration; 2. upsert/status/metrics API".
4. `apps/rux/qt/gallery/shell_pages.cpp:328` — replace `db.save_pose_graph_edges(edges);` with:

```cpp
    reusex::ProjectDB::Transaction tx(db);
    for (auto &e : edges) {
      e.source = "manual";
      db.upsert_pose_graph_edge(e, /*overwrite_rejected=*/true);
    }
    tx.commit();
```

5. `apps/rux/src/optimize.cpp`, after the "Factor-graph error" `spdlog::info`:

```cpp
    spdlog::info("Pose graph: {} stored edges in the solve{}{}",
                 result.graph_edges,
                 result.bridge_edges > 0
                     ? fmt::format(", {} bridging edges from the initial "
                                   "poses",
                                   result.bridge_edges)
                     : std::string(),
                 result.graph_initialised
                     ? " (odometry initialised from the current poses)"
                     : "");
```

- [ ] **Step 5: Run tests to verify they pass**

```bash
nix develop -c cmake --build build -j$(nproc)
nix develop -c build/tests/reusex_unit_tests "[stored_graph],[pose_graph],[optimize_stage],[plane_graph]"
```

Expected: PASS. `grep -rn save_pose_graph_edges libs apps tests bindings` prints nothing.

- [ ] **Step 6: Commit**

```bash
git add libs/reusex/src/slam/optimize_sensor_poses.cpp libs/reusex/include/slam/PlaneGraphOptimizer.hpp \
        libs/reusex/include/pipeline/optimize_stage.hpp libs/reusex/include/core/ProjectDB.hpp \
        libs/reusex/src/core/ProjectDB.cpp apps/rux/qt/gallery/shell_pages.cpp apps/rux/src/optimize.cpp \
        tests/unit/slam/test_optimize_stored_graph.cpp tests/unit/core/test_project_db_pose_graph.cpp
git commit -m "slam: optimize solves the stored pose graph — repeatable, never replaces edges

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_018dLKtE8qid8y5QVF9b2yri"
```

---

## Task 11: `rux edit perturb-poses` keeps the drift benchmark honest

**Files:**
- Modify: `libs/reusex/src/slam/perturb_poses.cpp:297-303`
- Modify: `libs/reusex/include/slam/perturb_poses.hpp` (doc of `perturb_sensor_poses`, :186-196)
- Test: `tests/unit/slam/test_perturb_poses.cpp` (append)

**Interfaces:**
- Consumes: `slam::init_pose_graph` (Task 7), `ProjectDB::set_initial_pose` (Task 4).
- Produces: `perturb_sensor_poses` additionally writes `initial_transform` (= the drifted pose) and, when `odometry_init` edges exist, re-derives them from the drifted poses.

Why: the drift benchmarks (`scripts/bench-arkitscenes-drift.sh`, `bench-office.sh`) perturb a project and then run `optimize`. Since optimize now seeds from `initial_transform`, a perturbation that only wrote `transform` would be invisible to it.

- [ ] **Step 1: Write the failing test**

Append to `tests/unit/slam/test_perturb_poses.cpp` (add includes `<reusex/core/ProjectDB.hpp>`, `<reusex/core/pose_graph.hpp>`, `<reusex/slam/pose_graph.hpp>`, `"../../support/pose_fixture.hpp"`, `"../../support/temp_path.hpp"`):

```cpp
TEST_CASE("perturb_sensor_poses_WithStoredGraph_ResnapshotsAndRederives",
          "[slam][drift][pose_graph]") {
  namespace pg = reusex::core::pose_graph;
  reusex::test_support::TempPath tmp("test_perturb_graph");
  reusex::ProjectDB db(tmp.path);
  const auto traj = room_trajectory(60);
  for (int k = 0; k < 60; ++k)
    db.save_sensor_frame(k + 1, reusex::test_support::make_color(8, 8),
                         cv::Mat(), cv::Mat(),
                         pg::row_major_from_matrix(traj[static_cast<size_t>(k)]),
                         reusex::test_support::make_intrinsics(8, 8, 4, 4, 8, 8),
                         k);
  reusex::slam::init_pose_graph(db);

  reusex::geometry::perturb_sensor_poses(db, PoseDriftOptions{},
                                         /*dry_run=*/false);

  bool any_changed = false;
  for (int k = 1; k <= 60; ++k) {
    REQUIRE(db.initial_pose(k).has_value());
    CHECK(*db.initial_pose(k) == db.sensor_frame_pose(k));
  }
  for (int k = 1; k < 60; ++k) {
    const auto e = db.pose_graph_edge(k, k + 1, "odometry");
    REQUIRE(e.has_value());
    const Eigen::Matrix4d drifted =
        pg::matrix_from_row_major(db.sensor_frame_pose(k)).inverse() *
        pg::matrix_from_row_major(db.sensor_frame_pose(k + 1));
    CHECK((pg::matrix_from_row_major(*e->transform) - drifted).norm() < 1e-9);
    const Eigen::Matrix4d original =
        traj[static_cast<size_t>(k - 1)].inverse() * traj[static_cast<size_t>(k)];
    any_changed = any_changed || (drifted - original).norm() > 1e-6;
  }
  CHECK(any_changed);
}

TEST_CASE("perturb_sensor_poses_WithoutGraph_OnlySnapshots",
          "[slam][drift][pose_graph]") {
  reusex::test_support::TempPath tmp("test_perturb_nograph");
  reusex::ProjectDB db(tmp.path);
  const auto traj = room_trajectory(60);
  for (int k = 0; k < 60; ++k)
    db.save_sensor_frame(
        k + 1, reusex::test_support::make_color(8, 8), cv::Mat(), cv::Mat(),
        reusex::core::pose_graph::row_major_from_matrix(
            traj[static_cast<size_t>(k)]),
        reusex::test_support::make_intrinsics(8, 8, 4, 4, 8, 8), k);
  reusex::geometry::perturb_sensor_poses(db, PoseDriftOptions{}, false);
  CHECK(db.list_pose_graph_edges().empty());
  for (int k = 1; k <= 60; ++k)
    CHECK(*db.initial_pose(k) == db.sensor_frame_pose(k));
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `nix develop -c cmake --build build --target reusex_unit_tests -j$(nproc) && nix develop -c build/tests/reusex_unit_tests "[drift][pose_graph]"`
Expected: FAIL — `initial_pose(k)` is empty / differs from the drifted pose.

- [ ] **Step 3: Implement**

In `perturb_poses.cpp` add includes `"core/pose_graph.hpp"` and `"slam/pose_graph.hpp"`, and replace the write-back block (:297-302) with:

```cpp
  for (size_t k = 0; k < posed_ids.size(); ++k)
    db.update_sensor_frame_pose(posed_ids[k], to_array16_double(poses[k]));
  core::info("PoseDrift: wrote {} drifted poses back to the database",
             posed_ids.size());

  // The drifted trajectory stands in for a new INPUT capture: `rux optimize`
  // seeds from initial_transform (spec 2026-10-13 §4.3), and odometry derived
  // from the old poses would quietly undo the drift.
  namespace pg = core::pose_graph;
  const auto stored = db.list_pose_graph_edges();
  const bool has_init_edges =
      std::any_of(stored.begin(), stored.end(), [](const auto &e) {
        return e.source == pg::kSourceOdometryInit &&
               e.status != pg::kStatusRejected;
      });
  const auto foreign = std::count_if(
      stored.begin(), stored.end(), [](const auto &e) {
        return e.edge_type == pg::kTypeOdometry &&
               e.source != pg::kSourceOdometryInit &&
               e.status != pg::kStatusRejected && e.transform.has_value();
      });
  if (has_init_edges) {
    slam::init_pose_graph(db, {.force = true});
  } else {
    for (size_t k = 0; k < posed_ids.size(); ++k)
      db.set_initial_pose(posed_ids[k], to_array16_double(poses[k]));
  }
  if (foreign > 0)
    core::warn("PoseDrift: {} stored odometry edges (e.g. from 'rux import "
               "rtabmap') were measured on the undrifted capture and will "
               "pull the solve back to it; reject them ('rux graph prune "
               "--source rtabmap --type odometry') for an honest drift "
               "benchmark",
               foreign);
  return r;
```

(`#include <algorithm>` if missing.) Update the header doc of `perturb_sensor_poses` with one sentence: "Also writes the drifted poses to `initial_transform` and, when the project has `odometry_init` edges, re-derives them (`slam::init_pose_graph(force)`), so `rux optimize` sees the drift."

- [ ] **Step 4: Run tests to verify they pass**

Run: `nix develop -c cmake --build build --target reusex_unit_tests -j$(nproc) && nix develop -c build/tests/reusex_unit_tests "[drift]"`
Expected: PASS (all old `perturb_trajectory_*` cases plus the two new ones).

- [ ] **Step 5: Commit**

```bash
git add libs/reusex/src/slam/perturb_poses.cpp libs/reusex/include/slam/perturb_poses.hpp tests/unit/slam/test_perturb_poses.cpp
git commit -m "slam: perturb-poses re-snapshots initial poses and re-derives odometry_init

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_018dLKtE8qid8y5QVF9b2yri"
```

---

## Task 12: `rux import rtabmap` stores RTABMap links as edges

**Files:**
- Create: `libs/reusex/include/io/rtabmap_links.hpp`, `libs/reusex/src/io/rtabmap_links.cpp`
- Modify: `libs/reusex/src/io/rtabmap.cpp` (pose array helper, `initial_transform`, link import)
- Modify: `libs/reusex/include/io/rtabmap.hpp:15-28` (doc)
- Test: `tests/unit/io/test_rtabmap_links.cpp`

**Interfaces:**
- Consumes: `core::pose_graph` helpers, `ProjectDB::{upsert_pose_graph_edge, set_initial_pose, Transaction}`.
- Produces:

```cpp
namespace reusex::io {
namespace rtabmap_link_type {   // mirrors rtabmap::Link::Type, static_asserted in rtabmap.cpp
inline constexpr int neighbor = 0, global_closure = 1, local_space_closure = 2,
    local_time_closure = 3, user_closure = 4, virtual_closure = 5, neighbor_merged = 6,
    pose_prior = 7, landmark = 8, gravity = 9;
}
struct RtabmapLink { int from = 0; int to = 0; int type = 0;
                     std::array<double, 16> transform{}; std::array<double, 36> information{}; };
enum class RtabmapLinkSkip { none, unsupported_type, self_link, endpoint_not_imported, invalid_transform };
struct RtabmapLinkConversion { std::optional<ProjectDB::PoseGraphEdge> edge; RtabmapLinkSkip skip = RtabmapLinkSkip::none; };
std::array<double, 16> rtabmap_transform_to_array(const float *data3x4);
std::array<double, 36> rtabmap_information_to_gtsam(const std::array<double, 36> &rtabmap_order);
RtabmapLinkConversion convert_rtabmap_link(const RtabmapLink &link, int id_offset,
    const std::map<int, std::array<double, 16>> &imported_poses);
}
```

**Frame convention (verified by reading the code):** `import_rtabmap` writes `sensor_frames.transform` = the RTABMap **base-frame** pose verbatim (`rtabmap.cpp:130-140`, `pose.data()` 3×4 row-major floats), and the camera's `localTransform` separately into the intrinsics (`:108-124`). RTABMap links are relative poses between the same base frames, with `pose(to) = pose(from) · link.transform()` — exactly the spec's `T_from⁻¹ · T_to` in the `sensor_frames.transform` convention. So the link transform is stored **unconverted**; both the pose write path and the link path go through the one helper `rtabmap_transform_to_array`, and `seed_disagreement_*` against the imported poses is the built-in check (≈ 0 for odometry links when the convention is right; Step 6 verifies it on real data). `infMatrix()` is in RTABMap's (x, y, z, roll, pitch, yaw) order and is permuted to GTSAM's (rx, ry, rz, tx, ty, tz). Direction is left as RTABMap gives it; `upsert_pose_graph_edge` canonicalises.

- [ ] **Step 1: Write the failing test**

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// RTABMap link -> pose_graph_edges conversion (spec §4.2). Pure: no rtabmap
// headers, so it is unit-testable without an RTABMap database.

#include <catch2/catch_test_macros.hpp>

#include <reusex/core/pose_graph.hpp>
#include <reusex/io/rtabmap_links.hpp>

#include <Eigen/Geometry>

#include <map>

namespace io = reusex::io;
namespace pg = reusex::core::pose_graph;

namespace {

Eigen::Matrix4d make_pose(double yaw, double x, double y) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.topLeftCorner<3, 3>() =
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  T(0, 3) = x;
  T(1, 3) = y;
  return T;
}

std::array<double, 36> rtabmap_diag(double tx, double ty, double tz,
                                     double rr, double rp, double ry) {
  std::array<double, 36> a{};
  a[0] = tx;
  a[7] = ty;
  a[14] = tz;
  a[21] = rr;
  a[28] = rp;
  a[35] = ry;
  return a;
}

io::RtabmapLink link(int from, int to, int type, const Eigen::Matrix4d &T) {
  io::RtabmapLink l;
  l.from = from;
  l.to = to;
  l.type = type;
  l.transform = pg::row_major_from_matrix(T);
  l.information = rtabmap_diag(100, 100, 100, 400, 400, 400);
  return l;
}

} // namespace

TEST_CASE("RtabmapTransformToArray_3x4Floats_RowMajorWithBottomRow",
          "[io][rtabmap]") {
  const float d[12] = {1, 0, 0, 1.5f, 0, 1, 0, -2.0f, 0, 0, 1, 0.25f};
  const auto a = io::rtabmap_transform_to_array(d);
  CHECK(a[3] == 1.5);
  CHECK(a[7] == -2.0);
  CHECK(a[11] == 0.25);
  CHECK(a[12] == 0.0);
  CHECK(a[15] == 1.0);
}

TEST_CASE("RtabmapInformation_PermutesToGtsamOrder", "[io][rtabmap]") {
  auto in = rtabmap_diag(1, 2, 3, 4, 5, 6);
  in[0 * 6 + 3] = in[3 * 6 + 0] = 0.5; // (x, roll) coupling
  const auto out = io::rtabmap_information_to_gtsam(in);
  CHECK(out[0] == 4.0);  // rx <- roll
  CHECK(out[14] == 6.0); // rz <- yaw
  CHECK(out[21] == 1.0); // tx <- x
  CHECK(out[35] == 3.0); // tz <- z
  CHECK(out[0 * 6 + 3] == 0.5); // (rx, tx) <- (roll, x)
  CHECK(out[3 * 6 + 0] == 0.5);
}

TEST_CASE("RtabmapLink_Neighbor_BecomesOdometryInTheStoredConvention",
          "[io][rtabmap]") {
  const Eigen::Matrix4d P1 = make_pose(0.3, 1.0, 2.0);
  const Eigen::Matrix4d P2 = make_pose(0.5, 1.4, 2.1);
  const std::map<int, std::array<double, 16>> imported = {
      {1, pg::row_major_from_matrix(P1)}, {2, pg::row_major_from_matrix(P2)}};
  const auto c = io::convert_rtabmap_link(
      link(1, 2, io::rtabmap_link_type::neighbor, P1.inverse() * P2),
      /*id_offset=*/100, imported);
  REQUIRE(c.edge.has_value());
  CHECK(c.skip == io::RtabmapLinkSkip::none);
  CHECK(c.edge->from_node_id == 101);
  CHECK(c.edge->to_node_id == 102);
  CHECK(c.edge->edge_type == "odometry");
  CHECK(c.edge->source == "rtabmap");
  CHECK(c.edge->status == "auto");
  CHECK(*c.edge->seed_disagreement_m < 1e-9); // stored == imported poses
  REQUIRE(c.edge->information.has_value());
  CHECK((*c.edge->information)[0] == 400.0); // rotation first
}

TEST_CASE("RtabmapLink_LoopClosureTypes_BecomeLoopClosure", "[io][rtabmap]") {
  const std::map<int, std::array<double, 16>> imported = {
      {3, pg::row_major_from_matrix(Eigen::Matrix4d::Identity())},
      {9, pg::row_major_from_matrix(make_pose(0, 3, 0))}};
  for (int type : {io::rtabmap_link_type::global_closure,
                   io::rtabmap_link_type::local_space_closure,
                   io::rtabmap_link_type::local_time_closure,
                   io::rtabmap_link_type::user_closure}) {
    INFO(type);
    // RTABMap stores loop closures newest -> oldest; direction is kept here
    // and canonicalised by ProjectDB::upsert_pose_graph_edge.
    const auto c = io::convert_rtabmap_link(
        link(9, 3, type, make_pose(0, -3, 0)), 0, imported);
    REQUIRE(c.edge.has_value());
    CHECK(c.edge->edge_type == "loop_closure");
    CHECK(c.edge->from_node_id == 9);
  }
}

TEST_CASE("RtabmapLink_UnsupportedOrDangling_IsSkippedWithAReason",
          "[io][rtabmap]") {
  const std::map<int, std::array<double, 16>> imported = {
      {1, pg::row_major_from_matrix(Eigen::Matrix4d::Identity())},
      {2, pg::row_major_from_matrix(Eigen::Matrix4d::Identity())}};
  const auto I = Eigen::Matrix4d::Identity();
  CHECK(io::convert_rtabmap_link(link(1, 2, io::rtabmap_link_type::virtual_closure, I), 0, imported).skip ==
        io::RtabmapLinkSkip::unsupported_type);
  CHECK(io::convert_rtabmap_link(link(1, -4, io::rtabmap_link_type::landmark, I), 0, imported).skip ==
        io::RtabmapLinkSkip::unsupported_type);
  CHECK(io::convert_rtabmap_link(link(1, 1, io::rtabmap_link_type::gravity, I), 0, imported).skip ==
        io::RtabmapLinkSkip::unsupported_type);
  CHECK(io::convert_rtabmap_link(link(2, 2, io::rtabmap_link_type::neighbor, I), 0, imported).skip ==
        io::RtabmapLinkSkip::self_link);
  CHECK(io::convert_rtabmap_link(link(1, 7, io::rtabmap_link_type::neighbor, I), 0, imported).skip ==
        io::RtabmapLinkSkip::endpoint_not_imported);
  auto broken = link(1, 2, io::rtabmap_link_type::neighbor, I);
  broken.transform = {};
  CHECK(io::convert_rtabmap_link(broken, 0, imported).skip ==
        io::RtabmapLinkSkip::invalid_transform);
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `nix develop -c cmake --build build --target reusex_unit_tests -j$(nproc)`
Expected: FAIL — `reusex/io/rtabmap_links.hpp: No such file or directory`.

- [ ] **Step 3: Write the header and the converter**

`libs/reusex/include/io/rtabmap_links.hpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// RTABMap graph links -> pose_graph_edges (spec 2026-10-13 §4.2). Kept free
// of rtabmap headers (they stay confined to io/rtabmap.cpp), so the mapping
// is unit-testable without an RTABMap database.

#include <reusex/core/ProjectDB.hpp>

#include <array>
#include <map>
#include <optional>

namespace reusex::io {

/// Mirrors rtabmap::Link::Type (rtabmap 0.23, Link.h); io/rtabmap.cpp
/// static_asserts every value.
namespace rtabmap_link_type {
inline constexpr int neighbor = 0;
inline constexpr int global_closure = 1;
inline constexpr int local_space_closure = 2;
inline constexpr int local_time_closure = 3;
inline constexpr int user_closure = 4;
inline constexpr int virtual_closure = 5;
inline constexpr int neighbor_merged = 6;
inline constexpr int pose_prior = 7;
inline constexpr int landmark = 8;
inline constexpr int gravity = 9;
} // namespace rtabmap_link_type

struct RtabmapLink {
  int from = 0; ///< RTABMap node id (before the import's id offset)
  int to = 0;
  int type = 0; ///< rtabmap_link_type
  /// pose(to) = pose(from) * transform, base frame, row-major 4x4.
  std::array<double, 16> transform{};
  /// RTABMap order (x, y, z, roll, pitch, yaw), row-major 6x6.
  std::array<double, 36> information{};
};

enum class RtabmapLinkSkip {
  none,
  unsupported_type,      ///< virtual / landmark / gravity / pose prior / other
  self_link,             ///< from == to
  endpoint_not_imported, ///< a node without a colour image was not imported
  invalid_transform,     ///< null or non-rigid link transform
};

struct RtabmapLinkConversion {
  std::optional<ProjectDB::PoseGraphEdge> edge;
  RtabmapLinkSkip skip = RtabmapLinkSkip::none;
};

/// RTABMap's 3x4 row-major float pose -> row-major 4x4 doubles. The pose
/// write path and the link path share it, so both use one convention.
std::array<double, 16> rtabmap_transform_to_array(const float *data3x4);

/// (x, y, z, roll, pitch, yaw) -> GTSAM (rx, ry, rz, tx, ty, tz).
std::array<double, 36>
rtabmap_information_to_gtsam(const std::array<double, 36> &rtabmap_order);

/// Map one link. Neighbor / NeighborMerged -> "odometry"; Global /
/// LocalSpace / LocalTime / User closures -> "loop_closure"; everything
/// else is skipped. Node ids get @p id_offset added. @p imported_poses holds
/// the stored transform of every imported node, keyed by RTABMap id; it
/// feeds seed_disagreement_*. The direction is kept as given.
RtabmapLinkConversion
convert_rtabmap_link(const RtabmapLink &link, int id_offset,
                     const std::map<int, std::array<double, 16>> &imported_poses);

} // namespace reusex::io
```

`libs/reusex/src/io/rtabmap_links.cpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "io/rtabmap_links.hpp"

#include "core/pose_graph.hpp"

#include <string>

namespace reusex::io {

namespace pg = core::pose_graph;

std::array<double, 16> rtabmap_transform_to_array(const float *d) {
  std::array<double, 16> a{};
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 4; ++c)
      a[static_cast<size_t>(r * 4 + c)] = static_cast<double>(d[r * 4 + c]);
  a[15] = 1.0;
  return a;
}

std::array<double, 36>
rtabmap_information_to_gtsam(const std::array<double, 36> &in) {
  // GTSAM index k reads RTABMap index p[k].
  constexpr int p[6] = {3, 4, 5, 0, 1, 2};
  std::array<double, 36> out{};
  for (int r = 0; r < 6; ++r)
    for (int c = 0; c < 6; ++c)
      out[static_cast<size_t>(r * 6 + c)] =
          in[static_cast<size_t>(p[r] * 6 + p[c])];
  return out;
}

RtabmapLinkConversion
convert_rtabmap_link(const RtabmapLink &link, int id_offset,
                     const std::map<int, std::array<double, 16>> &imported) {
  RtabmapLinkConversion out;
  std::string type;
  switch (link.type) {
  case rtabmap_link_type::neighbor:
  case rtabmap_link_type::neighbor_merged:
    type = std::string(pg::kTypeOdometry);
    break;
  case rtabmap_link_type::global_closure:
  case rtabmap_link_type::local_space_closure:
  case rtabmap_link_type::local_time_closure:
  case rtabmap_link_type::user_closure:
    type = std::string(pg::kTypeLoopClosure);
    break;
  default:
    out.skip = RtabmapLinkSkip::unsupported_type;
    return out;
  }
  if (link.from == link.to) {
    out.skip = RtabmapLinkSkip::self_link;
    return out;
  }
  const auto a = imported.find(link.from);
  const auto b = imported.find(link.to);
  if (a == imported.end() || b == imported.end()) {
    out.skip = RtabmapLinkSkip::endpoint_not_imported;
    return out;
  }
  if (!pg::is_rigid_transform(link.transform)) {
    out.skip = RtabmapLinkSkip::invalid_transform;
    return out;
  }

  ProjectDB::PoseGraphEdge e;
  e.from_node_id = link.from + id_offset;
  e.to_node_id = link.to + id_offset;
  e.edge_type = type;
  e.transform = link.transform;
  const auto info = rtabmap_information_to_gtsam(link.information);
  if (pg::is_valid_information(info))
    e.information = info; // else NULL: the per-type default sigma
  e.source = std::string(pg::kSourceRtabmap);
  e.status = std::string(pg::kStatusAuto);
  const auto err = pg::transform_error(pg::matrix_from_row_major(link.transform),
                                       pg::matrix_from_row_major(a->second),
                                       pg::matrix_from_row_major(b->second));
  e.seed_disagreement_m = err.translation_m;
  e.seed_disagreement_deg = err.rotation_deg;
  out.edge = std::move(e);
  return out;
}

} // namespace reusex::io
```

- [ ] **Step 4: Wire the importer**

In `libs/reusex/src/io/rtabmap.cpp`:

1. Add includes `"io/rtabmap_links.hpp"`, `"core/pose_graph.hpp"`, `<set>`, `<tuple>`, and after the includes:

```cpp
static_assert(reusex::io::rtabmap_link_type::neighbor == rtabmap::Link::kNeighbor);
static_assert(reusex::io::rtabmap_link_type::global_closure == rtabmap::Link::kGlobalClosure);
static_assert(reusex::io::rtabmap_link_type::local_space_closure == rtabmap::Link::kLocalSpaceClosure);
static_assert(reusex::io::rtabmap_link_type::local_time_closure == rtabmap::Link::kLocalTimeClosure);
static_assert(reusex::io::rtabmap_link_type::user_closure == rtabmap::Link::kUserClosure);
static_assert(reusex::io::rtabmap_link_type::virtual_closure == rtabmap::Link::kVirtualClosure);
static_assert(reusex::io::rtabmap_link_type::neighbor_merged == rtabmap::Link::kNeighborMerged);
static_assert(reusex::io::rtabmap_link_type::pose_prior == rtabmap::Link::kPosePrior);
static_assert(reusex::io::rtabmap_link_type::landmark == rtabmap::Link::kLandmark);
static_assert(reusex::io::rtabmap_link_type::gravity == rtabmap::Link::kGravity);
```

2. Update the comment at :31-34 ("pose_graph_edges has no FK constraint but is written only by `rux optimize` …") to: "pose_graph_edges has no FK; this importer writes RTABMap links for its own (offset) id range only."
3. Before the node loop declare `std::map<int, std::array<double, 16>> imported; // RTABMap id -> stored pose`.
4. Replace the "World pose as row-major 4x4" block (:128-141) with:

```cpp
      // ── World pose as row-major 4x4 ─────────────────────────────
      // The RTABMap BASE-frame pose, verbatim (the camera's localTransform
      // lives in the intrinsics). RTABMap links are relative poses between
      // the same base frames, so they need no conversion either.
      std::array<double, 16> worldPose =
          pose.isNull()
              ? std::array<double, 16>{1, 0, 0, 0, 0, 1, 0, 0,
                                       0, 0, 1, 0, 0, 0, 0, 1}
              : rtabmap_transform_to_array(pose.data());
```

5. After `db.save_sensor_frame(...)` add:

```cpp
      db.set_initial_pose(id + scan.id_offset, worldPose);
      imported.emplace(id, worldPose);
```

6. After the node loop, before `db.log_pipeline_end(logId, true);`:

```cpp
    // ── Links -> pose_graph_edges (spec §4.2) ─────────────────────
    int odometry = 0, loops = 0, duplicates = 0;
    std::map<RtabmapLinkSkip, int> skipped;
    std::set<std::tuple<int, int, std::string>> seen;
    {
      ProjectDB::Transaction tx(db);
      for (const auto &[key, l] : links) {
        RtabmapLink in;
        in.from = l.from();
        in.to = l.to();
        in.type = static_cast<int>(l.type());
        in.transform = l.transform().isNull()
                           ? std::array<double, 16>{}
                           : rtabmap_transform_to_array(l.transform().data());
        const cv::Mat &inf = l.infMatrix();
        if (inf.rows == 6 && inf.cols == 6 && inf.type() == CV_64FC1)
          for (int r = 0; r < 6; ++r)
            for (int c = 0; c < 6; ++c)
              in.information[static_cast<size_t>(r * 6 + c)] =
                  inf.at<double>(r, c);
        const auto conv = convert_rtabmap_link(in, scan.id_offset, imported);
        if (!conv.edge) {
          ++skipped[conv.skip];
          continue;
        }
        const auto &e = *conv.edge;
        if (!seen.emplace(std::min(e.from_node_id, e.to_node_id),
                          std::max(e.from_node_id, e.to_node_id), e.edge_type)
                 .second) {
          ++duplicates; // the same link listed in both directions
          continue;
        }
        db.upsert_pose_graph_edge(e);
        (e.edge_type == core::pose_graph::kTypeOdometry ? odometry : loops)++;
      }
      tx.commit();
    }
    core::info("RTABMap links: {} odometry + {} loop-closure edges stored "
               "in pose_graph_edges",
               odometry, loops);
    const int unsupported = skipped[RtabmapLinkSkip::unsupported_type];
    const int self = skipped[RtabmapLinkSkip::self_link];
    const int dangling = skipped[RtabmapLinkSkip::endpoint_not_imported];
    const int invalid = skipped[RtabmapLinkSkip::invalid_transform];
    if (unsupported + self + dangling + invalid + duplicates > 0)
      core::warn("RTABMap links skipped: {} unsupported type (virtual, "
                 "landmark, gravity, prior), {} self links, {} touching a "
                 "node that was not imported, {} with a null/non-rigid "
                 "transform, {} reverse-direction duplicates",
                 unsupported, self, dangling, invalid, duplicates);
```

7. In `io/rtabmap.hpp`, extend the doc: "Also stores RTABMap's graph links in `pose_graph_edges` (Neighbor/NeighborMerged → odometry, Global/LocalSpace/LocalTime/User closures → loop_closure, source `rtabmap`) and each imported pose in `sensor_frames.initial_transform`."

- [ ] **Step 5: Run tests to verify they pass**

```bash
nix develop -c cmake --build build -j$(nproc)
nix develop -c build/tests/reusex_unit_tests "[rtabmap]"
```

Expected: PASS (6 cases); `rux` links (the `static_assert`s hold).

- [ ] **Step 6: Verify the frame convention on a real RTABMap database**

```bash
DB=$( (ls ~/repos/StrayScannerToRTABMap/afb3234950/*.db; find ~/datasets ~/repos/StrayScannerToRTABMap -maxdepth 3 -name '*.db') 2>/dev/null | head -1)
echo "RTABMap database: ${DB:-none}"
S=/tmp/claude-1001/-home-mephisto-repos-ReUseX--worktrees-gui-graph-editor/b9a0a5e9-d5a5-48c6-adfb-71b8bc180a4a/scratchpad
rm -f $S/rtab_links.rux*
[ -n "$DB" ] && nix develop -c build/apps/rux/rux -v -p $S/rtab_links.rux import rtabmap "$DB"
python3 - "$S/rtab_links.rux" <<'PY'
import sqlite3, statistics, sys
c = sqlite3.connect(sys.argv[1])
for t in ("odometry", "loop_closure"):
    v = [r[0] for r in c.execute("SELECT seed_disagreement_m FROM pose_graph_edges "
                                  "WHERE source='rtabmap' AND edge_type=?", (t,))]
    print(t, len(v), "median seed_disagreement_m =", statistics.median(v) if v else None)
PY
```

Expected: a non-zero odometry count; odometry median `seed_disagreement_m` below 0.05 m (centimetre-level: RTABMap's optimized poses vs. its raw odometry). A median above 0.2 m means the convention is wrong — **stop and report**, do not "fix" it by guessing. If no RTABMap `.db` exists on the machine, record "real-data convention check skipped: no RTABMap database available" in the task notes.

- [ ] **Step 7: Commit**

```bash
git add libs/reusex/include/io/rtabmap_links.hpp libs/reusex/src/io/rtabmap_links.cpp \
        libs/reusex/src/io/rtabmap.cpp libs/reusex/include/io/rtabmap.hpp tests/unit/io/test_rtabmap_links.cpp
git commit -m "io: import RTABMap links as pose-graph edges; snapshot initial poses

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_018dLKtE8qid8y5QVF9b2yri"
```

---

## Task 13: `rux graph init`, the `graph_init` contract, scripts and docs for P2

**Files:**
- Create: `apps/rux/include/graph.hpp`, `apps/rux/src/graph.cpp`, `apps/rux/include/graph/init.hpp`, `apps/rux/src/graph/init.cpp`
- Modify: `apps/rux/src/rux.cpp:16,199`
- Modify: `libs/reusex/include/core/stage_contract.hpp:37-55`, `libs/reusex/src/core/stage_contract.cpp:17-82`
- Modify: `docs/CONTRACTS.md` (import section, new `graph_init` section, optimize section, FK note, pipeline order)
- Modify: `docs/DIRECTION.md` (changelog), `CLAUDE.md` (CLI table, slam section), `README.md:166-167`
- Modify: `scripts/bench-office.sh:93`
- Create: `tests/unit/rux_app/test_graph_cli.cpp`

**Interfaces:**
- Consumes: `slam::init_pose_graph`, `slam::PoseGraphExistsError` (Task 7), `rux::check_stage_prerequisites`.
- Produces: `setup_subcommand_graph(CLI::App&, std::shared_ptr<RuxOptions>)`; `SubcommandGraphInitOptions{bool force}`; `core::PipelineStage::graph_init` (token `"graph_init"`); artifact `pose_graph_edges` (`ArtifactKind::table`).

- [ ] **Step 1: Write the failing tests**

`tests/unit/rux_app/test_graph_cli.cpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// `rux graph ...` subcommands parsed and run through the real CLI11 setup.

#include <catch2/catch_test_macros.hpp>

#include <global-params.hpp>
#include <graph.hpp>

#include <reusex/core/ProjectDB.hpp>

#include "../../support/pose_fixture.hpp"
#include "../../support/temp_path.hpp"

#include <CLI/CLI.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

using namespace reusex::test_support;

namespace {

/// Parse and run `rux -p <project> <args...>`; true when it succeeded.
bool run_rux(const std::filesystem::path &project,
             std::vector<std::string> args) {
  CLI::App app{"rux"};
  auto global = std::make_shared<RuxOptions>();
  app.add_option("-p,--project", global->project_db);
  setup_subcommand_graph(app, global);
  args.insert(args.begin(), {"-p", project.string()});
  std::vector<std::string> reversed(args.rbegin(), args.rend());
  try {
    app.parse(reversed);
  } catch (const CLI::RuntimeError &) {
    return false;
  }
  return true;
}

void make_frames(const std::filesystem::path &path, int n) {
  reusex::ProjectDB db(path);
  for (int k = 1; k <= n; ++k)
    db.save_sensor_frame(k, make_color(8, 8), make_depth(8, 8, 1.0), cv::Mat(),
                         translation_pose(0.1 * k, 0, 0),
                         make_intrinsics(8, 8, 4, 4, 8, 8), k);
}

} // namespace

TEST_CASE("GraphCli_Init_RefusesASecondInitWithoutForce",
          "[rux_app][cli][graph]") {
  TempPath tmp("test_graph_cli_init");
  make_frames(tmp.path, 4);
  CHECK(run_rux(tmp.path, {"graph", "init"}));
  CHECK_FALSE(run_rux(tmp.path, {"graph", "init"}));
  CHECK(run_rux(tmp.path, {"graph", "init", "--force"}));
  reusex::ProjectDB db(tmp.path, /*readOnly=*/true);
  CHECK(db.list_pose_graph_edges().size() == 3);
  const auto log = db.pipeline_log();
  CHECK(std::count_if(log.begin(), log.end(), [](const auto &e) {
          return e.stage == "graph_init" && e.status == "success";
        }) == 2);
}

TEST_CASE("GraphCli_Init_NeedsTwoFrames", "[rux_app][cli][graph]") {
  TempPath tmp("test_graph_cli_init_empty");
  make_frames(tmp.path, 1);
  CHECK_FALSE(run_rux(tmp.path, {"graph", "init"}));
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `nix develop -c cmake --build build --target reusex_unit_tests_vision -j$(nproc)`
Expected: FAIL — `graph.hpp: No such file or directory`.

- [ ] **Step 3: The contract**

`stage_contract.hpp`: insert `graph_init,` after `import,` with the comment `///< snapshot initial poses + odometry edges (rux graph init)`.

`stage_contract.cpp`:
- Artifact registry, after `segmentation_images`:

```cpp
      {"pose_graph_edges", ArtifactKind::table, Alignment::none,
       "the stored pose graph: odometry and loop edges with measurements, "
       "status and solve metrics (schema v27)"},
```

- `import` row outputs → `{"sensor_frames", "pose_graph_edges"}` with the comment `// RTABMap links become pose_graph_edges (rtabmap only).`
- New row after `import`:

```cpp
      {PipelineStage::graph_init,
       "graph_init",
       {},
       "rux graph init",
       "snapshot the initial poses and freeze odometry edges between "
       "consecutive frames",
       {{{"sensor_frames"}, /*min_rows=*/2}},
       {"pose_graph_edges"}},
```

- `optimize` row outputs → `{"sensor_frames", "pose_graph_edges"}`.

Before editing, run `grep -rn "static_cast<int>(.*PipelineStage\|PipelineStage>(" libs apps tests` — the enum's numeric values must not be persisted anywhere (expected: no hits); inserting a value in the middle is then safe.

- [ ] **Step 4: The CLI**

`apps/rux/include/graph.hpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "global-params.hpp"

#include <CLI/CLI.hpp>
#include <memory>

/// `rux graph` — edit the stored pose graph (spec 2026-10-13 §4.3).
void setup_subcommand_graph(CLI::App &app,
                            std::shared_ptr<RuxOptions> global_opt);
```

`apps/rux/include/graph/init.hpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "../global-params.hpp"

#include <reusex/slam/pose_graph.hpp>

#include <CLI/CLI.hpp>
#include <memory>

struct SubcommandGraphInitOptions {
  bool force = reusex::slam::InitPoseGraphOptions{}.force;
};

void setup_subcommand_graph_init(CLI::App &parent,
                                 std::shared_ptr<RuxOptions> global_opt);
int run_subcommand_graph_init(const SubcommandGraphInitOptions &opt,
                              const RuxOptions &global_opt);
```

`apps/rux/src/graph.cpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "graph.hpp"
#include "graph/init.hpp"

void setup_subcommand_graph(CLI::App &app,
                            std::shared_ptr<RuxOptions> global_opt) {
  auto *sub = app.add_subcommand(
      "graph", "Edit the stored pose graph that 'rux optimize' solves");
  sub->footer(R"(
DESCRIPTION:
  The project's pose graph (pose_graph_edges) is the source of truth for
  'rux optimize': odometry edges (from 'rux import rtabmap' or 'graph init'),
  loop edges, and rejected edges kept as tombstones.

SUBCOMMANDS:
  init      Snapshot initial poses and create odometry edges

NOTES:
  - Use 'rux graph <subcommand> --help' for detailed options
)");
  sub->require_subcommand(1);
  setup_subcommand_graph_init(*sub, global_opt);
}
```

`apps/rux/src/graph/init.cpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "graph/init.hpp"
#include "exit_status.hpp"
#include "stage_prerequisites.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/stage_contract.hpp>

#include <fmt/format.h>
#include <spdlog/spdlog.h>

#include <exception>

void setup_subcommand_graph_init(CLI::App &parent,
                                 std::shared_ptr<RuxOptions> global_opt) {
  auto opt = std::make_shared<SubcommandGraphInitOptions>();
  auto *sub = parent.add_subcommand(
      "init", "Snapshot initial poses and create odometry edges");
  sub->footer(R"(
DESCRIPTION:
  Copies every frame's current pose into sensor_frames.initial_transform (the
  pose 'rux optimize' seeds from) and stores one odometry edge per pair of
  consecutive frames of each scan, measured from those poses. 'rux optimize'
  runs this automatically when the graph has no odometry edges.

  Refuses when odometry edges exist. --force replaces only the odometry_init
  edges and re-snapshots the initial poses; loop edges, RTABMap edges and
  rejected edges are kept. Use it after changing poses outside optimize
  (e.g. 'rux register') to optimise on top of that result.
)");
  sub->add_flag("--force", opt->force,
                "Re-derive the odometry_init edges and re-snapshot every "
                "initial pose from the current poses");
  sub->callback([opt, global_opt]() {
    rux::finish(run_subcommand_graph_init(*opt, *global_opt));
  });
}

int run_subcommand_graph_init(const SubcommandGraphInitOptions &opt,
                              const RuxOptions &global_opt) {
  try {
    reusex::ProjectDB db(global_opt.project_db);
    if (int rc = rux::check_stage_prerequisites(
            db, reusex::core::PipelineStage::graph_init);
        rc != RuxError::SUCCESS)
      return rc;
    const int logId = db.log_pipeline_start(
        "graph_init", fmt::format(R"({{"force":{}}})", opt.force));
    try {
      const auto r = reusex::slam::init_pose_graph(db, {.force = opt.force});
      db.log_pipeline_end(logId, true);
      spdlog::info("Pose graph: {} frames in {} scan(s); {} initial poses "
                   "snapshotted; {} odometry edges written",
                   r.frames, r.scans, r.snapshots_written,
                   r.odometry_edges_written);
      return RuxError::SUCCESS;
    } catch (const reusex::slam::PoseGraphExistsError &e) {
      db.log_pipeline_end(logId, false, e.what());
      spdlog::error("{}", e.what());
      return RuxError::INVALID_ARGUMENT;
    } catch (const std::exception &e) {
      db.log_pipeline_end(logId, false, e.what());
      throw;
    }
  } catch (const std::exception &e) {
    spdlog::error("graph init failed: {}", e.what());
    return RuxError::GENERIC;
  }
}
```

`apps/rux/src/rux.cpp`: add `#include <graph.hpp>` (alphabetically after `<get.hpp>`) and `setup_subcommand_graph(app, opt);` right after `setup_subcommand_optimize(app, opt);`.

- [ ] **Step 5: Docs and scripts**

`docs/CONTRACTS.md`:
- `import` table: `| Produces | \`sensor_frames\`, \`pose_graph_edges\` (RTABMap links; \`rux import rtabmap\` only), one row in \`scans\` |`; add after the "Node-ID remapping" paragraph:

```markdown
**Pose graph.** `rux import rtabmap` also stores RTABMap's graph links in
`pose_graph_edges` (Neighbor/NeighborMerged → `odometry`, Global/LocalSpace/
LocalTime/User closures → `loop_closure`, source `rtabmap`, ids offset like the
frames) and each imported pose in `sensor_frames.initial_transform`.
```

- Replace the "FK safety invariant" sentence about `pose_graph_edges` with: "The one table without a FK constraint (`pose_graph_edges`) only receives edges for the importing session's own (offset) id range."
- New section between `import` and `optimize`:

```markdown
### `graph_init` (`rux graph init`)

Snapshot each frame's current pose into `sensor_frames.initial_transform` and
store one odometry edge per consecutive pair of frames of each scan (source
`odometry_init`). `rux optimize` runs it automatically when the graph has no
odometry edge with a measurement.

| | |
|---|---|
| Consumes | `sensor_frames` |
| Produces | `pose_graph_edges` |
| Options  | `InitPoseGraphOptions` (`libs/reusex/include/slam/pose_graph.hpp`) |
| Checks   | ≥2 stored frames; refuses when odometry edges exist unless `--force` |
```

- `optimize` section: first paragraph → "Solve the stored pose graph: seeds from `sensor_frames.initial_transform`, one factor per active `pose_graph_edges` row, writes poses to `sensor_frames.transform` and residual/deviation/seed-disagreement back to every edge. Repeatable; never adds, removes or re-statuses edges (`rux register`: joint pairwise registration, writes poses only)."; `| Produces | \`sensor_frames\` (updated poses), \`pose_graph_edges\` (per-edge metrics; \`optimize\` only) |`.
- Pipeline order diagram: `import → [graph_init] → (optimize|register) → clouds → …`.

`docs/DIRECTION.md`, top of "## Direction changelog":

```markdown
- **2026-10-13** — **Pose graph: the stored graph is the source of truth;
  `rux optimize` is repeatable** ([spec](superpowers/specs/2026-10-13-pose-graph-editor-design.md)).
  `pose_graph_edges` (schema v27) stores each edge's measurement, information,
  source (`rtabmap | odometry_init | orb | panorama | file | icp | manual |
  legacy`) and status (`auto | confirmed | rejected`; a rejection is a
  tombstone), and `sensor_frames.initial_transform` snapshots the poses
  optimize seeds from. `rux optimize` solves exactly the active stored edges,
  writes poses plus per-edge residual/deviation, and never replaces the edge
  set, so re-running gives the same poses instead of compounding. Odometry
  comes from RTABMap links on `rux import rtabmap`, or from `rux graph init`
  (run automatically when missing). **CLI break:** `--loop-closure`,
  `--loop-*` (except `--loop-trust*`), `--use-panoramas`, `--pano-*` and
  `--loop-edges*` moved from `rux optimize` to `rux graph discover`;
  optimize refuses them with a pointer. Chaining `rux register` before
  `rux optimize` now needs `rux graph init --force` in between (optimize
  warns when it sees that order).
```

`CLAUDE.md`: in the CLI table add the row `| \`graph\` | \`init\` (stored pose graph) | \`src/graph/\` |` after `optimize`, change the `optimize` row's description to "— (solves the stored pose graph)", and append `graph` to the "`create`, `import`, … all `require_subcommand(1)`" list. In "**slam**": replace the `PlaneGraphOptimizer.hpp` bullet's last sentence ("Also hosts the optional wide-baseline loop-closure front-end …") with "Solves the stored pose graph (`pose_graph_edges`, seeded from `sensor_frames.initial_transform`; `optimize_graph`); `slam/pose_graph.hpp` is `rux graph init`, `slam/loop_candidates.hpp` the ORB / panorama / file loop front-ends.", and add a bullet `- \`pose_graph_conventions.hpp\`: stored (\`sensor_frames.transform\`) ↔ optical edge transforms and information; \`core/pose_graph.hpp\` holds the schema vocabulary and SE(3) helpers.` In "Core Module — ProjectDB" add after the v26 sentence: "Schema v27 makes `pose_graph_edges` the stored pose graph (measurement, GTSAM-order information, source, status, metrics; canonical `from < to`, `UNIQUE(from, to, type)`; `upsert_pose_graph_edge` / `set_pose_graph_edge_status` / `update_pose_graph_edge_metrics`), adds `sensor_frames.initial_transform` and `project_settings` (`setting()` / `set_setting()`)."

`README.md:166-167` → 

```
rux optimize                           # Plane-landmark pose graph (stored graph)
rux graph discover && rux optimize     # ...plus wide-baseline loop edges
```

`scripts/bench-office.sh:93` → `register-opt) "$rux" -v -p "$work" register && "$rux" -v -p "$work" graph init --force && "$rux" -v -p "$work" optimize ;;` and add above the `case` a comment line: `# optimize seeds from initial_transform (2026-10-13): re-snapshot after register so it chains.`

- [ ] **Step 6: Run tests to verify they pass**

```bash
nix develop -c cmake --build build -j$(nproc)
nix develop -c build/tests/reusex_unit_tests "[contract],[validate]"
nix develop -c build/tests/reusex_unit_tests_vision "[graph],[rux_app]"
nix develop -c build/apps/rux/rux graph --help
nix develop -c build/apps/rux/rux validate --help
```

Expected: PASS (including `ContractsDoc_StageSections_MatchContractTable` and `StageContractTable_*`); `rux graph --help` lists `init`.

- [ ] **Step 7: Run the full suite, then commit**

```bash
nix develop -c bash -c 'cd build && ctest --output-on-failure --parallel $(nproc) -E reusex_package_consumer'
git add apps/rux/include/graph.hpp apps/rux/src/graph.cpp apps/rux/include/graph/init.hpp apps/rux/src/graph/init.cpp \
        apps/rux/src/rux.cpp libs/reusex/include/core/stage_contract.hpp libs/reusex/src/core/stage_contract.cpp \
        docs/CONTRACTS.md docs/DIRECTION.md CLAUDE.md README.md scripts/bench-office.sh tests/unit/rux_app/test_graph_cli.cpp
git commit -m "rux: graph init; graph_init contract; docs for the stored-graph optimize

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_018dLKtE8qid8y5QVF9b2yri"
```

Expected ctest: everything that passed in Task 0 still passes.

---

# Phase P3 — `graph discover`, `graph prune`, `JobStage::graph_discover`

## Task 14: PCM with fixed inliers

**Files:**
- Modify: `libs/reusex/include/slam/LoopClosure.hpp` (after `filter_consistent_loop_edges`)
- Modify: `libs/reusex/src/slam/LoopClosure.cpp:359-440` (share the consistency predicate; add the indices function)
- Test: `tests/unit/slam/test_loop_closure.cpp` (append)

**Interfaces:**
- Consumes: `LoopEdge`, `LoopClosureOptions::{pcm_trans_threshold, pcm_rot_threshold, pcm_max_edges}`.
- Produces:

```cpp
namespace reusex::geometry {
std::vector<size_t> consistent_loop_edge_indices(const std::vector<LoopEdge> &edges,
                                                 const std::vector<Eigen::Matrix4d> &seed,
                                                 const LoopClosureOptions &options,
                                                 const std::vector<bool> &fixed);
}
```

`detail::pcm_filter` / `filter_consistent_loop_edges` keep their exact behaviour (ORB detection results must not move); only the consistency test is factored out.

- [ ] **Step 1: Write the failing test**

Append to `tests/unit/slam/test_loop_closure.cpp`:

```cpp
TEST_CASE("ConsistentLoopEdgeIndices_FixedEdge_OverridesALargerAliasingClique",
          "[slam][loop][pcm]") {
  // Six frames 1 m apart on x. True loops (0,4), (1,5) agree with the seed;
  // three aliasing edges (0,3), (1,4), (2,5) are each 2 m off in the same
  // way, so they agree with EACH OTHER and outvote the true pair — unless a
  // true edge is fixed (a user-confirmed stored edge, spec §4.3).
  std::vector<Eigen::Matrix4d> seed;
  for (int k = 0; k < 6; ++k)
    seed.push_back(planar_pose(1.0 * k, 0.0, 0.0));
  auto rel = [&](int i, int j) { return Eigen::Matrix4d(seed[i].inverse() * seed[j]); };
  Eigen::Matrix4d off = Eigen::Matrix4d::Identity();
  off(0, 3) = 2.0;
  const std::vector<LoopEdge> edges = {
      make_edge(0, 4, rel(0, 4), 100),       make_edge(1, 5, rel(1, 5), 100),
      make_edge(0, 3, rel(0, 3) * off, 100), make_edge(1, 4, rel(1, 4) * off, 100),
      make_edge(2, 5, rel(2, 5) * off, 100)};
  const LoopClosureOptions opt;

  const auto free = consistent_loop_edge_indices(edges, seed, opt, {});
  CHECK(free == std::vector<size_t>{2, 3, 4});

  const auto fixed = consistent_loop_edge_indices(
      edges, seed, opt, {true, false, false, false, false});
  CHECK(fixed == std::vector<size_t>{0, 1});
}

TEST_CASE("ConsistentLoopEdgeIndices_FixedEdgesSurviveTheCap",
          "[slam][loop][pcm]") {
  std::vector<Eigen::Matrix4d> seed;
  for (int k = 0; k < 6; ++k)
    seed.push_back(planar_pose(1.0 * k, 0.0, 0.0));
  auto rel = [&](int i, int j) { return Eigen::Matrix4d(seed[i].inverse() * seed[j]); };
  const std::vector<LoopEdge> edges = {make_edge(0, 4, rel(0, 4), 1),
                                       make_edge(1, 5, rel(1, 5), 500),
                                       make_edge(0, 5, rel(0, 5), 400)};
  LoopClosureOptions opt;
  opt.pcm_max_edges = 1; // would keep only the 500-inlier edge
  const auto kept =
      consistent_loop_edge_indices(edges, seed, opt, {true, false, false});
  CHECK(std::find(kept.begin(), kept.end(), size_t{0}) != kept.end());
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `nix develop -c cmake --build build --target reusex_unit_tests -j$(nproc)`
Expected: FAIL — `'consistent_loop_edge_indices' was not declared`.

- [ ] **Step 3: Implement**

Header, after `filter_consistent_loop_edges`:

```cpp
/// Indices (into @p edges, ascending) of the largest mutually consistent
/// subset that contains every edge with fixed[k] == true — `rux graph
/// discover` passes the user-confirmed stored edges as fixed (spec §4.3).
/// A non-fixed edge inconsistent with any fixed edge is never kept, even
/// when it belongs to a larger clique; fixed edges are always kept, and the
/// `pcm_max_edges` cap never drops one. Greedy max-clique like pcm_filter,
/// with deterministic (stable) ordering. @p fixed shorter than @p edges is
/// padded with false.
std::vector<size_t>
consistent_loop_edge_indices(const std::vector<LoopEdge> &edges,
                             const std::vector<Eigen::Matrix4d> &seed,
                             const LoopClosureOptions &options,
                             const std::vector<bool> &fixed);
```

In `LoopClosure.cpp`, inside `namespace detail` before `pcm_filter`, add the predicate and make `pcm_filter`'s `consistent` lambda call it (`return pcm_consistent(a, b, seed, opt);`):

```cpp
/// Two loop edges are consistent when chaining them through the seed
/// odometry closes a near-identity cycle (Mangelson et al., PCM).
bool pcm_consistent(const LoopEdge &a, const LoopEdge &b,
                    const std::vector<Eigen::Matrix4d> &seed,
                    const LoopClosureOptions &opt) {
  const Eigen::Matrix4d C = a.T_ij * (seed[a.j].inverse() * seed[b.j]) *
                            b.T_ij.inverse() *
                            (seed[b.i].inverse() * seed[a.i]);
  const double t = C.block<3, 1>(0, 3).norm();
  const double c = (C.block<3, 3>(0, 0).trace() - 1.0) / 2.0;
  const double r = std::acos(std::clamp(c, -1.0, 1.0));
  return t < opt.pcm_trans_threshold && r < opt.pcm_rot_threshold;
}
```

After `filter_consistent_loop_edges` (namespace `reusex::geometry`) add (with `#include <numeric>`):

```cpp
std::vector<size_t>
consistent_loop_edge_indices(const std::vector<LoopEdge> &edges,
                             const std::vector<Eigen::Matrix4d> &seed,
                             const LoopClosureOptions &opt,
                             const std::vector<bool> &fixed_in) {
  std::vector<bool> fixed = fixed_in;
  fixed.resize(edges.size(), false);

  // Fixed first, then by inlier support; stable so ties keep input order.
  std::vector<size_t> order(edges.size());
  std::iota(order.begin(), order.end(), size_t{0});
  std::stable_sort(order.begin(), order.end(), [&](size_t a, size_t b) {
    if (fixed[a] != fixed[b])
      return static_cast<bool>(fixed[a]);
    return edges[a].inliers > edges[b].inliers;
  });
  const size_t n_fixed =
      static_cast<size_t>(std::count(fixed.begin(), fixed.end(), true));
  if (opt.pcm_max_edges > 0) {
    const size_t cap =
        std::max(static_cast<size_t>(opt.pcm_max_edges), n_fixed);
    if (order.size() > cap)
      order.resize(cap);
  }
  const int M = static_cast<int>(order.size());
  if (M <= 2) {
    std::sort(order.begin(), order.end());
    return order;
  }

  std::vector<std::vector<char>> adj(M, std::vector<char>(M, 0));
  for (int a = 0; a < M; ++a)
    for (int b = a + 1; b < M; ++b)
      if (detail::pcm_consistent(edges[order[a]], edges[order[b]], seed, opt))
        adj[a][b] = adj[b][a] = 1;

  std::vector<int> fixed_v, cand;
  for (int v = 0; v < M; ++v) {
    if (fixed[order[v]]) {
      fixed_v.push_back(v);
      continue;
    }
    // fixed_v is complete here: fixed edges sort first.
    bool ok = true;
    for (int f : fixed_v)
      ok = ok && adj[v][f];
    if (ok)
      cand.push_back(v);
  }
  std::vector<int> deg(M, 0);
  for (int a : cand)
    for (int b : cand)
      if (a != b && adj[a][b])
        ++deg[a];
  std::stable_sort(cand.begin(), cand.end(),
                   [&](int a, int b) { return deg[a] > deg[b]; });

  std::vector<int> best;
  const int seeds = std::min(static_cast<int>(cand.size()), 64);
  for (int s = 0; s < seeds; ++s) {
    std::vector<int> clique{cand[static_cast<size_t>(s)]};
    for (int v : cand) {
      if (v == clique.front())
        continue;
      bool ok = true;
      for (int u : clique)
        if (!adj[v][u]) {
          ok = false;
          break;
        }
      if (ok)
        clique.push_back(v);
    }
    if (clique.size() > best.size())
      best = std::move(clique);
  }

  std::vector<size_t> out;
  for (int v : fixed_v)
    out.push_back(order[static_cast<size_t>(v)]);
  for (int v : best)
    out.push_back(order[static_cast<size_t>(v)]);
  std::sort(out.begin(), out.end());
  return out;
}
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `nix develop -c cmake --build build --target reusex_unit_tests -j$(nproc) && nix develop -c build/tests/reusex_unit_tests "[loop]"`
Expected: PASS (all old `[loop]` cases unchanged, plus the two new ones).

- [ ] **Step 5: Commit**

```bash
git add libs/reusex/include/slam/LoopClosure.hpp libs/reusex/src/slam/LoopClosure.cpp tests/unit/slam/test_loop_closure.cpp
git commit -m "slam: PCM with fixed inliers (consistent_loop_edge_indices)

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_018dLKtE8qid8y5QVF9b2yri"
```

---

## Task 15: `slam::discover_loop_edges`

**Files:**
- Create: `libs/reusex/include/slam/graph_discover.hpp`, `libs/reusex/src/slam/graph_discover.cpp`
- Test: `tests/unit/slam/test_graph_discover.cpp`

**Interfaces:**
- Consumes: `collect_loop_candidates`/`LoopCandidateOptions` (Task 8), `consistent_loop_edge_indices` (Task 14), `init_pose_graph`/`count_usable_odometry_edges` (Task 7), `stored_to_optical`/`optical_to_stored`/`information_optical_to_stored` (Task 5), `core::pose_graph` helpers.
- Produces: `GraphDiscoverOptions`, `GraphDiscoverResult`, `discover_loop_edges` (as in "Interfaces for the Qt plan").

Algorithm (spec §4.3 "graph discover", with the resolved details):
1. No source enabled → `std::invalid_argument`.
2. No usable odometry and not `dry_run` → `init_pose_graph` (logged at `info`).
3. Seeds: every frame with an initial pose (else its current rigid transform, `warn` with the count); optical seeds = seed · local.
4. `collect_loop_candidates`.
5. Drop a candidate whose node pair already carries a `loop_closure` or `panorama` row of **any** status (one factor per pair, like the front-ends; a tombstone blocks every loop type on the pair). Count rejected-blocked and existing-blocked separately.
6. PCM (`loop_closure.pcm`) over stored active loop edges with a measurement (confirmed = fixed) ∪ the remaining candidates. PCM-rejected candidates are dropped; stored `auto` edges PCM disagrees with are only reported (`warn`), never changed.
7. Unless `dry_run`, upsert survivors in one transaction: type `panorama` for source panorama, else `loop_closure`; transform/information converted to the stored convention; `status=auto`; `seed_disagreement_*` vs the stored seeds.

- [ ] **Step 1: Write the failing test**

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// `rux graph discover` (spec 2026-10-13 §4.3), driven through the external
// edge-file source so no image matching is needed.

#include <catch2/catch_test_macros.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/pose_graph.hpp>
#include <reusex/slam/graph_discover.hpp>
#include <reusex/slam/pose_graph.hpp>

#include "../../support/pose_fixture.hpp"
#include "../../support/temp_path.hpp"

#include <nlohmann/json.hpp>

#include <fstream>
#include <stdexcept>

using reusex::ProjectDB;
using namespace reusex::test_support;
namespace pg = reusex::core::pose_graph;
namespace slam = reusex::slam;

namespace {

Eigen::Matrix4d truth(int k) {
  return pg::matrix_from_row_major(translation_pose(0.5 * k, 0, 0));
}
Eigen::Matrix4d rel(int a, int b) { return truth(a).inverse() * truth(b); }

void make_project(ProjectDB &db, int n = 8) {
  for (int k = 1; k <= n; ++k)
    db.save_sensor_frame(k, make_color(8, 8), make_depth(8, 8, 1.0), cv::Mat(),
                         pg::row_major_from_matrix(truth(k)),
                         make_intrinsics(8, 8, 4, 4, 8, 8), k);
}

void write_edges(const std::filesystem::path &path,
                 const std::vector<std::tuple<int, int, Eigen::Matrix4d>> &es) {
  nlohmann::json j;
  j["schema"] = "reusex.loop_edges.v1";
  j["producer"] = "test";
  j["edges"] = nlohmann::json::array();
  for (const auto &[a, b, T] : es) {
    nlohmann::json e;
    e["node_i"] = a;
    e["node_j"] = b;
    std::vector<double> t;
    for (int r = 0; r < 4; ++r)
      for (int c = 0; c < 4; ++c)
        t.push_back(T(r, c));
    e["T_ij"] = t;
    e["sigma_rot"] = 0.05;
    e["sigma_trans"] = 0.1;
    e["inliers"] = 100;
    j["edges"].push_back(e);
  }
  std::ofstream(path) << j.dump();
}

slam::GraphDiscoverOptions file_only(const std::filesystem::path &file) {
  slam::GraphDiscoverOptions o;
  o.candidates.loop_closure.enable = false;
  o.candidates.loop_edges_file = file.string();
  o.candidates.loop_edges_min_seed_disagreement = 0.0;
  o.candidates.loop_edges_min_seed_disagreement_fraction = 0.0;
  return o;
}

void store_loop(ProjectDB &db, int a, int b, const Eigen::Matrix4d &T,
                const char *status) {
  ProjectDB::PoseGraphEdge e;
  e.from_node_id = a;
  e.to_node_id = b;
  e.edge_type = "loop_closure";
  e.transform = pg::row_major_from_matrix(T);
  e.source = "manual";
  e.status = "auto";
  REQUIRE(db.upsert_pose_graph_edge(e));
  if (std::string(status) != "auto")
    REQUIRE(db.set_pose_graph_edge_status(a, b, "loop_closure", status));
}

} // namespace

TEST_CASE("GraphDiscover_FileEdges_SkipStoredAndTombstonedPairs",
          "[slam][graph_discover]") {
  TempPath tmp("test_discover_skip");
  TempPath file("test_discover_skip", ".json");
  ProjectDB db(tmp.path);
  make_project(db);
  slam::init_pose_graph(db);
  store_loop(db, 2, 7, rel(2, 7), "auto");
  store_loop(db, 1, 7, rel(1, 7), "rejected");
  write_edges(file.path, {{1, 7, rel(1, 7)},
                          {7, 2, rel(7, 2)},
                          {1, 8, rel(1, 8)},
                          {2, 8, rel(2, 8)}});

  const auto r = slam::discover_loop_edges(db, file_only(file.path));
  CHECK(r.skipped_rejected == 1);
  CHECK(r.skipped_existing == 1);
  CHECK(r.pcm_rejected == 0);
  CHECK(r.added == 2);
  CHECK(r.added_file == 2);
  CHECK(db.pose_graph_edge(1, 7, "loop_closure")->status == "rejected");
  for (auto [a, b] : {std::pair{1, 8}, std::pair{2, 8}}) {
    const auto e = db.pose_graph_edge(a, b, "loop_closure");
    REQUIRE(e.has_value());
    CHECK(e->source == "file");
    CHECK(e->status == "auto");
    CHECK(e->information.has_value());
    CHECK(*e->seed_disagreement_m < 1e-9);
    CHECK((pg::matrix_from_row_major(*e->transform) - rel(a, b)).norm() <
          1e-9);
  }
}

TEST_CASE("GraphDiscover_ConfirmedStoredEdge_IsAFixedPcmInlier",
          "[slam][graph_discover]") {
  TempPath tmp("test_discover_fixed");
  TempPath file("test_discover_fixed", ".json");
  ProjectDB db(tmp.path);
  make_project(db);
  slam::init_pose_graph(db);
  Eigen::Matrix4d off = Eigen::Matrix4d::Identity();
  off(0, 3) = 2.0; // three mutually consistent aliasing edges
  write_edges(file.path, {{1, 4, rel(1, 4) * off},
                          {2, 5, rel(2, 5) * off},
                          {3, 6, rel(3, 6) * off}});

  SECTION("with a confirmed true edge they are all rejected") {
    store_loop(db, 1, 5, rel(1, 5), "confirmed");
    const auto r = slam::discover_loop_edges(db, file_only(file.path));
    CHECK(r.pcm_rejected == 3);
    CHECK(r.added == 0);
    CHECK(db.pose_graph_edge(1, 5, "loop_closure")->status == "confirmed");
  }
  SECTION("without it they agree with each other and are added") {
    const auto r = slam::discover_loop_edges(db, file_only(file.path));
    CHECK(r.pcm_rejected == 0);
    CHECK(r.added == 3);
  }
}

TEST_CASE("GraphDiscover_DryRun_WritesNothing", "[slam][graph_discover]") {
  TempPath tmp("test_discover_dry");
  TempPath file("test_discover_dry", ".json");
  ProjectDB db(tmp.path);
  make_project(db);
  write_edges(file.path, {{1, 8, rel(1, 8)}, {2, 8, rel(2, 8)}});
  auto o = file_only(file.path);
  o.dry_run = true;
  const auto r = slam::discover_loop_edges(db, o);
  CHECK(r.candidates == 2);
  CHECK(r.added == 0);
  CHECK_FALSE(r.initialised_graph);
  CHECK(db.list_pose_graph_edges().empty());
}

TEST_CASE("GraphDiscover_NoGraph_InitialisesOdometryFirst",
          "[slam][graph_discover]") {
  TempPath tmp("test_discover_init");
  TempPath file("test_discover_init", ".json");
  ProjectDB db(tmp.path);
  make_project(db);
  write_edges(file.path, {{1, 8, rel(1, 8)}});
  const auto r = slam::discover_loop_edges(db, file_only(file.path));
  CHECK(r.initialised_graph);
  CHECK(slam::count_usable_odometry_edges(db.list_pose_graph_edges()) == 7);
  CHECK(r.added == 1);
}

TEST_CASE("GraphDiscover_NoSource_IsInvalid", "[slam][graph_discover]") {
  TempPath tmp("test_discover_nosource");
  ProjectDB db(tmp.path);
  make_project(db);
  slam::GraphDiscoverOptions o;
  o.candidates.loop_closure.enable = false;
  CHECK_THROWS_AS(slam::discover_loop_edges(db, o), std::invalid_argument);
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `nix develop -c cmake --build build --target reusex_unit_tests -j$(nproc)`
Expected: FAIL — `reusex/slam/graph_discover.hpp: No such file or directory`.

- [ ] **Step 3: Write the header**

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// `rux graph discover` (spec 2026-10-13 §4.3): run the loop-edge front-ends
// seeded from the INITIAL poses, keep only pairs the stored graph does not
// already decide (including tombstones), PCM over stored active edges ∪ new
// candidates with confirmed edges as fixed inliers, and store the survivors
// with status "auto".

#include <reusex/slam/loop_candidates.hpp>

namespace reusex {
class ProjectDB;
}

namespace reusex::slam {

struct GraphDiscoverOptions {
  /// Front-ends; ORB frame-pair detection is ON by default here.
  LoopCandidateOptions candidates{.loop_closure = {.enable = true}};
  /// Report what would be added; write nothing (no graph init either).
  bool dry_run = false;

  bool operator==(const GraphDiscoverOptions &) const = default;
};

struct GraphDiscoverResult {
  int frames = 0;              ///< posed frames searched
  int proposed = 0;            ///< pairs the front-ends tried / edges loaded
  int candidates = 0;          ///< edges the front-ends returned
  int pcm_rejected = 0;        ///< candidates inconsistent with the rest
  int stored_inconsistent = 0; ///< stored auto loop edges PCM disagrees with
  int skipped_existing = 0;    ///< pair already has an active loop edge
  int skipped_rejected = 0;    ///< pair carries a rejected loop edge
  int added = 0;               ///< rows written (0 under dry_run)
  int added_orb = 0;
  int added_panorama = 0;
  int added_file = 0;
  bool initialised_graph = false; ///< odometry was created first
};

/// Opens its own ProjectDB::Transaction: do not call inside one.
/// @throws std::invalid_argument when no source is enabled;
///         std::runtime_error when fewer than 2 frames carry a pose.
GraphDiscoverResult discover_loop_edges(ProjectDB &db,
                                        const GraphDiscoverOptions &options);

} // namespace reusex::slam
```

- [ ] **Step 4: Write the implementation**

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "slam/graph_discover.hpp"

#include "core/ProjectDB.hpp"
#include "core/logging.hpp"
#include "core/pose_graph.hpp"
#include "slam/pose_graph.hpp"
#include "slam/pose_graph_conventions.hpp"

#include <fmt/format.h>

#include <algorithm>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>
#include <utility>

namespace reusex::slam {
namespace {

namespace pg = core::pose_graph;

std::pair<int, int> node_pair(int a, int b) {
  return {std::min(a, b), std::max(a, b)};
}

bool is_loop_type(const std::string &t) {
  return t == pg::kTypeLoopClosure || t == pg::kTypePanorama;
}

} // namespace

GraphDiscoverResult discover_loop_edges(ProjectDB &db,
                                        const GraphDiscoverOptions &options) {
  const auto &c = options.candidates;
  if (!c.loop_closure.enable && !c.panorama_loops.enable &&
      c.loop_edges_file.empty())
    throw std::invalid_argument(
        "graph discover: no edge source selected (ORB loop closure is off, "
        "no --use-panoramas, no --loop-edges)");

  GraphDiscoverResult res;
  if (!options.dry_run &&
      count_usable_odometry_edges(db.list_pose_graph_edges()) == 0) {
    const auto init = init_pose_graph(db);
    res.initialised_graph = true;
    core::info("GraphDiscover: the stored pose graph had no odometry; "
               "initialised {} odometry edges from the current poses "
               "(as 'rux graph init')",
               init.odometry_edges_written);
  }

  // --- Seeds: initial poses (stored convention) and optical seeds ---------
  auto ids = db.sensor_frame_ids();
  std::sort(ids.begin(), ids.end());
  std::vector<int> node_ids;
  std::vector<Eigen::Matrix4d> seed_stored, seed_optical, local;
  int fallback = 0;
  for (int id : ids) {
    std::optional<std::array<double, 16>> p = db.initial_pose(id);
    if (!p) {
      if (!db.has_sensor_frame_pose(id) ||
          !pg::is_rigid_transform(db.sensor_frame_pose(id)))
        continue;
      p = db.sensor_frame_pose(id);
      ++fallback;
    }
    const Eigen::Matrix4d L =
        pg::matrix_from_row_major(db.sensor_frame_intrinsics(id).local_transform);
    const Eigen::Matrix4d S = pg::matrix_from_row_major(*p);
    node_ids.push_back(id);
    seed_stored.push_back(S);
    seed_optical.push_back(S * L);
    local.push_back(L);
  }
  res.frames = static_cast<int>(node_ids.size());
  if (fallback > 0)
    core::warn("GraphDiscover: {} of {} frames have no initial pose; seeded "
               "from their current transform (run 'rux graph init --force' "
               "to snapshot them)",
               fallback, node_ids.size());
  if (node_ids.size() < 2)
    throw std::runtime_error(fmt::format(
        "graph discover: fewer than 2 posed sensor frames ({})",
        node_ids.size()));
  std::map<int, int> index_of;
  for (size_t k = 0; k < node_ids.size(); ++k)
    index_of[node_ids[k]] = static_cast<int>(k);

  // --- Candidates ----------------------------------------------------------
  LoopCandidateReport rep;
  auto cands = collect_loop_candidates(db, node_ids, seed_optical, c, &rep);
  res.proposed = rep.proposed;
  res.candidates = static_cast<int>(cands.size());

  // --- Drop pairs the stored graph already decides (any status) ----------
  const auto stored = db.list_pose_graph_edges();
  std::map<std::pair<int, int>, bool> decided; // pair -> any tombstone
  for (const auto &s : stored)
    if (is_loop_type(s.edge_type)) {
      auto &rejected = decided[node_pair(s.from_node_id, s.to_node_id)];
      rejected = rejected || s.status == pg::kStatusRejected;
    }
  std::vector<LoopCandidate> fresh;
  for (auto &cand : cands) {
    const auto key = node_pair(node_ids[static_cast<size_t>(cand.edge.i)],
                               node_ids[static_cast<size_t>(cand.edge.j)]);
    if (auto it = decided.find(key); it != decided.end()) {
      (it->second ? res.skipped_rejected : res.skipped_existing)++;
      continue;
    }
    fresh.push_back(std::move(cand));
  }

  // --- PCM over stored active loop edges ∪ fresh candidates ---------------
  std::vector<geometry::LoopEdge> pcm_set;
  std::vector<bool> fixed;
  std::vector<int> fresh_index; // -1 for a stored edge
  for (const auto &s : stored) {
    if (!is_loop_type(s.edge_type) || s.status == pg::kStatusRejected ||
        !s.transform)
      continue;
    const auto a = index_of.find(s.from_node_id);
    const auto b = index_of.find(s.to_node_id);
    if (a == index_of.end() || b == index_of.end())
      continue;
    geometry::LoopEdge e;
    e.i = a->second;
    e.j = b->second;
    e.T_ij = stored_to_optical(pg::matrix_from_row_major(*s.transform),
                               local[static_cast<size_t>(e.i)],
                               local[static_cast<size_t>(e.j)]);
    e.inliers = std::numeric_limits<int>::max() / 2; // never capped away
    pcm_set.push_back(e);
    fixed.push_back(s.status == pg::kStatusConfirmed);
    fresh_index.push_back(-1);
  }
  for (size_t k = 0; k < fresh.size(); ++k) {
    pcm_set.push_back(fresh[k].edge);
    fixed.push_back(false);
    fresh_index.push_back(static_cast<int>(k));
  }
  std::vector<bool> keep(fresh.size(), true);
  if (c.loop_closure.pcm && pcm_set.size() > 2) {
    const auto kept_idx =
        geometry::consistent_loop_edge_indices(pcm_set, seed_optical,
                                               c.loop_closure, fixed);
    std::vector<char> kept(pcm_set.size(), 0);
    for (size_t k : kept_idx)
      kept[k] = 1;
    std::map<std::string, std::pair<int, int>> per_source; // rejected, total
    for (size_t n = 0; n < pcm_set.size(); ++n) {
      const int f = fresh_index[n];
      if (f >= 0) {
        auto &[rej, total] = per_source[fresh[static_cast<size_t>(f)].source];
        ++total;
        if (!kept[n]) {
          keep[static_cast<size_t>(f)] = false;
          ++res.pcm_rejected;
          ++rej;
        }
      } else if (!kept[n] && !fixed[n]) {
        ++res.stored_inconsistent;
      }
    }
    // Same line shape the pre-v27 optimize logged; scripts/bench-loop-edges.sh
    // parses it.
    for (const auto &[source, counts] : per_source)
      if (source != "orb")
        core::info("PlaneGraph: PCM kept {} of {} unioned loop edges; "
                   "{} of {} {} edges rejected as inconsistent",
                   kept_idx.size(), pcm_set.size(), counts.first,
                   counts.second, source == "file" ? "external" : "panorama");
    if (res.stored_inconsistent > 0)
      core::warn("GraphDiscover: PCM finds {} stored auto loop edges "
                 "inconsistent with the rest; they are left unchanged — "
                 "review them in the editor or 'rux graph prune' them",
                 res.stored_inconsistent);
  } else if (!c.loop_closure.pcm && !fresh.empty()) {
    core::warn("GraphDiscover: PCM disabled (--loop-no-pcm) — {} new loop "
               "edges are stored WITHOUT a consistency filter",
               fresh.size());
  }

  const int survivors =
      static_cast<int>(std::count(keep.begin(), keep.end(), true));
  if (options.dry_run) {
    core::info("GraphDiscover dry-run: {} proposed, {} matched, {} "
               "PCM-rejected, {} skipped (already stored), {} skipped "
               "(rejected before); {} would be added",
               res.proposed, res.candidates, res.pcm_rejected,
               res.skipped_existing, res.skipped_rejected, survivors);
    return res;
  }

  // --- Store survivors ------------------------------------------------------
  ProjectDB::Transaction tx(db);
  for (size_t k = 0; k < fresh.size(); ++k) {
    if (!keep[k])
      continue;
    const auto &f = fresh[k];
    const auto i = static_cast<size_t>(f.edge.i);
    const auto j = static_cast<size_t>(f.edge.j);
    const Eigen::Matrix4d T = optical_to_stored(f.edge.T_ij, local[i], local[j]);
    ProjectDB::PoseGraphEdge row;
    row.from_node_id = node_ids[i];
    row.to_node_id = node_ids[j];
    row.edge_type = std::string(f.source == "panorama" ? pg::kTypePanorama
                                                       : pg::kTypeLoopClosure);
    row.transform = pg::row_major_from_matrix(T);
    row.information = pg::row_major_from_information(
        information_optical_to_stored(
            pg::information_from_sigmas(f.edge.sigma_rot, f.edge.sigma_trans),
            local[j]));
    row.source = f.source;
    row.status = std::string(pg::kStatusAuto);
    const auto err = pg::transform_error(T, seed_stored[i], seed_stored[j]);
    row.seed_disagreement_m = err.translation_m;
    row.seed_disagreement_deg = err.rotation_deg;
    if (!db.upsert_pose_graph_edge(row))
      continue;
    ++res.added;
    if (f.source == "orb")
      ++res.added_orb;
    else if (f.source == "panorama")
      ++res.added_panorama;
    else
      ++res.added_file;
  }
  tx.commit();
  core::info("GraphDiscover: {} proposed, {} matched, {} PCM-rejected, {} "
             "skipped (already stored), {} skipped (rejected before), {} "
             "added ({} ORB, {} panorama, {} file)",
             res.proposed, res.candidates, res.pcm_rejected,
             res.skipped_existing, res.skipped_rejected, res.added,
             res.added_orb, res.added_panorama, res.added_file);
  return res;
}

} // namespace reusex::slam
```

- [ ] **Step 5: Run tests to verify they pass**

Run: `nix develop -c cmake --build build --target reusex_unit_tests -j$(nproc) && nix develop -c build/tests/reusex_unit_tests "[graph_discover]"`
Expected: PASS (5 cases, 6 with the sections).

- [ ] **Step 6: Commit**

```bash
git add libs/reusex/include/slam/graph_discover.hpp libs/reusex/src/slam/graph_discover.cpp tests/unit/slam/test_graph_discover.cpp
git commit -m "slam: discover_loop_edges — front-ends vs the stored graph, PCM with confirmed inliers

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_018dLKtE8qid8y5QVF9b2yri"
```

---

## Task 16: `slam::prune_pose_graph`

**Files:**
- Create: `libs/reusex/include/slam/graph_prune.hpp`, `libs/reusex/src/slam/graph_prune.cpp`
- Test: `tests/unit/slam/test_graph_prune.cpp`

**Interfaces:**
- Consumes: `ProjectDB::{list_pose_graph_edges, set_pose_graph_edge_status, Transaction}`.
- Produces: `PruneFilter`, `PruneResult`, `prune_pose_graph` (as in "Interfaces for the Qt plan").

Rules: criteria AND together; at least one of `residual_above`, `deviation_above_m`, `deviation_above_deg`, `edge_type`, `source`, `pair` is required; rejected rows never match; confirmed rows match only with `include_confirmed`; a metric criterion never matches a row whose metric is NULL; `pair` matches either direction.

- [ ] **Step 1: Write the failing test**

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// `rux graph prune` (spec §4.3).

#include <catch2/catch_test_macros.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/slam/graph_prune.hpp>

#include "../../support/temp_path.hpp"

#include <stdexcept>
#include <vector>

using reusex::ProjectDB;
using reusex::test_support::TempPath;
namespace slam = reusex::slam;

namespace {

void add(ProjectDB &db, int a, int b, const char *type, const char *source,
         const char *status, double residual, double dev) {
  ProjectDB::PoseGraphEdge e;
  e.from_node_id = a;
  e.to_node_id = b;
  e.edge_type = type;
  e.source = source;
  e.status = status;
  REQUIRE(db.upsert_pose_graph_edge(e));
  const std::vector<ProjectDB::PoseGraphEdgeMetrics> m = {
      {a, b, type, residual, dev, dev * 10.0, std::nullopt, std::nullopt}};
  REQUIRE(db.update_pose_graph_edge_metrics(m) == 1);
}

void make_graph(ProjectDB &db) {
  add(db, 1, 2, "odometry", "odometry_init", "auto", 0.1, 0.01);
  add(db, 3, 7, "loop_closure", "orb", "auto", 50.0, 0.8);
  add(db, 2, 6, "loop_closure", "file", "auto", 60.0, 0.1);
  add(db, 4, 8, "loop_closure", "manual", "confirmed", 99.0, 2.0);
}

} // namespace

TEST_CASE("GraphPrune_CriteriaAndTogether_AndSpareConfirmed",
          "[slam][graph_prune]") {
  TempPath tmp("test_prune_and");
  ProjectDB db(tmp.path);
  make_graph(db);
  slam::PruneFilter f;
  f.residual_above = 10.0;
  f.deviation_above_m = 0.5;
  const auto r = slam::prune_pose_graph(db, f);
  REQUIRE(r.matched.size() == 1);
  CHECK(r.matched[0].from_node_id == 3);
  CHECK(r.rejected == 1);
  CHECK(db.pose_graph_edge(3, 7, "loop_closure")->status == "rejected");
  CHECK(db.pose_graph_edge(2, 6, "loop_closure")->status == "auto");
  CHECK(db.pose_graph_edge(4, 8, "loop_closure")->status == "confirmed");
}

TEST_CASE("GraphPrune_IncludeConfirmed_RejectsThemToo", "[slam][graph_prune]") {
  TempPath tmp("test_prune_confirmed");
  ProjectDB db(tmp.path);
  make_graph(db);
  slam::PruneFilter f;
  f.deviation_above_m = 0.5;
  f.include_confirmed = true;
  CHECK(slam::prune_pose_graph(db, f).rejected == 2);
  CHECK(db.pose_graph_edge(4, 8, "loop_closure")->status == "rejected");
}

TEST_CASE("GraphPrune_DryRun_WritesNothing", "[slam][graph_prune]") {
  TempPath tmp("test_prune_dry");
  ProjectDB db(tmp.path);
  make_graph(db);
  slam::PruneFilter f;
  f.edge_type = "loop_closure";
  f.dry_run = true;
  const auto r = slam::prune_pose_graph(db, f);
  CHECK(r.matched.size() == 2);
  CHECK(r.rejected == 0);
  CHECK(db.pose_graph_edge(3, 7, "loop_closure")->status == "auto");
}

TEST_CASE("GraphPrune_SourceAndPairFilters", "[slam][graph_prune]") {
  TempPath tmp("test_prune_pair");
  ProjectDB db(tmp.path);
  make_graph(db);
  slam::PruneFilter by_pair;
  by_pair.pair = std::pair{7, 3}; // either direction
  CHECK(slam::prune_pose_graph(db, by_pair).rejected == 1);
  slam::PruneFilter by_source;
  by_source.source = "file";
  CHECK(slam::prune_pose_graph(db, by_source).rejected == 1);
  // Already rejected rows never match again.
  CHECK(slam::prune_pose_graph(db, by_source).matched.empty());
}

TEST_CASE("GraphPrune_NoCriterion_IsInvalid", "[slam][graph_prune]") {
  TempPath tmp("test_prune_none");
  ProjectDB db(tmp.path);
  make_graph(db);
  slam::PruneFilter f;
  f.include_confirmed = true;
  CHECK_THROWS_AS(slam::prune_pose_graph(db, f), std::invalid_argument);
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `nix develop -c cmake --build build --target reusex_unit_tests -j$(nproc)`
Expected: FAIL — header not found.

- [ ] **Step 3: Write the header and implementation**

`libs/reusex/include/slam/graph_prune.hpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// `rux graph prune` (spec 2026-10-13 §4.3): mark edges "rejected" by filter.
// Rejected rows stay as tombstones so `graph discover` never re-adds them.

#include <reusex/core/ProjectDB.hpp>

#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace reusex::slam {

/// Criteria AND together; at least one of the optional criteria is required.
struct PruneFilter {
  std::optional<double> residual_above;      ///< residual > x (NULL never matches)
  std::optional<double> deviation_above_m;   ///< deviation_m > x
  std::optional<double> deviation_above_deg; ///< deviation_deg > x
  std::optional<std::string> edge_type;      ///< exact edge_type
  std::optional<std::string> source;         ///< exact source
  std::optional<std::pair<int, int>> pair;   ///< node pair, either direction
  bool include_confirmed = false; ///< confirmed edges match too
  bool dry_run = false;           ///< report only
};

struct PruneResult {
  std::vector<ProjectDB::PoseGraphEdge> matched; ///< as stored before pruning
  int rejected = 0;                              ///< rows marked (0 on dry run)
};

/// Opens its own ProjectDB::Transaction: do not call inside one.
/// @throws std::invalid_argument when @p filter has no criterion.
PruneResult prune_pose_graph(ProjectDB &db, const PruneFilter &filter);

} // namespace reusex::slam
```

`libs/reusex/src/slam/graph_prune.cpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "slam/graph_prune.hpp"

#include "core/logging.hpp"
#include "core/pose_graph.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace reusex::slam {

PruneResult prune_pose_graph(ProjectDB &db, const PruneFilter &f) {
  namespace pg = core::pose_graph;
  if (!f.residual_above && !f.deviation_above_m && !f.deviation_above_deg &&
      !f.edge_type && !f.source && !f.pair)
    throw std::invalid_argument(
        "graph prune: give at least one criterion (--residual-above, "
        "--deviation-above, --deviation-deg-above, --type, --source, --pair)");

  PruneResult r;
  for (const auto &e : db.list_pose_graph_edges()) {
    if (e.status == pg::kStatusRejected)
      continue;
    if (e.status == pg::kStatusConfirmed && !f.include_confirmed)
      continue;
    if (f.residual_above &&
        !(std::isfinite(e.residual) && e.residual > *f.residual_above))
      continue;
    if (f.deviation_above_m &&
        !(e.deviation_m && *e.deviation_m > *f.deviation_above_m))
      continue;
    if (f.deviation_above_deg &&
        !(e.deviation_deg && *e.deviation_deg > *f.deviation_above_deg))
      continue;
    if (f.edge_type && e.edge_type != *f.edge_type)
      continue;
    if (f.source && e.source != *f.source)
      continue;
    if (f.pair && !(std::min(f.pair->first, f.pair->second) ==
                        std::min(e.from_node_id, e.to_node_id) &&
                    std::max(f.pair->first, f.pair->second) ==
                        std::max(e.from_node_id, e.to_node_id)))
      continue;
    r.matched.push_back(e);
  }

  const auto odometry =
      std::count_if(r.matched.begin(), r.matched.end(), [](const auto &e) {
        return e.edge_type == pg::kTypeOdometry;
      });
  if (odometry > 0)
    core::warn("GraphPrune: {} of the {} matched edges are odometry; "
               "rejecting them splits the chain, which optimize bridges from "
               "the initial poses",
               odometry, r.matched.size());
  if (f.dry_run) {
    core::info("GraphPrune dry-run: {} edges would be rejected",
               r.matched.size());
    return r;
  }
  ProjectDB::Transaction tx(db);
  for (const auto &e : r.matched)
    r.rejected += db.set_pose_graph_edge_status(e.from_node_id, e.to_node_id,
                                                e.edge_type,
                                                pg::kStatusRejected)
                      ? 1
                      : 0;
  tx.commit();
  core::info("GraphPrune: rejected {} edges", r.rejected);
  return r;
}

} // namespace reusex::slam
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `nix develop -c cmake --build build --target reusex_unit_tests -j$(nproc) && nix develop -c build/tests/reusex_unit_tests "[graph_prune]"`
Expected: PASS (5 cases).

- [ ] **Step 5: Commit**

```bash
git add libs/reusex/include/slam/graph_prune.hpp libs/reusex/src/slam/graph_prune.cpp tests/unit/slam/test_graph_prune.cpp
git commit -m "slam: prune_pose_graph — reject edges by residual/deviation/type/source/pair

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_018dLKtE8qid8y5QVF9b2yri"
```

---

## Task 17: `JobStage::graph_discover` — parameters, contract, executor, Qt pipeline rows

**Files:**
- Create: `libs/reusex/include/slam/discover_parameters.hpp`, `libs/reusex/src/slam/discover_parameters.cpp`
- Modify: `libs/reusex/include/pipeline/stages.hpp:43-51`, `libs/reusex/src/pipeline/stages.cpp:57-87,563-598`
- Modify: `libs/reusex/src/pipeline/stage_parameters.cpp:272-376`
- Modify: `libs/reusex/include/pipeline/optimize_stage.hpp`
- Modify: `libs/reusex/include/core/stage_contract.hpp`, `libs/reusex/src/core/stage_contract.cpp`, `docs/CONTRACTS.md`
- Modify: `apps/rux/qt/src/pipeline_ui.cpp`, `apps/rux/qt/src/PipelineWorkspace.cpp:488-492,745-756`
- Modify: `tests/unit/pipeline/test_stages.cpp`, `tests/unit/pipeline/test_job_runner.cpp`, `tests/unit/pipeline/test_optimize_stage.cpp`, `tests/unit/pipeline/test_stage_contract_agreement.cpp:58-75`, `tests/unit/rux_app/test_cli_command_roundtrip.cpp:61`
- Create: `tests/unit/slam/test_discover_parameters.cpp`

**Interfaces:**
- Consumes: `GraphDiscoverOptions`, `discover_loop_edges` (Task 15).
- Produces: `JobStage::graph_discover`; `apply_discover_parameters`, `parse_loop_proposal`, `to_string(LoopProposal)`; `pipeline::run_graph_discover_stage(ProjectDB&, const StageContext&)`; `stage_executor_with_optimize()` handling both stages; `core::PipelineStage::graph_discover` (token `"graph_discover"`).

- [ ] **Step 1: Write the failing tests**

`tests/unit/slam/test_discover_parameters.cpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// The graph_discover stage JSON -> GraphDiscoverOptions, and the descriptor
// table's hard-coded defaults pinned against the struct (STANDARDS §4).

#include <catch2/catch_test_macros.hpp>

#include <reusex/pipeline/stage_parameters.hpp>
#include <reusex/slam/discover_parameters.hpp>
#include <reusex/slam/graph_discover.hpp>

#include <stdexcept>
#include <string>
#include <variant>

namespace slam = reusex::slam;
namespace pl = reusex::pipeline;

namespace {
const pl::ParameterDescriptor &descriptor(const std::string &key) {
  for (const auto &d : pl::stage_parameters(pl::JobStage::graph_discover))
    if (d.key == key)
      return d;
  throw std::logic_error("no descriptor " + key);
}
} // namespace

TEST_CASE("DiscoverParameters_Empty_KeepsTheDefaults",
          "[slam][discover_parameters]") {
  slam::GraphDiscoverOptions o;
  slam::apply_discover_parameters(o, "");
  CHECK(o == slam::GraphDiscoverOptions{});
}

TEST_CASE("DiscoverParameters_Keys_Applied", "[slam][discover_parameters]") {
  slam::GraphDiscoverOptions o;
  slam::apply_discover_parameters(
      o, R"({"loop_closure":false,"use_panoramas":true,
             "loop_edges_file":"e.json","loop_proposal":"spatial",
             "loop_min_frame_gap":7,"loop_min_inliers":30,"pcm":false,
             "dry_run":true})");
  CHECK_FALSE(o.candidates.loop_closure.enable);
  CHECK(o.candidates.panorama_loops.enable);
  CHECK(o.candidates.loop_edges_file == "e.json");
  CHECK(o.candidates.loop_closure.proposal ==
        reusex::geometry::LoopProposal::spatial);
  CHECK(o.candidates.loop_closure.min_frame_gap == 7);
  CHECK(o.candidates.loop_closure.min_match_inliers == 30);
  CHECK_FALSE(o.candidates.loop_closure.pcm);
  CHECK(o.dry_run);
}

TEST_CASE("DiscoverParameters_BadInput_InvalidArgument",
          "[slam][discover_parameters]") {
  slam::GraphDiscoverOptions o;
  for (const char *bad : {"[1]", "{oops", R"({"loop_min_inliers":"many"})",
                          R"({"loop_proposal":"psychic"})"}) {
    INFO(bad);
    CHECK_THROWS_AS(slam::apply_discover_parameters(o, bad),
                    std::invalid_argument);
  }
}

TEST_CASE("DiscoverParameters_DescriptorDefaults_MirrorTheStruct",
          "[slam][discover_parameters]") {
  const slam::GraphDiscoverOptions d;
  const auto &lc = d.candidates.loop_closure;
  CHECK(std::get<bool>(descriptor("loop_closure").default_value) == lc.enable);
  CHECK(std::get<bool>(descriptor("use_panoramas").default_value) ==
        d.candidates.panorama_loops.enable);
  CHECK(std::get<std::string>(descriptor("loop_edges_file").default_value) ==
        d.candidates.loop_edges_file);
  CHECK(std::get<std::string>(descriptor("loop_proposal").default_value) ==
        std::string(slam::to_string(lc.proposal)));
  CHECK(std::get<long long>(descriptor("loop_min_frame_gap").default_value) ==
        lc.min_frame_gap);
  CHECK(std::get<long long>(descriptor("loop_min_inliers").default_value) ==
        lc.min_match_inliers);
  CHECK(std::get<bool>(descriptor("pcm").default_value) == lc.pcm);
  CHECK(std::get<bool>(descriptor("dry_run").default_value) == d.dry_run);
}
```

Append to `tests/unit/pipeline/test_optimize_stage.cpp`:

```cpp
TEST_CASE("GraphDiscoverStage_BadParameters_RefusedAsInvalid",
          "[pipeline][optimize_stage]") {
  TempPath project("test_discover_stage_bad");
  {
    reusex::ProjectDB db(project.path);
  }
  StageContext ctx;
  ctx.project = project.path;
  ctx.stage = JobStage::graph_discover;
  for (const std::string params :
       {"[1]", R"({"loop_proposal":"psychic"})",
        R"({"loop_closure":false})"}) { // last: no source selected
    ctx.parameters = params;
    const StageResult r = stage_executor_with_optimize()(ctx);
    INFO(params << " -> " << r.message);
    CHECK_FALSE(r.ok);
    CHECK(r.invalid_input);
  }
}

TEST_CASE("GraphDiscoverStage_DefaultExecutor_Refuses",
          "[pipeline][optimize_stage]") {
  TempPath project("test_discover_stage_default");
  {
    reusex::ProjectDB db(project.path);
  }
  StageContext ctx;
  ctx.project = project.path;
  ctx.stage = JobStage::graph_discover;
  CHECK_FALSE(default_stage_executor()(ctx).ok);
}
```

In `tests/unit/pipeline/test_stages.cpp`, at the end of `PipelineLogName_RunnableStages_MatchesCliNames` add:

```cpp
  CHECK(pipeline_log_name(JobStage::graph_discover) == "graph_discover");
  CHECK(to_string(JobStage::graph_discover) == "graph_discover");
```

In `tests/unit/pipeline/test_job_runner.cpp` (after the annotate cancellation check) add `CHECK_FALSE(stage_supports_cancellation(JobStage::graph_discover));`.

In `tests/unit/pipeline/test_stage_contract_agreement.cpp` `contract_of()` add:

```cpp
  case pipeline::JobStage::annotate:
    return core::PipelineStage::annotate;
  case pipeline::JobStage::graph_discover:
    return core::PipelineStage::graph_discover;
```

In `tests/unit/rux_app/test_cli_command_roundtrip.cpp:61` add `"graph_discover"` to `kNoCliMapping` with a comment `// until Task 18 adds rux graph discover` (Task 18 removes it again).

- [ ] **Step 2: Run tests to verify they fail**

Run: `nix develop -c cmake --build build --target reusex_unit_tests -j$(nproc)`
Expected: compile FAIL — `'graph_discover' is not a member of 'JobStage'`.

- [ ] **Step 3: The JSON reader**

`libs/reusex/include/slam/discover_parameters.hpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// The graph_discover stage's JSON parameters -> GraphDiscoverOptions, in ONE
// place (mirrors slam/optimize_parameters.hpp): `rux graph discover`, the Qt
// client's in-process stage and ruxd read them through this function.

#include <reusex/slam/graph_discover.hpp>

#include <string_view>

namespace reusex::slam {

/// Keys: loop_closure, use_panoramas, loop_edges_file, loop_proposal,
/// loop_min_frame_gap, loop_min_inliers, pcm, dry_run. Absent or null keys
/// keep the value already in @p options; unknown keys are ignored.
/// @throws std::invalid_argument when @p parameters is not a JSON object, a
///         known key has the wrong type, or loop_proposal is unknown.
void apply_discover_parameters(GraphDiscoverOptions &options,
                               std::string_view parameters);

/// "auto" | "spatial" | "appearance" | "exhaustive".
/// @throws std::invalid_argument for anything else.
geometry::LoopProposal parse_loop_proposal(std::string_view name);
std::string_view to_string(geometry::LoopProposal proposal);

} // namespace reusex::slam
```

`libs/reusex/src/slam/discover_parameters.cpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "slam/discover_parameters.hpp"

#include <nlohmann/json.hpp>

#include <stdexcept>
#include <string>

namespace reusex::slam {

geometry::LoopProposal parse_loop_proposal(std::string_view name) {
  using geometry::LoopProposal;
  if (name == "auto")
    return LoopProposal::automatic;
  if (name == "spatial")
    return LoopProposal::spatial;
  if (name == "appearance")
    return LoopProposal::appearance;
  if (name == "exhaustive")
    return LoopProposal::exhaustive;
  throw std::invalid_argument("unknown loop_proposal '" + std::string(name) +
                              "' (auto, spatial, appearance, exhaustive)");
}

std::string_view to_string(geometry::LoopProposal proposal) {
  using geometry::LoopProposal;
  switch (proposal) {
  case LoopProposal::automatic:
    return "auto";
  case LoopProposal::spatial:
    return "spatial";
  case LoopProposal::appearance:
    return "appearance";
  case LoopProposal::exhaustive:
    return "exhaustive";
  }
  return "auto";
}

void apply_discover_parameters(GraphDiscoverOptions &options,
                               std::string_view parameters) {
  nlohmann::json params = nlohmann::json::object();
  if (!parameters.empty()) {
    params = nlohmann::json::parse(parameters, nullptr,
                                   /*allow_exceptions=*/false);
    if (params.is_discarded() || !params.is_object())
      throw std::invalid_argument(
          "graph_discover parameters must be a JSON object");
  }
  auto get = [&](const char *key, auto fallback) {
    auto it = params.find(key);
    if (it == params.end() || it->is_null())
      return fallback;
    try {
      return it->template get<decltype(fallback)>();
    } catch (const nlohmann::json::exception &e) {
      throw std::invalid_argument(std::string("graph_discover parameter '") +
                                  key + "': " + e.what());
    }
  };
  auto &c = options.candidates;
  c.loop_closure.enable = get("loop_closure", c.loop_closure.enable);
  c.panorama_loops.enable = get("use_panoramas", c.panorama_loops.enable);
  c.loop_edges_file = get("loop_edges_file", c.loop_edges_file);
  const std::string proposal =
      get("loop_proposal", std::string(to_string(c.loop_closure.proposal)));
  c.loop_closure.proposal = parse_loop_proposal(proposal);
  c.loop_closure.min_frame_gap =
      get("loop_min_frame_gap", c.loop_closure.min_frame_gap);
  c.loop_closure.min_match_inliers =
      get("loop_min_inliers", c.loop_closure.min_match_inliers);
  c.loop_closure.pcm = get("pcm", c.loop_closure.pcm);
  options.dry_run = get("dry_run", options.dry_run);
}

} // namespace reusex::slam
```

- [ ] **Step 4: The stage identity, parameters and contract**

`pipeline/stages.hpp` enum: append after `annotate,`:

```cpp
  graph_discover, ///< find loop edges and store them in the pose graph
```

`pipeline/stages.cpp`: `constexpr std::array<StageName, 8> kStages{{` and add the row after `annotate`:

```cpp
    // Not cancellable: ORB matching has no cooperative cancel hook. Like
    // optimize it needs reusex_slam, so dispatch() refuses it and the
    // header-only pipeline/optimize_stage.hpp executor runs it (#464).
    {JobStage::graph_discover, "graph_discover", "graph_discover",
     core::PipelineStage::graph_discover, false},
```

and in `dispatch()` before the closing of the switch:

```cpp
  case JobStage::graph_discover:
    return StageResult::failure(
        "the graph_discover stage requires the slam executor "
        "(pipeline::stage_executor_with_optimize, see #464)");
```

`stage_parameters.cpp`: add in the anonymous namespace after `annotate_parameters()`:

```cpp
const std::vector<ParameterDescriptor> &graph_discover_parameters() {
  // Defaults mirror slam::GraphDiscoverOptions (slam/graph_discover.hpp).
  // reusex_pipeline must not link reusex_slam (#464), so they are hard-coded
  // here and pinned by tests/unit/slam/test_discover_parameters.cpp.
  static const std::vector<ParameterDescriptor> table{
      boolean("loop_closure", "ORB loop closure",
              "Match temporally distant frame pairs (ORB features + depth, "
              "RANSAC) and propose loop edges.",
              true),
      boolean("use_panoramas", "Panorama loop edges",
              "Derive loop edges from the project's 360 panoramas (#236).",
              false),
      text("loop_edges_file", "External edge file",
           "Path to a reusex.loop_edges.v1 JSON file from an out-of-process "
           "matcher. Empty means none.",
           ""),
      text("loop_proposal", "Candidate proposal",
           "How candidate pairs are proposed: auto, appearance, spatial or "
           "exhaustive.",
           "auto"),
      integer("loop_min_frame_gap", "Min frame gap",
              "Skip pairs closer than this in frame index (odometry covers "
              "them).",
              50, 1.0, 100000.0),
      integer("loop_min_inliers", "Min RANSAC inliers",
              "Reject an ORB pair below this many RANSAC inliers.", 60, 3.0,
              100000.0),
      boolean("pcm", "Consistency filter (PCM)",
              "Keep only loop edges consistent with each other and with the "
              "stored active edges; confirmed edges are fixed inliers.",
              true),
      boolean("dry_run", "Dry run",
              "Report what would be added without writing edges.", false),
  };
  return table;
}
```

and in `stage_parameters()` add `case JobStage::graph_discover: return graph_discover_parameters();`.

`core/stage_contract.hpp`: add `graph_discover, ///< find loop edges into the pose graph (rux graph discover)` after `graph_init,`. `stage_contract.cpp`: after the `graph_init` row:

```cpp
      {PipelineStage::graph_discover,
       "graph_discover",
       {},
       "rux graph discover",
       "find wide-baseline loop edges and store them in the pose graph",
       {{{"sensor_frames"}, /*min_rows=*/2}},
       {"pose_graph_edges"}},
```

`docs/CONTRACTS.md`, new section after `graph_init`:

```markdown
### `graph_discover` (`rux graph discover`)

Run the loop-edge front-ends (ORB frame pairs, 360 panoramas, an external
`reusex.loop_edges.v1` file) seeded from the initial poses, skip every pair the
stored graph already decides (including rejected tombstones), PCM over stored
active edges ∪ new candidates (confirmed edges are fixed inliers), and store the
survivors with status `auto`. Initialises odometry first when there is none.

| | |
|---|---|
| Consumes | `sensor_frames` |
| Produces | `pose_graph_edges` |
| Options  | `GraphDiscoverOptions` (`libs/reusex/include/slam/graph_discover.hpp`) |
| Checks   | ≥2 stored frames; at least one source enabled |
```

and update the pipeline order line to `import → [graph_init] → [graph_discover] → (optimize|register) → …`.

- [ ] **Step 5: The executor**

In `pipeline/optimize_stage.hpp` add includes `<reusex/slam/discover_parameters.hpp>` and `<reusex/slam/graph_discover.hpp>`, and before `stage_executor_with_optimize()`:

```cpp
/// Run `graph_discover` on an open project through the same JSON reader
/// `rux graph discover` uses (slam::apply_discover_parameters).
inline StageResult run_graph_discover_stage(ProjectDB &db,
                                            const StageContext &ctx) {
  slam::GraphDiscoverOptions options;
  try {
    slam::apply_discover_parameters(options, ctx.parameters);
  } catch (const std::exception &e) {
    return StageResult::invalid(e.what());
  }
  try {
    const auto r = slam::discover_loop_edges(db, options);
    const std::string summary = fmt::format(
        "{}{} proposed, {} matched, {} PCM-rejected, {} already stored, {} "
        "rejected before, {} added",
        options.dry_run ? "dry run: " : "", r.proposed, r.candidates,
        r.pcm_rejected, r.skipped_existing, r.skipped_rejected, r.added);
    if (options.dry_run)
      return StageResult::success(summary);
    return StageResult::success(
        summary, {{"table", "pose_graph_edges", static_cast<int64_t>(r.added)}});
  } catch (const std::invalid_argument &e) {
    return StageResult::invalid(e.what());
  } catch (const std::exception &e) {
    return StageResult::failure(
        fmt::format("loop-edge discovery failed: {}", e.what()));
  }
}
```

and make the executor dispatch both stages:

```cpp
inline StageExecutor stage_executor_with_optimize() {
  auto default_exec = default_stage_executor();
  return [default_exec =
              std::move(default_exec)](const StageContext &ctx) -> StageResult {
    if (ctx.stage == JobStage::optimize ||
        ctx.stage == JobStage::graph_discover) {
      try {
        ProjectDB db(ctx.project, /*readOnly=*/false);
        return ctx.stage == JobStage::optimize
                   ? run_optimize_stage(db, ctx)
                   : run_graph_discover_stage(db, ctx);
      } catch (const std::exception &e) {
        return StageResult::failure(fmt::format(
            "could not open project '{}': {}", ctx.project.string(), e.what()));
      }
    }
    return default_exec(ctx);
  };
}
```

Update the header comment's first line to "The in-process `optimize` and `graph_discover` stages …".

- [ ] **Step 6: Qt pipeline rows**

`apps/rux/qt/src/pipeline_ui.cpp`:
- `stage_name_da`: `case pl::JobStage::graph_discover: return "Find kanter";`
- `stage_blurb_da`: `case pl::JobStage::graph_discover: return "Find nye løkkekanter (ORB, 360°-panoramaer, kantfil) og gem dem i posegrafen.";`
- `labels` map, after the `{5, …}` rows:

```cpp
      {{7, "loop_closure"}, "ORB-løkkelukning"},
      {{7, "use_panoramas"}, "Brug 360°-panoramaer"},
      {{7, "loop_edges_file"}, "Kantfil (JSON)"},
      {{7, "loop_proposal"}, "Kandidatforslag"},
      {{7, "loop_min_frame_gap"}, "Mindste billedafstand"},
      {{7, "loop_min_inliers"}, "Mindste inliers"},
      {{7, "pcm"}, "Konsistensfilter (PCM)"},
      {{7, "dry_run"}, "Prøvekørsel — skriv ikke kanterne"},
```

- `contract_stage`: `case pl::JobStage::graph_discover: return P::graph_discover;`

`apps/rux/qt/src/PipelineWorkspace.cpp`:
- row caption (:488-492):

```cpp
    const QString cmd =
        s == pl::JobStage::optimize         ? QString("optimize")
        : s == pl::JobStage::graph_discover ? QString("graph discover")
                                            : QString("create %1").arg(qs(name));
    auto *c = new QLabel(QString("rux %1").arg(cmd));
```

- pending-edits guard (:746): `if ((stage_ == pl::JobStage::optimize || stage_ == pl::JobStage::graph_discover) && pending_edits_ && pending_edits_() > 0) {` and change the informative text to `"Optimér poser og Find kanter bruger kun de gemte kanter. Gem dem i Database først, eller kør uden dem."`

- [ ] **Step 7: Run tests to verify they pass**

```bash
nix develop -c cmake --build build -j$(nproc)
nix develop -c build/tests/reusex_unit_tests "[discover_parameters],[pipeline],[contract],[validate],[rux_qt]"
nix develop -c build/tests/reusex_unit_tests_vision "[rux_app]"
```

Expected: PASS — including `StageParameters_EveryRunnableStage_DescribesWellFormedParameters`, `JobStage_NameRoundTrip_*`, `ContractsDoc_StageSections_MatchContractTable`, and the round-trip tests (graph_discover skipped via `kNoCliMapping` for now).

- [ ] **Step 8: Commit**

```bash
git add libs/reusex/include/slam/discover_parameters.hpp libs/reusex/src/slam/discover_parameters.cpp \
        libs/reusex/include/pipeline/stages.hpp libs/reusex/src/pipeline/stages.cpp \
        libs/reusex/src/pipeline/stage_parameters.cpp libs/reusex/include/pipeline/optimize_stage.hpp \
        libs/reusex/include/core/stage_contract.hpp libs/reusex/src/core/stage_contract.cpp docs/CONTRACTS.md \
        apps/rux/qt/src/pipeline_ui.cpp apps/rux/qt/src/PipelineWorkspace.cpp \
        tests/unit/slam/test_discover_parameters.cpp tests/unit/pipeline/test_optimize_stage.cpp \
        tests/unit/pipeline/test_stages.cpp tests/unit/pipeline/test_job_runner.cpp \
        tests/unit/pipeline/test_stage_contract_agreement.cpp tests/unit/rux_app/test_cli_command_roundtrip.cpp
git commit -m "pipeline: JobStage::graph_discover via the slam executor; contract and Qt rows

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_018dLKtE8qid8y5QVF9b2yri"
```

---

## Task 18: `rux graph discover` / `rux graph prune`, CLI round-trip, scripts and docs

**Files:**
- Create: `apps/rux/include/graph/discover.hpp`, `apps/rux/src/graph/discover.cpp`, `apps/rux/include/graph/prune.hpp`, `apps/rux/src/graph/prune.cpp`
- Modify: `apps/rux/src/graph.cpp`
- Modify: `apps/rux/qt/src/core/cli_command.cpp:80-86`
- Modify: `tests/unit/rux_app/test_cli_command_roundtrip.cpp` (setup, `changed_value`, `kNoCliMapping`)
- Modify: `tests/unit/rux_app/test_graph_cli.cpp` (append)
- Modify: `scripts/bench-loop-edges.sh:73-80,135-140`, `scripts/bench-arkitscenes-drift.sh:158,171`, `scripts/create_pseudo_gt.sh:11-15,276-283,333`, `scripts/spatial_loop_validate.sh:12,256-262`, `tools/loop_edges/README.md:8,130-131`, `tools/loop_edges/export_loop_edges.py:6,19,307-308`, `tools/loop_edges/propose_targeted_revisits.py:6,39,151-152`
- Modify: `CLAUDE.md` (CLI table, slam bullets), `docs/DIRECTION.md:104`

**Interfaces:**
- Consumes: `GraphDiscoverOptions`, `discover_loop_edges`, `apply_discover_parameters`, `parse_loop_proposal` (Tasks 15/17), `PruneFilter`, `prune_pose_graph` (Task 16), `optimize_moved_flags()` (Task 8).
- Produces:

```cpp
struct SubcommandGraphDiscoverOptions { /* one field per flag, defaults from GraphDiscoverOptions{} */ };
reusex::slam::GraphDiscoverOptions graph_discover_options(const SubcommandGraphDiscoverOptions &opt);
std::string graph_discover_stage_parameters(const SubcommandGraphDiscoverOptions &opt);
void setup_subcommand_graph_discover(CLI::App &parent, std::shared_ptr<RuxOptions> global_opt);
struct SubcommandGraphPruneOptions { ... };
void setup_subcommand_graph_prune(CLI::App &parent, std::shared_ptr<RuxOptions> global_opt);
```

- [ ] **Step 1: Write the failing tests**

Append to `tests/unit/rux_app/test_graph_cli.cpp` (add `#include <graph/discover.hpp>`, `#include <optimize.hpp>`, `#include <create/stage_bridge.hpp>`, `#include <reusex/slam/graph_discover.hpp>`):

```cpp
TEST_CASE("GraphCli_DiscoverFlagDefaults_MatchTheLibrary",
          "[rux_app][cli][graph]") {
  // STANDARDS §4: the CLI defaults mirror GraphDiscoverOptions{} through its
  // memberwise operator== (nested LoopClosure/PanoramaLoop options included).
  CHECK(graph_discover_options(SubcommandGraphDiscoverOptions{}) ==
        reusex::slam::GraphDiscoverOptions{});
}

TEST_CASE("GraphCli_EveryFlagMovedFromOptimize_ParsesOnDiscover",
          "[rux_app][cli][graph]") {
  for (const auto &[flag, takes_value] : optimize_moved_flags()) {
    if (flag == "--loop-closure")
      continue; // ORB is on by default in discover; its switch is --no-loop-closure
    INFO(flag);
    CLI::App app{"rux"};
    auto global = std::make_shared<RuxOptions>();
    app.add_option("-p,--project", global->project_db);
    setup_subcommand_graph(app, global);
    std::string stage;
    rux::set_stage_params_sink(
        [&](std::string_view s, const std::string &) { stage = s; });
    std::vector<std::string> args = {"graph", "discover", flag};
    if (takes_value)
      args.push_back(flag == "--loop-proposal" ? "spatial" : "1");
    std::vector<std::string> reversed(args.rbegin(), args.rend());
    CHECK_NOTHROW(app.parse(reversed));
    rux::set_stage_params_sink({});
    CHECK(stage == "graph_discover");
  }
}

TEST_CASE("GraphCli_Prune_RequiresACriterionAndRejects",
          "[rux_app][cli][graph]") {
  TempPath tmp("test_graph_cli_prune");
  make_frames(tmp.path, 4);
  CHECK(run_rux(tmp.path, {"graph", "init"}));
  CHECK_FALSE(run_rux(tmp.path, {"graph", "prune"}));
  CHECK(run_rux(tmp.path, {"graph", "prune", "--pair", "2,3", "--dry-run"}));
  {
    reusex::ProjectDB db(tmp.path, /*readOnly=*/true);
    CHECK(db.pose_graph_edge(2, 3, "odometry")->status == "auto");
  }
  CHECK(run_rux(tmp.path, {"graph", "prune", "--pair", "3,2"}));
  reusex::ProjectDB db(tmp.path, /*readOnly=*/true);
  CHECK(db.pose_graph_edge(2, 3, "odometry")->status == "rejected");
  CHECK_FALSE(run_rux(tmp.path, {"graph", "prune", "--pair", "nonsense"}));
}
```

In `tests/unit/rux_app/test_cli_command_roundtrip.cpp`:
- add `#include <graph.hpp>`; in `parse_through_cli` add `setup_subcommand_graph(app, global);` after `setup_subcommand_optimize(app, global);`;
- in `changed_value`, string branch, add `if (d.key == "loop_proposal") return "spatial";`;
- remove `"graph_discover"` from `kNoCliMapping` (and the Task 17 comment).

- [ ] **Step 2: Run tests to verify they fail**

Run: `nix develop -c cmake --build build --target reusex_unit_tests_vision -j$(nproc)`
Expected: FAIL — `graph/discover.hpp: No such file or directory`.

- [ ] **Step 3: `graph discover`**

`apps/rux/include/graph/discover.hpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "../global-params.hpp"

// Included to MIRROR the library defaults (docs/STANDARDS.md §4).
#include <reusex/slam/graph_discover.hpp>

#include <CLI/CLI.hpp>
#include <memory>
#include <string>

namespace graph_discover_defaults {
inline const reusex::slam::GraphDiscoverOptions kOptions{};
inline const auto &kLoop = kOptions.candidates.loop_closure;
inline const auto &kPano = kOptions.candidates.panorama_loops;
} // namespace graph_discover_defaults

/// `rux graph discover` flags; defaults mirror slam::GraphDiscoverOptions.
struct SubcommandGraphDiscoverOptions {
  bool no_loop_closure = !graph_discover_defaults::kLoop.enable;
  std::string loop_proposal = "auto";
  int loop_min_frame_gap = graph_discover_defaults::kLoop.min_frame_gap;
  float loop_max_distance = graph_discover_defaults::kLoop.max_candidate_distance;
  float loop_max_view_angle = graph_discover_defaults::kLoop.max_view_angle;
  int loop_max_candidates = graph_discover_defaults::kLoop.max_candidates_per_frame;
  int loop_min_inliers = graph_discover_defaults::kLoop.min_match_inliers;
  int loop_max_features = graph_discover_defaults::kLoop.max_features;
  float loop_ratio_test = graph_discover_defaults::kLoop.ratio_test;
  float loop_ransac_inlier_dist = graph_discover_defaults::kLoop.ransac_inlier_dist;
  float loop_max_seed_disagreement = graph_discover_defaults::kLoop.max_seed_disagreement;
  float loop_min_seed_disagreement = graph_discover_defaults::kLoop.min_seed_disagreement;
  float loop_min_seed_disagreement_fraction =
      graph_discover_defaults::kLoop.min_seed_disagreement_fraction;
  bool loop_no_pcm = !graph_discover_defaults::kLoop.pcm;
  bool use_panoramas = graph_discover_defaults::kPano.enable;
  int pano_max_frames = graph_discover_defaults::kPano.max_frames;
  int pano_min_inliers = graph_discover_defaults::kPano.min_frame_inliers;
  int pano_max_edges = graph_discover_defaults::kPano.max_edges_per_panorama;
  int pano_n_yaw = graph_discover_defaults::kPano.n_yaw;
  int pano_max_features = graph_discover_defaults::kPano.max_features;
  double pano_max_distance = graph_discover_defaults::kPano.max_pano_distance;
  std::string loop_edges_file = graph_discover_defaults::kOptions.candidates.loop_edges_file;
  double loop_edges_min_disagreement =
      graph_discover_defaults::kOptions.candidates.loop_edges_min_seed_disagreement;
  double loop_edges_min_disagreement_fraction =
      graph_discover_defaults::kOptions.candidates.loop_edges_min_seed_disagreement_fraction;
  unsigned seed = graph_discover_defaults::kLoop.seed;
  bool dry_run = graph_discover_defaults::kOptions.dry_run;
};

/// Every flag -> the library options (what `rux graph discover` runs with).
reusex::slam::GraphDiscoverOptions
graph_discover_options(const SubcommandGraphDiscoverOptions &opt);

/// The flags the in-process graph_discover stage reads, as its JSON.
std::string
graph_discover_stage_parameters(const SubcommandGraphDiscoverOptions &opt);

void setup_subcommand_graph_discover(CLI::App &parent,
                                     std::shared_ptr<RuxOptions> global_opt);
int run_subcommand_graph_discover(const SubcommandGraphDiscoverOptions &opt,
                                  const RuxOptions &global_opt);
```

`apps/rux/src/graph/discover.cpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "graph/discover.hpp"
#include "create/stage_bridge.hpp"
#include "exit_status.hpp"
#include "stage_prerequisites.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/stage_contract.hpp>
#include <reusex/slam/discover_parameters.hpp>

#include <spdlog/spdlog.h>

#include <exception>
#include <stdexcept>

void setup_subcommand_graph_discover(CLI::App &parent,
                                     std::shared_ptr<RuxOptions> global_opt) {
  auto opt = std::make_shared<SubcommandGraphDiscoverOptions>();
  auto *sub = parent.add_subcommand(
      "discover", "Find wide-baseline loop edges and add them to the graph");
  sub->footer(R"(
DESCRIPTION:
  Runs the loop-edge front-ends seeded from the INITIAL poses — ORB frame-pair
  matching (on by default), 360-panorama resections (--use-panoramas) and an
  external matcher file (--loop-edges) — then skips every frame pair the
  stored graph already decides (an existing edge, or one you rejected), keeps
  the largest set consistent with the stored edges (PCM; confirmed edges are
  fixed inliers) and stores the survivors with status 'auto'. Run
  'rux optimize' afterwards to use them.

EXAMPLES:
  rux graph discover                               # ORB loop closure
  rux graph discover --use-panoramas               # ...plus panorama edges
  rux graph discover --no-loop-closure --loop-edges edges.json
  rux graph discover --dry-run                     # report only
)");

  sub->add_flag("--no-loop-closure", opt->no_loop_closure,
                "Do not run ORB frame-pair loop detection");
  sub->add_option("--loop-proposal", opt->loop_proposal,
                  "Loop-candidate proposal: auto (exhaustive on small scans, "
                  "else appearance), appearance, spatial or exhaustive")
      ->default_val(opt->loop_proposal)
      ->check(CLI::IsMember({"auto", "appearance", "spatial", "exhaustive"}));
  sub->add_option("--loop-min-frame-gap", opt->loop_min_frame_gap,
                  "Skip pairs closer than this in frame index")
      ->default_val(opt->loop_min_frame_gap);
  sub->add_option("--loop-max-distance", opt->loop_max_distance,
                  "Max seed camera-centre distance to propose a loop pair (m)")
      ->default_val(opt->loop_max_distance);
  sub->add_option("--loop-max-view-angle", opt->loop_max_view_angle,
                  "Max viewing-direction angle to propose a loop pair (deg)")
      ->default_val(opt->loop_max_view_angle);
  sub->add_option("--loop-max-candidates", opt->loop_max_candidates,
                  "Max loop candidates per frame (nearest first)")
      ->default_val(opt->loop_max_candidates);
  sub->add_option("--loop-min-inliers", opt->loop_min_inliers,
                  "Reject a loop edge below this many RANSAC inliers")
      ->default_val(opt->loop_min_inliers);
  sub->add_option("--loop-max-features", opt->loop_max_features,
                  "ORB features per frame for loop matching")
      ->default_val(opt->loop_max_features);
  sub->add_option("--loop-ratio-test", opt->loop_ratio_test,
                  "Lowe ratio threshold for loop descriptor matches")
      ->default_val(opt->loop_ratio_test);
  sub->add_option("--loop-ransac-inlier-dist", opt->loop_ransac_inlier_dist,
                  "3D-3D RANSAC inlier threshold for loop relative pose (m)")
      ->default_val(opt->loop_ransac_inlier_dist);
  sub->add_option("--loop-max-seed-disagreement",
                  opt->loop_max_seed_disagreement,
                  "Reject a loop edge whose translation disagrees with the "
                  "seed by more than this (m, <=0 disables)")
      ->default_val(opt->loop_max_seed_disagreement);
  sub->add_option("--loop-min-seed-disagreement",
                  opt->loop_min_seed_disagreement,
                  "Absolute floor (m) of the lower seed-disagreement gate")
      ->default_val(opt->loop_min_seed_disagreement);
  sub->add_option("--loop-min-seed-disagreement-fraction",
                  opt->loop_min_seed_disagreement_fraction,
                  "Scale-relative part of that gate (fraction of the "
                  "trajectory extent, #339)")
      ->default_val(opt->loop_min_seed_disagreement_fraction);
  sub->add_flag("--loop-no-pcm", opt->loop_no_pcm,
                "Disable pairwise-consistency (PCM) filtering");
  sub->add_flag("--use-panoramas", opt->use_panoramas,
                "Derive loop edges from the project's 360 panoramas (#236)");
  sub->add_option("--pano-max-frames", opt->pano_max_frames,
                  "Max sensor frames resected against each panorama")
      ->default_val(opt->pano_max_frames);
  sub->add_option("--pano-min-inliers", opt->pano_min_inliers,
                  "Accept a per-frame resection above this many PnP inliers")
      ->default_val(opt->pano_min_inliers);
  sub->add_option("--pano-max-edges", opt->pano_max_edges,
                  "Max loop edges contributed per panorama")
      ->default_val(opt->pano_max_edges);
  sub->add_option("--pano-slices", opt->pano_n_yaw,
                  "Perspective slices rendered around the panorama equator")
      ->default_val(opt->pano_n_yaw);
  sub->add_option("--pano-max-features", opt->pano_max_features,
                  "ORB features per panorama slice / sensor frame")
      ->default_val(opt->pano_max_features);
  sub->add_option("--pano-max-distance", opt->pano_max_distance,
                  "Reject a resection placing the panorama further than this "
                  "from the frame (m, <=0 disables)")
      ->default_val(opt->pano_max_distance);
  sub->add_option("--loop-edges", opt->loop_edges_file,
                  "Externally computed loop edges (reusex.loop_edges.v1 JSON)");
  sub->add_option("--loop-edges-min-disagreement",
                  opt->loop_edges_min_disagreement,
                  "Drop file edges agreeing with the seed within this (m); "
                  "absolute floor of the gate")
      ->default_val(opt->loop_edges_min_disagreement);
  sub->add_option("--loop-edges-min-disagreement-fraction",
                  opt->loop_edges_min_disagreement_fraction,
                  "Scale-relative part of the file-edge gate (#339)")
      ->default_val(opt->loop_edges_min_disagreement_fraction);
  sub->add_option("--seed", opt->seed, "RANSAC seed (determinism)")
      ->default_val(opt->seed);
  sub->add_flag("--dry-run", opt->dry_run,
                "Report what would be added without writing edges");

  sub->callback([opt, global_opt]() {
    if (rux::capture_stage_params("graph_discover",
                                  graph_discover_stage_parameters(*opt)))
      return;
    rux::finish(run_subcommand_graph_discover(*opt, *global_opt));
  });
}

std::string
graph_discover_stage_parameters(const SubcommandGraphDiscoverOptions &opt) {
  // The subset the in-process stage reads (pipeline/stage_parameters.cpp).
  return rux::StageParams()
      .set("loop_closure", !opt.no_loop_closure)
      .set("use_panoramas", opt.use_panoramas)
      .set("loop_edges_file", opt.loop_edges_file)
      .set("loop_proposal", opt.loop_proposal)
      .set("loop_min_frame_gap", opt.loop_min_frame_gap)
      .set("loop_min_inliers", opt.loop_min_inliers)
      .set("pcm", !opt.loop_no_pcm)
      .set("dry_run", opt.dry_run)
      .dump();
}

reusex::slam::GraphDiscoverOptions
graph_discover_options(const SubcommandGraphDiscoverOptions &opt) {
  reusex::slam::GraphDiscoverOptions o;
  auto &lc = o.candidates.loop_closure;
  lc.enable = !opt.no_loop_closure;
  lc.proposal = reusex::slam::parse_loop_proposal(opt.loop_proposal);
  lc.min_frame_gap = opt.loop_min_frame_gap;
  lc.max_candidate_distance = opt.loop_max_distance;
  lc.max_view_angle = opt.loop_max_view_angle;
  lc.max_candidates_per_frame = opt.loop_max_candidates;
  lc.min_match_inliers = opt.loop_min_inliers;
  lc.max_features = opt.loop_max_features;
  lc.ratio_test = opt.loop_ratio_test;
  lc.ransac_inlier_dist = opt.loop_ransac_inlier_dist;
  lc.max_seed_disagreement = opt.loop_max_seed_disagreement;
  lc.min_seed_disagreement = opt.loop_min_seed_disagreement;
  lc.min_seed_disagreement_fraction = opt.loop_min_seed_disagreement_fraction;
  lc.pcm = !opt.loop_no_pcm;
  lc.seed = opt.seed;
  auto &pl = o.candidates.panorama_loops;
  pl.enable = opt.use_panoramas;
  pl.max_frames = opt.pano_max_frames;
  pl.min_frame_inliers = opt.pano_min_inliers;
  pl.max_edges_per_panorama = opt.pano_max_edges;
  pl.n_yaw = opt.pano_n_yaw;
  pl.max_features = opt.pano_max_features;
  pl.max_pano_distance = opt.pano_max_distance;
  pl.seed = opt.seed;
  o.candidates.loop_edges_file = opt.loop_edges_file;
  o.candidates.loop_edges_min_seed_disagreement =
      opt.loop_edges_min_disagreement;
  o.candidates.loop_edges_min_seed_disagreement_fraction =
      opt.loop_edges_min_disagreement_fraction;
  o.dry_run = opt.dry_run;
  return o;
}

int run_subcommand_graph_discover(const SubcommandGraphDiscoverOptions &opt,
                                  const RuxOptions &global_opt) {
  try {
    reusex::ProjectDB db(global_opt.project_db);
    if (int rc = rux::check_stage_prerequisites(
            db, reusex::core::PipelineStage::graph_discover);
        rc != RuxError::SUCCESS)
      return rc;
    // Flags -> options, then the stage JSON through the reader the GUI's
    // in-process stage uses: the two paths cannot read the keys apart.
    auto options = graph_discover_options(opt);
    const std::string params = graph_discover_stage_parameters(opt);
    reusex::slam::apply_discover_parameters(options, params);
    const int logId = db.log_pipeline_start("graph_discover", params);
    try {
      const auto r = reusex::slam::discover_loop_edges(db, options);
      db.log_pipeline_end(logId, true);
      spdlog::info("Loop edges: {} proposed, {} matched, {} PCM-rejected, {} "
                   "already stored, {} rejected before, {} added",
                   r.proposed, r.candidates, r.pcm_rejected,
                   r.skipped_existing, r.skipped_rejected, r.added);
      if (!opt.dry_run && r.added > 0)
        spdlog::info("Run 'rux optimize' to solve with the new edges.");
      return RuxError::SUCCESS;
    } catch (const std::exception &e) {
      db.log_pipeline_end(logId, false, e.what());
      throw;
    }
  } catch (const std::invalid_argument &e) {
    spdlog::error("graph discover: {}", e.what());
    return RuxError::INVALID_ARGUMENT;
  } catch (const std::exception &e) {
    spdlog::error("graph discover failed: {}", e.what());
    return RuxError::GENERIC;
  }
}
```

- [ ] **Step 4: `graph prune`**

`apps/rux/include/graph/prune.hpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "../global-params.hpp"

#include <CLI/CLI.hpp>
#include <memory>
#include <optional>
#include <string>

struct SubcommandGraphPruneOptions {
  std::optional<double> residual_above;
  std::optional<double> deviation_above;
  std::optional<double> deviation_deg_above;
  std::string type;   ///< "" = any
  std::string source; ///< "" = any
  std::string pair;   ///< "A,B" or ""
  bool include_confirmed = false;
  bool dry_run = false;
};

void setup_subcommand_graph_prune(CLI::App &parent,
                                  std::shared_ptr<RuxOptions> global_opt);
int run_subcommand_graph_prune(const SubcommandGraphPruneOptions &opt,
                               const RuxOptions &global_opt);
```

`apps/rux/src/graph/prune.cpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "graph/prune.hpp"
#include "exit_status.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/slam/graph_prune.hpp>

#include <fmt/format.h>
#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>
#include <exception>
#include <stdexcept>
#include <utility>

namespace {

std::pair<int, int> parse_pair(const std::string &text) {
  const auto comma = text.find(',');
  if (comma == std::string::npos)
    throw std::invalid_argument("--pair expects 'A,B', got '" + text + "'");
  try {
    return {std::stoi(text.substr(0, comma)), std::stoi(text.substr(comma + 1))};
  } catch (const std::exception &) {
    throw std::invalid_argument("--pair expects 'A,B', got '" + text + "'");
  }
}

std::string fmt_metric(double v) {
  return std::isfinite(v) ? fmt::format("{:.3f}", v) : std::string("-");
}

} // namespace

void setup_subcommand_graph_prune(CLI::App &parent,
                                  std::shared_ptr<RuxOptions> global_opt) {
  auto opt = std::make_shared<SubcommandGraphPruneOptions>();
  auto *sub = parent.add_subcommand(
      "prune", "Reject pose-graph edges by residual, deviation, type, source");
  sub->footer(R"(
DESCRIPTION:
  Marks matching edges 'rejected'. Rejected edges stay in the project as
  tombstones: 'rux optimize' ignores them and 'rux graph discover' never adds
  them again. Criteria AND together; at least one is required. Confirmed
  edges are spared unless --include-confirmed.

EXAMPLES:
  rux graph prune --residual-above 50 --type loop_closure --dry-run
  rux graph prune --deviation-above 0.5
  rux graph prune --pair 412,980
)");
  sub->add_option("--residual-above", opt->residual_above,
                  "Residual (0.5 x whitened chi^2) greater than this");
  sub->add_option("--deviation-above", opt->deviation_above,
                  "Deviation from the solved poses greater than this (m)");
  sub->add_option("--deviation-deg-above", opt->deviation_deg_above,
                  "Rotational deviation greater than this (deg)");
  sub->add_option("--type", opt->type,
                  "Edge type: odometry, loop_closure or panorama");
  sub->add_option("--source", opt->source,
                  "Edge source: rtabmap, odometry_init, orb, panorama, file, "
                  "icp, manual or legacy");
  sub->add_option("--pair", opt->pair, "Frame pair 'A,B' (either order)");
  sub->add_flag("--include-confirmed", opt->include_confirmed,
                "Also reject confirmed edges");
  sub->add_flag("--dry-run", opt->dry_run,
                "List the matching edges without rejecting them");
  sub->callback([opt, global_opt]() {
    rux::finish(run_subcommand_graph_prune(*opt, *global_opt));
  });
}

int run_subcommand_graph_prune(const SubcommandGraphPruneOptions &opt,
                               const RuxOptions &global_opt) {
  try {
    reusex::slam::PruneFilter f;
    f.residual_above = opt.residual_above;
    f.deviation_above_m = opt.deviation_above;
    f.deviation_above_deg = opt.deviation_deg_above;
    if (!opt.type.empty())
      f.edge_type = opt.type;
    if (!opt.source.empty())
      f.source = opt.source;
    if (!opt.pair.empty())
      f.pair = parse_pair(opt.pair);
    f.include_confirmed = opt.include_confirmed;
    f.dry_run = opt.dry_run;

    reusex::ProjectDB db(global_opt.project_db);
    nlohmann::json params = {{"dry_run", opt.dry_run},
                             {"include_confirmed", opt.include_confirmed}};
    const int logId = db.log_pipeline_start("graph_prune", params.dump());
    try {
      const auto r = reusex::slam::prune_pose_graph(db, f);
      db.log_pipeline_end(logId, true);
      for (const auto &e : r.matched)
        spdlog::info("  {:>6} {:>6}  {:<13} {:<14} {:<9} residual {} "
                     "deviation {} m",
                     e.from_node_id, e.to_node_id, e.edge_type, e.source,
                     e.status, fmt_metric(e.residual),
                     fmt_metric(e.deviation_m.value_or(NAN)));
      spdlog::info("{} edges matched; {} rejected{}", r.matched.size(),
                   r.rejected, opt.dry_run ? " (dry run)" : "");
      return RuxError::SUCCESS;
    } catch (const std::exception &e) {
      db.log_pipeline_end(logId, false, e.what());
      throw;
    }
  } catch (const std::invalid_argument &e) {
    spdlog::error("graph prune: {}", e.what());
    return RuxError::INVALID_ARGUMENT;
  } catch (const std::exception &e) {
    spdlog::error("graph prune failed: {}", e.what());
    return RuxError::GENERIC;
  }
}
```

`apps/rux/src/graph.cpp`: include `"graph/discover.hpp"` and `"graph/prune.hpp"`, call `setup_subcommand_graph_discover(*sub, global_opt);` and `setup_subcommand_graph_prune(*sub, global_opt);` after `init`, and extend the footer's SUBCOMMANDS list:

```
  init      Snapshot initial poses and create odometry edges
  discover  Find loop edges (ORB, panoramas, edge file) and add them
  prune     Reject edges by residual, deviation, type, source or pair
```

- [ ] **Step 5: Qt "Kopiér som rux-kommando" mapping**

`apps/rux/qt/src/core/cli_command.cpp`, append to `stage_specs()` after the `optimize` entry:

```cpp
      {"graph_discover",
       {"graph", "discover"},
       {{"loop_closure", {}, {}, "--no-loop-closure"},
        {"use_panoramas", {}, "--use-panoramas", {}},
        {"loop_edges_file", "--loop-edges", {}, {}},
        {"loop_proposal", "--loop-proposal", {}, {}},
        {"loop_min_frame_gap", "--loop-min-frame-gap", {}, {}},
        {"loop_min_inliers", "--loop-min-inliers", {}, {}},
        {"pcm", {}, {}, "--loop-no-pcm"},
        {"dry_run", {}, "--dry-run", {}}}},
```

and update the comment above `stage_specs()` to "Mirrors apps/rux/src/create/*.cpp, optimize.cpp and graph/discover.cpp."

- [ ] **Step 6: Scripts, tools and docs**

- `scripts/bench-loop-edges.sh:135-140`:

```bash
    if [[ -n "$edge_file" ]]; then
      { "$rux" -v -p "$rux_copy" graph discover --no-loop-closure \
          --loop-edges "$edge_file" &&
        "$rux" -v -p "$rux_copy" optimize --loop-trust \
          --odometry-sigma-trans 0.05 "$@"; } > "$opt_log" 2>&1 || true
    else
```

  and in the comment block (:73-80) replace "Parses the `rux optimize` log" with "Parses the `rux graph discover` + `rux optimize` log" and "(src/slam/optimize_sensor_poses.cpp)" with "(src/slam/graph_discover.cpp)".
- `scripts/bench-arkitscenes-drift.sh:158`: `"$rux" -v -p "$work" graph discover --no-loop-closure --loop-edges "$edges" && "$rux" -v -p "$work" optimize --no-plane-factors`; `:171`: `"$rux" -v -p "$work" graph discover --no-loop-closure --loop-edges "$edges" && "$rux" -v -p "$work" optimize`.
- `scripts/create_pseudo_gt.sh:276-283`:

```bash
echo "==> [4/5] graph discover --loop-edges; optimize --loop-trust --odometry-sigma-trans 0.05"
if $dry_run; then
  run "$rux" -v -p "$work" graph discover --no-loop-closure --loop-edges "$edges"
  run "$rux" -v -p "$work" optimize --loop-trust --odometry-sigma-trans 0.05
else
  { "$rux" -v -p "$work" graph discover --no-loop-closure --loop-edges "$edges" &&
    "$rux" -v -p "$work" optimize --loop-trust --odometry-sigma-trans 0.05; } \
    2>&1 | tee "$optimize_log"
fi
```

  and the prose at :11 and :333 ("`rux optimize --loop-edges <f> --loop-trust`" / "`optimize --loop-edges <edges> --loop-trust`") → "`rux graph discover --no-loop-closure --loop-edges <f>` + `rux optimize --loop-trust`".
- `scripts/spatial_loop_validate.sh:256-262`:

```bash
"$RUX" -p "$WORK_RUX" graph discover --no-loop-closure \
    --loop-edges "$EDGES_JSON" \
    2>&1 | sed 's/^/  [rux graph discover] /'
"$RUX" -p "$WORK_RUX" optimize \
    --loop-trust \
    --odometry-sigma-trans "$SIGMA_TRANS" \
    2>&1 | sed 's/^/  [rux optimize] /'
```

  and :12 "Run rux optimize --loop-edges" → "Run rux graph discover --loop-edges + rux optimize".
- `tools/loop_edges/README.md:8` → "Out-of-process producer for `rux graph discover --loop-edges` (issues #221 / #225 P2)."; `:130-131` →

```
rux -p PROJECT.rux graph discover --no-loop-closure --loop-edges edges-xfeat.json
rux -p PROJECT.rux optimize --loop-trust --odometry-sigma-trans 0.05
```

- `tools/loop_edges/export_loop_edges.py:6,19,307-308` and `propose_targeted_revisits.py:6,39,151-152`: replace "`rux optimize --loop-edges`" with "`rux graph discover --loop-edges`" (comments/help text only).
- `CLAUDE.md`: CLI table row `graph` → "`init`, `discover`, `prune` (stored pose graph)"; in **slam** bullets replace "`slam/loop_candidates.hpp` the ORB / panorama / file loop front-ends." with "`slam/loop_candidates.hpp` the ORB / panorama / file loop front-ends, `slam/graph_discover.hpp` (`rux graph discover`: front-ends vs the stored graph, PCM with confirmed edges as fixed inliers; also `JobStage::graph_discover`, run by `stage_executor_with_optimize()`), `slam/graph_prune.hpp` (`rux graph prune`)."; in `PanoramaLoopEdges.hpp` bullet "(`rux optimize --use-panoramas`, #236)" → "(`rux graph discover --use-panoramas`, #236)"; in the Qt client paragraph "its Pipeline workspace runs stages in-process through `pipeline::JobRunner` with `pipeline::stage_executor_with_optimize()`" add "(which runs `optimize` and `graph_discover`)".
- `docs/DIRECTION.md:104`: "(`rux optimize --use-panoramas`)" → "(`rux graph discover --use-panoramas`)".

- [ ] **Step 7: Run tests to verify they pass**

```bash
nix develop -c cmake --build build -j$(nproc)
nix develop -c build/tests/reusex_unit_tests_vision "[rux_app]"
nix develop -c build/apps/rux/rux graph discover --help
nix develop -c build/apps/rux/rux graph prune --help
bash -n scripts/bench-loop-edges.sh scripts/bench-arkitscenes-drift.sh scripts/create_pseudo_gt.sh scripts/spatial_loop_validate.sh scripts/bench-office.sh
grep -rn "optimize --loop-edges\|optimize --loop-closure\|optimize --use-panoramas" scripts tools docs CLAUDE.md README.md | grep -v "^docs/research\|^docs/superpowers"
```

Expected: PASS (all round-trip cases now include `graph_discover`; `GraphCli_*` pass); `bash -n` silent; the final `grep` prints nothing (historical `docs/research/` notes are exempt, STANDARDS §9.9).

- [ ] **Step 8: Commit**

```bash
git add apps/rux/include/graph/discover.hpp apps/rux/src/graph/discover.cpp apps/rux/include/graph/prune.hpp \
        apps/rux/src/graph/prune.cpp apps/rux/src/graph.cpp apps/rux/qt/src/core/cli_command.cpp \
        tests/unit/rux_app/test_cli_command_roundtrip.cpp tests/unit/rux_app/test_graph_cli.cpp \
        scripts/bench-loop-edges.sh scripts/bench-arkitscenes-drift.sh scripts/create_pseudo_gt.sh \
        scripts/spatial_loop_validate.sh tools/loop_edges CLAUDE.md docs/DIRECTION.md
git commit -m "rux: graph discover and graph prune; scripts move to discover + optimize

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_018dLKtE8qid8y5QVF9b2yri"
```

---

## Task 19: Verification — full suite, benchmark parity (STANDARDS §8), real-data smoke

**Files:** none changed (unless a check fails — then fix in the owning task's files and re-run).

- [ ] **Step 1: Full suite and package consumer**

```bash
nix develop -c cmake --build build -j$(nproc) 2>&1 | grep -iE "warning|error" | grep -v "^ninja" || true
nix develop -c bash -c 'cd build && ctest --output-on-failure --parallel $(nproc)'
fish tests/test_completions.fish
```

Expected: no new warnings (compare with Task 0); every test passes, including `reusex_package_consumer` (the new public headers `core/pose_graph.hpp`, `slam/pose_graph*.hpp`, `slam/loop_candidates.hpp`, `slam/graph_*.hpp`, `slam/discover_parameters.hpp`, `io/rtabmap_links.hpp` must install and include only `<reusex/...>` siblings).

- [ ] **Step 2: Benchmark parity on the drift benchmark**

The micro-benchmark suite (`scripts/bench.sh`, `tests/benchmarks/bench_geometry.cpp`) does not cover pose optimisation, so the spec's §7 check is the drift benchmark: with the stored graph equal to what the old code built (auto-init from the perturbed poses), the rewritten solve must give the same accuracy.

```bash
S=/tmp/claude-1001/-home-mephisto-repos-ReUseX--worktrees-gui-graph-editor/b9a0a5e9-d5a5-48c6-adfb-71b8bc180a4a/scratchpad
BASE=$(cat $S/pose-graph-baseline-sha.txt)
git worktree add --detach $S/baseline $BASE
nix develop -c cmake -S $S/baseline -B $S/baseline/build -G Ninja -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF -DBUILD_PYTHON_BINDINGS=OFF -DBUILD_DOCUMENTATION=OFF
nix develop -c cmake --build $S/baseline/build --target rux -j$(nproc)
# baseline (old optimize) and candidate (stored graph), same scenes/seed/scale
( time nix develop -c $S/baseline/scripts/bench-arkitscenes-drift.sh -c "none optimize noplane" -s "1.0" -o $S/drift-base ) 2> $S/drift-base.time
( time nix develop -c scripts/bench-arkitscenes-drift.sh -c "none optimize noplane" -s "1.0" -o $S/drift-cand ) 2> $S/drift-cand.time
diff <(sort $S/drift-base/drift-bench-summary.txt) <(sort $S/drift-cand/drift-bench-summary.txt) || true
```

Run the two benchmark lines in the background (`run_in_background`), they take tens of minutes. Expected per (scene, config) row: `F@50mm` within ±0.005 and `chamfer` within 5 % of the baseline; `none` rows identical (no pose stage). Wall time within +5 % (the stored-graph path adds one init and one metrics transaction). Record the per-row table in the PR description (STANDARDS §8). A larger difference is a regression: compare the `PlaneGraph:` log lines of one scene between the two runs (edge counts, `round` errors) before changing code. Afterwards: `git worktree remove $S/baseline`.

If `~/datasets/arkitscenes/<id>.rux` or the raw GT meshes are missing, record "drift benchmark skipped: dataset not present" and instead run the parity check on any available project: copy it twice, run the baseline `rux optimize` on one and the candidate `rux optimize` on the other, and compare `sensor_frames.transform` (max translation difference ≤ 1e-6 m) with a small `python3 sqlite3` script.

- [ ] **Step 3: Real-data smoke on an RTABMap import**

Reuse `$S/rtab_links.rux` from Task 12 Step 6 (skip with a note if it was skipped there):

```bash
R=$S/rtab_links.rux
nix develop -c build/apps/rux/rux -p $R graph discover --dry-run
nix develop -c build/apps/rux/rux -p $R graph discover
nix develop -c build/apps/rux/rux -p $R optimize
python3 - $R > $S/poses1.txt <<'PY'
import sqlite3, sys
for nid, t in sqlite3.connect(sys.argv[1]).execute("SELECT node_id, transform FROM sensor_frames ORDER BY node_id"):
    print(nid, t.hex() if t else None)
PY
nix develop -c build/apps/rux/rux -p $R optimize
python3 - $R > $S/poses2.txt <<'PY'
import sqlite3, sys
for nid, t in sqlite3.connect(sys.argv[1]).execute("SELECT node_id, transform FROM sensor_frames ORDER BY node_id"):
    print(nid, t.hex() if t else None)
PY
diff $S/poses1.txt $S/poses2.txt && echo REPEATABLE
nix develop -c build/apps/rux/rux -p $R graph prune --residual-above 100 --dry-run
nix develop -c build/apps/rux/rux -p $R validate --stage graph_discover
nix develop -c build/apps/rux/rux -p $R log
```

Expected: discover reports its counts; `REPEATABLE` is printed (bit-identical poses on the second optimize); prune lists candidates without writing; `validate` passes; `log` shows `graph_discover` and `pose_optimization_plane_graph` entries. If the two pose dumps differ only in the last bits, report it (GTSAM threading) rather than loosening anything.

- [ ] **Step 4: Report**

Summarise for the PR: test counts, the benchmark table (or the documented skip), the real-data results, and the two known follow-ups — `ruxd` must migrate off the deprecated `add_pose_graph_edge` / `delete_pose_graph_edges` and its `/posegraph` endpoints should read the v27 columns; the Qt plan (P4–P6) consumes "Interfaces for the Qt plan".

---

## Spec coverage map

| Spec item | Task |
|---|---|
| §3.1 new columns, UNIQUE, de-dup, legacy rows | 2 |
| §3.1 NULL transform skipped with warn + remedy | 10 |
| §3.2 `initial_transform`; optimize seeds from it, fallback warn | 4, 10 |
| §3.3 `project_settings`, `setting`/`set_setting` | 4 |
| §3.4 `PoseGraphEdge` fields, upsert/status/metrics, deprecated wrappers, initial pose API | 2, 3, 4 |
| §3.4 `save_pose_graph_edges` removed | 10 |
| §4.1 stored ↔ optical helper pair, round-trip test | 5 |
| §4.2 RTABMap link import (types, information permutation, frame convention, initial poses) | 12 |
| §4.3 `graph init` (+ `--force` semantics) | 7, 13 |
| §4.3 `graph discover` (initial seeds, skip existing/tombstones, PCM stored ∪ new, confirmed fixed, report counts) | 14, 15, 18 |
| §4.3 `graph prune` (filters AND, confirmed spared, dry run) | 16, 18 |
| §4.3 rewritten optimize (auto-init, active edges, seeds, metrics, never replaces) | 9, 10 |
| §4.3 CLI break with pointer | 8 |
| §4.3 scripts/docs | 13, 18 |
| §4.4 `JobStage::graph_discover`, reader, round-trip, executor, `pose_graph_edges` artifact, contracts, `validate --stage graph_discover` | 10, 17, 18 |
| §5.2 ICP `initial_guess` option (consumed by P5) | 6 |
| §6 DIRECTION.md changelog (P2) | 13 |
| §7 tests P1–P3; STANDARDS §8 benchmark | 1–18, 19 |
