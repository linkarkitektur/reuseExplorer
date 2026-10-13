# Pose-graph editor: stored graph as source of truth + Rediger workspace

Date: 2026-10-13 · Branch: `gui-graph-editor` · Status: design approved in chat, awaiting spec review

## 1. Intent

The user wants an RTABMap-database-editor-style workflow in the native Qt
client: find wrong loop closures quickly (e.g. a closure between two similar
office floors), inspect the pair as images and overlaid 3D clouds, fix it by
hand (manual alignment → ICP), confirm or reject it, and re-optimise.

Success looks like:

- Suspicious edges stand out by colour (strain) and sort to the top of a list.
- Clicking nodes fills frame slots A/B alternately; the pair shows images and a
  3D overlay where B can be moved by hand, then ICP-refined, then saved as an
  edge.
- User decisions (added, confirmed, rejected edges) survive every later
  `rux optimize` and `rux graph discover`.
- Re-running `rux optimize` is repeatable (same input → same poses), not
  compounding on its previous output.
- Database stays a read-only overview and can preview panoramas.

### Problems in the current code this fixes

- `pose_graph_edges` stores only `(from, to, type, residual, weight)` — no
  measurement. `optimize_sensor_poses.cpp:364` only *writes* the table
  (atomic replace-all) and never reads it, so GUI-added edges are never used
  and are wiped by the next optimize.
- Odometry factors are rebuilt each run from consecutive *current* poses in
  `sensor_frames.transform`, which optimize then overwrites (no original kept):
  every run chains onto the previous run.
- `import rtabmap` fetches RTABMap's links (`rtabmap.cpp:56`) and discards them.
- Posegraf is XY-only (`pose[3]`, `pose[7]`), read-only, its legend lists types
  that are rarely drawn, and the zoom is unclamped
  (`PoseGraphWorkspace.cpp:255`).
- Pair editing (`PairStrip`) is text-only and lives in the Database page.
- No panorama bitmap is shown anywhere in the client.

## 2. Decisions (from the brainstorm)

| # | Decision |
|---|---|
| D1 | Data model first; it is in scope for this branch. |
| D2 | **Stored graph is the source of truth.** `optimize` solves the stored graph; edge discovery and edge pruning are separate explicit steps. |
| D3 | Odometry edges come from RTABMap links on import; other projects get `rux graph init`, which freezes odometry from current poses and snapshots initial poses. |
| D4 | Discovered edges are active immediately (`status=auto`); rejection keeps a tombstone; user edges are `confirmed`. |
| D5 | Clean CLI break: `--loop-*`/`--pano-*`/`--loop-edges` move from `optimize` to `graph discover`; `optimize` errors with a pointer if given them. |
| D6 | Rediger layout: graph + edge list left, pair panel (images over 3D) right. |
| D7 | Model rotation = persistent display-only view yaw, stored in the project, reused by the 3D view and `rux render`. |

## 3. Data model — schema v27

`LATEST_SCHEMA_VERSION` 26 → 27. Migration is additive.

### 3.1 `pose_graph_edges` (extended)

New columns (all nullable so v15–v26 rows migrate in place):

| column | type | meaning |
|---|---|---|
| `transform` | BLOB (16 × f64, row-major) | Measurement `T_ij = T_from⁻¹ · T_to`, in the **`sensor_frames.transform` convention** (the stored world pose, not the optical frame). |
| `information` | BLOB (36 × f64, row-major) | 6×6 information matrix in **GTSAM Pose3 tangent order (rx, ry, rz, tx, ty, tz)**. NULL → per-type default σ from `PlaneGraphOptions`. |
| `source` | TEXT | `rtabmap \| odometry_init \| orb \| panorama \| file \| icp \| manual \| legacy` |
| `status` | TEXT NOT NULL DEFAULT `'auto'` | `auto \| confirmed \| rejected` |
| `created_at` | TEXT | ISO-8601 UTC |
| `seed_disagreement_m`, `seed_disagreement_deg` | REAL | Correction the edge asks for vs. the initial poses (‖log(T_ij⁻¹ · T_from,init⁻¹ T_to,init)‖ split into translation/rotation). Written when the edge is created and recomputed by `optimize`. |
| `deviation_m`, `deviation_deg` | REAL | Same quantity vs. the **solved** poses. Written by `optimize`. |

Existing columns keep their meaning: `edge_type` (`odometry | loop_closure |
panorama`), `residual` (0.5 × whitened χ² after the solve, written by
`optimize`), `weight` (1/σ_trans², now derived from `information`).

Constraints:

- `UNIQUE(from_node_id, to_node_id, edge_type)`. The migration de-duplicates
  first (keeps the lowest `id`). The unique key is what tombstones match on.
- Migrated v15 rows get `source='legacy'`, `status='auto'`, `transform=NULL`.
  `optimize` skips edges with a NULL transform and warns with the count and the
  remedy (`rux graph discover` / `rux graph init --force`) — STANDARDS §5.

### 3.2 `sensor_frames.initial_transform`

New nullable BLOB column, same layout as `transform`. Written by `graph init`
and by `import rtabmap`. `optimize` seeds from it; frames lacking it fall back
to `transform` with a warning.

### 3.3 `project_settings`

`CREATE TABLE project_settings (key TEXT PRIMARY KEY, value TEXT NOT NULL)`.
First key: `view.yaw_deg` (D7). `ProjectDB::setting(key)` → `std::optional<std::string>`,
`set_setting(key, value)`.

### 3.4 `ProjectDB` API

- `PoseGraphEdge` gains `std::optional<std::array<double,16>> transform`,
  `std::optional<std::array<double,36>> information`, `source`, `status`,
  `created_at`, and the four metric fields (`std::optional<double>`).
- Keep `list_pose_graph_edges()`, `has_pose_graph()`.
- `upsert_pose_graph_edge(edge)` — insert or replace measurement on the unique
  key; refuses (returns false, no write) when the existing row is `rejected`
  and the caller did not pass `overwrite_rejected`.
- `set_pose_graph_edge_status(from, to, type, status)`.
- `update_pose_graph_edge_metrics(span<const EdgeMetrics>)` — one transaction.
- `save_pose_graph_edges` (replace-all) is **removed**; its only caller is
  optimize. `add_pose_graph_edge` / `delete_pose_graph_edges` are superseded
  by the upsert/status calls (the GUI "delete" becomes reject) but stay as
  `[[deprecated]]` wrappers (`delete` → reject) because `ruxd` builds against
  the installed package and uses them; removal is a follow-up once `ruxd`
  migrates.
- `set_initial_pose(id, pose)`, `initial_pose(id)`, `has_initial_pose(id)`.

## 4. Library and CLI

### 4.1 Edge measurement conventions

The optimizer works on optical-frame poses `world_pose = worldTf · localTf`.
On load, a stored `T_ij` becomes `localTf_i⁻¹ · T_ij · localTf_j`; on write the
inverse. One helper pair in `slam/` does this, unit-tested for round-trip.

### 4.2 `import rtabmap`

Store `links` instead of discarding them:

- `Neighbor`, `NeighborMerged` → `odometry`, source `rtabmap`.
- `GlobalClosure`, `LocalSpaceClosure`, `LocalTimeClosure`, `UserClosure` →
  `loop_closure`, source `rtabmap`.
- `VirtualClosure`, `Landmark`, `Gravity`, others → skipped, counted in the
  import log.
- `infMatrix()` is (x, y, z, roll, pitch, yaw) → permute to GTSAM order.
- The link transform's frame (RTABMap base frame) must be converted to the
  `sensor_frames.transform` convention the importer writes; the plan verifies
  this against the importer's pose write path with a fixture.
- Also writes `initial_transform` = the imported pose.

### 4.3 `rux graph` (new top-level group, `apps/rux/src/graph/`)

Each subcommand is thin (STANDARDS §1) over a library entry point in `slam/`.

- `rux graph init [--force]` → `slam::init_pose_graph(db, options)`.
  Snapshots `initial_transform` and writes one `odometry` edge per
  consecutive-frame pair (source `odometry_init`, default odometry σ).
  Refuses if odometry edges exist unless `--force`; `--force` replaces only
  `odometry_init` edges and re-snapshots, never touches loop edges or tombstones.
- `rux graph discover [loop/panorama/file flags]` →
  `slam::discover_loop_edges(db, options)`. Runs the existing front-ends
  (ORB `detect_loop_edges`, `PanoramaLoopEdges`, `load_loop_edges`) seeded from
  the **initial poses**, then PCM over *stored active edges ∪ new candidates*
  (stored confirmed edges are fixed inliers). Skips any key that already exists
  (any status, including tombstones). Writes survivors with `status=auto` and
  their `seed_disagreement_*`. Reports proposed / matched / PCM-rejected /
  skipped-existing / skipped-rejected / added.
- `rux graph prune [--residual-above X] [--deviation-above M]
  [--deviation-deg-above D] [--type T] [--source S] [--pair A,B]
  [--include-confirmed] [--dry-run]` → `slam::prune_pose_graph(db, filter)`.
  Sets `status=rejected` on matches (filters AND together; at least one
  required). Prints the affected edges; `--dry-run` writes nothing.
- `rux optimize` → `optimize_sensor_poses` rewritten to:
  1. If no odometry edges exist, run `init_pose_graph` and log it at `info`.
  2. Load active edges (`auto` + `confirmed`, transform not NULL) as
     `BetweenFactor`s (odometry edges keep today's trusted/robust handling;
     loop edges keep `--loop-trust` handling, which stays on `optimize`).
  3. Seed from `initial_transform`; plane landmarks unchanged.
  4. Write poses to `transform` (never to `initial_transform`) and per-edge
     `residual`, `deviation_*`, `seed_disagreement_*` via
     `update_pose_graph_edge_metrics`. The edge set is never replaced.
- `--loop-closure`, `--loop-*` (except `--loop-trust*`), `--use-panoramas`,
  `--pano-*`, `--loop-edges*` are removed from `optimize`; passing one fails
  with "moved to `rux graph discover`" (CLI11 custom validator / extra-args
  check).
- `scripts/bench-*.sh` and `docs/` usages are updated in the same PR.

### 4.4 Pipeline

- New `JobStage::graph_discover`, executed (like `optimize`) by
  `pipeline/optimize_stage.hpp`'s `stage_executor_with_optimize()`, with a
  JSON parameter reader `slam/discover_parameters.hpp` mirroring
  `optimize_parameters.hpp`. The CLI round-trip test covers its defaults.
- `run_optimize_stage` reports `pose_graph_edges` as a touched artifact.
- `docs/CONTRACTS.md` gains the graph stage contracts (init/discover/optimize
  inputs and outputs) and `rux validate --stage graph_discover`.

## 5. Qt client

### 5.1 Navigation

Rail: **Oversigt (Database), 3D, Rediger, Pipeline, Log**. The Posegraf page is
removed; its canvas moves into Rediger. `PairStrip` leaves `DatabaseWorkspace`.
`workspaces.hpp`, `workspace_infos()`, the command palette entries and
`RUX_QT_PAGE` gain `rediger` and drop `posegraf`.

### 5.2 Rediger workspace (`EditWorkspace`)

```
+----------------------+-----------------+
| [XY|XZ|YZ] yaw  farve| A: 412   B: 980 |
|                      | +------+------+ |
|     GRAPH CANVAS     | | img A| img B| |
|                      | +------+------+ |
|                      | +-------------+ |
|                      | |  3D overlay | |
+----------------------+ +-------------+ |
| EDGE LIST (sortable) | ICP  Nulstil    |
|                      | [+kant] confirm |
+----------------------+-----------------+
```

Nested `QSplitter`s; each pane has a collapse button in its header and a
toolbar/palette toggle; sizes and visibility persist in `QSettings`
(`rediger/…`). The 3D pane is shown only when both A and B are set.

**Graph canvas** (evolves `PoseGraphView`):

- Projection XY / XZ / YZ — node positions come from the full translation
  (x, y, z), projected through the view yaw, then the selected plane.
- View yaw: slider/spinbox plus "Auto" (dominant horizontal direction of
  frame headings / plane normals; plan picks the cheapest robust estimate).
  Saved to `project_settings` `view.yaw_deg`; the 3D workspace and
  `rux render` apply it to their default camera.
- Nodes: screen-sized dots plus a heading tick (camera forward axis
  projected into the view plane, fixed pixel length), RTABMap-style.
- Zoom clamped to a min/max units-per-pixel; the "nodes disappear when
  zoomed" report is reproduced first and the root cause fixed (likely
  candidates: unclamped scale, item culling) — not guessed.
- Colour by: type · status · residual · deviation (m) · seed disagreement (m).
  Residual uses a fixed χ²-based scale (green ≤ χ²₆(0.95)/2, red ≥ 5× that);
  metric modes use the 5th–95th percentile of drawn edges. Rejected edges
  hidden by default (toggle shows them dashed).
- Legend generated from what is drawn, with counts per bucket, or a colour
  bar for continuous modes. Unknown edge types get their own swatch rather
  than silently rendering as odometry.
- Click node → fills the "next" slot (A, then B, alternating; toolbar shows
  which is next; Shift-click forces B). Click edge → loads its pair and
  selects its row in the edge list.

**Edge list**: `QTableView` over a Qt-free model (pair, type, source, status,
residual, deviation m/°, seed disagreement m/°), sortable, filter by
type/status. Row click selects the pair in the graph.

**Pair panel**:

- Images A | B with colour/depth toggle (reuses `FrameImageLoader`).
- 3D overlay (`SceneView`): both frames' depth clouds back-projected with their
  current poses, A tinted one colour, B another. B carries a manual offset
  `ΔT` edited with a VTK translate/rotate widget (no scaling), numeric fields
  (x, y, z, yaw, pitch, roll) and keyboard nudges; "Nulstil" clears `ΔT`.
- "ICP-forfin" runs `refine_frame_pair_icp` seeded with `ΔT` (the function
  gains an `initial_guess` option) and shows fitness / inliers / shift.
- Edge rows for the pair: add (type combo) → stages an edge with
  `transform` = current relative pose including `ΔT` (ICP-refined if run),
  information from ICP fitness or the default, `source` `icp`/`manual`,
  `status` `confirmed`; confirm; reject / un-reject.
- Toolbar: "Gem ændringer" (existing `EdgeEditor` staging + writer lease),
  "Find kanter" (`graph_discover` via `JobRunner`), "Optimer" (`optimize`).
  After a stage finishes the graph, list and colours reload.

### 5.3 Database: panorama preview

Selecting a panorama in the Database tree shows the equirect image in a
pan/zoom viewer (decoded off the GUI thread). Flat equirect only; no
spherical look-around in this scope.

### 5.4 Qt-free logic (`rux_qt_core`, tested in `tests/unit/rux_qt`)

- View projection (plane + yaw) and heading-tick geometry.
- Slot alternation state for A/B selection.
- Colour mapping per mode, legend bucketing with counts.
- Edge-list model data (sorting/filtering keys).
- Staged edit model extended for confirm/reject/un-reject and transforms.

## 6. Phasing (one PR each, merged in order)

| Phase | Content | Depends on |
|---|---|---|
| P1 | Schema v27, `ProjectDB` API, measurement conventions helper | — |
| P2 | `graph init`, rewritten `optimize`, CLI break, rtabmap link import, scripts/docs | P1 |
| P3 | `graph discover` + `graph prune`, `JobStage::graph_discover` | P2 |
| P4 | Rediger shell: nav change, splitter layout, graph canvas upgrades (planes, yaw, ticks, zoom fix, colouring, legend, slot alternation), edge list, move `PairStrip` | P1 (metrics show once P2 lands) |
| P5 | Pair panel: images, 3D overlay, manual `ΔT`, ICP with initial guess, edge add/confirm/reject with transforms | P4 |
| P6 | Panorama preview in Database | — (independent) |

`docs/DIRECTION.md` gets a dated changelog entry in P2 (optimize semantics
change) and P4 (Posegraf → Rediger).

## 7. Testing

- P1: migration v26→v27 (columns, de-dup, unique key, legacy rows); upsert
  refuses over a tombstone; metrics update; settings round-trip; transform
  convention round-trip.
- P2: `graph init` writes N−1 odometry edges and snapshots; `optimize` twice
  on the same project yields identical poses (repeatability); optimize never
  removes edges or changes status; legacy NULL-transform edges skipped with a
  warning; removed flags fail with the pointer message; rtabmap link import
  on a fixture (type mapping, information permutation, frame convention);
  CLI round-trip defaults still match.
- P3: discover skips tombstoned and existing keys; PCM keeps confirmed edges
  as inliers; prune filters AND, respects confirmed, `--dry-run` writes
  nothing; stage JSON reader.
- P4–P5: `rux_qt_core` unit tests (§5.4); `rux-qt-gallery` screenshots of
  Rediger in each projection and colour mode for design review
  (design-studio loop); manual smoke on a real RTABMap project.
- STANDARDS §8: benchmark `optimize` before/after on the drift benchmark to
  confirm the rewritten solve matches the old one when the stored graph
  equals what the old code would have built.

## 8. Out of scope

- Spherical panorama viewer; leave-one-out edge influence; re-orienting the
  actual model (D7 chose view-only); editing plane landmarks; `ruxd`/web GUI
  adoption of the new graph API (they will need a follow-up in their repos —
  `ruxd`'s `/posegraph` endpoints read `pose_graph_edges`).
