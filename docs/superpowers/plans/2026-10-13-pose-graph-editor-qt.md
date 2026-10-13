# Pose-graph editor — Qt client (Rediger, pair panel, panorama preview) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the read-only Posegraf page with a **Rediger** workspace in the native Qt client where a user finds suspicious pose-graph edges (colour + sorted list), inspects a frame pair as images and an overlaid 3D cloud, aligns B by hand, ICP-refines from that guess, and confirms / rejects / adds edges that survive every later `rux optimize` / `rux graph discover`; plus a panorama preview in the Database (now "Oversigt") page.

**Architecture:** Every decision a test can pin lives in `rux_qt_core` (standard library only, tested in the light `reusex_unit_tests` binary): view projection + yaw + heading ticks, A/B slot alternation, zoom limits, edge colouring + legend, the edge-list rows/sort/filter, the staged-edit model (now status- and transform-aware), and the manual-offset math. The Qt layer (`rux_qt_lib`) composes them: `EditWorkspace` (nested `QSplitter`s) holds `GraphPanel` (the evolved `PoseGraphView`), `EdgeListPanel`, `PairImages`, `PairOverlay` (`SceneView` + VTK box widget) and `PairStrip`. Writes go through the existing `EdgeEditor` (one transaction, off the GUI thread, under the pipeline writer lease); "Find kanter"/"Optimer" run through the Pipeline workspace's `JobRunner`.

**Tech Stack:** C++20, Qt 6 Widgets, VTK 9.5 (`GUISupportQt`, `InteractionWidgets`), Catch2 v3, the `rux-qt-gallery` + `qt_shot.sh` screenshot loop, OpenCV (panorama decode), `reusex_slam` (ICP), `reusex_visualize` (camera framing).

**Spec:** `docs/superpowers/specs/2026-10-13-pose-graph-editor-design.md` — this plan covers phases **P4, P5, P6** (§5, §6). P1–P3 (schema v27, `rux graph`, rewritten `optimize`, `JobStage::graph_discover`) are planned in `docs/superpowers/plans/2026-10-13-pose-graph-editor-library.md` and must be merged first.

---

## RECONCILIATION WITH THE LIBRARY PLAN (read first; overrides the section below)

The library plan (`2026-10-13-pose-graph-editor-library.md`, "Interfaces for
the Qt plan") is authoritative. When this plan's code disagrees, adapt this
plan's code to the library as actually merged (read the headers):

1. **Edge direction is canonical (`from < to`).** `upsert_pose_graph_edge`
   inverts the transform / maps information when given `from > to`; rows
   always read back with `from < to`. Staged-edit keys (Task 4) and edge-list
   keys must canonicalise the same way (`std::minmax`); Task 20's "re-measure
   an edge stored the other way round" collapses to a plain upsert.
2. **`set_pose_graph_edge_status` returns `bool`** (false = no row) and takes
   `std::string_view`; it throws `std::invalid_argument` on a bad status.
3. **`initial_pose(id)` returns `std::optional<std::array<double,16>>`.**
4. **ICP:** the option is `FramePairIcpOptions::initial_world_delta` (not
   `initial_guess`), applied to the `from` cloud in world space; the result
   carries `edge_transform` (== `relative_pose⁻¹`) and
   `slam::icp_edge_information(result)` gives the information
   (σ_t = max(fitness, 1 cm), σ_r = max(fitness/2, 0.005) rad). Task 15/19/20
   use these instead of their own ICP information formula: call
   `refine_frame_pair_icp(db, B, A, {.initial_world_delta = ΔT})` and stage
   edge `(B→A, r.edge_transform, icp_edge_information(r), "icp", "confirmed")`.
   Manual (non-ICP) edges keep Task 15's `T_A⁻¹·ΔT·T_B` as the A→B transform.
   Library Task 6 already provides these — Task 16 (`frame_world_points`)
   remains a Qt-plan task.
5. **Vocabulary constants** (`kStatus*`, `kSource*`, `kType*`,
   `kSettingViewYawDeg`) live in header-only `reusex/core/pose_graph.hpp`;
   `rux_qt_core` must stay standard-library-only, so mirror the string values
   there rather than including that header (it pulls Eigen).
6. **Already done by the library plan — skip/verify instead of redoing:**
   `EdgeEditor` moved to upsert/reject and hides rejected edges (lib Task 3) —
   Task 4 extends that rather than re-porting it; the gallery
   `posegraf-stress` page already uses upsert (lib Task 10); the
   `graph_discover` pipeline rows in `pipeline_ui`/`PipelineWorkspace`/
   `cli_command` (lib Tasks 17–18) — Task 12 only adds the Rediger toolbar
   buttons.
7. `PoseGraphEdge::residual`/`weight` default to NaN (NULL); `created_at`
   empty on write → now.

## Assumed library interfaces (reconcile with the library plan)

Written without sight of the library plan. Every Qt task below codes against exactly these; if the library plan differs, fix the names here and in the referencing tasks before executing.

From the spec (verbatim contract given to this plan):

```cpp
// reusex/core/ProjectDB.hpp
struct ProjectDB::PoseGraphEdge {
  int from_node_id, to_node_id;
  std::string edge_type;                       // "odometry" | "loop_closure" | "panorama" (others possible)
  double residual;
  double weight;
  std::optional<std::array<double, 16>> transform;    // row-major, T_from^-1 * T_to, sensor_frames.transform convention
  std::optional<std::array<double, 36>> information;  // row-major 6x6, GTSAM order (rx, ry, rz, tx, ty, tz)
  std::string source;                          // rtabmap | odometry_init | orb | panorama | file | icp | manual | legacy
  std::string status;                          // auto | confirmed | rejected
  std::string created_at;
  std::optional<double> seed_disagreement_m, seed_disagreement_deg, deviation_m, deviation_deg;
};
bool ProjectDB::upsert_pose_graph_edge(const PoseGraphEdge &, bool overwrite_rejected = false);
void ProjectDB::set_pose_graph_edge_status(int from, int to, const std::string &type, const std::string &status);
std::vector<PoseGraphEdge> ProjectDB::list_pose_graph_edges() const;
std::optional<std::string> ProjectDB::setting(std::string_view key) const;   // key "view.yaw_deg"
void ProjectDB::set_setting(std::string_view key, std::string_view value);
std::array<double, 16> ProjectDB::initial_pose(int id) const;

// reusex/slam/frame_pair_icp.hpp
struct FramePairIcpOptions { /* existing fields */ std::optional<std::array<double, 16>> initial_guess; };

// reusex/pipeline
enum class JobStage { /* existing */ graph_discover };   // runnable by JobRunner + stage_executor_with_optimize()
```

Further assumptions this plan makes (each one is load-bearing; verify):

1. `upsert_pose_graph_edge` writes **every** field of `PoseGraphEdge` it is given, including `residual`, `weight`, `status`, `source` and the four metric fields, and stamps `created_at` itself when the given one is empty. (Gallery pages seed synthetic metrics through it.)
2. `set_pose_graph_edge_status` throws `std::invalid_argument` for a status outside `auto|confirmed|rejected` and does nothing (no throw) when no row has that key.
3. `setting()` is `const` and returns `std::nullopt` on a pre-v27 file opened read-only (no `project_settings` table).
4. `FramePairIcpOptions::initial_guess` is a **world-space** rigid transform left-multiplied onto the `from` frame's world cloud before ICP (the same frame `FramePairIcpResult::world_delta` is expressed in), and the returned `world_delta` / `relative_pose` **include** the guess. With `refine_frame_pair_icp(db, from = B, to = A, opts)`, `relative_pose == T_A^-1 * world_delta * T_B` is the A→B edge measurement in the spec's convention.
5. `pipeline::job_stage_names()` lists `"graph_discover"`; `pipeline::pipeline_log_name(JobStage::graph_discover)` exists; `core::PipelineStage::graph_discover` exists (for `rux validate --stage graph_discover`); `rux::qt::build_cli_command("graph_discover", …)` returns `supported == true` with args `{"graph", "discover", …}` (P3 owns the key→flag table because its round-trip test lives in `tests/unit/rux_app`).
6. `save_pose_graph_edges` is removed by P1. Its one Qt caller is `gallery/shell_pages.cpp`'s `posegraf-stress`; Task 10 rewrites that page onto `upsert_pose_graph_edge`. If the library plan's P1 already migrated it, keep theirs and skip that step.
7. `add_pose_graph_edge` / `delete_pose_graph_edges` are `[[deprecated]]` after P1. This plan stops using them in Task 4 (so `-Werror` builds stay clean).

---

## Global Constraints

- C++20; Qt 6 Widgets; VTK 9.5. `rux_qt_core` stays **standard library only** (no Qt, no ProjectDB, no Eigen). Its sources are an **explicit list** in `apps/rux/qt/CMakeLists.txt` — every new `src/core/*.cpp` needs a line there. `rux_qt_lib` and the gallery glob `src/*.cpp` / `gallery/*.cpp`.
- No literal colour, radius, spacing or type size in `apps/rux/qt/`: only `var(--…)` in `styles/app.qss`, or `theme().color("--…")` / `theme().px("--…")` in C++. `python .claude/skills/design-studio/scripts/token_lint.py apps/rux/qt --qt` must pass after every UI task. Never hand-edit `apps/rux/qt/theme/tokens.css`.
- SPDX header on every new file (copy from a neighbour: `// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen` / `// SPDX-License-Identifier: GPL-3.0-or-later`).
- UI copy is **Danish**, in the existing tone ("Ingen kanter endnu — kør Find kanter"); button/action text goes through `NavItem::escape_mnemonic`. Numbers shown Danish (`format_decimal_da`, `format_count`), parsed with `std::from_chars` / `reusex::utils::parse_number` — never `stod`/`strtod`.
- Never touch `ProjectSession::db()` off the GUI thread: detached work opens its own `reusex::ProjectDB`, holds a `BackgroundWork`, and posts back with `QMetaObject::invokeMethod(qApp, …, Qt::QueuedConnection)` guarded by a `QPointer`.
- Every write to the project holds the pipeline writer lease (`JobRunner::try_acquire_writer(EdgeEditor::kLeaseTimeout)`), as `EdgeEditor::save()` does.
- Spec values, verbatim: residual colour scale is fixed — green ≤ χ²₆(0.95)/2 (= 6.2958), red ≥ 5× that (= 31.479); metric modes use the 5th–95th percentile of drawn edges; rejected edges hidden by default, shown dashed when toggled on; unknown edge types get their own swatch; settings key `view.yaw_deg` (display only); QSettings keys under `rediger/…`.
- Rail order and names: **Start, Oversigt, 3D, Rediger, Pipeline, Log** (`Workspace::database` keeps index 1 and is labelled "Oversigt"; `Workspace::edit` takes index 3). `RUX_QT_PAGE` accepts `start|oversigt|database|3d|rediger|pipeline|log`; `posegraf` is gone.
- Gallery pages never touch `QSettings` (`ShellOptions::persist_layout = false`) and only write to the project **copy** `qt_shot.sh` makes.
- Tests: Catch2 v3 in `tests/unit/rux_qt/` (light binary), tags `[rux_qt][…]`, temp files via `tests/support/temp_path.hpp`.
- Every UI task ends with the design-studio loop: `qt_shot.sh` both themes → **Read every PNG** → fix the 3–5 biggest problems → re-shoot → `token_lint.py` → `ctest`/unit run.
- Build/test commands (from the worktree root, inside `nix develop` or prefixed with `nix develop -c`):
  - light unit tests: `cmake --build build --target reusex_unit_tests && build/tests/reusex_unit_tests "[rux_qt]"`
  - shots: `bash .claude/skills/design-studio/scripts/qt_shot.sh --page <name> --theme both` (builds the gallery when stale; PNGs in `shots/qt/`)
  - app smoke: `cmake --build build --target rux` then `XDG_CONFIG_HOME=$(mktemp -d) QT_QPA_PLATFORM=offscreen RUX_QT_PAGE=rediger build/apps/rux/rux -p "$COPY" --quit-after-ms 3000` where `COPY` is a scratch copy of `~/.cache/reusex/qt-shot/office_corridor-*.rux`.
- Commit messages end with the attribution lines the executing session's system reminder specifies.

## Review Focus

1. **Big graphs** (thousands of nodes, 10 000+ edges): a colour-mode, plane or yaw change, a staged edit, or a list re-sort must stay interactive; device-space node painting must not regress paint time. Pinned by the `rediger-stress` gallery page's measurement (Task 10, Step 9: < 50 ms/frame at 1440×900, a staged edit < 100 ms).
2. **Graphs with no metrics** (legacy rows: `transform`/`source`/metrics NULL, never optimized; or 0–1 edges with a metric): metric colour modes must show "Ingen måling" in a neutral colour and a sane colour bar, never all-red or a crash; percentile of 0 or 1 values is defined. Pinned in Task 5 (`EdgeColour_MetricModes_WithoutValues_AreNeutral`, `Percentile_EdgeCases`).
3. **Read-only or locked project**: staging, colouring and yaw changes all work; saving fails with the Danish banner and keeps every edit pending; the yaw applies in memory and is not written. Pinned in Task 4 (`EdgeSave_LockedProject_…` rewritten on the new ops, `EdgeSave_UpsertOverTombstone_RefusesWithoutRevive`) and the `rediger-readonly` gallery page (Task 9, Step 8).
4. **Frames without pose, depth or a horizontal heading** (a camera looking straight down in plan view; identity `local_transform`; a frame with no depth): the node still draws, its tick is hidden; the overlay shows a Danish reason instead of an empty canvas. Pinned in Task 2 (`HeadingInView_LookingAlongTheViewAxis_IsHidden`) and Task 16 (`FrameWorldPoints_NoDepth_Throws`), Task 18 Step 7.
5. **An edge stored in the opposite direction or tombstoned**: adding A→B where B→A of that type exists must re-measure that edge with the inverted transform, and adding over a rejected key must revive it — never a duplicate reversed edge, never a silent no-op. Pinned in Task 4 (`PendingEdits_AddOverRejected_Revives`) and Task 15 (`OrientedEdge_ReversedStoredEdge_IsReMeasuredInverted`).

---

## File Structure

New Qt-free logic (`rux_qt_core`, each with a test file in `tests/unit/rux_qt/`):

| File | Responsibility |
|---|---|
| `apps/rux/qt/include/rux_qt/graph_view.hpp`, `src/core/graph_view.cpp` | zoom limits; view plane + yaw projection; camera forward + heading tick; auto yaw; yaw setting text; A/B `SlotPicker` |
| `apps/rux/qt/include/rux_qt/edge_colouring.hpp`, `src/core/edge_colouring.cpp` | colour modes, χ² thresholds, percentile scale, per-edge style, legend buckets |
| `apps/rux/qt/include/rux_qt/edge_table.hpp`, `src/core/edge_table.cpp` | edge-list columns, Danish cell text, sort (missing last), filter |
| `apps/rux/qt/include/rux_qt/pair_alignment.hpp`, `src/core/pair_alignment.cpp` | 4×4 helpers, manual offset ↔ matrix about a pivot, nudges, edge measurement, ICP information, orienting a new edge onto an existing reversed one |
| `apps/rux/qt/include/rux_qt/database_logic.hpp` (modify) | `EdgeRecord` v2, `PendingEdgeEdits` v2 (upsert / status ops), `pending_summary_da`, `fit_within` |
| `apps/rux/qt/include/rux_qt/palette_table.hpp` (modify) | page names (Oversigt, Rediger), `EditPane`, pane palette entries, `smoke_page_index` |

Qt layer (`rux_qt_lib`):

| File | Responsibility |
|---|---|
| `include/rux_qt/edge_store.hpp` (header-only, new) | `PoseGraphEdge` ↔ `EdgeRecord`, `apply_edge_ops()` — shared by `EdgeEditor` and the save test |
| `include/rux_qt/EditWorkspace.hpp`, `src/EditWorkspace.cpp` (new) | the Rediger page: toolbar, splitters, pane visibility + persistence, slot picking, wiring |
| `include/rux_qt/GraphPanel.hpp`, `src/GraphPanel.cpp` (renamed from `PoseGraphWorkspace.*`) | `PoseGraphView` canvas + plane/yaw/colour toolbar + legend |
| `include/rux_qt/EdgeListPanel.hpp`, `src/EdgeListPanel.cpp` (new) | `EdgeListModel` + filter bar + table |
| `include/rux_qt/PendingBanner.hpp`, `src/PendingBanner.cpp` (new, moved out of `DatabaseWorkspace`) | "N ændringer venter" banner with Gem/Kassér |
| `include/rux_qt/PairStrip.hpp` (new, declaration moved out of `FrameBrowser.hpp`), `src/PairStrip.cpp` | edge rows of the pair, ICP, add/confirm/reject |
| `include/rux_qt/PairImages.hpp`, `src/PairImages.cpp` (new) | A \| B images with colour/depth toggle |
| `include/rux_qt/PairOverlay.hpp`, `src/PairOverlay.cpp` (new) | 3D overlay, manual ΔT (box widget, fields, keys) |
| `include/rux_qt/ViewSettings.hpp`, `src/ViewSettings.cpp` (new) | project view yaw: read on open, debounced off-thread write |
| `include/rux_qt/PanoramaViewer.hpp`, `src/PanoramaViewer.cpp` (new) | off-thread equirect decode, pan/zoom view |
| `include/rux_qt/widgets.hpp`, `src/widgets.cpp` (modify) | `PaneFrame` (titled pane with collapse button) |

Library (small, P4/P5 only): `libs/reusex/include/visualize/scene.hpp` + `render_view.hpp` (`yaw_deg`), `libs/reusex/include/slam/frame_pair_icp.hpp` (`frame_world_points`).

---

# Phase P4 — Rediger

### Task 1: Reproduce and fix "nodes disappear when zoomed in"; clamp the zoom

The spec requires the bug be reproduced and its root cause found **before** any fix. Do this against today's `PoseGraphWorkspace` (before Task 9 rewrites it).

**Files:**
- Create: `apps/rux/qt/include/rux_qt/graph_view.hpp`, `apps/rux/qt/src/core/graph_view.cpp`, `tests/unit/rux_qt/test_graph_view.cpp`
- Modify: `apps/rux/qt/CMakeLists.txt` (rux_qt_core list), `apps/rux/qt/include/rux_qt/PoseGraphWorkspace.hpp` (accessor), `apps/rux/qt/src/PoseGraphWorkspace.cpp:66-114` (`NodesItem`), `:255-260` (`wheelEvent`), `apps/rux/qt/gallery/shell_pages.cpp` (harness page)

**Interfaces:**
- Produces: `rux::qt::ZoomLimits{double min_units_per_px, max_units_per_px}`, `ZoomLimits zoom_limits(double content_extent, int viewport_px)`, `double clamp_zoom_factor(double units_per_px, double factor, const ZoomLimits&)`, `constexpr double kMinUnitsPerPx = 0.001`; `QPointF PoseGraphWorkspace::node_scene_pos(int index) const`.

- [ ] **Step 1: Add the reproduction harness page**

In `PoseGraphWorkspace.hpp`, public section, add:

```cpp
  /// Scene position of node @p index (the gallery's zoom harness).
  QPointF node_scene_pos(int index) const {
    const auto &n = nodes_[static_cast<std::size_t>(index)];
    return {n.x, -n.y};
  }
```

In `gallery/shell_pages.cpp`, after `make_posegraf_click`, add (and `#include <rux_qt/Theme.hpp>`):

```cpp
/// Reproduces "the nodes disappear when zoomed in" (spec §5.2): zoom about
/// the middle node by setting the view transform directly (no clamp), and
/// count the pixels of the node's own colour within 6 px of where it should
/// be. stderr gets one line per level; 0 means the node vanished.
QWidget *make_posegraf_zoom(const PageContext &ctx) {
  auto *shell = make_pg(ctx);
  auto *probe = new QTimer(shell);
  probe->setInterval(50);
  QObject::connect(probe, &QTimer::timeout, shell, [shell, probe] {
    PoseGraphWorkspace *pg = shell->posegraph();
    PoseGraphView *v = pg->view();
    if (!shell->isVisible() || pg->node_count() == 0)
      return;
    probe->stop();
    const QPointF at = pg->node_scene_pos(pg->node_count() / 2);
    const QColor ink = theme().color("--color-on-chrome-muted");
    v->fit();
    const double fit_px_per_unit = 1.0 / v->units_per_pixel();
    for (const double level : {0.0625, 0.25, 1.0, 4.0, 16.0, 64.0, 256.0,
                               1024.0, 4096.0, 16384.0, 65536.0}) {
      v->setTransform(QTransform::fromScale(fit_px_per_unit * level,
                                            fit_px_per_unit * level));
      v->centerOn(at);
      QApplication::processEvents();
      const QPixmap shot = v->viewport()->grab();
      const QImage img = shot.toImage();
      const double dpr = shot.devicePixelRatio();
      const QPoint c = v->mapFromScene(at);
      int lit = 0;
      for (int dy = -6; dy <= 6; ++dy)
        for (int dx = -6; dx <= 6; ++dx) {
          const QPoint p((c.x() + dx) * dpr, (c.y() + dy) * dpr);
          if (!img.rect().contains(p))
            continue;
          const QColor px = img.pixelColor(p);
          lit += std::abs(px.red() - ink.red()) + std::abs(px.green() - ink.green()) +
                     std::abs(px.blue() - ink.blue()) < 40;
        }
      std::fprintf(stderr, "posegraf-zoom: x%g upp=%.3g centre=(%d,%d) node-pixels=%d\n",
                   level, v->units_per_pixel(), c.x(), c.y(), lit);
    }
  });
  probe->start();
  return shell;
}
```

Register it next to `posegraf-click`:

```cpp
  register_page({"posegraf-zoom",
                 "Posegraf: zoomer ind på én node og tæller dens pixels "
                 "(reproduktion af forsvindende noder)",
                 make_posegraf_zoom});
```

- [ ] **Step 2: Run the harness and record the failure**

Run: `bash .claude/skills/design-studio/scripts/qt_shot.sh --page posegraf-zoom --theme dark 2>&1 | grep posegraf-zoom`
Expected: one line per level; at some level `node-pixels=0` while `centre` is inside the viewport. Write the first failing level and its `upp` into your notes. If no level fails on the fixture (10 frames), rerun with a real RTABMap project copy: `--project <copy of a real .rux with >1000 frames>`. If it still never fails, also try zooming *out* (levels 0.0625/0.25 are included) and a 2560×1440 size (`--size 2560x1440`). Do not continue to Step 4 until a level reproduces `node-pixels=0`.

- [ ] **Step 3: Isolate the cause — one experiment at a time, revert each**

Run the harness after each temporary change (do not commit them):

- **E1 — item culling.** In `NodesItem::boundingRect()` return `QRectF(-1e7, -1e7, 2e7, 2e7)`. Nodes come back ⇒ the scene drops the item because its bounding rect misses the exposed rect.
- **E2 — how points are stroked.** `QPaintEngineEx::drawPoints` strokes each point as a segment `1/63` **scene units** long with the pen; with a cosmetic round-cap pen under a large world scale that segment is many pixels long or is clipped by the cosmetic stroker. Replace the `drawPoints` loop body with device-space dots (the Step 5 code). Nodes come back ⇒ this is it.
- **E3 — coordinate precision / scroll range.** Log `v->transform()`, `v->sceneRect()` and `v->mapFromScene(at)` at the failing level. A centre outside the viewport or a non-finite transform ⇒ `fit()`'s fixed `setSceneRect` limits `centerOn`, or the scale overflowed.

Write the confirmed cause as a two-line comment above `NodesItem` and keep it for the commit message.

- [ ] **Step 4: Write the failing zoom-limit tests**

Create `tests/unit/rux_qt/test_graph_view.cpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

// Qt-free logic of the Rediger graph canvas: zoom limits, the view
// projection and yaw, heading ticks, the A/B slot picker.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <rux_qt/graph_view.hpp>

#include <array>
#include <cmath>
#include <vector>

using namespace rux::qt;
using Catch::Approx;

TEST_CASE("ZoomLimits_ScaleWithTheGraphAndTheViewport",
          "[rux_qt][graphview][zoom]") {
  const auto z = zoom_limits(50.0, 1000);
  CHECK(z.min_units_per_px == Approx(kMinUnitsPerPx));
  CHECK(z.max_units_per_px == Approx(50.0 * 4.0 / 1000.0));
  // A tiny graph still allows a 1 m view; a zero viewport does not divide by 0.
  CHECK(zoom_limits(0.0, 1000).max_units_per_px == Approx(4.0 / 1000.0));
  CHECK(zoom_limits(10.0, 0).max_units_per_px == Approx(40.0));
}

TEST_CASE("ClampZoomFactor_StopsAtTheLimits", "[rux_qt][graphview][zoom]") {
  const ZoomLimits z{0.001, 0.2};
  CHECK(clamp_zoom_factor(0.01, 2.0, z) == Approx(2.0));    // inside: as asked
  CHECK(clamp_zoom_factor(0.002, 4.0, z) == Approx(2.0));   // lands on min
  CHECK(clamp_zoom_factor(0.1, 0.25, z) == Approx(0.5));    // lands on max
  CHECK(clamp_zoom_factor(0.0005, 1.0, z) == Approx(0.5));  // already past: back in
  CHECK(clamp_zoom_factor(0.0, 2.0, z) == Approx(1.0));     // degenerate: no-op
  CHECK(clamp_zoom_factor(0.01, -1.0, z) == Approx(1.0));
}
```

Add to `tests/` nothing else: the light binary globs `tests/unit/**/*.cpp`.

- [ ] **Step 5: Run to verify it fails**

Run: `cmake --build build --target reusex_unit_tests`
Expected: compile error `rux_qt/graph_view.hpp: No such file or directory`.

- [ ] **Step 6: Implement the zoom limits**

`apps/rux/qt/include/rux_qt/graph_view.hpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Qt-free geometry of the Rediger graph canvas (spec §5.2): how far the view
// may zoom, how a 3D pose lands on the chosen view plane after the view yaw,
// the heading tick, the "Auto" yaw, the yaw's text in project_settings, and
// which of A/B a node click fills. Standard library only (rux_qt_core).

namespace rux::qt {

// ----------------------------------------------------------------- zoom --

/// The closest zoom: one millimetre per screen pixel.
inline constexpr double kMinUnitsPerPx = 0.001;

struct ZoomLimits {
  double min_units_per_px = kMinUnitsPerPx;
  double max_units_per_px = 1.0;
};

/// Limits for a graph @p content_extent metres across shown in a viewport
/// @p viewport_px pixels across: in to kMinUnitsPerPx, out to four graphs
/// (at least 4 m) across the viewport.
ZoomLimits zoom_limits(double content_extent, int viewport_px);

/// The scale factor to apply for a requested @p factor (> 1 zooms in) at
/// @p units_per_px, clamped so the result stays within @p limits. 1 when
/// either input is not positive.
double clamp_zoom_factor(double units_per_px, double factor,
                         const ZoomLimits &limits);

} // namespace rux::qt
```

`apps/rux/qt/src/core/graph_view.cpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <rux_qt/graph_view.hpp>

#include <algorithm>
#include <cmath>

namespace rux::qt {

ZoomLimits zoom_limits(double content_extent, int viewport_px) {
  ZoomLimits z;
  const double px = static_cast<double>(std::max(viewport_px, 1));
  z.max_units_per_px =
      std::max(std::max(content_extent, 1.0) * 4.0 / px, z.min_units_per_px);
  return z;
}

double clamp_zoom_factor(double units_per_px, double factor,
                         const ZoomLimits &limits) {
  if (!(units_per_px > 0.0) || !(factor > 0.0))
    return 1.0;
  const double target = std::clamp(units_per_px / factor,
                                   limits.min_units_per_px,
                                   limits.max_units_per_px);
  return units_per_px / target;
}

} // namespace rux::qt
```

Add `src/core/graph_view.cpp` to the `add_library(rux_qt_core STATIC …)` list in `apps/rux/qt/CMakeLists.txt` (after `src/core/workspace_logic.cpp`).

- [ ] **Step 7: Run the tests**

Run: `cmake --build build --target reusex_unit_tests && build/tests/reusex_unit_tests "[graphview]"`
Expected: PASS (2 test cases).

- [ ] **Step 8: Fix the cause found in Step 3, and clamp the wheel**

If E2 (or E3's precision branch) was confirmed — the expected outcome — paint the nodes in device space. Replace `NodesItem::paint` with:

```cpp
  void paint(QPainter *p, const QStyleOptionGraphicsItem *opt,
             QWidget *) override {
    // Device-space dots: QPaintEngineEx::drawPoints strokes each point as a
    // 1/63-unit segment in SCENE units, which under a large view scale is no
    // longer a dot (see the harness page posegraf-zoom). Map each node to
    // pixels and draw a fixed-size disc instead.
    const QTransform to_device = p->worldTransform();
    const double r = kNodePx;
    const QRectF visible = to_device.mapRect(opt->exposedRect)
                               .adjusted(-2 * r, -2 * r, 2 * r, 2 * r);
    const int nslots = label_slots();
    p->save();
    p->resetTransform();
    p->setRenderHint(QPainter::Antialiasing);
    p->setPen(Qt::NoPen);
    std::size_t begin = 0;
    while (begin < points_.size()) {
      const int scan = scans_.empty() ? 0 : scans_[begin];
      std::size_t end = begin + 1;
      while (end < points_.size() && (scans_.empty() || scans_[end] == scan))
        ++end;
      p->setBrush(scan <= 1 ? theme().color("--color-on-chrome-muted")
                            : theme().color(QString("--label-%1").arg(scan % nslots)));
      for (std::size_t i = begin; i < end; ++i) {
        const QPointF d = to_device.map(points_[i]);
        if (visible.contains(d))
          p->drawEllipse(d, r, r);
      }
      begin = end;
    }
    p->restore();
  }
```

and in the `NodesItem` constructor add `setFlag(ItemUsesExtendedStyleOption);` (so `exposedRect` is filled). If E1 was confirmed instead, additionally make `boundingRect()` return `bounds_.adjusted(-pad_, -pad_, pad_, pad_)` with `pad_` recomputed in `paint()` from `kNodePx / p->worldTransform().m11()` and `prepareGeometryChange()` when it grows — but keep the device-space painting either way (Task 9 needs it for the ticks).

Replace `PoseGraphView::wheelEvent` with (add `#include <rux_qt/graph_view.hpp>`):

```cpp
void PoseGraphView::wheelEvent(QWheelEvent *e) {
  const double want = std::pow(1.0015, e->angleDelta().y());
  const double extent = std::max(content_.width(), content_.height());
  const int px = std::max(viewport()->width(), viewport()->height());
  const double f =
      clamp_zoom_factor(units_per_pixel(), want, zoom_limits(extent, px));
  if (f != 1.0)
    scale(f, f);
  user_view_ = true;
  e->accept();
}
```

- [ ] **Step 9: Verify the fix with the harness and the existing pages**

Run: `bash .claude/skills/design-studio/scripts/qt_shot.sh --page posegraf-zoom --theme dark 2>&1 | grep posegraf-zoom`
Expected: `node-pixels` > 0 at **every** level.
Run: `bash .claude/skills/design-studio/scripts/qt_shot.sh --page posegraf --page posegraf-stress --theme both`
Expected: exit 0; Read `shots/qt/posegraf-dark.png` and `-light.png`: dots the same size as before; stress line on stderr not slower than before by more than 20 %.

- [ ] **Step 10: Commit**

```bash
git add apps/rux/qt/include/rux_qt/graph_view.hpp apps/rux/qt/src/core/graph_view.cpp \
  apps/rux/qt/CMakeLists.txt apps/rux/qt/include/rux_qt/PoseGraphWorkspace.hpp \
  apps/rux/qt/src/PoseGraphWorkspace.cpp apps/rux/qt/gallery/shell_pages.cpp \
  tests/unit/rux_qt/test_graph_view.cpp
git commit -m "qt: fix vanishing Posegraf nodes at high zoom; clamp the zoom

Root cause: <the confirmed experiment from Step 3, one sentence>.
Nodes are now painted as device-space discs; the wheel zoom is clamped
to 1 mm/px .. four graph extents (rux_qt_core zoom_limits)."
```

---

### Task 2: View projection, heading tick and view yaw (Qt-free)

**Files:**
- Modify: `apps/rux/qt/include/rux_qt/graph_view.hpp`, `apps/rux/qt/src/core/graph_view.cpp`, `tests/unit/rux_qt/test_graph_view.cpp`

**Interfaces:**
- Consumes: Task 1's `graph_view.hpp`.
- Produces:
  - `enum class ViewPlane { xy, xz, yz }`; `std::string_view view_plane_key(ViewPlane)`; `std::optional<ViewPlane> parse_view_plane(std::string_view)`
  - `struct ViewProjection { ViewPlane plane = ViewPlane::xy; double yaw_deg = 0.0; }`; `struct Vec2 { double x = 0.0, y = 0.0; }`
  - `Vec2 project_point(const ViewProjection &, double x, double y, double z)` (math axes, +y up)
  - `std::array<double, 3> camera_forward(const std::array<double, 16> &world_pose, const std::array<double, 16> &local_transform)`
  - `struct Heading { Vec2 dir; bool visible = false; }`; `Heading heading_in_view(const ViewProjection &, const std::array<double, 3> &forward)`
  - `double auto_yaw_deg(const std::vector<std::array<double, 3>> &forwards)` in (−45, 45]
  - `double normalize_yaw_deg(double)` in (−180, 180]; `inline constexpr std::string_view kViewYawSetting = "view.yaw_deg"`; `std::optional<double> parse_yaw_setting(std::string_view)`; `std::string format_yaw_setting(double)`

- [ ] **Step 1: Write the failing tests** (append to `test_graph_view.cpp`)

```cpp
namespace {
std::array<double, 16> yaw_pose(double deg, double tx = 0, double ty = 0,
                                double tz = 0) {
  const double a = deg * 3.14159265358979323846 / 180.0;
  return {std::cos(a), -std::sin(a), 0, tx, std::sin(a), std::cos(a), 0, ty,
          0, 0, 1, tz, 0, 0, 0, 1};
}
// RTABMap's optical-from-base: optical z (forward) = base x, x = -base y,
// y = -base z.
const std::array<double, 16> kOpticalInBase = {0, 0, 1, 0, -1, 0, 0, 0,
                                               0, -1, 0, 0, 0, 0, 0, 1};
const std::array<double, 16> kIdentity = {1, 0, 0, 0, 0, 1, 0, 0,
                                          0, 0, 1, 0, 0, 0, 0, 1};
} // namespace

TEST_CASE("ProjectPoint_PlanesPickTwoAxesAfterTheYaw",
          "[rux_qt][graphview][projection]") {
  ViewProjection v;
  CHECK(project_point(v, 1, 2, 3).x == Approx(1));
  CHECK(project_point(v, 1, 2, 3).y == Approx(2));
  v.plane = ViewPlane::xz; // seen from -y: x right, z up
  CHECK(project_point(v, 1, 2, 3).x == Approx(1));
  CHECK(project_point(v, 1, 2, 3).y == Approx(3));
  v.plane = ViewPlane::yz; // seen from +x: y right, z up
  CHECK(project_point(v, 1, 2, 3).x == Approx(2));
  CHECK(project_point(v, 1, 2, 3).y == Approx(3));
  // The yaw turns the model counter-clockwise (seen from above) first.
  v = {ViewPlane::xy, 90.0};
  CHECK(project_point(v, 1, 0, 0).x == Approx(0).margin(1e-12));
  CHECK(project_point(v, 1, 0, 0).y == Approx(1));
  v.plane = ViewPlane::xz;
  CHECK(project_point(v, 1, 0, 5).x == Approx(0).margin(1e-12));
  CHECK(project_point(v, 1, 0, 5).y == Approx(5));
}

TEST_CASE("ViewPlane_KeysRoundTrip", "[rux_qt][graphview][projection]") {
  for (ViewPlane p : {ViewPlane::xy, ViewPlane::xz, ViewPlane::yz})
    CHECK(parse_view_plane(view_plane_key(p)) == p);
  CHECK_FALSE(parse_view_plane("zz").has_value());
}

TEST_CASE("CameraForward_UsesTheOpticalAxisThroughLocalTransform",
          "[rux_qt][graphview][heading]") {
  auto f = camera_forward(kIdentity, kOpticalInBase);
  CHECK(f[0] == Approx(1));
  CHECK(f[1] == Approx(0).margin(1e-12));
  f = camera_forward(yaw_pose(90), kOpticalInBase);
  CHECK(f[0] == Approx(0).margin(1e-12));
  CHECK(f[1] == Approx(1));
  // Identity local transform: the optical axis is the pose's own z.
  f = camera_forward(kIdentity, kIdentity);
  CHECK(f[2] == Approx(1));
}

TEST_CASE("HeadingInView_ProjectsTheForwardAxis",
          "[rux_qt][graphview][heading]") {
  const auto h = heading_in_view({ViewPlane::xy, 0.0}, {1, 0, 0});
  CHECK(h.visible);
  CHECK(h.dir.x == Approx(1));
  const auto r = heading_in_view({ViewPlane::xy, 90.0}, {1, 0, 0});
  CHECK(r.dir.y == Approx(1));
  const auto e = heading_in_view({ViewPlane::xz, 0.0}, {0.6, 0, 0.8});
  CHECK(e.visible);
  CHECK(e.dir.x == Approx(0.6));
  CHECK(e.dir.y == Approx(0.8));
}

TEST_CASE("HeadingInView_LookingAlongTheViewAxis_IsHidden",
          "[rux_qt][graphview][heading]") {
  // A camera looking straight down has no direction in the plan view.
  CHECK_FALSE(heading_in_view({ViewPlane::xy, 0.0}, {0, 0, -1}).visible);
  CHECK_FALSE(heading_in_view({ViewPlane::xy, 0.0}, {0.1, 0, -0.995}).visible);
  CHECK_FALSE(heading_in_view({ViewPlane::xy, 0.0}, {0, 0, 0}).visible);
}

TEST_CASE("AutoYaw_AlignsTheDominantAxisWithTheScreen",
          "[rux_qt][graphview][yaw]") {
  auto dir = [](double deg) {
    const double a = deg * 3.14159265358979323846 / 180.0;
    return std::array<double, 3>{std::cos(a), std::sin(a), 0.0};
  };
  CHECK(auto_yaw_deg({dir(30), dir(30), dir(31), dir(29)}) ==
        Approx(-30).margin(0.1));
  // Perpendicular corridors are one axis system (folded mod 90°).
  CHECK(auto_yaw_deg({dir(30), dir(120), dir(210), dir(-60)}) ==
        Approx(-30).margin(0.1));
  CHECK(auto_yaw_deg({dir(45)}) == Approx(45).margin(1e-9));
  CHECK(auto_yaw_deg({dir(-45)}) == Approx(45).margin(1e-9));
  // Nothing horizontal: no rotation.
  CHECK(auto_yaw_deg({}) == 0.0);
  CHECK(auto_yaw_deg({{0, 0, 1}, {0.1, 0, -1}}) == 0.0);
}

TEST_CASE("YawSetting_ParsesAndFormatsWithoutTheLocale",
          "[rux_qt][graphview][yaw]") {
  CHECK(parse_yaw_setting("12.5") == Approx(12.5));
  CHECK(parse_yaw_setting(" -190 ") == Approx(170));
  CHECK_FALSE(parse_yaw_setting("12,5").has_value());
  CHECK_FALSE(parse_yaw_setting("abc").has_value());
  CHECK_FALSE(parse_yaw_setting("nan").has_value());
  CHECK_FALSE(parse_yaw_setting("").has_value());
  CHECK(format_yaw_setting(12.54) == "12.5");
  CHECK(format_yaw_setting(190.0) == "-170.0");
  CHECK(format_yaw_setting(-0.04) == "0.0");
  CHECK(format_yaw_setting(180.0) == "180.0");
  CHECK(normalize_yaw_deg(-180.0) == Approx(180.0));
  CHECK(normalize_yaw_deg(720.0) == Approx(0.0));
}
```

- [ ] **Step 2: Run to verify it fails**

Run: `cmake --build build --target reusex_unit_tests`
Expected: compile errors (`ViewProjection`, `project_point` … not declared).

- [ ] **Step 3: Implement**

Append to `graph_view.hpp` (add `#include <array> <optional> <string> <string_view> <vector>` at the top):

```cpp
// ----------------------------------------------------------- projection --

/// Which two axes the canvas shows: xy = plan, xz = seen from -y (x right,
/// z up), yz = seen from +x (y right, z up).
enum class ViewPlane { xy, xz, yz };
std::string_view view_plane_key(ViewPlane plane); ///< "xy" | "xz" | "yz"
std::optional<ViewPlane> parse_view_plane(std::string_view key);

struct ViewProjection {
  ViewPlane plane = ViewPlane::xy;
  /// Display rotation of the model about world z, degrees, counter-clockwise
  /// seen from above (project_settings view.yaw_deg).
  double yaw_deg = 0.0;
};

struct Vec2 {
  double x = 0.0;
  double y = 0.0; ///< up (the canvas negates it for the screen)
};

/// World point -> canvas coordinates: rotate by the yaw, then pick the plane.
Vec2 project_point(const ViewProjection &view, double x, double y, double z);

/// The camera's forward (optical +z) axis in world coordinates: the rotation
/// of @p world_pose (sensor_frames.transform) times that of
/// @p local_transform (the optical frame in the pose frame), applied to z.
std::array<double, 3> camera_forward(const std::array<double, 16> &world_pose,
                                     const std::array<double, 16> &local_transform);

struct Heading {
  Vec2 dir;             ///< unit length when visible
  bool visible = false; ///< false when the camera looks along the view axis
};

/// Direction of @p forward on the canvas. Hidden when less than a quarter of
/// it lies in the view plane (it would be a meaningless stub).
Heading heading_in_view(const ViewProjection &view,
                        const std::array<double, 3> &forward);

// ------------------------------------------------------------------ yaw --

/// The yaw (degrees, in (-45, 45]) that turns the dominant horizontal
/// direction of @p forwards onto the screen's x axis. Directions are folded
/// mod 90° (perpendicular corridors are one axis system) and weighted by how
/// horizontal they are; ones more than 60° off horizontal are ignored. 0
/// when nothing is horizontal.
double auto_yaw_deg(const std::vector<std::array<double, 3>> &forwards);

/// @p deg wrapped into (-180, 180]; 0 for a non-finite value.
double normalize_yaw_deg(double deg);

/// project_settings key of the view yaw (spec D7).
inline constexpr std::string_view kViewYawSetting = "view.yaw_deg";
/// Locale-independent parse of a stored yaw ('.' decimal), normalised;
/// nullopt for anything that is not exactly one finite number.
std::optional<double> parse_yaw_setting(std::string_view text);
/// The stored text: normalised, one decimal, '.' separator ("-12.5").
std::string format_yaw_setting(double deg);
```

Append to `graph_view.cpp` (add `#include <charconv> <numbers> <system_error>`):

```cpp
namespace {
constexpr double kDeg = std::numbers::pi / 180.0;

Vec2 pick(ViewPlane plane, double x, double y, double z) {
  switch (plane) {
  case ViewPlane::xy:
    return {x, y};
  case ViewPlane::xz:
    return {x, z};
  case ViewPlane::yz:
    return {y, z};
  }
  return {x, y};
}
} // namespace

std::string_view view_plane_key(ViewPlane plane) {
  switch (plane) {
  case ViewPlane::xy:
    return "xy";
  case ViewPlane::xz:
    return "xz";
  case ViewPlane::yz:
    return "yz";
  }
  return "xy";
}

std::optional<ViewPlane> parse_view_plane(std::string_view key) {
  for (ViewPlane p : {ViewPlane::xy, ViewPlane::xz, ViewPlane::yz})
    if (view_plane_key(p) == key)
      return p;
  return std::nullopt;
}

Vec2 project_point(const ViewProjection &view, double x, double y, double z) {
  const double a = view.yaw_deg * kDeg;
  const double c = std::cos(a), s = std::sin(a);
  return pick(view.plane, x * c - y * s, x * s + y * c, z);
}

std::array<double, 3> camera_forward(const std::array<double, 16> &w,
                                     const std::array<double, 16> &l) {
  // Third column of the local rotation, then the world rotation.
  const double v0 = l[2], v1 = l[6], v2 = l[10];
  return {w[0] * v0 + w[1] * v1 + w[2] * v2, w[4] * v0 + w[5] * v1 + w[6] * v2,
          w[8] * v0 + w[9] * v1 + w[10] * v2};
}

Heading heading_in_view(const ViewProjection &view,
                        const std::array<double, 3> &f) {
  const double n = std::sqrt(f[0] * f[0] + f[1] * f[1] + f[2] * f[2]);
  if (!(n > 0.0))
    return {};
  const Vec2 p = project_point(view, f[0] / n, f[1] / n, f[2] / n);
  const double len = std::hypot(p.x, p.y);
  if (len < 0.25)
    return {};
  return {{p.x / len, p.y / len}, true};
}

double auto_yaw_deg(const std::vector<std::array<double, 3>> &forwards) {
  double c = 0.0, s = 0.0;
  for (const auto &f : forwards) {
    const double h = std::hypot(f[0], f[1]);
    const double n = std::hypot(h, f[2]);
    if (!(n > 0.0) || h < 0.5 * n)
      continue; // looks mostly up or down
    const double a = 4.0 * std::atan2(f[1], f[0]);
    c += h / n * std::cos(a);
    s += h / n * std::sin(a);
  }
  if (std::hypot(c, s) < 1e-9)
    return 0.0;
  double yaw = -std::atan2(s, c) / 4.0 / kDeg;
  if (yaw <= -45.0)
    yaw += 90.0;
  if (yaw > 45.0)
    yaw -= 90.0;
  return yaw + 0.0;
}

double normalize_yaw_deg(double deg) {
  if (!std::isfinite(deg))
    return 0.0;
  double r = std::fmod(deg, 360.0);
  if (r <= -180.0)
    r += 360.0;
  else if (r > 180.0)
    r -= 360.0;
  return r + 0.0;
}

std::optional<double> parse_yaw_setting(std::string_view t) {
  while (!t.empty() && t.front() == ' ')
    t.remove_prefix(1);
  while (!t.empty() && t.back() == ' ')
    t.remove_suffix(1);
  double v = 0.0;
  const auto r = std::from_chars(t.data(), t.data() + t.size(), v);
  if (t.empty() || r.ec != std::errc{} || r.ptr != t.data() + t.size() ||
      !std::isfinite(v))
    return std::nullopt;
  return normalize_yaw_deg(v);
}

std::string format_yaw_setting(double deg) {
  const double v = normalize_yaw_deg(std::round(normalize_yaw_deg(deg) * 10.0) / 10.0);
  char buf[32];
  const auto r = std::to_chars(buf, buf + sizeof buf, v + 0.0,
                               std::chars_format::fixed, 1);
  return std::string(buf, r.ptr);
}
```

- [ ] **Step 4: Run the tests**

Run: `cmake --build build --target reusex_unit_tests && build/tests/reusex_unit_tests "[graphview]"`
Expected: PASS (all `[graphview]` cases). If `format_yaw_setting(-0.04)` prints `-0.0`, the `+ 0.0` after rounding is missing.

- [ ] **Step 5: Commit**

```bash
git add apps/rux/qt/include/rux_qt/graph_view.hpp apps/rux/qt/src/core/graph_view.cpp tests/unit/rux_qt/test_graph_view.cpp
git commit -m "qt core: view plane + yaw projection, heading tick, auto yaw"
```

---

### Task 3: A/B slot picker (Qt-free)

**Files:**
- Modify: `apps/rux/qt/include/rux_qt/graph_view.hpp`, `apps/rux/qt/src/core/graph_view.cpp`, `tests/unit/rux_qt/test_graph_view.cpp`

**Interfaces:**
- Produces: `enum class Slot { a, b }`; `class SlotPicker { Slot next() const; int a() const; int b() const; std::optional<Slot> click(int id, bool force_b); void set_pair(int a, int b); void clear(); bool complete() const; }`

- [ ] **Step 1: Write the failing test**

```cpp
TEST_CASE("SlotPicker_AlternatesAThenB", "[rux_qt][graphview][slots]") {
  SlotPicker s;
  CHECK(s.next() == Slot::a);
  CHECK(s.click(10, false) == Slot::a);
  CHECK(s.a() == 10);
  CHECK(s.next() == Slot::b);
  CHECK_FALSE(s.complete());
  CHECK(s.click(20, false) == Slot::b);
  CHECK(s.b() == 20);
  CHECK(s.complete());
  CHECK(s.next() == Slot::a);
  CHECK(s.click(30, false) == Slot::a); // the next click replaces A
  CHECK(s.a() == 30);
  CHECK(s.b() == 20);
}

TEST_CASE("SlotPicker_ShiftForcesB_AndTheNextClickIsA",
          "[rux_qt][graphview][slots]") {
  SlotPicker s;
  s.click(10, false);
  s.click(20, false);
  CHECK(s.click(40, true) == Slot::b);
  CHECK(s.b() == 40);
  CHECK(s.next() == Slot::a);
}

TEST_CASE("SlotPicker_TheSameFrameNeverFillsBothSlots",
          "[rux_qt][graphview][slots]") {
  SlotPicker s;
  s.click(10, false);                  // A = 10, next B
  CHECK_FALSE(s.click(10, false).has_value());
  CHECK(s.b() == -1);
  CHECK(s.next() == Slot::b);          // still waiting for B
  CHECK_FALSE(s.click(10, true).has_value());
}

TEST_CASE("SlotPicker_SetPair_FromAnEdge", "[rux_qt][graphview][slots]") {
  SlotPicker s;
  s.click(1, false);
  s.set_pair(5, 9);
  CHECK(s.a() == 5);
  CHECK(s.b() == 9);
  CHECK(s.next() == Slot::a);
  s.clear();
  CHECK(s.a() == -1);
  CHECK(s.b() == -1);
  CHECK(s.next() == Slot::a);
}
```

- [ ] **Step 2: Run to verify it fails**

Run: `cmake --build build --target reusex_unit_tests`
Expected: compile error (`SlotPicker` not declared).

- [ ] **Step 3: Implement**

Header:

```cpp
// --------------------------------------------------------------- slots --

enum class Slot { a, b };

/// Which of the pair a node click fills (spec §5.2): A, then B, alternating;
/// Shift forces B. A frame is never both A and B.
class SlotPicker {
    public:
  Slot next() const { return next_; }
  int a() const { return a_; }
  int b() const { return b_; }
  bool complete() const { return a_ >= 0 && b_ >= 0; }
  /// A click on frame @p id: the slot it filled, or nullopt (no change) when
  /// @p id is already in the other slot.
  std::optional<Slot> click(int id, bool force_b);
  /// Both at once (an edge or a list row); the next click is A again.
  void set_pair(int a, int b);
  void clear();

    private:
  int a_ = -1;
  int b_ = -1;
  Slot next_ = Slot::a;
};
```

Source:

```cpp
std::optional<Slot> SlotPicker::click(int id, bool force_b) {
  const Slot target = force_b ? Slot::b : next_;
  if (target == Slot::a) {
    if (id == b_)
      return std::nullopt;
    a_ = id;
    next_ = Slot::b;
    return Slot::a;
  }
  if (id == a_)
    return std::nullopt;
  b_ = id;
  next_ = Slot::a;
  return Slot::b;
}

void SlotPicker::set_pair(int a, int b) {
  a_ = a;
  b_ = b;
  next_ = Slot::a;
}

void SlotPicker::clear() { set_pair(-1, -1); }
```

- [ ] **Step 4: Run the tests**

Run: `cmake --build build --target reusex_unit_tests && build/tests/reusex_unit_tests "[slots]"`
Expected: PASS (4 cases).

- [ ] **Step 5: Commit**

```bash
git add apps/rux/qt/include/rux_qt/graph_view.hpp apps/rux/qt/src/core/graph_view.cpp tests/unit/rux_qt/test_graph_view.cpp
git commit -m "qt core: A/B slot picker for node clicks"
```

---
### Task 4: Staged edit model v2 — status, transforms, upsert; `EdgeEditor` on the new store API

The GUI "delete" becomes **reject** (spec §3.4); a staged edge carries its measurement; saving uses `upsert_pose_graph_edge` / `set_pose_graph_edge_status` in one transaction.

**Files:**
- Modify: `apps/rux/qt/include/rux_qt/database_logic.hpp:73-161`, `apps/rux/qt/src/core/database_logic.cpp` (the `PendingEdgeEdits` section), `tests/unit/rux_qt/test_database_logic.cpp:100-262,423-440`
- Create: `apps/rux/qt/include/rux_qt/edge_store.hpp`
- Modify: `apps/rux/qt/include/rux_qt/EdgeEditor.hpp`, `apps/rux/qt/src/EdgeEditor.cpp` (save thread, `reload`, staging methods)
- Modify: `tests/unit/rux_qt/test_edge_save_locked.cpp` (use `apply_edge_ops`)
- Modify: `apps/rux/qt/src/PairStrip.cpp` (row actions), `apps/rux/qt/src/PoseGraphWorkspace.cpp:499-557` (`rebuild_edges` uses `effective()`), `apps/rux/qt/src/FrameBrowser.cpp:927-938` (`refresh_strip_marks`), `apps/rux/qt/src/DatabaseWorkspace.cpp:529-545` (banner text), `apps/rux/qt/gallery/shell_pages.cpp` (`make_db_pending`: `remove` → `reject`)

**Interfaces:**
- Consumes: assumed library API (`PoseGraphEdge` v27, `upsert_pose_graph_edge`, `set_pose_graph_edge_status`).
- Produces:
  - `struct EdgeRecord { EdgeKey key; double residual = 0.0; double weight = 1.0; std::optional<std::array<double,16>> transform; std::optional<std::array<double,36>> information; std::string source; std::string status = "auto"; std::optional<double> seed_disagreement_m, seed_disagreement_deg, deviation_m, deviation_deg; }`
  - `bool same_record(const EdgeRecord &, const EdgeRecord &)` (NaN-equal weight/residual)
  - `struct EdgeView { EdgeRecord edge; enum class Pending { none, added, replaced, status } pending; std::string stored_status; bool reversed; }`
  - `class PendingEdgeEdits` with `Result { added, replaced, revived, confirmed, rejected, unrejected, unstaged, unchanged, not_found, invalid }`, `struct Op { enum class Kind { upsert, set_status } kind; EdgeRecord edge; bool overwrite_rejected; }`, `add`, `confirm`, `reject`, `unreject`, `revert`, `discard`, `ops`, `effective`, `between`, `degree`, `degrees`, `commit_saved`, `base`, `count`, `empty`
  - `std::string pending_summary_da(const std::vector<PendingEdgeEdits::Op> &)`
  - `edge_store.hpp`: `EdgeRecord edge_record_from_row(const ProjectDB::PoseGraphEdge &)`, `ProjectDB::PoseGraphEdge edge_row_from_record(const EdgeRecord &)`, `void apply_edge_ops(ProjectDB &, const std::vector<PendingEdgeEdits::Op> &)`
  - `EdgeEditor::add(const EdgeRecord &)`, `confirm(const EdgeKey &)`, `reject(const EdgeKey &)`, `unreject(const EdgeKey &)`, `revert(const EdgeKey &)` (each returns `PendingEdgeEdits::Result`); `EdgeEditor::remove` is deleted.

- [ ] **Step 1: Replace the pending-edit tests with the v2 behaviour**

In `tests/unit/rux_qt/test_database_logic.cpp`, delete the test cases `PendingEdits_AddAndRemove_StageAndCancelOut`, `PendingEdits_RejectsInvalidEdges`, `PendingEdits_Ops_DeleteFirstThenAddInStagingOrder`, `PendingEdits_CommitSaved_KeepsEditsStagedDuringTheSave`, `PendingEdits_CommitSaved_ReappliesAnUndoMadeDuringTheSave`, `PendingEdits_Between_ShowsBothDirectionsWithState` and `PendingEdits_Degrees_MatchDegreePerFrame`. Replace the `edge()` helper and add:

```cpp
namespace {

EdgeRecord edge(int from, int to, const char *type = "loop_closure",
                const char *status = "auto", double residual = 0.0,
                double weight = 1.0) {
  EdgeRecord r{{from, to, type}, residual, weight};
  r.status = status;
  return r;
}

using R = PendingEdgeEdits::Result;
using K = PendingEdgeEdits::Op::Kind;

} // namespace

TEST_CASE("PendingEdits_Add_StagesAConfirmedMeasurement",
          "[rux_qt][database][pending]") {
  PendingEdgeEdits e;
  e.set_base({edge(1, 2, "odometry"), edge(2, 3, "loop_closure", "rejected")});
  EdgeRecord m = edge(1, 5);
  m.transform = std::array<double, 16>{1, 0, 0, 0.1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  m.source = "manual";
  CHECK(e.add(m) == R::added);
  CHECK(e.count() == 1);
  const auto ops = e.ops();
  REQUIRE(ops.size() == 1);
  CHECK(ops[0].kind == K::upsert);
  CHECK(ops[0].edge.status == "confirmed"); // a user edge is confirmed (D4)
  CHECK(ops[0].edge.transform.has_value());
  CHECK_FALSE(ops[0].overwrite_rejected);
  // Staging the same key again replaces the staged measurement.
  CHECK(e.add(edge(1, 5)) == R::replaced);
  CHECK(e.count() == 1);
  // Over a stored active edge: a new measurement for it.
  CHECK(e.add(edge(1, 2, "odometry")) == R::replaced);
  CHECK(e.count() == 2);
}

TEST_CASE("PendingEdits_AddOverRejected_Revives", "[rux_qt][database][pending]") {
  PendingEdgeEdits e;
  e.set_base({edge(2, 3, "loop_closure", "rejected")});
  CHECK(e.add(edge(2, 3)) == R::revived);
  const auto ops = e.ops();
  REQUIRE(ops.size() == 1);
  CHECK(ops[0].overwrite_rejected); // the store refuses a tombstone otherwise
}

TEST_CASE("PendingEdits_RejectsInvalidEdges", "[rux_qt][database][pending]") {
  PendingEdgeEdits e;
  CHECK(e.add(edge(3, 3)) == R::invalid);
  CHECK(e.add(edge(1, 2, "bogus")) == R::invalid);
  CHECK(e.add({{1, 2, "loop_closure"}, 0.0, 0.0}) == R::invalid);
  CHECK(e.add({{1, 2, "loop_closure"}, 0.0, -1.0}) == R::invalid);
  // No ICP: the weight is unknown (NaN) and that is allowed.
  CHECK(e.add({{1, 2, "loop_closure"}, 0.0,
               std::numeric_limits<double>::quiet_NaN()}) == R::added);
}

TEST_CASE("PendingEdits_ConfirmRejectUnreject", "[rux_qt][database][pending]") {
  PendingEdgeEdits e;
  e.set_base({edge(1, 2, "odometry"), edge(4, 5, "loop_closure", "rejected"),
              edge(6, 7, "loop_closure", "confirmed")});
  CHECK(e.confirm({1, 2, "odometry"}) == R::confirmed);
  CHECK(e.confirm({6, 7, "loop_closure"}) == R::unchanged);
  CHECK(e.confirm({8, 9, "loop_closure"}) == R::not_found);
  // Rejecting a staged confirmation replaces it.
  CHECK(e.reject({1, 2, "odometry"}) == R::rejected);
  CHECK(e.count() == 1);
  CHECK(e.ops()[0].edge.status == "rejected");
  // Un-rejecting a staged rejection of an active edge just drops it.
  CHECK(e.unreject({1, 2, "odometry"}) == R::unstaged);
  CHECK(e.empty());
  // A stored tombstone comes back as auto.
  CHECK(e.unreject({4, 5, "loop_closure"}) == R::unrejected);
  CHECK(e.ops()[0].edge.status == "auto");
  CHECK(e.reject({4, 5, "loop_closure"}) == R::unstaged); // back to the file
  CHECK(e.empty());
  // A staged new edge is simply dropped on reject.
  REQUIRE(e.add(edge(10, 11)) == R::added);
  CHECK(e.reject({10, 11, "loop_closure"}) == R::unstaged);
  CHECK(e.empty());
  CHECK(e.reject({10, 11, "loop_closure"}) == R::not_found);
}

TEST_CASE("PendingEdits_Revert_DropsWhateverIsStaged",
          "[rux_qt][database][pending]") {
  PendingEdgeEdits e;
  e.set_base({edge(1, 2, "odometry")});
  e.reject({1, 2, "odometry"});
  CHECK(e.revert({1, 2, "odometry"}) == R::unstaged);
  CHECK(e.revert({1, 2, "odometry"}) == R::unchanged);
  CHECK(e.empty());
}

TEST_CASE("PendingEdits_Ops_StatusChangesFirstThenMeasurements",
          "[rux_qt][database][pending]") {
  PendingEdgeEdits e;
  e.set_base({edge(1, 2, "odometry"), edge(4, 5, "panorama")});
  REQUIRE(e.add(edge(7, 8)) == R::added);
  REQUIRE(e.reject({1, 2, "odometry"}) == R::rejected);
  REQUIRE(e.add(edge(2, 9)) == R::added);
  REQUIRE(e.confirm({4, 5, "panorama"}) == R::confirmed);
  const auto ops = e.ops();
  REQUIRE(ops.size() == 4);
  CHECK(ops[0].kind == K::set_status);
  CHECK(ops[0].edge.key == EdgeKey{1, 2, "odometry"});
  CHECK(ops[1].kind == K::set_status);
  CHECK(ops[1].edge.key == EdgeKey{4, 5, "panorama"});
  CHECK(ops[2].edge.key == EdgeKey{7, 8, "loop_closure"});
  CHECK(ops[3].edge.key == EdgeKey{2, 9, "loop_closure"});
}

TEST_CASE("PendingEdits_Effective_AppliesPendingOnTheBase",
          "[rux_qt][database][pending]") {
  PendingEdgeEdits e;
  auto stored = edge(1, 2, "odometry", "auto", 3.5, 9.0);
  stored.deviation_m = 0.4;
  e.set_base({stored, edge(2, 1, "panorama"), edge(2, 3, "odometry")});
  REQUIRE(e.reject({2, 1, "panorama"}) == R::rejected);
  REQUIRE(e.add(edge(1, 2, "loop_closure")) == R::added);
  const auto all = e.effective();
  REQUIRE(all.size() == 4);
  CHECK(all[0].pending == EdgeView::Pending::none);
  CHECK(all[0].edge.deviation_m == 0.4);
  CHECK(all[1].pending == EdgeView::Pending::status);
  CHECK(all[1].edge.status == "rejected");
  CHECK(all[1].stored_status == "auto");
  CHECK(all[3].pending == EdgeView::Pending::added);
  CHECK(all[3].stored_status.empty());

  const auto v = e.between(1, 2);
  REQUIRE(v.size() == 3);
  CHECK_FALSE(v[0].reversed);
  CHECK(v[1].reversed);
  CHECK(e.between(2, 1)[0].reversed);
  CHECK(e.between(1, 3).empty());
  // Rejected edges do not count towards a frame's degree.
  CHECK(e.degree(1) == 2); // odometry + staged loop closure
  CHECK(e.degree(2) == 3);
  CHECK(e.degree(9) == 0);
}

TEST_CASE("PendingEdits_Degrees_MatchDegreePerFrame", "[rux_qt][database]") {
  PendingEdgeEdits e;
  e.set_base({edge(1, 2, "odometry"), edge(2, 3, "odometry"),
              edge(1, 2, "odometry"), // duplicate row, counted once
              edge(3, 1, "loop_closure")});
  e.reject({2, 3, "odometry"});
  e.add(edge(4, 1));
  const auto all = e.degrees();
  for (int f : {1, 2, 3, 4}) {
    INFO(f);
    const auto it = all.find(f);
    CHECK((it == all.end() ? 0 : it->second) == e.degree(f));
  }
}

TEST_CASE("PendingEdits_CommitSaved_KeepsEditsStagedDuringTheSave",
          "[rux_qt][database][pending]") {
  PendingEdgeEdits e;
  e.set_base({edge(1, 2, "odometry"), edge(4, 5, "panorama")});
  REQUIRE(e.add(edge(7, 8)) == R::added);
  REQUIRE(e.reject({1, 2, "odometry"}) == R::rejected);
  const auto snapshot = e.ops(); // the save starts with these two
  // While it runs: one more edit, and one that changes a saved one again.
  REQUIRE(e.confirm({4, 5, "panorama"}) == R::confirmed);
  REQUIRE(e.add(edge(7, 8, "loop_closure", "auto", 0.0, 4.0)) == R::replaced);
  e.commit_saved(snapshot);
  // The base now holds the saved state.
  REQUIRE(e.base().size() == 3);
  CHECK(e.base()[0].status == "rejected");
  CHECK(e.base()[2].key == EdgeKey{7, 8, "loop_closure"});
  // What was staged during the save stays pending.
  const auto left = e.ops();
  REQUIRE(left.size() == 2);
  CHECK(left[0].kind == K::set_status);
  CHECK(left[0].edge.key == EdgeKey{4, 5, "panorama"});
  CHECK(left[1].kind == K::upsert);
  CHECK(left[1].edge.weight == 4.0);
}

TEST_CASE("PendingEdits_CommitSaved_ManualEdgeWithoutWeight_LeavesNothing",
          "[rux_qt][database][pending]") {
  // A manual edge has a NaN weight; NaN != NaN must not keep it pending.
  PendingEdgeEdits e;
  REQUIRE(e.add({{1, 2, "loop_closure"}, 0.0,
                 std::numeric_limits<double>::quiet_NaN()}) == R::added);
  e.commit_saved(e.ops());
  CHECK(e.empty());
  CHECK(e.base().size() == 1);
}

TEST_CASE("PendingSummary_NamesTheKindsInDanish", "[rux_qt][database][pending]") {
  PendingEdgeEdits e;
  e.set_base({edge(1, 2, "odometry"), edge(3, 4, "loop_closure")});
  e.add(edge(5, 6));
  e.reject({1, 2, "odometry"});
  e.confirm({3, 4, "loop_closure"});
  CHECK(pending_summary_da(e.ops()) == "1 ny eller målt kant, 1 afvist, 1 bekræftet");
  e.discard();
  e.add(edge(5, 6));
  e.add(edge(7, 6));
  CHECK(pending_summary_da(e.ops()) == "2 nye eller målte kanter");
}
```

Keep `EdgeTypes_HaveDanishNames`, `WeightFromIcp_…` and the non-edge tests unchanged.

- [ ] **Step 2: Run to verify it fails**

Run: `cmake --build build --target reusex_unit_tests`
Expected: compile errors (`R::revived`, `EdgeView::Pending`, `pending_summary_da` …).

- [ ] **Step 3: Implement the model**

In `database_logic.hpp`, replace `struct EdgeRecord`, `struct EdgeView` and `class PendingEdgeEdits` (lines 83–150) with:

```cpp
struct EdgeRecord {
  EdgeKey key;
  double residual = 0.0;
  double weight = 1.0; ///< 1/σ_trans²; NaN when unknown (a manual edge)
  /// T_from^-1 · T_to in the sensor_frames.transform convention, row-major.
  std::optional<std::array<double, 16>> transform;
  /// 6x6 information, GTSAM order (rx, ry, rz, tx, ty, tz); nullopt = the
  /// optimizer's per-type default.
  std::optional<std::array<double, 36>> information;
  std::string source;          ///< rtabmap | odometry_init | orb | … ; "" unknown
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
class PendingEdgeEdits {
    public:
  enum class Result {
    added,      ///< a new edge is staged
    replaced,   ///< a new measurement for an existing / staged key
    revived,    ///< a new measurement over a rejected edge (overwrites it)
    confirmed,  ///< a status change to confirmed is staged
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

  /// Stage a measurement (always status "confirmed", metrics cleared).
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
```

In `database_logic.cpp`, replace the whole `PendingEdgeEdits` implementation (including `commit_succeeded`) with:

```cpp
namespace {
bool same_number(double a, double b) {
  return a == b || (std::isnan(a) && std::isnan(b));
}
} // namespace

bool same_record(const EdgeRecord &a, const EdgeRecord &b) {
  return a.key == b.key && same_number(a.residual, b.residual) &&
         same_number(a.weight, b.weight) && a.transform == b.transform &&
         a.information == b.information && a.source == b.source &&
         a.status == b.status &&
         a.seed_disagreement_m == b.seed_disagreement_m &&
         a.seed_disagreement_deg == b.seed_disagreement_deg &&
         a.deviation_m == b.deviation_m && a.deviation_deg == b.deviation_deg;
}

void PendingEdgeEdits::set_base(std::vector<EdgeRecord> edges) {
  base_ = std::move(edges);
  pending_.clear();
}

const EdgeRecord *PendingEdgeEdits::stored(const EdgeKey &k) const {
  for (const auto &e : base_)
    if (e.key == k)
      return &e;
  return nullptr;
}

int PendingEdgeEdits::pending_index(const EdgeKey &k) const {
  for (std::size_t i = 0; i < pending_.size(); ++i)
    if (pending_[i].edge.key == k)
      return static_cast<int>(i);
  return -1;
}

PendingEdgeEdits::Result PendingEdgeEdits::add(const EdgeRecord &edge) {
  if (edge.key.from == edge.key.to || !is_edge_type(edge.key.type) ||
      edge.weight <= 0.0)
    return Result::invalid;
  Op op;
  op.kind = Op::Kind::upsert;
  op.edge = edge;
  op.edge.status = "confirmed";
  op.edge.residual = 0.0;
  op.edge.seed_disagreement_m.reset();
  op.edge.seed_disagreement_deg.reset();
  op.edge.deviation_m.reset();
  op.edge.deviation_deg.reset();
  const EdgeRecord *s = stored(edge.key);
  op.overwrite_rejected = s && s->status == "rejected";
  if (const int i = pending_index(edge.key); i >= 0) {
    const bool was_upsert = pending_[i].kind == Op::Kind::upsert;
    pending_[static_cast<std::size_t>(i)] = op;
    if (was_upsert || !s)
      return Result::replaced;
    return op.overwrite_rejected ? Result::revived : Result::replaced;
  }
  pending_.push_back(op);
  if (!s)
    return Result::added;
  return op.overwrite_rejected ? Result::revived : Result::replaced;
}

PendingEdgeEdits::Result
PendingEdgeEdits::stage_status(const EdgeKey &k, const std::string &status,
                               Result on_change) {
  const EdgeRecord *s = stored(k);
  const int i = pending_index(k);
  if (i >= 0)
    pending_.erase(pending_.begin() + i);
  if (s->status == status)
    return i >= 0 ? Result::unstaged : Result::unchanged;
  Op op;
  op.kind = Op::Kind::set_status;
  op.edge.key = k;
  op.edge.status = status;
  pending_.push_back(op);
  return on_change;
}

PendingEdgeEdits::Result PendingEdgeEdits::confirm(const EdgeKey &k) {
  if (const int i = pending_index(k);
      i >= 0 && pending_[static_cast<std::size_t>(i)].kind == Op::Kind::upsert)
    return Result::unchanged; // a staged measurement is confirmed already
  if (!stored(k))
    return Result::not_found;
  return stage_status(k, "confirmed", Result::confirmed);
}

PendingEdgeEdits::Result PendingEdgeEdits::reject(const EdgeKey &k) {
  const EdgeRecord *s = stored(k);
  const int i = pending_index(k);
  if (!s) {
    if (i < 0)
      return Result::not_found;
    pending_.erase(pending_.begin() + i); // a staged new edge: just drop it
    return Result::unstaged;
  }
  return stage_status(k, "rejected", Result::rejected);
}

PendingEdgeEdits::Result PendingEdgeEdits::unreject(const EdgeKey &k) {
  const EdgeRecord *s = stored(k);
  if (!s)
    return Result::not_found;
  const int i = pending_index(k);
  if (s->status != "rejected") {
    if (i >= 0 &&
        pending_[static_cast<std::size_t>(i)].kind == Op::Kind::set_status &&
        pending_[static_cast<std::size_t>(i)].edge.status == "rejected") {
      pending_.erase(pending_.begin() + i);
      return Result::unstaged;
    }
    return Result::unchanged;
  }
  return stage_status(k, "auto", Result::unrejected);
}

PendingEdgeEdits::Result PendingEdgeEdits::revert(const EdgeKey &k) {
  const int i = pending_index(k);
  if (i < 0)
    return Result::unchanged;
  pending_.erase(pending_.begin() + i);
  return Result::unstaged;
}

std::vector<PendingEdgeEdits::Op> PendingEdgeEdits::ops() const {
  std::vector<Op> out;
  out.reserve(pending_.size());
  for (const auto &o : pending_)
    if (o.kind == Op::Kind::set_status)
      out.push_back(o);
  for (const auto &o : pending_)
    if (o.kind == Op::Kind::upsert)
      out.push_back(o);
  return out;
}

std::vector<EdgeView> PendingEdgeEdits::effective() const {
  std::vector<EdgeView> out;
  out.reserve(base_.size() + pending_.size());
  for (const auto &e : base_) {
    EdgeView v;
    v.edge = e;
    v.stored_status = e.status;
    if (const int i = pending_index(e.key); i >= 0) {
      const Op &o = pending_[static_cast<std::size_t>(i)];
      if (o.kind == Op::Kind::set_status) {
        v.edge.status = o.edge.status;
        v.pending = EdgeView::Pending::status;
      } else {
        v.edge = o.edge;
        v.pending = EdgeView::Pending::replaced;
      }
    }
    out.push_back(std::move(v));
  }
  for (const auto &o : pending_)
    if (o.kind == Op::Kind::upsert && !stored(o.edge.key)) {
      EdgeView v;
      v.edge = o.edge;
      v.pending = EdgeView::Pending::added;
      out.push_back(std::move(v));
    }
  return out;
}

std::vector<EdgeView> PendingEdgeEdits::between(int a, int b) const {
  std::vector<EdgeView> out;
  for (auto &v : effective()) {
    const auto &k = v.edge.key;
    if ((k.from == a && k.to == b) || (k.from == b && k.to == a)) {
      v.reversed = k.from == b && a != b;
      out.push_back(std::move(v));
    }
  }
  return out;
}

std::map<int, int> PendingEdgeEdits::degrees() const {
  std::set<std::tuple<int, int, std::string>> seen;
  std::map<int, int> out;
  for (const auto &v : effective()) {
    if (v.edge.status == "rejected")
      continue;
    const auto &k = v.edge.key;
    if (!seen.insert({k.from, k.to, k.type}).second)
      continue;
    ++out[k.from];
    ++out[k.to];
  }
  return out;
}

int PendingEdgeEdits::degree(int frame) const {
  const auto all = degrees();
  const auto it = all.find(frame);
  return it == all.end() ? 0 : it->second;
}

void PendingEdgeEdits::commit_saved(const std::vector<Op> &saved) {
  for (const Op &o : saved) {
    EdgeRecord *s = nullptr;
    for (auto &e : base_)
      if (e.key == o.edge.key)
        s = &e;
    if (o.kind == Op::Kind::set_status) {
      if (s)
        s->status = o.edge.status;
    } else if (s) {
      *s = o.edge;
    } else {
      base_.push_back(o.edge);
    }
    if (const int i = pending_index(o.edge.key); i >= 0) {
      const Op &p = pending_[static_cast<std::size_t>(i)];
      if (p.kind == o.kind && p.overwrite_rejected == o.overwrite_rejected &&
          same_record(p.edge, o.edge))
        pending_.erase(pending_.begin() + i);
    }
  }
  // Whatever is still pending is re-staged against the new base.
  std::vector<Op> rest;
  rest.swap(pending_);
  for (Op &o : rest) {
    const EdgeRecord *s = stored(o.edge.key);
    if (o.kind == Op::Kind::set_status) {
      if (s && s->status != o.edge.status)
        pending_.push_back(o);
    } else {
      o.overwrite_rejected = s && s->status == "rejected";
      pending_.push_back(o);
    }
  }
}

std::string pending_summary_da(const std::vector<PendingEdgeEdits::Op> &ops) {
  int measured = 0, rejected = 0, confirmed = 0, unrejected = 0;
  for (const auto &o : ops) {
    if (o.kind == PendingEdgeEdits::Op::Kind::upsert)
      ++measured;
    else if (o.edge.status == "rejected")
      ++rejected;
    else if (o.edge.status == "confirmed")
      ++confirmed;
    else
      ++unrejected;
  }
  std::vector<std::string> parts;
  if (measured)
    parts.push_back(measured == 1 ? "1 ny eller målt kant"
                                  : std::to_string(measured) +
                                        " nye eller målte kanter");
  if (rejected)
    parts.push_back(std::to_string(rejected) + " afvist");
  if (confirmed)
    parts.push_back(std::to_string(confirmed) + " bekræftet");
  if (unrejected)
    parts.push_back(std::to_string(unrejected) + " genoplivet");
  std::string out;
  for (std::size_t i = 0; i < parts.size(); ++i)
    out += (i ? ", " : "") + parts[i];
  return out;
}
```

(add `#include <cmath> <set> <tuple>` to `database_logic.cpp` if missing).

- [ ] **Step 4: Run the core tests**

Run: `cmake --build build --target reusex_unit_tests && build/tests/reusex_unit_tests "[pending]"`
Expected: the core compiles and every `[pending]` case passes. The build of `rux_qt_lib` is still broken (EdgeEditor uses the old API) — fixed next.

- [ ] **Step 5: Write the store mapping and the failing save tests**

Create `apps/rux/qt/include/rux_qt/edge_store.hpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// The pose-graph store <-> the staged edit model. Header-only so the save
// path EdgeEditor runs on its thread and the save test
// (tests/unit/rux_qt/test_edge_save_locked.cpp) are one and the same code.

#include <rux_qt/database_logic.hpp>

#include <reusex/core/ProjectDB.hpp>

#include <stdexcept>
#include <string>
#include <vector>

namespace rux::qt {

inline EdgeRecord edge_record_from_row(const reusex::ProjectDB::PoseGraphEdge &e) {
  EdgeRecord r;
  r.key = {e.from_node_id, e.to_node_id, e.edge_type};
  r.residual = e.residual;
  r.weight = e.weight;
  r.transform = e.transform;
  r.information = e.information;
  r.source = e.source;
  r.status = e.status.empty() ? std::string("auto") : e.status;
  r.seed_disagreement_m = e.seed_disagreement_m;
  r.seed_disagreement_deg = e.seed_disagreement_deg;
  r.deviation_m = e.deviation_m;
  r.deviation_deg = e.deviation_deg;
  return r;
}

/// created_at is left empty: the store stamps it.
inline reusex::ProjectDB::PoseGraphEdge edge_row_from_record(const EdgeRecord &r) {
  reusex::ProjectDB::PoseGraphEdge e;
  e.from_node_id = r.key.from;
  e.to_node_id = r.key.to;
  e.edge_type = r.key.type;
  e.residual = r.residual;
  e.weight = r.weight;
  e.transform = r.transform;
  e.information = r.information;
  e.source = r.source;
  e.status = r.status;
  e.seed_disagreement_m = r.seed_disagreement_m;
  e.seed_disagreement_deg = r.seed_disagreement_deg;
  e.deviation_m = r.deviation_m;
  e.deviation_deg = r.deviation_deg;
  return e;
}

/// Apply @p ops in ONE transaction. Throws (and rolls back) when the store
/// refuses an upsert: the key was rejected in the file after it was loaded.
inline void apply_edge_ops(reusex::ProjectDB &db,
                           const std::vector<PendingEdgeEdits::Op> &ops) {
  reusex::ProjectDB::Transaction tx(db);
  for (const auto &op : ops) {
    const auto &k = op.edge.key;
    if (op.kind == PendingEdgeEdits::Op::Kind::set_status) {
      db.set_pose_graph_edge_status(k.from, k.to, k.type, op.edge.status);
    } else if (!db.upsert_pose_graph_edge(edge_row_from_record(op.edge),
                                          op.overwrite_rejected)) {
      throw std::runtime_error(
          "edge " + std::to_string(k.from) + " -> " + std::to_string(k.to) +
          " (" + k.type +
          ") was rejected in the project after it was loaded; reload and "
          "add it again");
    }
  }
  tx.commit();
}

} // namespace rux::qt
```

In `tests/unit/rux_qt/test_edge_save_locked.cpp`: delete the local `apply_ops`, `#include <rux_qt/edge_store.hpp>`, replace every `apply_ops(` with `apply_edge_ops(`, replace the read-only test's op literal with `{{PendingEdgeEdits::Op::Kind::upsert, {{1, 2, "loop_closure"}, 0.0, 1.0}, false}}`, and append:

```cpp
TEST_CASE("EdgeSave_StatusAndMeasurement_RoundTrip", "[rux_qt][database][save]") {
  reusex::test_support::TempPath tmp("edge_save_roundtrip");
  ProjectDB db(tmp.path);
  PendingEdgeEdits edits;
  edits.set_base({});
  EdgeRecord m{{1, 2, "loop_closure"}, 0.0, 400.0};
  m.transform = std::array<double, 16>{1, 0, 0, 0.25, 0, 1, 0, 0,
                                       0, 0, 1, 0, 0, 0, 0, 1};
  m.source = "icp";
  REQUIRE(edits.add(m) == PendingEdgeEdits::Result::added);
  apply_edge_ops(db, edits.ops());
  edits.commit_saved(edits.ops());

  std::vector<EdgeRecord> back;
  for (const auto &row : db.list_pose_graph_edges())
    back.push_back(edge_record_from_row(row));
  REQUIRE(back.size() == 1);
  CHECK(back[0].status == "confirmed");
  CHECK(back[0].source == "icp");
  REQUIRE(back[0].transform.has_value());
  CHECK((*back[0].transform)[3] == 0.25);

  edits.set_base(back);
  REQUIRE(edits.reject({1, 2, "loop_closure"}) ==
          PendingEdgeEdits::Result::rejected);
  apply_edge_ops(db, edits.ops());
  CHECK(db.list_pose_graph_edges()[0].status == "rejected");
}

TEST_CASE("EdgeSave_UpsertOverTombstone_RefusesWithoutRevive",
          "[rux_qt][database][save]") {
  reusex::test_support::TempPath tmp("edge_save_tombstone");
  ProjectDB db(tmp.path);
  ProjectDB::PoseGraphEdge row;
  row.from_node_id = 1;
  row.to_node_id = 2;
  row.edge_type = "loop_closure";
  row.status = "rejected";
  row.source = "orb";
  REQUIRE(db.upsert_pose_graph_edge(row));
  // A stale base that never saw the rejection: the save must fail, unwritten.
  PendingEdgeEdits stale;
  stale.set_base({});
  REQUIRE(stale.add({{1, 2, "loop_closure"}, 0.0, 9.0}) ==
          PendingEdgeEdits::Result::added);
  CHECK_THROWS_AS(apply_edge_ops(db, stale.ops()), std::runtime_error);
  CHECK(db.list_pose_graph_edges()[0].status == "rejected");
  // Loaded properly, the same add revives it.
  PendingEdgeEdits fresh;
  fresh.set_base({edge_record_from_row(db.list_pose_graph_edges()[0])});
  REQUIRE(fresh.add({{1, 2, "loop_closure"}, 0.0, 9.0}) ==
          PendingEdgeEdits::Result::revived);
  apply_edge_ops(db, fresh.ops());
  CHECK(db.list_pose_graph_edges()[0].status == "confirmed");
}
```

Run: `cmake --build build --target reusex_unit_tests && build/tests/reusex_unit_tests "[save]"`
Expected: PASS once the library P1 API is merged (these exercise it end to end). A failure here means an assumption in "Assumed library interfaces" is wrong — reconcile before continuing.

- [ ] **Step 6: Move EdgeEditor onto the new model**

`EdgeEditor.hpp`: replace `add`/`remove` with

```cpp
  PendingEdgeEdits::Result add(const EdgeRecord &edge);
  PendingEdgeEdits::Result confirm(const EdgeKey &key);
  PendingEdgeEdits::Result reject(const EdgeKey &key);
  PendingEdgeEdits::Result unreject(const EdgeKey &key);
  PendingEdgeEdits::Result revert(const EdgeKey &key);
```

`EdgeEditor.cpp` (add `#include <rux_qt/edge_store.hpp>`): each method is the same three lines, e.g.

```cpp
PendingEdgeEdits::Result EdgeEditor::reject(const EdgeKey &key) {
  const auto r = edits_.reject(key);
  error_.clear();
  emit changed();
  return r;
}
```

(write `add`, `confirm`, `reject`, `unreject`, `revert` this way, calling the matching `edits_` method). In `save()`'s thread replace the transaction block with:

```cpp
      try {
        reusex::ProjectDB db(path, /*readOnly=*/false);
        apply_edge_ops(db, snapshot);
        ok = true;
      } catch (const std::exception &ex) {
        what = QString::fromUtf8(ex.what());
      }
```

In `reload()` replace the loop body with `base.push_back(edge_record_from_row(e));`. Update the header comment "deletions first, then additions" → "status changes first, then measurements".

- [ ] **Step 7: Move the callers**

- `PoseGraphWorkspace::rebuild_edges()`: replace the two loops that fill `edges_` from `edits.base()` / `edits.ops()` with:

```cpp
  for (const auto &v : editor_.edits().effective()) {
    if (v.edge.status == "rejected")
      continue; // Task 10 adds the "Vis afviste" toggle
    DrawnEdge d;
    d.edge = {v.edge.key.from, v.edge.key.to};
    d.type = v.edge.key.type;
    d.pending_add = v.pending != EdgeView::Pending::none;
    d.residual = v.edge.residual;
    d.weight = v.edge.weight;
    edges_.push_back(d);
  }
```

  and drop batch 4 ("staged deletion") from the batch list and from the `k` selection. Remove `pending_delete` from `DrawnEdge` and from `click_edge`/`update_meta` (a staged rejection simply disappears from the canvas until Task 10).
- `FrameBrowser::refresh_strip_marks()`: replace the second loop with `for (const auto &v : editor_.edits().effective()) for (int f : {v.edge.key.from, v.edge.key.to}) if (!degrees.contains(f)) degrees.insert(f, 0);`.
- `DatabaseWorkspace::sync_banner()`: replace the `adds/removes` block and the `parts.join(", ")` argument with `QString::fromStdString(pending_summary_da(editor_->edits().ops()))`.
- `PairStrip::rebuild()`: give each row a status pill and the right action. Replace the pill/action part of the row loop with:

```cpp
    const QString status = QString::fromStdString(v.edge.status);
    h->addWidget(new Pill(status == "confirmed"  ? QString("Bekræftet")
                          : status == "rejected" ? QString("Afvist")
                                                 : QString("Auto"),
                          status == "confirmed"  ? "good"
                          : status == "rejected" ? "crit"
                                                 : "outline"));
    if (v.pending != EdgeView::Pending::none)
      h->addWidget(new Pill("Ikke gemt", "accent"));
    const EdgeKey key = v.edge.key;
    auto action = [&](const QString &text, auto fn) {
      auto *b = new QPushButton(NavItem::escape_mnemonic(text));
      b->setProperty("kind", "ghost");
      b->setObjectName("edgeAction");
      b->setCursor(Qt::PointingHandCursor);
      connect(b, &QPushButton::clicked, this, [this, key, fn] { fn(editor_, key); });
      b->setMinimumHeight(b->sizeHint().height());
      h->addWidget(b);
    };
    if (v.pending != EdgeView::Pending::none)
      action("Fortryd", [](EdgeEditor &e, const EdgeKey &k) { e.revert(k); });
    else if (status == "rejected")
      action("Fortryd afvisning", [](EdgeEditor &e, const EdgeKey &k) { e.unreject(k); });
    else
      action("Afvis", [](EdgeEditor &e, const EdgeKey &k) { e.reject(k); });
```

  and in `edge_selection()` replace the `pending_add`/`pending_delete` pill logic with `v.pending != EdgeView::Pending::none ? "Ikke gemt" : (status Danish)`. In `add_edge()`, the `duplicate` note becomes: `if (r == PendingEdgeEdits::Result::replaced) note_->setText("Kanten findes — den nye måling erstatter den ved gem.");`.
- `gallery/shell_pages.cpp` `make_db_pending`: `ed.remove(ed.edits().base().front().key)` → `ed.reject(ed.edits().base().front().key)`.

- [ ] **Step 8: Build everything and shoot**

Run: `cmake --build build --target reusex_unit_tests rux rux-qt-gallery && build/tests/reusex_unit_tests "[rux_qt]"`
Expected: PASS.
Run: `bash .claude/skills/design-studio/scripts/qt_shot.sh --page db-pending --page db-saved --page posegraf --theme both`
Expected: exit 0. Read the four `db-*` PNGs: rows show a status pill + "Ikke gemt" + "Afvis"/"Fortryd"; banner reads e.g. "2 ændringer … — 1 ny eller målt kant, 1 afvist". Fix clipping, re-shoot. Run `python .claude/skills/design-studio/scripts/token_lint.py apps/rux/qt --qt` → clean.

- [ ] **Step 9: Commit**

```bash
git add apps/rux/qt tests/unit/rux_qt
git commit -m "qt: status- and transform-aware staged edits; save via upsert/status

Delete becomes reject (tombstone); a staged edge carries its measurement;
EdgeEditor and the save test share apply_edge_ops (edge_store.hpp)."
```

---
### Task 5: Edge colouring and legend (Qt-free)

**Files:**
- Create: `apps/rux/qt/include/rux_qt/edge_colouring.hpp`, `apps/rux/qt/src/core/edge_colouring.cpp`, `tests/unit/rux_qt/test_edge_colouring.cpp`
- Modify: `apps/rux/qt/CMakeLists.txt` (rux_qt_core list)

**Interfaces:**
- Consumes: `EdgeView` (Task 4), `edge_type_da` (database_logic).
- Produces:
  - `enum class EdgeColourMode { type, status, residual, deviation, seed }`; `const std::vector<EdgeColourMode> &colour_modes()`; `std::string_view colour_mode_key(EdgeColourMode)`; `std::optional<EdgeColourMode> parse_colour_mode(std::string_view)`; `std::string colour_mode_da(EdgeColourMode)`
  - `kChi2Dof6P95 = 12.591587243743977`, `kResidualGood`, `kResidualBad`, `kRampBuckets = 16`, `kRampTokens` (3 token names)
  - `std::optional<double> edge_metric(const EdgeView &, EdgeColourMode)`; `double percentile(std::vector<double>, double p)`
  - `struct ColourScale { double lo, hi; bool fixed, valid; }`; `ColourScale colour_scale(EdgeColourMode, const std::vector<EdgeView> &drawn)`; `double ramp_position(double, const ColourScale &)`; `int ramp_bucket(double t)`
  - `bool edge_drawn(const EdgeView &, bool show_rejected)`; `std::vector<std::string> unknown_edge_types(const std::vector<EdgeView> &)`
  - `struct EdgeStyle { std::string token; int ramp = -1; bool dashed = false; bool wide = true; }`; `EdgeStyle edge_style(const EdgeView &, EdgeColourMode, const ColourScale &, const std::vector<std::string> &unknown_types, int label_slots)`
  - `struct LegendEntry { std::string token, label; int count = 0; bool dashed = false; }`; `struct EdgeLegend { std::vector<LegendEntry> entries; bool continuous = false; ColourScale scale; std::string unit; int missing = 0, hidden_rejected = 0, staged = 0; }`; `EdgeLegend build_legend(EdgeColourMode, const std::vector<EdgeView> &all, bool show_rejected, int label_slots)`

- [ ] **Step 1: Write the failing tests**

`tests/unit/rux_qt/test_edge_colouring.cpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

// Edge colouring of the Rediger canvas (spec §5.2): the fixed χ²-based
// residual scale, percentile scales for the metric modes, categorical
// tokens, and the legend built from what is drawn.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <rux_qt/edge_colouring.hpp>

#include <cmath>
#include <vector>

using namespace rux::qt;
using Catch::Approx;

namespace {
EdgeView view(int from, int to, const char *type = "loop_closure",
              const char *status = "auto") {
  EdgeView v;
  v.edge.key = {from, to, type};
  v.edge.status = status;
  v.stored_status = status;
  return v;
}
EdgeView with_residual(double r) {
  auto v = view(1, 2);
  v.edge.residual = r;
  return v;
}
EdgeView with_deviation(double m) {
  auto v = view(1, 2);
  v.edge.deviation_m = m;
  return v;
}
} // namespace

TEST_CASE("EdgeColour_ResidualScale_IsTheChiSquareThreshold",
          "[rux_qt][colour]") {
  CHECK(kResidualGood == Approx(6.29579).epsilon(1e-5));
  CHECK(kResidualBad == Approx(31.479).epsilon(1e-4));
  const auto s = colour_scale(EdgeColourMode::residual, {});
  CHECK(s.valid);
  CHECK(s.fixed);
  const auto style = [&](double r) {
    return edge_style(with_residual(r), EdgeColourMode::residual, s, {}, 8);
  };
  CHECK(style(1.0).ramp == 0);                // well inside the 95 % gate
  CHECK(style(kResidualGood).ramp == 0);
  CHECK(style(kResidualBad).ramp == kRampBuckets - 1);
  CHECK(style(1000.0).ramp == kRampBuckets - 1);
  const int mid = style((kResidualGood + kResidualBad) / 2).ramp;
  CHECK(mid >= 7);
  CHECK(mid <= 8);
}

TEST_CASE("EdgeColour_StagedMeasurement_HasNoResidualYet", "[rux_qt][colour]") {
  auto v = with_residual(50.0);
  v.pending = EdgeView::Pending::added;
  const auto st = edge_style(v, EdgeColourMode::residual,
                             colour_scale(EdgeColourMode::residual, {}), {}, 8);
  CHECK(st.ramp == -1);
  CHECK(st.token == "--color-text-faint");
  CHECK(st.dashed);
}

TEST_CASE("Percentile_EdgeCases", "[rux_qt][colour]") {
  std::vector<double> v;
  for (int i = 1; i <= 100; ++i)
    v.push_back(i);
  CHECK(percentile(v, 0.05) == Approx(5.95));
  CHECK(percentile(v, 0.95) == Approx(95.05));
  CHECK(percentile({7.0}, 0.05) == Approx(7.0));
  CHECK(std::isnan(percentile({}, 0.5)));
  CHECK(percentile({3.0, 1.0, 2.0}, 0.5) == Approx(2.0)); // unsorted input
}

TEST_CASE("EdgeColour_MetricModes_UseThe5To95Percentile", "[rux_qt][colour]") {
  std::vector<EdgeView> drawn;
  for (int i = 1; i <= 100; ++i)
    drawn.push_back(with_deviation(i * 0.01));
  const auto s = colour_scale(EdgeColourMode::deviation, drawn);
  REQUIRE(s.valid);
  CHECK_FALSE(s.fixed);
  CHECK(s.lo == Approx(0.0595));
  CHECK(s.hi == Approx(0.9505));
  CHECK(edge_style(with_deviation(0.01), EdgeColourMode::deviation, s, {}, 8).ramp == 0);
  CHECK(edge_style(with_deviation(2.0), EdgeColourMode::deviation, s, {}, 8).ramp ==
        kRampBuckets - 1);
}

TEST_CASE("EdgeColour_MetricModes_WithoutValues_AreNeutral", "[rux_qt][colour]") {
  // Legacy rows / never optimized: no metric anywhere.
  const std::vector<EdgeView> drawn = {view(1, 2), view(2, 3, "odometry")};
  const auto s = colour_scale(EdgeColourMode::seed, drawn);
  CHECK_FALSE(s.valid);
  const auto st = edge_style(drawn[0], EdgeColourMode::seed, s, {}, 8);
  CHECK(st.ramp == -1);
  CHECK(st.token == "--color-text-faint");
  const auto legend = build_legend(EdgeColourMode::seed, drawn, false, 8);
  CHECK(legend.continuous);
  CHECK(legend.missing == 2);
  CHECK_FALSE(legend.scale.valid);
  // One value: lo == hi, every edge at the start of the ramp.
  const std::vector<EdgeView> one = {with_deviation(0.3)};
  const auto s1 = colour_scale(EdgeColourMode::deviation, one);
  CHECK(s1.valid);
  CHECK(edge_style(one[0], EdgeColourMode::deviation, s1, {}, 8).ramp == 0);
}

TEST_CASE("EdgeColour_Type_UnknownTypesGetTheirOwnSwatch", "[rux_qt][colour]") {
  const std::vector<EdgeView> all = {view(1, 2, "odometry"), view(2, 3),
                                     view(3, 4, "panorama"), view(4, 5, "imu"),
                                     view(5, 6, "gravity")};
  const auto unknown = unknown_edge_types(all);
  REQUIRE(unknown == std::vector<std::string>{"gravity", "imu"});
  const ColourScale none;
  CHECK(edge_style(all[0], EdgeColourMode::type, none, unknown, 8).token ==
        "--color-text-faint");
  CHECK_FALSE(edge_style(all[0], EdgeColourMode::type, none, unknown, 8).wide);
  CHECK(edge_style(all[1], EdgeColourMode::type, none, unknown, 8).token ==
        "--color-accent");
  CHECK(edge_style(all[2], EdgeColourMode::type, none, unknown, 8).token ==
        "--color-star");
  CHECK(edge_style(all[3], EdgeColourMode::type, none, unknown, 8).token ==
        "--label-1");
  CHECK(edge_style(all[4], EdgeColourMode::type, none, unknown, 8).token ==
        "--label-0");
  const auto legend = build_legend(EdgeColourMode::type, all, false, 8);
  REQUIRE(legend.entries.size() == 5);
  CHECK(legend.entries[0].label == "Odometri");
  CHECK(legend.entries[3].label == "gravity");
  CHECK(legend.entries[3].count == 1);
}

TEST_CASE("EdgeColour_Status_AndRejectedHiddenByDefault", "[rux_qt][colour]") {
  std::vector<EdgeView> all = {view(1, 2), view(2, 3, "loop_closure", "confirmed"),
                               view(3, 4, "loop_closure", "rejected"),
                               view(4, 5, "loop_closure", "rejected")};
  CHECK_FALSE(edge_drawn(all[2], false));
  CHECK(edge_drawn(all[2], true));
  const auto hidden = build_legend(EdgeColourMode::status, all, false, 8);
  CHECK(hidden.hidden_rejected == 2);
  REQUIRE(hidden.entries.size() == 2); // only what is drawn
  CHECK(hidden.entries[0].label == "Auto");
  CHECK(hidden.entries[1].label == "Bekræftet");
  const auto shown = build_legend(EdgeColourMode::status, all, true, 8);
  REQUIRE(shown.entries.size() == 3);
  CHECK(shown.entries[2].label == "Afvist");
  CHECK(shown.entries[2].count == 2);
  CHECK(shown.entries[2].dashed);
  const auto st = edge_style(all[2], EdgeColourMode::status, {}, {}, 8);
  CHECK(st.token == "--color-status-failed");
  CHECK(st.dashed);
}

TEST_CASE("EdgeColour_Legend_CountsStagedEdges", "[rux_qt][colour]") {
  auto staged = view(7, 8);
  staged.pending = EdgeView::Pending::added;
  const auto legend =
      build_legend(EdgeColourMode::type, {view(1, 2), staged}, false, 8);
  CHECK(legend.staged == 1);
  CHECK(legend.entries.back().label == "Ikke gemt");
  CHECK(legend.entries.back().dashed);
}

TEST_CASE("ColourMode_KeysRoundTrip", "[rux_qt][colour]") {
  for (auto m : colour_modes())
    CHECK(parse_colour_mode(colour_mode_key(m)) == m);
  CHECK(colour_mode_da(EdgeColourMode::seed) == "Frøafvigelse (m)");
  CHECK_FALSE(parse_colour_mode("rainbow").has_value());
}
```

- [ ] **Step 2: Run to verify it fails**

Run: `cmake --build build --target reusex_unit_tests`
Expected: compile error (`rux_qt/edge_colouring.hpp` missing).

- [ ] **Step 3: Implement**

`edge_colouring.hpp`:

```cpp
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
ColourScale colour_scale(EdgeColourMode mode, const std::vector<EdgeView> &drawn);
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
  std::string unit;    ///< "m", or "" for the residual
  int missing = 0;     ///< drawn edges without the metric
  int hidden_rejected = 0;
  int staged = 0;
};
EdgeLegend build_legend(EdgeColourMode mode, const std::vector<EdgeView> &all,
                        bool show_rejected, int label_slots);

} // namespace rux::qt
```

`edge_colouring.cpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <rux_qt/edge_colouring.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>

namespace rux::qt {
namespace {

bool known_type(const std::string &t) {
  return t == "odometry" || t == "loop_closure" || t == "panorama";
}

std::string type_token(const std::string &type,
                       const std::vector<std::string> &unknown, int slots) {
  if (type == "odometry")
    return "--color-text-faint";
  if (type == "loop_closure")
    return "--color-accent";
  if (type == "panorama")
    return "--color-star";
  const auto it = std::find(unknown.begin(), unknown.end(), type);
  const auto i = it == unknown.end() ? 0 : it - unknown.begin();
  return "--label-" + std::to_string(i % std::max(slots, 1));
}

std::string status_token(const std::string &status) {
  if (status == "confirmed")
    return "--color-status-succeeded";
  if (status == "rejected")
    return "--color-status-failed";
  return "--color-on-chrome-muted";
}

std::string status_da(const std::string &status) {
  if (status == "confirmed")
    return "Bekræftet";
  if (status == "rejected")
    return "Afvist";
  return "Auto";
}

} // namespace

const std::vector<EdgeColourMode> &colour_modes() {
  static const std::vector<EdgeColourMode> m = {
      EdgeColourMode::type, EdgeColourMode::status, EdgeColourMode::residual,
      EdgeColourMode::deviation, EdgeColourMode::seed};
  return m;
}

std::string_view colour_mode_key(EdgeColourMode m) {
  switch (m) {
  case EdgeColourMode::type:
    return "type";
  case EdgeColourMode::status:
    return "status";
  case EdgeColourMode::residual:
    return "residual";
  case EdgeColourMode::deviation:
    return "deviation";
  case EdgeColourMode::seed:
    return "seed";
  }
  return "type";
}

std::optional<EdgeColourMode> parse_colour_mode(std::string_view key) {
  for (auto m : colour_modes())
    if (colour_mode_key(m) == key)
      return m;
  return std::nullopt;
}

std::string colour_mode_da(EdgeColourMode m) {
  switch (m) {
  case EdgeColourMode::type:
    return "Type";
  case EdgeColourMode::status:
    return "Status";
  case EdgeColourMode::residual:
    return "Residual";
  case EdgeColourMode::deviation:
    return "Afvigelse (m)";
  case EdgeColourMode::seed:
    return "Frøafvigelse (m)";
  }
  return "Type";
}

std::optional<double> edge_metric(const EdgeView &v, EdgeColourMode mode) {
  switch (mode) {
  case EdgeColourMode::residual:
    if (v.pending == EdgeView::Pending::added ||
        v.pending == EdgeView::Pending::replaced)
      return std::nullopt;
    return v.edge.residual;
  case EdgeColourMode::deviation:
    return v.edge.deviation_m;
  case EdgeColourMode::seed:
    return v.edge.seed_disagreement_m;
  default:
    return std::nullopt;
  }
}

double percentile(std::vector<double> v, double p) {
  if (v.empty())
    return std::numeric_limits<double>::quiet_NaN();
  std::sort(v.begin(), v.end());
  const double pos = std::clamp(p, 0.0, 1.0) * static_cast<double>(v.size() - 1);
  const auto i = static_cast<std::size_t>(std::floor(pos));
  const double frac = pos - static_cast<double>(i);
  return i + 1 < v.size() ? v[i] + frac * (v[i + 1] - v[i]) : v[i];
}

ColourScale colour_scale(EdgeColourMode mode, const std::vector<EdgeView> &drawn) {
  ColourScale s;
  if (mode == EdgeColourMode::residual) {
    s.lo = kResidualGood;
    s.hi = kResidualBad;
    s.fixed = true;
    s.valid = true;
    return s;
  }
  if (mode != EdgeColourMode::deviation && mode != EdgeColourMode::seed)
    return s;
  std::vector<double> values;
  for (const auto &v : drawn)
    if (auto m = edge_metric(v, mode); m && std::isfinite(*m))
      values.push_back(*m);
  if (values.empty())
    return s;
  s.lo = percentile(values, 0.05);
  s.hi = percentile(values, 0.95);
  s.valid = true;
  return s;
}

double ramp_position(double value, const ColourScale &s) {
  if (!s.valid || !std::isfinite(value))
    return 0.0;
  if (!(s.hi > s.lo))
    return value > s.hi ? 1.0 : 0.0;
  return std::clamp((value - s.lo) / (s.hi - s.lo), 0.0, 1.0);
}

int ramp_bucket(double t) {
  return std::clamp(static_cast<int>(t * kRampBuckets), 0, kRampBuckets - 1);
}

bool edge_drawn(const EdgeView &v, bool show_rejected) {
  return show_rejected || v.edge.status != "rejected";
}

std::vector<std::string> unknown_edge_types(const std::vector<EdgeView> &edges) {
  std::set<std::string> s;
  for (const auto &v : edges)
    if (!known_type(v.edge.key.type))
      s.insert(v.edge.key.type);
  return {s.begin(), s.end()};
}

EdgeStyle edge_style(const EdgeView &v, EdgeColourMode mode,
                     const ColourScale &scale,
                     const std::vector<std::string> &unknown, int slots) {
  EdgeStyle s;
  s.wide = v.edge.key.type != "odometry";
  s.dashed = v.pending != EdgeView::Pending::none || v.edge.status == "rejected";
  switch (mode) {
  case EdgeColourMode::type:
    s.token = type_token(v.edge.key.type, unknown, slots);
    break;
  case EdgeColourMode::status:
    s.token = status_token(v.edge.status);
    break;
  default:
    if (auto m = edge_metric(v, mode); m && scale.valid && std::isfinite(*m))
      s.ramp = ramp_bucket(ramp_position(*m, scale));
    else
      s.token = "--color-text-faint";
  }
  return s;
}

EdgeLegend build_legend(EdgeColourMode mode, const std::vector<EdgeView> &all,
                        bool show_rejected, int slots) {
  EdgeLegend l;
  std::vector<EdgeView> drawn;
  for (const auto &v : all) {
    if (!edge_drawn(v, show_rejected)) {
      ++l.hidden_rejected;
      continue;
    }
    drawn.push_back(v);
    l.staged += v.pending != EdgeView::Pending::none;
  }
  auto count_if = [&](auto pred) {
    return static_cast<int>(std::count_if(drawn.begin(), drawn.end(), pred));
  };
  if (mode == EdgeColourMode::type) {
    const auto unknown = unknown_edge_types(drawn);
    std::vector<std::string> order = {"odometry", "loop_closure", "panorama"};
    order.insert(order.end(), unknown.begin(), unknown.end());
    for (const auto &t : order)
      if (int n = count_if([&](const EdgeView &v) { return v.edge.key.type == t; }))
        l.entries.push_back({type_token(t, unknown, slots), edge_type_da(t), n, false});
  } else if (mode == EdgeColourMode::status) {
    for (const std::string st : {"auto", "confirmed", "rejected"})
      if (int n = count_if([&](const EdgeView &v) { return v.edge.status == st; }))
        l.entries.push_back({status_token(st), status_da(st), n, st == "rejected"});
  } else {
    l.continuous = true;
    l.scale = colour_scale(mode, drawn);
    l.unit = mode == EdgeColourMode::residual ? "" : "m";
    l.missing = count_if([&](const EdgeView &v) {
      const auto m = edge_metric(v, mode);
      return !m || !std::isfinite(*m);
    });
  }
  if (l.staged > 0)
    l.entries.push_back({"--color-on-chrome", "Ikke gemt", l.staged, true});
  return l;
}

} // namespace rux::qt
```

Note `edge_type_da` returns the raw type for unknown types (database_logic), which is what the legend shows. Add `src/core/edge_colouring.cpp` to the rux_qt_core list.

- [ ] **Step 4: Run the tests**

Run: `cmake --build build --target reusex_unit_tests && build/tests/reusex_unit_tests "[colour]"`
Expected: PASS (9 cases).

- [ ] **Step 5: Commit**

```bash
git add apps/rux/qt/include/rux_qt/edge_colouring.hpp apps/rux/qt/src/core/edge_colouring.cpp apps/rux/qt/CMakeLists.txt tests/unit/rux_qt/test_edge_colouring.cpp
git commit -m "qt core: edge colour modes (type/status/residual/deviation/seed) and legend"
```

---

### Task 6: Edge-list rows, sort and filter (Qt-free)

**Files:**
- Create: `apps/rux/qt/include/rux_qt/edge_table.hpp`, `apps/rux/qt/src/core/edge_table.cpp`, `tests/unit/rux_qt/test_edge_table.cpp`
- Modify: `apps/rux/qt/CMakeLists.txt`

**Interfaces:**
- Consumes: `EdgeView` (Task 4), `EdgeColourMode` (Task 5), `format_decimal_da`, `edge_type_da`.
- Produces: `enum class EdgeColumn { pair, type, source, status, residual, deviation_m, deviation_deg, seed_m, seed_deg }`, `kEdgeColumnCount = 9`, `std::string edge_column_title(EdgeColumn)` (upper-case), `bool edge_column_numeric(EdgeColumn)`, `std::optional<double> edge_column_value(const EdgeView &, EdgeColumn)`, `std::string edge_cell_text(const EdgeView &, EdgeColumn)`, `std::string edge_status_da(std::string_view)`, `std::string edge_source_da(std::string_view)`, `struct EdgeFilter { std::string type; std::string status; bool show_rejected = false; }`, `bool edge_matches(const EdgeView &, const EdgeFilter &)`, `void sort_edges(std::vector<EdgeView> &, EdgeColumn, bool descending)`, `EdgeColumn default_sort_column(EdgeColourMode)`, `std::vector<EdgeView> edge_table(const std::vector<EdgeView> &, const EdgeFilter &, EdgeColumn, bool descending)`.

- [ ] **Step 1: Write the failing tests**

`tests/unit/rux_qt/test_edge_table.cpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

// The Rediger edge list (spec §5.2): columns, Danish cells, sort with
// missing values last in both directions, type/status filter.

#include <catch2/catch_test_macros.hpp>

#include <rux_qt/edge_table.hpp>

#include <vector>

using namespace rux::qt;

namespace {
EdgeView row(int from, int to, const char *type, const char *status,
             std::optional<double> dev = std::nullopt, double residual = 0.0) {
  EdgeView v;
  v.edge.key = {from, to, type};
  v.edge.status = status;
  v.edge.residual = residual;
  v.edge.deviation_m = dev;
  v.stored_status = status;
  return v;
}
std::vector<int> froms(const std::vector<EdgeView> &rows) {
  std::vector<int> out;
  for (const auto &r : rows)
    out.push_back(r.edge.key.from);
  return out;
}
} // namespace

TEST_CASE("EdgeTable_SortsMissingLastInBothDirections", "[rux_qt][edgetable]") {
  std::vector<EdgeView> rows = {row(1, 2, "odometry", "auto", 0.3),
                                row(2, 3, "odometry", "auto"),
                                row(3, 4, "loop_closure", "auto", 1.2),
                                row(4, 5, "loop_closure", "auto", 0.3)};
  sort_edges(rows, EdgeColumn::deviation_m, /*descending=*/true);
  CHECK(froms(rows) == std::vector<int>{3, 1, 4, 2}); // tie: by key
  sort_edges(rows, EdgeColumn::deviation_m, /*descending=*/false);
  CHECK(froms(rows) == std::vector<int>{1, 4, 3, 2});
  sort_edges(rows, EdgeColumn::pair, false);
  CHECK(froms(rows) == std::vector<int>{1, 2, 3, 4});
}

TEST_CASE("EdgeTable_StagedMeasurement_HasNoResidual", "[rux_qt][edgetable]") {
  auto v = row(1, 2, "loop_closure", "confirmed", std::nullopt, 9.0);
  v.pending = EdgeView::Pending::added;
  CHECK_FALSE(edge_column_value(v, EdgeColumn::residual).has_value());
  CHECK(edge_cell_text(v, EdgeColumn::residual) == "—");
  CHECK(edge_cell_text(v, EdgeColumn::status) == "Bekræftet · ikke gemt");
}

TEST_CASE("EdgeTable_CellsAreDanish", "[rux_qt][edgetable]") {
  auto v = row(412, 980, "loop_closure", "auto", 0.4567, 12.346);
  v.edge.deviation_deg = 3.21;
  v.edge.source = "orb";
  CHECK(edge_cell_text(v, EdgeColumn::pair) == "412 → 980");
  CHECK(edge_cell_text(v, EdgeColumn::type) == "Løkkelukning");
  CHECK(edge_cell_text(v, EdgeColumn::source) == "ORB");
  CHECK(edge_cell_text(v, EdgeColumn::status) == "Auto");
  CHECK(edge_cell_text(v, EdgeColumn::residual) == "12,35");
  CHECK(edge_cell_text(v, EdgeColumn::deviation_m) == "0,46");
  CHECK(edge_cell_text(v, EdgeColumn::deviation_deg) == "3,2");
  CHECK(edge_cell_text(v, EdgeColumn::seed_m) == "—");
  CHECK(edge_source_da("") == "—");
  CHECK(edge_source_da("odometry_init") == "Odometri (init)");
  CHECK(edge_column_title(EdgeColumn::seed_deg) == "FRØ °");
}

TEST_CASE("EdgeTable_Filter_TypeStatusAndRejected", "[rux_qt][edgetable]") {
  const std::vector<EdgeView> all = {row(1, 2, "odometry", "auto"),
                                     row(2, 3, "loop_closure", "confirmed"),
                                     row(3, 4, "loop_closure", "rejected"),
                                     row(4, 5, "panorama", "auto")};
  EdgeFilter f;
  CHECK(edge_table(all, f, EdgeColumn::pair, false).size() == 3); // rejected hidden
  f.show_rejected = true;
  CHECK(edge_table(all, f, EdgeColumn::pair, false).size() == 4);
  f = {};
  f.type = "loop_closure";
  CHECK(froms(edge_table(all, f, EdgeColumn::pair, false)) == std::vector<int>{2});
  f.status = "rejected"; // asking for rejected shows them regardless
  CHECK(froms(edge_table(all, f, EdgeColumn::pair, false)) == std::vector<int>{3});
}

TEST_CASE("EdgeTable_DefaultSortFollowsTheColourMode", "[rux_qt][edgetable]") {
  CHECK(default_sort_column(EdgeColourMode::residual) == EdgeColumn::residual);
  CHECK(default_sort_column(EdgeColourMode::deviation) == EdgeColumn::deviation_m);
  CHECK(default_sort_column(EdgeColourMode::seed) == EdgeColumn::seed_m);
  CHECK(default_sort_column(EdgeColourMode::type) == EdgeColumn::residual);
}
```

- [ ] **Step 2: Run to verify it fails**

Run: `cmake --build build --target reusex_unit_tests`
Expected: compile error (`rux_qt/edge_table.hpp` missing).

- [ ] **Step 3: Implement**

`edge_table.hpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// The Rediger edge list's data, Qt-free (spec §5.2): columns, the Danish
// cell text, sorting with missing values always last (so "most suspicious
// first" never starts with a page of dashes), and the type/status filter.

#include <rux_qt/database_logic.hpp>
#include <rux_qt/edge_colouring.hpp>

#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace rux::qt {

enum class EdgeColumn {
  pair = 0, type, source, status, residual, deviation_m, deviation_deg,
  seed_m, seed_deg
};
inline constexpr int kEdgeColumnCount = 9;

/// Upper-case header text (QSS header rules drop QFont::AllUppercase).
std::string edge_column_title(EdgeColumn c);
bool edge_column_numeric(EdgeColumn c);
/// The sort value of a numeric column; nullopt when missing.
std::optional<double> edge_column_value(const EdgeView &v, EdgeColumn c);
std::string edge_cell_text(const EdgeView &v, EdgeColumn c);
std::string edge_status_da(std::string_view status);
std::string edge_source_da(std::string_view source);

struct EdgeFilter {
  std::string type;   ///< empty = every type
  std::string status; ///< empty = every non-rejected (+ rejected if shown)
  bool show_rejected = false;
};
bool edge_matches(const EdgeView &v, const EdgeFilter &f);
/// Stable: ties by (from, to, type); missing numeric values last.
void sort_edges(std::vector<EdgeView> &rows, EdgeColumn column, bool descending);
/// The column a colour mode's suspicious edges sort to the top by.
EdgeColumn default_sort_column(EdgeColourMode mode);
std::vector<EdgeView> edge_table(const std::vector<EdgeView> &all,
                                 const EdgeFilter &f, EdgeColumn column,
                                 bool descending);

} // namespace rux::qt
```

`edge_table.cpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <rux_qt/edge_table.hpp>

#include <algorithm>
#include <cmath>
#include <tuple>

namespace rux::qt {
namespace {
auto key_tuple(const EdgeView &v) {
  return std::tie(v.edge.key.from, v.edge.key.to, v.edge.key.type);
}
} // namespace

std::string edge_column_title(EdgeColumn c) {
  switch (c) {
  case EdgeColumn::pair: return "PAR";
  case EdgeColumn::type: return "TYPE";
  case EdgeColumn::source: return "KILDE";
  case EdgeColumn::status: return "STATUS";
  case EdgeColumn::residual: return "RESIDUAL";
  case EdgeColumn::deviation_m: return "AFV. M";
  case EdgeColumn::deviation_deg: return "AFV. °";
  case EdgeColumn::seed_m: return "FRØ M";
  case EdgeColumn::seed_deg: return "FRØ °";
  }
  return {};
}

bool edge_column_numeric(EdgeColumn c) {
  return c == EdgeColumn::residual || c == EdgeColumn::deviation_m ||
         c == EdgeColumn::deviation_deg || c == EdgeColumn::seed_m ||
         c == EdgeColumn::seed_deg;
}

std::optional<double> edge_column_value(const EdgeView &v, EdgeColumn c) {
  switch (c) {
  case EdgeColumn::residual:
    return edge_metric(v, EdgeColourMode::residual);
  case EdgeColumn::deviation_m: return v.edge.deviation_m;
  case EdgeColumn::deviation_deg: return v.edge.deviation_deg;
  case EdgeColumn::seed_m: return v.edge.seed_disagreement_m;
  case EdgeColumn::seed_deg: return v.edge.seed_disagreement_deg;
  default: return std::nullopt;
  }
}

std::string edge_status_da(std::string_view s) {
  if (s == "confirmed") return "Bekræftet";
  if (s == "rejected") return "Afvist";
  return "Auto";
}

std::string edge_source_da(std::string_view s) {
  if (s == "rtabmap") return "RTABMap";
  if (s == "odometry_init") return "Odometri (init)";
  if (s == "orb") return "ORB";
  if (s == "panorama") return "Panorama";
  if (s == "file") return "Fil";
  if (s == "icp") return "ICP";
  if (s == "manual") return "Manuel";
  if (s == "legacy") return "Ældre";
  if (s.empty()) return "—";
  return std::string(s);
}

std::string edge_cell_text(const EdgeView &v, EdgeColumn c) {
  switch (c) {
  case EdgeColumn::pair:
    return std::to_string(v.edge.key.from) + " → " + std::to_string(v.edge.key.to);
  case EdgeColumn::type: return edge_type_da(v.edge.key.type);
  case EdgeColumn::source: return edge_source_da(v.edge.source);
  case EdgeColumn::status:
    return edge_status_da(v.edge.status) +
           (v.pending != EdgeView::Pending::none ? " · ikke gemt" : "");
  default: break;
  }
  const auto value = edge_column_value(v, c);
  if (!value || !std::isfinite(*value))
    return "—";
  const int decimals =
      (c == EdgeColumn::deviation_deg || c == EdgeColumn::seed_deg) ? 1 : 2;
  return format_decimal_da(*value, decimals);
}

bool edge_matches(const EdgeView &v, const EdgeFilter &f) {
  if (!f.type.empty() && v.edge.key.type != f.type)
    return false;
  if (!f.status.empty())
    return v.edge.status == f.status;
  return f.show_rejected || v.edge.status != "rejected";
}

void sort_edges(std::vector<EdgeView> &rows, EdgeColumn column, bool descending) {
  std::stable_sort(rows.begin(), rows.end(), [&](const EdgeView &a, const EdgeView &b) {
    if (edge_column_numeric(column)) {
      const auto va = edge_column_value(a, column), vb = edge_column_value(b, column);
      const bool ha = va && std::isfinite(*va), hb = vb && std::isfinite(*vb);
      if (ha != hb)
        return ha; // missing last, whatever the direction
      if (ha && *va != *vb)
        return descending ? *va > *vb : *va < *vb;
      return key_tuple(a) < key_tuple(b);
    }
    if (column != EdgeColumn::pair) {
      const auto ta = edge_cell_text(a, column), tb = edge_cell_text(b, column);
      if (ta != tb)
        return descending ? ta > tb : ta < tb;
      return key_tuple(a) < key_tuple(b);
    }
    return descending ? key_tuple(b) < key_tuple(a) : key_tuple(a) < key_tuple(b);
  });
}

EdgeColumn default_sort_column(EdgeColourMode mode) {
  switch (mode) {
  case EdgeColourMode::deviation: return EdgeColumn::deviation_m;
  case EdgeColourMode::seed: return EdgeColumn::seed_m;
  default: return EdgeColumn::residual;
  }
}

std::vector<EdgeView> edge_table(const std::vector<EdgeView> &all,
                                 const EdgeFilter &f, EdgeColumn column,
                                 bool descending) {
  std::vector<EdgeView> out;
  for (const auto &v : all)
    if (edge_matches(v, f))
      out.push_back(v);
  sort_edges(out, column, descending);
  return out;
}

} // namespace rux::qt
```

Add `src/core/edge_table.cpp` to the rux_qt_core list.

- [ ] **Step 4: Run the tests**

Run: `cmake --build build --target reusex_unit_tests && build/tests/reusex_unit_tests "[edgetable]"`
Expected: PASS (5 cases).

- [ ] **Step 5: Commit**

```bash
git add apps/rux/qt/include/rux_qt/edge_table.hpp apps/rux/qt/src/core/edge_table.cpp apps/rux/qt/CMakeLists.txt tests/unit/rux_qt/test_edge_table.cpp
git commit -m "qt core: edge-list columns, Danish cells, sort (missing last), filter"
```

---
### Task 7: Navigation (Oversigt, Rediger) and the `EditWorkspace` shell

Posegraf leaves the rail; Rediger takes its index with a splitter layout whose panes can be hidden and restored (pane header button, toolbar "Paneler" menu, Ctrl+K). The graph pane hosts today's `PoseGraphWorkspace` unchanged until Task 9. `EdgeEditor` moves to `AppShell` so Database (read-only use) and Rediger share one.

**Files:**
- Modify: `apps/rux/qt/include/rux_qt/workspaces.hpp` (enum), `apps/rux/qt/src/workspaces.cpp:35-61` (infos)
- Modify: `apps/rux/qt/include/rux_qt/palette_table.hpp`, `apps/rux/qt/src/core/palette_table.cpp:32-41,88-97`
- Modify: `apps/rux/qt/src/app.cpp:182-192` (smoke pages)
- Modify: `apps/rux/qt/include/rux_qt/AppShell.hpp`, `apps/rux/qt/src/AppShell.cpp` (construction, `show_page`, `page_selection`, `commands`, follow)
- Modify: `apps/rux/qt/include/rux_qt/DatabaseWorkspace.hpp`, `apps/rux/qt/src/DatabaseWorkspace.cpp:228-233` (ctor takes `EdgeEditor &`)
- Modify: `apps/rux/qt/include/rux_qt/widgets.hpp`, `apps/rux/qt/src/widgets.cpp` (`PaneFrame`)
- Create: `apps/rux/qt/include/rux_qt/EditWorkspace.hpp`, `apps/rux/qt/src/EditWorkspace.cpp`
- Modify: `apps/rux/qt/styles/app.qss` (pane + splitter rules)
- Modify: `apps/rux/qt/gallery/shell_pages.cpp` (`posegraf*` → `rediger*`, `persist_layout = false`)
- Test: `tests/unit/rux_qt/test_shell_logic.cpp`

**Interfaces:**
- Produces:
  - `enum class Workspace { start = 0, database, viewer3d, edit, pipeline, log }`
  - `enum class EditPane { graph = 0, edges, images, overlay }`, `kEditPaneCount = 4`, `std::string_view edit_pane_key(EditPane)`, `std::string edit_pane_name_da(EditPane)`, `std::optional<EditPane> parse_edit_pane(std::string_view)`, `int smoke_page_index(std::string_view)` (palette_table.hpp)
  - `ShellOptions::persist_layout` (bool, default true); `AppShell::edit() const -> EditWorkspace *`; `AppShell::edges() const -> EdgeEditor &`
  - `class PaneFrame : public QFrame { PaneFrame(const QString &title, QWidget *parent = nullptr); void set_content(QWidget *); QWidget *content() const; void add_header_widget(QWidget *); signals: void hide_requested(); }`
  - `class EditWorkspace : public QWidget { EditWorkspace(ProjectSession &, EdgeEditor &, bool persist_layout, QWidget *parent = nullptr); void activate(); PoseGraphWorkspace *graph() const; PaneFrame *pane(EditPane) const; void set_pane_visible(EditPane, bool); bool pane_visible(EditPane) const; void toggle_pane(EditPane); const Selection &selection() const; signals: void selection_changed(const rux::qt::Selection &); }`
  - `DatabaseWorkspace(ProjectSession &, EdgeEditor &, QWidget *parent = nullptr)`

- [ ] **Step 1: Write the failing palette/navigation tests**

In `tests/unit/rux_qt/test_shell_logic.cpp`, change `PaletteTable_ListsPagesActionsAndRecents`:

```cpp
TEST_CASE("PaletteTable_ListsPagesActionsAndRecents", "[rux_qt][palette]") {
  const auto t = build_palette(open_state());
  REQUIRE(t.size() == 6 + 8 + 4 + 3);
  CHECK(t[0].title == "Start");
  CHECK(t[0].badge == "Her");
  CHECK(t[1].title == "Oversigt");
  CHECK(t[1].shortcut == "Alt+2");
  CHECK(t[3].title == "Rediger");
  // Without a project there is nothing to close, reload, copy or show.
  PaletteState none;
  for (const auto &e : build_palette(none))
    CHECK((e.id != "close" && e.id != "reload" && e.id != "copy-path" &&
           e.id.rfind("pane:", 0) != 0));
  // The Rediger panes, after the project actions.
  CHECK(t[14].id == "pane:graph");
  CHECK(t[17].id == "pane:overlay");
  CHECK(t[17].title == "Vis eller skjul 3D-overlay");
  // Recents: generic project.rux shows its folder; missing ones are disabled.
  CHECK(t[18].title == "NewOffice");
  CHECK(t[18].badge == "Åben");
  CHECK(t[19].title == "Kontorhus Valby");
  CHECK_FALSE(t[19].enabled);
  CHECK(t[19].badge == "Mangler");
  CHECK(action_shortcut("open") == "Ctrl+O");
}
```

Rename `PaletteTable_Db_FindsDatabaseFirst` to `PaletteTable_Db_FindsOversigtFirst` with `CHECK(r.front() == "Oversigt");`, and add:

```cpp
TEST_CASE("PaletteTable_Posegraf_FindsRediger", "[rux_qt][palette]") {
  const auto r = ranked_titles("posegraf", build_palette(open_state()));
  REQUIRE_FALSE(r.empty());
  CHECK(r.front() == "Rediger");
}

TEST_CASE("SmokePage_NamesMapToRailIndices", "[rux_qt][shell]") {
  CHECK(smoke_page_index("start") == 0);
  CHECK(smoke_page_index("oversigt") == 1);
  CHECK(smoke_page_index("database") == 1); // the old name keeps working
  CHECK(smoke_page_index("3d") == 2);
  CHECK(smoke_page_index("rediger") == 3);
  CHECK(smoke_page_index("pipeline") == 4);
  CHECK(smoke_page_index("log") == 5);
  CHECK(smoke_page_index("posegraf") == -1);
  CHECK(smoke_page_index("") == -1);
}

TEST_CASE("EditPane_KeysRoundTrip", "[rux_qt][shell]") {
  for (int i = 0; i < kEditPaneCount; ++i) {
    const auto p = static_cast<EditPane>(i);
    CHECK(parse_edit_pane(edit_pane_key(p)) == p);
  }
  CHECK(edit_pane_name_da(EditPane::edges) == "Kantliste");
  CHECK_FALSE(parse_edit_pane("nope").has_value());
}
```

- [ ] **Step 2: Run to verify it fails**

Run: `cmake --build build --target reusex_unit_tests`
Expected: compile errors (`smoke_page_index`, `EditPane` undeclared).

- [ ] **Step 3: Implement the Qt-free part**

`palette_table.hpp` — add after `page_entries()` (and `#include <optional> <string_view>`):

```cpp
/// The Rediger workspace's panes (spec §5.2), in their QSettings order.
enum class EditPane { graph = 0, edges, images, overlay };
inline constexpr int kEditPaneCount = 4;
std::string_view edit_pane_key(EditPane pane); ///< "graph" | "edges" | …
std::string edit_pane_name_da(EditPane pane);  ///< "Graf", "Kantliste", …
std::optional<EditPane> parse_edit_pane(std::string_view key);

/// RUX_QT_PAGE value -> rail index, or -1. "database" is an alias of
/// "oversigt" (scripts written before the rename keep working).
int smoke_page_index(std::string_view name);
```

and update the comment on `page_entries()` to "(Start, Oversigt, 3D, Rediger, Pipeline, Log)".

`palette_table.cpp`: change the page table to

```cpp
      {"Start", "forside velkomst start"},
      {"Oversigt", "database db tabeller billeder frames databaseviewer data "
                   "oversigt panoramaer"},
      {"3D", "punktsky viewer vtk mesh scene"},
      {"Rediger", "posegraf pose graph kanter loop closure graf rediger edit "
                  "icp løkkelukning"},
      {"Pipeline", "kør trin stage job parametre"},
      {"Log", "kørselslog pipeline-log historik"},
```

inside `if (s.project_open) { … }` after the `copy-cli` action add

```cpp
    // Titles in the definite form ("grafen"), one per EditPane.
    static constexpr const char *kPaneTitles[kEditPaneCount] = {
        "grafen", "kantlisten", "billederne", "3D-overlay"};
    for (int i = 0; i < kEditPaneCount; ++i) {
      const auto p = static_cast<EditPane>(i);
      PaletteEntry e;
      e.id = "pane:" + std::string(edit_pane_key(p));
      e.group = "Handlinger";
      e.title = std::string("Vis eller skjul ") + kPaneTitles[i];
      e.keywords = "rediger panel vis skjul " + edit_pane_name_da(p);
      out.push_back(std::move(e));
    }
```

and implement:

```cpp
std::string_view edit_pane_key(EditPane p) {
  switch (p) {
  case EditPane::graph: return "graph";
  case EditPane::edges: return "edges";
  case EditPane::images: return "images";
  case EditPane::overlay: return "overlay";
  }
  return "graph";
}

std::string edit_pane_name_da(EditPane p) {
  switch (p) {
  case EditPane::graph: return "Graf";
  case EditPane::edges: return "Kantliste";
  case EditPane::images: return "Billeder";
  case EditPane::overlay: return "3D-overlay";
  }
  return {};
}

std::optional<EditPane> parse_edit_pane(std::string_view key) {
  for (int i = 0; i < kEditPaneCount; ++i)
    if (edit_pane_key(static_cast<EditPane>(i)) == key)
      return static_cast<EditPane>(i);
  return std::nullopt;
}

int smoke_page_index(std::string_view name) {
  static constexpr std::string_view kNames[] = {"start", "oversigt", "3d",
                                                "rediger", "pipeline", "log"};
  if (name == "database")
    return 1;
  for (int i = 0; i < 6; ++i)
    if (kNames[i] == name)
      return i;
  return -1;
}
```



- [ ] **Step 4: Run the tests**

Run: `cmake --build build --target reusex_unit_tests && build/tests/reusex_unit_tests "[palette],[shell]"`
Expected: PASS. If `PaletteTable_Db_FindsOversigtFirst` ranks another row first, move `db` to the front of Oversigt's keywords ("db database …") — ranking prefers an early contiguous run.

- [ ] **Step 5: `PaneFrame`**

`widgets.hpp`, after `Panel`:

```cpp
/// A pane of a splitter layout (Rediger): a caps title, optional header
/// widgets and a "Skjul" button over the content. Hiding is the owner's
/// job (hide_requested); the header never scrolls with the content.
class PaneFrame : public QFrame {
  Q_OBJECT
    public:
  explicit PaneFrame(const QString &title, QWidget *parent = nullptr);
  void set_content(QWidget *content);
  QWidget *content() const { return content_; }
  void add_header_widget(QWidget *w);

    signals:
  void hide_requested();

    private:
  QHBoxLayout *header_ = nullptr;
  QVBoxLayout *body_ = nullptr;
  QWidget *content_ = nullptr;
};
```

`widgets.cpp`:

```cpp
PaneFrame::PaneFrame(const QString &title, QWidget *parent) : QFrame(parent) {
  setObjectName("paneFrame");
  const Theme &t = theme();
  body_ = new QVBoxLayout(this);
  body_->setContentsMargins(0, 0, 0, 0);
  body_->setSpacing(0);
  auto *head = new QFrame;
  head->setObjectName("paneHeader");
  header_ = new QHBoxLayout(head);
  header_->setContentsMargins(t.px("--space-3"), t.px("--space-1"),
                              t.px("--space-2"), t.px("--space-1"));
  header_->setSpacing(t.px("--space-2"));
  header_->addWidget(new CapsLabel(title, "panelTitle"));
  header_->addStretch(1);
  auto *hide = new QPushButton(NavItem::escape_mnemonic("Skjul"));
  hide->setObjectName("paneHide");
  hide->setProperty("kind", "ghost");
  hide->setCursor(Qt::PointingHandCursor);
  hide->setToolTip("Skjul panelet — vis det igen under Paneler eller med Ctrl+K");
  hide->setMinimumHeight(hide->sizeHint().height());
  header_->addWidget(hide);
  body_->addWidget(head);
  connect(hide, &QPushButton::clicked, this, &PaneFrame::hide_requested);
}

void PaneFrame::set_content(QWidget *content) {
  if (content_) {
    content_->hide();
    body_->removeWidget(content_);
    content_->deleteLater();
  }
  content_ = content;
  body_->addWidget(content_, 1);
}

void PaneFrame::add_header_widget(QWidget *w) {
  header_->insertWidget(header_->count() - 1, w); // before "Skjul"
}
```

`styles/app.qss` (append, next to the `#dbToolbar` rules):

```css
QFrame#paneFrame {
  background: var(--color-surface);
}
QFrame#paneHeader {
  background: var(--color-surface-raised);
  border-bottom: 1px solid var(--color-border);
}
QSplitter::handle {
  background: var(--color-border);
}
QSplitter::handle:hover {
  background: var(--color-border-strong);
}
```

- [ ] **Step 6: `EditWorkspace` skeleton**

`include/rux_qt/EditWorkspace.hpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// The Rediger workspace (spec §5.2): the pose graph and its edge list on the
// left, the selected pair (images over a 3D overlay, then its edges) on the
// right, in nested QSplitters. Each pane can be hidden from its header and
// shown again from the toolbar's "Paneler" menu or the command palette;
// splitter sizes and visibility persist in QSettings under rediger/… (not
// in a gallery shot).

#include <rux_qt/palette_table.hpp>
#include <rux_qt/selection.hpp>

#include <QWidget>

#include <array>

class QAction;
class QLabel;
class QSplitter;

namespace rux::qt {

class EdgeEditor;
class PaneFrame;
class PoseGraphWorkspace;
class ProjectSession;

class EditWorkspace : public QWidget {
  Q_OBJECT
    public:
  EditWorkspace(ProjectSession &session, EdgeEditor &editor,
                bool persist_layout, QWidget *parent = nullptr);
  ~EditWorkspace() override;

  /// The page is on screen: load what is not loaded yet.
  void activate();
  PoseGraphWorkspace *graph() const { return graph_; }
  EdgeEditor &editor() const { return editor_; }
  PaneFrame *pane(EditPane p) const { return panes_[static_cast<int>(p)]; }
  /// The user's choice for @p p (the overlay also needs a complete pair).
  bool pane_visible(EditPane p) const { return wanted_[static_cast<int>(p)]; }
  void set_pane_visible(EditPane p, bool on);
  void toggle_pane(EditPane p) { set_pane_visible(p, !pane_visible(p)); }
  const Selection &selection() const { return selection_; }

    signals:
  void selection_changed(const rux::qt::Selection &selection);

    protected:
  /// Show what is wanted, and the overlay only with a complete pair.
  void sync_panes();
  bool pair_complete_ = false;

    private:
  QWidget *make_toolbar();
  void restore_layout();
  void save_layout() const;

  ProjectSession &session_;
  EdgeEditor &editor_;
  bool persist_ = true;
  QSplitter *main_ = nullptr;
  QSplitter *left_ = nullptr;
  QSplitter *right_ = nullptr;
  QWidget *right_column_ = nullptr;
  std::array<PaneFrame *, kEditPaneCount> panes_{};
  std::array<bool, kEditPaneCount> wanted_{true, true, true, true};
  std::array<QAction *, kEditPaneCount> pane_actions_{};
  PoseGraphWorkspace *graph_ = nullptr;
  QLabel *meta_ = nullptr;
  Selection selection_;
};

} // namespace rux::qt
```

`src/EditWorkspace.cpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <rux_qt/EdgeEditor.hpp>
#include <rux_qt/EditWorkspace.hpp>
#include <rux_qt/PoseGraphWorkspace.hpp>
#include <rux_qt/ProjectSession.hpp>
#include <rux_qt/Theme.hpp>
#include <rux_qt/widgets.hpp>

#include <QAction>
#include <QHBoxLayout>
#include <QLabel>
#include <QMenu>
#include <QPushButton>
#include <QSettings>
#include <QSplitter>
#include <QVBoxLayout>

namespace rux::qt {
namespace {

QLabel *empty_state(const QString &text) {
  auto *l = new QLabel(text);
  l->setObjectName("emptyText");
  l->setAlignment(Qt::AlignCenter);
  l->setWordWrap(true);
  return l;
}

QString key_of(EditPane p) {
  return QString::fromLatin1(edit_pane_key(p).data(),
                             static_cast<int>(edit_pane_key(p).size()));
}

} // namespace

EditWorkspace::EditWorkspace(ProjectSession &session, EdgeEditor &editor,
                             bool persist_layout, QWidget *parent)
    : QWidget(parent), session_(session), editor_(editor),
      persist_(persist_layout) {
  setObjectName("editWorkspace");
  const Theme &t = theme();
  auto *v = new QVBoxLayout(this);
  v->setContentsMargins(0, 0, 0, 0);
  v->setSpacing(0);
  v->addWidget(make_toolbar());

  graph_ = new PoseGraphWorkspace(session_, editor_);
  panes_[0] = new PaneFrame("Graf");
  panes_[0]->set_content(graph_);
  panes_[1] = new PaneFrame("Kanter");
  panes_[1]->set_content(empty_state("Kantlisten vises her."));
  panes_[2] = new PaneFrame("Billeder");
  panes_[2]->set_content(empty_state("Klik på en node for A, og en til for B."));
  panes_[3] = new PaneFrame("3D-overlay");
  panes_[3]->set_content(empty_state("Vælg A og B for at se deres skyer."));

  left_ = new QSplitter(Qt::Vertical);
  left_->addWidget(panes_[0]);
  left_->addWidget(panes_[1]);
  left_->setStretchFactor(0, 3);
  left_->setStretchFactor(1, 1);
  right_ = new QSplitter(Qt::Vertical);
  right_->addWidget(panes_[2]);
  right_->addWidget(panes_[3]);
  right_column_ = new QWidget;
  auto *rc = new QVBoxLayout(right_column_);
  rc->setContentsMargins(0, 0, 0, 0);
  rc->setSpacing(0);
  rc->addWidget(right_, 1);
  main_ = new QSplitter(Qt::Horizontal);
  main_->addWidget(left_);
  main_->addWidget(right_column_);
  main_->setStretchFactor(0, 3);
  main_->setStretchFactor(1, 2);
  for (QSplitter *s : {main_, left_, right_}) {
    s->setChildrenCollapsible(false);
    s->setHandleWidth(t.px("--space-1"));
  }
  v->addWidget(main_, 1);

  for (int i = 0; i < kEditPaneCount; ++i)
    connect(panes_[i], &PaneFrame::hide_requested, this,
            [this, i] { set_pane_visible(static_cast<EditPane>(i), false); });
  for (QSplitter *s : {main_, left_, right_})
    connect(s, &QSplitter::splitterMoved, this, [this] { save_layout(); });
  connect(graph_, &PoseGraphWorkspace::selection_changed, this,
          [this](const Selection &s) {
            selection_ = s;
            emit selection_changed(s);
          });
  restore_layout();
  sync_panes();
}

EditWorkspace::~EditWorkspace() = default;

QWidget *EditWorkspace::make_toolbar() {
  const Theme &t = theme();
  auto *bar = new QFrame;
  bar->setObjectName("dbToolbar");
  auto *bl = new QHBoxLayout(bar);
  bl->setContentsMargins(t.px("--space-4"), t.px("--space-2"),
                         t.px("--space-4"), t.px("--space-2"));
  bl->setSpacing(t.px("--space-3"));
  bl->addWidget(new CapsLabel("Rediger", "panelTitle"));
  meta_ = new QLabel;
  meta_->setObjectName("toolbarMeta");
  bl->addWidget(meta_, 1);
  auto *panes = new QPushButton(NavItem::escape_mnemonic("Paneler"));
  panes->setProperty("kind", "ghost");
  panes->setCursor(Qt::PointingHandCursor);
  auto *menu = new QMenu(panes);
  for (int i = 0; i < kEditPaneCount; ++i) {
    const auto p = static_cast<EditPane>(i);
    QAction *a = menu->addAction(QString::fromStdString(edit_pane_name_da(p)));
    a->setCheckable(true);
    a->setChecked(true);
    connect(a, &QAction::toggled, this,
            [this, p](bool on) { set_pane_visible(p, on); });
    pane_actions_[i] = a;
  }
  panes->setMenu(menu);
  panes->setMinimumHeight(panes->sizeHint().height());
  bl->addWidget(panes);
  return bar;
}

void EditWorkspace::activate() { graph_->activate(); }

void EditWorkspace::set_pane_visible(EditPane p, bool on) {
  const int i = static_cast<int>(p);
  wanted_[i] = on;
  if (pane_actions_[i] && pane_actions_[i]->isChecked() != on) {
    const QSignalBlocker b(pane_actions_[i]);
    pane_actions_[i]->setChecked(on);
  }
  sync_panes();
  save_layout();
}

void EditWorkspace::sync_panes() {
  for (int i = 0; i < kEditPaneCount; ++i) {
    const bool needs_pair = static_cast<EditPane>(i) == EditPane::overlay;
    panes_[i]->setVisible(wanted_[i] && (!needs_pair || pair_complete_));
  }
  // A splitter with nothing visible would leave an empty strip.
  left_->setVisible(wanted_[0] || wanted_[1]);
}

void EditWorkspace::restore_layout() {
  if (!persist_)
    return;
  QSettings s;
  main_->restoreState(s.value("rediger/split/main").toByteArray());
  left_->restoreState(s.value("rediger/split/left").toByteArray());
  right_->restoreState(s.value("rediger/split/right").toByteArray());
  for (int i = 0; i < kEditPaneCount; ++i) {
    wanted_[i] = s.value("rediger/visible/" + key_of(static_cast<EditPane>(i)),
                         true)
                     .toBool();
    const QSignalBlocker b(pane_actions_[i]);
    pane_actions_[i]->setChecked(wanted_[i]);
  }
}

void EditWorkspace::save_layout() const {
  if (!persist_)
    return;
  QSettings s;
  s.setValue("rediger/split/main", main_->saveState());
  s.setValue("rediger/split/left", left_->saveState());
  s.setValue("rediger/split/right", right_->saveState());
  for (int i = 0; i < kEditPaneCount; ++i)
    s.setValue("rediger/visible/" + key_of(static_cast<EditPane>(i)),
               wanted_[i]);
}

} // namespace rux::qt
```

- [ ] **Step 7: Wire the shell**

- `workspaces.hpp`: rename the enumerator `posegraph` → `edit`; update the header comment ("Start, Oversigt, 3D, Rediger, Pipeline, Log").
- `workspaces.cpp`: Database info subtitle → `"Billeder, panoramaer, skyer og alle tabeller — til at se på."`; replace the `posegraph` entry with:

```cpp
      {Workspace::edit,
       name(3),
       keywords(3),
       "Posegrafen: find de mistænkelige kanter, ret dem og bekræft dem.",
       "",
       {"Graf og kantliste, farvet efter type, status eller afvigelse",
        "Billedpar med 3D-overlay, manuel justering og ICP"},
       {"rux graph discover", "rux graph prune", "rux optimize"}},
```

- `AppShell.hpp`: `struct ShellOptions` gains `/// Splitter sizes and pane visibility persist in QSettings (false in a gallery shot). bool persist_layout = true;`. Replace `PoseGraphWorkspace *posegraph() const` with `EditWorkspace *edit() const { return edit_; }` and add `EdgeEditor &edges() const { return *edges_; }`; members `EditWorkspace *edit_ = nullptr; EdgeEditor *edges_ = nullptr;` (remove `posegraph_`); forward-declare `EditWorkspace`, `EdgeEditor`.
- `AppShell.cpp` constructor: before `database_ = …` add `edges_ = new EdgeEditor(session_, this);`; `database_ = new DatabaseWorkspace(session_, *edges_);`; replace the `posegraph_` construction with `edit_ = new EditWorkspace(session_, *edges_, options.persist_layout); stack_->addWidget(edit_);`; `follow(Workspace::posegraph)` connection → `connect(edit_, &EditWorkspace::selection_changed, this, follow(Workspace::edit));`; the three Posegraf→Database connections keep working through `edit_->graph()` (replace `posegraph_` with `edit_->graph()`; Task 8 removes them). `show_page`: `else if (w == Workspace::edit) edit_->activate();`. `page_selection`: `case Workspace::edit: return edit_->selection();`. In `commands()` add before the `QAction` branch:

```cpp
    } else if (id.startsWith("pane:")) {
      if (const auto p = parse_edit_pane(id.mid(5).toStdString()))
        c.run = [this, p = *p] {
          show_page(Workspace::edit);
          edit_->toggle_pane(p);
        };
```

  `database_->editor()` uses elsewhere (`set_runner_source`, pending) become `edges_->…` where they address the editor directly; `database_->pending_edits()` etc. stay until Task 8.
- `DatabaseWorkspace`: constructor `DatabaseWorkspace(ProjectSession &session, EdgeEditor &editor, QWidget *parent = nullptr)`; member `EdgeEditor *editor_` initialised to `&editor` (delete `new EdgeEditor(session_, this)`).
- `app.cpp` smoke block:

```cpp
  // Smoke tests: RUX_QT_PAGE=oversigt|3d|rediger|pipeline|log lands on that
  // workspace once the project is open (rux_qt/palette_table.hpp).
  const std::string smoke =
      qEnvironmentVariable("RUX_QT_PAGE").toLower().toStdString();
  if (const int page = smoke_page_index(smoke); page > 0)
    QObject::connect(&session, &ProjectSession::opened, &window,
                     [&window, page] {
                       window.shell()->show_page(static_cast<Workspace>(page));
                     });
```

  (add `#include <rux_qt/palette_table.hpp>`).
- Gallery `shell_pages.cpp`: in `make_shell` set `options.persist_layout = false;`; every `shell->posegraph()` → `shell->edit()->graph()`; `shell->show_page(Workspace::posegraph)` → `Workspace::edit`; rename the registered pages `posegraf` → `rediger`, `posegraf-stress` → `rediger-stress`, `posegraf-click` → `rediger-click`, `posegraf-zoom` → `rediger-zoom` (descriptions "Rediger: …"); add

```cpp
/// Rediger with the edge list and both pair panes hidden, from the palette.
QWidget *make_rediger_panes(const PageContext &ctx) {
  auto *shell = make_pg(ctx);
  shell->edit()->set_pane_visible(EditPane::edges, false);
  shell->edit()->set_pane_visible(EditPane::images, false);
  return shell;
}
```

  registered as `rediger-panes`. Remove `gallery/demo_frame.cpp`'s `rail->add_item("Posegraf")` → `"Rediger"`, and its `"Database"` item → `"Oversigt"`.

- [ ] **Step 8: Build, smoke, shoot**

Run: `cmake --build build --target reusex_unit_tests rux rux-qt-gallery && build/tests/reusex_unit_tests "[rux_qt]"`
Expected: PASS.
Run: `COPY=$(mktemp --suffix=.rux); cp ~/.cache/reusex/qt-shot/office_corridor-*.rux "$COPY"; XDG_CONFIG_HOME=$(mktemp -d) QT_QPA_PLATFORM=offscreen RUX_QT_PAGE=rediger build/apps/rux/rux -p "$COPY" --quit-after-ms 3000`
Expected: stderr `quitting after 3000 ms; window visible=1, page=3, project state=open`.
Run: `bash .claude/skills/design-studio/scripts/qt_shot.sh --page rediger --page rediger-panes --page shell-project --page palette-all --theme both`
Expected: exit 0. Read every PNG: rail shows Start / Oversigt / 3D / Rediger / Pipeline / Log; Rediger shows Graf over Kanter on the left, Billeder on the right, 3D-overlay hidden (no pair); `rediger-panes` shows only the graph and the right column; palette lists the four "Vis eller skjul …" entries. Fix the 3–5 worst issues (splitter handle contrast, header heights), re-shoot. Run `token_lint.py` → clean.

- [ ] **Step 9: Commit**

```bash
git add apps/rux/qt tests/unit/rux_qt
git commit -m "qt: Rediger workspace shell replaces Posegraf; Database is Oversigt

Nested splitters with hideable panes (header, Paneler menu, Ctrl+K),
persisted under rediger/… in QSettings; EdgeEditor is owned by the shell."
```

---

### Task 8: Move the pending banner and `PairStrip` into Rediger; node clicks fill A/B

**Files:**
- Create: `apps/rux/qt/include/rux_qt/PendingBanner.hpp`, `apps/rux/qt/src/PendingBanner.cpp`, `apps/rux/qt/include/rux_qt/PairStrip.hpp`
- Modify: `apps/rux/qt/include/rux_qt/FrameBrowser.hpp:166-206` (declaration out), `apps/rux/qt/src/FrameBrowser.cpp:735-736,807-808,905` (strip out), `apps/rux/qt/src/PairStrip.cpp` (include)
- Modify: `apps/rux/qt/include/rux_qt/DatabaseWorkspace.hpp`, `apps/rux/qt/src/DatabaseWorkspace.cpp:239-270,336-368,529-578` (banner + pending API out)
- Modify: `apps/rux/qt/include/rux_qt/EditWorkspace.hpp`, `apps/rux/qt/src/EditWorkspace.cpp`
- Modify: `apps/rux/qt/src/PoseGraphWorkspace.cpp:599-621` (`click_node` subtitle)
- Modify: `apps/rux/qt/src/AppShell.cpp` (pending → `edit_`, drop Posegraf→Database links), `apps/rux/qt/src/PipelineWorkspace.cpp:746-756` (copy)
- Modify: `apps/rux/qt/gallery/shell_pages.cpp` (`db-pending`/`db-readonly`/`db-saved` → `rediger-pending`/`rediger-readonly`/`rediger-saved`)

**Interfaces:**
- Consumes: `SlotPicker` (Task 3), `pending_summary_da` (Task 4), `EditWorkspace` (Task 7).
- Produces:
  - `class PendingBanner : public QFrame { explicit PendingBanner(EdgeEditor &, QWidget *parent = nullptr); void sync(); }`
  - `EditWorkspace`: `void set_pair(int a, int b)`, `int a() const`, `int b() const`, `Slot next_slot() const`, `PairStrip *strip() const`, `int pending_edits() const`, `bool save_edits()`, `bool save_edits_and_wait()`, `bool is_saving() const`, `void discard_edits()`, signal `void pair_changed(int a, int b)`
  - `DatabaseWorkspace` loses `pending_edits`, `save_edits`, `save_edits_and_wait`, `is_saving`, `discard_edits`.

- [ ] **Step 1: Extract `PendingBanner`**

`PendingBanner.hpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// The pose-graph edits banner (the web's WriteBanner): how many edits wait,
// what kind, why a save failed or is held back, and Gem / Kassér. Lives at
// the top of Rediger; it watches the EdgeEditor and needs nothing else.

#include <QFrame>

class QLabel;
class QPushButton;

namespace rux::qt {

class EdgeEditor;

class PendingBanner : public QFrame {
  Q_OBJECT
    public:
  explicit PendingBanner(EdgeEditor &editor, QWidget *parent = nullptr);
  void sync();

    private:
  EdgeEditor &editor_;
  QLabel *text_ = nullptr;
  QLabel *error_ = nullptr;
  QPushButton *save_ = nullptr;
  QPushButton *discard_ = nullptr;
};

} // namespace rux::qt
```

`PendingBanner.cpp`: move the banner-building block of `DatabaseWorkspace`'s constructor (from `banner_ = new QFrame;` to `banner_->setMinimumHeight(…)`) into the constructor with `banner_` → `this`, `banner_text_` → `text_`, `banner_error_` → `error_`; then

```cpp
  connect(save_, &QPushButton::clicked, this, [this] { editor_.save(); });
  connect(discard_, &QPushButton::clicked, this, [this] { editor_.discard(); });
  connect(&editor_, &EdgeEditor::changed, this, &PendingBanner::sync);
  sync();
```

and move `DatabaseWorkspace::sync_banner()` verbatim as `PendingBanner::sync()` (with the same renames, `editor_->` → `editor_.`, `banner_` → `this`, and the count line already using `pending_summary_da` from Task 4).

- [ ] **Step 2: Strip Database of editing**

In `DatabaseWorkspace`: delete the banner block, `sync_banner`, the `save_`/`discard_`/`banner_*` members and their connects, and the pending API methods. Keep `connect(editor_, &EdgeEditor::saved, this, [this] { tree_->rebuild(session_); });` (the edge count in the tree). Move the `PairStrip` class declaration from `FrameBrowser.hpp` into a new `PairStrip.hpp` (same text, plus `#include <rux_qt/database_logic.hpp> <rux_qt/selection.hpp> <QFrame> <map>` and forward declarations of `QComboBox QLabel QPushButton QVBoxLayout EdgeEditor ProjectSession`); `PairStrip.cpp` includes `<rux_qt/PairStrip.hpp>`. In `FrameBrowser`: delete the `strip_` member and lines that create, connect and update it; update the class comment ("…the frame's pose, intrinsics and time; and a filmstrip").

- [ ] **Step 3: Pair column in Rediger, with the slot picker**

`EditWorkspace.hpp` additions (include `<rux_qt/graph_view.hpp>`; forward-declare `PairStrip`, `PendingBanner`):

```cpp
  /// A and B at once (an edge, a list row, the gallery).
  void set_pair(int a, int b);
  int a() const { return slots_.a(); }
  int b() const { return slots_.b(); }
  Slot next_slot() const { return slots_.next(); }
  PairStrip *strip() const { return strip_; }
  int pending_edits() const;
  bool save_edits();
  bool save_edits_and_wait();
  bool is_saving() const;
  void discard_edits();

    signals:
  void pair_changed(int a, int b);

    private:
  void node_clicked(int id, bool force_b);
  void apply_pair();
  void sync_pair_header();
  SlotPicker slots_;
  PendingBanner *banner_ = nullptr;
  PairStrip *strip_ = nullptr;
  QLabel *pair_a_ = nullptr;
  QLabel *pair_b_ = nullptr;
  QLabel *next_hint_ = nullptr;
```

In the constructor: insert `banner_ = new PendingBanner(editor_); v->insertWidget(1, banner_);` after the toolbar; above `right_` in `right_column_`, add the pair header:

```cpp
  auto *head = new QFrame;
  head->setObjectName("paneHeader");
  auto *hl = new QHBoxLayout(head);
  hl->setContentsMargins(t.px("--space-3"), t.px("--space-2"),
                         t.px("--space-3"), t.px("--space-2"));
  hl->setSpacing(t.px("--space-3"));
  hl->addWidget(new CapsLabel("Par", "panelTitle"));
  pair_a_ = new QLabel;
  pair_a_->setObjectName("pairIds");
  pair_b_ = new QLabel;
  pair_b_->setObjectName("pairIds");
  hl->addWidget(pair_a_);
  hl->addWidget(pair_b_);
  hl->addStretch(1);
  next_hint_ = new QLabel;
  next_hint_->setObjectName("toolbarMeta");
  hl->addWidget(next_hint_);
  auto *swap = new QPushButton(NavItem::escape_mnemonic("Byt A og B"));
  swap->setProperty("kind", "ghost");
  swap->setCursor(Qt::PointingHandCursor);
  connect(swap, &QPushButton::clicked, this, [this] { set_pair(b(), a()); });
  hl->addWidget(swap);
  rc->insertWidget(0, head);
  strip_ = new PairStrip(session_, editor_);
  rc->addWidget(strip_);
```

and the wiring:

```cpp
  connect(graph_, &PoseGraphWorkspace::frame_requested, this,
          &EditWorkspace::node_clicked);
  connect(graph_, &PoseGraphWorkspace::pair_requested, this,
          &EditWorkspace::set_pair);
  connect(strip_, &PairStrip::edge_selected, this, [this](const Selection &s) {
    selection_ = s;
    emit selection_changed(s);
  });
  connect(&session_, &ProjectSession::state_changed, this, [this] {
    slots_.clear();
    apply_pair();
  });
```

Implementations:

```cpp
void EditWorkspace::node_clicked(int id, bool force_b) {
  if (slots_.click(id, force_b))
    apply_pair();
}

void EditWorkspace::set_pair(int a, int b) {
  slots_.set_pair(a, b);
  apply_pair();
}

void EditWorkspace::apply_pair() {
  graph_->set_pair(slots_.a(), slots_.b());
  strip_->set_pair(slots_.a(), slots_.b());
  pair_complete_ = slots_.complete();
  sync_pair_header();
  sync_panes();
  emit pair_changed(slots_.a(), slots_.b());
}

void EditWorkspace::sync_pair_header() {
  auto id = [](int v) { return v >= 0 ? QString::number(v) : QString("—"); };
  pair_a_->setText("A  " + id(slots_.a()));
  pair_b_->setText("B  " + id(slots_.b()));
  next_hint_->setText(slots_.next() == Slot::a
                          ? QString("Næste klik: A · Shift+klik: B")
                          : QString("Næste klik: B"));
}

int EditWorkspace::pending_edits() const { return editor_.pending(); }
bool EditWorkspace::save_edits() { return editor_.save(); }
bool EditWorkspace::save_edits_and_wait() { return editor_.save_and_wait(this); }
bool EditWorkspace::is_saving() const { return editor_.is_saving(); }
void EditWorkspace::discard_edits() { editor_.discard(); }
```

In `PoseGraphWorkspace::click_node`, the subtitle becomes `QString("Billede i posegrafen")` (Rediger, not the workspace, knows the slot). Update `PoseGraphWorkspace`'s header comment ("A click on a node is handed to Rediger's A/B picker; a click on an edge sets both").

- [ ] **Step 4: Shell and Pipeline**

`AppShell.cpp`: delete the `frame_requested`/`pair_requested` → Database connections and the `FrameBrowser::pair_changed` → graph connection. In `resolve_pending_edits` replace `database_` with `edit_` and `show_page(Workspace::database)` with `show_page(Workspace::edit)`. The pipeline hooks become `pipeline_->set_pending_edits_source([this] { return edit_->pending_edits(); });` and `project_changed` checks `edit_->pending_edits() == 0 && !edit_->is_saving()`. In `PipelineWorkspace::run()` the informative text becomes `"Optimér poser bruger kun de gemte kanter. Gem dem i Rediger først, eller kør uden dem."`.

- [ ] **Step 5: Gallery**

Move `make_db_pending`, `make_db_readonly`, `make_db_saved` to Rediger:

```cpp
/// Rediger with A and B a few frames apart (ids differ per project).
AppShell *make_rediger(const PageContext &ctx, Open open = Open::project) {
  auto *shell = sized(make_shell(ctx, open));
  shell->show_page(Workspace::edit);
  const auto &ids = shell->database()->frames()->pair().ids();
  if (ids.size() > 1) {
    const std::size_t a = ids.size() * 3 / 10;
    shell->edit()->set_pair(ids[a], ids[std::min(ids.size() - 1, a + 12)]);
  }
  return shell;
}

QWidget *make_rediger_pending(const PageContext &ctx) {
  auto *shell = make_rediger(ctx);
  EdgeEditor &ed = shell->edges();
  if (!ed.edits().base().empty())
    ed.reject(ed.edits().base().front().key);
  for (auto *b : shell->edit()->strip()->findChildren<QPushButton *>())
    if (b->text() == "ICP-forfin")
      b->click();
  ed.add({{shell->edit()->a(), shell->edit()->b(), "loop_closure"}, 0.0, 400.0});
  return shell;
}

QWidget *make_rediger_readonly(const PageContext &ctx) {
  auto *shell = make_rediger(ctx, Open::read_only);
  shell->edges().add({{shell->edit()->a(), shell->edit()->b(), "loop_closure"}, 0.0, 1.0});
  shell->edit()->save_edits();
  return shell;
}

QWidget *make_rediger_saved(const PageContext &ctx) {
  auto *shell = make_rediger(ctx);
  shell->edges().add({{shell->edit()->a(), shell->edit()->b(), "loop_closure"}, 0.0, 250.0});
  shell->edit()->save_edits();
  return shell;
}
```

register them as `rediger-pending`, `rediger-readonly`, `rediger-saved` (descriptions carried over, "Database:" → "Rediger:"), delete the three `db-*` registrations, and make `make_pg` call `shell->edit()->set_pair(at(0.30), at(0.81))` instead of the FrameBrowser setters, with `shell->edges().add(…)` for its staged closures.

- [ ] **Step 6: Build, smoke and shoot**

Run: `cmake --build build --target reusex_unit_tests rux rux-qt-gallery && build/tests/reusex_unit_tests "[rux_qt]"`
Expected: PASS.
Run: `bash .claude/skills/design-studio/scripts/qt_shot.sh --page rediger --page rediger-pending --page rediger-readonly --page rediger-saved --page db-frames --theme both`
Expected: exit 0. Read every PNG: Rediger has the banner on top ("… venter på at blive gemt — 1 ny eller målt kant, 1 afvist"), the pair header "A 4 · B 8 · Næste klik: A", the strip with rows and ICP line at the bottom of the right column; readonly shows the warn-tone banner; db-frames has no edge strip and no banner. Fix the 3–5 worst issues, re-shoot; `token_lint.py` clean.
Run the smoke from Task 7 Step 8 with `RUX_QT_PAGE=oversigt` and `=rediger` → both `window visible=1`.

- [ ] **Step 7: Commit**

```bash
git add apps/rux/qt
git commit -m "qt: pending banner and pair strip move to Rediger; clicks fill A then B

Oversigt (Database) is read-only again; a node click fills the next free
slot (Shift: B), an edge click both."
```

---
### Task 9: Graph canvas — view planes, view yaw (persisted), heading ticks

`PoseGraphWorkspace` becomes `GraphPanel`: nodes come from the full 3D translation, projected through the yaw and the chosen plane; each node gets a fixed-length heading tick; the yaw is a project setting written off the GUI thread.

**Files:**
- Rename: `apps/rux/qt/include/rux_qt/PoseGraphWorkspace.hpp` → `GraphPanel.hpp`, `apps/rux/qt/src/PoseGraphWorkspace.cpp` → `GraphPanel.cpp` (`git mv`), class `PoseGraphWorkspace` → `GraphPanel` everywhere (`EditWorkspace.*`, `AppShell.cpp`, `gallery/shell_pages.cpp`)
- Create: `apps/rux/qt/include/rux_qt/ViewSettings.hpp`, `apps/rux/qt/src/ViewSettings.cpp`
- Modify: `apps/rux/qt/include/rux_qt/AppShell.hpp`, `apps/rux/qt/src/AppShell.cpp` (own `ViewSettings`, runner source)
- Modify: `apps/rux/qt/styles/app.qss` (yaw controls), `apps/rux/qt/gallery/shell_pages.cpp`

**Interfaces:**
- Consumes: `ViewProjection`, `project_point`, `camera_forward`, `heading_in_view`, `auto_yaw_deg`, `parse_yaw_setting`, `format_yaw_setting`, `kViewYawSetting` (Task 2); `ProjectDB::setting` / `set_setting`.
- Produces:
  - `class ViewSettings : public QObject { ViewSettings(ProjectSession &, QObject *parent = nullptr); double yaw_deg() const; void set_yaw_deg(double); bool persists() const; void set_runner_source(EdgeEditor::RunnerSource); signals: void yaw_changed(double deg); }`
  - `class GraphPanel : public QWidget { GraphPanel(ProjectSession &, EdgeEditor &, ViewSettings &, bool persist, QWidget *parent = nullptr); void activate(); void set_pair(int, int); void set_plane(ViewPlane); const ViewProjection &projection() const; PoseGraphView *view() const; int node_count() const; QPointF node_scene_pos(int) const; void click_node(int id, bool as_b = false); void click_edge(int from, int to); signals: frame_requested(int, bool), pair_requested(int, int), selection_changed(const Selection &) }`
  - `AppShell::view_settings() const -> ViewSettings &`; `EditWorkspace(ProjectSession &, EdgeEditor &, ViewSettings &, bool persist_layout, QWidget *parent = nullptr)`; `EditWorkspace::graph() const -> GraphPanel *`

- [ ] **Step 1: `ViewSettings`**

`ViewSettings.hpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// The project's display settings (spec D7): today the view yaw, read from
// project_settings when a project opens and written back half a second
// after the last change — off the GUI thread, on its own connection, under
// the pipeline writer lease (like EdgeEditor::save). Display only: nothing
// in the model rotates. On a read-only project the yaw applies but is not
// written (persists() is false).

#include <rux_qt/EdgeEditor.hpp>

#include <QObject>

class QTimer;

namespace rux::qt {

class ProjectSession;

class ViewSettings : public QObject {
  Q_OBJECT
    public:
  explicit ViewSettings(ProjectSession &session, QObject *parent = nullptr);
  double yaw_deg() const { return yaw_; }
  /// Apply now (yaw_changed), write after a pause.
  void set_yaw_deg(double deg);
  bool persists() const;
  void set_runner_source(EdgeEditor::RunnerSource source) {
    runner_source_ = std::move(source);
  }

    signals:
  void yaw_changed(double deg);

    private:
  void reload();
  void write_now();
  ProjectSession &session_;
  EdgeEditor::RunnerSource runner_source_;
  QTimer *debounce_ = nullptr;
  double yaw_ = 0.0;
};

} // namespace rux::qt
```

`ViewSettings.cpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <rux_qt/ProjectSession.hpp>
#include <rux_qt/ViewSettings.hpp>
#include <rux_qt/background.hpp>
#include <rux_qt/graph_view.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/logging.hpp>

#include <QApplication>
#include <QPointer>
#include <QTimer>

#include <thread>

namespace rux::qt {

ViewSettings::ViewSettings(ProjectSession &session, QObject *parent)
    : QObject(parent), session_(session) {
  debounce_ = new QTimer(this);
  debounce_->setSingleShot(true);
  debounce_->setInterval(500);
  connect(debounce_, &QTimer::timeout, this, &ViewSettings::write_now);
  connect(&session_, &ProjectSession::opened, this, &ViewSettings::reload);
  connect(&session_, &ProjectSession::closed, this, &ViewSettings::reload);
}

bool ViewSettings::persists() const {
  return session_.is_open() && !session_.is_read_only();
}

void ViewSettings::reload() {
  debounce_->stop();
  double yaw = 0.0;
  if (const reusex::ProjectDB *db = session_.db()) {
    try {
      if (const auto text = db->setting(kViewYawSetting))
        yaw = parse_yaw_setting(*text).value_or(0.0);
    } catch (const std::exception &e) {
      reusex::warn("view yaw not read: {}", e.what());
    }
  }
  yaw_ = yaw;
  emit yaw_changed(yaw_);
}

void ViewSettings::set_yaw_deg(double deg) {
  const double y = normalize_yaw_deg(deg);
  if (format_yaw_setting(y) == format_yaw_setting(yaw_))
    return;
  yaw_ = y;
  emit yaw_changed(yaw_);
  if (persists())
    debounce_->start();
}

void ViewSettings::write_now() {
  if (!persists())
    return;
  const std::string path = session_.path().toStdString();
  const std::string text = format_yaw_setting(yaw_);
  std::weak_ptr<reusex::pipeline::JobRunner> runner;
  if (runner_source_)
    runner = runner_source_();
  auto work = std::make_shared<BackgroundWork>();
  QPointer<ViewSettings> guard(this);
  std::thread([path, text, runner, guard, work]() mutable {
    bool busy = false;
    std::shared_ptr<reusex::pipeline::JobRunner> holder = runner.lock();
    reusex::pipeline::WriterLease lease;
    if (holder) {
      lease = holder->try_acquire_writer(EdgeEditor::kLeaseTimeout);
      busy = !lease.owns_lock();
    }
    if (!busy)
      try {
        reusex::ProjectDB db(path, /*readOnly=*/false);
        db.set_setting(kViewYawSetting, text);
      } catch (const std::exception &e) {
        reusex::warn("view yaw not saved: {}", e.what());
      }
    if (busy) // a stage holds the project: try again shortly
      QMetaObject::invokeMethod(
          qApp,
          [guard] {
            if (guard)
              guard->debounce_->start();
          },
          Qt::QueuedConnection);
    if (lease.owns_lock())
      lease.unlock();
    holder.reset();
    work.reset();
  }).detach();
}

} // namespace rux::qt
```

`AppShell`: member `ViewSettings *view_settings_`, created right after `edges_` (`view_settings_ = new ViewSettings(session_, this);`), accessor `ViewSettings &view_settings() const { return *view_settings_; }`, and next to `edges_->set_runner_source(…)`: `view_settings_->set_runner_source([this] { return pipeline_->runner_handle(); });`. Pass it to `EditWorkspace` (new constructor parameter, forwarded to `GraphPanel`).

- [ ] **Step 2: Rename and reload nodes in 3D**

`git mv apps/rux/qt/include/rux_qt/PoseGraphWorkspace.hpp apps/rux/qt/include/rux_qt/GraphPanel.hpp` and the `.cpp`; `sed -i 's/PoseGraphWorkspace/GraphPanel/g'` over `apps/rux/qt` (check the diff: only class name, includes and comments). In `GraphPanel.hpp` add (include `<rux_qt/graph_view.hpp>`, `<array>`):

```cpp
  void set_plane(ViewPlane plane);
  const ViewProjection &projection() const { return projection_; }

    private:
  /// One posed frame as loaded: id, scan, world translation, camera forward.
  struct NodeData {
    int id = 0;
    int scan = 0;
    std::array<double, 3> t{};
    std::array<double, 3> forward{};
  };
  void project();
  void set_yaw(double deg);
  ViewSettings &view_settings_;
  bool persist_ = true;
  ViewProjection projection_;
  std::vector<NodeData> data_;
  std::vector<Heading> ticks_; ///< per node, after project()
  QButtonGroup *planes_ = nullptr;
  QSlider *yaw_slider_ = nullptr;
  QDoubleSpinBox *yaw_spin_ = nullptr;
```

(and remove `scan_of_`; scans now come from `data_`). `struct Loaded` holds `std::vector<NodeData> nodes`. In the loader thread replace the loop body with:

```cpp
        const auto pose = db.sensor_frame_pose(id);
        std::array<double, 16> local{1, 0, 0, 0, 0, 1, 0, 0,
                                     0, 0, 1, 0, 0, 0, 0, 1};
        try {
          const auto &l = db.sensor_frame_intrinsics(id).local_transform;
          for (std::size_t i = 0; i < 16; ++i)
            local[i] = l[i];
        } catch (const std::exception &) {
          // no intrinsics: the pose's own z is the optical axis
        }
        d->nodes.push_back({id, scan, {pose[3], pose[7], pose[11]},
                            camera_forward(pose, local)});
```

`loaded()` stores `data_ = std::move(d->nodes); project(); rebuild_scene();`. Add:

```cpp
void GraphPanel::project() {
  nodes_.clear();
  ticks_.clear();
  nodes_.reserve(data_.size());
  ticks_.reserve(data_.size());
  for (const auto &n : data_) {
    const Vec2 p = project_point(projection_, n.t[0], n.t[1], n.t[2]);
    nodes_.push_back({n.id, p.x, p.y});
    ticks_.push_back(heading_in_view(projection_, n.forward));
  }
}

void GraphPanel::set_plane(ViewPlane plane) {
  projection_.plane = plane;
  if (planes_)
    if (auto *b = planes_->button(static_cast<int>(plane)))
      b->setChecked(true);
  if (persist_)
    QSettings().setValue("rediger/plane",
                         QString::fromLatin1(view_plane_key(plane).data()));
  project();
  rebuild_scene();
  view_->fit();
}

void GraphPanel::set_yaw(double deg) {
  projection_.yaw_deg = deg;
  {
    const QSignalBlocker a(yaw_slider_), b(yaw_spin_);
    yaw_slider_->setValue(static_cast<int>(std::lround(deg)));
    yaw_spin_->setValue(deg);
  }
  project();
  rebuild_scene();
  view_->fit();
}
```

Everywhere the old code used `scan_of_[i]`, use `data_[i].scan`; the inspector's node section shows `X`, `Y` and `Z` from `data_[i].t` (world, not projected) plus "Scanning" and "Kanter".

- [ ] **Step 3: Heading ticks in `NodesItem`**

`NodesItem` gets the ticks: constructor `NodesItem(const std::vector<GraphNode> &nodes, const std::vector<int> &scans, std::vector<Heading> ticks)`, member `std::vector<Heading> ticks_`; in the Task 1 device-space loop, after `p->drawEllipse(d, r, r);`:

```cpp
        const Heading &h = ticks_[i];
        if (h.visible)
          p->drawLine(d, d + QPointF(h.dir.x, -h.dir.y) * kTickPx);
```

with `constexpr double kTickPx = 7.0;` beside `kNodePx`, and before each scan group `QPen tick(colour, 1.5); tick.setCapStyle(Qt::RoundCap); p->setPen(tick);` (the brush stays the dot fill; `drawEllipse` with a pen also outlines the dot in the same colour, which is fine). `rebuild_scene()` builds the trail and the markers from the projected `nodes_` as before and passes `ticks_` to `NodesItem`.

- [ ] **Step 4: Toolbar controls**

In the `GraphPanel` constructor's toolbar (after the meta label; delete the old legend block — Task 10 rebuilds it), add:

```cpp
  auto *seg = new QFrame;
  seg->setObjectName("segmented");
  auto *sl = new QHBoxLayout(seg);
  sl->setContentsMargins(0, 0, 0, 0);
  sl->setSpacing(0);
  planes_ = new QButtonGroup(this);
  const std::array<std::pair<const char *, const char *>, 3> planes = {
      {{"XY", "Plan, set ovenfra"},
       {"XZ", "Opstalt, set fra −y"},
       {"YZ", "Opstalt, set fra +x"}}};
  for (int i = 0; i < 3; ++i) {
    auto *b = new QPushButton(planes[i].first);
    b->setObjectName("segment");
    b->setCheckable(true);
    b->setCursor(Qt::PointingHandCursor);
    b->setProperty("position", i == 0 ? "first" : i == 2 ? "last" : "middle");
    b->setToolTip(planes[i].second);
    planes_->addButton(b, i);
    sl->addWidget(b);
  }
  bl->addWidget(seg);

  bl->addWidget(new CapsLabel("Drej", "fieldLabel"));
  yaw_slider_ = new QSlider(Qt::Horizontal);
  yaw_slider_->setObjectName("yawSlider");
  yaw_slider_->setRange(-180, 180);
  yaw_slider_->setMinimumWidth(theme().px("--layout-nav-width") / 2);
  bl->addWidget(yaw_slider_);
  yaw_spin_ = new QDoubleSpinBox;
  yaw_spin_->setObjectName("yawSpin");
  yaw_spin_->setRange(-180.0, 180.0);
  yaw_spin_->setDecimals(1);
  yaw_spin_->setSuffix(" °");
  yaw_spin_->setButtonSymbols(QAbstractSpinBox::NoButtons);
  yaw_spin_->setLocale(QLocale(QLocale::Danish, QLocale::Denmark));
  yaw_spin_->setToolTip("Visningens drejning om lodret akse. Gemmes i projektet "
                        "og bruges også af 3D og rux render; modellen ændres ikke.");
  bl->addWidget(yaw_spin_);
  auto *autob = new QPushButton(NavItem::escape_mnemonic("Auto"));
  autob->setProperty("kind", "ghost");
  autob->setCursor(Qt::PointingHandCursor);
  autob->setToolTip("Drej, så bygningens hovedretning ligger vandret — ud fra "
                    "kameraernes retninger");
  bl->addWidget(autob);

  connect(planes_, &QButtonGroup::idClicked, this,
          [this](int id) { set_plane(static_cast<ViewPlane>(id)); });
  connect(yaw_slider_, &QSlider::valueChanged, this,
          [this](int v) { view_settings_.set_yaw_deg(v); });
  connect(yaw_spin_, &QDoubleSpinBox::valueChanged, this,
          [this](double v) { view_settings_.set_yaw_deg(v); });
  connect(autob, &QPushButton::clicked, this, [this] {
    std::vector<std::array<double, 3>> f;
    f.reserve(data_.size());
    for (const auto &n : data_)
      f.push_back(n.forward);
    view_settings_.set_yaw_deg(auto_yaw_deg(f));
  });
  connect(&view_settings_, &ViewSettings::yaw_changed, this,
          &GraphPanel::set_yaw);
  ViewPlane plane = ViewPlane::xy;
  if (persist_)
    plane = parse_view_plane(QSettings().value("rediger/plane", "xy")
                                 .toString().toStdString())
                .value_or(ViewPlane::xy);
  projection_.plane = plane;
  planes_->button(static_cast<int>(plane))->setChecked(true);
  set_yaw(view_settings_.yaw_deg());
```

Update the status hint: `"Klik: næste af A/B · Shift+klik: B · klik på en kant: A og B · hjul: zoom · træk: panorér · F: tilpas"`.

- [ ] **Step 5: Gallery pages for planes and yaw**

```cpp
QWidget *make_rediger_plane(const PageContext &ctx, ViewPlane plane) {
  auto *shell = make_pg(ctx);
  settle();
  shell->edit()->graph()->set_plane(plane);
  return shell;
}
QWidget *make_rediger_xz(const PageContext &ctx) { return make_rediger_plane(ctx, ViewPlane::xz); }
QWidget *make_rediger_yz(const PageContext &ctx) { return make_rediger_plane(ctx, ViewPlane::yz); }

/// Yaw 30° (WRITES project_settings to the copy) and reads it back.
QWidget *make_rediger_yaw(const PageContext &ctx) {
  auto *shell = make_pg(ctx);
  settle();
  shell->view_settings().set_yaw_deg(30.0);
  QElapsedTimer t;
  t.start();
  while (t.elapsed() < 2000) { // the 500 ms debounce, then the write
    QApplication::processEvents();
    QThread::msleep(10);
  }
  settle();
  if (!ctx.project_path.isEmpty()) {
    reusex::ProjectDB db(ctx.project_path.toStdString(), /*readOnly=*/true);
    std::fprintf(stderr, "rediger-yaw: stored view.yaw_deg=%s\n",
                 db.setting(kViewYawSetting).value_or("<none>").c_str());
  }
  return shell;
}
```

Register `rediger-xz`, `rediger-yz`, `rediger-yaw`. In `make_rediger_readonly` add `shell->view_settings().set_yaw_deg(30.0);` and the same wait + print with the prefix `rediger-readonly:`.

- [ ] **Step 6: Build and smoke**

Run: `cmake --build build --target reusex_unit_tests rux rux-qt-gallery && build/tests/reusex_unit_tests "[rux_qt]"`
Expected: PASS.

- [ ] **Step 7: Shoot and review**

Run: `bash .claude/skills/design-studio/scripts/qt_shot.sh --page rediger --page rediger-xz --page rediger-yz --page rediger-yaw --page rediger-zoom --theme both 2>&1 | grep -E "rediger-(yaw|zoom)|error"`
Expected: exit 0; `rediger-yaw: stored view.yaw_deg=30.0`; every `rediger-zoom` level `node-pixels` > 0. Read every PNG: ticks visible and consistent in direction along the capture path in XY; XZ/YZ show the floor height spread; the yaw rotates the plan by 30° CCW. Critique (tick length vs dot, toolbar crowding at 1440 px), fix, re-shoot; `token_lint.py` clean.

- [ ] **Step 8: Read-only yaw**

Run: `bash .claude/skills/design-studio/scripts/qt_shot.sh --page rediger-readonly --theme dark 2>&1 | grep rediger-readonly`
Expected: `rediger-readonly: stored view.yaw_deg=<none>` and the PNG shows the plan rotated 30° (applied in memory).

- [ ] **Step 9: Commit**

```bash
git add -A apps/rux/qt
git commit -m "qt: Rediger graph in XY/XZ/YZ with a persisted view yaw and heading ticks"
```

---

### Task 10: Graph canvas — colour modes, generated legend, rejected toggle, selected edge

**Files:**
- Modify: `apps/rux/qt/include/rux_qt/GraphPanel.hpp`, `apps/rux/qt/src/GraphPanel.cpp` (`EdgesItem`, `rebuild_edges`, legend, `on_click`, `click_edge`)
- Modify: `apps/rux/qt/styles/app.qss` (legend), `apps/rux/qt/gallery/shell_pages.cpp` (seeded graphs, colour pages, stress page)

**Interfaces:**
- Consumes: Task 5 (`EdgeColourMode`, `colour_scale`, `edge_style`, `build_legend`, `kRampTokens`, `kRampBuckets`, `edge_drawn`, `unknown_edge_types`), Task 4 (`EdgeView`, `effective()`).
- Produces: `GraphPanel::set_colour_mode(EdgeColourMode)`, `EdgeColourMode colour_mode() const`, `void set_show_rejected(bool)`, `void set_highlight(std::optional<EdgeKey>)`; signals `void colour_mode_changed(rux::qt::EdgeColourMode mode)`, `void edge_clicked(const rux::qt::EdgeKey &key)`; `QColor ramp_colour(int bucket)` (file-local).

- [ ] **Step 1: Seed graphs for the shots (the gallery)**

In `shell_pages.cpp` add (and `#include <reusex/core/ProjectDB.hpp>`):

```cpp
/// Writes a deterministic pose graph to the project COPY: odometry along
/// the capture plus @p loops loop closures / panorama edges with residuals
/// spanning the χ² scale, metrics, every status, and legacy rows without
/// metrics — so every colour mode has something to show.
void seed_graph(const QString &path, std::size_t loops) {
  if (path.isEmpty())
    return;
  reusex::ProjectDB db(path.toStdString());
  const auto ids = db.sensor_frame_ids();
  std::uint64_t x = 0x9E3779B97F4A7C15ULL; // fixed seed: the same shot
  auto next = [&x] {
    x ^= x << 13;
    x ^= x >> 7;
    x ^= x << 17;
    return x;
  };
  reusex::ProjectDB::Transaction tx(db);
  for (std::size_t i = 1; i < ids.size(); ++i) {
    reusex::ProjectDB::PoseGraphEdge e;
    e.from_node_id = ids[i - 1];
    e.to_node_id = ids[i];
    e.edge_type = "odometry";
    e.source = "odometry_init";
    e.status = "auto";
    e.weight = 100.0;
    e.residual = 0.2 + static_cast<double>(next() % 100) / 50.0;
    e.deviation_m = static_cast<double>(next() % 100) / 2000.0;
    e.deviation_deg = static_cast<double>(next() % 100) / 50.0;
    e.seed_disagreement_m = static_cast<double>(next() % 100) / 1000.0;
    e.seed_disagreement_deg = static_cast<double>(next() % 100) / 40.0;
    db.upsert_pose_graph_edge(e);
  }
  for (std::size_t k = 0; k < loops && ids.size() > 2; ++k) {
    const int a = ids[next() % ids.size()];
    const int b = ids[next() % ids.size()];
    if (a == b)
      continue;
    reusex::ProjectDB::PoseGraphEdge e;
    e.from_node_id = a;
    e.to_node_id = b;
    e.edge_type = k % 9 == 0 ? "panorama" : "loop_closure";
    e.status = k % 7 == 0 ? "rejected" : k % 5 == 0 ? "confirmed" : "auto";
    e.weight = 25.0;
    if (k % 11 == 0) {
      e.source = "legacy"; // no measurement, no metrics
    } else {
      e.source = "orb";
      e.residual = static_cast<double>(next() % 1000) / 20.0; // 0..50
      e.deviation_m = static_cast<double>(next() % 1000) / 1000.0;
      e.deviation_deg = static_cast<double>(next() % 1000) / 100.0;
      e.seed_disagreement_m = static_cast<double>(next() % 1000) / 500.0;
      e.seed_disagreement_deg = static_cast<double>(next() % 1000) / 50.0;
    }
    db.upsert_pose_graph_edge(e);
  }
  tx.commit();
}

QWidget *make_rediger_colour(const PageContext &ctx, EdgeColourMode mode,
                             bool rejected = false) {
  seed_graph(ctx.project_path, 40);
  auto *shell = make_pg(ctx);
  settle();
  shell->edit()->graph()->set_show_rejected(rejected);
  shell->edit()->graph()->set_colour_mode(mode);
  return shell;
}
```

with one-line page factories `make_rediger_type/status/residual/deviation/seed` (status passes `rejected = true`), registered as `rediger-type`, `rediger-status`, `rediger-residual`, `rediger-deviation`, `rediger-seed` ("Rediger: kanter farvet efter …, seedet graf (skriver til projektkopien)"). Rewrite `make_posegraf_stress` (now `rediger-stress`) to write its 10 000 edges with `upsert_pose_graph_edge` inside one `Transaction` instead of `save_pose_graph_edges`, `status = "auto"`, `source = "orb"`, and keep its measurement printout.

- [ ] **Step 2: Batches carry a resolved colour**

In `GraphPanel.cpp` change `EdgesItem::Batch` to `{ QColor colour; double width = 1.0; bool dashed = false; std::vector<QLineF> lines; }` and its `paint` to `QPen pen(b.colour, b.width);`. Add a file-local helper:

```cpp
/// Continuous modes: mix the three ramp tokens (good -> middle -> bad).
QColor ramp_colour(int bucket) {
  const double t = (bucket + 0.5) / kRampBuckets;
  auto token = [](std::size_t i) {
    return theme().color(QString::fromLatin1(kRampTokens[i].data(),
                                             static_cast<int>(kRampTokens[i].size())));
  };
  const QColor a = token(0), m = token(1), b = token(2);
  auto mix = [](const QColor &x, const QColor &y, double u) {
    return QColor::fromRgbF(x.redF() + (y.redF() - x.redF()) * u,
                            x.greenF() + (y.greenF() - x.greenF()) * u,
                            x.blueF() + (y.blueF() - x.blueF()) * u);
  };
  return t < 0.5 ? mix(a, m, t * 2.0) : mix(m, b, t * 2.0 - 1.0);
}
```

- [ ] **Step 3: Build edges from the effective graph with the mode's style**

Members (header): `EdgeColourMode mode_ = EdgeColourMode::type; bool show_rejected_ = false; std::optional<EdgeKey> highlight_; std::vector<EdgeView> drawn_; QComboBox *colour_ = nullptr; QCheckBox *rejected_ = nullptr; QWidget *legend_ = nullptr; QHBoxLayout *legend_layout_ = nullptr;` — `DrawnEdge` and `edges_` are replaced by `drawn_`. `rebuild_edges()`:

```cpp
void GraphPanel::rebuild_edges() {
  QGraphicsScene *sc = view_->scene();
  for (QGraphicsItem *it : edge_items_) {
    sc->removeItem(it);
    delete it;
  }
  edge_items_.clear();
  drawn_.clear();
  const auto all = editor_.edits().effective();
  for (const auto &v : all)
    if (edge_drawn(v, show_rejected_))
      drawn_.push_back(v);
  rebuild_legend(all);
  if (nodes_.empty())
    return;
  std::map<int, QPointF> at;
  for (const auto &n : nodes_)
    at[n.id] = QPointF(n.x, -n.y);
  const ColourScale scale = colour_scale(mode_, drawn_);
  const auto unknown = unknown_edge_types(drawn_);
  const int slots = label_slots();
  // Batch by (wide, dashed, colour key): hairlines under wide strokes,
  // dashed (staged / rejected) on top, the highlighted edge last.
  std::map<std::tuple<bool, bool, std::string>, EdgesItem::Batch> batches;
  EdgesItem::Batch highlight{theme().color("--color-on-chrome"), 4.0, false, {}};
  for (const auto &v : drawn_) {
    const auto a = at.find(v.edge.key.from), b = at.find(v.edge.key.to);
    if (a == at.end() || b == at.end())
      continue;
    const EdgeStyle st = edge_style(v, mode_, scale, unknown, slots);
    const std::string key =
        st.ramp >= 0 ? "ramp:" + std::to_string(st.ramp) : st.token;
    auto &batch = batches[{st.wide, st.dashed, key}];
    if (batch.lines.empty()) {
      batch.colour = st.ramp >= 0 ? ramp_colour(st.ramp)
                                  : theme().color(QString::fromStdString(st.token));
      batch.width = st.wide ? 2.0 : 1.0;
      batch.dashed = st.dashed;
    }
    batch.lines.emplace_back(a->second, b->second);
    if (highlight_ && v.edge.key == *highlight_)
      highlight.lines.emplace_back(a->second, b->second);
  }
  std::vector<EdgesItem::Batch> ordered;
  for (auto &[k, b] : batches)
    ordered.push_back(std::move(b)); // map order: hairlines, then wide; solid before dashed
  if (!highlight.lines.empty())
    ordered.push_back(std::move(highlight));
  auto *item = new EdgesItem(std::move(ordered));
  sc->addItem(item);
  edge_items_.push_back(item);
}
```

`on_click` uses `drawn_` for the edge hit test and emits the key:

```cpp
  std::vector<GraphEdge> plain;
  plain.reserve(drawn_.size());
  for (const auto &v : drawn_)
    plain.push_back({v.edge.key.from, v.edge.key.to});
  if (const int e = nearest_edge(nodes_, plain, pos.x(), -pos.y(), tol); e >= 0) {
    const EdgeKey key = drawn_[static_cast<std::size_t>(e)].edge.key;
    click_edge(key.from, key.to);
    emit edge_clicked(key);
  }
```

`click_edge` builds the inspector selection from the matching `drawn_` entry: title `"%1 → %2"`, subtitle `edge_type_da`, pill from `edge_status_da` (+ `" · ikke gemt"` when pending), section "Kant" with rows Type, Kilde (`edge_source_da`), Residual, Afvigelse (m / °), Frøafvigelse (m / °) via `edge_cell_text`, and Vægt. Add `#include <rux_qt/edge_colouring.hpp> <rux_qt/edge_table.hpp>`.

- [ ] **Step 4: The toolbar's colour combo, rejected toggle and generated legend**

In the constructor, after the yaw controls:

```cpp
  bl->addWidget(new CapsLabel("Farv efter", "fieldLabel"));
  colour_ = new QComboBox;
  colour_->setObjectName("edgeColour");
  for (auto m : colour_modes())
    colour_->addItem(QString::fromStdString(colour_mode_da(m)),
                     QString::fromLatin1(colour_mode_key(m).data()));
  bl->addWidget(colour_);
  rejected_ = new QCheckBox("Vis afviste");
  rejected_->setToolTip("Afviste kanter bruges ikke af optimeringen; vist "
                        "stiplet");
  bl->addWidget(rejected_);
  connect(colour_, &QComboBox::currentIndexChanged, this, [this](int i) {
    set_colour_mode(colour_modes()[static_cast<std::size_t>(i)]);
  });
  connect(rejected_, &QCheckBox::toggled, this, &GraphPanel::set_show_rejected);
```

Under the toolbar, a legend row:

```cpp
  legend_ = new QFrame;
  legend_->setObjectName("graphLegend");
  legend_layout_ = new QHBoxLayout(legend_);
  legend_layout_->setContentsMargins(theme().px("--space-4"), theme().px("--space-1"),
                                     theme().px("--space-4"), theme().px("--space-1"));
  v->insertWidget(1, legend_); // under the toolbar, over the canvas
```

rebuilt by:

```cpp
void GraphPanel::rebuild_legend(const std::vector<EdgeView> &all) {
  while (QLayoutItem *it = legend_layout_->takeAt(0)) {
    if (QWidget *w = it->widget()) {
      w->hide();
      w->deleteLater();
    }
    delete it;
  }
  const Theme &t = theme();
  const EdgeLegend l = build_legend(mode_, all, show_rejected_, label_slots());
  auto text = [](const QString &s, const char *name = "legendText") {
    auto *q = new QLabel(s);
    q->setObjectName(name);
    return q;
  };
  if (l.continuous) {
    if (l.scale.valid) {
      const auto num = [&](double v) {
        return QString::fromStdString(format_decimal_da(v, 2)) +
               (l.unit.empty() ? QString() : " " + QString::fromStdString(l.unit));
      };
      legend_layout_->addWidget(text(num(l.scale.lo), "legendCount"));
      legend_layout_->addWidget(new RampBar);
      legend_layout_->addWidget(text(num(l.scale.hi), "legendCount"));
      legend_layout_->addWidget(text(l.scale.fixed ? QString("χ²₆ 95 % → 5×")
                                                   : QString("5.–95. percentil")));
    }
    if (l.missing > 0) {
      legend_layout_->addWidget(new Swatch("--color-text-faint"));
      legend_layout_->addWidget(text(QString("Ingen måling %1").arg(format_count(l.missing))));
    }
  }
  for (const auto &e : l.entries) {
    legend_layout_->addWidget(new Swatch(QString::fromStdString(e.token)));
    legend_layout_->addWidget(text(QString::fromStdString(e.label) +
                                   (e.dashed ? QString(" (stiplet)") : QString())));
    legend_layout_->addWidget(text(format_count(e.count), "legendCount"));
  }
  if (l.hidden_rejected > 0)
    legend_layout_->addWidget(
        text(QString("%1 afviste skjult").arg(format_count(l.hidden_rejected)),
             "legendNote"));
  legend_layout_->addStretch(1);
  legend_layout_->setSpacing(t.px("--space-2"));
}
```

`RampBar` (file-local `QWidget`, fixed size `theme().px("--space-7") * 2` × `theme().px("--space-2")`) paints `kRampBuckets` adjacent rectangles in `ramp_colour(i)`. Setters:

```cpp
void GraphPanel::set_colour_mode(EdgeColourMode m) {
  mode_ = m;
  {
    const QSignalBlocker b(colour_);
    colour_->setCurrentIndex(static_cast<int>(
        std::find(colour_modes().begin(), colour_modes().end(), m) -
        colour_modes().begin()));
  }
  if (persist_)
    QSettings().setValue("rediger/colour",
                         QString::fromLatin1(colour_mode_key(m).data()));
  rebuild_edges();
  emit colour_mode_changed(m);
}

void GraphPanel::set_show_rejected(bool on) {
  show_rejected_ = on;
  {
    const QSignalBlocker b(rejected_);
    rejected_->setChecked(on);
  }
  if (persist_)
    QSettings().setValue("rediger/show_rejected", on);
  rebuild_edges();
}

void GraphPanel::set_highlight(std::optional<EdgeKey> key) {
  highlight_ = std::move(key);
  rebuild_edges();
}
```

Restore `rediger/colour` / `rediger/show_rejected` in the constructor when `persist_`. QSS:

```css
QFrame#graphLegend {
  background: var(--color-chrome);
  border-bottom: 1px solid var(--color-chrome-border);
}
QLabel#legendCount {
  font-family: var(--font-mono);
  font-size: var(--font-size-xs);
  color: var(--color-on-chrome-muted);
}
QLabel#legendNote {
  font-size: var(--font-size-xs);
  color: var(--color-on-chrome-muted);
}
```

(check `QLabel#legendText` already exists for the old legend; reuse it.)

- [ ] **Step 5: Build**

Run: `cmake --build build --target reusex_unit_tests rux rux-qt-gallery && build/tests/reusex_unit_tests "[rux_qt]"`
Expected: PASS.

- [ ] **Step 6: Shoot every colour mode**

Run: `bash .claude/skills/design-studio/scripts/qt_shot.sh --page rediger-type --page rediger-status --page rediger-residual --page rediger-deviation --page rediger-seed --theme both`
Expected: exit 0. Read all ten PNGs and check: the legend lists only what is drawn, with counts; status shows the rejected edges dashed; residual's bar is labelled with the fixed scale; deviation/seed say "5.–95. percentil" and "Ingen måling N" for the legacy rows; odometry stays a hairline under the loop closures in every mode.

- [ ] **Step 7: Critique and fix**

Against `.claude/skills/design-studio/references/critique-checklist.md` "Qt client": legend contrast on the chrome strip in the light theme, ramp-bar size vs swatches, combo width, dashed edge legibility over the trail. Fix the 3–5 biggest problems, re-shoot, `token_lint.py` clean.

- [ ] **Step 8: Click an edge in the shot**

Update `make_posegraf_click` (now `rediger-click`) to seed the graph, then click the first drawn loop closure (`shell->edit()->graph()->click_edge(from, to)` for the first base edge with type `loop_closure`) — the shot shows it highlighted and the inspector filled. Shoot `rediger-click` both themes and Read.

- [ ] **Step 9: Performance — the stress page**

Run: `bash .claude/skills/design-studio/scripts/qt_shot.sh --page rediger-stress --theme dark 2>&1 | grep stress`
Expected: `… ms/frame` below 50 and `a staged edit` below 100 ms at 1440×900. If not, profile `rebuild_edges` (the `std::map` per edge is the first suspect: precompute `at` as a vector indexed through a sorted id list) before moving on.

- [ ] **Step 10: Commit**

```bash
git add apps/rux/qt
git commit -m "qt: colour edges by type/status/residual/deviation/seed with a generated legend

Rejected edges hidden by default (shown dashed on demand); unknown edge
types get their own swatch; the clicked edge is highlighted."
```

---
### Task 11: Edge list pane

**Files:**
- Create: `apps/rux/qt/include/rux_qt/EdgeListPanel.hpp`, `apps/rux/qt/src/EdgeListPanel.cpp`
- Modify: `apps/rux/qt/src/EditWorkspace.cpp` (edges pane content, wiring), `apps/rux/qt/include/rux_qt/EditWorkspace.hpp` (accessor), `apps/rux/qt/styles/app.qss`, `apps/rux/qt/gallery/shell_pages.cpp`

**Interfaces:**
- Consumes: Task 6 (`edge_table`, `EdgeColumn`, `edge_column_title`, `edge_cell_text`, `edge_column_numeric`, `EdgeFilter`, `default_sort_column`), Task 10 (`GraphPanel::colour_mode_changed`, `edge_clicked`, `set_highlight`), `style_table()` (TableBrowser.hpp).
- Produces: `class EdgeListModel : public QAbstractTableModel { void set_rows(std::vector<EdgeView>); const EdgeView *row_at(int) const; int row_of(const EdgeKey &) const; }`; `class EdgeListPanel : public QWidget { explicit EdgeListPanel(EdgeEditor &, QWidget *parent = nullptr); void set_sort(EdgeColumn, bool descending); void set_filter(const EdgeFilter &); void select_edge(const EdgeKey &); int row_count() const; signals: void edge_activated(const rux::qt::EdgeKey &key); }`; `EditWorkspace::edges() const -> EdgeListPanel *`.

- [ ] **Step 1: The model and panel**

`EdgeListPanel.hpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Rediger's edge list (spec §5.2): every effective edge (staged edits
// applied) with type, source, status and the optimizer's metrics; sorted so
// the suspicious ones are on top (the colour mode's metric, descending, by
// default), filtered by type and status. Rows come from rux_qt_core's
// edge_table(); this file only adapts them to a QTableView.

#include <rux_qt/edge_table.hpp>

#include <QAbstractTableModel>
#include <QWidget>

#include <vector>

class QButtonGroup;
class QCheckBox;
class QComboBox;
class QLabel;
class QTableView;

namespace rux::qt {

class EdgeEditor;

class EdgeListModel : public QAbstractTableModel {
  Q_OBJECT
    public:
  using QAbstractTableModel::QAbstractTableModel;
  void set_rows(std::vector<EdgeView> rows);
  const EdgeView *row_at(int row) const;
  int row_of(const EdgeKey &key) const;
  int rowCount(const QModelIndex &parent = {}) const override;
  int columnCount(const QModelIndex &parent = {}) const override;
  QVariant data(const QModelIndex &index, int role) const override;
  QVariant headerData(int section, Qt::Orientation o, int role) const override;

    private:
  std::vector<EdgeView> rows_;
};

class EdgeListPanel : public QWidget {
  Q_OBJECT
    public:
  explicit EdgeListPanel(EdgeEditor &editor, QWidget *parent = nullptr);
  void set_sort(EdgeColumn column, bool descending);
  void set_filter(const EdgeFilter &filter);
  /// Select and scroll to @p key without emitting edge_activated.
  void select_edge(const EdgeKey &key);
  int row_count() const;

    signals:
  void edge_activated(const rux::qt::EdgeKey &key);

    private:
  void rebuild();
  void sync_type_combo(const std::vector<EdgeView> &all);
  EdgeEditor &editor_;
  EdgeListModel *model_ = nullptr;
  QTableView *view_ = nullptr;
  QComboBox *type_ = nullptr;
  QButtonGroup *status_ = nullptr;
  QCheckBox *rejected_ = nullptr;
  QLabel *meta_ = nullptr;
  EdgeFilter filter_;
  EdgeColumn sort_ = EdgeColumn::residual;
  bool descending_ = true;
  bool selecting_ = false;
};

} // namespace rux::qt
```

`EdgeListPanel.cpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <rux_qt/EdgeEditor.hpp>
#include <rux_qt/EdgeListPanel.hpp>
#include <rux_qt/TableBrowser.hpp>
#include <rux_qt/Theme.hpp>
#include <rux_qt/widgets.hpp>

#include <QButtonGroup>
#include <QCheckBox>
#include <QComboBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QPushButton>
#include <QTableView>
#include <QVBoxLayout>

#include <set>

namespace rux::qt {

void EdgeListModel::set_rows(std::vector<EdgeView> rows) {
  beginResetModel();
  rows_ = std::move(rows);
  endResetModel();
}

const EdgeView *EdgeListModel::row_at(int row) const {
  return row >= 0 && row < static_cast<int>(rows_.size())
             ? &rows_[static_cast<std::size_t>(row)]
             : nullptr;
}

int EdgeListModel::row_of(const EdgeKey &key) const {
  for (std::size_t i = 0; i < rows_.size(); ++i)
    if (rows_[i].edge.key == key)
      return static_cast<int>(i);
  return -1;
}

int EdgeListModel::rowCount(const QModelIndex &parent) const {
  return parent.isValid() ? 0 : static_cast<int>(rows_.size());
}

int EdgeListModel::columnCount(const QModelIndex &parent) const {
  return parent.isValid() ? 0 : kEdgeColumnCount;
}

QVariant EdgeListModel::data(const QModelIndex &index, int role) const {
  const EdgeView *v = row_at(index.row());
  if (!v)
    return {};
  const auto c = static_cast<EdgeColumn>(index.column());
  switch (role) {
  case Qt::DisplayRole:
    return QString::fromStdString(edge_cell_text(*v, c));
  case Qt::TextAlignmentRole:
    return QVariant::fromValue(Qt::AlignVCenter |
                               (edge_column_numeric(c) ? Qt::AlignRight
                                                       : Qt::AlignLeft));
  case Qt::FontRole:
    if (edge_column_numeric(c) || c == EdgeColumn::pair)
      return theme().font(FontRole::mono, "--font-size-sm",
                          "--font-weight-regular");
    return {};
  case Qt::ForegroundRole:
    if (v->edge.status == "rejected")
      return theme().color("--color-text-faint");
    if (v->pending != EdgeView::Pending::none)
      return theme().color("--color-accent-deep");
    return {};
  default:
    return {};
  }
}

QVariant EdgeListModel::headerData(int section, Qt::Orientation o,
                                   int role) const {
  if (o != Qt::Horizontal || role != Qt::DisplayRole)
    return {};
  return QString::fromStdString(edge_column_title(static_cast<EdgeColumn>(section)));
}

EdgeListPanel::EdgeListPanel(EdgeEditor &editor, QWidget *parent)
    : QWidget(parent), editor_(editor) {
  setObjectName("edgeList");
  const Theme &t = theme();
  auto *v = new QVBoxLayout(this);
  v->setContentsMargins(0, 0, 0, 0);
  v->setSpacing(0);

  auto *bar = new QFrame;
  bar->setObjectName("filterBar");
  auto *bl = new QHBoxLayout(bar);
  bl->setContentsMargins(t.px("--space-3"), t.px("--space-1"),
                         t.px("--space-3"), t.px("--space-1"));
  bl->setSpacing(t.px("--space-2"));
  type_ = new QComboBox;
  type_->setObjectName("edgeTypeFilter");
  type_->addItem("Alle typer", QString());
  bl->addWidget(type_);
  auto *seg = new QFrame;
  seg->setObjectName("segmented");
  auto *sl = new QHBoxLayout(seg);
  sl->setContentsMargins(0, 0, 0, 0);
  sl->setSpacing(0);
  status_ = new QButtonGroup(this);
  const std::array<std::pair<const char *, const char *>, 4> statuses = {
      {{"Alle", ""}, {"Auto", "auto"}, {"Bekræftet", "confirmed"},
       {"Afvist", "rejected"}}};
  for (int i = 0; i < 4; ++i) {
    auto *b = new QPushButton(statuses[i].first);
    b->setObjectName("segment");
    b->setCheckable(true);
    b->setCursor(Qt::PointingHandCursor);
    b->setProperty("position", i == 0 ? "first" : i == 3 ? "last" : "middle");
    b->setProperty("status", statuses[i].second);
    status_->addButton(b, i);
    sl->addWidget(b);
  }
  status_->button(0)->setChecked(true);
  bl->addWidget(seg);
  rejected_ = new QCheckBox("Med afviste");
  bl->addWidget(rejected_);
  meta_ = new QLabel;
  meta_->setObjectName("toolbarMeta");
  bl->addWidget(meta_, 1, Qt::AlignRight);
  v->addWidget(bar);

  model_ = new EdgeListModel(this);
  view_ = new QTableView;
  style_table(view_);
  view_->setModel(model_);
  auto *h = view_->horizontalHeader();
  h->setSectionsClickable(true);
  h->setSortIndicatorShown(true);
  h->setSortIndicator(static_cast<int>(sort_), Qt::DescendingOrder);
  h->setStretchLastSection(true);
  v->addWidget(view_, 1);

  connect(h, &QHeaderView::sortIndicatorChanged, this,
          [this](int section, Qt::SortOrder order) {
            sort_ = static_cast<EdgeColumn>(section);
            descending_ = order == Qt::DescendingOrder;
            rebuild();
          });
  connect(type_, &QComboBox::currentIndexChanged, this, [this] {
    filter_.type = type_->currentData().toString().toStdString();
    rebuild();
  });
  connect(status_, &QButtonGroup::idClicked, this, [this](int id) {
    filter_.status =
        status_->button(id)->property("status").toString().toStdString();
    rebuild();
  });
  connect(rejected_, &QCheckBox::toggled, this, [this](bool on) {
    filter_.show_rejected = on;
    rebuild();
  });
  connect(view_->selectionModel(), &QItemSelectionModel::currentRowChanged,
          this, [this](const QModelIndex &cur) {
            if (selecting_)
              return;
            if (const EdgeView *v = model_->row_at(cur.row()))
              emit edge_activated(v->edge.key);
          });
  connect(&editor_, &EdgeEditor::changed, this, &EdgeListPanel::rebuild);
  rebuild();
}

void EdgeListPanel::set_sort(EdgeColumn column, bool descending) {
  view_->horizontalHeader()->setSortIndicator(
      static_cast<int>(column),
      descending ? Qt::DescendingOrder : Qt::AscendingOrder); // -> rebuild()
}

void EdgeListPanel::set_filter(const EdgeFilter &f) {
  filter_ = f;
  rebuild();
}

void EdgeListPanel::select_edge(const EdgeKey &key) {
  const int r = model_->row_of(key);
  if (r < 0)
    return;
  selecting_ = true;
  view_->selectRow(r);
  view_->scrollTo(model_->index(r, 0), QAbstractItemView::PositionAtCenter);
  selecting_ = false;
}

int EdgeListPanel::row_count() const { return model_->rowCount(); }

void EdgeListPanel::sync_type_combo(const std::vector<EdgeView> &all) {
  std::set<std::string> types;
  for (const auto &v : all)
    types.insert(v.edge.key.type);
  const QString current = type_->currentData().toString();
  const QSignalBlocker b(type_);
  type_->clear();
  type_->addItem("Alle typer", QString());
  for (const auto &ty : types)
    type_->addItem(QString::fromStdString(edge_type_da(ty)),
                   QString::fromStdString(ty));
  type_->setCurrentIndex(std::max(0, type_->findData(current)));
}

void EdgeListPanel::rebuild() {
  // Keep the selected key across a rebuild (a staged edit re-sorts).
  std::optional<EdgeKey> keep;
  if (const EdgeView *v = model_->row_at(view_->currentIndex().row()))
    keep = v->edge.key;
  const auto all = editor_.edits().effective();
  sync_type_combo(all);
  model_->set_rows(edge_table(all, filter_, sort_, descending_));
  meta_->setText(QString("%1 af %2 kanter")
                     .arg(format_count(model_->rowCount()))
                     .arg(format_count(all.size())));
  if (keep)
    select_edge(*keep);
}

} // namespace rux::qt
```

- [ ] **Step 2: Wire it into Rediger**

In `EditWorkspace`: member `EdgeListPanel *edges_ = nullptr;`, accessor `EdgeListPanel *edges() const { return edges_; }`; the edges pane becomes `edges_ = new EdgeListPanel(editor_); panes_[1]->set_content(edges_);`; then:

```cpp
  connect(edges_, &EdgeListPanel::edge_activated, this,
          [this](const EdgeKey &k) {
            graph_->set_highlight(k);
            graph_->click_edge(k.from, k.to); // inspector + pair_requested
          });
  connect(graph_, &GraphPanel::edge_clicked, this, [this](const EdgeKey &k) {
    graph_->set_highlight(k);
    edges_->select_edge(k);
  });
  connect(graph_, &GraphPanel::colour_mode_changed, this,
          [this](EdgeColourMode m) { edges_->set_sort(default_sort_column(m), true); });
  edges_->set_sort(default_sort_column(graph_->colour_mode()), true);
```

`QSS`: `QFrame#filterBar` exists (Log workspace) — reuse; nothing new unless the shot says otherwise.

- [ ] **Step 3: Gallery page**

```cpp
/// The edge list sorted by residual with a row selected: the graph
/// highlights it and A/B follow.
QWidget *make_rediger_list(const PageContext &ctx) {
  auto *shell = static_cast<AppShell *>(make_rediger_colour(ctx, EdgeColourMode::residual));
  EdgeListPanel *list = shell->edit()->edges();
  if (list->row_count() > 0)
    for (auto *v : list->findChildren<QTableView *>())
      v->selectRow(0); // -> edge_activated
  return shell;
}
```

register `rediger-list`.

- [ ] **Step 4: Build, shoot, review**

Run: `cmake --build build --target rux rux-qt-gallery && bash .claude/skills/design-studio/scripts/qt_shot.sh --page rediger-list --page rediger-deviation --theme both`
Expected: exit 0. Read the PNGs: the list's first row has the largest residual, its edge is highlighted in the graph, the pair header shows its ends; numbers mono right-aligned; "—" for the legacy rows at the bottom; headers upper-case; "N af M kanter". Fix the 3–5 worst issues (column widths: give PAR/TYPE/KILDE `ResizeToContents`), re-shoot, `token_lint.py` clean.

- [ ] **Step 5: Commit**

```bash
git add apps/rux/qt
git commit -m "qt: Rediger edge list (sortable, filterable) linked both ways with the graph"
```

---

### Task 12: "Find kanter" and "Optimer" from the Rediger toolbar

Both run through the Pipeline workspace's `JobRunner` (one runner, one writer lease); the graph, list and colours reload when the job ends.

**Files:**
- Modify: `apps/rux/qt/include/rux_qt/PipelineWorkspace.hpp`, `apps/rux/qt/src/PipelineWorkspace.cpp:455-500,742-760` (`run_stage`, row command text, pending prompt)
- Modify: `apps/rux/qt/src/pipeline_ui.cpp` (graph_discover cases)
- Modify: `apps/rux/qt/include/rux_qt/EditWorkspace.hpp`, `apps/rux/qt/src/EditWorkspace.cpp` (buttons, running state)
- Modify: `apps/rux/qt/src/AppShell.cpp` (wiring), `apps/rux/qt/CMakeLists.txt` (gallery links `reusex_slam reusex_pipeline`), `apps/rux/qt/gallery/shell_pages.cpp`

**Interfaces:**
- Consumes: assumed `JobStage::graph_discover`, `build_cli_command("graph_discover", …)`, `stage_executor_with_optimize()`.
- Produces: `bool PipelineWorkspace::run_stage(reusex::pipeline::JobStage stage)`; `EditWorkspace` signal `void run_requested(reusex::pipeline::JobStage stage)`, slots `void set_job_running(const QString &stage_da)` / `void set_job_finished()`.

- [ ] **Step 1: Stage names (skip any case P3 already added)**

`pipeline_ui.cpp`: in `stage_name_da` add `case pl::JobStage::graph_discover: return "Find kanter";`; in `stage_blurb_da` `case pl::JobStage::graph_discover: return "Find nye løkkelukninger (ORB, panoramaer, fil) fra de oprindelige poser og læg dem i posegrafen. Afviste kanter genopstår ikke.";`; in `contract_stage` `case pl::JobStage::graph_discover: return P::graph_discover;`. If `annotate` lacks cases there too, leave it (it is hidden).

- [ ] **Step 2: Row command text from the CLI table**

In `PipelineWorkspace::rebuild_stages()` replace the `optimize ? … : create …` ternary with:

```cpp
    const CliCommand words = build_cli_command(name, "", {});
    QStringList parts;
    for (const auto &w : words.args)
      parts << QString::fromStdString(w);
    auto *c = new QLabel("rux " + parts.join(' '));
```

(`name` is the loop's stage name; the args of an empty project are just the subcommand words: `create planes`, `optimize`, `graph discover`).

- [ ] **Step 3: `run_stage` and the pending prompt for both graph stages**

```cpp
bool PipelineWorkspace::run_stage(pl::JobStage stage) {
  select_stage(stage);
  return run();
}
```

In `run()` change the guard to `if ((stage_ == pl::JobStage::optimize || stage_ == pl::JobStage::graph_discover) && pending_edits_ && pending_edits_() > 0)` and the informative text to `QString("%1 bruger kun de gemte kanter. Gem dem i Rediger først, eller kør uden dem.").arg(stage_name_da(stage_))`.

- [ ] **Step 4: Toolbar buttons in Rediger**

In `EditWorkspace::make_toolbar()`, before "Paneler":

```cpp
  discover_ = new QPushButton(NavItem::escape_mnemonic("Find kanter"));
  discover_->setProperty("kind", "secondary");
  discover_->setCursor(Qt::PointingHandCursor);
  discover_->setToolTip("rux graph discover: nye løkkelukninger fra de "
                        "oprindelige poser (afviste kanter springes over)");
  optimize_ = new QPushButton(NavItem::escape_mnemonic("Optimer"));
  optimize_->setProperty("kind", "primary");
  optimize_->setCursor(Qt::PointingHandCursor);
  optimize_->setToolTip("rux optimize: løs den gemte posegraf");
  for (QPushButton *b : {discover_, optimize_}) {
    b->setMinimumHeight(b->sizeHint().height());
    bl->addWidget(b);
  }
  connect(discover_, &QPushButton::clicked, this,
          [this] { emit run_requested(reusex::pipeline::JobStage::graph_discover); });
  connect(optimize_, &QPushButton::clicked, this,
          [this] { emit run_requested(reusex::pipeline::JobStage::optimize); });
```

members `QPushButton *discover_ = nullptr, *optimize_ = nullptr; QString running_;`, and

```cpp
void EditWorkspace::set_job_running(const QString &stage_da) {
  running_ = stage_da;
  discover_->setEnabled(false);
  optimize_->setEnabled(false);
  meta_->setText(QString("%1 kører … — grafen opdateres, når trinnet er færdigt.")
                     .arg(stage_da));
}

void EditWorkspace::set_job_finished() {
  running_.clear();
  discover_->setEnabled(session_.is_open());
  optimize_->setEnabled(session_.is_open());
  meta_->clear();
}
```

(`#include <reusex/pipeline/stages.hpp>` for `JobStage`.)

- [ ] **Step 5: Shell wiring**

In `AppShell.cpp`:

```cpp
  connect(edit_, &EditWorkspace::run_requested, this,
          [this](reusex::pipeline::JobStage s) { pipeline_->run_stage(s); });
  connect(pipeline_, &PipelineWorkspace::job_started, edit_,
          &EditWorkspace::set_job_running);
  connect(pipeline_, &PipelineWorkspace::job_finished, edit_,
          &EditWorkspace::set_job_finished);
```

The existing `project_changed` handler re-reads the session when nothing is pending; `EdgeEditor::reload` (on `opened`) and `GraphPanel::reset` (on `state_changed`) then refresh the edges, list and colours. Check by the shot in Step 7, not by assumption.

- [ ] **Step 6: The gallery runs the real executor**

`apps/rux/qt/CMakeLists.txt`: `target_link_libraries(rux-qt-gallery PRIVATE rux_qt_lib reusex_slam reusex_pipeline ${VTK_LIBRARIES})`. In `shell_pages.cpp` `make_shell`: `options.stage_executor = reusex::pipeline::stage_executor_with_optimize();` (`#include <reusex/pipeline/optimize_stage.hpp>`). Add:

```cpp
/// "Optimer" from the Rediger toolbar on the copy (WRITES poses and edge
/// metrics), shown coloured by residual once it has finished.
QWidget *make_rediger_optimize(const PageContext &ctx) {
  auto *shell = make_pg(ctx);
  settle();
  for (auto *b : shell->edit()->findChildren<QPushButton *>())
    if (b->text() == "Optimer")
      b->click();
  settle(600000);
  shell->edit()->graph()->set_colour_mode(EdgeColourMode::residual);
  return shell;
}
```

registered as `rediger-optimize`. (On the office_corridor fixture `optimize` initialises the graph itself — P2, spec §4.3 step 1.)

- [ ] **Step 7: Build, shoot, review**

Run: `cmake --build build --target rux rux-qt-gallery && bash .claude/skills/design-studio/scripts/qt_shot.sh --page rediger-optimize --page pipeline --theme both`
Expected: exit 0. Read: Rediger after the run shows odometry edges coloured on the residual ramp with real counts in the legend and the list sorted by residual; Pipeline's stage list shows "Find kanter · rux graph discover" and no annotate. Run the app smoke with `RUX_QT_PAGE=rediger`. `token_lint.py` clean.

- [ ] **Step 8: Commit**

```bash
git add apps/rux/qt
git commit -m "qt: Find kanter / Optimer in Rediger via the pipeline runner"
```

---

### Task 13: The view yaw in the 3D workspace and `rux render`

**Files:**
- Modify: `libs/reusex/include/visualize/scene.hpp:192-200` (`CameraFraming::yaw_deg`), `libs/reusex/src/visualize/scene.cpp:965-1023` (`place_preset_camera`)
- Modify: `libs/reusex/include/visualize/render_view.hpp` (`RenderOptions::yaw_deg`), `libs/reusex/src/visualize/render_view.cpp:270-281`
- Test: `tests/unit/visualize/test_scene.cpp`
- Modify: `apps/rux/qt/include/rux_qt/Viewer3DWorkspace.hpp`, `apps/rux/qt/src/Viewer3DWorkspace.cpp:658-691` (ctor takes `ViewSettings &`, `frame_camera`), `apps/rux/qt/src/AppShell.cpp`

**Interfaces:**
- Consumes: `ProjectDB::setting`, `reusex::utils::parse_number`.
- Produces: `CameraFraming::yaw_deg` (double, default 0); `RenderOptions::yaw_deg` (`std::optional<double>`; nullopt = the project's `view.yaw_deg`, 0 when unset); `Viewer3DWorkspace(ProjectSession &, ViewSettings &, bool interactive, QWidget *parent = nullptr)`.

- [ ] **Step 1: Write the failing camera test**

Append to `tests/unit/visualize/test_scene.cpp` (add `#include <vtkCamera.h> <vtkNew.h> <vtkRenderer.h>` if missing):

```cpp
TEST_CASE("PlacePresetCamera_Yaw_TurnsTheViewAboutZ",
          "[visualize][scene][camera]") {
  using namespace reusex::visualize;
  SceneBounds b;
  b.add(0, 0, 0);
  b.add(10, 2, 3); // long in x
  auto up_for = [&](ViewPreset view, double yaw) {
    vtkNew<vtkRenderer> r;
    CameraFraming f;
    f.view = view;
    f.yaw_deg = yaw;
    place_preset_camera(r, f, b);
    double up[3];
    r->GetActiveCamera()->GetViewUp(up);
    return std::array<double, 3>{up[0], up[1], up[2]};
  };
  const auto up0 = up_for(ViewPreset::top, 0.0);
  CHECK(up0[1] == Catch::Approx(1.0));
  // +90°: the model turns counter-clockwise, so world +x points up the page.
  const auto up90 = up_for(ViewPreset::top, 90.0);
  CHECK(up90[0] == Catch::Approx(1.0));
  CHECK(up90[1] == Catch::Approx(0.0).margin(1e-9));

  vtkNew<vtkRenderer> r;
  CameraFraming f;
  f.view = ViewPreset::top;
  f.aspect = 1.0;
  const double s0 = [&] { place_preset_camera(r, f, b); return r->GetActiveCamera()->GetParallelScale(); }();
  f.yaw_deg = 90.0;
  place_preset_camera(r, f, b);
  // The 10 m side now runs up the page: a square viewport frames the same
  // extent, so the scale is unchanged; with a wide aspect it would grow.
  CHECK(r->GetActiveCamera()->GetParallelScale() == Catch::Approx(s0));
}
```

- [ ] **Step 2: Run to verify it fails**

Run: `cmake --build build --target reusex_unit_tests_vision`
Expected: compile error (`CameraFraming` has no member `yaw_deg`).

- [ ] **Step 3: Implement in the library**

`scene.hpp`, in `CameraFraming`: `/// Display rotation about world z, degrees, counter-clockwise seen from above (project_settings view.yaw_deg). Display only. double yaw_deg = 0.0;`. In `place_preset_camera` (`scene.cpp`), at the top:

```cpp
  const double yaw = framing.yaw_deg * std::numbers::pi / 180.0;
  const double cy = std::cos(yaw), sy = std::sin(yaw);
  // The camera turns by -yaw so the model appears turned by +yaw. Extents in
  // the turned frame decide the framing.
  const double ex = std::abs(cy) * bounds.extent(0) + std::abs(sy) * bounds.extent(1);
  const double ey = std::abs(sy) * bounds.extent(0) + std::abs(cy) * bounds.extent(1);
```

`top`/`plan`: `cam->SetViewUp(sy, cy, 0.0);` and `parallel_scale_for(ex, ey, aspect, framing.margin)`. `front`: `cam->SetPosition(center[0] - reach * sy, center[1] - reach * cy, center[2]);` and `parallel_scale_for(ex, bounds.extent(2), …)`. `orbit`: `const double azimuth = 2.0 * std::numbers::pi * index / count - yaw;`.

`render_view.hpp`, in `RenderOptions`: `/// View yaw in degrees; nullopt = the project's view.yaw_deg (0 when unset). std::optional<double> yaw_deg;`. `render_view.cpp`, where `framing` is filled:

```cpp
    double yaw = 0.0;
    if (opts.yaw_deg) {
      yaw = *opts.yaw_deg;
    } else if (const auto text = db.setting("view.yaw_deg")) {
      yaw = utils::parse_number<double>(*text).value_or(0.0);
    }
    framing.yaw_deg = yaw;
```

(`#include <reusex/utils/parse_number.hpp>`).

- [ ] **Step 4: Run the tests**

Run: `cmake --build build --target reusex_unit_tests_vision && build/tests/reusex_unit_tests_vision "[camera]"`
Expected: PASS. Then `build/tests/reusex_unit_tests_vision "[visualize]"` — no regressions (yaw 0 is the old camera).

- [ ] **Step 5: The 3D workspace follows the yaw**

`Viewer3DWorkspace`: constructor gains `ViewSettings &view_settings` (member `view_settings_`), `AppShell` passes `*view_settings_`. In `frame_camera()` add `f.yaw_deg = view_settings_.yaw_deg();` before `place_preset_camera`. In the constructor: `connect(&view_settings_, &ViewSettings::yaw_changed, this, [this] { if (camera_placed_ && !camera_moved_) frame_camera(); });`. Gallery `make_3d_plan`: after creating, `shell->view_settings().set_yaw_deg(30.0);` is NOT added (it writes the copy) — add a new page `3d-yaw` that does exactly that and shows the plan.

- [ ] **Step 6: Verify render and shots**

Run: `cmake --build build --target rux rux-qt-gallery && COPY=$(mktemp --suffix=.rux) && cp ~/.cache/reusex/qt-shot/office_corridor-*.rux "$COPY" && build/apps/rux/rux -p "$COPY" info >/dev/null && sqlite3 "$COPY" "INSERT OR REPLACE INTO project_settings(key,value) VALUES('view.yaw_deg','30.0')" && build/apps/rux/rux -p "$COPY" render -o /tmp/yaw30.png --view plan --layers cloud`
Expected: exit 0; Read `/tmp/yaw30.png`: the plan is turned 30° counter-clockwise versus the same command on a copy without the row.
Run: `bash .claude/skills/design-studio/scripts/qt_shot.sh --page 3d-yaw --page 3d-plan --theme both` → exit 0; Read: `3d-yaw` turned, `3d-plan` unchanged.

- [ ] **Step 7: Commit**

```bash
git add libs/reusex/include/visualize libs/reusex/src/visualize tests/unit/visualize apps/rux/qt
git commit -m "visualize: preset cameras honour the project's view yaw (3D workspace, rux render)"
```

---

### Task 14: P4 documentation and the full review pass

**Files:**
- Modify: `docs/DIRECTION.md` (changelog), `CLAUDE.md` (Qt client paragraph: `RUX_QT_PAGE`, Posegraf → Rediger, `EditWorkspace`), `.claude/skills/design-studio/references/qt-client.md` (§1 file list, gallery page names), `.claude/skills/design-studio/SKILL.md` (description: "Posegraf" → "Rediger"), `docs/CONTRACTS.md` only if P3 left the `view.yaw_deg` setting undocumented

- [ ] **Step 1: Docs**

`docs/DIRECTION.md`, top of "Direction changelog":

```markdown
- **2026-10-13** — **Qt client: Posegraf becomes Rediger** ([spec](superpowers/specs/2026-10-13-pose-graph-editor-design.md), P4).
  The rail is Start · Oversigt · 3D · Rediger · Pipeline · Log. Oversigt
  (formerly Database) is read-only; every pose-graph edit happens in Rediger:
  graph in XY/XZ/YZ with a project-wide view yaw (`project_settings`
  `view.yaw_deg`, also used by 3D and `rux render`), edges coloured by type,
  status, residual (fixed χ² scale), deviation or seed disagreement, a
  sortable edge list, and Find kanter / Optimer through the pipeline runner.
  A user "delete" is now a rejection that later discover/optimize runs keep.
```

`CLAUDE.md`: in the Architecture → Executables paragraph change `RUX_QT_PAGE=database|3d|posegraf|pipeline|log` to `RUX_QT_PAGE=oversigt|3d|rediger|pipeline|log` (`database` still accepted), and mention "Rediger (`EditWorkspace`: `GraphPanel`, `EdgeListPanel`, pair strip)" where the Qt workspaces are listed. `qt-client.md`: replace `PoseGraphWorkspace.hpp` in the tree with `EditWorkspace.hpp`, `GraphPanel.hpp`, `EdgeListPanel.hpp`, `PendingBanner.hpp`, `PairStrip.hpp`, `ViewSettings.hpp`, and the Qt-free `graph_view.hpp`, `edge_colouring.hpp`, `edge_table.hpp`; the Q3 pages paragraph lists `rediger*` instead of `posegraf*` and `db-pending|db-readonly|db-saved` move to `rediger-pending|-readonly|-saved`.

- [ ] **Step 2: Full shot pass**

Run: `bash .claude/skills/design-studio/scripts/qt_shot.sh --all`
Expected: exit 0 (no missing token, no missing font). Read every `rediger*`, `db-*`, `3d*`, `shell-*` PNG in both themes and run the critique checklist once more; fix what is a defect.

- [ ] **Step 3: Full light suite and lint**

Run: `cmake --build build --target reusex_unit_tests reusex_unit_tests_vision && build/tests/reusex_unit_tests "[rux_qt],[icp]" && build/tests/reusex_unit_tests_vision "[visualize]"`
Expected: all pass. Then the whole suite once: `ctest --test-dir build --output-on-failure --parallel $(nproc)` — no new failures against `main`. `python .claude/skills/design-studio/scripts/token_lint.py apps/rux/qt --qt` → clean. `reuse lint` → compliant.

- [ ] **Step 4: Commit**

```bash
git add docs/DIRECTION.md CLAUDE.md .claude/skills/design-studio
git commit -m "docs: Posegraf -> Rediger (direction changelog, CLAUDE.md, qt-client reference)"
```

---
# Phase P5 — Pair panel

Convention used by every P5 task (spec §3.1, assumption 4): an edge A→B stores `T_AB = T_A⁻¹ · T_B` in the `sensor_frames.transform` (pose-frame) convention. The manual offset `ΔT` is a **world-space** rigid transform applied to B's cloud, rotating about B's cloud centroid (the pivot). The measurement of a manually aligned pair is `T_A⁻¹ · ΔT · T_B`; ICP is run with `from = B, to = A, initial_guess = ΔT`, and its `world_delta` (guess included) becomes the new `ΔT`, so the same formula yields the ICP-refined measurement.

### Task 15: Pair alignment math (Qt-free)

**Files:**
- Create: `apps/rux/qt/include/rux_qt/pair_alignment.hpp`, `apps/rux/qt/src/core/pair_alignment.cpp`, `tests/unit/rux_qt/test_pair_alignment.cpp`
- Modify: `apps/rux/qt/CMakeLists.txt`

**Interfaces:**
- Consumes: `EdgeRecord`, `EdgeView`, `weight_from_icp_fitness` (database_logic).
- Produces: `using Mat4 = std::array<double, 16>; using Vec3 = std::array<double, 3>;` `Mat4 mat4_identity()`, `Mat4 mat4_mul(const Mat4 &, const Mat4 &)`, `Mat4 rigid_inverse(const Mat4 &)`, `Vec3 transform_point(const Mat4 &, const Vec3 &)`, `struct ManualOffset { double x, y, z, yaw_deg, pitch_deg, roll_deg; bool is_zero() const; }`, `Mat4 offset_matrix(const ManualOffset &, const Vec3 &pivot)`, `ManualOffset offset_from_matrix(const Mat4 &, const Vec3 &pivot)`, `enum class NudgeKey { x_minus, x_plus, y_minus, y_plus, z_minus, z_plus, yaw_minus, yaw_plus }`, `ManualOffset nudge(ManualOffset, NudgeKey, bool coarse)`, `kNudgeM = 0.01`, `kNudgeDeg = 0.5`, `kNudgeCoarse = 10.0`, `Mat4 edge_measurement(const Mat4 &pose_from, const Mat4 &pose_to, const Mat4 &world_delta_on_to)`, `std::array<double, 36> icp_information(double rms_m)`, `EdgeRecord oriented_edge(const EdgeRecord &a_to_b, const std::vector<EdgeView> &between_a_b)`.

- [ ] **Step 1: Write the failing tests**

`tests/unit/rux_qt/test_pair_alignment.cpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

// The pair panel's math (spec §5.2): B's manual offset about its centroid,
// keyboard nudges, the edge measurement T_A^-1 · ΔT · T_B, the information
// an ICP fit earns, and re-measuring an edge stored the other way round.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <rux_qt/pair_alignment.hpp>

using namespace rux::qt;
using Catch::Approx;

namespace {
Mat4 translate(double x, double y, double z) {
  return {1, 0, 0, x, 0, 1, 0, y, 0, 0, 1, z, 0, 0, 0, 1};
}
void check_mat(const Mat4 &a, const Mat4 &b) {
  for (std::size_t i = 0; i < 16; ++i) {
    INFO(i);
    CHECK(a[i] == Approx(b[i]).margin(1e-9));
  }
}
} // namespace

TEST_CASE("ManualOffset_RoundTripsThroughTheMatrix", "[rux_qt][pair]") {
  const ManualOffset o{0.2, -0.1, 0.05, 10.0, -5.0, 3.0};
  const Vec3 pivot{1.0, 2.0, 0.5};
  const ManualOffset back = offset_from_matrix(offset_matrix(o, pivot), pivot);
  CHECK(back.x == Approx(o.x));
  CHECK(back.y == Approx(o.y));
  CHECK(back.z == Approx(o.z));
  CHECK(back.yaw_deg == Approx(o.yaw_deg));
  CHECK(back.pitch_deg == Approx(o.pitch_deg));
  CHECK(back.roll_deg == Approx(o.roll_deg));
}

TEST_CASE("ManualOffset_RotatesAboutThePivot", "[rux_qt][pair]") {
  const Vec3 pivot{3.0, -1.0, 1.2};
  const Mat4 m = offset_matrix({0, 0, 0, 30.0, 0, 0}, pivot);
  const Vec3 p = transform_point(m, pivot);
  CHECK(p[0] == Approx(pivot[0]));
  CHECK(p[1] == Approx(pivot[1]));
  CHECK(p[2] == Approx(pivot[2]));
  check_mat(offset_matrix({}, pivot), mat4_identity());
  CHECK(ManualOffset{}.is_zero());
  CHECK_FALSE(ManualOffset{0, 0, 0, 0.1, 0, 0}.is_zero());
}

TEST_CASE("Nudge_FineAndCoarseSteps", "[rux_qt][pair]") {
  CHECK(nudge({}, NudgeKey::x_plus, false).x == Approx(kNudgeM));
  CHECK(nudge({}, NudgeKey::y_minus, true).y == Approx(-kNudgeM * kNudgeCoarse));
  CHECK(nudge({}, NudgeKey::z_plus, false).z == Approx(kNudgeM));
  CHECK(nudge({}, NudgeKey::yaw_plus, true).yaw_deg ==
        Approx(kNudgeDeg * kNudgeCoarse));
}

TEST_CASE("EdgeMeasurement_IsTFromInverseDeltaTTo", "[rux_qt][pair]") {
  const Mat4 a = translate(1, 0, 0), b = translate(3, 0, 0);
  CHECK(edge_measurement(a, b, mat4_identity())[3] == Approx(2.0));
  CHECK(edge_measurement(a, b, translate(0.5, 0, 0))[3] == Approx(2.5));
  const Mat4 d = offset_matrix({0.1, 0.2, 0, 20, 0, 0}, {3, 0, 0});
  check_mat(edge_measurement(a, b, d),
            mat4_mul(mat4_mul(rigid_inverse(a), d), b));
  check_mat(mat4_mul(rigid_inverse(d), d), mat4_identity());
}

TEST_CASE("IcpInformation_IsInverseVarianceWithAFloor", "[rux_qt][pair]") {
  const auto info = icp_information(0.002); // below the 1 cm floor
  CHECK(info[0] == Approx(1.0 / (0.01 * 0.01)));
  CHECK(info[35] == Approx(1.0 / (0.01 * 0.01)));
  CHECK(info[1] == 0.0);
  CHECK(icp_information(0.05)[21] == Approx(400.0));
  CHECK(icp_information(0.05)[21] == Approx(weight_from_icp_fitness(0.05)));
}

TEST_CASE("OrientedEdge_ReversedStoredEdge_IsReMeasuredInverted",
          "[rux_qt][pair]") {
  EdgeRecord ab{{1, 2, "loop_closure"}, 0.0, 9.0};
  ab.transform = translate(2, 0, 0);
  EdgeView stored;
  stored.edge = {{2, 1, "loop_closure"}, 0.0, 4.0};
  stored.reversed = true;
  const EdgeRecord r = oriented_edge(ab, {stored});
  CHECK(r.key == EdgeKey{2, 1, "loop_closure"});
  REQUIRE(r.transform.has_value());
  CHECK((*r.transform)[3] == Approx(-2.0));
  // Another type, or a forward edge present: unchanged.
  stored.edge.key.type = "odometry";
  CHECK(oriented_edge(ab, {stored}).key == EdgeKey{1, 2, "loop_closure"});
  EdgeView fwd;
  fwd.edge = {{1, 2, "loop_closure"}, 0.0, 4.0};
  stored.edge.key.type = "loop_closure";
  CHECK(oriented_edge(ab, {stored, fwd}).key == EdgeKey{1, 2, "loop_closure"});
}
```

- [ ] **Step 2: Run to verify it fails**

Run: `cmake --build build --target reusex_unit_tests`
Expected: compile error (`rux_qt/pair_alignment.hpp` missing).

- [ ] **Step 3: Implement**

`pair_alignment.hpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// The pair panel's math, Qt-free (spec §5.2). Row-major 4x4 rigid
// transforms. B's manual offset ΔT is a WORLD-space transform applied to B's
// cloud, rotating about @p pivot (B's centroid) so the cloud turns in place;
// R = Rz(yaw) · Ry(pitch) · Rx(roll). An edge A -> B measured with ΔT is
// T_A^-1 · ΔT · T_B (the store's convention); ICP from B onto A seeded with
// ΔT returns a world_delta that replaces ΔT.

#include <rux_qt/database_logic.hpp>

#include <array>
#include <vector>

namespace rux::qt {

using Mat4 = std::array<double, 16>;
using Vec3 = std::array<double, 3>;

Mat4 mat4_identity();
Mat4 mat4_mul(const Mat4 &a, const Mat4 &b);
/// Inverse of a rigid transform (R^T, -R^T t).
Mat4 rigid_inverse(const Mat4 &m);
Vec3 transform_point(const Mat4 &m, const Vec3 &p);

struct ManualOffset {
  double x = 0.0, y = 0.0, z = 0.0;                    ///< metres, world axes
  double yaw_deg = 0.0, pitch_deg = 0.0, roll_deg = 0.0; ///< about the pivot
  bool is_zero() const;
};

Mat4 offset_matrix(const ManualOffset &o, const Vec3 &pivot);
ManualOffset offset_from_matrix(const Mat4 &m, const Vec3 &pivot);

enum class NudgeKey { x_minus, x_plus, y_minus, y_plus, z_minus, z_plus,
                      yaw_minus, yaw_plus };
inline constexpr double kNudgeM = 0.01;
inline constexpr double kNudgeDeg = 0.5;
inline constexpr double kNudgeCoarse = 10.0;
ManualOffset nudge(ManualOffset o, NudgeKey key, bool coarse);

/// T_from^-1 · @p world_delta_on_to · T_to.
Mat4 edge_measurement(const Mat4 &pose_from, const Mat4 &pose_to,
                      const Mat4 &world_delta_on_to);

/// 6x6 information (GTSAM order rx, ry, rz, tx, ty, tz) from an ICP RMS:
/// σ_t = max(rms, 1 cm) for translation, σ_r = σ_t rad for rotation (a
/// point a metre away moving by σ_t). Diagonal; tx equals
/// weight_from_icp_fitness(rms).
std::array<double, 36> icp_information(double rms_m);

/// A new A -> B edge where the pair already has the same type stored B -> A
/// (and not A -> B) re-measures THAT edge: key flipped, transform inverted.
/// Otherwise @p a_to_b unchanged.
EdgeRecord oriented_edge(const EdgeRecord &a_to_b,
                         const std::vector<EdgeView> &between_a_b);

} // namespace rux::qt
```

`pair_alignment.cpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <rux_qt/pair_alignment.hpp>

#include <algorithm>
#include <cmath>
#include <numbers>

namespace rux::qt {
namespace {
constexpr double kDeg = std::numbers::pi / 180.0;
} // namespace

Mat4 mat4_identity() { return {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}; }

Mat4 mat4_mul(const Mat4 &a, const Mat4 &b) {
  Mat4 r{};
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j) {
      double s = 0.0;
      for (int k = 0; k < 4; ++k)
        s += a[i * 4 + k] * b[k * 4 + j];
      r[i * 4 + j] = s;
    }
  return r;
}

Mat4 rigid_inverse(const Mat4 &m) {
  Mat4 r = mat4_identity();
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      r[i * 4 + j] = m[j * 4 + i];
  for (int i = 0; i < 3; ++i)
    r[i * 4 + 3] = -(r[i * 4 + 0] * m[3] + r[i * 4 + 1] * m[7] + r[i * 4 + 2] * m[11]);
  return r;
}

Vec3 transform_point(const Mat4 &m, const Vec3 &p) {
  return {m[0] * p[0] + m[1] * p[1] + m[2] * p[2] + m[3],
          m[4] * p[0] + m[5] * p[1] + m[6] * p[2] + m[7],
          m[8] * p[0] + m[9] * p[1] + m[10] * p[2] + m[11]};
}

bool ManualOffset::is_zero() const {
  constexpr double e = 1e-9;
  return std::abs(x) < e && std::abs(y) < e && std::abs(z) < e &&
         std::abs(yaw_deg) < e && std::abs(pitch_deg) < e && std::abs(roll_deg) < e;
}

Mat4 offset_matrix(const ManualOffset &o, const Vec3 &pivot) {
  const double cz = std::cos(o.yaw_deg * kDeg), sz = std::sin(o.yaw_deg * kDeg);
  const double cp = std::cos(o.pitch_deg * kDeg), sp = std::sin(o.pitch_deg * kDeg);
  const double cr = std::cos(o.roll_deg * kDeg), sr = std::sin(o.roll_deg * kDeg);
  Mat4 m = mat4_identity();
  m[0] = cz * cp; m[1] = cz * sp * sr - sz * cr; m[2] = cz * sp * cr + sz * sr;
  m[4] = sz * cp; m[5] = sz * sp * sr + cz * cr; m[6] = sz * sp * cr - cz * sr;
  m[8] = -sp;     m[9] = cp * sr;                m[10] = cp * cr;
  // p -> R (p - pivot) + pivot + t
  const Vec3 rp = transform_point(m, pivot); // R · pivot (translation still 0)
  m[3] = pivot[0] - rp[0] + o.x;
  m[7] = pivot[1] - rp[1] + o.y;
  m[11] = pivot[2] - rp[2] + o.z;
  return m;
}

ManualOffset offset_from_matrix(const Mat4 &m, const Vec3 &pivot) {
  ManualOffset o;
  o.yaw_deg = std::atan2(m[4], m[0]) / kDeg;
  o.pitch_deg = std::asin(std::clamp(-m[8], -1.0, 1.0)) / kDeg;
  o.roll_deg = std::atan2(m[9], m[10]) / kDeg;
  Mat4 r = m;
  r[3] = r[7] = r[11] = 0.0;
  const Vec3 rp = transform_point(r, pivot);
  o.x = m[3] - (pivot[0] - rp[0]);
  o.y = m[7] - (pivot[1] - rp[1]);
  o.z = m[11] - (pivot[2] - rp[2]);
  return o;
}

ManualOffset nudge(ManualOffset o, NudgeKey key, bool coarse) {
  const double f = coarse ? kNudgeCoarse : 1.0;
  switch (key) {
  case NudgeKey::x_minus: o.x -= kNudgeM * f; break;
  case NudgeKey::x_plus: o.x += kNudgeM * f; break;
  case NudgeKey::y_minus: o.y -= kNudgeM * f; break;
  case NudgeKey::y_plus: o.y += kNudgeM * f; break;
  case NudgeKey::z_minus: o.z -= kNudgeM * f; break;
  case NudgeKey::z_plus: o.z += kNudgeM * f; break;
  case NudgeKey::yaw_minus: o.yaw_deg -= kNudgeDeg * f; break;
  case NudgeKey::yaw_plus: o.yaw_deg += kNudgeDeg * f; break;
  }
  return o;
}

Mat4 edge_measurement(const Mat4 &pose_from, const Mat4 &pose_to,
                      const Mat4 &world_delta_on_to) {
  return mat4_mul(mat4_mul(rigid_inverse(pose_from), world_delta_on_to), pose_to);
}

std::array<double, 36> icp_information(double rms_m) {
  const double sigma = std::max(rms_m, 0.01);
  const double info = 1.0 / (sigma * sigma);
  std::array<double, 36> m{};
  for (int i = 0; i < 6; ++i)
    m[static_cast<std::size_t>(i * 6 + i)] = info;
  return m;
}

EdgeRecord oriented_edge(const EdgeRecord &a_to_b,
                         const std::vector<EdgeView> &between) {
  const auto &k = a_to_b.key;
  bool forward = false, backward = false;
  for (const auto &v : between) {
    if (v.edge.key.type != k.type)
      continue;
    forward |= v.edge.key.from == k.from && v.edge.key.to == k.to;
    backward |= v.edge.key.from == k.to && v.edge.key.to == k.from;
  }
  if (forward || !backward)
    return a_to_b;
  EdgeRecord r = a_to_b;
  r.key = {k.to, k.from, k.type};
  if (r.transform)
    r.transform = rigid_inverse(*r.transform);
  return r; // the diagonal information is direction-free
}

} // namespace rux::qt
```

Add `src/core/pair_alignment.cpp` to the rux_qt_core list.

- [ ] **Step 4: Run the tests**

Run: `cmake --build build --target reusex_unit_tests && build/tests/reusex_unit_tests "[pair]"`
Expected: PASS (6 cases).

- [ ] **Step 5: Commit**

```bash
git add apps/rux/qt/include/rux_qt/pair_alignment.hpp apps/rux/qt/src/core/pair_alignment.cpp apps/rux/qt/CMakeLists.txt tests/unit/rux_qt/test_pair_alignment.cpp
git commit -m "qt core: manual offset about a pivot, nudges, edge measurement, ICP information"
```

---

### Task 16: `slam::frame_world_points` — the cloud ICP aligns, for the overlay

**Files:**
- Modify: `libs/reusex/include/slam/frame_pair_icp.hpp`, `libs/reusex/src/slam/frame_pair_icp.cpp:40-96`
- Test: `tests/unit/slam/test_frame_pair_icp.cpp`

**Interfaces:**
- Produces: `std::vector<std::array<float, 3>> reusex::slam::frame_world_points(const ProjectDB &db, int node_id, const FramePairIcpOptions &opts = {})` — throws `std::runtime_error` exactly as `refine_frame_pair_icp` does for no depth / no pose / empty range.

- [ ] **Step 1: Write the failing tests** (append to `test_frame_pair_icp.cpp`)

```cpp
TEST_CASE("FrameWorldPoints_IsTheBackProjectedDepthInWorld", "[slam][icp]") {
  const TempPath project("frame_world_points");
  const auto intr = make_intr();
  {
    reusex::ProjectDB db(project.path, /*readOnly=*/false);
    db.save_sensor_frame(1, dummy_color(), flat_depth(1000), full_confidence(),
                         identity_pose(), intr, 0.0);
    db.save_sensor_frame(2, dummy_color(), flat_depth(1000), full_confidence(),
                         translate_pose(0.15), intr, 1.0);
  }
  reusex::ProjectDB db(project.path, /*readOnly=*/true);
  const auto a = reusex::slam::frame_world_points(db, 1);
  const auto b = reusex::slam::frame_world_points(db, 2);
  // pixel_step 4 over 160x120: 40 x 30 samples, all at the 1 m plane.
  REQUIRE(a.size() == 1200);
  REQUIRE(b.size() == a.size());
  for (std::size_t i = 0; i < a.size(); i += 97) {
    CHECK(a[i][2] == Catch::Approx(1.0f));
    CHECK(b[i][0] - a[i][0] == Catch::Approx(0.15f).margin(1e-5));
  }
}

TEST_CASE("FrameWorldPoints_NoDepth_Throws", "[slam][icp]") {
  const TempPath project("frame_world_points_nodepth");
  {
    reusex::ProjectDB db(project.path, /*readOnly=*/false);
    db.save_sensor_frame(1, dummy_color(), cv::Mat{}, full_confidence(),
                         identity_pose(), make_intr(), 0.0);
  }
  reusex::ProjectDB db(project.path, /*readOnly=*/true);
  CHECK_THROWS_AS(reusex::slam::frame_world_points(db, 1), std::runtime_error);
}
```

- [ ] **Step 2: Run to verify it fails**

Run: `cmake --build build --target reusex_unit_tests`
Expected: compile error (`frame_world_points` is not a member of `reusex::slam`).

- [ ] **Step 3: Implement by extracting from `depth_to_world_cloud`**

Header, after `FramePairIcpResult`:

```cpp
/// The depth image of @p node_id back-projected into WORLD space with its
/// stored pose and intrinsics: exactly the cloud refine_frame_pair_icp()
/// aligns (the Qt client's pair overlay draws it).
/// @throws std::runtime_error when the frame has no depth or no valid pose,
///         or its depth has no return inside the range.
std::vector<std::array<float, 3>>
frame_world_points(const ProjectDB &db, int node_id,
                   const FramePairIcpOptions &opts = {});
```

(`#include <vector>`). In the `.cpp`, rename the body of `depth_to_world_cloud` into `frame_world_points` (public, outside the anonymous namespace), pushing `{p[0], p[1], p[2]}` into the vector instead of `cloud->emplace_back(...)`, with the same throws; `depth_to_world_cloud` becomes:

```cpp
CloudT::Ptr depth_to_world_cloud(const ProjectDB &db, int node_id,
                                 const FramePairIcpOptions &o) {
  const auto pts = frame_world_points(db, node_id, o);
  auto cloud = std::make_shared<CloudT>();
  cloud->reserve(pts.size());
  for (const auto &p : pts)
    cloud->emplace_back(p[0], p[1], p[2]);
  cloud->is_dense = true;
  cloud->width = static_cast<uint32_t>(cloud->size());
  cloud->height = 1;
  return cloud;
}
```

(the anonymous-namespace function now needs a forward declaration of `frame_world_points` above it, or move it below).

- [ ] **Step 4: Run the tests**

Run: `cmake --build build --target reusex_unit_tests && build/tests/reusex_unit_tests "[icp]"`
Expected: PASS, including the pre-existing ICP cases (unchanged numbers).

- [ ] **Step 5: Commit**

```bash
git add libs/reusex/include/slam/frame_pair_icp.hpp libs/reusex/src/slam/frame_pair_icp.cpp tests/unit/slam/test_frame_pair_icp.cpp
git commit -m "slam: expose frame_world_points (the depth cloud refine_frame_pair_icp aligns)"
```

---

### Task 17: Pair images A | B

**Files:**
- Create: `apps/rux/qt/include/rux_qt/PairImages.hpp`, `apps/rux/qt/src/PairImages.cpp`
- Modify: `apps/rux/qt/src/EditWorkspace.cpp` (images pane), `apps/rux/qt/include/rux_qt/EditWorkspace.hpp`, `apps/rux/qt/styles/app.qss`, `apps/rux/qt/gallery/shell_pages.cpp`

**Interfaces:**
- Consumes: `ImagePane`, `FrameLayer` (FrameBrowser.hpp), `FrameImageLoader`.
- Produces: `class PairImages : public QWidget { explicit PairImages(ProjectSession &, QWidget *parent = nullptr); void set_pair(int a, int b); void set_layer(FrameLayer); }`; `EditWorkspace::images() const -> PairImages *`.

- [ ] **Step 1: The widget**

`PairImages.hpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Rediger's pair images (spec §5.2): A | B side by side, colour or depth,
// decoded off the GUI thread by its own FrameImageLoader (Oversigt keeps
// its own cache; the two pages never wait on each other).

#include <rux_qt/FrameBrowser.hpp>

#include <QWidget>

class QButtonGroup;
class QLabel;

namespace rux::qt {

class FrameImageLoader;
class ProjectSession;

class PairImages : public QWidget {
  Q_OBJECT
    public:
  explicit PairImages(ProjectSession &session, QWidget *parent = nullptr);
  void set_pair(int a, int b);
  void set_layer(FrameLayer layer);

    private:
  void reload();
  void sync();
  void frame_ready(int id);
  ProjectSession &session_;
  FrameImageLoader *loader_ = nullptr;
  ImagePane *pane_a_ = nullptr;
  ImagePane *pane_b_ = nullptr;
  QLabel *caption_a_ = nullptr;
  QLabel *caption_b_ = nullptr;
  QButtonGroup *layers_ = nullptr;
  FrameLayer layer_ = FrameLayer::color;
  int a_ = -1, b_ = -1;
};

} // namespace rux::qt
```

`PairImages.cpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <rux_qt/FrameImageLoader.hpp>
#include <rux_qt/PairImages.hpp>
#include <rux_qt/ProjectSession.hpp>
#include <rux_qt/Theme.hpp>
#include <rux_qt/widgets.hpp>

#include <QButtonGroup>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>

namespace rux::qt {

PairImages::PairImages(ProjectSession &session, QWidget *parent)
    : QWidget(parent), session_(session) {
  setObjectName("pairImages");
  const Theme &t = theme();
  loader_ = new FrameImageLoader(this);
  auto *g = new QGridLayout(this);
  g->setContentsMargins(t.px("--space-2"), t.px("--space-2"), t.px("--space-2"),
                        t.px("--space-2"));
  g->setHorizontalSpacing(t.px("--space-2"));
  g->setVerticalSpacing(t.px("--space-1"));

  auto *seg = new QFrame;
  seg->setObjectName("segmented");
  auto *sl = new QHBoxLayout(seg);
  sl->setContentsMargins(0, 0, 0, 0);
  sl->setSpacing(0);
  layers_ = new QButtonGroup(this);
  const char *names[] = {"Farve", "Dybde"};
  for (int i = 0; i < 2; ++i) {
    auto *b = new QPushButton(names[i]);
    b->setObjectName("segment");
    b->setCheckable(true);
    b->setCursor(Qt::PointingHandCursor);
    b->setProperty("position", i == 0 ? "first" : "last");
    layers_->addButton(b, i);
    sl->addWidget(b);
  }
  layers_->button(0)->setChecked(true);
  g->addWidget(seg, 0, 0, 1, 2, Qt::AlignLeft);

  caption_a_ = new QLabel;
  caption_a_->setObjectName("pairCaptionA");
  caption_b_ = new QLabel;
  caption_b_->setObjectName("pairCaptionB");
  g->addWidget(caption_a_, 1, 0);
  g->addWidget(caption_b_, 1, 1);
  pane_a_ = new ImagePane;
  pane_b_ = new ImagePane;
  for (ImagePane *p : {pane_a_, pane_b_})
    p->set_overlay(false, 0);
  g->addWidget(pane_a_, 2, 0);
  g->addWidget(pane_b_, 2, 1);
  g->setRowStretch(2, 1);

  connect(layers_, &QButtonGroup::idClicked, this,
          [this](int id) { set_layer(id == 1 ? FrameLayer::depth : FrameLayer::color); });
  connect(loader_, &FrameImageLoader::frame_ready, this, &PairImages::frame_ready);
  connect(&session_, &ProjectSession::state_changed, this, &PairImages::reload);
  reload();
}

void PairImages::reload() {
  loader_->set_project(session_.is_open() ? session_.path() : QString());
  sync();
}

void PairImages::set_pair(int a, int b) {
  a_ = a;
  b_ = b;
  sync();
}

void PairImages::set_layer(FrameLayer layer) {
  layer_ = layer;
  pane_a_->set_layer(layer);
  pane_b_->set_layer(layer);
}

void PairImages::sync() {
  auto side = [this](ImagePane *pane, QLabel *caption, int id, const QString &slot) {
    caption->setText(id >= 0 ? QString("%1 · billede %2").arg(slot).arg(id)
                             : slot);
    if (id < 0) {
      pane->set_message(slot == "A" ? QString("Klik på en node i grafen for A.")
                                    : QString("Klik på en node mere for B."));
      return;
    }
    if (auto f = loader_->frame(id)) {
      pane->set_frame(f);
    } else {
      pane->set_loading(true);
      loader_->request(id);
    }
  };
  side(pane_a_, caption_a_, a_, "A");
  side(pane_b_, caption_b_, b_, "B");
}

void PairImages::frame_ready(int id) {
  auto f = loader_->frame(id);
  if (!f)
    return;
  if (id == a_)
    pane_a_->set_frame(f);
  if (id == b_)
    pane_b_->set_frame(f);
}

} // namespace rux::qt
```

QSS: captions as small caps chips in the slot colours (A = accent, B = on-chrome on the canvas, matching the graph's markers):

```css
QLabel#pairCaptionA,
QLabel#pairCaptionB {
  font-family: var(--font-mono);
  font-size: var(--font-size-xs);
  color: var(--color-text-muted);
}
QLabel#pairCaptionA {
  color: var(--color-accent-deep);
}
```

- [ ] **Step 2: Into Rediger**

`EditWorkspace`: member `PairImages *images_`, accessor `images()`; `panes_[2]->set_content(images_ = new PairImages(session_));`; `apply_pair()` also calls `images_->set_pair(slots_.a(), slots_.b());`.

- [ ] **Step 3: Gallery page**

```cpp
QWidget *make_rediger_pair(const PageContext &ctx) {
  auto *shell = make_rediger(ctx);
  return shell; // A/B set by make_rediger; images decode off-thread
}
QWidget *make_rediger_pair_depth(const PageContext &ctx) {
  auto *shell = make_rediger(ctx);
  shell->edit()->images()->set_layer(FrameLayer::depth);
  return shell;
}
```

registered `rediger-pair`, `rediger-pair-depth`.

- [ ] **Step 4: Build, shoot, review**

Run: `cmake --build build --target rux rux-qt-gallery && bash .claude/skills/design-studio/scripts/qt_shot.sh --page rediger-pair --page rediger-pair-depth --theme both`
Expected: exit 0; the gallery waits for `FrameImageLoader::busy()` so both images are decoded. Read the PNGs: A and B side by side at equal size, captions "A · billede N", depth colour scale as Oversigt's; layout at 1440×900 leaves the graph its share. Fix, re-shoot, `token_lint.py` clean.

- [ ] **Step 5: Commit**

```bash
git add apps/rux/qt
git commit -m "qt: Rediger pair images A | B with colour/depth"
```

---
### Task 18: 3D overlay with a manual ΔT on B

**Files:**
- Create: `apps/rux/qt/include/rux_qt/PairOverlay.hpp`, `apps/rux/qt/src/PairOverlay.cpp`
- Modify: `apps/rux/qt/CMakeLists.txt` (VTK `InteractionWidgets`), `apps/rux/qt/include/rux_qt/SceneView.hpp` + `src/SceneView.cpp` (`interactor()`)
- Modify: `apps/rux/qt/include/rux_qt/EditWorkspace.hpp`, `apps/rux/qt/src/EditWorkspace.cpp` (overlay pane, `interactive_3d`), `apps/rux/qt/src/AppShell.cpp` (pass `options.interactive_3d`)
- Modify: `apps/rux/qt/styles/app.qss`, `apps/rux/qt/gallery/shell_pages.cpp`

**Interfaces:**
- Consumes: Task 15 (`ManualOffset`, `offset_matrix`, `offset_from_matrix`, `nudge`, `NudgeKey`, `Mat4`, `Vec3`), Task 16 (`slam::frame_world_points`), `SceneView`, `visualize::place_preset_camera`.
- Produces: `class PairOverlay : public QWidget { PairOverlay(ProjectSession &, bool interactive, QWidget *parent = nullptr); void set_pair(int a, int b); const ManualOffset &offset() const; void set_offset(const ManualOffset &); void reset_offset(); Vec3 pivot() const; Mat4 delta() const; bool ready() const; SceneView *view() const; void widget_moved(const Mat4 &); signals: void offset_changed(); }`; `vtkRenderWindowInteractor *SceneView::interactor() const` (nullptr in snapshot mode); `EditWorkspace(ProjectSession &, EdgeEditor &, ViewSettings &, bool persist_layout, bool interactive_3d, QWidget *parent = nullptr)`, `EditWorkspace::overlay() const -> PairOverlay *`.

- [ ] **Step 1: Plumbing**

`apps/rux/qt/CMakeLists.txt`: add `InteractionWidgets` to the `find_package(VTK REQUIRED COMPONENTS …)` list. `SceneView.hpp`: forward-declare `class vtkRenderWindowInteractor;` and add `/// The interactive widget's interactor, or nullptr in snapshot mode. vtkRenderWindowInteractor *interactor() const;`; `SceneView.cpp`: `vtkRenderWindowInteractor *SceneView::interactor() const { return impl_->gl ? impl_->gl->interactor() : nullptr; }`.

- [ ] **Step 2: The overlay**

`PairOverlay.hpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Rediger's 3D overlay (spec §5.2): A's and B's depth clouds back-projected
// with their current poses (slam::frame_world_points, the clouds ICP
// aligns), A in the accent, B in the chrome ink. B carries a manual offset
// ΔT (rux_qt/pair_alignment.hpp): a VTK box widget that translates and
// rotates but never scales (interactive mode only), six numeric fields, and
// keys — ←/→ x, ↑/↓ y, PgUp/PgDn z, [ / ] yaw, Shift ×10 (VTK's own keys,
// e/q/r/…, are left alone). "Nulstil" clears ΔT. Clouds load off the GUI
// thread on their own read-only connection.

#include <rux_qt/pair_alignment.hpp>

#include <QWidget>

#include <array>
#include <memory>

class QDoubleSpinBox;
class QLabel;

namespace rux::qt {

class ProjectSession;
class SceneView;

class PairOverlay : public QWidget {
  Q_OBJECT
    public:
  PairOverlay(ProjectSession &session, bool interactive,
              QWidget *parent = nullptr);
  ~PairOverlay() override;

  void set_pair(int a, int b);
  const ManualOffset &offset() const { return offset_; }
  void set_offset(const ManualOffset &o);
  void reset_offset() { set_offset({}); }
  /// B's cloud centroid (world): the offset turns about it.
  Vec3 pivot() const { return pivot_; }
  Mat4 delta() const { return offset_matrix(offset_, pivot_); }
  /// Both clouds of the current pair are drawn.
  bool ready() const { return ready_; }
  SceneView *view() const { return view_; }
  /// The box widget moved B (called by its VTK observer).
  void widget_moved(const Mat4 &m);

    signals:
  void offset_changed();

    protected:
  bool eventFilter(QObject *watched, QEvent *event) override;

    private:
  struct Clouds;
  struct Vtk;
  void load();
  void loaded(std::shared_ptr<Clouds> c);
  void apply_offset(bool from_widget);
  void sync_fields();
  bool handle_key(int key, bool shift);
  ProjectSession &session_;
  SceneView *view_ = nullptr;
  std::array<QDoubleSpinBox *, 6> fields_{};
  QLabel *status_ = nullptr;
  std::unique_ptr<Vtk> vtk_;
  ManualOffset offset_;
  Vec3 pivot_{};
  int a_ = -1, b_ = -1;
  unsigned generation_ = 0;
  bool ready_ = false;
};

} // namespace rux::qt
```

`PairOverlay.cpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <rux_qt/PairOverlay.hpp>
#include <rux_qt/ProjectSession.hpp>
#include <rux_qt/SceneView.hpp>
#include <rux_qt/Theme.hpp>
#include <rux_qt/background.hpp>
#include <rux_qt/widgets.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/slam/frame_pair_icp.hpp>
#include <reusex/visualize/scene.hpp>

#include <QApplication>
#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QKeyEvent>
#include <QLabel>
#include <QLocale>
#include <QPointer>
#include <QVBoxLayout>

#include <vtkActor.h>
#include <vtkBoxRepresentation.h>
#include <vtkBoxWidget2.h>
#include <vtkCallbackCommand.h>
#include <vtkCellArray.h>
#include <vtkMatrix4x4.h>
#include <vtkNew.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkTransform.h>

#include <cmath>
#include <thread>

namespace rux::qt {
namespace {

constexpr double kPointPx = 2.0;
using Points = std::vector<std::array<float, 3>>;

vtkSmartPointer<vtkActor> points_actor(const Points &pts, const QColor &c) {
  vtkNew<vtkPoints> p;
  p->SetNumberOfPoints(static_cast<vtkIdType>(pts.size()));
  vtkNew<vtkCellArray> verts;
  for (vtkIdType i = 0; i < static_cast<vtkIdType>(pts.size()); ++i) {
    const auto &q = pts[static_cast<std::size_t>(i)];
    p->SetPoint(i, q[0], q[1], q[2]);
    verts->InsertNextCell(1, &i);
  }
  vtkNew<vtkPolyData> poly;
  poly->SetPoints(p);
  poly->SetVerts(verts);
  vtkNew<vtkPolyDataMapper> mapper;
  mapper->SetInputData(poly);
  auto actor = vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);
  actor->GetProperty()->SetColor(c.redF(), c.greenF(), c.blueF());
  actor->GetProperty()->SetPointSize(kPointPx);
  return actor;
}

void to_vtk(const Mat4 &m, vtkMatrix4x4 *out) {
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j)
      out->SetElement(i, j, m[static_cast<std::size_t>(i * 4 + j)]);
}

void on_box(vtkObject *caller, unsigned long, void *client, void *) {
  auto *w = vtkBoxWidget2::SafeDownCast(caller);
  auto *rep = vtkBoxRepresentation::SafeDownCast(w->GetRepresentation());
  vtkNew<vtkTransform> t;
  rep->GetTransform(t);
  Mat4 m{};
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j)
      m[static_cast<std::size_t>(i * 4 + j)] = t->GetMatrix()->GetElement(i, j);
  static_cast<PairOverlay *>(client)->widget_moved(m);
}

} // namespace

struct PairOverlay::Clouds {
  unsigned generation = 0;
  Points a, b;
  Vec3 pivot{};
  QString error;
};

struct PairOverlay::Vtk {
  vtkSmartPointer<vtkActor> a, b;
  vtkSmartPointer<vtkBoxWidget2> box;
  vtkSmartPointer<vtkCallbackCommand> observer;
};

PairOverlay::PairOverlay(ProjectSession &session, bool interactive,
                         QWidget *parent)
    : QWidget(parent), session_(session), vtk_(std::make_unique<Vtk>()) {
  setObjectName("pairOverlay");
  const Theme &t = theme();
  auto *v = new QVBoxLayout(this);
  v->setContentsMargins(0, 0, 0, 0);
  v->setSpacing(0);
  view_ = new SceneView(interactive);
  v->addWidget(view_, 1);

  auto *fields = new QFrame;
  fields->setObjectName("offsetFields");
  auto *g = new QGridLayout(fields);
  g->setContentsMargins(t.px("--space-3"), t.px("--space-2"),
                        t.px("--space-3"), t.px("--space-2"));
  g->setHorizontalSpacing(t.px("--space-2"));
  const char *labels[6] = {"X [m]", "Y [m]", "Z [m]",
                           "Drej [°]", "Hæld [°]", "Rul [°]"};
  for (int i = 0; i < 6; ++i) {
    auto *s = new QDoubleSpinBox;
    s->setObjectName("offsetField");
    s->setButtonSymbols(QAbstractSpinBox::NoButtons);
    s->setLocale(QLocale(QLocale::Danish, QLocale::Denmark));
    if (i < 3) {
      s->setRange(-50.0, 50.0);
      s->setDecimals(3);
      s->setSingleStep(kNudgeM);
    } else {
      s->setRange(-180.0, 180.0);
      s->setDecimals(2);
      s->setSingleStep(kNudgeDeg);
    }
    g->addWidget(new CapsLabel(labels[i], "fieldLabel"), 0, i);
    g->addWidget(s, 1, i);
    fields_[static_cast<std::size_t>(i)] = s;
    connect(s, &QDoubleSpinBox::valueChanged, this, [this, i](double val) {
      double *slot[6] = {&offset_.x, &offset_.y, &offset_.z,
                         &offset_.yaw_deg, &offset_.pitch_deg, &offset_.roll_deg};
      *slot[i] = val;
      apply_offset(false);
    });
  }
  status_ = new QLabel;
  status_->setObjectName("statusText");
  status_->setWordWrap(true);
  g->addWidget(status_, 2, 0, 1, 6);
  v->addWidget(fields);

  setFocusPolicy(Qt::StrongFocus);
  view_->installEventFilter(this);
  for (QWidget *w : view_->findChildren<QWidget *>())
    w->installEventFilter(this);
  connect(&session_, &ProjectSession::state_changed, this, [this] {
    a_ = b_ = -1;
    load();
  });
  load();
}

PairOverlay::~PairOverlay() {
  if (vtk_->box)
    vtk_->box->Off();
}

void PairOverlay::set_pair(int a, int b) {
  if (a == a_ && b == b_)
    return;
  a_ = a;
  b_ = b;
  offset_ = {}; // a new pair starts unmoved
  load();
}

void PairOverlay::load() {
  ++generation_;
  ready_ = false;
  if (vtk_->box)
    vtk_->box->Off();
  vtk_->box = nullptr;
  vtk_->a = vtk_->b = nullptr;
  view_->clear_scene();
  sync_fields();
  if (!session_.is_open() || a_ < 0 || b_ < 0 || a_ == b_) {
    view_->set_message("Vælg A og B i grafen.");
    return;
  }
  view_->set_message("Indlæser skyerne …");
  const std::string path = session_.path().toStdString();
  const int a = a_, b = b_;
  const unsigned gen = generation_;
  auto work = std::make_shared<BackgroundWork>();
  QPointer<PairOverlay> guard(this);
  std::thread([path, a, b, gen, guard, work]() mutable {
    auto c = std::make_shared<Clouds>();
    c->generation = gen;
    try {
      reusex::ProjectDB db(path, /*readOnly=*/true);
      c->a = reusex::slam::frame_world_points(db, a);
      c->b = reusex::slam::frame_world_points(db, b);
      double sx = 0, sy = 0, sz = 0;
      for (const auto &p : c->b) {
        sx += p[0];
        sy += p[1];
        sz += p[2];
      }
      const double n = static_cast<double>(std::max<std::size_t>(c->b.size(), 1));
      c->pivot = {sx / n, sy / n, sz / n};
    } catch (const std::exception &e) {
      c->error = QString::fromUtf8(e.what());
    }
    QMetaObject::invokeMethod(
        qApp,
        [guard, c] {
          if (guard)
            guard->loaded(c);
        },
        Qt::QueuedConnection);
    work.reset();
  }).detach();
}

void PairOverlay::loaded(std::shared_ptr<Clouds> c) {
  if (c->generation != generation_)
    return;
  if (!c->error.isEmpty()) {
    view_->set_message(QString("Skyerne kunne ikke læses — 3D-overlayet kræver "
                               "dybde og pose i begge billeder.\n%1")
                           .arg(c->error));
    return;
  }
  const Theme &t = theme();
  pivot_ = c->pivot;
  vtk_->a = points_actor(c->a, t.color("--color-accent"));
  vtk_->b = points_actor(c->b, t.color("--color-on-chrome"));
  vtkRenderer *r = view_->renderer();
  r->AddActor(vtk_->a);
  r->AddActor(vtk_->b);
  reusex::visualize::SceneBounds bounds;
  for (const Points *pts : {&c->a, &c->b})
    for (const auto &p : *pts)
      bounds.add(p[0], p[1], p[2]);
  reusex::visualize::CameraFraming f;
  f.view = reusex::visualize::ViewPreset::orbit;
  f.orbit_index = 1;
  f.orbit_elevation_deg = 35.0;
  f.aspect = view_->aspect();
  reusex::visualize::place_preset_camera(r, f, bounds);
  if (vtkRenderWindowInteractor *iren = view_->interactor()) {
    vtk_->box = vtkSmartPointer<vtkBoxWidget2>::New();
    vtk_->box->SetInteractor(iren);
    vtk_->box->SetScalingEnabled(false);
    vtk_->box->SetMoveFacesEnabled(false);
    vtk_->box->SetRotationEnabled(true);
    vtk_->box->SetTranslationEnabled(true);
    auto *rep = vtkBoxRepresentation::SafeDownCast(vtk_->box->GetRepresentation());
    rep->SetPlaceFactor(1.0);
    rep->PlaceWidget(vtk_->b->GetBounds()); // untransformed B
    vtk_->observer = vtkSmartPointer<vtkCallbackCommand>::New();
    vtk_->observer->SetCallback(on_box);
    vtk_->observer->SetClientData(this);
    vtk_->box->AddObserver(vtkCommand::InteractionEvent, vtk_->observer);
    vtk_->box->On();
  }
  view_->set_message({});
  ready_ = true;
  apply_offset(false);
}

void PairOverlay::set_offset(const ManualOffset &o) {
  offset_ = o;
  apply_offset(false);
}

void PairOverlay::widget_moved(const Mat4 &m) {
  offset_ = offset_from_matrix(m, pivot_);
  apply_offset(true);
}

void PairOverlay::apply_offset(bool from_widget) {
  sync_fields();
  if (vtk_->b) {
    vtkNew<vtkMatrix4x4> m;
    to_vtk(delta(), m);
    vtk_->b->SetUserMatrix(m);
    if (vtk_->box && !from_widget) {
      vtkNew<vtkTransform> tr;
      tr->SetMatrix(m);
      vtkBoxRepresentation::SafeDownCast(vtk_->box->GetRepresentation())
          ->SetTransform(tr);
    }
    view_->request_render();
  }
  const double shift = std::sqrt(offset_.x * offset_.x + offset_.y * offset_.y +
                                 offset_.z * offset_.z);
  status_->setText(
      offset_.is_zero()
          ? QString("B står på sin gemte pose. ←/→ x · ↑/↓ y · PgUp/PgDn z · "
                    "[ / ] drej · Shift: ×10")
          : QString("B er flyttet %1 cm og drejet %2° i forhold til den gemte pose.")
                .arg(QString::fromStdString(format_decimal_da(shift * 100.0, 1)),
                     QString::fromStdString(format_decimal_da(
                         std::abs(offset_.yaw_deg) + std::abs(offset_.pitch_deg) +
                             std::abs(offset_.roll_deg), 1))));
  emit offset_changed();
}

void PairOverlay::sync_fields() {
  const double values[6] = {offset_.x, offset_.y, offset_.z,
                            offset_.yaw_deg, offset_.pitch_deg, offset_.roll_deg};
  for (int i = 0; i < 6; ++i) {
    QDoubleSpinBox *s = fields_[static_cast<std::size_t>(i)];
    const QSignalBlocker b(s);
    s->setValue(values[i]);
    s->setEnabled(ready_);
  }
}

bool PairOverlay::handle_key(int key, bool shift) {
  NudgeKey k;
  switch (key) {
  case Qt::Key_Left: k = NudgeKey::x_minus; break;
  case Qt::Key_Right: k = NudgeKey::x_plus; break;
  case Qt::Key_Up: k = NudgeKey::y_plus; break;
  case Qt::Key_Down: k = NudgeKey::y_minus; break;
  case Qt::Key_PageUp: k = NudgeKey::z_plus; break;
  case Qt::Key_PageDown: k = NudgeKey::z_minus; break;
  case Qt::Key_BracketLeft: k = NudgeKey::yaw_plus; break;
  case Qt::Key_BracketRight: k = NudgeKey::yaw_minus; break;
  default: return false;
  }
  if (!ready_)
    return true;
  set_offset(nudge(offset_, k, shift));
  return true;
}

bool PairOverlay::eventFilter(QObject *watched, QEvent *event) {
  if (event->type() == QEvent::KeyPress) {
    auto *k = static_cast<QKeyEvent *>(event);
    if (handle_key(k->key(), k->modifiers().testFlag(Qt::ShiftModifier)))
      return true;
  }
  return QWidget::eventFilter(watched, event);
}

} // namespace rux::qt
```

`format_decimal_da` comes from `rux_qt/database_logic.hpp` (already included through `pair_alignment.hpp`). QSS: `QFrame#offsetFields { background: var(--color-surface-raised); border-top: 1px solid var(--color-border); }`.

- [ ] **Step 3: Into Rediger**

`EditWorkspace` constructor gains `bool interactive_3d`; `AppShell` passes `options.interactive_3d`. Member `PairOverlay *overlay_`, accessor `overlay()`; `panes_[3]->set_content(overlay_ = new PairOverlay(session_, interactive_3d));` and in the overlay pane header:

```cpp
  auto *reset = new QPushButton(NavItem::escape_mnemonic("Nulstil"));
  reset->setProperty("kind", "ghost");
  reset->setCursor(Qt::PointingHandCursor);
  reset->setToolTip("Sæt B tilbage på sin gemte pose");
  reset->setMinimumHeight(reset->sizeHint().height());
  connect(reset, &QPushButton::clicked, overlay_, &PairOverlay::reset_offset);
  panes_[3]->add_header_widget(reset);
```

`apply_pair()` also calls `overlay_->set_pair(slots_.a(), slots_.b());` (the overlay pane is already hidden until the pair is complete).

- [ ] **Step 4: Gallery pages**

```cpp
/// B moved by hand: 20 cm in x, 5° yaw. The interactive box widget needs
/// --gl (xvfb); the offscreen shot shows the fields and the moved cloud.
QWidget *make_rediger_offset(const PageContext &ctx) {
  auto *shell = make_rediger(ctx);
  settle(); // clouds loaded
  shell->edit()->overlay()->set_offset({0.20, 0.0, 0.0, 5.0, 0.0, 0.0});
  return shell;
}
```

registered `rediger-offset`.

- [ ] **Step 5: Build and shoot offscreen**

Run: `cmake --build build --target rux rux-qt-gallery && bash .claude/skills/design-studio/scripts/qt_shot.sh --page rediger-pair --page rediger-offset --theme both`
Expected: exit 0. Read: the overlay pane shows two tinted clouds; in `rediger-offset` B is visibly displaced and the fields read 0,200 / 5,00; status line "B er flyttet 20,0 cm og drejet 5,0° …".

- [ ] **Step 6: Shoot the real widget under xvfb**

Run: `bash .claude/skills/design-studio/scripts/qt_shot.sh --page rediger-offset --theme dark --gl`
Expected: a PNG (xvfb-run exits 1 on success — judge by the PNG): the box widget outlines B, without scale handles on the faces. If the handles show, `SetMoveFacesEnabled(false)` did not take: check the VTK version's API and use `rep->HandlesOff()` for the face handles only as the fallback.

- [ ] **Step 7: Missing depth**

Make a copy whose frame A has no depth (`sqlite3 "$COPY" "UPDATE sensor_frames SET depth = NULL WHERE node_id = (SELECT MIN(node_id) FROM sensor_frames)"` — check the column name with `.schema sensor_frames` first) and shoot `rediger-pair --project "$COPY"`; Expected: the overlay pane shows the Danish reason, no crash, the images pane still shows both colour images.

- [ ] **Step 8: Commit**

```bash
git add apps/rux/qt
git commit -m "qt: Rediger 3D overlay of the pair with a manual offset on B

Box widget (translate/rotate, no scale), six fields and key nudges;
clouds are slam::frame_world_points, loaded off the GUI thread."
```

---

### Task 19: ICP seeded with ΔT

**Files:**
- Modify: `apps/rux/qt/include/rux_qt/PairStrip.hpp`, `apps/rux/qt/src/PairStrip.cpp:349-394` (`run_icp`, `icp_finished`, status line)
- Modify: `apps/rux/qt/src/EditWorkspace.cpp` (wiring), `apps/rux/qt/gallery/shell_pages.cpp`

**Interfaces:**
- Consumes: assumed `FramePairIcpOptions::initial_guess` + `FramePairIcpResult::world_delta` semantics (assumption 4); Task 18 (`PairOverlay::delta()`, `offset()`, `ready()`, `set_offset`, `pivot`).
- Produces: `PairStrip::set_delta_source(std::function<std::optional<Mat4>()>)` (nullopt = no guess), signal `void icp_aligned(int a, int b, const rux::qt::Mat4 &world_delta)`; `IcpOutcome` gains `Mat4 world_delta` and `bool seeded`.

- [ ] **Step 1: Seed and report**

`PairStrip.hpp`: `#include <rux_qt/pair_alignment.hpp>`, add `void set_delta_source(std::function<std::optional<Mat4>()> source) { delta_source_ = std::move(source); }`, member `std::function<std::optional<Mat4>()> delta_source_;`, the signal above, and in `IcpOutcome` `Mat4 world_delta = mat4_identity(); bool seeded = false;`.

`run_icp()`: before the thread,

```cpp
  std::optional<Mat4> guess;
  if (delta_source_)
    guess = delta_source_();
```

capture `guess` and in the thread:

```cpp
      reusex::slam::FramePairIcpOptions opts;
      opts.initial_guess = guess;
      // B onto A: "from" B, "to" A, as an edge A -> B constrains B.
      const auto r = reusex::slam::refine_frame_pair_icp(db, b, a, opts);
      o.world_delta = r.world_delta;
      o.seeded = guess.has_value();
```

`icp_finished()`: after storing the result,

```cpp
  const IcpOutcome &o = icp_results_[{a, b}];
  if (o.ok && o.converged)
    emit icp_aligned(a, b, o.world_delta);
```

The converged status text gains a prefix when seeded: `(o.seeded ? QString("Fra din manuelle placering · ") : QString())`. The non-converged text adds `" B bliver stående."`.

- [ ] **Step 2: Wire the overlay**

In `EditWorkspace`:

```cpp
  strip_->set_delta_source([this]() -> std::optional<Mat4> {
    if (!overlay_->ready() || overlay_->offset().is_zero())
      return std::nullopt;
    return overlay_->delta();
  });
  connect(strip_, &PairStrip::icp_aligned, this,
          [this](int a, int b, const Mat4 &delta) {
            if (a == slots_.a() && b == slots_.b() && overlay_->ready())
              overlay_->set_offset(offset_from_matrix(delta, overlay_->pivot()));
          });
```

- [ ] **Step 3: Gallery**

```cpp
/// Move B by hand, then ICP-forfin from there: B snaps onto A.
QWidget *make_rediger_icp(const PageContext &ctx) {
  auto *shell = make_rediger(ctx);
  settle();
  shell->edit()->overlay()->set_offset({0.05, -0.03, 0.0, 2.0, 0.0, 0.0});
  for (auto *b : shell->edit()->strip()->findChildren<QPushButton *>())
    if (b->text() == "ICP-forfin")
      b->click();
  settle();
  return shell;
}
```

registered `rediger-icp`.

- [ ] **Step 4: Build, shoot, review**

Run: `cmake --build build --target rux rux-qt-gallery && bash .claude/skills/design-studio/scripts/qt_shot.sh --page rediger-icp --theme both`
Expected: exit 0. Read: the ICP line starts "Fra din manuelle placering · ICP konvergerede …", the fields now hold the ICP result (not 0,050 / 2,00), the clouds overlap. If B jumps back to the stored pose, the library returns `world_delta` *without* the guess — assumption 4 is wrong; reconcile with the library plan (compose `guess` and `world_delta` here only if the library documents the delta as relative to the guess).

- [ ] **Step 5: Commit**

```bash
git add apps/rux/qt
git commit -m "qt: ICP-forfin starts from the manual offset and moves B to its result"
```

---

### Task 20: Edge rows — add with a measured transform, confirm, reject / un-reject

**Files:**
- Modify: `apps/rux/qt/include/rux_qt/PairStrip.hpp`, `apps/rux/qt/src/PairStrip.cpp` (`add_edge`, rows, add button label)
- Modify: `apps/rux/qt/gallery/shell_pages.cpp`

**Interfaces:**
- Consumes: Task 15 (`edge_measurement`, `icp_information`, `oriented_edge`, `mat4_identity`), Task 19 (`delta_source_`, `IcpOutcome::world_delta`), Task 4 (`EdgeEditor::add/confirm/reject/unreject/revert`, `Result`).
- Produces: user-visible behaviour only; `PairStrip::add_edge()` stays private.

- [ ] **Step 1: `add_edge` stages a real measurement**

Replace `PairStrip::add_edge()`:

```cpp
void PairStrip::add_edge() {
  const reusex::ProjectDB *db = session_.db();
  if (!db || a_ < 0 || b_ < 0 || a_ == b_)
    return;
  if (!db->has_sensor_frame_pose(a_) || !db->has_sensor_frame_pose(b_)) {
    note_->setText("Begge billeder skal have en pose, før en kant kan måles.");
    note_->setVisible(true);
    return;
  }
  const Mat4 delta =
      delta_source_ ? delta_source_().value_or(mat4_identity()) : mat4_identity();
  EdgeRecord e;
  e.key = {a_, b_, type_->currentData().toString().toStdString()};
  e.transform = edge_measurement(db->sensor_frame_pose(a_),
                                 db->sensor_frame_pose(b_), delta);
  const auto it = icp_results_.find({a_, b_});
  bool from_icp = it != icp_results_.end() && it->second.ok && it->second.converged;
  if (from_icp)
    for (std::size_t i = 0; i < 16; ++i)
      from_icp = from_icp && std::abs(it->second.world_delta[i] - delta[i]) < 1e-9;
  if (from_icp) {
    e.source = "icp";
    e.information = icp_information(it->second.fitness);
    e.weight = weight_from_icp_fitness(it->second.fitness);
  } else {
    e.source = "manual"; // the optimizer's per-type default information
    e.weight = std::numeric_limits<double>::quiet_NaN();
  }
  e = oriented_edge(e, editor_.edits().between(a_, b_));
  QString msg;
  switch (editor_.add(e)) {
  case PendingEdgeEdits::Result::added:
    break;
  case PendingEdgeEdits::Result::replaced:
    msg = QString("Kanten %1 → %2 får den nye måling ved gem.")
              .arg(e.key.from).arg(e.key.to);
    break;
  case PendingEdgeEdits::Result::revived:
    msg = "Den afviste kant genoplives med den nye måling ved gem.";
    break;
  default:
    msg = "Kanten kan ikke tilføjes (samme billede eller ukendt type).";
  }
  note_->setText(msg);
  note_->setVisible(!msg.isEmpty());
}
```

(`#include <limits>`; `delta_source_` returns nullopt for a zero offset, which is the identity anyway.) Note: with no ICP and no offset this measures the *stored* relative pose — the add button's tooltip says so: `"Tilføj en kant med den nuværende relative pose (efter din placering og ICP, hvis kørt). Bekræftet; gemmes med Gem ændringer."`.

- [ ] **Step 2: Row actions and labels**

In `rebuild()`'s row loop, before the existing action selection, give `auto` rows a confirm button:

```cpp
    if (v.pending == EdgeView::Pending::none && status == "auto")
      action("Bekræft", [](EdgeEditor &e, const EdgeKey &k) { e.confirm(k); });
```

and replace the "Vægt …"/"Residual …" meta labels with the metrics that matter for judging an edge:

```cpp
    auto *meta = new QLabel(QString("Residual %1 · afvigelse %2 m / %3° · kilde %4")
        .arg(QString::fromStdString(edge_cell_text(v, EdgeColumn::residual)),
             QString::fromStdString(edge_cell_text(v, EdgeColumn::deviation_m)),
             QString::fromStdString(edge_cell_text(v, EdgeColumn::deviation_deg)),
             QString::fromStdString(edge_source_da(v.edge.source))));
    meta->setObjectName("edgeMeta");
    h->addWidget(meta);
```

(`#include <rux_qt/edge_table.hpp>`). Rejected rows get `row->setProperty("state", "rejected")` and QSS `QFrame#edgeRow[state="rejected"] QLabel { color: var(--color-text-faint); }`. After the loop, the add button reads "Erstat måling" when `views` holds an effective non-rejected edge of the combo's type in either direction, else "Tilføj kant"; refresh it on `type_`'s `currentIndexChanged` too. Add "Nulstil" next to "ICP-forfin" in the head row, emitting a new signal `reset_requested()` that `EditWorkspace` connects to `overlay_->reset_offset()`.

- [ ] **Step 3: Gallery**

```cpp
/// The staged states: a new ICP-measured edge, a confirmation and a
/// rejection, with the banner counting them.
QWidget *make_rediger_staged(const PageContext &ctx) {
  seed_graph(ctx.project_path, 40);
  auto *shell = make_rediger(ctx);
  settle();
  EdgeEditor &ed = shell->edges();
  for (const auto &v : ed.edits().effective())
    if (v.edge.status == "auto" && v.edge.key.type == "loop_closure") {
      ed.confirm(v.edge.key);
      break;
    }
  for (const auto &v : ed.edits().effective())
    if (v.edge.status == "auto" && v.edge.key.type == "odometry") {
      ed.reject(v.edge.key);
      shell->edit()->set_pair(v.edge.key.from, v.edge.key.to);
      break;
    }
  settle();
  for (auto *b : shell->edit()->strip()->findChildren<QPushButton *>())
    if (b->text() == "ICP-forfin")
      b->click();
  settle();
  for (auto *b : shell->edit()->strip()->findChildren<QPushButton *>())
    if (b->text() == "Tilføj kant" || b->text() == "Erstat måling") {
      b->click();
      break;
    }
  return shell;
}
```

registered `rediger-staged`.

- [ ] **Step 4: Build, shoot, review**

Run: `cmake --build build --target reusex_unit_tests rux rux-qt-gallery && build/tests/reusex_unit_tests "[rux_qt]" && bash .claude/skills/design-studio/scripts/qt_shot.sh --page rediger-staged --page rediger-pending --page rediger-readonly --theme both`
Expected: all pass, exit 0. Read: rows show Auto/Bekræftet/Afvist pills, "Ikke gemt", Bekræft / Afvis / Fortryd afvisning / Fortryd; the new edge is listed in the edge list (status "Bekræftet · ikke gemt", source ICP); the banner counts "1 ny eller målt kant, 1 afvist, 1 bekræftet"; readonly keeps the warn banner. Fix, re-shoot, `token_lint.py` clean.

- [ ] **Step 5: Save round trip on a copy**

Run: `bash .claude/skills/design-studio/scripts/qt_shot.sh --page rediger-saved --theme dark` then, on the same copy (`--project "$COPY"` with your own copy), `sqlite3 "$COPY" "SELECT from_node_id, to_node_id, edge_type, source, status, length(transform) FROM pose_graph_edges WHERE source IN ('icp','manual')"`.
Expected: one row with status `confirmed` and `length(transform) = 128`.

- [ ] **Step 6: Manual smoke on a real RTABMap project**

On a copy of a real project (after `rux graph init` / `rux optimize` from P2): `XDG_CONFIG_HOME=$(mktemp -d) build/apps/rux/rux -p "$COPY"` with a display; go to Rediger, colour by residual, open the top row of the list, move B with the box widget, ICP-forfin, Tilføj kant, Afvis the suspicious edge, Gem ændringer, Optimer. Expected: the rejected edge stays rejected after Optimer and Find kanter (dashed with "Vis afviste"), the new edge is confirmed. Note anything surprising in the PR description.

- [ ] **Step 7: Commit**

```bash
git add apps/rux/qt
git commit -m "qt: pair strip adds measured edges (ICP or manual), confirms and rejects"
```

---

# Phase P6 — Panorama preview in Oversigt

### Task 21: Panorama preview

**Files:**
- Modify: `apps/rux/qt/include/rux_qt/database_logic.hpp`, `apps/rux/qt/src/core/database_logic.cpp` (`fit_within`), `tests/unit/rux_qt/test_database_logic.cpp`
- Create: `apps/rux/qt/include/rux_qt/PanoramaViewer.hpp`, `apps/rux/qt/src/PanoramaViewer.cpp`
- Modify: `apps/rux/qt/include/rux_qt/DatabaseWorkspace.hpp` (`Kind::panorama`), `apps/rux/qt/src/DatabaseWorkspace.cpp:168-169,400-450` (tree children, `show_item`)
- Modify: `apps/rux/qt/CMakeLists.txt` (gallery links `opencv_core opencv_imgcodecs`), `apps/rux/qt/gallery/shell_pages.cpp`, `apps/rux/qt/styles/app.qss`

**Interfaces:**
- Consumes: `ProjectDB::list_panoramic_images()`, `ProjectDB::panoramic_image(int)`, `zoom_limits`/`clamp_zoom_factor` (Task 1).
- Produces: `struct ImageSize { int width = 0, height = 0; }`, `ImageSize fit_within(int width, int height, int max_side)`, `inline constexpr int kPanoramaMaxSide = 4096`; `class PanoramaViewer : public QWidget { explicit PanoramaViewer(QWidget *parent = nullptr); void show_panorama(const QString &project, int id, const QString &caption); void clear(); bool loading() const; }`; `ProjectTree::Kind::panorama`.

- [ ] **Step 1: Write the failing test**

Append to `test_database_logic.cpp`:

```cpp
TEST_CASE("FitWithin_KeepsTheAspectAndNeverEnlarges", "[rux_qt][database][panorama]") {
  CHECK(fit_within(8192, 4096, 4096).width == 4096);
  CHECK(fit_within(8192, 4096, 4096).height == 2048);
  CHECK(fit_within(2048, 1024, 4096).width == 2048); // small: unchanged
  CHECK(fit_within(1000, 3000, 1500).height == 1500);
  CHECK(fit_within(1000, 3000, 1500).width == 500);
  CHECK(fit_within(0, 100, 4096).width == 0);
  CHECK(fit_within(7, 3, 2).height >= 1); // never collapses to 0
}
```

- [ ] **Step 2: Run to verify it fails**

Run: `cmake --build build --target reusex_unit_tests`
Expected: compile error (`fit_within` not declared).

- [ ] **Step 3: Implement `fit_within`**

Header (formatting section):

```cpp
struct ImageSize {
  int width = 0;
  int height = 0;
};
/// The size to show a @p width x @p height image at so its longer side is
/// at most @p max_side (aspect kept, never enlarged, never below 1 px).
ImageSize fit_within(int width, int height, int max_side);
/// The longest side a panorama is decoded to for the preview.
inline constexpr int kPanoramaMaxSide = 4096;
```

Source:

```cpp
ImageSize fit_within(int width, int height, int max_side) {
  if (width <= 0 || height <= 0 || max_side <= 0)
    return {std::max(width, 0), std::max(height, 0)};
  const int longer = std::max(width, height);
  if (longer <= max_side)
    return {width, height};
  const double s = static_cast<double>(max_side) / longer;
  return {std::max(1, static_cast<int>(std::lround(width * s))),
          std::max(1, static_cast<int>(std::lround(height * s)))};
}
```

Run: `cmake --build build --target reusex_unit_tests && build/tests/reusex_unit_tests "[panorama]"` → PASS.

- [ ] **Step 4: `PanoramaViewer`**

`PanoramaViewer.hpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// A flat equirectangular panorama in Oversigt (spec §5.3): decoded off the
// GUI thread (own read-only ProjectDB), scaled to at most kPanoramaMaxSide,
// shown in a pan/zoom view — wheel zooms about the cursor (clamped), a drag
// pans, F fits. No spherical look-around.

#include <QGraphicsView>
#include <QWidget>

class QGraphicsPixmapItem;
class QLabel;

namespace rux::qt {

class PanoramaView : public QGraphicsView {
  Q_OBJECT
    public:
  explicit PanoramaView(QWidget *parent = nullptr);
  void fit();

    protected:
  void wheelEvent(QWheelEvent *e) override;
  void keyPressEvent(QKeyEvent *e) override;
  void resizeEvent(QResizeEvent *e) override;

    private:
  bool user_view_ = false;
};

class PanoramaViewer : public QWidget {
  Q_OBJECT
    public:
  explicit PanoramaViewer(QWidget *parent = nullptr);
  void show_panorama(const QString &project, int id, const QString &caption);
  void clear();
  bool loading() const { return loading_; }

    private:
  void loaded(unsigned generation, QImage image, QString error);
  PanoramaView *view_ = nullptr;
  QGraphicsPixmapItem *item_ = nullptr;
  QLabel *caption_ = nullptr;
  QLabel *message_ = nullptr;
  unsigned generation_ = 0;
  bool loading_ = false;
};

} // namespace rux::qt
```

`PanoramaViewer.cpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <rux_qt/PanoramaViewer.hpp>
#include <rux_qt/Theme.hpp>
#include <rux_qt/background.hpp>
#include <rux_qt/database_logic.hpp>
#include <rux_qt/graph_view.hpp>

#include <reusex/core/ProjectDB.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <QApplication>
#include <QGraphicsPixmapItem>
#include <QKeyEvent>
#include <QLabel>
#include <QPointer>
#include <QStackedLayout>
#include <QVBoxLayout>
#include <QWheelEvent>

#include <cmath>
#include <thread>

namespace rux::qt {

PanoramaView::PanoramaView(QWidget *parent) : QGraphicsView(parent) {
  setObjectName("panoramaView");
  setDragMode(QGraphicsView::ScrollHandDrag);
  setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
  setRenderHint(QPainter::SmoothPixmapTransform);
  setFrameShape(QFrame::NoFrame);
  setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  setFocusPolicy(Qt::StrongFocus);
}

void PanoramaView::fit() {
  if (!scene() || scene()->itemsBoundingRect().isEmpty())
    return;
  fitInView(scene()->itemsBoundingRect(), Qt::KeepAspectRatio);
  user_view_ = false;
}

void PanoramaView::wheelEvent(QWheelEvent *e) {
  const double upp = 1.0 / std::max(transform().m11(), 1e-12);
  const QRectF r = scene() ? scene()->itemsBoundingRect() : QRectF();
  // Image pixels per screen pixel: in to 8x magnification, out to the fit.
  ZoomLimits z;
  z.min_units_per_px = 0.125;
  z.max_units_per_px = std::max(r.width() / std::max(viewport()->width(), 1),
                                r.height() / std::max(viewport()->height(), 1));
  z.max_units_per_px = std::max(z.max_units_per_px, z.min_units_per_px);
  const double f = clamp_zoom_factor(upp, std::pow(1.0015, e->angleDelta().y()), z);
  if (f != 1.0)
    scale(f, f);
  user_view_ = true;
  e->accept();
}

void PanoramaView::keyPressEvent(QKeyEvent *e) {
  if (e->key() == Qt::Key_F && e->modifiers() == Qt::NoModifier) {
    fit();
    return;
  }
  QGraphicsView::keyPressEvent(e);
}

void PanoramaView::resizeEvent(QResizeEvent *e) {
  QGraphicsView::resizeEvent(e);
  if (!user_view_)
    fit();
}

PanoramaViewer::PanoramaViewer(QWidget *parent) : QWidget(parent) {
  setObjectName("panoramaViewer");
  const Theme &t = theme();
  auto *v = new QVBoxLayout(this);
  v->setContentsMargins(0, 0, 0, 0);
  v->setSpacing(0);
  auto *bar = new QFrame;
  bar->setObjectName("dbToolbar");
  auto *bl = new QHBoxLayout(bar);
  bl->setContentsMargins(t.px("--space-4"), t.px("--space-2"), t.px("--space-4"),
                         t.px("--space-2"));
  caption_ = new QLabel;
  caption_->setObjectName("toolbarMeta");
  bl->addWidget(caption_, 1);
  auto *hint = new QLabel("hjul: zoom · træk: panorér · F: tilpas");
  hint->setObjectName("toolbarMeta");
  bl->addWidget(hint);
  v->addWidget(bar);
  auto *host = new QWidget;
  auto *stack = new QStackedLayout(host);
  view_ = new PanoramaView;
  view_->setScene(new QGraphicsScene(view_));
  view_->setBackgroundBrush(t.color("--color-canvas"));
  item_ = view_->scene()->addPixmap(QPixmap());
  stack->addWidget(view_);
  message_ = new QLabel;
  message_->setObjectName("viewportEmpty");
  message_->setAlignment(Qt::AlignCenter);
  message_->setWordWrap(true);
  stack->addWidget(message_);
  v->addWidget(host, 1);
  connect(&theme(), &Theme::changed, this, [this] {
    view_->setBackgroundBrush(theme().color("--color-canvas"));
  });
}

void PanoramaViewer::clear() {
  ++generation_;
  loading_ = false;
  item_->setPixmap(QPixmap());
  caption_->clear();
}

void PanoramaViewer::show_panorama(const QString &project, int id,
                                   const QString &caption) {
  const unsigned gen = ++generation_;
  loading_ = true;
  caption_->setText(caption);
  message_->setText("Henter panorama …");
  static_cast<QStackedLayout *>(message_->parentWidget()->layout())
      ->setCurrentWidget(message_);
  const std::string path = project.toStdString();
  auto work = std::make_shared<BackgroundWork>();
  QPointer<PanoramaViewer> guard(this);
  std::thread([path, id, gen, guard, work]() mutable {
    QImage img;
    QString error;
    try {
      reusex::ProjectDB db(path, /*readOnly=*/true);
      cv::Mat bgr = db.panoramic_image(id);
      if (bgr.empty())
        throw std::runtime_error("empty image");
      const ImageSize s = fit_within(bgr.cols, bgr.rows, kPanoramaMaxSide);
      if (s.width != bgr.cols)
        cv::resize(bgr, bgr, cv::Size(s.width, s.height), 0, 0, cv::INTER_AREA);
      cv::Mat rgb;
      cv::cvtColor(bgr, rgb, bgr.channels() == 1 ? cv::COLOR_GRAY2RGB : cv::COLOR_BGR2RGB);
      img = QImage(rgb.data, rgb.cols, rgb.rows, static_cast<int>(rgb.step),
                   QImage::Format_RGB888)
                .copy();
    } catch (const std::exception &e) {
      error = QString::fromUtf8(e.what());
    }
    QMetaObject::invokeMethod(
        qApp,
        [guard, gen, img, error] {
          if (guard)
            guard->loaded(gen, img, error);
        },
        Qt::QueuedConnection);
    work.reset();
  }).detach();
}

void PanoramaViewer::loaded(unsigned generation, QImage image, QString error) {
  if (generation != generation_)
    return;
  loading_ = false;
  auto *stack = static_cast<QStackedLayout *>(message_->parentWidget()->layout());
  if (!error.isEmpty() || image.isNull()) {
    message_->setText(QString("Panoramaet kunne ikke læses.\n%1").arg(error));
    stack->setCurrentWidget(message_);
    return;
  }
  item_->setPixmap(QPixmap::fromImage(image));
  view_->scene()->setSceneRect(item_->boundingRect());
  stack->setCurrentWidget(view_);
  view_->fit();
}

} // namespace rux::qt
```

(`#include <QHBoxLayout>` too.) The viewer needs no new link: decoding happens in the library and `rux_qt_lib` already links `opencv_core opencv_imgproc`; only the gallery gains `opencv_core opencv_imgcodecs` (Step 6).

- [ ] **Step 5: The tree and the page**

`ProjectTree::Kind` gains `panorama`. In `ProjectTree::rebuild`, replace the "Panoramaer" line with:

```cpp
  auto *panos = item(nullptr, "Panoramaer", s.panoramic_images.total_count,
                     Kind::table, {}, "panoramic_images");
  try {
    const auto list = db->list_panoramic_images();
    constexpr std::size_t kMaxListed = 500; // the table lists the rest
    for (std::size_t i = 0; i < std::min(list.size(), kMaxListed); ++i) {
      const auto &p = list[i];
      auto *pi = item(panos, qs(p.filename), -1, Kind::panorama,
                      QString::number(p.id), "panoramic_images");
      pi->setFont(0, mono);
    }
  } catch (const std::exception &) {
  }
```

`DatabaseWorkspace`: member `PanoramaViewer *panorama_`, added to `stack_`; in `show_item`:

```cpp
  case ProjectTree::Kind::panorama: {
    stack_->setCurrentWidget(panorama_);
    const int id = key.toInt();
    Selection s;
    s.kind = "Panorama";
    try {
      for (const auto &p : session_.db()->list_panoramic_images())
        if (p.id == id) {
          s.title = QString::fromStdString(p.filename);
          SelectionSection sec{"Panorama", {}, {}};
          sec.rows.push_back({"Id", QString::number(p.id)});
          sec.rows.push_back({"Billede", p.node_id >= 0 ? QString::number(p.node_id)
                                                        : QString("—")});
          sec.rows.push_back({"Pose", p.pose_source == "aligned"
                                          ? QString("Justeret (rux align 360)")
                                          : QString("Fra tidsstempel"),
                              SelectionRow::Style::value});
          if (p.align_inliers >= 0)
            sec.rows.push_back({"Inliers", QString::number(p.align_inliers)});
          s.sections.push_back(sec);
        }
    } catch (const std::exception &) {
    }
    panorama_->show_panorama(session_.path(), id, s.title);
    set_selection(s);
    break;
  }
```

(on `ProjectSession::state_changed`, call `panorama_->clear()`).

- [ ] **Step 6: Gallery page with a synthetic panorama**

`CMakeLists.txt`: gallery `target_link_libraries(... opencv_core opencv_imgcodecs)`. In `shell_pages.cpp` (`#include <opencv2/imgcodecs.hpp>`):

```cpp
/// A panorama in Oversigt. The fixture has none: a synthetic 2:1 grid is
/// written to the COPY (so the shot shows the pan/zoom view, not real data).
QWidget *make_db_panorama(const PageContext &ctx) {
  if (!ctx.project_path.isEmpty()) {
    reusex::ProjectDB db(ctx.project_path.toStdString());
    if (db.panoramic_image_count() == 0) {
      cv::Mat img(1024, 2048, CV_8UC3);
      for (int y = 0; y < img.rows; ++y)
        for (int x = 0; x < img.cols; ++x)
          img.at<cv::Vec3b>(y, x) = {static_cast<uchar>(x / 8), static_cast<uchar>(y / 4),
                                     static_cast<uchar>((x / 64 + y / 64) % 2 ? 200 : 60)};
      std::vector<uchar> jpg;
      cv::imencode(".jpg", img, jpg);
      db.save_panoramic_image("syntetisk_360.jpg", jpg, 0.0, -1);
    }
  }
  auto *shell = make_db(ctx);
  int id = -1;
  if (!ctx.project_path.isEmpty()) {
    reusex::ProjectDB db(ctx.project_path.toStdString(), /*readOnly=*/true);
    const auto list = db.list_panoramic_images();
    if (!list.empty())
      id = list.front().id;
  }
  shell->database()->open_item(ProjectTree::Kind::panorama, QString::number(id));
  return shell;
}
```

Register `db-panorama` ("Oversigt: et panorama i pan/zoom-visning (skriver et syntetisk panorama til projektkopien)"). `PanoramaViewer::loading()` is covered by the gallery's `background_work_in_flight()` wait.

- [ ] **Step 7: Build, shoot, review**

Run: `cmake --build build --target reusex_unit_tests rux rux-qt-gallery && build/tests/reusex_unit_tests "[rux_qt]" && bash .claude/skills/design-studio/scripts/qt_shot.sh --page db-panorama --page db-frames --theme both`
Expected: exit 0. Read: the tree lists the panorama under "Panoramaer"; the centre shows the full 2:1 image fitted on the canvas with the caption and hint bar; the inspector lists Id / Billede / Pose. Fix, re-shoot; `token_lint.py` clean. If a real project with panoramas is at hand, shoot `db-panorama --project <copy>` too (an 8K panorama must appear within a couple of seconds and the window stays responsive while it decodes).

- [ ] **Step 8: Docs and commit**

Add one line to `qt-client.md`'s Database pages list (`db-panorama`) and to `DIRECTION.md`'s 2026-10-13 entry ("Oversigt previews 360° panoramas (flat equirect, pan/zoom)").

```bash
git add apps/rux/qt tests/unit/rux_qt .claude/skills/design-studio/references/qt-client.md docs/DIRECTION.md
git commit -m "qt: preview 360 panoramas in Oversigt (off-thread decode, pan/zoom)"
```

---

## Self-review notes (for the executor)

- **Spec coverage:** §5.1 → Task 7 (+14 docs); §5.2 graph canvas → Tasks 1, 2, 3, 9, 10; edge list → 6, 11; pair panel images → 17; 3D overlay + ΔT → 15, 16, 18; ICP with guess → 19; add/confirm/reject → 4, 20; toolbar Gem/Find kanter/Optimer → 8, 12; view yaw in 3D and `rux render` → 13; §5.3 → 21; §5.4 Qt-free logic → 1–6, 15; §6 DIRECTION entries for P4 → 14 (and 21 appends).
- **Not in this plan (spec §8 or other phases):** the library work of P1–P3; `ruxd` adoption; spherical panoramas.
