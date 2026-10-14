# Rediger correspondence alignment Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** In Rediger, align B to A from ORB feature correspondences between
the two images (found automatically, corrected by hand), refine with the
existing ICP-forfin, and drop the six ΔT spin boxes. Also fix the box
widget's translation bug.

**Architecture:** One new library entry point, `slam/frame_pair_matches.hpp`,
in `reusex_slam`. It reuses the panorama front-end's
`pano_detail::extract_frame_features` (ORB + depth lifting), the
LoopClosure Lowe ratio test, and `geometry::detail::ransac_rigid`. It
returns keypoints, matches and a rigid fit whose `world_delta` uses the
same ΔT convention as `PairOverlay` and `FramePairIcpOptions::initial_guess`.
The Qt client gets three new pieces:

- Qt-free match-set state and geometry in `rux_qt_core` (`match_editor.hpp`).
- A painted `MatchCanvas` widget that shows A | B with the keypoints and
  match lines.
- `PairImages`, rebuilt as the match editor: it runs find, lift and fit off
  the GUI thread and emits `align_ready`. EditWorkspace turns that signal
  into `PairOverlay::set_offset`, exactly as it does for ICP.

**Tech Stack:** C++20, OpenCV features2d (ORB, BFMatcher), Eigen (Umeyama
via `ransac_rigid`), Qt 6 Widgets (QPainter), VTK 9.5 (`vtkBoxWidget2`),
Catch2 v3, the `rux-qt-gallery` screenshot loop.

**Spec:** `docs/superpowers/specs/2026-10-14-rediger-correspondence-alignment-design.md`
(plus `docs/superpowers/specs/2026-10-13-pose-graph-editor-design.md` §5.2
for the ΔT / ICP conventions it builds on). Beads: `reusex-xlg.1` (Task 1),
`reusex-xlg.3` (Task 5), `reusex-xlg.2` (Tasks 2–4 and 6–9), epic `reusex-xlg`.

## Global Constraints

- Matching is classical: OpenCV ORB plus the Lowe ratio test through the
  library's existing front-end. No learned model and no new dependency (C2).
- Matches are working state only. They are discarded when A or B changes.
  No schema change (C4).
- The six ΔT spin boxes are removed. The one-line read-out
  ("Flyttet 12,4 cm · 3,1°"), Nulstil, the keyboard nudges
  (arrows / PgUp / PgDn / [ ]) and the box widget stay (C5).
- `FrameMatchOptions` defaults are `max_features = 3000` and
  `ratio_test = 0.8`.
- `rigid_from_matches` defaults are `inlier_dist_m = 0.05`,
  `iterations = 2000` and `seed = 1`.
- Determinism (STANDARDS §6): the RANSAC seed is fixed, and nothing depends
  on wall-clock time or thread order.
- Errors (missing image, depth or pose) throw `std::runtime_error` naming
  the frame id. An empty result is not an error; it is reported through
  counts or `reason`.
- Keypoints without valid depth are kept and drawn hollow, but never enter
  a fit.
- The public header has no GTSAM and no OpenCV types (STANDARDS §2). It
  includes siblings as `<reusex/...>`.
- Qt UI copy is Danish. Toolbar labels are verbatim: **Find match**, mode
  segments **Vælg / Tilføj / Område**, **Placér B fra match**, chip
  **Ryd område**, refusal "ingen dybde her".
- The read-out format is "38 matches · 24 inliers · 3 manuelle · 2 udeladt".
  The fit status format is "Fra matches: 24 inliers · RMS 2,1 cm".
- Colours come from tokens only: inlier `--color-status-succeeded`, outlier
  `--color-status-failed`, excluded `--color-text-faint`, manual matches
  dashed. `scripts/token_lint.py apps/rux/qt --qt` must pass.
- Gallery: pages `rediger-matches` and `rediger-matches-edit`, both themes,
  1440×900. The existing guards `TOO WIDE`, `OVERLAY TOO SMALL` and
  `STRIP CLIPPED` must pass.
- Builds and tests never exceed 8 jobs (the machine recently had RAM faults):
  - every `cmake --build` gets `-j8`;
  - every `ctest` gets `--parallel 8`;
  - `qt_shot.sh` is always run with `--no-build`, after an explicit `-j8`
    build of `rux-qt-gallery` and `rux`.
- Use `nix develop -c …` for every build, test and commit. Pre-commit
  hooks need the dev shell's config.
- Run `bd` from `/home/mephisto/repos/ReUseX` through `nix develop -c bd …`.
  The worktree shares the main checkout's beads database.
- Real-data checks only ever touch a scratch COPY of
  `~/repos/NewOffice/project.rux`. The source file is never opened read-write.
- SPDX header on every new file (copy from a neighbour).

### The ΔT convention (verified against the code, used by Tasks 2, 3 and 7)

- `PairOverlay::delta()` is `offset_matrix(offset_, pivot_)`
  (`apps/rux/qt/src/core/pair_alignment.cpp`). That is a world-space rigid
  transform applied to B's world cloud: `p ↦ R(p − c) + c + t`, with `c`
  being B's cloud centroid.
- `offset_from_matrix(W, c)` recovers `(t, yaw, pitch, roll)` with
  `offset_matrix(offset_from_matrix(W, c), c) == W`. This holds for any
  rigid `W` with |pitch| < 90° and a translation inside ±`kOffsetMaxM`
  (50 m) after `normalized()`. The pivot only parametrises `W`; it never
  changes it.
- `refine_frame_pair_icp(db, from = B, to = A, {initial_guess = W})` moves
  B's world cloud by its `world_delta` (the seed included). The pair strip
  places B with
  `overlay_->set_offset(offset_from_matrix(world_delta, overlay_->pivot()))`
  (EditWorkspace's `icp_aligned` handler).
- The new `RigidFit::world_delta` is the same kind of matrix:
  `world_delta · T_B · p_b ≈ T_A · p_a`, where `p_a` / `p_b` are points in
  each frame's stored-pose frame. A stored-pose-frame point is
  `local_transform · p_optical`, so `T · p` is exactly where
  `slam::frame_world_points` puts that pixel.
- The Qt side places B with the identical call:
  `overlay_->set_offset(offset_from_matrix(fit.world_delta, overlay_->pivot()))`.
  No pivot correction is needed.
- `rigid_from_matches` fits in A's frame for conditioning. It sets
  `src = T_A⁻¹·T_B·p_b` and `dst = p_a`, and RANSAC gives `M` with
  `dst ≈ M·src`. Then `world_delta = T_A · M · T_A⁻¹`. Proof:
  `T_A p_a ≈ T_A M T_A⁻¹ T_B p_b`.
- An edge added without ICP stays
  `edge_measurement(T_A, T_B, ΔT) = T_A⁻¹·ΔT·T_B` (A → B, source `manual`).
  ICP's `edge_transform` for the same alignment is its inverse (B → A).
  Task 3 pins both relations in a test.

## Review Focus

These are inputs the spec implies but does not state. Each one has a test
in the task that owns its code.

1. **A frame without depth or pose.** A panorama-only node or a frame with
   no stored pose, chosen as A or B, must make Find match say why
   ("billede 2241 har ingen dybde"). It must not crash or show an empty
   canvas silently.
   - Library test: Task 3, `FramePairMatches_FrameWithoutDepthOrPose_ThrowsNamingIt`.
   - Qt: Task 7 catches the error and shows it with tone `crit`.
2. **The pair changes while work runs.** If the user clicks another node,
   presses "Byt A og B" or the project reloads while Find match, a lift or
   Placér B runs, the result must be dropped. It must never colour or move
   the new pair.
   - Core test: Task 4, `MatchSet_SwappedPairIsANewPair_AndClears`.
   - Widget: Task 7, a generation check plus the gallery log.
3. **Three manual matches along a line.** Three hand-added matches on a door
   edge or a skirting board are almost collinear. They must be refused with
   a clear reason instead of spinning B about that line.
   - Test: Task 2, `RigidFromMatches_CollinearPoints_AreRefusedAsDegenerate`.
4. **Depth at lower resolution than colour.** iPad captures store depth at
   192×256 against 720×960 colour. Lifting a keypoint or a click must map
   through the resolution ratio and scale the snap radius with it.
   - Test: Task 3, `LiftPixel_HalfResolutionDepth_MatchesFullResolutionLift`.
5. **A region drawn partly outside the image, or a click-sized drag.** The
   region must be clamped to the image. A drag smaller than a few pixels is
   ignored. Nothing crashes and nothing searches an empty mask by surprise.
   - Core test: Task 4, `BoxFromDrag_ClampsToTheImage_AndIgnoresAClick`.
   - Library test: Task 3, `FramePairMatches_RegionPartlyOutside_KeepsTheIntersection`.

---

## File Structure

| File | Responsibility | Task |
|---|---|---|
| `apps/rux/qt/src/PairOverlay.cpp`, `include/rux_qt/PairOverlay.hpp` | box-widget translate fix + probes (T1); spin boxes removed (T5) | 1, 5 |
| `apps/rux/qt/gallery/shell_pages.cpp` | `rediger-drag-qt` repro/guard page (T1); `rediger-matches`, `rediger-matches-edit` (T8) | 1, 8 |
| `apps/rux/qt/gallery/main.cpp`, `gallery/demo_pages.hpp` | `report_page_failure()` → exit 6 | 1 |
| `.claude/skills/design-studio/scripts/qt_shot.sh` | grep `DRAG NOT TRANSLATED` | 1 |
| `libs/reusex/src/slam/panorama_features.{hpp,cpp}` | `extract_frame_features` takes `const ProjectDB&` + optional region; `depth_pinhole` / `lift_depth_pixel` factored out | 3 |
| `libs/reusex/include/slam/frame_pair_matches.hpp` | public API: `PixelRect`, `FrameMatchOptions`, `FrameKeypoint`, `FrameMatch`, `FramePairMatches`, `RigidFit`, `match_frame_pair`, `lift_pixel`, `rigid_from_matches` | 2, 3 |
| `libs/reusex/src/slam/frame_pair_matches.cpp` | implementation | 2, 3 |
| `tests/unit/slam/test_frame_pair_matches.cpp` | `[slam][matches]` | 2, 3 |
| `apps/rux/qt/include/rux_qt/match_editor.hpp`, `src/core/match_editor.cpp` | Qt-free: `MatchSet`, `ManualPick`, counts/read-out/fit texts, image↔canvas mapping, zoom, hit-testing, drag→box | 4 |
| `tests/unit/rux_qt/test_match_editor.cpp` | `[rux_qt][matches]` | 4 |
| `apps/rux/qt/CMakeLists.txt` | add `src/core/match_editor.cpp` to `rux_qt_core` | 4 |
| `apps/rux/qt/src/EditWorkspace.cpp` | right splitter 1:1 (T5); `align_ready` → overlay (T7) | 5, 7 |
| `apps/rux/qt/styles/app.qss` | drop `#offsetFields`/`#offsetField` (T5); `#matchToolbar`, `#matchStatus`, `#regionChip` (T7) | 5, 7 |
| `apps/rux/qt/include/rux_qt/pair_alignment.hpp` | comment: no more "six fields" | 5 |
| `apps/rux/qt/include/rux_qt/FrameBrowser.hpp`, `src/FrameBrowser.cpp` | `depth_colour_table()` shared with the canvas | 6 |
| `apps/rux/qt/include/rux_qt/MatchCanvas.hpp`, `src/MatchCanvas.cpp` | the painted A \| B canvas + interaction | 6 |
| `apps/rux/qt/include/rux_qt/PairImages.hpp`, `src/PairImages.cpp` | the match editor (toolbar, off-thread find/lift/fit, status) | 7 |
| `apps/rux/qt/src/PairStrip.cpp` | ICP line wording "Fra B's placering" | 7 |
| `.claude/skills/design-studio/references/qt-client.md` | document new pages, removed fields | 5, 8 |

`rux_qt_lib` already links `reusex_slam` PRIVATE (`apps/rux/qt/CMakeLists.txt`),
so `PairImages.cpp` may call the new library API with no CMake change.
`src/slam/` is globbed, so `frame_pair_matches.cpp` needs no CMake edit.
`rux_qt_core` has an explicit source list, so it does need one (Task 4).

---

### Task 0: Worktree build baseline

**Files:** none (creates `build/`, untracked)

**Interfaces:**
- Consumes: nothing
- Produces: a configured `build/` (Ninja, Release, ccache) with a recorded green baseline that later tasks compare against

- [ ] **Step 1: Configure**

Run:
```bash
cd /home/mephisto/repos/ReUseX__worktrees/rediger-matches
nix develop -c cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
```
Expected: `-- Generating done`. ccache is on by default (`USE_CCACHE=ON`).

- [ ] **Step 2: Build everything with at most 8 jobs**

Run:
```bash
nix develop -c cmake --build build -j8
```
Expected: exit 0. This can take a long time on a cold ccache. Do not raise `-j`.

- [ ] **Step 3: Run the suite, record the baseline**

Run:
```bash
nix develop -c ctest --test-dir build --output-on-failure --parallel 8 2>&1 | tee build/baseline-ctest.log | tail -5
```
Expected: `100% tests passed` (GPU-tagged tests may report skipped). If
anything fails, write its name into `build/baseline-ctest.log`'s
neighbour `build/baseline-failures.txt` and treat those exact tests as
pre-existing in later tasks; do not fix them here.

- [ ] **Step 4: Baseline gallery shots of Rediger**

Run:
```bash
nix develop -c cmake --build build -j8 --target rux-qt-gallery rux
nix develop -c bash .claude/skills/design-studio/scripts/qt_shot.sh --no-build --page rediger --page rediger-strip --page rediger-icp --theme dark --out shots/qt/baseline 2>&1 | tee build/baseline-shots.log
```
Expected: three PNG paths, each followed by an `overlay canvas WxH` log
line with H ≥ 250. Note the overlay height for comparison in Task 5.

(No commit: nothing tracked changed.)

---

### Task 1: Box widget translation (closes `reusex-xlg.1`)

The bug is in shipped code. Rotating B with the `vtkBoxWidget2` works on
NewOffice, but translating it does not. An earlier xvfb test drove VTK
events directly (`vtk_drag` in `make_rediger_drag`), so it bypassed
`QVTKOpenGLNativeWidget`'s Qt → VTK event mapping and HiDPI scaling. That
test translated fine. Follow superpowers:systematic-debugging. Reproduce
through the real Qt path, state the root cause, and only then fix.

**Files:**
- Modify: `apps/rux/qt/include/rux_qt/PairOverlay.hpp` (probe accessors)
- Modify: `apps/rux/qt/src/PairOverlay.cpp` (probes; then the fix)
- Modify: `apps/rux/qt/gallery/shell_pages.cpp` (page `rediger-drag-qt`)
- Modify: `apps/rux/qt/gallery/demo_pages.hpp`, `apps/rux/qt/gallery/main.cpp` (`report_page_failure`)
- Modify: `.claude/skills/design-studio/scripts/qt_shot.sh` (grep `DRAG NOT TRANSLATED`)

**Interfaces:**
- Consumes: `PairOverlay` (existing), `make_pg`, `settle` (gallery)
- Produces:
  - `std::optional<QPointF> PairOverlay::box_centre_px() const` gives the box centre in the canvas' logical widget coordinates.
  - `double PairOverlay::handle_radius_px() const` gives the centre handle's projected radius in logical px, or 0 without a box.
  - `int PairOverlay::box_state() const` returns `vtkBoxRepresentation::GetInteractionState()`, or -1.
  - `void rux::qt::gallery::report_page_failure()` makes the gallery exit 6.

- [ ] **Step 1: Claim the bead**

```bash
cd /home/mephisto/repos/ReUseX && nix develop -c bd update reusex-xlg.1 --claim
```

- [ ] **Step 2: Read the widget ↔ ΔT path before touching anything**

Read these in `apps/rux/qt/src/PairOverlay.cpp`:

- `RigidBoxRepresentation` (face handles hidden, centre handle kept);
- `on_box` (`rep->GetTransform` → `widget_moved`);
- `widget_moved` (`offset_from_matrix(m, pivot_)` → `normalized` → the clamp
  check);
- `apply_offset` (it sets B's user matrix and pushes the transform back into
  the rep unless `from_widget`);
- `loaded` (`PlaceWidget(B bounds)`, `SetTranslationEnabled(true)`).

Then read `apps/rux/qt/src/SceneView.cpp` around the
`QVTKOpenGLNativeWidget` creation (trackball style, no HiDPI setting
touched) and `make_rediger_drag` in `gallery/shell_pages.cpp` (VTK-level
`vtk_drag`).

Write down the four candidate causes:

- H1: the centre handle (the only translate grip) is a few display pixels
  wide, and smaller still in logical pixels at dpr 2. Faces are large, so
  rotation is easy to hit.
- H2: Qt → VTK event coordinates are mis-scaled at dpr ≠ 1, so a press on
  the handle lands elsewhere.
- H3: the handle is drawn inside B's dense cloud and hidden by it, so the
  user never finds it.
- H4: a mapping error in `offset_from_matrix` or the pivot for pure
  translations.

- [ ] **Step 3: Add the probes to `PairOverlay`**

In `PairOverlay.hpp`, inside `public:` after `widget_moved`:

```cpp
  /// Test probes for the gallery's Qt-level drag (rediger-drag-qt): the
  /// box centre in the GL widget's logical coordinates (nullopt without
  /// a box), the centre handle's projected radius in logical px (0 without
  /// a box), and the box representation's interaction state (-1 without).
  std::optional<QPointF> box_centre_px() const;
  double handle_radius_px() const;
  int box_state() const;
```
and add `#include <QPointF>` and `#include <optional>`.

In `PairOverlay.cpp`, expose the handle radius on the representation (add
inside `class RigidBoxRepresentation`, public section):

```cpp
  /// World radius of the centre (translate) handle.
  double centre_handle_radius() const {
    return this->HandleGeometry[6]->GetRadius();
  }
  const double *centre() const { return this->Points->GetPoint(14); }
```
and add `#include <vtkPoints.h>` / `#include <vtkSphereSource.h>` (the
former is already included). Then, at the end of the file before the
namespace closes:

```cpp
namespace {
/// World point -> logical widget px of the overlay canvas (VTK's display
/// origin is bottom-left, in device pixels).
std::optional<QPointF> to_logical(SceneView *view, const double w[3]) {
  vtkRenderer *r = view->renderer();
  if (!r || !r->GetRenderWindow())
    return std::nullopt;
  r->SetWorldPoint(w[0], w[1], w[2], 1.0);
  r->WorldToDisplay();
  double d[3];
  r->GetDisplayPoint(d);
  const double dpr = view->devicePixelRatioF();
  const int h = r->GetRenderWindow()->GetSize()[1];
  return QPointF(d[0] / dpr, (h - d[1]) / dpr);
}
} // namespace

std::optional<QPointF> PairOverlay::box_centre_px() const {
  if (!vtk_->box)
    return std::nullopt;
  auto *rep = static_cast<RigidBoxRepresentation *>(
      vtk_->box->GetRepresentation());
  const double *c = rep->centre();
  const double w[3] = {c[0], c[1], c[2]};
  return to_logical(view_, w);
}

double PairOverlay::handle_radius_px() const {
  if (!vtk_->box)
    return 0.0;
  auto *rep = static_cast<RigidBoxRepresentation *>(
      vtk_->box->GetRepresentation());
  const double *c = rep->centre();
  double up[3];
  view_->renderer()->GetActiveCamera()->GetViewUp(up);
  const double r = rep->centre_handle_radius();
  const double a[3] = {c[0], c[1], c[2]};
  const double b[3] = {c[0] + up[0] * r, c[1] + up[1] * r, c[2] + up[2] * r};
  const auto pa = to_logical(view_, a), pb = to_logical(view_, b);
  if (!pa || !pb)
    return 0.0;
  return std::hypot(pb->x() - pa->x(), pb->y() - pa->y());
}

int PairOverlay::box_state() const {
  if (!vtk_->box)
    return -1;
  return vtk_->box->GetRepresentation()->GetInteractionState();
}
```
(`#include <vtkCamera.h>`, `#include <vtkRenderWindow.h>`.) `to_logical`
reads `view->devicePixelRatioF()`. Step 5 logs it next to the GL widget's
`effectiveDevicePixelRatio()` and the render-window size, so a mismatch
between them (H2) shows up in the log.

- [ ] **Step 4: Let a page fail the gallery**

In `gallery/demo_pages.hpp`, inside `namespace rux::qt::gallery`:

```cpp
/// A page's own check failed (logged as "ERROR …" by the page): the
/// gallery still writes the PNG, then exits 6 like a layout guard.
void report_page_failure();
bool page_failed();
```
In `gallery/main.cpp` (namespace scope, same namespace):

```cpp
namespace {
bool g_page_failed = false;
}
void report_page_failure() { g_page_failed = true; }
bool page_failed() { return g_page_failed; }
```
and change `if (too_wide || too_small || strip_cut) return 6;` to
`if (too_wide || too_small || strip_cut || page_failed()) return 6;`.
In `qt_shot.sh`, extend both grep patterns `TOO WIDE|OVERLAY TOO SMALL|STRIP CLIPPED`
to `TOO WIDE|OVERLAY TOO SMALL|STRIP CLIPPED|DRAG NOT TRANSLATED`.

- [ ] **Step 5: Write the Qt-level repro page `rediger-drag-qt`**

In `gallery/shell_pages.cpp` add `#include <QVTKOpenGLNativeWidget.h>`,
`#include <QMouseEvent>`, `#include <vtkRenderWindow.h>` and
`#include <functional>` (skip any already there), then after
`make_rediger_drag`:

```cpp
namespace {
/// One left-button drag sent to @p w as Qt mouse events (logical px), the
/// way a real mouse reaches QVTKOpenGLNativeWidget — its interactor adapter
/// and HiDPI mapping are part of the path, unlike vtk_drag.
void qt_drag(QWidget *w, QPointF from, QPointF to,
             const std::function<void()> &after_press = {}) {
  const auto send = [w](QEvent::Type type, QPointF p, Qt::MouseButton b,
                        Qt::MouseButtons held) {
    QMouseEvent e(type, p, w->mapToGlobal(p), b, held, Qt::NoModifier);
    QApplication::sendEvent(w, &e);
  };
  send(QEvent::MouseMove, from, Qt::NoButton, Qt::NoButton);
  send(QEvent::MouseButtonPress, from, Qt::LeftButton, Qt::LeftButton);
  if (after_press)
    after_press();
  constexpr int kSteps = 8;
  for (int i = 1; i <= kSteps; ++i)
    send(QEvent::MouseMove, from + (to - from) * (double(i) / kSteps),
         Qt::NoButton, Qt::LeftButton);
  send(QEvent::MouseButtonRelease, to, Qt::LeftButton, Qt::NoButton);
  QApplication::processEvents();
}

const char *box_state_name(int s) {
  // vtkBoxRepresentation::InteractionStateType
  static const char *names[] = {"Outside", "MoveF0", "MoveF1",
                                "MoveF2",  "MoveF3", "MoveF4",
                                "MoveF5",  "Translating", "Rotating",
                                "Scaling"};
  return s >= 0 && s < 10 ? names[s] : "none";
}
} // namespace

/// Drag B's centre handle with Qt mouse events (needs --gl), once exactly
/// on the handle and once 6 logical px beside it (a realistic aim). Logs
/// dpr, sizes, the handle radius, the state at the press and the ΔT; fails
/// the shot (DRAG NOT TRANSLATED) when the on-handle drag does not move B
/// by >= 1 cm without turning it. Shoot with --gl --scale 1 and --scale 2.
QWidget *make_rediger_drag_qt(const PageContext &ctx) {
  auto *shell = make_pg(ctx);
  shell->show();
  settle();
  PairOverlay *ov = shell->edit()->overlay();
  auto *gl = ov->view()->findChild<QVTKOpenGLNativeWidget *>();
  const auto centre = ov->box_centre_px();
  if (!gl || !ov->ready() || !centre) {
    std::fprintf(stderr, "rediger-drag-qt: needs --gl and both clouds\n");
    return shell;
  }
  const int *win = gl->renderWindow()->GetSize();
  std::fprintf(stderr,
               "rediger-drag-qt: dpr view=%.2f gl=%.2f gl-size=%dx%d "
               "window=%dx%d handle=%.1f px centre=(%.1f,%.1f)\n",
               ov->view()->devicePixelRatioF(),
               gl->effectiveDevicePixelRatio(), gl->width(), gl->height(),
               win[0], win[1], ov->handle_radius_px(), centre->x(),
               centre->y());
  // box_centre_px() is in the render window's logical coordinates, i.e.
  // the GL widget's (it fills the canvas).
  const QPointF origin = *centre;
  const double step = 2.0 * theme().px("--space-7");
  bool failed = false;
  for (const double aim : {0.0, 6.0}) {
    ov->reset_offset();
    QApplication::processEvents();
    const QPointF from = origin + QPointF(aim, 0.0);
    int state = -1;
    qt_drag(gl, from, from + QPointF(step, 0.0),
            [&] { state = ov->box_state(); });
    const ManualOffset &o = ov->offset();
    const double moved = std::sqrt(o.x * o.x + o.y * o.y + o.z * o.z);
    const double turned = rotation_angle_deg(ov->delta());
    std::fprintf(stderr,
                 "rediger-drag-qt: aim +%.0f px state=%s -> moved %.4f m "
                 "turned %.3f deg (x=%.4f y=%.4f z=%.4f)\n",
                 aim, box_state_name(state), moved, turned, o.x, o.y, o.z);
    if (aim == 0.0 && (moved < 0.01 || turned > 0.5))
      failed = true;
  }
  if (failed) {
    std::fprintf(stderr, "rux-qt-gallery: ERROR DRAG NOT TRANSLATED: a drag "
                         "on the centre handle did not move B\n");
    report_page_failure();
  }
  ov->view()->request_render();
  return shell;
}
```
Register it next to `rediger-drag`:

```cpp
  register_page({"rediger-drag-qt",
                 "Rediger: midterhåndtaget trukket med Qt-musehændelser "
                 "(kræver --gl)",
                 make_rediger_drag_qt});
```

- [ ] **Step 6: Build and reproduce at dpr 1 and 2, fixture and NewOffice**

```bash
nix develop -c cmake --build build -j8 --target rux-qt-gallery rux
S=.claude/skills/design-studio/scripts/qt_shot.sh
nix develop -c bash $S --no-build --gl --scale 1 --page rediger-drag-qt --theme dark --out shots/qt/drag 2>&1 | tee /tmp/drag-s1.log
nix develop -c bash $S --no-build --gl --scale 2 --page rediger-drag-qt --theme dark --out shots/qt/drag 2>&1 | tee /tmp/drag-s2.log
```
Repeat both on a NewOffice copy. Make the copy once and reuse it for the
whole plan:

```bash
SCRATCH=<your session scratchpad directory>   # never /tmp of the source
mkdir -p $SCRATCH && cp ~/repos/NewOffice/project.rux $SCRATCH/newoffice.rux
ls ~/repos/NewOffice/project.rux-wal 2>/dev/null && cp ~/repos/NewOffice/project.rux-wal $SCRATCH/newoffice.rux-wal
```
Then run `qt_shot.sh` with `--project $SCRATCH/newoffice.rux`. qt_shot
copies it again per shot. Set `RUX_ROUNDTRIP_A` only if `make_pg`'s pair
has no depth. Expected: one `rediger-drag-qt: dpr …` line and two
`aim …` lines per run. Read the PNGs.

- [ ] **Step 7: State the root cause (before any fix)**

Decide from the logs:

| Observation | Root cause |
|---|---|
| `handle` < ~4 px at scale 2 (or on NewOffice) and the `aim +6` press reports `Outside`/`Rotating`, while `aim +0` reports `Translating` | **H1**: the translate grip is too small to hit |
| `aim +0` reports anything but `Translating` at scale 2 but `Translating` at scale 1, while `handle` ≥ 6 px; or `window` ≠ `gl-size × gl dpr` | **H2**: Qt → VTK event mapping |
| `aim +0` and `+6` both translate at both scales and `handle` ≥ 6 px, but the PNG shows the handle sphere hidden inside B's points | **H3**: the grip is invisible |
| `Translating` at the press but `moved` < 1 cm or `turned` > 0.5° | **H4**: the ΔT mapping |

Record the finding:
```bash
cd /home/mephisto/repos/ReUseX && nix develop -c bd update reusex-xlg.1 --notes "Root cause: <H1/H2/H3/H4 + the decisive log lines>"
```
If no row matches, stop here. Report the logs to the caller; do not guess
a fix.

- [ ] **Step 8: Fix the confirmed cause**

**H1, the grip size.** Size every handle in logical pixels instead of
VTK's default. In `RigidBoxRepresentation` (public):

```cpp
  /// Handle radius in DISPLAY pixels (logical px x the canvas' dpr); 0
  /// keeps VTK's own sizing.
  void set_handle_display_px(double px) { handle_px_ = px; }
```
(protected):
```cpp
  void SizeHandles() override {
    Superclass::SizeHandles();
    vtkRenderer *r = this->Renderer;
    if (handle_px_ <= 0.0 || !r || !r->GetActiveCamera())
      return;
    // The world length of handle_px_ display pixels at the box centre.
    double c[3];
    this->Points->GetPoint(14, c);
    r->SetWorldPoint(c[0], c[1], c[2], 1.0);
    r->WorldToDisplay();
    double d[3];
    r->GetDisplayPoint(d);
    r->SetDisplayPoint(d[0] + handle_px_, d[1], d[2]);
    r->DisplayToWorld();
    double w[4];
    r->GetWorldPoint(w);
    if (w[3] != 0.0)
      for (int i = 0; i < 3; ++i)
        w[i] /= w[3];
    const double radius = std::sqrt((w[0] - c[0]) * (w[0] - c[0]) +
                                    (w[1] - c[1]) * (w[1] - c[1]) +
                                    (w[2] - c[2]) * (w[2] - c[2]));
    for (int i = 0; i < 7; ++i)
      this->HandleGeometry[i]->SetRadius(radius);
  }
  double handle_px_ = 0.0;
```
In `PairOverlay::loaded`, before `vtk_->box->SetRepresentation(rep)`:
```cpp
    // The centre sphere is the only translate grip: --space-2 logical px
    // radius on any screen, so a HiDPI display does not shrink it to a
    // speck while the faces (rotate) stay easy to hit.
    rep->set_handle_display_px(theme().px("--space-2") *
                               view_->devicePixelRatioF());
```

**H2, the event mapping.** In `SceneView`'s interactive branch, right
after `impl_->gl = new QVTKOpenGLNativeWidget;`:
```cpp
    // Events and the render window must agree on the pixel ratio: VTK
    // sizes the window in device px and scales events by the same ratio.
    impl_->gl->setEnableHiDPI(true);
```
If the log showed the window in logical px, set the ratio instead:
`impl_->gl->setCustomDevicePixelRatio(devicePixelRatioF())`. Re-run Step 6
until `window == gl-size × dpr`.

**H3, an invisible grip.** Draw the widget in its own layer above the
clouds. In `PairOverlay::loaded`, before `vtk_->box->On()`:
```cpp
    // The box and its handles in a layer above the clouds, sharing the
    // camera, so B's points never hide the grip.
    vtkRenderWindow *rw = view_->renderer()->GetRenderWindow();
    vtk_->top = vtkSmartPointer<vtkRenderer>::New();
    vtk_->top->SetLayer(1);
    vtk_->top->InteractiveOff();
    vtk_->top->SetActiveCamera(view_->renderer()->GetActiveCamera());
    rw->SetNumberOfLayers(2);
    rw->AddRenderer(vtk_->top);
    vtk_->box->SetCurrentRenderer(vtk_->top);
```
Add `vtkSmartPointer<vtkRenderer> top;` to `struct PairOverlay::Vtk`. In
`load()`, after `vtk_->box = nullptr;`:
```cpp
  if (vtk_->top) {
    vtk_->top->GetRenderWindow()->RemoveRenderer(vtk_->top);
    vtk_->top = nullptr;
  }
```

**H4, the ΔT mapping.** Add a failing case to
`tests/unit/rux_qt/test_pair_alignment.cpp` first. Feed `widget_moved`'s
exact input (a pure translation `m` about a far pivot) through
`offset_from_matrix` / `normalized` and assert `offset_matrix(o, pivot) == m`.
Then fix `offset_from_matrix` or the clamp check in `widget_moved`.

In every case, update the status tooltip in `apply_offset` so users can
discover the grips. Prepend
`"Træk kuglen i midten for at flytte B; træk en side for at dreje.\n"` to
both tooltip branches.

- [ ] **Step 9: Verify the fix**

```bash
nix develop -c cmake --build build -j8 --target rux-qt-gallery rux reusex_unit_tests
nix develop -c bash $S --no-build --gl --scale 2 --page rediger-drag-qt --page rediger-drag --theme dark --out shots/qt/drag
nix develop -c bash $S --no-build --gl --scale 2 --page rediger-drag-qt --theme dark --project $SCRATCH/newoffice.rux --out shots/qt/drag
nix develop -c bash $S --no-build --page rediger --page rediger-strip --page rediger-icp --theme both
nix develop -c ctest --test-dir build -R "rux_qt|pair" --output-on-failure --parallel 8
```
Expected:
- no `DRAG NOT TRANSLATED`;
- `aim +0` and, for H1, `aim +6` both report `Translating` with
  `moved ≥ 0.01`, `turned ≤ 0.5`;
- the existing pages still pass their guards;
- the tests pass.

- [ ] **Step 10: Commit**

```bash
nix develop -c git add apps/rux/qt/include/rux_qt/PairOverlay.hpp apps/rux/qt/src/PairOverlay.cpp apps/rux/qt/src/SceneView.cpp apps/rux/qt/gallery .claude/skills/design-studio/scripts/qt_shot.sh tests/unit/rux_qt/test_pair_alignment.cpp
nix develop -c git commit -m "qt: B's box widget translates again (reusex-xlg.1)

Root cause: <one line from Step 7>. rediger-drag-qt drives the centre
handle through QVTKOpenGLNativeWidget with Qt mouse events at any dpr and
fails the shot (DRAG NOT TRANSLATED) when B does not move."
```
(Stage only the files you actually changed.)

- [ ] **Step 11: Ask the user to confirm by hand, then close**

The agent cannot drag a real mouse. Tell the caller to ask the user:

> The box widget's translation is fixed (cause: …). Please check it by
> hand. Run `rux -p <a copy of NewOffice>`, open Rediger, pick 2240 / 2241,
> drag the round handle in the middle of B's box, and say whether B
> follows the mouse.

Close only after the user confirms:
```bash
cd /home/mephisto/repos/ReUseX && nix develop -c bd close reusex-xlg.1 --reason "Fixed: <root cause>; confirmed by hand"
```
If the confirmation has not come yet, leave the bead `in_progress` and
continue with Task 2. The remaining tasks do not depend on it.

---

### Task 2: `rigid_from_matches` (pure fit, no database)

**Files:**
- Create: `libs/reusex/include/slam/frame_pair_matches.hpp`
- Create: `libs/reusex/src/slam/frame_pair_matches.cpp`
- Create: `tests/unit/slam/test_frame_pair_matches.cpp`

**Interfaces:**
- Consumes: `reusex::geometry::detail::ransac_rigid(const Eigen::Matrix3Xd&, const Eigen::Matrix3Xd&, const LoopClosureOptions&, std::mt19937&, Eigen::Matrix4d&)` (`slam/LoopClosure.hpp`)
- Produces (namespace `reusex::slam`, header `<reusex/slam/frame_pair_matches.hpp>`):
  ```cpp
  using Point3 = std::array<double, 3>;
  using PointPair = std::pair<Point3, Point3>;          // (in A's frame, in B's frame)
  enum class RigidFitFailure { none, too_few_pairs, no_consensus, degenerate };
  struct RigidFit { bool ok; std::array<double,16> world_delta; std::vector<int> inliers;
                    double rms_m; std::string reason; RigidFitFailure failure; };
  inline constexpr double kMinFitSpreadM = 0.01;
  RigidFit rigid_from_matches(const std::array<double,16> &pose_a, const std::array<double,16> &pose_b,
                              const std::vector<PointPair> &pairs, double inlier_dist_m = 0.05,
                              int iterations = 2000, std::uint32_t seed = 1);
  ```

- [ ] **Step 1: Claim the bead**

```bash
cd /home/mephisto/repos/ReUseX && nix develop -c bd update reusex-xlg.2 --claim
```

- [ ] **Step 2: Write the failing tests**

Create `tests/unit/slam/test_frame_pair_matches.cpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

// slam::frame_pair_matches (spec 2026-10-14 §3): the correspondence
// alignment behind Rediger's match editor. rigid_from_matches is tested on
// synthetic point pairs; match_frame_pair / lift_pixel on a real ProjectDB
// with synthetic frames (below the first section).

#include <catch2/catch_test_macros.hpp>

#include <reusex/slam/frame_pair_matches.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <random>
#include <vector>

using reusex::slam::PointPair;
using reusex::slam::RigidFitFailure;
using reusex::slam::rigid_from_matches;

namespace {

using RowMajor4d = Eigen::Matrix<double, 4, 4, Eigen::RowMajor>;

Eigen::Matrix4d M(const std::array<double, 16> &a) {
  return Eigen::Map<const RowMajor4d>(a.data());
}

std::array<double, 16> A(const Eigen::Matrix4d &m) {
  std::array<double, 16> a{};
  Eigen::Map<RowMajor4d>(a.data()) = m;
  return a;
}

/// Row-major pose: translate(tx, ty, tz) * Rz(yaw_deg).
std::array<double, 16> pose(double tx, double ty, double tz, double yaw_deg) {
  Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
  m.topLeftCorner<3, 3>() =
      Eigen::AngleAxisd(yaw_deg * M_PI / 180.0, Eigen::Vector3d::UnitZ())
          .toRotationMatrix();
  m.topRightCorner<3, 1>() = Eigen::Vector3d(tx, ty, tz);
  return A(m);
}

double translation_error(const std::array<double, 16> &a,
                         const Eigen::Matrix4d &b) {
  return (M(a).topRightCorner<3, 1>() - b.topRightCorner<3, 1>()).norm();
}

double angle_error_deg(const std::array<double, 16> &a,
                       const Eigen::Matrix4d &b) {
  const Eigen::Matrix3d r =
      M(a).topLeftCorner<3, 3>().transpose() * b.topLeftCorner<3, 3>();
  return std::acos(std::clamp((r.trace() - 1.0) / 2.0, -1.0, 1.0)) * 180.0 /
         M_PI;
}

reusex::slam::Point3 P(const Eigen::Vector3d &v) { return {v.x(), v.y(), v.z()}; }

/// n points in a 2 m box in A's frame, and the same world points in B's
/// frame when the TRUE world correction applied to B is `delta`:
/// T_A p_a = delta T_B p_b  =>  p_b = (delta T_B)^-1 T_A p_a.
std::vector<PointPair> synthetic_pairs(const std::array<double, 16> &pa,
                                       const std::array<double, 16> &pb,
                                       const Eigen::Matrix4d &delta, int n,
                                       unsigned seed) {
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> u(-1.0, 1.0);
  const Eigen::Matrix4d b_from_a = (delta * M(pb)).inverse() * M(pa);
  std::vector<PointPair> out;
  for (int k = 0; k < n; ++k) {
    const Eigen::Vector3d p(u(rng), u(rng), 1.5 + 0.5 * u(rng));
    const Eigen::Vector3d q = (b_from_a * p.homogeneous()).head<3>();
    out.emplace_back(P(p), P(q));
  }
  return out;
}

const std::array<double, 16> kPoseA = pose(1.0, 2.0, 0.0, 10.0);
const std::array<double, 16> kPoseB = pose(1.3, 2.1, 0.05, 16.0);

Eigen::Matrix4d true_delta() {
  return M(pose(0.12, -0.05, 0.02, 3.0)); // the drift to undo
}

} // namespace

TEST_CASE("RigidFromMatches_ExactPairs_RecoverTheWorldDelta",
          "[slam][matches]") {
  const auto pairs = synthetic_pairs(kPoseA, kPoseB, true_delta(), 40, 3);
  const auto fit = rigid_from_matches(kPoseA, kPoseB, pairs);
  REQUIRE(fit.ok);
  CHECK(fit.failure == RigidFitFailure::none);
  CHECK(fit.inliers.size() == pairs.size());
  CHECK(fit.rms_m < 1e-9);
  CHECK(translation_error(fit.world_delta, true_delta()) < 1e-6);
  CHECK(angle_error_deg(fit.world_delta, true_delta()) < 1e-6);
  // The convention, literally: world_delta * T_B * p_b == T_A * p_a.
  for (const auto &[pa, pb] : pairs) {
    const Eigen::Vector3d lhs =
        (M(fit.world_delta) * M(kPoseB) *
         Eigen::Vector3d(pb[0], pb[1], pb[2]).homogeneous())
            .head<3>();
    const Eigen::Vector3d rhs =
        (M(kPoseA) * Eigen::Vector3d(pa[0], pa[1], pa[2]).homogeneous())
            .head<3>();
    CHECK((lhs - rhs).norm() < 1e-9);
  }
}

TEST_CASE("RigidFromMatches_InjectedOutliers_AreRejected", "[slam][matches]") {
  auto pairs = synthetic_pairs(kPoseA, kPoseB, true_delta(), 30, 5);
  std::mt19937 rng(9);
  std::uniform_real_distribution<double> u(0.5, 1.5);
  for (int k = 0; k < 8; ++k) { // pairs 30..37 point 0.5–1.5 m off
    auto bad = pairs[static_cast<std::size_t>(k)];
    bad.second[0] += u(rng);
    bad.second[1] -= u(rng);
    pairs.push_back(bad);
  }
  const auto fit = rigid_from_matches(kPoseA, kPoseB, pairs);
  REQUIRE(fit.ok);
  CHECK(fit.inliers.size() == 30);
  for (int i : fit.inliers)
    CHECK(i < 30);
  CHECK(translation_error(fit.world_delta, true_delta()) < 1e-6);
}

TEST_CASE("RigidFromMatches_FewerThanThreePairs_IsRefusedWithAReason",
          "[slam][matches]") {
  const auto pairs = synthetic_pairs(kPoseA, kPoseB, true_delta(), 2, 1);
  const auto fit = rigid_from_matches(kPoseA, kPoseB, pairs);
  CHECK_FALSE(fit.ok);
  CHECK(fit.failure == RigidFitFailure::too_few_pairs);
  CHECK(fit.reason == "3 matches with depth needed, 2 given");
}

TEST_CASE("RigidFromMatches_CollinearPoints_AreRefusedAsDegenerate",
          "[slam][matches]") {
  // Five points on a door edge: a rotation about that line is free.
  std::vector<PointPair> pairs;
  const Eigen::Matrix4d b_from_a =
      (true_delta() * M(kPoseB)).inverse() * M(kPoseA);
  for (int k = 0; k < 5; ++k) {
    const Eigen::Vector3d p(0.3, -0.8 + 0.4 * k, 1.6);
    pairs.emplace_back(P(p), P((b_from_a * p.homogeneous()).head<3>()));
  }
  const auto fit = rigid_from_matches(kPoseA, kPoseB, pairs);
  CHECK_FALSE(fit.ok);
  CHECK(fit.failure == RigidFitFailure::degenerate);
  CHECK(fit.reason.find("collinear") != std::string::npos);
}

TEST_CASE("RigidFromMatches_NoConsensus_IsRefused", "[slam][matches]") {
  // Every pair disagrees with every other by metres.
  std::vector<PointPair> pairs;
  for (int k = 0; k < 6; ++k)
    pairs.push_back({{0.0 + k, 0.0, 1.0}, {5.0 * k * k, -3.0 * k, 9.0 - k}});
  const auto fit = rigid_from_matches(kPoseA, kPoseB, pairs);
  CHECK_FALSE(fit.ok);
  CHECK(fit.failure == RigidFitFailure::no_consensus);
}

TEST_CASE("RigidFromMatches_FixedSeed_IsDeterministic", "[slam][matches]") {
  auto pairs = synthetic_pairs(kPoseA, kPoseB, true_delta(), 25, 11);
  pairs.push_back({{0, 0, 1}, {3, 3, 3}});
  pairs.push_back({{1, 0, 1}, {-3, 2, 0}});
  const auto a = rigid_from_matches(kPoseA, kPoseB, pairs, 0.05, 500, 7);
  const auto b = rigid_from_matches(kPoseA, kPoseB, pairs, 0.05, 500, 7);
  CHECK(a.world_delta == b.world_delta);
  CHECK(a.inliers == b.inliers);
  CHECK(a.rms_m == b.rms_m);
}
```

- [ ] **Step 3: Write the header (all of the spec's API, so Task 3 only adds code)**

Create `libs/reusex/include/slam/frame_pair_matches.hpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Feature correspondences between two stored sensor frames and the rigid
// alignment they give (spec 2026-10-14, Rediger's match editor): ORB
// keypoints matched with a Lowe ratio test — the loop-closure front-end's
// matcher — lifted to 3D through the stored depth, and a RANSAC rigid fit
// of B onto A. Read-only on the project; GTSAM- and OpenCV-free interface.
//
// Frames: a keypoint's xyz is in its frame's STORED-POSE frame
// (sensor_frames.transform's frame: local_transform · optical point), so
// T · xyz is exactly where slam::frame_world_points puts that pixel.
//
// RigidFit::world_delta is a WORLD-space correction applied to B:
//   world_delta · T_B · p_b ≈ T_A · p_a
// — the ΔT the Qt pair overlay applies to B's cloud and the
// FramePairIcpOptions::initial_guess ICP-forfin accepts (refine B onto A).

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace reusex {
class ProjectDB;
}

namespace reusex::slam {

/// A pixel rectangle in a frame's colour image; w or h <= 0 = whole image.
struct PixelRect {
  int x = 0, y = 0, w = 0, h = 0;
};

struct FrameMatchOptions {
  int max_features = 3000; ///< ORB features per frame (LoopClosureOptions')
  double ratio_test = 0.8; ///< Lowe ratio: best / second-best distance
  /// Keypoints are lifted only within [min_depth_m, max_depth_m]
  /// (LoopClosureOptions' range); outside, they are kept without xyz.
  float min_depth_m = 0.3f;
  float max_depth_m = 5.0f;
  /// Detect only inside these colour-image rectangles (clamped to the
  /// image); nullopt = the whole image.
  std::optional<PixelRect> region_a, region_b;
};

struct FrameKeypoint {
  float u = 0, v = 0; ///< pixel in the stored colour image
  /// Stored-pose-frame 3D point; nullopt = no valid depth (drawn hollow,
  /// never fitted).
  std::optional<std::array<double, 3>> xyz;
};

/// One correspondence: indices into keypoints_a / keypoints_b.
struct FrameMatch {
  int a = -1, b = -1;
  float distance = 0; ///< Hamming distance of the ORB descriptors
};

struct FramePairMatches {
  std::vector<FrameKeypoint> keypoints_a, keypoints_b;
  std::vector<FrameMatch> matches; ///< in B's keypoint order
};

/// ORB keypoints of both frames and their ratio-test matches (B queried
/// against A). Deterministic for a given project and options.
/// @throws std::invalid_argument when @p a_id == @p b_id.
/// @throws std::runtime_error naming the frame when it has no colour image,
///         no depth image, no stored pose, or unusable intrinsics.
FramePairMatches match_frame_pair(const ProjectDB &db, int a_id, int b_id,
                                  const FrameMatchOptions &opts = {});

/// A user-added correspondence end: the colour pixel (@p u, @p v) lifted
/// through the frame's depth, snapping to the nearest valid depth pixel
/// within @p snap_px colour pixels (scaled to the depth grid). The result's
/// u, v are the snapped pixel's position in colour coordinates.
/// @returns nullopt when no depth in range lies within the snap radius, or
///          the pixel is outside the image.
/// @throws std::runtime_error naming the frame when it has no colour or no
///         depth image or unusable intrinsics.
std::optional<FrameKeypoint> lift_pixel(const ProjectDB &db, int frame_id,
                                        float u, float v, int snap_px = 4,
                                        float min_depth_m = 0.3f,
                                        float max_depth_m = 5.0f);

using Point3 = std::array<double, 3>;
/// (point in A's stored-pose frame, point in B's stored-pose frame).
using PointPair = std::pair<Point3, Point3>;

enum class RigidFitFailure {
  none,
  too_few_pairs, ///< fewer than 3 pairs given
  no_consensus,  ///< fewer than 3 pairs agree within inlier_dist_m
  degenerate,    ///< the inliers are (nearly) collinear
};

/// Inliers spread less than this (m) off their best-fit line are collinear.
inline constexpr double kMinFitSpreadM = 0.01;

struct RigidFit {
  bool ok = false;
  /// World-space correction applied to B (row-major 4x4), see the header.
  std::array<double, 16> world_delta{1, 0, 0, 0, 0, 1, 0, 0,
                                     0, 0, 1, 0, 0, 0, 0, 1};
  std::vector<int> inliers; ///< indices into the `pairs` argument, ascending
  double rms_m = 0;         ///< RMS 3D residual over the inliers
  std::string reason;       ///< when !ok, e.g. "3 matches with depth needed, 2 given"
  RigidFitFailure failure = RigidFitFailure::none;
};

/// RANSAC rigid fit of B onto A from 3D correspondences (fixed @p seed:
/// deterministic, STANDARDS §6). @p pose_a / @p pose_b are the frames'
/// current stored poses (row-major).
/// @throws std::invalid_argument when a pair has a non-finite coordinate.
RigidFit rigid_from_matches(const std::array<double, 16> &pose_a,
                            const std::array<double, 16> &pose_b,
                            const std::vector<PointPair> &pairs,
                            double inlier_dist_m = 0.05, int iterations = 2000,
                            std::uint32_t seed = 1);

} // namespace reusex::slam
```

`PointPair` is exactly the spec's
`std::pair<std::array<double,3>, std::array<double,3>>`. The alias only
names it.

- [ ] **Step 4: Run the tests to verify they fail**

```bash
nix develop -c cmake --build build -j8 --target reusex_unit_tests
```
Expected: the build FAILS at link time with
`undefined reference to reusex::slam::rigid_from_matches`.

- [ ] **Step 5: Implement `rigid_from_matches`**

Create `libs/reusex/src/slam/frame_pair_matches.cpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

// Rediger's correspondence alignment (spec 2026-10-14 §3). The fit reuses
// LoopClosure's RANSAC 3D-3D (detail::ransac_rigid); matching reuses the
// panorama front-end's ORB + depth lifting (pano_detail).

#include "slam/frame_pair_matches.hpp"

#include "slam/LoopClosure.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <fmt/format.h>

#include <cmath>
#include <random>
#include <stdexcept>

namespace reusex::slam {
namespace {

using RowMajor4d = Eigen::Matrix<double, 4, 4, Eigen::RowMajor>;

Eigen::Matrix4d to_eigen(const std::array<double, 16> &a) {
  return Eigen::Map<const RowMajor4d>(a.data());
}

std::array<double, 16> to_array(const Eigen::Matrix4d &m) {
  std::array<double, 16> a{};
  Eigen::Map<RowMajor4d>(a.data()) = m;
  return a;
}

bool finite(const Point3 &p) {
  return std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2]);
}

} // namespace

RigidFit rigid_from_matches(const std::array<double, 16> &pose_a,
                            const std::array<double, 16> &pose_b,
                            const std::vector<PointPair> &pairs,
                            double inlier_dist_m, int iterations,
                            std::uint32_t seed) {
  RigidFit fit;
  const int n = static_cast<int>(pairs.size());
  for (int k = 0; k < n; ++k)
    if (!finite(pairs[static_cast<std::size_t>(k)].first) ||
        !finite(pairs[static_cast<std::size_t>(k)].second))
      throw std::invalid_argument(
          fmt::format("rigid_from_matches: pair {} is not finite", k));
  if (n < 3) {
    fit.failure = RigidFitFailure::too_few_pairs;
    fit.reason = fmt::format("3 matches with depth needed, {} given", n);
    return fit;
  }

  // Fit in A's frame (well-conditioned whatever the world origin):
  // src = T_A^-1 T_B p_b, dst = p_a, dst ≈ M src.
  const Eigen::Matrix4d TA = to_eigen(pose_a);
  const Eigen::Matrix4d b_in_a = TA.inverse() * to_eigen(pose_b);
  Eigen::Matrix3Xd src(3, n), dst(3, n);
  for (int k = 0; k < n; ++k) {
    const auto &[pa, pb] = pairs[static_cast<std::size_t>(k)];
    src.col(k) = (b_in_a * Eigen::Vector4d(pb[0], pb[1], pb[2], 1.0)).head<3>();
    dst.col(k) = Eigen::Vector3d(pa[0], pa[1], pa[2]);
  }
  geometry::LoopClosureOptions o;
  o.ransac_iterations = iterations;
  o.ransac_inlier_dist = static_cast<float>(inlier_dist_m);
  std::mt19937 rng(seed);
  Eigen::Matrix4d M_fit = Eigen::Matrix4d::Identity();
  std::vector<int> inliers = geometry::detail::ransac_rigid(src, dst, o, rng, M_fit);
  const int m = static_cast<int>(inliers.size());
  if (m < 3) {
    fit.failure = RigidFitFailure::no_consensus;
    fit.inliers = std::move(inliers);
    fit.reason = fmt::format("no rigid fit: {} of {} pairs agree within {:.3f} m",
                             m, n, inlier_dist_m);
    return fit;
  }

  // Collinear inliers leave the rotation about their line free.
  Eigen::Matrix3Xd d(3, m);
  for (int k = 0; k < m; ++k)
    d.col(k) = dst.col(inliers[static_cast<std::size_t>(k)]);
  d.colwise() -= d.rowwise().mean();
  const Eigen::JacobiSVD<Eigen::Matrix3Xd> svd(d);
  const double spread = svd.singularValues()(1) / std::sqrt(static_cast<double>(m));
  if (spread < kMinFitSpreadM) {
    fit.failure = RigidFitFailure::degenerate;
    fit.inliers = std::move(inliers);
    fit.reason = fmt::format("inlier points are (nearly) collinear: {:.4f} m "
                             "spread off their line, {:.3f} m needed",
                             spread, kMinFitSpreadM);
    return fit;
  }

  double sq = 0.0;
  for (int i : inliers)
    sq += ((M_fit * src.col(i).homogeneous()).head<3>() - dst.col(i)).squaredNorm();
  fit.rms_m = std::sqrt(sq / m);
  fit.world_delta = to_array(TA * M_fit * TA.inverse());
  fit.inliers = std::move(inliers);
  fit.ok = true;
  return fit;
}

} // namespace reusex::slam
```

- [ ] **Step 6: Run the tests to verify they pass**

```bash
nix develop -c cmake --build build -j8 --target reusex_unit_tests
nix develop -c build/tests/reusex_unit_tests "[matches]"
```
Expected: `All tests passed (… assertions in 6 test cases)`.
`ransac_rigid` sorts nothing, so the inlier indices come out in column
order, i.e. ascending, as the header promises. If the no-consensus case
finds 3 "inliers" by chance, increase the disagreement in that test's data
(`5.0 * k * k`). Do not relax the threshold.

- [ ] **Step 7: Commit**

```bash
nix develop -c git add libs/reusex/include/slam/frame_pair_matches.hpp libs/reusex/src/slam/frame_pair_matches.cpp tests/unit/slam/test_frame_pair_matches.cpp
nix develop -c git commit -m "slam: rigid_from_matches, B's world correction from 3D correspondences

RANSAC (LoopClosure's ransac_rigid) in A's frame, world_delta in the pair
overlay's / ICP initial_guess convention; refuses < 3 pairs, no consensus
and collinear inliers with a reason."
```

---

### Task 3: `match_frame_pair` and `lift_pixel`

**Files:**
- Modify: `libs/reusex/src/slam/panorama_features.hpp` (signature + two helpers)
- Modify: `libs/reusex/src/slam/panorama_features.cpp:66-134` (`extract_frame_features`)
- Modify: `libs/reusex/src/slam/frame_pair_matches.cpp`
- Modify: `tests/unit/slam/test_frame_pair_matches.cpp`

**Interfaces:**
- Consumes: Task 2's header, `pano_detail::FrameFeatures`, `slam::refine_frame_pair_icp`, `slam::frame_world_points` (`slam/frame_pair_icp.hpp`), `rux::qt::edge_measurement` / `rigid_inverse` (`rux_qt/pair_alignment.hpp`, test only)
- Produces:
  - `pano_detail::extract_frame_features(const ProjectDB&, int, const cv::Ptr<cv::ORB>&, float, float, const std::optional<cv::Rect>& region = std::nullopt)`
  - `pano_detail::DepthPinhole depth_pinhole(const core::SensorIntrinsics&, int depth_cols, int depth_rows)`
  - `std::optional<Eigen::Vector3d> pano_detail::lift_depth_pixel(const cv::Mat &depth_m, const DepthPinhole&, int ud, int vd, float min_depth, float max_depth)`
  - the implementations of `match_frame_pair` and `lift_pixel` declared in Task 2

- [ ] **Step 1: Write the failing database tests**

Append to `tests/unit/slam/test_frame_pair_matches.cpp`. Add these includes
at the top:
`<reusex/core/ProjectDB.hpp>`, `<reusex/core/SensorIntrinsics.hpp>`,
`<reusex/slam/frame_pair_icp.hpp>`, `<rux_qt/pair_alignment.hpp>`,
`<opencv2/core.hpp>`, `<opencv2/imgproc.hpp>`, `"../../support/temp_path.hpp"`,
`<stdexcept>`.

```cpp
// --------------------------------------------------- database-backed tests --

using reusex::slam::FrameMatchOptions;
using reusex::slam::lift_pixel;
using reusex::slam::match_frame_pair;
using reusex::test_support::TempPath;

namespace {

constexpr int kW = 320, kH = 240;
constexpr double kF = 300.0;

/// RTABMap's optical -> base rotation: a non-identity local_transform, so a
/// keypoint's xyz must be local · optical to land where frame_world_points
/// puts the pixel.
reusex::core::SensorIntrinsics intrinsics() {
  reusex::core::SensorIntrinsics k;
  k.fx = k.fy = kF;
  k.cx = kW / 2.0;
  k.cy = kH / 2.0;
  k.width = kW;
  k.height = kH;
  k.local_transform = {0, 0, 1, 0, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 1};
  return k;
}

/// Blocky random colour: strong, distinctive ORB corners.
cv::Mat texture() {
  cv::RNG rng(12345);
  cv::Mat small(kH / 8, kW / 8, CV_8UC3);
  rng.fill(small, cv::RNG::UNIFORM, 0, 256);
  cv::Mat big;
  cv::resize(small, big, cv::Size(kW, kH), 0, 0, cv::INTER_NEAREST);
  return big;
}

/// A wedge (1.2 m rising to ~1.84 m left to right), a box 0.3 m nearer,
/// and a 20x20 hole without depth at (20..39, 20..39): non-planar, so ICP
/// is well-posed, and a hole for lift_pixel's snap.
uint16_t depth_mm(int u, int v) {
  if (u >= 20 && u < 40 && v >= 20 && v < 40)
    return 0;
  int z = 1200 + 2 * u;
  if (u >= 200 && u < 260 && v >= 60 && v < 140)
    z -= 300;
  return static_cast<uint16_t>(z);
}

/// The depth image at kW/cols of the colour resolution.
cv::Mat depth(int cols, int rows) {
  cv::Mat d(rows, cols, CV_16UC1);
  const int s = kW / cols;
  for (int v = 0; v < rows; ++v)
    for (int u = 0; u < cols; ++u)
      d.at<uint16_t>(v, u) = depth_mm(u * s, v * s);
  return d;
}

/// Frames 1 (A) and 2 (B): the SAME colour and depth, different stored
/// poses — the true correction is world_delta = T_A · T_B^-1. Frame 3: A's
/// view with half-resolution depth. Frame 4: colour, no depth. Frame 5:
/// colour and depth, no pose.
void write_fixture(const std::string &path) {
  reusex::ProjectDB db(path, /*readOnly=*/false);
  const auto k = intrinsics();
  const cv::Mat c = texture();
  const cv::Mat conf(kH, kW, CV_8UC1, cv::Scalar(255));
  db.save_sensor_frame(1, c, depth(kW, kH), conf, kPoseA, k, 0.0);
  db.save_sensor_frame(2, c, depth(kW, kH), conf, kPoseB, k, 1.0);
  db.save_sensor_frame(3, c, depth(kW / 2, kH / 2),
                       cv::Mat(kH / 2, kW / 2, CV_8UC1, cv::Scalar(255)),
                       kPoseA, k, 2.0);
  db.save_sensor_frame(4, c, cv::Mat{}, conf, kPoseA, k, 3.0);
  db.save_sensor_frame(5, c, depth(kW, kH), conf, kPoseA, k, 4.0);
}

/// No public API stores a frame with depth but no pose
/// (tests/unit/io/test_update_sensor_frame_pose.cpp says why), so NULL the
/// transform with raw SQL — after the ProjectDB above has closed (WAL).
void clear_pose(const std::string &path, int node_id) {
  sqlite3 *raw = nullptr;
  REQUIRE(sqlite3_open(path.c_str(), &raw) == SQLITE_OK);
  const std::string sql = "UPDATE sensor_frames SET transform = NULL WHERE "
                          "node_id = " + std::to_string(node_id) + ";";
  REQUIRE(sqlite3_exec(raw, sql.c_str(), nullptr, nullptr, nullptr) == SQLITE_OK);
  REQUIRE(sqlite3_changes(raw) == 1);
  sqlite3_close(raw);
}

/// The fixture as every database test opens it.
void make_fixture(const std::string &path) {
  write_fixture(path);
  clear_pose(path, 5);
  reusex::ProjectDB db(path, /*readOnly=*/true);
  REQUIRE_FALSE(db.has_sensor_frame_pose(5));
  REQUIRE(db.has_sensor_frame_pose(1));
}
```
Add `#include <sqlite3.h>` and `#include <string>` to the includes. Every
database test below opens it through `make_fixture`.

```cpp
std::vector<PointPair> depth_pairs(const reusex::slam::FramePairMatches &m) {
  std::vector<PointPair> out;
  for (const auto &mm : m.matches) {
    const auto &ka = m.keypoints_a[static_cast<std::size_t>(mm.a)];
    const auto &kb = m.keypoints_b[static_cast<std::size_t>(mm.b)];
    if (ka.xyz && kb.xyz)
      out.emplace_back(*ka.xyz, *kb.xyz);
  }
  return out;
}

const Eigen::Matrix4d kTruth = M(kPoseA) * M(kPoseB).inverse();

} // namespace

TEST_CASE("FramePairMatches_IdenticalViews_RecoverTheStoredPoseDifference",
          "[slam][matches]") {
  const TempPath project("frame_pair_matches_recover");
  make_fixture(project.path);
  reusex::ProjectDB db(project.path, /*readOnly=*/true);
  const auto m = match_frame_pair(db, 1, 2);
  REQUIRE(m.matches.size() >= 50);
  for (const auto &mm : m.matches) {
    REQUIRE(mm.a >= 0);
    REQUIRE(mm.a < static_cast<int>(m.keypoints_a.size()));
    REQUIRE(mm.b >= 0);
    REQUIRE(mm.b < static_cast<int>(m.keypoints_b.size()));
  }
  // The hole: keypoints there are kept, without xyz.
  const auto fit = rigid_from_matches(kPoseA, kPoseB, depth_pairs(m));
  REQUIRE(fit.ok);
  CHECK(translation_error(fit.world_delta, kTruth) < 0.01);
  CHECK(angle_error_deg(fit.world_delta, kTruth) < 0.5);
}

TEST_CASE("FramePairMatches_Region_DropsKeypointsOutside", "[slam][matches]") {
  const TempPath project("frame_pair_matches_region");
  make_fixture(project.path);
  reusex::ProjectDB db(project.path, /*readOnly=*/true);
  FrameMatchOptions o;
  o.region_a = reusex::slam::PixelRect{0, 0, kW / 2, kH};
  const auto m = match_frame_pair(db, 1, 2, o);
  REQUIRE_FALSE(m.keypoints_a.empty());
  // ORB's mask test rounds a keypoint to its pixel: allow half a pixel.
  for (const auto &k : m.keypoints_a)
    CHECK(k.u < kW / 2.0f + 0.5f);
  const bool b_right = std::any_of(m.keypoints_b.begin(), m.keypoints_b.end(),
                                   [](const auto &k) { return k.u > kW / 2.0f; });
  CHECK(b_right); // B unrestricted
}

TEST_CASE("FramePairMatches_RegionPartlyOutside_KeepsTheIntersection",
          "[slam][matches]") {
  const TempPath project("frame_pair_matches_region_out");
  make_fixture(project.path);
  reusex::ProjectDB db(project.path, /*readOnly=*/true);
  FrameMatchOptions o;
  o.region_b = reusex::slam::PixelRect{kW / 2, -50, kW, kH + 100};
  const auto m = match_frame_pair(db, 1, 2, o);
  REQUIRE_FALSE(m.keypoints_b.empty());
  for (const auto &k : m.keypoints_b)
    CHECK(k.u >= kW / 2.0f - 0.5f);
  o.region_b = reusex::slam::PixelRect{kW + 10, 0, 50, 50}; // all outside
  const auto none = match_frame_pair(db, 1, 2, o);
  CHECK(none.keypoints_b.empty());
  CHECK(none.matches.empty());
}

TEST_CASE("FramePairMatches_FrameWithoutDepthOrPose_ThrowsNamingIt",
          "[slam][matches]") {
  const TempPath project("frame_pair_matches_missing");
  make_fixture(project.path);
  reusex::ProjectDB db(project.path, /*readOnly=*/true);
  const auto message = [&](int a, int b) -> std::string {
    try {
      match_frame_pair(db, a, b);
    } catch (const std::runtime_error &e) {
      return e.what();
    }
    return {};
  };
  CHECK(message(1, 4).find("frame 4") != std::string::npos);
  CHECK(message(5, 1).find("frame 5") != std::string::npos);
  CHECK_THROWS_AS(match_frame_pair(db, 2, 2), std::invalid_argument);
}

TEST_CASE("LiftPixel_SnapsToNearestDepth_AndRefusesFarFromAny",
          "[slam][matches]") {
  const TempPath project("frame_pair_matches_lift");
  make_fixture(project.path);
  reusex::ProjectDB db(project.path, /*readOnly=*/true);
  const auto direct = lift_pixel(db, 1, 100.0f, 80.0f);
  REQUIRE(direct);
  CHECK(direct->u == 100.0f);
  CHECK(direct->v == 80.0f);
  REQUIRE(direct->xyz);
  const auto snapped = lift_pixel(db, 1, 22.0f, 30.0f); // 3 px into the hole
  REQUIRE(snapped);
  CHECK(snapped->u == 19.0f);
  CHECK(snapped->v == 30.0f);
  CHECK_FALSE(lift_pixel(db, 1, 30.0f, 30.0f)); // 10+ px from any depth
  CHECK_FALSE(lift_pixel(db, 1, -5.0f, 30.0f)); // outside the image
  CHECK_THROWS_AS(lift_pixel(db, 4, 10.0f, 10.0f), std::runtime_error);
}

TEST_CASE("LiftPixel_HalfResolutionDepth_MatchesFullResolutionLift",
          "[slam][matches]") {
  const TempPath project("frame_pair_matches_halfres");
  make_fixture(project.path);
  reusex::ProjectDB db(project.path, /*readOnly=*/true);
  const auto full = lift_pixel(db, 1, 100.0f, 80.0f);
  const auto half = lift_pixel(db, 3, 100.0f, 80.0f);
  REQUIRE(full);
  REQUIRE(half);
  CHECK(half->u == 100.0f);
  CHECK(half->v == 80.0f);
  for (int i = 0; i < 3; ++i)
    CHECK(std::abs((*half->xyz)[static_cast<std::size_t>(i)] -
                   (*full->xyz)[static_cast<std::size_t>(i)]) < 1e-9);
}

TEST_CASE("LiftPixel_PoseTimesXyz_IsTheOverlaysWorldPoint", "[slam][matches]") {
  // The stored-pose-frame convention: T_A · xyz must be a point of the very
  // cloud the overlay draws (frame_world_points, every pixel).
  const TempPath project("frame_pair_matches_world");
  make_fixture(project.path);
  reusex::ProjectDB db(project.path, /*readOnly=*/true);
  const auto k = lift_pixel(db, 1, 150.0f, 100.0f);
  REQUIRE(k);
  const Eigen::Vector3d w =
      (M(kPoseA) * Eigen::Vector3d((*k->xyz)[0], (*k->xyz)[1], (*k->xyz)[2])
                       .homogeneous())
          .head<3>();
  reusex::slam::FramePairIcpOptions o;
  o.pixel_step = 1;
  o.max_depth_m = 5.0f;
  double best = 1e9;
  for (const auto &p : reusex::slam::frame_world_points(db, 1, o))
    best = std::min(best, (Eigen::Vector3d(p[0], p[1], p[2]) - w).norm());
  CHECK(best < 1e-4);
}

TEST_CASE("RigidFromMatches_WorldDelta_RoundTripsWithIcpAndEdgeMeasurement",
          "[slam][matches]") {
  // The matches' world_delta is ICP-forfin's seed and, without ICP, the
  // manual edge. ICP seeded with it stays there, and the manual A -> B
  // edge T_A^-1 ΔT T_B is the inverse of ICP's B -> A edge_transform.
  const TempPath project("frame_pair_matches_icp");
  make_fixture(project.path);
  reusex::ProjectDB db(project.path, /*readOnly=*/true);
  const auto fit =
      rigid_from_matches(kPoseA, kPoseB, depth_pairs(match_frame_pair(db, 1, 2)));
  REQUIRE(fit.ok);
  reusex::slam::FramePairIcpOptions o;
  o.initial_guess = fit.world_delta;
  const auto icp = reusex::slam::refine_frame_pair_icp(db, 2, 1, o);
  REQUIRE(icp.converged);
  CHECK(translation_error(icp.world_delta, M(fit.world_delta)) < 0.01);
  CHECK(angle_error_deg(icp.world_delta, M(fit.world_delta)) < 0.5);
  const auto manual = rux::qt::edge_measurement(kPoseA, kPoseB, fit.world_delta);
  const auto icp_a_to_b = rux::qt::rigid_inverse(icp.edge_transform);
  CHECK(translation_error(manual, M(icp_a_to_b)) < 0.01);
  CHECK(angle_error_deg(manual, M(icp_a_to_b)) < 0.5);
}
```

The spec suggests "rendered checkerboard/texture + flat depth". This plan
deliberately uses a textured wedge-and-box instead. A flat plane leaves
ICP free to slide in-plane, so the ICP round-trip check could not tell a
right convention from a wrong one.

- [ ] **Step 2: Run them to verify they fail**

```bash
nix develop -c cmake --build build -j8 --target reusex_unit_tests
```
Expected: link error for `match_frame_pair` / `lift_pixel`.

- [ ] **Step 3: Extend the shared front-end in `panorama_features`**

In `libs/reusex/src/slam/panorama_features.hpp`, add `#include <optional>`
and `#include "core/SensorIntrinsics.hpp"`, then change the declaration to:

```cpp
/// Detect ORB features in a frame's colour image and lift them to 3D using the
/// stored depth map and intrinsics. Returns an empty result (no descriptors)
/// when the frame has no colour/depth or degenerate intrinsics. @p region
/// restricts detection to that colour-image rectangle (clamped; nullopt =
/// the whole image) — Rediger's match editor (frame_pair_matches.cpp).
FrameFeatures extract_frame_features(const ProjectDB &db, int node_id,
                                     const cv::Ptr<cv::ORB> &orb,
                                     float min_depth, float max_depth,
                                     const std::optional<cv::Rect> &region =
                                         std::nullopt);

/// Pinhole intrinsics scaled to a depth grid of @p depth_cols x
/// @p depth_rows (the colour calibration is intr.width x intr.height).
struct DepthPinhole {
  double fx = 0, fy = 0, cx = 0, cy = 0;
};
DepthPinhole depth_pinhole(const core::SensorIntrinsics &intr, int depth_cols,
                           int depth_rows);

/// Optical-frame 3D point of depth pixel (@p ud, @p vd) of @p depth_m
/// (CV_32F metres); nullopt outside the grid or when the depth is not
/// finite or outside [min_depth, max_depth].
std::optional<Eigen::Vector3d> lift_depth_pixel(const cv::Mat &depth_m,
                                                const DepthPinhole &k, int ud,
                                                int vd, float min_depth,
                                                float max_depth);
```

In `panorama_features.cpp`, change `extract_frame_features` to:
- take `const ProjectDB &db` and the `region`;
- use the two helpers (behaviour unchanged);
- build the ORB mask from `region`.

The body from `cv::Mat depth_f;` to the end becomes:

```cpp
  cv::Mat depth_f;
  depth16.convertTo(depth_f, CV_32FC1, 1.0 / 1000.0); // mm -> m

  const core::SensorIntrinsics intr = db.sensor_frame_intrinsics(node_id);
  const DepthPinhole k = depth_pinhole(intr, depth_f.cols, depth_f.rows);
  if (k.fx <= 0.0 || k.fy <= 0.0)
    return out;

  if (!db.has_sensor_frame_pose(node_id)) {
    core::debug("panorama_features: node {} has no usable stored pose, "
                "contributing no correspondences",
                node_id);
    return out;
  }
  out.T_world_cam = to_matrix4(db.sensor_frame_pose(node_id));

  cv::Mat mask; // empty = the whole image
  if (region) {
    mask = cv::Mat::zeros(gray.size(), CV_8U);
    const cv::Rect r = *region & cv::Rect(0, 0, gray.cols, gray.rows);
    if (r.area() > 0)
      mask(r).setTo(255);
  }
  orb->detectAndCompute(gray, mask, out.keypoints, out.descriptors);
  if (out.keypoints.empty()) {
    out.descriptors.release();
    return out;
  }

  out.local.resize(out.keypoints.size());
  out.valid.assign(out.keypoints.size(), 0);
  for (size_t i = 0; i < out.keypoints.size(); ++i) {
    const int ud = static_cast<int>(std::lround(
        out.keypoints[i].pt.x * static_cast<double>(depth_f.cols) / gray.cols));
    const int vd = static_cast<int>(std::lround(
        out.keypoints[i].pt.y * static_cast<double>(depth_f.rows) / gray.rows));
    if (const auto p =
            lift_depth_pixel(depth_f, k, ud, vd, min_depth, max_depth)) {
      out.local[i] = *p;
      out.valid[i] = 1;
    }
  }
  return out;
}

DepthPinhole depth_pinhole(const core::SensorIntrinsics &intr, int depth_cols,
                           int depth_rows) {
  const double sx = static_cast<double>(depth_cols) / std::max(1, intr.width);
  const double sy = static_cast<double>(depth_rows) / std::max(1, intr.height);
  return {intr.fx * sx, intr.fy * sy, intr.cx * sx, intr.cy * sy};
}

std::optional<Eigen::Vector3d> lift_depth_pixel(const cv::Mat &depth_m,
                                                const DepthPinhole &k, int ud,
                                                int vd, float min_depth,
                                                float max_depth) {
  if (ud < 0 || vd < 0 || ud >= depth_m.cols || vd >= depth_m.rows)
    return std::nullopt;
  const float z = depth_m.at<float>(vd, ud);
  if (!std::isfinite(z) || z < min_depth || z > max_depth)
    return std::nullopt;
  return Eigen::Vector3d((ud - k.cx) * z / k.fx, (vd - k.cy) * z / k.fy, z);
}
```
The callers in `PanoramaAlignment.cpp` and `PanoramaLoopEdges.cpp` pass a
`ProjectDB&`, which binds to the new `const&`. No caller changes.

- [ ] **Step 4: Implement `match_frame_pair` and `lift_pixel`**

In `frame_pair_matches.cpp` add the includes `"panorama_features.hpp"`,
`"core/ProjectDB.hpp"`, `"core/SensorIntrinsics.hpp"`, `"core/logging.hpp"`,
`<opencv2/core.hpp>`, `<opencv2/features2d.hpp>`, `<algorithm>` and
`<limits>`. Then add to the anonymous namespace:

```cpp
namespace pd = reusex::geometry::pano_detail;

/// Throws, naming the frame, when it cannot be matched: no depth, no pose,
/// or intrinsics that cannot place a pixel. (A missing colour image is
/// diagnosed after extraction, which decodes it anyway.)
void require_frame(const ProjectDB &db, int id, bool need_pose) {
  if (db.sensor_frame_depth(id).empty())
    throw std::runtime_error(fmt::format("frame {} has no stored depth image", id));
  if (need_pose && !db.has_sensor_frame_pose(id))
    throw std::runtime_error(fmt::format("frame {} has no stored pose", id));
  const auto k = db.sensor_frame_intrinsics(id);
  if (k.width <= 0 || k.height <= 0 || k.fx <= 0.0 || k.fy <= 0.0)
    throw std::runtime_error(fmt::format(
        "frame {} has unusable intrinsics ({}x{}, f {:.1f}/{:.1f})", id,
        k.width, k.height, k.fx, k.fy));
}

Eigen::Matrix4d local_of(const ProjectDB &db, int id) {
  return to_eigen(db.sensor_frame_intrinsics(id).local_transform);
}

FrameKeypoint keypoint(float u, float v, const std::optional<Eigen::Vector3d> &opt,
                       const Eigen::Matrix4d &local) {
  FrameKeypoint k;
  k.u = u;
  k.v = v;
  if (opt) {
    const Eigen::Vector3d p = (local * opt->homogeneous()).head<3>();
    k.xyz = std::array<double, 3>{p.x(), p.y(), p.z()};
  }
  return k;
}

std::vector<FrameKeypoint> keypoints_of(const pd::FrameFeatures &f,
                                        const Eigen::Matrix4d &local) {
  std::vector<FrameKeypoint> out;
  out.reserve(f.keypoints.size());
  for (std::size_t i = 0; i < f.keypoints.size(); ++i)
    out.push_back(keypoint(f.keypoints[i].pt.x, f.keypoints[i].pt.y,
                           f.valid[i] ? std::optional(f.local[i]) : std::nullopt,
                           local));
  return out;
}

std::optional<cv::Rect> to_cv(const std::optional<PixelRect> &r) {
  if (!r || r->w <= 0 || r->h <= 0)
    return std::nullopt;
  return cv::Rect(r->x, r->y, r->w, r->h);
}
```
and, after the namespace:

```cpp
FramePairMatches match_frame_pair(const ProjectDB &db, int a_id, int b_id,
                                  const FrameMatchOptions &opts) {
  if (a_id == b_id)
    throw std::invalid_argument(fmt::format(
        "match_frame_pair: A and B are the same frame ({})", a_id));
  require_frame(db, a_id, /*need_pose=*/true);
  require_frame(db, b_id, /*need_pose=*/true);

  auto orb = cv::ORB::create(opts.max_features);
  const pd::FrameFeatures fa = pd::extract_frame_features(
      db, a_id, orb, opts.min_depth_m, opts.max_depth_m, to_cv(opts.region_a));
  const pd::FrameFeatures fb = pd::extract_frame_features(
      db, b_id, orb, opts.min_depth_m, opts.max_depth_m, to_cv(opts.region_b));
  for (const auto &[id, f] : {std::pair{a_id, &fa}, std::pair{b_id, &fb}})
    if (f->keypoints.empty() && db.sensor_frame_image(id).empty())
      throw std::runtime_error(fmt::format("frame {} has no stored colour image", id));

  FramePairMatches out;
  out.keypoints_a = keypoints_of(fa, local_of(db, a_id));
  out.keypoints_b = keypoints_of(fb, local_of(db, b_id));
  if (fa.descriptors.empty() || fb.descriptors.empty()) {
    reusex::debug("match_frame_pair: {} / {} keypoints in frames {} / {}; "
                  "nothing to match",
                  fa.keypoints.size(), fb.keypoints.size(), a_id, b_id);
    return out;
  }
  // LoopClosure's matcher: B queried against A, Lowe ratio test.
  cv::BFMatcher matcher(cv::NORM_HAMMING);
  std::vector<std::vector<cv::DMatch>> knn;
  matcher.knnMatch(fb.descriptors, fa.descriptors, knn, 2);
  for (const auto &m : knn) {
    if (m.size() < 2 || m[0].distance > opts.ratio_test * m[1].distance)
      continue;
    out.matches.push_back({m[0].trainIdx, m[0].queryIdx, m[0].distance});
  }
  reusex::debug("match_frame_pair: {} matches between frames {} and {}",
                out.matches.size(), a_id, b_id);
  return out;
}

std::optional<FrameKeypoint> lift_pixel(const ProjectDB &db, int frame_id,
                                        float u, float v, int snap_px,
                                        float min_depth_m, float max_depth_m) {
  require_frame(db, frame_id, /*need_pose=*/false);
  const cv::Mat color = db.sensor_frame_image(frame_id);
  if (color.empty())
    throw std::runtime_error(
        fmt::format("frame {} has no stored colour image", frame_id));
  if (!(u >= 0.0f && v >= 0.0f && u <= color.cols - 1.0f &&
        v <= color.rows - 1.0f))
    return std::nullopt;
  cv::Mat depth_m;
  db.sensor_frame_depth(frame_id).convertTo(depth_m, CV_32FC1, 1.0 / 1000.0);
  const double sx = static_cast<double>(depth_m.cols) / color.cols;
  const double sy = static_cast<double>(depth_m.rows) / color.rows;
  const pd::DepthPinhole k =
      pd::depth_pinhole(db.sensor_frame_intrinsics(frame_id), depth_m.cols,
                        depth_m.rows);
  // The keypoint mapping (extract_frame_features): colour px -> depth px.
  const int u0 = static_cast<int>(std::lround(u * sx));
  const int v0 = static_cast<int>(std::lround(v * sy));
  const int r = std::max(0, static_cast<int>(std::lround(snap_px * std::max(sx, sy))));
  // Nearest valid depth pixel within r; ties: first in row-major order.
  int best_d2 = std::numeric_limits<int>::max(), bu = 0, bv = 0;
  std::optional<Eigen::Vector3d> best;
  for (int dv = -r; dv <= r; ++dv)
    for (int du = -r; du <= r; ++du) {
      const int d2 = du * du + dv * dv;
      if (d2 > r * r || d2 >= best_d2)
        continue;
      if (auto p = pd::lift_depth_pixel(depth_m, k, u0 + du, v0 + dv,
                                        min_depth_m, max_depth_m)) {
        best = p;
        best_d2 = d2;
        bu = u0 + du;
        bv = v0 + dv;
      }
    }
  if (!best)
    return std::nullopt;
  return keypoint(static_cast<float>(bu / sx), static_cast<float>(bv / sy),
                  best, local_of(db, frame_id));
}
```

- [ ] **Step 5: Run the new tests and the old slam tests**

```bash
nix develop -c cmake --build build -j8 --target reusex_unit_tests
nix develop -c build/tests/reusex_unit_tests "[matches]"
nix develop -c ctest --test-dir build -R "Panorama|LoopClosure|FramePair" --output-on-failure --parallel 8
```
Expected: all pass. The panorama tests guard the `extract_frame_features`
refactor.

If `FramePairMatches_IdenticalViews…` finds fewer than 50 matches, check
`m.keypoints_a.size()`. A blank texture means `rng.fill` produced a
constant image; never lower the threshold below 50.

If the "with depth" pairs include points in the box's step edge, the
lifting is still exact (identical images), so the error stays below
1e-6 m.

- [ ] **Step 6: Commit**

```bash
nix develop -c git add libs/reusex/src/slam/panorama_features.hpp libs/reusex/src/slam/panorama_features.cpp libs/reusex/src/slam/frame_pair_matches.cpp tests/unit/slam/test_frame_pair_matches.cpp
nix develop -c git commit -m "slam: match_frame_pair and lift_pixel for Rediger's match editor

ORB + Lowe ratio between two stored frames through the panorama
front-end's extract_frame_features (now const, with an optional region),
keypoints in the stored-pose frame; lift_pixel snaps a click to the
nearest depth. Tests pin the world_delta convention against ICP and the
manual edge."
```

---

### Task 4: Qt-free match-editor logic (`rux_qt_core`)

**Files:**
- Create: `apps/rux/qt/include/rux_qt/match_editor.hpp`
- Create: `apps/rux/qt/src/core/match_editor.cpp`
- Modify: `apps/rux/qt/CMakeLists.txt` (add the source to `rux_qt_core`)
- Create: `tests/unit/rux_qt/test_match_editor.cpp`

**Interfaces:**
- Consumes: `Vec3` (`rux_qt/pair_alignment.hpp`), `format_decimal_da` (`rux_qt/database_logic.hpp`)
- Produces (namespace `rux::qt`):
  ```cpp
  enum class MatchSide { a, b };
  enum class MatchMode { select, add, region };
  enum class MatchFit { neutral, inlier, outlier };
  enum class LineTone { neutral, inlier, outlier, excluded };
  enum class MatchFitFailure { too_few_pairs, no_consensus, degenerate };
  struct MatchPoint { float u = 0, v = 0; std::optional<Vec3> xyz; };
  struct MatchLine { int a = -1, b = -1; float distance = 0; bool manual = false; bool excluded = false; MatchFit fit = MatchFit::neutral; };
  struct PixelBox { int x = 0, y = 0, w = 0, h = 0; bool empty() const; };
  struct MatchCounts { int total = 0, inliers = 0, manual = 0, excluded = 0, usable = 0; bool fitted = false; bool searched = false; };
  struct FitInput { std::vector<std::pair<Vec3, Vec3>> pairs; std::vector<int> lines; };
  class MatchSet { set_pair, a, b, clear, set_detected, add_manual, toggle_excluded, fit_input, apply_fit, keypoints, lines, counts, set_region, region, has_region, clear_regions };
  class ManualPick { pick, cancel, pending, side, point };
  LineTone line_tone(const MatchLine &);
  std::string match_readout(const MatchCounts &);
  std::string match_fit_text(int inliers, double rms_m);
  std::string match_fit_failure_text(MatchFitFailure, int pairs, int inliers, double inlier_dist_m);
  struct ViewRect { double x = 0, y = 0, w = 0, h = 0; bool contains(double px, double py) const; };
  struct ImageView { double zoom = 1.0, pan_x = 0.0, pan_y = 0.0; };
  inline constexpr double kMaxImageZoom = 16.0;
  std::array<ViewRect, 2> pair_slots(double width, double height, double gap);
  ViewRect image_rect(const ViewRect &slot, int img_w, int img_h, const ImageView &view);
  std::array<double, 2> image_to_canvas(const ViewRect &drawn, int img_w, int img_h, double u, double v);
  std::array<double, 2> canvas_to_image(const ViewRect &drawn, int img_w, int img_h, double x, double y);
  ImageView zoom_about(const ViewRect &slot, int img_w, int img_h, ImageView view, double x, double y, double factor);
  struct Segment { double x0, y0, x1, y1; };
  int nearest_segment(const std::vector<Segment> &segs, double x, double y, double max_dist);
  PixelBox box_from_drag(double u0, double v0, double u1, double v1, int img_w, int img_h, int min_px = 4);
  ```

- [ ] **Step 1: Write the failing tests**

Create `tests/unit/rux_qt/test_match_editor.cpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

// Rediger's match editor, the Qt-free half (spec 2026-10-14 §4.4).

#include <catch2/catch_test_macros.hpp>

#include <rux_qt/match_editor.hpp>

#include <cmath>

using namespace rux::qt;

namespace {

MatchPoint pt(float u, float v, bool depth = true) {
  MatchPoint p;
  p.u = u;
  p.v = v;
  if (depth)
    p.xyz = Vec3{u / 100.0, v / 100.0, 1.0};
  return p;
}

/// Three keypoints a side, matched 0-0, 1-1, 2-2; keypoint 2 of B has no depth.
void detect3(MatchSet &s) {
  s.set_detected({pt(1, 1), pt(2, 2), pt(3, 3)},
                 {pt(1, 1), pt(2, 2), pt(3, 3, false)},
                 {{0, 0, 10.0f}, {1, 1, 11.0f}, {2, 2, 12.0f}});
}

} // namespace

TEST_CASE("MatchSet_SwappedPairIsANewPair_AndClears", "[rux_qt][matches]") {
  MatchSet s;
  s.set_pair(7, 8);
  detect3(s);
  s.set_region(MatchSide::a, PixelBox{0, 0, 10, 10});
  s.set_pair(7, 8); // same pair: kept
  CHECK(s.lines().size() == 3);
  s.set_pair(8, 7); // Byt A og B: a different pair
  CHECK(s.lines().empty());
  CHECK(s.keypoints(MatchSide::a).empty());
  CHECK_FALSE(s.has_region());
  CHECK_FALSE(s.counts().searched);
}

TEST_CASE("MatchSet_FitInput_TakesEnabledLinesWithDepthAtBothEnds",
          "[rux_qt][matches]") {
  MatchSet s;
  s.set_pair(1, 2);
  detect3(s);
  s.toggle_excluded(0);
  const FitInput in = s.fit_input();
  REQUIRE(in.lines == std::vector<int>{1}); // 0 excluded, 2 lacks depth in B
  CHECK(in.pairs.size() == 1);
  CHECK(s.counts().usable == 1);
}

TEST_CASE("MatchSet_ApplyFit_ColoursFittedLinesOnly", "[rux_qt][matches]") {
  MatchSet s;
  s.set_pair(1, 2);
  s.set_detected({pt(1, 1), pt(2, 2), pt(3, 3), pt(4, 4)},
                 {pt(1, 1), pt(2, 2), pt(3, 3), pt(4, 4, false)},
                 {{0, 0, 1}, {1, 1, 1}, {2, 2, 1}, {3, 3, 1}});
  const FitInput in = s.fit_input(); // lines 0, 1, 2
  s.apply_fit(in, {0, 2});            // pairs 0 and 2 are inliers
  CHECK(s.lines()[0].fit == MatchFit::inlier);
  CHECK(s.lines()[1].fit == MatchFit::outlier);
  CHECK(s.lines()[2].fit == MatchFit::inlier);
  CHECK(s.lines()[3].fit == MatchFit::neutral); // never fitted: no depth
  const MatchCounts c = s.counts();
  CHECK(c.fitted);
  CHECK(c.inliers == 2);
  s.toggle_excluded(0); // excluding an inlier: faint, no longer counted
  CHECK(line_tone(s.lines()[0]) == LineTone::excluded);
  CHECK(s.counts().inliers == 1);
}

TEST_CASE("MatchSet_RedetectKeepsManualMatches_AndResetsTheFit",
          "[rux_qt][matches]") {
  MatchSet s;
  s.set_pair(1, 2);
  detect3(s);
  s.add_manual(pt(50, 50), pt(60, 60));
  s.apply_fit(s.fit_input(), {0, 1, 2});
  detect3(s); // Find match again (e.g. with a region)
  const MatchCounts c = s.counts();
  CHECK(c.total == 4);
  CHECK(c.manual == 1);
  CHECK_FALSE(c.fitted);
  const MatchLine &m = s.lines().back();
  CHECK(m.manual);
  CHECK(s.keypoints(MatchSide::a)[static_cast<std::size_t>(m.a)].u == 50.0f);
  CHECK(s.keypoints(MatchSide::b)[static_cast<std::size_t>(m.b)].u == 60.0f);
}

TEST_CASE("ManualPick_CompletesAcrossSides_AndReplacesOnTheSameSide",
          "[rux_qt][matches]") {
  ManualPick p;
  CHECK_FALSE(p.pick(MatchSide::a, pt(1, 1)));
  CHECK_FALSE(p.pick(MatchSide::a, pt(2, 2))); // re-aim A
  REQUIRE(p.pending());
  CHECK(p.point()->u == 2.0f);
  const auto done = p.pick(MatchSide::b, pt(9, 9));
  REQUIRE(done);
  CHECK(done->first.u == 2.0f);
  CHECK(done->second.u == 9.0f);
  CHECK_FALSE(p.pending());
  CHECK_FALSE(p.pick(MatchSide::b, pt(3, 3))); // B first works too
  const auto rev = p.pick(MatchSide::a, pt(4, 4));
  REQUIRE(rev);
  CHECK(rev->first.u == 4.0f); // always (A, B)
  p.pick(MatchSide::a, pt(5, 5));
  p.cancel(); // Esc
  CHECK_FALSE(p.pending());
}

TEST_CASE("MatchReadout_NamesWhatThereIs", "[rux_qt][matches]") {
  CHECK(match_readout({}) == "Ingen match endnu");
  MatchCounts none;
  none.searched = true;
  CHECK(match_readout(none) == "Ingen match fundet");
  MatchCounts c;
  c.searched = true;
  c.total = 38;
  c.inliers = 24;
  c.manual = 3;
  c.excluded = 2;
  c.fitted = true;
  CHECK(match_readout(c) == "38 matches · 24 inliers · 3 manuelle · 2 udeladt");
  MatchCounts one;
  one.searched = true;
  one.total = 1;
  one.manual = 1;
  CHECK(match_readout(one) == "1 match · 1 manuel");
}

TEST_CASE("MatchFitTexts_AreDanish", "[rux_qt][matches]") {
  CHECK(match_fit_text(24, 0.021) == "Fra matches: 24 inliers · RMS 2,1 cm");
  CHECK(match_fit_failure_text(MatchFitFailure::too_few_pairs, 2, 0, 0.05) ==
        "Placér B kræver mindst 3 match med dybde i begge billeder — der er 2.");
  CHECK(match_fit_failure_text(MatchFitFailure::no_consensus, 9, 2, 0.05) ==
        "Ingen sammenhængende placering: kun 2 af 9 match passer sammen inden "
        "for 5 cm.");
  CHECK(match_fit_failure_text(MatchFitFailure::degenerate, 5, 5, 0.05) ==
        "Matchene ligger næsten på en linje — tilføj match et andet sted i "
        "billedet.");
}

TEST_CASE("ImageMapping_RoundTrips_AndCentresPixels", "[rux_qt][matches]") {
  const ViewRect slot{10, 20, 200, 100};
  const ViewRect r = image_rect(slot, 400, 100, {}); // fits the width
  CHECK(r.x == 10.0);
  CHECK(r.w == 200.0);
  CHECK(r.h == 50.0);
  CHECK(r.y == 45.0);
  const auto c = image_to_canvas(r, 400, 100, 0.0, 0.0); // pixel 0's centre
  CHECK(c[0] == 10.25);
  const auto back = canvas_to_image(r, 400, 100, 123.0, 67.0);
  const auto again = image_to_canvas(r, 400, 100, back[0], back[1]);
  CHECK(std::abs(again[0] - 123.0) < 1e-9);
  CHECK(std::abs(again[1] - 67.0) < 1e-9);
}

TEST_CASE("ZoomAbout_KeepsThePointUnderTheCursor_AndClamps",
          "[rux_qt][matches]") {
  const ViewRect slot{0, 0, 300, 200};
  const ImageView v0{};
  const ViewRect r0 = image_rect(slot, 600, 400, v0);
  const auto under = canvas_to_image(r0, 600, 400, 80.0, 50.0);
  const ImageView v1 = zoom_about(slot, 600, 400, v0, 80.0, 50.0, 2.0);
  CHECK(v1.zoom == 2.0);
  const auto at = image_to_canvas(image_rect(slot, 600, 400, v1), 600, 400,
                                  under[0], under[1]);
  CHECK(std::abs(at[0] - 80.0) < 1e-9);
  CHECK(std::abs(at[1] - 50.0) < 1e-9);
  CHECK(zoom_about(slot, 600, 400, v1, 0, 0, 1000.0).zoom == kMaxImageZoom);
  const ImageView out = zoom_about(slot, 600, 400, v1, 10, 10, 0.01);
  CHECK(out.zoom == 1.0); // back to fitted, centred
  CHECK(out.pan_x == 0.0);
  CHECK(out.pan_y == 0.0);
}

TEST_CASE("PairSlots_SplitWithAGap", "[rux_qt][matches]") {
  const auto s = pair_slots(410, 100, 10);
  CHECK(s[0].x == 0.0);
  CHECK(s[0].w == 200.0);
  CHECK(s[1].x == 210.0);
  CHECK(s[1].w == 200.0);
}

TEST_CASE("NearestSegment_HitsWithinTheTolerance", "[rux_qt][matches]") {
  const std::vector<Segment> segs{{0, 0, 100, 0}, {0, 10, 100, 10}};
  CHECK(nearest_segment(segs, 50, 3, 4) == 0);
  CHECK(nearest_segment(segs, 50, 8, 4) == 1);
  CHECK(nearest_segment(segs, 50, 30, 4) == -1);
  CHECK(nearest_segment(segs, 105, 0, 4) == -1); // past the end: 5 px
  CHECK(nearest_segment(segs, 50, 5, 6) == 0);   // tie: lowest index
}

TEST_CASE("BoxFromDrag_ClampsToTheImage_AndIgnoresAClick",
          "[rux_qt][matches]") {
  const PixelBox b = box_from_drag(250, 90, -20.5, 10.2, 200, 100);
  CHECK(b.x == 0);
  CHECK(b.y == 10);
  CHECK(b.w == 200);
  CHECK(b.h == 80);
  CHECK(box_from_drag(10, 10, 12, 11, 200, 100).empty());
  CHECK(box_from_drag(300, 300, 400, 400, 200, 100).empty()); // off-image
}
```

- [ ] **Step 2: Run them to verify they fail**

```bash
nix develop -c cmake --build build -j8 --target reusex_unit_tests
```
Expected: compile error, `rux_qt/match_editor.hpp: No such file`.

- [ ] **Step 3: Write the header**

Create `apps/rux/qt/include/rux_qt/match_editor.hpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Rediger's match editor, Qt-free (spec 2026-10-14 §4.4): the working set
// of correspondences between A and B (automatic and manual, excluded or
// not, coloured by the last fit), a half-made manual match, the Danish
// read-outs, and the canvas geometry — image <-> canvas mapping under a
// per-image zoom/pan, zoom about the cursor, line hit-testing, a dragged
// region. Nothing here is stored: a different pair clears it all (C4).

#include <rux_qt/pair_alignment.hpp>

#include <array>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace rux::qt {

enum class MatchSide { a, b };
/// Vælg (toggle a line, drag pans), Tilføj (click A then B), Område (drag
/// a rectangle that limits the next Find match).
enum class MatchMode { select, add, region };
enum class MatchFit { neutral, inlier, outlier };
enum class LineTone { neutral, inlier, outlier, excluded };
enum class MatchFitFailure { too_few_pairs, no_consensus, degenerate };

struct MatchPoint {
  float u = 0, v = 0;       ///< colour-image pixel
  std::optional<Vec3> xyz; ///< stored-pose-frame point; none = no depth
};

struct MatchLine {
  int a = -1, b = -1; ///< keypoint indices on A's / B's side
  float distance = 0;
  bool manual = false;
  bool excluded = false;
  MatchFit fit = MatchFit::neutral; ///< from the last Placér B fra match
};

/// A colour-image rectangle; empty when w or h <= 0.
struct PixelBox {
  int x = 0, y = 0, w = 0, h = 0;
  bool empty() const { return w <= 0 || h <= 0; }
};

struct MatchCounts {
  int total = 0;    ///< lines, automatic and manual
  int inliers = 0;  ///< fitted inliers not excluded since
  int manual = 0;
  int excluded = 0;
  int usable = 0;   ///< enabled lines with depth at both ends
  bool fitted = false;   ///< a fit coloured the lines
  bool searched = false; ///< Find match ran for this pair
};

/// What a fit consumes: one (A xyz, B xyz) per usable line, and the line
/// each pair came from.
struct FitInput {
  std::vector<std::pair<Vec3, Vec3>> pairs;
  std::vector<int> lines;
};

class MatchSet {
    public:
  /// A different pair (a swap included) clears everything.
  void set_pair(int a, int b);
  int a() const { return a_; }
  int b() const { return b_; }
  void clear();
  /// Find match's result: replaces the automatic lines, keeps the manual
  /// ones (and whether they are excluded), resets every fit colour.
  void set_detected(std::vector<MatchPoint> keypoints_a,
                    std::vector<MatchPoint> keypoints_b,
                    std::vector<MatchLine> lines);
  /// A manual line between two lifted points; neutral until the next fit.
  void add_manual(const MatchPoint &a, const MatchPoint &b);
  /// Vælg: a line in or out; an excluded line loses its fit colour.
  void toggle_excluded(int line);
  FitInput fit_input() const;
  /// @p inliers index into @p in.pairs (RigidFit::inliers): those lines
  /// become inliers, the rest of @p in.lines outliers, others neutral.
  void apply_fit(const FitInput &in, const std::vector<int> &inliers);
  const std::vector<MatchPoint> &keypoints(MatchSide side) const;
  const std::vector<MatchLine> &lines() const { return lines_; }
  MatchCounts counts() const;
  void set_region(MatchSide side, std::optional<PixelBox> box);
  const std::optional<PixelBox> &region(MatchSide side) const;
  bool has_region() const;
  void clear_regions();

    private:
  int a_ = -1, b_ = -1;
  std::vector<MatchPoint> kp_a_, kp_b_;
  std::vector<MatchLine> lines_;
  std::optional<PixelBox> region_a_, region_b_;
  bool fitted_ = false, searched_ = false;
};

/// Tilføj mode's half-made match: the first click's lifted point, waiting
/// for one on the other side.
class ManualPick {
    public:
  /// The completed (A, B) pair when the other side was picked before; a
  /// pick on the same side replaces the waiting point.
  std::optional<std::pair<MatchPoint, MatchPoint>> pick(MatchSide side,
                                                        const MatchPoint &p);
  void cancel() { side_.reset(); }
  bool pending() const { return side_.has_value(); }
  std::optional<MatchSide> side() const { return side_; }
  const MatchPoint *point() const { return side_ ? &point_ : nullptr; }

    private:
  std::optional<MatchSide> side_;
  MatchPoint point_;
};

LineTone line_tone(const MatchLine &l);
/// "38 matches · 24 inliers · 3 manuelle · 2 udeladt" (inliers only after a
/// fit, manual/excluded only when > 0); "Ingen match endnu" before Find
/// match, "Ingen match fundet" after one that found none.
std::string match_readout(const MatchCounts &c);
/// "Fra matches: 24 inliers · RMS 2,1 cm".
std::string match_fit_text(int inliers, double rms_m);
std::string match_fit_failure_text(MatchFitFailure f, int pairs, int inliers,
                                   double inlier_dist_m);

// ---------------------------------------------------------- canvas geometry

struct ViewRect {
  double x = 0, y = 0, w = 0, h = 0;
  bool contains(double px, double py) const {
    return px >= x && px <= x + w && py >= y && py <= y + h;
  }
};

/// One image's zoom (1 = fitted) and pan (canvas px), independent per side.
struct ImageView {
  double zoom = 1.0, pan_x = 0.0, pan_y = 0.0;
};
inline constexpr double kMaxImageZoom = 16.0;

/// A's and B's slots side by side, @p gap apart (the match lines cross it).
std::array<ViewRect, 2> pair_slots(double width, double height, double gap);
/// Where an @p img_w x @p img_h image is drawn: fitted into @p slot
/// (aspect kept, centred), scaled by the zoom about the slot centre, panned.
ViewRect image_rect(const ViewRect &slot, int img_w, int img_h,
                    const ImageView &view);
/// Pixel (u, v)'s CENTRE on the canvas (pixel u spans [u, u+1) * scale).
std::array<double, 2> image_to_canvas(const ViewRect &drawn, int img_w,
                                      int img_h, double u, double v);
std::array<double, 2> canvas_to_image(const ViewRect &drawn, int img_w,
                                      int img_h, double x, double y);
/// The view after zooming by @p factor about canvas point (x, y), keeping
/// the image point under it fixed; clamped to [1, kMaxImageZoom], and at 1
/// back to the fitted, unpanned view.
ImageView zoom_about(const ViewRect &slot, int img_w, int img_h,
                     ImageView view, double x, double y, double factor);

struct Segment {
  double x0 = 0, y0 = 0, x1 = 0, y1 = 0;
};
/// The segment nearest (x, y) within @p max_dist; -1 when none. Ties go to
/// the lowest index.
int nearest_segment(const std::vector<Segment> &segs, double x, double y,
                    double max_dist);
/// A dragged rectangle in image px, normalised and clamped to the image;
/// empty when it is smaller than @p min_px either way (a click, not a drag)
/// or lies outside the image.
PixelBox box_from_drag(double u0, double v0, double u1, double v1, int img_w,
                       int img_h, int min_px = 4);

} // namespace rux::qt
```

- [ ] **Step 4: Implement**

Create `apps/rux/qt/src/core/match_editor.cpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <rux_qt/database_logic.hpp>
#include <rux_qt/match_editor.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace rux::qt {

void MatchSet::set_pair(int a, int b) {
  if (a == a_ && b == b_)
    return;
  clear();
  a_ = a;
  b_ = b;
}

void MatchSet::clear() {
  kp_a_.clear();
  kp_b_.clear();
  lines_.clear();
  region_a_.reset();
  region_b_.reset();
  fitted_ = searched_ = false;
}

void MatchSet::set_detected(std::vector<MatchPoint> keypoints_a,
                            std::vector<MatchPoint> keypoints_b,
                            std::vector<MatchLine> lines) {
  std::vector<std::pair<std::pair<MatchPoint, MatchPoint>, bool>> manual;
  for (const MatchLine &l : lines_)
    if (l.manual)
      manual.push_back({{kp_a_[static_cast<std::size_t>(l.a)],
                         kp_b_[static_cast<std::size_t>(l.b)]},
                        l.excluded});
  kp_a_ = std::move(keypoints_a);
  kp_b_ = std::move(keypoints_b);
  lines_ = std::move(lines);
  for (MatchLine &l : lines_) {
    l.manual = l.excluded = false;
    l.fit = MatchFit::neutral;
  }
  for (const auto &[pts, excluded] : manual) {
    add_manual(pts.first, pts.second);
    lines_.back().excluded = excluded;
  }
  fitted_ = false;
  searched_ = true;
}

void MatchSet::add_manual(const MatchPoint &a, const MatchPoint &b) {
  kp_a_.push_back(a);
  kp_b_.push_back(b);
  MatchLine l;
  l.a = static_cast<int>(kp_a_.size()) - 1;
  l.b = static_cast<int>(kp_b_.size()) - 1;
  l.manual = true;
  lines_.push_back(l);
}

void MatchSet::toggle_excluded(int line) {
  if (line < 0 || line >= static_cast<int>(lines_.size()))
    return;
  MatchLine &l = lines_[static_cast<std::size_t>(line)];
  l.excluded = !l.excluded;
  if (l.excluded)
    l.fit = MatchFit::neutral;
}

FitInput MatchSet::fit_input() const {
  FitInput in;
  for (int i = 0; i < static_cast<int>(lines_.size()); ++i) {
    const MatchLine &l = lines_[static_cast<std::size_t>(i)];
    const MatchPoint &pa = kp_a_[static_cast<std::size_t>(l.a)];
    const MatchPoint &pb = kp_b_[static_cast<std::size_t>(l.b)];
    if (l.excluded || !pa.xyz || !pb.xyz)
      continue;
    in.pairs.emplace_back(*pa.xyz, *pb.xyz);
    in.lines.push_back(i);
  }
  return in;
}

void MatchSet::apply_fit(const FitInput &in, const std::vector<int> &inliers) {
  for (MatchLine &l : lines_)
    l.fit = MatchFit::neutral;
  for (int line : in.lines)
    lines_[static_cast<std::size_t>(line)].fit = MatchFit::outlier;
  for (int k : inliers)
    if (k >= 0 && k < static_cast<int>(in.lines.size()))
      lines_[static_cast<std::size_t>(in.lines[static_cast<std::size_t>(k)])]
          .fit = MatchFit::inlier;
  fitted_ = true;
}

const std::vector<MatchPoint> &MatchSet::keypoints(MatchSide side) const {
  return side == MatchSide::a ? kp_a_ : kp_b_;
}

MatchCounts MatchSet::counts() const {
  MatchCounts c;
  c.total = static_cast<int>(lines_.size());
  for (const MatchLine &l : lines_) {
    c.manual += l.manual ? 1 : 0;
    c.excluded += l.excluded ? 1 : 0;
    c.inliers += !l.excluded && l.fit == MatchFit::inlier ? 1 : 0;
  }
  c.usable = static_cast<int>(fit_input().lines.size());
  c.fitted = fitted_;
  c.searched = searched_;
  return c;
}

void MatchSet::set_region(MatchSide side, std::optional<PixelBox> box) {
  if (box && box->empty())
    box.reset();
  (side == MatchSide::a ? region_a_ : region_b_) = box;
}

const std::optional<PixelBox> &MatchSet::region(MatchSide side) const {
  return side == MatchSide::a ? region_a_ : region_b_;
}

bool MatchSet::has_region() const { return region_a_ || region_b_; }

void MatchSet::clear_regions() {
  region_a_.reset();
  region_b_.reset();
}

std::optional<std::pair<MatchPoint, MatchPoint>>
ManualPick::pick(MatchSide side, const MatchPoint &p) {
  if (!side_ || *side_ == side) {
    side_ = side;
    point_ = p;
    return std::nullopt;
  }
  const MatchPoint first = point_;
  side_.reset();
  return side == MatchSide::b ? std::pair{first, p} : std::pair{p, first};
}

LineTone line_tone(const MatchLine &l) {
  if (l.excluded)
    return LineTone::excluded;
  switch (l.fit) {
  case MatchFit::inlier:
    return LineTone::inlier;
  case MatchFit::outlier:
    return LineTone::outlier;
  case MatchFit::neutral:
    break;
  }
  return LineTone::neutral;
}

std::string match_readout(const MatchCounts &c) {
  if (c.total == 0)
    return c.searched ? "Ingen match fundet" : "Ingen match endnu";
  std::string s = std::to_string(c.total) + (c.total == 1 ? " match" : " matches");
  if (c.fitted)
    s += " · " + std::to_string(c.inliers) + (c.inliers == 1 ? " inlier" : " inliers");
  if (c.manual > 0)
    s += " · " + std::to_string(c.manual) + (c.manual == 1 ? " manuel" : " manuelle");
  if (c.excluded > 0)
    s += " · " + std::to_string(c.excluded) + " udeladt";
  return s;
}

std::string match_fit_text(int inliers, double rms_m) {
  return "Fra matches: " + std::to_string(inliers) +
         (inliers == 1 ? " inlier" : " inliers") + " · RMS " +
         format_decimal_da(rms_m * 100.0, 1) + " cm";
}

std::string match_fit_failure_text(MatchFitFailure f, int pairs, int inliers,
                                   double inlier_dist_m) {
  switch (f) {
  case MatchFitFailure::too_few_pairs:
    return "Placér B kræver mindst 3 match med dybde i begge billeder — der "
           "er " + std::to_string(pairs) + ".";
  case MatchFitFailure::no_consensus:
    return "Ingen sammenhængende placering: kun " + std::to_string(inliers) +
           " af " + std::to_string(pairs) + " match passer sammen inden for " +
           format_decimal_da(inlier_dist_m * 100.0, 0) + " cm.";
  case MatchFitFailure::degenerate:
    break;
  }
  return "Matchene ligger næsten på en linje — tilføj match et andet sted i "
         "billedet.";
}

std::array<ViewRect, 2> pair_slots(double width, double height, double gap) {
  const double w = std::max(0.0, (width - gap) / 2.0);
  return {ViewRect{0.0, 0.0, w, height}, ViewRect{w + gap, 0.0, w, height}};
}

ViewRect image_rect(const ViewRect &slot, int img_w, int img_h,
                    const ImageView &view) {
  if (img_w <= 0 || img_h <= 0 || slot.w <= 0 || slot.h <= 0)
    return {};
  const double s = std::min(slot.w / img_w, slot.h / img_h) * view.zoom;
  const double w = img_w * s, h = img_h * s;
  return {slot.x + (slot.w - w) / 2.0 + view.pan_x,
          slot.y + (slot.h - h) / 2.0 + view.pan_y, w, h};
}

std::array<double, 2> image_to_canvas(const ViewRect &d, int img_w, int img_h,
                                      double u, double v) {
  return {d.x + (u + 0.5) * d.w / img_w, d.y + (v + 0.5) * d.h / img_h};
}

std::array<double, 2> canvas_to_image(const ViewRect &d, int img_w, int img_h,
                                      double x, double y) {
  return {(x - d.x) * img_w / d.w - 0.5, (y - d.y) * img_h / d.h - 0.5};
}

ImageView zoom_about(const ViewRect &slot, int img_w, int img_h,
                     ImageView view, double x, double y, double factor) {
  const double zoom = std::clamp(view.zoom * factor, 1.0, kMaxImageZoom);
  if (zoom == 1.0)
    return {};
  const auto under =
      canvas_to_image(image_rect(slot, img_w, img_h, view), img_w, img_h, x, y);
  ImageView next{zoom, view.pan_x, view.pan_y};
  const auto now = image_to_canvas(image_rect(slot, img_w, img_h, next), img_w,
                                   img_h, under[0], under[1]);
  next.pan_x += x - now[0];
  next.pan_y += y - now[1];
  return next;
}

int nearest_segment(const std::vector<Segment> &segs, double x, double y,
                    double max_dist) {
  int best = -1;
  double best_d = max_dist;
  for (int i = 0; i < static_cast<int>(segs.size()); ++i) {
    const Segment &s = segs[static_cast<std::size_t>(i)];
    const double dx = s.x1 - s.x0, dy = s.y1 - s.y0;
    const double len2 = dx * dx + dy * dy;
    const double t =
        len2 > 0 ? std::clamp(((x - s.x0) * dx + (y - s.y0) * dy) / len2, 0.0, 1.0)
                 : 0.0;
    const double d = std::hypot(x - (s.x0 + t * dx), y - (s.y0 + t * dy));
    if (d < best_d || (d == best_d && best < 0)) {
      best = i;
      best_d = d;
    }
  }
  return best;
}

PixelBox box_from_drag(double u0, double v0, double u1, double v1, int img_w,
                       int img_h, int min_px) {
  if (std::abs(u1 - u0) < min_px || std::abs(v1 - v0) < min_px)
    return {};
  const int x0 = std::clamp(static_cast<int>(std::floor(std::min(u0, u1))), 0, img_w);
  const int y0 = std::clamp(static_cast<int>(std::floor(std::min(v0, v1))), 0, img_h);
  const int x1 = std::clamp(static_cast<int>(std::ceil(std::max(u0, u1))), 0, img_w);
  const int y1 = std::clamp(static_cast<int>(std::ceil(std::max(v0, v1))), 0, img_h);
  return {x0, y0, x1 - x0, y1 - y0};
}

} // namespace rux::qt
```
Two tests depend on boundary behaviour. `nearest_segment` uses a strict
`d < best_d`, which keeps the lowest index on ties. `best` starts at -1,
and a first segment exactly at `max_dist` is accepted. The `(105, 0)`
case is 5 px away with a 4 px tolerance, so it yields -1.

In `box_from_drag`, the off-image case `(300, 300)–(400, 400)` on 200×100
clamps to `x0 = x1 = 200`, so `w = 0` and the box is empty.

The first case, `(250, 90)–(-20.5, 10.2)`, clamps to
`x = 0, y = floor(10.2) = 10, w = 200, h = ceil(90) − 10 = 80`.

In `CMakeLists.txt` add `src/core/match_editor.cpp` after
`src/core/pair_alignment.cpp` in `add_library(rux_qt_core …)`.

- [ ] **Step 5: Run the tests**

```bash
nix develop -c cmake --build build -j8 --target reusex_unit_tests
nix develop -c build/tests/reusex_unit_tests "[rux_qt][matches]"
```
Expected: all pass (11 test cases).

- [ ] **Step 6: Commit**

```bash
nix develop -c git add apps/rux/qt/include/rux_qt/match_editor.hpp apps/rux/qt/src/core/match_editor.cpp apps/rux/qt/CMakeLists.txt tests/unit/rux_qt/test_match_editor.cpp
nix develop -c git commit -m "qt core: match set, manual pick, read-outs and canvas geometry

Rediger's match editor without Qt: automatic and manual lines, exclusion,
fit colouring, discard on a new pair, the Danish read-outs, image<->canvas
mapping with zoom about the cursor, line hit-testing, a dragged region."
```

---

### Task 5: Remove the ΔT spin boxes (closes `reusex-xlg.3`)

**Files:**
- Modify: `apps/rux/qt/include/rux_qt/PairOverlay.hpp` (drop `fields_`, `sync_fields`, `QDoubleSpinBox`; header comment)
- Modify: `apps/rux/qt/src/PairOverlay.cpp` (constructor's fields block, `apply_offset`, `sync_fields`)
- Modify: `apps/rux/qt/styles/app.qss` (drop `QFrame#offsetFields` and `QDoubleSpinBox#offsetField`)
- Modify: `apps/rux/qt/include/rux_qt/pair_alignment.hpp:64-67` (comment)
- Modify: `apps/rux/qt/src/EditWorkspace.cpp` (`right_->setSizes`)
- Modify: `apps/rux/qt/gallery/shell_pages.cpp` (`rediger-offset` / `rediger-drag` descriptions)
- Modify: `.claude/skills/design-studio/references/qt-client.md`

**Interfaces:**
- Consumes: Task 1's `PairOverlay` (probes stay)
- Produces: a `PairOverlay` whose public API is unchanged except that the fields are gone; the overlay's whole height is the canvas

- [ ] **Step 1: Claim the bead**

```bash
cd /home/mephisto/repos/ReUseX && nix develop -c bd update reusex-xlg.3 --claim
```

- [ ] **Step 2: Remove the fields**

In `PairOverlay.cpp`'s constructor, delete everything from
`auto *fields = new QFrame;` through the end of the `for (int i = 0; i < 6; ++i) {…}`
loop. Also delete `v->addWidget(fields);`. Keep the `status_` creation.

Delete `PairOverlay::sync_fields()`, and its call at the top of
`apply_offset`. In the header, delete `void sync_fields();`, the
`fields_` member, `class QDoubleSpinBox;`, and from the `.cpp`
`#include <QDoubleSpinBox>`, `<QGridLayout>` (if no longer used — the
legend still uses `QGridLayout`, keep it), `<QLocale>`, `<QSignalBlocker>`.

Rewrite the header comment's second sentence to:

```cpp
// … B carries a manual offset ΔT (rux_qt/pair_alignment.hpp) — set by the
// VTK box widget (translate by the centre sphere, rotate by a side; never
// scales; interactive mode only), the keys — ←/→ x, ↑/↓ y, PgUp/PgDn z,
// [ / ] yaw, Shift ×10 — Rediger's match editor or ICP-forfin. A one-line
// read-out in the pane header says how far B moved; "Nulstil" clears ΔT.
```

In `pair_alignment.hpp`, replace
`/// Angles wrapped into (-180, 180], translation clamped to ±kOffsetMaxM:`
`/// what the six fields can show, so they always equal the real ΔT.` with
`/// Angles wrapped into (-180, 180], translation clamped to ±kOffsetMaxM:`
`/// the ΔT the overlay holds, seeds ICP with and reads out.`.

In `app.qss`, delete the `QFrame#offsetFields { … }` rule, the comment
line above `QDoubleSpinBox#offsetField`, and that rule.

- [ ] **Step 3: Give the images the freed height**

In `EditWorkspace.cpp`, replace

```cpp
  // The images over the overlay, 1:3: the overlay's fields take a fixed
  // share of its part, and its canvas must stay usable at 1440x900 with
  // the pending banner and a few edge rows (an even split left ~30 px).
  right_->setSizes({kUnit, 3 * kUnit});
```
with
```cpp
  // The images (now the match editor) over the overlay, 1:1: without the
  // ΔT fields the overlay is all canvas, and it stays above the gallery's
  // 250 px OVERLAY TOO SMALL floor at 1440x900 with the pending banner and
  // a few edge rows.
  right_->setSizes({kUnit, kUnit});
```
and `right_->setStretchFactor(0, 1); right_->setStretchFactor(1, 3);` with
`(0, 1)` and `(1, 1)`.

- [ ] **Step 4: Build, shoot, check the guard**

```bash
nix develop -c cmake --build build -j8 --target rux-qt-gallery rux reusex_unit_tests
S=.claude/skills/design-studio/scripts/qt_shot.sh
nix develop -c bash $S --no-build --page rediger --page rediger-strip --page rediger-pending --page rediger-offset --page rediger-icp --theme both 2>&1 | tee /tmp/t5-shots.log
grep "overlay canvas" /tmp/t5-shots.log
```
Expected:
- every `overlay canvas WxH` has H ≥ 250 and no guard error;
- the PNGs show no spin boxes, the header read-out still shows
  "Flyttet … cm · …°" on `rediger-offset`, and the images pane is taller
  than in `shots/qt/baseline`.

If any H < 250, use `{2 * kUnit, 3 * kUnit}` (and stretch 2:3) instead,
re-shoot, and say so in the commit message.

Open the dark and light `rediger-offset` PNGs with Read and look.

- [ ] **Step 5: Update the gallery descriptions and the reference doc**

In `shell_pages.cpp`, change `make_rediger_offset`'s comment to
`/// B moved by hand: 20 cm in x, 5° yaw (the moved cloud and the header read-out).`.
In `make_rediger_drag`'s comment, change
`the shot shows the fields that followed the widget` to
`the shot shows the read-out that followed the widget`.

In `references/qt-client.md`, in the Rediger pages paragraph, add
`rediger-drag-qt` (centre handle dragged with Qt mouse events, `--gl`;
fails with `DRAG NOT TRANSLATED`). Then add a sentence:
"The overlay has no ΔT fields: B is moved by the box widget, the keys, the
match editor or ICP-forfin, and its header reads out the move." Also add
`DRAG NOT TRANSLATED` to the guard list in the failure paragraph.

- [ ] **Step 6: Lint and test**

```bash
nix develop -c python .claude/skills/design-studio/scripts/token_lint.py apps/rux/qt --qt
nix develop -c ctest --test-dir build -R "rux_qt|Qss|Token|pair" --output-on-failure --parallel 8
```
Expected: lint clean; tests pass (the QSS token tests catch a dangling
rule).

- [ ] **Step 7: Commit and close the bead**

```bash
nix develop -c git add apps/rux/qt .claude/skills/design-studio/references/qt-client.md
nix develop -c git commit -m "qt: Rediger's overlay drops the six ΔT fields (reusex-xlg.3)

B is placed by the box widget, keys, the match editor or ICP; the header
read-out and Nulstil stay. The images and the overlay now split the
column 1:1."
cd /home/mephisto/repos/ReUseX && nix develop -c bd close reusex-xlg.3 --reason "Spin boxes removed; read-out + Nulstil + keys + box widget remain"
```

---

### Task 6: `MatchCanvas`, the painted A | B canvas

**Files:**
- Modify: `apps/rux/qt/include/rux_qt/FrameBrowser.hpp`, `apps/rux/qt/src/FrameBrowser.cpp` (`depth_colour_table()`)
- Create: `apps/rux/qt/include/rux_qt/MatchCanvas.hpp`
- Create: `apps/rux/qt/src/MatchCanvas.cpp`

**Interfaces:**
- Consumes: Task 4 (`MatchSet`, `ManualPick`, `MatchMode`, `MatchSide`, `PixelBox`, geometry functions, `line_tone`); `DecodedFrame` (`FrameImageLoader.hpp`); `FrameLayer` (`FrameBrowser.hpp`)
- Produces:
  ```cpp
  QVector<QRgb> depth_colour_table();            // FrameBrowser.hpp, index 0 transparent
  class MatchCanvas : public QWidget {
    void set_frame(MatchSide, std::shared_ptr<const DecodedFrame>);
    void set_message(MatchSide, const QString &);  // clears that side's frame
    void set_loading(MatchSide, bool);
    void set_layer(FrameLayer);
    void set_data(const MatchSet *set, const ManualPick *pick);  // not owned
    void set_mode(MatchMode);
    void reset_views();
    ViewRect drawn_rect(MatchSide) const;          // the image as drawn now
    QSize image_size(MatchSide) const;             // colour size (else depth)
   signals:
    void line_clicked(int line);
    void pixel_clicked(rux::qt::MatchSide side, float u, float v);
    void region_dragged(rux::qt::MatchSide side, rux::qt::PixelBox box);
    void cancel_requested();
  };
  ```

- [ ] **Step 1: Share the depth colour table**

In `FrameBrowser.hpp`, after `enum class FrameLayer …;` add:

```cpp
/// The depth plane's colour table: index 0 (no return) transparent, then
/// far = a dim tint over --color-canvas up to near = --color-on-chrome.
QVector<QRgb> depth_colour_table();
```
In `FrameBrowser.cpp`, outside the anonymous namespace, define it with the
three lines currently in `ImagePane::rebuild_tables`:

```cpp
QVector<QRgb> depth_colour_table() {
  const Theme &t = theme();
  const QColor canvas = t.color("--color-canvas");
  const QColor ink = t.color("--color-on-chrome");
  QVector<QRgb> table(256, kNoData);
  for (int i = 1; i < 256; ++i)
    table[i] = mix(canvas, ink, 0.12 + 0.88 * (i - 1) / 254.0);
  return table;
}
```
In `rebuild_tables`, replace those lines with `depth_table_ = depth_colour_table();`
(keep `const Theme &t = theme();` — the confidence and label tables use it).

- [ ] **Step 2: Write the header**

Create `apps/rux/qt/include/rux_qt/MatchCanvas.hpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Rediger's match canvas (spec 2026-10-14 §4.1): A and B side by side on
// the near-black canvas, each with its own zoom (wheel, about the cursor;
// double-click resets) and pan (drag in Vælg, middle-drag in any mode),
// the keypoints (hollow = no depth) and the match lines across the gap —
// inlier --color-status-succeeded, outlier --color-status-failed,
// excluded --color-text-faint, before any fit --color-accent; manual
// lines dashed; the hovered line thicker with both ends ringed. Vælg:
// click a line; Tilføj: click a point; Område: drag a rectangle. Esc
// cancels a half-made match. All state lives in the MatchSet it is given.

#include <rux_qt/FrameBrowser.hpp>
#include <rux_qt/match_editor.hpp>

#include <QPointF>
#include <QRectF>
#include <QVector>
#include <QWidget>

#include <array>
#include <memory>
#include <optional>

namespace rux::qt {

struct DecodedFrame;

class MatchCanvas : public QWidget {
  Q_OBJECT
    public:
  explicit MatchCanvas(QWidget *parent = nullptr);
  void set_frame(MatchSide side, std::shared_ptr<const DecodedFrame> frame);
  /// A text instead of that side's image (an empty slot, an error).
  void set_message(MatchSide side, const QString &text);
  void set_loading(MatchSide side, bool loading);
  void set_layer(FrameLayer layer);
  void set_data(const MatchSet *set, const ManualPick *pick);
  void set_mode(MatchMode mode);
  /// Both images back to fitted (a new pair).
  void reset_views();
  ViewRect drawn_rect(MatchSide side) const;
  QSize image_size(MatchSide side) const;
  QSize sizeHint() const override;

    signals:
  void line_clicked(int line);
  void pixel_clicked(rux::qt::MatchSide side, float u, float v);
  void region_dragged(rux::qt::MatchSide side, rux::qt::PixelBox box);
  void cancel_requested();

    protected:
  void paintEvent(QPaintEvent *) override;
  void mousePressEvent(QMouseEvent *e) override;
  void mouseMoveEvent(QMouseEvent *e) override;
  void mouseReleaseEvent(QMouseEvent *e) override;
  void mouseDoubleClickEvent(QMouseEvent *e) override;
  void wheelEvent(QWheelEvent *e) override;
  void keyPressEvent(QKeyEvent *e) override;
  void leaveEvent(QEvent *e) override;

    private:
  std::array<ViewRect, 2> slots() const;
  std::optional<MatchSide> side_at(QPointF p) const;
  /// Canvas position of keypoint @p k on @p side, or nullopt when it is
  /// outside that side's slot (zoomed or panned away).
  std::optional<QPointF> at(MatchSide side, const MatchPoint &k) const;
  /// The visible lines as segments, and the MatchSet line each one is.
  std::vector<Segment> segments(std::vector<int> &line_of) const;
  int line_at(QPointF p) const;
  void paint_side(class QPainter &p, MatchSide side) const;

  std::array<std::shared_ptr<const DecodedFrame>, 2> frames_;
  std::array<QString, 2> messages_;
  std::array<bool, 2> loading_{};
  std::array<ImageView, 2> views_{};
  FrameLayer layer_ = FrameLayer::color;
  const MatchSet *set_ = nullptr;
  const ManualPick *pick_ = nullptr;
  MatchMode mode_ = MatchMode::select;
  QVector<QRgb> depth_table_;
  int hover_ = -1;
  // One press: which button, where, on which side, and whether it moved
  // past the click slop (then it pans or draws a region, never clicks).
  Qt::MouseButton button_ = Qt::NoButton;
  QPointF press_, last_;
  std::optional<MatchSide> press_side_;
  bool dragged_ = false;
  std::optional<QRectF> rubber_; ///< Område drag, canvas coordinates
};

} // namespace rux::qt
```

- [ ] **Step 3: Implement**

Create `apps/rux/qt/src/MatchCanvas.cpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <rux_qt/FrameImageLoader.hpp>
#include <rux_qt/MatchCanvas.hpp>
#include <rux_qt/Theme.hpp>

#include <QKeyEvent>
#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QWheelEvent>

#include <cmath>

namespace rux::qt {
namespace {

constexpr double kWheelStep = 1.25; ///< zoom factor per wheel notch

int idx(MatchSide s) { return s == MatchSide::a ? 0 : 1; }

const char *tone_token(LineTone t) {
  switch (t) {
  case LineTone::inlier:
    return "--color-status-succeeded";
  case LineTone::outlier:
    return "--color-status-failed";
  case LineTone::excluded:
    return "--color-text-faint";
  case LineTone::neutral:
    break;
  }
  return "--color-accent";
}

} // namespace

MatchCanvas::MatchCanvas(QWidget *parent) : QWidget(parent) {
  setObjectName("matchCanvas");
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  setMinimumSize(theme().px("--space-7") * 2, theme().px("--space-7"));
  setMouseTracking(true);
  setFocusPolicy(Qt::ClickFocus);
  depth_table_ = depth_colour_table();
  connect(&theme(), &Theme::changed, this, [this] {
    depth_table_ = depth_colour_table();
    update();
  });
}

QSize MatchCanvas::sizeHint() const {
  return {theme().px("--layout-panel-width"),
          theme().px("--layout-panel-width") / 2};
}

void MatchCanvas::set_frame(MatchSide side,
                            std::shared_ptr<const DecodedFrame> frame) {
  frames_[idx(side)] = std::move(frame);
  messages_[idx(side)].clear();
  loading_[idx(side)] = false;
  update();
}

void MatchCanvas::set_message(MatchSide side, const QString &text) {
  frames_[idx(side)].reset();
  messages_[idx(side)] = text;
  update();
}

void MatchCanvas::set_loading(MatchSide side, bool loading) {
  loading_[idx(side)] = loading;
  update();
}

void MatchCanvas::set_layer(FrameLayer layer) {
  layer_ = layer;
  update();
}

void MatchCanvas::set_data(const MatchSet *set, const ManualPick *pick) {
  set_ = set;
  pick_ = pick;
  hover_ = -1;
  update();
}

void MatchCanvas::set_mode(MatchMode mode) {
  mode_ = mode;
  rubber_.reset();
  setCursor(mode == MatchMode::select ? Qt::OpenHandCursor : Qt::CrossCursor);
  update();
}

void MatchCanvas::reset_views() {
  views_ = {};
  update();
}

std::array<ViewRect, 2> MatchCanvas::slots() const {
  return pair_slots(width(), height(), theme().px("--space-4"));
}

QSize MatchCanvas::image_size(MatchSide side) const {
  const auto &f = frames_[idx(side)];
  if (!f)
    return {};
  // The colour frame decides the geometry (keypoints are colour pixels);
  // the depth plane is drawn stretched into the same rect, as ImagePane does.
  return !f->color.isNull() ? f->color.size() : f->depth.size();
}

ViewRect MatchCanvas::drawn_rect(MatchSide side) const {
  const QSize s = image_size(side);
  return image_rect(slots()[idx(side)], s.width(), s.height(), views_[idx(side)]);
}

std::optional<MatchSide> MatchCanvas::side_at(QPointF p) const {
  const auto s = slots();
  if (s[0].contains(p.x(), p.y()))
    return MatchSide::a;
  if (s[1].contains(p.x(), p.y()))
    return MatchSide::b;
  return std::nullopt;
}

std::optional<QPointF> MatchCanvas::at(MatchSide side, const MatchPoint &k) const {
  const QSize s = image_size(side);
  if (s.isEmpty())
    return std::nullopt;
  const auto c = image_to_canvas(drawn_rect(side), s.width(), s.height(), k.u, k.v);
  if (!slots()[idx(side)].contains(c[0], c[1]))
    return std::nullopt;
  return QPointF(c[0], c[1]);
}

std::vector<Segment> MatchCanvas::segments(std::vector<int> &line_of) const {
  std::vector<Segment> out;
  line_of.clear();
  if (!set_)
    return out;
  const auto &ka = set_->keypoints(MatchSide::a);
  const auto &kb = set_->keypoints(MatchSide::b);
  for (int i = 0; i < static_cast<int>(set_->lines().size()); ++i) {
    const MatchLine &l = set_->lines()[static_cast<std::size_t>(i)];
    const auto pa = at(MatchSide::a, ka[static_cast<std::size_t>(l.a)]);
    const auto pb = at(MatchSide::b, kb[static_cast<std::size_t>(l.b)]);
    if (!pa || !pb)
      continue; // an end zoomed or panned out of view: not drawn, not hit
    out.push_back({pa->x(), pa->y(), pb->x(), pb->y()});
    line_of.push_back(i);
  }
  return out;
}

int MatchCanvas::line_at(QPointF p) const {
  std::vector<int> line_of;
  const int s = nearest_segment(segments(line_of), p.x(), p.y(),
                                theme().px("--space-2"));
  return s < 0 ? -1 : line_of[static_cast<std::size_t>(s)];
}

void MatchCanvas::paint_side(QPainter &p, MatchSide side) const {
  const Theme &t = theme();
  const ViewRect slot = slots()[idx(side)];
  const QRectF sr(slot.x, slot.y, slot.w, slot.h);
  p.save();
  QPainterPath clip;
  clip.addRoundedRect(sr, t.px("--radius-md"), t.px("--radius-md"));
  p.setClipPath(clip);
  p.fillRect(sr, t.color("--color-canvas"));
  const auto &f = frames_[idx(side)];
  const auto centred = [&](const QString &text) {
    p.setPen(t.color("--color-on-chrome-muted"));
    p.setFont(t.font(FontRole::sans, "--font-size-sm"));
    p.drawText(sr.adjusted(t.px("--space-3"), 0, -t.px("--space-3"), 0),
               Qt::AlignCenter | Qt::TextWordWrap, text);
  };
  if (!f) {
    centred(!messages_[idx(side)].isEmpty() ? messages_[idx(side)]
            : loading_[idx(side)]           ? QString("Henter billedet …")
                                            : QString());
    p.restore();
    return;
  }
  if (!f->error.isEmpty()) {
    centred("Billedet kunne ikke læses.\n" + f->error);
    p.restore();
    return;
  }
  QImage base = layer_ == FrameLayer::depth ? f->depth : f->color;
  if (layer_ == FrameLayer::depth && !base.isNull())
    base.setColorTable(depth_table_);
  const ViewRect d = drawn_rect(side);
  if (!base.isNull()) {
    p.setRenderHint(QPainter::SmoothPixmapTransform, true);
    p.drawImage(QRectF(d.x, d.y, d.w, d.h), base);
  } else {
    centred(layer_ == FrameLayer::depth ? "Intet dybdebillede"
                                        : "Intet farvebillede");
  }
  const QSize s = image_size(side);
  // The search region, dashed accent.
  if (set_ && set_->region(side) && !s.isEmpty()) {
    const PixelBox &b = *set_->region(side);
    const auto tl = image_to_canvas(d, s.width(), s.height(), b.x - 0.5, b.y - 0.5);
    const auto br = image_to_canvas(d, s.width(), s.height(), b.x + b.w - 0.5,
                                    b.y + b.h - 0.5);
    QPen pen(t.color("--color-accent"), 1.5, Qt::DashLine);
    p.setPen(pen);
    p.setBrush(Qt::NoBrush);
    p.drawRect(QRectF(QPointF(tl[0], tl[1]), QPointF(br[0], br[1])));
  }
  // Every keypoint: a small dot, hollow without depth.
  if (set_) {
    const double r = t.px("--space-1") * 0.5;
    p.setRenderHint(QPainter::Antialiasing, false);
    const QColor ink = t.color("--color-on-chrome-muted");
    for (const MatchPoint &k : set_->keypoints(side)) {
      const auto c = at(side, k);
      if (!c)
        continue;
      p.setPen(k.xyz ? Qt::NoPen : QPen(ink, 1.0));
      p.setBrush(k.xyz ? QBrush(ink) : Qt::NoBrush);
      p.drawEllipse(*c, r, r);
    }
  }
  if (loading_[idx(side)]) {
    QColor veil = t.color("--color-canvas");
    veil.setAlphaF(0.45);
    p.fillRect(sr, veil);
  }
  p.restore();
}

void MatchCanvas::paintEvent(QPaintEvent *) {
  const Theme &t = theme();
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing);
  paint_side(p, MatchSide::a);
  paint_side(p, MatchSide::b);
  if (!set_)
    return;
  // The lines, each over a scrim halo so it reads on any photo.
  std::vector<int> line_of;
  const auto segs = segments(line_of);
  QColor halo = t.color("--color-scrim");
  halo.setAlphaF(0.6);
  const double w = 1.5;
  for (std::size_t k = 0; k < segs.size(); ++k) {
    const MatchLine &l = set_->lines()[static_cast<std::size_t>(line_of[k])];
    const bool hot = line_of[k] == hover_;
    const QLineF line(segs[k].x0, segs[k].y0, segs[k].x1, segs[k].y1);
    p.setPen(QPen(halo, (hot ? 2 * w : w) + 2.0, Qt::SolidLine, Qt::RoundCap));
    p.drawLine(line);
    p.setPen(QPen(t.color(tone_token(line_tone(l))), hot ? 2 * w : w,
                  l.manual ? Qt::DashLine : Qt::SolidLine, Qt::RoundCap));
    p.drawLine(line);
    if (hot) {
      p.setBrush(Qt::NoBrush);
      const double ring = t.px("--space-2") * 0.75;
      p.drawEllipse(line.p1(), ring, ring);
      p.drawEllipse(line.p2(), ring, ring);
    }
  }
  // The half-made manual match: a ring where the first click landed.
  if (pick_ && pick_->pending()) {
    if (const auto c = at(*pick_->side(), *pick_->point())) {
      p.setPen(QPen(t.color("--color-accent"), 2.0));
      p.setBrush(Qt::NoBrush);
      const double ring = t.px("--space-2");
      p.drawEllipse(*c, ring, ring);
    }
  }
  if (rubber_) {
    p.setPen(QPen(t.color("--color-accent"), 1.0, Qt::DashLine));
    QColor fill = t.color("--color-accent");
    fill.setAlphaF(0.12);
    p.setBrush(fill);
    p.drawRect(rubber_->normalized());
  }
}

void MatchCanvas::mousePressEvent(QMouseEvent *e) {
  button_ = e->button();
  press_ = last_ = e->position();
  press_side_ = side_at(e->position());
  dragged_ = false;
  if (button_ == Qt::LeftButton && mode_ == MatchMode::select)
    setCursor(Qt::ClosedHandCursor);
}

void MatchCanvas::mouseMoveEvent(QMouseEvent *e) {
  const QPointF pos = e->position();
  if (button_ == Qt::NoButton) {
    const int h = line_at(pos);
    if (h != hover_) {
      hover_ = h;
      update();
    }
    return;
  }
  if ((pos - press_).manhattanLength() > theme().px("--space-1"))
    dragged_ = true;
  if (!dragged_ || !press_side_)
    return;
  const bool pan = button_ == Qt::MiddleButton ||
                   (button_ == Qt::LeftButton && mode_ != MatchMode::region);
  if (pan) {
    ImageView &v = views_[idx(*press_side_)];
    if (v.zoom > 1.0) { // a fitted image does not pan
      v.pan_x += pos.x() - last_.x();
      v.pan_y += pos.y() - last_.y();
      update();
    }
  } else if (button_ == Qt::LeftButton && mode_ == MatchMode::region) {
    rubber_ = QRectF(press_, pos);
    update();
  }
  last_ = pos;
}

void MatchCanvas::mouseReleaseEvent(QMouseEvent *e) {
  const QPointF pos = e->position();
  const Qt::MouseButton b = button_;
  button_ = Qt::NoButton;
  if (mode_ == MatchMode::select)
    setCursor(Qt::OpenHandCursor);
  if (b != Qt::LeftButton || !press_side_)
    return;
  const MatchSide side = *press_side_;
  const QSize s = image_size(side);
  if (mode_ == MatchMode::region && rubber_) {
    rubber_.reset();
    if (!s.isEmpty()) {
      const ViewRect d = drawn_rect(side);
      const auto p0 = canvas_to_image(d, s.width(), s.height(), press_.x(), press_.y());
      const auto p1 = canvas_to_image(d, s.width(), s.height(), pos.x(), pos.y());
      const PixelBox box =
          box_from_drag(p0[0], p0[1], p1[0], p1[1], s.width(), s.height());
      if (!box.empty())
        emit region_dragged(side, box);
    }
    update();
    return;
  }
  if (dragged_)
    return; // a pan, not a click
  if (mode_ == MatchMode::select) {
    if (const int l = line_at(pos); l >= 0)
      emit line_clicked(l);
  } else if (mode_ == MatchMode::add && !s.isEmpty()) {
    const auto px = canvas_to_image(drawn_rect(side), s.width(), s.height(),
                                    pos.x(), pos.y());
    if (px[0] >= 0 && px[1] >= 0 && px[0] <= s.width() - 1 &&
        px[1] <= s.height() - 1)
      emit pixel_clicked(side, static_cast<float>(px[0]),
                         static_cast<float>(px[1]));
  }
}

void MatchCanvas::mouseDoubleClickEvent(QMouseEvent *e) {
  if (const auto side = side_at(e->position())) {
    views_[idx(*side)] = {};
    update();
  }
}

void MatchCanvas::wheelEvent(QWheelEvent *e) {
  const auto side = side_at(e->position());
  const QSize s = side ? image_size(*side) : QSize();
  if (!side || s.isEmpty() || e->angleDelta().y() == 0)
    return;
  const double factor =
      std::pow(kWheelStep, e->angleDelta().y() / 120.0);
  ImageView &v = views_[idx(*side)];
  v = zoom_about(slots()[idx(*side)], s.width(), s.height(), v,
                 e->position().x(), e->position().y(), factor);
  hover_ = -1;
  update();
  e->accept();
}

void MatchCanvas::keyPressEvent(QKeyEvent *e) {
  if (e->key() == Qt::Key_Escape) {
    rubber_.reset();
    emit cancel_requested();
    update();
    return;
  }
  QWidget::keyPressEvent(e);
}

void MatchCanvas::leaveEvent(QEvent *e) {
  hover_ = -1;
  update();
  QWidget::leaveEvent(e);
}

} // namespace rux::qt
```

- [ ] **Step 4: Build and lint**

```bash
nix develop -c cmake --build build -j8 --target rux_qt_lib
nix develop -c python .claude/skills/design-studio/scripts/token_lint.py apps/rux/qt --qt
```
Expected: builds (AUTOMOC picks up the new header through the
`include/rux_qt/*.hpp` glob); lint clean. Fix any literal it flags with
a token (`setAlphaF` fractions are not colours and are allowed, as in
`FrameBrowser.cpp`).

- [ ] **Step 5: Commit**

```bash
nix develop -c git add apps/rux/qt/include/rux_qt/MatchCanvas.hpp apps/rux/qt/src/MatchCanvas.cpp apps/rux/qt/include/rux_qt/FrameBrowser.hpp apps/rux/qt/src/FrameBrowser.cpp
nix develop -c git commit -m "qt: MatchCanvas, A | B with keypoints and match lines

Per-image zoom about the cursor and pan, lines coloured by the last fit
(tokens only), manual lines dashed, hover, Vælg/Tilføj/Område input."
```
(The canvas is shown and screenshotted in Tasks 7–8; it has no page yet.)

---

### Task 7: `PairImages` becomes the match editor, wired to the overlay

**Files:**
- Modify: `apps/rux/qt/include/rux_qt/PairImages.hpp` (rewrite)
- Modify: `apps/rux/qt/src/PairImages.cpp` (rewrite)
- Modify: `apps/rux/qt/src/EditWorkspace.cpp` (connect `align_ready`)
- Modify: `apps/rux/qt/src/PairStrip.cpp:436` (wording)
- Modify: `apps/rux/qt/styles/app.qss` (new object names)

**Interfaces:**
- Consumes:
  - Task 2/3: `slam::match_frame_pair`, `slam::lift_pixel`,
    `slam::rigid_from_matches`, `slam::RigidFitFailure`,
    `slam::FrameMatchOptions`, `slam::PixelRect`;
  - Task 4: `MatchSet`, `ManualPick`, the text functions;
  - Task 6: `MatchCanvas`;
  - existing: `FrameImageLoader`, `BackgroundWork`, `pair_image_caption`
    and `pair_image_hint`, `offset_from_matrix`, `PairOverlay::set_offset`
    and `pivot()`.
- Produces (used by the gallery in Task 8):
  ```cpp
  class PairImages : public QWidget {
    void set_pair(int a, int b, Slot next);
    void set_layer(FrameLayer); FrameLayer layer() const; QWidget *layer_switch() const;
    void find_matches();
    void align_from_matches();
    void set_mode(MatchMode); MatchMode mode() const;
    void pick(MatchSide side, float u, float v);     // Tilføj: lift, then add
    void toggle_line(int line);
    void set_region(MatchSide side, PixelBox box);
    void clear_regions();
    void align_applied(bool placed);                 // EditWorkspace's answer to align_ready
    const MatchSet &matches() const;
    MatchCanvas *canvas() const;
    QString status_text() const;
   signals:
    void align_ready(int a, int b, const rux::qt::Mat4 &world_delta);
  };
  ```

- [ ] **Step 1: Rewrite the header**

Replace `apps/rux/qt/include/rux_qt/PairImages.hpp` with:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Rediger's pair images as the match editor (spec 2026-10-14 §4): A | B on
// one MatchCanvas (colour or depth, decoded off the GUI thread by its own
// FrameImageLoader), and a toolbar — Find match (ORB + ratio test,
// slam::match_frame_pair), the Vælg / Tilføj / Område modes, Ryd område,
// Placér B fra match (slam::rigid_from_matches on the enabled matches with
// depth at both ends) — and one status line. Find, a click's lift and the
// fit run on detached threads with their own read-only ProjectDB; a result
// for a pair no longer shown is dropped (generation). A different pair
// clears the matches (C4). A fit is handed out as align_ready(world_delta);
// EditWorkspace places B and answers align_applied().

#include <rux_qt/FrameBrowser.hpp>
#include <rux_qt/graph_view.hpp>
#include <rux_qt/match_editor.hpp>
#include <rux_qt/pair_alignment.hpp>

#include <QWidget>

#include <memory>

class QButtonGroup;
class QLabel;
class QPushButton;

namespace rux::qt {

class ElidedLabel;
class FrameImageLoader;
class MatchCanvas;
class ProjectSession;

class PairImages : public QWidget {
  Q_OBJECT
    public:
  explicit PairImages(ProjectSession &session, QWidget *parent = nullptr);
  /// A and B (-1 = empty) and the slot the next plain click fills, which
  /// an empty side's hint names. A different pair clears the matches.
  void set_pair(int a, int b, Slot next);
  void set_layer(FrameLayer layer);
  FrameLayer layer() const { return layer_; }
  /// The Farve / Dybde switch, for the pane header.
  QWidget *layer_switch() const { return layer_switch_; }

  void find_matches();
  void align_from_matches();
  void set_mode(MatchMode mode);
  MatchMode mode() const { return mode_; }
  /// A Tilføj click at colour pixel (u, v) of @p side: lifted off-thread,
  /// then the first or second end of a manual match.
  void pick(MatchSide side, float u, float v);
  void toggle_line(int line);
  void set_region(MatchSide side, PixelBox box);
  void clear_regions();
  /// Whether EditWorkspace placed B on the last align_ready.
  void align_applied(bool placed);
  const MatchSet &matches() const { return set_; }
  MatchCanvas *canvas() const { return canvas_; }
  QString status_text() const;

    signals:
  /// Placér B fra match fitted the current pair: @p world_delta is B's new
  /// world-space offset (RigidFit::world_delta, the overlay's ΔT).
  void align_ready(int a, int b, const rux::qt::Mat4 &world_delta);

    private:
  enum class Busy { none, finding, lifting, aligning };
  struct Found;
  struct Lifted;
  struct Fitted;
  void reload();
  void sync_images();
  void frame_ready(int id);
  void found(std::shared_ptr<Found> r);
  void lifted(std::shared_ptr<Lifted> r);
  void fitted(std::shared_ptr<Fitted> r);
  void note(const QString &text, const QString &tone);
  void refresh();
  bool pair_ok() const;

  ProjectSession &session_;
  FrameImageLoader *loader_ = nullptr;
  MatchCanvas *canvas_ = nullptr;
  QLabel *caption_a_ = nullptr;
  QLabel *caption_b_ = nullptr;
  QButtonGroup *layers_ = nullptr;
  QWidget *layer_switch_ = nullptr;
  QButtonGroup *modes_ = nullptr;
  QPushButton *find_ = nullptr;
  QPushButton *align_ = nullptr;
  QPushButton *clear_region_ = nullptr;
  ElidedLabel *status_ = nullptr;
  FrameLayer layer_ = FrameLayer::color;
  MatchMode mode_ = MatchMode::select;
  MatchSet set_;
  ManualPick pick_;
  int a_ = -1, b_ = -1;
  Slot next_ = Slot::a;
  unsigned generation_ = 0; ///< bumped on every pair change / reload
  Busy busy_ = Busy::none;
  QString note_, tone_;     ///< the last event; empty = the read-out
  int last_inliers_ = 0;
  double last_rms_ = 0.0;
};

} // namespace rux::qt
```

- [ ] **Step 2: Rewrite the implementation**

Replace `apps/rux/qt/src/PairImages.cpp` with:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <rux_qt/FrameImageLoader.hpp>
#include <rux_qt/MatchCanvas.hpp>
#include <rux_qt/PairImages.hpp>
#include <rux_qt/ProjectSession.hpp>
#include <rux_qt/Theme.hpp>
#include <rux_qt/background.hpp>
#include <rux_qt/widgets.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/slam/frame_pair_matches.hpp>

#include <QApplication>
#include <QButtonGroup>
#include <QFrame>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPointer>
#include <QPushButton>

#include <thread>

namespace rux::qt {
namespace slam = reusex::slam;

namespace {
/// Rediger's fit threshold: rigid_from_matches' default, named for the text.
constexpr double kInlierDistM = 0.05;

MatchPoint to_point(const slam::FrameKeypoint &k) {
  MatchPoint p;
  p.u = k.u;
  p.v = k.v;
  if (k.xyz)
    p.xyz = Vec3{(*k.xyz)[0], (*k.xyz)[1], (*k.xyz)[2]};
  return p;
}

std::optional<slam::PixelRect> to_rect(const std::optional<PixelBox> &b) {
  if (!b)
    return std::nullopt;
  return slam::PixelRect{b->x, b->y, b->w, b->h};
}

QPushButton *segment(const QString &text, int position, int count) {
  auto *b = new QPushButton(NavItem::escape_mnemonic(text));
  b->setObjectName("segment");
  b->setCheckable(true);
  b->setCursor(Qt::PointingHandCursor);
  b->setProperty("position", position == 0             ? "first"
                             : position == count - 1   ? "last"
                                                       : "middle");
  return b;
}
} // namespace

struct PairImages::Found {
  unsigned generation = 0;
  std::vector<MatchPoint> a, b;
  std::vector<MatchLine> lines;
  QString error;
};
struct PairImages::Lifted {
  unsigned generation = 0;
  MatchSide side = MatchSide::a;
  std::optional<MatchPoint> point;
  QString error;
};
struct PairImages::Fitted {
  unsigned generation = 0;
  FitInput input;
  slam::RigidFit fit;
  QString error;
};

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

  // Farve / Dybde, placed in the pane header by EditWorkspace.
  auto *seg = new QFrame;
  seg->setObjectName("segmented");
  auto *sl = new QHBoxLayout(seg);
  sl->setContentsMargins(0, 0, 0, 0);
  sl->setSpacing(0);
  layers_ = new QButtonGroup(this);
  const char *layer_names[] = {"Farve", "Dybde"};
  for (int i = 0; i < 2; ++i) {
    auto *b = segment(layer_names[i], i, 2);
    layers_->addButton(b, i);
    sl->addWidget(b);
  }
  layers_->button(0)->setChecked(true);
  layer_switch_ = seg;

  // Row 1: Find match and the mode; row 2: Placér B, Ryd område, status.
  // Two rows: the column is ~375 px at 1440 and must never widen it.
  auto *row1 = new QHBoxLayout;
  row1->setSpacing(t.px("--space-2"));
  find_ = new QPushButton(NavItem::escape_mnemonic("Find match"));
  find_->setProperty("kind", "secondary");
  find_->setCursor(Qt::PointingHandCursor);
  find_->setToolTip("Find ORB-match mellem A og B (inden for områderne, "
                    "hvis du har tegnet nogen). Ændrer ikke projektet.");
  row1->addWidget(find_);
  auto *modes = new QFrame;
  modes->setObjectName("segmented");
  auto *ml = new QHBoxLayout(modes);
  ml->setContentsMargins(0, 0, 0, 0);
  ml->setSpacing(0);
  modes_ = new QButtonGroup(this);
  const char *mode_names[] = {"Vælg", "Tilføj", "Område"};
  const char *mode_tips[] = {
      "Klik på en linje for at udelade eller tage den med; træk for at "
      "panorere, hjul zoomer, dobbeltklik nulstiller",
      "Klik et punkt i A og så det samme i B (Esc fortryder)",
      "Træk et rektangel i A eller B: næste Find match søger kun der"};
  for (int i = 0; i < 3; ++i) {
    auto *b = segment(mode_names[i], i, 3);
    b->setToolTip(mode_tips[i]);
    modes_->addButton(b, i);
    ml->addWidget(b);
  }
  modes_->button(0)->setChecked(true);
  row1->addWidget(modes);
  row1->addStretch(1);

  auto *row2 = new QHBoxLayout;
  row2->setSpacing(t.px("--space-2"));
  align_ = new QPushButton(NavItem::escape_mnemonic("Placér B fra match"));
  align_->setProperty("kind", "primary");
  align_->setCursor(Qt::PointingHandCursor);
  align_->setToolTip("Flyt B i 3D-overlayet til den stive placering, de "
                     "valgte match med dybde giver. ICP-forfin starter derfra.");
  row2->addWidget(align_);
  clear_region_ = new QPushButton(NavItem::escape_mnemonic("Ryd område"));
  clear_region_->setObjectName("regionChip");
  clear_region_->setProperty("kind", "ghost");
  clear_region_->setCursor(Qt::PointingHandCursor);
  row2->addWidget(clear_region_);
  status_ = new ElidedLabel(QString(), Qt::ElideRight);
  status_->setObjectName("matchStatus");
  status_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
  row2->addWidget(status_, 1);

  auto *bar = new QFrame;
  bar->setObjectName("matchToolbar");
  auto *bv = new QVBoxLayout(bar);
  bv->setContentsMargins(0, 0, 0, 0);
  bv->setSpacing(t.px("--space-1"));
  bv->addLayout(row1);
  bv->addLayout(row2);
  g->addWidget(bar, 0, 0, 1, 2);

  caption_a_ = new QLabel;
  caption_a_->setObjectName("pairCaptionA");
  caption_b_ = new QLabel;
  caption_b_->setObjectName("pairCaptionB");
  g->addWidget(caption_a_, 1, 0);
  g->addWidget(caption_b_, 1, 1);
  canvas_ = new MatchCanvas;
  canvas_->set_data(&set_, &pick_);
  g->addWidget(canvas_, 2, 0, 1, 2);
  g->setRowStretch(2, 1);
  g->setColumnStretch(0, 1);
  g->setColumnStretch(1, 1);

  connect(layers_, &QButtonGroup::idClicked, this, [this](int id) {
    set_layer(id == 1 ? FrameLayer::depth : FrameLayer::color);
  });
  connect(modes_, &QButtonGroup::idClicked, this,
          [this](int id) { set_mode(static_cast<MatchMode>(id)); });
  connect(find_, &QPushButton::clicked, this, &PairImages::find_matches);
  connect(align_, &QPushButton::clicked, this, &PairImages::align_from_matches);
  connect(clear_region_, &QPushButton::clicked, this, &PairImages::clear_regions);
  connect(canvas_, &MatchCanvas::line_clicked, this, &PairImages::toggle_line);
  connect(canvas_, &MatchCanvas::pixel_clicked, this, &PairImages::pick);
  connect(canvas_, &MatchCanvas::region_dragged, this, &PairImages::set_region);
  connect(canvas_, &MatchCanvas::cancel_requested, this, [this] {
    pick_.cancel();
    note({}, {});
  });
  connect(loader_, &FrameImageLoader::frame_ready, this, &PairImages::frame_ready);
  connect(&session_, &ProjectSession::state_changed, this, &PairImages::reload);
  reload();
}

bool PairImages::pair_ok() const {
  return session_.is_open() && a_ >= 0 && b_ >= 0 && a_ != b_;
}

void PairImages::reload() {
  loader_->set_project(session_.is_open() ? session_.path() : QString());
  ++generation_;
  busy_ = Busy::none;
  set_.clear();
  set_.set_pair(-1, -1); // the next set_pair() is always a new pair
  set_.set_pair(a_, b_);
  pick_.cancel();
  note_.clear();
  canvas_->reset_views();
  sync_images();
  refresh();
}

void PairImages::set_pair(int a, int b, Slot next) {
  const bool new_pair = a != a_ || b != b_;
  a_ = a;
  b_ = b;
  next_ = next;
  if (new_pair) {
    ++generation_; // anything still running describes the old pair
    busy_ = Busy::none;
    set_.set_pair(a, b);
    pick_.cancel();
    note_.clear();
    canvas_->reset_views();
  }
  sync_images();
  refresh();
}

void PairImages::set_layer(FrameLayer layer) {
  layer_ = layer;
  if (QAbstractButton *b = layers_->button(layer == FrameLayer::depth ? 1 : 0))
    b->setChecked(true);
  canvas_->set_layer(layer);
}

void PairImages::set_mode(MatchMode mode) {
  mode_ = mode;
  if (QAbstractButton *b = modes_->button(static_cast<int>(mode)))
    b->setChecked(true);
  if (mode != MatchMode::add)
    pick_.cancel();
  canvas_->set_mode(mode);
  refresh();
}

void PairImages::sync_images() {
  auto side = [this](MatchSide s, QLabel *caption, Slot slot, int id) {
    caption->setText(QString::fromStdString(pair_image_caption(slot, id)));
    if (id < 0) {
      canvas_->set_message(
          s, QString::fromStdString(pair_image_hint(slot, a_, b_, next_)));
      return;
    }
    if (auto f = loader_->frame(id)) {
      canvas_->set_frame(s, f);
      return;
    }
    canvas_->set_message(s, {});
    canvas_->set_loading(s, true);
    loader_->request(id);
  };
  side(MatchSide::a, caption_a_, Slot::a, a_);
  side(MatchSide::b, caption_b_, Slot::b, b_);
}

void PairImages::frame_ready(int id) {
  if (id != a_ && id != b_)
    return;
  auto f = loader_->frame(id);
  if (!f)
    return;
  if (id == a_)
    canvas_->set_frame(MatchSide::a, f);
  if (id == b_)
    canvas_->set_frame(MatchSide::b, f);
}

void PairImages::find_matches() {
  if (!pair_ok() || busy_ != Busy::none)
    return;
  slam::FrameMatchOptions o;
  o.region_a = to_rect(set_.region(MatchSide::a));
  o.region_b = to_rect(set_.region(MatchSide::b));
  busy_ = Busy::finding;
  note("Finder match …", "wait");
  const std::string path = session_.path().toStdString();
  const int a = a_, b = b_;
  const unsigned gen = generation_;
  auto work = std::make_shared<BackgroundWork>();
  QPointer<PairImages> guard(this);
  std::thread([path, a, b, o, gen, guard, work]() mutable {
    auto r = std::make_shared<Found>();
    r->generation = gen;
    try {
      reusex::ProjectDB db(path, /*readOnly=*/true);
      const auto m = slam::match_frame_pair(db, a, b, o);
      for (const auto &k : m.keypoints_a)
        r->a.push_back(to_point(k));
      for (const auto &k : m.keypoints_b)
        r->b.push_back(to_point(k));
      for (const auto &mm : m.matches) {
        MatchLine l;
        l.a = mm.a;
        l.b = mm.b;
        l.distance = mm.distance;
        r->lines.push_back(l);
      }
    } catch (const std::exception &e) {
      r->error = QString::fromUtf8(e.what());
    }
    QMetaObject::invokeMethod(
        qApp, [guard, r] { if (guard) guard->found(r); }, Qt::QueuedConnection);
    work.reset();
  }).detach();
}

void PairImages::found(std::shared_ptr<Found> r) {
  if (r->generation != generation_)
    return; // another pair meanwhile: dropped
  busy_ = Busy::none;
  if (!r->error.isEmpty()) {
    note("Find match fejlede: " + r->error, "crit");
    return;
  }
  set_.set_detected(std::move(r->a), std::move(r->b), std::move(r->lines));
  note({}, {});
}

void PairImages::pick(MatchSide side, float u, float v) {
  if (!pair_ok() || busy_ != Busy::none)
    return;
  busy_ = Busy::lifting;
  refresh();
  const std::string path = session_.path().toStdString();
  const int id = side == MatchSide::a ? a_ : b_;
  const unsigned gen = generation_;
  auto work = std::make_shared<BackgroundWork>();
  QPointer<PairImages> guard(this);
  std::thread([path, id, side, u, v, gen, guard, work]() mutable {
    auto r = std::make_shared<Lifted>();
    r->generation = gen;
    r->side = side;
    try {
      reusex::ProjectDB db(path, /*readOnly=*/true);
      if (const auto k = slam::lift_pixel(db, id, u, v))
        r->point = to_point(*k);
    } catch (const std::exception &e) {
      r->error = QString::fromUtf8(e.what());
    }
    QMetaObject::invokeMethod(
        qApp, [guard, r] { if (guard) guard->lifted(r); }, Qt::QueuedConnection);
    work.reset();
  }).detach();
}

void PairImages::lifted(std::shared_ptr<Lifted> r) {
  if (r->generation != generation_)
    return;
  busy_ = Busy::none;
  if (!r->error.isEmpty()) {
    note("Punktet kunne ikke læses: " + r->error, "crit");
    return;
  }
  if (!r->point) {
    note("ingen dybde her — klik tættere på en flade med dybde", "warn");
    return;
  }
  if (const auto done = pick_.pick(r->side, *r->point)) {
    set_.add_manual(done->first, done->second);
    note({}, {});
    return;
  }
  note(r->side == MatchSide::a ? "Klik nu det samme punkt i B (Esc fortryder)"
                               : "Klik nu det samme punkt i A (Esc fortryder)",
       {});
}

void PairImages::toggle_line(int line) {
  set_.toggle_excluded(line);
  note({}, {});
}

void PairImages::set_region(MatchSide side, PixelBox box) {
  set_.set_region(side, box);
  note(side == MatchSide::a ? "Område i A — Find match søger kun der"
                            : "Område i B — Find match søger kun der",
       {});
}

void PairImages::clear_regions() {
  set_.clear_regions();
  note({}, {});
}

void PairImages::align_from_matches() {
  if (!pair_ok() || busy_ != Busy::none)
    return;
  const FitInput in = set_.fit_input();
  if (in.pairs.size() < 3) {
    note(QString::fromStdString(match_fit_failure_text(
             MatchFitFailure::too_few_pairs, static_cast<int>(in.pairs.size()),
             0, kInlierDistM)),
         "warn");
    return;
  }
  busy_ = Busy::aligning;
  note("Placerer B …", "wait");
  const std::string path = session_.path().toStdString();
  const int a = a_, b = b_;
  const unsigned gen = generation_;
  auto work = std::make_shared<BackgroundWork>();
  QPointer<PairImages> guard(this);
  std::thread([path, a, b, in, gen, guard, work]() mutable {
    auto r = std::make_shared<Fitted>();
    r->generation = gen;
    r->input = in;
    try {
      reusex::ProjectDB db(path, /*readOnly=*/true);
      for (int id : {a, b})
        if (!db.has_sensor_frame_pose(id))
          throw std::runtime_error("frame " + std::to_string(id) +
                                   " has no stored pose");
      std::vector<slam::PointPair> pairs;
      pairs.reserve(in.pairs.size());
      for (const auto &[pa, pb] : in.pairs)
        pairs.emplace_back(pa, pb);
      r->fit = slam::rigid_from_matches(db.sensor_frame_pose(a),
                                        db.sensor_frame_pose(b), pairs,
                                        kInlierDistM);
    } catch (const std::exception &e) {
      r->error = QString::fromUtf8(e.what());
    }
    QMetaObject::invokeMethod(
        qApp, [guard, r] { if (guard) guard->fitted(r); }, Qt::QueuedConnection);
    work.reset();
  }).detach();
}

void PairImages::fitted(std::shared_ptr<Fitted> r) {
  if (r->generation != generation_)
    return;
  busy_ = Busy::none;
  if (!r->error.isEmpty()) {
    note("Placér B fejlede: " + r->error, "crit");
    return;
  }
  const slam::RigidFit &f = r->fit;
  if (!f.ok) {
    const MatchFitFailure why =
        f.failure == slam::RigidFitFailure::too_few_pairs
            ? MatchFitFailure::too_few_pairs
        : f.failure == slam::RigidFitFailure::degenerate
            ? MatchFitFailure::degenerate
            : MatchFitFailure::no_consensus;
    note(QString::fromStdString(match_fit_failure_text(
             why, static_cast<int>(r->input.pairs.size()),
             static_cast<int>(f.inliers.size()), kInlierDistM)),
         "warn");
    return;
  }
  set_.apply_fit(r->input, f.inliers);
  last_inliers_ = static_cast<int>(f.inliers.size());
  last_rms_ = f.rms_m;
  emit align_ready(a_, b_, f.world_delta); // EditWorkspace answers align_applied
}

void PairImages::align_applied(bool placed) {
  const QString fit =
      QString::fromStdString(match_fit_text(last_inliers_, last_rms_));
  note(placed ? fit : fit + " — B er ikke flyttet: 3D-overlayet var ikke klar.",
       placed ? "good" : "warn");
}

void PairImages::note(const QString &text, const QString &tone) {
  note_ = text;
  tone_ = tone;
  refresh();
}

QString PairImages::status_text() const { return status_->full_text(); }

void PairImages::refresh() {
  const QString readout =
      QString::fromStdString(match_readout(set_.counts()));
  status_->set_full_text(note_.isEmpty() ? readout : note_);
  status_->setToolTip(note_.isEmpty() ? readout : note_ + "\n" + readout);
  status_->setProperty("tone", tone_.isEmpty() ? QVariant() : QVariant(tone_));
  repolish(status_);
  const bool ok = pair_ok();
  const bool idle = busy_ == Busy::none;
  find_->setEnabled(ok && idle);
  align_->setEnabled(ok && idle && !set_.lines().empty());
  for (QAbstractButton *b : modes_->buttons())
    b->setEnabled(ok);
  clear_region_->setVisible(set_.has_region());
  canvas_->update();
}

} // namespace rux::qt
```
`QVBoxLayout` is used for `bv`, so add `#include <QVBoxLayout>`.

`reload()` uses `set_pair(-1, -1)` followed by `set_pair(a_, b_)`. That
guarantees a clear on the same pair (a reload of the project), because
`set_pair` only clears on change. The explicit `set_.clear()` above it
already does that; keep both, the second line documents intent.

- [ ] **Step 3: Wire EditWorkspace and the ICP wording**

In `EditWorkspace.cpp`, after the `icp_aligned` connect, add:

```cpp
  // Placér B fra match: the fit's world_delta is the overlay's ΔT (the
  // same matrix ICP's world_delta is), placed exactly as ICP's result is.
  connect(images_, &PairImages::align_ready, this,
          [this](int a, int b, const Mat4 &delta) {
            const bool placed =
                a == slots_.a() && b == slots_.b() && overlay_->ready();
            if (placed)
              overlay_->set_offset(offset_from_matrix(delta, overlay_->pivot()));
            images_->align_applied(placed);
          });
```
In `PairStrip.cpp`, replace `QString("Fra din manuelle placering · ")`
with `QString("Fra B's placering · ")`. The seed may now come from the
match editor as well as the hand.

- [ ] **Step 4: Styles**

In `app.qss`, after the `QLabel#icpLine[tone="wait"]` rule:

```css
/* Rediger's match editor: the status line under its toolbar. */
QLabel#matchStatus {
  font-size: var(--font-size-xs);
  color: var(--color-text-muted);
}
QLabel#matchStatus[tone="good"] { color: var(--color-status-succeeded); }
QLabel#matchStatus[tone="warn"] { color: var(--tone-warn-ink); }
QLabel#matchStatus[tone="crit"] { color: var(--color-status-failed); }
QLabel#matchStatus[tone="wait"] { color: var(--color-status-running); }
QPushButton#regionChip {
  padding: var(--space-1) var(--space-2);
  font-size: var(--font-size-xs);
}
```
Check that `#segment[position="middle"]` needs no rule. The existing
`QPushButton#segment` covers it, and `first`/`last` only round the
corners. If the middle segment shows a doubled border in the shot, add
`QPushButton#segment[position="middle"] { border-left: none; }` to match
how `last` is styled.

- [ ] **Step 5: Build, smoke-shoot the existing pages**

```bash
nix develop -c cmake --build build -j8 --target rux-qt-gallery rux reusex_unit_tests
S=.claude/skills/design-studio/scripts/qt_shot.sh
nix develop -c bash $S --no-build --page rediger --page rediger-pair --page rediger-pair-depth --page rediger-pair-half --page rediger-strip --theme both
nix develop -c python .claude/skills/design-studio/scripts/token_lint.py apps/rux/qt --qt
```
Expected:
- no `TOO WIDE`, `OVERLAY TOO SMALL` or `STRIP CLIPPED`;
- the images pane shows the two-row toolbar, the captions and both images
  on one canvas; the status reads "Ingen match endnu";
- lint is clean.

Read the PNGs. If `TOO WIDE` fires, the toolbar's row 1 is the minimum
width. Shorten nothing in the copy (the labels are fixed by the spec).
Instead, move `modes` to row 2 and `Placér B fra match` to row 1, which
balances the two rows, and re-shoot.

- [ ] **Step 6: Commit**

```bash
nix develop -c git add apps/rux/qt
nix develop -c git commit -m "qt: Rediger's pair images become the match editor

Find match, Vælg/Tilføj/Område, Ryd område and Placér B fra match over
one A | B canvas; find, lift and fit run off-thread and drop stale
results; the fit places B in the overlay exactly as ICP's result does."
```

---

### Task 8: Gallery pages, screenshots, critique

**Files:**
- Modify: `apps/rux/qt/gallery/shell_pages.cpp` (two pages)
- Modify: `.claude/skills/design-studio/references/qt-client.md`

**Interfaces:**
- Consumes: Task 7's `PairImages` API (`find_matches`, `align_from_matches`, `toggle_line`, `pick`, `set_region`, `set_mode`, `matches()`, `canvas()`, `status_text()`), `make_pg`, `settle`, `click_strip_button`, `report_page_failure`
- Produces: gallery pages `rediger-matches`, `rediger-matches-edit`; log lines `rediger-matches: …`

- [ ] **Step 1: Add the pages**

In `shell_pages.cpp` (include `<rux_qt/MatchCanvas.hpp>`,
`<rux_qt/PairImages.hpp>` is already there via EditWorkspace usage, add it
if not), after `make_rediger_icp_moved`:

```cpp
namespace {
/// The pair the match pages use: RUX_MATCHES_A / RUX_MATCHES_B on a real
/// copy (e.g. NewOffice 2240 / 2241), else two neighbouring frames of the
/// fixture (consecutive views overlap; make_pg's pair is 0.30 / 0.81).
AppShell *matches_shell(const PageContext &ctx) {
  auto *shell = make_pg(ctx);
  bool ok_a = false, ok_b = false;
  const int a = qEnvironmentVariableIntValue("RUX_MATCHES_A", &ok_a);
  const int b = qEnvironmentVariableIntValue("RUX_MATCHES_B", &ok_b);
  if (ok_a && ok_b) {
    shell->edit()->set_pair(a, b);
  } else {
    const auto &ids = shell->database()->frames()->pair().ids();
    if (ids.size() >= 2) {
      const std::size_t i = ids.size() * 3 / 10;
      shell->edit()->set_pair(ids[i], ids[std::min(i + 1, ids.size() - 1)]);
    }
  }
  settle(); // images and clouds
  return shell;
}

void log_matches(const char *page, PairImages *im) {
  const MatchCounts c = im->matches().counts();
  std::fprintf(stderr,
               "%s: %d matches, %d usable, %d inliers, %d manual, %d excluded "
               "— \"%s\"\n",
               page, c.total, c.usable, c.inliers, c.manual, c.excluded,
               qPrintable(im->status_text()));
}
} // namespace

/// Find match, then Placér B fra match: inliers green, outliers red, B
/// moved in the overlay. RUX_MATCHES_ICP=1 runs ICP-forfin after and logs
/// it (the real-data check, plan Task 9).
QWidget *make_rediger_matches(const PageContext &ctx) {
  auto *shell = matches_shell(ctx);
  PairImages *im = shell->edit()->images();
  im->find_matches();
  settle();
  log_matches("rediger-matches found", im);
  im->align_from_matches();
  settle();
  log_matches("rediger-matches aligned", im);
  const ManualOffset &o = shell->edit()->overlay()->offset();
  std::fprintf(stderr,
               "rediger-matches: B at x=%.4f y=%.4f z=%.4f yaw=%.3f "
               "pitch=%.3f roll=%.3f (moved %.1f cm, %.2f deg)\n",
               o.x, o.y, o.z, o.yaw_deg, o.pitch_deg, o.roll_deg,
               100.0 * std::sqrt(o.x * o.x + o.y * o.y + o.z * o.z),
               rotation_angle_deg(shell->edit()->overlay()->delta()));
  if (qEnvironmentVariableIntValue("RUX_MATCHES_ICP") == 1) {
    click_strip_button(shell, {"ICP-forfin"});
    settle();
    for (auto *l : shell->edit()->strip()->findChildren<QLabel *>("icpLine"))
      std::fprintf(stderr, "rediger-matches: ICP \"%s\"\n",
                   qPrintable(l->text()));
    const ManualOffset &p = shell->edit()->overlay()->offset();
    std::fprintf(stderr,
                 "rediger-matches: after ICP x=%.4f y=%.4f z=%.4f yaw=%.3f\n",
                 p.x, p.y, p.z, p.yaw_deg);
  }
  return shell;
}

/// After align: two lines excluded, one manual match added through the
/// real click path (lift_pixel off-thread), a region drawn in A, Område on.
QWidget *make_rediger_matches_edit(const PageContext &ctx) {
  auto *shell = matches_shell(ctx);
  PairImages *im = shell->edit()->images();
  im->find_matches();
  settle();
  im->align_from_matches();
  settle();
  const auto &lines = im->matches().lines();
  for (int i = 0; i < std::min<int>(2, static_cast<int>(lines.size())); ++i)
    im->toggle_line(i);
  // A manual match: the first keypoint pair with depth, re-lifted by
  // click. Copied out first: the add reallocates the set's vectors.
  std::optional<std::pair<MatchPoint, MatchPoint>> target;
  for (const MatchLine &l : lines) {
    const MatchPoint pa =
        im->matches().keypoints(MatchSide::a)[static_cast<std::size_t>(l.a)];
    const MatchPoint pb =
        im->matches().keypoints(MatchSide::b)[static_cast<std::size_t>(l.b)];
    if (!l.excluded && pa.xyz && pb.xyz) {
      target = std::pair{pa, pb};
      break;
    }
  }
  if (target) {
    im->set_mode(MatchMode::add);
    im->pick(MatchSide::a, target->first.u + 3.0f, target->first.v + 3.0f);
    settle();
    im->pick(MatchSide::b, target->second.u + 3.0f, target->second.v + 3.0f);
    settle();
  }
  const QSize s = im->canvas()->image_size(MatchSide::a);
  im->set_region(MatchSide::a,
                 PixelBox{s.width() / 8, s.height() / 4, s.width() / 2,
                          s.height() / 2});
  im->set_mode(MatchMode::region);
  log_matches("rediger-matches-edit", im);
  if (im->matches().counts().manual != 1) {
    std::fprintf(stderr, "rux-qt-gallery: ERROR rediger-matches-edit: the "
                         "manual match was not added\n");
    report_page_failure();
  }
  return shell;
}
```
Register them after `rediger-icp-moved`:

```cpp
  register_page({"rediger-matches",
                 "Rediger: Find match og Placér B fra match — inliers og "
                 "outliers, B flyttet",
                 make_rediger_matches});
  register_page({"rediger-matches-edit",
                 "Rediger: match udeladt, et manuelt match og et område i A",
                 make_rediger_matches_edit});
```
Also log the match canvas size in both pages (copy of the overlay log):
`std::fprintf(stderr, "rediger-matches: canvas %dx%d\n", im->canvas()->width(), im->canvas()->height());`
right after `matches_shell`.

- [ ] **Step 2: Build and shoot both themes**

```bash
nix develop -c cmake --build build -j8 --target rux-qt-gallery rux
S=.claude/skills/design-studio/scripts/qt_shot.sh
nix develop -c bash $S --no-build --page rediger-matches --page rediger-matches-edit --theme both 2>&1 | tee /tmp/t8-shots.log
grep -E "rediger-matches|overlay canvas|pair strip" /tmp/t8-shots.log
```
Expected:
- four PNGs and no guard error;
- `rediger-matches aligned: N matches, … K inliers` with K ≥ 3, and
  "Fra matches: K inliers · RMS …";
- `rediger-matches-edit` has 1 manual and 2 excluded;
- the overlay canvas H ≥ 250.

If the fixture pair yields fewer than 3 usable matches, check the frame
ids in the log. Try `RUX_MATCHES_A`/`RUX_MATCHES_B` set to two
neighbouring ids from `rux -p <fixture copy> info`, then hard-code the
better neighbour offset (`i + 1` → `i + 2`) in `matches_shell`.

- [ ] **Step 3: Look, critique, fix (2–3 rounds)**

Open all four PNGs with Read. Check against
`.claude/skills/design-studio/references/critique-checklist.md` ("Qt
client"):

- lines legible over the photo in both themes;
- inlier/outlier/excluded distinguishable at a glance;
- the dashed manual line;
- hollow dots for no-depth keypoints;
- the region rectangle and the "Ryd område" chip;
- the toolbar not crowding the 375 px column;
- the captions aligned over each image.

Write the 3–5 biggest problems, fix them using tokens only, and re-shoot.
Remove one decorative element that isn't earning its place.

- [ ] **Step 4: Document the pages**

In `references/qt-client.md`'s Rediger paragraph add:

> `rediger-matches` (Find match, then Placér B fra match: inliers green,
> outliers red, B moved; on a real copy pass
> `RUX_MATCHES_A=<id> RUX_MATCHES_B=<id>`, and `RUX_MATCHES_ICP=1` to run
> ICP-forfin after it and log the result), `rediger-matches-edit` (two
> lines excluded, one manual match added through the click path, a region
> in A).

- [ ] **Step 5: Lint, full tests, commit**

```bash
nix develop -c python .claude/skills/design-studio/scripts/token_lint.py apps/rux/qt --qt
nix develop -c cmake --build build -j8
nix develop -c ctest --test-dir build --output-on-failure --parallel 8 2>&1 | tail -5
nix develop -c git add apps/rux/qt .claude/skills/design-studio/references/qt-client.md
nix develop -c git commit -m "qt gallery: rediger-matches and rediger-matches-edit"
```
Expected: lint clean; `100% tests passed` except the ones listed in
`build/baseline-failures.txt` (Task 0).

---

### Task 9: Real-data check on NewOffice, close `reusex-xlg.2`

**Files:** none tracked (results go into the bead and the hand-off)

**Interfaces:**
- Consumes: the `rediger-matches` page with `RUX_MATCHES_A/B/ICP` (Task 8), `$SCRATCH/newoffice.rux` (made in Task 1 Step 6; re-make it if missing — never touch the source)
- Produces: the numbers spec §4.5 asks for

- [ ] **Step 1: Pick a good-odometry pair**

```bash
python3 - <<'EOF'
import sqlite3, os
db = os.path.expandvars("$SCRATCH/newoffice.rux")
c = sqlite3.connect(f"file:{db}?mode=ro", uri=True)
print(c.execute("""select from_node_id, to_node_id, residual, deviation_m
                   from pose_graph_edges
                   where edge_type='odometry' and from_node_id between 2100 and 2400
                     and residual is not null
                   order by coalesce(deviation_m, 0), residual limit 3""").fetchall())
EOF
```
Take the first row's `(from, to)` as the good pair. (The query reads the
scratch copy, read-only.)

- [ ] **Step 2: Run both pairs through the real code path**

```bash
S=.claude/skills/design-studio/scripts/qt_shot.sh
RUX_MATCHES_A=2240 RUX_MATCHES_B=2241 RUX_MATCHES_ICP=1 nix develop -c bash $S --no-build --page rediger-matches --theme dark --project $SCRATCH/newoffice.rux --out shots/qt/newoffice 2>&1 | tee /tmp/no-2240.log
RUX_MATCHES_A=<from> RUX_MATCHES_B=<to> RUX_MATCHES_ICP=1 nix develop -c bash $S --no-build --page rediger-matches --theme dark --project $SCRATCH/newoffice.rux --out shots/qt/newoffice-good 2>&1 | tee /tmp/no-good.log
grep "rediger-matches" /tmp/no-2240.log /tmp/no-good.log
```
`qt_shot.sh` passes the environment through to the gallery. If it does
not (no `rediger-matches:` lines naming those ids), run the gallery
directly. Use the `env -u DISPLAY -u WAYLAND_DISPLAY QT_QPA_PLATFORM=offscreen build/apps/rux/qt/rux-qt-gallery --page rediger-matches --theme dark --project <a fresh cp of the scratch copy> --screenshot <png> --dev`
line from `qt_shot.sh` with the variables set.

Expected:
- the good pair aligns with K inliers, a small RMS and a near-identity
  ΔT ("moved" ≲ a few cm, ≲ 1°), and ICP converges near it;
- 2240/2241 logs its inliers, RMS, ΔT and the ICP line.

Read both PNGs.

- [ ] **Step 3: Record and close**

```bash
cd /home/mephisto/repos/ReUseX && nix develop -c bd update reusex-xlg.2 --notes "NewOffice copy: 2240/2241 -> <N> matches, <K> inliers, RMS <r> cm, B moved <d> cm/<a>°, ICP <line>. Good pair <from>/<to> -> <K'> inliers, RMS <r'> cm, moved <d'> cm (≈ identity)."
cd /home/mephisto/repos/ReUseX && nix develop -c bd close reusex-xlg.2 --reason "Match editor shipped: find/exclude/add/region, Placér B fra match -> ICP; real-data numbers in notes"
```
If 2240/2241 gives no consensus, do not close. Record the numbers and
report them. The spec's success criterion is that the editor lets a user
fix that pair, and an honest "the automatic matches fail here, manual
matches needed" is the finding to hand back.

- [ ] **Step 4: File the follow-up and hand off**

`LoopClosure.cpp` keeps its private copy of the ORB extraction that
`pano_detail::extract_frame_features` now generalises. File it:
```bash
cd /home/mephisto/repos/ReUseX && nix develop -c bd create "slam: LoopClosure's extract_features onto pano_detail::extract_frame_features" -t task -p 3 --parent reusex-xlg -d "LoopClosure.cpp keeps its own ORB+depth lifting; frame_pair_matches (reusex-xlg.2) uses the panorama front-end's. Fold LoopClosure onto it with test_loop_closure as the guard."
nix develop -c git status
```
Report to the caller:
- the commits;
- the NewOffice numbers;
- that `reusex-xlg.1` waits on the user's manual confirmation (Task 1
  Step 11), if it does;
- the screenshots under `shots/qt/`.

---

## Self-review notes (resolved while writing)

- **The spec names `FrameMatcher`, but no such type exists.** It appears
  only as a comment in `LoopClosure.hpp`. The real reusable front-end is
  `pano_detail::extract_frame_features`, which returns keypoints, an
  optical 3D point per keypoint and validity. LoopClosure's own
  `extract_features` drops the keypoints, so it cannot serve the editor.
  The plan reuses the panorama function and generalises it (`const`,
  optional region). The Lowe ratio test is copied from LoopClosure's
  matching loop, and the fit is `detail::ransac_rigid`. Deduplicating
  LoopClosure is filed as a follow-up (Task 9).
- **Reading "stored-pose frame".** An `xyz` value is
  `local_transform · optical`, so `T · xyz` matches `frame_world_points`.
  A test pins this with a non-identity `local_transform`.
- **Exact `world_delta` conversion.** It is stated in Global Constraints:
  `offset_from_matrix(world_delta, pivot)`, the same call ICP uses, with no
  pivot correction. Two tests pin it: the ICP round-trip and the
  `edge_measurement` test.
- **Additions to the spec's struct.** The plan adds
  `FrameMatchOptions::{min,max}_depth_m`, `RigidFit::failure` (so the Qt
  side writes Danish text instead of showing the library's English
  `reason`), `kMinFitSpreadM` and the collinearity refusal. It also adds
  `lift_pixel`'s depth range parameters (defaulted).
- **Re-running Find match.** Automatic lines are replaced, manual ones are
  kept, and the fit colours are reset. The spec does not say; this keeps a
  user's hand-added matches across a region change.
- **Manual matches can be started on either side.** The spec says "A then
  B"; the pair is always stored as (A, B).
- **The test fixture uses wedge-plus-box depth instead of flat depth.** A
  flat plane makes the ICP round-trip degenerate. Because A and B share
  their images, the true stored-pose relative pose is the identity. A
  wrong `world_delta` convention still fails both checks: ICP moves away
  from a wrong seed, and the manual edge stops being the identity.
- **Layout.** The right splitter moves to 1:1 (Task 5), and the toolbar
  uses two rows to fit the ~375 px column at 1440. Both are verified by
  the existing guards and need no new guard.
- **Fit status placement.** The fit status
  ("Fra matches: 24 inliers · RMS 2,1 cm") and the read-out share one
  status line under the toolbar. The last event shows, and the read-out
  sits in its tooltip.
- **Task 1's fix is conditional on the measured cause, by design**
  (systematic debugging). Each candidate's fix is written out, and an
  unmatched observation stops the task instead of guessing.
