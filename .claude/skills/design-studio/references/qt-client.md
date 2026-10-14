# The native Qt client — design loop reference

The second surface of the one design system. Status: **in progress** (Stream
Q of the ruxd-multiuser design doc, removed 2026-10-14 — see git history and
`docs/DIRECTION.md`'s 2026-10-08 changelog entries).
Q0 — this loop, the theme loader, the gallery and two demo pages — Q1 —
the app shell plain `rux` opens — Q2 — the Database workspace — and Q3 —
3D, Rediger, Pipeline and Log — are done. P4 renamed Posegraf to Rediger
(now the only place pose-graph edits happen; Database/Oversigt is read-only)
and gave it its own edge list.

## 1. Where things live

```
apps/rux/qt/
├── CMakeLists.txt            three targets (below)
├── include/rux_qt/
│   ├── tokens.hpp            Qt-free: parse tokens.css, resolve var(--x)
│   ├── gallery_args.hpp      Qt-free: the gallery's command line
│   ├── Theme.hpp             tokens -> QSS + QPalette + fonts, hot reload
│   ├── fonts.hpp             bundled Archivo / Oswald / JetBrains Mono
│   ├── widgets.hpp           CapsLabel, Pill, Panel, StatCard, NavRail,
│   │                         NavItem, LabelLegend, Swatch, PropertyList
│   ├── SceneView.hpp         3D canvas: QVTKOpenGLNativeWidget or EGL
│   │                         offscreen image; LOD swap; hardware picking
│   ├── Viewer3DWorkspace.hpp Q3: layers, colour-by + legend, cut, presets
│   ├── EditWorkspace.hpp     Rediger: PendingBanner over GraphPanel +
│   │                         EdgeListPanel left, the selected pair (images,
│   │                         3D overlay, split from the canvas floors) +
│   │                         PairStrip right
│   ├── GraphPanel.hpp        Rediger graph pane: XY/XZ/YZ, view yaw, ticks,
│   │                         edges coloured by type/status/residual/…
│   ├── EdgeListPanel.hpp     Rediger edge list: sortable/filterable QTableView
│   │                         over rux_qt_core's edge_table()
│   ├── PendingBanner.hpp     pending pose-graph edits banner (count, Gem/Kassér)
│   ├── PairStrip.hpp         A<->B edges between the selected pair: ICP
│   │                         refine, add, delete
│   ├── ViewSettings.hpp      project display settings (view yaw)
│   ├── PipelineWorkspace.hpp Q3: stage form, JobRunner, log tail, CLI line
│   ├── pipeline_ui.hpp       Danish stage/field names, JSON -> rux line
│   ├── cli_command.hpp       Qt-free: "Kopiér som rux-kommando"
│   ├── workspace_logic.hpp   Qt-free: log filter, graph hit tests, log tap
│   ├── graph_view.hpp        Qt-free: Rediger canvas geometry (zoom, view
│   │                         yaw projection, heading tick, A/B node-click rule)
│   ├── edge_colouring.hpp    Qt-free: edge colour modes, legends, percentiles
│   ├── edge_table.hpp        Qt-free: edge list columns, Danish cell text,
│   │                         sort, type/status filter
│   ├── edge_store.hpp        Qt-free: pose-graph store <-> staged edit model
│   ├── pages.hpp             page registry (name -> factory(PageContext))
│   ├── fuzzy.hpp recent.hpp  Qt-free: palette ranking, recent list,
│   │   launch.hpp            what plain `rux` does, open-error text (Danish)
│   ├── ProjectSession.hpp    owns the ProjectDB; opens off the GUI thread
│   ├── RecentProjects.hpp    QSettings (or in-memory for the gallery)
│   ├── AppShell.hpp          title bar, nav rail, stack, Inspector
│   ├── StartPage.hpp         open / drop / recent / summary / error cards
│   ├── CommandPalette.hpp    Ctrl+K overlay
│   ├── workspaces.hpp        page order, names, palette aliases
│   ├── DatabaseWorkspace.hpp Q2/Oversigt: ProjectTree + stack, read-only
│   │                         (pose-graph edits moved to Rediger)
│   ├── FrameBrowser.hpp      A/B sides, ImagePane, filmstrip
│   ├── FrameImageLoader.hpp  off-thread decode (own read-only ProjectDBs)
│   ├── PanoramaViewer.hpp    Oversigt's panorama: off-thread decode to
│   │                         ≤ 4096 px, pan/zoom view (wheel, drag, F)
│   ├── EdgeEditor.hpp        staged pose-graph edits -> one transaction
│   ├── TableBrowser.hpp      any sqlite table (paged), pipeline log
│   ├── selection.hpp         what the inspector shows for a selection
│   ├── database_logic.hpp    Qt-free: A/B pair, keys, pending edits,
│   │   background.hpp        paging, formatters; detached-work counter
│   └── app.hpp               run_app(): QApplication + MainWindow
├── src/  src/core/           implementations (core/ = the Qt-free half)
├── styles/app.qss            THE stylesheet template — var(--token) only
├── resources/rux_qt.qrc      release snapshot: tokens.css, app.qss, fonts
├── resources/fonts/*.ttf     OFL-1.1 (LICENSES/OFL-1.1.txt, REUSE.toml)
└── gallery/                  rux-qt-gallery: main.cpp, demo pages and
                              shell_pages.cpp (the real AppShell)
```

The Database/Oversigt pages (Q2, read-only), best shot with a real project
copy (`--project` <copy of a project with frames>; the default fixture has
few frames): `db-frames`, `db-depth`, `db-confidence`, `db-table`, `db-log`,
`db-cloud`, `db-panorama` (a panorama in the pan/zoom view; on a project
without panoramas — the default fixture — it writes a synthetic 2:1 one to
the copy; `--project <copy of NewOffice>` shows a real 5422 px one). The
gallery waits for `FrameImageLoader::busy()` and
`background_work_in_flight()` to reach 0 (twice) before it grabs.
`RUX_QT_PAGE=oversigt rux -p <copy> --quit-after-ms N` smokes the workspace
in the real binary (`database` is still accepted as an alias).

The Q3 pages: `3d`, `3d-pick` (rooms + legend + a picked point), `3d-plan`
(planes, frustums, plan view), `3d-yaw` (plan view rotated 30°, writes the
yaw to the project copy), `3d-labels`, `log`, `log-filter` — shoot these
with `--project <copy of NewOffice>` — and `pipeline`, `pipeline-run` (runs
`create planes` on the copy; the default office_corridor fixture is right
for it), `pipeline-cancel` (the same run cancelled mid-stage). `viewport` is now an alias of `3d`. Pages that need off-thread work
done first (a pick, a run) pump the event loop until
`background_work_in_flight()` is 0.
`RUX_QT_PAGE=3d|pipeline|log rux -p <copy> --quit-after-ms N` smokes them in
the real binary (under xvfb for the GL widget).

The Rediger pages (P4; all write their edits to the project **copy**, never
the tracked fixture): `rediger` (graph + A/B + pending loop closures),
`rediger-xz` / `rediger-yz` (elevations), `rediger-yaw` /
`rediger-yaw-zoomed` / `rediger-yaw-close` (the view yaw rotated 30°; zoom
scale and centre survive the rotation; the yaw is written even when the
page closes immediately), `rediger-panes` (edge list and images hidden),
`rediger-pending` (an ICP result plus staged edits), `rediger-readonly`
(save refused on a read-only project, Danish banner — `rediger/-readonly`
replaces the old `db-readonly`), `rediger-saved` (a saved edge — replaces
`db-saved`), `rediger-stress` (10 000 edges; measures paint time),
`rediger-click` (an edge click fills A/B on a seeded graph), `rediger-optimize`
(Optimer run from the toolbar, edges by residual), `rediger-discover` (Find
kanter run from the toolbar, with the discover count shown), and, all on a
seeded graph: `rediger-type`, `rediger-status` (rejected edges shown),
`rediger-residual`, `rediger-deviation`, `rediger-seed` (the five colour
modes), `rediger-list` (edge list sorted by residual, top row selected),
`rediger-strip` (a pair with four edges in the strip — auto, confirmed with a
staged un-confirm, rejected, pending — every row's actions in view),
`rediger-roundtrip` (an edge from the list, B moved, ICP-forfin, Erstat
måling, Gem, Optimer — each step and the stored row logged; on a real
copy pass `RUX_ROUNDTRIP_A=<id>` for a pair ICP can fit), `rediger-reload` (zoom, a clicked edge, A/B and a yaw turned inside its
debounce all survive a same-project reload — logged), `rediger-zoom` (zoomed to one node, pixel count logged — the
vanishing-node repro), `rediger-matches` (Find match, then Placér B fra
match: inliers green, outliers red, B moved; on a real copy pass
`RUX_MATCHES_A=<id> RUX_MATCHES_B=<id>`, and `RUX_MATCHES_ICP=1` to run
ICP-forfin after it and log the result; it first checks, and logs, that a
Find started before Byt A og B is dropped and that, on a scratch copy with
A's depth / pose / colour NULLed, Find match says "billede N har ingen
dybde" / "… ingen pose" / "… intet farvebillede"), `rediger-matches-edit`
(two lines excluded, one manual match added through the click path, a
region in A), `rediger-matches-readonly` (the tightest right column of
normal use: a read-only project with staged edits and a refused save, Find
match, Placér B and ICP-forfin, the read-only note — the canvas guards must
hold here; pass `RUX_MATCHES_A/B` on a real copy). The images/overlay split
is not tuned by hand: `EditWorkspace` gives each pane its measured chrome
plus its canvas floor (`kMatchCanvasFloorPx` / `kOverlayCanvasFloorPx`,
`workspace_logic.hpp`, the same constants the guards use) and shares the
rest evenly (`balanced_canvas_split`), until the user drags the handle. The
pending banner sits over the left column only, and the strip's ICP line and
note are one elided line each, so neither takes height from the canvases.
`RUX_QT_PAGE=rediger rux -p <copy> --quit-after-ms N`
smokes the workspace in the real binary (`posegraf` is gone, not an alias).
The overlay has no ΔT fields: B is moved by the box widget, the keys, the
match editor or ICP-forfin, and its header reads out the move.

The app shell's gallery pages: `start`, `start-readonly`, `start-empty`,
`start-error`, `shell-empty`, `shell-project`, `palette-open` (query "pro"),
`palette-all`. They build the same `AppShell` the binary shows, open the
project synchronously (`ProjectSession::open_blocking`) and use an in-memory
recent list with two missing entries — a shot never touches QSettings.
The real binary: `rux -p <copy> --quit-after-ms 3000` under `xvfb-run -a env
QT_QPA_PLATFORM=xcb` (set `XDG_CONFIG_HOME` to a scratch dir so the recent
list and window geometry stay out of your settings).

| Target | Links | Why separate |
|---|---|---|
| `rux_qt_core` | std only | tokens.css parsing + gallery args, tested in the LIGHT unit binary (`tests/unit/rux_qt/`) |
| `rux_qt_lib` | Qt6 Widgets/OpenGLWidgets, VTK GUISupportQt, `reusex_core`, `reusex_visualize` (private) | the Qt layer; never the `reusex` umbrella, so no libtorch/TensorRT — the gallery links in seconds |
| `rux-qt-gallery` | `rux_qt_lib` | renders any page to PNG headless |

All three (and `tests/unit/rux_qt`) are gated by `-DBUILD_QT_CLIENT=ON`
(default), for a configure whose VTK lacks Qt support.

AUTOMOC/AUTORCC are set **per target** (they are off globally). Public headers
with `Q_OBJECT` are listed as sources, because AUTOMOC only looks for headers
next to a `.cpp`. Sources are globbed (`src/*.cpp`, `gallery/*.cpp`).

## 2. The loop

```bash
# Every page x both themes, 1440x900 at 2x, into shots/qt/
bash <skill-dir>/scripts/qt_shot.sh --all
# One page, one theme
bash <skill-dir>/scripts/qt_shot.sh --page components --theme dark
# The real interactive VTK widget, under Xvfb
bash <skill-dir>/scripts/qt_shot.sh --page viewport --gl
```

Then **Read every PNG** and critique (checklist §Qt). Measured on this box
(2026-10-08):

| Edit | What happens | Time |
|---|---|---|
| `app.qss` or `tokens.css` | no build (the gallery reads both live with `--dev`); one page x one theme | **~1.3 s** |
| same, all pages x both themes | 4 shots | ~6.7 s |
| a `.cpp` in apps/rux/qt, from inside `nix develop` | incremental build (one TU + link) + shot | **~10 s** |
| same, from outside the dev shell | + `nix develop` start-up | ~22 s |

`qt_shot.sh` only builds when an `apps/rux/qt` C++/CMake/qrc/ttf file is newer
than the binary, so a pure style iteration never pays for the dev shell. For
live tweaking on a display: `build/apps/rux/qt/rux-qt-gallery --dev --page
components` (hot reload is also on by default in a Debug build) —
`QFileSystemWatcher` re-applies the stylesheet on save and the gallery
rebuilds the page, so values read in code refresh too. Editors that replace
the file are re-watched; one that deletes and recreates it gets up to 5 s:
the previous theme stays on screen meanwhile, never an empty magenta one.

**Fixture:** by default `qt_shot.sh` prepares a copy of
`tests/fixtures/scans/office_corridor.rux` once (`rux create clouds -g 0.02
--sampling-factor 2` + `create planes`; the tracked fixture has no cloud) in
`~/.cache/reusex/qt-shot/`, keyed on the fixture's hash, and copies it again
for every shot. It never opens the tracked file. `--project` takes your own
copy. The one-time preparation needs `build/apps/rux/rux`: `qt_shot.sh`
builds it if missing (or, with `--no-build`, fails and prints the command).
Only a successful preparation is cached; a failure prints the tail of
`~/.cache/reusex/qt-shot/prepare.log` and leaves nothing behind.

**Failure is loud:** an unknown token renders **magenta**, is logged as
`MISSING token --x`, makes the gallery exit 3 and `qt_shot.sh` fail. A
bundled font family that did not register (`MISSING font family …`) fails the
same way. In CI, `AppQss_EveryVarReference_ResolvesAgainstRealTokens` and
`QtSources_EveryTokenStringLiteral_ExistsInRealTokens` (light ctest) catch a
design-side rename that `app.qss` or a `theme().color("--…")` still uses. Other
exit codes: 2 bad flags, 4 project/page not found, 5 PNG not written, 6 a
layout regression: the page's minimum width is wider than the window
(`TOO WIDE` — the inspector would be off a 1440 px screen), or, at 1440x900,
Rediger's 3D overlay canvas with A and B loaded is under 250 px tall
(`OVERLAY TOO SMALL`; every shot logs `overlay canvas WxH`), or, at
1440x900, Rediger's match canvas with both A and B images loaded is under
200 px tall (`MATCH CANVAS TOO SMALL`; every shot logs `match canvas WxH`),
or Rediger's
pair strip is clipped (`STRIP CLIPPED`: an edge row's action button runs past
the rows' right edge, the edge list shows less than its first row, or the
strip ends below the window; every shot logs `pair strip bottom Y of H`).
A page can also fail its own check: `rediger-drag-qt` (needs `--gl`) drags
B's box centre handle with Qt mouse events and logs `DRAG NOT TRANSLATED`
when B does not move, the handle is too small, or the page cannot run;
`rediger-matches` / `rediger-matches-edit` log `MATCH EDIT FAILED` when a
stale Find lands on the swapped pair, a missing depth / pose / colour is
not named in Danish, a line toggle during Find is not refused, the manual
match was not added, or (`RUX_MATCHES_ICP=1`) the fit status still reads
"Fra matches …" after ICP-forfin moved B; `rediger-rgb` /
`rediger-tint` log
`RGB NOT SHOWN`. Every one prints `ERROR <WORDS> (TOO …|NOT …|FAILED|
CLIPPED)` and makes the gallery exit 6: `qt_shot.sh` fails on that exit code
and on that shape of line, so a new page check needs no edit there.
`RUX_QT_DUMP_QSS=<file>` writes the resolved stylesheet for debugging.

## 3. Token mapping

`rux::qt::Theme` parses `:root` (light) and overlays `[data-theme='dark']`
(dark); `@media` blocks are ignored. Values are normalised for QSS:

| CSS | Qt |
|---|---|
| `0.75rem` | `12px` (16 px root, rounded to whole px — QSS font sizes are ints) |
| `rgb(29 45 61 / 10%)` | `rgba(29, 45, 61, 10%)` |
| multi-line font stack | one line; QSS `font-family` lists fall back correctly |
| `var(--x)` inside a token | resolved |
| `var(--qt-icon-check)` / `--qt-icon-chevron` | **generated at run time**: glyph PNGs painted in `--color-on-accent` / `--color-text-muted` (no QtSvg in the shell to tint an SVG) |

In C++ never write a value; ask the theme: `theme().color("--color-canvas")`,
`theme().px("--space-4")`, `theme().em("--tracking-caps")`,
`theme().font(FontRole::display, "--font-size-3xl", "--font-weight-bold")`.
Every lookup of an unknown name is recorded as missing, exactly like the QSS.

**QPalette** (for what Fusion draws and QSS does not reach — menus, tooltips,
focus, selection): Window=`surface`, Base/Button/Light=`surface-raised`,
AlternateBase=`surface-sunken`, Text/WindowText/ButtonText=`text`,
PlaceholderText + all Disabled text=`text-faint`, Highlight=`accent-muted`
(+ HighlightedText=`text`, as the web's selected row), Accent=`accent`,
Link=`accent-deep`, Midlight=`border`, Mid/Dark=`border-strong`,
Shadow=`scrim`, ToolTipBase=`surface-overlay`. Style is always Fusion so no
platform theme leaks into a shot.

## 4. What QSS cannot do — and the substitute

| CSS feature | Substitute |
|---|---|
| `text-transform: uppercase`, `letter-spacing` | `CapsLabel` paints with `QFont::AllUppercase` + absolute spacing = `--tracking-*` em x pixel size. Give it a QSS `role` for colour/size. |
| `box-shadow` (`--shadow-*`) | not used: elevation is a 1 px `--color-border` + a raised surface. `QGraphicsDropShadowEffect` exists but is slow and blurs children — avoid. |
| transitions, easing (`--duration-*`) | `QPropertyAnimation` with the token duration (none yet). |
| `calc()`, `color-mix()`, custom properties | compute in code from tokens. |
| per-instance colour (a legend swatch) | `Swatch` paints `theme().color(token)`; a per-widget `setStyleSheet` is forbidden. |
| descendant selectors on a parent's state (`:checked QLabel`) | set a dynamic property on the child (`active`) and call `repolish(child)`. |
| line-height | layout spacing (`--space-*`) instead. |
| `:focus-visible` | `:focus` — QSS has no keyboard-only focus; keep the focus border subtle. |

Also: QSS `font-size` 0 or a missing family silently gives you a default. If a
shot shows microscopic or wrong-face text, dump the QSS (`RUX_QT_DUMP_QSS`).

## 5. Gotchas (recorded the hard way)

- **`&` is a mnemonic** in `QPushButton`/`QAction`/`QCheckBox` text: "Miljø &
  prøver" renders "Miljø _prøver". Pass button labels through
  `NavItem::escape_mnemonic()` (`&` -> `&&`). A plain `QLabel` is safe.
- **Locale**: `QApplication` calls `setlocale(LC_ALL, "")`; under `da_DK`
  `std::stod("0.75")` is 0. Parse numbers with `std::from_chars` (the token
  parser does; a unit test pins it). Found as "every font size is 0" — and
  again in Q2 as wrong intrinsics and a fake ICP result, because the
  LIBRARY parsed with strtod. Now `run_app` and the gallery set
  `LC_NUMERIC` back to "C" right after the QApplication, and library code
  uses `reusex/utils/parse_number.hpp`. Never add a `stod`/`strtod`.
- **QVTKOpenGLNativeWidget is blank under `QT_QPA_PLATFORM=offscreen`** (no GL
  context) and `minimalegl` core-dumps. Screenshot mode therefore renders the
  same vtkRenderer into VTK's EGL offscreen window and shows the image in the
  pane (`SceneView`); `--gl` runs the real widget under `xvfb-run` (whose exit
  code is 1 on success: judge by the PNG). Both modes share the scene and the
  camera, so they frame identically.
- **Static library resources**: `Q_INIT_RESOURCE(rux_qt)` must run (it does, in
  `ensure_bundled_fonts()`), or the linker drops the fonts and the snapshot.
- **Fonts**: bundle static TTFs. fontsource's woff2 registers Archivo 400 as
  the family "Archivo SemiBold", and nix sandboxes have no fontconfig at all.
  `ensure_bundled_fonts()` checks Archivo, Oswald and JetBrains Mono are
  registered and logs an error per missing family.
- **qrc is XML**: no `--` inside a comment.
- **Numbers** are formatted Danish (`format_count` -> `39.723`), mono
  (`JetBrains Mono`), right-aligned — same as the web's `da-DK` figures.
  Mono is for figures and code (`Ctrl+K`, `cloud`) only, never prose.
- **Caps**: eyebrows over a region (PROJEKT, INSPEKTØR, LAG, nav groups) use
  `--tracking-wide`; every other caps label — panel titles, field labels,
  table headers — `--tracking-caps`, 2xs/xs bold. A table header gets its
  tracking on the `QHeaderView` font in code.
- Installed binaries need Qt's platform plugins: `default.nix` keeps
  `dontWrapQtApps = true` and calls `wrapQtApp $out/bin/rux` in postFixup.
- **Linkage**: `rux_qt_lib` goes on the `rux` executable only (main.cpp
  injects a `GuiLauncher` into `rux::run`). Never link it into `rux_lib`: the
  heavy test binary links `rux_lib`. Qt-free logic (palette table, launch
  decision, recent list) lives in `rux_qt_core`, which is always built.
- **Vetting a file**: `ProjectDB::probe(path)` (read-only, silent) says
  whether it is a ReUseX project before a read-write open would give a
  foreign sqlite file ReUseX's tables. Never lower the global log level.
- **A widget taken out of a layout still paints** until its `deleteLater()`
  runs: `hide()` it first when rebuilding a panel, or the old content shows
  through the new (seen as overlapping text in the inspector).
- **QSS specificity**: `QFrame#card QLabel { background: transparent }` beats
  `QLabel#chip { background: … }` (one id each, more type selectors win).
  Scope the specific rule the same way: `QFrame#card QLabel#chip`.
- A `QPushButton` given a layout of labels (a clickable row) does not size
  from it: `setMinimumHeight(layout->sizeHint().height())`.
- **`slots` is a Qt macro** (also `signals`, `emit`): a variable named
  `slots` fails with "expected primary-expression" far from the cause.
- **QSS cannot draw a slider handle taller than its groove** without a
  negative margin, which the token lint rejects: the handle is a pill on a
  6 px track instead.
- **`QHeaderView::section` font rules replace a font set in code**, losing
  `QFont::AllUppercase`: return upper-case header text from the model.
- **Off-thread images**: never touch the session's ProjectDB from a worker;
  open a read-only one per thread. Planes coloured by the theme (depth,
  confidence, labels) are decoded as Indexed8 and get their colour table at
  paint time, so a theme switch needs no re-decode.
- **3D draws through `visualize::populate_scene()`** (the `rux render`
  builder) into a vtkRenderer built on a worker thread, then moves the actors
  to the view's renderer on the GUI thread. Never construct the scene
  inline: render and app must agree.
- **Picking**: use `vtkHardwareSelector` (it renders ids, so it respects
  the cut plane's clipping); `vtkPointPicker` is geometric and returns the
  ceiling a plan view has clipped away.
- **Frame on what is visible**: `place_preset_camera()` fits the bounds it
  is given; pass bounds clamped to the cut, and re-frame on resize until the
  user moves the camera (a page laid out after the first frame is otherwise
  framed for the wrong aspect).
- **QListWidget + setItemWidget clips** a row wider than its hint: rows of
  checkable QPushButtons (with `setMinimumHeight` from the layout) are
  simpler and style cleanly.
- **Spin boxes**: `setButtonSymbols(NoButtons)` — Fusion's arrows under our
  QSS render as two dots. Set a Danish `QLocale` on them.
- **Floats in a command line**: `format_number(float)` — widening a float
  to double first prints `0.07000000029802322`.
- **Only the view on screen drives the inspector**: a frame finishing its
  decode behind a table must not replace the selected row.
- **A Fixed-height widget in a QHBoxLayout caps the row**: the row's
  height is limited by the smallest maximum of its unaligned items, so a
  small chip (`Ryd område`) cut the buttons beside it in half. Add it with
  `Qt::AlignVCenter`.
- **QStackedLayout raises the page it switches to**: a widget floating
  over a `SceneView` (the overlay's Tint | RGB, its legend) must be raised
  again after `set_message()`.
- A path in a narrow column: `ElidedLabel` (Qt::ElideMiddle keeps the file
  name); plain QLabel text never elides and widens its parent.

## 6. Building a page

1. A factory `QWidget *make_x(const PageContext &)`, registered with
   `register_page({"x", "one line", make_x})` — explicit, not a static
   registrar (the static lib would drop it).
2. Compose the shared widgets; give anything QSS must style an `objectName`
   or a `kind`/`tone`/`role` property; put its rules in `styles/app.qss`.
3. Real data from `ctx.db` (a project copy) and a designed empty state when it
   is `nullptr` or the data is missing ("Ingen punktsky endnu — kør rux create
   clouds"). Danish copy.
4. `python <skill-dir>/scripts/token_lint.py apps/rux/qt --qt` must pass.
5. `qt_shot.sh --page x` in both themes; Read the PNGs; iterate.
