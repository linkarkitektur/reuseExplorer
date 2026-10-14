---
name: design-studio
description: The design workflow for the native Qt client of the rux GUI (apps/rux/qt, in progress: QSS generated from tokens.css, screenshot loop via qt_shot.sh) and for the Ressourcekortlægning PDF report (Typst template apps/rux/resources/report.typ, page-screenshot loop via `nix develop .#report` / report-shots). Use this whenever you design, build, restyle, redesign, or "make it look better/more polished" for any Qt client page or workspace (Database/Oversigt, 3D/Viewer, Rediger, Pipeline, Log, the app shell); whenever you touch styles/app.qss or a Qt widget/page; whenever you work on the PDF report, its Typst template, its layout, its sample data or print quality; and whenever you work a design-backlog bead for a Qt page. Also use when matching the existing rux GUI look, or when a token in apps/rux/qt/theme/tokens.css needs refreshing from rux-frontend. The web half of the rux GUI (React/Vite/TS, apps/rux/frontend) moved to the standalone rux-frontend repo — its design-studio skill lives there now, not here.
---

# Design Studio — the Qt client (rux GUI)

Claude Code writes good Qt/C++ UI code. What this skill adds is *process* and the
project's *house rules*: work inside the design system that already exists,
build in the real stack, **look** at the result, and iterate. The single most
important step is §5 — never ship a screen you haven't seen, in both themes.

This repo is **one of two surfaces that share one token-based design system**:
- **The `rux-frontend` repo:** the web frontend — React 19 + Vite + TypeScript +
  three.js, CSS Modules. `src/tokens.css` is **owned there** — the
  `/design-sync` target. Its own `design-studio` skill covers that surface;
  read it in that repo, not here.
- **This repo (`ReUseX`):** the native **Qt 6** client, `apps/rux/qt/`
  (Stream Q). Its QSS and QPalette are generated at run time from a
  **vendored copy** of rux-frontend's tokens — `apps/rux/qt/theme/tokens.css`,
  refreshed with `scripts/sync-tokens.sh <path-or-url>` — so every visual
  decision is still a token. It has its own screenshot loop (`qt_shot.sh`).
  See §Qt loop below and `references/qt-client.md`.

The token *names and roles* are the cross-surface contract; the *values* are
owned by rux-frontend and the Claude Design project "ReUseX GUI". A token
renamed or removed in rux-frontend is a breaking change here — re-run
`scripts/sync-tokens.sh` and fix whatever the Qt token ctests then flag.

Directory map, the full Qt widget inventory, the token-mapping table (CSS →
QSS), and every recorded gotcha live in `references/qt-client.md` — read it
before your first Qt change.

## The loop

Brief → Locate in the system → (Direction, only if greenfield) → Build → Screenshot → Critique & fix → Hand off

### 1. Pin down the brief

If the request is vague, ask up to 3 short questions, and only ones whose
answers change the design:
- Which workspace/page? (e.g. a new panel on the 3D workspace, the Pipeline
  form, a fresh `rux_qt/pages.hpp` entry.)
- Fidelity: throwaway mockup to agree a layout, or production Qt/C++ to merge?
- Does the data already come from `ProjectDB`/`pipeline::JobRunner`, or does
  this need a new reader? The Qt client never talks to `ruxd`'s REST API —
  it runs stages and reads the project in-process.

If the user wants speed or has already said enough, state your assumptions in
one line and proceed. Current design backlog items live in beads, not a
docs/ file — check `bd ready` / `bd list -l workstream:gui` (or the
equivalent in rux-frontend's own tracker for web-side work) before starting
from scratch.

### 2. Locate the work in the existing system (this replaces "invent a design system")

The design system already exists. Your job is to *conform*, not to create one.

- **Read `apps/rux/qt/theme/tokens.css` for the token names and their roles**
  (surfaces, border, text, accent, status, `--label-*`, spacing, type, radius,
  shadow, layout, motion). It is a vendored, read-only copy — see §3.
- **Find the nearest existing widget/page and copy its patterns.** Grep
  `apps/rux/qt/include/rux_qt/widgets.hpp` and `apps/rux/qt/src/` for something
  similar — `Panel`, `StatCard`, `NavRail`, `LabelLegend`, `PropertyList`, a
  table workspace. Reuse before inventing a parallel one.
- **Hard rules that come from the token ownership boundary:**
  - **Never hand-edit `apps/rux/qt/theme/tokens.css`.** It is a vendored
    snapshot of rux-frontend's `src/tokens.css`; a wrong value there is a
    stale sync, not a local fix — run `scripts/sync-tokens.sh` instead.
  - **No literal colour, radius, spacing or type size anywhere in
    `apps/rux/qt/`** — only `var(--…)` in `styles/app.qss`, or
    `theme().color("--…")` / `theme().px("--…")` in C++.
    `scripts/token_lint.py apps/rux/qt --qt` enforces it; run it before you
    finish.
  - **SPDX header on every new file** (`.cpp`, `.hpp`, `.qss`) — copy the
    two/three line header from any neighbour. The REUSE lint job fails
    without it.

### 3. Refresh tokens — only when rux-frontend's tokens changed

Most work here is *additive*: a new panel or page that must look like the
rest of the app, using tokens that already exist locally. Only pull a fresh
`tokens.css` when rux-frontend's design project actually re-synced:

```bash
scripts/sync-tokens.sh ../rux-frontend/src/tokens.css   # local checkout
scripts/sync-tokens.sh https://raw.githubusercontent.com/.../src/tokens.css
```

This copies the file to `apps/rux/qt/theme/tokens.css` and runs the Qt token
ctests, which fail loudly if `app.qss` or a C++ `theme().color(...)` call
names a token that no longer exists. Fix those before building anything new.

### 4. Build

**Production deliverable = the real Qt/C++ stack**, not a mockup:
- Compose from `apps/rux/qt/include/rux_qt/widgets.hpp`; style by
  `objectName` / `kind` / `tone` / `role` dynamic properties in
  `styles/app.qss`; read any value code needs from the Theme
  (`theme().color("--x")`, `theme().px(...)`, `theme().font(...)`).
- **Real copy**, in Danish, specific to the domain ("Ingen punktsky endnu —
  kør rux create clouds") — not "Feature one". See `qt-client.md` §6 for the
  shape of a new page.
- **Quality floor:** focus visible on buttons and fields, body contrast
  ≥ 4.5:1 in *both* themes, tap targets sized for a desktop pointer, no
  clipped/overlapping content, no stock-Fusion chrome leaking through.

**Throwaway mockups** are rarely useful here (QSS has no live-reload outside
the gallery/`--dev` loop below) — iterate directly against `qt_shot.sh`
instead.

### 5. Look at it (mandatory), in both themes

```bash
# Every page x both themes, 1440x900 at 2x, into shots/qt/
bash <skill-dir>/scripts/qt_shot.sh --all
# One page, one theme
bash <skill-dir>/scripts/qt_shot.sh --page components --theme dark
```

Then **open every PNG with your Read tool and actually look.** It **fails on
any missing token** (magenta pixel = unresolved `var(--x)`). `--help` for
`--size`, `--scale`, `--project`, `--gl` (the real VTK widget under Xvfb).
Full timing table, fixture handling and failure modes: `qt-client.md` §2.

**Headless:** `QT_QPA_PLATFORM=offscreen` renders without any display — VTK's
EGL offscreen window stands in for the 3D pane — so this works over SSH or in
a worktree with no monitor. Never skip the screenshot because "there's no
display".

### 6. Critique and fix

Review the screenshots against `references/critique-checklist.md`'s "Qt
client" section. Write down the 3–5 biggest problems, fix them, re-screenshot.
Expect 2–3 rounds; stop when what's left is taste, not defects. Before
finishing:
- Run `python <skill-dir>/scripts/token_lint.py apps/rux/qt --qt` and fix
  every literal it flags — `.qss` literals, unknown `var(--x)` names, and Qt
  C++ literals (`QColor(…)`, `Qt::red`, hex strings, `setPixelSize(12)`,
  `setStyleSheet("…")`).
- Build and run `tests/unit/rux_qt` (light binary): `ctest -R rux_qt` (or the
  specific token/page test you touched).
- Remove one decorative element that isn't earning its place.

### 7. Hand off

In a few lines: what changed and where, which existing widgets you reused,
which tokens the look depends on (so a `sync-tokens.sh` refresh doesn't
silently break it), and known gaps. Leave screenshots in `shots/qt/`. If the
look needs a *new* token name or role, flag it for rux-frontend's design
project — add it there first, then `sync-tokens.sh`, never invent a
Qt-only value.

## Iterating on feedback

Treat comments like inline notes on a canvas: find the exact widget, change
it, re-screenshot, confirm. If a comment implies a *system* change ("this
accent is too loud", "cards need more air"), it is a token concern — do
**not** patch the one instance with a literal. Either the token already
exists (check spelling/role) or it's missing from rux-frontend's system —
flag it there. Keeping the fix at the token layer is what keeps the two
surfaces (web + Qt) in step.

## Correctness constraints (not taste)

Two token groups are correctness, not aesthetics — do not "improve" them away:
- **The 3D viewport canvas stays near-black** (`--color-canvas`) in *both*
  themes. Point clouds are additive light on a dark field; a light canvas
  destroys the depth read.
- **`--label-0..7` is a colourblind-safe categorical scale** (Okabe-Ito).
  Users read semantic segmentation classes off these colours. If a change
  touches them, the replacement must stay distinguishable under
  deuteranopia/protanopia/tritanopia — and must match what rux-frontend
  shows for the same classes.

## Qt loop reference

Directory map, every widget/workspace file, the token-mapping table
(CSS → QSS units), the full gotcha list (mnemonics, locale, VTK offscreen,
QSS specificity, …), and how to build a new page end to end:
`references/qt-client.md`.

## PDF report (Typst)

The Ressourcekortlægning report is an official document (it is attached to
a demolition-permit application), so print quality is correctness, not taste.
`apps/rux/resources/report.typ` is the only copy of the template; CMake
embeds it, so no C++ build is needed to see a change.

The loop, in `nix develop .#report`:

1. `report-shots [DATA_DIR]` — compile and render every page to
   `shots/report/page-NN.png` (default data: the committed
   `apps/rux/resources/report-sample/`).
2. **Read every PNG** — all pages, not the first one.
3. Critique against the checklist in `references/pdf-report.md`.
4. Fix `report.typ`, repeat. Keep `report-watch` running in another pane
   when the user wants to watch.

Real data: `rux -p x.rux export pdf --dump-data shots/report-data`, then
`report-shots shots/report-data`. Never commit a real project's dump.
Data contract: `docs/guides/report-template.md`. Read
`references/pdf-report.md` before the first change.
