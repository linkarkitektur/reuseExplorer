<!--
SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
SPDX-License-Identifier: GPL-3.0-or-later
-->

# ReUseX — Direction

<!-- Source of truth for project heading. Agents: read this before planning
     or reviewing work. Update via PR; date every change in the changelog. -->

This document owns **intent**: where the project is heading and why. GitHub owns
**state**: what is in flight, what is done. Nothing here tracks per-issue status —
each workstream points at one anchor issue and says where it should end up.

Companion documents, referenced rather than duplicated here:

- [`docs/STANDARDS.md`](STANDARDS.md) — the objective bar every change must meet
  (module boundaries, label contract, determinism, testing, Definition of Done)
- [`docs/CONTRACTS.md`](CONTRACTS.md) — what each pipeline stage consumes and
  produces in a `.rux` project database
- [`CLAUDE.md`](../CLAUDE.md) — how to work in this repo (build, conventions, style)

---

## Mission (stable)

ReUseX turns 3D scans of existing building interiors into **semantic models that
support reuse and renovation decisions**. A capture goes in — RTABMap/ARKit
sensor frames, MuSHRoom or ARKitScenes benchmark data, E57/PLY clouds, 360°
panoramas — and what comes out is a labelled, room-partitioned geometric model
whose components (walls, windows, doors) carry structured **material passports**
in the spirit of the EU Digital Product Passport, exportable into the tools
architects and engineers actually use (Rhino/OpenNURBS, Speckle, CSV,
MaterialEPAS).

The value is not the point cloud and not the mesh. It is the answer to *"what is
in this building, in what quantity, in what condition, and can it be reused?"* —
derived from a scan rather than from drawings that no longer match reality.

Everything else in this document is in service of making that answer **accurate
enough to act on** and **cheap enough to produce** for a real building.

## Principles & constraints

- **GPL-3.0-or-later.** Dependencies and vendored code must be licence-compatible.
  Non-commercial or research-only weights and models are disqualifying, not
  negotiable — see the deferred-matcher note under Non-goals.
- **Nix-flake reproducibility is non-negotiable; Linux-first.** If it does not
  build from the flake, it is not done. Other platforms are not a goal.
- **Solo maintainer plus an agent workforce.** Prefer boring, maintainable,
  well-documented choices over clever ones. Code an agent can safely modify six
  months from now beats code that is 10% faster today.
- **Library-first.** Logic lives in `libs/reusex`; `rux`, `ruxd` (own repo,
  via the installed CMake package), the Python bindings, the Blender add-on
  and the web frontend (own repo) are thin shells over the same entry points.
  Anything that can only be done through the CLI is a design bug. (STANDARDS §1)
- **Every pipeline stage independently re-runnable against `ProjectDB`.** Re-annotate
  without re-importing; re-mesh without re-segmenting. `.rux` is the only
  handoff between stages. (CONTRACTS.md)
- **Measurable before improvable.** Quality claims need a metric and a fixture.
  `rux analyze quality|accuracy` and the benchmark harness exist so that
  "better" is a number, not an impression. (STANDARDS §8)
- **Deterministic by default.** Seeded, bounded, reproducible runs; no unbounded
  iteration counts. (STANDARDS §6)
- **Fail loudly.** A stage that produces empty or degenerate output must say so,
  with the reason and the numbers. (STANDARDS §5)

## Current priorities (ordered, dated)

1. **SLAM / reconstruction quality** — anchors #221, #225. Everything downstream
   inherits pose error: drifting poses smear walls, fragment plane segmentation
   and starve the solid-model solve. Target: **~10 mm plane-flatness RMS** on the
   reference office scan. As of 2026-09 the measured baseline sits around
   20 mm RMS, and joint pairwise registration alone saturates near 18 mm because
   pairwise point-to-plane cannot express global consistency — hence the owned
   plane-landmark back-end in #225. This is priority one because no amount of
   work on the stages above it can recover geometry the poses have destroyed.
2. **GUI application** — anchor #265. The pipeline is currently reachable only by
   a developer at a terminal plus a keyboard-driven PCL window. Without a
   discoverable interface the project cannot be used or evaluated by the
   architects it is for, which caps both adoption and the quality of feedback.
3. **Gaussian splatting** — anchor #240. Native C++/CUDA 3DGS seeded from the
   point cloud and trained on the posed frames and aligned panoramas. Two
   payoffs: a genuinely presentable visual record of a scanned building, and a
   photometric consistency signal that is complementary to the geometric one.
4. **Infrastructure and test health** — anchors #268, #262, plus CI (#202).
   Slow coverage runs and parallel-ctest flakiness tax every other workstream and
   directly degrade the agent development loop. This sits fourth by *ordering*,
   not by importance — it is the cost of everything above it.

Last reviewed: 2026-09-08

## Active workstreams

The anchor is where discussion belongs; sub-issues link back to it. Each carries
a `workstream: *` label so alignment can be queried with
`gh issue list --label "workstream: <name>"`.

| Workstream | Label | Anchor | Intent (one line) |
|---|---|---|---|
| SLAM / reconstruction quality | `workstream: slam-quality` | #221, #225 | Reach ~10 mm plane flatness via an owned global pose-optimization stage (plane landmarks + GNC), measured against fixture scans — not by tuning RTABMap forever |
| GUI application | `workstream: gui` | #265 | `ruxd` serving a designed web frontend — `ruxd --local` for one person, and a multi-user server (logins, roles per case, Postgres) over the same API contract (`/api/v1/cases/{cid}`); a native Qt client (plain `rux`) alongside; visual language authored in Claude Design, behaviour owned by the repo |
| Gaussian splatting | `workstream: gsplat` | #240 | Native C++/CUDA 3DGS trained from `ProjectDB` — point-cloud-seeded Gaussians, sensor frames and sliced 360 panoramas as views, Apache-licensed gsplat kernels under a GPL trainer |
| 360 integration | `workstream: 360-integration` | #236 | Turn panoramas into wide-baseline pose-graph constraints; a panorama that sees many temporally-distant frames supplies exactly the loop closures the plane-landmark back-end cannot. Mechanism landed and tested (`rux graph discover --use-panoramas`); **blocked on matcher quality** — cross-camera ORB resections are too weak to help, see [`research/panorama-loop-closure.md`](research/panorama-loop-closure.md) |
| Agent-driven modeling | `workstream: agent-modeling` | #267 | Evaluate an agent + MCP gateway + Blender path to a simplified, tagged building model as a complement to the geometric pipeline; requires headless rendering so the agent has eyes |
| Infrastructure & test health | `workstream: infra` | #268 | Keep the build, test and coverage loop fast and trustworthy so agents can iterate — includes #262 (parallel-ctest flakiness) and hosted CI (#202) |

Work that fits none of these rows is either a bugfix, a standing maintenance
obligation (STANDARDS/CONTRACTS drift, licence compliance), or a **new
direction** — and a new direction requires an edit to this document, not just an
issue.

## Non-goals / deferred

- **Non-Linux platforms.** Nix-flake-first with a CUDA-heavy dependency set; the
  cost of Windows/macOS parity buys nothing for the target user today.
  *Would change if:* a collaborator or deployment target makes it a hard
  requirement.
- **Non-commercially-licensed learned matchers (e.g. MASt3R).** Attractive on
  paper for wide-baseline loop closure, but CC-BY-NC weights are incompatible
  with this project's licence and intended use. Apache/MIT alternatives are the
  only candidates, and RGB-D input already makes the metric-scale argument for
  MASt3R moot. *Would change if:* a permissively-licensed model of comparable
  quality lands, or the matcher is fine-tuned from a permissive base.
- **Training foundation models from scratch.** Fine-tuning a permissive base is
  in scope; pretraining is not — no data, no compute, no reason.
  *Would change if:* nothing plausible.
- **HiGHS GPU (PDLP) acceleration.** Disabled at the overlay level on purpose:
  the solid-model solve is a MIP with binary variables and PDLP handles only
  continuous LP, so it could never be used. GPU acceleration for that solve goes
  through cuOpt instead. *Would change if:* HiGHS ships a GPU MIP path.
---

## Review-loop protocol

The workflow this document exists to support: a **supervising agent** reviews the
repository on a cadence, proposes work and files issues; **executor agents**
implement them and report back; the **maintainer** steers by editing this file.

### Supervisor / review agent

Runs as a scheduled routine or a manual session. Each pass:

1. **Read this document first** — Mission, Current priorities, Active workstreams.
2. **Review reality**: recent commits, open and recently-closed issues, PR queue,
   CI status, and any benchmark or quality regressions.
3. **File issues linked to a workstream.** Every filed issue names the workstream
   it serves, carries the matching `workstream: *` label, and references its
   anchor. An issue that serves no existing workstream must say so explicitly:
   *"new direction — requires DIRECTION.md update"*.
4. **Flag misalignment**: in-flight work serving no workstream, workstreams with
   no activity, or priorities contradicted by where effort is actually going.
   Flagging is reporting — not unilateral re-prioritisation.
5. **Propose direction edits as a PR, never silently.** When reality has drifted
   from this document — a workstream finished, an anchor closed, a priority
   overtaken — open a PR editing the workstream table and adding a dated
   changelog entry. Mission and Current priorities are proposed with reasoning
   and left for the maintainer to accept.

### Executor agents

- Read this document before starting; understand which workstream the task serves
  and why it matters.
- Report progress and outcomes **as comments on the issue**, not only in the
  session transcript.
- **Never edit Mission or Current priorities.** If the work reveals that a
  priority is wrong, say so in the issue comment and let the maintainer decide.
- If the work changes the project's direction, update this document **in the same
  PR**, with a dated changelog entry.

### Maintainer

Edits this document directly, whenever intentions change. This file is the
steering wheel; the workstream table, the labels, the issue backlog and the
agents' plans all follow from it. A direction change is a diff, dated in the
changelog — that history is the point of keeping it in the repo.

### Ownership rule

| Section | Who may change it |
|---|---|
| Mission | Maintainer only (or an agent explicitly asked to) |
| Principles & constraints | Maintainer only |
| Current priorities | Maintainer only |
| Active workstreams | Agents may propose via PR; maintainer accepts |
| Non-goals / deferred | Agents may propose via PR; maintainer accepts |
| Direction changelog | Appended by whoever makes the change, always dated |

---

## Direction changelog

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

- **2026-10-12** — **Deployment: Supabase auth, Docker/GHCR fleet, CPU only.**
  `ruxd`'s own username/password accounts (argon2id, `ruxd admin create-user`
  / `set-password` / `disable-user`, `POST /auth/login`) are gone. Accounts
  now live in a self-hosted Supabase (GoTrue); the browser signs in with
  supabase-js and bridges to ruxd's existing HttpOnly session cookie via
  `POST /api/v1/auth/session`. ruxd verifies the Supabase JWT (ES256 via
  JWKS; HS256 still accepted) and holds no password or minting secret. Roles
  stay owner/editor/viewer per case; admin comes from `app_metadata.role`,
  granted with `ruxd admin grant-admin <email>` / revoked with
  `revoke-admin`. `ruxd --local` remains the no-auth single-user mode,
  unchanged. None of this lives in this repo — see `ruxd`'s own CLAUDE.md
  and API docs.
  - **New org, four repos.** `ReUse-X` on GitHub now holds this repo
    (library + `rux` CLI + Qt client, installs a CMake package), `ruxd`
    (HTTP server), `rux-frontend` (the React web GUI) and `reusex-fleet`
    (deployment). **Deployment questions belong in `reusex-fleet`, not
    here.**
  - **Docker/GHCR, nix-free for operators.** `ruxd` and `rux-frontend` are
    each built and pushed to `ghcr.io/reuse-x/{ruxd,rux-frontend}` by their
    own repo's CI (`ruxd`'s image is still a nix `dockerTools` build; the
    frontend's is a plain Dockerfile served by Caddy on :8080). A colleague
    deploys with `docker login ghcr.io`, `reusex-fleet/docker/scripts/gen-supabase-keys.sh`,
    then `docker compose pull && up -d` — no nix required on the target
    machine. Runbook: `reusex-fleet/docker/README.md`.
  - **CPU only, for now.** The deployed stack runs CPU inference only. A GPU
    worker design — pull workers on a Tailscale-reachable host polling
    Postgres with `SKIP LOCKED` — is written up in `ruxd`'s
    `docs/design/gpu-worker.md`, not implemented.
  - **Speckle dropped from the fleet, not from this repo.**
    `reusex-fleet`'s self-hosted Speckle deployment is removed (unused,
    Penpot and the base infra stay). `rux export speckle` and
    `io/speckle.hpp` here are unaffected — they push to any Speckle
    server/cloud instance, not to one this project hosts.
- **2026-10-09** — **Repo split: `ruxd` and the web frontend move out**
  ([design](superpowers/specs/2026-10-09-repo-split-design.md)). `ruxd` (the
  HTTP service worker) and the React/Vite web GUI (`apps/rux/frontend`) now
  live in their own repos under the `ReUse-X` org — `ruxd` and
  `rux-frontend` — with history preserved via `git filter-repo`. This repo
  keeps the library, the `rux` CLI and its native Qt client.
  - **This repo installs a CMake package.** `cmake/Installation.cmake` +
    `cmake/ReUseXConfig.cmake.in` add `install(EXPORT ReUseXTargets NAMESPACE
    ReUseX::)`; headers land under `include/reusex/`, static libraries under
    `lib/`, the config under `lib/cmake/ReUseX/`. `find_package(ReUseX CONFIG
    REQUIRED)` is now the only supported way to consume the library from
    outside — `ruxd` builds against it. An in-tree consumer test
    (`tests/package/`, `ctest -L package`) proves the export doesn't rot.
  - **Removed from this repo:** `apps/ruxd`, `apps/rux/frontend`,
    `tests/unit/ruxd{,_api,_cli,_pg}`, `docs/gui/`, the Crow overlay/patches,
    `pkgs/reusex-gui-frontend`, `modules/ruxd.nix`,
    `completions/ruxd.fish`, `devshells/services.nix`, and the
    ruxd-container/nixosModules.ruxd flake outputs.
  - **The Qt client vendors tokens.css.** `apps/rux/qt/theme/tokens.css` is a
    checked-in copy of rux-frontend's `src/tokens.css`, refreshed with
    `scripts/sync-tokens.sh <path-or-url>` (which also re-runs the Qt token
    ctests). The `design-studio` skill here now covers the Qt half only; the
    web half moved to rux-frontend's copy of the skill.
  - **ruxd provisions SAM3 and runs stages via this repo's exported headers**
    (`EngineBuilder`, `pipeline::JobRunner`, `optimize_stage.hpp`), so none of
    that logic duplicates across repos.
- **2026-10-08** — **GUI: Qt client and ruxd server merged; one optimize
  stage** (Stream Q merged onto Stream S). Both GUIs now run `optimize`
  through the same code: `pipeline/optimize_stage.hpp`
  (`stage_executor_with_optimize()`, header-only so `reusex_pipeline` stays
  free of GTSAM) on top of `slam/optimize_parameters.hpp`, the reader `rux
  optimize` uses too. Before, ruxd's web stage kept its own reader and let a
  wrong-typed parameter escape as an exception; it is now an invalid input.
  ruxd's `/posegraph/icp` is a thin adapter over `slam::refine_frame_pair_icp`,
  like the Qt client's pair strip.
- **2026-10-08** — **Native Qt client, phase Q3: 3D, Posegraf, Pipeline, Log**
  ([spec](superpowers/specs/2026-10-08-ruxd-multiuser-and-qt-client-design.md),
  Stream Q). The client's remaining workspaces. **3D** draws with
  `visualize::populate_scene()`, extracted from `render_view()` so `rux
  render` and the app share one scene builder (render output unchanged
  except label colours): layers, "Farv efter" any label cloud with a
  `--label-*` legend, frustums, panoramas, a cut plane, presets framed on
  what is below the cut, point picking, and a coarse LOD twin (bit-reversed
  Morton prefix, else a stride) while the camera moves. Label layers in
  `rux render` now use the design tokens' Okabe-Ito scale with the web's
  `(label - 1) % size` rule, so render, web and Qt legends agree; `--layers`
  gains `frustums` and `panoramas`. **Posegraf** draws the frames and edges
  in 2D and drives the Database's A/B selection. **Pipeline** runs any
  stage in-process on `pipeline::JobRunner` from a form generated from
  `stage_parameters()`, with progress, cancel and the log tail. **Log**
  filters the pipeline log. Every run carries **"Kopiér som
  rux-kommando"**: the exact CLI line, round-trip tested through the real
  CLI11 setup.
  Known divergence: `export semantic-images`, `export rhino` and the
  on-screen-display overlays still colour labels with PCL's Glasbey LUT, so
  their colours no longer match `rux render`, the web viewport or the Qt
  client. Out-of-contract labels (a −1 that wrapped to 0xFFFFFFFF in a point
  cloud, STANDARDS §3.1) now draw in a non-class `--label-invalid` colour
  (new token, value pending a design ruling) instead of borrowing slot 6.
  `rux create mesh` and `rux optimize` now read their stage parameters
  through the same functions as the in-process stages
  (`pipeline::mesh_options_from_parameters`, `apply_optimize_parameters`),
  and log them under the same keys.
- **2026-10-08** — **Native Qt client, phase Q2: the Database workspace**
  ([spec](superpowers/specs/2026-10-08-ruxd-multiuser-and-qt-client-design.md),
  Stream Q). RTABMap's DatabaseViewer, made calm: a project tree with
  counts; an A/B frame browser (colour, depth or confidence under the label
  overlay and legend; pose, camera and time; ←/→ and Shift+←/→; filmstrip)
  decoding off the GUI thread; a pair strip that shows the pose-graph edges
  between A and B and edits them — ICP refine, add, delete — as pending
  edits saved in one transaction with "Gem ændringer"; a read-only viewer
  for every sqlite table; the pipeline log; and an inspector for whatever is
  selected. Two library additions came with it: `ProjectDB::list_tables /
  table_columns / table_rows` (read-only, blob sizes without the bytes) and
  `slam::refine_frame_pair_icp`, the ICP that was app code behind the web
  GUI's `/posegraph/icp` and is now shared by both clients. A WAL-mode
  project in a read-only directory now probes and opens read-only.
- **2026-10-08** — **Native Qt client, phase Q1: plain `rux` is an app**
  ([spec](superpowers/specs/2026-10-08-ruxd-multiuser-and-qt-client-design.md),
  Stream Q). `rux` with no subcommand (or `rux -p x.rux`) now opens the Qt
  client when there is a display, and prints help (exit 0) when there is
  none; every subcommand is unchanged. The shell has a title bar with schema
  and access status, a nav rail (Start, Database, 3D, Posegraf, Pipeline,
  Log), a collapsible inspector and a Ctrl+K command palette (fuzzy,
  Danish-folding, keyboard-first). The start page opens projects by dialog,
  recent list or drag-and-drop and shows a summary card; a locked,
  read-only, missing or foreign file gets a Danish explanation instead of a
  crash. Projects open off the GUI thread in a `ProjectSession` that owns
  the ProjectDB — the hook Q2's Database workspace builds on. Workspaces are
  designed placeholders until Q2/Q3. The installed `rux` is wrapped for Qt's
  platform plugins.
- **2026-10-08** — **Native Qt client, phase Q0: the design loop**
  ([spec](superpowers/specs/2026-10-08-ruxd-multiuser-and-qt-client-design.md),
  Stream Q). Before any Qt workspace is designed, the design-studio skill can
  see Qt: `apps/rux/qt/` adds `rux_qt_core` (Qt-free token parsing, light
  tests), `rux_qt_lib` (theme loader: `tokens.css` -> QSS template + QPalette
  at run time, hot reload with `--dev`, bundled OFL fonts) and
  `rux-qt-gallery`, which renders a page headless to PNG (3D through VTK's
  EGL offscreen window). `qt_shot.sh` fails on any missing token; a QSS edit
  re-shoots in ~1.3 s, a C++ edit in ~10 s. One design system, two renderers:
  the web GUI's tokens are now also the Qt client's. Plain-`rux` launch and the
  workspaces follow in Q1–Q3.
- **2026-10-08** — **GUI: ruxd becomes a multi-user server** ([spec](superpowers/specs/2026-10-08-ruxd-multiuser-and-qt-client-design.md),
  Stream S, phase S3). `ruxd` without `--local` now serves the same frontend
  and API to many people:
  - **Postgres** holds users, sessions, API tokens, cases, case members, jobs
    and an audit log (`apps/ruxd/migrations/NNN_*.sql`, embedded, applied at
    start in `schema_migrations` under an advisory lock). Case **files** stay
    in `--data-dir`; the `cases` table points at them. `ruxd --local` keeps its
    in-memory stores and `.ruxd/cases.json`.
  - **Logins**: argon2id passwords (OpenSSL 3 `EVP_KDF`), cookie sessions
    stored only as SHA-256 (HttpOnly, SameSite=Strict, Secure unless on
    loopback; 12 h sliding, 14 days absolute), logout revokes, and failed
    logins back off (see the fix round below). Scripts use `rxt_…` API tokens (optionally one
    case); `--auth-token` is a superuser token. `ruxd admin` creates the
    first administrator and manages users; passwords never on argv.
  - **Roles per case**: viewer (GET only), editor (all but deleting the case
    and managing members), owner (all); administrators everything. A case you
    are not a member of is a 404. Mutations are audited.
  - The S2 blocker is closed: the access decision runs in a **patched Crow
    header phase**, before a request body is read.
  - Frontend: a login page, a user menu, and a case **Indstillinger →
    Medlemmer** panel. Local mode never shows the login page.
  - The old service stub (`/`, `/livez`, `/health`, `/segment/planes`,
    `/openapi.json`, BearerAuthMiddleware) is gone; `/api/v1/readyz` checks
    Postgres.
  - Security review fix round (same day): an `Upgrade:` header on a
    request Crow does not upgrade (HTTP/1.0, h2c) skipped the middleware
    entirely — closed, with raw-socket regression tests. Also: events
    sockets close when access ends; `model_path` is refused in server mode;
    login back-off per account and per address (IPv6 /64) with
    `--trusted-proxy`; API tokens expire and can be listed and revoked;
    atomic last-owner check; `__Host-` cookie when Secure; a 32-character
    minimum superuser token and secret files; audit retention; viewers no
    longer see the main edit controls, and an ended session returns to the
    login page.
  - Final-review fix round (same day): case files are untrusted input —
    every ProjectDB connection ruxd opens is hardened
    (`ProjectDB::OpenOptions::hardened`: `SQLITE_DBCONFIG_DEFENSIVE`,
    `trusted_schema=OFF`, triggers and views disabled), and uploads and
    registered files must pass `PRAGMA quick_check` and hold no trigger or
    view (422 otherwise); the `rux` CLI is unchanged. Also: `RUXD_BIND` and
    the OCI image's `RUXD_BIND=0.0.0.0` / `RUXD_DATA_DIR=/data`; 400 for a
    duplicated Host header; control characters refused in asset paths;
    viewers see no write control anywhere.
  - Still open: no web UI for user administration beyond the `/users` API;
    Redis and S3 are reserved — the flags and NixOS options are accepted and
    marked unused, the client code is kept for S3 snapshots; the
    `[postgres]` tests do not run in CI (the nix check sandbox has no
    PostgreSQL — `ctest -L postgres` locally); `quick_check` runs on the
    upload request's worker thread. Deferred as before: S3 snapshots and
    multi-instance locks. Issue drafts 37 (multi-case list) and 38 (LAN
    pairing) are retargeted to what is left after S2/S3.

- **2026-10-08** — **GUI: one ruxd serves many cases** ([spec](superpowers/specs/2026-10-08-ruxd-multiuser-and-qt-client-design.md),
  Stream S, phase S2). A ruxd process is no longer bound to one project:
  - Every project route lives under **`/api/v1/cases/{cid}/…`**, the events
    WebSocket included; `/api/v1/cases` (list, create, rename/archive,
    delete-to-trash) and chunked `/api/v1/uploads` are server-level, as are
    `/health`, `/endpoints` and the SAM3 model status.
  - `ruxd --local <dir>` serves every `.rux` in the directory as a case (ids
    are slugs of the file names) and stores created and uploaded cases there,
    one directory each (`--data-dir`). Cases open lazily and close when idle,
    at most `--max-open-cases` at once.
  - One process-wide job scheduler (`--job-workers`, default 1 because the GPU
    is shared; at most one running job per case, round robin between cases).
    Progress is now **per job** (`core::ScopedProgressObserver`), so two
    cases' stages can run at once without mixing their progress. Job records
    go through a store interface — in memory in local mode, Postgres in S3.
  - The frontend's case screens move under `/sager/:cid/…` and `/sager` is a
    real case list with create and upload; old unprefixed links forward to the
    last-used case.
  - Crow buffers request bodies in memory with no limit of its own, so it is
    patched (`overlays/crow.nix`) to refuse a body over 72 MiB at its headers;
    uploads go up in chunks under that cap.
  - Next: S3 (users, sessions, roles, Postgres-backed cases and jobs). **S3
    blocker:** the body cap is checked before routing but the access token
    only after the headers — fine for one trusted user, not for anonymous
    traffic; S3 should also open uploaded files with
    `SQLITE_DBCONFIG_DEFENSIVE` and `PRAGMA quick_check` them. Still deferred:
    S3 snapshots of case files and multi-instance advisory locks.

- **2026-10-08** — **GUI: the web backend moves into `ruxd`; `rux gui` is
  removed** ([spec](superpowers/specs/2026-10-08-ruxd-multiuser-and-qt-client-design.md),
  Stream S, phase S1). The direction changes from "`rux gui` locally, `ruxd`
  headless" to "`ruxd` serves the web GUI, single-user now and multi-user
  next; plain `rux` becomes a native Qt client (Stream Q)":
  - The GUI's REST + WebSocket API moved, with history, to
    `apps/ruxd/{src,include}/api/` as `ruxd_api_lib` (light: core + pipeline
    only; tests in `tests/unit/ruxd_api/`). The heavy pieces it needs (SAM3,
    renderer, ICP, `optimize`) are injected by ruxd.
  - **`ruxd --local <file.rux | dir>`** replaces `rux -p x.rux gui`: no
    Postgres, Redis or S3, `127.0.0.1:8420` by default, and a bind beyond
    loopback requires `--auth-token` (Bearer, a per-port cookie, or `?token=`,
    which redirects the token out of the URL); a Host allowlist guards the
    token-less loopback default against DNS rebinding. Routes are unchanged
    and single-project.
  - The Morton tile index the GUI streams clouds by is now built by the
    clouds stage itself, so a GUI-submitted clouds job gets one too, and
    `rux` does not link the web API.
  - ruxd's legacy single-project routes (`/materials`, `/material-columns`,
    `/export-templates`, `/exports/csv`, `/reports/…`) shared one `ProjectDB`
    across threads; they are deleted in favour of the moved API, which covers
    all of them.
  - Next: S2 re-roots every route under `/api/v1/cases/{cid}` with a
    per-case registry and one job scheduler; S3 adds Postgres users, sessions,
    roles and the login UI. Deferred follow-ups: S3 snapshots of case files
    and multi-instance advisory locks (several ruxd hosts sharing cases).

- **2026-10-06** — **GUI: Segmentering view and mask → resource**
  ([spec](superpowers/specs/2026-10-06-kortlaegning-segmentering-fixes-design.md),
  Stream B). Interactive SAM3 segmentation gets its own screen and can now
  feed the survey:
  - **`/segmentering`** (§B3), a case-section nav entry between Viewport and
    Miljø & prøver: a frame filmstrip, a stage for drawing boxes and clicking
    points, the per-prompt result list with the mask overlay, and "Tilføj til
    kø" for the neighbouring frames. It replaces the SegmentPanel fold-out in
    Billeder, which now links here; Kortlægning's **S** key opens the selected
    part's best frame with its pixel as a point prompt.
  - **No model path in the GUI** (§B1): the server's managed SAM3 model is
    always used; a status chip reports download/build progress and a pending
    run restarts by itself when the model is ready.
  - **Box and point prompts reach SAM3 on TensorRT** (engine recipe v2,
    `recipe_version: 2`): the geometry encoder and decoder are built with room
    for geometry tokens, so a box selects the object(s) it covers and a click
    the object under it. Existing managed installs rebuild those two engines
    once on first use (no download; the status reports `update_engines`). An
    export whose graph cannot take the v2 profile falls back to a text-only
    build, recorded permanently with the ONNX's sha256 and retried only when
    the recipe or the ONNX changes; the segment response's
    `geometry_prompts_used` tells the UI when geometry was only clipped to.
  - **Mask → resource** (§B2, `POST /frames/{id}/segment/resource`): one
    label of a frame's saved mask is projected through the frame's pose and
    depth into the cloud and filed as a new instance and survey part, never
    in a rejected (Afvist) type. The request echoes the run's `mask_revision`,
    so a mask overwritten in between (the label queue) is refused with 409.
  - **`clouds.changed`**: a WebSocket message naming the rewritten clouds;
    the Viewport reloads them in place.

- **2026-10-06** — **GUI: Kortlægning and navigation fixes**
  ([spec](superpowers/specs/2026-10-06-kortlaegning-segmentering-fixes-design.md),
  Stream A). Several independent fixes to the 2026-09-30 GUI redesign, landed
  together:
  - **Reject ≠ delete** (§A3). Afvis keeps a type, now listed in its own
    **Afvist** tab; **Slet** is a separate, confirmed action for any part or a
    whole type. Deleting a scan-backed part records its instance guid in a
    tombstone table, `survey_dismissed_instances` (schema v26), which
    `sync_survey` respects, so a deleted resource stays deleted across "Opret
    kortlægning" re-runs. Because the tombstones are kept, a later **"Gendan
    slettede scan-ressourcer"** (restore deleted scan resources) is possible:
    drop the rows and re-sync. Not built yet.
  - **Projektdata retires as a screen** (§A2): its tables (point clouds,
    meshes, components, schema/path) and the pipeline log move into a
    collapsed-by-default disclosure at the foot of Overblik, next to the
    existing KPIs. `Dashboard.tsx` is deleted and `/projektdata` redirects to
    `/` — the nav entry is gone.
  - **Værktøjer is collapsible** (§A1): the sidebar's technical-tools group is
    closed by default, auto-opens while the active route is one of its own,
    and remembers the viewer's explicit choice (localStorage, degrades to
    closed).
  - **Photo and 360° evidence** (§A4/§A5): Kortlægning's table rows and detail
    panel now show a thumbnail and photo count per part (`GET
    /survey/photos`, occlusion-aware against each frame's depth image, not
    just frustum membership), and the evidence panel gained a real **360°**
    tab (`GET /instances/{cloud}/{id}/panoramas`) alongside Plan · Foto ·
    Punktsky · Rum — five evidence tabs in all, keys 1–5. 360° prefers a
    resected panorama (measured heading) over a nearer levelled one (position
    only), and shows "retning ukendt" with no marker when it falls back to
    the latter.

- **2026-10-02** — **GUI: resources, templates and navigation cleanup**
  ([spec](superpowers/specs/2026-10-02-resources-templates-ia-design.md)).
  Materials are now *resources* in the GUI: one per survey part, carrying any
  number of key/value pairs, with the material passport as one set of keys
  (schema v25). Project-stored **templates** select keys by category or one by
  one; Kortlægning edits through them, the new **Skabeloner** page maintains
  them (two seeds: Materialepas (fuld), Hurtig genbrugsscreening), and
  **Rapport** exports CSV and a PDF Ressourcetabel through them. Eksport and
  Materialedata are retired into Rapport and Kortlægning, Viewport moved into
  the case section, and **On-site left `rux gui` and the backend** — it moves
  to the future mobile app. xlsx export, a global template library and the
  Excel-supplied category selection are deferred.

- **2026-10-02** — **GUI application (#265): the prototype-v2 redesign is
  complete — Phases 1–6 have landed.** Phase 6 added Sager (the single-project
  case list `rux gui` can serve) and On-site (the phone sheet that writes ★, a
  note and a sample against a bygningsdel; schema v24 records the part a sample
  was taken at), made the shell responsive below 900px, and put every screen's
  writes on one app-wide chain. What remains are follow-ups, not phases: the
  multi-case list and switching belong to `ruxd` (#265's own Phase 6, a
  different numbering), and photo capture waits for blob storage — see the
  spec's Out of scope list and `.github/issue-drafts/36`–`42`.

- **2026-10-01** — **GUI application (#265): Phases 1–5 of the prototype-v2
  redesign landed.** Identity & shell, the survey backend (schema v22/v23),
  the Kortlægning workbench, Miljø & prøver, and Overblik/Rapport/Indberetning
  are all built and replace the old Dashboard-first layout — see
  [`docs/design/gui-kortlaegning-redesign.md`](design/gui-kortlaegning-redesign.md)
  for the full screen-by-screen spec and the remaining follow-ups. Phase 6
  (Sager & On-site — multi-project case list via `ruxd`, the phone capture
  sheet) is the only phase left on the anchor issue.

- **2026-09-21** — **Viewport LOD: extend RUXP, do not adopt Potree** (#321
  closed, decision accepted by the maintainer). The evaluation
  ([`research/potree-vs-ruxp-lod.md`](research/potree-vs-ruxp-lod.md), PR #384)
  found a scale mismatch (our 1–15 M-point clouds vs Potree's >100 M design
  point), an asymmetric integration cost (no LAS export path; PotreeConverter
  rejects our PLY; a custom Potree 2.0 writer is ~1 500 lines), and no Potree
  equivalent for RUXP's index-aligned label streaming (`lod_source`). The GUI
  workstream instead builds LOD on the proven RUXP stack: Morton-ordered cloud
  storage at `create clouds` time (#394), a spatial tile index with
  frustum-culled fetching (#395), and stride-sampled multi-level LOD (#396) —
  per #320's original design goal that paging and LOD collapse into one
  mechanism. *Revisit Potree if:* scans consistently exceed ~50 M points, or
  multi-user CDN/edge serving becomes a requirement.

- **2026-09-10** — **The ~10 mm office flatness target should be retired as an
  acceptance criterion** (#225 §11), because it is simultaneously *already
  reachable* and *not measuring what priority 1 wants measured*. Three findings,
  each measured. (1) §9.8's determinism smell is a **float32 round-trip in the
  pose write-back**: `ProjectDB` stores poses as `double[16]` but the surfel /
  write-back path is `Affine3f`, so a solve that provably cannot move a pose
  still rewrites 237 of 238 stored blobs, by up to 1.43 µm. The pipeline itself
  is bit-reproducible. (2) That µm-scale change matters because **the office
  flatness metric is chaotic**: a deliberate 4.4 µm pose perturbation moves
  `flatness_rms` across 11.76–13.25 mm (sd 0.64 mm) and the plane count across
  63–67, since the 5 cm voxel grid and the region-growing thresholds are
  discrete and the metric averages over whatever plane set comes out. It is also
  strongly confounded by that count — `flatness = 25.4 − 0.206 × planes`,
  r = −0.80 over 78 runs — so it **rewards fragmentation**. Every office
  comparison must now be seed-averaged (`scripts/bench-office.sh`, mean ± sd over
  five µm-perturbation seeds) and must report the plane count; the smallest
  difference a single pair of runs can support is ~1.5 mm, not the ~0.2 mm §9.8
  suggested. Re-measured that way the shipped `optimize` default is a genuine
  local optimum (11.45 ± 0.25 mm; every knob in either direction is neutral or
  worse), its win over no pose stage survives at ~3.5σ, and one recorded result
  **reverses**: `--plane-noise fit` loses on office (12.10 ± 0.33), its recorded
  11.66 mm having been one lucky draw from a more-fragmented ensemble. Its
  ARKitScenes GT evidence stands. One result is **corrected**: office XFeat edges
  are not "163 PCM-surviving" — 163 is the pre-PCM count, PCM keeps **3**, and
  the configuration is bit-identical to plain `optimize`. (3) `rux register`
  clears the target — 9.83 ± 0.34 mm, or **8.33 ± 0.61 mm** tuned — but JPR
  minimises point-to-plane residual and `flatness_rms` *is* point-to-plane
  residual, and the honka laser-GT control shows that same stage improving
  flatness while **lowering GT F 0.795 → 0.756** with *fewer* planes. So the
  target is satisfiable by a stage measured to make the reconstruction worse.
  The blocker to settling it is quantified: JPR proposes **542 070 candidate
  pairs on 41069050's 1 859 frames** (vs 1 293 on office's 238) because these
  captures orbit a small room, projecting to ~17 h per run — the GT benchmark
  cannot currently score the one stage that hits the target. **Highest-value
  next item: bound JPR's candidate-pair set**, then re-anchor priority 1 on
  F@50 mm over the #338 drifted variants instead of on office flatness.

- **2026-09-10** — The benchmark blocker named on 2026-09-09 is **cleared**, and
  it reverses the previous entry's headline. Synthetic, seeded drift on the
  ARKitScenes seed poses (`rux edit perturb-poses`, #338) turns three unusable
  scans into a drifting benchmark with absolute GT, and on it **`rux optimize`
  is GT-positive for the first time in this workstream**: at a moderate drift
  level (0.09–0.13 m, ~2x the measured loop-edge error) F@50mm goes 0.4640 →
  0.7927, 0.5393 → 0.6378 and 0.4182 → 0.7607, with median accuracy roughly
  halved. The mechanism is the **plane term** — `--no-plane-factors` reproduces
  *no pose stage* bit-exactly on every drifted row, because with the plane term
  off the graph is odometry plus a gauge prior and the odometry chain *is* the
  drifted chain. So "the plane term costs absolute accuracy on drifting
  captures" was an artefact of measuring it on captures with no drift to remove;
  the term is a drift-recovery mechanism whose benefit is conditional on drift
  existing. Two consequences worth carrying: the **GT-free flatness metric and
  absolute accuracy now agree** on every drifted row (they were only ever
  measured to disagree where the seed was already right), which raises the trust
  in the office scan's numbers; and recovery **collapses at heavy drift**
  (0.36–0.51 m) because plane association is seeded from the drifted poses
  through a 0.10 m gate, so the landmarks that would tie distant frames together
  are never formed — visible as the same surfaces shattering into ~2x the plane
  count. That gave a testable prediction about what a global constraint is
  needed *for* — not to replace the plane term but to pull the seed inside its
  association basin — and **the test passes**: at heavy drift, `optimize` with
  the plane term *and* XFeat loop edges reaches F@50mm **0.6042** (41069050) and
  **0.5406** (41069051), against 0.2980 / 0.2305 for the plane term alone and
  0.3478 / 0.3361 for the edges alone. Not additive — the edges restore the
  association basin and the plane term then does the fine correction, visible in
  the plane count returning from 35 to 15 on 41069051. This is the **first
  measurement in the workstream of loop closure paying against absolute GT**,
  and it re-reads #337's negative result as scan-dependent rather than
  fundamental. It also makes **matcher error a ranked objective**: the same
  configuration does nothing on 41069048, whose XFeat edges carry 169 mm of
  their own error versus ~58 mm on the other two. Loop closure (#236, #311) and
  an association-gate sweep are now both measurable experiments rather than
  guesses.
  Also landed: the loop-edge seed-disagreement gate is **scale-relative** (#339),
  since a 1.8 m room scan and an 18 m walk can no longer share an absolute
  threshold. Full write-up: `docs/research/registration-improvements.md` §10.

- **2026-09-09** — The #225 benchmark itself is now the blocker, not the
  algorithm. Two measured questions both answered no: a **plane-term weight
  sweep against absolute GT** has no interior optimum on the drifting scans
  (accuracy improves monotonically as the term is weakened; the best setting is
  the new `--no-plane-factors`, which reproduces *no pose stage* bit-exactly),
  and **XFeat loop edges** — the one front-end whose edges survive PCM and which
  closes 16.66 m of office drift — **lose GT accuracy in every configuration**
  on all three ARKitScenes scans. The reason is a signal-to-noise argument that
  reframes the workstream: those trajectories span only **1.8–2.6 m** and their
  seed poses already reconstruct to **15.7–18.6 mm** median accuracy, while an
  XFeat edge disagrees with the seed by **58–169 mm** — its own error, four
  times larger than the error it would correct (on office the same measurement
  is 14.5 m of disagreement on an 18 m trajectory, i.e. real drift). So **the
  ARKitScenes scans cannot adjudicate pose refinement at all**: on a well-posed
  capture the correct behaviour of every pose stage is to do nothing, and that
  is exactly what wins. #221 Tier 2 asked for a drifting scan *with* absolute
  GT; these supply the GT but not the drift. The next increment is therefore
  **benchmark work, not solver work** — cheapest first: synthesise realistic
  drift on the ARKitScenes seeds so their GT meshes become a drifting benchmark.
  Also measured and worth carrying: the plane term is *correctly* calibrated for
  well-posed captures (honka's GT optimum and office's GT-free optimum are both
  the shipped default), so this is a capture-dependent split, not a
  miscalibration; and the shipped 0.50 m seed-disagreement gate is
  scale-dependent — on a 2 m trajectory it selects the *wrong* edges and drops
  F to 0.29, so it should become relative to trajectory extent. Full write-up:
  `docs/research/registration-improvements.md` §9.

- **2026-09-09** — MapAnything evaluated as a loop-edge matcher (#264) and
  **rejected on measurement, not on licence**. The licence question closes
  cleanly in our favour: `facebook/map-anything-apache` is genuinely Apache-2.0
  (verified on the model card), so a pointmap *foundation* model is commercially
  available to us for the first time. But on the office scan's 1225-pair endcap
  set it produced 148 edges with the highest inlier support of any backend
  (9203, above the MASt3R NC oracle) — of which **PCM rejected 145 as mutually
  inconsistent**, leaving a 0.21 m correction on a scan that needs ~16 m, where
  XFeat's 163 edges survive intact and apply 16.66 m. This is the #236 failure
  repeated from a different front-end: a pointmap model asked about two blank
  repetitive walls always returns a geometry, so per-pair confidence is high and
  cross-pair agreement is absent. **Σ inliers is therefore not a matcher
  ranking** — it rewards precisely what PCM punishes, and any future matcher
  comparison should be judged on edges-surviving-PCM and applied correction.
  (#312 carried this into the tooling: `scripts/bench-loop-edges.sh` now emits
  `pcm_kept` and `max_shift_m` as its primary columns and §5.1's table leads
  with them, so the comparison cannot be read the wrong way round.)
  The shipping recommendation is unchanged (XFeat / LightGlue+ALIKED); the C++
  ingestion path for MapAnything is **not** worth building. Full write-up:
  [`research/loop-closure-learned-matchers.md`](research/loop-closure-learned-matchers.md)
  §5.5–5.6.

- **2026-09-08** — 360-driven pose-graph loop closure (#236) implemented and
  measured on the NewOffice drifted scan: the mechanism, guardrails and CLI
  landed, but the effect is negative (flatness_rms 25.15 → 27.08 mm vs the
  `optimize` baseline) because cross-camera ORB resections between the 360
  camera and the iPad are too weak — forcing the edges through `--loop-trust`
  collapses quality to 35.03 mm, confirming they are genuinely wrong rather than
  merely suppressed. The workstream's next step is therefore a **learned
  matcher**, which moves it behind the #221/#225 matcher work rather than ahead
  of it. Notably this does **not** settle the #225 odometry-trust hypothesis:
  edge quality is a confound, and the clean test still needs accurate
  wide-baseline edges. Full write-up:
  [`research/panorama-loop-closure.md`](research/panorama-loop-closure.md).

- **2026-09-08** — Document created (#266). Established the four-way priority
  ordering (SLAM/reconstruction quality → GUI → Gaussian splatting →
  infrastructure/test health) and the six active workstreams. Seeded from the
  open issue set, the "Reuse Development" project board, `docs/CONTRACTS.md`,
  `docs/STANDARDS.md`, and an unpublished 2026-05 progress draft — the last of
  which was verified against the tree and found stale in several places
  (module split #222, CLI spellings, SAM2 → SAM3/3.1, HiGHS-only → cuOpt/HiGHS
  solver choice, schema v4 → v11); the code won every disagreement.
