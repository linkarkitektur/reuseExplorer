<!--
SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
SPDX-License-Identifier: GPL-3.0-or-later
-->

# ReUseX Engineering Standards

This document defines the objective bar every change must meet — whether written
by a human or an AI agent. It complements `CLAUDE.md` (naming conventions, build
instructions, TODO format) and is referenced by CI checks and code review.

Rules marked **[target]** describe the intended state; existing code may not yet
comply. New code must comply; touched code should be migrated opportunistically.

---

## 1. Module boundaries

The library (`libs/reusex/`) is layered into one CMake target per module
(`reusex_<module>`, see `libs/reusex/cmake/reusexLibrary.cmake`). A module may
only depend on modules in lower layers, and this is **link-enforced** (#222):
an illegal dependency is a link error, not just a convention.

```
Layer 4:  visualize  pipeline                  (visualize: PCL/VTK — see note below)
                                               (pipeline: stage execution + job runner)
Layer 3:  segmentation  reconstruction  slam  io  vision
          gsplat_common  gsplat                (peers — MUST NOT link each other)
                                               (gsplat: optional — WITH_CUDA + gsplat-cuda;
                                                gsplat_common: always built, torch-free)
Layer 2:  core                                 (ProjectDB, logging, materials, stages)
Layer 1½: geometry_common                      (shared CGAL/PCL helpers: utils, cgal_utils,
                                                CoplanarPolygon, BuildingComponent)
Layer 1:  utils, types.hpp                     (no internal dependencies)
External: apps/rux                             (may use everything; keeps logic thin)
          ruxd (own repo, via find_package(ReUseX))
```

The former single `geometry` module was split into the pipeline-stage peers
`segmentation`, `reconstruction`, and `slam`; the CGAL/PCL primitives they share
live in `geometry_common` (just above `utils`) so no peer or upward dependency
is needed to reach them. `geometry_common` links neither `core` nor any peer.

`visualize` is not an optional build: `rux` links PCL visualization
unconditionally, and there is no option to disable it. The module holds the
rendering logic that is not tied to an interactive session — `viewport_layout`
and the headless `render_view()` (#294), which drives VTK's off-screen render
window directly and needs no display server. The *interactive* viewer
(keyboard callbacks, panorama navigation, toggles) still lives in
`apps/rux/src/view*` (External layer), because it is a UI, not a library
concern.

`gsplat` is **two** Layer-3 targets, split by dependency rather than by
responsibility (#332). `reusex_gsplat_common` is the torch-free half —
`GaussianCloud`, the `ProjectDB -> TrainingView` loader, the view-sampling
helpers — and is built unconditionally, like any other peer.
`reusex_gsplat` adds the CUDA training loop and exists only under `WITH_CUDA`
plus the vendored rasterizer; it links `reusex_gsplat_common` PUBLIC. The split
exists because CI builds the CPU-only variant: a single CUDA-gated target meant
the module and all its tests disappeared from the only build a pull request
gets to prove anything with. Both halves list their sources **explicitly** in
`reusexLibrary.cmake` (like `geometry_common`), so a new file must be assigned
to one side deliberately — adding a `src/gsplat/*.cpp` needs a CMake edit.

`pipeline` (#265) is the other Layer-4 module. It holds the database-level
stage runners (`run_stage(ProjectDB&, StageContext)`) and the in-process
`JobRunner` that the Qt client drives directly and that `ruxd`'s web API
drives from its own repo (`ruxd --local` today, the multi-case server next).
**It is the one module
permitted to link several Layer-3 peers at once** — running a stage end to end
inherently spans `core` (the ProjectDB read/write) plus whichever peers
implement that stage, which is exactly the combination a peer is forbidden to
make. Confining that combination to a single named module above the peer layer
is what keeps the peer rule meaningful instead of eroding it stage by stage.

The allowance comes with obligations:

- **Nothing may depend on `pipeline` except `apps/` and `tests/`.** If a
  Layer-3 peer ever needs something from it, that something belongs in `core`
  or `geometry_common`, not in a new upward edge.
- **It links only the peers it actually calls.** Today that is `segmentation`;
  `reconstruction` joins when the mesh stage gets a runner. Linking a peer
  "for later" hides which combinations are really in play.
- **It owns no algorithms.** `pipeline` loads named clouds, calls a peer's
  entry point, saves the results, and records the run in `pipeline_log`. An
  algorithm that appears there belongs in a peer.

**Rules:**

- `core/` MUST NOT include PCL visualization, CGAL, RTABMap, or ML headers.
  The PCL/Eigen-typed visualization payloads live in `core/visual_observer.hpp`,
  which core headers never include; `core/processing_observer.hpp` is PCL-free
  and exposes the visual observer only as an opaque forward-declared pointer.
- `segmentation`, `reconstruction`, `slam`, `io`, and `vision` are peers.
  Cross-peer data exchange goes through types defined in `core/`,
  `geometry_common`, or `types.hpp`, not through peer headers. Documented
  exceptions (explicit, commented links in `reusexLibrary.cmake`): `io ->
  reconstruction` (export_scene serializes a reconstructed scene) and `slam ->
  segmentation` (registration reuses surfel extraction). Code that legitimately
  needs to combine several peers goes in `pipeline` (Layer 4), not in a peer.
- `pipeline` MUST NOT be linked by any library module. It is reachable from
  `apps/` and `tests/` only; the umbrella `reusex` target links it for
  backward compatibility with consumers that link the umbrella.
- Persistence contracts are core-owned PODs. `ProjectDB` stores building
  components as `core::ComponentRecord` (`core/component_record.hpp`) — plain
  strings, blobs and scalars mirroring the table columns — so core needs no
  geometry types. The `BuildingComponent <-> ComponentRecord` mapping lives on
  the geometry side, in the header-only adapter
  `geometry/component_persistence.hpp`, which is compiled into its consumers
  (`io`, `visualize`, `apps/rux`, tests) and deliberately not into
  `reusex_geometry_common`,
  so that module keeps its Layer-1½ position (#227).
  `pipeline/optimize_stage.hpp` follows the same pattern: the in-process
  `optimize` stage needs `slam` (GTSAM), which `reusex_pipeline` must not
  link (#464), so it is header-only and compiled into the apps that run it
  (`rux` for the Qt client, and `ruxd`, in its own repo, for the web GUI),
  never into a library module.
- `apps/rux/` subcommands are thin wrappers: parse arguments, validate, call
  one library entry point, report. Business logic lives in the library.
- **An app target owns exactly one symbol the tests cannot link: `main`.**
  Everything else in `apps/<app>/src/` goes into a static library, and the
  executable is the `main()` translation unit plus a link line (#249). See
  the table below.
- New heavyweight dependencies (anything that adds > 30 s to a clean build)
  require an issue discussing alternatives before being added.

**Pipeline data contract:** the *data* dependencies between stages (which named
clouds/tables each stage consumes and produces) are specified in
[`CONTRACTS.md`](CONTRACTS.md) and enforced at runtime by
`rux validate --stage <name>`.

### 1.1 App targets

`apps/` sits above the library layers and may link anything, but it is not
exempt from being tested. A bare `add_executable(<app> <all the sources>)`
makes every line in it unreachable from ctest, because a translation unit
defining `main` cannot be linked into a Catch2 binary that supplies its own
(#249). So:

| Target | Contents | Test directory | Test binary |
|---|---|---|---|
| `rux_core_lib` | app logic that stops at `reusex_core`: `path_parser`, `filter_utils`, `stage_prerequisites`, the stdin/format handlers | `tests/unit/rux/` | light |
| `rux_lib` | everything else under `apps/rux/src`: subcommands, `database/*_router.cpp`, the interactive viewer, `rux::run()` | `tests/unit/rux_app/` | heavy |
| `rux` | `src/main.cpp` — `return rux::run(argc, argv);` | — | — |

`ruxd` (the HTTP service worker) and its own app-target split
(`ruxd_api_lib`, `ruxd_cli_lib`, `ruxd_pg_lib`, `ruxd_lib`) moved to the
standalone `ruxd` repo (repo split, P4, 2026-10-09); this table only covers
`apps/rux` now. `ruxd` consumes this library through its installed CMake
package (`find_package(ReUseX CONFIG REQUIRED)`), linking `ReUseX::core` +
`ReUseX::pipeline` for its light API layer, the same way `rux_core_lib` does
in-tree.

Rules for adding to `apps/`:

- **New source files need no CMake edit.** `rux_lib` is
  `GLOB_RECURSE ... CONFIGURE_DEPENDS`; only `rux_core_lib` is an explicit
  list, and only because membership there is a *claim* — that the file links
  no further than `reusex_core`, and therefore that its tests can stay in the
  fast binary. When in doubt, leave the file in `rux_lib`.
- **New app code does not go in `main.cpp`.** `apps/rux/src/main.cpp` is one
  statement and stays that way; CLI wiring belongs in `rux::run()`
  (`apps/rux/src/rux.cpp`), inside the library.
- **Splitting an app library further is a dependency decision, not a
  responsibility one.** `rux_core_lib` exists because it keeps the ML/viewer
  closure out of the light test binary (§7), which is the only thing that
  makes its tests cheap. Do not split for tidiness.

## 2. Header hygiene

- Public headers (`libs/reusex/include/**`) include the *minimum* needed.
  Prefer forward declarations; use the Pimpl idiom to keep sqlite3, RTABMap,
  TensorRT, and CGAL out of public headers (as `ProjectDB` and `Solidifier`
  already do).
- Do not add new includes of PCL, CGAL, Boost.Graph, or OpenCV to a public
  header if a forward declaration or a `.cpp`-local include suffices.
  `core/ProjectDB.hpp` forward-declares `cv::Mat` and the PCL mesh types
  (`pcl::PolygonMesh`/`pcl::TextureMesh`) rather than including their headers.
- The `types.hpp` aliases are split so a TU pulls in only what it uses:
  `types/point_types.hpp` (PCL point-cloud aliases) and
  `types/eigen_types.hpp` (Eigen helpers). `types.hpp` remains a
  backward-compatible umbrella re-exporting both; prefer the narrower headers
  in new/touched code.
- Public headers include siblings via the `reusex/` prefix
  (e.g. `#include "reusex/core/logging.hpp"`).
- `#pragma once` everywhere; SPDX header on every file.

## 3. Label & identity contract

This is the single source of truth for label semantics. Any code that reads or
writes labels MUST follow it; any deviation is a bug.

### 3.1 Encodings per layer

| Layer                                   | Type       | Unlabeled/background | Valid labels |
|-----------------------------------------|------------|----------------------|--------------|
| `segmentation_images` storage (PNG)     | `CV_16U`   | `0` (stored as label+1) | `1..65535` (i.e. label ≤ 65534) |
| `ProjectDB` segmentation API            | `CV_32S`   | `-1`                 | `0..N-1` (annotate prompt order) |
| In-memory point labels (`pcl::Label`)   | `uint32_t` | `0` **[target]**     | `1..N`       |

**Rules:**

- `0` means "unlabeled" in every `CloudL` point cloud (labels, planes, rooms,
  instances). Valid labels start at `1`. **[target]** — room segmentation
  currently initializes to `-1` (wraps to `0xFFFFFFFF`).
- **Semantic point label = API class + 1.** `api_to_point_label(c)` is
  `c + 1` for `c >= 0` (background `-1` → `0`) and `point_label_to_api()` is
  its inverse, so API class 0 — the first prompt class — is point label 1,
  never unlabeled (decision D8, schema v29). A semantic label cloud's
  `label_definitions` are keyed by point label, and every writer of one stamps
  `"label_encoding": "api_plus_one"` (`kLabelEncodingKey` /
  `kLabelEncodingApiPlusOne`) into the cloud's `parameters`. The
  `annotate_class_map` pipeline-log entry and `segmentation_images` stay
  API-keyed. Structural label clouds (planes, rooms, instances) number their
  own ids from 1 and carry no encoding key. The v29 migration shifted every
  stored semantic cloud lacking the key by +1 (0 stays 0), re-keyed its
  definitions, and re-based `instances.semantic_class`, the `SM<cls>-`
  instance definitions and `survey_types.semantic_class` with it.
- Never index a vector with `label - 1` without first checking `label >= 1`.
- Conversions between the layers above happen ONLY through the helpers in
  `core/label_semantics.hpp` (`api_to_point_label`, `point_label_to_api`,
  `is_valid_label`, `label_to_index`, `kUnlabeled`, `kBackgroundApi`,
  `kMaxStorableLabel`, …), never inline `label ± 1` arithmetic or raw casts
  at call sites. **[target]** — existing call sites are still being migrated.
- Writing a label ≥ 65535 to segmentation storage MUST throw, not wrap.

### 3.2 Identity & referential integrity

- **Stable identity**: entities that outlive a single pipeline run (material
  passports, building components, instances) are identified by GUID, never by
  sequential index alone. **[target]** — instance IDs are currently
  regenerated sequentially on every `segment_instances` run.
- **No silent orphaning**: an operation that invalidates an association
  (regenerating instances, downsampling a cloud with derived label clouds)
  MUST either migrate the association or fail/warn loudly. Silent breakage of
  `instance_materials` links is a bug.
- **Parallel clouds move together**: "cloud", "normals", "labels", "planes",
  "rooms", "instances" for the same scan are index-aligned. Any filter,
  downsample, or reorder MUST be applied to all of them (or invalidate the
  derived ones explicitly).

## 4. Parameters

- Every tunable parameter lives in exactly one place: an options struct in the
  library header (e.g. `SegmentPlanesOptions`). CLI flags mirror the library
  defaults — never define a second default in `apps/rux`. **[target]**
- No new magic numbers in algorithm code. Thresholds, epsilons, and limits go
  into the options struct or a named `constexpr`.
- Geometric epsilon policy: use the named constants in
  `utils/tolerances.hpp` (`#include "utils/tolerances.hpp"` internally, or
  `<reusex/utils/tolerances.hpp>` from consumers) rather than ad-hoc
  `1e-6`/`1e-9`/`1e-15` literals. Absolute tolerances state their unit (meters)
  in the name (`...M`) or a comment. The constants and their intended use:

  | Constant | Value | Unit | Use for |
  |----------|-------|------|---------|
  | `kEpsilonLengthM` | `1e-9` | meters | Absolute length zero-check: is a distance / edge length / vector norm effectively zero? Only meaningful on quantities in the scene's metric frame — recenter georeferenced coordinates first (see §4 recentering). |
  | `kEpsilonAngleRad` | `1e-6` | radians | Absolute angular zero-check / parallelism when the quantity is an angle (e.g. `acos(dot)`). |
  | `kEpsilonRelative` | `1e-12` | — | Dimensionless / normalized comparisons: dot products of unit vectors, squared norms of unit normals, ratios. This is the guard for divisions by a squared norm. |
  | `kToleranceCoplanarM` | `0.01` | meters | Coplanarity / spatial-adjacency tolerance: planes whose parallel offsets differ by less than this are the same surface; inlier patches with a gap below this are adjacent. |

  When comparing two vectors for parallelism, convert to an angle and use
  `kEpsilonAngleRad`, or compare `1 - |dot|` against `kEpsilonRelative`; do not
  invent a per-call-site literal.

- **Coordinate normalization before CGAL**: georeferenced (UTM / 500 km-scale)
  coordinates fed to CGAL's inexact-construction kernel lose precision.
  Recenter geometry to its bounding-box centroid before cell-complex /
  arrangement construction and restore the offset on output. `CellComplex`'s
  constructor does this internally.

## 5. Error handling & diagnostics

- **No silent failure.** A pipeline stage that produces empty/degenerate output
  (0 planes, 0 cells, infeasible MIP, empty mesh) MUST log at `warn` or above
  *with the reason and the relevant quantities* (e.g. "MIP infeasible:
  412 cells, 87 planes — consider stronger plane merging").
- Long-running external calls (MIP solve, Leiden clustering) MUST have a
  configurable timeout/iteration limit. Unbounded loops are bugs.
- Validate inputs at library entry points: sizes match, clouds non-empty,
  required property maps present. Fail fast with `std::runtime_error` and a
  message that names the missing/mismatched thing.
- `assert()` is for programmer invariants only — never the sole guard against
  bad data (it vanishes in Release builds).
- **No locale-dependent number parsing in library code.** `std::stod`,
  `std::stof` and `strtod` follow the process's `LC_NUMERIC`, and any
  `QApplication` calls `setlocale(LC_ALL, "")`: under `da_DK` `std::stod`
  reads "0.75" as 0 and a `strtod` loop stops dead at the '.'. This broke the
  Qt client twice (Q2 review: wrong sensor intrinsics and a fake ICP result,
  both from a library `strtod`, not from Qt). Use `reusex::utils::to_double` /
  `to_float` / `parse_number<T>` (`utils/parse_number.hpp`, built on
  `std::from_chars`, which never consults the locale) for every text -> number
  conversion in library code (`SensorIntrinsics`, `materialepas`,
  `arkitscenes`, `glass_filter`, the TensorRT `Dataset`, …). A `std::stod`/
  `strtod` in `libs/reusex/` is a bug, not a style preference.

## 6. Determinism & reproducibility

- Algorithms with randomness (RANSAC, Leiden, shuffling) MUST accept a seed in
  their options struct. Default seed is fixed (`42`); pass entropy explicitly
  when non-determinism is desired. **[target]** — `Dataloader` currently uses
  `std::random_device` unconditionally.
- Given the same inputs, seeds, and thread count, a pipeline stage must
  produce identical output. Parallel reductions that break this need a
  deterministic mode.
- This is what makes before/after comparison — and therefore agent-driven
  optimization — possible. Treat regressions in determinism like functional
  regressions.

## 7. Testing

- **Unit tests** (Catch2, `tests/unit/<module>/`): every new public function
  in `geometry/`, `io/`, `core/` gets at least one test with a real (possibly
  synthetic) input, including one degenerate case (empty cloud, single point,
  all-collinear, etc.).
- **Integration tests** (`tests/integration/`): changes to pipeline stages
  must keep the end-to-end pipeline test green.
- Tests use fixed seeds and `WithinAbs`/`WithinRel` matchers for floats.
- Bug fixes come with a regression test that fails before the fix.
- **Temp files**: never derive a temp path from an object address
  (`reinterpret_cast<uintptr_t>(this)`). `catch_discover_tests` gives every
  test case its own process, and under ASLR two concurrent ctest workers
  routinely land on the same address and then fight over the same file
  (#262). Use `tests/support/temp_path.hpp` —
  `reusex::test_support::TempPath` / `TempDir` — which names files
  `<prefix>_<pid>_<salt>_<counter>` and cleans up sqlite `-wal`/`-shm`
  sidecars. The suite must stay green under `ctest --parallel $(nproc)`.
- **Which test binary**: unit tests build into `reusex_unit_tests` (light) or
  `reusex_unit_tests_vision` (full dependency closure). A test lands in the
  heavy binary by living in `tests/unit/vision/`, `tests/unit/rux_app/`,
  `tests/unit/visualize/` or `tests/unit/gsplat/cuda/`. Keep new tests out of
  those directories unless they genuinely need the ML backends, `rux_lib`,
  the PCL/Qt viewer or torch: linking libtorch/TensorRT adds ~0.6 s of
  dynamic-loader time to *every* test process in that binary (#268). `ruxd`'s
  own tests (formerly `tests/unit/ruxd{,_api,_cli,_pg}`) moved with it to the
  standalone `ruxd` repo (repo split, P4).
- **App-layer tests** (`apps/rux`) are unit tests like any other; the app
  code is reachable because the app is a static library plus a one-line
  `main.cpp` (§1.1). Which directory a test goes in follows the same
  light/heavy rule as everything else: `tests/unit/rux/` for anything covered
  by `rux_core_lib`, `tests/unit/rux_app/` only when the test needs a
  subcommand, a `database/*_router.cpp` or the viewer. Adding a file to
  either needs no CMake change — `tests/unit/**/*.cpp` is globbed and the
  libraries are already linked.
- **gsplat tests**: `tests/unit/gsplat/cuda/` is for tests that need torch or
  the rasterizer; it is dropped entirely from a build without
  `REUSEX_HAVE_GSPLAT`. Tests directly under `tests/unit/gsplat/` cover
  `reusex_gsplat_common` and build everywhere, including the CPU-only variant
  CI runs — so that is where a gsplat test belongs unless it truly cannot be
  written without torch (#332).
- **`[gpu]` tests SKIP, they do not fail.** A test that needs a CUDA device
  opens with a device probe (`gsplat::has_cuda_device()`, `cuda::is_available()`)
  and Catch2's `SKIP()`. "You ran the wrong ctest command" is not a test
  result; a skip is.

### 7.1 Test naming convention

Every `TEST_CASE` name follows `Subject_Scenario_ExpectedBehavior` (#319):

- **Subject** — the function, method, or class under test (e.g. `LimitToRange`,
  `SplitCommand`). For integration/pipeline tests, where there is no single
  free function under test, use the subject actually under test — a fixture
  or pipeline stage (e.g. `RealScanFixture_Reconstruction_PlaneCountWithinBounds`
  exercises the `RealScanFixture` running the `Reconstruction` stage).
- **Scenario** — the input condition or state being exercised (e.g.
  `ValueBelowRange`, `UnsupportedManufacturer`, `NegativeIndex`).
- **ExpectedBehavior** — the observable outcome, stated as a fact about what
  happens, not a wish (e.g. `ReturnsMinimum`, `LeavesCommandUntouched`,
  `Throws`).

Canonical examples (from #319):

```
LimitToRange_ValueBelowRange_ReturnsMinimum
SplitCommand_UnsupportedManufacturer_LeavesCommandUntouched
InfinitElementAt_NegativeIndex_Throws
```

Catch2 specifics:

- The convention governs the `TEST_CASE` name string (its first argument)
  only. Tags — the second `TEST_CASE` argument, e.g. `"[gpu]"` — are
  unaffected and keep whatever tagging scheme is already in use.
- `SECTION` names stay free-form/descriptive; they are not bound by this
  convention. When one `TEST_CASE` covers several scenarios via `SECTION`s
  (a common pre-#319 pattern), name the `TEST_CASE` for the scenario *class*
  the sections share rather than any single one of them — e.g. a case whose
  sections cover the default, custom, and inverted output ranges of `remap`
  becomes `Remap_VariousRanges_MapsLinearly`, and the individual sections keep
  their own descriptive names (`"Remap to custom output range"`, etc.).
- `ctest -R <pattern>` matches against the `TEST_CASE` name, so a bulk rename
  changes which regex targets a given test. Call this out explicitly in any
  PR that renames tests in bulk (link old → new for anything a script or CI
  job depends on by name).
- `BENCHMARK` names (`tests/benchmarks/`) are not covered by this convention;
  they describe a measured operation, not a pass/fail expectation.

## 8. Performance

- Changes to hot paths (reconstruction, segmentation, cell complex, MIP,
  inference) must run the benchmark suite before and after; report the delta
  in the PR (see §8.1 for the one-command workflow).
- A change may not regress a benchmark by more than 5 % without an explicit
  justification in the PR description.
- Prefer measuring to guessing: use `reusex::core::stopwatch` for coarse
  timing; profile before optimizing.

### 8.1 Baseline vs candidate workflow

`scripts/bench-compare.py` turns the "run before/after and eyeball it"
step into a one-command answer, matching benchmarks by test-case + benchmark
name and flagging any mean regression beyond `--threshold` percent (default
5, matching the threshold above):

```bash
git checkout main   && scripts/bench.sh              # -> bench-results/bench-<stamp>-<sha>.xml (baseline)
git checkout <branch> && scripts/bench.sh             # -> bench-results/bench-<stamp>-<sha>.xml (candidate)
scripts/bench-compare.py bench-results/bench-<baseline>.xml bench-results/bench-<candidate>.xml
```

The script prints a per-benchmark table (baseline mean, candidate mean,
delta %, both stddevs) and exits non-zero if any benchmark regressed beyond
the threshold — paste the table into the PR description as the "report the
delta" evidence required above. Benchmarks present in only one report are
reported as `ADDED`/`REMOVED` rather than causing a crash. Python 3 stdlib
only, no build step required.

## 9. Definition of Done

A change is done when ALL of the following hold:

1. Builds cleanly (`cmake --build build`) with no new warnings.
2. `ctest` passes, including integration tests.
3. New/changed behavior is covered by tests (§7).
4. Benchmarks are neutral or improved, or the regression is justified (§8).
5. `clang-format` clean; naming follows `CLAUDE.md`.
6. No new violations of the module boundaries (§1) or label contract (§3).
7. SPDX header present on new files; TODO comments follow the tdg format
   (see `CLAUDE.md`).
8. Pipeline-stage failures introduced or touched by the change are loud (§5).
9. **Docs touched by the change are updated in the same PR.** If the change adds,
   renames or removes a namespace, path, CMake option, `rux` command/flag,
   library target, dependency, or a stage's inputs/outputs, then the documents
   that name it are corrected — at minimum check `CLAUDE.md`, `README.md`,
   `ARCHITECTURE.md`, `docs/README.md`, `CONTRACTS.md` and this file. The bar is
   the one from #243: *no doc names a namespace, path, CMake option, command or
   dependency that `grep` cannot find in the source tree.* Historical design
   notes under `docs/design/` are exempt provided they say so at the top.

## 10. Process notes for AI agents

- Work from a GitHub issue; keep the change scoped to it. Split unrelated
  fixes into separate PRs.
- If a rule here conflicts with what existing code does, the rule wins for new
  code — but do not mass-migrate old code in the same PR unless the issue asks
  for it. File a follow-up issue instead.
- When a rule marked **[target]** blocks you (e.g. the label helpers don't
  exist yet), implement the smallest slice of the target that unblocks you,
  or flag it in the PR.
