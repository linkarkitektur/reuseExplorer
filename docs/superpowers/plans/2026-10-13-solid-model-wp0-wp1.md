<!--
SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen

SPDX-License-Identifier: GPL-3.0-or-later
-->

# Solid model WP0 + WP1 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Land the fixes and the measuring instruments the solid-model workstream rests on (WP0), then the levelled frame and the `storeys` table and stage (WP1), so WP2+ can be measured and built on stable interfaces.

**Architecture:** Every task reads and writes only `ProjectDB`; records are core PODs (`core/solid_model.hpp`), persisted by `ProjectDB`, carried to the JSON reference format by `io/solid_model_json.hpp`, scored by `reconstruction/solid_metrics.hpp`. The storey detector lives in `segmentation/level.hpp` and is wired as a normal pipeline stage (contract row, `JobStage`, CLI). The MIP learns to slice only at storey heights.

**Tech Stack:** C++20, PCL, Eigen, CGAL (Polygon_set_2, AABB tree), sqlite3 via `ProjectDB`, nlohmann_json, CLI11, Catch2 v3, Boost.Uuid (header-only), bash + python3 stdlib for scripts.

**Spec:** `docs/superpowers/specs/2026-10-13-solid-model-floor-plan-design.md` (sections cited per task). Algorithm sources: `docs/research/solid-model-scan-to-bim.md` §1.3(b).

**Beads:** one task = one bead under `reusex-z1v.2` (WP0) or `reusex-z1v.3` (WP1). Claim with `bd update <id> --claim`, close with `bd close <id> --reason="<metric or test evidence>"`. The task headings name the bead.

## Global Constraints

- Build and test from `nix develop`; `scripts/check.sh` must pass before any commit claims "done" (STANDARDS §9).
- Module layering is link-enforced (STANDARDS §1): `core` must not include PCL/CGAL; `reconstruction` must not include `segmentation` headers; header-only adapters in `geometry/` that include `core/` are compiled into consumers only (the `#227` pattern).
- Public headers include siblings as `<reusex/...>`; `#pragma once`; SPDX header on every new file: `// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen` / `// SPDX-License-Identifier: GPL-3.0-or-later`.
- Naming: snake_case functions, no `get_` prefix, PascalCase types, trailing `_` members, `Options` struct per algorithm is the single home of every tunable (STANDARDS §4).
- Determinism (STANDARDS §6): seeded RNGs, sorted iteration, lowest-id tie-breaks, grid origins anchored to the level frame origin.
- No silent degeneracy (STANDARDS §5): empty or single-row output logs the reason and the numbers at `warn` or above.
- Tests: Catch2 v3, synthetic data built in code, temp files through `tests/support/temp_path.hpp`; put tests in the light binary (`tests/unit/<module>/` except `vision`, `rux_app`, `visualize`, `gsplat/cuda`).
- Label contract (STANDARDS §3): `0` = unlabeled in every `CloudL`; after Task 3 semantic point labels are `api + 1`.
- Schema: `LATEST_SCHEMA_VERSION` in `libs/reusex/src/core/ProjectDB.cpp:410`; migrations use `CREATE TABLE IF NOT EXISTS` and a test that rolls a fresh project back and reopens it. **Version numbers in this plan assume Task 3 (v27) lands before Task 10 (v28); if not, take the next free number.**
- Commit after every task with the attribution line `Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>`; never push.

## Review Focus

1. **A project migrated twice** (v27 migration re-run, or a `labels` cloud already carrying `label_encoding`): labels must not shift twice. Pinned in Task 3 by `MigrationV27_AlreadyEncodedCloud_IsNotShiftedAgain`.
2. **A labels cloud holding an out-of-contract value** (`0xFFFFFFFF`, a wrapped −1 from before #214): the migration must leave it untouched and never overflow. Pinned in Task 3 by `MigrationV27_OutOfContractLabel_IsLeftAlone`.
3. **A scan with no ceiling** (handheld capture that never looked up): `detect_storeys` must still return one storey, with a low confidence and a warning, not zero rows and not a crash. Pinned in Task 9 by `DetectStoreys_NoCeiling_OneStoreyLowConfidence`.
4. **Stale storeys** (planes re-run after storeys): the mesh stage must not silently drop every horizontal plane when none matches a storey height. Pinned in Task 12 by `Mesh_StoreysMatchNoPlane_FallsBackToAllHorizontals`.
5. **A malformed reference plan** (wall of zero length, polygon with fewer than three vertices, unknown opening kind): `analyze solid` must reject it with the offending element named, not score garbage. Pinned in Task 5 by `SolidModelJson_DegenerateWall_Throws` and `SolidModelJson_PolygonUnderThreeVertices_Throws`.

---

## File structure

| File | Responsibility | Task |
|---|---|---|
| `apps/rux/include/create/mesh.hpp`, `apps/rux/src/create/mesh.cpp` | drop dead `-a/-l` | 1 |
| `docs/CONTRACTS.md`, `docs/guides/Process walk through.md` | truthful mesh row | 1 |
| `tests/unit/geometry/test_merge_planes.cpp`, `libs/reusex/src/geometry/utils.cpp` | opposite faces never merge | 2 |
| `libs/reusex/include/core/label_semantics.hpp` | `api + 1` rule | 3 |
| `libs/reusex/src/core/ProjectDB.cpp` | v27 migration | 3 |
| `libs/reusex/src/segmentation/reconstruct.cpp`, `libs/reusex/src/core/mask_selection.cpp`, `apps/rux/src/create/project.cpp` | writers stamp `label_encoding` | 3 |
| `libs/reusex/include/vision/project.hpp`, `libs/reusex/src/vision/project.cpp` | multi-view vote, options | 4 |
| `libs/reusex/include/core/solid_model.hpp` | all records + `SolidModel` | 5 |
| `libs/reusex/include/io/solid_model_json.hpp`, `libs/reusex/src/io/solid_model_json.cpp` | JSON ⇄ `SolidModel`, `load_solid_model(db)` | 5 |
| `docs/design/solid-model-reference.schema.json`, `docs/design/solid-model-json.md` | the format | 5 |
| `libs/reusex/include/reconstruction/solid_metrics.hpp`, `src/reconstruction/solid_metrics.cpp` | metrics | 6 |
| `apps/rux/include/analyze/solid.hpp`, `apps/rux/src/analyze/solid.cpp`, `apps/rux/src/analyze.cpp` | `rux analyze solid` | 6 |
| `scripts/bench-solid.sh`, `scripts/bench-solid-compare.py`, `scripts/bench-solid.manifest` | the bench | 6 |
| `libs/reusex/include/visualize/render_view.hpp`, `src/visualize/scene.cpp`, `apps/rux/src/render.cpp` | `--plan ref.json` overlay | 7 |
| `apps/blender/reusex_panel/plan_io.py` | Blender import/export of the plan JSON | 7 |
| `docs/guides/solid-model-reference-authoring.md`, `tests/fixtures/solid_model/newoffice.plan.json` | authoring loop + first reference | 7 |
| `libs/reusex/include/io/s3dis.hpp`, `src/io/s3dis.cpp`, `apps/rux/src/import/s3dis.cpp` | local S3DIS loader | 8 |
| `libs/reusex/include/segmentation/level.hpp`, `src/segmentation/level.cpp`, `include/geometry/level_frame.hpp` | storeys + level frame | 9 |
| `libs/reusex/include/core/guid.hpp`, `src/core/guid.cpp`, `ProjectDB.{hpp,cpp}` | `deterministic_guid`, storeys table (v28) | 10 |
| `stage_contract.{hpp,cpp}`, `pipeline/stages.{hpp,cpp}`, `stage_parameters.cpp`, `apps/rux/src/create/storeys.cpp`, `database/storey_router.cpp`, `qt/src/pipeline_ui.cpp`, `qt/src/core/cli_command.cpp` | stage wiring | 11 |
| `libs/reusex/include/reconstruction/mesh.hpp`, `src/reconstruction/mesh.cpp`, `src/pipeline/stages.cpp` (`run_mesh`) | storey-only sections | 12 |

---

### Task 1: Remove the dead `create mesh -a/-l` flags and fix the docs (bead `reusex-z1v.2.3`, W0.3)

**Files:**
- Modify: `apps/rux/include/create/mesh.hpp:36-41` (drop `grid_size`, `angle_threshold`, `distance_threshold` and the comment that excuses them)
- Modify: `apps/rux/src/create/mesh.cpp:52-61` (drop the two `add_option` calls) and the footer line `rux create mesh -a 15 -l 0.3` and the two NOTES lines about angle/distance
- Modify: `docs/CONTRACTS.md:182-191` (`### mesh` section)
- Modify: `docs/guides/Process walk through.md:319-330`
- Test: `tests/unit/rux_app/test_cli_command_roundtrip.cpp` (existing; must stay green), new case in `tests/unit/rux_app/test_create_mesh_flags.cpp`

**Interfaces:**
- Consumes: `SubcommandMeshOptions`, `setup_subcommand_create_mesh(CLI::App&, std::shared_ptr<RuxOptions>)`.
- Produces: nothing new; `rux create mesh --help` no longer lists `-a`/`-l`.

- [ ] **Step 1: Write the failing test**

Create `tests/unit/rux_app/test_create_mesh_flags.cpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// `rux create mesh -a/-l` were parsed but never reached MeshOptions (spec
// §2.1). A flag that does nothing is a lie in --help; they are gone.

#include <catch2/catch_test_macros.hpp>

#include <create.hpp>
#include <global-params.hpp>

#include <CLI/CLI.hpp>

#include <memory>
#include <string>
#include <vector>

TEST_CASE("CreateMesh_AngleAndDistanceFlags_AreUnknown", "[rux_app][cli]") {
  CLI::App app{"rux"};
  auto global = std::make_shared<RuxOptions>();
  setup_subcommand_create(app, global);

  std::vector<std::string> args = {"rux", "create", "mesh", "-a", "15"};
  std::vector<char *> argv;
  for (auto &a : args)
    argv.push_back(a.data());
  REQUIRE_THROWS_AS(app.parse(static_cast<int>(argv.size()), argv.data()),
                    CLI::ExtrasError);
}
```

- [ ] **Step 2: Run it to verify it fails**

Run: `scripts/check.sh -R CreateMesh_AngleAndDistanceFlags`
Expected: FAIL (`-a 15` parses today, so no exception is thrown).

- [ ] **Step 3: Remove the flags and the fields**

In `apps/rux/include/create/mesh.hpp` delete the three fields and replace the struct comment:

```cpp
/// Options of `rux create mesh`. Every default is derived from
/// reusex::geometry::MeshOptions (docs/STANDARDS.md §4); there is no CLI-only
/// tunable.
struct SubcommandMeshOptions {
  std::string output_mesh_name = "mesh"; ///< Mesh name in ProjectDB

  double search_threshold = reusex::geometry::MeshOptions{}.search_threshold;
  double new_plane_offset = reusex::geometry::MeshOptions{}.new_plane_offset;
  // ... unchanged from here
```

In `apps/rux/src/create/mesh.cpp` delete the `-a, --angle` and `-l, --distance` `add_option` blocks; in the footer replace `rux create mesh -a 15 -l 0.3         # Custom angle/distance` with `rux create mesh -t 0.8 -o 0.3         # Custom pairing search/offset` and delete the two NOTES lines starting `- Angle threshold` and `- Distance threshold`. Also change `"Generate watertight mesh from planes"` to `"Generate the MIP envelope mesh from planes"` and the first DESCRIPTION sentence to: `Builds a cell complex from the detected planes and solves a mixed-integer program (MIP) that selects the interior cells. The output is ONE envelope mesh of the selected volume; room ids and the solver's wall labels are not kept (see spec WP9).`

- [ ] **Step 4: Fix the two docs**

`docs/CONTRACTS.md` `### mesh` section becomes:

```markdown
### `mesh` (`rux create mesh`)

Solve the cell complex MIP and emit the selected volume as one envelope mesh.
The mesh is not partitioned per room and carries no wall labels (the solver
computes both and `mesh.cpp` discards them); the solid-model workstream's
`solids` stage is the typed replacement, and WP9 may make this stage keep
its labels.

| | |
|---|---|
| Consumes | `cloud`, `normals`, `rooms`, `planes`, `plane_centroids`, `plane_normals` |
| Produces | `mesh` (`meshes` table) |
| Options  | `MeshOptions` → `mesh()` (`libs/reusex/include/reconstruction/mesh.hpp`); `pipeline::mesh_options_from_parameters` reads them |
| Checks   | `cloud`/`normals`/`rooms`/`planes` present + aligned; `plane_centroids`/`plane_normals` present + aligned |
```

`docs/guides/Process walk through.md` Phase 15: replace the second code line with `rux -p building.rux create mesh --solver highs --time-limit 300   # force the CPU solver, longer budget`, and the output line with `*Output: one envelope mesh of the selected volume, saved to the project's `meshes` table (default name `mesh`). It is not split per room.*`

- [ ] **Step 5: Run the tests**

Run: `scripts/check.sh -R "CreateMesh|CliCommandRoundTrip|StageContract"`
Expected: PASS (the CONTRACTS mirror test parses the new row; the round-trip test never used `-a/-l`).

- [ ] **Step 6: Commit**

```bash
git add apps/rux/include/create/mesh.hpp apps/rux/src/create/mesh.cpp docs/CONTRACTS.md "docs/guides/Process walk through.md" tests/unit/rux_app/test_create_mesh_flags.cpp
git commit -m "rux: drop the dead create mesh -a/-l flags; docs say envelope mesh

Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>"
```

---

### Task 2: `merge_planes` must never merge the two faces of one wall (bead `reusex-z1v.2.4`, W0.4)

**Files:**
- Test: `tests/unit/geometry/test_merge_planes.cpp` (append)
- Modify: `libs/reusex/src/geometry/utils.cpp:221-247` (`plane_distance`) and `:329-335` (`mergeable`)

**Interfaces:**
- Consumes: `merge_planes(planes, inliers, centroids, cloud, angle_threshold = 0.1, distance_threshold = 0.5)`; helpers `add_x_wall_patch`, `centroid_of` already in the test file.
- Produces: unchanged signature; new behaviour: planes whose normals face each other (dot < 0) are never merged.

- [ ] **Step 1: Write the failing test**

Append to `tests/unit/geometry/test_merge_planes.cpp`:

```cpp
TEST_CASE("MergePlanes_TwoFacesOfOneWall_NeverMerge",
          "[geometry][merge_planes]") {
  // A 15 cm wall seen from both rooms: face A at x = 0 with normal -x
  // (pointing into room A), face B at x = 0.15 with normal +x. Today
  // plane_distance() treats +-n as equal and the 0.5 m offset tolerance
  // swallows the 0.15 m gap, so the faces merge and make_pairs() later
  // invents a back face 0.25 m away (spec §2.1, research §3).
  auto cloud = std::make_shared<Cloud>();
  auto ia = add_x_wall_patch(cloud, 0.0, 0.0, 4.0, 0.0, 2.5);
  auto ib = add_x_wall_patch(cloud, 0.15, 0.0, 4.0, 0.0, 2.5);
  cloud->width = static_cast<uint32_t>(cloud->size());
  cloud->height = 1;

  EigenVectorContainer<double, 4> planes{Eigen::Vector4d(-1, 0, 0, 0),
                                         Eigen::Vector4d(1, 0, 0, -0.15)};
  std::vector<IndicesPtr> inliers{ia, ib};
  EigenVectorContainer<double, 3> centroids{centroid_of(cloud, ia),
                                            centroid_of(cloud, ib)};

  auto [Pm, Im, Cm] = merge_planes(planes, inliers, centroids, cloud);

  REQUIRE(Pm.size() == 2);
  REQUIRE(Im[0]->size() == ia->size());
  REQUIRE(Im[1]->size() == ib->size());
}

TEST_CASE("MergePlanes_SplitWallSameNormalSmallOffset_StillMerges",
          "[geometry][merge_planes]") {
  // Guard: the fix must not break #215 — two patches of the SAME face whose
  // fitted offsets differ by 2 cm still merge.
  auto cloud = std::make_shared<Cloud>();
  auto ia = add_x_wall_patch(cloud, 0.0, 0.0, 2.0, 0.0, 2.5);
  auto ib = add_x_wall_patch(cloud, 0.02, 2.5, 4.5, 0.0, 2.5);
  cloud->width = static_cast<uint32_t>(cloud->size());
  cloud->height = 1;

  EigenVectorContainer<double, 4> planes{Eigen::Vector4d(1, 0, 0, 0),
                                         Eigen::Vector4d(1, 0, 0, -0.02)};
  std::vector<IndicesPtr> inliers{ia, ib};
  EigenVectorContainer<double, 3> centroids{centroid_of(cloud, ia),
                                            centroid_of(cloud, ib)};

  auto [Pm, Im, Cm] = merge_planes(planes, inliers, centroids, cloud);
  REQUIRE(Pm.size() == 1);
}
```

- [ ] **Step 2: Run it to verify it fails**

Run: `scripts/check.sh -R MergePlanes_TwoFacesOfOneWall`
Expected: FAIL with `Pm.size() == 1`.

- [ ] **Step 3: Make opposite-facing planes unmergeable**

In `libs/reusex/src/geometry/utils.cpp`, `plane_distance` (line 221): keep the unsigned angle for the *coplanarity* test but report orientation to the caller. Replace the function with:

```cpp
/// Pairwise plane distance used by the agglomerative clustering: the
/// normal-angle disagreement normalised by the angle tolerance, combined
/// with the offset difference along the average normal normalised by the
/// coplanarity tolerance. A value < 1 means "within tolerance on both axes".
///
/// Planes whose normals face each other (dot < 0) are the two faces of one
/// wall, never one surface seen twice: they get +infinity so they can never
/// merge, whatever their offset (spec §2.1; the two-face test in
/// test_merge_planes.cpp pins this).
double plane_distance(const Eigen::Vector4d &Pi, const Eigen::Vector3d &Ci,
                      const Eigen::Vector4d &Pj, const Eigen::Vector3d &Cj,
                      double angle_tol_rad, double coplanar_tol_m) {
  const Eigen::Vector3d ni = Pi.head<3>();
  const Eigen::Vector3d nj = Pj.head<3>();
  const double dot = std::clamp(ni.dot(nj), -1.0, 1.0);
  if (dot < 0.0)
    return std::numeric_limits<double>::infinity();
  const double angle = std::acos(dot);

  Eigen::Vector3d n_avg = ni + nj;
  const double n_avg_norm = n_avg.norm();
  double offset_diff;
  if (n_avg_norm < reusex::kEpsilonRelative) {
    offset_diff = std::numeric_limits<double>::infinity();
  } else {
    n_avg /= n_avg_norm;
    const Eigen::Vector3d midpoint = 0.5 * (Ci + Cj);
    const double di = n_avg.dot(midpoint - Ci);
    const double dj = n_avg.dot(midpoint - Cj);
    offset_diff = std::abs(di - dj);
  }

  const double a = angle / angle_tol_rad;
  const double o = offset_diff / coplanar_tol_m;
  return std::sqrt(a * a + o * o);
}
```

Update the doc comment on `merge_planes` in `libs/reusex/include/geometry/utils.hpp:102-110` by adding: `Opposite-facing planes (the two faces of a wall) never merge; make_pairs() pairs them instead.`

- [ ] **Step 4: Run the geometry tests**

Run: `scripts/check.sh -R "MergePlanes|merge_planes"`
Expected: PASS, including the four pre-existing cases.

- [ ] **Step 5: Measure the effect on the MIP (STANDARDS §8.1)**

Run, with a NewOffice copy (`cp ~/repos/NewOffice/project.rux /tmp/no.rux` after `rux create planes` has been run on it; copy the pre-change `rux` binary first if needed):

```bash
build/apps/rux/rux -v -p /tmp/no.rux create mesh 2>&1 | grep -E "Number of planes after merging|Number of plane pairs|cells|Solidification|saved"
```

Record "planes after merging", "plane pairs", cell count and solve time before and after in the bead notes (`bd update reusex-z1v.2.4 --notes "..."`). Once Task 6 lands, also record `rux analyze solid --mesh mesh` fit.

- [ ] **Step 6: Commit**

```bash
git add libs/reusex/src/geometry/utils.cpp libs/reusex/include/geometry/utils.hpp tests/unit/geometry/test_merge_planes.cpp
git commit -m "geometry: merge_planes never merges opposite-facing planes (two faces of one wall)

Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>"
```

---

### Task 3: Fix the label-0 collision: point label = API class + 1, with a v27 migration (bead `reusex-z1v.2.1`, W0.1; decision D8)

**Files:**
- Modify: `libs/reusex/include/core/label_semantics.hpp:79-103`
- Modify: `libs/reusex/src/core/ProjectDB.cpp` (`LATEST_SCHEMA_VERSION`, migration chain, new `migrateToV27()`)
- Modify: `libs/reusex/src/segmentation/reconstruct.cpp:505-508` and `:524-537`
- Modify: `apps/rux/src/create/project.cpp:95`
- Modify: `libs/reusex/src/core/mask_selection.cpp` (`provenance()`)
- Modify: `docs/STANDARDS.md` §3.1 table and rules
- Test: `tests/unit/core/test_label_semantics.cpp`, `tests/unit/vision/test_project_labels.cpp:148-212`, new `tests/unit/core/test_project_db_migration_v27.cpp`

**Interfaces:**
- Consumes: `api_to_point_label(int)`, `point_label_to_api(uint32_t)`, `ProjectDB::save_point_cloud(name, CloudL, stage, paramsJson)`, `point_cloud_provenance(name)`, `label_definitions(name)`, `save_label_definitions(name, map<int,string>)`.
- Produces: the constant `inline constexpr std::string_view kLabelEncodingKey = "label_encoding";` and `kLabelEncodingApiPlusOne = "api_plus_one";` in `label_semantics.hpp`; every writer of a semantic `CloudL` stamps `"label_encoding":"api_plus_one"` into the cloud's parameters JSON; `label_definitions` of a semantic cloud are keyed by **point label** (1-based).

- [ ] **Step 1: Write the failing unit tests**

In `tests/unit/core/test_label_semantics.cpp` replace the section `"valid class ids pass through"` with:

```cpp
  SECTION("API class ids shift by one so class 0 is never unlabeled") {
    REQUIRE(api_to_point_label(0) == 1u);
    REQUIRE(api_to_point_label(5) == 6u);
    REQUIRE(point_label_to_api(1u) == 0);
    REQUIRE(point_label_to_api(6u) == 5);
    REQUIRE(is_valid_label(api_to_point_label(0)));
  }

  SECTION("out-of-contract point labels do not round-trip") {
    REQUIRE_THROWS_AS(point_label_to_api(0xFFFFFFFFu), std::out_of_range);
  }
```

and keep the existing `"round-trip through api for valid labels"` section (it still holds). In `tests/unit/vision/test_project_labels.cpp` change line 166 to `REQUIRE(got == static_cast<uint32_t>(kPatchClass) + 1u);`, and in `Project_ClassIdZero_MapsToUnlabeledPointLabel` (line 198) rename it `Project_ClassIdZero_MapsToPointLabelOne` and assert `== 1u` where it asserted `== kUnlabeled`.

Create `tests/unit/core/test_project_db_migration_v27.cpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
// Schema v26 -> v27 (spec W0.1, decision D8): semantic label clouds written
// before v27 carried point label == API class, so class 0 collided with
// kUnlabeled. The migration rewrites every semantic label cloud by +1 and
// shifts its label_definitions keys, once, and marks the cloud with
// parameters.label_encoding = "api_plus_one".

#include <catch2/catch_test_macros.hpp>

#include <core/ProjectDB.hpp>
#include <core/label_semantics.hpp>

#include "../../support/temp_path.hpp"

#include <nlohmann/json.hpp>
#include <sqlite3.h>

#include <filesystem>
#include <map>
#include <string>

using reusex::ProjectDB;
namespace fs = std::filesystem;

namespace {
struct TempDB : reusex::test_support::TempPath {
  TempDB() : reusex::test_support::TempPath("test_migration_v27") {}
};

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

void roll_back_to_v26(const fs::path &path) {
  exec_raw(path, "DELETE FROM schema_version WHERE version >= 27;");
}

reusex::CloudL labels_of(std::initializer_list<uint32_t> values) {
  reusex::CloudL c;
  for (uint32_t v : values) {
    reusex::LabelT p;
    p.label = v;
    c.push_back(p);
  }
  c.width = static_cast<uint32_t>(c.size());
  c.height = 1;
  return c;
}

std::string encoding_of(const ProjectDB &db, const std::string &name) {
  const auto [stage, params] = db.point_cloud_provenance(name);
  if (params.empty())
    return "";
  const auto j = nlohmann::json::parse(params);
  return j.value(std::string(reusex::core::kLabelEncodingKey), "");
}
} // namespace

TEST_CASE("MigrationV27_OldSemanticCloud_ShiftsLabelsAndDefinitions",
          "[ProjectDB][migration]") {
  TempDB tmp;
  {
    ProjectDB db(tmp.path);
    // Pre-v27 encoding: class 0 ("wall") written as point label 0.
    db.save_point_cloud("labels", labels_of({0u, 0u, 1u, 2u, 0u}),
                        "project_labels", "{}");
    db.save_label_definitions("labels", {{0, "wall"}, {1, "floor"}, {2, "door"}});
    // A structural label cloud is NOT semantic and must not move.
    db.save_point_cloud("planes", labels_of({0u, 1u, 2u}), "segment_planes",
                        "{}");
  }
  roll_back_to_v26(tmp.path);
  ProjectDB db(tmp.path);
  CHECK(db.schema_version() == 27);

  auto labels = db.point_cloud_label("labels");
  REQUIRE(labels->size() == 5);
  CHECK((*labels)[0].label == 1u);
  CHECK((*labels)[2].label == 2u);
  CHECK((*labels)[3].label == 3u);
  CHECK(encoding_of(db, "labels") == reusex::core::kLabelEncodingApiPlusOne);

  const auto defs = db.label_definitions("labels");
  CHECK(defs == std::map<int, std::string>{{1, "wall"}, {2, "floor"}, {3, "door"}});

  auto planes = db.point_cloud_label("planes");
  CHECK((*planes)[0].label == 0u);
  CHECK((*planes)[1].label == 1u);
  CHECK(encoding_of(db, "planes") == "");
}

TEST_CASE("MigrationV27_AlreadyEncodedCloud_IsNotShiftedAgain",
          "[ProjectDB][migration]") {
  TempDB tmp;
  {
    ProjectDB db(tmp.path);
    db.save_point_cloud("labels", labels_of({1u, 2u}), "project_labels",
                        R"({"label_encoding":"api_plus_one"})");
  }
  roll_back_to_v26(tmp.path);
  ProjectDB db(tmp.path);
  auto labels = db.point_cloud_label("labels");
  CHECK((*labels)[0].label == 1u);
  CHECK((*labels)[1].label == 2u);
}

TEST_CASE("MigrationV27_OutOfContractLabel_IsLeftAlone",
          "[ProjectDB][migration]") {
  TempDB tmp;
  {
    ProjectDB db(tmp.path);
    db.save_point_cloud("labels", labels_of({0xFFFFFFFFu, 0u, 3u}),
                        "project_labels", "{}");
  }
  roll_back_to_v26(tmp.path);
  ProjectDB db(tmp.path);
  auto labels = db.point_cloud_label("labels");
  CHECK((*labels)[0].label == 0xFFFFFFFFu);
  CHECK((*labels)[1].label == 0u);
  CHECK((*labels)[2].label == 4u);
}
```

- [ ] **Step 2: Run them to verify they fail**

Run: `scripts/check.sh -R "ApiPointLabelConversion|MigrationV27|Project_PointsInsideLabelledSquare"`
Expected: FAIL (`api_to_point_label(0) == 0`; `kLabelEncodingKey` does not compile — comment the migration tests out for this run if the build must link; the point is the semantics test fails).

- [ ] **Step 3: Change the two scalar conversions**

In `libs/reusex/include/core/label_semantics.hpp` replace the two functions (lines 79-103) and add the constants:

```cpp
/// Key in a semantic label cloud's `parameters` JSON that records how its
/// point labels relate to API class ids. Absent = pre-v27 (identity).
inline constexpr std::string_view kLabelEncodingKey = "label_encoding";
/// Value of kLabelEncodingKey written by every semantic-label writer since
/// schema v27: point label = API class + 1.
inline constexpr std::string_view kLabelEncodingApiPlusOne = "api_plus_one";

/// API (CV_32S, -1 = background) → point label (uint32_t, 0 = unlabeled).
///
/// Background maps to `kUnlabeled`; class id `k` maps to `k + 1`, so the
/// first class (id 0, "wall" with the TensorRT defaults) is a valid point
/// label and never collides with unlabeled (STANDARDS §3.1; spec W0.1).
///
/// @throws std::out_of_range if @p api_label is negative but not the
/// background sentinel.
inline uint32_t api_to_point_label(int api_label) {
  if (api_label == kBackgroundApi)
    return kUnlabeled;
  if (api_label < 0)
    throw std::out_of_range("api_to_point_label: negative label " +
                            std::to_string(api_label) +
                            " is not the background sentinel (-1)");
  return static_cast<uint32_t>(api_label) + 1u;
}

/// Point label (uint32_t, 0 = unlabeled) → API (CV_32S, -1 = background).
/// Inverse of api_to_point_label(): `kUnlabeled` → `kBackgroundApi`,
/// `k + 1` → `k`.
///
/// @throws std::out_of_range for an out-of-contract label (a wrapped -1).
inline int point_label_to_api(uint32_t point_label) {
  if (point_label == kUnlabeled)
    return kBackgroundApi;
  if (is_out_of_contract_label(point_label))
    throw std::out_of_range("point_label_to_api: label " +
                            std::to_string(point_label) +
                            " is out of contract (a wrapped -1)");
  return static_cast<int>(point_label - 1u);
}
```

Add `#include <string_view>` to the header. Update the file-top table comment so the in-memory row reads `` `uint32_t` | `0` | `1..N+1` (= API class + 1) ``.

- [ ] **Step 4: Stamp every semantic writer**

`libs/reusex/src/segmentation/reconstruct.cpp:505-508`: append `,"label_encoding":"api_plus_one"` inside the `paramsJson` format string (it is one JSON object; add the pair before the closing `}}`). At lines 524-537 change `class_map[std::stoi(key)] = ...` to `class_map[static_cast<int>(core::api_to_point_label(std::stoi(key)))] = val.get<std::string>();`.

`apps/rux/src/create/project.cpp:95`: `db.save_point_cloud("labels", *labels, "project_labels", R"({"label_encoding":"api_plus_one"})");` (Task 4 replaces this literal with the options' JSON; keep the key.)

`libs/reusex/src/core/mask_selection.cpp`, in `provenance(...)`: when the cloud is created fresh, return `{"segment_resource", R"({"label_encoding":"api_plus_one"})"}` instead of an empty parameters string; when it exists, keep its parameters unchanged (they already carry the key after migration).

- [ ] **Step 5: Write the migration**

In `libs/reusex/src/core/ProjectDB.cpp`: set `LATEST_SCHEMA_VERSION = 27`; after `if (current < 26) { migrateToV26(); }` add `if (current < 27) { migrateToV27(); }`; add next to `migrateToV26()`:

```cpp
  /// A cloud is a semantic label cloud when it is the canonical "labels"
  /// cloud or was written by a semantic stage. Structural label clouds
  /// (planes, rooms, instances) keep their numbering.
  static bool isSemanticLabelCloud(const std::string &name,
                                   const std::string &stage) {
    return name == "labels" || stage == "project_labels" ||
           stage == "segment_resource";
  }

  void migrateToV27() {
    reusex::info("Migrating database to schema version 27");
    // Spec W0.1 / decision D8: point label = API class + 1. Every semantic
    // label cloud written before now used the identity, so class 0 was
    // indistinguishable from unlabeled. Shift each such cloud once, mark it,
    // and shift its label_definitions keys to match.
    struct Row {
      int id;
      std::string name, stage, params;
    };
    std::vector<Row> rows;
    {
      const char *sql = "SELECT id, name, COALESCE(stage,''), "
                        "COALESCE(parameters,'') FROM point_clouds "
                        "WHERE point_type = 'Label' ORDER BY id;";
      sqlite3_stmt *stmt;
      if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
        throw std::runtime_error("v27: failed to list label clouds");
      StmtGuard guard(stmt);
      while (sqlite3_step(stmt) == SQLITE_ROW)
        rows.push_back({sqlite3_column_int(stmt, 0), column_text(stmt, 1),
                        column_text(stmt, 2), column_text(stmt, 3)});
    }
    size_t shifted = 0;
    for (const auto &row : rows) {
      if (!isSemanticLabelCloud(row.name, row.stage))
        continue;
      nlohmann::json params = nlohmann::json::object();
      if (!row.params.empty()) {
        try {
          params = nlohmann::json::parse(row.params);
        } catch (const std::exception &) {
          params = nlohmann::json::object();
        }
        if (!params.is_object())
          params = nlohmann::json::object();
      }
      if (params.value(std::string(core::kLabelEncodingKey), "") ==
          core::kLabelEncodingApiPlusOne)
        continue; // already encoded: never shift twice

      auto meta = loadCloudRaw(row.name);
      auto cloud = deserializeLabel(meta.data.data(), meta.data.size(),
                                    meta.width, meta.height);
      for (auto &p : cloud->points)
        if (p.label != core::kUnlabeled &&
            !core::is_out_of_contract_label(p.label))
          p.label += 1u;
      params[std::string(core::kLabelEncodingKey)] =
          std::string(core::kLabelEncodingApiPlusOne);
      // Rewrite chunks + parameters in place; stage, created_at and
      // tile_index are untouched (savePointCloudMeta would reset them).
      const auto data = serializeLabel(*cloud);
      replaceCloudChunks(row.id, data);
      {
        const char *sql =
            "UPDATE point_clouds SET parameters = ? WHERE id = ?;";
        sqlite3_stmt *stmt;
        if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
          throw std::runtime_error("v27: failed to prepare parameters update");
        StmtGuard guard(stmt);
        const std::string dumped = params.dump();
        sqlite3_bind_text(stmt, 1, dumped.c_str(), -1, SQLITE_TRANSIENT);
        sqlite3_bind_int(stmt, 2, row.id);
        if (sqlite3_step(stmt) != SQLITE_DONE)
          throw std::runtime_error("v27: failed to update parameters");
      }
      // label_definitions: key k -> k + 1, in one statement (descending so
      // the PRIMARY KEY (cloud_id, label_id) never collides mid-update).
      {
        const char *sql = "UPDATE label_definitions SET label_id = label_id "
                          "+ 1 WHERE cloud_id = ? AND label_id >= 0 "
                          "ORDER BY label_id DESC;";
        sqlite3_stmt *stmt;
        if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK) {
          // SQLite without UPDATE ... ORDER BY support: do it in two hops.
          execOrThrow(("UPDATE label_definitions SET label_id = -label_id - 2 "
                       "WHERE cloud_id = " + std::to_string(row.id) +
                       " AND label_id >= 0;")
                          .c_str());
          execOrThrow(("UPDATE label_definitions SET label_id = -label_id - 1 "
                       "WHERE cloud_id = " + std::to_string(row.id) +
                       " AND label_id < -1;")
                          .c_str());
        } else {
          StmtGuard guard(stmt);
          sqlite3_bind_int(stmt, 1, row.id);
          if (sqlite3_step(stmt) != SQLITE_DONE)
            throw std::runtime_error("v27: failed to shift label definitions");
        }
      }
      ++shifted;
      reusex::info("v27: shifted semantic label cloud '{}' ({} points)",
                   row.name, cloud->size());
    }
    insertSchemaVersion(27, "Point label = API class + 1 for semantic label "
                            "clouds (label-0 collision, spec W0.1)");
    reusex::info("Migration to schema version 27 complete ({} cloud(s) shifted)",
                 shifted);
  }
```

Add the small helper next to `savePointCloudMeta` (the chunk loop is the one there, lines 3675-3745; factor it out so both call it):

```cpp
  /// Replace every point_cloud_data chunk of @p cloudId with @p data, split
  /// at POINT_CLOUD_CHUNK_BYTES (at least one row, possibly zero-length).
  void replaceCloudChunks(int cloudId, const std::vector<uint8_t> &data) {
    {
      sqlite3_stmt *stmt;
      if (sqlite3_prepare_v2(db,
                             "DELETE FROM point_cloud_data WHERE cloud_id = ?;",
                             -1, &stmt, nullptr) != SQLITE_OK)
        throw std::runtime_error("Failed to prepare chunk delete");
      StmtGuard guard(stmt);
      sqlite3_bind_int(stmt, 1, cloudId);
      if (sqlite3_step(stmt) != SQLITE_DONE)
        throw std::runtime_error("Failed to delete old chunks");
    }
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db,
                           "INSERT INTO point_cloud_data (cloud_id, "
                           "chunk_index, data) VALUES (?, ?, ?);",
                           -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to prepare chunk insert");
    StmtGuard guard(stmt);
    const size_t total = data.size();
    size_t offset = 0;
    int chunkIndex = 0;
    do {
      const size_t len = std::min(POINT_CLOUD_CHUNK_BYTES, total - offset);
      sqlite3_bind_int(stmt, 1, cloudId);
      sqlite3_bind_int(stmt, 2, chunkIndex);
      sqlite3_bind_blob(stmt, 3, data.data() + offset, static_cast<int>(len),
                        SQLITE_STATIC);
      if (sqlite3_step(stmt) != SQLITE_DONE)
        throw std::runtime_error("Failed to insert chunk");
      sqlite3_reset(stmt);
      offset += len;
      ++chunkIndex;
    } while (offset < total);
  }
```

The whole migration runs inside the migration chain's existing transaction. Use the two-hop variant unconditionally if the sqlite build in the flake lacks `SQLITE_ENABLE_UPDATE_DELETE_LIMIT` (the prepare fails at runtime; the fallback handles it). Read `column_text` — it already exists as a file-local helper (used in `point_cloud_provenance`).

- [ ] **Step 6: Audit every reader**

Grep and fix any inline arithmetic on *semantic* labels; the fact sheet found none in `segment_instances.cpp`, `create_windows.cpp`, `survey_service.cpp`, `project.cpp`, `reconstruct.cpp`, `mask_selection.cpp`, `bindings.cpp`. Re-run the grep to confirm:

```bash
grep -rn "label *[-+] *1\b\|\.label *- *1\|\.label *+ *1" libs/reusex/src apps/rux/src bindings/python/src | grep -v "palette_slot\|planes\|rooms\|instances"
```

Expected: no semantic hit. `visualize/scene.cpp:576` (`label_palette_slot`) is palette math over point labels and stays. `rux get labels` prints `label_definitions` as stored (now point-label keyed) — correct by construction.

- [ ] **Step 7: Update STANDARDS §3.1**

In `docs/STANDARDS.md` change the in-memory row to `` | In-memory point labels (`pcl::Label`) | `uint32_t` | `0` | `1..N+1` — semantic clouds store **API class + 1** (`label_encoding = api_plus_one` in the cloud's parameters since schema v27) | `` and add a rule bullet: `- `label_definitions` of a semantic cloud are keyed by **point label** (1-based), never by API class id.`

- [ ] **Step 8: Run the whole light suite and the vision test**

Run: `scripts/check.sh -R "label_semantics|MigrationV2|Project_|mask_selection|survey|instances"` then `scripts/check.sh`
Expected: PASS.

- [ ] **Step 9: Integration check on a real project**

On a copy of a project that has `segmentation_images` (NewOffice): `rux -p /tmp/no.rux create project && rux -p /tmp/no.rux create instances && rux -p /tmp/no.rux get instances --json | head`. Expected: an instance whose `semantic_class` is `1` (the first prompt class) now exists. Record the count in the bead notes.

- [ ] **Step 10: Commit**

```bash
git add libs/reusex/include/core/label_semantics.hpp libs/reusex/src/core/ProjectDB.cpp libs/reusex/src/segmentation/reconstruct.cpp apps/rux/src/create/project.cpp libs/reusex/src/core/mask_selection.cpp docs/STANDARDS.md tests/unit/core/test_label_semantics.cpp tests/unit/core/test_project_db_migration_v27.cpp tests/unit/vision/test_project_labels.cpp
git commit -m "core: point label = API class + 1; v27 migrates semantic label clouds (label-0 collision)

Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>"
```

Also note for the `ruxd` repo (file an issue there from the bead): `label_definitions` keys are now point labels; a legend that adds 1 itself would be off by one.

---

### Task 4: `create project`: multi-view majority vote, deterministic ties, parameters (bead `reusex-z1v.2.2`, W0.2)

**Files:**
- Modify: `libs/reusex/include/vision/project.hpp`
- Modify: `libs/reusex/src/vision/project.cpp:95-330`
- Modify: `apps/rux/include/create/project.hpp`, `apps/rux/src/create/project.cpp`
- Modify: `libs/reusex/src/segmentation/reconstruct.cpp:1158-1178` (voxel vote)
- Test: `tests/unit/vision/test_project_labels.cpp` (append), `tests/unit/core/test_label_semantics.cpp` (append the `majority_label` cases)

**Interfaces:**
- Produces in `vision/project.hpp`:

```cpp
struct ProjectLabelsOptions {
  float max_depth_m = 7.0f;        ///< points beyond this never take a label
  float depth_tolerance_m = 0.15f; ///< z-buffer agreement for a visible point
};
auto project(const ProjectDB &db, CloudConstPtr cloud,
             const ProjectLabelsOptions &options = {}) -> CloudLPtr;
std::string project_labels_parameters(const ProjectLabelsOptions &options); // JSON incl. label_encoding
```

- Produces in `core/label_semantics.hpp` (namespace `reusex::core`): `uint32_t majority_label(const std::vector<std::pair<uint32_t, int>> &counts)` — ignores `kUnlabeled`, lowest label wins ties, returns `kUnlabeled` when nothing else voted. It lives in `core` because both `vision` and `segmentation` need it and peers must not include each other (STANDARDS §1).

- [ ] **Step 1: Write the failing tests**

Append to `tests/unit/vision/test_project_labels.cpp` (reuse `make_label_image`, `make_wall`, `make_intrinsics`, `identity4`, `pixel_of`, `inside_patch`):

```cpp
TEST_CASE("Project_TwoFramesDisagree_MajorityWinsAndBackgroundNeverOverwrites",
          "[vision][project]") {
  // Frame 1 and 2 label the square as class 7, frame 3 as class 2, frame 4
  // sees background there. Majority: 7. Background must not erase it.
  auto db = std::make_shared<ProjectDB>(":memory:");
  cv::Mat color(kHeight, kWidth, CV_8UC3, cv::Scalar(0, 0, 0));
  for (int id = 1; id <= 4; ++id)
    db->save_sensor_frame(id, color, cv::Mat(), cv::Mat(), identity4(),
                          make_intrinsics());
  cv::Mat seven = make_label_image();
  cv::Mat two = make_label_image();
  two.setTo(2, two == kPatchClass);
  cv::Mat bg(kHeight, kWidth, CV_32S, cv::Scalar(reusex::core::kBackgroundApi));
  db->save_segmentation_image(1, seven);
  db->save_segmentation_image(2, seven);
  db->save_segmentation_image(3, two);
  db->save_segmentation_image(4, bg);

  auto cloud = make_wall();
  auto labels = reusex::vision::project(*db, cloud);
  const uint32_t expected = reusex::core::api_to_point_label(kPatchClass);
  for (size_t i = 0; i < cloud->size(); ++i) {
    const auto [px, py] = pixel_of((*cloud)[i]);
    if (inside_patch(px, py))
      REQUIRE((*labels)[i].label == expected);
  }
}

TEST_CASE("Project_TieBetweenClasses_LowestClassWins", "[vision][project]") {
  auto db = std::make_shared<ProjectDB>(":memory:");
  cv::Mat color(kHeight, kWidth, CV_8UC3, cv::Scalar(0, 0, 0));
  for (int id = 1; id <= 2; ++id)
    db->save_sensor_frame(id, color, cv::Mat(), cv::Mat(), identity4(),
                          make_intrinsics());
  cv::Mat seven = make_label_image();
  cv::Mat two = make_label_image();
  two.setTo(2, two == kPatchClass);
  db->save_segmentation_image(1, seven);
  db->save_segmentation_image(2, two);
  auto cloud = make_wall();
  auto a = reusex::vision::project(*db, cloud);
  auto b = reusex::vision::project(*db, cloud);
  for (size_t i = 0; i < cloud->size(); ++i) {
    const auto [px, py] = pixel_of((*cloud)[i]);
    if (inside_patch(px, py))
      REQUIRE((*a)[i].label == reusex::core::api_to_point_label(2));
    REQUIRE((*a)[i].label == (*b)[i].label); // bit-identical across runs
  }
}

TEST_CASE("Project_MaxDepthOption_IsHonoured", "[vision][project]") {
  auto db = make_project(make_label_image());
  auto cloud = make_wall(); // at kDepth = 2.0 m
  reusex::vision::ProjectLabelsOptions opt;
  opt.max_depth_m = 1.0f;
  auto labels = reusex::vision::project(*db, cloud, opt);
  for (const auto &p : labels->points)
    REQUIRE(p.label == reusex::core::kUnlabeled);
}
```

Append to `tests/unit/core/test_label_semantics.cpp`:

```cpp
TEST_CASE("MajorityLabel_UnlabeledVotesAreIgnored", "[label_semantics][vote]") {
  REQUIRE(majority_label({{kUnlabeled, 10}, {3u, 1}}) == 3u);
}

TEST_CASE("MajorityLabel_TieGoesToLowestLabel", "[label_semantics][vote]") {
  REQUIRE(majority_label({{5u, 2}, {2u, 2}, {9u, 1}}) == 2u);
}

TEST_CASE("MajorityLabel_OnlyUnlabeled_ReturnsUnlabeled",
          "[label_semantics][vote]") {
  REQUIRE(majority_label({{kUnlabeled, 4}}) == kUnlabeled);
  REQUIRE(majority_label({}) == kUnlabeled);
}
```

(add `#include <utility>` and `#include <vector>` to that test file.)

- [ ] **Step 2: Run to verify they fail**

Run: `scripts/check.sh -R "Project_TwoFramesDisagree|Project_TieBetween|Project_MaxDepth|MajorityLabel"`
Expected: FAIL / does not compile (missing function and options).

- [ ] **Step 3: Add `majority_label` to `core/label_semantics.hpp`**

Append inside `namespace reusex::core` (add `#include <utility>` and `#include <vector>`):

```cpp
/// The majority point label of @p counts (label, votes), with the rules every
/// label fusion in the pipeline shares (STANDARDS §6, spec W0.2):
///   - `kUnlabeled` votes are ignored: background never outvotes a class;
///   - ties go to the LOWEST label, so a run is bit-reproducible whatever the
///     order the votes arrived in;
///   - no class votes at all -> `kUnlabeled`.
inline uint32_t
majority_label(const std::vector<std::pair<uint32_t, int>> &counts) {
  uint32_t best = kUnlabeled;
  int best_count = 0;
  for (const auto &[label, count] : counts) {
    if (label == kUnlabeled || count <= 0)
      continue;
    if (count > best_count || (count == best_count && label < best)) {
      best = label;
      best_count = count;
    }
  }
  return best;
}
```

- [ ] **Step 4: Rewrite the projection loop as a vote**

In `libs/reusex/include/vision/project.hpp` add the options struct and the two declarations from the Interfaces block (keep the old single-argument call working through the default). In `libs/reusex/src/vision/project.cpp`:

1. Before the frame loop allocate the vote store:

```cpp
  // Per point, up to kSlots (label, count) pairs. Exact when a point sees at
  // most kSlots distinct classes across all frames (the normal case); a fifth
  // class replaces the weakest slot. Background votes are not counted at all
  // (spec W0.2: background never overwrites a class).
  constexpr int kSlots = 4;
  struct Slot {
    uint32_t label = core::kUnlabeled;
    uint16_t count = 0;
  };
  std::vector<std::array<Slot, kSlots>> votes(cloud->size());
  auto cast_vote = [](std::array<Slot, kSlots> &slots, uint32_t label) {
    for (auto &s : slots)
      if (s.label == label) {
        if (s.count < 0xFFFF)
          ++s.count;
        return;
      }
    for (auto &s : slots)
      if (s.count == 0) {
        s.label = label;
        s.count = 1;
        return;
      }
    auto weakest = std::min_element(
        slots.begin(), slots.end(),
        [](const Slot &a, const Slot &b) { return a.count < b.count; });
    weakest->label = label;
    weakest->count = 1;
  };
```

2. Replace the two hard-coded numbers: `7.0f` → `options.max_depth_m`, `0.15f` → `options.depth_tolerance_m` (delete the TODO above them).

3. Replace the assignment at line 294-296 with:

```cpp
          const uint32_t label =
              reusex::core::api_to_point_label(labeledImage.at<int>(py, px));
          if (label != core::kUnlabeled)
            cast_vote(votes[idx], label); // idx is unique per iteration
```

4. After the frame loop, resolve:

```cpp
  std::vector<std::pair<uint32_t, int>> counts;
  counts.reserve(kSlots);
  size_t labelled = 0;
  for (size_t i = 0; i < votes.size(); ++i) {
    counts.clear();
    for (const auto &s : votes[i])
      if (s.count > 0)
        counts.emplace_back(s.label, static_cast<int>(s.count));
    const uint32_t label = segmentation::majority_label(counts);
    labels->points[i].label = label;
    labelled += label != core::kUnlabeled;
  }
```

and include `labelled` in the final `info` line (`core::majority_label` from Step 3).

5. Add `project_labels_parameters()`:

```cpp
std::string project_labels_parameters(const ProjectLabelsOptions &options) {
  nlohmann::json j;
  j["max_depth_m"] = options.max_depth_m;
  j["depth_tolerance_m"] = options.depth_tolerance_m;
  j[std::string(core::kLabelEncodingKey)] =
      std::string(core::kLabelEncodingApiPlusOne);
  return j.dump();
}
```

6. In `libs/reusex/src/segmentation/reconstruct.cpp:1164-1177` replace the `unordered_map` vote with a `std::vector<std::pair<uint32_t,int>>` built by linear search (the neighbourhood is tiny) and `core::majority_label(counts)`.

7. CLI: `SubcommandProjectOptions` gains `float max_depth = reusex::vision::ProjectLabelsOptions{}.max_depth_m; float depth_tolerance = reusex::vision::ProjectLabelsOptions{}.depth_tolerance_m;`, flags `--max-depth` (`CLI::Range(0.1, 100.0)`) and `--depth-tolerance` (`CLI::Range(0.001, 1.0)`), and the save becomes `db.save_point_cloud("labels", *labels, "project_labels", reusex::vision::project_labels_parameters(opts));` with `log_pipeline_start("project_labels", <same JSON>)`.

- [ ] **Step 5: Run the tests**

Run: `scripts/check.sh -R "Project_|MajorityLabel|label_semantics"`
Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add libs/reusex/include/vision/project.hpp libs/reusex/src/vision/project.cpp libs/reusex/include/core/label_semantics.hpp libs/reusex/src/segmentation/reconstruct.cpp apps/rux/include/create/project.hpp apps/rux/src/create/project.cpp tests/unit/vision/test_project_labels.cpp tests/unit/core/test_label_semantics.cpp
git commit -m "vision: create project votes across frames; background never overwrites; depth cutoffs are options

Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>"
```

---

### Task 5: Solid-model records and the JSON reference format (bead `reusex-z1v.2.5`, W0.5 part 1)

**Files:**
- Create: `libs/reusex/include/core/solid_model.hpp`
- Create: `libs/reusex/include/io/solid_model_json.hpp`, `libs/reusex/src/io/solid_model_json.cpp`
- Create: `docs/design/solid-model-reference.schema.json`, `docs/design/solid-model-json.md`
- Test: `tests/unit/io/test_solid_model_json.cpp`

**Interfaces:**
- Produces (`core/solid_model.hpp`, namespace `reusex::core`): the records below and `struct SolidModel`.
- Produces (`io/solid_model_json.hpp`, namespace `reusex::io`):

```cpp
inline constexpr int kSolidModelJsonVersion = 1;
std::string solid_model_to_json(const core::SolidModel &model, int indent = 2);
core::SolidModel solid_model_from_json(std::string_view text);   // validates, throws std::invalid_argument naming the element
core::SolidModel load_solid_model(const ProjectDB &db);          // WP0: empty; Task 10 fills storeys
```

- [ ] **Step 1: Write the failing test**

Create `tests/unit/io/test_solid_model_json.cpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
// The one JSON format for authored references (W0.6), detected models
// (rux get model --json, W7.2) and the bench (W0.5). Spec §4 W0.5, §3.2.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <reusex/core/solid_model.hpp>
#include <reusex/io/solid_model_json.hpp>

#include <stdexcept>
#include <string>

using reusex::core::OpeningRecord;
using reusex::core::SolidModel;
using reusex::core::SpaceRecord;
using reusex::core::StoreyRecord;
using reusex::core::WallRecord;
using reusex::io::solid_model_from_json;
using reusex::io::solid_model_to_json;

namespace {
SolidModel two_wall_model() {
  SolidModel m;
  StoreyRecord s;
  s.guid = "st-0";
  s.idx = 0;
  s.z_floor = 0.0;
  s.z_ceiling = 2.6;
  m.storeys.push_back(s);
  WallRecord w;
  w.guid = "w-1";
  w.storey_idx = 0;
  w.x0 = 0; w.y0 = 0; w.x1 = 4; w.y1 = 0;
  w.z_base = 0; w.height = 2.6; w.thickness = 0.12;
  m.walls.push_back(w);
  w.guid = "w-2";
  w.x0 = 4; w.y0 = 0; w.x1 = 4; w.y1 = 3;
  m.walls.push_back(w);
  SpaceRecord sp;
  sp.guid = "sp-1";
  sp.storey_idx = 0;
  sp.outer = {{0, 0}, {4, 0}, {4, 3}, {0, 3}};
  sp.area = 12.0;
  m.spaces.push_back(sp);
  OpeningRecord o;
  o.guid = "op-1";
  o.wall_guid = "w-1";
  o.kind = "door";
  o.s_start = 1.0; o.s_end = 1.9; o.z_sill = 0.0; o.z_head = 2.1;
  m.openings.push_back(o);
  return m;
}
} // namespace

TEST_CASE("SolidModelJson_RoundTrip_IsLossless", "[io][solid_model]") {
  const auto text = solid_model_to_json(two_wall_model());
  const auto back = solid_model_from_json(text);
  REQUIRE(back.storeys.size() == 1);
  REQUIRE(back.walls.size() == 2);
  REQUIRE(back.spaces.size() == 1);
  REQUIRE(back.openings.size() == 1);
  CHECK(back.walls[0].thickness == Catch::Approx(0.12));
  CHECK(back.spaces[0].outer.size() == 4);
  CHECK(back.openings[0].wall_guid == "w-1");
  CHECK(solid_model_to_json(back) == text);
}

TEST_CASE("SolidModelJson_DegenerateWall_Throws", "[io][solid_model]") {
  auto m = two_wall_model();
  m.walls[1].x1 = m.walls[1].x0;
  m.walls[1].y1 = m.walls[1].y0;
  REQUIRE_THROWS_WITH(solid_model_from_json(solid_model_to_json(m)),
                      Catch::Matchers::ContainsSubstring("w-2"));
}

TEST_CASE("SolidModelJson_PolygonUnderThreeVertices_Throws",
          "[io][solid_model]") {
  auto m = two_wall_model();
  m.spaces[0].outer = {{0, 0}, {1, 0}};
  REQUIRE_THROWS_WITH(solid_model_from_json(solid_model_to_json(m)),
                      Catch::Matchers::ContainsSubstring("sp-1"));
}

TEST_CASE("SolidModelJson_UnknownOpeningKindOrWall_Throws",
          "[io][solid_model]") {
  auto m = two_wall_model();
  m.openings[0].kind = "hatch";
  REQUIRE_THROWS_AS(solid_model_from_json(solid_model_to_json(m)),
                    std::invalid_argument);
  m = two_wall_model();
  m.openings[0].wall_guid = "nope";
  REQUIRE_THROWS_AS(solid_model_from_json(solid_model_to_json(m)),
                    std::invalid_argument);
}

TEST_CASE("SolidModelJson_WrongVersion_Throws", "[io][solid_model]") {
  REQUIRE_THROWS_AS(solid_model_from_json(R"({"model_version":99})"),
                    std::invalid_argument);
}
```

Add `#include <catch2/matchers/catch_matchers_string.hpp>` at the top.

- [ ] **Step 2: Run to verify it fails**

Run: `scripts/check.sh -R SolidModelJson`
Expected: does not compile (headers missing).

- [ ] **Step 3: Write the records**

Create `libs/reusex/include/core/solid_model.hpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// The typed building model (spec §3.2): plain records mirroring the
// ProjectDB tables, with no Eigen/CGAL/PCL types so `core` stays free of
// geometry (the #227 ComponentRecord pattern). Cross references are GUIDs,
// not row ids, so the same records describe an authored reference (no rows)
// and a detected model. Coordinates are in the storey's level frame
// (spec §3.3), metres, z up, floor at z = 0.

#include <array>
#include <string>
#include <vector>

namespace reusex::core {

/// A 2D ring, counter-clockwise for outer rings, in the level frame.
using Ring2 = std::vector<std::array<double, 2>>;

inline constexpr std::array<double, 16> kIdentity16{1, 0, 0, 0, 0, 1, 0, 0,
                                                    0, 0, 1, 0, 0, 0, 0, 1};

/// Provenance shared by every typed record.
struct Provenance {
  std::string source = "detected"; ///< detected | manual | imported
  bool locked = false;              ///< a re-run must not overwrite this row
  double confidence = -1.0;         ///< 0..1, -1 unknown
  std::string metadata;             ///< JSON for evidence without a column
};

struct StoreyRecord {
  int id = -1;
  std::string guid;
  int idx = 0; ///< 0 = lowest
  std::string name;
  double z_floor = 0.0;   ///< level frame of storey 0
  double z_ceiling = 0.0; ///< level frame of storey 0
  std::array<double, 4> floor_plane{0, 0, 0, 0};   ///< world, (a,b,c,d)
  std::array<double, 4> ceiling_plane{0, 0, 0, 0}; ///< world, (a,b,c,d)
  std::array<double, 16> level_transform = kIdentity16; ///< world -> level, row-major
  Provenance prov;
};

struct WallRecord {
  int id = -1;
  std::string guid;
  int storey_idx = 0;
  double x0 = 0, y0 = 0, x1 = 0, y1 = 0; ///< axis (centre line), level frame
  double z_base = 0.0, height = 0.0;
  double thickness = -1.0, thickness_ci_lo = -1.0, thickness_ci_hi = -1.0;
  double lean_deg = 0.0;
  int is_external = -1; ///< 1 external, 0 internal, -1 unknown
  bool assumed = false; ///< one-sided: thickness defaulted
  double seen_share = -1.0;
  int face_a_points = 0, face_b_points = 0;
  double co_visibility = -1.0;
  int axis_family = -1;
  Provenance prov;
};

struct WallJoinRecord {
  std::string wall_a, wall_b; ///< GUIDs
  std::string kind;           ///< corner | tee | collinear
  std::string at_a, at_b;     ///< start | end | path
};

struct SpaceRecord {
  int id = -1;
  std::string guid;
  int storey_idx = 0;
  std::string name, kind; ///< kind: room | corridor | open_plan | unknown
  Ring2 outer;
  std::vector<Ring2> holes;
  double z_floor = 0.0, z_ceiling = 0.0;
  double area = 0.0, perimeter = 0.0, volume = 0.0, compactness = 0.0;
  int room_label = -1; ///< link to the "rooms" label cloud
  Provenance prov;
};

struct SpaceBoundaryRecord {
  std::string space_guid, wall_guid;
  std::string side; ///< a | b
  double s_start = 0.0, s_end = 0.0;
};

struct OpeningRecord {
  int id = -1;
  std::string guid;
  std::string wall_guid;
  std::string kind; ///< door | window | opening
  double s_start = 0.0, s_end = 0.0, z_sill = 0.0, z_head = 0.0;
  std::string state; ///< open | closed | unknown (doors)
  int is_external = -1;
  std::string source_instance_guid;
  Provenance prov;
};

/// The whole typed model of one project (or one authored reference).
struct SolidModel {
  std::vector<StoreyRecord> storeys;
  std::vector<WallRecord> walls;
  std::vector<WallJoinRecord> joins;
  std::vector<SpaceRecord> spaces;
  std::vector<SpaceBoundaryRecord> boundaries;
  std::vector<OpeningRecord> openings;

  bool empty() const {
    return storeys.empty() && walls.empty() && spaces.empty() &&
           openings.empty();
  }
};

/// Length of a wall axis, metres.
inline double wall_length(const WallRecord &w) {
  const double dx = w.x1 - w.x0, dy = w.y1 - w.y0;
  return std::sqrt(dx * dx + dy * dy);
}

} // namespace reusex::core
```

Add `#include <cmath>`.

- [ ] **Step 4: Write the JSON codec**

Create `libs/reusex/include/io/solid_model_json.hpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// One JSON document for the typed building model (spec §4 W0.5 / W7.2):
// authored references, `rux get model --json`, the bench, ruxd and the
// Blender add-on all read and write this. Schema:
// docs/design/solid-model-reference.schema.json; prose:
// docs/design/solid-model-json.md.

#include "reusex/core/solid_model.hpp"

#include <string>
#include <string_view>

namespace reusex {
class ProjectDB;
}

namespace reusex::io {

inline constexpr int kSolidModelJsonVersion = 1;

std::string solid_model_to_json(const core::SolidModel &model, int indent = 2);

/// Parse and validate. @throws std::invalid_argument naming the offending
/// element: wrong version, wall shorter than 1 mm, ring with < 3 vertices,
/// opening with an unknown kind or a wall_guid that is not in the document,
/// duplicate GUIDs.
core::SolidModel solid_model_from_json(std::string_view text);

/// The detected model stored in @p db. Tables that do not exist in this
/// schema version read as empty vectors, so a report can say "n/a".
core::SolidModel load_solid_model(const ProjectDB &db);

} // namespace reusex::io
```

Create `libs/reusex/src/io/solid_model_json.cpp` with nlohmann_json: `to_json`/`from_json` free functions per record (fields named exactly as the struct members; `prov` flattened as `source`, `locked`, `confidence`, `metadata`; `outer`/`holes` as arrays of `[x, y]`), a top-level object `{"model_version": 1, "storeys": [...], "walls": [...], "joins": [...], "spaces": [...], "boundaries": [...], "openings": [...]}`, and a `validate(const SolidModel&)` that throws `std::invalid_argument("solid model: wall '<guid>' has zero length")`, `"... space '<guid>' outer ring has <n> vertices"`, `"... opening '<guid>' kind '<k>' is not door|window|opening"`, `"... opening '<guid>' references unknown wall '<g>'"`, `"... duplicate guid '<g>'"`. `load_solid_model(db)` returns `{}` in this task (Task 10 fills `storeys`). Write `docs/design/solid-model-reference.schema.json` (JSON Schema draft 2020-12 for the document, every field with its unit in `description`) and `docs/design/solid-model-json.md` (one paragraph per record, the level-frame convention, the version rule: additive fields never bump the version).

- [ ] **Step 5: Run the tests**

Run: `scripts/check.sh -R SolidModelJson`
Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add libs/reusex/include/core/solid_model.hpp libs/reusex/include/io/solid_model_json.hpp libs/reusex/src/io/solid_model_json.cpp docs/design/solid-model-reference.schema.json docs/design/solid-model-json.md tests/unit/io/test_solid_model_json.cpp
git commit -m "core/io: typed solid-model records and the JSON reference format (v1)

Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>"
```

---

### Task 6: Solid-model metrics, `rux analyze solid`, `bench-solid.sh` (bead `reusex-z1v.2.5`, W0.5 part 2)

**Files:**
- Create: `libs/reusex/include/reconstruction/solid_metrics.hpp`, `libs/reusex/src/reconstruction/solid_metrics.cpp`
- Create: `apps/rux/include/analyze/solid.hpp`, `apps/rux/src/analyze/solid.cpp`; modify `apps/rux/src/analyze.cpp`
- Create: `scripts/bench-solid.sh`, `scripts/bench-solid-compare.py`, `scripts/bench-solid.manifest`
- Test: `tests/unit/reconstruction/test_solid_metrics.cpp`, `scripts/test_bench_solid_compare.py`

**Interfaces:**
- Produces (`reconstruction/solid_metrics.hpp`, namespace `reusex::geometry`):

```cpp
struct SolidMetricsOptions {
  double wall_match_angle_deg = 10.0; ///< detected wall must be this parallel to match
  double wall_match_dist_m = 0.5;     ///< and its axis within this of the reference axis
  double opening_match_iou = 0.5;     ///< s-interval IoU on the same wall
  double fit_within_m = 0.02;         ///< point-to-solid "good" distance
};
struct SolidModelReport {
  std::optional<int> room_count_error;       // detected - reference
  std::optional<double> room_iou_mean;       // matched by max IoU per reference space
  std::optional<double> wall_position_rms_m; // matched walls: mean of endpoint->line distances
  std::optional<double> wall_thickness_err_mean_m, wall_thickness_err_p95_m;
  std::optional<double> wall_recall, wall_precision;
  std::optional<double> opening_recall, opening_precision;
  std::optional<double> fit_mean_m, fit_within_share; // point-to-solid
  std::optional<double> seen_share_mean, assumed_share;
  std::string to_json(int indent = 2) const; // null for every nullopt
};
SolidModelReport compare_solid_models(const core::SolidModel &detected,
                                      const core::SolidModel &reference,
                                      const SolidMetricsOptions &opt = {});
/// Point-to-mesh fit of @p cloud against @p mesh (CGAL AABB tree over the
/// triangles). Fills fit_mean_m / fit_within_share of @p report.
void fit_report(const Cloud &cloud, const pcl::PolygonMesh &mesh,
                const SolidMetricsOptions &opt, SolidModelReport &report);
```

- [ ] **Step 1: Write the failing tests**

Create `tests/unit/reconstruction/test_solid_metrics.cpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
// Spec §1 metric table, §4 W0.5. Every metric is nullopt when its inputs are
// missing, so a WP can add its line without the others existing.

#include "../../support/synthetic_scene.hpp"

#include <reusex/core/solid_model.hpp>
#include <reusex/reconstruction/solid_metrics.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <pcl/PolygonMesh.h>
#include <pcl/conversions.h>

using reusex::core::SolidModel;
using reusex::core::SpaceRecord;
using reusex::core::WallRecord;
using reusex::geometry::compare_solid_models;
using reusex::geometry::fit_report;
using reusex::geometry::SolidMetricsOptions;
using reusex::geometry::SolidModelReport;

namespace {
WallRecord wall(const char *guid, double x0, double y0, double x1, double y1,
                double t) {
  WallRecord w;
  w.guid = guid;
  w.x0 = x0; w.y0 = y0; w.x1 = x1; w.y1 = y1;
  w.thickness = t;
  w.height = 2.6;
  return w;
}
SpaceRecord rect(const char *guid, double x0, double y0, double x1,
                 double y1) {
  SpaceRecord s;
  s.guid = guid;
  s.outer = {{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}};
  return s;
}
} // namespace

TEST_CASE("CompareSolidModels_EmptyDetected_AllMetricsNullopt",
          "[reconstruction][solid_metrics]") {
  SolidModel ref;
  ref.walls.push_back(wall("w", 0, 0, 4, 0, 0.1));
  const auto r = compare_solid_models(SolidModel{}, ref);
  CHECK_FALSE(r.wall_position_rms_m.has_value());
  CHECK_FALSE(r.room_iou_mean.has_value());
  CHECK_FALSE(r.opening_recall.has_value());
  CHECK(r.to_json().find("null") != std::string::npos);
}

TEST_CASE("CompareSolidModels_ShiftedWall_ReportsPositionAndThicknessError",
          "[reconstruction][solid_metrics]") {
  SolidModel ref, det;
  ref.walls.push_back(wall("r", 0, 0, 4, 0, 0.10));
  det.walls.push_back(wall("d", 0, 0.03, 4, 0.03, 0.14)); // 3 cm off, 4 cm thick
  const auto r = compare_solid_models(det, ref);
  REQUIRE(r.wall_position_rms_m.has_value());
  CHECK(*r.wall_position_rms_m == Catch::Approx(0.03).margin(1e-6));
  CHECK(*r.wall_thickness_err_mean_m == Catch::Approx(0.04).margin(1e-6));
  CHECK(*r.wall_recall == Catch::Approx(1.0));
  CHECK(*r.wall_precision == Catch::Approx(1.0));
}

TEST_CASE("CompareSolidModels_UnmatchedWalls_LowerRecallAndPrecision",
          "[reconstruction][solid_metrics]") {
  SolidModel ref, det;
  ref.walls.push_back(wall("r1", 0, 0, 4, 0, 0.1));
  ref.walls.push_back(wall("r2", 0, 3, 4, 3, 0.1));
  det.walls.push_back(wall("d1", 0, 0, 4, 0, 0.1));
  det.walls.push_back(wall("d2", 10, 0, 10, 4, 0.1)); // nowhere near
  const auto r = compare_solid_models(det, ref);
  CHECK(*r.wall_recall == Catch::Approx(0.5));
  CHECK(*r.wall_precision == Catch::Approx(0.5));
}

TEST_CASE("CompareSolidModels_RoomIoU_HalfOverlap", "[reconstruction][solid_metrics]") {
  SolidModel ref, det;
  ref.spaces.push_back(rect("r", 0, 0, 4, 4));
  det.spaces.push_back(rect("d", 2, 0, 6, 4)); // overlap 8 of union 24
  const auto r = compare_solid_models(det, ref);
  REQUIRE(r.room_iou_mean.has_value());
  CHECK(*r.room_iou_mean == Catch::Approx(8.0 / 24.0).margin(1e-6));
  CHECK(*r.room_count_error == 0);
}

TEST_CASE("FitReport_PointsOnMesh_WithinShareIsOne",
          "[reconstruction][solid_metrics]") {
  const auto scene = reusex::test_support::make_room(4.0F, 3.0F, 2.5F, 0.1F,
                                                     /*sigma=*/0.0F);
  // The room's own box as a mesh: 8 corners, 12 triangles.
  pcl::PolygonMesh mesh;
  pcl::PointCloud<pcl::PointXYZ> v;
  for (int i = 0; i < 8; ++i)
    v.emplace_back((i & 1) ? 4.0F : 0.0F, (i & 2) ? 3.0F : 0.0F,
                   (i & 4) ? 2.5F : 0.0F);
  pcl::toPCLPointCloud2(v, mesh.cloud);
  const int faces[12][3] = {{0, 1, 2}, {1, 3, 2}, {4, 6, 5}, {5, 6, 7},
                            {0, 4, 1}, {1, 4, 5}, {2, 3, 6}, {3, 7, 6},
                            {0, 2, 4}, {2, 6, 4}, {1, 5, 3}, {3, 5, 7}};
  for (const auto &f : faces) {
    pcl::Vertices tri;
    tri.vertices = {static_cast<uint32_t>(f[0]), static_cast<uint32_t>(f[1]),
                    static_cast<uint32_t>(f[2])};
    mesh.polygons.push_back(tri);
  }
  SolidModelReport r;
  fit_report(*scene.cloud, mesh, SolidMetricsOptions{}, r);
  REQUIRE(r.fit_mean_m.has_value());
  CHECK(*r.fit_mean_m < 1e-5);
  CHECK(*r.fit_within_share == Catch::Approx(1.0));
}
```

(Check `make_room`'s box origin in `tests/support/synthetic_scene.hpp:35-68` and adjust the corner coordinates to its extent if it is not `[0,sx]×[0,sy]×[0,sz]`.)

- [ ] **Step 2: Run to verify they fail**

Run: `scripts/check.sh -R "CompareSolidModels|FitReport"`
Expected: does not compile.

- [ ] **Step 3: Implement the metrics**

`libs/reusex/src/reconstruction/solid_metrics.cpp`:
- **Wall matching:** for every reference wall, candidates are detected walls on the same `storey_idx` whose direction angle (modulo 180°) is within `wall_match_angle_deg` and whose two endpoints have mean distance to the reference's infinite line below `wall_match_dist_m` and whose projection onto the reference axis overlaps it by at least 50% of the shorter wall. Pick the lowest mean distance; a detected wall matches at most one reference wall (greedy by distance, deterministic by guid order). Position error per match = mean of the two detected endpoints' distances to the reference line; RMS over matches. Thickness error = `|det.thickness - ref.thickness|` where both are ≥ 0; mean and P95 (nearest-rank). `wall_recall = matched_ref / ref.size()`, `wall_precision = matched_det / det.size()`; nullopt when the respective side is empty.
- **Rooms:** `room_count_error = det.spaces.size() - ref.spaces.size()` when the reference has spaces; IoU with CGAL `Polygon_set_2<Exact_predicates_exact_constructions_kernel>` (outer ring minus holes), per reference space the max IoU over detected spaces of the same storey; mean over reference spaces.
- **Openings:** match by `wall_guid`-resolved reference wall ↔ matched detected wall, same `kind`, s-interval IoU ≥ `opening_match_iou`; recall/precision.
- **Evidence:** `seen_share_mean` over detected walls with `seen_share >= 0`; `assumed_share` = assumed / walls.
- **fit_report:** build `CGAL::AABB_tree` over the mesh triangles (`CGAL::AABB_face_graph_triangle_primitive` on a `CGAL::Surface_mesh` built from the `PolygonMesh`, or the simpler `AABB_triangle_primitive` over a `std::list<Triangle_3>`), `closest_point_and_primitive` per finite cloud point; mean distance and the share within `fit_within_m`. Skip non-finite points.
- `to_json`: nlohmann, `nullptr` for every `nullopt`.

- [ ] **Step 4: `rux analyze solid`**

`apps/rux/include/analyze/solid.hpp` (same shape as `analyze/accuracy.hpp`):

```cpp
struct SubcommandAnalyzeSolidOptions {
  std::string reference_path;  ///< W0.5 JSON reference ("" = no comparison)
  std::string mesh_name;       ///< mesh to fit against ("" = skip the fit)
  std::string cloud_name = "cloud";
  double fit_within = reusex::geometry::SolidMetricsOptions{}.fit_within_m;
  std::string output_path;     ///< JSON report ("" = stdout)
};
void setup_subcommand_analyze_solid(CLI::App &, std::shared_ptr<RuxOptions>);
int run_subcommand_analyze_solid(SubcommandAnalyzeSolidOptions const &, const RuxOptions &);
```

`apps/rux/src/analyze/solid.cpp`: open read-only; `detected = io::load_solid_model(db)`; if `--reference` given, read the file, `solid_model_from_json`, `compare_solid_models`; if `--mesh` given and `db.has_mesh(name)`, `fit_report(cloud, mesh, ...)`; print `report.to_json()` to stdout or `-o`. Errors: unreadable/invalid reference → `RuxError::INVALID_ARGUMENT` with the validator's message; missing mesh → `INVALID_ARGUMENT`. Register in `apps/rux/src/analyze.cpp` (`setup_subcommand_analyze_solid(*sub, global_opt);` and a `solid` line in the footer).

- [ ] **Step 5: The bench scripts**

`scripts/bench-solid.manifest` (one fixture per line: `name|project env var|reference path or -|local?`):

```
newoffice|REUSEX_BENCH_NEWOFFICE|tests/fixtures/solid_model/newoffice.plan.json|
mushroom-room|REUSEX_BENCH_MUSHROOM|tests/fixtures/solid_model/mushroom-room.plan.json|
s3dis-area1|REUSEX_S3DIS_AREA1_RUX|-|local
```

`scripts/bench-solid.sh` (pattern of `bench-office.sh`): `-o out_dir` (default `./bench-solid`), `-c "stages"` (default `"planes storeys"`; each name is run as `rux create <name>` on a disposable copy), `-m mesh` (optional fit target, default none). For each manifest line whose env var is set (skip with a note otherwise; `local` rows additionally print `local, not committed`): copy the project, run the stages, run `rux analyze solid --reference <ref> [--mesh <m>] -o "$out/<name>.solid.json"`, delete the copy. Finally write `"$out/summary.json"` = `{name: report}` and print a table (python3 inline, one row per fixture, columns `rooms_err`, `room_iou`, `wall_rms_mm`, `thick_mm`, `open_recall`, `fit_mm`, `fit_2cm`).

`scripts/bench-solid-compare.py baseline.json candidate.json [--threshold 5]`: loads two `summary.json`, per fixture and per metric computes the delta; "better" direction table: lower-is-better = `wall_position_rms_m`, `wall_thickness_err_mean_m`, `wall_thickness_err_p95_m`, `fit_mean_m`, `assumed_share`, `|room_count_error|`; higher-is-better = the rest. Exit 1 if any metric worsens by more than `--threshold` percent (absolute for shares); `null` on either side prints `n/a` and never fails. `scripts/test_bench_solid_compare.py` (stdlib `unittest`, like `test_bench_compare.py`): three cases — improvement passes, regression beyond threshold fails, null is n/a.

- [ ] **Step 6: Run the tests and the CLI**

Run: `scripts/check.sh -R "CompareSolidModels|FitReport|SolidModelJson"` and `python3 scripts/test_bench_solid_compare.py`
Expected: PASS. Then `build/apps/rux/rux -p /tmp/no.rux analyze solid --mesh mesh` prints a JSON with every comparison metric `null` and a numeric `fit_mean_m` (if the MIP mesh exists); `rux analyze solid --reference tests/unit/io/<a saved two_wall json>` works.

- [ ] **Step 7: Commit**

```bash
git add libs/reusex/include/reconstruction/solid_metrics.hpp libs/reusex/src/reconstruction/solid_metrics.cpp apps/rux/include/analyze/solid.hpp apps/rux/src/analyze/solid.cpp apps/rux/src/analyze.cpp scripts/bench-solid.sh scripts/bench-solid-compare.py scripts/bench-solid.manifest scripts/test_bench_solid_compare.py tests/unit/reconstruction/test_solid_metrics.cpp
git commit -m "reconstruction: solid-model metrics, rux analyze solid, bench-solid.sh

Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>"
```

Close `reusex-z1v.2.5` after Tasks 5 and 6 with the reason naming the test names and the first NewOffice fit number.

---

### Task 7: Reference-plan authoring loop (bead `reusex-z1v.2.6`, W0.6)

**Files:**
- Modify: `libs/reusex/include/visualize/render_view.hpp` (`RenderOptions`/`SceneOptions` gain `std::optional<std::filesystem::path> plan_overlay`), `libs/reusex/src/visualize/scene.cpp` (draw the overlay), `apps/rux/src/render.cpp` (`--plan <json>`)
- Create: `apps/blender/reusex_panel/plan_io.py` (+ operator registration in the panel's `__init__.py`)
- Create: `docs/guides/solid-model-reference-authoring.md`
- Create: `tests/fixtures/solid_model/newoffice.plan.json` (authored)
- Test: `tests/unit/visualize/test_scene.cpp` (append), `apps/blender/reusex_panel/tests/test_plan_io.py`

**Interfaces:**
- Consumes: `io::solid_model_from_json`, `visualize::populate_scene`.
- Produces: `rux render --plan ref.json --view plan --layers cloud` draws reference walls (thick lines, `--label-2` token colour), spaces (outlines + name), openings (short ticks on the wall) in the **world** frame by applying the inverse of the matching storey's `level_transform` (identity when the reference has none). `plan_io.py`: Blender operators `reusex.import_plan` / `reusex.export_plan` mapping walls ↔ boxes, spaces ↔ flat polygons, openings ↔ boxes parented to their wall, GUIDs kept as custom properties.

- [ ] **Step 1: Write the failing tests**

Append to `tests/unit/visualize/test_scene.cpp` (heavy binary; needs a GL device for the full render, so test the layer builder only):

```cpp
TEST_CASE("PlanOverlay_TwoWallsOneSpace_ProducesActors", "[visualize][plan]") {
  // Build the overlay actors from an in-memory reference and count them:
  // one polyline actor per wall and per space outline, one per opening.
  reusex::core::SolidModel m;
  reusex::core::StoreyRecord s; s.guid = "s"; m.storeys.push_back(s);
  reusex::core::WallRecord w; w.guid = "a"; w.x1 = 4; w.height = 2.6; m.walls.push_back(w);
  w.guid = "b"; w.x0 = 4; w.y1 = 3; m.walls.push_back(w);
  reusex::core::SpaceRecord sp; sp.guid = "r"; sp.outer = {{0,0},{4,0},{4,3},{0,3}}; m.spaces.push_back(sp);
  reusex::core::OpeningRecord o; o.guid = "o"; o.wall_guid = "a"; o.kind = "door"; o.s_start = 1; o.s_end = 2; o.z_head = 2.1; m.openings.push_back(o);

  vtkNew<vtkRenderer> renderer;
  reusex::visualize::SceneBounds bounds;
  const auto actors = reusex::visualize::add_plan_overlay_actors(renderer, m, bounds);
  CHECK(actors.size() == 4);
  CHECK(bounds.valid);
  CHECK(bounds.max[0] == Catch::Approx(4.0));
}
```

`apps/blender/reusex_panel/tests/test_plan_io.py` (pure-python part of `plan_io.py`, no `bpy`): round-trip a two-wall model through `plan_io.model_from_dict` / `plan_io.model_to_dict` and assert equality; a wall with zero length raises `ValueError`.

- [ ] **Step 2: Run to verify they fail**

Run: `scripts/check.sh -R PlanOverlay` and `python3 -m pytest apps/blender/reusex_panel/tests/test_plan_io.py`
Expected: FAIL / does not compile.

- [ ] **Step 3: Implement the overlay**

In `visualize/scene.hpp` declare `std::vector<vtkActor *> add_plan_overlay_actors(vtkRenderer *renderer, const core::SolidModel &model, SceneBounds &bounds);` and a `SceneOptions::plan_overlay` (`std::optional<core::SolidModel>`); `populate_scene` calls it last when set. Lines: wall axis as a `vtkPolyLine` of width 4 in the `--label-2` colour (dimmer when `assumed`), space outer ring closed polyline width 2 in `--label-4`, openings as a segment from `s_start` to `s_end` along the wall at `z_head` width 6 in `--label-6`. `rux render --plan <file>` reads the JSON through `io::solid_model_from_json` and sets the option; `--plan` requires `--layers` to include `cloud` or `planes` so the overlay has something to be checked against (CLI11 `needs`).

- [ ] **Step 4: Blender import/export**

`plan_io.py`: pure functions `model_from_dict(d) -> dict-of-lists` and `model_to_dict(model)` validating like the C++ (`ValueError` naming the guid), then `bpy` operators that create one mesh object per wall (box: length × thickness × height, origin at the axis start, rotated by `atan2`), per space (ngon at `z_floor`), per opening (box parented to the wall object) in a collection `ReUseX Plan`, and export reading the same objects back (`guid`, `kind`, `storey_idx` as custom properties). Register both in the panel.

- [ ] **Step 5: The guide and the first reference**

`docs/guides/solid-model-reference-authoring.md`: (1) `rux -p x.rux render -o plan.png --view plan --layers cloud` to see the cut plan; (2) write `plan.json` by hand or in Blender (`reusex.import_plan` over a `cloud` imported with the existing add-on at 1/10 density); (3) `rux render --plan plan.json ...` and iterate until the walls sit on the point ridges; (4) commit under `tests/fixtures/solid_model/<name>.plan.json`; (5) never commit S3DIS-derived plans. Then author `newoffice.plan.json` from `~/repos/NewOffice/project.rux` this way (the colleague's Rhino output may be traced as the starting point but every wall is checked against the cloud) and add its env var to `scripts/bench-solid.manifest`'s row. Record in the bead notes how many walls, spaces and openings it has and how long authoring took.

- [ ] **Step 6: Run the tests**

Run: `scripts/check.sh -R "PlanOverlay|SolidModelJson"` and the python test. `scripts/bench-solid.sh` with `REUSEX_BENCH_NEWOFFICE=/path/project.rux` runs the newoffice row (all comparison metrics `null` until WP3 but the fit line, if `-m mesh`, is numeric).

- [ ] **Step 7: Commit**

```bash
git add libs/reusex/include/visualize libs/reusex/src/visualize/scene.cpp apps/rux/src/render.cpp apps/blender/reusex_panel docs/guides/solid-model-reference-authoring.md tests/fixtures/solid_model/newoffice.plan.json tests/unit/visualize/test_scene.cpp scripts/bench-solid.manifest
git commit -m "visualize/blender: reference-plan overlay and Blender round-trip; NewOffice reference plan

Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>"
```

---

### Task 8: S3DIS local loader (bead `reusex-z1v.2.7`, W0.7)

**Files:**
- Create: `libs/reusex/include/io/s3dis.hpp`, `libs/reusex/src/io/s3dis.cpp`
- Create: `apps/rux/include/import/s3dis.hpp`, `apps/rux/src/import/s3dis.cpp`; modify `apps/rux/src/import.cpp`
- Modify: `.gitignore` (`tests/fixtures/solid_model/*s3dis*`), `scripts/bench-solid.sh` (`local` rows)
- Test: `tests/unit/io/test_s3dis.cpp` (synthetic directory built in a `TempDir`), `tests/unit/io/test_no_s3dis_fixtures.cpp`

**Interfaces:**
- Produces (`io/s3dis.hpp`, namespace `reusex::io`):

```cpp
struct S3disImportOptions { float voxel_m = 0.02f; };
struct S3disImportResult { size_t rooms = 0, points = 0; core::SolidModel reference; };
/// Reads <area>/<room>/<room>.txt (x y z r g b) and <area>/<room>/Annotations/<class>_<n>.txt
/// into "cloud" + "labels" (class ids from a fixed S3DIS class list through
/// api_to_point_label, label_definitions written) and returns a reference
/// SolidModel: one space per room folder (outer ring = 2D convex hull of the
/// room's floor-class points), one wall face per wall annotation (axis = PCA
/// major axis of the wall points at their mean height, thickness -1).
S3disImportResult import_s3dis_area(ProjectDB &db, const std::filesystem::path &area_dir,
                                    const S3disImportOptions &opt = {});
```

- [ ] **Step 1: Write the failing tests**

`tests/unit/io/test_s3dis.cpp`: in a `TempDir`, write `Area_X/office_1/office_1.txt` with a 2×2×2.5 m box of points (floor, ceiling, four walls, 5 cm grid) and `Annotations/floor_1.txt`, `ceiling_1.txt`, `wall_1.txt`…`wall_4.txt` with the matching subsets; open a `TempPath` ProjectDB; call `import_s3dis_area`; assert `rooms == 1`, the `labels` cloud has `floor` points labelled `api_to_point_label(index_of("floor"))`, `reference.spaces.size() == 1` with area ≈ 4 m², `reference.walls.size() == 4`. `tests/unit/io/test_no_s3dis_fixtures.cpp`: iterate `tests/fixtures/solid_model/` (path from `REUSEX_SOURCE_DIR` compile definition or relative to the test binary; follow how other tests locate `tests/fixtures`) and `REQUIRE` no file name contains `s3dis` (case-insensitive).

- [ ] **Step 2: Run to verify they fail**

Run: `scripts/check.sh -R S3dis`
Expected: does not compile.

- [ ] **Step 3: Implement**

Parse with `std::ifstream` line by line (`x y z r g b`), voxel-downsample with `pcl::VoxelGrid` at `voxel_m`, assign labels by nearest annotated point (`pcl::KdTreeFLANN`, radius `voxel_m`), save `cloud`, `normals` (`pcl::NormalEstimationOMP`, 0.1 m), `labels` (stage `s3dis_import`, parameters with `label_encoding`), `label_definitions` keyed by point label; build the reference model as in the Interfaces block (`CGAL::convex_hull_2` for the ring; PCA for wall axes); write the reference JSON next to the project as `<project>.reference.json` from the CLI. `rux import s3dis <area dir> [--voxel 0.02]`. In `bench-solid.sh`, a `local` row with `-` as reference uses `<project>.reference.json` and prints `(local, not committed)`.

- [ ] **Step 4: Run the tests**

Run: `scripts/check.sh -R "S3dis|NoS3dis"`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add libs/reusex/include/io/s3dis.hpp libs/reusex/src/io/s3dis.cpp apps/rux/include/import/s3dis.hpp apps/rux/src/import/s3dis.cpp apps/rux/src/import.cpp .gitignore scripts/bench-solid.sh tests/unit/io/test_s3dis.cpp tests/unit/io/test_no_s3dis_fixtures.cpp
git commit -m "io: rux import s3dis for local benchmarking (never committed)

Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>"
```

---

### Task 9: `segmentation/level.hpp`: floor/ceiling fit, bisector vertical, storeys (bead `reusex-z1v.3.1`, W1.1)

**Files:**
- Create: `libs/reusex/include/segmentation/level.hpp`, `libs/reusex/src/segmentation/level.cpp`
- Create: `libs/reusex/include/geometry/level_frame.hpp` (header-only; includes `core/solid_model.hpp` and `geometry/transform_utils.hpp`; compiled into consumers only, like `component_persistence.hpp`)
- Test: `tests/unit/segmentation/test_detect_storeys.cpp`, `tests/unit/geometry/test_level_frame.cpp`

**Interfaces:**
- Produces (`segmentation/level.hpp`, namespace `reusex::geometry` like the other segmentation headers):

```cpp
struct LevelOptions {
  double horizontal_cos = 0.9848;   ///< |n.z| above this = horizontal (10 deg)
  double inlier_m = 0.02;           ///< iterated LSQ inlier band
  int max_fit_iterations = 10;
  double hist_bin_m = 0.05;         ///< Z histogram bin
  double peak_min_share = 0.25;     ///< a bin is a level if >= this * max bin
  double slab_merge_m = 0.6;        ///< ceiling_i / floor_{i+1} closer than this = one level
  double min_storey_height_m = 2.0; ///< consecutive levels closer than this are not a storey
  double default_height_m = 2.6;    ///< used when no ceiling level exists
};
struct StoreysResult {
  std::vector<core::StoreyRecord> storeys; ///< idx ascending, z in the level frame
  std::array<double, 3> vertical{0, 0, 1};  ///< world up as measured
  int horizontal_planes = 0, levels = 0;    ///< diagnostics
};
/// @throws std::invalid_argument when cloud/planes sizes disagree or no plane exists.
StoreysResult detect_storeys(const Cloud &cloud, const CloudL &plane_labels,
                             const CloudLoc &plane_centroids, const CloudN &plane_normals,
                             const LevelOptions &opt = {});
```

- Produces (`geometry/level_frame.hpp`, namespace `reusex::geometry`):

```cpp
struct LevelFrame {
  Eigen::Affine3d world_to_level = Eigen::Affine3d::Identity();
  static LevelFrame from_record(const core::StoreyRecord &s) { return {to_affine(s.level_transform)}; }
  Eigen::Vector3d to_level(const Eigen::Vector3d &p) const { return world_to_level * p; }
  Eigen::Vector3d to_world(const Eigen::Vector3d &p) const { return world_to_level.inverse() * p; }
  std::array<double, 16> to_array() const { return to_array16(world_to_level); }
  /// The world z at which the level frame's horizontal plane z = z_level crosses
  /// the frame origin's vertical (used to pick section planes in Task 12).
  double world_z_at(double z_level) const { return to_world(Eigen::Vector3d(0, 0, z_level)).z(); }
};
/// Rotation taking @p vertical to +z and translation putting @p floor_point at z = 0.
LevelFrame make_level_frame(const Eigen::Vector3d &vertical, const Eigen::Vector3d &floor_point);
```

- [ ] **Step 1: Write the failing tests**

`tests/unit/geometry/test_level_frame.cpp`:

```cpp
TEST_CASE("MakeLevelFrame_TiltedVertical_MapsToZUpWithFloorAtZero",
          "[geometry][level_frame]") {
  const Eigen::Vector3d v = Eigen::Vector3d(0.0, std::sin(0.02), std::cos(0.02)).normalized();
  const Eigen::Vector3d floor(1.0, 2.0, 0.3);
  const auto f = reusex::geometry::make_level_frame(v, floor);
  CHECK(f.to_level(floor).z() == Catch::Approx(0.0).margin(1e-9));
  const Eigen::Vector3d up = f.world_to_level.linear() * v;
  CHECK(up.z() == Catch::Approx(1.0).margin(1e-9));
  CHECK(f.to_world(f.to_level(Eigen::Vector3d(3, 4, 5))).isApprox(Eigen::Vector3d(3, 4, 5)));
  reusex::core::StoreyRecord s; s.level_transform = f.to_array();
  CHECK(reusex::geometry::LevelFrame::from_record(s).to_level(floor).z() == Catch::Approx(0.0).margin(1e-9));
}
```

`tests/unit/segmentation/test_detect_storeys.cpp` (helpers: build `planes`/`plane_centroids`/`plane_normals` from the synthetic scene's ground-truth labels by PCA per label; a `tilt(scene, rad)` that rotates every point and normal about x):

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
// Spec §3.3, W1.1; research §1.3(b).

#include "../../support/synthetic_scene.hpp"

#include <reusex/core/solid_model.hpp>
#include <reusex/segmentation/level.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <pcl/common/pca.h>
#include <pcl/common/transforms.h>

#include <map>

using reusex::geometry::detect_storeys;
using reusex::geometry::LevelOptions;
using reusex::test_support::make_room;
using reusex::test_support::make_two_storey_room;

namespace {
struct PlaneSet {
  reusex::CloudLoc centroids;
  reusex::CloudN normals;
};
/// Per-label PCA plane from the scene's ground-truth labels (1-based).
PlaneSet planes_from_labels(const reusex::Cloud &cloud, const reusex::CloudL &labels) {
  std::map<uint32_t, pcl::Indices> by_label;
  for (size_t i = 0; i < labels.size(); ++i)
    if (labels[i].label != 0)
      by_label[labels[i].label].push_back(static_cast<int>(i));
  PlaneSet out;
  out.centroids.resize(by_label.rbegin()->first);
  out.normals.resize(by_label.rbegin()->first);
  for (const auto &[label, idx] : by_label) {
    pcl::PCA<reusex::PointT> pca;
    pca.setInputCloud(cloud.makeShared());
    pca.setIndices(std::make_shared<pcl::Indices>(idx));
    const auto n = pca.getEigenVectors().col(2);
    const auto c = pca.getMean();
    out.centroids[label - 1] = {c.x(), c.y(), c.z()};
    out.normals[label - 1].normal_x = n.x();
    out.normals[label - 1].normal_y = n.y();
    out.normals[label - 1].normal_z = n.z();
  }
  return out;
}
void tilt_about_x(reusex::test_support::SyntheticScene &s, double rad) {
  Eigen::Affine3f t = Eigen::Affine3f(Eigen::AngleAxisf(static_cast<float>(rad), Eigen::Vector3f::UnitX()));
  pcl::transformPointCloud(*s.cloud, *s.cloud, t);
  for (auto &n : s.normals->points) {
    Eigen::Vector3f v = t.linear() * n.getNormalVector3fMap();
    n.normal_x = v.x(); n.normal_y = v.y(); n.normal_z = v.z();
  }
}
} // namespace

TEST_CASE("DetectStoreys_SingleRoom_OneStoreyWithHeight", "[segmentation][storeys]") {
  const auto s = make_room(4.0F, 3.0F, 2.5F, 0.05F, 0.0F);
  const auto p = planes_from_labels(*s.cloud, *s.labels);
  const auto r = detect_storeys(*s.cloud, *s.labels, p.centroids, p.normals);
  REQUIRE(r.storeys.size() == 1);
  CHECK(r.storeys[0].idx == 0);
  CHECK(r.storeys[0].z_floor == Catch::Approx(0.0).margin(0.02));
  CHECK(r.storeys[0].z_ceiling == Catch::Approx(2.5).margin(0.02));
  CHECK(r.storeys[0].prov.confidence > 0.5);
}

TEST_CASE("DetectStoreys_TwoStoreys_TwoRowsOrderedByHeight", "[segmentation][storeys]") {
  const auto s = make_two_storey_room(4.0F, 3.0F, 2.5F, 0.05F, 0.0F);
  const auto p = planes_from_labels(*s.cloud, *s.labels);
  const auto r = detect_storeys(*s.cloud, *s.labels, p.centroids, p.normals);
  REQUIRE(r.storeys.size() == 2);
  CHECK(r.storeys[0].z_floor == Catch::Approx(0.0).margin(0.02));
  CHECK(r.storeys[0].z_ceiling == Catch::Approx(2.5).margin(0.02));
  CHECK(r.storeys[1].z_floor == Catch::Approx(2.5).margin(0.02));
  CHECK(r.storeys[1].z_ceiling == Catch::Approx(5.0).margin(0.02));
  CHECK(r.storeys[1].idx == 1);
}

TEST_CASE("DetectStoreys_TiltedOneDegree_RecoversVerticalAndLevels", "[segmentation][storeys]") {
  auto s = make_room(4.0F, 3.0F, 2.5F, 0.05F, 0.0F);
  const double tilt = 1.0 * M_PI / 180.0;
  tilt_about_x(s, tilt);
  const auto p = planes_from_labels(*s.cloud, *s.labels);
  const auto r = detect_storeys(*s.cloud, *s.labels, p.centroids, p.normals);
  REQUIRE(r.storeys.size() == 1);
  const Eigen::Vector3d expected(0.0, -std::sin(tilt), std::cos(tilt));
  const Eigen::Vector3d got(r.vertical[0], r.vertical[1], r.vertical[2]);
  CHECK(std::acos(std::abs(got.dot(expected))) < 0.1 * M_PI / 180.0);
  CHECK(r.storeys[0].z_ceiling - r.storeys[0].z_floor == Catch::Approx(2.5).margin(0.02));
  // The floor maps to z = 0 in the level frame.
  const Eigen::Affine3d T = reusex::geometry::to_affine(r.storeys[0].level_transform);
  for (size_t i = 0; i < s.labels->size(); ++i)
    if ((*s.labels)[i].label == 1u) { // 1 = floor in make_room
      const Eigen::Vector3d q = T * (*s.cloud)[i].getVector3fMap().cast<double>();
      REQUIRE(std::abs(q.z()) < 0.02);
      break;
    }
}

TEST_CASE("DetectStoreys_NoCeiling_OneStoreyLowConfidence", "[segmentation][storeys]") {
  auto s = make_room(4.0F, 3.0F, 2.5F, 0.05F, 0.0F);
  // Drop the ceiling (label 2 in make_room): a handheld scan that never looked up.
  reusex::Cloud cloud; reusex::CloudL labels;
  for (size_t i = 0; i < s.labels->size(); ++i)
    if ((*s.labels)[i].label != 2u) { cloud.push_back((*s.cloud)[i]); labels.push_back((*s.labels)[i]); }
  cloud.width = cloud.size(); cloud.height = 1; labels.width = labels.size(); labels.height = 1;
  const auto p = planes_from_labels(cloud, labels);
  const auto r = detect_storeys(cloud, labels, p.centroids, p.normals);
  REQUIRE(r.storeys.size() == 1);
  CHECK(r.storeys[0].z_floor == Catch::Approx(0.0).margin(0.02));
  CHECK(r.storeys[0].z_ceiling >= 2.4); // max z of the walls, not the default
  CHECK(r.storeys[0].prov.confidence < 0.5);
}

TEST_CASE("DetectStoreys_TwoRuns_BitIdentical", "[segmentation][storeys]") {
  const auto s = make_two_storey_room();
  const auto p = planes_from_labels(*s.cloud, *s.labels);
  const auto a = detect_storeys(*s.cloud, *s.labels, p.centroids, p.normals);
  const auto b = detect_storeys(*s.cloud, *s.labels, p.centroids, p.normals);
  REQUIRE(a.storeys.size() == b.storeys.size());
  for (size_t i = 0; i < a.storeys.size(); ++i)
    CHECK(a.storeys[i].level_transform == b.storeys[i].level_transform);
}
```

Check `make_room`'s label numbering in `tests/support/synthetic_scene.hpp` and fix the floor/ceiling label constants in the tests if they differ.

- [ ] **Step 2: Run to verify they fail**

Run: `scripts/check.sh -R "DetectStoreys|MakeLevelFrame"`
Expected: does not compile.

- [ ] **Step 3: Implement `level_frame.hpp`**

```cpp
inline LevelFrame make_level_frame(const Eigen::Vector3d &vertical,
                                   const Eigen::Vector3d &floor_point) {
  const Eigen::Vector3d v = vertical.normalized();
  const Eigen::Quaterniond q =
      Eigen::Quaterniond::FromTwoVectors(v, Eigen::Vector3d::UnitZ());
  Eigen::Affine3d T = Eigen::Affine3d::Identity();
  T.linear() = q.toRotationMatrix();
  // Rotate about the floor point so XY stay near their world values, then
  // drop the floor to z = 0.
  T.translation() = -T.linear() * floor_point;
  T.translation().x() += floor_point.x();
  T.translation().y() += floor_point.y();
  return LevelFrame{T};
}
```

- [ ] **Step 4: Implement `detect_storeys`**

`libs/reusex/src/segmentation/level.cpp`, in order:

1. Validate sizes (`cloud.size() == plane_labels.size()`, `plane_centroids.size() == plane_normals.size()`, at least one plane) else `std::invalid_argument`.
2. Horizontal planes: label `l` (1-based) is horizontal when `|normal_z| >= opt.horizontal_cos`; collect per-plane inlier index lists from `plane_labels` (one pass).
3. Z histogram: bins of `opt.hist_bin_m` from `z_min` of all horizontal inliers, count = points. Levels = bins that are local maxima with count ≥ `peak_min_share × max`, refined to the mean z of the points in that bin and its neighbours. Merge levels closer than `slab_merge_m` into one (keep both heights: `lower`, `upper`). Sort ascending.
4. Storeys = consecutive level pairs `(L_i, L_{i+1})` with `L_{i+1}.lower - L_i.upper >= min_storey_height_m`; storey i's floor height = `L_i.upper` (top of the slab), ceiling = `L_{i+1}.lower`. If only one level: one storey, ceiling = max z of all points, `confidence = 0.3`, `warn("detect_storeys: one horizontal level found ({} horizontal planes); using the cloud's max z {:.2f} m as ceiling")`. If zero horizontal planes: throw `std::runtime_error` with the counts (STANDARDS §5).
5. Plane fits: for storey 0's floor and ceiling levels take every horizontal inlier within `±2 × hist_bin_m` of the level height; iterated LSQ (`pcl::PCA` on the current inliers, then keep points within `inlier_m` of the plane, repeat up to `max_fit_iterations` or until the inlier set stops changing); orient both normals so `normal.z > 0`. `vertical = (n_floor + n_ceiling).normalized()` (floor only: `n_floor`). Floor point = the floor fit's centroid.
6. `frame = make_level_frame(vertical, floor_point)`; for every storey `z_floor`, `z_ceiling` = the level heights transformed (`frame.to_level(Eigen::Vector3d(cx, cy, h)).z()` using the floor centroid's x,y); `floor_plane`/`ceiling_plane` as `(a,b,c,d)` from the fits where they exist (storey 0 always; others from their own level fits, same routine); `level_transform = frame.to_array()` on every row; `idx` ascending; `name = "Etage " + idx`; `prov.source = "detected"`, `confidence = 0.9` when both fits exist with ≥ 100 inliers, else 0.3.
7. `info` with: horizontal planes, levels, storeys and their heights.

- [ ] **Step 5: Run the tests**

Run: `scripts/check.sh -R "DetectStoreys|MakeLevelFrame"`
Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add libs/reusex/include/segmentation/level.hpp libs/reusex/src/segmentation/level.cpp libs/reusex/include/geometry/level_frame.hpp tests/unit/segmentation/test_detect_storeys.cpp tests/unit/geometry/test_level_frame.cpp
git commit -m "segmentation: detect_storeys (floor/ceiling fit, bisector vertical, Z-histogram levels) and LevelFrame

Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>"
```

---

### Task 10: `storeys` table, deterministic GUIDs, schema v28 (bead `reusex-z1v.3.2`, W1.2)

**Files:**
- Modify: `libs/reusex/include/core/guid.hpp`, `libs/reusex/src/core/guid.cpp`
- Modify: `libs/reusex/include/core/ProjectDB.hpp` (storeys API), `libs/reusex/src/core/ProjectDB.cpp` (v28 migration, Impl bodies, wrappers)
- Modify: `libs/reusex/src/io/solid_model_json.cpp` (`load_solid_model` fills `storeys`)
- Test: `tests/unit/core/test_guid.cpp`, `tests/unit/core/test_project_db_storeys.cpp`

**Interfaces:**
- Produces (`core/guid.hpp`): `std::string deterministic_guid(std::string_view ns_guid, std::string_view name);` — RFC 4122 v5 over SHA-1 (`boost::uuids::name_generator_sha1` with `boost::uuids::string_generator()(ns_guid)`), lowercase hyphenated.
- Produces (`ProjectDB`):

```cpp
  // --- Storeys (solid model, spec §3.2) ---
  /// Replace every row whose `locked` is 0 with @p storeys (matched by guid:
  /// a locked row with the same guid is kept, a locked row with no match is
  /// kept too). Returns the number written.
  size_t save_storeys(const std::vector<core::StoreyRecord> &storeys, int run_id = -1);
  std::vector<core::StoreyRecord> storeys() const;   // idx ascending
  std::optional<core::StoreyRecord> storey_by_guid(std::string_view guid) const;
  bool has_storeys() const;
  void delete_storeys(); // all, including locked (an explicit user action)
  /// The project's own stable guid (projects.guid), the namespace for deterministic_guid().
  std::string project_guid() const;
```

- [ ] **Step 1: Write the failing tests**

`tests/unit/core/test_guid.cpp`:

```cpp
TEST_CASE("DeterministicGuid_SameInputs_SameOutput_DifferentNamesDiffer", "[core][guid]") {
  const std::string ns = "6ba7b810-9dad-11d1-80b4-00c04fd430c8";
  const auto a = reusex::core::deterministic_guid(ns, "storeys/0");
  const auto b = reusex::core::deterministic_guid(ns, "storeys/0");
  const auto c = reusex::core::deterministic_guid(ns, "storeys/1");
  CHECK(a == b);
  CHECK(a != c);
  CHECK(a.size() == 36);
  CHECK(a[14] == '5'); // RFC 4122 version nibble
}
```

`tests/unit/core/test_project_db_storeys.cpp` (TempDB pattern from `test_project_db_migration_v26.cpp`):

```cpp
TEST_CASE("Storeys_SaveLoad_RoundTripsAndGuidIsStable", "[ProjectDB][storeys]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  reusex::core::StoreyRecord s;
  s.idx = 0; s.z_floor = 0.0; s.z_ceiling = 2.6; s.name = "Etage 0";
  s.level_transform[11] = -0.3; // translation z
  s.guid = reusex::core::deterministic_guid(db.project_guid(), "storeys/0");
  REQUIRE(db.save_storeys({s}) == 1);
  auto got = db.storeys();
  REQUIRE(got.size() == 1);
  CHECK(got[0].guid == s.guid);
  CHECK(got[0].z_ceiling == Catch::Approx(2.6));
  CHECK(got[0].level_transform[11] == Catch::Approx(-0.3));
  CHECK(got[0].id > 0);
  // A second save of the same storey keeps the guid and does not duplicate.
  s.z_ceiling = 2.7;
  REQUIRE(db.save_storeys({s}) == 1);
  got = db.storeys();
  REQUIRE(got.size() == 1);
  CHECK(got[0].z_ceiling == Catch::Approx(2.7));
}

TEST_CASE("Storeys_LockedRow_SurvivesRerun", "[ProjectDB][storeys]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  reusex::core::StoreyRecord s;
  s.guid = "manual-0"; s.idx = 0; s.z_ceiling = 3.0; s.prov.locked = true; s.prov.source = "manual";
  db.save_storeys({s});
  reusex::core::StoreyRecord d;
  d.guid = "det-0"; d.idx = 0; d.z_ceiling = 2.5;
  db.save_storeys({d});
  auto got = db.storeys();
  REQUIRE(got.size() == 2);
  CHECK(db.storey_by_guid("manual-0")->z_ceiling == Catch::Approx(3.0));
}

TEST_CASE("MigrationV28_FromV27_AddsStoreysTable", "[ProjectDB][migration]") {
  TempDB tmp;
  { ProjectDB db(tmp.path); CHECK(db.schema_version() == 28); }
  exec_raw(tmp.path, "DROP TABLE storeys; DELETE FROM schema_version WHERE version >= 28;");
  ProjectDB db(tmp.path);
  CHECK(db.schema_version() == 28);
  CHECK_FALSE(db.has_storeys());
  CHECK(reusex::io::load_solid_model(db).storeys.empty());
}
```

- [ ] **Step 2: Run to verify they fail**

Run: `scripts/check.sh -R "DeterministicGuid|Storeys_|MigrationV28"`
Expected: does not compile.

- [ ] **Step 3: Implement**

`guid.cpp`:

```cpp
#include <boost/uuid/name_generator_sha1.hpp>
#include <boost/uuid/string_generator.hpp>
#include <boost/uuid/uuid_io.hpp>

std::string deterministic_guid(std::string_view ns_guid, std::string_view name) {
  const boost::uuids::uuid ns =
      boost::uuids::string_generator()(std::string(ns_guid));
  boost::uuids::name_generator_sha1 gen(ns);
  return boost::uuids::to_string(gen(name.data(), name.size()));
}
```

(If Boost.Uuid's name generator is not available in the flake's Boost, implement SHA-1 locally in `guid.cpp` — RFC 3174, ~70 lines — and set the version/variant nibbles by hand; the test pins both.)

`ProjectDB.cpp`: `LATEST_SCHEMA_VERSION = 28`; `migrateToV28()` creates the `storeys` table exactly as in spec §3.2 (plus `CREATE INDEX IF NOT EXISTS idx_storeys_idx ON storeys(idx);`) and ensures `projects.guid` exists (check `project_guid()`'s source: if `projects` has no guid column, add `guid TEXT` and fill it once with `generate_guid()`); `insertSchemaVersion(28, "storeys table (solid model WP1)")`. Impl: `saveStoreys` in a `Savepoint`: `DELETE FROM storeys WHERE locked = 0 AND guid NOT IN (<locked guids>)`, then per record: `INSERT ... ON CONFLICT(guid) DO UPDATE SET ... WHERE storeys.locked = 0` (a locked row is never overwritten); blobs bound with `sqlite3_bind_blob` from the `std::array<double,N>` bytes; `getStoreys` reads back `ORDER BY idx, id`. `io::load_solid_model(db)`: `model.storeys = db.storeys();` when `db.has_storeys()`.

- [ ] **Step 4: Run the tests**

Run: `scripts/check.sh -R "DeterministicGuid|Storeys_|MigrationV2"`
Expected: PASS (v26 and v27 migration tests still green).

- [ ] **Step 5: Commit**

```bash
git add libs/reusex/include/core/guid.hpp libs/reusex/src/core/guid.cpp libs/reusex/include/core/ProjectDB.hpp libs/reusex/src/core/ProjectDB.cpp libs/reusex/src/io/solid_model_json.cpp tests/unit/core/test_guid.cpp tests/unit/core/test_project_db_storeys.cpp
git commit -m "core: storeys table (schema v28), deterministic uuid5 GUIDs, locked rows survive re-runs

Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>"
```

---

### Task 11: The `storeys` stage end to end (bead `reusex-z1v.3.3`, W1.3)

**Files:**
- Modify: `libs/reusex/include/core/stage_contract.hpp` (enum: `storeys` after `instances`), `libs/reusex/src/core/stage_contract.cpp` (artifact + row), `tests/unit/core/test_stage_contract.cpp` (`all_stages()`), `docs/CONTRACTS.md` (section after `### instances`)
- Modify: `libs/reusex/include/pipeline/stages.hpp` (`JobStage::storeys` **appended last**), `libs/reusex/src/pipeline/stages.cpp` (`kStages` size 8, `run_storeys`, dispatch), `libs/reusex/src/pipeline/stage_parameters.cpp` (`storeys_parameters()`, switch), `tests/unit/pipeline/test_stage_contract_agreement.cpp` (`contract_of`, `runnable_stages`)
- Create: `apps/rux/include/create/storeys.hpp`, `apps/rux/src/create/storeys.cpp`; modify `apps/rux/src/create.cpp`
- Create: `apps/rux/include/database/storey_router.hpp`, `apps/rux/src/database/storey_router.cpp`; modify `apps/rux/src/database/resource_router.cpp`
- Modify: `apps/rux/qt/src/pipeline_ui.cpp` (4 switches), `apps/rux/qt/src/core/cli_command.cpp` (`storeys` spec)
- Test: `tests/unit/pipeline/test_storeys_stage.cpp`, `tests/unit/rux_app/test_cli_command_roundtrip.cpp` (automatic)

**Interfaces:**
- Consumes: `geometry::detect_storeys`, `LevelOptions`, `ProjectDB::save_storeys`, `deterministic_guid`, `project_guid`.
- Produces: `PipelineStage::storeys`, `JobStage::storeys`, `rux create storeys [--hist-bin --peak-min-share --slab-merge --min-storey-height --default-height --inlier --max-fit-iterations]`, `rux get storeys[/<idx>] [--json]`, parameter keys = `LevelOptions` field names.

- [ ] **Step 1: Write the failing tests**

`tests/unit/pipeline/test_storeys_stage.cpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "../../support/synthetic_scene.hpp"
#include "../../support/temp_path.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/guid.hpp>
#include <reusex/pipeline/stages.hpp>
#include <reusex/segmentation/segment_planes.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

using reusex::ProjectDB;
namespace pl = reusex::pipeline;

TEST_CASE("StoreysStage_TwoStoreyRoom_WritesTwoRowsWithStableGuids",
          "[pipeline][storeys]") {
  reusex::test_support::TempPath tmp("test_storeys_stage");
  ProjectDB db(tmp.path);
  const auto s = reusex::test_support::make_two_storey_room();
  db.save_point_cloud("cloud", *s.cloud, "test");
  db.save_point_cloud("normals", *s.normals, "test");
  auto [labels, centroids, normals] =
      reusex::geometry::segment_planes(s.cloud, s.normals);
  db.save_point_cloud("planes", *labels, "segment_planes");
  db.save_point_cloud("plane_centroids", *centroids, "segment_planes");
  db.save_point_cloud("plane_normals", *normals, "segment_planes");

  pl::StageContext ctx;
  ctx.project = tmp.path;
  ctx.stage = pl::JobStage::storeys;
  const auto r1 = pl::run_stage(db, ctx);
  REQUIRE(r1.ok);
  REQUIRE(r1.outputs.size() == 1);
  CHECK(r1.outputs[0].kind == "table");
  CHECK(r1.outputs[0].name == "storeys");
  CHECK(r1.outputs[0].count == 2);
  const auto first = db.storeys();
  REQUIRE(first.size() == 2);
  CHECK(first[0].guid ==
        reusex::core::deterministic_guid(db.project_guid(), "storeys/0"));

  const auto r2 = pl::run_stage(db, ctx);
  REQUIRE(r2.ok);
  const auto second = db.storeys();
  REQUIRE(second.size() == 2);
  CHECK(second[0].guid == first[0].guid);
  CHECK(second[0].id == first[0].id);
}

TEST_CASE("StoreysStage_NoPlanes_IsRefused", "[pipeline][storeys]") {
  reusex::test_support::TempPath tmp("test_storeys_stage_refuse");
  ProjectDB db(tmp.path);
  pl::StageContext ctx;
  ctx.project = tmp.path;
  ctx.stage = pl::JobStage::storeys;
  const auto r = pl::run_stage(db, ctx);
  CHECK_FALSE(r.ok);
  CHECK(r.invalid_input);
}
```

- [ ] **Step 2: Run to verify they fail**

Run: `scripts/check.sh -R StoreysStage`
Expected: does not compile (`JobStage::storeys`).

- [ ] **Step 3: Contract and pipeline**

`stage_contract.hpp`: insert `storeys, ///< detect building storeys and the level frame` after `instances`. `stage_contract.cpp`: artifact `{"storeys", ArtifactKind::table, Alignment::none, "building storeys: level frame, floor/ceiling heights (one row per storey)"}`; row:

```cpp
    {PipelineStage::storeys,
     "storeys",
     {},
     "rux create storeys",
     "detect building storeys and the levelled frame",
     {{{"cloud"}}, {{"planes"}}, {{"plane_centroids"}}, {{"plane_normals"}}},
     {"storeys"}},
```

`test_stage_contract.cpp`: add `PipelineStage::storeys` to `all_stages()` after `instances`. `docs/CONTRACTS.md` after `### instances`:

```markdown
### `storeys` (`rux create storeys`)

Fit floor and ceiling, measure true vertical, level the scan and detect the
storeys (spec `superpowers/specs/2026-10-13-solid-model-floor-plan-design.md` §3.3).
The scan is never rewritten: each row stores the world→level transform.

| | |
|---|---|
| Consumes | `cloud`, `planes`, `plane_centroids`, `plane_normals` |
| Produces | `storeys` (table) |
| Options  | `LevelOptions` (`libs/reusex/include/segmentation/level.hpp`) |
| Checks   | `cloud`/`planes` present + aligned; `plane_centroids`/`plane_normals` present + aligned |

A single-storey scan is one row; a scan with no ceiling gets its ceiling from
the cloud's top with a low confidence and a warning. Locked rows (manual
edits) survive a re-run; detected rows are replaced, keeping their GUIDs
(`deterministic_guid(project_guid, "storeys/<idx>")`).
```

`stages.hpp`: append `storeys, ///< detect storeys and the level frame -> "storeys" table` **after `annotate`** (so no Qt ordinal shifts). `stages.cpp`: `kStages` becomes `std::array<StageName, 8>` with `{JobStage::storeys, "storeys", "segment_storeys", core::PipelineStage::storeys, true}`; `run_storeys`:

```cpp
StageResult run_storeys(ProjectDB &db, const StageContext &ctx,
                        const json &params) {
  geometry::LevelOptions options;
  options.hist_bin_m = param_or(params, "hist_bin_m", options.hist_bin_m);
  options.peak_min_share = param_or(params, "peak_min_share", options.peak_min_share);
  options.slab_merge_m = param_or(params, "slab_merge_m", options.slab_merge_m);
  options.min_storey_height_m = param_or(params, "min_storey_height_m", options.min_storey_height_m);
  options.default_height_m = param_or(params, "default_height_m", options.default_height_m);
  options.inlier_m = param_or(params, "inlier_m", options.inlier_m);
  options.max_fit_iterations = param_or(params, "max_fit_iterations", options.max_fit_iterations);

  auto cloud = db.point_cloud_xyzrgb("cloud");
  auto planes = db.point_cloud_label("planes");
  auto centroids = db.point_cloud_xyz("plane_centroids");
  auto normals = db.point_cloud_normal("plane_normals");

  auto result = geometry::detect_storeys(*cloud, *planes, *centroids, *normals, options);
  if (ctx.is_cancelled())
    return StageResult::cancel("storey detection cancelled");

  const std::string ns = db.project_guid();
  for (auto &s : result.storeys)
    s.guid = core::deterministic_guid(ns, "storeys/" + std::to_string(s.idx));
  const size_t written = db.save_storeys(result.storeys);
  return StageResult::success(
      fmt::format("detected {} storey(s) from {} horizontal plane(s)",
                  result.storeys.size(), result.horizontal_planes),
      {{"table", "storeys", static_cast<int64_t>(written)}});
}
```

plus `case JobStage::storeys: return run_storeys(db, ctx, params);`. `stage_parameters.cpp`: `storeys_parameters()` with one descriptor per option (`num(...)`/`integer(...)` with the `LevelOptions{}` defaults and sensible bounds) and the switch case. `test_stage_contract_agreement.cpp`: `case pipeline::JobStage::storeys: return core::PipelineStage::storeys;` and add it to `runnable_stages()`.

- [ ] **Step 4: CLI, router, Qt**

`apps/rux/src/create/storeys.cpp` = `create/planes.cpp` with `SubcommandStoreysOptions` (defaults from `LevelOptions{}`), flags `--hist-bin`, `--peak-min-share`, `--slab-merge`, `--min-storey-height`, `--default-height`, `--inlier`, `--max-fit-iterations`, `storeys_stage_parameters()` emitting the `LevelOptions` field names as keys, and `run_subcommand_storeys` through `pipeline::run_stage`. Register in `create.cpp` after `instances` (include, footer line `storeys      Detect building storeys and level the scan`, setup call). `StoreyRouter` (copy `ComponentRouter`): `list()` = `"0".."n-1"`; `get()` = all rows or one by idx, each as JSON with every record field; `set`/`del` throw `"storeys are created by `rux create storeys`"`. `pipeline_ui.cpp`: `stage_name_da` → `"Etager"`, `stage_blurb_da` → `"Finder etager og retter scanningen op (gulv og loft, lodret)."`, `parameter_label_da` entries for the seven keys under the new ordinal, `contract_stage` → `core::PipelineStage::storeys`. `cli_command.cpp`: `{"storeys", {"create", "storeys"}, {{"hist_bin_m", "--hist-bin", {}, {}}, {"peak_min_share", "--peak-min-share", {}, {}}, {"slab_merge_m", "--slab-merge", {}, {}}, {"min_storey_height_m", "--min-storey-height", {}, {}}, {"default_height_m", "--default-height", {}, {}}, {"inlier_m", "--inlier", {}, {}}, {"max_fit_iterations", "--max-fit-iterations", {}, {}}}}`.

- [ ] **Step 5: Run the tests**

Run: `scripts/check.sh -R "StoreysStage|StageContract|CliCommandRoundTrip|stage_parameters|rux_qt"`
Expected: PASS. Then on NewOffice: `rux -p /tmp/no.rux create storeys && rux -p /tmp/no.rux get storeys --json` prints ≥ 1 row; `rux validate --stage storeys` passes; the Qt gallery (`RUX_QT_PAGE=pipeline`) lists Etager.

- [ ] **Step 6: Commit**

```bash
git add libs/reusex/include/core/stage_contract.hpp libs/reusex/src/core/stage_contract.cpp tests/unit/core/test_stage_contract.cpp docs/CONTRACTS.md libs/reusex/include/pipeline/stages.hpp libs/reusex/src/pipeline/stages.cpp libs/reusex/src/pipeline/stage_parameters.cpp tests/unit/pipeline/test_stage_contract_agreement.cpp tests/unit/pipeline/test_storeys_stage.cpp apps/rux/include/create/storeys.hpp apps/rux/src/create/storeys.cpp apps/rux/src/create.cpp apps/rux/include/database/storey_router.hpp apps/rux/src/database/storey_router.cpp apps/rux/src/database/resource_router.cpp apps/rux/qt/src/pipeline_ui.cpp apps/rux/qt/src/core/cli_command.cpp
git commit -m "pipeline/rux: storeys stage (contract row, runner, CLI, get route, Qt pipeline entry)

Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>"
```

---

### Task 12: The MIP slices only at storey planes (bead `reusex-z1v.3.4`, W1.4)

**Files:**
- Modify: `libs/reusex/include/reconstruction/mesh.hpp` (`MeshOptions::section_heights`, `section_tolerance_m`), `libs/reusex/src/reconstruction/mesh.cpp:120-190`
- Modify: `libs/reusex/src/pipeline/stages.cpp` (`run_mesh` fills `section_heights` from storeys), `apps/rux/src/create/mesh.cpp` (same), `libs/reusex/src/pipeline/mesh_parameters.cpp` (no new JSON key: the heights come from the table, not from parameters)
- Test: `tests/unit/reconstruction/test_mesh_sections.cpp`

**Interfaces:**
- Produces in `MeshOptions`:

```cpp
  /// World z of every storey floor/ceiling (from the storeys table, through
  /// LevelFrame::world_z_at). When non-empty, only horizontal planes within
  /// section_tolerance_m of one of these heights become section planes of
  /// the cell complex; table tops and shelves no longer slice the building
  /// (spec W1.4). Empty = every horizontal plane is a section (legacy).
  std::vector<double> section_heights;
  double section_tolerance_m = 0.15;
```

- A free function in `mesh.hpp` so it is unit-testable without a solve:

```cpp
/// The subset of @p horizontals (indices into @p planes) kept as section
/// planes for @p section_heights. Returns @p horizontals unchanged when the
/// heights are empty, or when NO plane matches (with a warn): a stale storeys
/// table must not silence the mesh stage.
std::vector<size_t> select_section_planes(const EigenVectorContainer<double, 4> &planes,
                                          const std::vector<size_t> &horizontals,
                                          const std::vector<double> &section_heights,
                                          double tolerance_m);
```

- [ ] **Step 1: Write the failing tests**

`tests/unit/reconstruction/test_mesh_sections.cpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
// Spec W1.4: with a storeys table, only storey floor/ceiling planes slice the
// cell complex; furniture-height planes are not sections.

#include <reusex/reconstruction/mesh.hpp>

#include <catch2/catch_test_macros.hpp>

using reusex::EigenVectorContainer;
using reusex::geometry::select_section_planes;

namespace {
Eigen::Vector4d z_plane(double h) { return {0.0, 0.0, 1.0, -h}; }
} // namespace

TEST_CASE("SelectSectionPlanes_TableTopDropped_FloorAndCeilingKept",
          "[reconstruction][sections]") {
  EigenVectorContainer<double, 4> planes{z_plane(0.0), z_plane(0.75),
                                         z_plane(2.5), Eigen::Vector4d(1, 0, 0, 0)};
  const std::vector<size_t> horizontals{0, 1, 2};
  const auto kept = select_section_planes(planes, horizontals, {0.0, 2.5}, 0.15);
  CHECK(kept == std::vector<size_t>{0, 2});
}

TEST_CASE("SelectSectionPlanes_EmptyHeights_KeepsEverything",
          "[reconstruction][sections]") {
  EigenVectorContainer<double, 4> planes{z_plane(0.0), z_plane(0.75)};
  const std::vector<size_t> horizontals{0, 1};
  CHECK(select_section_planes(planes, horizontals, {}, 0.15) == horizontals);
}

TEST_CASE("Mesh_StoreysMatchNoPlane_FallsBackToAllHorizontals",
          "[reconstruction][sections]") {
  // Stale storeys (planes re-run, heights no longer match): never drop every
  // section silently.
  EigenVectorContainer<double, 4> planes{z_plane(0.0), z_plane(2.5)};
  const std::vector<size_t> horizontals{0, 1};
  CHECK(select_section_planes(planes, horizontals, {10.0, 12.5}, 0.15) == horizontals);
}
```

- [ ] **Step 2: Run to verify they fail**

Run: `scripts/check.sh -R "SelectSectionPlanes|Mesh_StoreysMatchNoPlane"`
Expected: does not compile.

- [ ] **Step 3: Implement**

`mesh.cpp`:

```cpp
std::vector<size_t>
select_section_planes(const EigenVectorContainer<double, 4> &planes,
                      const std::vector<size_t> &horizontals,
                      const std::vector<double> &section_heights,
                      double tolerance_m) {
  if (section_heights.empty())
    return horizontals;
  std::vector<size_t> kept;
  for (size_t id : horizontals) {
    const auto &p = planes[id];
    if (std::abs(p[2]) < reusex::kEpsilonRelative)
      continue;
    const double h = -p[3] / p[2];
    for (double sh : section_heights)
      if (std::abs(h - sh) <= tolerance_m) {
        kept.push_back(id);
        break;
      }
  }
  if (kept.empty()) {
    reusex::warn("mesh: none of the {} horizontal planes lies within {:.2f} m "
                 "of a storey height ({} heights); the storeys table is "
                 "probably stale — re-run `rux create storeys`. Using every "
                 "horizontal plane as a section instead.",
                 horizontals.size(), tolerance_m, section_heights.size());
    return horizontals;
  }
  reusex::info("mesh: {} of {} horizontal planes are storey sections "
               "(tolerance {:.2f} m)",
               kept.size(), horizontals.size(), tolerance_m);
  return kept;
}
```

and in `mesh()` right after `separate_planes`: `horizontal = select_section_planes(planes, horizontal, opt.section_heights, opt.section_tolerance_m);`. In `pipeline/stages.cpp` `run_mesh` and in `apps/rux/src/create/mesh.cpp`, after reading options: if `db.has_storeys()`, for every storey `s`: `const auto f = geometry::LevelFrame::from_record(s); options.section_heights.push_back(f.world_z_at(s.z_floor)); options.section_heights.push_back(f.world_z_at(s.z_ceiling));` (include `reusex/geometry/level_frame.hpp`; `pipeline` and `apps` may include it). The parameters JSON gains nothing; the `pipeline_log` message records how many heights were used.

- [ ] **Step 4: Run the tests**

Run: `scripts/check.sh -R "SelectSectionPlanes|Mesh_|sectioned"`
Expected: PASS.

- [ ] **Step 5: Benchmark before/after (STANDARDS §8.1)**

On NewOffice copies: baseline = `create planes && create rooms && create mesh` with no storeys table (`-vv`, note `cells`, solve time, `Solidification` status); candidate = the same with `create storeys` first. Then `rux analyze solid --mesh mesh -o {before,after}.json` and `scripts/bench-solid-compare.py before.json after.json` (wrap each in `{"newoffice": ...}` or use `bench-solid.sh -c "planes rooms mesh"` vs `-c "planes rooms storeys mesh" -m mesh`). Record cell count, solve time, `fit_mean_m` and `fit_within_share` before/after in `bd update reusex-z1v.3.4 --notes`.

- [ ] **Step 6: Commit**

```bash
git add libs/reusex/include/reconstruction/mesh.hpp libs/reusex/src/reconstruction/mesh.cpp libs/reusex/src/pipeline/stages.cpp apps/rux/src/create/mesh.cpp tests/unit/reconstruction/test_mesh_sections.cpp
git commit -m "reconstruction: cell complex slices only at storey heights when storeys exist

Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>"
```

---

## After the last task

- Close the beads: `bd close reusex-z1v.2.1 reusex-z1v.2.2 reusex-z1v.2.3 reusex-z1v.2.4 reusex-z1v.2.5 reusex-z1v.2.6 reusex-z1v.2.7 reusex-z1v.3.1 reusex-z1v.3.2 reusex-z1v.3.3 reusex-z1v.3.4 --reason="..."` (each with its test names and numbers), then the epics `reusex-z1v.2` and `reusex-z1v.3`.
- Append a dated entry to `docs/DIRECTION.md`'s changelog naming the WP0/WP1 landing and the NewOffice before/after numbers.
- `bd ready` then shows `reusex-z1v.4.4` (write the WP2 plan) as the next bead.
