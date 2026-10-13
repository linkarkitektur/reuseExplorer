# Resource View Record Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Give every resource (survey part) one stored, user-changeable image, the *view record* (sensor frame or 360° panorama plus a normalised region). It is set automatically on sync and on mask → resource, PATCHable through the resource key `sys:view`, and it is the only image the GUI table thumbnail and Foto tab show.

**Architecture:** ReUseX stores the record in six nullable `survey_parts.view_*` columns (schema v27). It validates the record in one pure `core/resource_view` unit and exposes it through `SurveyPartRecord::view`, `core::Resource::view` and a new built-in key `sys:view`. Because `core` may not link `vision`, the instance→frame bbox projection moves from `vision` into a new `core/frame_projection` unit, and `vision` keeps an alias. ruxd serialises the record (`view` on every resource, `region` on every instance frame) and passes the mask bbox on mask → resource. rux-frontend renders the record with a region crop and offers "Brug som ressourcebillede" / "Fjern billede". It never falls back to a ranking.

**Tech Stack:** C++20, SQLite (schema migration), Eigen, OpenCV core, nlohmann::json, Catch2 v3 (ReUseX + ruxd); Crow/ruxd API layer; React 19 + TypeScript + vitest (rux-frontend).

**Spec:** `docs/superpowers/specs/2026-10-13-resource-view-record-design.md` (in the ReUseX `resource-view` worktree). Executors read the spec alongside this plan.

## Global Constraints

- Merge order **ReUseX → ruxd → rux-frontend** (spec §6). Each Part is executed by a different agent, in its own repo/branch, in that order. Part 2 needs Part 1 built and installed; Part 3 needs Part 2's JSON.
- Repos and branches:
  - ReUseX: worktree `/home/mephisto/repos/ReUseX__worktrees/resource-view`, branch `resource-view` (already exists; do not touch `/home/mephisto/repos/ReUseX` or its `build/`, which is a shared benchmark resource).
  - ruxd: `/home/mephisto/repos/ruxd`, currently on `main`. **Create branch `resource-view` before editing** (`git switch -c resource-view`).
  - rux-frontend: `/home/mephisto/repos/rux-frontend`, currently on `main`. **Create branch `resource-view` before editing**.
- Schema: `LATEST_SCHEMA_VERSION` goes 26 → **27**. The migration is additive, and existing rows stay NULL.
- View record fields (spec §2, copied): `kind` `frame` | `panorama`; `id` int; `region` `[x, y, w, h]` normalised 0..1 or null (whole image). For a panorama `x + w > 1` wraps across the seam. Invariant: `view_kind`/`view_id` both null or both set. Region all-null or all-set with `w, h > 0`, `x, y ≥ 0`, `y + h ≤ 1`, `w ≤ 1`.
- **No fallback ranking** anywhere a view is consumed. A part without a view shows its "no image" state.
- Automatic regions are padded **15 % per side** (of the region's own width/height) and clamped to the image.
- Panorama choice window: **~60° × 45°** centred on the instance's `u,v` (= `w = 1/6`, `h = 1/4`).
- `sys:view` wire value: compact JSON `{"kind":"frame","id":123,"region":[x,y,w,h]}` (keys in that order), or unset. `display_value` gives `Foto #123` / `360° #4`.
- Danish UI copy: `Intet billede valgt`, `Brug som ressourcebillede`, `Valgt`, `Fjern billede`.
- ReUseX: `core` MUST NOT link `vision` (STANDARDS §1). `core` also does not link `geometry_common` (`reusexLibrary.cmake:337`), so the projection helper lives in `core`.
- ReUseX: no `std::stod`/`strtod` in library code (STANDARDS §5). Use nlohmann/`from_chars`.
- ReUseX tests: `TEST_CASE` names follow `Subject_Scenario_ExpectedBehavior` (STANDARDS §7.1). New tests go in `tests/unit/core/` (light binary), temp paths via `tests/support/temp_path.hpp`, floats via `WithinAbs`.
- Every new file gets an SPDX header (`2026 Povl Filip Sonne-Frederiksen`, `GPL-3.0-or-later`).
- ruxd: an API change updates `docs/api/openapi.yaml` **in the same commit** (ruxd CLAUDE.md). Note: the spec says `docs/gui/`; the contract actually lives in `docs/api/openapi.yaml`.
- rux-frontend: tokens only (`var(--…)`), Danish copy, pure-logic modules stay DOM-free, and `src/api/types.ts` mirrors the contract exactly.
- Commits end with `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`.

## Review Focus

1. **A chosen frame/panorama is deleted later.** The part must read as "no view" (null) with one `warn`. The GUI shows "Intet billede valgt", not a broken image or a 500. The test is in Task 1.3 (`SurveyPartView_DanglingPanorama_ReadsNullAndWarns`).
2. **A padded region that touches the image edge** (`y + h` computes to 1.0000000000000002). It must not trip the setter or the schema CHECK. The test is in Task 1.3 (`SetSurveyPartView_RoundedEdgeRegion_IsAccepted`).
3. **ruxd opens a stale v26 case read-only.** There are no `view_*` columns, so `survey_parts()` and `list_resources()` must still work, with `view` null. The test is in Task 1.3 (`MigrationV27_ReadOnlyV26_ReadsWithoutViews`).
4. **A panorama region across the seam** (`u` ≈ 0.02 → `x` ≈ 0.94, `x + w` > 1). The server accepts it, and the frontend crop wraps instead of showing a blank strip. A *frame* region that wraps is refused. Tests are in Task 1.1 (`ParseView_PanoramaWrap_AcceptedFrameWrap_Throws`), Task 3.1 (`panoView` wrap) and Task 3.2 (`cropBackground` wrap).
5. **Sync on a project that has frames but no `cloud` positions cloud** (or misaligned clouds). `rux create survey` / `POST /survey/sync` must still succeed. Parts get no view, with a `warn` naming the reason. The test is in Task 1.5 (`SyncSurvey_FramesButNoPositionsCloud_SucceedsWithoutViews`).

---

## Interfaces between parts

This section is the contract the three agents share. Names and shapes here are binding. If a task needs to deviate, it updates this section in the same commit.

### A. ReUseX library (Part 1 produces, Part 2 consumes)

New header `libs/reusex/include/core/resource_view.hpp` (installed as `<reusex/core/resource_view.hpp>`):

```cpp
namespace reusex::core {
enum class ViewKind { frame, panorama };
std::string_view to_string(ViewKind kind);                       // "frame" | "panorama"
std::optional<ViewKind> view_kind_from_string(std::string_view s);
using ViewRegion = std::array<double, 4>;                        // [x, y, w, h], 0..1
inline constexpr double kViewRegionEpsilon = 1e-9;
inline constexpr double kViewRegionPadding = 0.15;
struct ResourceView {
  ViewKind kind = ViewKind::frame;
  int id = 0;
  std::optional<ViewRegion> region; // nullopt = whole image
  bool operator==(const ResourceView &) const = default;
};
void validate_view_shape(const ResourceView &view);   // throws std::invalid_argument
std::string view_to_json(const ResourceView &view);   // {"kind":..,"id":..,"region":[..]|null}
ResourceView parse_view(std::string_view json);       // throws std::invalid_argument
std::string view_display(const ResourceView &view);   // "Foto #123" | "360° #4"
ViewRegion pad_region(const ViewRegion &region, double padding = kViewRegionPadding);
}
```

New header `libs/reusex/include/core/frame_projection.hpp` (`<reusex/core/frame_projection.hpp>`):

```cpp
namespace reusex::core {
struct ProjectedBox { int x_min = 0, y_min = 0, x_max = 0, y_max = 0; int in_bounds = 0; };
std::optional<ProjectedBox>
project_points_to_box(const std::vector<Eigen::Vector3f> &world_points,
                      const std::array<double, 16> &pose,
                      const SensorIntrinsics &intrinsics, int image_width, int image_height);
std::optional<ViewRegion>
project_instance_bbox(const ProjectDB &db, int frame_id,
                      const std::vector<Eigen::Vector3f> &points,
                      double padding = kViewRegionPadding);
std::map<std::uint32_t, std::vector<Eigen::Vector3f>>
instance_points(const ProjectDB &db, std::string_view label_cloud,
                std::string_view positions_cloud = "cloud");
}
// vision/instance_crop.hpp keeps: namespace reusex::vision { using core::ProjectedBox; using core::project_points_to_box; }
```

`ProjectDB` (`include/core/ProjectDB.hpp`):

```cpp
using ResourceView = core::ResourceView;                    // public alias in class ProjectDB
struct SurveyPartRecord { /* … existing fields …, instance_guid{} */ std::optional<core::ResourceView> view{}; }; // LAST
bool has_panoramic_image(int id) const;
void set_survey_part_view(std::string_view code, const std::optional<core::ResourceView> &view);
//   throws std::invalid_argument (shape), std::out_of_range (no part / no frame / no panorama)
```

Resources and keys:

```cpp
// resource_keys.hpp
inline constexpr std::string_view kViewKey = "sys:view";
// builtin_keys() gains, LAST: {id "sys:view", label "Ressourcebillede", category "Kortlægning",
//   scope part, data_type "view", editable true, source builtin, field "view"}
// resources.hpp
struct Resource { std::string code; int64_t type_id; bool manual; std::vector<ResourceValue> values;
                  std::optional<ResourceView> view; };  // view always filled, independent of `keys`
```

Survey sync and mask selection:

```cpp
// survey_service.hpp
struct SurveySyncOptions { /* … */ std::string positions_cloud = "cloud"; };
struct SurveySyncReport  { /* … */ std::size_t views_attached = 0; std::size_t views_unavailable = 0; };
// mask_selection.hpp
std::optional<ViewRegion> mask_region(const cv::Mat &mask); // unpadded normalised bbox of non-zero pixels
struct MaskSelectionOptions { /* … */ std::optional<ViewRegion> view_region; };
struct MaskSelectionResult  { /* … */ std::optional<ResourceView> view; };
```

### B. The `sys:view` value (ReUseX ↔ ruxd ↔ frontend)

- In `values["sys:view"]`: a string, canonical compact JSON, key order `kind, id, region`. Example: `"{\"kind\":\"frame\",\"id\":7,\"region\":[0.1,0.2,0.3,0.4]}"`. `region` is `null` for the whole image. The value is `null` when unset.
- PATCH input: any JSON string `parse_view` accepts (unknown fields refused), or `null`/`""` to clear. The server canonicalises it. A shape error or a source that does not exist is a **400** (`KeyValueError`).

### C. ruxd JSON (Part 2 produces, Part 3 consumes)

- **Resource** (`GET /resources`, `PATCH /resources/{code}` → `resource`/`siblings[]`, `POST /resources`):
  ```json
  {"code":"RX-001","type_id":3,"manual":false,
   "values":{"sys:view":"{\"kind\":\"frame\",\"id\":7,\"region\":[0.1,0.2,0.3,0.4]}", "...":"..."},
   "view":{"kind":"frame","id":7,"region":[0.1,0.2,0.3,0.4]}}
  ```
  `view` is always present (object or `null`), even when a `?template=` filter leaves `sys:view` out of `values`.
- **Key catalogue** (`GET /resources/keys`): includes `{"id":"sys:view","label":"Ressourcebillede","category":"Kortlægning","scope":"part","data_type":"view","unit":null,"options":[],"editable":true}`. `data_type` enum gains `view`.
- **Instance frames** (`GET /instances/{cloud}/{id}/frames`): every `frames[]` item gains `"region": [x,y,w,h] | null`. This is the padded, normalised bbox of the instance in that frame (`core::project_instance_bbox`). It is null when the instance does not project into that frame or the base `cloud` is missing.
- **Mask → resource** (`POST /frames/{id}/segment/resource`, 201): gains `"view": {"kind":"frame","id":<id>,"region":[…]} | null`.
- **Survey sync** (`POST /survey/sync`): gains `"views_attached": int, "views_unavailable": int`.
- Panorama image URL (unchanged): `GET /panoramas/{id}/image?max_size=N`. Frame image URL (unchanged): `GET /frames/{id}/image?kind=color&max_size=N`.

### D. Frontend types (Part 3)

```ts
export type ViewRegion = [number, number, number, number];
export interface ResourceView { kind: 'frame' | 'panorama'; id: number; region: ViewRegion | null }
export interface Resource { code: string; type_id: number; manual: boolean; values: Record<string, string | null>; view: ResourceView | null }
export type ResourceDataType = 'text' | 'number' | 'enum' | 'multiselect' | 'boolean' | 'date' | 'view';
export interface InstanceFrame extends VisibleFrame { region: ViewRegion | null }
// SurveySyncReport: views_attached?: number; views_unavailable?: number;
// SegmentResourceResult: view: ResourceView | null;
```

---

# Part 1 — ReUseX (library, schema v27)

Work in `/home/mephisto/repos/ReUseX__worktrees/resource-view` on branch `resource-view`. Enter the dev shell first: `cd /home/mephisto/repos/ReUseX__worktrees/resource-view && nix develop`. The worktree has no build dir yet. The first configure + build is long (the full CUDA closure). Every later task reuses `build/`:

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=ON
cmake --build build --target reusex_unit_tests --parallel
```

Run a test subset with `ctest --test-dir build -R '<regex>' --output-on-failure --parallel "$(nproc)"`. `catch_discover_tests` registers one ctest per `TEST_CASE` name, so a new test only appears after a rebuild.

### Task 1.1: `core/resource_view` — the view record value type

**Files:**
- Create: `libs/reusex/include/core/resource_view.hpp`
- Create: `libs/reusex/src/core/resource_view.cpp`
- Test: `tests/unit/core/test_resource_view.cpp`

**Interfaces:**
- Consumes: nothing.
- Produces: everything in Interfaces §A under `resource_view.hpp`.

- [ ] **Step 1: Write the failing test** `tests/unit/core/test_resource_view.cpp`

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// The resource view record (spec 2026-10-13-resource-view-record §2): shape
// validation, the canonical sys:view JSON, its display text and padding.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <core/resource_view.hpp>

#include <stdexcept>
#include <string>

using namespace reusex::core;
using Catch::Matchers::WithinAbs;

TEST_CASE("ViewToJson_FrameWithRegion_IsCompactKindIdRegion", "[core][view]") {
  const ResourceView v{ViewKind::frame, 123, ViewRegion{0.1, 0.2, 0.3, 0.4}};
  CHECK(view_to_json(v) == R"({"kind":"frame","id":123,"region":[0.1,0.2,0.3,0.4]})");
  const ResourceView whole{ViewKind::panorama, 4, std::nullopt};
  CHECK(view_to_json(whole) == R"({"kind":"panorama","id":4,"region":null})");
}

TEST_CASE("ParseView_RoundTrip_IsIdentity", "[core][view]") {
  const ResourceView v{ViewKind::frame, 7, ViewRegion{0.25, 0.5, 0.5, 0.25}};
  CHECK(parse_view(view_to_json(v)) == v);
  // Field order and a missing region do not matter on input.
  CHECK(parse_view(R"({"id":3,"kind":"panorama"})") ==
        ResourceView{ViewKind::panorama, 3, std::nullopt});
}

TEST_CASE("ParseView_BadShapes_Throw", "[core][view]") {
  for (const char *bad : {
           "not json", "[]", R"({"kind":"photo","id":1})", R"({"kind":"frame"})",
           R"({"kind":"frame","id":"1"})", R"({"kind":"frame","id":1.5})",
           R"({"kind":"frame","id":-1})", R"({"kind":"frame","id":1,"extra":0})",
           R"({"kind":"frame","id":1,"region":[0,0,1]})",
           R"({"kind":"frame","id":1,"region":[0,0,"1",1]})",
           R"({"kind":"frame","id":1,"region":[0,0,0,0.5]})",     // w = 0
           R"({"kind":"frame","id":1,"region":[-0.1,0,0.5,0.5]})", // x < 0
           R"({"kind":"frame","id":1,"region":[0,0.6,0.5,0.5]})",  // y + h > 1
           R"({"kind":"frame","id":1,"region":[0,0,1.5,0.5]})",    // w > 1
           R"({"kind":"panorama","id":1,"region":[1.0,0,0.2,0.5]})", // x >= 1
           R"({"kind":"frame","id":99999999999,"region":null})"}) {
    INFO(bad);
    CHECK_THROWS_AS(parse_view(bad), std::invalid_argument);
  }
}

TEST_CASE("ParseView_PanoramaWrap_AcceptedFrameWrap_Throws", "[core][view]") {
  // A panorama region may cross the equirect seam; a frame region may not.
  CHECK(parse_view(R"({"kind":"panorama","id":2,"region":[0.9,0.4,0.2,0.25]})")
            .region.has_value());
  CHECK_THROWS_AS(
      parse_view(R"({"kind":"frame","id":2,"region":[0.9,0.4,0.2,0.25]})"),
      std::invalid_argument);
}

TEST_CASE("ValidateViewShape_EpsilonOvershoot_IsAccepted", "[core][view]") {
  // y + h one ulp above 1 is rounding, not an error.
  const double y = 0.3, h = 0.7000000000000001;
  CHECK_NOTHROW(validate_view_shape({ViewKind::frame, 1, ViewRegion{0, y, 1.0, h}}));
}

TEST_CASE("ViewDisplay_KindAndId_DanishLabel", "[core][view]") {
  CHECK(view_display({ViewKind::frame, 123, std::nullopt}) == "Foto #123");
  CHECK(view_display({ViewKind::panorama, 4, std::nullopt}) == "360° #4");
}

TEST_CASE("PadRegion_FifteenPercentPerSide_ClampedToImage", "[core][view]") {
  const auto r = pad_region({0.4, 0.4, 0.2, 0.1});
  CHECK_THAT(r[0], WithinAbs(0.37, 1e-12));
  CHECK_THAT(r[1], WithinAbs(0.385, 1e-12));
  CHECK_THAT(r[2], WithinAbs(0.26, 1e-12));
  CHECK_THAT(r[3], WithinAbs(0.13, 1e-12));
  const auto edge = pad_region({0.0, 0.9, 1.0, 0.1});
  CHECK(edge[0] == 0.0);
  CHECK(edge[2] == 1.0);
  CHECK(edge[1] + edge[3] <= 1.0 + kViewRegionEpsilon);
}

TEST_CASE("ViewKindFromString_Unknown_ReturnsNullopt", "[core][view]") {
  CHECK(view_kind_from_string("frame") == ViewKind::frame);
  CHECK(view_kind_from_string("panorama") == ViewKind::panorama);
  CHECK_FALSE(view_kind_from_string("Frame"));
  CHECK(to_string(ViewKind::panorama) == "panorama");
}
```

- [ ] **Step 2: Run the test and confirm it fails**

Run: `cmake --build build --target reusex_unit_tests --parallel`
Expected: compile error `core/resource_view.hpp: No such file or directory`.

- [ ] **Step 3: Write the header** `libs/reusex/include/core/resource_view.hpp`

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

/// The view record of a resource (docs/superpowers/specs/
/// 2026-10-13-resource-view-record-design.md §2): the one chosen image a
/// survey part is shown by — a sensor frame or a 360° panorama — plus the
/// region of it that shows the part. Pure value type: no ProjectDB, so
/// ProjectDB.hpp, resource keys and ruxd can all share the validation.

#include <array>
#include <optional>
#include <string>
#include <string_view>

namespace reusex::core {

enum class ViewKind { frame, panorama };
std::string_view to_string(ViewKind kind); ///< "frame" | "panorama"
std::optional<ViewKind> view_kind_from_string(std::string_view s);

/// [x, y, w, h], normalised 0..1 of the image (the equirect for a panorama).
using ViewRegion = std::array<double, 4>;

/// Slack on `y + h <= 1` (and a frame's `x + w <= 1`), so a region computed
/// by clamping never fails on rounding. The schema CHECK uses the same value.
inline constexpr double kViewRegionEpsilon = 1e-9;
/// Padding on every side of an automatically chosen region, as a fraction
/// of the region's own width/height (spec §2: "padded 15 % per side").
inline constexpr double kViewRegionPadding = 0.15;

struct ResourceView {
  ViewKind kind = ViewKind::frame;
  int id = 0; ///< sensor_frames.node_id or panoramic_images.id
  std::optional<ViewRegion> region; ///< nullopt: the whole image
  bool operator==(const ResourceView &) const = default;
};

/// @throws std::invalid_argument naming the broken rule: id < 0; a
///         non-finite coordinate; w or h <= 0; x or y < 0; x >= 1; w > 1;
///         y + h > 1; for a frame, x + w > 1 (only a panorama wraps across
///         its seam). Existence of the frame/panorama is not checked here.
void validate_view_shape(const ResourceView &view);

/// Compact JSON, keys in the order kind, id, region (region null for the
/// whole image) — the canonical `sys:view` value.
std::string view_to_json(const ResourceView &view);

/// Parse a `sys:view` value: an object with "kind" and "id" and an optional
/// "region" (null or four numbers); any other field is refused. Validated
/// with validate_view_shape. @throws std::invalid_argument.
ResourceView parse_view(std::string_view json);

/// "Foto #123" for a frame, "360° #4" for a panorama (CSV display).
std::string view_display(const ResourceView &view);

/// @p region grown by @p padding of its own width/height on every side,
/// clamped to [0, 1].
ViewRegion pad_region(const ViewRegion &region,
                      double padding = kViewRegionPadding);

} // namespace reusex::core
```

- [ ] **Step 4: Write the implementation** `libs/reusex/src/core/resource_view.cpp`

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "reusex/core/resource_view.hpp"

#include <fmt/format.h>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

namespace reusex::core {

std::string_view to_string(ViewKind kind) {
  return kind == ViewKind::panorama ? "panorama" : "frame";
}

std::optional<ViewKind> view_kind_from_string(std::string_view s) {
  if (s == "frame")
    return ViewKind::frame;
  if (s == "panorama")
    return ViewKind::panorama;
  return std::nullopt;
}

void validate_view_shape(const ResourceView &v) {
  if (v.id < 0)
    throw std::invalid_argument(fmt::format("view id must be >= 0, got {}", v.id));
  if (!v.region)
    return;
  const auto &r = *v.region;
  for (double c : r)
    if (!std::isfinite(c))
      throw std::invalid_argument("view region must hold finite numbers");
  const double x = r[0], y = r[1], w = r[2], h = r[3];
  if (!(w > 0.0) || !(h > 0.0))
    throw std::invalid_argument(
        fmt::format("view region needs w, h > 0, got w={} h={}", w, h));
  if (x < 0.0 || y < 0.0)
    throw std::invalid_argument(
        fmt::format("view region needs x, y >= 0, got x={} y={}", x, y));
  if (x >= 1.0)
    throw std::invalid_argument(fmt::format("view region needs x < 1, got {}", x));
  if (w > 1.0 + kViewRegionEpsilon)
    throw std::invalid_argument(fmt::format("view region needs w <= 1, got {}", w));
  if (y + h > 1.0 + kViewRegionEpsilon)
    throw std::invalid_argument(
        fmt::format("view region needs y + h <= 1, got {}", y + h));
  if (v.kind == ViewKind::frame && x + w > 1.0 + kViewRegionEpsilon)
    throw std::invalid_argument(fmt::format(
        "a frame view region needs x + w <= 1, got {} (only a panorama "
        "wraps across its seam)",
        x + w));
}

std::string view_to_json(const ResourceView &v) {
  nlohmann::ordered_json j;
  j["kind"] = std::string(to_string(v.kind));
  j["id"] = v.id;
  j["region"] = v.region ? nlohmann::ordered_json(*v.region)
                         : nlohmann::ordered_json(nullptr);
  return j.dump();
}

ResourceView parse_view(std::string_view text) {
  const auto j = nlohmann::json::parse(text.begin(), text.end(), nullptr,
                                       /*allow_exceptions=*/false);
  if (j.is_discarded() || !j.is_object())
    throw std::invalid_argument(
        "a view must be a JSON object {\"kind\", \"id\", \"region\"}");
  for (const auto &item : j.items())
    if (item.key() != "kind" && item.key() != "id" && item.key() != "region")
      throw std::invalid_argument("unknown view field '" + item.key() + "'");
  const auto kind = j.find("kind");
  const auto parsed_kind =
      kind != j.end() && kind->is_string()
          ? view_kind_from_string(kind->get<std::string>())
          : std::nullopt;
  if (!parsed_kind)
    throw std::invalid_argument("view 'kind' must be \"frame\" or \"panorama\"");
  const auto id = j.find("id");
  if (id == j.end() || !id->is_number_integer())
    throw std::invalid_argument("view 'id' must be an integer");
  const bool too_big =
      id->is_number_unsigned()
          ? id->get<std::uint64_t>() >
                static_cast<std::uint64_t>(std::numeric_limits<int>::max())
          : id->get<std::int64_t>() > std::numeric_limits<int>::max() ||
                id->get<std::int64_t>() < std::numeric_limits<int>::min();
  if (too_big)
    throw std::invalid_argument("view 'id' is out of range");
  ResourceView v{*parsed_kind, static_cast<int>(id->get<std::int64_t>()),
                 std::nullopt};
  if (const auto r = j.find("region"); r != j.end() && !r->is_null()) {
    if (!r->is_array() || r->size() != 4)
      throw std::invalid_argument("view 'region' must be null or [x, y, w, h]");
    ViewRegion region{};
    for (std::size_t i = 0; i < 4; ++i) {
      if (!(*r)[i].is_number())
        throw std::invalid_argument("view 'region' must hold four numbers");
      region[i] = (*r)[i].get<double>();
    }
    v.region = region;
  }
  validate_view_shape(v);
  return v;
}

std::string view_display(const ResourceView &v) {
  return v.kind == ViewKind::frame ? fmt::format("Foto #{}", v.id)
                                   : fmt::format("360° #{}", v.id);
}

ViewRegion pad_region(const ViewRegion &r, double padding) {
  const double dx = r[2] * padding;
  const double dy = r[3] * padding;
  const double x0 = std::max(0.0, r[0] - dx);
  const double y0 = std::max(0.0, r[1] - dy);
  const double x1 = std::min(1.0, r[0] + r[2] + dx);
  const double y1 = std::min(1.0, r[1] + r[3] + dy);
  return {x0, y0, x1 - x0, y1 - y0};
}

} // namespace reusex::core
```

- [ ] **Step 5: Run the tests and confirm they pass**

Run: `cmake --build build --target reusex_unit_tests --parallel && ctest --test-dir build -R 'ViewToJson|ParseView|ValidateViewShape|ViewDisplay|PadRegion|ViewKindFromString' --output-on-failure --parallel "$(nproc)"`
Expected: 8 tests PASS. (`src/core/*.cpp` is globbed with `CONFIGURE_DEPENDS`, so no CMake edit is needed.)

- [ ] **Step 6: Commit**

```bash
git add libs/reusex/include/core/resource_view.hpp libs/reusex/src/core/resource_view.cpp tests/unit/core/test_resource_view.cpp
git commit -m "core: resource view record value type (sys:view shape, JSON, padding)

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

### Task 1.2: `core/frame_projection` — instance bbox in a frame, shared with `vision`

**Files:**
- Create: `libs/reusex/include/core/frame_projection.hpp`
- Create: `libs/reusex/src/core/frame_projection.cpp`
- Modify: `libs/reusex/include/vision/instance_crop.hpp` (becomes aliases)
- Delete: `libs/reusex/src/vision/instance_crop.cpp`
- Move: `tests/unit/vision/test_instance_crop.cpp` → `tests/unit/core/test_frame_projection.cpp` (`git mv`, then edit)

**Interfaces:**
- Consumes: `ViewRegion`, `kViewRegionPadding`, `pad_region` (Task 1.1); `ProjectDB::has_sensor_frame_pose`, `sensor_frame_pose`, `sensor_frame_intrinsics`, `point_cloud_xyzrgb`, `point_cloud_label`, `has_point_cloud`; `core::CloudMisalignedError` (`core/instance_evidence.hpp`).
- Produces: `core::ProjectedBox`, `core::project_points_to_box`, `core::project_instance_bbox`, `core::instance_points` (Interfaces §A). `vision::project_points_to_box` / `vision::ProjectedBox` remain valid names, so `src/vision/describe.cpp` compiles unchanged.

- [ ] **Step 1: Move the vision test into core and add the new cases**

```bash
git mv tests/unit/vision/test_instance_crop.cpp tests/unit/core/test_frame_projection.cpp
```

In `tests/unit/core/test_frame_projection.cpp`:
- Change the header comment to `// Pinhole projection of instance points into a sensor frame (#373; moved from vision for the resource view record).`
- Replace `#include <vision/instance_crop.hpp>` with `#include <core/frame_projection.hpp>`.
- Replace `using reusex::vision::project_points_to_box;` with `using reusex::core::project_points_to_box;`.
- Change every tag `"[vision][crop]"` to `"[core][projection]"`. Keep the `TEST_CASE` names, so `ctest -R` patterns are unchanged.

Then append:

```cpp
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <core/ProjectDB.hpp>
#include <core/instance_evidence.hpp>

#include "../../support/temp_path.hpp"

#include <opencv2/core.hpp>
#include <pcl/point_types.h>

using Catch::Matchers::WithinAbs;
using reusex::ProjectDB;
using reusex::core::instance_points;
using reusex::core::project_instance_bbox;

namespace {
/// fx = fy = 100, principal point (64, 64), 128 x 128 — the instance
/// evidence fixture's camera.
SensorIntrinsics pinhole128() {
  SensorIntrinsics k;
  k.fx = k.fy = 100.0;
  k.cx = k.cy = 64.0;
  k.width = k.height = 128;
  return k;
}
void frame_at_origin(ProjectDB &db, int id) {
  db.save_sensor_frame(id, cv::Mat(128, 128, CV_8UC3, cv::Scalar(40, 90, 160)),
                       cv::Mat(), cv::Mat(), kIdentityPose, pinhole128(),
                       static_cast<double>(id), -1);
}
/// Instance 1: (-0.1, 0, 2) and (0.1, 0, 2); instance 2: (0, 0, -10)
/// (behind the camera); one unlabeled point and one NaN point of instance 1.
void seed_instances(ProjectDB &db) {
  reusex::Cloud positions;
  reusex::CloudL labels;
  auto add = [&](float x, float y, float z, std::uint32_t label) {
    pcl::PointXYZRGB p;
    p.x = x;
    p.y = y;
    p.z = z;
    positions.push_back(p);
    pcl::Label l;
    l.label = label;
    labels.push_back(l);
  };
  add(-0.1F, 0.0F, 2.0F, 1);
  add(0.1F, 0.0F, 2.0F, 1);
  add(std::numeric_limits<float>::quiet_NaN(), 0.0F, 2.0F, 1);
  add(0.0F, 0.0F, -10.0F, 2);
  add(5.0F, 5.0F, 5.0F, 0);
  db.save_point_cloud("cloud", positions, "test", "{}");
  db.save_point_cloud("instances", labels, "test", "{}");
}
} // namespace

TEST_CASE("InstancePoints_GroupsFinitePointsByLabel_SkipsUnlabeled",
          "[core][projection]") {
  const reusex::test_support::TempPath tmp("instance_points");
  ProjectDB db(tmp.path);
  seed_instances(db);
  const auto pts = instance_points(db, "instances");
  REQUIRE(pts.size() == 2);
  CHECK(pts.at(1).size() == 2); // the NaN point is dropped
  CHECK(pts.at(2).size() == 1);
  CHECK(pts.count(0) == 0);
  CHECK_THROWS_AS(instance_points(db, "nope"), std::invalid_argument);
  CHECK_THROWS_AS(instance_points(db, "instances", "nope"), std::invalid_argument);
  reusex::CloudL longer;
  longer.resize(9);
  db.save_point_cloud("instances", longer, "test", "{}");
  CHECK_THROWS_AS(instance_points(db, "instances"),
                  reusex::core::CloudMisalignedError);
}

TEST_CASE("ProjectInstanceBbox_SeenInstance_ReturnsPaddedNormalisedRegion",
          "[core][projection]") {
  const reusex::test_support::TempPath tmp("instance_bbox");
  ProjectDB db(tmp.path);
  seed_instances(db);
  frame_at_origin(db, 1);
  const auto pts = instance_points(db, "instances");
  // u = 59 and 69, v = 64: box [59, 64, 11 px, 1 px] of 128, padded 15 %.
  const auto r = project_instance_bbox(db, 1, pts.at(1));
  REQUIRE(r);
  CHECK_THAT((*r)[0], WithinAbs(57.35 / 128.0, 1e-9));
  CHECK_THAT((*r)[1], WithinAbs(63.85 / 128.0, 1e-9));
  CHECK_THAT((*r)[2], WithinAbs(14.3 / 128.0, 1e-9));
  CHECK_THAT((*r)[3], WithinAbs(1.3 / 128.0, 1e-9));
  const auto unpadded = project_instance_bbox(db, 1, pts.at(1), 0.0);
  REQUIRE(unpadded);
  CHECK_THAT((*unpadded)[2], WithinAbs(11.0 / 128.0, 1e-9));
}

TEST_CASE("ProjectInstanceBbox_UnseenOrUnposed_ReturnsNullopt",
          "[core][projection]") {
  const reusex::test_support::TempPath tmp("instance_bbox_none");
  ProjectDB db(tmp.path);
  seed_instances(db);
  frame_at_origin(db, 1);
  const auto pts = instance_points(db, "instances");
  CHECK_FALSE(project_instance_bbox(db, 1, pts.at(2))); // behind the camera
  CHECK_FALSE(project_instance_bbox(db, 99, pts.at(1))); // no such frame
  CHECK_FALSE(project_instance_bbox(db, 1, {}));         // no points
}
```

Also add `#include <cstdint>` and `#include <limits>` at the top of the file.

- [ ] **Step 2: Run the tests and confirm they fail**

Run: `cmake --build build --target reusex_unit_tests --parallel`
Expected: compile error `core/frame_projection.hpp: No such file or directory`.

- [ ] **Step 3: Write the header** `libs/reusex/include/core/frame_projection.hpp`

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

/// Where an instance lands in a sensor frame: the pinhole bbox of a point set
/// (moved here from vision/instance_crop, #373) and the padded, normalised
/// region the resource view record stores (spec 2026-10-13-resource-view-
/// record §2). In core because sync_survey and apply_mask_selection need it
/// and core may not link vision (STANDARDS §1); vision re-exports it.

#include "reusex/core/SensorIntrinsics.hpp"
#include "reusex/core/resource_view.hpp"

#include <Eigen/Core>

#include <array>
#include <cstdint>
#include <map>
#include <optional>
#include <string_view>
#include <vector>

namespace reusex {
class ProjectDB;
}

namespace reusex::core {

/// A projected 2D bounding box (inclusive pixel bounds), with the count of
/// points that landed inside the image.
struct ProjectedBox {
  int x_min = 0, y_min = 0, x_max = 0, y_max = 0;
  int in_bounds = 0;
};

/// Project @p world_points through @p pose (row-major 4x4 base-to-world) and
/// @p intrinsics (scaled from their native size to @p image_width x
/// @p image_height); the bbox of the points in front of the camera and inside
/// the image. nullopt when none is, for empty input or degenerate intrinsics.
std::optional<ProjectedBox>
project_points_to_box(const std::vector<Eigen::Vector3f> &world_points,
                      const std::array<double, 16> &pose,
                      const SensorIntrinsics &intrinsics, int image_width,
                      int image_height);

/// The region of frame @p frame_id that shows @p points: their bbox in the
/// frame's intrinsics resolution, normalised to 0..1 and padded by
/// @p padding per side (pad_region). nullopt when the frame has no usable
/// pose (has_sensor_frame_pose), degenerate intrinsics, or no point projects
/// inside it. No occlusion test: occluded points of the instance widen it.
std::optional<ViewRegion>
project_instance_bbox(const ProjectDB &db, int frame_id,
                      const std::vector<Eigen::Vector3f> &points,
                      double padding = kViewRegionPadding);

/// The finite points of every instance of @p label_cloud, positions taken
/// from the index-aligned @p positions_cloud. Label 0 (unlabeled) skipped.
/// @throws std::invalid_argument when either cloud is not in the project,
///         CloudMisalignedError (instance_evidence.hpp) when they differ in
///         size, std::runtime_error when a cloud cannot be loaded.
std::map<std::uint32_t, std::vector<Eigen::Vector3f>>
instance_points(const ProjectDB &db, std::string_view label_cloud,
                std::string_view positions_cloud = "cloud");

} // namespace reusex::core
```

- [ ] **Step 4: Write the implementation** `libs/reusex/src/core/frame_projection.cpp`

Move the body of `vision::project_points_to_box` from `src/vision/instance_crop.cpp` verbatim into `reusex::core` (same logic, namespace `reusex::core`). Then add the two new functions:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "core/frame_projection.hpp"

#include "core/ProjectDB.hpp"
#include "core/instance_evidence.hpp"

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace reusex::core {

std::optional<ProjectedBox>
project_points_to_box(const std::vector<Eigen::Vector3f> &world_points,
                      const std::array<double, 16> &pose,
                      const SensorIntrinsics &intrinsics, int image_width,
                      int image_height) {
  // … the exact body of the former vision::project_points_to_box …
}

std::optional<ViewRegion>
project_instance_bbox(const ProjectDB &db, int frame_id,
                      const std::vector<Eigen::Vector3f> &points,
                      double padding) {
  if (points.empty() || !db.has_sensor_frame_pose(frame_id))
    return std::nullopt;
  const auto intr = db.sensor_frame_intrinsics(frame_id);
  if (intr.width <= 0 || intr.height <= 0)
    return std::nullopt;
  const auto box = project_points_to_box(points, db.sensor_frame_pose(frame_id),
                                         intr, intr.width, intr.height);
  if (!box)
    return std::nullopt;
  const double w = intr.width;
  const double h = intr.height;
  return pad_region({box->x_min / w, box->y_min / h,
                     (box->x_max - box->x_min + 1) / w,
                     (box->y_max - box->y_min + 1) / h},
                    padding);
}

std::map<std::uint32_t, std::vector<Eigen::Vector3f>>
instance_points(const ProjectDB &db, std::string_view label_cloud,
                std::string_view positions_cloud) {
  if (!db.has_point_cloud(label_cloud))
    throw std::invalid_argument("instance_points: no point cloud '" +
                                std::string(label_cloud) + "'");
  if (!db.has_point_cloud(positions_cloud))
    throw std::invalid_argument("instance_points: no positions cloud '" +
                                std::string(positions_cloud) + "'");
  const auto positions = db.point_cloud_xyzrgb(positions_cloud);
  const auto labels = db.point_cloud_label(label_cloud);
  if (!positions || !labels)
    throw std::runtime_error("instance_points: could not load '" +
                             std::string(positions_cloud) + "' or '" +
                             std::string(label_cloud) + "'");
  if (positions->size() != labels->size())
    throw CloudMisalignedError(
        "instance-label cloud '" + std::string(label_cloud) + "' has " +
        std::to_string(labels->size()) + " points but base cloud '" +
        std::string(positions_cloud) + "' has " +
        std::to_string(positions->size()) + " — they are not index-aligned");
  std::map<std::uint32_t, std::vector<Eigen::Vector3f>> out;
  for (std::size_t i = 0; i < labels->size(); ++i) {
    const auto label = labels->points[i].label;
    if (label == 0)
      continue; // unlabeled (STANDARDS §3)
    const auto &p = positions->points[i];
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
      continue;
    out[label].emplace_back(p.x, p.y, p.z);
  }
  return out;
}

} // namespace reusex::core
```

- [ ] **Step 5: Turn the vision header into aliases and delete the vision source**

Replace the body of `libs/reusex/include/vision/instance_crop.hpp` (keep the SPDX header and `#pragma once`) with:

```cpp
// The pinhole bbox projection now lives in core (core/frame_projection.hpp)
// so sync_survey and apply_mask_selection can use it; vision keeps its names.

#include "reusex/core/frame_projection.hpp"

namespace reusex::vision {
using core::ProjectedBox;
using core::project_points_to_box;
} // namespace reusex::vision
```

```bash
git rm libs/reusex/src/vision/instance_crop.cpp
```

- [ ] **Step 6: Build both test binaries and run**

Run: `cmake --build build --target reusex_unit_tests reusex_unit_tests_vision --parallel && ctest --test-dir build -R 'ProjectPointsToBox|InstancePoints|ProjectInstanceBbox' --output-on-failure --parallel "$(nproc)"`
Expected: 9 tests PASS. `src/vision/describe.cpp` compiles through the aliases. (A re-configure picks up the deleted file: `CONFIGURE_DEPENDS`.)

- [ ] **Step 7: Commit**

```bash
git add -A libs/reusex/include/core/frame_projection.hpp libs/reusex/src/core/frame_projection.cpp libs/reusex/include/vision/instance_crop.hpp libs/reusex/src/vision tests/unit/core/test_frame_projection.cpp tests/unit/vision
git commit -m "core: move the instance bbox projection from vision to core

vision may not be linked from core (STANDARDS §1); vision keeps aliases.
Adds project_instance_bbox (padded, normalised) and instance_points.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

### Task 1.3: Schema v27 — `survey_parts.view_*`, the setter and the reader

**Files:**
- Modify: `libs/reusex/include/core/ProjectDB.hpp` (include `resource_view.hpp`; `ResourceView` alias; `SurveyPartRecord::view`; `has_panoramic_image(int)`; `set_survey_part_view`)
- Modify: `libs/reusex/src/core/ProjectDB.cpp`: `Impl` flags at `:402-410`, `configure()` at `:527-528`, `runMigrations` at `:814-816`, `migrateToV26` neighbour at `:2421`, `survey_part_select`/`read_survey_part` at `:8909-8946`, `survey_parts()`/`survey_part()` at `:9001-9024`
- Modify: `tests/unit/core/test_project_db_migration_v26.cpp:67-68,101` (stop pinning the latest version to 26)
- Create: `tests/unit/core/test_project_db_migration_v27.cpp`
- Create: `tests/unit/core/test_project_db_survey_view.cpp`

**Interfaces:**
- Consumes: `ResourceView`, `ViewKind`, `view_kind_from_string`, `validate_view_shape`, `kViewRegionEpsilon` (Task 1.1).
- Produces: `ProjectDB::ResourceView`, `SurveyPartRecord::view`, `ProjectDB::has_panoramic_image(int) const`, `ProjectDB::set_survey_part_view(std::string_view, const std::optional<core::ResourceView>&)`. A dangling view reads as `nullopt` and logs `warn`.

- [ ] **Step 1: Write the failing migration test** `tests/unit/core/test_project_db_migration_v27.cpp`

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
// Schema v26 -> v27 (resource view record): six nullable survey_parts.view_*
// columns with a CHECK on the invariant. A fresh project is rolled back to
// v26 with raw SQL, then reopened read-write so migrateToV27 runs on real
// v26 data, and read-only to check a stale project still reads.

#include <catch2/catch_test_macros.hpp>

#include <core/ProjectDB.hpp>

#include "../../support/survey_fixture.hpp"
#include "../../support/temp_path.hpp"

#include <sqlite3.h>

#include <filesystem>
#include <string>

using reusex::ProjectDB;
namespace fs = std::filesystem;
using namespace reusex::test_support;

namespace {
struct TempDB : TempPath {
  TempDB() : TempPath("test_migration_v27") {}
};

int exec_rc(const fs::path &path, const std::string &sql) {
  sqlite3 *raw = nullptr;
  REQUIRE(sqlite3_open(path.string().c_str(), &raw) == SQLITE_OK);
  const int rc = sqlite3_exec(raw, sql.c_str(), nullptr, nullptr, nullptr);
  sqlite3_close(raw);
  return rc;
}

/// view_h first: its CHECK names the other view columns, and SQLite refuses
/// to drop a column another column's CHECK still names.
void roll_back_to_v26(const fs::path &path) {
  REQUIRE(exec_rc(path, "ALTER TABLE survey_parts DROP COLUMN view_h;"
                        "ALTER TABLE survey_parts DROP COLUMN view_w;"
                        "ALTER TABLE survey_parts DROP COLUMN view_y;"
                        "ALTER TABLE survey_parts DROP COLUMN view_x;"
                        "ALTER TABLE survey_parts DROP COLUMN view_id;"
                        "ALTER TABLE survey_parts DROP COLUMN view_kind;"
                        "DELETE FROM schema_version WHERE version >= 27;") ==
          SQLITE_OK);
}
} // namespace

TEST_CASE("MigrationV27_FreshProject_HasViewColumnsAndCheck",
          "[ProjectDB][migration]") {
  TempDB tmp;
  {
    ProjectDB db(tmp.path);
    CHECK(db.schema_version() == 27);
    CHECK(ProjectDB::latest_schema_version() == 27);
    const auto t = make_type(db, "Døre");
    make_part(db, "RX-001", t);
    CHECK_FALSE(db.survey_part("RX-001")->view);
  }
  // The invariant holds in the schema, not only in the setter.
  CHECK(exec_rc(tmp.path, "UPDATE survey_parts SET view_kind = 'frame';") !=
        SQLITE_OK); // kind without id
  CHECK(exec_rc(tmp.path, "UPDATE survey_parts SET view_kind = 'photo', "
                          "view_id = 1;") != SQLITE_OK);
  CHECK(exec_rc(tmp.path, "UPDATE survey_parts SET view_kind = 'frame', "
                          "view_id = 1, view_x = 0.1;") !=
        SQLITE_OK); // partial region
  CHECK(exec_rc(tmp.path,
                "UPDATE survey_parts SET view_kind = 'frame', view_id = 1, "
                "view_x = 0, view_y = 0.6, view_w = 0.5, view_h = 0.5;") !=
        SQLITE_OK); // y + h > 1
  CHECK(exec_rc(tmp.path,
                "UPDATE survey_parts SET view_kind = 'frame', view_id = 1, "
                "view_x = 0.1, view_y = 0.1, view_w = 0.5, view_h = 0.5;") ==
        SQLITE_OK);
}

TEST_CASE("MigrationV27_FromV26_KeepsPartsWithNullViews",
          "[ProjectDB][migration]") {
  TempDB tmp;
  {
    ProjectDB db(tmp.path);
    make_instance_cloud(db, 1);
    const auto t = make_type(db, "Døre");
    make_part(db, "RX-001", t, 1u, 3.0);
  }
  roll_back_to_v26(tmp.path);
  {
    ProjectDB db(tmp.path);
    CHECK(db.schema_version() == 27);
    const auto p = db.survey_part("RX-001");
    REQUIRE(p);
    CHECK(p->quantity == 3.0);
    CHECK(p->instance_guid == "guid-inst-1");
    CHECK_FALSE(p->view);
  }
}

TEST_CASE("MigrationV27_ReadOnlyV26_ReadsWithoutViews",
          "[ProjectDB][migration]") {
  TempDB tmp;
  {
    ProjectDB db(tmp.path);
    const auto t = make_type(db, "Døre");
    make_part(db, "RX-001", t);
  }
  roll_back_to_v26(tmp.path);
  ProjectDB db(tmp.path, /*readOnly=*/true);
  CHECK(db.schema_version() == 26);
  const auto parts = db.survey_parts();
  REQUIRE(parts.size() == 1);
  CHECK_FALSE(parts[0].view);
}
```

- [ ] **Step 2: Write the failing setter/reader test** `tests/unit/core/test_project_db_survey_view.cpp`

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
// ProjectDB::set_survey_part_view and the view a survey part reads back
// (spec 2026-10-13-resource-view-record §2): validation, existence of the
// source, clearing, and a dangling view reading as none.

#include <catch2/catch_test_macros.hpp>

#include <core/ProjectDB.hpp>
#include <core/logging.hpp>
#include <core/resource_view.hpp>

#include "../../support/survey_fixture.hpp"
#include "../../support/temp_path.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <algorithm>
#include <stdexcept>
#include <string>
#include <vector>

using reusex::ProjectDB;
using namespace reusex::core;
using namespace reusex::test_support;

namespace {
struct TempDB : TempPath {
  TempDB() : TempPath("test_survey_view") {}
};
std::vector<std::uint8_t> tiny_jpeg() {
  std::vector<std::uint8_t> out;
  REQUIRE(cv::imencode(".jpg", cv::Mat(8, 16, CV_8UC3, cv::Scalar(1, 2, 3)), out));
  return out;
}
/// A part RX-001, sensor frame 5 and panorama 1.
void seed(ProjectDB &db) {
  make_part(db, "RX-001", make_type(db, "Døre"));
  db.save_sensor_frame(5, cv::Mat(8, 8, CV_8UC3, cv::Scalar(1, 2, 3)));
  db.save_panoramic_image("a.jpg", tiny_jpeg());
}
struct WarnCapture {
  std::vector<std::string> lines;
  LogLevel saved = get_log_level();
  WarnCapture() {
    set_log_handler([this](LogLevel l, std::string_view m) {
      if (l == LogLevel::warn)
        lines.emplace_back(m);
    });
    set_log_level(LogLevel::info);
  }
  ~WarnCapture() {
    reset_log_handler();
    set_log_level(saved);
  }
};
} // namespace

TEST_CASE("SetSurveyPartView_FrameAndPanorama_RoundTripAndClear",
          "[ProjectDB][survey][view]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  seed(db);
  const ResourceView frame{ViewKind::frame, 5, ViewRegion{0.1, 0.2, 0.3, 0.4}};
  db.set_survey_part_view("RX-001", frame);
  CHECK(db.survey_part("RX-001")->view == frame);
  CHECK(db.survey_parts().at(0).view == frame);
  const ResourceView pano{ViewKind::panorama, 1, ViewRegion{0.9, 0.3, 0.2, 0.25}};
  db.set_survey_part_view("RX-001", pano); // wraps the seam: allowed
  CHECK(db.survey_part("RX-001")->view == pano);
  db.set_survey_part_view("RX-001", ResourceView{ViewKind::frame, 5, std::nullopt});
  CHECK_FALSE(db.survey_part("RX-001")->view->region);
  db.set_survey_part_view("RX-001", std::nullopt);
  CHECK_FALSE(db.survey_part("RX-001")->view);
}

TEST_CASE("SetSurveyPartView_BadInput_ThrowsAndWritesNothing",
          "[ProjectDB][survey][view]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  seed(db);
  CHECK_THROWS_AS(db.set_survey_part_view("RX-404", ResourceView{ViewKind::frame, 5, std::nullopt}),
                  std::out_of_range);
  CHECK_THROWS_AS(db.set_survey_part_view("RX-001", ResourceView{ViewKind::frame, 6, std::nullopt}),
                  std::out_of_range); // no such frame
  CHECK_THROWS_AS(db.set_survey_part_view("RX-001", ResourceView{ViewKind::panorama, 9, std::nullopt}),
                  std::out_of_range); // no such panorama
  CHECK_THROWS_AS(db.set_survey_part_view("RX-001", ResourceView{ViewKind::frame, 5, ViewRegion{0.9, 0, 0.2, 0.5}}),
                  std::invalid_argument); // a frame cannot wrap
  CHECK_THROWS_AS(db.set_survey_part_view("RX-001", ResourceView{ViewKind::frame, 5, ViewRegion{0, 0, 0, 0.5}}),
                  std::invalid_argument);
  CHECK_FALSE(db.survey_part("RX-001")->view);
}

TEST_CASE("SetSurveyPartView_RoundedEdgeRegion_IsAccepted",
          "[ProjectDB][survey][view]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  seed(db);
  const auto r = pad_region({0.1, 0.7, 0.5, 0.3}); // clamped at the bottom
  CHECK_NOTHROW(db.set_survey_part_view("RX-001", ResourceView{ViewKind::frame, 5, r}));
  CHECK_NOTHROW(db.set_survey_part_view(
      "RX-001", ResourceView{ViewKind::frame, 5, ViewRegion{0.0, 0.3, 1.0, 0.7000000000000001}}));
}

TEST_CASE("SurveyPartView_DanglingPanorama_ReadsNullAndWarns",
          "[ProjectDB][survey][view]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  seed(db);
  db.set_survey_part_view("RX-001", ResourceView{ViewKind::panorama, 1, std::nullopt});
  db.delete_panoramic_image(1);
  WarnCapture warns;
  CHECK_FALSE(db.survey_part("RX-001")->view);
  REQUIRE(warns.lines.size() == 1);
  CHECK(warns.lines[0].find("RX-001") != std::string::npos);
}

TEST_CASE("HasPanoramicImage_ById_TrueOnlyForStoredIds", "[ProjectDB][panorama]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  seed(db);
  CHECK(db.has_panoramic_image(1));
  CHECK_FALSE(db.has_panoramic_image(2));
  CHECK(db.has_panoramic_image("a.jpg")); // the filename overload still works
}
```

- [ ] **Step 3: Stop the v26 test pinning the latest version**

In `tests/unit/core/test_project_db_migration_v26.cpp`, replace lines 67-68:

```cpp
    CHECK(db.schema_version() == 26);
    CHECK(ProjectDB::latest_schema_version() == 26);
```
with
```cpp
    CHECK(db.schema_version() == ProjectDB::latest_schema_version());
```
Replace line 101 `CHECK(db.schema_version() == 26);` with `CHECK(db.schema_version() == ProjectDB::latest_schema_version());`.

- [ ] **Step 4: Run and confirm failure**

Run: `cmake --build build --target reusex_unit_tests --parallel`
Expected: compile errors (`'struct SurveyPartRecord' has no member named 'view'`, no `set_survey_part_view`).

- [ ] **Step 5: Header changes in `libs/reusex/include/core/ProjectDB.hpp`**

Add `#include "reusex/core/resource_view.hpp"` after `#include "reusex/core/component_record.hpp"`.

Inside `class ProjectDB { public:` (next to the other survey structs, before `struct SurveyTypeRecord`):

```cpp
  /// The chosen image of a resource (schema v27, core/resource_view.hpp).
  using ResourceView = core::ResourceView;
```

In `struct SurveyPartRecord`, after `std::optional<std::string> instance_guid{};`, and update that member's comment from "Kept LAST" to "Kept before view":

```cpp
    /// The resource's chosen image (schema v27: survey_parts.view_*).
    /// nullopt when none was chosen, before v27, or when the frame/panorama
    /// it names no longer exists (logged at warn). Read-only on add/patch:
    /// set through set_survey_part_view. Kept LAST so positional aggregate
    /// initializers still compile.
    std::optional<core::ResourceView> view{};
```

After `bool has_panoramic_image(std::string_view filename) const;`:

```cpp
  /// True when a panoramic_images row with this id exists.
  bool has_panoramic_image(int id) const;
```

After `void delete_survey_part(std::string_view code);`:

```cpp
  /// Set or clear (nullopt) a part's view record (schema v27). Validates the
  /// shape (core::validate_view_shape) and that the frame / panorama exists.
  /// Does not open a transaction (one UPDATE); call inside one to batch.
  /// @throws std::invalid_argument (shape), std::out_of_range (no part, no
  ///         sensor frame, no panorama).
  void set_survey_part_view(std::string_view code,
                            const std::optional<core::ResourceView> &view);
```

- [ ] **Step 6: Migration in `libs/reusex/src/core/ProjectDB.cpp`**

In `class ProjectDB::Impl`, after `bool hasDismissedTable = false;`:

```cpp
  /// survey_parts.view_* exist (schema v27); same lifetime rule.
  bool hasViewColumns = false;
```

Change `static constexpr int LATEST_SCHEMA_VERSION = 26;` to `= 27;`.

In `configure()`, after `hasDismissedTable = tableExists("survey_dismissed_instances");`:

```cpp
    hasViewColumns = columnExists("survey_parts", "view_h");
```

In `runMigrations`, after the `if (current < 26) { migrateToV26(); }` block:

```cpp
    if (current < 27) {
      migrateToV27();
    }
```

After `migrateToV26()`:

```cpp
  void migrateToV27() {
    reusex::info("Migrating database to schema version 27");
    // Resource view record (docs/superpowers/specs/
    // 2026-10-13-resource-view-record-design.md §2): the chosen image of a
    // survey part. ADD COLUMN cannot add a table constraint, so the
    // cross-column invariant is a column CHECK on view_h, the last column
    // added, when every column it names exists. Each step is guarded so a
    // test that rolls the version back re-runs it safely. Existing rows stay
    // NULL, which the CHECK accepts. 1e-9 = core::kViewRegionEpsilon.
    static constexpr const char *kViewH =
        "REAL CHECK ((view_kind IS NULL) = (view_id IS NULL) "
        "AND (view_kind IS NOT NULL OR view_x IS NULL) "
        "AND ((view_x IS NULL AND view_y IS NULL AND view_w IS NULL "
        "AND view_h IS NULL) OR (view_x IS NOT NULL AND view_y IS NOT NULL "
        "AND view_w IS NOT NULL AND view_h IS NOT NULL AND view_w > 0 "
        "AND view_h > 0 AND view_x >= 0 AND view_y >= 0 "
        "AND view_w <= 1.000000001 AND view_y + view_h <= 1.000000001)))";
    const std::pair<const char *, const char *> columns[] = {
        {"view_kind", "TEXT CHECK (view_kind IN ('frame','panorama'))"},
        {"view_id", "INTEGER"},
        {"view_x", "REAL"},
        {"view_y", "REAL"},
        {"view_w", "REAL"},
        {"view_h", kViewH}};
    execOrThrow("BEGIN TRANSACTION;");
    try {
      for (const auto &[name, decl] : columns)
        if (!columnExists("survey_parts", name))
          execOrThrow((std::string("ALTER TABLE survey_parts ADD COLUMN ") +
                       name + " " + decl + ";")
                          .c_str());
      insertSchemaVersion(27, "Resource view record: survey_parts.view_* "
                              "(sys:view, the chosen image of a resource)");
      execOrThrow("COMMIT;");
    } catch (...) {
      if (sqlite3_exec(db, "ROLLBACK;", nullptr, nullptr, nullptr) != SQLITE_OK)
        reusex::warn("migrateToV27: ROLLBACK failed: {}", sqlite3_errmsg(db));
      throw;
    }
    reusex::info("Migration to schema version 27 complete");
  }
```

> If SQLite refuses the cross-column CHECK in `ADD COLUMN` (check the test in Step 1), keep only the per-column `view_kind` CHECK. Enforce the rest in the setter. Report it as a deviation in the PR. Do not silently drop it.

- [ ] **Step 7: Reader (`survey_part_select` / `read_survey_part`)**

Replace `survey_part_select` and `read_survey_part` (`ProjectDB.cpp:8909-8946`) with:

```cpp
std::string survey_part_select(bool has_passport_column, bool has_view_columns) {
  std::string sql = "SELECT p.code, p.type_id, pc.name, i.instance_id, "
                    "p.room_id, p.room_name, p.quantity, p.starred, p.note, ";
  sql += has_passport_column
             ? "COALESCE(p.passport_guid, CASE WHEN NOT EXISTS (SELECT 1 "
               "FROM survey_parts o WHERE o.passport_guid = "
               "im.material_guid) THEN im.material_guid END)"
             : "im.material_guid";
  sql += ", p.instance_guid";
  // Columns 11-17: the view record, and whether its source still exists
  // (a frame or panorama deleted after the choice reads as no view).
  if (has_view_columns)
    sql += ", p.view_kind, p.view_id, p.view_x, p.view_y, p.view_w, "
           "p.view_h, CASE p.view_kind WHEN 'frame' THEN EXISTS (SELECT 1 "
           "FROM sensor_frames f WHERE f.node_id = p.view_id) "
           "WHEN 'panorama' THEN EXISTS (SELECT 1 FROM panoramic_images pi "
           "WHERE pi.id = p.view_id) ELSE 0 END";
  sql += " FROM survey_parts p "
         "LEFT JOIN instances i ON i.guid = p.instance_guid "
         "LEFT JOIN point_clouds pc ON pc.id = i.cloud_id "
         "LEFT JOIN instance_materials im ON im.cloud_id = i.cloud_id AND "
         "im.instance_id = i.instance_id ";
  return sql;
}

ProjectDB::SurveyPartRecord read_survey_part(sqlite3_stmt *s,
                                             bool has_view_columns) {
  ProjectDB::SurveyPartRecord r;
  // … existing reads of columns 0-10, unchanged …
  if (has_view_columns && sqlite3_column_type(s, 11) != SQLITE_NULL) {
    const std::string kind_text = column_text(s, 11);
    const int id = sqlite3_column_int(s, 12);
    const auto kind = core::view_kind_from_string(kind_text);
    if (!kind || sqlite3_column_int(s, 17) == 0) {
      // STANDARDS §5: a dangling view is not silently dropped.
      reusex::warn("survey part {}: its view ({} #{}) no longer exists; read "
                   "as no image",
                   r.code, kind_text, id);
    } else {
      core::ResourceView v{*kind, id, std::nullopt};
      if (sqlite3_column_type(s, 13) != SQLITE_NULL)
        v.region = core::ViewRegion{
            sqlite3_column_double(s, 13), sqlite3_column_double(s, 14),
            sqlite3_column_double(s, 15), sqlite3_column_double(s, 16)};
      r.view = v;
    }
  }
  return r;
}
```

Update both callers, `survey_parts()` and `survey_part()`: `survey_part_select(impl_->hasPassportColumn, impl_->hasViewColumns)` and `read_survey_part(stmt, impl_->hasViewColumns)`.

- [ ] **Step 8: `has_panoramic_image(int)` and `set_survey_part_view`**

Next to `ProjectDB::has_panoramic_image(std::string_view)` (`:6986`):

```cpp
bool ProjectDB::has_panoramic_image(int id) const {
  sqlite3_stmt *stmt = prepare_or_throw(
      impl_->db, "SELECT 1 FROM panoramic_images WHERE id = ?;",
      "has_panoramic_image");
  StmtGuard guard(stmt);
  sqlite3_bind_int(stmt, 1, id);
  return sqlite3_step(stmt) == SQLITE_ROW;
}
```

After `ProjectDB::delete_survey_part`:

```cpp
void ProjectDB::set_survey_part_view(
    std::string_view code, const std::optional<core::ResourceView> &view) {
  impl_->checkWritable();
  if (view)
    core::validate_view_shape(*view);
  if (!survey_part(code))
    throw std::out_of_range("no survey part '" + std::string(code) + "'");
  if (view) {
    const bool frame = view->kind == core::ViewKind::frame;
    if (frame ? !has_sensor_frame(view->id) : !has_panoramic_image(view->id))
      throw std::out_of_range(std::string(frame ? "no sensor frame " : "no panorama ") +
                              std::to_string(view->id));
  }
  sqlite3_stmt *stmt = prepare_or_throw(
      impl_->db,
      "UPDATE survey_parts SET view_kind = ?, view_id = ?, view_x = ?, "
      "view_y = ?, view_w = ?, view_h = ? WHERE code = ?;",
      "set_survey_part_view");
  StmtGuard guard(stmt);
  for (int i = 1; i <= 6; ++i)
    sqlite3_bind_null(stmt, i);
  if (view) {
    bind_text(stmt, 1, core::to_string(view->kind));
    sqlite3_bind_int(stmt, 2, view->id);
    if (view->region)
      for (int i = 0; i < 4; ++i)
        sqlite3_bind_double(stmt, 3 + i, (*view->region)[i]);
  }
  bind_text(stmt, 7, code);
  if (sqlite3_step(stmt) != SQLITE_DONE)
    throw std::runtime_error("set_survey_part_view '" + std::string(code) +
                             "': " + sqlite3_errmsg(impl_->db));
}
```

- [ ] **Step 9: Run the tests and confirm they pass**

Run: `cmake --build build --target reusex_unit_tests --parallel && ctest --test-dir build -R 'MigrationV2[67]|SetSurveyPartView|SurveyPartView|HasPanoramicImage|ProjectDB.*[Ss]urvey' --output-on-failure --parallel "$(nproc)"`
Expected: all PASS, including the existing v25/v26 migration tests.

- [ ] **Step 10: Commit**

```bash
git add libs/reusex/include/core/ProjectDB.hpp libs/reusex/src/core/ProjectDB.cpp tests/unit/core/test_project_db_migration_v26.cpp tests/unit/core/test_project_db_migration_v27.cpp tests/unit/core/test_project_db_survey_view.cpp
git commit -m "core: schema v27 — survey_parts view record, setter and reader

Additive view_* columns with a CHECK on the invariant; a dangling view
reads as none and is logged at warn.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

### Task 1.4: Resource key `sys:view` — keys, list, patch, templates, CSV, report table

**Files:**
- Modify: `libs/reusex/include/core/resource_keys.hpp` (`kViewKey`; the `builtin_keys()` comment says 11 → 12)
- Modify: `libs/reusex/src/core/resource_keys.cpp` (`builtin_keys()`, `normalise_value()`)
- Modify: `libs/reusex/include/core/resources.hpp` (`Resource::view`)
- Modify: `libs/reusex/src/core/resources.cpp` (`builtin_value`, `build`, `patch_resource`)
- Modify: `libs/reusex/src/core/resource_templates.cpp:97` (`seed_templates`: the screening seed leaves out `sys:view`)
- Modify: `libs/reusex/src/core/resource_export.cpp` (`display_value`, `resource_tables`)
- Test: `tests/unit/core/test_resource_keys.cpp`, `tests/unit/core/test_resources.cpp`, `tests/unit/core/test_resource_templates.cpp`, `tests/unit/core/test_resource_template_service.cpp:67,81`, `tests/unit/core/test_resource_export.cpp`

**Interfaces:**
- Consumes: `parse_view`, `view_to_json`, `view_display` (1.1); `SurveyPartRecord::view`, `set_survey_part_view`, `has_panoramic_image(int)`, `has_sensor_frame` (1.3).
- Produces: `kViewKey`. The 12th built-in key `sys:view` (`data_type "view"`). `Resource::view` is filled for every resource whatever `keys` is. `patch_resource` accepts `sys:view` (shape error or missing source → `KeyValueError`, nothing written). `display_value(sys:view)` gives `Foto #n` / `360° #n`. `resource_tables` never has a `sys:view` column. The screening seed has the 11 old keys.

- [ ] **Step 1: Write the failing tests**

`tests/unit/core/test_resource_keys.cpp`, in `ResourceKeys_Builtins_MatchTheSpecTable`: append `"sys:view"` to the expected id list (after `"sys:starred"`) and add:

```cpp
  CHECK(keys[11].label == "Ressourcebillede");
  CHECK(keys[11].scope == KeyScope::part);
  CHECK(keys[11].data_type == "view");
  CHECK(keys[11].editable);
```

and add a new test case:

```cpp
TEST_CASE("NormaliseValue_ViewKey_CanonicalisesAndRefusesBadShapes",
          "[resources][keys]") {
  const auto keys = builtin_keys();
  const auto &k = *find_key(keys, "sys:view");
  CHECK(normalise_value(k, std::string(R"({"id":3,"kind":"frame"})")) ==
        R"({"kind":"frame","id":3,"region":null})");
  CHECK_FALSE(normalise_value(k, std::string("")).has_value());
  CHECK_FALSE(normalise_value(k, std::nullopt).has_value());
  CHECK_THROWS_AS(normalise_value(k, std::string("Foto #3")), KeyValueError);
  CHECK_THROWS_AS(
      normalise_value(k, std::string(R"({"kind":"frame","id":3,"region":[0,0.9,0.5,0.5]})")),
      KeyValueError);
}
```

`tests/unit/core/test_resources.cpp`: add `#include <opencv2/core.hpp>` and:

```cpp
TEST_CASE("Resources_Patch_View_RoundTripsChecksSourceAndClears",
          "[resources][view]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto t = make_type(db, "Døre");
  make_part(db, "RX-001", t);
  db.save_sensor_frame(7, cv::Mat(8, 8, CV_8UC3, cv::Scalar(1, 2, 3)));
  // Unset: the key is listed (null) and Resource::view is empty.
  const auto before = resource(db, "RX-001");
  CHECK_FALSE(before.view);
  CHECK(std::any_of(before.values.begin(), before.values.end(),
                    [](const auto &v) { return v.key == "sys:view" && !v.value; }));

  const std::string in = R"({"region":[0.1,0.2,0.3,0.4],"kind":"frame","id":7})";
  const auto out = patch_resource(db, "RX-001", {{"sys:view", in}});
  CHECK(value_of(out.resource, "sys:view") ==
        R"({"kind":"frame","id":7,"region":[0.1,0.2,0.3,0.4]})");
  REQUIRE(out.resource.view);
  CHECK(out.resource.view->id == 7);
  CHECK(out.siblings.empty()); // part-scoped
  CHECK(db.survey_part("RX-001")->view == out.resource.view);
  // A keys filter without sys:view still carries Resource::view.
  CHECK(resource(db, "RX-001", std::vector<std::string>{"sys:name"}).view);

  // No such frame / bad shape: KeyValueError, nothing written.
  CHECK_THROWS_AS(patch_resource(db, "RX-001", {{"sys:note", "x"}, {"sys:view", R"({"kind":"frame","id":8})"}}),
                  KeyValueError);
  CHECK_THROWS_AS(patch_resource(db, "RX-001", {{"sys:view", R"({"kind":"panorama","id":1})"}}),
                  KeyValueError);
  CHECK_THROWS_AS(patch_resource(db, "RX-001", {{"sys:view", "nope"}}), KeyValueError);
  CHECK(db.survey_part("RX-001")->note.empty());
  CHECK(db.survey_part("RX-001")->view->id == 7);

  patch_resource(db, "RX-001", {{"sys:view", std::nullopt}});
  CHECK_FALSE(db.survey_part("RX-001")->view);
}
```

`tests/unit/core/test_resource_templates.cpp:90`: change the screening expectation loop to skip `sys:view`:

```cpp
  for (const auto &k : builtin_keys())
    if (k.id != kViewKey)
      screening.push_back(key(k.id));
```

`tests/unit/core/test_resource_template_service.cpp:67` and `:81`: `CHECK(v.resolved.keys.size() == 11);` → `== 12` and `CHECK(edited.resolved.keys.size() == 11);` → `== 12` (the Kortlægning category now holds `sys:view`).

`tests/unit/core/test_resource_export.cpp`, new case:

```cpp
TEST_CASE("ResourceExport_ViewKey_DisplayedInCsvLeftOutOfReportTables",
          "[resources][csv][pdf]") {
  const auto cat = key_catalogue({});
  const auto &view = *find_key(cat, "sys:view");
  CHECK(display_value(view, std::string(R"({"kind":"frame","id":12,"region":null})")) == "Foto #12");
  CHECK(display_value(view, std::string(R"({"kind":"panorama","id":4,"region":null})")) == "360° #4");
  CHECK(display_value(view, std::string("garbage")) == "garbage");
  CHECK(display_value(view, std::nullopt).empty());
  std::vector<ResourceKey> cols{*find_key(cat, "sys:name"), view,
                                *find_key(cat, "sys:note")};
  Resource r{"RX-007", 1, false, {{"sys:name", "Døre"}, {"sys:view", R"({"kind":"frame","id":12,"region":null})"}, {"sys:note", "n"}}};
  const auto tables = resource_tables(cols, {r});
  REQUIRE(tables.size() == 1);
  CHECK(tables[0].headers == std::vector<std::string>{"Betegnelse", "Note"});
}
```

- [ ] **Step 2: Run and confirm failure**

Run: `cmake --build build --target reusex_unit_tests --parallel && ctest --test-dir build -R 'ResourceKeys|NormaliseValue_View|Resources_Patch_View|ResourceTemplates|TemplateService|ResourceExport' --output-on-failure --parallel "$(nproc)"`
Expected: FAIL. The builtin list does not match, `kViewKey` is undeclared, and `Resource` has no member `view`.

- [ ] **Step 3: Keys** (`resource_keys.hpp` / `.cpp`)

In `resource_keys.hpp`, after `kColumnCategory`:

```cpp
/// The resource view record key (schema v27, core/resource_view.hpp).
inline constexpr std::string_view kViewKey = "sys:view";
```
Change the `builtin_keys()` comment to `///< the 12 sys: keys, spec order (sys:view last)`. Add `#include "reusex/core/resource_view.hpp"`. In the `normalise_value` doc, add: "a view (sys:view) must be a parse_view-able JSON object, stored canonical (view_to_json)".

In `builtin_keys()` (`resource_keys.cpp`), append to the initializer list after `k("starred", …)`:

```cpp
      k("view", "Ressourcebillede", KeyScope::part, "view"),
```

In `normalise_value`, before the `if (key.data_type == "date")` block:

```cpp
  if (key.data_type == "view") {
    try {
      return view_to_json(parse_view(v));
    } catch (const std::invalid_argument &e) {
      throw KeyValueError(key.id, "'" + key.id + "' " + e.what() + ", got '" + v + "'");
    }
  }
```

- [ ] **Step 4: Resources** (`resources.hpp` / `.cpp`)

`resources.hpp`, in `struct Resource`, after `values`:

```cpp
  /// The part's view record (schema v27), filled whatever `keys` selects —
  /// the GUI needs it even when a template leaves sys:view out.
  std::optional<ResourceView> view;
```

`resources.cpp`:
- `builtin_value`: before the final `return std::nullopt;` add
  ```cpp
  if (f == "view")
    return p.view ? std::optional<std::string>(view_to_json(*p.view)) : std::nullopt;
  ```
- `build`: after `Resource r{…};` add `r.view = p.view;`.
- `patch_resource`, validation loop: replace `plan.push_back({k, normalise_value(*k, w.value)});` with
  ```cpp
    auto value = normalise_value(*k, w.value);
    if (k->id == kViewKey && value) {
      // The setter checks this too, but inside the write transaction; refuse
      // here so a bad view writes nothing (validate first, write second).
      const auto view = parse_view(*value);
      const bool frame = view.kind == ViewKind::frame;
      if (frame ? !db.has_sensor_frame(view.id) : !db.has_panoramic_image(view.id))
        throw KeyValueError(k->id, "'" + k->id + "' names " +
                                       (frame ? "sensor frame " : "panorama ") +
                                       std::to_string(view.id) +
                                       ", which does not exist");
    }
    plan.push_back({k, std::move(value)});
  ```
- `patch_resource`, routing: declare `std::optional<std::optional<ResourceView>> view_write;` next to `ProjectDB::SurveyPartPatch pp;`. In the built-in `if/else` chain add
  ```cpp
    else if (f == "view")
      view_write = v ? std::optional<ResourceView>(parse_view(*v)) : std::nullopt;
  ```
- `patch_resource`, inside the transaction after `db.update_survey_part(code, pp);`:
  ```cpp
    if (view_write)
      db.set_survey_part_view(code, *view_write);
  ```

- [ ] **Step 5: Seed, CSV display, report tables**

`resource_templates.cpp` `seed_templates()`:

```cpp
  // The screening seed lists the built-in keys a reader scans; the view
  // record is an image, shown by the report itself (sub-project C).
  for (const auto &k : builtin_keys())
    if (k.id != kViewKey)
      screening.members.push_back({MemberKind::key, k.id});
```

`resource_export.cpp` `display_value`, after the `sys:environment` block:

```cpp
  if (key.id == kViewKey) {
    try {
      return view_display(parse_view(*value));
    } catch (const std::invalid_argument &) {
      return *value; // not ours to hide; show what is stored
    }
  }
```

`resource_tables`: change `if (k.id != "sys:name")` to

```cpp
    // sys:name leads every table; sys:view is an image — the report shows
    // the picture (sub-project C), never "Foto #12" in a table cell.
    if (k.id != "sys:name" && k.id != kViewKey)
```

- [ ] **Step 6: Run the tests and confirm they pass**

Run: `cmake --build build --target reusex_unit_tests --parallel && ctest --test-dir build -R 'Resource|Template|NormaliseValue' --output-on-failure --parallel "$(nproc)"`
Expected: all PASS.

- [ ] **Step 7: Commit**

```bash
git add libs/reusex/include/core/resource_keys.hpp libs/reusex/src/core/resource_keys.cpp libs/reusex/include/core/resources.hpp libs/reusex/src/core/resources.cpp libs/reusex/src/core/resource_templates.cpp libs/reusex/src/core/resource_export.cpp tests/unit/core/test_resource_keys.cpp tests/unit/core/test_resources.cpp tests/unit/core/test_resource_templates.cpp tests/unit/core/test_resource_template_service.cpp tests/unit/core/test_resource_export.cpp
git commit -m "core: sys:view resource key (list, patch, CSV display; not in report tables)

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

### Task 1.5: `sync_survey` attaches the best frame as the view (and backfills)

**Files:**
- Modify: `libs/reusex/include/core/survey_service.hpp` (`SurveySyncOptions::positions_cloud`, `SurveySyncReport::views_attached/views_unavailable`, `sync_survey` doc)
- Modify: `libs/reusex/src/core/survey_service.cpp` (new `attach_views`, called from `sync_survey` before the orphan report)
- Modify: `apps/rux/src/create/survey.cpp` (print the counts)
- Test: `tests/unit/core/test_survey_service.cpp`

**Interfaces:**
- Consumes: `core::instance_evidence` (`core/instance_evidence.hpp`), `instance_points`, `project_instance_bbox` (1.2), `set_survey_part_view` (1.3).
- Produces: `SurveySyncReport::views_attached`, `views_unavailable`; `SurveySyncOptions::positions_cloud = "cloud"`.

- [ ] **Step 1: Write the failing tests** (append to `tests/unit/core/test_survey_service.cpp`)

Add the includes `#include <catch2/matchers/catch_matchers_floating_point.hpp>`, `#include <core/resource_view.hpp>` and `#include <opencv2/core.hpp>`, then:

```cpp
namespace {
reusex::core::SensorIntrinsics pinhole128() {
  reusex::core::SensorIntrinsics i;
  i.fx = i.fy = 100.0;
  i.cx = i.cy = 64.0;
  i.width = i.height = 128;
  return i;
}
/// A frame at the origin looking down +z, no depth (frustum test only).
void posed_frame(ProjectDB &db, int id) {
  db.save_sensor_frame(id, cv::Mat(128, 128, CV_8UC3, cv::Scalar(40, 90, 160)),
                       cv::Mat(), cv::Mat(),
                       {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1},
                       pinhole128(), static_cast<double>(id), -1);
}
/// Instance 1 in front of the camera (two points at z = 2), instance 2
/// behind every camera. Both class 3.
void seed_seen_scan(ProjectDB &db, bool with_positions = true) {
  reusex::Cloud positions;
  reusex::CloudL labels;
  auto add = [&](float x, float y, float z, std::uint32_t label) {
    pcl::PointXYZRGB p;
    p.x = x;
    p.y = y;
    p.z = z;
    positions.push_back(p);
    pcl::Label l;
    l.label = label;
    labels.push_back(l);
  };
  add(-0.1F, 0.0F, 2.0F, 1);
  add(0.1F, 0.0F, 2.0F, 1);
  add(0.0F, 0.0F, -10.0F, 2);
  if (with_positions)
    db.save_point_cloud("cloud", positions, "test", "{}");
  db.save_point_cloud("instances", labels, "test", "{}");
  db.save_instances("instances", {{1, "g1", 3, 2}, {2, "g2", 3, 1}});
}
} // namespace

TEST_CASE("SyncSurvey_AttachesBestFrameView_BackfillsAndNeverOverwrites",
          "[survey][sync][view]") {
  using Catch::Matchers::WithinAbs;
  TempDB tmp;
  ProjectDB db(tmp.path);
  seed_seen_scan(db);

  // No sensor frames yet: parts are created, no view anywhere.
  const auto first = sync_survey(db);
  CHECK(first.parts_created == 2);
  CHECK(first.views_attached == 0);
  CHECK(first.views_unavailable == 2);
  CHECK_FALSE(db.survey_part("RX-001")->view);

  // Frames arrive later (an old project): the next sync backfills.
  posed_frame(db, 1);
  const auto second = sync_survey(db);
  CHECK(second.parts_created == 0);
  CHECK(second.views_attached == 1);
  CHECK(second.views_unavailable == 1); // instance 2 is seen by no frame
  const auto view = db.survey_part("RX-001")->view;
  REQUIRE(view);
  CHECK(view->kind == ViewKind::frame);
  CHECK(view->id == 1);
  REQUIRE(view->region);
  CHECK_THAT((*view->region)[0], WithinAbs(57.35 / 128.0, 1e-9));
  CHECK_THAT((*view->region)[2], WithinAbs(14.3 / 128.0, 1e-9));
  CHECK_FALSE(db.survey_part("RX-002")->view);

  // The user's choice survives a rerun.
  const ResourceView chosen{ViewKind::frame, 1, std::nullopt};
  db.set_survey_part_view("RX-001", chosen);
  const auto third = sync_survey(db);
  CHECK(third.views_attached == 0);
  CHECK(db.survey_part("RX-001")->view == chosen);
}

TEST_CASE("SyncSurvey_FramesButNoPositionsCloud_SucceedsWithoutViews",
          "[survey][sync][view]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  seed_seen_scan(db, /*with_positions=*/false);
  posed_frame(db, 1);
  const auto r = sync_survey(db);
  CHECK(r.parts_created == 2);
  CHECK(r.views_attached == 0);
  CHECK(r.views_unavailable == 2);
}
```

- [ ] **Step 2: Run and confirm failure**

Run: `cmake --build build --target reusex_unit_tests --parallel`
Expected: compile error, `'struct reusex::core::SurveySyncReport' has no member named 'views_attached'`.

- [ ] **Step 3: Header** (`survey_service.hpp`)

In `SurveySyncOptions`, after `rooms_cloud`:

```cpp
  /// Positions of the instance cloud's points (index-aligned), for placing
  /// instances in sensor frames when attaching views.
  std::string positions_cloud = "cloud";
```

In `SurveySyncReport`, after `links_restored`:

```cpp
  /// Instance-backed parts without a view that got one: the best-ranked
  /// frame (core::instance_evidence), region = the instance's padded bbox
  /// in it (spec 2026-10-13-resource-view-record §2). Never overwrites.
  std::size_t views_attached = 0;
  /// Instance-backed parts still without a view: no frame sees the
  /// instance, the project has no sensor frames, or no positions cloud.
  std::size_t views_unavailable = 0;
```

Append to the `sync_survey` doc: "Then every instance-backed part without a view (new or pre-existing: the backfill for old projects) gets the best-ranked frame as its view; see views_attached / views_unavailable."

- [ ] **Step 4: Implementation** (`survey_service.cpp`)

Add the includes `#include "reusex/core/frame_projection.hpp"`, `#include "reusex/core/instance_evidence.hpp"`, `#include "reusex/core/resource_view.hpp"` and `#include <Eigen/Core>`. In the anonymous namespace (before `sync_survey`):

```cpp
/// Attach a view to every instance-backed part of opts.instances_cloud that
/// has none: the best-ranked frame of one instance_evidence pass over all
/// parts (as ruxd's PhotoEvidenceCache ranks them), region = the instance's
/// padded bbox in it. Never overwrites a view.
void attach_views(ProjectDB &db, const SurveySyncOptions &opts,
                  SurveySyncReport &report) {
  std::vector<ProjectDB::SurveyPartRecord> pending;
  for (auto &p : db.survey_parts())
    if (!p.view && p.instance_id && p.cloud_name == opts.instances_cloud)
      pending.push_back(std::move(p));
  if (pending.empty())
    return;
  const auto unavailable = [&](std::string_view why, bool loud) {
    report.views_unavailable = pending.size();
    if (loud)
      reusex::warn("sync_survey: {}; {} part(s) get no resource image", why,
                   pending.size());
    else
      reusex::info("sync_survey: {}; {} part(s) get no resource image", why,
                   pending.size());
  };
  if (db.sensor_frame_ids().empty())
    return unavailable("the project has no sensor frames", false);
  if (!db.has_point_cloud(opts.positions_cloud))
    return unavailable("no '" + opts.positions_cloud +
                           "' cloud to place instances in frames (run `rux "
                           "create clouds`)",
                       true);
  std::map<std::uint32_t, InstanceEvidence> evidence;
  std::map<std::uint32_t, std::vector<Eigen::Vector3f>> points;
  try {
    evidence = instance_evidence(db, opts.instances_cloud, PhotoQuery{},
                                 opts.positions_cloud);
    points = instance_points(db, opts.instances_cloud, opts.positions_cloud);
  } catch (const CloudMisalignedError &e) {
    return unavailable(e.what(), true);
  }
  std::vector<std::pair<std::string, ResourceView>> writes;
  for (const auto &p : pending) {
    const auto ev = evidence.find(*p.instance_id);
    if (ev == evidence.end() || ev->second.frames.empty()) {
      ++report.views_unavailable;
      continue;
    }
    const int frame = ev->second.frames.front().frame_id;
    const auto pts = points.find(*p.instance_id);
    writes.emplace_back(
        p.code, ResourceView{ViewKind::frame, frame,
                             pts == points.end()
                                 ? std::nullopt
                                 : project_instance_bbox(db, frame, pts->second)});
  }
  if (!writes.empty()) {
    ProjectDB::Transaction tx(db);
    for (const auto &[code, view] : writes)
      db.set_survey_part_view(code, view);
    tx.commit();
  }
  report.views_attached = writes.size();
  if (report.views_attached == 0)
    reusex::warn("sync_survey: no sensor frame sees any of the {} part(s) "
                 "without a resource image",
                 report.views_unavailable);
  else
    reusex::info("sync_survey: attached {} resource image(s); {} part(s) "
                 "seen by no frame keep none",
                 report.views_attached, report.views_unavailable);
}
```

`unavailable` returns `void`. `return unavailable(...)` in a `void` function is valid C++. If a reviewer objects, call it and then `return;`.

In `sync_survey`, call `attach_views(db, opts, report);` just before the comment "Parts whose instance_guid is set but no longer resolves…".

- [ ] **Step 5: CLI output** (`apps/rux/src/create/survey.cpp`)

After `dismissed_note`:

```cpp
    const std::string views_note =
        report.views_attached + report.views_unavailable > 0
            ? ", " + std::to_string(report.views_attached) +
                  " resource image(s) attached, " +
                  std::to_string(report.views_unavailable) +
                  " part(s) without one"
            : "";
```

Add a fourth `{}` to the `spdlog::info` format string and pass `views_note` last. In the subcommand footer, add the line `  Each part without a resource image gets the sensor frame that sees it best.` under DESCRIPTION.

- [ ] **Step 6: Run the tests and confirm they pass**

Run: `cmake --build build --target reusex_unit_tests rux --parallel && ctest --test-dir build -R 'SyncSurvey' --output-on-failure --parallel "$(nproc)"`
Expected: all `SyncSurvey_*` PASS (the old ones have no frames, so they skip quietly).

- [ ] **Step 7: Smoke-test the CLI on the real fixture**

```bash
cp tests/fixtures/scans/office_corridor.rux /tmp/claude-view-smoke.rux
./build/apps/rux/rux -p /tmp/claude-view-smoke.rux create survey
```
Expected: an `info` line ending `…, N resource image(s) attached, M part(s) without one`, with N > 0 if the fixture has instances and posed frames. If the fixture has no `instances` cloud, the command reports the `rux create instances` hint, and the unit tests are the evidence. (Find the binary with `find build -name rux -type f -perm -u+x` if the path differs.)

- [ ] **Step 8: Commit**

```bash
git add libs/reusex/include/core/survey_service.hpp libs/reusex/src/core/survey_service.cpp apps/rux/src/create/survey.cpp tests/unit/core/test_survey_service.cpp
git commit -m "core: sync_survey attaches the best frame as each part's view

Backfills parts of old projects; never overwrites a chosen view.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

### Task 1.6: Mask → resource sets the new part's view

**Files:**
- Modify: `libs/reusex/include/core/mask_selection.hpp` (`mask_region`, `MaskSelectionOptions::view_region`, `MaskSelectionResult::view`, docs)
- Modify: `libs/reusex/src/core/mask_selection.cpp` (`mask_region`; early validation; set the view after `add_survey_part` at `:423`)
- Test: `tests/unit/core/test_mask_selection.cpp`

**Interfaces:**
- Consumes: `pad_region`, `validate_view_shape` (1.1), `project_instance_bbox` (1.2), `set_survey_part_view` (1.3).
- Produces: `core::mask_region(const cv::Mat&) -> std::optional<ViewRegion>` (unpadded), `MaskSelectionOptions::view_region`, `MaskSelectionResult::view`.

- [ ] **Step 1: Write the failing tests** (append to `tests/unit/core/test_mask_selection.cpp`)

Add the includes `#include <catch2/matchers/catch_matchers_floating_point.hpp>` and `#include <core/resource_view.hpp>`, then:

```cpp
TEST_CASE("MaskRegion_Rectangle_NormalisedUnpaddedBbox", "[core][mask_selection]") {
  using Catch::Matchers::WithinAbs;
  const auto r = reusex::core::mask_region(make_mask()); // 256², rect (100,100,80,80)
  REQUIRE(r);
  CHECK_THAT((*r)[0], WithinAbs(100.0 / 256.0, 1e-12));
  CHECK_THAT((*r)[1], WithinAbs(100.0 / 256.0, 1e-12));
  CHECK_THAT((*r)[2], WithinAbs(80.0 / 256.0, 1e-12));
  CHECK_THAT((*r)[3], WithinAbs(80.0 / 256.0, 1e-12));
  cv::Mat labels(4, 4, CV_32S, cv::Scalar(0));
  labels.at<int>(2, 1) = 7;
  CHECK(reusex::core::mask_region(labels) == reusex::core::ViewRegion{0.25, 0.5, 0.25, 0.25});
  CHECK_FALSE(reusex::core::mask_region(cv::Mat(4, 4, CV_8UC1, cv::Scalar(0))));
  CHECK_THROWS_AS(reusex::core::mask_region(cv::Mat(4, 4, CV_8UC3, cv::Scalar(1, 1, 1))),
                  std::invalid_argument);
}

TEST_CASE("ApplyMaskSelection_WithFrame_SetsPaddedView", "[core][mask_selection]") {
  using Catch::Matchers::WithinAbs;
  using reusex::core::ViewKind;
  const TempPath tmp("mask_apply_view");
  ProjectDB db(tmp.path);
  db.save_point_cloud("cloud", line_cloud(10));
  save_frame(db, 1);

  SECTION("from the mask region the caller passes") {
    MaskSelectionOptions opts;
    opts.frame_id = 1;
    opts.view_region = reusex::core::ViewRegion{0.375, 0.375, 0.25, 0.25};
    const auto r = apply_mask_selection(db, {0, 2}, "Dør", opts);
    REQUIRE(r.view);
    CHECK(r.view->kind == ViewKind::frame);
    CHECK(r.view->id == 1);
    REQUIRE(r.view->region);
    CHECK_THAT((*r.view->region)[0], WithinAbs(0.3375, 1e-12));
    CHECK_THAT((*r.view->region)[2], WithinAbs(0.325, 1e-12));
    CHECK(db.survey_part(r.resource_code)->view == r.view);
  }
  SECTION("without a region, from the selection projected into the frame") {
    MaskSelectionOptions opts;
    opts.frame_id = 1;
    // Points 0 and 2 (x = 0, 0.02 at z = 2): u = 64, 65; v = 64.
    const auto r = apply_mask_selection(db, {0, 2}, "Dør", opts);
    REQUIRE(r.view);
    REQUIRE(r.view->region);
    CHECK_THAT((*r.view->region)[0], WithinAbs(63.7 / 128.0, 1e-6));
    CHECK_THAT((*r.view->region)[2], WithinAbs(2.6 / 128.0, 1e-6));
  }
  SECTION("no frame, or an unknown one: no view, part still created") {
    MaskSelectionOptions none;
    CHECK_FALSE(apply_mask_selection(db, {0}, "Dør", none).view);
    MaskSelectionOptions missing;
    missing.frame_id = 99;
    const auto r = apply_mask_selection(db, {1}, "Dør", missing);
    CHECK_FALSE(r.view);
    CHECK_FALSE(db.survey_part(r.resource_code)->view);
  }
  SECTION("an invalid region is refused before anything is written") {
    MaskSelectionOptions bad;
    bad.frame_id = 1;
    bad.view_region = reusex::core::ViewRegion{0.1, 0.1, 0.0, 0.2};
    CHECK_THROWS_AS(apply_mask_selection(db, {0}, "Dør", bad), std::invalid_argument);
    CHECK(db.survey_parts().empty());
  }
}
```

- [ ] **Step 2: Run and confirm failure**

Run: `cmake --build build --target reusex_unit_tests --parallel`
Expected: compile error, `'mask_region' is not a member of 'reusex::core'`.

- [ ] **Step 3: Header** (`mask_selection.hpp`)

Add `#include "reusex/core/resource_view.hpp"`. After `project_frame_mask`:

```cpp
/// Normalised, unpadded bbox [x, y, w, h] of the non-zero pixels of a
/// single-channel CV_8U or CV_32S mask; nullopt when none is set.
/// @throws std::invalid_argument for a multi-channel or other-depth mask.
std::optional<ViewRegion> mask_region(const cv::Mat &mask);
```

In `MaskSelectionOptions`, after `frame_id`:

```cpp
  /// The mask's bbox in frame_id's image (mask_region()), unpadded. The new
  /// part's view is frame_id with this region padded 15 % per side. Unset
  /// with frame_id >= 0: the selected points' bbox in that frame is used.
  /// Validated up front (std::invalid_argument, nothing written).
  std::optional<ViewRegion> view_region;
```

In `MaskSelectionResult`, after `shrunk_instances`:

```cpp
  /// The new part's view (frame_id + region); nullopt without a frame.
  std::optional<ResourceView> view;
```

Add to the `apply_mask_selection` doc: "- View: with `frame_id >= 0` naming an existing frame, the part's view (see MaskSelectionOptions::view_region); an unknown frame is logged at warn and the part gets none."

- [ ] **Step 4: Implementation** (`mask_selection.cpp`)

Add `#include "core/frame_projection.hpp"`. In the anonymous-namespace-free part (before `apply_mask_selection`):

```cpp
std::optional<ViewRegion> mask_region(const cv::Mat &mask) {
  if (mask.empty())
    return std::nullopt;
  if (mask.channels() != 1 || (mask.depth() != CV_8U && mask.depth() != CV_32S))
    throw std::invalid_argument(
        "mask_region: mask must be single-channel CV_8U or CV_32S");
  int x0 = mask.cols, y0 = mask.rows, x1 = -1, y1 = -1;
  for (int y = 0; y < mask.rows; ++y)
    for (int x = 0; x < mask.cols; ++x) {
      const bool on = mask.depth() == CV_8U ? mask.at<std::uint8_t>(y, x) != 0
                                            : mask.at<std::int32_t>(y, x) != 0;
      if (!on)
        continue;
      x0 = std::min(x0, x);
      y0 = std::min(y0, y);
      x1 = std::max(x1, x);
      y1 = std::max(y1, y);
    }
  if (x1 < 0)
    return std::nullopt;
  const double w = mask.cols;
  const double h = mask.rows;
  return ViewRegion{x0 / w, y0 / h, (x1 - x0 + 1) / w, (y1 - y0 + 1) / h};
}
```

In `apply_mask_selection`, right after the `indices.empty()` check (before the `Transaction`):

```cpp
  if (opts.view_region)
    validate_view_shape({ViewKind::frame, 0, opts.view_region});
```

After `db.add_survey_part(part); out.resource_code = part.code;`:

```cpp
  if (opts.frame_id >= 0) {
    if (!db.has_sensor_frame(opts.frame_id)) {
      reusex::warn("apply_mask_selection: frame {} does not exist; part {} "
                   "gets no resource image",
                   opts.frame_id, part.code);
    } else {
      std::optional<ViewRegion> region;
      if (opts.view_region) {
        region = pad_region(*opts.view_region);
      } else {
        const auto base = db.point_cloud_xyzrgb(opts.base_cloud);
        std::vector<Eigen::Vector3f> pts;
        pts.reserve(sel.size());
        for (const auto i : sel)
          pts.emplace_back(base->points[i].x, base->points[i].y, base->points[i].z);
        region = project_instance_bbox(db, opts.frame_id, pts);
      }
      out.view = ResourceView{ViewKind::frame, opts.frame_id, region};
      db.set_survey_part_view(part.code, out.view);
    }
  }
```

(`sel` is the sorted, de-duplicated index list declared earlier in the function. Check the name in the current source before using it.)

- [ ] **Step 5: Run the tests and confirm they pass**

Run: `cmake --build build --target reusex_unit_tests --parallel && ctest --test-dir build -R 'MaskRegion|ApplyMaskSelection|apply_mask_selection|project_frame_mask' --output-on-failure --parallel "$(nproc)"`
Expected: all PASS.

- [ ] **Step 6: Commit**

```bash
git add libs/reusex/include/core/mask_selection.hpp libs/reusex/src/core/mask_selection.cpp tests/unit/core/test_mask_selection.cpp
git commit -m "core: mask -> resource sets the new part's view (mask bbox, padded)

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

### Task 1.7: Docs, full suite, install the package for ruxd

**Files:**
- Modify: `CLAUDE.md` (Core Module, ProjectDB bullet at the "Schema v26 adds the tombstone table" sentence)
- Modify: `docs/CONTRACTS.md` (`### survey` at `:234-258`; and the `segment_resource` entry if one exists, found with `grep -n segment_resource docs/CONTRACTS.md`)

- [ ] **Step 1: Update docs**

`CLAUDE.md`, after the sentence ending "…so the part is not re-created":

```markdown
  Schema v27 adds the resource view record (`survey_parts.view_*`,
  key `sys:view`, `core/resource_view.hpp`): the one chosen image (sensor
  frame or panorama + normalised region) a resource is shown by, set by
  `sync_survey` / mask → resource and read with no fallback ranking
```

`docs/CONTRACTS.md` `survey` table: Consumes gains `` `cloud` (positions) and posed `sensor_frames` (for views, optional) ``. Produces gains `` `survey_parts.view_*` (schema v27) ``. Append a paragraph:

```markdown
Then every instance-backed part without a view (new or pre-existing — the
backfill for old projects) gets the best-ranked frame (`instance_evidence`)
as its view, region = the instance's bbox in that frame padded 15 % per side.
A view is never overwritten. No sensor frames: skipped with one `info` line;
no positions cloud or misaligned clouds: `warn`, no views; parts no frame
sees keep none (`views_unavailable`).
```

- [ ] **Step 2: Full build + suite (Definition of Done, STANDARDS §9)**

```bash
cmake --build build --parallel
ctest --test-dir build --output-on-failure --parallel "$(nproc)"
clang-format --dry-run --Werror $(git diff --name-only main -- '*.cpp' '*.hpp')
reuse lint
```
Expected: the build has no new warnings, all tests pass (the `[gpu]` ones skip), clang-format is clean and REUSE is clean.

- [ ] **Step 3: Install the CMake package for Part 2**

```bash
cmake --install build --prefix /home/mephisto/.cache/reusex-prefix/resource-view --component ReUseX_Development
ls /home/mephisto/.cache/reusex-prefix/resource-view/include/reusex/core/resource_view.hpp \
   /home/mephisto/.cache/reusex-prefix/resource-view/include/reusex/core/frame_projection.hpp \
   /home/mephisto/.cache/reusex-prefix/resource-view/lib/cmake/ReUseX/ReUseXConfig.cmake
```
Expected: all three paths exist.

- [ ] **Step 4: Commit**

```bash
git add CLAUDE.md docs/CONTRACTS.md
git commit -m "docs: schema v27 resource view record

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

# Part 2 — ruxd

Work in `/home/mephisto/repos/ruxd`. It **builds against Part 1's installed package**. ruxd's `flake.lock` pins ReUseX at `97e4051c`, the parent of the ReUseX `resource-view` branch. ruxd's nixpkgs (`419fe0f4…`) is the same rev as the ReUseX worktree's, so a ReUseX built in the ReUseX dev shell and installed to a prefix is ABI-consistent with ruxd's dev shell. `CMakePresets.json` puts `$REUSEX_PREFIX` **first** on `CMAKE_PREFIX_PATH`, ahead of the flake's ReUseX that `nix develop` provides.

### Task 2.0: Branch and build against the local ReUseX

- [ ] **Step 1: Branch**

```bash
cd /home/mephisto/repos/ruxd
git switch -c resource-view
```

- [ ] **Step 2: Configure against the Part 1 prefix and confirm which ReUseX was found**

```bash
nix develop
export REUSEX_PREFIX=/home/mephisto/.cache/reusex-prefix/resource-view
cmake --preset dev --fresh
grep '^ReUseX_DIR' build/dev/CMakeCache.txt
```
Expected: `ReUseX_DIR:PATH=/home/mephisto/.cache/reusex-prefix/resource-view/lib/cmake/ReUseX`. If it points into `/nix/store`, the flake's copy won. Re-run with `cmake --preset dev --fresh -DReUseX_DIR=$REUSEX_PREFIX/lib/cmake/ReUseX`.

Alternative (slow, rebuilds ReUseX through nix; use it only if the prefix route fails): `nix develop --override-input reusex git+file:///home/mephisto/repos/ReUseX__worktrees/resource-view`. This needs the ReUseX branch committed, because flakes see only tracked files.

- [ ] **Step 3: Baseline build + tests**

```bash
cmake --build --preset dev
ctest --preset dev --parallel "$(nproc)" -LE postgres
```
Expected: PASS. Part 1 appends `sys:view` *last* to the built-in keys and leaves the seeds unchanged, so `GuiResources_Keys_IsTheCatalogueArray` (which checks `keys[0..8]`) and `GuiTemplates_*` stay green. If any test fails here, stop and report it as a Part 1 regression. Do not patch around it in ruxd.

### Task 2.1: Resource JSON carries `view`

**Files:**
- Modify: `include/api/resources.hpp` (declare `view_json`)
- Modify: `src/api/resources.cpp:39-47` (`resource_json`) + new `view_json`
- Modify: `docs/api/openapi.yaml` (`Resource` at `:6621`, `ResourceKey.data_type` enum at `~:6604`, new `ResourceView`/`ViewRegion` schemas, `PATCH /resources/{code}` description)
- Test: `tests/ruxd_api/test_api_resources.cpp`

**Interfaces:**
- Consumes: `core::Resource::view`, `core::ResourceView`, `core::to_string(ViewKind)` (Part 1).
- Produces: `nlohmann::json ruxd::api::view_json(const std::optional<reusex::core::ResourceView> &view);` (object or `null`), and `"view"` on every resource JSON (Interfaces §C).

- [ ] **Step 1: Write the failing test** (append to `tests/ruxd_api/test_api_resources.cpp`; add `#include <opencv2/core.hpp>`)

```cpp
TEST_CASE("GuiResources_View_ListPatchAndClear", "[gui][resources][view]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto t = make_type(db, "Døre");
  make_part(db, "RX-001", t);
  db.save_sensor_frame(7, cv::Mat(8, 8, CV_8UC3, cv::Scalar(1, 2, 3)));

  const auto before = resources_json(db, {}).at("resources")[0];
  CHECK(before.at("view").is_null());
  CHECK(before.at("values").at("sys:view").is_null());

  const std::string value =
      R"({"kind":"frame","id":7,"region":[0.1,0.2,0.3,0.4]})";
  const auto out = patch_resource_json(
      db, "RX-001", {}, json{{"values", {{"sys:view", value}}}}.dump());
  CHECK(out.at("resource").at("view") ==
        json::parse(R"({"kind":"frame","id":7,"region":[0.1,0.2,0.3,0.4]})"));
  CHECK(out.at("resource").at("values").at("sys:view") == value);

  // A template that leaves sys:view out still carries `view`.
  const auto filtered = resources_json(
      db, params_of({{"template", std::to_string(screening_id(db))}}));
  CHECK_FALSE(filtered.at("resources")[0].at("values").contains("sys:view"));
  CHECK(filtered.at("resources")[0].at("view").at("id") == 7);

  for (const std::string bad :
       {std::string("nope"), std::string(R"({"kind":"photo","id":7})"),
        std::string(R"({"kind":"frame","id":7,"region":[0,0.8,0.5,0.5]})"),
        std::string(R"({"kind":"frame","id":99})")}) {
    INFO(bad);
    CHECK(status_of([&] {
            patch_resource_json(db, "RX-001", {},
                                json{{"values", {{"sys:view", bad}}}}.dump());
          }) == 400);
  }
  CHECK(db.survey_part("RX-001")->view->id == 7);

  const auto cleared = patch_resource_json(
      db, "RX-001", {}, json{{"values", {{"sys:view", nullptr}}}}.dump());
  CHECK(cleared.at("resource").at("view").is_null());
}

TEST_CASE("GuiResources_Keys_ViewKeyIsAnEditableViewType", "[gui][resources][view]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto keys = resource_keys_json(db);
  const auto it = std::find_if(keys.begin(), keys.end(), [](const json &k) {
    return k.at("id") == "sys:view";
  });
  REQUIRE(it != keys.end());
  CHECK(it->at("data_type") == "view");
  CHECK(it->at("scope") == "part");
  CHECK(it->at("editable") == true);
  CHECK(it->at("label") == "Ressourcebillede");
}
```

(Add `#include <algorithm>`.)

- [ ] **Step 2: Run and confirm failure**

Run: `cmake --build --preset dev && ctest --preset dev -R 'GuiResources_View|GuiResources_Keys_View' --output-on-failure`
Expected: FAIL, because `before.at("view")` throws `json.exception.out_of_range` (key `view` not found).

- [ ] **Step 3: Implement**

`include/api/resources.hpp`, after the includes add `#include <reusex/core/resource_view.hpp>` and `#include <optional>`. Then declare in the namespace:

```cpp
/// A resource's view record (`sys:view`, ReUseX schema v27) on the wire:
/// `{kind: "frame"|"panorama", id, region: [x,y,w,h]|null}`, or null.
nlohmann::json view_json(const std::optional<reusex::core::ResourceView> &view);
```

`src/api/resources.cpp`, outside the anonymous namespace (it is declared in the header):

```cpp
json view_json(const std::optional<core::ResourceView> &view) {
  if (!view)
    return nullptr;
  return {{"kind", std::string(core::to_string(view->kind))},
          {"id", view->id},
          {"region", view->region ? json(*view->region) : json(nullptr)}};
}
```

`resource_json` is in the anonymous namespace above it. Forward-declare it or move `view_json` above that namespace. Then add `{"view", view_json(r.view)}` to its returned object.

- [ ] **Step 4: Contract** (`docs/api/openapi.yaml`)

In `components/schemas`, add:

```yaml
    ViewRegion:
      type: array
      minItems: 4
      maxItems: 4
      items: { type: number, format: double }
      description: |
        [x, y, w, h], normalised 0..1 of the image (the equirect for a
        panorama). w, h > 0; x, y >= 0; x < 1; w <= 1; y + h <= 1. A
        panorama's x + w may exceed 1 (wraps across the seam); a frame's not.
    ResourceView:
      type: object
      description: |
        The resource's chosen image (ReUseX schema v27, key `sys:view`): a
        sensor frame (`/frames/{id}/image`) or a 360° panorama
        (`/panoramas/{id}/image`) and the region of it that shows the part.
        Set by `POST /survey/sync` (best frame) and mask → resource; changed
        with `PATCH /resources/{code}` `{"values":{"sys:view": "<json>"|null}}`.
        There is no fallback: null means no image.
      required: [kind, id, region]
      properties:
        kind: { type: string, enum: [frame, panorama] }
        id: { type: integer, description: "sensor_frames.node_id or panoramic_images.id" }
        region:
          oneOf:
            - $ref: "#/components/schemas/ViewRegion"
            - type: "null"
```

In `Resource`: add `view` to `required`, and the property `view: { oneOf: [{ $ref: "#/components/schemas/ResourceView" }, { type: "null" }], description: "Always present, even when a template leaves sys:view out of values" }`. In `ResourceKey.data_type`, add `view` to the enum and to its description: "a `view` value is the compact JSON of a ResourceView, sent and returned as a string". In the `PATCH /resources/{code}` description, note that `sys:view` takes that JSON string or null, and that a bad shape or a frame/panorama that does not exist is a 400.

- [ ] **Step 5: Run the tests and the contract check**

Run: `cmake --build --preset dev && ctest --preset dev -R 'GuiResources|GuiTemplates|EndpointTable' --output-on-failure && python3 scripts/check-openapi.py`
Expected: PASS; the contract parses.

- [ ] **Step 6: Commit**

```bash
git add include/api/resources.hpp src/api/resources.cpp tests/ruxd_api/test_api_resources.cpp docs/api/openapi.yaml
git commit -m "api: resources carry their view record (sys:view)

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

### Task 2.2: Mask → resource passes the mask region and returns the view

**Files:**
- Modify: `src/api/api.cpp:1974-2016` (`execute_segment_resource`); add `#include "api/resources.hpp"`
- Modify: `include/api/api.hpp:462-470` (doc of the return shape)
- Modify: `docs/api/openapi.yaml` (`/cases/{cid}/frames/{id}/segment/resource` 201 schema at `~:1972`)
- Test: `tests/ruxd_api/test_api_segment.cpp` (`ExecuteSegmentResource_CreatesInstanceAndPart` at `:628`)

**Interfaces:**
- Consumes: `core::mask_region`, `MaskSelectionOptions::view_region`, `MaskSelectionResult::view` (Part 1), `view_json` (2.1).
- Produces: the 201 body gains `"view"` (Interfaces §C).

- [ ] **Step 1: Extend the failing test** (in `ExecuteSegmentResource_CreatesInstanceAndPart`, after the existing checks; add `#include <catch2/matchers/catch_matchers_floating_point.hpp>`)

```cpp
  // The mask (label 0 = pixels [24, 40) of 64) padded 15 % per side.
  const auto &view = out.at("view");
  CHECK(view.at("kind") == "frame");
  CHECK(view.at("id") == 1);
  const auto region = view.at("region").get<std::vector<double>>();
  REQUIRE(region.size() == 4);
  using Catch::Matchers::WithinAbs;
  CHECK_THAT(region[0], WithinAbs(0.3375, 1e-12));
  CHECK_THAT(region[1], WithinAbs(0.3375, 1e-12));
  CHECK_THAT(region[2], WithinAbs(0.325, 1e-12));
  CHECK_THAT(region[3], WithinAbs(0.325, 1e-12));
  REQUIRE(part->view);
  CHECK(part->view->id == 1);
```

- [ ] **Step 2: Run and confirm failure**

Run: `cmake --build --preset dev && ctest --preset dev -R 'ExecuteSegmentResource_CreatesInstanceAndPart' --output-on-failure`
Expected: FAIL. `out.at("view")` throws.

- [ ] **Step 3: Implement** (`execute_segment_resource`)

After `opts.frame_id = frame_id;`:

```cpp
  // The new part's view: this frame, the mask's bbox (core pads it 15 %).
  opts.view_region = reusex::core::mask_region(mask);
```

In the returned JSON, add `{"view", view_json(r.view)},` after `{"label_created", r.label_created},`. Update the `api.hpp` doc `@return` list to include `view`.

- [ ] **Step 4: Contract**: in the 201 schema add `view` to `required` and to `properties`: `view: { oneOf: [{ $ref: "#/components/schemas/ResourceView" }, { type: "null" }], description: "The new part's view: this frame, region = the mask's bbox padded 15 % per side" }`.

- [ ] **Step 5: Run**

Run: `cmake --build --preset dev && ctest --preset dev -R 'ExecuteSegmentResource|ParseSegmentResource' --output-on-failure && python3 scripts/check-openapi.py`
Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add src/api/api.cpp include/api/api.hpp tests/ruxd_api/test_api_segment.cpp docs/api/openapi.yaml
git commit -m "api: mask -> resource stores the mask region as the part's view

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

### Task 2.3: Instance frames carry the instance's region per frame

**Files:**
- Modify: `src/api/api.cpp:1597-1640` (`instance_frames_json`), new helper `add_frame_regions` in the anonymous namespace above it; add `#include <reusex/core/frame_projection.hpp>`
- Modify: `include/api/api.hpp` (`instance_frames_json` doc, `~:600-620`)
- Modify: `docs/api/openapi.yaml` (`/cases/{cid}/instances/{cloud}/{id}/frames` at `:2967`; new `InstanceFrame` schema)
- Test: `tests/ruxd_api/test_api_handlers.cpp` (`InstanceFrames_*` near `:1706-1855`)

**Interfaces:**
- Consumes: `core::instance_points`, `core::project_instance_bbox`, `core::CloudMisalignedError`.
- Produces: `frames[].region` (Interfaces §C). The frontend's "Brug som ressourcebillede" stores it verbatim.

- [ ] **Step 1: Write the failing tests** (in `test_api_handlers.cpp`, after `InstanceFrames_ServedFromCacheWhenGiven`; add `#include <catch2/matchers/catch_matchers_floating_point.hpp>`)

```cpp
TEST_CASE("InstanceFrames_EachFrameCarriesTheInstanceRegion", "[gui][routes]") {
  using Catch::Matchers::WithinAbs;
  const TempPath project("gui_instance_frames_region");
  reusex::ProjectDB db(project.path);
  save_visibility_frame(db, 1, 0.0, 0.0, 0.0);
  save_evidence_instances(db);
  const auto body = instance_frames_json(db, "instances", 1, Params{});
  REQUIRE(body.at("frames").size() == 1);
  const auto region = body.at("frames")[0].at("region").get<std::vector<double>>();
  REQUIRE(region.size() == 4);
  // u = 59..69, v = 64 of a 128² camera, padded 15 % per side.
  CHECK_THAT(region[0], WithinAbs(57.35 / 128.0, 1e-9));
  CHECK_THAT(region[1], WithinAbs(63.85 / 128.0, 1e-9));
  CHECK_THAT(region[2], WithinAbs(14.3 / 128.0, 1e-9));
  CHECK_THAT(region[3], WithinAbs(1.3 / 128.0, 1e-9));
}
```

In `InstanceFrames_ServedFromCacheWhenGiven`, after `CHECK(hit.at("frames").at(0).at("frame_id") == 42);` add:

```cpp
  // Frame 42 does not exist: the instance cannot be placed in it.
  CHECK(hit.at("frames").at(0).at("region").is_null());
```

- [ ] **Step 2: Run and confirm failure**

Run: `cmake --build --preset dev && ctest --preset dev -R 'InstanceFrames' --output-on-failure`
Expected: FAIL. `at("region")` throws.

- [ ] **Step 3: Implement** (`src/api/api.cpp`, in the anonymous namespace that holds `instance_probe_of`)

```cpp
/// Each listed frame's `region`: the instance's padded, normalised bbox in
/// that frame (core::project_instance_bbox) — the region a "Brug som
/// ressourcebillede" choice stores — or null when it does not project into
/// the frame. Loads the instance's points once per request.
void add_frame_regions(const reusex::ProjectDB &db, const std::string &cloud,
                       int instance_id, json &out) {
  std::vector<Eigen::Vector3f> points;
  try {
    auto all = reusex::core::instance_points(db, cloud, "cloud");
    if (const auto it = all.find(static_cast<std::uint32_t>(instance_id));
        it != all.end())
      points = std::move(it->second);
  } catch (const reusex::core::CloudMisalignedError &e) {
    throw HttpError(409, e.what());
  } catch (const std::invalid_argument &) {
    // No base cloud: frames still list, without regions.
  }
  for (auto &f : out.at("frames")) {
    const auto region =
        points.empty() ? std::nullopt
                       : reusex::core::project_instance_bbox(
                             db, f.at("frame_id").get<int>(), points);
    f["region"] = region ? json(*region) : json(nullptr);
  }
}
```

In `instance_frames_json`, call `add_frame_regions(db, cloud, instance_id, out);` immediately before **both** `return out;` statements (the cache-hit branch and the computed branch).

- [ ] **Step 4: Contract**

Add the schema `InstanceFrame: { allOf: [{ $ref: "#/components/schemas/VisibleFrame" }, { type: object, required: [region], properties: { region: { oneOf: [{ $ref: "#/components/schemas/ViewRegion" }, { type: "null" }], description: "The instance's bbox in this frame, normalised and padded 15 % per side — what a resource view choice stores; null when the instance does not project into the frame" } } }] }`. In the instance frames 200 response, add to the second `allOf` member a property override `frames: { type: array, items: { $ref: "#/components/schemas/InstanceFrame" } }`. Add one sentence to the description: "Each frame also carries `region`, the candidate region for the resource's view (spec A §4)."

- [ ] **Step 5: Run**

Run: `cmake --build --preset dev && ctest --preset dev -R 'InstanceFrames|InstancePanoramas|SurveyPhotos' --output-on-failure && python3 scripts/check-openapi.py`
Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add src/api/api.cpp include/api/api.hpp tests/ruxd_api/test_api_handlers.cpp docs/api/openapi.yaml
git commit -m "api: instance frames carry the instance's region per frame

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

### Task 2.4: Survey sync reports the view counts

**Files:**
- Modify: `src/api/survey.cpp:408-418` (`sync_survey_json`)
- Modify: `docs/api/openapi.yaml` (`POST /survey/sync` 200 schema at `~:3960-3990`)
- Test: `tests/ruxd_api/test_api_survey.cpp` (`SyncSurveyJson_ReportsInstancesSeenAndBackfilled` at `:452`)

- [ ] **Step 1: Extend the failing test**: after `CHECK(first.at("parts_created") == 2);`:

```cpp
  // No sensor frames: no views, both parts counted as without one.
  CHECK(first.at("views_attached") == 0);
  CHECK(first.at("views_unavailable") == 2);
```

- [ ] **Step 2: Run and confirm failure**: `ctest --preset dev -R SyncSurveyJson --output-on-failure` fails with `key 'views_attached' not found`.

- [ ] **Step 3: Implement**: add `{"views_attached", r.views_attached}, {"views_unavailable", r.views_unavailable}` to the returned object.

- [ ] **Step 4: Contract**: add to the sync 200 properties:

```yaml
                  views_attached:
                    type: integer
                    description: |
                      Instance-backed parts without a view that got one: the
                      best-ranked sensor frame, region = the instance's bbox
                      padded 15 % per side. A view is never overwritten.
                  views_unavailable:
                    type: integer
                    description: |
                      Instance-backed parts still without a view: no frame
                      sees the instance, or the project has no sensor frames
                      or no base `cloud`.
```

- [ ] **Step 5: Run**: `cmake --build --preset dev && ctest --preset dev -R 'SyncSurvey' --output-on-failure && python3 scripts/check-openapi.py`. Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add src/api/survey.cpp tests/ruxd_api/test_api_survey.cpp docs/api/openapi.yaml
git commit -m "api: survey sync reports views attached / unavailable

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

### Task 2.5: Full verification and the binary for Part 3

- [ ] **Step 1: Everything**

```bash
cmake --build --preset dev
ctest --preset dev --parallel "$(nproc)"          # incl. [postgres] when initdb is available
python3 scripts/check-openapi.py
reuse lint
```
Expected: PASS. `[postgres]` tests that cannot start a server skip or fail for environmental reasons only. Note them in the PR; they are not related to this change.

- [ ] **Step 2: Locate the dev binary for Part 3**

Run: `find /home/mephisto/repos/ruxd/build/dev -maxdepth 2 -name ruxd -type f -perm -u+x`
Expected: one path, normally `/home/mephisto/repos/ruxd/build/dev/ruxd`. Part 3 uses it as `RUXD_BIN`.

- [ ] **Step 3: Note for merging**: ruxd CI builds against the ReUseX pinned in `flake.lock`. After Part 1 is merged to ReUseX `main`, run `nix flake update reusex` on this branch and commit the lock bump before merging. Until then, CI cannot see the new ReUseX API. Do not merge before the bump.

---

# Part 3 — rux-frontend

Work in `/home/mephisto/repos/rux-frontend`: `git switch -c resource-view && npm ci`. The tests are vitest, against pure modules only (repo convention: no DOM tests). "Component tests" in the spec are tests of the pure helpers each component exports or uses.

### Task 3.1: Contract types and the view model (`kortlaegning/view.ts`)

**Files:**
- Modify: `src/api/types.ts` (`ResourceDataType` `:1229`, `Resource` `:1250`, `VisibleFrame` neighbours `:699`, `SurveySyncReport` `:1138`, `SegmentResourceResult` `:885`)
- Modify: `src/api/client.ts:1086-1098` (`instanceFrames` returns `InstanceFrame[]`)
- Modify: `src/kortlaegning/photo.ts` (`FrameLookup.frames: InstanceFrame[]`; move `resolveHighlightPart` here from `EvidencePanel.tsx` and re-export it there)
- Create: `src/kortlaegning/view.ts`
- Modify: `src/test/resourceFixtures.ts` (`resource()` gets `view: null`)
- Test: `src/test/kortlaegning.view.test.ts`

**Interfaces:**
- Consumes: ruxd JSON (Interfaces §C).
- Produces (all exported from `src/kortlaegning/view.ts`): `VIEW_KEY`, `VIEW_TEXT`, `PANO_WINDOW_DEG`, `frameView(frame)`, `panoView(id, u, v, resected)`, `viewValue(view)`, `isViewOf(view, kind, id)`, `viewCellText(value)`, `viewCaption(view)`, `selectionView(type, part, index)`, `rowThumbView(parts, index)`, `viewFetchSize(view, displaySize, cap?)`, `viewImageUrl(view, maxSize)`, `photoThumbState(view, frameId)`. Types: `ResourceView`, `ViewRegion`, `InstanceFrame` in `api/types.ts`.

- [ ] **Step 1: Write the failing test** `src/test/kortlaegning.view.test.ts`

```ts
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import type { ResourceView, SurveyPart, SurveyType } from '../api/types';
import { resourceIndex } from '../kortlaegning/resources';
import {
  frameView,
  isViewOf,
  panoView,
  photoThumbState,
  rowThumbView,
  selectionView,
  VIEW_KEY,
  VIEW_TEXT,
  viewCaption,
  viewCellText,
  viewFetchSize,
  viewImageUrl,
  viewValue,
} from '../kortlaegning/view';
import { resource } from './resourceFixtures';

function part(code: string, instance_id: number | null = 1): SurveyPart {
  return {
    code, type_id: 1, cloud: instance_id === null ? null : 'instances', instance_id,
    room_id: null, room_name: '', quantity: 1, starred: false, note: '',
    material_guid: null, instance_guid: instance_id === null ? null : `g-${code}`, orphaned: false,
  };
}
function type(parts: SurveyPart[]): SurveyType {
  return { parts } as unknown as SurveyType; // selectionView reads parts only
}
const FRAME: ResourceView = { kind: 'frame', id: 12, region: [0.1, 0.2, 0.3, 0.4] };
const PANO: ResourceView = { kind: 'panorama', id: 4, region: null };

describe('view values', () => {
  it('serialises kind, id, region in that order, null clears', () => {
    expect(viewValue(FRAME)).toBe('{"kind":"frame","id":12,"region":[0.1,0.2,0.3,0.4]}');
    expect(viewValue(null)).toBeNull();
    expect(VIEW_KEY).toBe('sys:view');
  });

  it('a frame choice stores the server region verbatim', () => {
    expect(frameView({ frame_id: 3, region: [0.2, 0.2, 0.1, 0.1] })).toEqual({
      kind: 'frame', id: 3, region: [0.2, 0.2, 0.1, 0.1],
    });
    expect(frameView({ frame_id: 3, region: null }).region).toBeNull();
  });

  it('a panorama choice is a 60° x 45° window centred on u, v', () => {
    const v = panoView(5, 0.5, 0.5, true);
    expect(v.kind).toBe('panorama');
    expect(v.region![0]).toBeCloseTo(0.5 - 1 / 12, 12);
    expect(v.region![1]).toBeCloseTo(0.375, 12);
    expect(v.region![2]).toBeCloseTo(1 / 6, 12);
    expect(v.region![3]).toBeCloseTo(0.25, 12);
  });

  it('wraps the window across the seam and clamps it vertically', () => {
    const v = panoView(5, 0.02, 0.05, true);
    expect(v.region![0]).toBeCloseTo(0.02 - 1 / 12 + 1, 12); // x in [0, 1)
    expect(v.region![0] + v.region![2]).toBeGreaterThan(1);
    expect(v.region![1]).toBe(0);
  });

  it('a levelled panorama (heading unknown) stores the whole image', () => {
    expect(panoView(5, 0.3, 0.5, false).region).toBeNull();
  });

  it('table cell text: Foto #n / 360° #n, blank for garbage', () => {
    expect(viewCellText('{"kind":"frame","id":12,"region":null}')).toBe('Foto #12');
    expect(viewCellText('{"kind":"panorama","id":4,"region":null}')).toBe('360° #4');
    expect(viewCellText('nope')).toBe('');
    expect(viewCellText(null)).toBe('');
  });

  it('captions and image URLs follow the kind', () => {
    expect(viewCaption(FRAME)).toBe('Ressourcebillede · ramme 12');
    expect(viewCaption(PANO)).toBe('Ressourcebillede · 360° 4');
    expect(viewImageUrl(FRAME, 512)).toContain('/frames/12/image');
    expect(viewImageUrl(FRAME, 512)).toContain('max_size=512');
    expect(viewImageUrl(PANO, 2048)).toContain('/panoramas/4/image');
  });

  it('asks for enough pixels that the region, not the image, fills the box', () => {
    expect(viewFetchSize(PANO, 96)).toBe(96);
    expect(viewFetchSize(FRAME, 96)).toBe(240); // 96 / 0.4
    expect(viewFetchSize({ kind: 'frame', id: 1, region: [0, 0, 0.01, 0.01] }, 96, 512)).toBe(512);
  });

  it('copy is Danish', () => {
    expect(VIEW_TEXT).toMatchObject({
      none: 'Intet billede valgt', use: 'Brug som ressourcebillede', chosen: 'Valgt', clear: 'Fjern billede',
    });
  });
});

describe('which view a selection shows (no ranking fallback)', () => {
  const index = resourceIndex([
    resource({ code: 'RX-001', view: null }),
    resource({ code: 'RX-002', view: FRAME }),
  ]);

  it('a part shows and writes its own view', () => {
    expect(selectionView(null, part('RX-001'), index)).toEqual({ target: 'RX-001', targetView: null, shown: null });
    expect(selectionView(null, part('RX-002'), index).shown).toEqual(FRAME);
  });

  it('a type shows its first part with a view, but a choice goes to its first linked part', () => {
    const sel = selectionView(type([part('RX-001'), part('RX-002')]), null, index);
    expect(sel.shown).toEqual(FRAME);
    expect(sel.target).toBe('RX-001');
    expect(sel.targetView).toBeNull();
  });

  it('a type with no view shows none, never a best frame', () => {
    expect(selectionView(type([part('RX-001')]), null, index).shown).toBeNull();
    expect(rowThumbView([part('RX-001')], index)).toBeNull();
    expect(rowThumbView([part('RX-001'), part('RX-002')], index)).toEqual(FRAME);
  });

  it('marks the chosen candidate', () => {
    expect(isViewOf(FRAME, 'frame', 12)).toBe(true);
    expect(isViewOf(FRAME, 'panorama', 12)).toBe(false);
    expect(isViewOf(null, 'frame', 12)).toBe(false);
    expect(photoThumbState(FRAME, 12)).toEqual({ chosen: true, label: 'Ramme 12 · Valgt' });
    expect(photoThumbState(FRAME, 13)).toEqual({ chosen: false, label: 'Brug som ressourcebillede: ramme 13' });
  });
});
```

- [ ] **Step 2: Run and confirm failure**

Run: `npx vitest run src/test/kortlaegning.view.test.ts`
Expected: FAIL, `Failed to resolve import "../kortlaegning/view"`.

- [ ] **Step 3: Types** (`src/api/types.ts`)

Next to `VisibleFrame` (the second declaration, `:699`):

```ts
/** `[x, y, w, h]`, normalised 0..1 of the image (the equirect for a panorama); a panorama's `x + w > 1` wraps across the seam. */
export type ViewRegion = [number, number, number, number];

/** `ResourceView` — a resource's chosen image (`sys:view`, ReUseX schema v27). No fallback: `null` means no image. */
export interface ResourceView {
  kind: 'frame' | 'panorama';
  /** `sensor_frames.node_id` or `panoramic_images.id`. */
  id: number;
  /** `null` = the whole image. */
  region: ViewRegion | null;
}

/** One frame of `GET /instances/{cloud}/{id}/frames`: a `VisibleFrame` plus the instance's padded bbox in it. */
export interface InstanceFrame extends VisibleFrame {
  /** The candidate region for the resource's view; `null` when the instance does not project into the frame. */
  region: ViewRegion | null;
}
```

`ResourceDataType`: append `| 'view'`. `Resource`: add `/** The chosen image; always present, even when a template leaves `sys:view` out of `values`. */ view: ResourceView | null;`. `SurveySyncReport`: add `/** Parts given the best frame as their view (absent from older servers). */ views_attached?: number; /** Instance-backed parts still without a view. */ views_unavailable?: number;`. `SegmentResourceResult`: add `/** The new part's view: this frame, region = the mask's bbox padded 15 %. */ view: ResourceView | null;`.

`src/api/client.ts` `instanceFrames`: the return type is `Promise<InstanceFrame[]>`, the generic is `{ frames: InstanceFrame[] }`, and `InstanceFrame` is imported.

`src/kortlaegning/photo.ts`: change `FrameLookup.frames` to `InstanceFrame[]`. Move `resolveHighlightPart` from `EvidencePanel.tsx` into `photo.ts` (same body and doc) and export it. In `EvidencePanel.tsx`, import it from `photo` and add it to the existing re-export line, so `PhotoStrip.tsx`'s import keeps working.

`src/test/resourceFixtures.ts`: `return { code: 'RX-008', type_id: 6, manual: true, values: {}, view: null, ...over };`.

In tests that build `FrameLookup.frames` (`kortlaegning.photo.test.ts` `frames()`, `kortlaegning.evidencePanel.test.ts` `frame()`), add `region: null` to the object literals so `npm run typecheck` passes.

- [ ] **Step 4: Implement** `src/kortlaegning/view.ts`

```ts
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * The resource view record as data (spec 2026-10-13-resource-view-record):
 * which image a selection shows, what a "Brug som ressourcebillede" choice
 * writes, and the URL/size to fetch. There is deliberately no fallback to a
 * ranked "best frame": a part without a view shows "Intet billede valgt".
 */

import { api } from '../api/client';
import type { InstanceFrame, ResourceView, SurveyPart, SurveyType, ViewRegion } from '../api/types';
import { hasInstanceLink } from './photo';
import type { ResourceIndex } from './resources';

export const VIEW_KEY = 'sys:view';

export const VIEW_TEXT = {
  none: 'Intet billede valgt',
  hint: 'Vælg et foto under Fotos eller en 360°-optagelse med »Brug som ressourcebillede«.',
  use: 'Brug som ressourcebillede',
  chosen: 'Valgt',
  clear: 'Fjern billede',
} as const;

/** The panorama window a choice stores: ~60° wide, 45° tall (spec §4). */
export const PANO_WINDOW_DEG = { width: 60, height: 45 } as const;

export function frameView(frame: Pick<InstanceFrame, 'frame_id' | 'region'>): ResourceView {
  return { kind: 'frame', id: frame.frame_id, region: frame.region };
}

/**
 * A panorama choice: the window centred on the part's equirect `u, v`,
 * wrapped so `x` is in [0, 1) (it may cross the seam) and clamped vertically.
 * A levelled panorama's `u` rests on a guessed heading, so it stores the
 * whole image instead of a window that may point at the wrong wall.
 */
export function panoView(panoramaId: number, u: number, v: number, resected: boolean): ResourceView {
  if (!resected) return { kind: 'panorama', id: panoramaId, region: null };
  const w = PANO_WINDOW_DEG.width / 360;
  const h = PANO_WINDOW_DEG.height / 180;
  const x = (((u - w / 2) % 1) + 1) % 1;
  const y = Math.min(Math.max(v - h / 2, 0), 1 - h);
  return { kind: 'panorama', id: panoramaId, region: [x, y, w, h] };
}

/** The `sys:view` PATCH value: compact JSON in the server's key order, or null to clear. */
export function viewValue(view: ResourceView | null): string | null {
  return view === null ? null : JSON.stringify({ kind: view.kind, id: view.id, region: view.region });
}

export function isViewOf(view: ResourceView | null, kind: ResourceView['kind'], id: number): boolean {
  return view !== null && view.kind === kind && view.id === id;
}

/** The table cell for a stored `sys:view` value: `Foto #12` / `360° #4`; '' when unset or unreadable. */
export function viewCellText(value: string | null): string {
  if (value === null) return '';
  try {
    const v = JSON.parse(value) as Partial<ResourceView>;
    if (v.kind === 'frame') return `Foto #${v.id}`;
    if (v.kind === 'panorama') return `360° #${v.id}`;
  } catch {
    // fall through
  }
  return '';
}

export function viewCaption(view: ResourceView): string {
  return view.kind === 'frame' ? `Ressourcebillede · ramme ${view.id}` : `Ressourcebillede · 360° ${view.id}`;
}

/**
 * What a selection shows (`shown`) and where a choice made on it goes
 * (`target`, with its current view `targetView`). A part: its own. A type:
 * shows its first part (in part order) with a view; a choice goes to the
 * part whose frames the photo strip lists — the type's first linked part.
 */
export function selectionView(
  type: SurveyType | null,
  part: SurveyPart | null,
  index: ResourceIndex,
): { target: string | null; targetView: ResourceView | null; shown: ResourceView | null } {
  if (part) {
    const own = index.get(part.code)?.view ?? null;
    return { target: part.code, targetView: own, shown: own };
  }
  if (!type) return { target: null, targetView: null, shown: null };
  const linked = type.parts.find((p) => hasInstanceLink(p)) ?? null;
  return {
    target: linked?.code ?? null,
    targetView: linked ? (index.get(linked.code)?.view ?? null) : null,
    shown: rowThumbView(type.parts, index),
  };
}

/** A row thumbnail's view: a part's own; a type's first part that has one. `null` = placeholder. */
export function rowThumbView(parts: readonly Pick<SurveyPart, 'code'>[], index: ResourceIndex): ResourceView | null {
  for (const p of parts) {
    const v = index.get(p.code)?.view;
    if (v) return v;
  }
  return null;
}

/** Edge length to fetch so the region (not the whole image) fills `displaySize` px, at most `cap`. */
export function viewFetchSize(view: ResourceView, displaySize: number, cap = 2048): number {
  const span = view.region ? Math.max(view.region[2], view.region[3]) : 1;
  return Math.min(cap, Math.max(displaySize, Math.ceil(displaySize / span)));
}

export function viewImageUrl(view: ResourceView, maxSize: number): string {
  return view.kind === 'frame'
    ? api.frameImageUrl(view.id, 'color', { maxSize })
    : api.panoramaImageUrl(view.id, { maxSize });
}

/** A Fotos-strip thumbnail: whether it is the chosen view, and its accessible label. */
export function photoThumbState(view: ResourceView | null, frameId: number): { chosen: boolean; label: string } {
  const chosen = isViewOf(view, 'frame', frameId);
  return { chosen, label: chosen ? `Ramme ${frameId} · ${VIEW_TEXT.chosen}` : `${VIEW_TEXT.use}: ramme ${frameId}` };
}

export type { ViewRegion };
```

- [ ] **Step 5: Run**

Run: `npx vitest run src/test/kortlaegning.view.test.ts && npm run typecheck`
Expected: PASS, and typecheck is clean. Other failures from the type change are expected only in files later tasks rewrite: `EvidencePanel.tsx` (it still compiles, since `frames` is assignable) and the `SurveyTable` users. Fix any remaining type error here.

- [ ] **Step 6: Commit**

```bash
git add src/api/types.ts src/api/client.ts src/kortlaegning/photo.ts src/kortlaegning/view.ts src/components/kortlaegning/EvidencePanel.tsx src/test/resourceFixtures.ts src/test/kortlaegning.view.test.ts src/test/kortlaegning.photo.test.ts src/test/kortlaegning.evidencePanel.test.ts
git commit -m "kortlægning: resource view types and model (sys:view)

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

### Task 3.2: Region crop maths and `ViewImage`

**Files:**
- Modify: `src/kortlaegning/view.ts` (add `cropBackground`)
- Create: `src/components/kortlaegning/ViewImage.tsx`, `src/components/kortlaegning/ViewImage.module.css`
- Test: `src/test/kortlaegning.view.test.ts` (append)

**Interfaces:**
- Produces: `cropBackground(region: ViewRegion | null, imageAspect: number, box: { width: number; height: number }, wrap: boolean): { backgroundSize: string; backgroundPosition: string; backgroundRepeat: string }`. Also `<ViewImage view displaySize maxFetch? className? label onError? />` (label `null` = decorative, `aria-hidden`).

- [ ] **Step 1: Write the failing tests** (append)

```ts
import { cropBackground } from '../kortlaegning/view';

describe('cropBackground (the whole region visible, centred)', () => {
  it('no region in a box of the same aspect: the image fills the box', () => {
    expect(cropBackground(null, 4 / 3, { width: 400, height: 300 }, false)).toEqual({
      backgroundSize: '400px 300px', backgroundPosition: '0px 0px', backgroundRepeat: 'no-repeat',
    });
  });

  it('scales the region up to the box and centres it', () => {
    expect(cropBackground([0.5, 0.5, 0.5, 0.5], 1, { width: 100, height: 100 }, false)).toEqual({
      backgroundSize: '200px 200px', backgroundPosition: '-100px -100px', backgroundRepeat: 'no-repeat',
    });
  });

  it('a panorama region across the seam repeats horizontally, seam centred', () => {
    // x + w = 1.125: centre at u = 1.0 ≡ 0; image drawn 400 x 200 px.
    // (Binary-exact fractions, so the strings compare exactly.)
    expect(cropBackground([0.875, 0.25, 0.25, 0.5], 2, { width: 200, height: 100 }, true)).toEqual({
      backgroundSize: '400px 200px', backgroundPosition: '100px -50px', backgroundRepeat: 'repeat-x',
    });
  });

  it('falls back to cover before the box or the image is measured', () => {
    expect(cropBackground([0, 0, 1, 1], 0, { width: 0, height: 0 }, false).backgroundSize).toBe('cover');
  });
});
```

- [ ] **Step 2: Run and confirm failure**: `npx vitest run src/test/kortlaegning.view.test.ts` fails with `cropBackground is not a function`.

- [ ] **Step 3: Implement `cropBackground`** (in `view.ts`)

```ts
export interface CropStyle {
  backgroundSize: string;
  backgroundPosition: string;
  backgroundRepeat: string;
}

/**
 * Background geometry that shows `region` of an image (aspect = width /
 * height) whole and centred in `box`: scaled so the region just fits
 * (the image around it fills the rest). A panorama (`wrap`) repeats
 * horizontally, so a region across the seam shows both sides.
 */
export function cropBackground(
  region: ViewRegion | null,
  imageAspect: number,
  box: { width: number; height: number },
  wrap: boolean,
): CropStyle {
  const repeat = wrap ? 'repeat-x' : 'no-repeat';
  if (!(imageAspect > 0) || !(box.width > 0) || !(box.height > 0)) {
    return { backgroundSize: 'cover', backgroundPosition: 'center', backgroundRepeat: repeat };
  }
  const [x, y, w, h] = region ?? [0, 0, 1, 1];
  const imgH = Math.min(box.width / (w * imageAspect), box.height / h);
  const imgW = imgH * imageAspect;
  let px = box.width / 2 - (x + w / 2) * imgW;
  const py = box.height / 2 - (y + h / 2) * imgH;
  if (wrap) px = ((px % imgW) + imgW) % imgW;
  return { backgroundSize: `${imgW}px ${imgH}px`, backgroundPosition: `${px}px ${py}px`, backgroundRepeat: repeat };
}
```

- [ ] **Step 4: `ViewImage` component** (`src/components/kortlaegning/ViewImage.tsx`)

```tsx
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * A resource view: its image cropped to its region (`cropBackground`), a
 * panorama wrapping across its seam. Measures its own box and probes the
 * image once for its aspect; a failed load shows "Intet billede".
 */

import { useEffect, useRef, useState } from 'react';

import type { ResourceView } from '../../api/types';
import { cropBackground, viewFetchSize, viewImageUrl } from '../../kortlaegning/view';
import styles from './ViewImage.module.css';

export interface ViewImageProps {
  view: ResourceView;
  /** Longest edge it is drawn at (px), for the fetch size. */
  displaySize: number;
  /** Fetch cap (a table thumbnail keeps it small). */
  maxFetch?: number;
  className?: string;
  /** Accessible name; `null` for a decorative thumbnail. */
  label: string | null;
  onError?: () => void;
}

export function ViewImage({ view, displaySize, maxFetch = 2048, className, label, onError }: ViewImageProps) {
  const ref = useRef<HTMLDivElement>(null);
  const [box, setBox] = useState({ width: 0, height: 0 });
  const [aspect, setAspect] = useState(0);
  const [failed, setFailed] = useState(false);
  const url = viewImageUrl(view, viewFetchSize(view, displaySize, maxFetch));

  useEffect(() => {
    const el = ref.current;
    if (!el) return;
    const measure = () => setBox({ width: el.clientWidth, height: el.clientHeight });
    measure();
    if (typeof ResizeObserver === 'undefined') return;
    const observer = new ResizeObserver(measure);
    observer.observe(el);
    return () => observer.disconnect();
  }, []);

  useEffect(() => {
    let live = true;
    setFailed(false);
    setAspect(0);
    const probe = new Image();
    probe.onload = () => {
      if (live && probe.naturalHeight > 0) setAspect(probe.naturalWidth / probe.naturalHeight);
    };
    probe.onerror = () => {
      if (!live) return;
      setFailed(true);
      onError?.();
    };
    probe.src = url;
    return () => {
      live = false;
    };
    // onError is a callback prop; re-probing on its identity would loop.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [url]);

  const classes = [styles.view, className].filter(Boolean).join(' ');
  if (failed) {
    return (
      <span className={[styles.fallback, className].filter(Boolean).join(' ')} aria-hidden={label === null || undefined}>
        Intet billede
      </span>
    );
  }
  const crop = cropBackground(view.region, aspect, box, view.kind === 'panorama');
  return (
    <div
      ref={ref}
      className={classes}
      role={label === null ? undefined : 'img'}
      aria-label={label ?? undefined}
      aria-hidden={label === null || undefined}
      style={aspect > 0 ? { backgroundImage: `url("${url}")`, ...crop } : undefined}
    />
  );
}
```

`ViewImage.module.css`:

```css
/*
 * SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

/* A resource view, cropped to its region (ViewImage.tsx). */

.view {
  width: 100%;
  height: 100%;
  background-color: var(--color-surface-sunken);
}

.fallback {
  display: flex;
  align-items: center;
  justify-content: center;
  width: 100%;
  height: 100%;
  font-size: var(--font-size-2xs);
  color: var(--color-text-muted);
  background: var(--color-surface-sunken);
}
```

- [ ] **Step 5: Run**

Run: `npx vitest run src/test/kortlaegning.view.test.ts && npm run typecheck && python3 .claude/skills/design-studio/scripts/token_lint.py src --tsx`
Expected: PASS; no token violations. If the linter flags the computed inline `style`, that is the same pattern `PanoStrip` uses. Check how PanoStrip passes the linter and follow it.

- [ ] **Step 6: Commit**

```bash
git add src/kortlaegning/view.ts src/components/kortlaegning/ViewImage.tsx src/components/kortlaegning/ViewImage.module.css src/test/kortlaegning.view.test.ts
git commit -m "kortlægning: region crop for resource views (ViewImage)

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

### Task 3.3: Table thumbnail shows the view; `sys:view` cells are read-only

**Files:**
- Modify: `src/components/kortlaegning/SurveyTable.tsx:29,109-125,413,465` (`RowThumb`)
- Modify: `src/kortlaegning/photo.ts` (delete `rowThumbFrame`, `:84-99`)
- Modify: `src/kortlaegning/resources.ts` (`cellModel` `:76-87`, `touchesSurvey` `:103`, `PANEL_FIELD_KEYS` `:127`)
- Modify: `src/kortlaegning/cellDraft.ts:120-145` (`cellDisplay` case `'view'`)
- Modify: `src/components/kortlaegning/ResourceCell.tsx:133` (read-only for `view`)
- Test: `src/test/kortlaegning.photo.test.ts` (drop the `rowThumbFrame` describe and import), `src/test/kortlaegning.resources.test.ts`, `src/test/kortlaegning.cellDraft.test.ts`

**Interfaces:**
- Consumes: `rowThumbView`, `viewCellText`, `VIEW_KEY` (3.1); `ViewImage` (3.2).

- [ ] **Step 1: Write the failing tests**

`src/test/kortlaegning.resources.test.ts`, in `describe('cellModel', …)`:

```ts
  it('a view key is shown, never edited, in table cells', () => {
    const key = resourceKey({ id: 'sys:view', data_type: 'view', editable: true });
    const col = { key, typeScoped: false } as ResourceColumn;
    const index = resourceIndex([resource({ code: 'RX-001', values: { 'sys:view': '{"kind":"frame","id":3,"region":null}' } })]);
    const m = cellModel({ kind: 'part', typeId: 1, partCode: 'RX-001' } as Row, type, col, index);
    expect(m.value).toBe('{"kind":"frame","id":3,"region":null}');
    expect(m.target).toBeNull();
  });
```

(Reuse the `type`, `Row`, `ResourceColumn` and fixture imports this file already has. Match the existing `cellModel` tests' construction of `row`/`type`.)

In `it('only sys: writes need the survey re-read', …)` add `expect(touchesSurvey(['sys:view'])).toBe(false);`. In the `PANEL_FIELD_KEYS` test (`'Alle egenskaber leaves out keys the panel already edits elsewhere'`), add `expect(PANEL_FIELD_KEYS.has('sys:view')).toBe(true);`.

`src/test/kortlaegning.cellDraft.test.ts`:

```ts
  it('shows a view as Foto #n / 360° #n', () => {
    const key = resourceKey({ id: 'sys:view', data_type: 'view' });
    expect(cellDisplay(key, '{"kind":"frame","id":3,"region":null}')).toBe('Foto #3');
    expect(cellDisplay(key, '{"kind":"panorama","id":4,"region":[0,0,0.5,0.5]}')).toBe('360° #4');
  });
```

- [ ] **Step 2: Run and confirm failure**: `npx vitest run src/test/kortlaegning.resources.test.ts src/test/kortlaegning.cellDraft.test.ts` gives FAIL (the target is not null; the display is the raw JSON).

- [ ] **Step 3: Implement**

`resources.ts`: `import { VIEW_KEY } from './view';`. In `cellModel`, compute `const writable = key.editable && key.data_type !== 'view';` and use `writable` instead of `key.editable` in both `target:` expressions. Change `touchesSurvey` to `return keyIds.some((id) => id.startsWith('sys:') && id !== VIEW_KEY);` and add the comment `// sys:view is a part column; the survey has no view`. Add `'sys:view',` to `PANEL_FIELD_KEYS` with the comment `// the Foto tab shows and changes it`.

`cellDraft.ts` `cellDisplay`: add `case 'view': return viewCellText(value);` (import from `./view`).

`ResourceCell.tsx:133`: `if (!editing || key.data_type === 'multiselect' || key.data_type === 'view') return <CellText … />;`. Add the comment `// sys:view is chosen in the Foto tab, not typed`.

`SurveyTable.tsx`: replace the `rowThumbFrame` import with `import { rowThumbView } from '../../kortlaegning/view';` (keep `photoCountText, ROW_THUMB_MAX_SIZE` from `photo`). Import `ResourceView` and `ViewImage`. Replace `RowThumb`:

```tsx
/** Table-thumbnail fetch cap: a tiny region must not pull a full-size image per row. */
const ROW_THUMB_FETCH_CAP = 512;

/**
 * A row's thumbnail: the resource's chosen view (`sys:view`), cropped to its
 * region. No view (none chosen, or the resources not loaded yet) is a quiet
 * placeholder of the same size — never a ranked "best frame".
 */
function RowThumb({ view }: { view: ResourceView | null }) {
  if (view === null) return <span className={styles.thumb} aria-hidden="true" />;
  return (
    <ViewImage view={view} displaySize={ROW_THUMB_MAX_SIZE} maxFetch={ROW_THUMB_FETCH_CAP} className={styles.thumb} label={null} />
  );
}
```

At `:413` use `<RowThumb view={rowThumbView(type.parts, resources)} />`, and at `:465` use `<RowThumb view={rowThumbView([part], resources)} />`. Update the `photos` prop doc to "Photo count per part code (`GET /survey/photos`) for the "n fotos" column". The thumbnails no longer come from it.

`photo.ts`: delete `rowThumbFrame` and its doc. In `kortlaegning.photo.test.ts`, remove the `rowThumbFrame` import and its `describe` (Task 3.1's `rowThumbView` tests replace it).

- [ ] **Step 4: Run**: `npm test && npm run typecheck`. Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add src/components/kortlaegning/SurveyTable.tsx src/kortlaegning/photo.ts src/kortlaegning/resources.ts src/kortlaegning/cellDraft.ts src/components/kortlaegning/ResourceCell.tsx src/test/kortlaegning.photo.test.ts src/test/kortlaegning.resources.test.ts src/test/kortlaegning.cellDraft.test.ts
git commit -m "kortlægning: table thumbnail shows the resource view, no best-frame fallback

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

### Task 3.4: Foto tab shows the view; 360° tab and "Fjern billede" actions

**Files:**
- Modify: `src/components/kortlaegning/EvidencePanel.tsx` (props, `evidenceSources`, `EvidenceImage`, both variants; drop its own frames `useAsync` and `resolvePhotoState` use)
- Modify: `src/components/kortlaegning/EvidencePanel.module.css` (`.hint`, `.viewAction`; tokens only)
- Modify: `src/kortlaegning/photo.ts` (delete `resolvePhotoState` if nothing else uses it: `grep -rn resolvePhotoState src`)
- Test: `src/test/kortlaegning.evidencePanel.test.ts`

**Interfaces:**
- Consumes: `selectionView` output (`shown`, `targetView`), `viewImageUrl`, `viewFetchSize`, `viewCaption`, `panoView`, `isViewOf`, `VIEW_TEXT` (3.1); `ViewImage` (3.2).
- Produces: `evidenceSources(type, part, view?: ResourceView | null, pano?)`. The Foto source gets `view?: ResourceView` when set, else `url: null, empty: VIEW_TEXT.none`. Props: `EvidencePanelProps` gains `view: ResourceView | null` (shown), `targetView: ResourceView | null`, and `onChooseView?: (view: ResourceView | null) => void` (absent = read-only).

- [ ] **Step 1: Rewrite the Foto tests** in `src/test/kortlaegning.evidencePanel.test.ts`

Replace the whole `describe('Foto', …)` block with:

```ts
  describe('Foto (the resource view, no fallback)', () => {
    const view = { kind: 'frame' as const, id: 12, region: [0.1, 0.2, 0.3, 0.4] as [number, number, number, number] };

    it('says no image is chosen when the part has no view — linked or not', () => {
      const p = part();
      const foto = tabOf(evidenceSources(type({}, [p]), p, null), 'foto');
      expect(foto.url).toBeNull();
      expect(foto.empty).toBe('Intet billede valgt');
      expect(foto.view).toBeUndefined();
      const manual = part({ cloud: null, instance_id: null });
      expect(tabOf(evidenceSources(type({}, [manual]), manual, null), 'foto').empty).toBe('Intet billede valgt');
    });

    it('shows the chosen frame cropped to its region, captioned', () => {
      const p = part();
      const foto = tabOf(evidenceSources(type({}, [p]), p, view), 'foto');
      expect(foto.view).toEqual(view);
      expect(foto.url).toContain('/frames/12/image');
      expect(foto.caption).toBe('Ressourcebillede · ramme 12');
    });

    it('shows a chosen panorama through the panorama image URL', () => {
      const p = part();
      const foto = tabOf(evidenceSources(type({}, [p]), p, { kind: 'panorama', id: 4, region: null }), 'foto');
      expect(foto.url).toContain('/panoramas/4/image');
      expect(foto.caption).toBe('Ressourcebillede · 360° 4');
    });
  });
```

Update the 360° tests' calls `evidenceSources(type({}, [p]), p, null, false, {…})` to `evidenceSources(type({}, [p]), p, null, {…})`, since the `photoFailed` argument is gone. Delete `describe('kortlægning resolvePhotoState', …)` if Step 3 deletes `resolvePhotoState`. Update the import list accordingly.

- [ ] **Step 2: Run and confirm failure**: `npx vitest run src/test/kortlaegning.evidencePanel.test.ts` gives FAIL (the empty text is the old `PHOTO_EMPTY_TEXT`; there is no `view`).

- [ ] **Step 3: Implement** (`EvidencePanel.tsx`)

- Update the module doc: the Foto tab shows the resource's chosen view (`sys:view`). No frames lookup is passed in any more. The `photoFrameId`/`photoFailed` paragraph is gone.
- `EvidenceSource` gains `/** Foto only: the chosen view, drawn cropped (ViewImage). */ view?: ResourceView;`.
- `evidenceSources(type, part, view?: ResourceView | null, pano?: {…})`: build the Foto source as

```ts
  const fotoSource: EvidenceSource = view
    ? { tab: 'foto', label: EVIDENCE_LABEL.foto, caption: viewCaption(view),
        url: viewImageUrl(view, viewFetchSize(view, 960)), empty: GENERIC_IMAGE_ERROR, view }
    : { tab: 'foto', label: EVIDENCE_LABEL.foto, caption: 'Ressourcebillede', url: null, empty: VIEW_TEXT.none };
```
  and use it in the returned array.
- `EvidenceImage`: before the `source.pano` branch:

```tsx
  if (source.view) {
    return (
      <div className={wellClassName}>
        <ViewImage view={source.view} displaySize={compact ? 160 : 960} label={compact ? null : source.caption}
          onError={() => setErrored(true)} />
      </div>
    );
  }
```
- `EvidencePanelProps`: add `view`, `targetView` and `onChooseView` (documented as in **Interfaces** above).
- `EvidencePanel`: delete the `frames` `useAsync` and the `resolvePhotoState` call. Call `evidenceSources(type, part, view, resolvePano(currentKey, panoramas.data))`.
- Under the image, in **both** variants (after the caption row / tagline), render one actions row:

```tsx
function ViewActions({ active, targetView, onChooseView }: {
  active: EvidenceSource; targetView: ResourceView | null; onChooseView?: (v: ResourceView | null) => void;
}) {
  if (active.tab === 'foto') {
    if (!active.view) return <p className={styles.hint}>{VIEW_TEXT.hint}</p>;
    return onChooseView && targetView ? (
      <button type="button" className={styles.viewAction} onClick={() => onChooseView(null)}>{VIEW_TEXT.clear}</button>
    ) : null;
  }
  if (active.tab === 'pano' && active.pano && onChooseView) {
    const p = active.pano;
    return isViewOf(targetView, 'panorama', p.id) ? (
      <span className={styles.viewChosen}>{VIEW_TEXT.chosen} ✓</span>
    ) : (
      <button type="button" className={styles.viewAction} onClick={() => onChooseView(panoView(p.id, p.u, p.v, p.marker))}>
        {VIEW_TEXT.use}
      </button>
    );
  }
  return null;
}
```
  (`p.marker` is `true` exactly for a resected panorama; see `evidenceSources`.)
- The Foto hint names the Fotos strip below. The candidates are the existing `PhotoStrip` in `DetailPanel` (panel variant) and `EditDialog` (stage variant), which Task 3.5 makes choosable. They are not duplicated inside this panel.
- CSS (`EvidencePanel.module.css`): `.hint { font-size: var(--font-size-xs); color: var(--color-text-muted); }`. For `.viewAction` and `.viewChosen`, copy the look of an existing small ghost button/link in the same file (e.g. `.openLink`), using tokens only.
- `photo.ts`: if `grep -rn resolvePhotoState src --include=*.ts --include=*.tsx | grep -v src/test` finds nothing, delete `resolvePhotoState` and drop it from `EvidencePanel.tsx`'s re-export line. Also fix the stale `resolvePhotoState` reference in the `pano.ts` comment (`:50`) to point at `resolvePano`'s own rule.
- `PHOTO_EMPTY_TEXT` (`vocab.ts:135`): if no consumer is left (`grep -rn PHOTO_EMPTY_TEXT src`), delete it.

- [ ] **Step 4: Run**: `npm test && npm run typecheck`. Expected: PASS. (`KortlaegningPage`/`EditDialog` do not pass the new props yet. They are required props, so typecheck fails until Task 3.5. To keep this commit green, make `view` and `targetView` optional in this task, `view?: ResourceView | null`, defaulting to `null`. Task 3.5 makes them required.)

- [ ] **Step 5: Commit**

```bash
git add src/components/kortlaegning/EvidencePanel.tsx src/components/kortlaegning/EvidencePanel.module.css src/kortlaegning/photo.ts src/kortlaegning/pano.ts src/kortlaegning/vocab.ts src/test/kortlaegning.evidencePanel.test.ts
git commit -m "kortlægning: Foto tab shows the chosen resource view; 360° can be chosen

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

### Task 3.5: Choose from the Fotos strip; wire the page, detail panel and edit dialog

**Files:**
- Modify: `src/components/kortlaegning/PhotoStrip.tsx` (thumbnails become choosable; "Valgt" mark)
- Modify: `src/components/kortlaegning/PhotoStrip.module.css` (`.thumbButton`, `.chosen`; tokens only)
- Modify: `src/components/kortlaegning/DetailPanel.tsx:62-…,285` (props `view`, `onChooseView` → `PhotoStrip`)
- Modify: `src/components/kortlaegning/EditDialog.tsx:59-82,351,363` (props `view`, `shownView`, `onChooseView` → `PhotoStrip` + `EvidencePanel`)
- Modify: `src/routes/KortlaegningPage.tsx` (compute `selectionView`, `chooseView`; pass to `EvidencePanel` `:838`, `DetailPanel` `:846`, `EditDialog` `:872`)
- Modify: `src/components/kortlaegning/EvidencePanel.tsx` (make `view`/`targetView` required again)
- Test: `src/test/kortlaegning.page.test.ts` (`chooseViewWrite`)

**Interfaces:**
- Consumes: `selectionView`, `frameView`, `photoThumbState`, `viewValue`, `VIEW_KEY` (3.1); `commitCell(code, keyId, value)` in `KortlaegningPage` (`:455`); `useCanEdit` (`src/app/CaseRoleContext`).
- Produces: `chooseViewWrite(target: string | null, view: ResourceView | null): { code: string; keyId: 'sys:view'; value: string | null } | null`, exported from `KortlaegningPage.tsx` (pure, tested).

- [ ] **Step 1: Write the failing test** (append to `src/test/kortlaegning.page.test.ts`; add `chooseViewWrite` to the import from `../routes/KortlaegningPage`)

```ts
describe('chooseViewWrite', () => {
  it('writes the chosen view to the target part as sys:view JSON', () => {
    expect(chooseViewWrite('RX-001', { kind: 'frame', id: 3, region: [0.1, 0.2, 0.3, 0.4] })).toEqual({
      code: 'RX-001', keyId: 'sys:view', value: '{"kind":"frame","id":3,"region":[0.1,0.2,0.3,0.4]}',
    });
  });

  it('clears with null, and does nothing without a target', () => {
    expect(chooseViewWrite('RX-001', null)).toEqual({ code: 'RX-001', keyId: 'sys:view', value: null });
    expect(chooseViewWrite(null, { kind: 'frame', id: 3, region: null })).toBeNull();
  });
});
```

- [ ] **Step 2: Run and confirm failure**: `npx vitest run src/test/kortlaegning.page.test.ts` gives FAIL (`chooseViewWrite` is not exported).

- [ ] **Step 3: Page** (`KortlaegningPage.tsx`)

Near `syncMessage`:

```ts
/** The cell write a "Brug som ressourcebillede" / "Fjern billede" makes, or null without a target part. */
export function chooseViewWrite(
  target: string | null,
  view: ResourceView | null,
): { code: string; keyId: typeof VIEW_KEY; value: string | null } | null {
  if (target === null) return null;
  return { code: target, keyId: VIEW_KEY, value: viewValue(view) };
}
```

In the component, after `index` is computed:

```ts
  const viewSel = selectionView(selType, selPart, index);
  const chooseView = canEdit
    ? (view: ResourceView | null) => {
        const write = chooseViewWrite(viewSel.target, view);
        if (write) void commitCell(write.code, write.keyId, write.value);
      }
    : undefined;
```

`commitCell` already toasts a refused value with the key's label ("Ressourcebillede") and merges the PATCH response into `resources`. The thumbnail, Foto tab and strip update from that.

Pass `view={viewSel.shown} targetView={viewSel.targetView} onChooseView={chooseView}` to `<EvidencePanel>`, and `view={viewSel.targetView} onChooseView={chooseView}` to `<DetailPanel>`. Pass `view={viewSel.targetView} shownView={viewSel.shown} onChooseView={chooseView}` to `<EditDialog>`.

- [ ] **Step 4: PhotoStrip** (`PhotoStrip.tsx`)

Props: add `/** The target part's view, to mark the chosen frame. */ view: ResourceView | null;` and `/** Absent: read-only (no edit role). */ onChooseView?: (view: ResourceView) => void;`. Replace `PhotoThumb` with:

```tsx
/** One Fotos thumbnail: choosable when editable, marked "Valgt" when it is the view. */
function PhotoThumb({ frame, view, onChoose }: {
  frame: InstanceFrame; view: ResourceView | null; onChoose?: (view: ResourceView) => void;
}) {
  const [errored, setErrored] = useState(false);
  const { chosen, label } = photoThumbState(view, frame.frame_id);
  const image = errored ? (
    <span className={styles.photoFallback} title={`Ramme ${frame.frame_id} kunne ikke hentes`}>Intet billede</span>
  ) : (
    <img className={styles.photo} src={api.frameImageUrl(frame.frame_id, 'color', { maxSize: PHOTO_STRIP_THUMB_SIZE })}
      alt={onChoose ? '' : `Ramme ${frame.frame_id}`} loading="lazy" onError={() => setErrored(true)} />
  );
  const mark = chosen ? <span className={styles.chosen}>{VIEW_TEXT.chosen}</span> : null;
  if (!onChoose) return chosen ? <span className={styles.thumbButton} data-chosen>{image}{mark}</span> : image;
  return (
    <button type="button" className={styles.thumbButton} data-chosen={chosen || undefined} aria-pressed={chosen}
      aria-label={label} title={chosen ? VIEW_TEXT.chosen : VIEW_TEXT.use}
      onClick={() => { if (!chosen) onChoose(frameView(frame)); }}>
      {image}{mark}
    </button>
  );
}
```

Render with `<PhotoThumb key={f.frame_id} frame={f} view={view} onChoose={onChooseView} />`. CSS: `.thumbButton { position: relative; padding: 0; border: none; background: none; cursor: pointer; }`, `.thumbButton[data-chosen] .photo { outline: 2px solid var(--color-accent); }` (use the accent/focus token the file family already uses; check `tokens.css`), `.chosen { position: absolute; inset-inline: 0; bottom: 0; font-size: var(--font-size-2xs); font-weight: var(--font-weight-bold); text-align: center; background: var(--color-surface); color: var(--color-text); }`. No literals.

- [ ] **Step 5: DetailPanel / EditDialog**: add the props (documented) and pass them to `PhotoStrip` (`view`, `onChooseView`). EditDialog also passes `view={shownView} targetView={view} onChooseView={onChooseView}` to its `EvidencePanel`. In `EvidencePanel.tsx`, make `view`/`targetView` required again (undo Task 3.4's optional stop-gap).

- [ ] **Step 6: Run everything**

```bash
npm test
npm run typecheck
npm run build
python3 .claude/skills/design-studio/scripts/token_lint.py src --tsx
RUXD_CONTRACT=/home/mephisto/repos/ruxd/docs/api/openapi.yaml npm run contract:check
reuse lint
```
Expected: all PASS. (`contract:check` checks paths only; no new paths were added.)

- [ ] **Step 7: Commit**

```bash
git add src/components/kortlaegning/PhotoStrip.tsx src/components/kortlaegning/PhotoStrip.module.css src/components/kortlaegning/DetailPanel.tsx src/components/kortlaegning/EditDialog.tsx src/components/kortlaegning/EvidencePanel.tsx src/routes/KortlaegningPage.tsx src/test/kortlaegning.page.test.ts
git commit -m "kortlægning: choose the resource image from Fotos / 360°, mark it Valgt

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

### Task 3.6: See it work against the Part 2 server (screenshots, both themes)

Use the repo's `.claude/skills/design-studio/` skill (load it) for the dev env and screenshots.

- [ ] **Step 1: Start ruxd (Part 2 binary) + Vite on a throwaway copy of the fixture**

```bash
RUXD_BIN=/home/mephisto/repos/ruxd/build/dev/ruxd \
  .claude/skills/design-studio/scripts/dev_env.sh start /home/mephisto/repos/ReUseX/tests/fixtures/scans/office_corridor.rux
```
In the GUI, run "Opret kortlægning fra instanser" if the survey is empty. The sync attaches views. If the case already had a survey, sync again for the backfill.

- [ ] **Step 2: Check each behaviour**, screenshotting with `shot.sh` in light and dark:
  1. The table thumbnails show cropped views. A part with no view shows the placeholder.
  2. The Foto tab shows the view with the caption `Ressourcebillede · ramme N`. A part without one shows `Intet billede valgt` plus the hint.
  3. "Brug som ressourcebillede" on a Fotos thumbnail updates the thumbnail, the Foto tab and the `Valgt` mark.
  4. 360° tab: choose a panorama, and the Foto tab shows the cropped equirect window (it wraps when near the seam).
  5. "Fjern billede" gives the no-image state.
  6. Read-only role (if the env supports it): no actions shown.

- [ ] **Step 3: Stop the env** (`dev_env.sh stop`). Attach the screenshots to the PR description. There is nothing to commit unless a visual fix was needed. If one was, commit it with its own message.

---

## Self-review notes (for the executor and reviewers)

- **Spec coverage.** §2 storage → 1.3. Library API → 1.1, 1.3, 1.4. Projection helper in core, reused by vision → 1.2. Auto-attach in sync, with backfill, report and CLI → 1.5. Mask → resource → 1.6 + 2.2. Create by name stays null → unchanged `create_resource` (covered by `Resources_CreateAndDelete`, which keeps passing). §3 ruxd → 2.1 to 2.4. §4 frontend → 3.1 to 3.6. §5 out of scope: no PDF rendering, no drag-edit.
- **Deliberate interpretations** (also listed in the hand-off report): `MaskSelectionOptions::view_region`, because `apply_mask_selection` never sees the mask. Top-level `view` on the resource JSON. Frame regions may not wrap. `sys:view` is left out of the screening seed and the PDF table. Data type `view`. A levelled panorama stores the whole image. A type-level choice targets the first linked part. The candidate strip is the existing PhotoStrip.
