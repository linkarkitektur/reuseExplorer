# Resource view record (sub-project A)

Date: 2026-10-13 · Status: draft for review · Repos: ReUseX (library,
schema), ruxd, rux-frontend

Part of the PDF report rework; see
the table at the top of `2026-10-13-report-template-workflow-design.md` for the A/B/C split.

## 1. Intent

A resource (survey part) needs one **chosen** image that identifies it — in
the GUI and in the PDF report. Today the GUI shows the best-ranked sensor
frame (`core::instance_photos` → `frames.front()`), recomputed on every
request. That ranking can change when poses, instances or the cloud change,
and it cannot express "I defined this resource from *that* image" or "this
other view shows it better". So the choice becomes data: a **view record**
stored on the part, set automatically when the part is created, changeable
by the user, and the only thing the report reads.

## 2. The view record

A view is an image source plus a region of it:

| Field | Type | Meaning |
|---|---|---|
| `kind` | `frame` \| `panorama` | sensor frame (`sensor_frames.node_id`; imported photos are frames too) or 360° panorama (`panoramic_images.id`) |
| `id` | int | node id / panorama id |
| `region` | `[x, y, w, h]` or null | normalised 0..1 of the image (equirect for a panorama). null = whole image. For a panorama, `x + w > 1` wraps across the seam. |

A part with no view has none (null) — there is **no fallback ranking**
anywhere a view is consumed; the consumer shows its "no image" state.

### Storage (schema v27)

`survey_parts` gains nullable columns `view_kind TEXT CHECK (view_kind IN
('frame','panorama'))`, `view_id INTEGER`, `view_x`, `view_y`, `view_w`,
`view_h REAL`. Additive migration, existing rows NULL. Invariant (enforced
by the setter, and a CHECK): `view_kind` and `view_id` are both null or
both set; the region is all-null or all-set with `w, h > 0`, `x, y ≥ 0`,
`y + h ≤ 1`, `w ≤ 1`.

A referenced frame/panorama can be deleted later. Readers treat a dangling
view as null and log at `warn` (STANDARDS §5); nothing cascades.

### Library API

- `ProjectDB`: `struct ResourceView { ViewKind kind; int id;
  std::optional<std::array<double,4>> region; }`;
  `SurveyPartRecord::view` (`std::optional<ResourceView>`);
  `set_survey_part_view(code, std::optional<ResourceView>)` — validates the
  invariant and that the frame/panorama exists, throws
  `std::invalid_argument` / `std::out_of_range`.
- Resource key `sys:view` (part-scoped, built-in), value = compact JSON
  `{"kind":"frame","id":123,"region":[x,y,w,h]}` or unset. It travels
  through `list_resources` / `patch_resource` / templates like every other
  key, so ruxd and the CSV get it for free. `display_value` renders it as
  `Foto #123` / `360° #4` (CSV, not the report). `normalise_value` parses
  and validates it via the same checks as the setter.

### Automatic attachment

- **`sync_survey`**: for every instance-backed part whose view is null
  (newly created *or* pre-existing — this is the backfill for old
  projects), attach the best-ranked frame from `core::instance_photos`
  (one pass for all parts, as ruxd's `PhotoEvidenceCache` does). Region =
  the bounding box of the instance's points projected into that frame,
  padded 15 % per side, clamped to the image. Never overwrites a non-null
  view. Parts whose instance no frame sees stay null; the sync report
  counts `views_attached` / `views_unavailable` and the CLI prints them.
  No sensor frames at all: skip with one `info` line.
- **Mask → resource** (`core::apply_mask_selection` with
  `MaskSelectionOptions::frame_id`): the new part's view = that frame,
  region = the mask's pixel bbox, padded 15 %, normalised.
- **Resource created by name** (`create_resource`): null.

The projection of an instance into a frame to get its bbox needs a
core-level helper (the existing `vision::project_points_to_box` is in
`vision`, which `core` may not link — STANDARDS §1). Add
`core::project_instance_bbox(db, frame_id, points)` (or move the pinhole
math to `geometry_common`) and have `vision` reuse it rather than keep two
copies.

## 3. ruxd

- Resource JSON (`resource_json`) carries `view` (object or null) — it
  already serialises every key, so mostly verify + document in
  `docs/gui/` (the API contract).
- `PATCH /resources/<code>` accepts `sys:view` like any key (validation
  errors → 400/422 via the existing `KeyValueError` mapping).
- Mask → resource endpoint: nothing to add beyond passing `frame_id`
  (already done); its response includes the new view.
- `/survey/photos` and `/instances/.../frames` stay — they are now the
  *candidates* for picking a view, not the displayed image.
- `PhotoEvidenceCache` is unaffected.
- Tests: resource JSON includes `view`; PATCH round-trip; invalid view →
  4xx; mask → resource sets the view.

## 4. rux-frontend

- **Table thumbnail** (`SurveyTable` `RowThumb`) and **Foto tab**
  (`EvidencePanel`): show the part's `view` — frame via
  `frameImageUrl(id, 'color', {maxSize})`, panorama via the panorama image
  URL; when a region is set, show the region (CSS crop via
  `object-view-box` or a background-position crop; panorama wrap handled).
  A type row uses its first part with a view. No `best_frame_id` fallback.
- **No view**: Foto tab shows "Intet billede valgt" with the candidate
  strip below and a one-line hint.
- **Choosing**: each `PhotoStrip` thumbnail and the 360° strip get a
  "Brug som ressourcebillede" action → `PATCH sys:view` with the
  instance's region for that frame (frames: from the existing
  `/instances/.../frames` data if it carries a bbox, else add it there in
  ruxd; panorama: a fixed ~60° × 45° window centred on the instance's
  `u,v`). The current view is marked in the strip ("Valgt").
- "Fjern billede" clears it (PATCH null).
- Tests per the repo's conventions (vitest for the region/crop maths and
  the fallback-free selection; component test for the action).

## 5. Out of scope

Rendering views into the PDF (C — it reads `sys:view`, crops frames by
region and reprojects panorama regions to a perspective view via
`geometry::EquirectProjection`). Editing the region by dragging (later;
the record already supports it). Multiple images per resource.

## 6. Execution

Three branches, one agent each, merged in order **ReUseX → ruxd →
rux-frontend** (ruxd builds against the installed ReUseX package; the
frontend needs ruxd's JSON). ReUseX work happens in its own worktree,
separate from sub-project B's.
