# Rediger: correspondence-based pair alignment

Date: 2026-10-14 · Beads: `reusex-xlg.2`, `reusex-xlg.3` (epic `reusex-xlg`)
Builds on: [`2026-10-13-pose-graph-editor-design.md`](2026-10-13-pose-graph-editor-design.md) (Rediger, pair panel, ΔT, seeded ICP)
Status: design approved in chat, awaiting spec review

## 1. Intent

The user reviewing a pair (A, B) in Rediger wants to align B to A from
**feature correspondences between the two images**, the way RTABMap's
database editor shows them, instead of entering a transformation by hand.
Correspondences are found automatically and can be corrected by hand; the
alignment they give is then refined with the existing ICP-forfin and stored
with the existing "Tilføj kant".

Success:

- "Find matches" shows keypoints and match lines on the A | B images, inliers
  and outliers distinguishable at a glance (RTABMap-style).
- The user can exclude bad matches, add matches by clicking, and limit
  detection to a region of either image.
- "Align from matches" moves B in the 3D overlay to the rigid fit of the
  enabled matches; ICP-forfin starts from there.
- Nobody types numbers: the six ΔT spin boxes are gone.

## 2. Decisions (from the brainstorm)

| # | Decision |
|---|---|
| C1 | Correspondences produce an **alignment of B** (ΔT), then ICP refines it; they are not stored as a measurement. |
| C2 | Matching is **classical, as in RTABMap**: OpenCV ORB features + ratio test through the library's existing `FrameMatcher`; no learned model, no new dependency. |
| C3 | Manual editing: click a match to exclude/include; add a match by clicking A then B; drag a region in A or B to limit detection. "Align from matches" re-runs RANSAC on the enabled matches. |
| C4 | Matches are **working state only**, discarded when A or B changes. No schema change. |
| C5 | The ΔT spin boxes are **removed**; a one-line read-out + Nulstil remains, keyboard nudges stay. |

## 3. Library (`reusex_slam`)

Reuse, do not rewrite: `FrameMatcher` / ORB + ratio test, the depth lifting and
`detail::ransac_rigid` already in `LoopClosure.{hpp,cpp}`.

New public header `slam/frame_pair_matches.hpp` (no GTSAM, no OpenCV types in
the public interface beyond what `LoopClosure.hpp` already exposes):

```cpp
struct PixelRect { int x = 0, y = 0, w = 0, h = 0; };   // empty = whole image

struct FrameMatchOptions {
  int max_features = 3000;           // mirrors LoopClosureOptions defaults
  double ratio_test = 0.8;
  std::optional<PixelRect> region_a, region_b;
};

struct FrameKeypoint {
  float u = 0, v = 0;                         // pixel in the stored colour image
  std::optional<std::array<double, 3>> xyz;   // frame-local 3D (stored-pose frame), nullopt = no valid depth
};

struct FrameMatch { int a = -1, b = -1; float distance = 0; };   // indices into keypoints

struct FramePairMatches {
  std::vector<FrameKeypoint> keypoints_a, keypoints_b;
  std::vector<FrameMatch> matches;
};

FramePairMatches match_frame_pair(const ProjectDB &db, int a_id, int b_id,
                                  const FrameMatchOptions &opts = {});

// A user-added correspondence: lift (u, v) through the frame's depth,
// snapping to the nearest valid depth pixel within `snap_px`; nullopt if none.
std::optional<FrameKeypoint> lift_pixel(const ProjectDB &db, int frame_id,
                                        float u, float v, int snap_px = 4);

struct RigidFit {
  bool ok = false;
  std::array<double, 16> world_delta{};   // ΔT applied to B in world space (same convention as PairOverlay / FramePairIcpOptions::initial_guess)
  std::vector<int> inliers;               // indices into the `pairs` argument
  double rms_m = 0;
  std::string reason;                     // when !ok: e.g. "3 matches with depth needed, 2 given"
};

// pairs: (point in A's frame, point in B's frame), both frame-local;
// poses are the frames' current stored poses.
RigidFit rigid_from_matches(const std::array<double, 16> &pose_a,
                            const std::array<double, 16> &pose_b,
                            const std::vector<std::pair<std::array<double,3>, std::array<double,3>>> &pairs,
                            double inlier_dist_m = 0.05, int iterations = 2000,
                            std::uint32_t seed = 1);
```

- `world_delta` is chosen so that `world_delta · T_B · p_b ≈ T_A · p_a` for
  inliers, i.e. exactly the ΔT the overlay applies to B and ICP accepts as
  `initial_guess` (pose-graph spec §5.2, verified convention).
- Deterministic (STANDARDS §6): fixed RANSAC seed.
- Keypoints whose depth is invalid are kept (drawn hollow) but never enter a fit.
- Errors (missing image/depth/pose) throw `std::runtime_error` with the frame
  id; an empty result is not an error but is reported in `reason`/counts.

Tests (`tests/unit/slam/test_frame_pair_matches.cpp`, `[slam][matches]`):
synthetic pair with a known ΔT (rendered checkerboard/texture + flat depth)
recovers ΔT within 1 cm / 0.5°; region restriction drops keypoints outside;
`lift_pixel` snaps and refuses; `rigid_from_matches` rejects injected outliers,
refuses < 3 pairs with a reason, is deterministic for a fixed seed; the
returned `world_delta` round-trips through `edge_measurement` to the same edge
as an ICP B→A `edge_transform` for the same alignment.

## 4. Qt client

### 4.1 Match editor (replaces the plain A | B images in Rediger's Billeder pane)

- One canvas showing A and B side by side (existing colour/depth toggle kept),
  each image independently zoom/pan (wheel / drag with the hand in select mode).
- Overlay drawing (tokens only):
  - keypoints: small dots; hollow = no depth;
  - matches: lines across the gap — inlier `--color-status-succeeded`,
    outlier `--color-status-failed`, excluded `--color-text-faint`,
    manual matches drawn dashed; hover highlights the line and both ends.
- Toolbar (Danish UI): **Find match** (auto detection; called "Find matches" below) · mode segmented **Vælg / Tilføj / Område** ·
  **Placér B fra match** ("Align from matches" below) · read-out "38 matches · 24 inliers · 3 manuelle ·
  2 udeladt".
  - Vælg: click a line toggles excluded/included.
  - Tilføj: click a point in A, then in B; each click goes through
    `lift_pixel` off-thread; a refused click shows "ingen dybde her".
    Esc cancels a half-made match.
  - Område: drag a rectangle in A or B; it restricts the next Find matches
    (a "Ryd område" chip clears it).
- Find matches and lifting run off the GUI thread with their own `ProjectDB`;
  results for a stale pair are dropped (same pattern as ICP / image decode).
- Inlier/outlier colouring comes from the last "Align from matches" (before
  that, all enabled matches are neutral).

### 4.2 Align from matches → overlay → ICP

- "Align from matches" calls `rigid_from_matches` on enabled matches with
  depth on both ends, off-thread. On success: `PairOverlay::set_offset` with
  the fit's `world_delta` (normalised), inlier colouring updated, status
  "Fra matches: 24 inliers · RMS 2,1 cm". On failure: the fit's `reason`.
- ICP-forfin, the ICP-currentness rule and Tilføj kant are unchanged: an edge
  added after a matches-alignment without ICP is a `manual` edge with the
  measurement `T_A⁻¹·ΔT·T_B` (as today).

### 4.3 ΔT fields (C5)

Remove the six spin boxes and their labels. The overlay header keeps the
read-out "Flyttet 12,4 cm · 3,1°" and Nulstil; arrow/PgUp/PgDn/[ ] nudges and
the box widget stay. The freed height goes to the 3D canvas (the
`OVERLAY TOO SMALL` gallery guard stays).

### 4.4 Qt-free logic (`rux_qt_core`, tested in `tests/unit/rux_qt`)

Match-set state (enabled/excluded/manual, counts, inlier flags after a fit,
discard on pair change), hit-testing a click to the nearest line within N px,
image ↔ canvas coordinate mapping under per-image zoom/pan, and the read-out
text.

### 4.5 Gallery / verification

Pages `rediger-matches` (auto matches with inliers/outliers after align),
`rediger-matches-edit` (excluded + manual + region), both themes, 1440×900,
existing guards pass. Real-data check on a scratch copy of NewOffice: a pair
where odometry is good (matches-align ≈ identity ΔT) and the pair from the
user's screenshot (2240/2241); report inliers, RMS and the ICP result after.

## 5. Out of scope

Learned matchers (LightGlue/XFeat via ONNX) — `FrameMatcher` keeps the seam;
persisting matches; matches between a frame and a panorama; segmentation
editing in Oversigt (`reusex-xlg.4`, separate spec); the overlay translation
bug (`reusex-xlg.1`, separate bounded fix).
