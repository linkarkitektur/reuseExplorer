<!--
SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
SPDX-License-Identifier: GPL-3.0-or-later
-->

# Why `rux align 360` fails for most panoramas

**Bead reusex-bx2.1** (epic reusex-bx2). Measured on NewOffice, 2026-10-15.
This is research: it names failure modes and tests candidate causes. It does
not propose code; the fixes the evidence supports are filed as beads under
reusex-bx2 (listed at the end).

## TL;DR

- **The dominant failure is that the matches contain no detectable geometric
  signal.** In 15 of the 18 panoramas, the best slice's PnP inlier count
  (0–15) falls inside the range we measured after **randomly shuffling the
  2D↔3D pairings** (0–17). Whichever gate then "rejects" the panorama
  (`upside_down`, `too_few_pooled_inliers`, `refinement_collapsed`,
  `no_slice_pnp`) only reports how a random pose happened to fail. All four
  gates describe the same single failure.
- **Accepting a pose does not mean the pose is right.** With 20 candidate
  frames, shuffled (pure-noise) correspondences reach 54–262 pooled inliers at
  0.73–0.77° RMS. That clears `min_inliers` comfortably. Only the 5 m
  plausibility gate stops them. Two of the 4 panoramas accepted at the CLI
  defaults have a best-slice count at the noise level (`003`: 13, `013`: 9).
  They are corrected 2.5 m and 4.2 m, which is not plausible for a camera held
  in the same hand as the iPad. They are false positives.
- **Where real signal exists, it comes from only 1–3 frames.** The resection is
  then ill-conditioned in translation. It either trips the correction gate
  (deltas of 7–44 m) or gets accepted metres away from where the camera was.
- **Contributing input causes.** The candidate frames are motion-blurred, tilted
  down at a textureless floor, and cover a narrow wedge (about 50°) of the
  sphere. In the long open-plan hall (`004`–`006`), clipping depth at 6 m
  throws away 53–75 % of the frame keypoints, and with them the textured
  content.
- **Not the cause:** the dual-fisheye seam, and our own `.insp` stitch versus
  Hugin's. Changing the slice tiling, or the descriptor alone (SIFT, AKAZE,
  mutual check, stricter Lowe ratio), changes which panoramas pass, but
  nothing pushes more than 4 of 18 above the noise floor *and* within 1 m of
  the seed.
- **#264 transfers only in part.** The symptom is the same: hand-crafted
  cross-camera features are weak, and SIFT/AKAZE do not help. The binding
  problem here sits upstream of the matcher, though. The correspondence pool is
  ~95 % junk by construction, and the acceptance gates are not calibrated
  against noise. A learned matcher dropped into this pipeline unchanged would
  still feed it the same frames and the same gates.

## Setup

- Project: a private copy of `~/repos/NewOffice/project.rux`, at schema v28:
  3876 frames and 18 panoramas, Hugin-stitched 5422×2711 equirect `.jpg`. The
  raw `.insp` files sit next to them. Every run was
  `rux align 360 --dry-run --overwrite`.
- Instrumentation. This study added permanent diagnostics in commit 40c189f1,
  "slam: name the gate that rejects a panorama":
  - `PanoramaAlignmentResult::outcome` names the gate that decided the result.
  - `PanoramaAlignmentDiagnostics` holds the matching funnel per slice and per
    candidate frame, the gravity-check dot product, pooled/final inliers, and
    the seed delta, recorded also for gated poses.
  - `-vv` logs one funnel line per panorama and `-vvv` one line per slice and
    frame.
  - `rux align 360 --diagnostics FILE.json [--diagnostics-correspondences]`
    writes the records.

  Before this, every failure was logged as "insufficient matches".
- Experiments that change the algorithm (null model, matcher variants, depth
  range) ran from a throwaway patch that read environment variables. It was not
  committed.
- **A plausibility reference without ground truth.** The panoramas show the
  operator holding the 360 camera in one hand and the iPad in the other (the
  iPad is visible in the nadir). The panorama is also taken within 2 s of its
  seed frame (`|pano − frame dt|` ≤ 1.7 s for all 18). So a correct pose
  should sit **within about 1 m of the seed frame's centre**. The tables count
  an accepted pose as *credible* when its delta is < 1 m.

## Per-panorama numbers (CLI defaults)

The CLI defaults are `--window 12 --max-candidates 8 --min-inliers 40`, i.e.
the seed frame and its 7 nearest frames by id, roughly ±4 frames or ±5 s.
Columns:

- **frame kp** and **depth %**: summed over the candidate frames.
- **ratio**: slice keypoints whose 1-NN in a frame passed Lowe's 0.85 ratio.
- **corr**: the subset with depth (the PnP input).
- **best**: the best single slice's PnP inliers.
- **pooled → final**: the inliers gated by that seed pose, then after
  Gauss-Newton.
- **iF**: distinct frames among the final inliers.

| pano | outcome | frame kp | depth % | ratio | corr | best | pooled → final | RMS ° | delta m | iF |
|---|---|---:|---:|---:|---:|---:|---|---:|---:|---:|
| 003 | aligned | 23998 | 63 | 13535 | 8477 | 13 | 49 → 60 | 0.61 | 2.49 | 6 |
| 004 | too_few_pooled_inliers | 23883 | 47 | 13596 | 5186 | 6 | 9 → – | | | |
| 005 | no_slice_pnp | 23901 | 33 | 12313 | 4180 | 0 | – | | | |
| 006 | too_few_pooled_inliers | 23885 | 25 | 13156 | 3018 | 15 | 14 → – | | | |
| 007 | upside_down (dot −0.86) | 23960 | 94 | 11650 | 11118 | 15 | – | | | |
| 008 | upside_down (dot −0.31) | 23957 | 86 | 10435 | 9151 | 15 | – | | | |
| 009 | correction_gate | 17884 | 69 | 11393 | 8669 | 14 | 59 → 58 | 0.58 | 6.90 | 6 |
| 010 | **aligned** | 20023 | 93 | 14601 | 13832 | **62** | 461 → 649 | 0.39 | **0.58** | 1 |
| 011 | correction_gate | 19605 | 44 | 14865 | 8249 | 36 | 183 → 214 | 0.67 | 7.57 | 6 |
| 012 | upside_down (dot −0.34) | 19799 | 100 | 9891 | 9891 | 9 | – | | | |
| 013 | aligned | 22008 | 100 | 8628 | 8628 | 9 | 44 → 51 | 0.51 | 4.16 | 6 |
| 014 | correction_gate | 18367 | 100 | 9549 | 9549 | 8 | 43 → 52 | 0.71 | 11.25 | 8 |
| 015 | too_few_pooled_inliers | 10184 | 68 | 6468 | 4829 | 14 | 36 → – | | | |
| 016 | upside_down (dot −0.52) | 13514 | 100 | 2206 | 2206 | 12 | – | | | |
| 017 | refinement_collapsed | 10022 | 100 | 1664 | 1664 | 11 | 42 → – | | | |
| 018 | **aligned** | 9885 | 100 | 1729 | 1729 | **28** | 92 → 101 | 0.49 | **1.00** | 2 |
| 019 | upside_down (dot −0.97) | 18917 | 100 | 12533 | 12516 | 14 | – | | | |
| 020 | upside_down (dot −0.20) | 7024 | 100 | 6719 | 6719 | 14 | – | | | |

Outcome counts: 4 aligned (2 credible), 6 upside_down, 3
too_few_pooled_inliers, 3 correction_gate, 1 refinement_collapsed, 1
no_slice_pnp.

The library defaults (`--window 25 --max-candidates 20 --min-inliers 25`)
accept 5, of which only `010` is credible: `007` 4.28 m, `011` 2.00 m, `003`
1.21 m, `018` 2.69 m. They also produce a 57 699 m "correction" on `008` that
the gate rejects. The panorama-integration write-up reported 6/18; which
panoramas pass shuffles with every parameter, and that is itself a symptom.

## Failure mode 1: no signal above the noise floor

**The null model.** In the throwaway patch, each slice's world points were
shuffled among that slice's own correspondences. Everything else stayed
identical: same sets, same counts, same seed. True geometry was removed.

| run | best-slice PnP inliers, 18 panoramas | outcomes |
|---|---|---|
| CLI defaults, real | 13 6 0 15 15 15 14 **62** **36** 9 9 8 14 12 11 **28** 14 14 | 4 A, 6 U, 3 P, 3 G, 1 R, 1 N |
| CLI defaults, **shuffled** | 0 0 0 6 15 15 15 10 0 6 10 6 8 12 9 10 8 17 | 0 A, 9 U, 5 P, 4 N |
| 20 frames, **shuffled** | 6 6 0 7 22 27 10 12 11 9 11 11 21 26 15 8 20 20 | 6 G (pooled 54–262 @ 0.73–0.77°), 8 U, … |

(A = aligned, U = upside_down, P = too_few_pooled_inliers, G =
correction_gate, R = refinement_collapsed, N = no_slice_pnp.)

Only 3 panoramas (`010`, `011`, `018`) beat the CLI-default noise ceiling of
17. The rest are indistinguishable from noise, and the gate mix the noise
produces is the same mix the real data produces. In particular, **the
`upside_down` gate (#364) is mostly not a degeneracy of good data.** It is a
coin flip on a random pose: about half of all random rotations put the
panorama's down axis against the seed's.

**Why the pool is junk.** Every slice keypoint (3000 per slice × 10 slices)
takes its nearest neighbour in every candidate frame. The Lowe test at 0.85
then passes **40–50 % of slice keypoints in every slice**, including slices
that face directions no candidate frame sees. Example, `010`:

- Slice 5 (yaw 225°) yields 1421 ratio-passing matches and 0 inliers.
- All 649 final inliers come from one frame (3017) and two slices.

The nadir cap contributes 12 989 correspondences (10 % of the pool over all
18 panoramas) and **zero** final inliers: it shows the operator, the hand and
the stick. Across the panoramas the final inliers are 1–5 % of the
correspondences. `solvePnPRansac` (EPnP, 500 iterations, 5-point samples) has
essentially no chance of drawing an all-inlier sample at that rate. It finds
the real pose only when one slice happens to hold a dense, coherent cluster
(`010`).

**What the inputs look like.** For 6 of the 18 panoramas, the candidate
frames (±4 by id) span only 3–29° of viewing direction. The iPad is carried tilted
down: half of most frames is bare floor. Frames taken while walking are
visibly motion-blurred. The seed frame for `010` (3022) is blurred; frame
3017, 3.3 s earlier and sharp, carries every inlier. The frames see a ~50°
wedge of the sphere, so at most 1–2 of the 10 slices can match at all.

## Failure mode 2: genuine signal, ill-conditioned translation

When real matches exist, they come from 1–3 frames that all look the same way.
A narrow match cone is weakly constrained in translation by bearing
resection, so the centre slides along the viewing direction while the angular
RMS stays small. Examples:

- `011`: 36 best-slice inliers, delta 7.6 m (CLI defaults); 2.0 m (library
  defaults); 44 m (depth ≤ 20 m).
- `006` with 12×60° slices: 440 inliers from **one** frame, 0.15° RMS, delta
  3.8 m.

The fixed-translation mode (`--fix-translation`) fixes the centre, but it
admits correspondences at 5.7°. With that gate, `010` "gains" 1964 inliers at
2.9° RMS, which means the loose gate is soaking up junk. It is no better
overall: 3 aligned.

## Failure mode 3: depth clipped at 6 m in large rooms

The LiDAR depth stored for NewOffice extends well beyond 6 m. With
`max_depth = 20` (throwaway patch):

| | 004 | 006 | 010 | 011 |
|---|---:|---:|---:|---:|
| depth coverage (6 m → 20 m) | 47 → 100 % | 25 → 100 % | 93 → 100 % | 44 → 100 % |
| best-slice inliers (6 m → 20 m) | 6 → **46** | 15 → **43** | 62 → 83 | 36 → 56 |

`004` goes from noise to aligned at 1.9 m. Overall the count moves from 4 to
5 aligned and stays at 2 credible: the newly found signal runs straight into
failure mode 2. Far depth is less accurate, so this is a contributing input
problem, not the root cause.

## Candidate causes tested

| candidate | test | result | verdict |
|---|---|---|---|
| Seam artefacts at the dual-fisheye blend | Angular distance of every correspondence and final inlier to the stitch seam (great circle ⟂ the front-lens axis, yaw 28.8°/pitch 15.5° from `io/insta360_x4.cpp`'s calibration) | ±10° seam band: 17.4 % of the sphere, 12.7 % of correspondences, **16.2 %** of final inliers | **Rejected.** Inliers are not depleted near the seam. |
| Stitch quality (Hugin `.jpg` vs our `.insp` stitcher) | Re-import the 18 raw `.insp` through `rux import 360` into a copy; same runs | CLI defaults 3 aligned / 2 credible (vs 4 / 2); library defaults 0 (vs 5 / 1); best-slice counts in the same noise range | **Rejected.** Our stitch is no better and no worse in a way that matters. |
| Depth coverage of matched frames | `depth %` per panorama; `max_depth` 6 → 20 m | Hall panoramas lose 53–75 % of keypoints; at 20 m, `004` and `006` rise above noise | **Contributing** (failure mode 3) |
| Slice tiling overlap/count | `n_yaw`×`fov` 8×90 (default), 12×60, 16×45, 4×120, 12×90; slice 1008 → 2016 px; no pole caps | Aligned / credible: 4/2, 5/4, 2/0, 2/1, 5/1; 2016 px: 2/1; no caps: 4/2 | **Not the cause.** 12×60 is the best single setting, but which panoramas pass is not stable across settings. |
| Matcher weakness (descriptor) | ORB ratio 0.7; mutual check; mutual + 0.8; SIFT (0.8); AKAZE (0.8) | Aligned / credible: 4/1, 5/2, 5/1, 3/2, 4/2; every variant leaves 13+ panoramas at noise level | **Not fixable by swapping the hand-crafted descriptor** |
| RANSAC budget | 500 → 5000 iterations | 7 aligned / 3 credible; best-slice inliers rise on `008`/`009`/`010`/`014`/`015` | **Contributing**, consistent with failure mode 1's inlier-rate argument |
| Candidate window (clock offset) | `pano − seed dt` ≤ 1.7 s for all 18; ±25-frame window | More frames add junk faster than signal: 5 aligned / 1 credible | **Rejected as clock skew.** Frame *selection* matters, but by quality, not by time. |

## Does the #264 / panorama-loop-closure finding transfer?

**Partly.**
[`panorama-loop-closure.md`](panorama-loop-closure.md) concluded that
cross-camera ORB resections are too weak, and pointed at learned matchers.

- **What carries over:** hand-crafted descriptors do not bridge the 360-camera
  / iPad appearance gap. SIFT and AKAZE are no better than ORB here.
- **What does not:** most of the 18 failures happen *before* matcher quality
  could matter:
  - the pool is built so that most slices cannot contain a true match, yet
    they contribute junk at a 40–50 % ratio-pass rate;
  - the candidate frames are blurred, floor-facing, and from one direction;
  - the acceptance gates (`min_slice_correspondences = 6`, `min_inliers`, a
    5 m correction gate) sit at or below the noise floor, so noise poses can
    be accepted (`003`, `013`) and real ones rejected.

  A learned matcher would raise the true-match count. It would not fix the
  acceptance logic, and with the current pooling its junk would still dominate
  the slices that face away from the frames.
- **What alignment has that loop closure does not:** a strong prior. The
  panorama is hand-held within about 1 m of the iPad and within 2 s of the
  seed frame. Any fix should use it (gate the centre near 1 m, not 5 m).
  `--fix-translation` exists, but its 5.7° gate undoes the benefit.

The epic's other lead, an IMU/gravity prior from the raw `.insp`
(reusex-bx2.2), would remove the `upside_down` coin flip outright, and with it
two of the six rotational degrees of freedom.

## Follow-ups filed (under reusex-bx2)

Each bead carries its evidence from this document. (1, **reusex-bx2.4**) Calibrate acceptance against a noise floor, and gate the
centre on the hand-held prior. (2, **reusex-bx2.5**) Build the correspondence pool so junk
cannot dominate: frame-driven matching, ratio test across the whole sphere,
drop the nadir, a larger or USAC RANSAC budget. (3, **reusex-bx2.6**) Select candidate frames
by sharpness, texture and view diversity, and lift features beyond 6 m with
an uncertainty model. (4, **reusex-bx2.7**) A bug: the `rux align 360` CLI defaults diverge from
`PanoramaAlignmentOptions` (STANDARDS §4). Evaluate a learned matcher only
after (1)–(3), so that it is measured against calibrated gates.

## Follow-up: calibrated acceptance (reusex-bx2.4, 2026-10-15)

Two gates were changed in response to this write-up:

- **Noise floor (`below_noise_floor`).** For each panorama, the per-slice PnP
  is re-run 5 times on the panorama's own correspondences, with each slice's
  2 px location groups permuted. The real best slice must then beat 1.5× the
  best null count. Permuting single pairs is not enough: the pool holds runs
  of near-identical pairs, and a per-pair shuffle underestimates the floor.
  On `003` the per-pair null gave 7 and the grouped null 14.
- **Hand-held prior.** `max_correction_m` goes from 5 m to 1.5 m.

The library and CLI defaults are now one set, window 12 and 8 candidates
(reusex-bx2.7). Same NewOffice copy, `--dry-run`:

| pano | before (CLI defaults) | after | best / needed | delta m |
|---|---|---|---:|---:|
| 003 | aligned (2.49 m) | below_noise_floor | 13 / 22 | – |
| 004 | too_few_pooled_inliers | below_noise_floor | 6 / 14 | – |
| 005 | no_slice_pnp | no_slice_pnp | 0 / – | – |
| 006 | too_few_pooled_inliers | too_few_pooled_inliers | 15 / 10 | – |
| 007 | upside_down | below_noise_floor | 15 / 26 | – |
| 008 | upside_down | below_noise_floor | 15 / 22 | – |
| 009 | correction_gate | below_noise_floor | 14 / 16 | – |
| 010 | **aligned** | **aligned** | 62 / 35 | 0.58 |
| 011 | correction_gate | correction_gate | 36 / 11 | 7.57 |
| 012 | upside_down | below_noise_floor | 9 / 20 | – |
| 013 | aligned (4.16 m) | below_noise_floor | 9 / 17 | – |
| 014 | correction_gate | below_noise_floor | 8 / 23 | – |
| 015 | too_few_pooled_inliers | too_few_pooled_inliers | 14 / 14 | – |
| 016 | upside_down | below_noise_floor | 12 / 22 | – |
| 017 | refinement_collapsed | below_noise_floor | 11 / 23 | – |
| 018 | **aligned** | **aligned** | 28 / 19 | 1.00 |
| 019 | upside_down | below_noise_floor | 14 / 16 | – |
| 020 | upside_down | below_noise_floor | 14 / 32 | – |

- **After:** 2 aligned, both credible, against 4 aligned / 2 credible before.
  The two false positives, `003` and `013`, now fail the noise floor.
- **Prior off** (`--max-correction 0`): `011` is the only addition, at
  7.6 m. It is genuine signal with ill-conditioned translation (failure
  mode 2).
- **Window 25 / 20 candidates:** 1 aligned (`010`). More frames raise the
  noise floor (null up to 30) faster than they raise the real support.
- **Research null re-run** (pairings shuffled, the original patch): 0 accepted
  at both window settings, also with the prior off. 14 to 17 of 18 stop at
  `below_noise_floor`; the rest have no PnP at all.
- **Determinism:** two runs give byte-identical diagnostics.

`upside_down` no longer fires at the defaults. All six panoramas it used to
stop are now rejected earlier, as noise. This supports the reading above
that it was a coin flip on random poses.

## Follow-up: the correspondence pool (reusex-bx2.5, 2026-10-15)

The pool is now built from the panorama's keypoints **pooled over the whole
sphere**. Each change has its own `PanoramaAlignmentOptions` field and
`rux align 360` flag:

- **Overlap dedupe** (`dedupe_overlap`, `--[no-]dedupe-overlap`). Each
  direction of the sphere is owned by one slice: of the slices that see it,
  the one whose axis is closest. A keypoint seen by two overlapping slices
  enters the pool once.
- **Nadir mask** (`nadir_mask_deg = 45`, `--nadir-mask`). Drops the
  operator, the hand and the stick.
- **Frame-driven matching** (`frame_to_sphere`, `--[no-]frame-to-sphere`).
  Each frame keypoint takes its nearest panorama keypoint over the whole
  sphere. It is Lowe-tested against the nearest keypoint at a *different*
  place (`distinct_deg = 1`), so ORB's duplicate detection of the same
  corner is not its own rival.
- **One correspondence per panorama keypoint** (`one_per_keypoint`). The
  lowest Hamming distance over all candidate frames is kept.
- **USAC** (`usac`, `--[no-]usac`). The per-slice PnP and its noise-floor
  null use OpenCV's USAC: MAGSAC++ scoring, local optimisation and 3-point
  samples, seeded by `--seed`, with 500 iterations.

With all of them off, the diagnostics are byte-identical to the bx2.4
defaults. The pool shrinks from 129 611 to 16 489 depth-backed
correspondences over the 18 panoramas.

**Measurement.** Every configuration was run on all 18 panoramas at seeds
42, 1, 2 and 3, each time with the research null (pairings shuffled). The
table counts accepted panoramas per run, all four seeds pooled. It also
counts how often the null accepted anything. Every accepted pose is within
the 1.5 m prior by construction.

| configuration | accepted / run | null accepts | notes |
|---|---:|---:|---|
| bx2.4 defaults (before) | 2.0 | 0 | `010`, `018` |
| **all five (new defaults)** | **5.0** | **0** | `006` `010` `011` `019` 4/4, `013` 3/4, `009` 1/4 |
| without the nadir mask | 3.2 | 0 | loses `011`, `013` |
| without USAC | 2.0 | 0 | only `006`, `010` |
| without frame-driven matching | 4.0 | 0 | loses `019` |
| without one-per-keypoint | 6.8 | 0 | adds `007`, `008`, `015` at best/null ≈ 2; `008`'s centre moves 1.9 m between seeds |
| without the overlap dedupe | 5.5 | 0 | loses `006`, adds `008` (1.05 m spread) and `012` |
| nadir mask 30° / 55° | 4.0 / 2.0 | 0 | 45° is best |
| USAC alone | 4.8 | 0 | `010` centre moves 0.9 m between seeds |
| EPnP at 5000 iterations alone (seed 42) | 3 | – | 7× slower than 500 |
| USAC at 2000 iterations | 4.2 | 0 | steadier across seeds, fewer accepts |
| zenith mask 45° alone (seed 42) | 2 | – | no effect, dropped |
| slices PnP every pooled match in view (not only owned ones) | 3.5 | 0 | dropped |

**Margin.** At the default seed, the six accepted panoramas beat their noise
floor by 2.8× to 10.8× (best-slice / null inliers). Before, the two accepted
panoramas beat it by 2.7× (`010`) and 2.3× (`018`). The shuffled null now
reaches at most 11 to 15 best-slice inliers, against 23 before. Runs are
byte-identical when repeated.

**Consistency.** The poses agree across seeds and across configurations,
which is the closest thing to ground truth here:

- `010` lands at 0.45 m from the seed in every run of every
  frame-driven configuration, all within 0.02 m of each other. The old
  0.58 m pose sits 0.8 m from that consensus.
- The other accepted centres agree within 0.2 m to 0.8 m across seeds.

**Regression: `018`.** It is no longer accepted. Its old acceptance rested on
two overlapping slices (2 and 3) that each counted the same matched
keypoints, plus 1905 of its 3044 keypoints in the nadir. Deduplicated, its
best slice holds 17 inliers against a null of 11 (1.5×), at the floor. Its
pooled support is 21, below `min_inliers = 40`. Lowering `min_inliers` to
20–25 recovers `018` in 1 of 4 seeds. It also admits `008`, whose centre
then moves 1.6 m between seeds. So `min_inliers` was kept at 40.

**Still open.**

- `007` and `012` clear the noise floor by 3.8× and 5.1× but fail
  `min_inliers = 40` on the deduplicated pool (30 and 36 pooled inliers).
  That threshold was set on the old, double-counted pool. A pooled gate
  calibrated against the null, like the best-slice gate, would be the
  principled replacement.
- Most accepted poses draw their final inliers from **one** candidate frame,
  so the translation is still poorly conditioned (failure mode 2). The
  candidates that do carry matches are rarely the seed frame. That is
  reusex-bx2.6's ground: frame selection by sharpness and view diversity.

## Reproduce

```bash
cp ~/repos/NewOffice/project.rux /tmp/no.rux
rux -vv -p /tmp/no.rux align 360 --dry-run --overwrite \
    --diagnostics diag.json --diagnostics-correspondences
# the bx2.4 pool, for comparison
rux -vv -p /tmp/no.rux align 360 --dry-run --overwrite --no-usac \
    --no-dedupe-overlap --nadir-mask 0 --no-frame-to-sphere \
    --no-one-per-keypoint --diagnostics before.json
```

The null model, matcher, depth-range and RANSAC experiments came from an
uncommitted, environment-variable patch. Its knobs are listed in the notes of reusex-bx2.1, so they can be re-created if needed.
