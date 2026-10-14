<!--
SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
SPDX-License-Identifier: GPL-3.0-or-later
-->

# `tools/loop_edges` — external wide-baseline loop-edge exporter

Out-of-process producer for `rux graph discover --loop-edges` (issues #221 / #225 P2).
Runs an image matcher in Python, lifts the 2D matches to metric 3D using the
scan's own stored depth, estimates a robust relative pose per candidate pair,
and writes a JSON file (schema `reusex.loop_edges.v1`) that the C++ optimizer
ingests as **data**. See
[`docs/research/loop-closure-learned-matchers.md`](../../docs/research/loop-closure-learned-matchers.md).

## Why out-of-process

Two problems with putting a learned matcher directly into the GPL C++ binary:

1. **Integration** — the best matchers are Python-only research code.
2. **Licensing** — MASt3R and MapAnything's 13-dataset checkpoint are
   **CC-BY-NC** (non-commercial); ReUseX is a commercial product.

Both vanish if the matcher runs out-of-process and hands the C++ a plain JSON of
relative-pose constraints. A non-commercial model can then act as an offline
accuracy-**ceiling oracle** without ever entering the shipped binary, while a
commercial-safe matcher writes the identical file for production.

## Matcher backends

| backend | licence | use |
|---|---|---|
| `orb` | BSD (OpenCV) | zero-dependency baseline / parity check; ships as the C++ default |
| `xfeat` | Apache-2.0 | commercial-safe learned features |
| `lightglue` | Apache-2.0 | commercial-safe (LightGlue + ALIKED/DISK — **not** SuperPoint, which is NC) |
| `mast3r` | **CC-BY-NC-SA** | **oracle only**; needs `--allow-noncommercial` |
| `mapanything` | Apache-2.0 (`--variant apache`, default) or CC-BY-NC (`--variant nc`) | pointmap model, **not** a descriptor matcher: correspondence = mutual nearest 3D neighbour between the two per-view pointmaps. Finds matches on blank walls where descriptors have nothing to key on; see the caveats in `docs/research/loop-closure-learned-matchers.md` §5.5 |

**How to compare two backends (#312).** Rank them on **edges that survive PCM**
and the **correction actually applied** (`pcm_kept` / `max_shift_m` in
`scripts/bench-loop-edges.sh`'s `summary.tsv`) — never on the edge count or the
Σ inliers of the exported JSON. Those are per-pair confidence; the pose graph
consumes cross-pair *consistency*, and the two can invert. MapAnything led the
office endcap set on Σ inliers (9203, above the NC oracle) and PCM rejected 145
of its 148 edges, applying 0.21 m where ~16 m was needed. A matcher that
abstains on an unmatchable surface is doing the right thing.

**Commercial rule:** output from `mast3r` or `mapanything --variant nc` is an
evaluation artefact. It must not be bundled into or shipped with a commercial
deliverable. The tool prints a banner and gates these behind
`--allow-noncommercial`. `mapanything` with the default `--variant apache`
needs no gate: Meta ships that checkpoint under Apache-2.0 precisely so it can
be used commercially (verified 2026-09-09 — the `license:` field on
[`facebook/map-anything-apache`](https://huggingface.co/facebook/map-anything-apache)
reads `apache-2.0`, against `cc-by-nc-4.0` on `facebook/map-anything`, and the
upstream README names the apache checkpoint as the commercial one).

## Environment

Each learned backend has its own venv (dependency isolation). `orb` needs only
`numpy` + `opencv-python` (already present in the dev shell). Venvs live under
`~/loop-edges-work/<backend>/`, **outside the repo** — never committed, and no
model weights land in the tree either (they cache under `~/.cache/huggingface`
and `~/.cache/torch/hub`; see `models/README.md` for the project convention).

`mapanything`, following the venv recipe in `python/README.md` — prebuilt
wheels, never a Nix source build of torch:

```bash
W=~/loop-edges-work/mapanything && mkdir -p $W
python3 -m venv $W/.venv
$W/.venv/bin/pip install torch==2.11.0 torchvision==0.26.0 \
    --index-url https://download.pytorch.org/whl/cu128
git clone --depth 1 https://github.com/facebookresearch/map-anything.git $W/map-anything
$W/.venv/bin/pip install -e $W/map-anything     # pulls uniception, timm, scipy
```

On NixOS `import torch` then fails with `libstdc++.so.6: cannot open shared
object file` until the loader can see gcc's libs, the driver, and torch's
bundled CUDA libs — the same gotcha `docs/sam3.1-export-guide.md` §1.2
documents. Write `$W/env.sh` once and source it before every run:

```bash
export MA_PY=~/loop-edges-work/mapanything/.venv/bin/python
GCC=$(ls -d /nix/store/*gcc-14.3.0-lib/lib | head -1)
NVLIBS=$(echo ~/loop-edges-work/mapanything/.venv/lib/python*/site-packages/nvidia/*/lib | tr ' ' ':')
export LD_LIBRARY_PATH="$GCC:/run/opengl-driver/lib:$NVLIBS"
```

First run downloads the checkpoint (~5 GB) into the HF cache and takes ~2 min
to load; inference is ~1 s per pair on an RTX 6000 Ada at ~9 GB VRAM.

## Usage

```bash
# ORB baseline (no GPU needed)
python3 tools/loop_edges/export_loop_edges.py PROJECT.rux \
    -o edges-orb.json --matcher orb --proposal exhaustive --min-frame-gap 50

# XFeat, commercial-safe (GPU)
~/loop-edges-work/xfeat/.venv/bin/python tools/loop_edges/export_loop_edges.py \
    PROJECT.rux -o edges-xfeat.json --matcher xfeat

# Target a start<->end drift loop the spatial proposer is blind to
... --proposal endcap --band-frac 0.15

# Intra-building revisits: same room, different time (--proposal spatial)
# Proposes pairs whose seed-pose camera centres are within --spatial-radius metres
# and whose frame indices differ by at least --min-frame-gap.
# DRIFT CAVEAT: if accumulated drift > room radius, revisits may appear far apart
# in the seed poses; increase --spatial-radius (5-8 m) or use appearance retrieval.
~/loop-edges-work/mast3r/.venv/bin/python tools/loop_edges/export_loop_edges.py \
    PROJECT.rux -o edges-spatial.json --matcher mast3r --allow-noncommercial \
    --proposal spatial --spatial-radius 3.0 --min-frame-gap 300 \
    --stride 4 --max-pairs 2000

# MASt3R oracle (research/eval only)
~/loop-edges-work/mast3r/.venv/bin/python tools/loop_edges/export_loop_edges.py \
    PROJECT.rux -o edges-mast3r.json --matcher mast3r --allow-noncommercial

# MapAnything, apache checkpoint — commercial-safe, no gate needed (source env.sh first)
$MA_PY tools/loop_edges/export_loop_edges.py PROJECT.rux \
    -o edges-mapanything.json --matcher mapanything --proposal endcap

# Look at what a backend actually proposes before trusting a table of counts.
# Green = survives the same RANSAC the exporter runs; red = rejected.
$MA_PY tools/loop_edges/visualize_matches.py PROJECT.rux \
    -o matches.jpg --pair 7:231 --pair 11:224 --matchers orb,mapanything

# Feed edges into the pose graph (license-clean; C++ never links a matcher)
rux -p PROJECT.rux graph discover --no-loop-closure --loop-edges edges-xfeat.json
rux -p PROJECT.rux optimize --loop-trust --odometry-sigma-trans 0.05
rux -p PROJECT.rux create clouds
```

## Edge file schema (`reusex.loop_edges.v1`)

```json
{
  "schema": "reusex.loop_edges.v1",
  "producer": "xfeat",
  "edges": [
    { "node_i": 12, "node_j": 230,
      "T_ij": [ 16 doubles, row-major 4x4 = pose(i)^-1 * pose(j) ],
      "sigma_rot": 0.03, "sigma_trans": 0.05, "inliers": 120 }
  ]
}
```

`node_i` / `node_j` are `sensor_frames.node_id`. `T_ij` maps a point from frame
`j`'s optical frame into frame `i`'s optical frame (optical→world convention).
Edges referencing unknown node ids, self-loops, duplicates, or malformed entries
are skipped with a warning; an unreadable/invalid file fails loudly.
