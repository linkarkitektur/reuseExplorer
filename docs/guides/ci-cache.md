# CI binary cache (`reusex.cachix.org`) and local-first CI

**In short (reusex-37v):** build and test locally with `scripts/ci-local.sh`
before pushing a PR; CI then substitutes the result from Cachix and compiles
nothing. The sections "Build locally, CI confirms" and "Local ccache" below
describe that flow; the rest of this page is about the cache itself.

`.github/workflows/ci.yml` builds `.#checks.x86_64-linux.tests` — the CPU
variant of ReUseX with `BUILD_TESTS=ON`, plus `ctest` — on stock GitHub-hosted
runners. That is only possible because the parts of the dependency closure
that `cache.nixos.org` cannot serve come pre-built from our own Cachix cache:

- **Cache**: <https://reusex.cachix.org>
- **Visibility**: public. Reads need no token; `nix` fetches from it as an
  ordinary substituter. `flake.nix`'s `nixConfig` already declares it, so a
  local `nix build` offers to use it the first time you accept the flake
  config; CI opts in with `accept-flake-config = true`.
- **Public key**:
  `reusex.cachix.org-1:0+y68O2+rBxXytqIepgqOSRsWa+5Ccpst9weS0sUBak=`
- **Plan**: Cachix free tier, **5 GB** of (compressed) storage. Everything
  below exists to stay inside that number.
- **Write credential**: `CACHIX_AUTH_TOKEN`, an **environment** secret on the
  `CI` environment of the repo — *not* a repo-level secret. A job that does
  not declare `environment: CI` sees `secrets.CACHIX_AUTH_TOKEN` as the empty
  string and silently degrades to a read-only, push-skipping run.

## Build locally, CI confirms (reusex-37v)

A full CI build + ctest took 25-45 min per PR on a 4-vCPU runner. It does not
have to: `checks.tests` is a pure derivation, so once anyone has realised it
and pushed the output to `reusex.cachix.org`, every other machine evaluating
the same derivation can substitute it instead of building it.

```bash
# On a clean, committed tree, before pushing (or before opening the PR):
scripts/ci-local.sh          # build .#checks.x86_64-linux.tests for HEAD, push the stamp
scripts/ci-local.sh --check  # would CI on HEAD be a cache hit?
scripts/ci-local.sh --no-push
```

What each piece does:

- **The output is a pass stamp** (`flake.nix`, `checks.tests`): once the
  build, `ctest` and the package install check have passed inside the
  sandbox, `$out` is replaced by `ctest.log` and a `passed` file. No runtime
  references, a few KiB. Its presence in a store means "this exact derivation
  built and passed". The installable package is `.#cpu`, unchanged.
- **`scripts/ci-local.sh`** refuses a dirty tree (the hash would include
  uncommitted changes CI never sees), prints the commit, derivation and
  output path, skips everything if the stamp is already in the cache, builds
  with `--cores 8 --max-jobs 2` (`CI_LOCAL_CORES` / `CI_LOCAL_MAX_JOBS`;
  the dev machine had RAM faults at higher `-j`), and pushes the one stamp
  path with `cachix push reusex`. It needs a write token: `CACHIX_AUTH_TOKEN`,
  or `cachix authtoken <token>` once (stored in
  `~/.config/cachix/cachix.dhall`); without one it says so before building.
  `cachix` comes from the flake's nixpkgs if it is not on `PATH`.
- **`ci.yml`** checks out the **PR head commit** (`pull_request.head.sha`),
  not GitHub's synthetic `refs/pull/N/merge`, because the head is what was
  built locally. It prints the derivation, then runs
  `nix build --max-jobs 0` — substitute only, never build. On a hit the job
  logs `checks.tests substituted` (a `::notice`) and `copying path
  '…-ReUseX-tests-…' from 'https://reusex.cachix.org'`, and is done. On a miss
  it logs `checks.tests built` and runs the full build exactly as before.
- **Warning issues** (`scripts/file_build_warnings.py`) are synced only after
  a real build on `main`: a substituted run has no compiler log, and an empty
  log would read as "every warning fixed".

To make this automatic before every push, an alias is enough (deliberately
not a pre-push hook: git-hooks.nix and bd own the hooks directory, and
pre-commit hides a long hook's output until it ends):

```bash
git config --local alias.cipush '!scripts/ci-local.sh && git push'
git cipush -u origin my-branch
```

### Why the local and CI derivations are identical

Nothing commit-specific enters the derivation:

- `src` is `lib.fileset.toSource` over the paths the build and the tests
  read (`default.nix`): `CMakeLists.txt`, `cmake/`, `LICENSE.md`, `libs/`,
  `apps/rux/`, `bindings/`, `tests/`, `docs/CONTRACTS.md`,
  `python/reusex_sam3/engine-build.json`, and the two
  `scripts/*bench_solid_compare*` files one ctest runs. The store path of the
  source is a hash of those files' contents only — not of the commit, its
  message or the rest of the tree. The nix files (`flake.nix`, `default.nix`,
  `overlays/`, `pkgs/`, `flake.lock`) matter only through what they evaluate
  to.
- LFS fixtures are real content on both sides: CI checks out with
  `lfs: true`, and a local `git+file` flake reads the smudged working tree.
- The CPU variant is selected by attribute name (`checks.tests` is
  `packages.cpu` + checks), so the dev machine's CUDA default never leaks in.
- The ccache wrapper (`overlays/ccache.nix`) is part of the derivation on
  both sides; whether `/var/cache/ccache` exists is decided at build time,
  inside the builder, so it does not affect the hash.

Measured on this branch (commit `f12c67fc`):

| Change on top of the commit | `checks.tests` derivation |
|---|---|
| none (clean tree) | `084ds3wkbsba95xqvdq30jd49cn5qiyn` |
| `docs/DIRECTION.md`, `CLAUDE.md`, `.github/workflows/ci.yml`, `scripts/check.sh` edited | `084ds3wkbsba95xqvdq30jd49cn5qiyn` (unchanged) |
| one comment line appended to `libs/reusex/src/core/logging.cpp` | `0ppbs1jig1qhr6i4akd29mynjcas9jl5` (changed) |

CI-vs-local: the CI job prints the derivation it evaluated; it must equal `nix path-info --derivation .#checks.x86_64-linux.tests` run locally on the same commit (see the PR that introduced this, below).

The flip side: a file that a test reads at run time but that is outside the
fileset exists in a dev build tree and not in the sandbox, so that test fails
only under nix. `bench_solid_compare_selftest` did exactly that from the day
it was added — unnoticed because of the next point.

### `pipefail`: CI was green on test failures until reusex-37v

GitHub runs a `run:` step without an explicit `shell:` as `bash -e {0}` —
**no `pipefail`**. The build step was `nix build … | tee build-warnings.log`,
so its exit status was `tee`'s, always 0. Every PR run and main run checked
on 2026-10-16 had `99% tests passed, 1 (or 2) tests failed` and `error:
Cannot build '…-ReUseX-tests-0.0.5.drv'` in the log, yet concluded
`success` (e.g. runs 38826697173, 38830354555): `bench_solid_compare_selftest`
every time, `JobRunner_Listener_ObservesFullLifecycle` sometimes (reusex-dkt).
The step now declares `shell: bash` (`-eo pipefail`).

### Trust

A substituted stamp means "someone holding a reusex.cachix.org write token
built this derivation and it passed". Cachix signs what it serves, so the
stamp cannot be forged by a third party, but a token holder could push
anything under that path. That is the same trust CI already placed in the
dependency paths in the cache. The full build is still what runs on any
commit nobody pushed a stamp for, and on every `main` push whose merge
commit differs (in build-relevant files) from the PR head that was checked.

### Alternatives considered

| Option | CI time, warm / cold | Guarantees | Cost |
|---|---|---|---|
| **Cachix-substituted stamp** (chosen) | ~2-3 min when `ci-local.sh` ran / full build otherwise | hermetic (same derivation as CI), bound to exact inputs | one script; developer runs it (≈ a local nix build, ccache-warm) |
| Dev-shell CI + ccache via `actions/cache` | ~10 min (restore + link + ctest ~4 min) / ~25 min | not hermetic (dev-shell build, impure env); needs a CPU-only CI devshell, ccache dir in the 10 GB Actions cache | moderate; CI still compiles and runs ctest every time |
| "Attestation": `ci-local.sh` posts a commit status, CI only lints | seconds / — | only the developer's word; nothing binds the status to what was built | low; but strictly weaker than the stamp, which costs the same |

The stamp gives the attestation's CI time with a guarantee bound to the
derivation's inputs, so (3) adds nothing. (2) would speed up the fallback
path only; the same effect is reachable without giving up hermeticity by
restoring a ccache directory into the CI sandbox (`extra-sandbox-paths` on
the runner, plus `actions/cache`) — not done yet, tracked as a follow-up.

## Local ccache

### Dev shell (`cmake` builds in any checkout)

`CMakeLists.txt` uses ccache as the compiler launcher. Until reusex-37v the
cache ran on ccache's defaults, and the numbers showed it: 9 % hit rate over
81k compiles, the 5 GiB default size full with 1640 cleanups. Every worktree
has its own absolute path, ccache hashes absolute paths, so worktrees never
shared an object and evicted each other's instead.

`shell.nix` now sets, for every compile run from the dev shell:

| Variable | Value | Why |
|---|---|---|
| `CCACHE_BASEDIR` | the directory holding the shared `.git` (the main checkout) | absolute paths under it are rewritten relative to the build dir, so `.worktrees/a/build` and `.worktrees/b/build` hash the same `../libs/…`; covers the main checkout and every `.worktrees/*`. A worktree outside it gets its own toplevel. Set in `shellHook`; an explicit value wins. |
| `CCACHE_NOHASHDIR` | `1` | do not hash the working directory (only matters with `-g`, i.e. Debug/RelWithDebInfo) |
| `CCACHE_COMPILERCHECK` | `content` | nix store files all have mtime 1, and two gcc-wrapper scripts with same-length store paths have the same size, so the default `mtime` check could match a different compiler |
| `CCACHE_MAXSIZE` | `50G` | one full CUDA build with tests and the Qt client is a few GiB compressed; ten or so build generations across worktrees fit, ~14 % of the dev machine's free disk (355 GiB) at the time |

Two compile definitions defeated sharing even with `CCACHE_BASEDIR`, because
ccache hashes the command line verbatim and does not rewrite paths inside
`-D` values: `REUSEX_SOURCE_DIR="/abs/path"` on both test executables (every
test TU) and `RUX_QT_SOURCE_DIR` / `RUX_QT_TOKENS_CSS` on `rux_qt_lib`. Each
now sits on the one TU that reads it (`tests/support/source_dir.cpp`,
`apps/rux/qt/src/Theme.cpp`). **Keep absolute paths out of target-wide
compile definitions.**

Measured (isolated cache directories, so the numbers are only these builds;
fresh build dir in each tree, `-j8`, other agents building on the machine at
the same time):

| Settings | Target | Tree 1 (cold cache) | Tree 2 (other checkout, same commit) |
|---|---|---|---|
| before (ccache defaults) | `reusex_segmentation` | 54 compiles, 0 hits, 121 s | **0 / 54 hits (0 %)**, 123 s |
| after (`shell.nix` + the two `-D` moves) | `reusex_unit_tests` (306 compiles) | 0 hits, 737 s | **305 / 306 hits (99.7 %)**, 12 s |

The one miss in tree 2 is `tests/support/source_dir.cpp`, the TU that
carries the absolute path by design. Tree 2 was a clone in a scratch
directory (its own toplevel as base dir), which is the harder case; a
`.worktrees/*` checkout shares the main checkout's base dir.

### Inside the nix sandbox (`nix build`, `scripts/ci-local.sh`)

`packages.cpu` (and so `checks.tests`) is built with `ccacheStdenv` from
`overlays/ccache.nix`, which points ccache at `/var/cache/ccache`. On the dev
machine NixOS makes that directory (`programs.ccache`, `root:nixbld 0770`)
and exposes it to the sandbox (`sandbox-paths` contains `/var/cache/ccache`;
check with `nix config show | grep sandbox-paths`). Where it is missing — a
GitHub runner — the wrapper falls back to an empty per-build cache, which is
why the derivation and its hash are the same on both sides. The sandbox
always builds in `/build/source`, so no base-dir setting is needed there.

Measured with `nix build .#checks.x86_64-linux.tests --cores 8 --max-jobs 2`
on the dev machine (other builds running alongside), each after a
one-line change (a `static_assert` appended to
`libs/reusex/src/segmentation/downsample.cpp`):

| Build | configure | build | ctest | install check | wall |
|---|---:|---:|---:|---:|---:|
| first build of this branch (cache warm from older commits: 405 hits, 137 misses) | 51 s | 10 min 3 s | 3 min 36 s | 1 min 2 s | 15 min 49 s |
| one-line change, **sandbox ccache** (all build TUs hit but the changed one) | 42 s | **54 s** | 3 min 19 s | 1 min 2 s | **6 min 11 s** |
| one-line change, **no ccache** (`--option sandbox-paths` without `/var/cache/ccache`) | 40 s | 19 min 19 s | 3 min 33 s | 1 min 2 s | 24 min 50 s |

So a `scripts/ci-local.sh` run after a small change costs about six
minutes, nearly all of it ctest and the package install check, which no
cache can skip because they are the point. The stats come from a throwaway
derivation reading `/var/cache/ccache` from inside the sandbox
(`ccache --print-stats`; the directory is `root:nixbld 0770`). No `nix.conf`
change is needed on the dev machine: `sandbox-paths` already lists
`/var/cache/ccache`. On a machine without it, add to the NixOS config:

```nix
programs.ccache.enable = true;          # creates /var/cache/ccache root:nixbld 0770
nix.settings.extra-sandbox-paths = [ "/var/cache/ccache" ];
```

## What is in the cache, and why only that

The build closure of `.#checks.x86_64-linux.tests` is **1666 store paths /
9.62 GiB** of unpacked NARs. Almost all of it — compilers, Qt, VTK, boost,
ffmpeg, and even our overlaid PCL, which happens to hash-match nixpkgs — is
already on `cache.nixos.org`: when the closure was first pushed, `cachix`
reported `Pushing 18 paths (1648 are already present)`. Those 18 are the only
ones that cost us anything, and they compress to **0.53 GiB — roughly 11% of
the 5 GB tier**:

| Path | In cache (compressed) | NAR size | Why cache.nixos.org cannot serve it |
|---|---:|---:|---|
| `reusex-gui-frontend-…-npm-deps` | 338.5 MiB | 338.9 MiB | fixed-output npm dep tarball (already compressed) |
| `libtorch` | 108.6 MiB | 592.2 MiB | `pkgs/libtorch` — pinned upstream binary release |
| `opencv-4.13.0` | 29.2 MiB | 91.2 MiB | `overlays/` — non-default feature set |
| `openmvs-2.4.0` | 19.4 MiB | — | `pkgs/openmvs` — not in nixpkgs |
| `opennurbs-…` | 12.7 MiB | 44.5 MiB | `pkgs/opennurbs` — not in nixpkgs |
| `rtabmap-0.23.2` | 9.4 MiB | 30.1 MiB | `pkgs/rtabmap` — not in nixpkgs |
| `tokenizers-cpp-0.1.1` | 6.0 MiB | 29.4 MiB | `pkgs/tokenizers-cpp` — not in nixpkgs |
| `gtsam-4.2.1` | 3.8 MiB | 12.4 MiB | `overlays/` — pinned version |
| `highs-1.14.0` | ~2 MiB | 7.5 MiB | `overlays/highs.nix` — `CUPDLP_GPU=OFF` |
| `aws-sdk-cpp` (+ `-dev`) | 2.3 MiB | 10.7 MiB | pulled in by OpenMVS |
| `reusex-gui-frontend`, `zed-open-capture`, patches, sources | 8.5 MiB | ~11.5 MiB | vendored inputs / our own outputs |

`cachix push` already knows to skip anything the cache's configured upstream
(`cache.nixos.org`) can serve, so handing it the whole closure is safe — it
uploads the delta, not 9.62 GiB. Note `libtorch` and OpenCV are the paths
where compression pays off most; the npm dep tarball is already compressed and
is the single biggest line item.

The `ReUseX-tests` output is not part of that seeding: since reusex-37v it
is a pass stamp of a few KiB, pushed per commit by `scripts/ci-local.sh`
(below).

## Seeding / topping up from a dev machine

```bash
# 1. Build the exact thing CI builds
nix build .#checks.x86_64-linux.tests

# 2. Push the full build closure; upstream-cached paths are skipped
nix-store -q --requisites --include-outputs \
  $(nix path-info --derivation .#checks.x86_64-linux.tests) \
  | grep -v '\.drv$' \
  | cachix push reusex
```

You need a personal Cachix auth token (`cachix authtoken <token>`, stored in
`~/.config/cachix/cachix.dhall`). `cachix` is not in the dev shell; use
`nix run nixpkgs#cachix -- …`.

To see what a push *would* cost before doing it:

```bash
nix path-info --json $(…path list…) \
  | jq '[.[].narSize] | add / 1024 / 1024 / 1024'
```

## Staying inside 5 GB

Two design decisions keep the cache from growing without bound:

1. **CI writes only from `main`, and per-PR results are stamps.** A commit
   that touches any build-relevant file produces a distinct `ReUseX-tests`
   output path. That output used to be the whole installed package (static
   libraries, `rux`, a runtime closure); it is now a pass stamp (the ctest
   log and a `passed` file, no references), so the one path
   `scripts/ci-local.sh` pushes per commit costs a few KiB. `ci.yml` still
   sets `skipPush` for everything that is not a push to `main`: on `main`
   the cachix action pushes whatever the job built (dependency paths after an
   overlay bump, the `.#cpu` package downstream repos fetch).

2. **Only the CPU variant is cached.** The CUDA closure (TensorRT, cuOpt,
   CUDA-enabled OpenCV, the `.cu` kernels) is many times larger than the whole
   free tier and is useless to GitHub-hosted runners, which have no GPU. CUDA
   coverage stays a local / self-hosted-runner concern (#202's optional task).

If the cache does fill up, the recovery is to delete old `ReUseX-*` outputs
from the Cachix dashboard — the dependency paths are the ones that must
survive.

## Runner disk

Not a problem, contrary to the usual assumption. Measured on a green run
carrying no cleanup step at all: `/` is **145 GB and still had 80 GB free at
the end of the job**, with `/nix/store` at **7.0 GB**. `ci.yml` therefore does
*not* carry the widely-copied "free up runner disk space" step — it cost 38 s
to reclaim space nothing needed. The `Report disk usage` step is the early
warning if a future dependency bump changes this.

## Where the time actually goes (full-build fallback)

This is what a CI run costs when no pass stamp was pushed for its
derivation. With one, the job substitutes a few KiB and skips all of it
("Build locally, CI confirms" above).

Measured on a green PR run (25 min 39 s wall, all of it on `ubuntu-latest`,
4 vCPU):

| Phase | Time |
|---|---:|
| Job setup, checkout, install-nix, cachix-action | 50 s |
| **Pull the whole non-substitutable closure from Cachix** | **~38 s** |
| Compile ReUseX (CPU variant, `-j4`) | ~20 min |
| `ctest --parallel` (577 tests) | 3 min 48 s |
| Post steps | 2 s |

The cache does its job completely — 38 seconds for everything
`cache.nixos.org` could not serve. What it *cannot* fix is the ~19-minute
compile for a PR that touches build-relevant files, because every change in
`libs/`, `apps/`, `tests/`, etc. produces a new source hash and requires
recompiling ReUseX from scratch. PRs that touch only excluded paths (docs,
workflows, markdown) are pure cache hits — they skip the compile entirely.
That distinction is why run times split: doc-only PRs land in ~90 s, while
source-touching PRs still take ~25 min.

## Cold vs warm

- **Warm** (cache hit on every dependency): ~25 min, dominated by compiling
  ReUseX itself on a 4-vCPU runner.
- **Cold** (an overlay bump invalidated, say, OpenCV): the runner rebuilds
  that dependency from source. `timeout-minutes: 120` exists so such a run can
  still finish and repopulate the cache on `main` rather than being killed
  halfway, leaving the cache stale for the next PR.

### Filtered `src`: doc-only changes are pure cache hits

`default.nix` uses `lib.fileset.toSource` to include only the files the C++
build actually needs: `CMakeLists.txt`, `cmake/`, `libs/`, `apps/rux/`,
`bindings/`, `tests/`, and specific docs files the test suite reads at
runtime (`docs/CONTRACTS.md`, and since reusex-37v the two
`scripts/*bench_solid_compare*` files a ctest runs). `apps/ruxd/` and
`docs/gui/` moved to the standalone `ruxd` repo (repo split, P4, 2026-10-09)
and no longer appear here.

Everything else — `docs/`, `.github/`, `*.md`, `apps/blender/`, `tools/`,
`.claude/`, `python/` (except `engine-build.json`), the rest of `scripts/` —
is excluded from the source. (`overlays/`, `pkgs/` and `flake.lock` are not
in the source either, but they change the derivation through the
dependencies they evaluate to.) A PR that only touches excluded paths
produces the same `ReUseX-tests` derivation hash, so if that derivation's
stamp is already in the cache, CI substitutes it instead of recompiling.

**Verification performed in #357:**

| Change | Derivation hash | Expected |
|---|---|---|
| Baseline (`src = ./.`) | `wm3ig0mn2sc1m8k3511fn5j8dabmgfmh` | — |
| After filter, no file change | `vwfcqjy23w8q1d9mglqg3fh6093h57a6` | changed (filter itself is new) |
| Append blank line to `docs/DIRECTION.md` | `vwfcqjy23w8q1d9mglqg3fh6093h57a6` | **unchanged** ✓ |
| Append comment to `libs/reusex/src/core/logging.cpp` | `qvngp54bppgmhihwrzqvidw24kcsipi2` | **changed** ✓ |
