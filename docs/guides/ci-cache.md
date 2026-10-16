# CI binary cache (`reusex.cachix.org`) and local-first CI

**In short (reusex-37v, reusex-uk8):** before pushing a branch, build what
CI builds and push it to Cachix with one plain command:

```bash
nix build .#ci --print-out-paths | cachix push reusex
```

CI then runs the same `nix build .#ci`, substitutes everything and compiles
nothing. The sections "Build locally, CI confirms" and "Local ccache" below
describe that flow; the rest of this page is about the cache itself.

`.github/workflows/ci.yml` realises `.#ci` — the CPU variant of ReUseX with
`BUILD_TESTS=ON`, its ctest run and the package install check — on stock
GitHub-hosted runners. That is only possible because the parts of the
dependency closure that `cache.nixos.org` cannot serve come pre-built from
our own Cachix cache:

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

## Build locally, CI confirms

A full CI build + ctest took 25-45 min per PR on a 4-vCPU runner. It does not
have to: every output CI needs is a pure derivation, so once anyone has
realised it and pushed it to `reusex.cachix.org`, every other machine
evaluating the same derivation substitutes it instead of building it.

### The derivations (`flake.nix`, `default.nix`)

| Output | What it is | Compiles? |
|---|---|---|
| `checks.cpu` = `packages.cpu`, output `out` | the installed CPU package: `bin/rux`, headers, static libs, `lib/cmake/ReUseX` — what ruxd builds against | yes, once |
| `checks.cpu` = `packages.cpu`, output `testsuite` | the test executables plus every `CTestTestfile.cmake` and `catch_discover_tests` list, with the build's absolute paths replaced by placeholders | (same build) |
| `checks.tests` = `packages.cpu.check` | runs ctest over `testsuite` and `tests/package/consumer` against `out`; output is a pass stamp (`ctest.log`, `passed`) with **no references** (`allowedReferences = []`) | no (only the few-line consumer) |
| `packages.ci` | a `runCommand` linking the three paths above | no |

Before reusex-uk8, `checks.tests` was `packages.cpu.overrideAttrs` with
`doCheck`/`doInstallCheck`: one derivation that compiled, tested and kept only
the stamp. So the compiled package was never a cached output of that work,
the main-branch "build the CPU package for downstream consumers" step
compiled it again from scratch, and a fixture edit recompiled the library.

`checks.tests` runs binaries compiled in another sandbox, so the two
compiled-in paths they need are overridable at run time: the test source
tree (`reusex::test_support::source_dir()`, env `REUSEX_TEST_SOURCE_DIR`) and
the `rux` the integration tests spawn (`rux_executable()`, env
`REUSEX_TEST_RUX_EXECUTABLE`, pointed at the installed `out/bin/rux`). The
ctest files get their placeholders substituted in a writable copy of the
build tree under `$TMPDIR`.

### The one command, locally and in CI

```bash
# From the dev shell (it carries cachix), on a clean, committed tree:
nix build .#ci --print-out-paths | cachix push reusex
```

- **No script.** The maintainer's rule (2026-10-16): "it should be a flake
  output and we just run nix build … no custom scripts — that just risks
  things diverging." What CI realises is defined only by `packages.ci` in
  `flake.nix`; `scripts/ci-local.sh` (reusex-37v) is gone.
- **Clean tree.** A flake built from a dirty tree hashes the uncommitted
  changes (nix prints `warning: Git tree … is dirty`), so CI can never
  evaluate that derivation. Untracked files are fine; flakes do not see them.
- **What gets pushed.** `cachix push` takes the `ci` path's closure and
  uploads only what neither `reusex.cachix.org` nor its upstream
  `cache.nixos.org` already has. The dependencies are already there (CI
  substitutes them on every run), so in practice that is our own outputs:
  `cachix` reported `Pushing 4 paths (536 are already present)` for
  `.#ci` on `b914f307`: `ReUseX-0.0.5` (`out`, 81.3 MiB NAR / 23.8 MiB
  compressed), `ReUseX-0.0.5-testsuite` (74.0 / 26.8 MiB), the
  `ReUseX-tests` stamp (435 KiB / 74 KiB) and `reusex-ci` (848 B), about
  **51 MiB per pushed commit**, ~1 % of the tier.
- **Resources.** Nix's own `cores` / `max-jobs` from `nix.conf` apply. On
  the dev machine (RAM faults at high load) add `--cores 8 --max-jobs 1`.
- **Token.** `cachix authtoken <token>` once (stored in
  `~/.config/cachix/cachix.dhall`) or `CACHIX_AUTH_TOKEN` in the
  environment; a personal token with write access to `reusex`.
- **Checking first.** `nix build .#ci --dry-run` prints what nix would fetch
  and what it would build; if it would build nothing, CI will not either.

`ci.yml` (one step) checks out the **PR head commit**
(`pull_request.head.sha`), not GitHub's synthetic `refs/pull/N/merge`,
because the head is what was built locally. It then:

1. prints the commit and the `ci`, `cpu` and `tests` derivations, and nix's
   `--dry-run` plan, for comparison with a local
   `nix path-info --derivation .#ci`;
2. runs `nix build .#ci --max-jobs 0` — substitute only, never build. On a
   hit it logs the `::notice` **`.#ci substituted`** and is done;
3. on a miss runs `nix build .#ci`, which builds only what is missing, and
   says which case it is: **`.#ci built (tests only)`** when the plan does not
   contain the `ReUseX-<version>.drv` (the package was cached, so only ctest
   and the install check run, ~6 min), **`.#ci built (full compile)`** when
   it does.

On `main` a second step evaluates `github:ReUse-X/ReUseX/<sha>#cpu` (the
ref downstream repos fetch), compares its derivation with `.#cpu`'s and,
equal, realises it substitute-only (`::notice downstream .#cpu = checkout
.#cpu`); if they ever differ it warns and builds it, and the cachix action
pushes it.

**Warning issues** (`scripts/file_build_warnings.py`) are synced only when
the job compiled ReUseX on `main`: a substituted package has no compiler log,
and an empty log would read as "every warning fixed".

To make this automatic before every push, an alias is enough (deliberately
not a pre-push hook: git-hooks.nix and bd own the hooks directory, and
pre-commit hides a long hook's output until it ends):

```bash
git config --local alias.cipush \
  '!nix build .#ci --print-out-paths | cachix push reusex && git push'
git cipush -u origin my-branch
```

### Why the local and CI derivations are identical

Nothing commit-specific enters a derivation:

- Sources are `lib.fileset.toSource` sets (`default.nix`), whose store paths
  hash the files' contents only — not the commit, its message or the rest of
  the tree:
  - the **build** source (`buildFileset`): `CMakeLists.txt`, `cmake/`,
    `LICENSE.md`, `libs/`, `apps/rux/`, `bindings/`, `tests/` **minus
    `tests/fixtures`**, `python/reusex_sam3/engine-build.json`;
  - the **test** source (`testFileset`, used only by `checks.tests`): the
    build source plus `tests/fixtures`, `docs/CONTRACTS.md`,
    `docs/design/pose-edits.{md,schema.json}` and the four
    `scripts/*bench_*compare*.py` files ctest runs.

  The nix files (`flake.nix`, `default.nix`, `overlays/`, `pkgs/`,
  `flake.lock`) matter only through what they evaluate to.
- **LFS.** `tests/fixtures/scans/office_corridor.rux` is the one LFS file
  either set holds. CI checks out with `lfs: true` and a local `git+file`
  flake reads the smudged work tree, so the test source matches. A `github:`
  flake ref, though, is a GitHub tarball with the LFS **pointer** — which is
  why the fixtures are out of the build source: with them in, `github:…#cpu`
  and `.#cpu` were different derivations and every `main` push compiled the
  downstream package that nobody had pushed. Measured on `9734c6a4` (before)
  with a `git archive` tree whose `.rux` was swapped for its pointer:
  `.#cpu` `4hz4vc6k…` vs `63rfnbjy…`; on this branch both give `2c24b4qn…` (commit `b914f307`).
- The CPU variant is selected by attribute name, so the dev machine's CUDA
  default never leaks in.
- The ccache wrapper (`overlays/ccache.nix`) is part of the derivation on
  both sides; whether `/var/cache/ccache` exists is decided at build time,
  inside the builder, so it does not affect the hash.

Measured on this branch (commit `b914f307`), derivation hashes of
`.#cpu` / `checks.tests` / `.#ci`:

| Change on top of the commit | `cpu` | `tests` | `ci` |
|---|---|---|---|
| none (clean tree) | `2c24b4qn…` | `0apawa5j…` | `rm65n13f…` |
| `docs/DIRECTION.md`, `CLAUDE.md`, `.github/workflows/ci.yml` edited | unchanged | unchanged | unchanged |
| `docs/CONTRACTS.md` edited | unchanged | changed | changed |
| `tests/fixtures/scans/README.md` edited | unchanged | changed | changed |
| a test `.cpp` edited | changed | changed | changed |
| `libs/reusex/src/core/logging.cpp` edited | changed | changed | changed |

So a contract or fixture edit reruns only the tests (`.#cpu` stays
substitutable). A test *source* edit does rebuild `.#cpu`: the test binaries
are compiled in the same CMake build as the package — compiling once is the
point of the split — but with sandbox ccache only the edited TU misses.

CI-vs-local: the CI job prints the derivations it evaluated; they must equal
`nix path-info --derivation .#ci` (etc.) run locally on the same commit
(PR #62, run 38946617047, commit `b914f307`: CI printed `rm65n13f…`,
`2c24b4qn…`, `0apawa5j…`, the local values above, fetched 537 paths / 1.3 GiB
and built nothing: the `nix build .#ci` step took 45 s, the whole job
1 min 1 s). The `github:ReUse-X/ReUseX/b914f307…#cpu` ref evaluated locally
also gives `2c24b4qn…`.

**Hash identity is not output identity.** Substitution keys on the
derivation hash, so the scheme needs only that local and CI *evaluate* the
same derivation (above). It does not need — or prove — that two builds of it
are bit-identical; Nix does not guarantee that
([CS SYD, "Nix does not guarantee reproducibility"](https://cs-syd.eu/posts/2025-03-14-nix-does-not-guarantee-reproducibility)).
It would matter only for content-addressed derivations. It is not a
given here: `nix build .#cpu --rebuild` of `b914f307` without ccache
produced an `out` that differs from the ccache-built one in exactly one
file, the pybind11 module `reusex/_reusex.cpython-313-x86_64-linux-gnu.so`
(3 541 336 vs 3 529 016 bytes); every static library, header, CMake file and
`bin/rux` was identical. Harmless for substitution, tracked as reusex-knw.

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
built this derivation and it passed"; a substituted package, "… built this
derivation". Cachix signs what it serves, so neither can be forged by a
third party, but a token holder could push anything under those paths. That
is the same trust CI already placed in the dependency paths in the cache.
The build still runs on any commit nobody pushed `.#ci` for, and on every
`main` push whose merge commit differs (in build- or test-relevant files)
from the PR head that was pushed.

### Alternatives considered

| Option | CI time, warm / cold | Guarantees | Cost |
|---|---|---|---|
| **Cachix-substituted `.#ci`** (chosen) | ~2-3 min when the developer pushed it / tests only or full build otherwise | hermetic (same derivation as CI), bound to exact inputs | one `nix build`, ccache-warm locally |
| Dev-shell CI + ccache via `actions/cache` | ~10 min (restore + link + ctest ~4 min) / ~25 min | not hermetic (dev-shell build, impure env); needs a CPU-only CI devshell, ccache dir in the 10 GB Actions cache | moderate; CI still compiles and runs ctest every time |
| "Attestation": a local script posts a commit status, CI only lints | seconds / — | only the developer's word; nothing binds the status to what was built | low; but strictly weaker than the stamp, which costs the same |

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

### Inside the nix sandbox (`nix build .#ci`)

`packages.cpu` (the only derivation of `.#ci` that compiles ReUseX) is built with `ccacheStdenv` from
`overlays/ccache.nix`, which points ccache at `/var/cache/ccache`. On the dev
machine NixOS makes that directory (`programs.ccache`, `root:nixbld 0770`)
and exposes it to the sandbox (`sandbox-paths` contains `/var/cache/ccache`;
check with `nix config show | grep sandbox-paths`). Where it is missing — a
GitHub runner — the wrapper falls back to an empty per-build cache, which is
why the derivation and its hash are the same on both sides. The sandbox
always builds in `/build/source`, so no base-dir setting is needed there.

Measured before reusex-uk8 (build and tests still one derivation) with
`nix build .#checks.x86_64-linux.tests --cores 8 --max-jobs 2` on the dev machine (other builds running alongside), each after a
one-line change (a `static_assert` appended to
`libs/reusex/src/segmentation/downsample.cpp`):

| Build | configure | build | ctest | install check | wall |
|---|---:|---:|---:|---:|---:|
| first build of this branch (cache warm from older commits: 405 hits, 137 misses) | 51 s | 10 min 3 s | 3 min 36 s | 1 min 2 s | 15 min 49 s |
| one-line change, **sandbox ccache** (all build TUs hit but the changed one) | 42 s | **54 s** | 3 min 19 s | 1 min 2 s | **6 min 11 s** |
| one-line change, **no ccache** (`--option sandbox-paths` without `/var/cache/ccache`) | 40 s | 19 min 19 s | 3 min 33 s | 1 min 2 s | 24 min 50 s |

So a local `nix build .#ci` after a small change costs about six to eight
minutes, nearly all of it ctest and the package install check, which no
cache can skip because they are the point. Measured after the split
(reusex-uk8, `--cores 8 --max-jobs 1`, sandbox ccache warm):

| Change | `.#cpu` configure | build | `checks.tests` (ctest 2064 tests + install check) | wall |
|---|---:|---:|---:|---:|
| two test TUs + `tests/support/source_dir.cpp` changed (first build of the branch) | 47 s | 1 min 30 s | 5 min 51 s | 8 min 34 s |
| two test TUs reformatted (clang-format), then `cachix push` | 42 s | 1 min 8 s | 5 min 9 s | 7 min 41 s (incl. dev shell entry and push) |
| `.#cpu` cold: same commit, `--rebuild`, **no ccache** (`--option sandbox-paths` without `/var/cache/ccache`) | 43 s | 21 min 25 s | — | 22 min 18 s |

A change that touches only test run-time data (a fixture, `CONTRACTS.md`)
skips the first two columns: `.#cpu` is unchanged and substituted. The stats come from a throwaway
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

Our own per-commit outputs are not part of that seeding; they are pushed
with `.#ci` per branch (next sections).

## Seeding / topping up from a dev machine

```bash
# 1. Build the exact thing CI builds
nix build .#ci

# 2. Push the full build closure; upstream-cached paths are skipped
nix-store -q --requisites --include-outputs \
  $(nix path-info --derivation .#ci) \
  | grep -v '\.drv$' \
  | cachix push reusex
```

You need a personal Cachix auth token (`cachix authtoken <token>`, stored in
`~/.config/cachix/cachix.dhall`). `cachix` is in the dev shell; elsewhere use
`nix run nixpkgs#cachix -- …`.

To see what a push *would* cost before doing it:

```bash
nix path-info --json $(…path list…) \
  | jq '[.[].narSize] | add / 1024 / 1024 / 1024'
```

## Staying inside 5 GB

Two design decisions keep the cache from growing without bound:

1. **CI writes only from `main`; per-branch outputs come from the
   developer's push, and they are small.** A pushed `.#ci` adds our own
   outputs only: about **51 MiB compressed per pushed commit** (`out`
   23.8 MiB, `testsuite` 26.8 MiB, stamp 74 KiB), so ~100 pushed commits
   fit in the tier next to the ~0.5 GiB of dependencies. That is the price
   of CI compiling nothing on `main` too: before reusex-uk8 a PR pushed only
   the 74 KiB stamp, and every `main` push compiled `.#cpu` in CI and pushed
   its 24 MiB anyway. Delete old `ReUseX-*` outputs first when the cache
   fills (below). The stamp has no references; `out` and
   `testsuite` reference only dependency paths that are already in the cache
   (static ReUseX libraries, so no shared build-tree libraries). A commit
   that changes only tests' run-time data adds just a new stamp, since
   `.#cpu` is unchanged. `ci.yml` sets `skipPush` for everything that is not
   a push to `main`: on `main` the cachix action pushes whatever the job
   built (dependency paths after an overlay bump, or `.#ci` when nobody
   pushed it).

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

This is what a CI run cost when nothing was pushed for its derivation
(measured before reusex-uk8). With `.#ci` pushed, the job substitutes and
skips all of it ("Build locally, CI confirms" above); with only `.#cpu`
cached, it skips the compile and runs the tests.

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
build needs (`buildFileset`) and, for the test run, the files tests read at
run time on top (`testFileset`); "Why the local and CI derivations are
identical" above lists both. `apps/ruxd/` and `docs/gui/` moved to the
standalone `ruxd` repo (repo split, P4, 2026-10-09) and no longer appear here.

Everything else — `docs/` (but the three test-read files), `.github/`,
`*.md`, `apps/blender/`, `tools/`, `.claude/`, `python/` (except
`engine-build.json`), the rest of `scripts/` — is excluded from both.
(`overlays/`, `pkgs/` and `flake.lock` are not in the sources either, but
they change the derivations through the dependencies they evaluate to.) A PR
that only touches excluded paths produces the same `.#ci` derivation, so if
it is already in the cache, CI substitutes it instead of recompiling.

**Verification performed in #357:**

| Change | Derivation hash | Expected |
|---|---|---|
| Baseline (`src = ./.`) | `wm3ig0mn2sc1m8k3511fn5j8dabmgfmh` | — |
| After filter, no file change | `vwfcqjy23w8q1d9mglqg3fh6093h57a6` | changed (filter itself is new) |
| Append blank line to `docs/DIRECTION.md` | `vwfcqjy23w8q1d9mglqg3fh6093h57a6` | **unchanged** ✓ |
| Append comment to `libs/reusex/src/core/logging.cpp` | `qvngp54bppgmhihwrzqvidw24kcsipi2` | **changed** ✓ |
