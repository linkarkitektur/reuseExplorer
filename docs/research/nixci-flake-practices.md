<!--
SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
SPDX-License-Identifier: GPL-3.0-or-later
-->

# NixCI flake practices, and what ReUseX took from them

Research note for bead reusex-pr6, written alongside reusex-uk8 (build and
test as separate derivations, CI substitutes what the developer pushed).
[NixCI](https://nix-ci.com) is a hosted CI for Nix flakes by Tom Sydney
Kerckhove (CS SYD) ([about](https://nix-ci.com/about)). Its premise is "if it
builds with `nix flake check` locally, it builds on NixCI"
([comparison](https://nix-ci.com/comparison)). It is the hosted form of an
older pattern by the same author,
[the ci.nix pattern](https://cs-syd.eu/posts/2021-04-11-the-ci-nix-pattern)
(2021): one attribute set lists everything to build, and the identical
command builds it locally and in CI, so the part of the setup that only CI
knows about stays as small as possible. That post also recommends filling the
binary cache locally before pushing, so that CI usually finds nothing left to
build. That is the reusex-uk8 goal.

## What NixCI expects of a flake

From [Your first flake](https://nix-ci.com/documentation/your-first-flake),
[How it works](https://nix-ci.com/documentation/how-it-works) and
[What gets built](https://nix-ci.com/documentation/what-gets-built):

- `nix flake show` and `nix flake check` succeed, and `x86_64-linux` is one
  of the systems.
- **Every `packages.<system>.*` (and `devShells.<system>.*`) is also a
  `checks.<system>.*` entry**, so `nix flake check` alone builds everything.
  The flake, not the CI config, defines what "everything" means.
- A smaller `nix flake check` that always passes is preferred over a larger
  one that sometimes fails for unrelated reasons (sandbox limits, missing
  hardware).
- NixCI walks `devShell(s)`, `formatter`, `packages` and `checks`, one job
  per attribute. `nixosConfigurations` are built only if they are exposed as
  checks.
- A plain-data `nix-ci.nix` at the repo root, read without a clone
  ([configuration](https://nix-ci.com/documentation/configuration)), scopes
  builds (`onlyBuild`, `doNotBuild`, systems, timeout) and configures caches.
  Its `shallow-clone` (default on) builds exactly the pushed commit.
- NixCI infers dependencies between outputs, so a check that uses a package
  waits for it and can substitute it instead of rebuilding
  ([dependency discovery](https://nix-ci.com/documentation/automatic-dependency-discovery)).
- Pure builds (packages, checks) are separate from impure tests (declared in
  `nix-ci.nix`, run with `nix run`, which get secrets and network)
  ([impure tests](https://nix-ci.com/documentation/test)). NixCI splits them
  for purity, not to avoid compiling twice, but the shape is the same: the
  thing that compiles is cached separately from the thing that exercises it.
- Caching: NixCI's own cache plus a persistent store per worker. Cachix can
  be added on top ([cachix](https://nix-ci.com/documentation/cachix),
  [NixCI cache](https://nix-ci.com/documentation/nix-ci-cache)). The docs give
  no guidance on push policy or a size budget for the Cachix add-on.

## What ReUseX does, after reusex-uk8

| NixCI / CS SYD practice | ReUseX |
|---|---|
| One flake attribute set defines what CI builds, and the same command runs locally and in CI | **Adopted.** `packages.ci` links the pass stamp (`checks.tests`) and both outputs of the CPU package (`checks.cpu`). The developer runs `nix build .#ci --print-out-paths \| cachix push reusex`, and CI runs `nix build .#ci`. There is no CI script. `scripts/ci-local.sh` was removed. |
| Every package is also a check | **Adopted for the variant CI can build.** `checks.cpu = packages.cpu`. The CUDA and ROCm variants stay out of `nix flake check` because the runners have no GPU. This is a deliberate exception. |
| The build is one cached output and the test run is a thin derivation on top | **Adopted.** `packages.cpu` compiles once (`out` plus a `testsuite` output). `checks.tests` runs only ctest and the install check against them (`default.nix`). |
| Build the exact commit, not a merge commit | **Already done in reusex-37v.** `ci.yml` checks out `pull_request.head.sha`. |
| Fill the cache locally before pushing | **Done.** See the local command above. |
| A clear "substituted or built" signal | **Done.** CI first tries `nix build .#ci --max-jobs 0` (substitute only), logs the `--dry-run` plan, and emits a `::notice` naming the case. |
| Filter the source so that unrelated edits keep the hash | **Done, and finer than before.** `lib.fileset` with separate build and test sources (`docs/guides/ci-cache.md`). |
| Push policy and size budget for the shared cache | ReUseX documents more than NixCI here: only `main` pushes from CI, per-branch outputs come from the developer's push, and the 5 GB free tier is tracked in `docs/guides/ci-cache.md`. |
| A plain-data per-repo config such as `nix-ci.nix` | Not adopted. CI is one job, and the flake already says what to build. |
| Persistent warm store per worker | No equivalent: GitHub-hosted runners are ephemeral. This would only matter on a self-hosted runner. |

## Caveat: hash identity is not bit identity

[CS SYD, "Nix does not guarantee reproducibility"](https://cs-syd.eu/posts/2025-03-14-nix-does-not-guarantee-reproducibility)
(2025) separates three guarantees Nix does not fully give:

- a build that once succeeded always succeeds (an input can disappear
  upstream);
- a build that once failed always fails (failures from OOM, load or
  timing);
- a successful build is bit-identical every time (embedded randomness,
  timestamps, nondeterministic parallel compilers).

The ReUseX scheme relies only on local and CI *evaluating* the same
derivation, because substitution keys on the derivation hash
(`docs/guides/ci-cache.md` shows that measurement). It does not rely on two
builds producing the same bytes. Bit identity would matter only for
content-addressed derivations, which this flake does not use. The second
point does apply to the test run: a timing-sensitive test that passed once
locally is substituted as passed in CI.

## Sources

- https://nix-ci.com/about, https://nix-ci.com/comparison
- https://nix-ci.com/documentation/your-first-flake
- https://nix-ci.com/documentation/how-it-works
- https://nix-ci.com/documentation/what-gets-built
- https://nix-ci.com/documentation/configuration
- https://nix-ci.com/documentation/automatic-dependency-discovery
- https://nix-ci.com/documentation/test
- https://nix-ci.com/documentation/cachix, https://nix-ci.com/documentation/nix-ci-cache
- https://cs-syd.eu/posts/2021-04-11-the-ci-nix-pattern
- https://cs-syd.eu/posts/2025-03-14-nix-does-not-guarantee-reproducibility
- Not read in full (only the titles could be retrieved):
  https://cs-syd.eu/posts/2026-03-09_ci-should-fail-on-your-machine-first,
  https://cs-syd.eu/posts/2026-02-06-the-purpose-of-continuous-integration-is-to-fail
