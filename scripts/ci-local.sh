#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Run CI's build-and-test locally and hand CI the result (reusex-37v).
#
# Builds `.#checks.x86_64-linux.tests` (the CPU variant, BUILD_TESTS=ON, ctest
# and the package install check, all inside the nix sandbox) for the commit
# that is checked out, then pushes its output to reusex.cachix.org. That
# output is a pass stamp of a few KiB (flake.nix), so the push costs nothing
# against the 5 GB tier. CI on the same commit (.github/workflows/ci.yml)
# substitutes the stamp and compiles nothing.
#
# The local build reuses /var/cache/ccache through the sandbox
# (overlays/ccache.nix), so a rebuild after a small change is mostly linking
# and running tests.
#
# Usage:
#   scripts/ci-local.sh              # build HEAD, push the stamp
#   scripts/ci-local.sh --no-push    # build only
#   scripts/ci-local.sh --check      # only report whether CI would hit
#
# Environment:
#   CI_LOCAL_CORES      cores per build (default 4; this machine had RAM
#                       faults and MCEs at higher load)
#   CI_LOCAL_MAX_JOBS   parallel derivations (default 1)
#   CACHIX_AUTH_TOKEN   write token for reusex.cachix.org; alternatively
#                       `cachix authtoken <token>` once, which stores it in
#                       ~/.config/cachix/cachix.dhall
#
# The commit must be clean: a flake built from a dirty tree hashes the
# uncommitted changes, so CI could never evaluate the same derivation.
# Untracked files are fine (flakes do not see them). Push the branch as is
# afterwards; CI tests the PR head commit, not a merge commit.

set -euo pipefail

cache=reusex
attr='.#checks.x86_64-linux.tests'
cores="${CI_LOCAL_CORES:-4}"
max_jobs="${CI_LOCAL_MAX_JOBS:-1}"
push=1
check_only=0

for arg in "$@"; do
  case "$arg" in
  --no-push) push=0 ;;
  --check) check_only=1 ;;
  -h | --help)
    sed -n '5,34p' "$0" | sed 's/^# \{0,1\}//'
    exit 0
    ;;
  *)
    echo "ci-local: unknown argument '$arg' (see --help)" >&2
    exit 2
    ;;
  esac
done

repo_root="$(git rev-parse --show-toplevel)"
cd "$repo_root"

if [[ -n "$(git status --porcelain --untracked-files=no)" ]]; then
  echo "ci-local: the working tree has uncommitted changes to tracked files." >&2
  echo "          Commit (or stash) them first: CI builds the commit, and a" >&2
  echo "          dirty tree gives a derivation CI will never evaluate." >&2
  git status --short --untracked-files=no >&2
  exit 1
fi

sha="$(git rev-parse HEAD)"
drv="$(nix path-info --derivation "$attr")"
out="$(nix eval --raw "$attr.outPath")"
echo "commit:     $sha"
echo "derivation: $drv"
echo "output:     $out"

# narinfo-cache-negative-ttl=0: a "not there" answer cached by an earlier
# run would otherwise hide a push made since.
in_cache() {
  nix path-info --store "https://$cache.cachix.org" \
    --option narinfo-cache-negative-ttl 0 "$out" >/dev/null 2>&1
}

if in_cache; then
  echo "ci-local: already in $cache.cachix.org; CI on $sha will substitute it."
  exit 0
fi
if [[ $check_only -eq 1 ]]; then
  echo "ci-local: not in $cache.cachix.org; CI on $sha would do the full build."
  exit 1
fi

cachix_cmd=()
if [[ $push -eq 1 ]]; then
  if command -v cachix >/dev/null 2>&1; then
    cachix_cmd=(cachix)
  else
    # Same nixpkgs as the flake, so no extra evaluation surprises.
    cachix_cmd=(nix run --inputs-from "$repo_root" nixpkgs#cachix --)
  fi
  # Fail before an hour-long build, not after it.
  if [[ -z "${CACHIX_AUTH_TOKEN:-}" &&
    ! -s "${XDG_CONFIG_HOME:-$HOME/.config}/cachix/cachix.dhall" ]]; then
    echo "ci-local: no Cachix write token. Set CACHIX_AUTH_TOKEN, or run" >&2
    echo "          '${cachix_cmd[*]} authtoken <token>' once (a personal token" >&2
    echo "          with write access to $cache, from app.cachix.org)." >&2
    echo "          Or pass --no-push to build without pushing." >&2
    exit 1
  fi
fi

echo "ci-local: building with --cores $cores --max-jobs $max_jobs"
start=$(date +%s)
nix build "$attr" --no-link --print-build-logs \
  --cores "$cores" --max-jobs "$max_jobs"
echo "ci-local: built and tested in $(($(date +%s) - start)) s"
cat "$out/passed"

if [[ $push -eq 0 ]]; then
  echo "ci-local: --no-push given; not pushing."
  exit 0
fi

# The stamp has no references, so this uploads exactly one tiny path.
"${cachix_cmd[@]}" push "$cache" "$out"
if in_cache; then
  echo "ci-local: pushed. CI on $sha will substitute $out and build nothing."
else
  echo "ci-local: push reported success but $out is not visible in" >&2
  echo "          $cache.cachix.org yet." >&2
  exit 1
fi
