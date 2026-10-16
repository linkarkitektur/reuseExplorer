#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Run CI's build-and-test locally and hand CI the results (reusex-37v,
# reusex-uk8).
#
# Build and test are two derivations (flake.nix, default.nix):
#   .#cpu                        compiles ReUseX once (CPU variant,
#                                BUILD_TESTS=ON). Outputs: `out`, the
#                                installed package downstream repos (ruxd)
#                                build against, and `testsuite`, the test
#                                binaries + ctest files.
#   .#checks.x86_64-linux.tests  runs ctest over `testsuite` and the
#                                package install check against `out`,
#                                compiling nothing; its output is a pass stamp.
# What CI realises is listed once, in scripts/ci-outputs.txt, read by this
# script and by .github/workflows/ci.yml alike. This script builds every
# entry (all outputs) for the commit that is checked out, inside the nix
# sandbox, and pushes every output path to reusex.cachix.org: the stamp,
# `out` and `testsuite`. CI on the same commit then substitutes all it needs
# and compiles nothing.
#
# The local build reuses /var/cache/ccache through the sandbox
# (overlays/ccache.nix), so a rebuild after a small change is mostly linking
# and running tests.
#
# Usage:
#   scripts/ci-local.sh              # build HEAD, push package, testsuite, stamp
#   scripts/ci-local.sh --no-push    # build only
#   scripts/ci-local.sh --check      # only report whether CI would hit
#
# Environment:
#   CI_LOCAL_CORES      cores per build (default 8; this machine had RAM
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
cores="${CI_LOCAL_CORES:-8}"
max_jobs="${CI_LOCAL_MAX_JOBS:-1}"
push=1
check_only=0

for arg in "$@"; do
  case "$arg" in
  --no-push) push=0 ;;
  --check) check_only=1 ;;
  -h | --help)
    sed -n '5,44p' "$0" | sed 's/^# \{0,1\}//'
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
echo "commit: $sha"

# Every installable in scripts/ci-outputs.txt (scope ignored: CI on main
# needs them all), and every output path of each, without building.
installables=()
outputs=()
while read -r scope installable _; do
  [[ -z "$scope" || "$scope" == \#* ]] && continue
  installables+=("$installable")
  drv="$(nix path-info --derivation "$installable")"
  echo "$installable ($scope): $drv"
  while read -r p; do
    echo "  -> $p"
    outputs+=("$p")
  done < <(nix-store --query --outputs "$drv")
done <scripts/ci-outputs.txt

# narinfo-cache-negative-ttl=0: a "not there" answer cached by an earlier
# run would otherwise hide a push made since.
in_cache() {
  nix path-info --store "https://$cache.cachix.org" \
    --option narinfo-cache-negative-ttl 0 "$1" >/dev/null 2>&1
}

missing=0
for p in "${outputs[@]}"; do
  if in_cache "$p"; then
    echo "ci-local: in $cache.cachix.org:     $p"
  else
    echo "ci-local: not in $cache.cachix.org: $p"
    missing=$((missing + 1))
  fi
done

if [[ $missing -eq 0 ]]; then
  echo "ci-local: all in $cache.cachix.org; CI on $sha will substitute them."
  exit 0
fi
if [[ $check_only -eq 1 ]]; then
  echo "ci-local: $missing path(s) missing; CI on $sha would build them."
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
# `^*`: every output, so `testsuite` is realised and pushed with `out`.
build_args=()
for i in "${installables[@]}"; do build_args+=("$i^*"); done
nix build "${build_args[@]}" --no-link --print-build-logs \
  --cores "$cores" --max-jobs "$max_jobs"
echo "ci-local: built and tested in $(($(date +%s) - start)) s"

nar_mib() {
  nix path-info --size "$1" | awk '{ printf "%.1f MiB", $2 / 1048576 }'
}
# Compressed size as stored in the cache, from the path's .narinfo.
cached_mib() {
  local hash
  hash="$(basename "$1" | cut -c1-32)"
  curl -fsS "https://$cache.cachix.org/$hash.narinfo" 2>/dev/null |
    awk '/^FileSize:/ { printf "%.1f MiB compressed", $2 / 1048576 }'
}
for p in "${outputs[@]}"; do
  echo "ci-local: built $(nar_mib "$p") NAR: $p"
  if [[ -f "$p/passed" ]]; then cat "$p/passed"; fi
done

if [[ $push -eq 0 ]]; then
  echo "ci-local: --no-push given; not pushing."
  exit 0
fi

# cachix pushes each path's closure minus what reusex.cachix.org or its
# upstream (cache.nixos.org) already has. The dependencies are already there
# (CI substitutes them on every run), so this uploads our own outputs only;
# check its "Pushing N paths (M are already present)" line. The stamp has no
# references at all (allowedReferences = [] in default.nix).
"${cachix_cmd[@]}" push "$cache" "${outputs[@]}"

fail=0
for p in "${outputs[@]}"; do
  if in_cache "$p"; then
    echo "ci-local: in $cache.cachix.org ($(cached_mib "$p")): $p"
  else
    echo "ci-local: push reported success but $p is not visible in" >&2
    echo "          $cache.cachix.org yet." >&2
    fail=1
  fi
done
[[ $fail -eq 0 ]] || exit 1
echo "ci-local: pushed. CI on $sha will substitute everything and build nothing."
