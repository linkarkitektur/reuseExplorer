#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Screenshot pages of the native Qt client headless — the Qt half of the
# design-studio loop (references/qt-client.md).
#
# Builds `rux-qt-gallery` if it is stale, then renders each page x theme to a
# PNG with QT_QPA_PLATFORM=offscreen (no display needed; 3D panes render
# through VTK's EGL offscreen window). Styles are read live from the source
# tree (--dev), so a tokens.css or app.qss edit needs NO rebuild: just shoot
# again. FAILS (exit 3) if any token or bundled font family is missing — a
# missing token is magenta in the PNG and an error, never a silent fallback.
#
# Usage: qt_shot.sh [options]
#   --page NAME      page to shoot (repeatable; default: components)
#   --all            every page the gallery registers (--list-pages)
#   --theme T        dark | light | both (default: both)
#   --size WxH       logical window size (default: 1440x900)
#   --scale N        device pixel ratio (default: 2 — review shots)
#   --project FILE   project to read (a COPY is made per shot). Default: the
#                    office_corridor fixture, prepared once with a 2 cm cloud
#                    and planes so the 3D page has something to draw (builds
#                    rux first if needed; a failed preparation is never cached).
#   --out DIR        where PNGs go (default: shots/qt)
#   --gl             render the real QVTKOpenGLNativeWidget under xvfb-run
#   --embedded       use the stylesheet snapshot compiled into the binary
#                    instead of the live source files
#   --no-build       skip the gallery build (it is skipped anyway when no
#                    apps/rux/qt C++/CMake/qrc file is newer than the binary)
#   --build-dir DIR  CMake build dir (default: <repo>/build)
#
# Example:
#   qt_shot.sh --all --theme both --out shots/qt
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(git -C "$PWD" rev-parse --show-toplevel 2>/dev/null || git -C "$SCRIPT_DIR" rev-parse --show-toplevel)"

PAGES=()
ALL=0
THEMES="both"
SIZE="1440x900"
SCALE="2"
PROJECT=""
OUT="shots/qt"
GL=0
EMBEDDED=0
BUILD=1
NO_BUILD=0
BUILD_DIR="$ROOT/build"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --page) PAGES+=("$2"); shift 2 ;;
    --all) ALL=1; shift ;;
    --theme) THEMES="$2"; shift 2 ;;
    --size) SIZE="$2"; shift 2 ;;
    --scale) SCALE="$2"; shift 2 ;;
    --project) PROJECT="$2"; shift 2 ;;
    --out) OUT="$2"; shift 2 ;;
    --gl) GL=1; shift ;;
    --embedded) EMBEDDED=1; shift ;;
    --no-build) BUILD=0; NO_BUILD=1; shift ;;
    --build-dir) BUILD_DIR="$2"; shift 2 ;;
    -h|--help) sed -n '2,/^set -euo/p' "$0" | sed 's/^# \{0,1\}//; /^set -euo/d'; exit 0 ;;
    *) echo "qt_shot.sh: unknown option $1" >&2; exit 2 ;;
  esac
done

# Run a command inside the dev shell unless we are already in one.
in_dev() {
  if [[ -n "${IN_NIX_SHELL:-}" ]]; then "$@"; else nix develop "$ROOT" -c "$@"; fi
}

GALLERY="$BUILD_DIR/apps/rux/qt/rux-qt-gallery"
t0=$EPOCHREALTIME
# Stale = a C++/CMake/qrc file of the Qt client is newer than the binary.
# Style edits (app.qss, tokens.css) never need a build in --dev mode, and
# skipping the no-op build saves the dev-shell start-up (~20 s).
if [[ $BUILD -eq 1 && -x "$GALLERY" ]]; then
  pattern=( -name '*.cpp' -o -name '*.hpp' -o -name 'CMakeLists.txt' -o -name '*.qrc' -o -name '*.ttf' )
  [[ $EMBEDDED -eq 1 ]] && pattern+=( -o -name '*.qss' )
  if [[ -z "$(find "$ROOT/apps/rux/qt" \( "${pattern[@]}" \) -newer "$GALLERY" -print -quit)" ]] &&
     { [[ $EMBEDDED -eq 0 ]] || [[ ! "$ROOT/apps/rux/qt/theme/tokens.css" -nt "$GALLERY" ]]; }; then
    BUILD=0
  fi
fi
if [[ $BUILD -eq 1 ]]; then
  in_dev cmake --build "$BUILD_DIR" --target rux-qt-gallery >"$BUILD_DIR/qt_shot-build.log" 2>&1 || {
    tail -40 "$BUILD_DIR/qt_shot-build.log" >&2
    echo "qt_shot.sh: gallery build failed (log: $BUILD_DIR/qt_shot-build.log)" >&2
    exit 1
  }
fi
[[ -x "$GALLERY" ]] || { echo "qt_shot.sh: $GALLERY not built" >&2; exit 1; }
t_build=$EPOCHREALTIME

mkdir -p "$OUT"
WORK="$(mktemp -d "${TMPDIR:-/tmp}/qt_shot.XXXXXX")"
PREP_TMP=""
cleanup() {
  rm -rf "$WORK"
  [[ -n "$PREP_TMP" ]] && rm -f "$PREP_TMP" "$PREP_TMP-wal" "$PREP_TMP-shm"
  return 0
}
trap cleanup EXIT

# The default project: a prepared copy of the tracked fixture (which has no
# point cloud). Never the tracked file itself — ProjectDB migrates on open and
# leaves -wal/-shm. Only a SUCCESSFULLY prepared copy is cached; anything else
# fails loudly, so a cloudless fixture can never be cached by accident.
if [[ -z "$PROJECT" ]]; then
  CACHE="${XDG_CACHE_HOME:-$HOME/.cache}/reusex/qt-shot"
  FIX="$ROOT/tests/fixtures/scans/office_corridor.rux"
  PREP="$CACHE/office_corridor-$(sha1sum "$FIX" | cut -c1-12).rux"
  if [[ ! -f "$PREP" ]]; then
    mkdir -p "$CACHE"
    RUX="$BUILD_DIR/apps/rux/rux"
    if [[ ! -x "$RUX" ]]; then
      if [[ $NO_BUILD -eq 1 ]]; then
        echo "qt_shot.sh: FAIL the fixture needs a one-time preparation with rux, which is not built." >&2
        echo "  run: nix develop -c cmake --build $BUILD_DIR --target rux" >&2
        exit 1
      fi
      echo "qt_shot.sh: building rux once to prepare the fixture (minutes on a cold build) …" >&2
      in_dev cmake --build "$BUILD_DIR" --target rux >"$BUILD_DIR/qt_shot-rux-build.log" 2>&1 || {
        tail -30 "$BUILD_DIR/qt_shot-rux-build.log" >&2
        echo "qt_shot.sh: FAIL building rux (log: $BUILD_DIR/qt_shot-rux-build.log)" >&2
        echo "  run: nix develop -c cmake --build $BUILD_DIR --target rux" >&2
        exit 1
      }
    fi
    # A unique temp name: two first runs at once must not share one file.
    PREP_TMP="$(mktemp "$CACHE/prepare.XXXXXX")"
    LOG="$CACHE/prepare.log"
    : >"$LOG"
    echo "qt_shot.sh: preparing fixture once (2 cm cloud + planes) …" >&2
    cp "$FIX" "$PREP_TMP"
    for step in "create clouds -g 0.02 --sampling-factor 2" "create planes"; do
      # shellcheck disable=SC2086 # $step is a word list on purpose
      if ! in_dev "$RUX" -p "$PREP_TMP" $step >>"$LOG" 2>&1; then
        tail -30 "$LOG" >&2
        echo "qt_shot.sh: FAIL preparing the fixture: rux $step (log: $LOG)" >&2
        exit 1
      fi
    done
    rm -f "$PREP_TMP-wal" "$PREP_TMP-shm"
    mv -f "$PREP_TMP" "$PREP" # atomic: a reader sees all or nothing
    PREP_TMP=""
  fi
  PROJECT="$PREP"
fi

if [[ $ALL -eq 1 ]]; then
  mapfile -t PAGES < <(QT_QPA_PLATFORM=offscreen "$GALLERY" --list-pages | awk '{print $1}')
fi
[[ ${#PAGES[@]} -gt 0 ]] || PAGES=(components)
case "$THEMES" in
  both) THEME_LIST=(dark light) ;;
  dark|light) THEME_LIST=("$THEMES") ;;
  *) echo "qt_shot.sh: --theme must be dark, light or both" >&2; exit 2 ;;
esac

STYLE=(--dev)
[[ $EMBEDDED -eq 1 ]] && STYLE=()

XVFB=()
if [[ $GL -eq 1 ]]; then
  if command -v xvfb-run >/dev/null; then XVFB=(xvfb-run -a -s "-screen 0 3840x2400x24")
  else XVFB=(nix shell nixpkgs#xvfb-run -c xvfb-run -a -s "-screen 0 3840x2400x24"); fi
fi

status=0
for page in "${PAGES[@]}"; do
  for theme in "${THEME_LIST[@]}"; do
    copy="$WORK/$(basename "$PROJECT" | sed -E 's/-[0-9a-f]{12}\.rux$/.rux/')"
    rm -f "$copy" "$copy-wal" "$copy-shm"
    cp "$PROJECT" "$copy"
    suffix=""
    [[ $GL -eq 1 ]] && suffix="-gl"
    png="$OUT/${page}-${theme}${suffix}@${SCALE}x.png"
    rm -f "$png"
    log="$WORK/${page}-${theme}.log"
    args=(--page "$page" --theme "$theme" --size "$SIZE" --scale "$SCALE"
          --project "$copy" --screenshot "$png" "${STYLE[@]}")
    if [[ $GL -eq 1 ]]; then
      # xvfb-run exits 1 from its own teardown even on success: judge by the PNG.
      env -u WAYLAND_DISPLAY QT_QPA_PLATFORM=xcb "${XVFB[@]}" "$GALLERY" "${args[@]}" --gl >"$log" 2>&1 || true
    else
      env -u DISPLAY -u WAYLAND_DISPLAY QT_QPA_PLATFORM=offscreen "$GALLERY" "${args[@]}" >"$log" 2>&1 || true
    fi
    if grep -q "MISSING" "$log"; then
      grep "MISSING" "$log" | sort -u >&2
      echo "qt_shot.sh: FAIL $png — missing tokens or fonts (magenta / fallback faces in the image)" >&2
      status=3
    elif [[ ! -s "$png" ]]; then
      cat "$log" >&2
      echo "qt_shot.sh: FAIL no PNG for $page/$theme" >&2
      status=1
    else
      grep -E "ERROR" "$log" >&2 || true
      # A page's own measurement lines ("rediger-stress: 1234 nodes …"),
      # so they can be grepped from qt_shot.sh's stderr.
      grep -E '^[a-z-]+: ' "$log" >&2 || true
      echo "$png"
    fi
  done
done
t_end=$EPOCHREALTIME
LC_ALL=C awk -v a="${t0/,/.}" -v b="${t_build/,/.}" -v c="${t_end/,/.}" \
  'BEGIN { printf "qt_shot.sh: build %.2fs, shots %.2fs\n", b - a, c - b }' >&2
exit $status
