# SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later
#
# `nix develop .#report` — the PDF report template loop (Typst). Light on
# purpose: no inputsFrom the C++ package, so it enters in seconds.
# docs/guides/report-template.md explains the loop and the data contract.
{pkgs, ...}: let
  motd = ''
    echo ""
    echo "  ┌─────────────────────────────────────────────┐"
    echo "  │       ReUseX  •  Report template shell      │"
    echo "  └─────────────────────────────────────────────┘"
    echo "  report-watch [DATA_DIR]   recompile shots/report/report.pdf on every save"
    echo "  report-build [DATA_DIR]   compile once (non-zero exit on a Typst error)"
    echo "  report-shots [DATA_DIR]   compile, then shots/report/page-NN.png per page"
    echo "  menu                      show this message"
    echo ""
    echo "  DATA_DIR defaults to apps/rux/resources/report-sample."
    echo "  Real data: rux -p x.rux export pdf --dump-data DIR"
    echo ""
  '';
  # Shared by all three commands. Typst refuses a source file outside --root
  # and resolves symlinks before checking, so the template cannot be linked
  # into the data dir. Instead --root is / and report.typ reads the data dir
  # from `--input data-dir=...`.
  preamble = name: ''
    set -euo pipefail
    repo=$(git rev-parse --show-toplevel 2>/dev/null || pwd)
    template="''${REUSEX_REPORT_TEMPLATE:-$repo/apps/rux/resources/report.typ}"
    data="''${1:-$repo/apps/rux/resources/report-sample}"
    if [ ! -f "$data/data.json" ]; then
      echo "${name}: no data.json in $data" >&2
      echo "  (dump a project's data with: rux -p x.rux export pdf --dump-data DIR)" >&2
      exit 2
    fi
    data=$(realpath "$data")
    template=$(realpath "$template")
    out="$repo/shots/report"
    mkdir -p "$out"
    export TYPST_FONT_PATHS="$repo/apps/rux/qt/resources/fonts"
    export TYPST_IGNORE_SYSTEM_FONTS=1
  '';
  report-build = pkgs.writeShellScriptBin "report-build" ''
    ${preamble "report-build"}
    typst compile --ignore-system-fonts --root / --input "data-dir=$data" "$template" "$out/report.pdf"
    echo "report-build: $out/report.pdf (data: $data)"
  '';
  report-watch = pkgs.writeShellScriptBin "report-watch" ''
    ${preamble "report-watch"}
    echo "report-watch: $out/report.pdf (data: $data), Ctrl-C to stop"
    exec typst watch --ignore-system-fonts --root / --input "data-dir=$data" "$template" "$out/report.pdf"
  '';
  report-shots = pkgs.writeShellScriptBin "report-shots" ''
    ${preamble "report-shots"}
    typst compile --ignore-system-fonts --root / --input "data-dir=$data" "$template" "$out/report.pdf"
    rm -f "$out"/page-*.png
    pdftoppm -r 110 -png "$out/report.pdf" "$out/page"
    # pdftoppm pads page numbers to the page count's width (page-1.png for
    # fewer than 10 pages); normalise to page-NN.png so a listing sorts.
    for f in "$out"/page-*.png; do
      n=''${f##*/page-}
      n=''${n%.png}
      new=$(printf '%s/page-%02d.png' "$out" "$((10#$n))")
      [ "$f" = "$new" ] || mv "$f" "$new"
    done
    ls "$out"/page-*.png
  '';
in
  pkgs.mkShell {
    packages = [
      pkgs.typst
      pkgs.poppler-utils
      pkgs.imagemagick # scripts/report-sample-images.sh
      pkgs.git
      report-build
      report-watch
      report-shots
      (pkgs.writeShellScriptBin "menu" motd)
    ];

    shellHook = ''
      export TYPST_FONT_PATHS="$PWD/apps/rux/qt/resources/fonts"
      export TYPST_IGNORE_SYSTEM_FONTS=1
      ${motd}
    '';
  }
