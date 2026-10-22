#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Regenerates the synthetic placeholder photos of the PDF report sample
# (apps/rux/resources/report-sample/images/): labelled gradients, no
# third-party content. Needs ImageMagick 7 (`magick`), e.g. from
# `nix develop .#report` or `nix shell --inputs-from . nixpkgs#imagemagick`.
set -euo pipefail
root=$(git rev-parse --show-toplevel)
out="${1:-$root/apps/rux/resources/report-sample/images}"
font="$root/apps/rux/qt/resources/fonts/Archivo-SemiBold.ttf"
mkdir -p "$out"
# name  size  top-colour  bottom-colour  label
shot() {
  magick -size "$2" "gradient:$3-$4" -font "$font" -fill '#1d1d1b' \
    -pointsize 30 -gravity center -annotate +0+0 "$5" \
    -quality 72 -strip "$out/$1.jpg"
}
shot sample-01 640x480 '#b9893f' '#e7cf96' 'Mursten, gule\nRX-001'
shot sample-02 640x480 '#7a5a3a' '#c9a77c' 'Tagspær, fyr\nRX-002'
shot sample-03 480x640 '#5b4027' '#a8845c' 'Indvendig dør\nmassiv eg\nRX-003'
shot sample-04 640x400 '#c08a52' '#ead2b0' 'Gulvbrædder, bøg\nRX-004'
shot sample-05 640x480 '#6d6f73' '#c4c6c9' 'Radiator,\nstøbejern\nRX-005'
shot sample-06 480x640 '#e4e9ec' '#8fa3ad' 'Vindue,\nkoblede rammer\nRX-006'
shot sample-07 800x450 '#9a9a96' '#d8d8d2' 'Facadeelement,\nsandwich\nRX-007'
shot sample-08 640x480 '#f4f4f0' '#bfc4c0' 'Køkkenlåger\nRX-008'
# The Forside's cover image (data.json cover_path).
shot cover 1200x560 '#8c9aa3' '#dfe5e8' 'Forsidebillede\nSkolevej 12, Kolding'
echo "wrote 9 images to $out"
