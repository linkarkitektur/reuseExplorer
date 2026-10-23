// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

/// @file
/// The built-in SAM3 concept prompts: the one default class list of
/// `rux create annotate` (and of segment_panorama), shared by every backend
/// (TensorRT, ONNX Runtime). Standard library only.
///
/// SAM3 is open-vocabulary, so this list IS the class set of a run that
/// passes no `--prompts` / `--prompts-file`; class ids follow its order
/// (API class k = the k-th entry, point label k + 1, STANDARDS §3.1). A run
/// records the list it used as the project's `annotate_class_map`, and
/// every consumer resolves a class by that stored name, so changing this
/// list never renumbers an annotated project.
///
/// Every entry must have a role in core/class_role_map.json (the classify
/// stage turns a plane's majority class into a role); the unit tests check
/// that. The list is kept short: each prompt is one decoder pass per image.

#include <string>
#include <vector>

namespace reusex::vision {

/// The default concept prompts, in class-id order.
inline const std::vector<std::string> &default_prompt_texts() {
  static const std::vector<std::string> texts{
      // Structure and openings.
      "wall",
      "floor",
      "ceiling",
      "column",
      "door",
      "window",
      "glass partition",
      // Furniture and fixtures that stand like walls or in front of them: a
      // vertical plane of one of these is not a wall
      // (core/class_role_map.json).
      "desk divider",
      "cabinet",
      "shelf",
      "desk",
      "table",
      "chair",
      "monitor",
      "radiator",
      "plant",
  };
  return texts;
}

} // namespace reusex::vision
