// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "reusex/vision/IData.hpp"
#include "reusex/vision/default_prompts.hpp"
#include "reusex/vision/tensor_rt/Sam3Type.hpp"

#include <opencv2/core/mat.hpp>

#include <array>
#include <memory>
#include <utility>
#include <vector>

namespace reusex::vision::tensor_rt {

/* TensorRTData is a struct that implements the IData interface and contains a
 * cv::Mat image. This struct is used to store the image data that will be
 * processed by TensorRT.
 */
struct TensorRTData : IData {

  using Vec = std::array<int64_t, 32>;
  using Prompt = std::pair<std::shared_ptr<Vec>, std::shared_ptr<Vec>>;

  cv::Mat image;
  // The built-in default class list (vision/default_prompts.hpp), shared
  // with the ONNX backend. Override per-run with `rux create annotate
  // --prompts` / `--prompts-file` (the prompt list IS the class set for this
  // open-vocab model).
  std::vector<Sam3PromptUnit> prompts = default_prompt_units();

  float confidence_threshold = 0.5f;

  /// Input: write each detection's prompt position (0..N-1 in `prompts`) as
  /// its label value instead of the model's per-text cache id. Set by the
  /// single-image callers (segment_image, segment_panorama), whose callers
  /// map label k to prompt k. Left false by the annotate dataset, whose class
  /// map relies on the cache-id numbering. See sam3_label_value().
  bool label_by_prompt_index = false;

  /// Input: for a prompt with boxes or points, keep only the detections they
  /// select (vision::sam3_detection_selected; a point keeps the one object
  /// under it). SAM3 treats a box as an exemplar and returns every similar
  /// object in the image; the single-image selection callers (segment_image)
  /// want the object the geometry points at. Points are only fed to the model
  /// when this is set. Left false by the annotate dataset.
  bool select_box_instances = false;

  /// Output (on forward() results): true when the loaded engines take
  /// geometry, i.e. prompt boxes (and points) were fed to SAM3. False means
  /// they were ignored by the model and only the text drove the detections.
  bool geometry_prompts_used = false;

  /// Returns the plain text of the built-in default prompt list so callers
  /// can merge it with extra prompts (e.g. glass classes) without including
  /// this backend-specific header. Same as vision::default_prompt_texts().
  static const std::vector<std::string> &default_prompt_text_list() {
    return default_prompt_texts();
  }

  /// The default list as prompt units (text only, global threshold).
  static std::vector<Sam3PromptUnit> default_prompt_units() {
    std::vector<Sam3PromptUnit> units;
    units.reserve(default_prompt_texts().size());
    for (const auto &text : default_prompt_texts())
      units.emplace_back(text);
    return units;
  }
};
} // namespace reusex::vision::tensor_rt
