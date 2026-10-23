// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "reusex/vision/IData.hpp"
#include "reusex/vision/default_prompts.hpp"
#include "reusex/vision/onnx/Sam3Type.hpp"

#include <opencv2/core/mat.hpp>

#include <vector>

namespace reusex::vision::onnx {

/// @brief Data carrier for ONNX Runtime SAM3 inference pipeline.
///
/// Holds the input image, text prompts, confidence threshold, and the output
/// label image for a single sample processed through the ONNX SAM3 model.
struct ONNXSam3Data : IData {
  cv::Mat image; ///< Input image (BGR, HWC) / Output label image (CV_32S)

  /// @brief Text prompts for semantic segmentation classes: the built-in
  /// default list (vision/default_prompts.hpp), shared with TensorRT.
  std::vector<Sam3PromptUnit> prompts = [] {
    std::vector<Sam3PromptUnit> units;
    units.reserve(default_prompt_texts().size());
    for (const auto &text : default_prompt_texts())
      units.emplace_back(text);
    return units;
  }();

  float confidence_threshold = 0.5f; ///< Minimum confidence for detections
};

} // namespace reusex::vision::onnx
