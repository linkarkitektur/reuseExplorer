// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "vision/segment_panorama.hpp"

#include "core/logging.hpp"
#include "geometry/EquirectProjection.hpp"
#include "vision/IData.hpp"
#include "vision/IDataset.hpp"
#include "vision/tensor_rt/Data.hpp"

#include <opencv2/core.hpp>

#include <memory>
#include <span>
#include <utility>
#include <vector>

namespace reusex::vision {

cv::Mat segment_panorama(IModel &model, const cv::Mat &equirect_bgr,
                         const SegmentPanoramaOptions &opts) {
  using tensor_rt::Sam3PromptUnit;
  using tensor_rt::TensorRTData;

  if (equirect_bgr.empty()) {
    reusex::warn("segment_panorama called with an empty panorama");
    return cv::Mat(equirect_bgr.size(), CV_32S, cv::Scalar(-1));
  }

  // 1. Slice the equirect into overlapping perspective tiles the SAM3 model
  //    can consume at its native resolution.
  auto tiles = geometry::overlapping_views(equirect_bgr, opts.n_yaw,
                                           opts.fov_deg, opts.tile);
  reusex::info("segment_panorama: {} tiles (n_yaw={}, fov={}deg, tile={}px)",
               tiles.size(), opts.n_yaw, opts.fov_deg, opts.tile);

  // Build the prompt list once: either the caller's texts or the TensorRTData
  // default prompt set. Using the same list for every tile keeps SAM3's
  // per-text class-id assignment stable across the whole panorama.
  std::vector<Sam3PromptUnit> prompts;
  if (opts.prompts.empty()) {
    prompts = TensorRTData::default_prompt_units(); // vision/default_prompts
  } else {
    prompts.reserve(opts.prompts.size());
    for (const auto &text : opts.prompts)
      prompts.emplace_back(text);
  }

  // 2. Build one IDataset::Pair per tile exactly as the tensor_rt dataset does
  //    (a TensorRTData carrying the tile image, prompt list, and confidence),
  //    then run them all through forward() as a single batch (the model chunks
  //    internally).
  std::vector<IDataset::Pair> batch;
  batch.reserve(tiles.size());
  for (std::size_t i = 0; i < tiles.size(); ++i) {
    auto data = std::make_unique<TensorRTData>();
    data->image = tiles[i].image;
    data->prompts = prompts;
    data->confidence_threshold = opts.confidence;
    // Label k = prompt k, the same numbering the ONNX backend and the GUI's
    // `labels` map use (not the model's per-text cache id).
    data->label_by_prompt_index = true;
    batch.emplace_back(std::move(data), i);
  }

  std::span<IDataset::Pair> batch_span(batch);
  auto results = model.forward(batch_span);

  // Extract the per-tile CV_32S label maps from the forward() output. Each
  // result Pair's IData is a TensorRTData whose image is the CV_32S (-1 bg)
  // label map produced by the SAM3 postprocess/OSD path.
  std::vector<cv::Mat> tile_labels(tiles.size());
  std::size_t detections = 0;
  for (auto &[item, index] : results) {
    if (index >= tile_labels.size()) {
      reusex::warn("segment_panorama: result index {} out of range", index);
      continue;
    }
    auto *trt = dynamic_cast<TensorRTData *>(item.get());
    if (!trt || trt->image.empty()) {
      reusex::warn("segment_panorama: tile {} produced no label map", index);
      tile_labels[index] =
          cv::Mat(tiles[index].image.size(), CV_32S, cv::Scalar(-1));
      continue;
    }
    tile_labels[index] = trt->image;
    detections += static_cast<std::size_t>(cv::countNonZero(trt->image != -1));
  }

  // Fill any tiles the model skipped (e.g. empty results) with background so
  // stitch sees a 1:1 tile/label alignment.
  for (std::size_t i = 0; i < tile_labels.size(); ++i)
    if (tile_labels[i].empty())
      tile_labels[i] = cv::Mat(tiles[i].image.size(), CV_32S, cv::Scalar(-1));

  reusex::debug("segment_panorama: {} labelled pixels across {} tiles",
                detections, tiles.size());

  // 3. Stitch the per-tile label maps back into a single equirect label image.
  return geometry::stitch_labels_to_equirect(equirect_bgr.size(), tiles,
                                             tile_labels);
}

} // namespace reusex::vision
