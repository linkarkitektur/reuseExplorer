// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// A pose-independent cache of the sampled depth pixels of a project's sensor
// frames (reusex-rcv.1.17). `rux optimize --watch` re-measures the drift and
// re-draws the preview cloud on every cycle with new candidate poses, but the
// depth pixels they back-project do not depend on the poses: only the world
// transform changes. Reading and decoding every depth image once and keeping
// the sampled pixels (u, v, depth) lets each cycle skip the database.
//
// The cache stores pixels, not points, so every consumer still evaluates its
// own back-projection formula (float in the drift diagnostics, double in the
// preview cloud) on exactly the values it would have read from the database:
// a cached run is bit-identical to an uncached one (STANDARDS §6).
//
// Memory: one DepthSample (8 bytes) per kept pixel plus a small per-frame
// header; FrameDepthCache::bytes() reports it.

#include "reusex/core/SensorIntrinsics.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace reusex {
class ProjectDB;
}

namespace reusex::core {

/// Which frames and pixels a cache keeps. The consumers (drift diagnostics,
/// preview cloud) refuse a cache made with other values.
struct DepthSampling {
  int frame_step = 1;       ///< every Nth sensor frame (node order)
  int depth_stride = 8;     ///< every Nth depth pixel in u and v
  double min_depth_m = 0.2; ///< closer returns are not kept ...
  double max_depth_m = 4.0; ///< ... nor farther ones
  bool operator==(const DepthSampling &) const = default;
};

/// One kept depth pixel: raster position and depth [m] as the decoded
/// CV_32F depth image holds it (stored mm / 1000).
struct DepthSample {
  std::uint16_t u = 0, v = 0;
  float z = 0.0f;
};

/// The sampled pixels of one sensor frame.
struct CachedDepthFrame {
  std::int64_t node_id = 0;
  std::size_t order = 0;  ///< index in ProjectDB::sensor_frame_ids()
  bool has_depth = false; ///< a depth image is stored
  int depth_cols = 0, depth_rows = 0;
  SensorIntrinsics intrinsics; ///< as stored (colour resolution)
  /// Raster order (v, then u, both multiples of the stride). A pixel is
  /// kept when its depth lies in [min, max] compared in float OR in double,
  /// so each consumer re-applies its own exact range test.
  std::vector<DepthSample> samples;
};

/// The sampled frames of a project, node order.
struct FrameDepthCache {
  DepthSampling sampling;
  std::size_t frames_total = 0; ///< sensor frames in the project
  std::vector<CachedDepthFrame> frames;

  [[nodiscard]] std::size_t samples() const;
  /// Heap bytes held (samples and per-frame headers).
  [[nodiscard]] std::size_t bytes() const;
  /// True when made with exactly @p s for a project of @p frames frames.
  [[nodiscard]] bool matches(const DepthSampling &s, std::size_t frames) const;
};

/// Read, decode and sample the depth of one frame (node_id, stride, range).
/// A frame without a stored depth image yields has_depth = false and no
/// samples. Read-only.
CachedDepthFrame cache_depth_frame(const ProjectDB &db, int node_id,
                                   int depth_stride, double min_depth_m,
                                   double max_depth_m);

/// Cache every sampled frame of @p db (every frame_step-th id of
/// sensor_frame_ids()). Read-only; logs its size at debug level.
/// @throws std::invalid_argument for frame_step or depth_stride < 1, or
///         max_depth_m <= min_depth_m.
FrameDepthCache cache_frame_depth(const ProjectDB &db,
                                  const DepthSampling &sampling = {});

/// The frame's pixels with depth in [@p min_depth_m, @p max_depth_m]
/// (compared in float), back-projected in float into its POSE frame
/// (local_transform applied, no world pose): the points
/// slam::frame_pose_frame_points() returns, bit for bit. Empty when none.
std::vector<std::array<float, 3>>
pose_frame_points(const CachedDepthFrame &frame, float min_depth_m,
                  float max_depth_m);

} // namespace reusex::core
