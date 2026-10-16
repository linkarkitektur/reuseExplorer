// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "core/frame_depth_cache.hpp"

#include "core/ProjectDB.hpp"
#include "core/logging.hpp"

#include <Eigen/Core>
#include <opencv2/core.hpp>

#include <algorithm>
#include <limits>
#include <stdexcept>

namespace reusex::core {

std::size_t FrameDepthCache::samples() const {
  std::size_t n = 0;
  for (const auto &f : frames)
    n += f.samples.size();
  return n;
}

std::size_t FrameDepthCache::bytes() const {
  std::size_t n = frames.capacity() * sizeof(CachedDepthFrame);
  for (const auto &f : frames)
    n += f.samples.capacity() * sizeof(DepthSample);
  return n;
}

bool FrameDepthCache::matches(const DepthSampling &s,
                              std::size_t frames_in_project) const {
  return sampling == s && frames_total == frames_in_project;
}

CachedDepthFrame cache_depth_frame(const ProjectDB &db, int node_id,
                                   int depth_stride, double min_depth_m,
                                   double max_depth_m) {
  CachedDepthFrame f;
  f.node_id = node_id;
  const cv::Mat depth16 = db.sensor_frame_depth(node_id);
  f.intrinsics = db.sensor_frame_intrinsics(node_id);
  if (depth16.empty())
    return f;
  f.has_depth = true;
  cv::Mat depth;
  depth16.convertTo(depth, CV_32FC1, 1.0 / 1000.0);
  f.depth_cols = depth.cols;
  f.depth_rows = depth.rows;
  if (depth.cols > std::numeric_limits<std::uint16_t>::max() + 1 ||
      depth.rows > std::numeric_limits<std::uint16_t>::max() + 1)
    throw std::runtime_error("frame " + std::to_string(node_id) +
                             ": depth image too large to cache");
  const int step = std::max(1, depth_stride);
  const auto minf = static_cast<float>(min_depth_m);
  const auto maxf = static_cast<float>(max_depth_m);
  f.samples.reserve(static_cast<std::size_t>(depth.rows / step + 1) *
                    static_cast<std::size_t>(depth.cols / step + 1));
  for (int v = 0; v < depth.rows; v += step) {
    const float *row = depth.ptr<float>(v);
    for (int u = 0; u < depth.cols; u += step) {
      const float z = row[u];
      const double zd = z;
      // The float test is slam's, the double test the preview cloud's.
      if ((z >= minf && z <= maxf) || (zd >= min_depth_m && zd <= max_depth_m))
        f.samples.push_back(
            {static_cast<std::uint16_t>(u), static_cast<std::uint16_t>(v), z});
    }
  }
  f.samples.shrink_to_fit();
  return f;
}

FrameDepthCache cache_frame_depth(const ProjectDB &db, const DepthSampling &s) {
  if (s.frame_step < 1 || s.depth_stride < 1 ||
      !(s.max_depth_m > s.min_depth_m))
    throw std::invalid_argument(
        "cache_frame_depth: frame_step and depth_stride must be >= 1 and "
        "max_depth > min_depth");
  const reusex::stopwatch clock;
  FrameDepthCache c;
  c.sampling = s;
  const auto ids = db.sensor_frame_ids();
  c.frames_total = ids.size();
  for (std::size_t k = 0; k < ids.size();
       k += static_cast<std::size_t>(s.frame_step)) {
    auto f = cache_depth_frame(db, ids[k], s.depth_stride, s.min_depth_m,
                               s.max_depth_m);
    f.order = k;
    c.frames.push_back(std::move(f));
  }
  c.frames.shrink_to_fit();
  core::debug("frame depth cache: {} frames, {} samples, {:.1f} MB in {:.1f} s",
              c.frames.size(), c.samples(),
              static_cast<double>(c.bytes()) / (1024.0 * 1024.0),
              clock.elapsed());
  return c;
}

std::vector<std::array<float, 3>> pose_frame_points(const CachedDepthFrame &f,
                                                    float min_depth_m,
                                                    float max_depth_m) {
  std::vector<std::array<float, 3>> pts;
  if (!f.has_depth)
    return pts;
  using RowMajor4d = Eigen::Matrix<double, 4, 4, Eigen::RowMajor>;
  const auto &intr = f.intrinsics;
  // Scale the intrinsics to the depth resolution (as reconstruct.cpp does).
  const double sx = static_cast<double>(f.depth_cols) / std::max(1, intr.width);
  const double sy =
      static_cast<double>(f.depth_rows) / std::max(1, intr.height);
  const float fx = static_cast<float>(intr.fx * sx);
  const float fy = static_cast<float>(intr.fy * sy);
  const float cx = static_cast<float>(intr.cx * sx);
  const float cy = static_cast<float>(intr.cy * sy);
  // world_tf * local_tf with an identity world pose, spelled as
  // slam::frame_world_points() spells it so the two agree bit for bit.
  constexpr std::array<double, 16> identity{1, 0, 0, 0, 0, 1, 0, 0,
                                            0, 0, 1, 0, 0, 0, 0, 1};
  const Eigen::Matrix4f world_tf =
      Eigen::Map<const RowMajor4d>(identity.data()).cast<float>();
  const Eigen::Matrix4f local_tf =
      Eigen::Map<const RowMajor4d>(intr.local_transform.data()).cast<float>();
  const Eigen::Matrix4f tf = world_tf * local_tf;
  pts.reserve(f.samples.size());
  for (const auto &s : f.samples) {
    const float z = s.z;
    if (!(z >= min_depth_m && z <= max_depth_m))
      continue;
    const float x = (static_cast<float>(s.u) - cx) * z / fx;
    const float y = (static_cast<float>(s.v) - cy) * z / fy;
    const Eigen::Vector4f p = tf * Eigen::Vector4f(x, y, z, 1.0f);
    pts.push_back({p[0], p[1], p[2]});
  }
  return pts;
}

} // namespace reusex::core
