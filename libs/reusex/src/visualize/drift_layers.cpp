// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// The VTK-free half of the drift-loop layers (reusex-rcv.1.4): the preview
// cloud, the colour scales and the drift-report reader. The actors are built
// in scene.cpp next to the other layers.

#include "visualize/drift_layers.hpp"

#include "core/ProjectDB.hpp"
#include "core/SensorIntrinsics.hpp"
#include "core/logging.hpp"
#include "visualize/scene.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <tuple>
#include <unordered_set>

namespace reusex::visualize {

namespace {

using RowMajor4d = Eigen::Matrix<double, 4, 4, Eigen::RowMajor>;

Eigen::Matrix4d to_matrix(const std::array<double, 16> &a) {
  return Eigen::Map<const RowMajor4d>(a.data());
}

/// Hash of a (voxel, segment) key; the key itself decides equality.
struct VoxelKey {
  std::int64_t x, y, z;
  int segment;
  bool operator==(const VoxelKey &) const = default;
};
struct VoxelHash {
  std::size_t operator()(const VoxelKey &k) const noexcept {
    std::size_t h = std::hash<std::int64_t>{}(k.x);
    h = h * 1000003u ^ std::hash<std::int64_t>{}(k.y);
    h = h * 1000003u ^ std::hash<std::int64_t>{}(k.z);
    return h * 1000003u ^ std::hash<int>{}(k.segment);
  }
};

} // namespace

int pass_segment(std::size_t k, std::size_t n, int segments) {
  if (n == 0 || segments < 1)
    return 0;
  const auto s = static_cast<std::size_t>(segments);
  return static_cast<int>(std::min(k * s / n, s - 1));
}

std::array<std::uint8_t, 3> pass_color(const LabelPalette &palette,
                                       int segment) {
  if (palette.colors.empty())
    return palette.unlabeled;
  const auto n = static_cast<int>(palette.colors.size());
  return palette.colors[static_cast<std::size_t>(((segment % n) + n) % n)];
}

core::DepthSampling preview_depth_sampling(const PreviewCloudOptions &o) {
  core::DepthSampling s;
  s.frame_step = o.frame_step;
  s.depth_stride = o.depth_stride;
  s.min_depth_m = o.min_depth_m;
  s.max_depth_m = o.max_depth_m;
  return s;
}

PreviewCloud preview_cloud(const ProjectDB &db, const PreviewCloudOptions &o,
                           std::span<const Eigen::Matrix4d> poses,
                           const core::FrameDepthCache *depth_cache) {
  if (o.segments < 1 || o.frame_step < 1 || o.depth_stride < 1 ||
      !(o.max_depth_m > o.min_depth_m) || o.voxel_m < 0.0)
    throw std::invalid_argument(
        "preview_cloud: segments, frame_step and depth_stride must be >= 1, "
        "max_depth > min_depth and voxel >= 0");
  const auto ids = db.sensor_frame_ids();
  if (!poses.empty() && poses.size() != ids.size())
    throw std::invalid_argument(
        "preview_cloud: " + std::to_string(poses.size()) + " poses for " +
        std::to_string(ids.size()) + " sensor frames");
  if (depth_cache &&
      !depth_cache->matches(preview_depth_sampling(o), ids.size()))
    throw std::invalid_argument(
        "preview_cloud: the depth cache was made with another frame step, "
        "depth stride or depth range, or for another project (" +
        std::to_string(depth_cache->frames_total) + " frames, this one has " +
        std::to_string(ids.size()) + ")");

  PreviewCloud pc;
  std::vector<std::size_t> order; // index into ids of each sampled frame
  for (std::size_t k = 0; k < ids.size();
       k += static_cast<std::size_t>(o.frame_step)) {
    pc.node_ids.push_back(ids[k]);
    order.push_back(k);
  }
  const std::size_t nf = pc.node_ids.size();
  pc.frame_segment.resize(nf);
  for (std::size_t k = 0; k < nf; ++k)
    pc.frame_segment[k] = pass_segment(k, nf, o.segments);

  // The poses first (the database is read on this thread only).
  std::vector<std::optional<Eigen::Matrix4d>> pose_of(nf);
  for (std::size_t k = 0; k < nf; ++k) {
    const int id = static_cast<int>(pc.node_ids[k]);
    if (!poses.empty())
      pose_of[k] = poses[order[k]];
    else if (db.has_sensor_frame_pose(id))
      pose_of[k] = to_matrix(db.sensor_frame_pose(id));
  }

  // Segments are runs of consecutive frames and the voxel key holds the
  // segment, so each segment thins on its own: with the cache, segments in
  // parallel, then joined in segment (= frame) order -- the points and
  // their order a single pass makes (reusex-rcv.1.17).
  std::vector<PassSegment> seg(static_cast<std::size_t>(o.segments));
  for (int s = 0; s < o.segments; ++s)
    seg[static_cast<std::size_t>(s)].index = s;
  struct SegmentOut {
    std::vector<Eigen::Vector3f> points;
    std::vector<std::uint32_t> frame;
    std::size_t without_pose = 0, without_depth = 0;
  };
  std::vector<SegmentOut> outs(static_cast<std::size_t>(o.segments));
  std::vector<std::size_t> seg_begin(static_cast<std::size_t>(o.segments) + 1,
                                     nf);
  for (std::size_t k = nf; k-- > 0;)
    seg_begin[static_cast<std::size_t>(pc.frame_segment[k])] = k;
  for (int s = o.segments - 1; s >= 0; --s)
    seg_begin[static_cast<std::size_t>(s)] =
        std::min(seg_begin[static_cast<std::size_t>(s)],
                 seg_begin[static_cast<std::size_t>(s) + 1]);

  auto run_segment = [&](int s) {
    auto &sg = seg[static_cast<std::size_t>(s)];
    auto &out = outs[static_cast<std::size_t>(s)];
    std::unordered_set<VoxelKey, VoxelHash> kept;
    for (std::size_t k = seg_begin[static_cast<std::size_t>(s)];
         k < seg_begin[static_cast<std::size_t>(s) + 1]; ++k) {
      const int id = static_cast<int>(pc.node_ids[k]);
      if (sg.frames == 0)
        sg.first_node = id;
      sg.last_node = id;
      ++sg.frames;
      if (!pose_of[k]) {
        ++out.without_pose;
        continue;
      }
      const Eigen::Matrix4d &pose = *pose_of[k];
      // From the cache, or this frame's depth read now: the same pixels
      // (core::cache_depth_frame()) through the same formula below.
      core::CachedDepthFrame read;
      if (!depth_cache)
        read = core::cache_depth_frame(db, id, o.depth_stride, o.min_depth_m,
                                       o.max_depth_m);
      const core::CachedDepthFrame &frame =
          depth_cache ? depth_cache->frames[k] : read;
      const core::SensorIntrinsics &intr = frame.intrinsics;
      if (!frame.has_depth || intr.fx <= 0.0 || intr.fy <= 0.0 ||
          intr.width <= 0 || intr.height <= 0) {
        ++out.without_depth;
        continue;
      }
      // The intrinsics describe the colour frame; scale them to the depth
      // resolution (as segmentation/reconstruct.cpp and slam do).
      const double sx = static_cast<double>(frame.depth_cols) / intr.width;
      const double sy = static_cast<double>(frame.depth_rows) / intr.height;
      const double fx = intr.fx * sx, fy = intr.fy * sy;
      const double cx = intr.cx * sx, cy = intr.cy * sy;
      const Eigen::Matrix4d c2w = pose * to_matrix(intr.local_transform);
      const Eigen::Matrix3d r = c2w.topLeftCorner<3, 3>();
      const Eigen::Vector3d t = c2w.topRightCorner<3, 1>();
      std::size_t added = 0;
      for (const auto &px : frame.samples) {
        const int u = px.u, v = px.v;
        const double z = px.z;
        if (!(z >= o.min_depth_m && z <= o.max_depth_m))
          continue;
        const Eigen::Vector3d w =
            r * Eigen::Vector3d((u - cx) * z / fx, (v - cy) * z / fy, z) + t;
        if (o.voxel_m > 0.0) {
          const VoxelKey key{
              static_cast<std::int64_t>(std::floor(w.x() / o.voxel_m)),
              static_cast<std::int64_t>(std::floor(w.y() / o.voxel_m)),
              static_cast<std::int64_t>(std::floor(w.z() / o.voxel_m)), s};
          if (!kept.insert(key).second)
            continue;
        }
        out.points.push_back(w.cast<float>());
        out.frame.push_back(static_cast<std::uint32_t>(k));
        ++added;
      }
      sg.points += added;
    }
  };
  if (depth_cache) {
#pragma omp parallel for schedule(dynamic, 1)
    for (int s = 0; s < o.segments; ++s)
      run_segment(s);
  } else {
    for (int s = 0; s < o.segments; ++s)
      run_segment(s);
  }
  std::size_t total = 0;
  for (const auto &out : outs)
    total += out.points.size();
  pc.points.reserve(total);
  pc.frame.reserve(total);
  for (auto &out : outs) {
    pc.points.insert(pc.points.end(), out.points.begin(), out.points.end());
    pc.frame.insert(pc.frame.end(), out.frame.begin(), out.frame.end());
    pc.frames_without_pose += out.without_pose;
    pc.frames_without_depth += out.without_depth;
  }
  for (const auto &s : seg)
    if (s.frames > 0)
      pc.segments.push_back(s);
  if (pc.frames_without_pose + pc.frames_without_depth > 0)
    core::warn("preview cloud: {} of {} sampled frames contribute no points "
               "({} without a pose, {} without depth or intrinsics)",
               pc.frames_without_pose + pc.frames_without_depth, nf,
               pc.frames_without_pose, pc.frames_without_depth);
  if (pc.points.empty())
    throw std::runtime_error(
        "preview cloud: none of the " + std::to_string(nf) +
        " sampled sensor frames has a pose and depth between " +
        std::to_string(o.min_depth_m) + " and " +
        std::to_string(o.max_depth_m) + " m to back-project");
  return pc;
}

std::array<std::uint8_t, 3> residual_color(double r,
                                           const GraphLayerOptions &opts) {
  if (!std::isfinite(r))
    return opts.unsolved_rgb;
  DeviationOptions d;
  d.within_m = opts.residual_good;
  d.uncovered_m = opts.residual_bad;
  return deviation_color(r, d);
}

std::array<std::uint8_t, 3> drift_color(double r,
                                        const DriftLayerOptions &opts) {
  DeviationOptions d;
  d.within_m = opts.good_m;
  d.uncovered_m = opts.bad_m;
  return deviation_color(r, d);
}

DriftHeatmap drift_heatmap_from_json(std::string_view text) {
  using nlohmann::json;
  json j;
  try {
    j = json::parse(text);
  } catch (const json::exception &e) {
    throw std::invalid_argument(
        std::string("drift report: not a JSON document: ") + e.what());
  }
  if (!j.is_object() || !j.contains("cells") || !j["cells"].is_array())
    throw std::invalid_argument(
        "drift report: no \"cells\" array (expected the JSON of `rux "
        "analyze drift -o`)");
  const auto num = [](const json &o, const char *k, double fallback) {
    return o.contains(k) && o[k].is_number() ? o[k].get<double>() : fallback;
  };
  DriftHeatmap h;
  try {
    h.cell_m = num(j, "cell_m", 0.5);
    if (j.contains("noise_floor_m") && j["noise_floor_m"].is_number())
      h.noise_floor_m = j["noise_floor_m"].get<double>();
    if (j.contains("residual_median_m") && j["residual_median_m"].is_number())
      h.residual_median_m = j["residual_median_m"].get<double>();
    if (j.contains("storeys") && j["storeys"].is_array())
      for (const auto &s : j["storeys"]) {
        DriftHeatStorey d;
        d.idx = s.at("idx").get<int>();
        d.name = s.value("name", std::string());
        d.z_floor = num(s, "z_floor", 0.0);
        d.z_ceiling = num(s, "z_ceiling", 0.0);
        if (s.contains("level_transform"))
          d.level_transform =
              s["level_transform"].get<std::array<double, 16>>();
        h.storeys.push_back(d);
      }
    for (const auto &c : j["cells"]) {
      DriftHeatCell d;
      d.storey = c.contains("storey") ? c["storey"].get<int>() : -1;
      d.x = c.at("x").get<double>();
      d.y = c.at("y").get<double>();
      d.z_m = num(c, "z_m", 0.0);
      d.residual_m = c.at("residual_m").get<double>();
      d.points = c.value("points", std::size_t{0});
      h.cells.push_back(d);
    }
  } catch (const json::exception &e) {
    throw std::invalid_argument(std::string("drift report: ") + e.what());
  }
  if (!(h.cell_m > 0.0))
    throw std::invalid_argument("drift report: cell_m must be positive");
  return h;
}

} // namespace reusex::visualize
