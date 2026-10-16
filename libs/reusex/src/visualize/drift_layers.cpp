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
#include <opencv2/core.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
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

PreviewCloud preview_cloud(const ProjectDB &db, const PreviewCloudOptions &o,
                           std::span<const Eigen::Matrix4d> poses) {
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

  std::unordered_set<VoxelKey, VoxelHash> kept;
  std::vector<PassSegment> seg(static_cast<std::size_t>(o.segments));
  for (int s = 0; s < o.segments; ++s)
    seg[static_cast<std::size_t>(s)].index = s;

  for (std::size_t k = 0; k < nf; ++k) {
    const int id = static_cast<int>(pc.node_ids[k]);
    auto &sg = seg[static_cast<std::size_t>(pc.frame_segment[k])];
    if (sg.frames == 0)
      sg.first_node = id;
    sg.last_node = id;
    ++sg.frames;

    Eigen::Matrix4d pose;
    if (!poses.empty()) {
      pose = poses[order[k]];
    } else if (db.has_sensor_frame_pose(id)) {
      pose = to_matrix(db.sensor_frame_pose(id));
    } else {
      ++pc.frames_without_pose;
      continue;
    }
    const cv::Mat depth16 = db.sensor_frame_depth(id);
    const core::SensorIntrinsics intr = db.sensor_frame_intrinsics(id);
    if (depth16.empty() || intr.fx <= 0.0 || intr.fy <= 0.0 ||
        intr.width <= 0 || intr.height <= 0) {
      ++pc.frames_without_depth;
      continue;
    }
    cv::Mat depth;
    depth16.convertTo(depth, CV_32FC1, 1.0 / 1000.0);
    // The intrinsics describe the colour frame; scale them to the depth
    // resolution (as segmentation/reconstruct.cpp and slam do).
    const double sx = static_cast<double>(depth.cols) / intr.width;
    const double sy = static_cast<double>(depth.rows) / intr.height;
    const double fx = intr.fx * sx, fy = intr.fy * sy;
    const double cx = intr.cx * sx, cy = intr.cy * sy;
    const Eigen::Matrix4d c2w = pose * to_matrix(intr.local_transform);
    const Eigen::Matrix3d r = c2w.topLeftCorner<3, 3>();
    const Eigen::Vector3d t = c2w.topRightCorner<3, 1>();
    std::size_t added = 0;
    for (int v = 0; v < depth.rows; v += o.depth_stride) {
      const float *row = depth.ptr<float>(v);
      for (int u = 0; u < depth.cols; u += o.depth_stride) {
        const double z = row[u];
        if (!(z >= o.min_depth_m && z <= o.max_depth_m))
          continue;
        const Eigen::Vector3d w =
            r * Eigen::Vector3d((u - cx) * z / fx, (v - cy) * z / fy, z) + t;
        if (o.voxel_m > 0.0) {
          const VoxelKey key{
              static_cast<std::int64_t>(std::floor(w.x() / o.voxel_m)),
              static_cast<std::int64_t>(std::floor(w.y() / o.voxel_m)),
              static_cast<std::int64_t>(std::floor(w.z() / o.voxel_m)),
              pc.frame_segment[k]};
          if (!kept.insert(key).second)
            continue;
        }
        pc.points.push_back(w.cast<float>());
        pc.frame.push_back(static_cast<std::uint32_t>(k));
        ++added;
      }
    }
    if (added == 0 && sg.points == 0)
      core::debug("preview cloud: frame {} adds no point", id);
    sg.points += added;
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
