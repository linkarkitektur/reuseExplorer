// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "core/mask_selection.hpp"

#include "core/ProjectDB.hpp"
#include "core/SensorIntrinsics.hpp"
#include "core/frame_projection.hpp"
#include "core/guid.hpp"
#include "core/label_semantics.hpp"
#include "core/logging.hpp"
#include "core/survey.hpp"

#include <Eigen/Dense>
#include <fmt/format.h>
#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>

#include <algorithm>
#include <climits>
#include <cmath>
#include <map>
#include <set>

namespace reusex::core {

namespace {

using Matrix4dRM = Eigen::Matrix<double, 4, 4, Eigen::RowMajor>;

/// Largest label a CloudL uses, ignoring the pre-#214 wrapped -1
/// (0xFFFFFFFF) so a stray sentinel cannot push the next id past INT_MAX.
uint32_t max_label(const CloudL &cloud) {
  uint32_t m = 0;
  for (const auto &p : cloud)
    if (is_valid_label(p.label) && p.label <= static_cast<uint32_t>(INT_MAX))
      m = std::max(m, p.label);
  return m;
}

std::string trimmed(const std::string &s) {
  const auto b = s.find_first_not_of(" \t\r\n");
  if (b == std::string::npos)
    return {};
  const auto e = s.find_last_not_of(" \t\r\n");
  return s.substr(b, e - b + 1);
}

/// `rux create instances`' label definition for an instance.
std::string instance_definition(uint32_t semantic_class, uint32_t instance_id,
                                int points) {
  return fmt::format("SM{}-{} ({}p)", semantic_class, instance_id, points);
}

/// Load an index-aligned label cloud, or an all-zero one of @p n points when
/// it does not exist. Throws when it exists with the wrong size.
CloudLPtr label_cloud_or_zeros(const ProjectDB &db, const std::string &name,
                               std::size_t n, bool &created) {
  created = !db.has_point_cloud(name);
  if (created) {
    auto cloud = std::make_shared<CloudL>();
    cloud->resize(n);
    for (auto &p : *cloud)
      p.label = kUnlabeled;
    cloud->width = static_cast<uint32_t>(n);
    cloud->height = 1;
    return cloud;
  }
  auto cloud = db.point_cloud_label(name);
  if (!cloud || cloud->size() != n)
    throw MaskSelectionError(fmt::format(
        "apply_mask_selection: '{}' has {} points but the base cloud has {} "
        "— the clouds are out of sync (re-run the stage that wrote '{}')",
        name, cloud ? cloud->size() : 0, n, name));
  return cloud;
}

} // namespace

std::vector<std::size_t> project_frame_mask(const ProjectDB &db, int frame_id,
                                            const cv::Mat &mask,
                                            const MaskProjectionOptions &opts) {
  if (mask.empty() || mask.channels() != 1)
    throw std::invalid_argument(
        "project_frame_mask: mask must be a non-empty single-channel image");
  if (!db.has_sensor_frame(frame_id))
    throw MaskSelectionError(
        fmt::format("sensor frame {} does not exist", frame_id));
  if (!db.has_sensor_frame_pose(frame_id))
    throw MaskSelectionError(fmt::format(
        "sensor frame {} has no stored pose; it cannot be projected into the "
        "cloud",
        frame_id));
  const auto intr = db.sensor_frame_intrinsics(frame_id);
  if (!(intr.fx > 0.0) || !(intr.fy > 0.0) || intr.width <= 0 ||
      intr.height <= 0)
    throw MaskSelectionError(fmt::format(
        "sensor frame {} has degenerate intrinsics (fx={}, fy={}, {}x{})",
        frame_id, intr.fx, intr.fy, intr.width, intr.height));

  const cv::Mat raw_depth = db.sensor_frame_depth(frame_id);
  if (raw_depth.empty())
    throw MaskSelectionError(
        fmt::format("sensor frame {} has no depth image; its mask cannot be "
                    "occlusion-tested against the cloud",
                    frame_id));
  cv::Mat depth; // CV_32F metres
  if (raw_depth.type() == CV_16UC1)
    raw_depth.convertTo(depth, CV_32FC1, 1.0 / 1000.0);
  else if (raw_depth.type() == CV_32FC1)
    depth = raw_depth;
  else
    throw MaskSelectionError(fmt::format(
        "sensor frame {} has a depth image of unsupported type {} (expected "
        "CV_16U millimetres or CV_32F metres)",
        frame_id, raw_depth.type()));

  if (!db.has_point_cloud(opts.cloud))
    throw MaskSelectionError(fmt::format(
        "no '{}' cloud to project into — run `rux create clouds` first",
        opts.cloud));
  const auto cloud = db.point_cloud_xyzrgb(opts.cloud);

  cv::Mat mask8;
  if (mask.type() == CV_8UC1)
    mask8 = mask;
  else
    mask8 = mask != 0; // CV_8U, 255 where non-zero

  const auto pose_array = db.sensor_frame_pose(frame_id); // base -> world
  const Matrix4dRM pose = Eigen::Map<const Matrix4dRM>(pose_array.data());
  const Matrix4dRM local = // camera -> base
      Eigen::Map<const Matrix4dRM>(intr.local_transform.data());
  const Eigen::Matrix4d world_to_camera = (pose * local).inverse();

  const double w = intr.width, h = intr.height;
  const double mask_sx = mask8.cols / w, mask_sy = mask8.rows / h;
  const double depth_sx = depth.cols / w, depth_sy = depth.rows / h;

  std::size_t behind = 0, too_far = 0, out_of_bounds = 0, outside_mask = 0,
              no_depth = 0, occluded = 0;
  std::vector<std::size_t> selected;
  for (std::size_t i = 0; i < cloud->size(); ++i) {
    const auto &pt = (*cloud)[i];
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))
      continue;
    const Eigen::Vector4d pc =
        world_to_camera * Eigen::Vector4d(pt.x, pt.y, pt.z, 1.0);
    const double z = pc.z();
    if (!(z > 0.0)) {
      ++behind;
      continue;
    }
    if (opts.max_depth > 0.0 && z > opts.max_depth) {
      ++too_far;
      continue;
    }
    const double u = intr.fx * pc.x() / z + intr.cx;
    const double v = intr.fy * pc.y() / z + intr.cy;
    if (!(u >= 0.0 && u < w && v >= 0.0 && v < h)) {
      ++out_of_bounds;
      continue;
    }
    const int mu = std::min(static_cast<int>(u * mask_sx), mask8.cols - 1);
    const int mv = std::min(static_cast<int>(v * mask_sy), mask8.rows - 1);
    if (mask8.at<uint8_t>(mv, mu) == 0) {
      ++outside_mask;
      continue;
    }
    const int du = std::min(static_cast<int>(u * depth_sx), depth.cols - 1);
    const int dv = std::min(static_cast<int>(v * depth_sy), depth.rows - 1);
    const float d = depth.at<float>(dv, du);
    if (!(d > 0.0f) || !std::isfinite(d)) {
      ++no_depth;
      continue;
    }
    if (std::abs(z - static_cast<double>(d)) > opts.depth_tolerance) {
      ++occluded;
      continue;
    }
    selected.push_back(i);
  }

  if (selected.empty())
    reusex::warn(
        "project_frame_mask: frame {} selected 0 of {} '{}' points ({} behind "
        "the camera, {} beyond max_depth {:.2f} m, {} outside the image, {} "
        "outside the mask, {} with no depth reading, {} occluded beyond {:.2f} "
        "m; mask has {} non-zero pixels)",
        frame_id, cloud->size(), opts.cloud, behind, too_far, opts.max_depth,
        out_of_bounds, outside_mask, no_depth, occluded, opts.depth_tolerance,
        cv::countNonZero(mask8));
  else
    reusex::info("project_frame_mask: frame {} selected {} of {} '{}' points "
                 "({} outside the mask, {} occluded, {} with no depth)",
                 frame_id, selected.size(), cloud->size(), opts.cloud,
                 outside_mask, occluded, no_depth);
  return selected;
}

std::optional<ViewRegion> mask_region(const cv::Mat &mask) {
  if (mask.empty())
    return std::nullopt;
  if (mask.channels() != 1 || (mask.depth() != CV_8U && mask.depth() != CV_32S))
    throw std::invalid_argument(
        "mask_region: mask must be single-channel CV_8U or CV_32S");
  int x0 = mask.cols, y0 = mask.rows, x1 = -1, y1 = -1;
  for (int y = 0; y < mask.rows; ++y)
    for (int x = 0; x < mask.cols; ++x) {
      const bool on = mask.depth() == CV_8U ? mask.at<std::uint8_t>(y, x) != 0
                                            : mask.at<std::int32_t>(y, x) != 0;
      if (!on)
        continue;
      x0 = std::min(x0, x);
      y0 = std::min(y0, y);
      x1 = std::max(x1, x);
      y1 = std::max(y1, y);
    }
  if (x1 < 0)
    return std::nullopt;
  const double w = mask.cols;
  const double h = mask.rows;
  return ViewRegion{x0 / w, y0 / h, (x1 - x0 + 1) / w, (y1 - y0 + 1) / h};
}

MaskSelectionResult
apply_mask_selection(ProjectDB &db, const std::vector<std::size_t> &indices,
                     const std::string &class_name,
                     const MaskSelectionOptions &opts) {
  const std::string cls = trimmed(class_name);
  if (cls.empty())
    throw std::invalid_argument(
        "apply_mask_selection: class_name must be non-empty");
  if (indices.empty())
    throw MaskSelectionError("the selection contains no points (the mask did "
                             "not cover any visible point of the cloud)");
  if (opts.view_region)
    validate_view_shape({ViewKind::frame, 0, opts.view_region});

  // Every read below happens under BEGIN IMMEDIATE: with_write serialises
  // writers inside the server only, and a CLI process (`rux create
  // instances`, `rux set`) committing between our read and our write would
  // otherwise be overwritten from stale copies.
  ProjectDB::Transaction tx(db);
  if (!db.has_point_cloud(opts.base_cloud))
    throw MaskSelectionError(fmt::format(
        "no '{}' cloud — run `rux create clouds` first", opts.base_cloud));
  if (opts.type_id) {
    const auto type = db.survey_type(*opts.type_id);
    if (!type)
      throw std::out_of_range("no survey type " +
                              std::to_string(*opts.type_id));
    // A rejected (Afvist) type is out of the survey: a part filed there
    // would be hidden from the moment it is created.
    if (type->review_status == ReviewStatus::rejected)
      throw MaskSelectionError(
          fmt::format("survey type {} ('{}') is rejected; choose another type "
                      "or omit type_id",
                      type->id, type->name));
  }

  const std::size_t n = db.point_cloud_page(opts.base_cloud, 0, 0).total;
  std::vector<std::size_t> sel(indices);
  std::sort(sel.begin(), sel.end());
  sel.erase(std::unique(sel.begin(), sel.end()), sel.end());
  if (sel.back() >= n)
    throw std::invalid_argument(fmt::format(
        "apply_mask_selection: point index {} is out of range for '{}' ({} "
        "points)",
        sel.back(), opts.base_cloud, n));

  MaskSelectionResult out;

  // --- Semantic class ------------------------------------------------------
  bool labels_created = false;
  auto labels =
      label_cloud_or_zeros(db, opts.semantic_cloud, n, labels_created);
  auto label_defs = labels_created ? std::map<int, std::string>{}
                                   : db.label_definitions(opts.semantic_cloud);
  for (const auto &[id, name] : label_defs)
    if (id >= 1 && name == cls) {
      out.label_id = static_cast<uint32_t>(id);
      break;
    }
  if (out.label_id == 0) {
    uint32_t next = max_label(*labels);
    if (!label_defs.empty())
      next = std::max<uint32_t>(
          next, static_cast<uint32_t>(std::max(0, label_defs.rbegin()->first)));
    out.label_id = next + 1;
    out.label_created = true;
    label_defs[static_cast<int>(out.label_id)] = cls;
  }
  for (auto i : sel)
    (*labels)[i].label = out.label_id;

  // --- Instance ------------------------------------------------------------
  bool instances_created = false;
  auto inst =
      label_cloud_or_zeros(db, opts.instance_cloud, n, instances_created);
  const auto rows = instances_created ? std::vector<ProjectDB::InstanceRecord>{}
                                      : db.instances(opts.instance_cloud);
  auto inst_defs = instances_created
                       ? std::map<int, std::string>{}
                       : db.label_definitions(opts.instance_cloud);
  uint32_t next_inst = max_label(*inst);
  for (const auto &r : rows)
    next_inst = std::max(next_inst, r.instance_id);
  if (!inst_defs.empty())
    next_inst = std::max<uint32_t>(
        next_inst,
        static_cast<uint32_t>(std::max(0, inst_defs.rbegin()->first)));
  out.instance_id = next_inst + 1;

  std::set<uint32_t> donors;
  for (auto i : sel) {
    const uint32_t old = (*inst)[i].label;
    if (is_valid_label(old))
      donors.insert(old);
    (*inst)[i].label = out.instance_id;
  }
  out.point_count = static_cast<int>(sel.size());
  out.instance_guid = generate_guid();

  // Exact recount of every instance that gave up points.
  std::map<uint32_t, int> donor_counts;
  for (auto d : donors)
    donor_counts[d] = 0;
  if (!donors.empty())
    for (const auto &p : *inst)
      if (auto it = donor_counts.find(p.label); it != donor_counts.end())
        ++it->second;
  std::map<uint32_t, int> class_of_row;
  for (const auto &r : rows)
    class_of_row[r.instance_id] = r.semantic_class;
  for (const auto &[id, count] : donor_counts) {
    out.shrunk_instances.emplace_back(id, count);
    const auto def = inst_defs.find(static_cast<int>(id));
    const auto row_cls = class_of_row.find(id);
    if (row_cls != class_of_row.end() && row_cls->second >= 0)
      inst_defs[static_cast<int>(id)] = instance_definition(
          static_cast<uint32_t>(row_cls->second), id, count);
    else if (def != inst_defs.end())
      reusex::debug("apply_mask_selection: instance {} has no semantic class; "
                    "kept its definition '{}'",
                    id, def->second);
  }
  inst_defs[static_cast<int>(out.instance_id)] =
      instance_definition(out.label_id, out.instance_id, out.point_count);

  // --- Room (majority of the selected points), as sync_survey does ---------
  std::optional<uint32_t> room_id;
  std::string room_name;
  if (db.has_point_cloud(opts.rooms_cloud)) {
    const auto rooms = db.point_cloud_label(opts.rooms_cloud);
    if (rooms && rooms->size() == n) {
      std::map<uint32_t, std::size_t> votes;
      for (auto i : sel)
        if (const auto r = (*rooms)[i].label;
            is_valid_label(r) && r <= static_cast<uint32_t>(INT_MAX))
          ++votes[r];
      if (!votes.empty()) {
        const auto best = std::max_element(
            votes.begin(), votes.end(),
            [](const auto &a, const auto &b) { return a.second < b.second; });
        room_id = best->first;
        const auto names = db.label_definitions(opts.rooms_cloud);
        const auto nm = names.find(static_cast<int>(*room_id));
        room_name =
            nm != names.end() ? nm->second : "Rum " + std::to_string(*room_id);
      }
    } else {
      reusex::warn("apply_mask_selection: '{}' has {} labels but '{}' has {} "
                   "points; the new part gets no room",
                   opts.rooms_cloud, rooms ? rooms->size() : 0, opts.base_cloud,
                   n);
    }
  }

  // --- Write everything in one transaction ---------------------------------
  const nlohmann::json params{{"frame_id", opts.frame_id},
                              {"class_name", cls},
                              {"points", out.point_count},
                              {"semantic_cloud", opts.semantic_cloud},
                              {"instance_cloud", opts.instance_cloud}};
  const int log_id = db.log_pipeline_start("segment_resource", params.dump());

  // An edited cloud keeps the stage/parameters it was produced with (e.g.
  // reconstruct's storage_order); the edit itself is in pipeline_log. A
  // cloud created here is attributed to segment_resource.
  const auto provenance = [&](const std::string &name, bool created) {
    return created ? std::pair<std::string, std::string>{"segment_resource", ""}
                   : db.point_cloud_provenance(name);
  };
  const auto label_prov = provenance(opts.semantic_cloud, labels_created);
  const auto inst_prov = provenance(opts.instance_cloud, instances_created);
  db.save_point_cloud(opts.semantic_cloud, *labels, label_prov.first,
                      label_prov.second);
  db.save_label_definitions(opts.semantic_cloud, label_defs);
  db.save_point_cloud(opts.instance_cloud, *inst, inst_prov.first,
                      inst_prov.second);
  for (const auto &[id, count] : out.shrunk_instances)
    if (class_of_row.count(id) != 0)
      db.set_instance_point_count(opts.instance_cloud, id, count);
  db.add_instance(opts.instance_cloud,
                  {out.instance_id, out.instance_guid,
                   static_cast<int>(out.label_id), out.point_count});
  db.save_label_definitions(opts.instance_cloud, inst_defs);

  if (opts.type_id) {
    out.type_id = *opts.type_id;
  } else {
    // Rejected (Afvist) types never take a new part: a match there would
    // hide the part the moment it is created. With only a rejected match,
    // a new type is created rather than the rejected one revived — reviving
    // would overturn the reviewer's decision behind their back.
    auto types = db.survey_types();
    std::erase_if(types, [](const auto &t) {
      return t.review_status == ReviewStatus::rejected;
    });
    // The type sync_survey files this class under, else one named after it.
    // A class created just now has no sync_survey type: a type carrying its
    // id is a leftover from an earlier `labels` generation, not this class.
    auto it = types.end();
    if (!out.label_created)
      it = std::find_if(types.begin(), types.end(), [&](const auto &t) {
        return t.semantic_class == static_cast<int>(out.label_id);
      });
    if (it == types.end())
      it = std::find_if(types.begin(), types.end(),
                        [&](const auto &t) { return t.name == cls; });
    if (it != types.end()) {
      out.type_id = it->id;
    } else {
      ProjectDB::SurveyTypeRecord t;
      t.name = cls;
      t.semantic_class = static_cast<int>(out.label_id);
      out.type_id = db.add_survey_type(t).id;
      out.type_created = true;
    }
  }

  ProjectDB::SurveyPartRecord part;
  part.code = part_code(db.max_survey_part_number() + 1);
  part.type_id = out.type_id;
  part.cloud_name = opts.instance_cloud;
  part.instance_id = out.instance_id;
  part.room_id = room_id;
  part.room_name = room_name;
  db.add_survey_part(part);
  out.resource_code = part.code;

  if (opts.frame_id >= 0) {
    if (!db.has_sensor_frame(opts.frame_id)) {
      reusex::warn("apply_mask_selection: frame {} does not exist; part {} "
                   "gets no resource image",
                   opts.frame_id, part.code);
    } else {
      std::optional<ViewRegion> region;
      if (opts.view_region) {
        region = pad_region(*opts.view_region);
      } else {
        const auto base = db.point_cloud_xyzrgb(opts.base_cloud);
        if (!base) {
          reusex::warn("apply_mask_selection: '{}' did not load; part {} "
                       "gets no fallback region in frame {}",
                       opts.base_cloud, part.code, opts.frame_id);
        } else {
          std::vector<Eigen::Vector3f> pts;
          pts.reserve(sel.size());
          for (const auto i : sel)
            pts.emplace_back(base->points[i].x, base->points[i].y,
                             base->points[i].z);
          region = project_instance_bbox(db, opts.frame_id, pts);
        }
      }
      out.view = ResourceView{ViewKind::frame, opts.frame_id, region};
      db.set_survey_part_view(part.code, out.view);
    }
  }

  db.log_pipeline_end(log_id, true);
  tx.commit();

  for (const auto &[id, count] : out.shrunk_instances)
    if (count == 0)
      reusex::warn("apply_mask_selection: instance {} in '{}' lost all its "
                   "points to new instance {}; its row, material link and "
                   "survey part are kept with 0 points",
                   id, opts.instance_cloud, out.instance_id);
  reusex::info("apply_mask_selection: {} points -> class {} '{}'{}, instance "
               "{} ({}), part {} in type {}{}; {} instance(s) gave up points",
               out.point_count, out.label_id, cls,
               out.label_created ? " (new)" : "", out.instance_id,
               out.instance_guid, out.resource_code, out.type_id,
               out.type_created ? " (new)" : "", out.shrunk_instances.size());
  return out;
}

} // namespace reusex::core
