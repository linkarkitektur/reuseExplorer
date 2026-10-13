// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "core/instance_evidence.hpp"

#include "core/ProjectDB.hpp"
#include "core/logging.hpp"

#include "instance_points.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <numbers>

namespace reusex::core {

namespace {

using Matrix4dRM = Eigen::Matrix<double, 4, 4, Eigen::RowMajor>;

bool finite(const std::array<double, 16> &pose) {
  return std::all_of(pose.begin(), pose.end(),
                     [](double v) { return std::isfinite(v); });
}

} // namespace

std::map<std::uint32_t, InstanceProbe>
instance_probes(const ProjectDB &db, std::string_view label_cloud,
                std::string_view positions_cloud, std::size_t samples) {
  // Finite points per instance; label 0 is unlabeled (STANDARDS §3).
  const auto members = detail::load_instance_points(
      db, label_cloud, positions_cloud, "instance_probes");

  std::map<std::uint32_t, InstanceProbe> out;
  for (const auto &[label, points] : members) {
    InstanceProbe probe;
    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
    for (const auto &p : points)
      sum += p.cast<double>();
    probe.centroid.point_count = points.size();
    probe.centroid.centroid = sum / static_cast<double>(points.size());

    const std::size_t k = std::min(samples, points.size());
    probe.samples.reserve(k);
    for (std::size_t j = 0; j < k; ++j) {
      // Middle of the j-th of k equal slices of the point list.
      const auto &p = points[(2 * j + 1) * points.size() / (2 * k)];
      probe.samples.emplace_back(p.x(), p.y(), p.z());
    }
    out.emplace(label, std::move(probe));
  }
  return out;
}

std::map<std::uint32_t, InstanceCentroid>
instance_centroids(const ProjectDB &db, std::string_view label_cloud,
                   std::string_view positions_cloud) {
  std::map<std::uint32_t, InstanceCentroid> out;
  for (auto &[label, probe] :
       instance_probes(db, label_cloud, positions_cloud, 0))
    out.emplace(label, probe.centroid);
  return out;
}

std::map<std::uint32_t, InstanceEvidence>
instance_evidence(const ProjectDB &db, std::string_view label_cloud,
                  const PhotoQuery &query, std::string_view positions_cloud,
                  const std::atomic<bool> *cancel) {
  const auto by_instance = instance_probes(db, label_cloud, positions_cloud,
                                           query.samples_per_instance);
  std::vector<VisibilityProbe> probes;
  probes.reserve(by_instance.size());
  for (const auto &[id, probe] : by_instance)
    probes.push_back(probe.probe());
  auto ranked = visible_frames_occluded(db, probes, query.occlusion, cancel);
  std::map<std::uint32_t, InstanceEvidence> out;
  std::size_t i = 0;
  for (const auto &[id, probe] : by_instance)
    out.emplace(id, InstanceEvidence{probe.centroid, std::move(ranked[i++])});
  return out;
}

std::map<std::uint32_t, PartPhotos>
instance_photos(const ProjectDB &db, std::string_view label_cloud,
                const PhotoQuery &query, std::string_view positions_cloud) {
  std::map<std::uint32_t, PartPhotos> out;
  for (const auto &[id, evidence] :
       instance_evidence(db, label_cloud, query, positions_cloud))
    out.emplace(id, evidence.photos());
  return out;
}

Eigen::Vector2d equirect_uv(const Eigen::Vector3d &bearing) {
  const double length = bearing.norm();
  if (!(length > 0.0))
    return {0.5, 0.5};
  const Eigen::Vector3d d = bearing / length;
  const double theta = std::atan2(d.x(), d.z());
  const double phi = std::asin(std::clamp(-d.y(), -1.0, 1.0));
  return {(theta + std::numbers::pi) / (2.0 * std::numbers::pi),
          (std::numbers::pi / 2.0 - phi) / std::numbers::pi};
}

std::vector<PanoramaSighting>
panoramas_for_point(const ProjectDB &db, const Eigen::Vector3d &world_point,
                    const PanoramaQuery &query) {
  const auto panoramas = db.list_panoramic_images();

  // The levelled, heading-free basis (frontend `LEVELLED_BASIS`): columns
  // are the panorama's +x (right), +y (down), +z (forward) in world.
  Eigen::Matrix3d levelled;
  levelled.col(0) = Eigen::Vector3d(0, -1, 0);
  levelled.col(1) = Eigen::Vector3d(0, 0, -1);
  levelled.col(2) = Eigen::Vector3d(1, 0, 0);

  std::vector<PanoramaSighting> out;
  std::size_t placeable = 0;
  for (const auto &pano : panoramas) {
    Eigen::Vector3d centre;
    Eigen::Matrix3d basis; // panorama -> world
    PanoramaHeading heading = PanoramaHeading::resected;
    if (pano.has_pose && finite(pano.pose)) {
      const Matrix4dRM pose = Eigen::Map<const Matrix4dRM>(pano.pose.data());
      centre = pose.block<3, 1>(0, 3);
      basis = pose.block<3, 3>(0, 0);
    } else if (pano.node_id >= 0 && db.has_sensor_frame_pose(pano.node_id)) {
      const auto frame_pose = db.sensor_frame_pose(pano.node_id);
      if (!finite(frame_pose))
        continue;
      centre = Eigen::Vector3d(frame_pose[3], frame_pose[7], frame_pose[11]);
      basis = levelled;
      heading = PanoramaHeading::levelled;
    } else {
      continue;
    }
    ++placeable;

    const Eigen::Vector3d offset = world_point - centre;
    const double distance = offset.norm();
    if (query.max_distance > 0.0 && distance > query.max_distance)
      continue;
    const Eigen::Vector2d uv = equirect_uv(basis.transpose() * offset);
    out.push_back(PanoramaSighting{pano.id, pano.node_id, distance, uv.x(),
                                   uv.y(), heading});
  }

  std::sort(out.begin(), out.end(),
            [](const PanoramaSighting &a, const PanoramaSighting &b) {
              if (a.distance != b.distance)
                return a.distance < b.distance;
              return a.panorama_id < b.panorama_id;
            });

  if (!panoramas.empty() && placeable == 0) {
    reusex::warn("panoramas_for_point: none of {} panoramas can be placed (no "
                 "aligned pose and no posed matched frame) — 0 panoramas near "
                 "({:.3f}, {:.3f}, {:.3f})",
                 panoramas.size(), world_point.x(), world_point.y(),
                 world_point.z());
  } else {
    reusex::debug("panoramas_for_point: {}/{} placeable panoramas within {} m "
                  "of ({:.3f}, {:.3f}, {:.3f})",
                  out.size(), placeable, query.max_distance, world_point.x(),
                  world_point.y(), world_point.z());
  }
  return out;
}

} // namespace reusex::core
