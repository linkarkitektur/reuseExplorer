// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "segmentation/segment_rooms.hpp"
#include "core/label_semantics.hpp"
#include "core/logging.hpp"
#include "core/visual_observer.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <stdexcept>
#include <unordered_set>

namespace reusex::geometry {

namespace {

/// options.min_room_area as a sample count: one sample per grid cell.
std::size_t min_room_samples(const SegmentRoomsOptions &options) {
  if (options.min_room_area <= 0.0F || options.grid_size <= 0.0F)
    return 1;
  const double cell =
      static_cast<double>(options.grid_size) * options.grid_size;
  return std::max<std::size_t>(
      1, static_cast<std::size_t>(
             std::ceil(static_cast<double>(options.min_room_area) / cell)));
}

} // namespace

auto propagate_room_labels(CloudConstPtr cloud, CloudLPtr labels,
                           IndicesConstPtr sampled_indices,
                           IndicesConstPtr missing_indices, int k,
                           float max_radius) -> size_t {
  // Distance-bounded k-NN majority propagation. For each non-sampled point we
  // poll up to k sampled neighbours, discard any beyond max_radius, and assign
  // the majority room label among those that remain. Points with no labelled
  // neighbour inside the radius keep their current (unlabeled) label and are
  // counted so the failure is visible (STANDARDS §5: no silent failure).
  pcl::KdTreeFLANN<PointT> kdtree;
  kdtree.setInputCloud(cloud, sampled_indices);

  const int kk = std::max(1, k);
  const float max_sqr_dist = max_radius * max_radius;
  size_t unpropagated = 0;
  std::vector<int> nn_indices(kk);
  std::vector<float> nn_sqr_dists(kk);

  for (size_t i = 0; i < missing_indices->size(); ++i) {
    const size_t idx = missing_indices->at(i);
    const int found =
        kdtree.nearestKSearch(cloud->points[idx], kk, nn_indices, nn_sqr_dists);
    if (found <= 0) {
      // Guard against nearestKSearch returning 0 (empty tree / no neighbour).
      ++unpropagated;
      continue;
    }

    // Majority vote over the in-radius labelled neighbours. Ties break
    // deterministically towards the smallest room label so identical inputs
    // always produce identical output (STANDARDS §6).
    std::map<uint32_t, int> votes;
    for (int n = 0; n < found; ++n) {
      if (nn_sqr_dists[n] > max_sqr_dist)
        continue; // neighbour too far away
      const uint32_t neighbour_label = labels->points[nn_indices[n]].label;
      if (neighbour_label == core::kUnlabeled)
        continue; // never propagate the unlabeled sentinel
      ++votes[neighbour_label];
    }

    if (votes.empty()) {
      // No labelled neighbour within the radius: leave unlabeled and count it.
      ++unpropagated;
      continue;
    }

    uint32_t best_label = core::kUnlabeled;
    int best_count = 0;
    for (const auto &[label, count] : votes) {
      // std::map iterates in ascending key order, so the first label reaching
      // the max wins ties -> smallest room id, deterministic.
      if (count > best_count) {
        best_count = count;
        best_label = label;
      }
    }
    labels->points[idx].label = best_label;
  }

  return unpropagated;
}

/**
 * @brief Implementation of room segmentation using community detection.
 *
 * Segments a point cloud into rooms using the Leiden community detection
 * algorithm based on visual relations between points. Uses uniform sampling
 * and Embree ray tracing to build a visibility graph.
 *
 * @param cloud Input point cloud.
 * @param normals Point cloud normals.
 * @param planes Labeled point cloud with plane IDs.
 * @param options Segmentation options (Leiden parameters, filter, etc.).
 * @return Labeled point cloud with room assignments.
 */
auto segment_rooms_impl(CloudConstPtr cloud, CloudNConstPtr normals,
                        CloudLConstPtr planes,
                        const SegmentRoomsOptions &options) -> CloudLPtr {

  if (options.cancel_token != nullptr && options.cancel_token->load()) {
    reusex::warn("segment_rooms: cancellation requested before execution.");
    throw std::runtime_error("Room segmentation cancelled.");
  }

  // Build set of filtered indices for fast lookup if filter is provided
  std::unordered_set<int> filtered_indices_set;
  if (options.filter) {
    filtered_indices_set.insert(options.filter->begin(), options.filter->end());
    reusex::debug("Room segmentation using {} filtered points",
                  options.filter->size());
  }

  std::unordered_map<int, IndicesPtr> plane_inlier_map;
  for (size_t i = 0; i < planes->points.size(); ++i) {
    // Skip if not in filter (when filter is provided)
    if (options.filter && filtered_indices_set.find(static_cast<int>(i)) ==
                              filtered_indices_set.end()) {
      continue;
    }

    const int label = planes->points[i].label;

    // Skip unlabeled points
    if (label < 1)
      continue;

    auto [it, inserted] = plane_inlier_map.try_emplace(label);
    if (inserted) {
      it->second = IndicesPtr(new Indices);
    }
    it->second->push_back(i);
  }

  IndicesPtr indices(new Indices);
  pcl::UniformSampling<PointT> us;
  us.setInputCloud(cloud);
  us.setRadiusSearch(options.grid_size);
  for (const auto &[key, idx] : plane_inlier_map) {
    us.setIndices(idx);
    IndicesPtr local_indices(new Indices);
    us.filter(*local_indices);
    indices->insert(indices->end(), local_indices->begin(),
                    local_indices->end());
  }

  pcl::CommunityClustering<PointT, NormalT, LabelT> cc;
  cc.setResolution(options.resolution);
  cc.setBeta(options.beta);
  cc.setMaxIterations(options.max_iter);

  cc.setGridSize(options.grid_size);
  cc.setMinClusterSize(min_room_samples(options));
  cc.setInputCloud(cloud);
  cc.setInputNormals(normals);
  cc.setIndices(indices);

  if (auto *obs = reusex::core::get_visual_observer()) {
    cc.registerVisualizationCallback(
        [obs](pcl::PointCloud<pcl::PointXYZ>::Ptr points,
              std::shared_ptr<std::vector<pcl::Vertices>> vertices,
              pcl::CorrespondencesPtr correspondences) {
          obs->viewer_add_visibility_graph(
              "visibility_graph", points, vertices, correspondences,
              reusex::core::Stage::room_segmentation);
        });
  }

  if (auto *obs = reusex::core::get_visual_observer()) {
    CloudPtr sampled(new Cloud);
    CloudLPtr sampled_planes(new CloudL);
    sampled->reserve(indices->size());
    sampled_planes->reserve(indices->size());
    for (const auto idx : *indices) {
      sampled->push_back(cloud->points[idx]);
      sampled_planes->push_back(planes->points[idx]);
    }
    obs->viewer_add_labeled_cloud("planes", sampled, sampled_planes,
                                  reusex::core::Stage::room_segmentation);
  }

  reusex::trace("Initialize labels and copy xyzrgb data to labels");
  CloudLPtr labels(new CloudL);
  pcl::copyPointCloud(*cloud, *labels);
  // Point-cloud convention (docs/STANDARDS.md §3.1): 0 = unlabeled, rooms
  // start at 1. Points not touched by clustering stay unlabeled.
  for (size_t i = 0; i < labels->points.size(); ++i)
    labels->points[i].label = core::kUnlabeled;

  cc.cluster(*labels);
  reusex::trace("Done clustering");

  // CommunityClustering assigns 0-based cluster ids to the sampled points.
  // Shift them to the 1-based room convention so that 0 remains "unlabeled".
  // A sample whose community was too small and that sees no room is not a
  // seed: it stays unlabeled and is propagated to like an unsampled point.
  {
    const std::size_t sampled = indices->size();
    IndicesPtr seeds(new Indices);
    seeds->reserve(indices->size());
    for (const int idx : *indices) {
      auto &label = labels->points[idx].label;
      if (label == decltype(cc)::kUnassigned) {
        label = core::kUnlabeled;
      } else {
        label += 1;
        seeds->push_back(idx);
      }
    }
    if (seeds->empty() && sampled > 0)
      reusex::warn("segment_rooms: no community of the {} sampled points "
                   "reaches min_room_area={:.2f} m^2 ({} samples at grid "
                   "{:.2f} m); every point stays unlabeled. Lower "
                   "--min-room-area for a scan this small.",
                   sampled, options.min_room_area, min_room_samples(options),
                   options.grid_size);
    indices = seeds;
  }

  if (auto *obs = reusex::core::get_visual_observer()) {
    CloudPtr sampled(new Cloud);
    CloudLPtr sampled_rooms(new CloudL);
    sampled->reserve(indices->size());
    sampled_rooms->reserve(indices->size());
    for (const auto idx : *indices) {
      sampled->push_back(cloud->points[idx]);
      sampled_rooms->push_back(labels->points[idx]);
    }
    obs->viewer_add_labeled_cloud("rooms", sampled, sampled_rooms,
                                  reusex::core::Stage::room_segmentation);
  }

  // Assign the label to all points
  IndicesPtr missing_indices(new Indices);

  std::sort(indices->begin(), indices->end());

  // When a filter is provided, propagate labels only to filtered points that
  // were not sampled. Points outside the filter must keep their initial
  // unlabeled (0) label so --filter actually narrows the labeled region.
  if (options.filter) {
    missing_indices->reserve(options.filter->size());
    IndicesPtr filter_sorted(new Indices(*options.filter));
    std::sort(filter_sorted->begin(), filter_sorted->end());
    size_t j = 0;
    for (int idx : *filter_sorted) {
      while (j < indices->size() && indices->at(j) < idx)
        ++j;
      if (j < indices->size() && indices->at(j) == idx)
        continue; // already labeled by clustering
      missing_indices->push_back(idx);
    }
    reusex::trace("Propagating to {} filtered points (cloud size: {}, "
                  "sampled: {}, filter: {})",
                  missing_indices->size(), cloud->points.size(),
                  indices->size(), options.filter->size());
  } else {
    missing_indices->reserve(cloud->points.size() - indices->size());
    int j = 0;
    for (int i = 0; i < static_cast<int>(cloud->size()); ++i) {
      if (j < static_cast<int>(indices->size()) && indices->at(j) == i) {
        ++j; // skip
      } else {
        missing_indices->push_back(i);
      }
    }
    reusex::trace("Propagating to {} missing points (cloud size: {}, "
                  "sampled: {})",
                  missing_indices->size(), cloud->points.size(),
                  indices->size());
  }

  // No seed at all (nothing reached min_room_area, warned above): there is
  // nothing to propagate, and a k-d tree over zero points cannot be searched.
  const size_t unpropagated =
      indices->empty()
          ? missing_indices->size()
          : propagate_room_labels(cloud, labels, indices, missing_indices,
                                  options.propagate_k,
                                  options.propagate_max_radius);

  if (!missing_indices->empty()) {
    const double pct = 100.0 * static_cast<double>(unpropagated) /
                       static_cast<double>(cloud->points.size());
    if (pct > 1.0) {
      reusex::warn("Room propagation: {} of {} points ({:.1f}%) had no room "
                   "label within {:.2f} m and remain unlabeled.",
                   unpropagated, cloud->points.size(), pct,
                   options.propagate_max_radius);
    } else if (unpropagated > 0) {
      reusex::debug("Room propagation: {} of {} points ({:.2f}%) remain "
                    "unlabeled (within tolerance).",
                    unpropagated, cloud->points.size(), pct);
    }
  }

  reusex::info("Number of clusters found: {}", cc.getNumClusters());

  return labels;
}

auto segment_rooms(CloudConstPtr cloud, CloudNConstPtr normals,
                   CloudLConstPtr planes, const SegmentRoomsOptions &options)
    -> CloudLPtr {
  return segment_rooms_impl(cloud, normals, planes, options);
}
} // namespace reusex::geometry
