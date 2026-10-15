// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "segmentation/reconstruct.hpp"
#include "core/ProjectDB.hpp"
#include "core/SensorIntrinsics.hpp"
#include "core/annotate_class_map.hpp"
#include "core/label_semantics.hpp"
#include "core/logging.hpp"
#include "core/processing_observer.hpp"
#include "geometry/morton.hpp"
#include "geometry/transform_utils.hpp"
#include "segmentation/depth_filters.hpp"
#include "utils/fmt_formatter.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <fmt/format.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <numeric>

namespace reusex::geometry {

// to_affine() is shared via geometry/transform_utils.hpp.

void reconstruct_point_clouds(ProjectDB &db,
                              const ReconstructionParams &params) {
  core::info("Reconstructing point clouds from sensor frames");
  core::stopwatch timer;

  // Fail fast: --glass-filter requires pre-computed glass confidence images
  // (STANDARDS §5 — no silent no-op when the user explicitly requested it).
  if (params.glass_filter && db.glass_confidence_image_ids().empty()) {
    throw std::runtime_error(
        "glass-filter requires glass confidence images in the database. "
        "Run 'rux create annotate --glass-filter' first.");
  }

  auto frameIds = db.sensor_frame_ids();
  if (frameIds.empty()) {
    core::warn("No sensor frames in database, nothing to reconstruct");
    return;
  }

  core::info("Processing {} sensor frames", frameIds.size());

  // Accumulated clouds (serial accumulation; per-frame work dominates)
  auto merged_cloud = std::make_shared<Cloud>();
  auto merged_normals = std::make_shared<CloudN>();
  auto merged_labels = std::make_shared<CloudL>();

  auto observer =
      core::ProgressObserver(core::Stage::assembling_cloud, frameIds.size());

  // Skip counters, reported once after the loop (STANDARDS §5).
  std::size_t skipped_no_pose = 0;
  std::size_t total_glass_suppressed = 0; // depth pixels zeroed by glass filter

  for (int nodeId : frameIds) {
    // ── Read from ProjectDB ──────────────────────────────────────
    cv::Mat color = db.sensor_frame_image(nodeId);
    cv::Mat depth16 = db.sensor_frame_depth(nodeId);         // CV_16UC1 mm
    cv::Mat confidence = db.sensor_frame_confidence(nodeId); // CV_8UC1
    auto intr = db.sensor_frame_intrinsics(nodeId);

    if (color.empty() || depth16.empty()) {
      core::debug("Node {} missing color or depth, skipping", nodeId);
      ++observer;
      continue;
    }

    // A frame with no usable stored pose would be back-projected through
    // `sensor_frame_pose()`'s identity fallback: its points land at the world
    // origin in whatever orientation the depth image happened to have, merge
    // into the fused cloud, and are then indistinguishable from real geometry
    // for every stage downstream (#336). An all-zero transform is worse still
    // — it collapses the whole frame onto a single point.
    if (!db.has_sensor_frame_pose(nodeId)) {
      // Per-frame at debug: a scan imported without poses has thousands of
      // these, and the line that matters is the count reported after the loop.
      core::debug("Node {} has no usable stored pose, skipping", nodeId);
      ++skipped_no_pose;
      ++observer;
      continue;
    }
    auto pose = db.sensor_frame_pose(nodeId);

    // ── Dimension validation ──────────────────────────────────────
    if (intr.width <= 0 || intr.height <= 0) {
      core::warn("Node {}: intrinsics have invalid dimensions {}x{}, skipping",
                 nodeId, intr.width, intr.height);
      ++observer;
      continue;
    }

    if (!confidence.empty() &&
        (confidence.cols != depth16.cols || confidence.rows != depth16.rows)) {
      core::warn("Node {}: confidence {}x{} differs from depth {}x{}", nodeId,
                 confidence.cols, confidence.rows, depth16.cols, depth16.rows);
    }

    core::debug("Node {}: depth={}x{}, color={}x{}, intrinsics={}x{}", nodeId,
                depth16.cols, depth16.rows, color.cols, color.rows, intr.width,
                intr.height);
    if (depth16.cols != intr.width || depth16.rows != intr.height) {
      core::debug("Node {}: depth/intrinsics dimension mismatch — will scale "
                  "intrinsics to depth size",
                  nodeId);
    }

    // Convert depth to float meters for filtering
    cv::Mat depth_f;
    depth16.convertTo(depth_f, CV_32FC1, 1.0 / 1000.0);

    // ── Depth filters ────────────────────────────────────────────
    apply_depth_discontinuity_filter(depth_f, confidence, 0.5f);
    apply_ray_consistency_filter(depth_f, confidence, 0.2f);

    // ── Glass depth filter ────────────────────────────────────────
    if (params.glass_filter && db.has_glass_confidence_image(nodeId)) {
      cv::Mat glass_conf = db.glass_confidence_image(nodeId); // CV_8U
      if (!glass_conf.empty()) {
        // Resize to depth resolution if needed (glass map is at color res).
        if (glass_conf.size() != depth_f.size()) {
          cv::resize(glass_conf, glass_conf, depth_f.size(), 0, 0,
                     cv::INTER_NEAREST);
        }
        const auto thresh =
            static_cast<uint8_t>(params.glass_threshold * 255.0f);
        cv::Mat suppress_mask = glass_conf < thresh;
        // Count only pixels that have valid depth (would have been projected).
        total_glass_suppressed += static_cast<std::size_t>(
            cv::countNonZero(suppress_mask & (depth_f > 0.0f)));
        depth_f.setTo(0.0f, suppress_mask);
      }
    }

    // ── Build per-frame transforms ───────────────────────────────
    Eigen::Affine3f localTf = to_affine(intr.local_transform).cast<float>();
    Eigen::Affine3f worldTf = to_affine(pose).cast<float>();

    // Debug logging for first frame to verify transform correctness
    if (nodeId == frameIds[0]) {
      core::debug("=== First frame transform verification (nodeId={}) ===",
                  nodeId);
      core::debug("localTf matrix:\n{}", localTf.matrix().format(OctaveFmt));
      core::debug("worldTf matrix:\n{}", worldTf.matrix().format(OctaveFmt));
      core::debug("worldTf translation: [{:.3f}, {:.3f}, {:.3f}]",
                  worldTf.translation().x(), worldTf.translation().y(),
                  worldTf.translation().z());

      // Warn if localTf is approximately identity (likely wrong for
      // phone/tablet scanners)
      if (localTf.matrix().isApprox(Eigen::Matrix4f::Identity(), 1e-4f)) {
        core::warn("Node {}: localTransform is identity — per-frame "
                   "orientation may be incorrect if sensor has a non-trivial "
                   "camera-to-base transform",
                   nodeId);
      }
    }

    // ── Scale intrinsics to depth resolution ──────────────────────
    const double scale_x =
        static_cast<double>(depth_f.cols) / std::max(1, intr.width);
    const double scale_y =
        static_cast<double>(depth_f.rows) / std::max(1, intr.height);
    const float fx = static_cast<float>(intr.fx * scale_x);
    const float fy = static_cast<float>(intr.fy * scale_y);
    const float cx_cam = static_cast<float>(intr.cx * scale_x);
    const float cy_cam = static_cast<float>(intr.cy * scale_y);
    const int step = params.sampling_factor;
    const float min_d = params.min_distance;
    const float max_d = params.max_distance;
    const int conf_thresh = params.confidence_threshold;

    // ── Read segmentation labels if available ────────────────────
    cv::Mat seg_labels; // CV_32S or empty
    if (db.has_segmentation_image(nodeId))
      seg_labels = db.segmentation_image(nodeId); // CV_32S, -1 = bg

    // ── Pinhole back-projection ──────────────────────────────────
    auto frame_cloud = std::make_shared<Cloud>();
    auto frame_labels = std::make_shared<CloudL>();

    size_t estimated_pts =
        static_cast<size_t>((depth_f.rows / step) * (depth_f.cols / step));
    frame_cloud->reserve(estimated_pts);
    frame_labels->reserve(estimated_pts);

    for (int v = 0; v < depth_f.rows; v += step) {
      const float *depth_row = depth_f.ptr<float>(v);
      const uchar *conf_row =
          confidence.empty() ? nullptr : confidence.ptr<uchar>(v);

      for (int u = 0; u < depth_f.cols; u += step) {
        float z = depth_row[u];
        if (z <= 0.0f)
          continue;
        if (z < min_d || z > max_d)
          continue;
        if (conf_row && conf_row[u] < conf_thresh)
          continue;

        // Camera-frame 3D point in optical frame
        float x = (static_cast<float>(u) - cx_cam) * z / fx;
        float y = (static_cast<float>(v) - cy_cam) * z / fy;

        Eigen::Vector3f cam_pt(x, y, z);
        // Apply local transform (optical → sensor) then world transform (sensor
        // → world)
        Eigen::Vector3f world_pt = worldTf * (localTf * cam_pt);

        // Map depth pixel (v, u) to color coordinates (may differ in
        // resolution)
        int col_u = u * color.cols / depth_f.cols;
        int col_v = v * color.rows / depth_f.rows;
        col_u = std::clamp(col_u, 0, color.cols - 1);
        col_v = std::clamp(col_v, 0, color.rows - 1);
        const auto &pixel = color.at<cv::Vec3b>(col_v, col_u);

        PointT pt;
        pt.x = world_pt.x();
        pt.y = world_pt.y();
        pt.z = world_pt.z();
        pt.b = pixel[0];
        pt.g = pixel[1];
        pt.r = pixel[2];
        frame_cloud->push_back(pt);

        // Label (map depth coords to segmentation image coords). API CV_32S
        // (-1 = bg) is translated to point labels (0 = unlabeled).
        LabelT lbl;
        if (!seg_labels.empty()) {
          int su = u * seg_labels.cols / depth_f.cols;
          int sv = v * seg_labels.rows / depth_f.rows;
          su = std::clamp(su, 0, seg_labels.cols - 1);
          sv = std::clamp(sv, 0, seg_labels.rows - 1);
          lbl.label = core::api_to_point_label(seg_labels.at<int>(sv, su));
        } else
          lbl.label = core::kUnlabeled;
        frame_labels->push_back(lbl);
      }
    }

    if (frame_cloud->empty()) {
      ++observer;
      continue;
    }

    // ── Normal estimation ────────────────────────────────────────
    pcl::NormalEstimationOMP<PointT, NormalT> ne;
    ne.setInputCloud(frame_cloud);
    ne.setRadiusSearch(0.1);

    // Orient toward the camera's optical centre in world coordinates. These
    // per-frame orientations are what the voxel normals inherit below.
    const Eigen::Vector3f eye = (worldTf * localTf).translation();
    ne.setViewPoint(eye.x(), eye.y(), eye.z());

    auto frame_normals = std::make_shared<CloudN>();
    ne.compute(*frame_normals);

    // ── Accumulate ───────────────────────────────────────────────
    *merged_cloud += *frame_cloud;
    *merged_normals += *frame_normals;
    *merged_labels += *frame_labels;

    core::trace("Node {}: {} points", nodeId, frame_cloud->size());
    ++observer;
  }

  if (skipped_no_pose > 0)
    core::warn("Skipped {} of {} sensor frames for having no usable stored "
               "pose; their depth contributes nothing to the fused cloud. Run "
               "'rux optimize' or re-import with poses if this is unexpected.",
               skipped_no_pose, frameIds.size());

  if (total_glass_suppressed > 0)
    core::warn("Glass depth filter suppressed {} depth pixels across {} frames "
               "(threshold {:.2f}); these pixels were not added to the cloud.",
               total_glass_suppressed, frameIds.size(), params.glass_threshold);

  if (merged_cloud->empty()) {
    core::error("No points generated from any sensor frame ({} of {} frames "
                "skipped for having no usable stored pose)",
                skipped_no_pose, frameIds.size());
    return;
  }

  core::info("Accumulated {} points from {} frames", merged_cloud->size(),
             frameIds.size());

  // ── Voxelization ─────────────────────────────────────────────────
  core::info("Voxel grid filtering (resolution={}, {} points)",
             params.resolution, merged_cloud->size());
  timer.reset();

  pcl::VoxelGrid<PointT> vg;
  vg.setInputCloud(merged_cloud);
  vg.setLeafSize(params.resolution, params.resolution, params.resolution);
  auto ds_cloud = std::make_shared<Cloud>();
  vg.filter(*ds_cloud);

  core::debug("Downsampled to {} points in {:.3f}s", ds_cloud->size(), timer);

  // ── Remove NaN normals ───────────────────────────────────────────
  // Build KdTree on original cloud for label majority voting
  if (merged_cloud->size() < 10) {
    core::error("Merged cloud too small for KdTree ({} points)",
                merged_cloud->size());
    return;
  }

  pcl::KdTreeFLANN<PointT> kdtree;
  kdtree.setInputCloud(merged_cloud);

  // ── Normal estimation on downsampled cloud ───────────────────────
  // Re-estimate normals on the voxelized cloud for consistency
  pcl::NormalEstimationOMP<PointT, NormalT> ne_ds;
  ne_ds.setInputCloud(ds_cloud);
  ne_ds.setRadiusSearch(0.1);
  auto ds_normals = std::make_shared<CloudN>();
  ne_ds.compute(*ds_normals);

  // Orient the re-estimated normals toward the sensors that saw them. With no
  // viewpoint set, ne_ds flips every normal toward (0,0,0), so the two faces
  // of an interior wall share a sign whenever the origin lies on one side of
  // it, and downstream make_pairs / merge_planes cannot tell them apart
  // (reusex-0pi). The per-frame normals were each oriented toward their own
  // camera, so each voxel normal takes the sign the finite original normals
  // inside its voxel agree on (sum of dot products), falling back to the
  // kOrientFallbackK nearest originals when the voxel radius finds none.
  // Each iteration reads the shared kd-tree and writes only its own normal,
  // and the neighbour order is fixed by the tree, so the result does not
  // depend on thread scheduling (STANDARDS §6).
  {
    constexpr int kOrientFallbackK = 16;
    const float orient_radius = params.resolution / 2.0f;
    const auto n_ds = static_cast<std::ptrdiff_t>(ds_cloud->size());
    std::size_t flipped = 0;
    std::size_t unresolved = 0;

    // Sum of dot(nd, original normal) over the finite originals in `nn`;
    // returns false when none of them is finite.
    auto vote_over = [&](const Eigen::Vector3f &nd, const pcl::Indices &nn,
                         double &vote) {
      bool any = false;
      for (const auto idx : nn) {
        const auto &no = (*merged_normals)[idx];
        if (!std::isfinite(no.normal_x) || !std::isfinite(no.normal_y) ||
            !std::isfinite(no.normal_z))
          continue;
        vote += nd.dot(Eigen::Vector3f(no.normal_x, no.normal_y, no.normal_z));
        any = true;
      }
      return any;
    };

#pragma omp parallel for schedule(dynamic, 1024)                               \
    reduction(+ : flipped, unresolved)
    for (std::ptrdiff_t i = 0; i < n_ds; ++i) {
      auto &n = (*ds_normals)[i];
      if (!std::isfinite(n.normal_x) || !std::isfinite(n.normal_y) ||
          !std::isfinite(n.normal_z))
        continue; // dropped by the NaN filter below
      const Eigen::Vector3f nd(n.normal_x, n.normal_y, n.normal_z);

      pcl::Indices nn;
      std::vector<float> nn_d2;
      double vote = 0.0;
      kdtree.radiusSearch((*ds_cloud)[i], orient_radius, nn, nn_d2);
      bool any = vote_over(nd, nn, vote);
      if (!any && kdtree.nearestKSearch((*ds_cloud)[i], kOrientFallbackK, nn,
                                        nn_d2) > 0)
        any = vote_over(nd, nn, vote);
      if (!any) {
        ++unresolved;
        continue;
      }
      if (vote < 0.0) {
        n.normal_x = -n.normal_x;
        n.normal_y = -n.normal_y;
        n.normal_z = -n.normal_z;
        ++flipped;
      }
    }
    core::debug("Oriented voxel normals toward their sensors: {} of {} "
                "flipped, {} without a usable per-frame normal",
                flipped, ds_cloud->size(), unresolved);
    if (unresolved > 0)
      core::warn("{} of {} voxel normals had no finite per-frame normal "
                 "among their {} nearest source points and keep PCL's "
                 "toward-origin orientation",
                 unresolved, ds_cloud->size(), kOrientFallbackK);
  }

  // Remove NaN normals
  pcl::Indices nan_filter;
  pcl::removeNaNNormalsFromPointCloud(*ds_normals, *ds_normals, nan_filter);

  // Apply same filter to cloud
  auto ds_cloud_filtered = std::make_shared<Cloud>();
  pcl::ExtractIndices<PointT> extract_nan;
  extract_nan.setInputCloud(ds_cloud);
  auto nan_indices = std::make_shared<pcl::Indices>(nan_filter);
  extract_nan.setIndices(nan_indices);
  extract_nan.filter(*ds_cloud_filtered);
  ds_cloud = ds_cloud_filtered;

  // ── Label majority voting ────────────────────────────────────────
  core::info("Label majority voting");
  auto ds_labels = std::make_shared<CloudL>();
  ds_labels->resize(ds_cloud->size());

  const float half_res = params.resolution / 2.0f;
  for (size_t i = 0; i < ds_cloud->size(); ++i) {
    std::vector<int> indices;
    std::vector<float> distances;
    kdtree.radiusSearch((*ds_cloud)[i], half_res, indices, distances);

    // One vote per merged point; majority_label() ignores kUnlabeled and
    // breaks a tie towards the lowest label, independent of the order the
    // KdTree returns neighbours in (STANDARDS §6).
    std::vector<std::pair<uint32_t, int>> votes;
    votes.reserve(indices.size());
    for (int idx : indices)
      votes.emplace_back((*merged_labels)[idx].label, 1);
    (*ds_labels)[i].label = core::majority_label(votes);
  }

  // ── Outlier removal ──────────────────────────────────────────────
  core::info("Filtering outliers ({} points)", ds_cloud->size());
  timer.reset();

  // Statistical outlier removal
  pcl::StatisticalOutlierRemoval<PointT> sor;
  sor.setInputCloud(ds_cloud);
  sor.setMeanK(50);
  sor.setStddevMulThresh(1.0);
  auto stat_inliers = std::make_shared<pcl::Indices>();
  sor.filter(*stat_inliers);
  core::debug("SOR kept {}/{}", stat_inliers->size(), ds_cloud->size());

  // Radius outlier removal
  pcl::RadiusOutlierRemoval<PointT> ror;
  ror.setInputCloud(ds_cloud);
  ror.setIndices(stat_inliers);
  ror.setRadiusSearch(params.resolution * 2.0);
  ror.setMinNeighborsInRadius(5);
  auto filtered_indices = std::make_shared<pcl::Indices>();
  ror.filter(*filtered_indices);
  core::debug("ROR kept {}/{}", filtered_indices->size(), stat_inliers->size());

  // Extract filtered results
  auto out_cloud = std::make_shared<Cloud>();
  pcl::ExtractIndices<PointT> ext_pts;
  ext_pts.setInputCloud(ds_cloud);
  ext_pts.setIndices(filtered_indices);
  ext_pts.filter(*out_cloud);

  auto out_normals = std::make_shared<CloudN>();
  pcl::ExtractIndices<NormalT> ext_nrm;
  ext_nrm.setInputCloud(ds_normals);
  ext_nrm.setIndices(filtered_indices);
  ext_nrm.filter(*out_normals);

  auto out_labels = std::make_shared<CloudL>();
  pcl::ExtractIndices<LabelT> ext_lbl;
  ext_lbl.setInputCloud(ds_labels);
  ext_lbl.setIndices(filtered_indices);
  ext_lbl.filter(*out_labels);

  core::info("Filtering complete in {:.3f}s: {}/{} points ({:.1f}%)", timer,
             out_cloud->size(), ds_cloud->size(),
             100.0 * out_cloud->size() / ds_cloud->size());

  // ── Morton reorder ────────────────────────────────────────────────────────
  // Sort cloud, normals, and labels by the BIT-REVERSED 30-bit Morton code
  // (10 bits per axis) so that any stored prefix is a spatially stratified
  // sample that covers all octants before refining any single one.
  //
  // Plain ascending Morton order is z-major: the first k% of points all fall
  // in the lowest-code spatial octant (x<mid ∧ y<mid ∧ z<mid), not the
  // whole scene.  Bit-reversing the 30-bit code promotes the coarsest-level
  // octant bits to the most-significant position of the sort key, so the sort
  // visits ONE representative from each of the 8 top-level octants, then ONE
  // from each of the 64 sub-octants, etc. — breadth-first spatial coverage.
  // Any prefix then gives a uniform spatial sample rather than a corner.
  //
  // Same input → same order: codes are deterministic over the bbox, and
  // std::stable_sort breaks ties by storage index (STANDARDS §6).
  //
  // The same permutation is applied to out_normals and out_labels so the three
  // arrays remain index-aligned (CONTRACTS.md, STANDARDS §3.2).
  if (out_cloud->size() > 1) {
    const size_t n = out_cloud->size();

    // Bounding box over all points.
    Eigen::Vector3f lo(std::numeric_limits<float>::max(),
                       std::numeric_limits<float>::max(),
                       std::numeric_limits<float>::max());
    Eigen::Vector3f hi(-std::numeric_limits<float>::max(),
                       -std::numeric_limits<float>::max(),
                       -std::numeric_limits<float>::max());
    for (const auto &p : *out_cloud) {
      lo = lo.cwiseMin(Eigen::Vector3f(p.x, p.y, p.z));
      hi = hi.cwiseMax(Eigen::Vector3f(p.x, p.y, p.z));
    }
    // Avoid division by zero for degenerate (flat/single-point) clouds.
    Eigen::Vector3f range = (hi - lo).cwiseMax(1e-9f);

    // Precompute bit-reversed Morton codes so each is evaluated once, not
    // twice per comparison.  Shared helpers from geometry/morton.hpp.
    namespace gm = reusex::geometry;
    constexpr uint32_t kMax10 = 1023u;
    std::vector<uint32_t> codes(n);
    for (size_t i = 0; i < n; ++i) {
      const auto &p = (*out_cloud)[i];
      const uint32_t xi = static_cast<uint32_t>(
          std::clamp((p.x - lo.x()) / range.x(), 0.0f, 1.0f) * kMax10);
      const uint32_t yi = static_cast<uint32_t>(
          std::clamp((p.y - lo.y()) / range.y(), 0.0f, 1.0f) * kMax10);
      const uint32_t zi = static_cast<uint32_t>(
          std::clamp((p.z - lo.z()) / range.z(), 0.0f, 1.0f) * kMax10);
      codes[i] = gm::morton_reverse_bits30(gm::morton_expand3(xi) |
                                           (gm::morton_expand3(yi) << 1u) |
                                           (gm::morton_expand3(zi) << 2u));
    }

    std::vector<size_t> perm(n);
    std::iota(perm.begin(), perm.end(), size_t{0});
    std::stable_sort(perm.begin(), perm.end(),
                     [&](size_t a, size_t b) { return codes[a] < codes[b]; });

    // Apply the same permutation to all three index-aligned arrays.
    auto apply_perm = [&perm, n](auto &cloud_ptr) {
      using C = std::remove_reference_t<decltype(*cloud_ptr)>;
      auto sorted = std::make_shared<C>();
      sorted->width = static_cast<uint32_t>(n);
      sorted->height = 1;
      sorted->is_dense = cloud_ptr->is_dense;
      sorted->points.resize(n);
      for (size_t i = 0; i < n; ++i)
        sorted->points[i] = cloud_ptr->points[perm[i]];
      cloud_ptr = std::move(sorted);
    };
    apply_perm(out_cloud);
    apply_perm(out_normals);
    apply_perm(out_labels);

    core::debug("Morton reorder complete ({} points)", n);
  }

  // ── Save to ProjectDB ────────────────────────────────────────────
  core::info("Saving point clouds to database");
  std::string paramsJson = fmt::format(
      R"({{"resolution":{},"min_distance":{},"max_distance":{},"sampling_factor":{},"confidence_threshold":{},"storage_order":"morton_10bit_bitrev"}})",
      params.resolution, params.min_distance, params.max_distance,
      params.sampling_factor, params.confidence_threshold);

  if (!out_cloud->empty())
    db.save_point_cloud("cloud", *out_cloud, "reconstruct", paramsJson);

  if (!out_normals->empty())
    db.save_point_cloud("normals", *out_normals, "reconstruct", paramsJson);

  if (!out_labels->empty()) {
    // The semantic cloud records its encoding (point label = API class + 1,
    // STANDARDS §3.1) so the schema v29 migration never shifts it twice.
    auto labelParams = nlohmann::json::parse(paramsJson);
    labelParams[core::kLabelEncodingKey] = core::kLabelEncodingApiPlusOne;
    db.save_point_cloud("labels", *out_labels, "reconstruct",
                        labelParams.dump());

    // Class names of the latest annotate run, keyed by point label.
    if (auto definitions = core::annotated_label_definitions(db);
        !definitions.empty())
      db.save_label_definitions("labels", definitions);
  }

  core::info("Reconstruction complete: {} points, {} normals, {} labels",
             out_cloud->size(), out_normals->size(), out_labels->size());
}

} // namespace reusex::geometry
