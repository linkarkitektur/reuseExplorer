// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "io/rtabmap.hpp"
#include "core/ProjectDB.hpp"
#include "core/SensorIntrinsics.hpp"
#include "core/logging.hpp"
#include "core/pose_graph.hpp"
#include "core/processing_observer.hpp"
#include "io/rtabmap_links.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <rtabmap/core/DBDriver.h>
#include <rtabmap/core/Graph.h>
#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/util3d.h>

#include <algorithm>
#include <map>
#include <set>
#include <string>
#include <tuple>

static_assert(reusex::io::rtabmap_link_type::neighbor ==
              rtabmap::Link::kNeighbor);
static_assert(reusex::io::rtabmap_link_type::global_closure ==
              rtabmap::Link::kGlobalClosure);
static_assert(reusex::io::rtabmap_link_type::local_space_closure ==
              rtabmap::Link::kLocalSpaceClosure);
static_assert(reusex::io::rtabmap_link_type::local_time_closure ==
              rtabmap::Link::kLocalTimeClosure);
static_assert(reusex::io::rtabmap_link_type::user_closure ==
              rtabmap::Link::kUserClosure);
static_assert(reusex::io::rtabmap_link_type::virtual_closure ==
              rtabmap::Link::kVirtualClosure);
static_assert(reusex::io::rtabmap_link_type::neighbor_merged ==
              rtabmap::Link::kNeighborMerged);
static_assert(reusex::io::rtabmap_link_type::pose_prior ==
              rtabmap::Link::kPosePrior);
static_assert(reusex::io::rtabmap_link_type::landmark ==
              rtabmap::Link::kLandmark);
static_assert(reusex::io::rtabmap_link_type::gravity ==
              rtabmap::Link::kGravity);

namespace reusex::io {

void import_rtabmap(ProjectDB &db,
                    const std::filesystem::path &rtabmap_db_path) {
  core::info("Importing RTABMap database: {}", rtabmap_db_path);
  core::stopwatch timer;

  // Create a scan record and get the ID offset for this session.
  // id_offset = MAX(existing node_id), ensuring new IDs never collide with
  // previously imported sessions. FK-constrained tables (segmentation_images,
  // panoramic_images, glass_confidence_images) cannot hold references above
  // the offset because SQLite FK enforcement is on. pose_graph_edges has no
  // FK; this importer writes RTABMap links for its own (offset) id range only.
  auto scan = db.create_scan(rtabmap_db_path.string());
  core::info("Multi-session import: scan id={} id_offset={}", scan.id,
             scan.id_offset);

  // ── Initialise RTABMap and load the optimised graph ──────────────
  rtabmap::ParametersMap params;
  rtabmap::Rtabmap rtabmap;
  rtabmap.init(params, rtabmap_db_path.c_str());
  // NOTE: RTABMap's working directory is deliberately left unset (issue #245).
  // It is only ever used for optional side outputs — the LogF.txt/LogI.txt
  // statistics logs, gated on `Rtabmap/StatisticLogged` (default false), and a
  // debug-only global scan-map PCD dump from createGlobalScanMap(). This import
  // passes an empty ParametersMap, never calls process(), and getGraph() does
  // not touch it, so the previous setWorkingDirectory() call — which pointed
  // at the process working directory — wrote nothing; it only implied a
  // dependency we do not have. Do not reintroduce it: if statistics logging
  // (or the scan-map dump) is ever enabled here,
  // pass an explicit directory derived from the destination project instead.
  core::debug("RTABMap initialized in {:.3f}s", timer);

  timer.reset();
  std::map<int, rtabmap::Transform> poses;
  std::multimap<int, rtabmap::Link> links;
  std::map<int, rtabmap::Signature> nodes;
  rtabmap.getGraph(poses, links,
                   /*optimized=*/true, /*global=*/true, &nodes,
                   /*withImages=*/true, /*withScan=*/true,
                   /*withUserData=*/true, /*withGrid=*/true);
  core::debug("Graph loaded in {:.3f}s ({} nodes)", timer, poses.size());

  // ── Import each node ─────────────────────────────────────────────
  auto observer =
      core::ProgressObserver(core::Stage::importing_data, poses.size());

  int logId = db.log_pipeline_start(
      "import", fmt::format(R"({{"source":"{}"}})", rtabmap_db_path.string()));

  int imported_count = 0;
  std::map<int, std::array<double, 16>> imported; // RTABMap id -> stored pose
  try {
    for (const auto &[id, pose] : poses) {
      auto nodeIt = nodes.find(id);
      if (nodeIt == nodes.end()) {
        core::warn("Node {} present in poses but missing from signatures, "
                   "skipping",
                   id);
        ++observer;
        continue;
      }

      rtabmap::SensorData data = nodeIt->second.sensorData();
      data.uncompressData();

      // ── Extract raw sensor data ──────────────────────────────────
      cv::Mat color = data.imageRaw();
      cv::Mat depth = data.depthOrRightRaw();
      cv::Mat confidence = data.depthConfidenceRaw();

      if (color.empty()) {
        core::debug("Node {} has no color image, skipping", id);
        ++observer;
        continue;
      }

      // Store images in original RTABMap orientation (no rotation).
      // Any rotation for display should be handled at visualization time.

      // ── Camera intrinsics ────────────────────────────────────────
      core::SensorIntrinsics intrinsics;
      if (!data.cameraModels().empty()) {
        const auto &cm = data.cameraModels().front();
        // Store intrinsics as-is from RTABMap
        intrinsics.fx = cm.fx();
        intrinsics.fy = cm.fy();
        intrinsics.cx = cm.cx();
        intrinsics.cy = cm.cy();
        intrinsics.width = cm.imageWidth();
        intrinsics.height = cm.imageHeight();

        // Convert local transform to row-major array
        const auto &lt = cm.localTransform();
        if (lt.isNull()) {
          core::warn("Node {}: localTransform is null, using identity. "
                     "Per-frame orientation may be incorrect.",
                     id);
        }
        if (!lt.isNull()) {
          const float *d = lt.data();
          for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 4; ++c)
              intrinsics.local_transform[r * 4 + c] =
                  static_cast<double>(d[r * 4 + c]);
          intrinsics.local_transform[12] = 0;
          intrinsics.local_transform[13] = 0;
          intrinsics.local_transform[14] = 0;
          intrinsics.local_transform[15] = 1;
        }
      }

      // ── World pose as row-major 4x4 ─────────────────────────────
      // The RTABMap BASE-frame pose, verbatim (the camera's localTransform
      // lives in the intrinsics). RTABMap links are relative poses between
      // the same base frames, so they need no conversion either.
      std::array<double, 16> worldPose =
          pose.isNull() ? std::array<double, 16>{1, 0, 0, 0, 0, 1, 0, 0,
                                                 0, 0, 1, 0, 0, 0, 0, 1}
                        : rtabmap_transform_to_array(pose.data());

      // ── Timestamp ────────────────────────────────────────────────
      // Try SensorData stamp first, fall back to Signature stamp.
      // RTABMap returns 0.0 when no timestamp is set; treat as "unknown".
      double rawStamp = data.stamp();
      if (rawStamp <= 0.0)
        rawStamp = nodeIt->second.getStamp();
      double stamp = rawStamp > 0.0 ? rawStamp : -1.0;

      // ── Store in ProjectDB ───────────────────────────────────────
      db.save_sensor_frame(id + scan.id_offset, color, depth, confidence,
                           worldPose, intrinsics, stamp, scan.id);
      db.set_initial_pose(id + scan.id_offset, worldPose);
      imported.emplace(id, worldPose);

      core::trace("Imported node {} ({}x{}, depth {}x{})", id, color.cols,
                  color.rows, depth.cols, depth.rows);
      ++imported_count;
      ++observer;
    }

    // ── Links -> pose_graph_edges (spec §4.2) ─────────────────────
    int odometry = 0, loops = 0, duplicates = 0;
    std::map<RtabmapLinkSkip, int> skipped;
    std::set<std::tuple<int, int, std::string>> seen;
    {
      ProjectDB::Transaction tx(db);
      for (const auto &[key, l] : links) {
        RtabmapLink in;
        in.from = l.from();
        in.to = l.to();
        in.type = static_cast<int>(l.type());
        in.transform = l.transform().isNull()
                           ? std::array<double, 16>{}
                           : rtabmap_transform_to_array(l.transform().data());
        const cv::Mat &inf = l.infMatrix();
        if (inf.rows == 6 && inf.cols == 6 && inf.type() == CV_64FC1)
          for (int r = 0; r < 6; ++r)
            for (int c = 0; c < 6; ++c)
              in.information[static_cast<size_t>(r * 6 + c)] =
                  inf.at<double>(r, c);
        const auto conv = convert_rtabmap_link(in, scan.id_offset, imported);
        if (!conv.edge) {
          ++skipped[conv.skip];
          continue;
        }
        const auto &e = *conv.edge;
        if (!seen.emplace(std::min(e.from_node_id, e.to_node_id),
                          std::max(e.from_node_id, e.to_node_id), e.edge_type)
                 .second) {
          ++duplicates; // the same link listed in both directions
          continue;
        }
        db.upsert_pose_graph_edge(e);
        (e.edge_type == core::pose_graph::kTypeOdometry ? odometry : loops)++;
      }
      tx.commit();
    }
    core::info("RTABMap links: {} odometry + {} loop-closure edges stored "
               "in pose_graph_edges",
               odometry, loops);
    const int unsupported = skipped[RtabmapLinkSkip::unsupported_type];
    const int self = skipped[RtabmapLinkSkip::self_link];
    const int dangling = skipped[RtabmapLinkSkip::endpoint_not_imported];
    const int invalid = skipped[RtabmapLinkSkip::invalid_transform];
    if (unsupported + self + dangling + invalid + duplicates > 0)
      core::warn("RTABMap links skipped: {} unsupported type (virtual, "
                 "landmark, gravity, prior), {} self links, {} touching a "
                 "node that was not imported, {} with a null/non-rigid "
                 "transform, {} reverse-direction duplicates",
                 unsupported, self, dangling, invalid, duplicates);

    db.log_pipeline_end(logId, true);
  } catch (...) {
    db.log_pipeline_end(logId, false, "import_rtabmap failed");
    rtabmap.close();
    throw;
  }

  rtabmap.close();
  core::info("Import complete: scan {} ({} frames, id_offset={}), {} total "
             "sensor frames in project",
             scan.id, imported_count, scan.id_offset,
             db.sensor_frame_ids().size());
}

} // namespace reusex::io
