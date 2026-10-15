// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// See PanoramaAlignment.hpp for the design. Front-end (ORB + Lowe ratio + depth
// lift) mirrors slam/LoopClosure.cpp; the panorama pose is resected per slice
// with cv::solvePnPRansac and refined over all slices' inliers in
// panorama-bearing space with a small Gauss-Newton.

#include "slam/PanoramaAlignment.hpp"

#include "core/ProjectDB.hpp"
#include "core/SensorIntrinsics.hpp"
#include "core/logging.hpp"
#include "geometry/EquirectProjection.hpp"
#include "panorama_features.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <Eigen/Geometry>

#include <fmt/format.h>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <limits>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace reusex::geometry {

namespace {

// Pose/rotation helpers, the frame feature extractor and the bearing-space
// Gauss-Newton refinement now live in panorama_features.hpp, shared with the
// panorama loop-edge front-end (issue #236).
using pano_detail::bearing_angle;
using pano_detail::from_matrix4;
using pano_detail::to_matrix4;

// ORB features of a frame with world-space 3D points. The extraction itself is
// shared (pano_detail::extract_frame_features, which lifts to the frame's OWN
// optical coordinates); alignment resects a single pooled pose in WORLD, so the
// points are composed with the frame's seed pose here.
struct FrameWorldFeatures {
  pano_detail::FrameFeatures f;
  std::vector<Eigen::Vector3d> world;

  const cv::Mat &descriptors() const { return f.descriptors; }
  bool valid(int k) const { return f.valid[k] != 0; }
};

FrameWorldFeatures extract_frame(ProjectDB &db, int node_id,
                                 const PanoramaAlignmentOptions &opt,
                                 const cv::Ptr<cv::ORB> &orb) {
  FrameWorldFeatures out;
  out.f = pano_detail::extract_frame_features(db, node_id, orb, opt.min_depth,
                                              opt.max_depth);
  out.world.resize(out.f.local.size());
  for (size_t k = 0; k < out.f.local.size(); ++k)
    if (out.f.valid[k])
      out.world[k] = out.f.world(k);
  return out;
}

// One 2D(slice pixel)-3D(world) + bearing correspondence.
struct Corr {
  cv::Point2f slice_px;
  Eigen::Vector3d world;
  Eigen::Vector3d bearing; ///< unit bearing of slice_px in the panorama frame
  int slice_idx = -1;      ///< which panorama slice
  int frame_node = -1;     ///< which sensor frame supplied the 3D point
  int frame_idx = -1;      ///< index into the candidate list
  cv::Point2f frame_px;    ///< matched pixel in that sensor frame
  float hamming = 0.0f;    ///< descriptor distance of the match
};

constexpr double kRadToDeg = 180.0 / M_PI;

// One debug line per panorama summarising the matching funnel, plus one trace
// line per slice and per frame. This is what answers "why did this panorama
// keep its timestamp placement" without a debugger (reusex-bx2.1).
void log_diagnostics(const PanoramaAlignmentResult &r) {
  if (!core::should_log(core::LogLevel::debug))
    return;
  const auto &d = r.diagnostics;
  int slice_kp = 0, ratio = 0, frame_kp = 0, frame_depth = 0;
  for (const auto &s : d.slices) {
    slice_kp += s.keypoints;
    ratio += s.ratio_matches;
  }
  for (const auto &f : d.frames) {
    frame_kp += f.keypoints;
    frame_depth += f.keypoints_with_depth;
  }
  core::debug("PanoramaAlignment: panorama {} -> {} | {} frames ({} kp, {} "
              "with depth) | {} slices ({} kp) | ratio-pass {} -> "
              "depth-backed {} | best slice {} ({} PnP inliers) | pooled {} "
              "-> final {} over {} frames / {} slices | rms {:.3f} deg | "
              "delta {:.2f} m",
              r.pano_id, to_string(r.outcome), d.frames.size(), frame_kp,
              frame_depth, d.slices.size(), slice_kp, ratio,
              d.correspondences_total, d.best_slice, d.best_slice_inliers,
              d.initial_inliers, d.final_inliers, d.inlier_frames,
              d.inlier_slices, d.rms_deg, d.delta_from_seed_m);
  if (!core::should_log(core::LogLevel::trace))
    return;
  for (size_t i = 0; i < d.slices.size(); ++i) {
    const auto &s = d.slices[i];
    core::trace("PanoramaAlignment:   slice {:2d} yaw {:6.1f} pitch {:5.1f}: "
                "{:5d} kp, {:4d} ratio-pass, {:4d} depth-backed, PnP {:4d}, "
                "final {:4d}",
                i, s.yaw_deg, s.pitch_deg, s.keypoints, s.ratio_matches,
                s.correspondences, s.pnp_inliers, s.final_inliers);
  }
  for (const auto &f : d.frames)
    core::trace("PanoramaAlignment:   frame {:6d} dt {:+7.2f} s dist {:5.2f} "
                "m: {:5d} kp ({:5d} with depth), {:4d} ratio-pass, {:4d} "
                "depth-backed, final {:4d}",
                f.node_id, f.dt_from_seed_s, f.dist_from_seed_m, f.keypoints,
                f.keypoints_with_depth, f.ratio_matches, f.correspondences,
                f.final_inliers);
}

// Draw an ORB-correspondence figure: the panorama slice (left) beside the
// best-matching sensor frame (right), inlier matches connected by lines. This
// is the literal "360 aligned with the scan image" visual.
void draw_correspondence_figure(ProjectDB &db, const cv::Mat &slice_img,
                                int frame_node,
                                const std::vector<const Corr *> &inliers,
                                const std::string &path) {
  cv::Mat frame = db.sensor_frame_image(frame_node);
  if (frame.empty() || slice_img.empty())
    return;
  cv::Mat lhs = slice_img.clone();
  cv::Mat rhs;
  if (frame.channels() == 1)
    cv::cvtColor(frame, rhs, cv::COLOR_GRAY2BGR);
  else
    rhs = frame.clone();
  // match heights for a clean side-by-side
  const int h = lhs.rows;
  const double s = static_cast<double>(h) / rhs.rows;
  cv::resize(rhs, rhs, cv::Size(), s, s);
  cv::Mat canvas;
  cv::hconcat(std::vector<cv::Mat>{lhs, rhs}, canvas);

  cv::RNG rng(12345);
  for (const Corr *c : inliers) {
    const cv::Point p0(cv::saturate_cast<int>(c->slice_px.x),
                       cv::saturate_cast<int>(c->slice_px.y));
    const cv::Point p1(cv::saturate_cast<int>(c->frame_px.x * s) + lhs.cols,
                       cv::saturate_cast<int>(c->frame_px.y * s));
    const cv::Scalar col(rng.uniform(0, 255), rng.uniform(0, 255),
                         rng.uniform(0, 255));
    cv::circle(canvas, p0, 3, col, 1, cv::LINE_AA);
    cv::circle(canvas, p1, 3, col, 1, cv::LINE_AA);
    cv::line(canvas, p0, p1, col, 1, cv::LINE_AA);
  }
  cv::putText(canvas,
              "panorama slice            |            scan frame  (" +
                  std::to_string(inliers.size()) + " inlier matches)",
              cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, 0.7,
              cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
  std::filesystem::create_directories(
      std::filesystem::path(path).parent_path());
  cv::imwrite(path, canvas);
}

} // namespace

const char *to_string(PanoramaAlignmentOutcome o) {
  switch (o) {
  case PanoramaAlignmentOutcome::aligned:
    return "aligned";
  case PanoramaAlignmentOutcome::no_image:
    return "no_image";
  case PanoramaAlignmentOutcome::seed_not_found:
    return "seed_not_found";
  case PanoramaAlignmentOutcome::no_slice_pnp:
    return "no_slice_pnp";
  case PanoramaAlignmentOutcome::below_noise_floor:
    return "below_noise_floor";
  case PanoramaAlignmentOutcome::upside_down:
    return "upside_down";
  case PanoramaAlignmentOutcome::too_few_pooled_inliers:
    return "too_few_pooled_inliers";
  case PanoramaAlignmentOutcome::refinement_collapsed:
    return "refinement_collapsed";
  case PanoramaAlignmentOutcome::correction_gate:
    return "correction_gate";
  case PanoramaAlignmentOutcome::no_seed_pose:
    return "no_seed_pose";
  }
  return "unknown";
}

std::string
panorama_alignment_diagnostics_json(const PanoramaAlignmentResult &r) {
  using nlohmann::json;
  const auto &d = r.diagnostics;
  // NaN is not JSON; unknown quantities are written as null.
  auto num = [](double v) -> json {
    return std::isfinite(v) ? json(v) : json(nullptr);
  };
  json j;
  j["pano_id"] = r.pano_id;
  j["seed_node"] = r.seed_node;
  j["outcome"] = to_string(r.outcome);
  j["aligned"] = r.aligned;
  j["translation_fixed"] = r.translation_fixed;
  j["correspondences_total"] = d.correspondences_total;
  j["sphere_keypoints"] = d.sphere_keypoints;
  j["dropped_overlap"] = d.dropped_overlap;
  j["dropped_pole"] = d.dropped_pole;
  j["dropped_repeats"] = d.dropped_repeats;
  j["best_slice"] = d.best_slice;
  j["best_slice_inliers"] = d.best_slice_inliers;
  j["null_best_slice_inliers"] = d.null_best_slice_inliers;
  j["noise_floor_threshold"] = d.noise_floor_threshold;
  j["max_correction_m"] = d.max_correction_m;
  j["down_dot"] = num(d.down_dot);
  j["initial_inliers"] = d.initial_inliers;
  j["final_inliers"] = d.final_inliers;
  j["rms_deg"] = num(d.rms_deg);
  j["delta_from_seed_m"] = num(d.delta_from_seed_m);
  j["inlier_frames"] = d.inlier_frames;
  j["inlier_slices"] = d.inlier_slices;
  if (r.aligned) // panorama-optical->world, row-major 4x4
    j["pose"] = r.pose;
  json slices = json::array();
  for (const auto &s : d.slices)
    slices.push_back({{"yaw_deg", s.yaw_deg},
                      {"pitch_deg", s.pitch_deg},
                      {"keypoints", s.keypoints},
                      {"pooled_keypoints", s.pooled_keypoints},
                      {"ratio_matches", s.ratio_matches},
                      {"correspondences", s.correspondences},
                      {"pnp_inliers", s.pnp_inliers},
                      {"final_inliers", s.final_inliers}});
  j["slices"] = std::move(slices);
  json frames = json::array();
  for (const auto &f : d.frames)
    frames.push_back({{"node_id", f.node_id},
                      {"dt_from_seed_s", num(f.dt_from_seed_s)},
                      {"dist_from_seed_m", num(f.dist_from_seed_m)},
                      {"keypoints", f.keypoints},
                      {"keypoints_with_depth", f.keypoints_with_depth},
                      {"ratio_matches", f.ratio_matches},
                      {"correspondences", f.correspondences},
                      {"final_inliers", f.final_inliers}});
  j["frames"] = std::move(frames);
  if (!d.correspondences.empty()) {
    json corr = json::array();
    for (const auto &c : d.correspondences)
      corr.push_back({c.slice, c.frame_node, c.azimuth_deg, c.elevation_deg,
                      c.slice_x, c.slice_y, c.frame_x, c.frame_y, c.hamming,
                      c.final_inlier});
    j["correspondence_columns"] = {"slice",         "frame_node", "azimuth_deg",
                                   "elevation_deg", "slice_x",    "slice_y",
                                   "frame_x",       "frame_y",    "hamming",
                                   "final_inlier"};
    j["correspondences"] = std::move(corr);
  }
  return j.dump();
}

std::string panorama_alignment_summary(const PanoramaAlignmentResult &r) {
  const auto &d = r.diagnostics;
  const std::string name = to_string(r.outcome);
  switch (r.outcome) {
  case PanoramaAlignmentOutcome::aligned:
    return fmt::format("aligned: {} inliers, {:.3f} deg RMS, centre {:.2f} m "
                       "from seed (best slice {} PnP inliers, noise floor {})",
                       r.inliers, r.rms_deg, r.delta_from_seed_m,
                       d.best_slice_inliers, d.noise_floor_threshold);
  case PanoramaAlignmentOutcome::no_slice_pnp:
    return fmt::format("{}: {} depth-backed correspondences, best slice {} PnP "
                       "inliers",
                       name, d.correspondences_total, d.best_slice_inliers);
  case PanoramaAlignmentOutcome::below_noise_floor:
    return fmt::format("{}: best slice {} PnP inliers, needs >= {} (shuffled "
                       "null {}; {} depth-backed correspondences)",
                       name, d.best_slice_inliers, d.noise_floor_threshold,
                       d.null_best_slice_inliers, d.correspondences_total);
  case PanoramaAlignmentOutcome::upside_down:
    return fmt::format("{}: PnP seed down . seed-frame down = {:.2f} < 0 (best "
                       "slice {} PnP inliers, noise floor {})",
                       name, d.down_dot, d.best_slice_inliers,
                       d.noise_floor_threshold);
  case PanoramaAlignmentOutcome::too_few_pooled_inliers:
  case PanoramaAlignmentOutcome::refinement_collapsed:
    return fmt::format("{}: pooled {} -> final {} inliers (best slice {} PnP "
                       "inliers)",
                       name, d.initial_inliers, d.final_inliers,
                       d.best_slice_inliers);
  case PanoramaAlignmentOutcome::correction_gate:
    return fmt::format("{}: centre {:.2f} m from seed > {:.2f} m ({} inliers, "
                       "{:.3f} deg RMS, best slice {} PnP inliers)",
                       name, d.delta_from_seed_m, d.max_correction_m,
                       d.final_inliers, d.rms_deg, d.best_slice_inliers);
  case PanoramaAlignmentOutcome::no_image:
  case PanoramaAlignmentOutcome::seed_not_found:
  case PanoramaAlignmentOutcome::no_seed_pose:
    break;
  }
  return fmt::format("{} (seed node {})", name, r.seed_node);
}

PanoramaAlignmentResult align_panorama(ProjectDB &db, int pano_id,
                                       int seed_node_id,
                                       const PanoramaAlignmentOptions &opt) {
  PanoramaAlignmentResult res;
  res.pano_id = pano_id;
  res.seed_node = seed_node_id;
  auto &diag = res.diagnostics;
  // Every exit goes through here so the outcome is always named and logged.
  auto finish = [&res](PanoramaAlignmentOutcome o) -> PanoramaAlignmentResult {
    res.outcome = o;
    res.aligned = (o == PanoramaAlignmentOutcome::aligned);
    log_diagnostics(res);
    return res;
  };

  cv::setRNGSeed(static_cast<int>(opt.seed)); // deterministic solvePnPRansac

  cv::Mat equirect = db.panoramic_image(pano_id);
  if (equirect.empty()) {
    core::warn("PanoramaAlignment: panorama {} has no image", pano_id);
    return finish(PanoramaAlignmentOutcome::no_image);
  }

  // --- candidate sensor frames: window around the seed in id order ----------
  std::vector<int> all_ids = db.sensor_frame_ids();
  std::sort(all_ids.begin(), all_ids.end());
  auto seed_it = std::find(all_ids.begin(), all_ids.end(), seed_node_id);
  if (seed_it == all_ids.end()) {
    core::warn("PanoramaAlignment: seed node {} not found for panorama {}",
               seed_node_id, pano_id);
    return finish(PanoramaAlignmentOutcome::seed_not_found);
  }
  const int seed_idx = static_cast<int>(seed_it - all_ids.begin());
  // order candidates by |offset| so the nearest frames are matched first
  std::vector<int> cand;
  for (int d = 0; d <= opt.candidate_window; ++d) {
    for (int s : (d == 0 ? std::vector<int>{0} : std::vector<int>{-d, +d})) {
      const int idx = seed_idx + s;
      if (idx >= 0 && idx < static_cast<int>(all_ids.size()))
        cand.push_back(all_ids[idx]);
      if (static_cast<int>(cand.size()) >= opt.max_candidates)
        break;
    }
    if (static_cast<int>(cand.size()) >= opt.max_candidates)
      break;
  }

  auto orb = cv::ORB::create(opt.max_features);

  // frame features (world 3D)
  std::vector<FrameWorldFeatures> frames;
  frames.reserve(cand.size());
  for (int nid : cand)
    frames.push_back(extract_frame(db, nid, opt, orb));

  const bool seed_has_pose = db.has_sensor_frame_pose(seed_node_id);
  Eigen::Vector3d seed_centre = Eigen::Vector3d::Zero();
  if (seed_has_pose)
    seed_centre =
        to_matrix4(db.sensor_frame_pose(seed_node_id)).block<3, 1>(0, 3);
  const double seed_ts = db.sensor_frame_timestamp(seed_node_id);
  diag.frames.resize(cand.size());
  for (size_t fi = 0; fi < cand.size(); ++fi) {
    auto &fs = diag.frames[fi];
    const auto &fr = frames[fi];
    fs.node_id = cand[fi];
    const double ts = db.sensor_frame_timestamp(cand[fi]);
    fs.dt_from_seed_s = (ts >= 0.0 && seed_ts >= 0.0)
                            ? ts - seed_ts
                            : std::numeric_limits<double>::quiet_NaN();
    if (seed_has_pose && !fr.descriptors().empty())
      fs.dist_from_seed_m =
          (fr.f.T_world_cam.block<3, 1>(0, 3) - seed_centre).norm();
    fs.keypoints = static_cast<int>(fr.f.keypoints.size());
    fs.keypoints_with_depth = static_cast<int>(
        std::count(fr.f.valid.begin(), fr.f.valid.end(), char{1}));
  }

  // panorama slices
  const std::vector<PerspectiveView> slices =
      overlapping_views(equirect, opt.n_yaw, opt.fov_deg, opt.slice);
  diag.slices.resize(slices.size());
  for (size_t si = 0; si < slices.size(); ++si) {
    diag.slices[si].yaw_deg = slices[si].yaw_deg;
    diag.slices[si].pitch_deg = slices[si].pitch_deg;
  }

  // --- the panorama's keypoints over the whole sphere (reusex-bx2.5) -------
  std::vector<pano_detail::SliceKeypoints> slice_kps(slices.size());
  for (size_t si = 0; si < slices.size(); ++si) {
    const auto &view = slices[si];
    auto &sk = slice_kps[si];
    sk.K = view.K;
    sk.R_pano_from_view = view.R_pano_from_view;
    sk.size = view.image.size();
    cv::Mat sgray;
    cv::cvtColor(view.image, sgray, cv::COLOR_BGR2GRAY);
    std::vector<cv::KeyPoint> skps;
    orb->detectAndCompute(sgray, cv::noArray(), skps, sk.descriptors);
    diag.slices[si].keypoints = static_cast<int>(skps.size());
    if (sk.descriptors.empty())
      continue;
    sk.pixels.reserve(skps.size());
    for (const auto &kp : skps)
      sk.pixels.push_back(kp.pt);
  }
  pano_detail::SpherePoolOptions sopt;
  sopt.dedupe_overlap = opt.dedupe_overlap;
  sopt.nadir_mask_deg = opt.nadir_mask_deg;
  const pano_detail::SphereKeypoints sphere =
      pano_detail::pool_sphere_keypoints(slice_kps, sopt);
  diag.sphere_keypoints = static_cast<int>(sphere.bearings.size());
  diag.dropped_overlap = sphere.dropped_overlap;
  diag.dropped_pole = sphere.dropped_pole;
  for (int k = 0; k < static_cast<int>(sphere.slice.size()); ++k)
    ++diag.slices[sphere.slice[k]].pooled_keypoints;

  // --- match every candidate frame against the sphere ------------------------
  struct Match {
    int frame_idx;
    pano_detail::SphereMatch m;
  };
  std::vector<Match> matches;
  for (size_t fi = 0; fi < frames.size(); ++fi) {
    const auto &fr = frames[fi];
    if (fr.descriptors().empty())
      continue;
    const std::vector<pano_detail::SphereMatch> fm =
        opt.frame_to_sphere
            ? pano_detail::match_frame_to_sphere(
                  fr.descriptors(), sphere, opt.ratio_test, opt.distinct_deg)
            : pano_detail::match_sphere_to_frame(fr.descriptors(), sphere,
                                                 opt.ratio_test);
    for (const auto &m : fm) {
      ++diag.slices[sphere.slice[m.sphere_kp]].ratio_matches;
      ++diag.frames[fi].ratio_matches;
      if (fr.valid(m.frame_kp))
        matches.push_back({static_cast<int>(fi), m});
    }
  }
  // One bearing constraint per panorama keypoint, however many near-identical
  // frames it matched (the lowest Hamming distance wins).
  if (opt.one_per_keypoint) {
    std::vector<int> kp(matches.size());
    std::vector<float> ham(matches.size());
    for (size_t i = 0; i < matches.size(); ++i) {
      kp[i] = matches[i].m.sphere_kp;
      ham[i] = matches[i].m.hamming;
    }
    std::vector<Match> kept;
    for (int i : pano_detail::one_per_sphere_keypoint(kp, ham))
      kept.push_back(matches[i]);
    diag.dropped_repeats = static_cast<int>(matches.size() - kept.size());
    matches = std::move(kept);
  }
  // Slice-major, then frame, then keypoint: the order the per-slice PnP and
  // its RANSAC see (and the order the original slice-by-slice loop produced).
  std::sort(matches.begin(), matches.end(),
            [&](const Match &a, const Match &b) {
              const int sa = sphere.slice[a.m.sphere_kp],
                        sb = sphere.slice[b.m.sphere_kp];
              if (sa != sb)
                return sa < sb;
              if (a.frame_idx != b.frame_idx)
                return a.frame_idx < b.frame_idx;
              return a.m.sphere_kp < b.m.sphere_kp;
            });

  // Per-slice correspondences (kept separate so each slice PnPs with its own
  // K), plus a global pool for the bearing-space refinement.
  std::vector<std::vector<Corr>> slice_corr(slices.size());
  std::vector<Corr> pool;
  pool.reserve(matches.size());
  for (const Match &mt : matches) {
    const auto &fr = frames[mt.frame_idx];
    const int k = mt.m.sphere_kp;
    Corr corr;
    corr.slice_px = sphere.pixels[k];
    corr.world = fr.world[mt.m.frame_kp];
    corr.bearing = sphere.bearings[k];
    corr.slice_idx = sphere.slice[k];
    corr.frame_node = cand[mt.frame_idx];
    corr.frame_idx = mt.frame_idx;
    corr.frame_px = fr.f.keypoints[mt.m.frame_kp].pt;
    corr.hamming = mt.m.hamming;
    ++diag.slices[corr.slice_idx].correspondences;
    ++diag.frames[mt.frame_idx].correspondences;
    slice_corr[corr.slice_idx].push_back(corr);
    pool.push_back(corr);
  }

  diag.correspondences_total = static_cast<int>(pool.size());

  // Optional raw correspondence dump; final_inlier is filled in once the
  // refinement has settled (pool and diag.correspondences stay index-aligned).
  if (opt.collect_correspondences) {
    diag.correspondences.reserve(pool.size());
    for (const auto &c : pool) {
      PanoramaCorrespondence pc;
      pc.slice = c.slice_idx;
      pc.frame_node = c.frame_node;
      pc.azimuth_deg = std::atan2(c.bearing.x(), c.bearing.z()) * kRadToDeg;
      pc.elevation_deg =
          std::asin(std::clamp(-c.bearing.y(), -1.0, 1.0)) * kRadToDeg;
      pc.slice_x = c.slice_px.x;
      pc.slice_y = c.slice_px.y;
      pc.frame_x = c.frame_px.x;
      pc.frame_y = c.frame_px.y;
      pc.hamming = c.hamming;
      diag.correspondences.push_back(pc);
    }
  }

  // --- per-slice PnP; keep the strongest as the initial pose ----------------
  std::vector<pano_detail::SlicePnPInput> pnp_in(slices.size());
  for (size_t si = 0; si < slices.size(); ++si) {
    pnp_in[si].K = slices[si].K;
    pnp_in[si].R_pano_from_view = slices[si].R_pano_from_view;
    pnp_in[si].pixels.reserve(slice_corr[si].size());
    pnp_in[si].world.reserve(slice_corr[si].size());
    for (const auto &c : slice_corr[si]) {
      pnp_in[si].pixels.push_back(c.slice_px);
      pnp_in[si].world.push_back(c.world);
    }
  }
  pano_detail::SlicePnPOptions popt;
  popt.min_correspondences = opt.min_slice_correspondences;
  popt.ransac_iterations = opt.ransac_iterations;
  popt.ransac_reproj_px = opt.ransac_reproj_px;
  popt.usac = opt.usac;
  popt.usac_seed = static_cast<int>(opt.seed);
  const pano_detail::SliceResection best =
      pano_detail::resect_best_slice(pnp_in, popt);
  for (size_t si = 0; si < slices.size(); ++si)
    diag.slices[si].pnp_inliers = best.slice_inliers[si];
  diag.best_slice = best.best_slice;
  diag.best_slice_inliers = best.best_inliers;
  const Eigen::Matrix3d Q_best = best.Q; // pano_from_world
  const Eigen::Vector3d t_best = best.t;
  const int best_inliers = best.best_inliers;

  if (best.best_slice < 0 || pool.empty()) {
    core::debug("PanoramaAlignment: panorama {} — insufficient PnP support "
                "(best slice inliers {} < {})",
                pano_id, best_inliers, opt.min_slice_correspondences);
    return finish(PanoramaAlignmentOutcome::no_slice_pnp);
  }

  // --- noise floor (reusex-bx2.4) --------------------------------------------
  // With the pool ~95 % junk, solvePnPRansac returns a "pose" with a dozen
  // inliers on pure noise; the shuffled-correspondence null measured on THIS
  // panorama's slices says how many. A best slice that does not clear it by
  // `null_margin` carries no detectable geometry: every later gate would only
  // report how a random pose happened to fail (or, worse, accept it).
  if (opt.null_trials > 0) {
    diag.null_best_slice_inliers = pano_detail::shuffled_best_slice_inliers(
        pnp_in, popt, opt.null_trials, opt.seed);
    diag.noise_floor_threshold = pano_detail::noise_floor_threshold(
        diag.null_best_slice_inliers, opt.null_margin,
        opt.min_slice_correspondences);
    if (best_inliers < diag.noise_floor_threshold) {
      core::debug("PanoramaAlignment: panorama {} rejected — best slice {} "
                  "PnP inliers < {} (shuffled null {} over {} trials x {:.2f})",
                  pano_id, best_inliers, diag.noise_floor_threshold,
                  diag.null_best_slice_inliers, opt.null_trials,
                  opt.null_margin);
      return finish(PanoramaAlignmentOutcome::below_noise_floor);
    }
  }

  // --- gravity-alignment check on the PnP seed rotation --------------------
  // The per-slice PnP can converge to an upside-down local minimum when matched
  // features are concentrated near the equatorial band and provide no vertical
  // leverage — a known degeneracy of bearing resection without a gravity prior.
  // The bearing-space GN then inherits that flipped seed and stores an
  // upside-down pose.
  //
  // We detect this by comparing the panorama's Y-axis (camera down, +Y_pano)
  // in world space, as implied by Q_best, against the seed frame's camera down
  // direction. Both should be co-oriented (dot product > 0) because both
  // cameras observe the same gravity direction. A negative dot product means
  // the PnP picked the upside-down solution.
  //
  // The equatorial-band features that produced the flipped Q_best are
  // ambiguous: a 180-deg flip leaves their AZIMUTHAL positions unchanged while
  // inverting elevation, so a corrected initial Q predicts those same features
  // at opposite azimuth — far outside any angular gate, and GN will not
  // converge from there. Accordingly, the correct action when the upside-down
  // case is detected is to REJECT the alignment (return without setting
  // res.aligned) rather than to flip and retry. The owner will see these
  // panoramas with their timestamp-based fallback poses rather than with a
  // geometrically wrong aligned pose. (See issue #364.)
  //
  // This guard is skipped when no usable seed pose is available; the 6-DOF
  // path also applies it so that an inverted Q_best does not seed a bad GN
  // trajectory there either.
  if (db.has_sensor_frame_pose(seed_node_id)) {
    const Eigen::Matrix4d T_seed =
        to_matrix4(db.sensor_frame_pose(seed_node_id));
    // Seed camera's Y (down) in world — column 1 of the rotation block.
    const Eigen::Vector3d seed_down_world = T_seed.block<3, 1>(0, 1);
    // Panorama's Y (down) in world — from Q_best = pano_from_world:
    //   column 1 of world_from_pano = column 1 of Q_best^T = row 1 of Q_best.
    const Eigen::Vector3d pano_down_world = Q_best.row(1).transpose();
    const double down_dot = pano_down_world.dot(seed_down_world);
    diag.down_dot = down_dot;
    if (down_dot < 0.0) {
      core::debug(
          "PanoramaAlignment: panorama {} rejected — PnP seed is upside-down "
          "(pano_down·seed_down={:.3f} < 0); equatorial-band features are "
          "ambiguous about vertical orientation",
          pano_id, down_dot);
      // keeps res.aligned = false → timestamp-based fallback
      return finish(PanoramaAlignmentOutcome::upside_down);
    }
  }

  // --- gate the pool by consistency with the initial pose -------------------
  // angular threshold from the slice reprojection tolerance (small-angle),
  // relaxed 2x to admit the honest inliers of the other slices.
  const double fx0 = slices.empty() ? 1.0 : slices[0].K(0, 0);
  const double ang_gate = 2.0 * std::atan(opt.ransac_reproj_px / fx0);

  // Parallel arrays for the shared bearing-space refinement: each pooled
  // correspondence's WORLD point against the panorama bearing it was seen
  // under.
  std::vector<Eigen::Vector3d> pool_pts, pool_bearings;
  pool_pts.reserve(pool.size());
  pool_bearings.reserve(pool.size());
  for (const auto &c : pool) {
    pool_pts.push_back(c.world);
    pool_bearings.push_back(c.bearing);
  }

  // --- Gauss-Newton refine pano_from_world (Q,t) over pooled inliers --------
  // Two paths depending on opt.fix_translation:
  //
  // Default (fix_translation=false):
  //   Shared with the loop-edge front-end (pano_detail::refine_bearing_pose):
  //   left perturbation T <- exp(xi) T, residual e = b_hat - b_obs, re-gating
  //   the inlier set as the pose improves. Full 6-DOF update.
  //
  // fix_translation=true:
  //   The panorama centre is fixed to the seed frame's stored position. Only
  //   the rotation block is solved
  //   (pano_detail::refine_bearing_pose_rotation_only). The per-slice PnP
  //   initial rotation seed (Q_best) is still used; the PnP-estimated
  //   translation is discarded and replaced with the seed position.
  Eigen::Matrix3d Q = Q_best;
  Eigen::Vector3d t = t_best;
  int initial_inliers = 0;
  // In fix-translation mode the rotation seed Q_best was estimated by PnP using
  // the PnP's OWN translation, not fixed_centre. Forcing the centre to
  // fixed_centre shifts the predicted bearings by an amount proportional to the
  // distance between t_pnp and the fixed seed position. On real datasets with
  // SLAM drift this offset can be 1-5 m, shifting bearings by 0.05-0.5 rad at
  // typical indoor scene depth (2-10 m) — far above the tight production
  // ang_gate (~0.02 rad). Using ang_gate for the rotation-only path therefore
  // collapses the initial inlier count to 0 and GN never runs. (issue #364)
  //
  // Fix (Option A): for the rotation-only path, use a 5x looser gate throughout
  // (both the initial seed assessment and the GN re-gate and final gate). The
  // 5x multiplier (≈0.10 rad ≈ 5.7 deg) is consistent with the accepted-pose
  // quality bar for fix-translation: the rotation is solved under a centre that
  // may be tens of centimetres from the true panorama position, so residuals
  // below 0.10 rad are still geometrically meaningful. The tight gate is kept
  // unchanged for the 6-DoF path, so no-flag behaviour is byte-for-byte
  // identical.
  const double rot_only_gate = opt.fix_translation ? 5.0 * ang_gate : ang_gate;
  // For the rotation-only path, allow GN to start from a smaller initial
  // inlier set (min_inliers / 2, floor 5) so that borderline panoramas get a
  // chance to converge. The full opt.min_inliers threshold is enforced on the
  // FINAL inlier count (checked after GN in PanoramaAlignment). The 6-DoF path
  // is unchanged (ro.min_inliers == opt.min_inliers).
  const int rot_only_min =
      opt.fix_translation ? std::max(5, opt.min_inliers / 2) : opt.min_inliers;
  pano_detail::BearingRefineOptions ro{rot_only_gate, opt.refine_iterations,
                                       rot_only_min};
  ro.initial_ang_gate = 0.0;
  std::vector<int> inl_idx;
  // Valid only in the fix_translation path; kept here so the later t-recompute
  // doesn't need a second DB read.
  Eigen::Vector3d fixed_centre = Eigen::Vector3d::Zero();

  if (opt.fix_translation) {
    // Seed the centre from the seed frame's stored pose (world_from_pano
    // convention: the 4th column of the world_from_cam matrix is the camera
    // centre in world space). Fail loud if no usable seed pose is available —
    // without a reliable centre fix_translation mode makes no sense.
    if (!db.has_sensor_frame_pose(seed_node_id)) {
      core::warn(
          "PanoramaAlignment: panorama {} — fix_translation requested "
          "but seed node {} has no usable stored pose; cannot fix centre",
          pano_id, seed_node_id);
      return finish(PanoramaAlignmentOutcome::no_seed_pose);
    }
    fixed_centre =
        to_matrix4(db.sensor_frame_pose(seed_node_id)).block<3, 1>(0, 3);
    // Recompute t from Q_best and the fixed centre so the initial pose is
    // consistent with the fixed-centre constraint before refinement begins.
    t = -Q * fixed_centre;
    inl_idx = pano_detail::refine_bearing_pose_rotation_only(
        pool_pts, pool_bearings, fixed_centre, ro, Q, &initial_inliers);
  } else {
    inl_idx = pano_detail::refine_bearing_pose(pool_pts, pool_bearings, ro, Q,
                                               t, &initial_inliers);
  }

  // In the 6-DoF path the refinement function enforces opt.min_inliers
  // internally, so initial_inliers < opt.min_inliers is a reliable early-exit
  // signal (the function already returned {} and inl_idx is empty).
  //
  // In the rotation-only path ro.min_inliers was lowered to rot_only_min to
  // allow GN to start from a smaller initial inlier set. The FINAL acceptance
  // threshold is still opt.min_inliers and is checked on inl_idx.size() below.
  diag.initial_inliers = initial_inliers;
  if (!opt.fix_translation && initial_inliers < opt.min_inliers) {
    core::debug("PanoramaAlignment: panorama {} — {} pooled inliers < {}",
                pano_id, initial_inliers, opt.min_inliers);
    return finish(PanoramaAlignmentOutcome::too_few_pooled_inliers);
  }
  if (inl_idx.empty()) {
    core::debug("PanoramaAlignment: panorama {} — refinement fell below {} "
                "inliers (started at {})",
                pano_id, opt.min_inliers, initial_inliers);
    return finish(PanoramaAlignmentOutcome::refinement_collapsed);
  }

  // Tally where the final inliers came from (also for a pose that a later
  // gate rejects — that is exactly when the numbers are wanted).
  {
    diag.final_inliers = static_cast<int>(inl_idx.size());
    std::set<int> inl_frames, inl_slices;
    for (int k : inl_idx) {
      const Corr &c = pool[k];
      ++diag.slices[c.slice_idx].final_inliers;
      ++diag.frames[c.frame_idx].final_inliers;
      inl_frames.insert(c.frame_node);
      inl_slices.insert(c.slice_idx);
      if (opt.collect_correspondences)
        diag.correspondences[k].final_inlier = true;
    }
    diag.inlier_frames = static_cast<int>(inl_frames.size());
    diag.inlier_slices = static_cast<int>(inl_slices.size());
  }
  // Enforce the full opt.min_inliers threshold on the final inlier count for
  // the rotation-only path (the function used a lower rot_only_min internally).
  if (opt.fix_translation &&
      static_cast<int>(inl_idx.size()) < opt.min_inliers) {
    core::debug("PanoramaAlignment: panorama {} — rotation-only final inlier "
                "count {} < {} (started at {})",
                pano_id, static_cast<int>(inl_idx.size()), opt.min_inliers,
                initial_inliers);
    return finish(PanoramaAlignmentOutcome::refinement_collapsed);
  }

  std::vector<const Corr *> inl;
  inl.reserve(inl_idx.size());
  for (int k : inl_idx)
    inl.push_back(&pool[k]);

  // In fix_translation mode Q has been updated by the rotation-only GN while
  // `t` was only set before refinement. Recompute t from the final Q and the
  // cached fixed centre so that bearing_angle() and the world_from_pano
  // extraction below use a consistent pose. (In the 6-DOF path GN updates
  // both Q and t together, so this is a no-op there.)
  if (opt.fix_translation)
    t = -Q * fixed_centre;

  double sq = 0.0;
  for (const Corr *c : inl) {
    const double a = bearing_angle(Q, t, c->world, c->bearing);
    sq += a * a;
  }
  const double rms_rad = std::sqrt(sq / inl.size());
  diag.rms_deg = rms_rad * kRadToDeg;

  // world_from_pano: R = Q^T, c = -R t
  const Eigen::Matrix3d R = Q.transpose();
  const Eigen::Vector3d c = -R * t;
  Eigen::Matrix4d T_world_pano = Eigen::Matrix4d::Identity();
  T_world_pano.block<3, 3>(0, 0) = R;
  T_world_pano.block<3, 1>(0, 3) = c;

  // Correction relative to the timestamp-seed placement.
  //
  // fix_translation mode: the centre IS the seed position by construction, so
  // delta is definitionally 0 and the plausibility gate is vacuous. Report the
  // special sentinel so callers can emit informative dry-run output.
  //
  // Default mode: the ONLY use of the seed's stored pose here — the resection
  // itself is driven by the world points in `pool`, which came from frames that
  // already had to pass `has_sensor_frame_pose()` inside
  // extract_frame_features() (#336). With no usable seed pose there is no
  // baseline to measure against: `sensor_frame_pose()`'s identity fallback
  // would silently make `delta` the distance from the world origin, which
  // either waves a bad alignment through or rejects a good one depending on
  // where the scan happens to sit. Report the delta as unmeasurable and skip
  // the gate rather than gate on a number that means nothing.
  //
  // -1.0 is the field's documented "no seed measurement" sentinel; NaN would
  // poison the caller's running average of the corrections.
  double delta = -1.0;
  diag.max_correction_m = opt.max_correction_m;
  if (opt.fix_translation) {
    // Centre came from the seed — has_sensor_frame_pose() was already checked
    // above and returned early if missing.
    delta = 0.0;
    res.translation_fixed = true;
  } else {
    const bool seed_pose_usable = db.has_sensor_frame_pose(seed_node_id);
    if (seed_pose_usable) {
      const Eigen::Matrix4d T_seed =
          to_matrix4(db.sensor_frame_pose(seed_node_id));
      delta = (c - T_seed.block<3, 1>(0, 3)).norm();
      diag.delta_from_seed_m = delta;
    } else {
      core::warn("PanoramaAlignment: panorama {} — seed node {} has no usable "
                 "stored pose, so the {:.1f} m plausibility gate cannot be "
                 "applied to this alignment",
                 pano_id, seed_node_id, opt.max_correction_m);
    }
    // Plausibility gate: an implausibly large correction means the bearing
    // resection was ill-conditioned (narrow match cone) rather than a real fix.
    if (seed_pose_usable && opt.max_correction_m > 0.0 &&
        delta > opt.max_correction_m) {
      core::debug(
          "PanoramaAlignment: panorama {} rejected — {:.1f} m correction "
          "exceeds {:.1f} m gate ({} inliers, {:.3f} deg RMS; likely "
          "ill-conditioned)",
          pano_id, delta, opt.max_correction_m, static_cast<int>(inl.size()),
          rms_rad * 180.0 / M_PI);
      // aligned stays false -> keep timestamp placement
      return finish(PanoramaAlignmentOutcome::correction_gate);
    }
  }
  res.delta_from_seed_m = delta;

  if (opt.fix_translation)
    diag.delta_from_seed_m = 0.0;
  res.inliers = static_cast<int>(inl.size());
  res.rms_deg = rms_rad * 180.0 / M_PI;
  res.pose = from_matrix4(T_world_pano);

  if (res.translation_fixed) {
    core::info("PanoramaAlignment: panorama {} aligned (rotation-only, "
               "translation fixed to seed) — {} inliers, {:.3f} deg RMS",
               pano_id, res.inliers, res.rms_deg);
  } else {
    core::info("PanoramaAlignment: panorama {} aligned — {} inliers, {:.3f} "
               "deg RMS, {:.3f} m from seed",
               pano_id, res.inliers, res.rms_deg, res.delta_from_seed_m);
  }

  // Optional correspondence figure: pick the (slice, frame) pair contributing
  // the most inliers and draw the matches between the panorama slice and that
  // scan frame.
  if (!opt.debug_dir.empty()) {
    std::map<std::pair<int, int>, std::vector<const Corr *>> by_pair;
    for (const Corr *c : inl)
      by_pair[{c->slice_idx, c->frame_node}].push_back(c);
    const std::vector<const Corr *> *best = nullptr;
    std::pair<int, int> best_key;
    for (const auto &kv : by_pair)
      if (!best || kv.second.size() > best->size()) {
        best = &kv.second;
        best_key = kv.first;
      }
    if (best && best_key.first >= 0 &&
        best_key.first < static_cast<int>(slices.size())) {
      const std::string name =
          opt.debug_name.empty() ? std::to_string(pano_id) : opt.debug_name;
      draw_correspondence_figure(db, slices[best_key.first].image,
                                 best_key.second, *best,
                                 opt.debug_dir + "/" + name + "_matches.jpg");
    }
  }
  return finish(PanoramaAlignmentOutcome::aligned);
}

} // namespace reusex::geometry
