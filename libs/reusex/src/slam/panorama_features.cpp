// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// See panorama_features.hpp. Extracted verbatim from PanoramaAlignment.cpp when
// issue #236 added a second consumer; the only behavioural change is that
// keypoints are lifted to the frame's OPTICAL coordinates and composed to world
// on demand, instead of being baked into world during extraction.

#include "panorama_features.hpp"

#include "core/ProjectDB.hpp"
#include "core/SensorIntrinsics.hpp"
#include "core/logging.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <map>
#include <random>

namespace reusex::geometry::pano_detail {

Eigen::Matrix4d to_matrix4(const std::array<double, 16> &a) {
  Eigen::Matrix4d M;
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c)
      M(r, c) = a[r * 4 + c];
  return M;
}

std::array<double, 16> from_matrix4(const Eigen::Matrix4d &M) {
  std::array<double, 16> a{};
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c)
      a[r * 4 + c] = M(r, c);
  return a;
}

Eigen::Matrix3d exp_so3(const Eigen::Vector3d &w) {
  const double th = w.norm();
  if (th < 1e-12)
    return Eigen::Matrix3d::Identity();
  const Eigen::Vector3d axis = w / th;
  Eigen::Matrix3d K;
  K << 0, -axis.z(), axis.y(), axis.z(), 0, -axis.x(), -axis.y(), axis.x(), 0;
  return Eigen::Matrix3d::Identity() + std::sin(th) * K +
         (1 - std::cos(th)) * K * K;
}

Eigen::Matrix3d skew(const Eigen::Vector3d &v) {
  Eigen::Matrix3d S;
  S << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
  return S;
}

double bearing_angle(const Eigen::Matrix3d &Q, const Eigen::Vector3d &t,
                     const Eigen::Vector3d &X, const Eigen::Vector3d &b_obs) {
  const Eigen::Vector3d m = Q * X + t;
  const double n = m.norm();
  if (n < 1e-9)
    return M_PI;
  const double c = std::clamp((m / n).dot(b_obs), -1.0, 1.0);
  return std::acos(c);
}

FrameFeatures extract_frame_features(const ProjectDB &db, int node_id,
                                     const cv::Ptr<cv::ORB> &orb,
                                     float min_depth, float max_depth,
                                     const std::optional<cv::Rect> &region,
                                     bool require_pose) {
  FrameFeatures out;
  out.node_id = node_id;

  cv::Mat color = db.sensor_frame_image(node_id);
  cv::Mat depth16 = db.sensor_frame_depth(node_id);
  if (color.empty() || depth16.empty())
    return out;

  cv::Mat gray;
  if (color.channels() == 3)
    cv::cvtColor(color, gray, cv::COLOR_BGR2GRAY);
  else if (color.channels() == 4)
    cv::cvtColor(color, gray, cv::COLOR_BGRA2GRAY);
  else
    gray = color;

  cv::Mat depth_f;
  depth16.convertTo(depth_f, CV_32FC1, 1.0 / 1000.0); // mm -> m

  const core::SensorIntrinsics intr = db.sensor_frame_intrinsics(node_id);
  const DepthPinhole k = depth_pinhole(intr, depth_f.cols, depth_f.rows);
  if (k.fx <= 0.0 || k.fy <= 0.0)
    return out;

  // T_world_cam turns this frame's depth into the WORLD 3D points a panorama
  // is resected against. `sensor_frame_pose()`'s identity fallback would place
  // that whole point set at the origin and a degenerate transform would
  // collapse it, either way handing the resection correspondences that are
  // geometrically wrong but perfectly well-formed (#336). An empty
  // `descriptors` is the established "this frame contributes nothing" signal
  // every caller already checks.
  //
  // require_pose == false (reusex-467, LoopClosure): the caller only wants
  // the frame-local 3D points and supplies its own seed poses, so a missing
  // stored pose is not a reason to drop the frame — T_world_cam is simply
  // left at Identity, which such a caller must never read.
  if (db.has_sensor_frame_pose(node_id)) {
    out.T_world_cam = to_matrix4(db.sensor_frame_pose(node_id));
  } else if (require_pose) {
    core::debug("panorama_features: node {} has no usable stored pose, "
                "contributing no correspondences",
                node_id);
    return out;
  }

  cv::Mat mask; // empty = the whole image
  if (region) {
    mask = cv::Mat::zeros(gray.size(), CV_8U);
    const cv::Rect r = *region & cv::Rect(0, 0, gray.cols, gray.rows);
    if (r.area() > 0)
      mask(r).setTo(255);
  }
  orb->detectAndCompute(gray, mask, out.keypoints, out.descriptors);
  if (out.keypoints.empty()) {
    out.descriptors.release();
    return out;
  }

  out.local.resize(out.keypoints.size());
  out.valid.assign(out.keypoints.size(), 0);
  for (size_t i = 0; i < out.keypoints.size(); ++i) {
    const int ud = static_cast<int>(std::lround(
        out.keypoints[i].pt.x * static_cast<double>(depth_f.cols) / gray.cols));
    const int vd = static_cast<int>(std::lround(
        out.keypoints[i].pt.y * static_cast<double>(depth_f.rows) / gray.rows));
    if (const auto p =
            lift_depth_pixel(depth_f, k, ud, vd, min_depth, max_depth)) {
      out.local[i] = *p;
      out.valid[i] = 1;
    }
  }
  return out;
}

DepthPinhole depth_pinhole(const core::SensorIntrinsics &intr, int depth_cols,
                           int depth_rows) {
  const double sx = static_cast<double>(depth_cols) / std::max(1, intr.width);
  const double sy = static_cast<double>(depth_rows) / std::max(1, intr.height);
  return {intr.fx * sx, intr.fy * sy, intr.cx * sx, intr.cy * sy};
}

std::optional<Eigen::Vector3d> lift_depth_pixel(const cv::Mat &depth_m,
                                                const DepthPinhole &k, int ud,
                                                int vd, float min_depth,
                                                float max_depth) {
  if (ud < 0 || vd < 0 || ud >= depth_m.cols || vd >= depth_m.rows)
    return std::nullopt;
  const float z = depth_m.at<float>(vd, ud);
  if (!std::isfinite(z) || z < min_depth || z > max_depth)
    return std::nullopt;
  return Eigen::Vector3d((ud - k.cx) * z / k.fx, (vd - k.cy) * z / k.fy, z);
}

std::vector<int>
refine_bearing_pose(const std::vector<Eigen::Vector3d> &points,
                    const std::vector<Eigen::Vector3d> &bearings,
                    const BearingRefineOptions &opt, Eigen::Matrix3d &Q,
                    Eigen::Vector3d &t, int *out_initial_inliers) {
  auto gate = [&](const Eigen::Matrix3d &Qc, const Eigen::Vector3d &tc) {
    std::vector<int> sel;
    for (size_t k = 0; k < points.size(); ++k)
      if (bearing_angle(Qc, tc, points[k], bearings[k]) < opt.ang_gate)
        sel.push_back(static_cast<int>(k));
    return sel;
  };

  std::vector<int> inl = gate(Q, t);
  if (out_initial_inliers)
    *out_initial_inliers = static_cast<int>(inl.size());
  if (static_cast<int>(inl.size()) < opt.min_inliers)
    return {};

  for (int it = 0; it < opt.iterations; ++it) {
    Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 1> g = Eigen::Matrix<double, 6, 1>::Zero();
    for (int k : inl) {
      const Eigen::Vector3d m = Q * points[k] + t;
      const double n = m.norm();
      if (n < 1e-9)
        continue;
      const Eigen::Vector3d bh = m / n;
      const Eigen::Vector3d e = bh - bearings[k];
      const Eigen::Matrix3d dbh_dm =
          (Eigen::Matrix3d::Identity() - bh * bh.transpose()) / n;
      Eigen::Matrix<double, 3, 6> Jm;
      Jm.block<3, 3>(0, 0) = -skew(m);                    // wrt rotation
      Jm.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity(); // wrt translation
      const Eigen::Matrix<double, 3, 6> J = dbh_dm * Jm;
      H += J.transpose() * J;
      g += J.transpose() * e;
    }
    H += 1e-9 * Eigen::Matrix<double, 6, 6>::Identity();
    const Eigen::Matrix<double, 6, 1> dx = H.ldlt().solve(-g);
    if (!dx.allFinite())
      break;
    const Eigen::Matrix3d dR = exp_so3(dx.head<3>());
    Q = dR * Q;
    t = dR * t + dx.tail<3>();
    if (dx.norm() < 1e-8)
      break;
    // Re-gate so the inlier set follows the improving pose.
    if (it + 1 < opt.iterations)
      inl = gate(Q, t);
    if (static_cast<int>(inl.size()) < opt.min_inliers)
      break;
  }

  inl = gate(Q, t);
  if (static_cast<int>(inl.size()) < opt.min_inliers)
    return {};
  return inl;
}

std::vector<int> refine_bearing_pose_rotation_only(
    const std::vector<Eigen::Vector3d> &points,
    const std::vector<Eigen::Vector3d> &bearings,
    const Eigen::Vector3d &c_world, const BearingRefineOptions &opt,
    Eigen::Matrix3d &Q, int *out_initial_inliers) {
  // With a fixed centre c_world, the pano_from_world transform is:
  //   m = Q * X + t,  where t = -Q * c_world
  // so m = Q * (X - c_world).  The Jacobian wrt the rotation perturbation dw
  // (left perturbation: Q <- exp(dw) Q) gives:
  //   dm/dw = -skew(Q * (X - c_world)) = -skew(m)
  // same as the rotation block in the 6-DOF case — only the 3-DOF translation
  // column is removed.

  auto t_from_Q = [&](const Eigen::Matrix3d &Qc) -> Eigen::Vector3d {
    return -Qc * c_world;
  };

  // Tight gate used for re-gating during and after GN (preserves final
  // quality).
  auto gate = [&](const Eigen::Matrix3d &Qc) {
    const Eigen::Vector3d tc = t_from_Q(Qc);
    std::vector<int> sel;
    for (size_t k = 0; k < points.size(); ++k)
      if (bearing_angle(Qc, tc, points[k], bearings[k]) < opt.ang_gate)
        sel.push_back(static_cast<int>(k));
    return sel;
  };

  // Looser seed gate for the INITIAL inlier assessment only. Used by the
  // fix-translation path: the PnP rotation seed Q_best was estimated with the
  // PnP's own translation, not fixed_centre, so forcing the centre to
  // fixed_centre shifts the predicted bearings and most correspondences exceed
  // the tight production ang_gate before GN has run. The loose seed gate lets
  // GN start from a non-empty inlier set; subsequent re-gates use the tight
  // ang_gate so final quality is unchanged. (issue #364)
  const double seed_gate =
      (opt.initial_ang_gate > 0.0) ? opt.initial_ang_gate : opt.ang_gate;
  auto seed_gate_fn = [&](const Eigen::Matrix3d &Qc) {
    const Eigen::Vector3d tc = t_from_Q(Qc);
    std::vector<int> sel;
    for (size_t k = 0; k < points.size(); ++k)
      if (bearing_angle(Qc, tc, points[k], bearings[k]) < seed_gate)
        sel.push_back(static_cast<int>(k));
    return sel;
  };

  std::vector<int> inl = seed_gate_fn(Q);
  if (out_initial_inliers)
    *out_initial_inliers = static_cast<int>(inl.size());
  if (static_cast<int>(inl.size()) < opt.min_inliers)
    return {};

  for (int it = 0; it < opt.iterations; ++it) {
    Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
    Eigen::Vector3d g = Eigen::Vector3d::Zero();
    const Eigen::Vector3d t = t_from_Q(Q);
    for (int k : inl) {
      const Eigen::Vector3d m = Q * points[k] + t; // = Q * (X - c_world)
      const double n = m.norm();
      if (n < 1e-9)
        continue;
      const Eigen::Vector3d bh = m / n;
      const Eigen::Vector3d e = bh - bearings[k];
      const Eigen::Matrix3d dbh_dm =
          (Eigen::Matrix3d::Identity() - bh * bh.transpose()) / n;
      const Eigen::Matrix<double, 3, 3> J =
          dbh_dm * (-skew(m)); // rotation only
      H += J.transpose() * J;
      g += J.transpose() * e;
    }
    H += 1e-9 * Eigen::Matrix3d::Identity();
    const Eigen::Vector3d dw = H.ldlt().solve(-g);
    if (!dw.allFinite())
      break;
    const Eigen::Matrix3d dR = exp_so3(dw);
    Q = dR * Q;
    if (dw.norm() < 1e-8)
      break;
    if (it + 1 < opt.iterations)
      inl = gate(Q);
    if (static_cast<int>(inl.size()) < opt.min_inliers)
      break;
  }

  inl = gate(Q);
  if (static_cast<int>(inl.size()) < opt.min_inliers)
    return {};
  return inl;
}

// --- per-slice PnP and its noise floor (reusex-bx2.4) ----------------------

namespace {

/// Correspondences whose slice pixels fall in the same cell of this size are
/// one group for the noise-floor null (they move together).
constexpr float kNullGroupCellPx = 2.0f;

// One slice's solvePnPRansac (EPnP). Returns the inlier count (0 when PnP
// failed) and, on success, the pano_from_world pose it implies.
int pnp_slice(const SlicePnPInput &s, const std::vector<cv::Point2f> &pixels,
              const std::vector<Eigen::Vector3d> &world,
              const SlicePnPOptions &opt, Eigen::Matrix3d *Q,
              Eigen::Vector3d *t) {
  std::vector<cv::Point3f> obj;
  obj.reserve(world.size());
  for (const auto &w : world)
    obj.emplace_back(static_cast<float>(w.x()), static_cast<float>(w.y()),
                     static_cast<float>(w.z()));
  const cv::Mat Kcv = (cv::Mat_<double>(3, 3) << s.K(0, 0), 0, s.K(0, 2), 0,
                       s.K(1, 1), s.K(1, 2), 0, 0, 1);
  cv::Mat rvec, tvec, inliers;
  bool ok = false;
  if (opt.usac) {
    // MAGSAC++ scoring with its own local optimisation; the generator state
    // is the only randomness, so a fixed seed makes it reproducible.
    cv::UsacParams up;
    up.confidence = 0.99;
    up.isParallel = false; // parallel USAC is not reproducible
    up.maxIterations = opt.ransac_iterations;
    up.threshold = opt.ransac_reproj_px;
    up.randomGeneratorState = opt.usac_seed;
    up.sampler = cv::SAMPLING_UNIFORM;
    up.score = cv::SCORE_METHOD_MAGSAC;
    up.loMethod = cv::LOCAL_OPTIM_SIGMA;
    up.loIterations = 10;
    up.loSampleSize = 50;
    try {
      ok = cv::solvePnPRansac(obj, pixels, Kcv, cv::noArray(), rvec, tvec,
                              inliers, up);
    } catch (const cv::Exception &) {
      ok = false; // degenerate input: treated like a failed PnP
    }
  } else {
    ok = cv::solvePnPRansac(obj, pixels, Kcv, cv::noArray(), rvec, tvec,
                            /*useExtrinsicGuess=*/false, opt.ransac_iterations,
                            opt.ransac_reproj_px, 0.99, inliers,
                            cv::SOLVEPNP_EPNP);
  }
  if (!ok || rvec.empty() || tvec.empty())
    return 0;
  if (Q && t) {
    cv::Mat Rcv;
    cv::Rodrigues(rvec, Rcv);
    Eigen::Matrix3d R_ws; // world -> slice-cam
    Eigen::Vector3d t_ws;
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c)
        R_ws(r, c) = Rcv.at<double>(r, c);
      t_ws(r) = tvec.at<double>(r);
    }
    // pano_from_world = R_pano_from_view * (world -> slice-cam)
    *Q = s.R_pano_from_view * R_ws;
    *t = s.R_pano_from_view * t_ws;
  }
  return inliers.rows;
}

bool pnp_eligible(const SlicePnPInput &s, const SlicePnPOptions &opt) {
  return s.pixels.size() == s.world.size() &&
         static_cast<int>(s.pixels.size()) >= opt.min_correspondences;
}

} // namespace

SliceResection resect_best_slice(const std::vector<SlicePnPInput> &slices,
                                 const SlicePnPOptions &opt) {
  SliceResection out;
  out.slice_inliers.assign(slices.size(), -1);
  for (size_t si = 0; si < slices.size(); ++si) {
    if (!pnp_eligible(slices[si], opt))
      continue;
    Eigen::Matrix3d Q;
    Eigen::Vector3d t;
    const int n =
        pnp_slice(slices[si], slices[si].pixels, slices[si].world, opt, &Q, &t);
    out.slice_inliers[si] = n;
    if (n < opt.min_correspondences || n <= out.best_inliers)
      continue;
    out.best_slice = static_cast<int>(si);
    out.best_inliers = n;
    out.Q = Q;
    out.t = t;
  }
  return out;
}

NullFloor shuffled_null_floor(const std::vector<SlicePnPInput> &slices,
                              const SlicePnPOptions &opt, int trials,
                              unsigned seed,
                              const BearingRefineOptions *pooled) {
  NullFloor out;
  std::mt19937 rng(seed);
  int ceiling = 0;
  std::vector<cv::Point2f> pixels;
  // With `pooled`: every trial's permuted pixels per slice, and every null
  // slice pose, to gate that trial's whole permuted pool afterwards.
  struct NullPose {
    int trial;
    Eigen::Matrix3d Q;
    Eigen::Vector3d t;
  };
  std::vector<NullPose> poses;
  std::vector<std::vector<std::vector<cv::Point2f>>> trial_pixels(
      pooled ? slices.size() : 0);
  for (size_t si = 0; si < slices.size(); ++si) {
    const auto &s = slices[si];
    if (!pnp_eligible(s, opt))
      continue;
    // Group the correspondences by slice location. The junk the matcher
    // emits is not independent per pair: one slice keypoint mis-matched to
    // the same surface point in several near-identical candidate frames, or
    // ORB's duplicate detections of one corner on several pyramid levels,
    // give runs of (almost) the same pair. Any pose that explains one of them
    // explains the run, so permuting single pairs would underestimate the
    // noise floor. Instead whole location groups are moved together.
    std::map<std::pair<int, int>, int> cell_of;
    std::vector<int> group(s.pixels.size());
    std::vector<cv::Point2f> anchor; // first pixel of each group
    for (size_t k = 0; k < s.pixels.size(); ++k) {
      const std::pair<int, int> cell{
          static_cast<int>(std::floor(s.pixels[k].x / kNullGroupCellPx)),
          static_cast<int>(std::floor(s.pixels[k].y / kNullGroupCellPx))};
      auto [it, inserted] =
          cell_of.emplace(cell, static_cast<int>(anchor.size()));
      if (inserted)
        anchor.push_back(s.pixels[k]);
      group[k] = it->second;
    }
    std::vector<int> perm(anchor.size());
    for (int trial = 0; trial < trials; ++trial) {
      // Fisher-Yates with a plain modulo draw: std::shuffle and
      // std::uniform_int_distribution are implementation-defined, mt19937's
      // output sequence is not (STANDARDS §6).
      for (size_t g = 0; g < perm.size(); ++g)
        perm[g] = static_cast<int>(g);
      for (size_t i = perm.size() - 1; i > 0; --i)
        std::swap(perm[i], perm[rng() % (i + 1)]);
      pixels.resize(s.pixels.size());
      for (size_t k = 0; k < s.pixels.size(); ++k)
        pixels[k] = anchor[perm[group[k]]] + (s.pixels[k] - anchor[group[k]]);
      if (!pooled) {
        ceiling = std::max(
            ceiling, pnp_slice(s, pixels, s.world, opt, nullptr, nullptr));
        continue;
      }
      NullPose np{trial, Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero()};
      const int n = pnp_slice(s, pixels, s.world, opt, &np.Q, &np.t);
      ceiling = std::max(ceiling, n);
      if (n >= opt.min_correspondences)
        poses.push_back(np);
      trial_pixels[si].push_back(pixels);
    }
  }
  out.best_slice_inliers = ceiling;
  if (!pooled)
    return out;
  out.pooled_initial = 0;
  out.pooled_final = 0;
  if (trials <= 0 || poses.empty())
    return out;

  // Slices too small to PnP still enter the real pool; their pairings are
  // permuted too, with a generator of their own so the best-slice draws
  // above stay those of shuffled_best_slice_inliers().
  std::mt19937 rng_small(seed ^ 0x9e3779b9u);
  std::vector<Eigen::Vector3d> pts, brg;
  for (int trial = 0; trial < trials; ++trial) {
    pts.clear();
    brg.clear();
    for (size_t si = 0; si < slices.size(); ++si) {
      const auto &s = slices[si];
      if (s.pixels.size() != s.world.size())
        continue;
      if (pnp_eligible(s, opt)) {
        const auto &px = trial_pixels[si][trial];
        for (size_t k = 0; k < px.size(); ++k) {
          pts.push_back(s.world[k]);
          brg.push_back(slice_bearing(s.K, s.R_pano_from_view, px[k]));
        }
        continue;
      }
      std::vector<int> perm(s.pixels.size());
      for (size_t k = 0; k < perm.size(); ++k)
        perm[k] = static_cast<int>(k);
      for (size_t i = perm.size(); i > 1; --i)
        std::swap(perm[i - 1], perm[rng_small() % i]);
      for (size_t k = 0; k < s.pixels.size(); ++k) {
        pts.push_back(s.world[k]);
        brg.push_back(
            slice_bearing(s.K, s.R_pano_from_view, s.pixels[perm[k]]));
      }
    }
    for (const NullPose &np : poses) {
      if (np.trial != trial)
        continue;
      Eigen::Matrix3d Q = np.Q;
      Eigen::Vector3d t = np.t;
      int initial = 0;
      const int fin = static_cast<int>(
          refine_bearing_pose(pts, brg, *pooled, Q, t, &initial).size());
      out.pooled_initial = std::max(out.pooled_initial, initial);
      out.pooled_final = std::max(out.pooled_final, fin);
    }
  }
  return out;
}

int shuffled_best_slice_inliers(const std::vector<SlicePnPInput> &slices,
                                const SlicePnPOptions &opt, int trials,
                                unsigned seed) {
  return shuffled_null_floor(slices, opt, trials, seed, nullptr)
      .best_slice_inliers;
}

int noise_floor_threshold(int null_best, double margin,
                          int min_correspondences) {
  const int beat =
      static_cast<int>(std::floor(margin * std::max(0, null_best))) + 1;
  return std::max(min_correspondences, null_best > 0 ? beat : 0);
}

} // namespace reusex::geometry::pano_detail

namespace reusex::geometry::pano_detail {

// --- the correspondence pool (reusex-bx2.5) --------------------------------

Eigen::Vector3d slice_bearing(const Eigen::Matrix3d &K,
                              const Eigen::Matrix3d &R_pano_from_view,
                              const cv::Point2f &px) {
  const Eigen::Vector3d ray((px.x - K(0, 2)) / K(0, 0),
                            (px.y - K(1, 2)) / K(1, 1), 1.0);
  return (R_pano_from_view * ray).normalized();
}

namespace {

// Angle (rad) between unit vectors.
double angle_between(const Eigen::Vector3d &a, const Eigen::Vector3d &b) {
  return std::acos(std::clamp(a.dot(b), -1.0, 1.0));
}

} // namespace

bool project_to_slice(const SliceKeypoints &s, const Eigen::Vector3d &bearing,
                      cv::Point2f *px) {
  const Eigen::Vector3d d = s.R_pano_from_view.transpose() * bearing;
  if (d.z() <= 1e-9)
    return false;
  const double u = s.K(0, 0) * d.x() / d.z() + s.K(0, 2);
  const double v = s.K(1, 1) * d.y() / d.z() + s.K(1, 2);
  if (!(u >= 0.0 && v >= 0.0 && u < s.size.width && v < s.size.height))
    return false;
  if (px)
    *px = cv::Point2f(static_cast<float>(u), static_cast<float>(v));
  return true;
}

int owning_slice(const std::vector<SliceKeypoints> &slices,
                 const Eigen::Vector3d &bearing) {
  int best = -1;
  double best_cos = -2.0;
  for (size_t i = 0; i < slices.size(); ++i) {
    if (!project_to_slice(slices[i], bearing, nullptr))
      continue;
    const double c = slices[i].R_pano_from_view.col(2).dot(bearing);
    if (c > best_cos + 1e-12) {
      best_cos = c;
      best = static_cast<int>(i);
    }
  }
  return best;
}

SphereKeypoints pool_sphere_keypoints(const std::vector<SliceKeypoints> &slices,
                                      const SpherePoolOptions &opt) {
  SphereKeypoints out;
  const Eigen::Vector3d down(0.0, 1.0, 0.0); // +y of the panorama frame
  const double nadir = opt.nadir_mask_deg * M_PI / 180.0;
  for (size_t si = 0; si < slices.size(); ++si) {
    const auto &s = slices[si];
    for (size_t k = 0; k < s.pixels.size(); ++k) {
      if (static_cast<int>(k) >= s.descriptors.rows)
        break;
      const Eigen::Vector3d b =
          slice_bearing(s.K, s.R_pano_from_view, s.pixels[k]);
      if (nadir > 0.0 && angle_between(b, down) < nadir) {
        ++out.dropped_pole;
        continue;
      }
      if (opt.dedupe_overlap) {
        // a keypoint on the slice's own edge can fall just outside its image
        // after the float round trip; the detecting slice then still owns it
        const int owner = owning_slice(slices, b);
        if (owner >= 0 && owner != static_cast<int>(si)) {
          ++out.dropped_overlap;
          continue;
        }
      }
      out.pixels.push_back(s.pixels[k]);
      out.slice.push_back(static_cast<int>(si));
      out.bearings.push_back(b);
      out.descriptors.push_back(s.descriptors.row(static_cast<int>(k)));
    }
  }
  return out;
}

std::vector<SphereMatch> match_frame_to_sphere(const cv::Mat &frame_desc,
                                               const SphereKeypoints &sphere,
                                               float ratio, double distinct_deg,
                                               int k) {
  std::vector<SphereMatch> out;
  if (frame_desc.empty() || sphere.descriptors.rows < 2)
    return out;
  cv::BFMatcher matcher(cv::NORM_HAMMING);
  std::vector<std::vector<cv::DMatch>> knn;
  matcher.knnMatch(frame_desc, sphere.descriptors, knn, std::max(2, k));
  const double distinct = distinct_deg * M_PI / 180.0;
  for (const auto &m : knn) {
    if (m.empty())
      continue;
    const Eigen::Vector3d &b0 = sphere.bearings[m[0].trainIdx];
    // the nearest descriptor at a different place on the sphere
    const cv::DMatch *rival = nullptr;
    for (size_t j = 1; j < m.size(); ++j)
      if (angle_between(sphere.bearings[m[j].trainIdx], b0) > distinct) {
        rival = &m[j];
        break;
      }
    if (rival && m[0].distance > ratio * rival->distance)
      continue;
    out.push_back({m[0].queryIdx, m[0].trainIdx, m[0].distance});
  }
  return out;
}

std::vector<SphereMatch> match_sphere_to_frame(const cv::Mat &frame_desc,
                                               const SphereKeypoints &sphere,
                                               float ratio) {
  std::vector<SphereMatch> out;
  if (frame_desc.rows < 2 || sphere.descriptors.empty())
    return out;
  cv::BFMatcher matcher(cv::NORM_HAMMING);
  std::vector<std::vector<cv::DMatch>> knn;
  matcher.knnMatch(sphere.descriptors, frame_desc, knn, 2);
  for (const auto &m : knn) {
    if (m.size() < 2 || m[0].distance > ratio * m[1].distance)
      continue;
    out.push_back({m[0].trainIdx, m[0].queryIdx, m[0].distance});
  }
  return out;
}

std::vector<int> one_per_sphere_keypoint(const std::vector<int> &sphere_kp,
                                         const std::vector<float> &hamming) {
  std::map<int, int> best; // sphere keypoint -> entry
  for (size_t i = 0; i < sphere_kp.size(); ++i) {
    auto [it, inserted] = best.emplace(sphere_kp[i], static_cast<int>(i));
    if (!inserted && hamming[i] < hamming[it->second])
      it->second = static_cast<int>(i);
  }
  std::vector<int> keep;
  keep.reserve(best.size());
  for (const auto &kv : best)
    keep.push_back(kv.second);
  std::sort(keep.begin(), keep.end());
  return keep;
}

// --- candidate frame selection (reusex-bx2.6) ------------------------------

namespace {
constexpr int kSharpnessWidth = 320;
}

double image_sharpness(const cv::Mat &image) {
  if (image.empty())
    return 0.0;
  cv::Mat gray;
  if (image.channels() == 3)
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
  else if (image.channels() == 4)
    cv::cvtColor(image, gray, cv::COLOR_BGRA2GRAY);
  else
    gray = image;
  cv::Mat small;
  const double s = static_cast<double>(kSharpnessWidth) / gray.cols;
  cv::resize(
      gray, small,
      cv::Size(kSharpnessWidth,
               std::max(1, static_cast<int>(std::lround(gray.rows * s)))),
      0, 0, cv::INTER_AREA);
  cv::Mat lap;
  cv::Laplacian(small, lap, CV_64F);
  cv::Scalar mean, stddev;
  cv::meanStdDev(lap, mean, stddev);
  return stddev[0] * stddev[0];
}

std::vector<int> select_candidate_frames(const std::vector<FrameChoice> &frames,
                                         const FrameSelectOptions &opt) {
  double best = 0.0;
  for (const auto &f : frames)
    best = std::max(best, f.sharpness);
  std::vector<int> order;
  for (int i = 0; i < static_cast<int>(frames.size()); ++i)
    if (frames[i].sharpness >= opt.min_sharpness_ratio * best)
      order.push_back(i);
  std::sort(order.begin(), order.end(), [&](int a, int b) {
    const auto &fa = frames[a], &fb = frames[b];
    if (fa.sharpness != fb.sharpness)
      return fa.sharpness > fb.sharpness;
    if (std::abs(fa.dt_s) != std::abs(fb.dt_s))
      return std::abs(fa.dt_s) < std::abs(fb.dt_s);
    return fa.node_id < fb.node_id;
  });
  const double cos_spread = std::cos(opt.view_spread_deg * M_PI / 180.0);
  std::vector<int> picked;
  std::vector<char> used(frames.size(), 0);
  auto opens_new_direction = [&](int i) {
    for (int j : picked)
      if (frames[i].view_dir.normalized().dot(frames[j].view_dir.normalized()) >
          cos_spread)
        return false;
    return true;
  };
  // first every frame that opens a new direction, sharpest first...
  bool grew = true;
  while (grew && static_cast<int>(picked.size()) < opt.max_frames) {
    grew = false;
    for (int i : order)
      if (!used[i] && opens_new_direction(i)) {
        picked.push_back(i);
        used[i] = 1;
        grew = true;
        break;
      }
  }
  // ...then the sharpest of the rest
  for (int i : order) {
    if (static_cast<int>(picked.size()) >= opt.max_frames)
      break;
    if (!used[i]) {
      picked.push_back(i);
      used[i] = 1;
    }
  }
  return picked;
}

} // namespace reusex::geometry::pano_detail
