// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// P2 loop-closure edge detection. See LoopClosure.hpp for the design.
//
// Front-end: OpenCV ORB (BSD, no model weights) detects and describes features;
// matches are lifted to metric 3D via the stored per-frame depth (exactly the
// pinhole convention used by surfel_extraction) and a robust relative pose is
// estimated with RANSAC over 3D-3D Umeyama fits. The matcher is intentionally
// simple and license-clean; a learned matcher can replace it behind the same
// 3D-correspondence interface without changing the graph wiring.

#include "slam/LoopClosure.hpp"

#include "core/ProjectDB.hpp"
#include "core/SensorIntrinsics.hpp"
#include "core/logging.hpp"
#include "panorama_features.hpp"

#include <opencv2/features2d.hpp>

#include <Eigen/Geometry>

#include <fmt/format.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <numeric>
#include <random>
#include <set>
#include <stdexcept>
#include <utility>

namespace reusex::geometry {

namespace {

// LoopClosure's own ORB+depth lifting (once duplicated here) now reuses the
// panorama front-end's `pano_detail::extract_frame_features` (reusex-xlg.11):
// same ORB detect+compute, same colour-pixel -> depth-pixel mapping, same
// pinhole back-projection, same [min_depth, max_depth] gate. The 3D points
// come back in `local` (optical frame) instead of the old `pts3d` name;
// `valid` and `descriptors` are unchanged. (Used unaliased: `detect_loop_edges`
// below already has a local variable named `pd`.)

/// Descriptor width every routine here assumes: OpenCV ORB always describes
/// with 32 bytes / 256 bits. hamming32() reads exactly this many bytes per
/// descriptor, so a block of any other width must be rejected, not trusted.
constexpr int kOrbDescriptorBytes = 32;

/// True when `m` is an empty or a usable (32-column CV_8U) descriptor block.
bool is_orb_descriptor_block(const cv::Mat &m) {
  return m.empty() || (m.type() == CV_8U && m.cols == kOrbDescriptorBytes);
}

/// Thin wrapper around the shared extractor that preserves two LoopClosure
/// safety nets the shared helper does not have:
///  - a LOUD warn-and-skip on degenerate intrinsics (width/height <= 0, or a
///    non-positive focal length), mirroring extract_frame_surfels. The shared
///    helper instead clamps a non-positive width to 1 (PanoramaAlignment /
///    PanoramaLoopEdges silently accept that); gated on `has_sensor_frame` so
///    a frame with literally nothing stored (the degenerate-input tests)
///    stays silent, exactly as before.
///  - rejecting a malformed ORB descriptor block outright instead of trusting
///    it into the Hamming-distance routines below.
///
/// Known behavioural difference from the old local implementation: the
/// shared helper also requires `db.has_sensor_frame_pose(node_id)` (it
/// composes a world pose for the OTHER two callers) and returns no
/// keypoints at all when that is false, even with perfectly good
/// colour/depth/intrinsics. The old LoopClosure extractor never looked at
/// the stored pose — `detect_loop_edges` takes `seed_poses` as an explicit
/// argument instead. In practice every caller (`collect_loop_candidates` via
/// `graph_discover`) only ever passes node ids that already have a stored
/// pose (it requires `has_initial_pose` or `has_sensor_frame_pose` to include
/// a frame at all), so this does not change today's results, but a future
/// caller supplying externally-computed seed poses for frames with no stored
/// `sensor_frames.transform` would now silently get zero features for them.
pano_detail::FrameFeatures extract_features(ProjectDB &db, int node_id,
                                            const LoopClosureOptions &opt,
                                            const cv::Ptr<cv::ORB> &orb) {
  if (db.has_sensor_frame(node_id)) {
    const core::SensorIntrinsics intr = db.sensor_frame_intrinsics(node_id);
    if (intr.width <= 0 || intr.height <= 0) {
      core::warn("LoopClosure: node {} has invalid intrinsic dimensions {}x{}; "
                 "skipping frame",
                 node_id, intr.width, intr.height);
      return {};
    }
    if (intr.fx <= 0.0 || intr.fy <= 0.0) {
      core::warn("LoopClosure: node {} has non-positive focal length "
                 "({:.3f}, {:.3f}); skipping frame",
                 node_id, intr.fx, intr.fy);
      return {};
    }
  }

  pano_detail::FrameFeatures f = pano_detail::extract_frame_features(
      db, node_id, orb, opt.min_depth, opt.max_depth);
  if (!is_orb_descriptor_block(f.descriptors)) {
    core::warn("LoopClosure: node {} produced {}x{} descriptors of type {} "
               "(expected Nx{} CV_8U); skipping frame",
               node_id, f.descriptors.rows, f.descriptors.cols,
               f.descriptors.type(), kOrbDescriptorBytes);
    f.descriptors.release();
  }
  return f;
}

/// 3D-3D rigid fit of src -> dst (both 3xN), returning the 4x4 transform T with
/// dst ~= T * src. Wraps Eigen::umeyama (Kabsch, no scaling).
Eigen::Matrix4d rigid_fit(const Eigen::Matrix3Xd &src,
                          const Eigen::Matrix3Xd &dst) {
  return Eigen::umeyama(src, dst, false);
}

} // namespace

// --- Test-visible internals (declared in LoopClosure.hpp, `detail`) --------

namespace detail {

/// RANSAC 3D-3D: estimate T with dst ~= T*src from putative correspondences.
/// Returns inlier indices (empty if the fit is too weak).
std::vector<int> ransac_rigid(const Eigen::Matrix3Xd &src,
                              const Eigen::Matrix3Xd &dst,
                              const LoopClosureOptions &opt, std::mt19937 &rng,
                              Eigen::Matrix4d &best_T) {
  const int n = static_cast<int>(src.cols());
  std::vector<int> best_inliers;
  if (n < 3)
    return best_inliers;
  const double thr2 = static_cast<double>(opt.ransac_inlier_dist) *
                      static_cast<double>(opt.ransac_inlier_dist);
  std::uniform_int_distribution<int> pick(0, n - 1);

  for (int it = 0; it < opt.ransac_iterations; ++it) {
    int a = pick(rng), b = pick(rng), c = pick(rng);
    if (a == b || b == c || a == c)
      continue;
    Eigen::Matrix3Xd s3(3, 3), d3(3, 3);
    s3.col(0) = src.col(a);
    s3.col(1) = src.col(b);
    s3.col(2) = src.col(c);
    d3.col(0) = dst.col(a);
    d3.col(1) = dst.col(b);
    d3.col(2) = dst.col(c);
    const Eigen::Matrix4d T = rigid_fit(s3, d3);
    if (!T.allFinite())
      continue;
    const Eigen::Matrix3d R = T.block<3, 3>(0, 0);
    const Eigen::Vector3d t = T.block<3, 1>(0, 3);
    std::vector<int> inliers;
    inliers.reserve(n);
    for (int k = 0; k < n; ++k) {
      const Eigen::Vector3d e = (R * src.col(k) + t) - dst.col(k);
      if (e.squaredNorm() <= thr2)
        inliers.push_back(k);
    }
    if (inliers.size() > best_inliers.size()) {
      best_inliers = std::move(inliers);
      best_T = T;
    }
  }

  // Refit on the full inlier set for a lower-variance estimate.
  if (static_cast<int>(best_inliers.size()) >= 3) {
    Eigen::Matrix3Xd s(3, best_inliers.size()), d(3, best_inliers.size());
    for (size_t k = 0; k < best_inliers.size(); ++k) {
      s.col(k) = src.col(best_inliers[k]);
      d.col(k) = dst.col(best_inliers[k]);
    }
    const Eigen::Matrix4d T = rigid_fit(s, d);
    if (T.allFinite())
      best_T = T;
  }
  return best_inliers;
}
} // namespace detail

namespace {

// Hamming distance between two kOrbDescriptorBytes-byte ORB descriptors.
// std::memcpy (not reinterpret_cast) is what makes the 64-bit popcount
// well-defined: cv::Mat rows carry no uint64_t alignment guarantee and
// type-punning through a pointer cast is UB.
inline int hamming32(const uchar *a, const uchar *b) {
  std::uint64_t x[4], y[4];
  std::memcpy(x, a, sizeof(x));
  std::memcpy(y, b, sizeof(y));
  static_assert(sizeof(x) == kOrbDescriptorBytes,
                "hamming32 assumes 32-byte descriptors");
  int d = 0;
  for (int k = 0; k < 4; ++k)
    d += __builtin_popcountll(x[k] ^ y[k]);
  return d;
}

/// Build a binary visual vocabulary (k-majority, the Hamming-space analogue of
/// k-means) from a random sample of the frames' ORB descriptors. Returns a
/// K x kOrbDescriptorBytes CV_8U matrix of word centres. K may shrink if too
/// few samples exist.
cv::Mat build_vocabulary(const std::vector<pano_detail::FrameFeatures> &feats,
                         const LoopClosureOptions &opt, std::mt19937 &rng) {
  std::vector<cv::Mat> sample; // each row one descriptor (1 x 32 CV_8U)
  for (const auto &f : feats) {
    if (f.descriptors.empty())
      continue;
    const int rows = f.descriptors.rows;
    const int take = std::min(rows, opt.vocab_sample_per_frame);
    std::vector<int> idx(rows);
    for (int r = 0; r < rows; ++r)
      idx[r] = r;
    std::shuffle(idx.begin(), idx.end(), rng);
    for (int t = 0; t < take; ++t)
      sample.push_back(f.descriptors.row(idx[t]));
  }
  int K = std::min<int>(opt.vocab_size, static_cast<int>(sample.size()));
  if (K < 2)
    return cv::Mat();
  cv::Mat samples(static_cast<int>(sample.size()), kOrbDescriptorBytes, CV_8U);
  for (size_t s = 0; s < sample.size(); ++s)
    sample[s].copyTo(samples.row(static_cast<int>(s)));

  // Init centres from distinct random samples.
  std::vector<int> order(samples.rows);
  for (int r = 0; r < samples.rows; ++r)
    order[r] = r;
  std::shuffle(order.begin(), order.end(), rng);
  cv::Mat vocab(K, kOrbDescriptorBytes, CV_8U);
  for (int c = 0; c < K; ++c)
    samples.row(order[c]).copyTo(vocab.row(c));

  std::vector<int> assign(samples.rows, 0);
  for (int it = 0; it < std::max(1, opt.vocab_iterations); ++it) {
    // Assign each sample to the nearest word.
    for (int s = 0; s < samples.rows; ++s) {
      const uchar *sp = samples.ptr<uchar>(s);
      int best = 0, bestd = 1 << 30;
      for (int c = 0; c < K; ++c) {
        const int d = hamming32(sp, vocab.ptr<uchar>(c));
        if (d < bestd) {
          bestd = d;
          best = c;
        }
      }
      assign[s] = best;
    }
    // Recompute each word by per-bit majority vote of its members.
    std::vector<std::array<int, 256>> ones(K);
    std::vector<int> cnt(K, 0);
    for (auto &a : ones)
      a.fill(0);
    for (int s = 0; s < samples.rows; ++s) {
      const int c = assign[s];
      ++cnt[c];
      const uchar *sp = samples.ptr<uchar>(s);
      for (int byte = 0; byte < kOrbDescriptorBytes; ++byte)
        for (int bit = 0; bit < 8; ++bit)
          if (sp[byte] & (1 << bit))
            ++ones[c][byte * 8 + bit];
    }
    for (int c = 0; c < K; ++c) {
      if (cnt[c] == 0)
        continue; // keep an empty word as-is
      uchar *vp = vocab.ptr<uchar>(c);
      for (int byte = 0; byte < kOrbDescriptorBytes; ++byte) {
        uchar v = 0;
        for (int bit = 0; bit < 8; ++bit)
          if (2 * ones[c][byte * 8 + bit] > cnt[c])
            v |= (1 << bit);
        vp[byte] = v;
      }
    }
  }
  return vocab;
}

/// TF-IDF, L2-normalised bag-of-words histogram per frame over `vocab`.
std::vector<std::vector<float>>
bow_histograms(const std::vector<pano_detail::FrameFeatures> &feats,
               const cv::Mat &vocab) {
  const int N = static_cast<int>(feats.size());
  const int K = vocab.rows;
  std::vector<std::vector<float>> hist(N, std::vector<float>(K, 0.0f));
  std::vector<int> df(K, 0); // document frequency per word
  for (int i = 0; i < N; ++i) {
    const auto &d = feats[i].descriptors;
    std::vector<char> seen(K, 0);
    for (int r = 0; r < d.rows; ++r) {
      const uchar *dp = d.ptr<uchar>(r);
      int best = 0, bestd = 1 << 30;
      for (int c = 0; c < K; ++c) {
        const int hd = hamming32(dp, vocab.ptr<uchar>(c));
        if (hd < bestd) {
          bestd = hd;
          best = c;
        }
      }
      hist[i][best] += 1.0f;
      seen[best] = 1;
    }
    for (int c = 0; c < K; ++c)
      if (seen[c])
        ++df[c];
  }
  std::vector<float> idf(K, 0.0f);
  for (int c = 0; c < K; ++c)
    idf[c] = std::log((static_cast<float>(N) + 1.0f) /
                      (static_cast<float>(df[c]) + 1.0f));
  for (int i = 0; i < N; ++i) {
    double norm = 0.0;
    for (int c = 0; c < K; ++c) {
      hist[i][c] *= idf[c];
      norm += static_cast<double>(hist[i][c]) * hist[i][c];
    }
    norm = std::sqrt(norm);
    if (norm > 1e-9)
      for (int c = 0; c < K; ++c)
        hist[i][c] /= static_cast<float>(norm);
  }
  return hist;
}

} // namespace

namespace detail {

/// Two loop edges are consistent when chaining them through the seed
/// odometry closes a near-identity cycle (Mangelson et al., PCM).
static bool pcm_consistent(const LoopEdge &a, const LoopEdge &b,
                           const std::vector<Eigen::Matrix4d> &seed,
                           const LoopClosureOptions &opt) {
  // Cycle: pose(a.i) --T_a--> pose(a.j) --seed odom--> pose(b.j) --T_b^-1-->
  //        pose(b.i) --seed odom--> pose(a.i). Identity if consistent.
  const Eigen::Matrix4d C = a.T_ij * (seed[a.j].inverse() * seed[b.j]) *
                            b.T_ij.inverse() *
                            (seed[b.i].inverse() * seed[a.i]);
  const double t = C.block<3, 1>(0, 3).norm();
  const double c = (C.block<3, 3>(0, 0).trace() - 1.0) / 2.0;
  const double r = std::acos(std::clamp(c, -1.0, 1.0));
  return t < opt.pcm_trans_threshold && r < opt.pcm_rot_threshold;
}

/// Pairwise Consistency Maximization (Mangelson et al. 2018). Two loop edges
/// are "consistent" if chaining edge A, the seed odometry between the two
/// edges' endpoints, edge B (inverse), and the odometry back forms a
/// near-identity cycle — true loops satisfy this even under drift; aliasing
/// false positives do not. Returns the largest mutually-consistent subset via a
/// greedy max-clique on the consistency graph (exact max-clique is NP-hard;
/// greedy from the highest-degree seeds is the standard RPGO approximation).
std::vector<LoopEdge> pcm_filter(std::vector<LoopEdge> edges,
                                 const std::vector<Eigen::Matrix4d> &seed,
                                 const LoopClosureOptions &opt) {
  // Cap to the strongest edges before the O(M^2) test.
  if (opt.pcm_max_edges > 0 &&
      static_cast<int>(edges.size()) > opt.pcm_max_edges) {
    std::sort(edges.begin(), edges.end(),
              [](const LoopEdge &a, const LoopEdge &b) {
                return a.inliers > b.inliers;
              });
    edges.resize(opt.pcm_max_edges);
  }
  const int M = static_cast<int>(edges.size());
  if (M <= 2)
    return edges;

  auto consistent = [&](const LoopEdge &a, const LoopEdge &b) {
    return pcm_consistent(a, b, seed, opt);
  };

  std::vector<std::vector<char>> adj(M, std::vector<char>(M, 0));
  std::vector<int> deg(M, 0);
  for (int a = 0; a < M; ++a)
    for (int b = a + 1; b < M; ++b)
      if (consistent(edges[a], edges[b])) {
        adj[a][b] = adj[b][a] = 1;
        ++deg[a];
        ++deg[b];
      }

  std::vector<int> order(M);
  for (int i = 0; i < M; ++i)
    order[i] = i;
  std::sort(order.begin(), order.end(),
            [&](int a, int b) { return deg[a] > deg[b]; });

  // Greedy max-clique from the top-degree seeds.
  std::vector<int> best;
  const int seeds = std::min(M, 64);
  for (int s = 0; s < seeds; ++s) {
    const int seed_v = order[s];
    std::vector<int> clique{seed_v};
    for (int idx = 0; idx < M; ++idx) {
      const int v = order[idx];
      if (v == seed_v)
        continue;
      bool ok = true;
      for (int u : clique)
        if (!adj[v][u]) {
          ok = false;
          break;
        }
      if (ok)
        clique.push_back(v);
    }
    if (clique.size() > best.size())
      best = std::move(clique);
  }

  std::vector<LoopEdge> kept;
  kept.reserve(best.size());
  for (int v : best)
    kept.push_back(edges[v]);
  return kept;
}

} // namespace detail

std::vector<LoopEdge>
filter_consistent_loop_edges(std::vector<LoopEdge> edges,
                             const std::vector<Eigen::Matrix4d> &seed,
                             const LoopClosureOptions &options) {
  return detail::pcm_filter(std::move(edges), seed, options);
}

std::vector<size_t>
consistent_loop_edge_indices(const std::vector<LoopEdge> &edges,
                             const std::vector<Eigen::Matrix4d> &seed,
                             const LoopClosureOptions &opt,
                             const std::vector<bool> &fixed_in) {
  std::vector<bool> fixed = fixed_in;
  fixed.resize(edges.size(), false);

  // Indices may come from a database (rux graph discover); an out-of-range
  // one would read past @p seed inside pcm_consistent.
  const auto n_seed = static_cast<long long>(seed.size());
  for (size_t k = 0; k < edges.size(); ++k) {
    const auto &e = edges[k];
    if (e.i < 0 || e.j < 0 || e.i >= n_seed || e.j >= n_seed)
      throw std::invalid_argument(fmt::format(
          "consistent_loop_edge_indices: edge {} addresses frame index "
          "({}, {}) outside the {} seed poses",
          k, e.i, e.j, seed.size()));
  }

  // Fixed first, then by inlier support; stable so ties keep input order.
  std::vector<size_t> order(edges.size());
  std::iota(order.begin(), order.end(), size_t{0});
  std::stable_sort(order.begin(), order.end(), [&](size_t a, size_t b) {
    if (fixed[a] != fixed[b])
      return static_cast<bool>(fixed[a]);
    return edges[a].inliers > edges[b].inliers;
  });
  const size_t n_fixed =
      static_cast<size_t>(std::count(fixed.begin(), fixed.end(), true));
  if (opt.pcm_max_edges > 0) {
    // The cap counts candidates only: fixed edges come on top of it.
    const size_t cap = static_cast<size_t>(opt.pcm_max_edges) + n_fixed;
    if (order.size() > cap)
      order.resize(cap);
  }
  const int M = static_cast<int>(order.size());

  std::vector<std::vector<char>> adj(M, std::vector<char>(M, 0));
  for (int a = 0; a < M; ++a)
    for (int b = a + 1; b < M; ++b)
      if (detail::pcm_consistent(edges[order[a]], edges[order[b]], seed, opt))
        adj[a][b] = adj[b][a] = 1;

  // Mutually inconsistent confirmed edges: all are kept (the user's call),
  // but no candidate can agree with every one of them.
  int fixed_conflicts = 0;
  for (size_t a = 0; a < n_fixed; ++a)
    for (size_t b = a + 1; b < n_fixed; ++b)
      if (!adj[a][b])
        ++fixed_conflicts;
  if (fixed_conflicts > 0)
    core::warn("PCM: {} pairs of confirmed loop edges are mutually "
               "inconsistent (of {} confirmed); every new candidate that "
               "contradicts one of them is rejected. Review or reject one of "
               "the confirmed edges in Rediger / `rux graph prune --pair`",
               fixed_conflicts, n_fixed);

  // Fixed-edge filter BEFORE the small-set shortcut (ruling R9): a single
  // candidate that contradicts a confirmed edge must still be rejected.
  std::vector<int> fixed_v, cand;
  for (int v = 0; v < M; ++v) {
    if (fixed[order[v]]) {
      fixed_v.push_back(v);
      continue;
    }
    // fixed_v is complete here: fixed edges sort first.
    bool ok = true;
    for (int f : fixed_v)
      ok = ok && adj[v][f];
    if (ok)
      cand.push_back(v);
  }

  std::vector<int> best;
  if (M <= 2) {
    // A pair cannot outvote itself: keep every surviving candidate.
    best = cand;
  } else {
    std::vector<int> deg(M, 0);
    for (int a : cand)
      for (int b : cand)
        if (a != b && adj[a][b])
          ++deg[a];
    std::stable_sort(cand.begin(), cand.end(),
                     [&](int a, int b) { return deg[a] > deg[b]; });

    const int seeds = std::min(static_cast<int>(cand.size()), 64);
    for (int s = 0; s < seeds; ++s) {
      std::vector<int> clique{cand[static_cast<size_t>(s)]};
      for (int v : cand) {
        if (v == clique.front())
          continue;
        bool ok = true;
        for (int u : clique)
          if (!adj[v][u]) {
            ok = false;
            break;
          }
        if (ok)
          clique.push_back(v);
      }
      if (clique.size() > best.size())
        best = std::move(clique);
    }
  }

  std::vector<size_t> out;
  out.reserve(fixed_v.size() + best.size());
  for (int v : fixed_v)
    out.push_back(order[static_cast<size_t>(v)]);
  for (int v : best)
    out.push_back(order[static_cast<size_t>(v)]);
  std::sort(out.begin(), out.end());
  return out;
}

double trajectory_extent(const std::vector<Eigen::Matrix4d> &poses) {
  if (poses.size() < 2)
    return 0.0;
  Eigen::Vector3d lo = poses.front().block<3, 1>(0, 3);
  Eigen::Vector3d hi = lo;
  for (const auto &P : poses) {
    const Eigen::Vector3d c = P.block<3, 1>(0, 3);
    lo = lo.cwiseMin(c);
    hi = hi.cwiseMax(c);
  }
  return (hi - lo).norm();
}

double seed_disagreement_gate(double extent, double fraction, double floor_m) {
  const double relative =
      (fraction > 0.0 && extent > 0.0) ? fraction * extent : 0.0;
  return std::max(relative, std::max(floor_m, 0.0));
}

std::vector<LoopEdge>
detect_loop_edges(ProjectDB &db, const std::vector<int> &node_ids,
                  const std::vector<Eigen::Matrix4d> &seed_poses,
                  const LoopClosureOptions &opt,
                  LoopClosureResult *out_result) {
  std::vector<LoopEdge> edges;
  const int N = static_cast<int>(node_ids.size());
  if (N < 2 || static_cast<int>(seed_poses.size()) != N)
    return edges;

  // Scale-relative lower gate (#339): resolved ONCE from this capture's own
  // extent so every candidate is judged against the same threshold, and logged
  // because a gate that silently changes with the input is the kind of thing
  // §9.4 spent a whole experiment mis-attributing to the matcher.
  const double extent = trajectory_extent(seed_poses);
  const double min_gate = seed_disagreement_gate(
      extent, opt.min_seed_disagreement_fraction, opt.min_seed_disagreement);
  core::info("LoopClosure: trajectory extent {:.2f} m -> seed-disagreement "
             "gate {:.3f} m (fraction {:.5f}, floor {:.3f} m)",
             extent, min_gate, opt.min_seed_disagreement_fraction,
             opt.min_seed_disagreement);

  // --- 1. Per-frame features (cached) -------------------------------------
  auto orb = cv::ORB::create(opt.max_features);
  std::vector<pano_detail::FrameFeatures> feats(N);
  for (int i = 0; i < N; ++i)
    feats[i] = extract_features(db, node_ids[i], opt, orb);
  core::info("LoopClosure: extracted ORB features for {} frames", N);

  // --- 2. Candidate proposal --------------------------------------------
  // Dedup as ordered (i<j) pairs regardless of proposal mode.
  std::mt19937 rng(opt.seed);
  std::set<std::pair<int, int>> candidate_set;
  const int gap = std::max(1, opt.min_frame_gap);
  const int K = std::max(1, opt.max_candidates_per_frame);

  // Resolve `automatic`: exhaustive when the temporally-distant pair count fits
  // the budget (small scans; the reliable choice), else appearance retrieval.
  LoopProposal mode = opt.proposal;
  if (mode == LoopProposal::automatic) {
    long pairs = 0;
    for (int i = 0; i < N; ++i)
      pairs += std::max(0, N - (i + gap));
    mode = (pairs <= opt.exhaustive_budget) ? LoopProposal::exhaustive
                                            : LoopProposal::appearance;
    core::info("LoopClosure: automatic proposal -> {} ({} candidate pairs vs "
               "budget {})",
               mode == LoopProposal::exhaustive ? "exhaustive" : "appearance",
               pairs, opt.exhaustive_budget);
  }

  if (mode == LoopProposal::spatial) {
    // Spatial shortlist from the seed poses (nearest K per frame). Blind to
    // drift-separated loops — see LoopProposal::spatial.
    std::vector<Eigen::Vector3d> center(N), viewdir(N);
    for (int i = 0; i < N; ++i) {
      center[i] = seed_poses[i].block<3, 1>(0, 3);
      viewdir[i] = seed_poses[i].block<3, 3>(0, 0) * Eigen::Vector3d(0, 0, 1);
    }
    const double cos_view = std::cos(opt.max_view_angle * M_PI / 180.0);
    const double max_d2 = static_cast<double>(opt.max_candidate_distance) *
                          static_cast<double>(opt.max_candidate_distance);
    for (int i = 0; i < N; ++i) {
      std::vector<std::pair<double, int>> near;
      for (int j = 0; j < N; ++j) {
        if (std::abs(i - j) < gap)
          continue;
        const double d2 = (center[i] - center[j]).squaredNorm();
        if (d2 > max_d2 || viewdir[i].dot(viewdir[j]) < cos_view)
          continue;
        near.emplace_back(d2, j);
      }
      std::sort(near.begin(), near.end());
      for (int k = 0; k < std::min<int>(near.size(), K); ++k)
        candidate_set.emplace(std::min(i, near[k].second),
                              std::max(i, near[k].second));
    }
    core::info("LoopClosure: {} spatial candidates proposed",
               candidate_set.size());
  } else if (mode == LoopProposal::exhaustive) {
    for (int i = 0; i < N; ++i)
      for (int j = i + gap; j < N; ++j)
        candidate_set.emplace(i, j);
    core::info("LoopClosure: {} exhaustive candidates proposed (all pairs with "
               "index gap >= {})",
               candidate_set.size(), gap);
  } else {
    // Appearance shortlist: pose-INDEPENDENT bag-of-words retrieval. This is
    // what finds the start/end revisit that trajectory drift has pulled apart.
    const cv::Mat vocab = build_vocabulary(feats, opt, rng);
    if (vocab.empty()) {
      core::warn("LoopClosure: could not build a visual vocabulary (too few "
                 "features); no appearance candidates");
    } else {
      const auto hist = bow_histograms(feats, vocab);
      for (int i = 0; i < N; ++i) {
        std::vector<std::pair<float, int>> best; // (-similarity, j)
        for (int j = 0; j < N; ++j) {
          if (std::abs(i - j) < gap || feats[j].descriptors.empty())
            continue;
          float sim = 0.0f;
          for (int c = 0; c < vocab.rows; ++c)
            sim += hist[i][c] * hist[j][c];
          best.emplace_back(-sim, j);
        }
        std::sort(best.begin(), best.end());
        for (int k = 0; k < std::min<int>(best.size(), K); ++k)
          candidate_set.emplace(std::min(i, best[k].second),
                                std::max(i, best[k].second));
      }
      core::info("LoopClosure: {} appearance candidates proposed "
                 "(bag-of-words, {} words)",
                 candidate_set.size(), vocab.rows);
    }
  }
  std::vector<std::pair<int, int>> candidates(candidate_set.begin(),
                                              candidate_set.end());

  // --- 3. Match + robust relative pose per candidate ----------------------
  cv::BFMatcher matcher(cv::NORM_HAMMING);
  int matched = 0, total_inliers = 0;
  int best_putative_seen = 0, best_inliers_seen = 0; // loop-strength ceiling

  for (const auto &cand : candidates) {
    const int i = cand.first, j = cand.second;
    const auto &fi = feats[i];
    const auto &fj = feats[j];
    if (fi.descriptors.empty() || fj.descriptors.empty())
      continue;
    ++matched;

    std::vector<std::vector<cv::DMatch>> knn;
    matcher.knnMatch(fj.descriptors, fi.descriptors, knn,
                     2); // query=j, train=i

    // Lowe ratio test + require valid depth on both sides.
    std::vector<Eigen::Vector3d> ps, pd; // src = j, dst = i
    ps.reserve(knn.size());
    pd.reserve(knn.size());
    for (const auto &m : knn) {
      if (m.size() < 2)
        continue;
      if (m[0].distance > opt.ratio_test * m[1].distance)
        continue;
      const int qj = m[0].queryIdx; // index into frame j
      const int ti = m[0].trainIdx; // index into frame i
      if (!fj.valid[qj] || !fi.valid[ti])
        continue;
      ps.push_back(fj.local[qj]);
      pd.push_back(fi.local[ti]);
    }
    best_putative_seen =
        std::max(best_putative_seen, static_cast<int>(ps.size()));
    if (static_cast<int>(ps.size()) < 4)
      continue; // too few even to fit a rigid transform

    Eigen::Matrix3Xd src(3, ps.size()), dst(3, pd.size());
    for (size_t k = 0; k < ps.size(); ++k) {
      src.col(k) = ps[k];
      dst.col(k) = pd[k];
    }
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    const std::vector<int> inliers =
        detail::ransac_rigid(src, dst, opt, rng, T);
    best_inliers_seen =
        std::max(best_inliers_seen, static_cast<int>(inliers.size()));
    if (static_cast<int>(inliers.size()) < opt.min_match_inliers)
      continue;

    // Gates against the seed relative pose.
    const Eigen::Matrix4d seed_rel = seed_poses[i].inverse() * seed_poses[j];
    const double dt = (T.block<3, 1>(0, 3) - seed_rel.block<3, 1>(0, 3)).norm();
    // LOWER gate: drop edges that already agree with the seed — they carry no
    // drift-correction information and only re-impose ORB+depth noise on
    // already-correct poses (keeps loop closure a no-op on well-aligned scans).
    if (dt < min_gate)
      continue;
    // Optional UPPER gate: coarse guard against gross mismatches (off by
    // default; GNC/PCM handle outliers).
    if (opt.max_seed_disagreement > 0.0f && dt > opt.max_seed_disagreement)
      continue;

    LoopEdge e;
    e.i = i;
    e.j = j;
    e.T_ij = T;
    e.inliers = static_cast<int>(inliers.size());
    const double s = std::sqrt(static_cast<double>(opt.min_match_inliers) /
                               std::max(1, e.inliers));
    e.sigma_trans = std::max(static_cast<double>(opt.min_sigma_trans),
                             opt.base_sigma_trans * s);
    e.sigma_rot = std::max(static_cast<double>(opt.min_sigma_rot),
                           opt.base_sigma_rot * s);
    edges.push_back(std::move(e));
    total_inliers += static_cast<int>(inliers.size());
  }

  // Reject perceptual-aliasing false positives: keep only the largest mutually
  // consistent set of loop edges (PCM). This is what makes the edges safe to
  // trust on scans with repeated structure.
  if (opt.pcm && edges.size() > 2) {
    const size_t before = edges.size();
    edges = detail::pcm_filter(std::move(edges), seed_poses, opt);
    total_inliers = 0;
    for (const auto &e : edges)
      total_inliers += e.inliers;
    core::info("LoopClosure: PCM kept {} of {} edges as mutually consistent",
               edges.size(), before);
  }

  // Diagnostics: how far do accepted edges reach (frame gap), and how much do
  // they disagree with the seed poses (the drift-correction signal)?
  int max_gap = 0, drift_edges = 0;
  double max_disagreement = 0.0;
  for (const auto &e : edges) {
    max_gap = std::max(max_gap, std::abs(e.j - e.i));
    const Eigen::Matrix4d seed_rel =
        seed_poses[e.i].inverse() * seed_poses[e.j];
    const double dt =
        (e.T_ij.block<3, 1>(0, 3) - seed_rel.block<3, 1>(0, 3)).norm();
    max_disagreement = std::max(max_disagreement, dt);
    if (dt > 0.15)
      ++drift_edges;
  }

  core::info("LoopClosure: {} candidates matched, {} loop edges accepted "
             "({} total inliers)",
             matched, edges.size(), total_inliers);
  core::info("LoopClosure: strongest pair seen had {} putative depth-valid "
             "matches and {} RANSAC inliers (acceptance threshold {})",
             best_putative_seen, best_inliers_seen, opt.min_match_inliers);
  core::info("LoopClosure: accepted edges span max frame-gap {}, max seed "
             "disagreement {:.3f} m, {} edges correct >0.15 m of drift",
             max_gap, max_disagreement, drift_edges);
  if (out_result) {
    out_result->candidates = matched;
    out_result->edges = static_cast<int>(edges.size());
    out_result->total_inliers = total_inliers;
  }
  return edges;
}

} // namespace reusex::geometry
