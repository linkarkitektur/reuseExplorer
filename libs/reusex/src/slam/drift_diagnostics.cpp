// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Drift diagnostics (reusex-rcv.1.1): the C++ port of the research
// prototypes of docs/research/visual-drift-loop.md §3 (revisit.py / floor.py
// for the pass-to-pass residual and its noise floor, sens2.py for the segment
// matrix, halves.py + icp.py for the segment suggestions, crossings.py for
// the through-wall crossings, ghost2.py / ghost3.py for the doubled walls).
// The numbers they produced are the acceptance targets of the bead.

#include "slam/drift_diagnostics.hpp"

#include "core/ProjectDB.hpp"
#include "core/logging.hpp"
#include "core/solid_model.hpp"
#include "slam/frame_pair_icp.hpp"

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <Eigen/LU>

#include <nlohmann/json.hpp>

// Equal distances return the lower index first (STANDARDS §6).
#define NANOFLANN_FIRST_MATCH
#include <nanoflann.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace reusex::slam {
namespace {

using RowMajor4d = Eigen::Matrix<double, 4, 4, Eigen::RowMajor>;
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

/// Ridge added to the ICP normal equations (the prototype's 1e-9 * I): keeps
/// a direction no surface constrains from blowing up the solve.
constexpr double kIcpRidge = 1e-9;
/// ICP stops when the update's norm (rad + m) falls below this.
constexpr double kIcpConverged = 1e-6;
/// Chunks the ICP normal equations are summed in (fixed, see segment_icp).
constexpr std::size_t kIcpChunks = 64;
/// Gaussian smoothing of the doubled-wall offset histogram, in bins
/// (scipy.ndimage.gaussian_filter1d(sigma=1, truncate=4)).
constexpr double kHistSmoothSigmaBins = 1.0;
constexpr int kHistSmoothRadiusBins = 4;
/// k-d tree leaf size (nanoflann's default) and build threads. A threaded
/// build makes the same splits as a serial one (the tree is a function of
/// the points), it only builds the subtrees concurrently.
constexpr std::size_t kTreeLeafSize = 10;
constexpr unsigned int kTreeBuildThreads = 8;

Eigen::Matrix4d to_matrix(const std::array<double, 16> &a) {
  return Eigen::Matrix4d(Eigen::Map<const RowMajor4d>(a.data()));
}

/// numpy.percentile(v, q) with linear interpolation; nullopt when empty.
/// Reorders @p v.
std::optional<double> percentile(std::vector<double> &v, double q) {
  if (v.empty())
    return std::nullopt;
  std::sort(v.begin(), v.end());
  const double pos = q / 100.0 * static_cast<double>(v.size() - 1);
  const auto lo = static_cast<std::size_t>(std::floor(pos));
  const auto hi = std::min(lo + 1, v.size() - 1);
  const double f = pos - static_cast<double>(lo);
  return v[lo] + (v[hi] - v[lo]) * f;
}

std::optional<double> median(std::vector<double> &v) {
  return percentile(v, 50.0);
}

double rotation_deg_of(const Eigen::Matrix3d &r) {
  const double tr = r.trace();
  return std::acos(std::clamp((tr - 1.0) / 2.0, -1.0, 1.0)) * 180.0 / M_PI;
}

PointCloud::Ptr make_cloud(std::size_t n) {
  auto c = std::make_shared<PointCloud>();
  c->resize(n);
  c->width = static_cast<std::uint32_t>(n);
  c->height = 1;
  c->is_dense = true;
  return c;
}

/// nanoflann view of a PointCloud.
struct CloudAdaptor {
  const PointCloud *cloud = nullptr;
  [[nodiscard]] std::size_t kdtree_get_point_count() const {
    return cloud->size();
  }
  [[nodiscard]] float kdtree_get_pt(std::size_t i, std::size_t d) const {
    return (*cloud)[i].data[d];
  }
  template <class Bbox> bool kdtree_get_bbox(Bbox & /*bb*/) const {
    return false;
  }
};

/// Exact k-d tree over a PointCloud. nanoflann rather than PCL's FLANN
/// wrapper: the latter allocates on every query, and the diagnostics make
/// tens of millions of them from all threads at once.
class Tree {
    public:
  explicit Tree(PointCloud::Ptr cloud)
      : cloud_(std::move(cloud)), adaptor_{cloud_.get()},
        index_(3, adaptor_,
               nanoflann::KDTreeSingleIndexAdaptorParams(
                   kTreeLeafSize,
                   nanoflann::KDTreeSingleIndexAdaptorFlags::None,
                   kTreeBuildThreads)) {}

  /// The @p k nearest points within @p radius (all when radius <= 0),
  /// ascending; returns how many. Thread-safe, allocation-free after the
  /// first call on a thread.
  std::size_t knn(const pcl::PointXYZ &p, int k, double radius,
                  std::vector<std::uint32_t> &idx,
                  std::vector<float> &d2) const {
    const auto kk = static_cast<std::size_t>(std::min<std::size_t>(
        static_cast<std::size_t>(std::max(k, 0)), cloud_->size()));
    idx.resize(kk);
    d2.resize(kk);
    if (kk == 0)
      return 0;
    std::size_t n = 0;
    if (radius > 0) {
      nanoflann::RKNNResultSet<float, std::uint32_t, std::size_t> rs(
          kk, static_cast<float>(radius * radius));
      rs.init(idx.data(), d2.data());
      index_.findNeighbors(rs, p.data);
      n = rs.size();
    } else {
      nanoflann::KNNResultSet<float, std::uint32_t, std::size_t> rs(kk);
      rs.init(idx.data(), d2.data());
      index_.findNeighbors(rs, p.data);
      n = rs.size();
    }
    idx.resize(n);
    d2.resize(n);
    return n;
  }

    private:
  using Index = nanoflann::KDTreeSingleIndexAdaptor<
      nanoflann::L2_Simple_Adaptor<float, CloudAdaptor>, CloudAdaptor, 3,
      std::uint32_t>;
  PointCloud::Ptr cloud_;
  CloudAdaptor adaptor_;
  Index index_;
};

std::unique_ptr<Tree> make_tree(const PointCloud::Ptr &cloud) {
  return std::make_unique<Tree>(cloud);
}

/// PCA surface normal of point @p i of @p cloud from its @p k nearest
/// neighbours (itself included), as the prototype's normals(); NaN where
/// fewer than 3 neighbours exist. The sign is arbitrary.
Eigen::Vector3f normal_at(const PointCloud &cloud, const Tree &tree, int k,
                          std::size_t i) {
  // Per-thread scratch: a query must not allocate (millions of them).
  thread_local std::vector<std::uint32_t> idx;
  thread_local std::vector<float> d2;
  if (tree.knn(cloud[i], k, 0.0, idx, d2) < 3)
    return Eigen::Vector3f::Constant(NAN);
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  for (const auto j : idx)
    mean += cloud[static_cast<std::size_t>(j)].getVector3fMap().cast<double>();
  mean /= static_cast<double>(idx.size());
  Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
  for (const auto j : idx) {
    const Eigen::Vector3d d =
        cloud[static_cast<std::size_t>(j)].getVector3fMap().cast<double>() -
        mean;
    cov.noalias() += d * d.transpose();
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(cov);
  return eig.eigenvectors().col(0).cast<float>();
}

std::vector<Eigen::Vector3f> pca_normals(const PointCloud &cloud,
                                         const Tree &tree, int k) {
  const auto n = static_cast<std::int64_t>(cloud.size());
  std::vector<Eigen::Vector3f> out(cloud.size());
#pragma omp parallel for schedule(static)
  for (std::int64_t i = 0; i < n; ++i)
    out[static_cast<std::size_t>(i)] =
        normal_at(cloud, tree, k, static_cast<std::size_t>(i));
  return out;
}

/// Nearest neighbour of @p p in @p tree within @p radius: its index and
/// squared distance, or -1 when there is none. Bounded by the radius (the
/// prototype's distance_upper_bound): an unbounded exact search from a
/// point far from the target visits most of the tree. Allocation-free.
std::pair<int, float> nearest(const Tree &tree, const pcl::PointXYZ &p,
                              double radius) {
  thread_local std::vector<std::uint32_t> idx;
  thread_local std::vector<float> d2;
  if (tree.knn(p, 1, radius, idx, d2) < 1)
    return {-1, 0.0f};
  return {static_cast<int>(idx[0]), d2[0]};
}

/// The back-projected depth of the sampled frames, pooled.
struct Pooled {
  PointCloud::Ptr cloud = make_cloud(0);
  std::vector<int> frame;               ///< sampled-frame index per point
  std::vector<Eigen::Vector3d> centres; ///< pose-frame origin per frame
};

/// Index-aligned with the sampled frames.
struct SampledFrame {
  std::int64_t node_id = 0;
  std::size_t order = 0; ///< index in db.sensor_frame_ids()
  double time = 0.0;     ///< timestamp, or the node-order index
  std::optional<Eigen::Matrix4d> pose;
  std::size_t points = 0;
  int segment = 0;
};

/// residual of @p p against point @p q of @p cloud with normal @p n.
double plane_residual(const pcl::PointXYZ &p, const pcl::PointXYZ &q,
                      const Eigen::Vector3f &n) {
  return std::abs(
      static_cast<double>((p.getVector3fMap() - q.getVector3fMap()).dot(n)));
}

/// A target cloud of the segment matching (sens2.py / halves.py): its tree,
/// and its normals computed on first use (each one depends only on the
/// cloud, so laziness does not change a result; it skips the many points no
/// query ever reaches).
struct SegmentCloud {
  PointCloud::Ptr cloud;
  std::unique_ptr<Tree> tree;
  int k = 12;
  std::vector<Eigen::Vector3f> normals;
  std::vector<char> has_normal;

  /// Compute the normals of @p hits (indices into cloud, -1 ignored) that
  /// are not known yet.
  void ensure_normals(const std::vector<int> &hits) {
    std::vector<std::size_t> todo;
    for (const int h : hits)
      if (h >= 0 && !has_normal[static_cast<std::size_t>(h)]) {
        has_normal[static_cast<std::size_t>(h)] = 1;
        todo.push_back(static_cast<std::size_t>(h));
      }
    const auto n = static_cast<std::int64_t>(todo.size());
#pragma omp parallel for schedule(static)
    for (std::int64_t i = 0; i < n; ++i) {
      const auto j = todo[static_cast<std::size_t>(i)];
      normals[j] = normal_at(*cloud, *tree, k, j);
    }
  }
};

SegmentCloud segment_cloud(const PointCloud &all,
                           const std::vector<std::size_t> &idx, int k) {
  SegmentCloud s;
  s.k = k;
  s.cloud = make_cloud(idx.size());
  for (std::size_t i = 0; i < idx.size(); ++i)
    (*s.cloud)[i] = all[idx[i]];
  s.normals.assign(idx.size(), Eigen::Vector3f::Constant(NAN));
  s.has_normal.assign(idx.size(), 0);
  if (!idx.empty())
    s.tree = make_tree(s.cloud);
  return s;
}

/// Point-to-plane residuals of @p pts (moved by @p r, @p t about @p c)
/// against @p target within @p gate; NaN for a miss. Also the matched
/// target index (-1 for a miss).
void match(const std::vector<Eigen::Vector3d> &pts, const Eigen::Matrix3d &r,
           const Eigen::Vector3d &t, const Eigen::Vector3d &c,
           SegmentCloud &target, double gate, std::vector<double> &res,
           std::vector<int> &hit) {
  const auto n = static_cast<std::int64_t>(pts.size());
  res.assign(pts.size(), kNaN);
  hit.assign(pts.size(), -1);
  const double gate2 = gate * gate;
  std::vector<Eigen::Vector3d> moved(pts.size());
#pragma omp parallel for schedule(static)
  for (std::int64_t i = 0; i < n; ++i) {
    const auto ui = static_cast<std::size_t>(i);
    moved[ui] = r * pts[ui] + t + c;
    pcl::PointXYZ p;
    p.getVector3fMap() = moved[ui].cast<float>();
    const auto [j, d2] = nearest(*target.tree, p, gate);
    if (j >= 0 && static_cast<double>(d2) < gate2)
      hit[ui] = j;
  }
  target.ensure_normals(hit);
#pragma omp parallel for schedule(static)
  for (std::int64_t i = 0; i < n; ++i) {
    const auto ui = static_cast<std::size_t>(i);
    if (hit[ui] < 0)
      continue;
    const auto j = static_cast<std::size_t>(hit[ui]);
    const Eigen::Vector3f &nn = target.normals[j];
    if (!nn.allFinite()) {
      hit[ui] = -1;
      continue;
    }
    res[ui] = (moved[ui] - (*target.cloud)[j].getVector3fMap().cast<double>())
                  .dot(nn.cast<double>());
  }
}

std::vector<double> finite_abs(const std::vector<double> &v) {
  std::vector<double> out;
  out.reserve(v.size());
  for (const double x : v)
    if (std::isfinite(x))
      out.push_back(std::abs(x));
  return out;
}

/// halves.py + icp.py: robust point-to-plane ICP of @p pts onto @p target,
/// about the centroid of @p pts. Fills the suggestion's numbers.
void segment_icp(const std::vector<Eigen::Vector3d> &world_pts,
                 SegmentCloud &target, const DriftDiagnosticsOptions &o,
                 SegmentSuggestion &s) {
  Eigen::Vector3d c = Eigen::Vector3d::Zero();
  for (const auto &p : world_pts)
    c += p;
  c /= static_cast<double>(world_pts.size());
  std::vector<Eigen::Vector3d> pts(world_pts.size());
  for (std::size_t i = 0; i < pts.size(); ++i)
    pts[i] = world_pts[i] - c;

  Eigen::Matrix3d r = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t = Eigen::Vector3d::Zero();
  std::vector<double> res;
  std::vector<int> hit;
  match(pts, r, t, c, target, o.icp_max_corr_m, res, hit);
  {
    auto a = finite_abs(res);
    s.before_m = median(a).value_or(kNaN);
  }
  using Vector6d = Eigen::Matrix<double, 6, 1>;
  using Matrix6d = Eigen::Matrix<double, 6, 6>;
  int iterations = 0;
  for (int it = 0; it < o.icp_iterations; ++it) {
    ++iterations;
    if (it > 0)
      match(pts, r, t, c, target, o.icp_max_corr_m, res, hit);
    // Normal equations summed in a FIXED number of chunks, each in index
    // order, then the chunks in order: parallel, yet independent of the
    // thread count (STANDARDS §6).
    std::vector<Matrix6d> hc(kIcpChunks, Matrix6d::Zero());
    std::vector<Vector6d> gc(kIcpChunks, Vector6d::Zero());
    const std::size_t per = (pts.size() + kIcpChunks - 1) / kIcpChunks;
#pragma omp parallel for schedule(static)
    for (std::int64_t ch = 0; ch < static_cast<std::int64_t>(kIcpChunks);
         ++ch) {
      const auto uc = static_cast<std::size_t>(ch);
      Matrix6d hh = Matrix6d::Zero();
      Vector6d gg = Vector6d::Zero();
      for (std::size_t i = uc * per; i < std::min(pts.size(), (uc + 1) * per);
           ++i) {
        if (!std::isfinite(res[i]))
          continue;
        const Eigen::Vector3d a = r * pts[i] + t; // centred
        const Eigen::Vector3d n =
            target.normals[static_cast<std::size_t>(hit[i])].cast<double>();
        const double ri = res[i];
        const double w =
            std::abs(ri) < o.huber_m ? 1.0 : o.huber_m / std::abs(ri);
        Vector6d j;
        j.head<3>() = a.cross(n);
        j.tail<3>() = n;
        hh.noalias() += w * j * j.transpose();
        gg.noalias() += w * j * ri;
      }
      hc[uc] = hh;
      gc[uc] = gg;
    }
    Matrix6d h = Matrix6d::Zero();
    Vector6d g = Vector6d::Zero();
    for (std::size_t ch = 0; ch < kIcpChunks; ++ch) {
      h += hc[ch];
      g += gc[ch];
    }
    const Vector6d x = -(h + kIcpRidge * Matrix6d::Identity()).ldlt().solve(g);
    if (!x.allFinite())
      break;
    const Eigen::Vector3d w = x.head<3>();
    const double angle = w.norm();
    const Eigen::Matrix3d dr =
        angle > 0 ? Eigen::AngleAxisd(angle, w / angle).toRotationMatrix()
                  : Eigen::Matrix3d::Identity();
    r = dr * r;
    t = dr * t + x.tail<3>();
    if (x.norm() < kIcpConverged)
      break;
  }
  core::debug("analyze drift: segment {} ICP: {} iterations", s.segment,
              iterations);
  match(pts, r, t, c, target, o.icp_max_corr_m, res, hit);
  {
    auto a = finite_abs(res);
    s.after_m = median(a).value_or(kNaN);
  }
  // Centred x -> r x + t, so world p -> r p + (t + c - r c).
  s.delta = Eigen::Matrix4d::Identity();
  s.delta.topLeftCorner<3, 3>() = r;
  s.delta.topRightCorner<3, 1>() = t + c - r * c;
  s.shift_m = s.delta.topRightCorner<3, 1>().norm();
  s.centroid_shift_m = t.norm();
  s.rotation_deg = rotation_deg_of(r);
  s.yaw_deg = std::atan2(r(1, 0), r(0, 0)) * 180.0 / M_PI;
}

/// scipy.ndimage.gaussian_filter1d(h, sigma=1) with mode 'reflect'.
std::vector<double> smooth(const std::vector<double> &h) {
  const int n = static_cast<int>(h.size());
  std::array<double, 2 * kHistSmoothRadiusBins + 1> k{};
  double sum = 0;
  for (int i = -kHistSmoothRadiusBins; i <= kHistSmoothRadiusBins; ++i) {
    k[static_cast<std::size_t>(i + kHistSmoothRadiusBins)] =
        std::exp(-0.5 * i * i / (kHistSmoothSigmaBins * kHistSmoothSigmaBins));
    sum += k[static_cast<std::size_t>(i + kHistSmoothRadiusBins)];
  }
  auto at = [&](int i) {
    // reflect: (d c b a | a b c d | d c b a)
    while (i < 0 || i >= n)
      i = i < 0 ? -i - 1 : 2 * n - i - 1;
    return h[static_cast<std::size_t>(i)];
  };
  std::vector<double> out(h.size(), 0.0);
  for (int i = 0; i < n; ++i)
    for (int d = -kHistSmoothRadiusBins; d <= kHistSmoothRadiusBins; ++d)
      out[static_cast<std::size_t>(i)] +=
          k[static_cast<std::size_t>(d + kHistSmoothRadiusBins)] / sum *
          at(i + d);
  return out;
}

/// ghost2.py analyse(): the separation of the best two-layer split of a
/// patch's offsets along its mean normal, 0 when there is none.
double patch_separation(const std::vector<double> &offsets,
                        const DriftDiagnosticsOptions &o) {
  const int bins = static_cast<int>(
      std::lround(2.0 * o.doubled_hist_half_m / o.doubled_hist_bin_m));
  std::vector<double> h(static_cast<std::size_t>(bins), 0.0);
  for (const double x : offsets) {
    const double f = (x + o.doubled_hist_half_m) / o.doubled_hist_bin_m;
    if (f < 0 || f > bins)
      continue;
    h[static_cast<std::size_t>(std::min(bins - 1, static_cast<int>(f)))] += 1;
  }
  const auto hs = smooth(h);
  const double top = *std::max_element(hs.begin(), hs.end());
  std::vector<int> peaks;
  for (int i = 1; i + 1 < bins; ++i) {
    const auto u = static_cast<std::size_t>(i);
    if (hs[u] >= hs[u - 1] && hs[u] > hs[u + 1] &&
        hs[u] >= o.doubled_peak_ratio * top)
      peaks.push_back(i);
  }
  std::pair<double, double> best{0.0, 0.0}; // (smaller peak, separation)
  for (std::size_t a = 0; a < peaks.size(); ++a)
    for (std::size_t b = a + 1; b < peaks.size(); ++b) {
      const int i = peaks[a], j = peaks[b];
      const double sep = (j - i) * o.doubled_hist_bin_m;
      if (sep < o.doubled_min_separation_m)
        continue;
      const auto ui = static_cast<std::size_t>(i);
      const auto uj = static_cast<std::size_t>(j);
      const double valley =
          *std::min_element(hs.begin() + i, hs.begin() + j + 1);
      const double lower = std::min(hs[ui], hs[uj]);
      if (valley <= o.doubled_valley_ratio * lower)
        best = std::max(best, std::make_pair(lower, sep));
    }
  return best.second;
}

/// ghost3.py on the cloud fused from the poses under test, per storey band.
std::vector<DoubledWallStats>
doubled_walls(const Pooled &pooled, const std::vector<core::StoreyRecord> &st,
              const DriftDiagnosticsOptions &o) {
  std::vector<DoubledWallStats> out;
  if (st.empty() || pooled.cloud->empty())
    return out;
  // One point per voxel: the first in pooled order (np.unique return_index).
  std::map<std::tuple<std::int64_t, std::int64_t, std::int64_t>, std::size_t>
      first;
  for (std::size_t i = 0; i < pooled.cloud->size(); ++i) {
    const auto &p = (*pooled.cloud)[i];
    first.emplace(
        std::make_tuple(
            static_cast<std::int64_t>(std::floor(p.x / o.doubled_voxel_m)),
            static_cast<std::int64_t>(std::floor(p.y / o.doubled_voxel_m)),
            static_cast<std::int64_t>(std::floor(p.z / o.doubled_voxel_m))),
        i);
  }
  std::vector<std::size_t> keep;
  keep.reserve(first.size());
  for (const auto &kv : first)
    keep.push_back(kv.second);
  std::sort(keep.begin(), keep.end());
  auto fused = segment_cloud(*pooled.cloud, keep, o.normal_neighbours);
  fused.normals = pca_normals(*fused.cloud, *fused.tree, o.normal_neighbours);
  // Orient each normal towards the camera that saw the point, so the two
  // layers of a doubled wall seen from one side share an azimuth bin.
  std::vector<Eigen::Vector3d> normals(keep.size());
  for (std::size_t i = 0; i < keep.size(); ++i) {
    Eigen::Vector3d n = fused.normals[i].cast<double>();
    const Eigen::Vector3d view =
        pooled.centres[static_cast<std::size_t>(pooled.frame[keep[i]])] -
        (*fused.cloud)[i].getVector3fMap().cast<double>();
    if (n.allFinite() && n.dot(view) < 0)
      n = -n;
    normals[i] = n;
  }

  for (const auto &s : st) {
    const Eigen::Matrix4d l = to_matrix(s.level_transform);
    const double z0 = s.z_floor + o.doubled_band_lo_m;
    const double z1 = std::min(s.z_floor + o.doubled_band_hi_m,
                               s.z_ceiling - o.doubled_ceiling_clear_m);
    std::map<std::tuple<std::int64_t, std::int64_t, std::int64_t>,
             std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>>
        patches;
    for (std::size_t i = 0; i < keep.size(); ++i) {
      if (!normals[i].allFinite())
        continue;
      const Eigen::Vector3d p =
          l.topLeftCorner<3, 3>() *
              (*fused.cloud)[i].getVector3fMap().cast<double>() +
          l.topRightCorner<3, 1>();
      const Eigen::Vector3d n =
          (l.topLeftCorner<3, 3>() * normals[i]).normalized();
      if (!(p.z() > z0 && p.z() < z1) ||
          !(std::abs(n.z()) < o.doubled_max_abs_nz))
        continue;
      double az = std::atan2(n.y(), n.x()) * 180.0 / M_PI;
      az = std::fmod(az + 360.0, 360.0);
      patches[{static_cast<std::int64_t>(std::floor(p.x() / o.doubled_cell_m)),
               static_cast<std::int64_t>(std::floor(p.y() / o.doubled_cell_m)),
               static_cast<std::int64_t>(
                   std::floor(az / o.doubled_azimuth_bin_deg))}]
          .emplace_back(p, n);
    }
    DoubledWallStats d;
    d.storey = s.idx;
    std::vector<double> seps;
    for (const auto &kv : patches) {
      const auto &pts = kv.second;
      if (static_cast<int>(pts.size()) < o.doubled_min_patch_points)
        continue;
      Eigen::Vector3d n = Eigen::Vector3d::Zero();
      for (const auto &pn : pts)
        n += pn.second;
      n.z() = 0;
      if (n.norm() <= 0)
        continue;
      n.normalize();
      std::vector<double> off(pts.size());
      for (std::size_t i = 0; i < pts.size(); ++i)
        off[i] = pts[i].first.dot(n);
      std::vector<double> tmp = off;
      const double med = *median(tmp);
      for (auto &x : off)
        x -= med;
      ++d.patches;
      const double sep = patch_separation(off, o);
      if (sep > 0) {
        ++d.bimodal;
        seps.push_back(sep);
      }
    }
    d.bimodal_share =
        d.patches > 0 ? static_cast<double>(d.bimodal) / d.patches : 0.0;
    d.separation_median_m = median(seps);
    out.push_back(d);
  }
  std::sort(out.begin(), out.end(),
            [](const auto &a, const auto &b) { return a.storey < b.storey; });
  return out;
}

void warn_and_record(DriftReport &r, std::string msg) {
  core::warn("analyze drift: {}", msg);
  r.warnings.push_back(std::move(msg));
}

} // namespace

std::size_t CrossingSummary::through_wall() const {
  return static_cast<std::size_t>(
      std::count_if(crossings.begin(), crossings.end(),
                    [](const WallCrossing &c) { return !c.at_opening; }));
}

CrossingSummary
trajectory_wall_crossings(std::span<const std::int64_t> node_ids,
                          std::span<const Eigen::Matrix4d> poses,
                          const core::SolidModel &plan,
                          const DriftDiagnosticsOptions &o) {
  if (node_ids.size() != poses.size())
    throw std::invalid_argument(
        "trajectory_wall_crossings: " + std::to_string(node_ids.size()) +
        " node ids but " + std::to_string(poses.size()) + " poses");
  CrossingSummary out;
  for (const auto &st : plan.storeys) {
    const Eigen::Matrix4d l = to_matrix(st.level_transform);
    std::vector<Eigen::Vector3d> c(poses.size());
    for (std::size_t k = 0; k < poses.size(); ++k)
      c[k] = (l * poses[k] * Eigen::Vector4d(0, 0, 0, 1)).head<3>();
    auto inside = [&](const Eigen::Vector3d &p) {
      return p.z() > st.z_floor - o.crossing_storey_below_m &&
             p.z() < st.z_ceiling + o.crossing_storey_above_m;
    };
    for (std::size_t k = 0; k + 1 < c.size(); ++k) {
      if (!inside(c[k]) || !inside(c[k + 1]))
        continue;
      const Eigen::Vector2d p = c[k].head<2>(), q = c[k + 1].head<2>();
      if ((q - p).norm() > o.crossing_max_step_m)
        continue;
      ++out.steps;
      for (const auto &w : plan.walls) {
        if (w.storey != st.idx)
          continue;
        const Eigen::Vector2d a(w.x0, w.y0), b(w.x1, w.y1);
        Eigen::Matrix2d m;
        m.col(0) = q - p;
        m.col(1) = a - b;
        if (std::abs(m.determinant()) < 1e-12) // parallel: no crossing
          continue;
        const Eigen::Vector2d tu = m.partialPivLu().solve(a - p);
        if (tu.x() < 0 || tu.x() > 1 || tu.y() < 0 || tu.y() > 1)
          continue;
        WallCrossing x;
        x.from_node = node_ids[k];
        x.to_node = node_ids[k + 1];
        x.wall_guid = w.guid;
        x.storey = st.idx;
        x.s_m = tu.y() * (b - a).norm();
        const Eigen::Vector2d at = a + tu.y() * (b - a);
        x.x = at.x();
        x.y = at.y();
        for (const auto &op : plan.openings)
          if (op.wall == w.guid &&
              x.s_m >= op.s_start - o.crossing_opening_tol_m &&
              x.s_m <= op.s_end + o.crossing_opening_tol_m)
            x.at_opening = true;
        out.crossings.push_back(std::move(x));
      }
    }
  }
  std::stable_sort(out.crossings.begin(), out.crossings.end(),
                   [](const WallCrossing &a, const WallCrossing &b) {
                     return std::tie(a.from_node, a.wall_guid) <
                            std::tie(b.from_node, b.wall_guid);
                   });
  return out;
}

DriftReport diagnose_drift(const ProjectDB &db,
                           const DriftDiagnosticsOptions &o,
                           const core::SolidModel *plan,
                           std::span<const Eigen::Matrix4d> poses_override) {
  if (o.segments < 1 || o.frame_step < 1 || o.depth_stride < 1 ||
      o.normal_neighbours < 3 || o.revisit_neighbours < 1 ||
      o.floor_max_frames < 1 || !(o.cell_m > 0) || !(o.doubled_voxel_m > 0) ||
      !(o.doubled_cell_m > 0) || !(o.doubled_hist_bin_m > 0) ||
      !(o.doubled_azimuth_bin_deg > 0))
    throw std::invalid_argument(
        "diagnose_drift: segments, frame_step, depth_stride, "
        "revisit_neighbours and floor_max_frames must be >= 1, "
        "normal_neighbours >= 3, cell and bin sizes > 0");
  const reusex::stopwatch clock;
  DriftReport r;
  r.cell_m = o.cell_m;
  r.pose_source =
      o.pose_source == DriftPoseSource::initial ? "initial" : "current";

  const auto ids = db.sensor_frame_ids();
  r.frames_total = ids.size();
  if (!poses_override.empty() && poses_override.size() != ids.size())
    throw std::invalid_argument("diagnose_drift: poses_override has " +
                                std::to_string(poses_override.size()) +
                                " poses for " + std::to_string(ids.size()) +
                                " sensor frames");

  // ---- poses of every frame ------------------------------------------------
  std::size_t initial_missing = 0;
  auto pose_of = [&](std::size_t k) -> std::optional<Eigen::Matrix4d> {
    if (!poses_override.empty())
      return poses_override[k];
    const int id = ids[k];
    if (o.pose_source == DriftPoseSource::initial) {
      if (const auto p = db.initial_pose(id))
        return to_matrix(*p);
      ++initial_missing;
    }
    if (!db.has_sensor_frame_pose(id))
      return std::nullopt;
    return to_matrix(db.sensor_frame_pose(id));
  };
  std::vector<std::optional<Eigen::Matrix4d>> all_poses(ids.size());
  for (std::size_t k = 0; k < ids.size(); ++k)
    all_poses[k] = pose_of(k);
  if (initial_missing > 0)
    warn_and_record(r, fmt::format("{} of {} frames have no initial pose; "
                                   "they use their current pose",
                                   initial_missing, ids.size()));

  // ---- sampled frames and their timestamps ---------------------------------
  std::vector<SampledFrame> frames;
  for (std::size_t k = 0; k < ids.size();
       k += static_cast<std::size_t>(o.frame_step)) {
    SampledFrame f;
    f.node_id = ids[k];
    f.order = k;
    f.pose = all_poses[k];
    frames.push_back(f);
  }
  r.frames_sampled = frames.size();
  r.timestamps =
      !frames.empty() &&
      std::all_of(frames.begin(), frames.end(), [&](const auto &f) {
        return db.sensor_frame_timestamp(static_cast<int>(f.node_id)) >= 0.0;
      });
  for (auto &f : frames)
    f.time = r.timestamps
                 ? db.sensor_frame_timestamp(static_cast<int>(f.node_id))
                 : static_cast<double>(f.order);
  const double gap = r.timestamps ? o.gap_s : o.gap_frames;
  if (!r.timestamps)
    core::info("analyze drift: the frames carry no timestamps; passes are "
               "split by node order (> {} frames apart)",
               o.gap_frames);
  const auto nf = frames.size();
  for (std::size_t k = 0; k < nf; ++k)
    frames[k].segment = static_cast<int>(
        std::min<std::size_t>(k * static_cast<std::size_t>(o.segments) / nf,
                              static_cast<std::size_t>(o.segments - 1)));

  // ---- back-projection -----------------------------------------------------
  FramePairIcpOptions bp;
  bp.pixel_step = o.depth_stride;
  bp.min_depth_m = static_cast<float>(o.min_depth_m);
  bp.max_depth_m = static_cast<float>(o.max_depth_m);
  Pooled pooled;
  std::size_t no_depth = 0, no_pose = 0;
  for (std::size_t k = 0; k < nf; ++k) {
    auto &f = frames[k];
    pooled.centres.push_back(
        f.pose ? Eigen::Vector3d(f.pose->topRightCorner<3, 1>())
               : Eigen::Vector3d::Zero());
    if (!f.pose) {
      ++no_pose;
      continue;
    }
    std::vector<std::array<float, 3>> pts;
    try {
      pts = frame_pose_frame_points(db, static_cast<int>(f.node_id), bp);
    } catch (const std::runtime_error &e) {
      core::debug("analyze drift: frame {} skipped: {}", f.node_id, e.what());
      ++no_depth;
      continue;
    }
    const Eigen::Matrix3d rot = f.pose->topLeftCorner<3, 3>();
    const Eigen::Vector3d tr = f.pose->topRightCorner<3, 1>();
    for (const auto &p : pts) {
      const Eigen::Vector3d w = rot * Eigen::Vector3d(p[0], p[1], p[2]) + tr;
      pcl::PointXYZ q;
      q.getVector3fMap() = w.cast<float>();
      pooled.cloud->push_back(q);
      pooled.frame.push_back(static_cast<int>(k));
    }
    f.points = pts.size();
    ++r.frames_used;
  }
  if (no_pose + no_depth > 0)
    warn_and_record(r, fmt::format("{} of {} sampled frames contribute no "
                                   "points ({} without a pose, {} without "
                                   "depth in {}-{} m)",
                                   no_pose + no_depth, nf, no_pose, no_depth,
                                   o.min_depth_m, o.max_depth_m));
  const PointCloud &cloud = *pooled.cloud;
  r.points = cloud.size();
  if (cloud.empty())
    throw std::runtime_error("diagnose_drift: no sampled frame has a pose "
                             "and depth to back-project (" +
                             std::to_string(nf) + " sampled frames)");

  core::debug("analyze drift: back-projection done at {:.1f} s",
              clock.elapsed());
  // ---- pooled pass-to-pass residual and noise floor ------------------------
  const auto tree = make_tree(pooled.cloud);
  const auto normals = pca_normals(cloud, *tree, o.normal_neighbours);
  std::vector<double> far_res(cloud.size(), kNaN),
      floor_res(cloud.size(), kNaN);
  {
    const auto n = static_cast<std::int64_t>(cloud.size());
    const double r2 = o.revisit_radius_m * o.revisit_radius_m;
    const int kk =
        std::min<int>(o.revisit_neighbours, static_cast<int>(cloud.size()));
#pragma omp parallel for schedule(static)
    for (std::int64_t i = 0; i < n; ++i) {
      const auto ui = static_cast<std::size_t>(i);
      thread_local std::vector<std::uint32_t> idx;
      thread_local std::vector<float> d2;
      // The kk nearest within the radius, ascending (bounded like the
      // prototype's distance_upper_bound).
      tree->knn(cloud[ui], kk, o.revisit_radius_m, idx, d2);
      const int fi = pooled.frame[ui];
      const double ti = frames[static_cast<std::size_t>(fi)].time;
      bool far_done = false, floor_done = false;
      for (std::size_t m = 0; m < idx.size() && !(far_done && floor_done);
           ++m) {
        if (!(static_cast<double>(d2[m]) < r2))
          break;
        const auto j = static_cast<std::size_t>(idx[m]);
        const int fj = pooled.frame[j];
        if (!far_done &&
            std::abs(frames[static_cast<std::size_t>(fj)].time - ti) > gap) {
          far_done = true;
          if (normals[j].allFinite())
            far_res[ui] = plane_residual(cloud[ui], cloud[j], normals[j]);
        }
        const int df = std::abs(fj - fi);
        if (!floor_done && df >= 1 && df <= o.floor_max_frames) {
          floor_done = true;
          if (normals[j].allFinite())
            floor_res[ui] = plane_residual(cloud[ui], cloud[j], normals[j]);
        }
      }
    }
  }
  {
    auto f = finite_abs(floor_res);
    r.noise_floor_m = median(f);
    r.noise_floor_p90_m = percentile(f, 90.0);
    auto a = finite_abs(far_res);
    r.revisit_points = a.size();
    r.residual_median_m = median(a);
    r.residual_p90_m = percentile(a, 90.0);
  }

  core::debug("analyze drift: pooled residual done at {:.1f} s",
              clock.elapsed());
  // ---- storeys (the cells' keys and the doubled-wall bands) ---------------
  std::vector<core::StoreyRecord> storeys;
  if (plan && !plan->storeys.empty())
    storeys = plan->storeys;
  else if (db.has_storeys())
    storeys = db.storeys();
  std::sort(storeys.begin(), storeys.end(),
            [](const auto &a, const auto &b) { return a.idx < b.idx; });

  // ---- per frame and per cell ----------------------------------------------
  {
    // reusex-98o: with storeys, a residual point is binned in the level
    // frame of the storey whose [z_floor, z_ceiling] band is nearest (inside
    // one = distance 0; ties to the lower storey), so storeys stacked in one
    // footprint get separate cells.
    const bool by_storey = o.cells_by_storey && !storeys.empty();
    std::vector<Eigen::Matrix4d> to_level;
    if (by_storey)
      for (const auto &s : storeys) {
        to_level.push_back(to_matrix(s.level_transform));
        DriftStorey d;
        d.idx = s.idx;
        d.name = s.name;
        d.z_floor = s.z_floor;
        d.z_ceiling = s.z_ceiling;
        d.level_transform = s.level_transform;
        r.storeys.push_back(d);
      }
    struct CellAcc {
      std::vector<double> res, z;
    };
    std::vector<std::vector<double>> per_frame(nf);
    std::map<std::tuple<int, std::int64_t, std::int64_t>, CellAcc> cells;
    for (std::size_t i = 0; i < cloud.size(); ++i) {
      if (!std::isfinite(far_res[i]))
        continue;
      per_frame[static_cast<std::size_t>(pooled.frame[i])].push_back(
          far_res[i]);
      const Eigen::Vector3d w = cloud[i].getVector3fMap().cast<double>();
      int key = -1;
      Eigen::Vector3d p = w;
      if (by_storey) {
        double best = std::numeric_limits<double>::infinity();
        for (std::size_t s = 0; s < storeys.size(); ++s) {
          const Eigen::Vector3d l = to_level[s].topLeftCorner<3, 3>() * w +
                                    to_level[s].topRightCorner<3, 1>();
          const double d = std::max(
              {storeys[s].z_floor - l.z(), l.z() - storeys[s].z_ceiling, 0.0});
          if (d < best) {
            best = d;
            key = storeys[s].idx;
            p = l;
          }
        }
      }
      auto &c =
          cells[{key, static_cast<std::int64_t>(std::floor(p.x() / o.cell_m)),
                 static_cast<std::int64_t>(std::floor(p.y() / o.cell_m))}];
      c.res.push_back(far_res[i]);
      c.z.push_back(p.z());
    }
    std::vector<double> frame_meds;
    for (std::size_t k = 0; k < nf; ++k) {
      FrameDrift fd;
      fd.node_id = frames[k].node_id;
      fd.points = frames[k].points;
      fd.revisit_share =
          frames[k].points > 0
              ? static_cast<double>(per_frame[k].size()) / frames[k].points
              : 0.0;
      if (static_cast<int>(per_frame[k].size()) >= o.min_frame_points) {
        fd.residual_m = median(per_frame[k]);
        frame_meds.push_back(*fd.residual_m);
        if (*fd.residual_m > o.frame_flag_m)
          ++r.frames_flagged;
      }
      r.frames.push_back(fd);
    }
    r.revisit_frames = frame_meds.size();
    r.frame_residual_median_m = median(frame_meds);
    r.frame_residual_p90_m = percentile(frame_meds, 90.0);
    for (auto &kv : cells) {
      if (static_cast<int>(kv.second.res.size()) < o.min_cell_points)
        continue;
      DriftCell c;
      c.storey = std::get<0>(kv.first);
      c.x = (static_cast<double>(std::get<1>(kv.first)) + 0.5) * o.cell_m;
      c.y = (static_cast<double>(std::get<2>(kv.first)) + 0.5) * o.cell_m;
      c.points = kv.second.res.size();
      c.residual_m = *median(kv.second.res);
      c.z_m = *percentile(kv.second.z, 10.0);
      r.cells.push_back(c);
    }
  }
  if (r.revisit_points == 0)
    warn_and_record(
        r, fmt::format("no pass-to-pass overlap: none of {} points found a "
                       "point of a frame more than {} {} away within {} m "
                       "(a single-pass capture, or the passes never see the "
                       "same surfaces); the pass-to-pass residual is n/a",
                       r.points, gap, r.timestamps ? "s" : "frames",
                       o.revisit_radius_m));

  // ---- segments ------------------------------------------------------------
  std::vector<std::vector<std::size_t>> seg_idx(
      static_cast<std::size_t>(o.segments));
  for (std::size_t i = 0; i < cloud.size(); ++i)
    seg_idx[static_cast<std::size_t>(
                frames[static_cast<std::size_t>(pooled.frame[i])].segment)]
        .push_back(i);
  for (int s = 0; s < o.segments; ++s) {
    DriftSegment d;
    d.index = s;
    bool any = false;
    for (const auto &f : frames)
      if (f.segment == s) {
        if (!any)
          d.first_node = f.node_id;
        d.last_node = f.node_id;
        any = true;
        ++d.frames;
      }
    d.points = seg_idx[static_cast<std::size_t>(s)].size();
    if (any)
      r.segments.push_back(d);
  }

  // sens2.py: every ordered pair (a into b).
  for (int b = 0; b < o.segments; ++b) {
    const auto &ib = seg_idx[static_cast<std::size_t>(b)];
    if (ib.size() < 3)
      continue;
    auto target = segment_cloud(cloud, ib, o.normal_neighbours);
    for (int a = 0; a < o.segments; ++a) {
      if (a == b)
        continue;
      const auto &ia = seg_idx[static_cast<std::size_t>(a)];
      std::vector<Eigen::Vector3d> pts(ia.size());
      for (std::size_t i = 0; i < ia.size(); ++i)
        pts[i] = cloud[ia[i]].getVector3fMap().cast<double>();
      std::vector<double> res;
      std::vector<int> hit;
      match(pts, Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(),
            Eigen::Vector3d::Zero(), target, o.match_radius_m, res, hit);
      auto v = finite_abs(res);
      if (static_cast<int>(v.size()) < o.min_pair_points)
        continue;
      SegmentPair p;
      p.a = a;
      p.b = b;
      p.points = v.size();
      p.residual_m = *median(v);
      r.pairs.push_back(p);
    }
  }
  std::sort(r.pairs.begin(), r.pairs.end(), [](const auto &x, const auto &y) {
    return std::tie(x.a, x.b) < std::tie(y.a, y.b);
  });
  {
    const int h = o.segments / 2;
    std::vector<double> within, across;
    for (const auto &p : r.pairs)
      ((p.a < h) == (p.b < h) ? within : across).push_back(p.residual_m);
    r.halves.within_pairs = within.size();
    r.halves.across_pairs = across.size();
    r.halves.within_m = median(within);
    r.halves.across_m = median(across);
  }
  if (o.segments > 1 && r.pairs.empty())
    warn_and_record(r, fmt::format("no two of the {} segments overlap by {} "
                                   "points within {} m; the segment matrix "
                                   "is empty",
                                   o.segments, o.min_pair_points,
                                   o.match_radius_m));

  core::debug("analyze drift: segment pairs done at {:.1f} s", clock.elapsed());
  // halves.py: each segment against the union of all the others.
  if (o.segments > 1) {
    for (int a = 0; a < o.segments; ++a) {
      std::vector<std::size_t> ib;
      ib.reserve(cloud.size());
      for (std::size_t i = 0; i < cloud.size(); ++i)
        if (frames[static_cast<std::size_t>(pooled.frame[i])].segment != a)
          ib.push_back(i);
      const auto &ia = seg_idx[static_cast<std::size_t>(a)];
      if (ib.size() < 3 || ia.empty())
        continue;
      auto target = segment_cloud(cloud, ib, o.normal_neighbours);
      std::vector<Eigen::Vector3d> pts(ia.size());
      for (std::size_t i = 0; i < ia.size(); ++i)
        pts[i] = cloud[ia[i]].getVector3fMap().cast<double>();
      std::vector<double> res;
      std::vector<int> hit;
      match(pts, Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(),
            Eigen::Vector3d::Zero(), target, o.icp_max_corr_m, res, hit);
      std::vector<Eigen::Vector3d> overlap;
      for (std::size_t i = 0; i < pts.size(); ++i)
        if (hit[i] >= 0)
          overlap.push_back(pts[i]);
      if (static_cast<int>(overlap.size()) < o.min_suggestion_points) {
        core::debug("analyze drift: segment {} overlaps the rest by {} "
                    "points (< {}); no suggestion",
                    a, overlap.size(), o.min_suggestion_points);
        continue;
      }
      SegmentSuggestion s;
      s.segment = a;
      for (const auto &d : r.segments)
        if (d.index == a) {
          s.first_node = d.first_node;
          s.last_node = d.last_node;
        }
      s.points = overlap.size();
      segment_icp(overlap, target, o, s);
      r.suggestions.push_back(s);
    }
  }

  core::debug("analyze drift: suggestions done at {:.1f} s", clock.elapsed());
  // ---- crossings -----------------------------------------------------------
  if (plan) {
    std::vector<std::int64_t> cids;
    std::vector<Eigen::Matrix4d> cposes;
    for (std::size_t k = 0; k < ids.size(); ++k)
      if (all_poses[k]) {
        cids.push_back(ids[k]);
        cposes.push_back(*all_poses[k]);
      }
    r.crossings = trajectory_wall_crossings(cids, cposes, *plan, o);
  }

  core::debug("analyze drift: crossings done at {:.1f} s", clock.elapsed());
  // ---- doubled walls -------------------------------------------------------
  if (o.doubled_walls) {
    if (storeys.empty())
      core::info("analyze drift: no storeys (plan or project); the doubled-"
                 "wall patches are skipped");
    else
      r.doubled_walls = doubled_walls(pooled, storeys, o);
  }

  core::info("analyze drift: {} frames ({} points), pass-to-pass {} mm vs "
             "noise floor {} mm, {} segment pairs, {} suggestions in {:.1f} s",
             r.frames_used, r.points,
             r.residual_median_m
                 ? fmt::format("{:.1f}", *r.residual_median_m * 1e3)
                 : std::string("n/a"),
             r.noise_floor_m ? fmt::format("{:.1f}", *r.noise_floor_m * 1e3)
                             : std::string("n/a"),
             r.pairs.size(), r.suggestions.size(), clock.elapsed());
  return r;
}

std::string drift_report_to_json(const DriftReport &r, int indent) {
  using nlohmann::json;
  auto opt = [](const std::optional<double> &v) -> json {
    return v ? json(*v) : json(nullptr);
  };
  json j;
  j["pose_source"] = r.pose_source;
  j["timestamps"] = r.timestamps;
  j["frames_total"] = r.frames_total;
  j["frames_sampled"] = r.frames_sampled;
  j["frames_used"] = r.frames_used;
  j["points"] = r.points;
  j["revisit_points"] = r.revisit_points;
  j["noise_floor_m"] = opt(r.noise_floor_m);
  j["noise_floor_p90_m"] = opt(r.noise_floor_p90_m);
  j["residual_median_m"] = opt(r.residual_median_m);
  j["residual_p90_m"] = opt(r.residual_p90_m);
  j["frame_residual_median_m"] = opt(r.frame_residual_median_m);
  j["frame_residual_p90_m"] = opt(r.frame_residual_p90_m);
  j["revisit_frames"] = r.revisit_frames;
  j["frames_flagged"] = r.frames_flagged;
  j["frames"] = json::array();
  for (const auto &f : r.frames)
    j["frames"].push_back({{"node_id", f.node_id},
                           {"residual_m", opt(f.residual_m)},
                           {"revisit_share", f.revisit_share},
                           {"points", f.points}});
  j["segments"] = json::array();
  for (const auto &s : r.segments)
    j["segments"].push_back({{"index", s.index},
                             {"first_node", s.first_node},
                             {"last_node", s.last_node},
                             {"frames", s.frames},
                             {"points", s.points}});
  j["pairs"] = json::array();
  for (const auto &p : r.pairs)
    j["pairs"].push_back({{"a", p.a},
                          {"b", p.b},
                          {"residual_m", p.residual_m},
                          {"points", p.points}});
  j["halves"] = {{"within_m", opt(r.halves.within_m)},
                 {"across_m", opt(r.halves.across_m)},
                 {"within_pairs", r.halves.within_pairs},
                 {"across_pairs", r.halves.across_pairs}};
  j["suggestions"] = json::array();
  for (const auto &s : r.suggestions) {
    json delta = json::array();
    for (int a = 0; a < 4; ++a)
      for (int b = 0; b < 4; ++b)
        delta.push_back(s.delta(a, b));
    j["suggestions"].push_back({{"segment", s.segment},
                                {"first_node", s.first_node},
                                {"last_node", s.last_node},
                                {"delta", delta},
                                {"before_m", s.before_m},
                                {"after_m", s.after_m},
                                {"points", s.points},
                                {"shift_m", s.shift_m},
                                {"centroid_shift_m", s.centroid_shift_m},
                                {"rotation_deg", s.rotation_deg},
                                {"yaw_deg", s.yaw_deg}});
  }
  j["cells"] = json::array();
  for (const auto &c : r.cells)
    j["cells"].push_back({{"storey", c.storey},
                          {"x", c.x},
                          {"y", c.y},
                          {"z_m", c.z_m},
                          {"residual_m", c.residual_m},
                          {"points", c.points}});
  j["cell_m"] = r.cell_m;
  j["storeys"] = json::array();
  for (const auto &s : r.storeys)
    j["storeys"].push_back({{"idx", s.idx},
                            {"name", s.name},
                            {"z_floor", s.z_floor},
                            {"z_ceiling", s.z_ceiling},
                            {"level_transform", s.level_transform}});
  if (r.crossings) {
    json c = {{"steps", r.crossings->steps},
              {"crossings", r.crossings->crossings.size()},
              {"through_wall", r.crossings->through_wall()},
              {"list", json::array()}};
    for (const auto &x : r.crossings->crossings)
      c["list"].push_back({{"from_node", x.from_node},
                           {"to_node", x.to_node},
                           {"wall", x.wall_guid},
                           {"storey", x.storey},
                           {"s_m", x.s_m},
                           {"x", x.x},
                           {"y", x.y},
                           {"at_opening", x.at_opening}});
    j["crossings"] = c;
  } else {
    j["crossings"] = nullptr;
  }
  j["doubled_walls"] = json::array();
  for (const auto &d : r.doubled_walls)
    j["doubled_walls"].push_back(
        {{"storey", d.storey},
         {"patches", d.patches},
         {"bimodal", d.bimodal},
         {"bimodal_share", d.bimodal_share},
         {"separation_median_m", opt(d.separation_median_m)}});
  j["warnings"] = r.warnings;
  return j.dump(indent);
}

} // namespace reusex::slam
