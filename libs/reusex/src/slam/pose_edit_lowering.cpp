// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "slam/pose_edit_lowering.hpp"

#include "core/ProjectDB.hpp"
#include "core/SensorIntrinsics.hpp"
#include "core/logging.hpp"
#include "core/pose_graph.hpp"
#include "geometry/level_frame.hpp"
#include "slam/frame_pair_icp.hpp"
#include "slam/pose_graph_conventions.hpp"

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

#include <fmt/format.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <random>
#include <stdexcept>

namespace reusex::slam {
namespace {

namespace pg = core::pose_graph;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using core::PoseEdit;
using core::PoseEditKind;

constexpr double kDeg = M_PI / 180.0;

[[noreturn]] void fail(const PoseEdit &e, const std::string &what) {
  throw std::runtime_error(fmt::format("pose edit '{}' ({}): {}", e.id,
                                       core::to_string(e.kind), what));
}

/// Diagonal information over [rx, ry, rz, tx, ty, tz] from sigmas and a DoF
/// mask (Dof::roll/pitch/yaw = rotation about x/y/z).
Matrix6d masked_information(const PoseEdit &e, double scale = 1.0) {
  const double ir = 1.0 / std::pow(e.sigma_rot_deg * kDeg, 2);
  const double it = 1.0 / std::pow(e.sigma_trans_m, 2);
  Matrix6d I = Matrix6d::Zero();
  const std::uint8_t bits[6] = {core::Dof::roll, core::Dof::pitch,
                                core::Dof::yaw,  core::Dof::x,
                                core::Dof::y,    core::Dof::z};
  for (int k = 0; k < 6; ++k)
    if (e.dofs & bits[k])
      I(k, k) = (k < 3 ? ir : it) * scale;
  return I;
}

struct Frame {
  Eigen::Matrix4d to_world = Eigen::Matrix4d::Identity(); ///< doc -> world
  Eigen::Matrix4d to_doc = Eigen::Matrix4d::Identity();   ///< world -> doc

  /// A pose given in the document frame, in world.
  Eigen::Matrix4d pose_to_world(const Eigen::Matrix4d &T) const {
    return to_world * T;
  }
  /// A world-frame delta (acting on world points) from a document-frame one.
  Eigen::Matrix4d delta_to_world(const Eigen::Matrix4d &D) const {
    return to_world * D * to_doc;
  }
};

Frame document_frame(const ProjectDB &db, const core::PoseEdits &edits) {
  Frame f;
  if (edits.frame == "world")
    return f;
  const std::string guid = edits.frame.substr(std::string("storey:").size());
  for (const auto &s : db.storeys())
    if (s.guid == guid) {
      const auto L = geometry::LevelFrame::from_record(s);
      f.to_doc = L.world_to_level.matrix();
      f.to_world = L.world_to_level.inverse(Eigen::Isometry).matrix();
      return f;
    }
  throw std::runtime_error(fmt::format(
      "pose edits: frame: no storey with guid '{}' in this project", guid));
}

/// Per-node data the lowering needs.
struct Node {
  Eigen::Matrix4d local = Eigen::Matrix4d::Identity();   ///< local_transform
  Eigen::Matrix4d initial = Eigen::Matrix4d::Identity(); ///< stored conv.
  Eigen::Matrix4d current = Eigen::Matrix4d::Identity(); ///< stored conv.
};

/// The nodes of @p range in the solve, as indices into node_order.
std::vector<int> solve_indices(const PoseEdit &e, const core::NodeRange &range,
                               const std::map<int, int> &index_of,
                               const char *side) {
  std::vector<int> out;
  int missing = 0;
  if (range.is_list()) {
    for (const auto id : range.list) {
      const auto it = index_of.find(static_cast<int>(id));
      if (it == index_of.end())
        ++missing;
      else
        out.push_back(it->second);
    }
  } else {
    for (const auto &[id, k] : index_of)
      if (range.contains(id))
        out.push_back(k);
  }
  std::sort(out.begin(), out.end());
  if (out.empty())
    fail(e, fmt::format("none of the nodes of {} is in the solve (no such "
                        "node, or no depth / surfels)",
                        side));
  if (missing > 0)
    core::warn("pose edit '{}': {} listed nodes of {} are not in the solve "
               "and were skipped",
               e.id, missing, side);
  return out;
}

Eigen::Matrix4d resolve_pose(const PoseEdit &e, const Node &n,
                             const Frame &frame) {
  switch (e.pose.source) {
  case core::PoseSpec::Source::current:
    return n.current;
  case core::PoseSpec::Source::initial:
    return n.initial;
  case core::PoseSpec::Source::given:
    break;
  }
  return frame.pose_to_world(e.pose.value.to_matrix());
}

struct Plane {
  Eigen::Vector3d n = Eigen::Vector3d::UnitZ();
  double d = 0.0; ///< n.p + d = 0
};

Plane pca_plane(const std::vector<Eigen::Vector3d> &pts) {
  Eigen::Vector3d c = Eigen::Vector3d::Zero();
  for (const auto &p : pts)
    c += p;
  c /= static_cast<double>(pts.size());
  Eigen::Matrix3d C = Eigen::Matrix3d::Zero();
  for (const auto &p : pts)
    C += (p - c) * (p - c).transpose();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(C);
  Plane pl;
  pl.n = es.eigenvectors().col(0).normalized();
  pl.d = -pl.n.dot(c);
  return pl;
}

/// RANSAC + PCA refit; returns the plane and its inliers.
Plane fit_plane(const std::vector<Eigen::Vector3d> &pts,
                const SameSurfaceOptions &o, std::vector<Eigen::Vector3d> &in) {
  std::mt19937 rng(o.seed);
  std::uniform_int_distribution<size_t> pick(0, pts.size() - 1);
  Plane best;
  size_t best_count = 0;
  for (int it = 0; it < o.ransac_iterations; ++it) {
    const auto &a = pts[pick(rng)];
    const auto &b = pts[pick(rng)];
    const auto &c = pts[pick(rng)];
    const Eigen::Vector3d n = (b - a).cross(c - a);
    if (n.norm() < 1e-9)
      continue;
    Plane pl{n.normalized(), 0.0};
    pl.d = -pl.n.dot(a);
    size_t count = 0;
    for (const auto &p : pts)
      count += std::abs(pl.n.dot(p) + pl.d) < o.ransac_distance_m;
    if (count > best_count) {
      best_count = count;
      best = pl;
    }
  }
  in.clear();
  for (const auto &p : pts)
    if (std::abs(best.n.dot(p) + best.d) < o.ransac_distance_m)
      in.push_back(p);
  if (in.size() < 3)
    return best;
  return pca_plane(in);
}

/// A world point and the solve index of the frame that saw it.
struct TaggedPoint {
  int frame;
  Eigen::Vector3d p;
};

/// World points of @p frames (indices) inside @p region (document xy),
/// back-projected with the INITIAL poses.
std::vector<TaggedPoint>
region_points(const ProjectDB &db, const std::vector<int> &frames,
              const std::vector<int> &node_order,
              const std::vector<Node> &nodes, const std::array<double, 4> &r,
              const Frame &frame, const SameSurfaceOptions &o) {
  FramePairIcpOptions icp;
  icp.pixel_step = o.pixel_step;
  std::vector<TaggedPoint> out;
  for (const int k : frames) {
    const auto &n = nodes[static_cast<size_t>(k)];
    // frame_world_points() uses the CURRENT pose; re-express in the initial.
    const Eigen::Matrix4d re = n.initial * n.current.inverse();
    std::vector<std::array<float, 3>> pts;
    try {
      pts = frame_world_points(db, node_order[static_cast<size_t>(k)], icp);
    } catch (const std::exception &) {
      continue; // a frame without depth in range contributes nothing
    }
    for (const auto &q : pts) {
      const Eigen::Vector3d w =
          (re * Eigen::Vector4d(q[0], q[1], q[2], 1.0)).head<3>();
      const Eigen::Vector3d d =
          (frame.to_doc * Eigen::Vector4d(w.x(), w.y(), w.z(), 1.0)).head<3>();
      if (d.x() >= r[0] && d.x() <= r[2] && d.y() >= r[1] && d.y() <= r[3])
        out.push_back({k, w});
    }
  }
  if (static_cast<int>(out.size()) > o.max_points_per_side) {
    const size_t stride =
        (out.size() + static_cast<size_t>(o.max_points_per_side) - 1) /
        static_cast<size_t>(o.max_points_per_side);
    std::vector<TaggedPoint> thin;
    for (size_t i = 0; i < out.size(); i += stride)
      thin.push_back(out[i]);
    out.swap(thin);
  }
  return out;
}

std::vector<Eigen::Vector3d> positions(const std::vector<TaggedPoint> &t) {
  std::vector<Eigen::Vector3d> out;
  out.reserve(t.size());
  for (const auto &p : t)
    out.push_back(p.p);
  return out;
}

/// Up to @p n entries of @p v, evenly spread, in order.
std::vector<int> spread(const std::vector<int> &v, size_t n) {
  if (v.size() <= n)
    return v;
  std::vector<int> out;
  for (size_t k = 0; k < n; ++k)
    out.push_back(v[(k * (v.size() - 1)) / (n - 1)]);
  return out;
}

/// The rigid world motion (about @p c) that best puts @p pts on @p plane in
/// the point-to-plane sense; unconstrained directions stay zero.
Eigen::Matrix4d fit_onto_plane(const std::vector<Eigen::Vector3d> &pts,
                               const Plane &plane, const Eigen::Vector3d &c) {
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t = Eigen::Vector3d::Zero();
  for (int it = 0; it < 8; ++it) {
    Matrix6d H = Matrix6d::Zero();
    Eigen::Matrix<double, 6, 1> g = Eigen::Matrix<double, 6, 1>::Zero();
    for (const auto &p : pts) {
      const Eigen::Vector3d q = c + R * (p - c) + t;
      const double r = plane.n.dot(q) + plane.d;
      Eigen::Matrix<double, 6, 1> J;
      J.head<3>() = (q - c).cross(plane.n);
      J.tail<3>() = plane.n;
      H += J * J.transpose();
      g += J * r;
    }
    Eigen::SelfAdjointEigenSolver<Matrix6d> es(H);
    const double top = es.eigenvalues().maxCoeff();
    Eigen::Matrix<double, 6, 1> step = Eigen::Matrix<double, 6, 1>::Zero();
    for (int k = 0; k < 6; ++k)
      if (es.eigenvalues()(k) > 1e-8 * top)
        step -= es.eigenvectors().col(k) *
                (es.eigenvectors().col(k).dot(g) / es.eigenvalues()(k));
    const Eigen::Vector3d w = step.head<3>();
    const Eigen::Matrix3d dR =
        w.norm() > 0.0
            ? Eigen::AngleAxisd(w.norm(), w.normalized()).toRotationMatrix()
            : Eigen::Matrix3d::Identity();
    // q' = c + dR (q - c) + v  with q = c + R (p - c) + t
    R = dR * R;
    t = dR * t + step.tail<3>();
    if (step.norm() < 1e-10)
      break;
  }
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.topLeftCorner<3, 3>() = R;
  T.topRightCorner<3, 1>() = c + t - R * c;
  return T;
}

} // namespace

LoweredEdits lower_pose_edits(const ProjectDB &db, const core::PoseEdits &edits,
                              const std::vector<int> &node_order,
                              const SameSurfaceOptions &so) {
  LoweredEdits out;
  if (edits.edits.empty())
    return out;
  const Frame frame = document_frame(db, edits);

  std::map<int, int> index_of;
  std::vector<Node> nodes(node_order.size());
  for (size_t k = 0; k < node_order.size(); ++k) {
    const int id = node_order[k];
    index_of[id] = static_cast<int>(k);
    nodes[k].local = pg::matrix_from_row_major(
        db.sensor_frame_intrinsics(id).local_transform);
    nodes[k].current = pg::matrix_from_row_major(db.sensor_frame_pose(id));
    const auto init = db.initial_pose(id);
    nodes[k].initial =
        init ? pg::matrix_from_row_major(*init) : nodes[k].current;
  }
  auto optical = [&](int k, const Eigen::Matrix4d &stored) {
    return Eigen::Matrix4d(stored * nodes[static_cast<size_t>(k)].local);
  };

  for (const auto &e : edits.edits) {
    switch (e.kind) {
    case PoseEditKind::anchor: {
      const int k = solve_indices(e, e.nodes, index_of, "node").front();
      geometry::PosePrior p;
      p.i = k;
      p.pose =
          optical(k, resolve_pose(e, nodes[static_cast<size_t>(k)], frame));
      p.information = masked_information(e);
      p.axes = frame.to_doc.topLeftCorner<3, 3>();
      p.tag = e.id;
      out.priors.push_back(std::move(p));
      break;
    }
    case PoseEditKind::move: {
      const auto ks = solve_indices(e, e.nodes, index_of, "nodes");
      // Default pivot: centroid of the moved nodes' initial positions, in
      // the document frame (where the delta is written).
      Eigen::Vector3d pivot = Eigen::Vector3d::Zero();
      for (const int k : ks) {
        const Eigen::Vector4d p =
            frame.to_doc * nodes[static_cast<size_t>(k)].initial.col(3);
        pivot += p.head<3>();
      }
      pivot /= static_cast<double>(ks.size());
      const Eigen::Matrix4d D = frame.delta_to_world(e.delta.to_matrix(pivot));
      const Matrix6d I =
          masked_information(e, 1.0 / static_cast<double>(ks.size()));
      for (const int k : ks) {
        geometry::PosePrior p;
        p.i = k;
        p.pose = optical(k, D * nodes[static_cast<size_t>(k)].initial);
        p.information = I;
        p.axes = frame.to_doc.topLeftCorner<3, 3>();
        p.tag = e.id;
        out.priors.push_back(std::move(p));
      }
      break;
    }
    case PoseEditKind::relative: {
      const int a = solve_indices(e, e.nodes, index_of, "a").front();
      const int b = solve_indices(e, e.nodes_b, index_of, "b").front();
      const auto &na = nodes[static_cast<size_t>(a)];
      const auto &nb = nodes[static_cast<size_t>(b)];
      Eigen::Matrix4d T_ab; // stored convention: pose of b in a
      switch (e.pose.source) {
      case core::PoseSpec::Source::current:
        T_ab = na.current.inverse() * nb.current;
        break;
      case core::PoseSpec::Source::initial:
        T_ab = na.initial.inverse() * nb.initial;
        break;
      case core::PoseSpec::Source::given:
        T_ab = e.pose.value.to_matrix();
        break;
      }
      geometry::SurfaceConstraint s;
      s.i = a;
      s.j = b;
      s.T_ij = stored_to_optical(T_ab, na.local, nb.local);
      s.information =
          information_stored_to_optical(masked_information(e), nb.local);
      s.tag = e.id;
      out.surfaces.push_back(std::move(s));

      // docs/design/pose-edits.md, ### relative: the stored edge between a
      // and b, in either direction, is left out of the solve as if it had
      // been rejected -- naming the pair implies the same tombstone
      // reject_edge writes explicitly, for every edge type that could be
      // stored between them.
      const int node_a = static_cast<int>(e.nodes.first);
      const int node_b = static_cast<int>(e.nodes_b.first);
      for (const std::string_view type :
           {pg::kTypeOdometry, pg::kTypeLoopClosure, pg::kTypePanorama}) {
        EdgeTombstone t;
        t.from = std::min(node_a, node_b);
        t.to = std::max(node_a, node_b);
        t.edge_type = std::string(type);
        t.tag = e.id;
        out.tombstones.push_back(std::move(t));
      }
      break;
    }
    case PoseEditKind::same_surface: {
      const auto ka = solve_indices(e, e.nodes, index_of, "a.nodes");
      const auto kb = solve_indices(e, e.nodes_b, index_of, "b.nodes");
      const auto ta =
          region_points(db, ka, node_order, nodes, *e.region, frame, so);
      const auto tb =
          region_points(db, kb, node_order, nodes, *e.region_b, frame, so);
      if (static_cast<int>(ta.size()) < so.min_points)
        fail(e, fmt::format("a.region holds {} points of a.nodes (need {})",
                            ta.size(), so.min_points));
      if (static_cast<int>(tb.size()) < so.min_points)
        fail(e, fmt::format("b.region holds {} points of b.nodes (need {})",
                            tb.size(), so.min_points));
      std::vector<Eigen::Vector3d> ina, inb;
      const Plane A = fit_plane(positions(ta), so, ina);
      const Plane B = fit_plane(positions(tb), so, inb);
      if (static_cast<int>(ina.size()) < so.min_points ||
          static_cast<int>(inb.size()) < so.min_points)
        fail(e, fmt::format("too few points on a plane: a {} of {}, b {} of "
                            "{} within {:.3f} m (need {})",
                            ina.size(), ta.size(), inb.size(), tb.size(),
                            so.ransac_distance_m, so.min_points));
      const double cosang = std::abs(A.n.dot(B.n));
      const double angle = std::acos(std::min(1.0, cosang)) / kDeg;
      if (angle > so.max_normal_angle_deg)
        fail(e, fmt::format("the two sides' planes differ by {:.1f} deg (max "
                            "{:.1f}); not the same surface?",
                            angle, so.max_normal_angle_deg));
      Eigen::Vector3d cb = Eigen::Vector3d::Zero();
      for (const auto &p : inb)
        cb += p;
      cb /= static_cast<double>(inb.size());
      // The correction of b onto a's plane: offset along the normal and the
      // two tilts; nothing along the surface.
      const Eigen::Matrix4d D = fit_onto_plane(inb, A, cb);

      // b's plane points per frame (inliers of b's own plane fit).
      std::map<int, std::vector<Eigen::Vector3d>> per_frame;
      for (const auto &t : tb)
        if (std::abs(B.n.dot(t.p) + B.d) < so.ransac_distance_m)
          per_frame[t.frame].push_back(t.p);
      std::vector<int> used_b;
      size_t k_total = 0;
      for (const auto &[k, pts] : per_frame)
        if (static_cast<int>(pts.size()) >= so.min_points_per_frame) {
          used_b.push_back(k);
          k_total += pts.size();
        }
      std::vector<int> frames_a;
      for (const auto &t : ta)
        if (frames_a.empty() || frames_a.back() != t.frame)
          frames_a.push_back(t.frame);
      frames_a.erase(std::unique(frames_a.begin(), frames_a.end()),
                     frames_a.end());
      used_b = spread(used_b, static_cast<size_t>(so.max_frames_per_side));
      const auto used_a =
          spread(frames_a, static_cast<size_t>(so.max_frames_per_side));
      if (used_b.empty() || used_a.empty())
        fail(e, fmt::format("no frame of b has {} plane points of its own",
                            so.min_points_per_frame));
      k_total = 0;
      for (const int k : used_b)
        k_total += per_frame[k].size();

      // One constraint per (a frame, b frame) pair, so both blocks act as
      // wholes: b's frames are pulled onto a's surface by a's frames, and
      // the stretch goes into the links between the blocks rather than
      // inside them. The information of b frame j comes from j's own plane
      // points, J = [q x n_j, n_j] per point (q in j's optical frame), and
      // the pairs share one constraint's authority:
      //   I_aj = J_j^T J_j / (|A| * K * sigma^2).
      const double denom = static_cast<double>(used_a.size()) *
                           static_cast<double>(k_total) * e.sigma_m * e.sigma_m;
      for (const int j : used_b) {
        const Eigen::Matrix4d Xj =
            optical(j, nodes[static_cast<size_t>(j)].initial);
        const Eigen::Matrix4d Xj_target = D * Xj;
        const Eigen::Vector3d nj =
            Xj_target.topLeftCorner<3, 3>().transpose() * A.n;
        const Eigen::Matrix4d Xj_inv = Xj.inverse();
        Matrix6d I = Matrix6d::Zero();
        for (const auto &p : per_frame[j]) {
          const Eigen::Vector3d q =
              (Xj_inv * Eigen::Vector4d(p.x(), p.y(), p.z(), 1.0)).head<3>();
          Eigen::Matrix<double, 6, 1> J;
          J.head<3>() = q.cross(nj);
          J.tail<3>() = nj;
          I += J * J.transpose();
        }
        I /= denom;
        for (const int i : used_a) {
          if (i == j)
            continue;
          geometry::SurfaceConstraint c;
          c.i = i;
          c.j = j;
          c.T_ij = optical(i, nodes[static_cast<size_t>(i)].initial).inverse() *
                   Xj_target;
          c.information = 0.5 * (I + I.transpose());
          c.tag = e.id;
          out.surfaces.push_back(std::move(c));
        }
      }

      LoweredEdits::SurfaceFit fit;
      fit.tag = e.id;
      fit.offset_m =
          A.n.dot(D.topLeftCorner<3, 3>() * cb + D.topRightCorner<3, 1>() - cb);
      fit.tilt_deg =
          Eigen::AngleAxisd(Eigen::Matrix3d(D.topLeftCorner<3, 3>())).angle() /
          kDeg;
      fit.points_a = static_cast<int>(ina.size());
      fit.points_b = static_cast<int>(inb.size());
      core::info("pose edit '{}': same_surface fit moves b {:+.4f} m along "
                 "the normal and tilts it {:.3f} deg ({} / {} plane points; "
                 "{} x {} frame pairs)",
                 e.id, fit.offset_m, fit.tilt_deg, fit.points_a, fit.points_b,
                 used_a.size(), used_b.size());
      out.surface_fits.push_back(fit);
      break;
    }
    case PoseEditKind::on_plan_wall:
      core::warn("pose edit '{}': on_plan_wall is not lowered yet "
                 "(reusex-rcv.1.12); the edit is ignored in this solve",
                 e.id);
      out.skipped.push_back(e.id);
      break;
    case PoseEditKind::reject_edge: {
      EdgeTombstone t;
      t.from = static_cast<int>(std::min(e.nodes.first, e.nodes_b.first));
      t.to = static_cast<int>(std::max(e.nodes.first, e.nodes_b.first));
      t.edge_type = e.edge_type;
      t.tag = e.id;
      out.tombstones.push_back(std::move(t));
      break;
    }
    }
  }
  return out;
}

} // namespace reusex::slam
