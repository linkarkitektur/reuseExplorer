// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "optimize_watch.hpp"
#include "exit_status.hpp"
#include "plan_watch.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/frame_depth_cache.hpp>
#include <reusex/core/pose_graph.hpp>
#include <reusex/io/pose_edits.hpp>
#include <reusex/io/solid_model_json.hpp>
#include <reusex/reconstruction/plan_support.hpp>
#include <reusex/slam/PlaneGraphOptimizer.hpp>
#include <reusex/slam/drift_diagnostics.hpp>
#include <reusex/types/point_types.hpp>
#include <reusex/utils/file_watcher.hpp>
#include <reusex/visualize/drift_layers.hpp>
#include <reusex/visualize/render_view.hpp>

#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <fmt/format.h>
#include <fmt/ranges.h>
#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <numeric>
#include <regex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace fs = std::filesystem;
namespace pg = reusex::core::pose_graph;
namespace viz = reusex::visualize;
using json = nlohmann::json;

namespace rux {

namespace {

std::atomic<bool> g_stop{false};

extern "C" void optimize_watch_signal_handler(int /*sig*/) {
  g_stop.store(true, std::memory_order_release);
}

/// SIGINT / SIGTERM stop the loop for as long as this lives.
class StopOnSignal {
    public:
  StopOnSignal() {
    g_stop.store(false, std::memory_order_release);
    previous_int_ = std::signal(SIGINT, optimize_watch_signal_handler);
    previous_term_ = std::signal(SIGTERM, optimize_watch_signal_handler);
  }
  ~StopOnSignal() {
    std::signal(SIGINT, previous_int_);
    std::signal(SIGTERM, previous_term_);
  }
  StopOnSignal(const StopOnSignal &) = delete;
  StopOnSignal &operator=(const StopOnSignal &) = delete;

    private:
  using Handler = void (*)(int);
  Handler previous_int_ = SIG_DFL;
  Handler previous_term_ = SIG_DFL;
};

using Clock = std::chrono::steady_clock;
double seconds_since(Clock::time_point t0) {
  return std::chrono::duration<double>(Clock::now() - t0).count();
}

std::string mm(const std::optional<double> &v) {
  return v ? fmt::format("{:.1f} mm", *v * 1000.0) : std::string("n/a");
}

double rotation_deg(const Eigen::Matrix3d &r) {
  const double c = std::clamp((r.trace() - 1.0) / 2.0, -1.0, 1.0);
  return std::acos(c) * 180.0 / M_PI;
}

/// Median of @p v (sorted in place); nullopt when empty.
std::optional<double> median(std::vector<double> v) {
  if (v.empty())
    return std::nullopt;
  std::sort(v.begin(), v.end());
  const std::size_t n = v.size();
  return n % 2 == 1 ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
}

/// "a.png", or "a.png (+9 more)".
std::string images_line(const std::vector<fs::path> &written) {
  if (written.empty())
    return "none";
  if (written.size() == 1)
    return written.front().string();
  return fmt::format("{} (+{} more)", written.front().string(),
                     written.size() - 1);
}

/// World -> document frame of an edits file (identity for `world`).
Eigen::Matrix4d
document_frame(const std::string &frame,
               const std::vector<reusex::core::StoreyRecord> &storeys) {
  constexpr std::string_view kPrefix = "storey:";
  if (frame.rfind(kPrefix, 0) != 0)
    return Eigen::Matrix4d::Identity();
  const std::string guid = frame.substr(kPrefix.size());
  for (const auto &s : storeys)
    if (s.guid == guid)
      return pg::matrix_from_row_major(s.level_transform);
  return Eigen::Matrix4d::Identity(); // the lowering refuses it anyway
}

/// R = Rz(yaw) Ry(pitch) Rx(roll) -> (yaw, pitch, roll) in degrees.
std::array<double, 3> ypr_deg(const Eigen::Matrix3d &r) {
  const double pitch = -std::asin(std::clamp(r(2, 0), -1.0, 1.0));
  const double yaw = std::atan2(r(1, 0), r(0, 0));
  const double roll = std::atan2(r(2, 1), r(2, 2));
  constexpr double k = 180.0 / M_PI;
  return {yaw * k, pitch * k, roll * k};
}

/// The sigmas of a suggested move: about one odometry link
/// (PlaneGraphOptions::odometry_sigma_*), so the block wins against the one
/// link that holds the break; relax them in the file for a gentler pull.
constexpr double kMoveSigmaM = 0.005;
constexpr double kMoveSigmaDeg = 0.1;

/// A sheared segment fits worse than in the reference by more than this
/// (and by more than kShearWorseRel of the reference): about the noise of a
/// segment's median residual at the pinned sampling (calibration §9.4).
constexpr double kShearWorseM = 0.001;
constexpr double kShearWorseRel = 0.10;
/// The candidate's cross-segment median within this of the reference's
/// counts as the same (the verdict line still prints the difference).
constexpr double kSameAsReferenceM = 1e-4;

double round_to(double v, double step) { return std::round(v / step) * step; }

/// A pose / delta [R, t] as `{xyz, yaw_deg, pitch_deg, roll_deg}` (about the
/// frame origin), rounded to 0.1 mm and 0.001 deg so the YAML stays legible.
reusex::core::RigidTransform readable(const Eigen::Matrix4d &m) {
  reusex::core::RigidTransform t;
  const auto ypr = ypr_deg(m.topLeftCorner<3, 3>());
  t.yaw_deg = round_to(ypr[0], 1e-3);
  t.pitch_deg = round_to(ypr[1], 1e-3);
  t.roll_deg = round_to(ypr[2], 1e-3);
  for (int i = 0; i < 3; ++i)
    t.xyz[static_cast<std::size_t>(i)] = round_to(m(i, 3), 1e-4);
  return t;
}

/// A number for hand-edited YAML: at most 4 decimals, no trailing zeros.
std::string num(double v) {
  std::string t = fmt::format("{:.4f}", v);
  while (!t.empty() && t.back() == '0')
    t.pop_back();
  if (!t.empty() && t.back() == '.')
    t.pop_back();
  if (t == "-0" || t.empty())
    t = "0";
  return t;
}

std::string transform_yaml(const reusex::core::RigidTransform &t) {
  std::string out = fmt::format(
      "{{xyz: [{}, {}, {}], yaw_deg: {}, pitch_deg: {}, roll_deg: {}",
      num(t.xyz[0]), num(t.xyz[1]), num(t.xyz[2]), num(t.yaw_deg),
      num(t.pitch_deg), num(t.roll_deg));
  if (t.pivot)
    out += fmt::format(", pivot: [{}, {}, {}]", num((*t.pivot)[0]),
                       num((*t.pivot)[1]), num((*t.pivot)[2]));
  return out + "}";
}

/// One suggested edit (a move or an anchor) as a compact YAML list item,
/// indented to paste under an `edits:` key written the usual way. The test
/// suite reads every one back with io::pose_edits_from_yaml().
std::string edit_yaml(const reusex::core::PoseEdit &e) {
  using reusex::core::PoseEditKind;
  std::string out = fmt::format("  - id: {}\n    kind: {}\n", e.id,
                                reusex::core::to_string(e.kind));
  if (e.kind == PoseEditKind::move) {
    out += fmt::format("    nodes: [{}, {}]\n", e.nodes.first, e.nodes.last);
    out += "    delta: " + transform_yaml(e.delta) + "\n";
  } else {
    out += fmt::format("    node: {}\n", e.nodes.first);
    out += "    pose: " + transform_yaml(e.pose.value) + "\n";
  }
  if (e.dofs != reusex::core::Dof::all)
    out += fmt::format("    dofs: [{}]\n",
                       fmt::join(reusex::core::dof_names(e.dofs), ", "));
  out += fmt::format("    sigma: {{trans_m: {}, rot_deg: {}}}\n",
                     num(e.sigma_trans_m), num(e.sigma_rot_deg));
  std::string note = e.note;
  std::replace(note.begin(), note.end(), '"', '\'');
  out += fmt::format("    note: \"{}\"\n", note);
  return out;
}

} // namespace

// ── Free helpers (tested directly) ──────────────────────────────────────────

MoveRealisation move_realisation(const std::vector<Eigen::Matrix4d> &initial,
                                 const std::vector<Eigen::Matrix4d> &solved,
                                 const Eigen::Matrix4d &delta) {
  MoveRealisation m;
  const std::size_t n = std::min(initial.size(), solved.size());
  if (n == 0)
    return m;
  m.nodes = static_cast<int>(n);
  const std::vector<Eigen::Matrix4d> from(initial.begin(),
                                          initial.begin() + std::ptrdiff_t(n));
  const std::vector<Eigen::Matrix4d> to(solved.begin(),
                                        solved.begin() + std::ptrdiff_t(n));
  const Eigen::Matrix4d a = fit_rigid_delta(from, to);
  Eigen::Vector3d c = Eigen::Vector3d::Zero();
  for (const auto &f : from)
    c += f.topRightCorner<3, 1>();
  c /= static_cast<double>(n);
  auto at_c = [&c](const Eigen::Matrix4d &x) -> Eigen::Vector3d {
    return x.topLeftCorner<3, 3>() * c + x.topRightCorner<3, 1>();
  };
  auto angle_deg = [](const Eigen::Matrix3d &r) {
    return Eigen::AngleAxisd(r).angle() * 180.0 / M_PI;
  };
  m.move_trans_m = (at_c(delta) - c).norm();
  m.move_rot_deg = angle_deg(delta.topLeftCorner<3, 3>());
  m.remaining_trans_m = (at_c(delta) - at_c(a)).norm();
  m.remaining_rot_deg = angle_deg(delta.topLeftCorner<3, 3>().transpose() *
                                  a.topLeftCorner<3, 3>());
  // 1 - remaining / move; a zero move is all realised only if nothing
  // remains (it then passes on the 2 sigma bound, not on its share).
  auto share = [](double remaining, double move) {
    if (move > 1e-12)
      return 1.0 - remaining / move;
    return remaining <= 1e-12 ? 1.0 : 0.0;
  };
  m.share_trans = share(m.remaining_trans_m, m.move_trans_m);
  m.share_rot = share(m.remaining_rot_deg, m.move_rot_deg);
  double t2 = 0.0, r2 = 0.0;
  for (std::size_t k = 0; k < n; ++k) {
    const Eigen::Matrix4d rigid = a * from[k];
    t2 += (to[k].topRightCorner<3, 1>() - rigid.topRightCorner<3, 1>())
              .squaredNorm();
    const double r = angle_deg(rigid.topLeftCorner<3, 3>().transpose() *
                               to[k].topLeftCorner<3, 3>());
    r2 += r * r;
  }
  m.bend_trans_m = std::sqrt(t2 / static_cast<double>(n));
  m.bend_rot_deg = std::sqrt(r2 / static_cast<double>(n));
  return m;
}

std::string describe_realisation(const MoveRealisation &m) {
  return fmt::format("realised {:.0f}% / {:.0f}% of {:.3f} m / {:.2f} deg, "
                     "{:.1f} mm / {:.2f} deg from target, block bent "
                     "{:.1f} mm / {:.2f} deg (not judged)",
                     100.0 * m.share_trans, 100.0 * m.share_rot, m.move_trans_m,
                     m.move_rot_deg, 1000.0 * m.remaining_trans_m,
                     m.remaining_rot_deg, 1000.0 * m.bend_trans_m,
                     m.bend_rot_deg);
}

EditVerdict judge_edit(const reusex::geometry::EditResidual &residual,
                       const reusex::core::PoseEdit *edit,
                       const MoveRealisation *realisation) {
  using reusex::core::PoseEditKind;
  if (!residual.gnc_inlier)
    return {false, "GNC dropped it: it contradicts the graph or another edit"};
  if (edit == nullptr ||
      (edit->kind != PoseEditKind::anchor && edit->kind != PoseEditKind::move &&
       edit->kind != PoseEditKind::relative))
    return {};
  // reusex-9ge: an all-DoF move is judged by its rigid part. Its RMS
  // residual also carries the block's bend (the fast graph measures plain
  // optimize's shape, not the initial one's), which no rigid move removes.
  if (realisation != nullptr && realisation->nodes > 0 &&
      edit->kind == PoseEditKind::move &&
      edit->dofs == reusex::core::Dof::all) {
    const auto &m = *realisation;
    const bool t_ok =
        m.share_trans >= kHonouredMoveShare ||
        m.remaining_trans_m <= kHonouredSigmas * edit->sigma_trans_m;
    const bool r_ok =
        m.share_rot >= kHonouredMoveShare ||
        m.remaining_rot_deg <= kHonouredSigmas * edit->sigma_rot_deg;
    if (t_ok && r_ok)
      return {};
    return {false,
            fmt::format("the graph fought the move off: the block realised "
                        "{:.0f}% / {:.0f}% of its {:.3f} m / {:.2f} deg and "
                        "ended {:.3f} m / {:.2f} deg from its target (needs "
                        "{:.0f}% or {} sigma in each)",
                        100.0 * m.share_trans, 100.0 * m.share_rot,
                        m.move_trans_m, m.move_rot_deg, m.remaining_trans_m,
                        m.remaining_rot_deg, 100.0 * kHonouredMoveShare,
                        kHonouredSigmas)};
  }
  const double t_max = kHonouredSigmas * edit->sigma_trans_m;
  const double r_max = kHonouredSigmas * edit->sigma_rot_deg;
  std::vector<std::string> off;
  if (residual.rms_trans_m > t_max)
    off.push_back(fmt::format("{:.4f} m > {} sigma = {:.4f} m",
                              residual.rms_trans_m, kHonouredSigmas, t_max));
  if (residual.rms_rot_deg > r_max)
    off.push_back(fmt::format("{:.3f} deg > {} sigma = {:.3f} deg",
                              residual.rms_rot_deg, kHonouredSigmas, r_max));
  if (off.empty())
    return {};
  return {false, fmt::format("{} off its target by {}",
                             edit->kind == PoseEditKind::move ? "the block is"
                                                              : "it is",
                             fmt::join(off, " and "))};
}

std::optional<MovePrediction>
move_prediction(const reusex::core::PoseEdit &edit) {
  if (edit.kind != reusex::core::PoseEditKind::move)
    return std::nullopt;
  // The note the suggestion loop writes (see the suggestions below).
  static const std::regex re(
      R"(seg (\d+) \(\d+-\d+\) into the rest ([0-9.]+) -> ([0-9.]+) mm)");
  std::smatch m;
  if (!std::regex_search(edit.note, m, re))
    return std::nullopt;
  MovePrediction p;
  p.segment = std::stoi(m[1].str());
  p.before_m = std::stod(m[2].str()) / 1000.0;
  p.after_m = std::stod(m[3].str()) / 1000.0;
  return p;
}

std::optional<bool> move_realised(const reusex::core::PoseEdit &e,
                                  const std::map<int, double> &segment_now) {
  const auto p = move_prediction(e);
  if (!p)
    return std::nullopt;
  const auto now = segment_now.find(p->segment);
  if (now == segment_now.end())
    return std::nullopt;
  return now->second <=
         p->before_m - kRealisedShare * (p->before_m - p->after_m);
}

std::optional<std::string>
blocked_replacement(const reusex::core::PoseEdit &replaced,
                    const std::map<int, double> &segment_now) {
  const auto realised = move_realised(replaced, segment_now);
  if (!realised || *realised)
    return std::nullopt;
  const auto p = *move_prediction(replaced);
  return fmt::format(
      "it would replace {}, whose predicted seg {} {:.1f} -> {:.1f} mm was "
      "not realised (now {:.1f} mm); drop {} rather than replace it",
      replaced.id, p.segment, p.before_m * 1000.0, p.after_m * 1000.0,
      segment_now.at(p.segment) * 1000.0, replaced.id);
}

PoseChange pose_change(const std::vector<std::int64_t> &ids,
                       const std::vector<Eigen::Matrix4d> &a,
                       const std::vector<Eigen::Matrix4d> &b,
                       const std::vector<bool> *mask) {
  PoseChange c;
  if (a.size() != ids.size() || b.size() != ids.size())
    return c;
  std::vector<double> d;
  d.reserve(ids.size());
  for (std::size_t k = 0; k < ids.size(); ++k) {
    if (mask != nullptr && !(*mask)[k])
      continue;
    const double t =
        (a[k].topRightCorner<3, 1>() - b[k].topRightCorner<3, 1>()).norm();
    const double r = rotation_deg(a[k].topLeftCorner<3, 3>().transpose() *
                                  b[k].topLeftCorner<3, 3>());
    d.push_back(t);
    if (t > c.max_m || c.max_node < 0) {
      c.max_m = t;
      c.max_node = ids[k];
    }
    c.max_rot_deg = std::max(c.max_rot_deg, r);
  }
  c.nodes = d.size();
  if (d.empty())
    return c;
  std::sort(d.begin(), d.end());
  c.median_m = *median(d);
  c.p90_m = d[std::min(d.size() - 1,
                       static_cast<std::size_t>(0.9 * (d.size() - 1) + 0.5))];
  return c;
}

std::vector<std::int64_t> edited_nodes(const reusex::core::PoseEdits &edits,
                                       const std::vector<std::int64_t> &ids) {
  using reusex::core::PoseEditKind;
  std::vector<std::int64_t> sorted = ids;
  std::sort(sorted.begin(), sorted.end());
  std::set<std::int64_t> out;
  for (const auto &e : edits.edits) {
    if (e.kind == PoseEditKind::reject_edge)
      continue;
    for (auto id : e.nodes.select(sorted))
      out.insert(id);
    if (e.kind == PoseEditKind::relative ||
        e.kind == PoseEditKind::same_surface)
      for (auto id : e.nodes_b.select(sorted))
        out.insert(id);
  }
  return {out.begin(), out.end()};
}

Eigen::Matrix4d fit_rigid_delta(const std::vector<Eigen::Matrix4d> &from,
                                const std::vector<Eigen::Matrix4d> &to) {
  Eigen::Matrix4d out = Eigen::Matrix4d::Identity();
  const std::size_t n = std::min(from.size(), to.size());
  if (n == 0)
    return out;
  // Each pose: its origin and the tips of its unit axes, so even one pose
  // or a straight path fixes the rotation.
  std::vector<Eigen::Vector3d> p, q;
  p.reserve(4 * n);
  q.reserve(4 * n);
  for (std::size_t k = 0; k < n; ++k)
    for (int a = -1; a < 3; ++a) {
      Eigen::Vector4d local(0, 0, 0, 1);
      if (a >= 0)
        local[a] = 1.0;
      p.push_back((from[k] * local).head<3>());
      q.push_back((to[k] * local).head<3>());
    }
  Eigen::Vector3d cp = Eigen::Vector3d::Zero(), cq = Eigen::Vector3d::Zero();
  for (std::size_t k = 0; k < p.size(); ++k) {
    cp += p[k];
    cq += q[k];
  }
  cp /= static_cast<double>(p.size());
  cq /= static_cast<double>(q.size());
  Eigen::Matrix3d h = Eigen::Matrix3d::Zero();
  for (std::size_t k = 0; k < p.size(); ++k)
    h += (p[k] - cp) * (q[k] - cq).transpose();
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(h, Eigen::ComputeFullU |
                                               Eigen::ComputeFullV);
  Eigen::Matrix3d d = Eigen::Matrix3d::Identity();
  if ((svd.matrixV() * svd.matrixU().transpose()).determinant() < 0)
    d(2, 2) = -1.0;
  const Eigen::Matrix3d r = svd.matrixV() * d * svd.matrixU().transpose();
  out.topLeftCorner<3, 3>() = r;
  out.topRightCorner<3, 1>() = cq - r * cp;
  return out;
}

std::vector<StoreyRun>
storey_runs(const std::vector<std::int64_t> &nodes,
            const std::vector<Eigen::Vector3d> &positions,
            const std::vector<reusex::core::StoreyRecord> &storeys,
            std::size_t min_run) {
  std::vector<StoreyRun> runs;
  if (nodes.empty() || positions.size() != nodes.size())
    return runs;
  auto storey_of = [&](const Eigen::Vector3d &p) {
    int best = -1;
    double best_d = 1e300;
    for (const auto &s : storeys) {
      const Eigen::Matrix4d l = pg::matrix_from_row_major(s.level_transform);
      const double z = (l * p.homogeneous()).z();
      const double lo = s.z_floor - 0.5, hi = s.z_ceiling + 0.3;
      const double d = z < lo ? lo - z : (z > hi ? z - hi : 0.0);
      if (d < best_d) {
        best_d = d;
        best = s.idx;
      }
    }
    return best;
  };
  for (std::size_t k = 0; k < nodes.size(); ++k) {
    const int s = storeys.empty() ? -1 : storey_of(positions[k]);
    if (!runs.empty() && runs.back().storey == s) {
      runs.back().last = nodes[k];
      ++runs.back().frames;
    } else {
      runs.push_back({nodes[k], nodes[k], s, 1});
    }
  }
  // Short runs (a camera that dips below a band for a frame or two) go to
  // the run before them (the first one to the run after it).
  std::vector<StoreyRun> merged;
  for (const auto &r : runs) {
    if (r.frames < min_run && !merged.empty()) {
      merged.back().last = r.last;
      merged.back().frames += r.frames;
    } else if (!merged.empty() && merged.back().storey == r.storey) {
      merged.back().last = r.last;
      merged.back().frames += r.frames;
    } else {
      merged.push_back(r);
    }
  }
  if (merged.size() > 1 && merged.front().frames < min_run) {
    merged[1].first = merged.front().first;
    merged[1].frames += merged.front().frames;
    merged.erase(merged.begin());
  }
  return merged;
}

ShearIndicator shear_indicator(const std::vector<std::int64_t> &ids,
                               const std::vector<Eigen::Matrix4d> &poses,
                               const std::vector<Eigen::Matrix4d> &reference,
                               const std::vector<bool> &untouched,
                               const std::vector<ShearSegmentFit> &segments,
                               double flag_m) {
  ShearIndicator sh;
  if (poses.size() != ids.size() || reference.size() != ids.size() ||
      untouched.size() != ids.size())
    return sh;
  sh.change = pose_change(ids, poses, reference, &untouched);
  sh.untouched = sh.change.nodes;
  // The side of the last edited node each untouched node is on.
  std::int64_t last_edited = std::numeric_limits<std::int64_t>::min();
  for (std::size_t k = 0; k < ids.size(); ++k)
    if (!untouched[k])
      last_edited = std::max(last_edited, ids[k]);
  const bool any_edit = last_edited != std::numeric_limits<std::int64_t>::min();
  auto moved = [&](std::size_t k) {
    return (poses[k].topRightCorner<3, 1>() -
            reference[k].topRightCorner<3, 1>())
        .norm();
  };
  for (std::size_t k = 0; k < ids.size(); ++k)
    if (untouched[k] && moved(k) > flag_m)
      ++(any_edit && ids[k] > last_edited ? sh.downstream_moved
                                          : sh.upstream_moved);

  for (const auto &f : segments) {
    ShearIndicator::Segment seg;
    seg.index = f.index;
    seg.first_node = f.first_node;
    seg.last_node = f.last_node;
    seg.residual_m = f.residual_m;
    seg.reference_residual_m = f.reference_residual_m;
    seg.downstream = any_edit && f.first_node > last_edited;
    bool edited = false;
    std::size_t moved_nodes = 0;
    for (std::size_t k = 0; k < ids.size(); ++k) {
      if (ids[k] < f.first_node || ids[k] > f.last_node)
        continue;
      if (!untouched[k]) {
        edited = true;
        break;
      }
      const double m = moved(k);
      seg.max_m = std::max(seg.max_m, m);
      if (m > flag_m)
        ++moved_nodes;
    }
    if (edited)
      continue;
    if (seg.max_m > flag_m) {
      if (f.residual_m && f.reference_residual_m) {
        // Worse by more than the sampling noise of a segment median.
        const double margin =
            std::max(kShearWorseM, kShearWorseRel * *f.reference_residual_m);
        seg.sheared = *f.residual_m > *f.reference_residual_m + margin;
      } else {
        seg.sheared = !seg.downstream; // no overlap to judge: conservative
      }
    }
    if (seg.sheared)
      sh.flagged += moved_nodes;
    sh.segments.push_back(seg);
  }
  if (segments.empty())
    sh.flagged = sh.upstream_moved;
  return sh;
}

std::map<int, double>
segment_cross_residuals(const reusex::slam::DriftReport &report) {
  std::map<int, std::vector<double>> by;
  for (const auto &p : report.pairs)
    if (p.a != p.b) {
      by[p.a].push_back(p.residual_m);
      by[p.b].push_back(p.residual_m);
    }
  std::map<int, double> out;
  for (auto &[k, v] : by)
    if (auto m = median(std::move(v)))
      out[k] = *m;
  return out;
}

reusex::geometry::StoredPoseGraph
fast_graph_at(const reusex::geometry::StoredPoseGraph &graph,
              const std::vector<Eigen::Matrix4d> &reference) {
  if (reference.size() != graph.frames.size())
    throw std::invalid_argument(
        fmt::format("fast_graph_at: {} reference poses for {} frames",
                    reference.size(), graph.frames.size()));
  reusex::geometry::StoredPoseGraph out = graph;
  std::vector<Eigen::Matrix4d> optical(reference.size());
  for (std::size_t k = 0; k < out.frames.size(); ++k) {
    auto &f = out.frames[k];
    optical[k] = reference[k] * out.local[k];
    f.world_pose.matrix() = optical[k];
    f.points =
        std::make_shared<reusex::Cloud>(); // no plane factors: no surfels
    f.normals = std::make_shared<reusex::CloudN>();
  }
  out.seed_stored = reference;
  for (auto &e : out.edges)
    if (e.solver) {
      const auto i = static_cast<std::size_t>(e.solver->i);
      const auto j = static_cast<std::size_t>(e.solver->j);
      e.solver->T_ij = optical[i].inverse() * optical[j];
    }
  return out;
}

std::vector<viz::GraphLayerEdge>
candidate_graph_edges(const reusex::geometry::StoredPoseGraph &graph,
                      const std::vector<std::string> &stored_status,
                      const reusex::geometry::PlaneGraphResult &result) {
  std::vector<viz::GraphLayerEdge> out;
  out.reserve(graph.edges.size());
  for (std::size_t k = 0; k < graph.edges.size(); ++k) {
    const auto &e = graph.edges[k];
    viz::GraphLayerEdge g;
    g.from_node_id = e.from_node_id;
    g.to_node_id = e.to_node_id;
    g.edge_type = e.edge_type;
    if (k < stored_status.size())
      g.status = stored_status[k];
    out.push_back(std::move(g));
  }
  for (const auto &rec : result.edges)
    if (rec.tag >= 0 && static_cast<std::size_t>(rec.tag) < out.size())
      out[static_cast<std::size_t>(rec.tag)].residual = rec.residual;
  return out;
}

// ── The session ─────────────────────────────────────────────────────────────

struct OptimizeWatchSession::State {
  SubcommandOptimizeOptions opt;
  RuxOptions global;
  reusex::geometry::PlaneGraphOptions options;

  std::unique_ptr<reusex::ProjectDB> db;
  std::optional<reusex::geometry::StoredPoseGraph> graph;
  std::string load_error;
  double load_seconds = 0.0;
  std::vector<std::int64_t> ids;       ///< db.sensor_frame_ids(), as given
  std::vector<int> graph_index;        ///< per id: index in graph, or -1
  std::vector<Eigen::Matrix4d> stored; ///< per id: `transform` at load
  std::vector<Eigen::Matrix4d> initial;
  /// reusex-rcv.1.21 / .24: the no-edit FULL solve (plain `rux optimize`),
  /// per id. What shear, the guards and the "vs plain optimize" line
  /// measure against, and what `--fast` cycles solve on top of.
  std::vector<Eigen::Matrix4d> reference;
  std::optional<reusex::slam::DriftReport> reference_drift;
  std::optional<double> reference_cross_m;
  std::map<int, double> reference_segment_m; ///< per segment, cross pairs
  std::string reference_note;   ///< why the reference is not the full solve
  std::int64_t gauge_node = -1; ///< the frame the gauge prior holds
  std::vector<reusex::core::StoreyRecord> storeys;
  std::optional<reusex::core::SolidModel> plan;
  std::string plan_note;
  std::string project_guid;

  /// reusex-rcv.1.17: the sampled depth of every frame the drift
  /// diagnostics and the preview cloud back-project, read once at load (the
  /// pixels do not depend on the poses; each cycle only re-transforms them).
  std::optional<reusex::core::FrameDepthCache> depth_cache;
  /// Status of each stored pose-graph row (the graph layer hides rejected
  /// edges; reusex-rcv.1.18 colours them by the candidate solve).
  std::vector<std::string> edge_status;

  std::optional<reusex::core::PoseEdits> edits; ///< last good
  std::vector<Eigen::Matrix4d> shown;           ///< last good solve's poses
  int shown_pass = -1;
  OptimizeWatchPass shown_block; ///< the block of the last good solve
  int passes = 0;

  bool load(std::string &out);
  [[nodiscard]] reusex::slam::DriftDiagnosticsOptions drift_options() const;
  [[nodiscard]] reusex::visualize::PreviewCloudOptions preview_options() const;
  std::vector<Eigen::Matrix4d>
  aligned(const reusex::geometry::StoredGraphSolve &s) const;
};

OptimizeWatchSession::OptimizeWatchSession(SubcommandOptimizeOptions opt,
                                           RuxOptions global_opt)
    : state_(std::make_unique<State>()) {
  state_->opt = std::move(opt);
  state_->global = std::move(global_opt);
  state_->options = plane_graph_options(state_->opt);
  if (state_->opt.fast) {
    state_->options.use_plane_factors = false;
    state_->options.manhattan = false;
  }
}

OptimizeWatchSession::~OptimizeWatchSession() = default;

std::vector<fs::path> OptimizeWatchSession::watched_paths() const {
  return {fs::path(state_->opt.edits)};
}

std::vector<Eigen::Matrix4d> OptimizeWatchSession::State::aligned(
    const reusex::geometry::StoredGraphSolve &s) const {
  std::vector<Eigen::Matrix4d> out = stored;
  for (std::size_t k = 0; k < ids.size(); ++k)
    if (graph_index[k] >= 0)
      out[k] = s.poses[static_cast<std::size_t>(graph_index[k])];
  return out;
}

reusex::slam::DriftDiagnosticsOptions
OptimizeWatchSession::State::drift_options() const {
  reusex::slam::DriftDiagnosticsOptions d;
  d.frame_step = opt.drift_frame_step;
  d.depth_stride = opt.drift_depth_stride;
  d.segments = opt.drift_segments;
  d.doubled_walls = false;
  return d;
}

reusex::visualize::PreviewCloudOptions
OptimizeWatchSession::State::preview_options() const {
  const auto d = drift_options();
  reusex::visualize::PreviewCloudOptions p;
  p.segments = d.segments;
  p.frame_step = d.frame_step;
  p.depth_stride = d.depth_stride;
  p.min_depth_m = d.min_depth_m;
  p.max_depth_m = d.max_depth_m;
  return p;
}

bool OptimizeWatchSession::State::load(std::string &out) {
  using namespace reusex;
  const auto t0 = Clock::now();
  try {
    db = std::make_unique<ProjectDB>(global.project_db,
                                     /*readOnly=*/!opt.apply);
    for (int id : db->sensor_frame_ids())
      ids.push_back(id);
    try {
      project_guid = db->project_guid();
    } catch (const std::exception &) {
      project_guid.clear(); // pre-v30 copy opened read-only: not checked
    }
    if (!opt.plan.empty()) {
      try {
        plan = io::solid_model_from_file(opt.plan);
      } catch (const std::exception &e) {
        plan_note = e.what();
      }
    }
    storeys = plan && !plan->storeys.empty() ? plan->storeys : db->storeys();

    for (const auto &e : db->list_pose_graph_edges())
      edge_status.push_back(e.status);
    // The full options (plane factors on), for the reference solve.
    const auto full_options = plane_graph_options(opt);
    auto loaded = geometry::load_stored_pose_graph(*db, full_options);

    std::map<int, int> index_of;
    for (std::size_t k = 0; k < loaded.frames.size(); ++k)
      index_of[loaded.frames[k].node_id] = static_cast<int>(k);
    graph_index.assign(ids.size(), -1);
    stored.resize(ids.size());
    initial.resize(ids.size());
    for (std::size_t k = 0; k < ids.size(); ++k) {
      const int id = static_cast<int>(ids[k]);
      stored[k] = pg::matrix_from_row_major(db->sensor_frame_pose(id));
      initial[k] = stored[k];
      if (auto it = index_of.find(id); it != index_of.end()) {
        graph_index[k] = it->second;
        initial[k] = loaded.seed_stored[static_cast<std::size_t>(it->second)];
      }
    }
    gauge_node = loaded.frames.front().node_id;

    // reusex-rcv.1.21 / .24: the reference is the no-edit FULL solve. Under
    // --fast the cycles solve the graph linearised at it (fast_graph_at): an
    // edit bends plain optimize's trajectory, not the drifted odometry.
    const auto tr = Clock::now();
    std::vector<Eigen::Matrix4d> ref_graph = loaded.seed_stored;
    try {
      ref_graph =
          geometry::solve_stored_pose_graph(*db, loaded, full_options).poses;
    } catch (const std::exception &e) {
      reference_note = fmt::format("the full solve FAILED ({}); the initial "
                                   "poses stand in",
                                   e.what());
    }
    const double reference_seconds = seconds_since(tr);
    if (opt.fast) {
      graph = fast_graph_at(loaded, ref_graph);
    } else {
      graph = std::move(loaded);
    }
    reference = stored;
    for (std::size_t k = 0; k < ids.size(); ++k)
      if (graph_index[k] >= 0)
        reference[k] = ref_graph[static_cast<std::size_t>(graph_index[k])];

    // One cache serves both: the preview samples as the drift does.
    const auto tc = Clock::now();
    const auto sampling = slam::drift_depth_sampling(drift_options());
    if (!(viz::preview_depth_sampling(preview_options()) == sampling))
      throw std::logic_error("optimize --watch: the preview cloud and the "
                             "drift diagnostics sample the depth apart");
    depth_cache = core::cache_frame_depth(*db, sampling);
    out += fmt::format(
        "frame cache           {} frames, {} depth samples, {:.1f} MB; read "
        "once in {:.1f} s (frame step {}, depth stride {})\n",
        depth_cache->frames.size(), depth_cache->samples(),
        static_cast<double>(depth_cache->bytes()) / (1024.0 * 1024.0),
        seconds_since(tc), sampling.frame_step, sampling.depth_stride);
    // The reference's own drift numbers, once.
    try {
      reference_drift =
          slam::diagnose_drift(*db, drift_options(), plan ? &*plan : nullptr,
                               reference, &*depth_cache);
      std::vector<double> cross;
      for (const auto &p : reference_drift->pairs)
        if (p.a != p.b)
          cross.push_back(p.residual_m);
      reference_cross_m = median(cross);
      reference_segment_m = segment_cross_residuals(*reference_drift);
    } catch (const std::exception &e) {
      reference_note +=
          fmt::format("{}its drift diagnostics FAILED: {}",
                      reference_note.empty() ? "" : "; ", e.what());
    }
    load_seconds = seconds_since(t0);
    out += fmt::format(
        "graph                 {} of {} frames solved, {} "
        "stored edges{}; loaded in {:.1f} s (incl. the reference solve)\n",
        graph->frames.size(), graph->frames_total, graph->edges.size(),
        graph->odometry_in_memory ? ", odometry derived in memory" : "",
        load_seconds);
    out += fmt::format(
        "reference             plain optimize (the no-edit full solve, plane "
        "factors on; {:.1f} s): cross-segment {}, pass-to-pass {}{}\n",
        reference_seconds, mm(reference_cross_m),
        mm(reference_drift ? reference_drift->residual_median_m
                           : std::optional<double>{}),
        reference_note.empty() ? "" : "; " + reference_note);
    if (opt.fast)
      out += "                      --fast cycles solve on top of it (no "
             "plane factors): an edit must beat it\n";
    return true;
  } catch (const std::exception &e) {
    load_error = e.what();
    graph.reset();
    db.reset();
    ids.clear();
    out +=
        fmt::format("graph                 FAILED to load: {}\n", load_error);
    return false;
  }
}

OptimizeWatchPass OptimizeWatchSession::pass(bool edits_changed) {
  using namespace reusex;
  State &s = *state_;
  const auto t0 = Clock::now();
  OptimizeWatchPass r;
  r.index = s.passes++;
  std::string out = fmt::format("── cycle {}: {} ──\n", r.index,
                                fs::path(s.opt.edits).filename().string());

  if (!s.graph && !s.load(out)) {
    r.report = std::move(out);
    r.seconds = seconds_since(t0);
    return r;
  }
  const bool first = s.shown_pass < 0;

  // --- edits: parse, check, lower (inside the solve) -----------------------
  std::optional<core::PoseEdits> candidate;
  if (edits_changed || first) {
    r.edits_read = true;
    try {
      candidate = io::read_pose_edits(s.opt.edits, s.project_guid);
    } catch (const std::exception &e) {
      r.edits_error = e.what();
    }
  }

  // --- solve ----------------------------------------------------------------
  std::optional<geometry::StoredGraphSolve> solve;
  double solve_seconds = 0.0;
  if (candidate) {
    const auto ts = Clock::now();
    try {
      solve = geometry::solve_stored_pose_graph(*s.db, *s.graph, s.options,
                                                &*candidate);
      const auto &res = solve->result;
      const bool nothing = res.landmarks == 0 && res.loop_edges == 0 &&
                           res.edit_factors == 0 && res.graph_edges > 0;
      if (!res.converged && !nothing) {
        r.edits_error = "the solve did not converge (poses unchanged)";
        solve.reset();
      }
    } catch (const std::exception &e) {
      r.edits_error = e.what(); // lowering names the edit
      solve.reset();
    }
    solve_seconds = seconds_since(ts);
  }
  r.edits_ok = solve.has_value();

  if (r.edits_read && !r.edits_ok) {
    out += fmt::format("edits                 INVALID: {}\n", r.edits_error);
    if (!first) {
      out += fmt::format("                      keeping the last good solve "
                         "(cycle {})\n",
                         s.shown_pass);
      r.shown_solve = s.shown_pass;
      r.poses = s.shown;
      r.cross_segment_median_m = s.shown_block.cross_segment_median_m;
      r.shear = s.shown_block.shear;
      r.suggestions_yaml = s.shown_block.suggestions_yaml;
      r.suggestion_items = s.shown_block.suggestion_items;
      out += "images                unchanged\n";
      r.report = std::move(out);
      r.seconds = seconds_since(t0);
      return r;
    }
    // Nothing good yet: show the solve without edits.
    out += "                      no good edits yet: showing the solve "
           "without edits\n";
    candidate = core::PoseEdits{};
    try {
      solve = geometry::solve_stored_pose_graph(*s.db, *s.graph, s.options);
    } catch (const std::exception &e) {
      out += fmt::format("solve                 FAILED: {}\n", e.what());
      r.report = std::move(out);
      r.seconds = seconds_since(t0);
      return r;
    }
  } else if (!r.edits_read) {
    out += "edits                 unchanged\n";
    r.report = std::move(out);
    r.shown_solve = s.shown_pass;
    r.poses = s.shown;
    r.seconds = seconds_since(t0);
    return r;
  }

  // A good solve: it is what the block describes from here on.
  r.solved = true;
  r.shown_solve = r.index;
  const core::PoseEdits &edits = *candidate;
  r.poses = s.aligned(*solve);
  const auto &res = solve->result;
  {
    std::map<std::string, int> kinds;
    for (const auto &e : edits.edits)
      ++kinds[std::string(core::to_string(e.kind))];
    std::vector<std::string> parts;
    for (const auto &[k, n] : kinds)
      parts.push_back(fmt::format("{} {}", n, k));
    out += fmt::format(
        "edits                 ok: {} edit{}{} (frame {})\n",
        edits.edits.size(), edits.edits.size() == 1 ? "" : "s",
        parts.empty() ? std::string()
                      : " (" + fmt::format("{}", fmt::join(parts, ", ")) + ")",
        edits.frame);
  }
  out += fmt::format(
      "solve                 {}{}: {} frames, {} landmarks, {} loop "
      "edges, {} edit factors, error {:.3f} -> {:.3f}, {:.1f} s\n",
      res.converged ? "converged" : "nothing to solve",
      s.opt.fast ? " (--fast: no plane factors)" : "", res.frames,
      res.landmarks, res.loop_edges, res.edit_factors, res.initial_error,
      res.final_error, solve_seconds);
  if (!res.edits_skipped.empty())
    out += fmt::format("                      not lowered: {}\n",
                       fmt::join(res.edits_skipped, ", "));
  json j;
  j["cycle"] = r.index;
  j["edits_file"] = s.opt.edits;
  j["edits"] = json::array();
  for (const auto &e : res.edit_residuals) {
    // reusex-rcv.1.28: judged by the edit's own residual against its sigma,
    // not by the per-factor GNC threshold alone.
    const core::PoseEdit *named = nullptr;
    for (const auto &x : edits.edits)
      if (x.id == e.tag)
        named = &x;
    // reusex-9ge: a move's block, initial vs solved, against its delta (in
    // the world frame, about the default pivot the lowering uses).
    std::optional<MoveRealisation> realised;
    if (named != nullptr && named->kind == core::PoseEditKind::move) {
      const Eigen::Matrix4d to_doc = document_frame(edits.frame, s.storeys);
      std::vector<Eigen::Matrix4d> from, to;
      Eigen::Vector3d pivot = Eigen::Vector3d::Zero();
      for (std::size_t k = 0; k < s.ids.size(); ++k)
        if (s.graph_index[k] >= 0 && named->nodes.contains(s.ids[k])) {
          from.push_back(s.initial[k]);
          to.push_back(r.poses[k]);
          pivot += (to_doc * s.initial[k]).topRightCorner<3, 1>();
        }
      if (!from.empty()) {
        pivot /= static_cast<double>(from.size());
        const Eigen::Matrix4d d =
            to_doc.inverse() * named->delta.to_matrix(pivot) * to_doc;
        realised = move_realisation(from, to, d);
      }
    }
    const EditVerdict v = judge_edit(e, named, realised ? &*realised : nullptr);
    out += fmt::format("  edit {:<24} {:>8.4f} m {:>7.3f} deg (rms {:.4f} m "
                       "{:.3f} deg)  chi2 {:>9.2f}  {} factor(s)  {}\n",
                       e.tag, e.trans_m, e.rot_deg, e.rms_trans_m,
                       e.rms_rot_deg, e.chi2, e.factors,
                       v.honoured ? std::string("honoured")
                                  : "NOT HONOURED: " + v.why);
    if (realised)
      out += fmt::format("       {:<24} {}\n", "",
                         describe_realisation(*realised));
    json je = {{"id", e.tag},
               {"trans_m", e.trans_m},
               {"rot_deg", e.rot_deg},
               {"rms_trans_m", e.rms_trans_m},
               {"rms_rot_deg", e.rms_rot_deg},
               {"chi2", e.chi2},
               {"factors", e.factors},
               {"inlier", e.gnc_inlier},
               {"honoured", v.honoured},
               {"why", v.why}};
    if (realised)
      je["realisation"] = {{"move_trans_m", realised->move_trans_m},
                           {"move_rot_deg", realised->move_rot_deg},
                           {"remaining_trans_m", realised->remaining_trans_m},
                           {"remaining_rot_deg", realised->remaining_rot_deg},
                           {"share_trans", realised->share_trans},
                           {"share_rot", realised->share_rot},
                           {"bend_trans_m", realised->bend_trans_m},
                           {"bend_rot_deg", realised->bend_rot_deg}};
    j["edits"].push_back(std::move(je));
  }

  // --- pose change and shear ------------------------------------------------
  const auto vs_last =
      s.shown.empty() ? PoseChange{} : pose_change(s.ids, r.poses, s.shown);
  const auto vs_initial = pose_change(s.ids, r.poses, s.initial);
  out += fmt::format(
      "pose change           vs last cycle: max {:.3f} m (node {}), median "
      "{:.4f} m; vs initial: max {:.3f} m (node {}), median {:.4f} m, max "
      "{:.2f} deg\n",
      vs_last.max_m, vs_last.max_node, vs_last.median_m, vs_initial.max_m,
      vs_initial.max_node, vs_initial.median_m, vs_initial.max_rot_deg);
  auto change_json = [](const PoseChange &c) {
    return json{{"nodes", c.nodes},       {"median_m", c.median_m},
                {"p90_m", c.p90_m},       {"max_m", c.max_m},
                {"max_node", c.max_node}, {"max_rot_deg", c.max_rot_deg}};
  };
  j["pose_change"] = {{"vs_last", change_json(vs_last)},
                      {"vs_initial", change_json(vs_initial)}};

  const auto touched = edited_nodes(edits, s.ids);
  std::vector<bool> untouched(s.ids.size(), true);
  for (std::size_t k = 0; k < s.ids.size(); ++k)
    untouched[k] =
        !std::binary_search(touched.begin(), touched.end(), s.ids[k]);
  // --- drift diagnostics on the candidate poses -----------------------------
  const slam::DriftDiagnosticsOptions dopt = s.drift_options();
  std::optional<slam::DriftReport> drift;
  double drift_seconds = 0.0;
  {
    const auto td = Clock::now();
    try {
      drift = slam::diagnose_drift(*s.db, dopt, s.plan ? &*s.plan : nullptr,
                                   r.poses,
                                   s.depth_cache ? &*s.depth_cache : nullptr);
    } catch (const std::exception &e) {
      out += fmt::format("drift                 FAILED: {}\n", e.what());
    }
    drift_seconds = seconds_since(td);
  }

  // reusex-rcv.1.22: shear against the reference, judged by the fit.
  {
    std::vector<ShearSegmentFit> fits;
    if (drift) {
      const auto cand = segment_cross_residuals(*drift);
      for (const auto &g : drift->segments) {
        ShearSegmentFit f{g.index, g.first_node, g.last_node, {}, {}};
        if (auto it = cand.find(g.index); it != cand.end())
          f.residual_m = it->second;
        if (auto it = s.reference_segment_m.find(g.index);
            it != s.reference_segment_m.end())
          f.reference_residual_m = it->second;
        fits.push_back(f);
      }
    }
    r.shear = shear_indicator(s.ids, r.poses, s.reference, untouched, fits,
                              s.opt.shear_flag_m);
  }

  std::vector<std::string> warnings;
  if (drift) {
    const auto &d = *drift;
    std::vector<double> cross;
    for (const auto &p : d.pairs)
      if (p.a != p.b)
        cross.push_back(p.residual_m);
    r.cross_segment_median_m = median(cross);
    out += fmt::format(
        "sampling              frame step {}, depth stride {}, {} segments "
        "(pinned: the residuals depend on them, reusex-74q)\n",
        dopt.frame_step, dopt.depth_stride, dopt.segments);
    out += fmt::format("noise floor           {} (p90 {})\n",
                       mm(d.noise_floor_m), mm(d.noise_floor_p90_m));
    out += fmt::format("pass-to-pass          {} (p90 {}); {} of {} frames "
                       "flagged\n",
                       mm(d.residual_median_m), mm(d.residual_p90_m),
                       d.frames_flagged, d.revisit_frames);
    out += fmt::format("cross-segment         median {} over {} pairs "
                       "(halves: within {}, across {})\n",
                       mm(r.cross_segment_median_m), cross.size(),
                       mm(d.halves.within_m), mm(d.halves.across_m));
  }
  // reusex-rcv.1.24: plain optimize on the same sampling, every cycle: the
  // edits must beat doing nothing.
  r.reference_cross_segment_median_m = s.reference_cross_m;
  if (s.reference_drift)
    r.reference_pass_to_pass_m = s.reference_drift->residual_median_m;
  const auto vs_reference = pose_change(s.ids, r.poses, s.reference);
  if (r.cross_segment_median_m && s.reference_cross_m) {
    const double diff = *r.cross_segment_median_m - *s.reference_cross_m;
    r.beats_reference = diff < -kSameAsReferenceM;
    out += fmt::format(
        "vs plain optimize     cross-segment {} vs {} ({}: {:+.1f} mm; under "
        "~1 mm is sampling noise); pass-to-pass {} vs {}; poses median "
        "{:.4f} m, max {:.3f} m from it\n",
        mm(r.cross_segment_median_m), mm(s.reference_cross_m),
        *r.beats_reference         ? "better"
        : diff > kSameAsReferenceM ? "WORSE than no edits"
                                   : "the same",
        diff * 1000.0,
        mm(drift ? drift->residual_median_m : std::optional<double>{}),
        mm(r.reference_pass_to_pass_m), vs_reference.median_m,
        vs_reference.max_m);
  } else {
    out += fmt::format("vs plain optimize     n/a (no cross-segment median"
                       "{})\n",
                       s.reference_note.empty() ? "" : "; " + s.reference_note);
  }
  j["reference"] = {
      {"cross_segment_median_m",
       s.reference_cross_m ? json(*s.reference_cross_m) : json()},
      {"residual_median_m",
       r.reference_pass_to_pass_m ? json(*r.reference_pass_to_pass_m) : json()},
      {"beats", r.beats_reference ? json(*r.beats_reference) : json()},
      {"pose_change", change_json(vs_reference)},
      {"note", s.reference_note}};
  // reusex-sb2 / rcv.1.22: the GT-free numbers above reward a sheared
  // trajectory; this says which untouched parts moved and whether they now
  // fit worse than in plain optimize (that, not movement, is shear).
  out += fmt::format(
      "shear (untouched)     {} nodes no edit names: vs plain optimize "
      "median {:.4f} m, p90 {:.3f} m, max {:.3f} m (node {}); moved > {:.2f} "
      "m: {} upstream (gauge side), {} downstream (carried tail, expected); "
      "{} in segments that now fit worse{}\n",
      r.shear.untouched, r.shear.change.median_m, r.shear.change.p90_m,
      r.shear.change.max_m, r.shear.change.max_node, s.opt.shear_flag_m,
      r.shear.upstream_moved, r.shear.downstream_moved, r.shear.flagged,
      r.shear.flagged > 0 ? "  <- SHEAR" : "");
  {
    std::vector<std::string> segs;
    for (const auto &g : r.shear.segments)
      if (g.max_m > s.opt.shear_flag_m)
        segs.push_back(fmt::format(
            "seg {} ({}-{}) {:.3f} m, {} vs {}{}{}", g.index, g.first_node,
            g.last_node, g.max_m, mm(g.residual_m), mm(g.reference_residual_m),
            g.downstream ? ", carried" : "", g.sheared ? " SHEARED" : ""));
    if (!segs.empty())
      out += fmt::format("                      untouched segments moved "
                         "(residual vs plain optimize): {}\n",
                         fmt::join(segs, "; "));
  }
  j["shear"] = {{"untouched", r.shear.untouched},
                {"flagged", r.shear.flagged},
                {"upstream_moved", r.shear.upstream_moved},
                {"downstream_moved", r.shear.downstream_moved},
                {"reference", "no-edit full solve"},
                {"flag_m", s.opt.shear_flag_m},
                {"change", change_json(r.shear.change)}};
  j["shear"]["segments"] = json::array();
  for (const auto &g : r.shear.segments)
    j["shear"]["segments"].push_back(
        {{"segment", g.index},
         {"first_node", g.first_node},
         {"last_node", g.last_node},
         {"max_m", g.max_m},
         {"residual_m", g.residual_m ? json(*g.residual_m) : json()},
         {"reference_residual_m",
          g.reference_residual_m ? json(*g.reference_residual_m) : json()},
         {"downstream", g.downstream},
         {"sheared", g.sheared}});

  if (drift) {
    const auto &d = *drift;
    j["sampling"] = {{"frame_step", dopt.frame_step},
                     {"depth_stride", dopt.depth_stride},
                     {"segments", dopt.segments}};
    j["drift"] = {
        {"noise_floor_m", d.noise_floor_m ? json(*d.noise_floor_m) : json()},
        {"residual_median_m",
         d.residual_median_m ? json(*d.residual_median_m) : json()},
        {"residual_p90_m", d.residual_p90_m ? json(*d.residual_p90_m) : json()},
        {"cross_segment_median_m",
         r.cross_segment_median_m ? json(*r.cross_segment_median_m) : json()},
        {"frames_flagged", d.frames_flagged}};

    // Worst five segment pairs.
    auto pairs = d.pairs;
    std::sort(pairs.begin(), pairs.end(), [](const auto &a, const auto &b) {
      return a.residual_m != b.residual_m
                 ? a.residual_m > b.residual_m
                 : std::tie(a.a, a.b) < std::tie(b.a, b.b);
    });
    auto seg_nodes = [&](int i) {
      for (const auto &g : d.segments)
        if (g.index == i)
          return fmt::format("{}-{}", g.first_node, g.last_node);
      return std::string("?");
    };
    out += "worst segment pairs   (segment a into b)\n";
    j["worst_pairs"] = json::array();
    for (std::size_t i = 0; i < std::min<std::size_t>(5, pairs.size()); ++i) {
      out += fmt::format("  {:>2} ({:>11}) into {:>2} ({:>11})  {:6.1f} mm  "
                         "{} points\n",
                         pairs[i].a, seg_nodes(pairs[i].a), pairs[i].b,
                         seg_nodes(pairs[i].b), pairs[i].residual_m * 1000.0,
                         pairs[i].points);
      j["worst_pairs"].push_back({{"a", pairs[i].a},
                                  {"b", pairs[i].b},
                                  {"residual_m", pairs[i].residual_m},
                                  {"points", pairs[i].points}});
    }

    // --- suggestions as ready-to-paste YAML --------------------------------
    auto sug = d.suggestions;
    std::sort(sug.begin(), sug.end(), [](const auto &a, const auto &b) {
      const double ga = a.before_m - a.after_m, gb = b.before_m - b.after_m;
      return ga != gb ? ga > gb : a.segment < b.segment;
    });
    std::set<std::string> taken;
    for (const auto &e : edits.edits)
      taken.insert(e.id);
    auto unique_id = [&](std::string id) {
      std::string u = id;
      for (int n = 2; taken.count(u) > 0; ++n)
        u = fmt::format("{}-{}", id, n);
      taken.insert(u);
      return u;
    };
    const Eigen::Matrix4d to_doc = document_frame(edits.frame, s.storeys);
    std::string yaml;
    j["suggestions"] = json::array();
    int emitted = 0;
    std::string gauge_comment;
    // reusex-rcv.1.28: each segment's residual now, to judge whether the
    // move a suggestion would replace delivered the gain it predicted.
    std::map<int, double> segment_now;
    for (const auto &g : d.suggestions)
      segment_now[g.segment] = g.before_m;
    std::string held_back_comment;
    j["held_back"] = json::array();
    for (const auto &g : sug) {
      if (emitted >= s.opt.suggestions || g.after_m >= g.before_m)
        break;
      // reusex-rcv.1.23: the segment holding the gauge prior is never
      // suggested. Its move outvotes the one gauge prior and the whole
      // trajectory follows it rigidly: it is the inverse move of all the
      // others, which the suggestions for those segments already cover.
      if (s.gauge_node >= g.first_node && s.gauge_node <= g.last_node) {
        gauge_comment = fmt::format(
            "  # seg {} ({}-{}: {:.1f} -> {:.1f} mm) holds the gauge (node "
            "{}): not suggested -- moving it moves the whole trajectory\n",
            g.segment, g.first_node, g.last_node, g.before_m * 1000.0,
            g.after_m * 1000.0, s.gauge_node);
        continue;
      }
      // The segment's nodes and where the candidate solve puts them.
      std::vector<std::int64_t> nodes;
      std::vector<Eigen::Vector3d> positions;
      std::vector<std::size_t> at;
      for (std::size_t k = 0; k < s.ids.size(); ++k)
        if (s.ids[k] >= g.first_node && s.ids[k] <= g.last_node &&
            s.graph_index[k] >= 0) {
          nodes.push_back(s.ids[k]);
          positions.push_back(r.poses[k].topRightCorner<3, 1>());
          at.push_back(k);
        }
      if (nodes.empty())
        continue;
      // reusex-sb2: a segment that changes storey is not one rigid block;
      // the move covers its longest single-storey run.
      const auto runs = storey_runs(nodes, positions, s.storeys);
      const auto best = std::max_element(
          runs.begin(), runs.end(),
          [](const auto &a, const auto &b) { return a.frames < b.frames; });
      std::vector<Eigen::Matrix4d> from, to;
      for (std::size_t i = 0; i < nodes.size(); ++i)
        if (nodes[i] >= best->first && nodes[i] <= best->last) {
          from.push_back(s.initial[at[i]]);
          to.push_back(g.delta * r.poses[at[i]]);
        }
      // The suggestion is measured on the candidate; a move applies to the
      // initial poses: fit initial -> corrected candidate.
      const Eigen::Matrix4d delta_world = fit_rigid_delta(from, to);
      const Eigen::Matrix4d delta_doc = to_doc * delta_world * to_doc.inverse();
      Eigen::Vector3d c = Eigen::Vector3d::Zero();
      for (const auto &f : from)
        c += (to_doc * f).topRightCorner<3, 1>();
      c /= static_cast<double>(from.size());
      for (int i = 0; i < 3; ++i)
        c[i] = round_to(c[i], 0.01);
      // About pivot c: p' = c + xyz + R (p - c), so xyz = t + R c - c.
      core::RigidTransform delta = readable(delta_doc);
      const Eigen::Vector3d xyz = delta_doc.topRightCorner<3, 1>() +
                                  delta_doc.topLeftCorner<3, 3>() * c - c;
      for (int i = 0; i < 3; ++i)
        delta.xyz[static_cast<std::size_t>(i)] = round_to(xyz[i], 1e-4);
      delta.pivot = std::array<double, 3>{c.x(), c.y(), c.z()};

      std::vector<std::string> overlaps;
      for (const auto &e : edits.edits)
        if (e.kind == core::PoseEditKind::move && !e.nodes.is_list() &&
            e.nodes.first <= best->last && e.nodes.last >= best->first)
          overlaps.push_back(e.id);
      // reusex-rcv.1.28: no REPLACES ping-pong. A suggested move is not
      // replaced until its predicted gain came true; a realised one is, and
      // the comment says so.
      std::optional<std::string> held;
      std::vector<std::string> realised;
      for (const auto &e : edits.edits) {
        if (std::find(overlaps.begin(), overlaps.end(), e.id) == overlaps.end())
          continue;
        if (auto why = blocked_replacement(e, segment_now)) {
          held = std::move(why);
          break;
        }
        if (move_realised(e, segment_now).value_or(false))
          realised.push_back(e.id);
      }
      if (held) {
        held_back_comment += fmt::format(
            "  # seg {} ({}-{}: {:.1f} -> {:.1f} mm): held back -- {}\n",
            g.segment, best->first, best->last, g.before_m * 1000.0,
            g.after_m * 1000.0, *held);
        j["held_back"].push_back({{"segment", g.segment},
                                  {"first_node", best->first},
                                  {"last_node", best->last},
                                  {"before_m", g.before_m},
                                  {"after_m", g.after_m},
                                  {"why", *held}});
        continue;
      }
      const std::string realised_note =
          realised.empty() ? std::string()
                           : fmt::format(" (its predicted gain realised: {})",
                                         fmt::join(realised, ", "));

      core::PoseEdit m;
      m.id = unique_id(fmt::format("m-{}-{}", best->first, best->last));
      m.kind = core::PoseEditKind::move;
      m.nodes.first = best->first;
      m.nodes.last = best->last;
      m.delta = delta;
      // A block carries one anchor's authority (docs/design/pose-edits.md):
      // as tight as one odometry link, so the link at the break yields.
      m.sigma_trans_m = kMoveSigmaM;
      m.sigma_rot_deg = kMoveSigmaDeg;
      std::string left_out;
      for (const auto &run : runs)
        if (&run != &*best)
          left_out +=
              fmt::format("{}{}-{} (storey {})", left_out.empty() ? "" : ", ",
                          run.first, run.last, run.storey);
      m.note = fmt::format(
          "cycle {}: seg {} ({}-{}) into the rest {:.1f} -> {:.1f} mm "
          "(segment ICP on the candidate solve){}{}",
          r.index, g.segment, g.first_node, g.last_node, g.before_m * 1000.0,
          g.after_m * 1000.0,
          left_out.empty()
              ? std::string()
              : fmt::format("; split at the storey change, left out {}",
                            left_out),
          overlaps.empty()
              ? std::string()
              : fmt::format("; replaces {}{}",
                            fmt::format("{}", fmt::join(overlaps, ", ")),
                            realised_note));
      const std::string comment =
          fmt::format("  # seg {}: {:.1f} -> {:.1f} mm, move {:.3f} m "
                      "(centroid {:.3f} m), {:.2f} deg{}{}\n",
                      g.segment, g.before_m * 1000.0, g.after_m * 1000.0,
                      g.shift_m, g.centroid_shift_m, g.rotation_deg,
                      left_out.empty() ? "" : "; SPLIT at a storey change",
                      overlaps.empty() ? std::string()
                                       : fmt::format("; REPLACES {}{}",
                                                     fmt::join(overlaps, ", "),
                                                     realised_note));
      const std::string item = comment + edit_yaml(m);
      r.suggestion_items.push_back(item);
      yaml += item;
      j["suggestions"].push_back({{"id", m.id},
                                  {"segment", g.segment},
                                  {"first_node", m.nodes.first},
                                  {"last_node", m.nodes.last},
                                  {"before_m", g.before_m},
                                  {"after_m", g.after_m},
                                  {"split", !left_out.empty()}});
      ++emitted;
    }
    // Shear guards: pin each SHEARED untouched segment (moved, and fits
    // worse than in plain optimize) at its middle node, all six DoF, where
    // the reference -- the no-edit FULL solve -- has it (reusex-rcv.1.21:
    // never at the drifted --fast pose, which froze the drift). The
    // regression-221 lesson: anchors in the middle, not relative
    // constraints between the ends; measured on house2, [x, y, yaw] anchors
    // leave the vertical shear in place. Without residuals the untouched
    // node that moved most is pinned instead.
    std::vector<std::pair<std::size_t, double>> guards; // index, moved
    for (const auto &g : r.shear.segments) {
      if (!g.sheared)
        continue;
      std::vector<std::size_t> in;
      for (std::size_t k = 0; k < s.ids.size(); ++k)
        if (s.ids[k] >= g.first_node && s.ids[k] <= g.last_node &&
            s.graph_index[k] >= 0)
          in.push_back(k);
      if (!in.empty())
        guards.emplace_back(in[in.size() / 2], g.max_m);
    }
    if (guards.empty() && r.shear.segments.empty() && r.shear.flagged > 0 &&
        r.shear.change.max_node >= 0) {
      const auto it =
          std::find(s.ids.begin(), s.ids.end(), r.shear.change.max_node);
      guards.emplace_back(static_cast<std::size_t>(it - s.ids.begin()),
                          r.shear.change.max_m);
    }
    for (const auto &[k, moved] : guards) {
      core::PoseEdit a;
      a.id = unique_id(fmt::format("a-{}", s.ids[k]));
      a.kind = core::PoseEditKind::anchor;
      a.nodes.first = a.nodes.last = s.ids[k];
      a.pose.source = core::PoseSpec::Source::given;
      a.pose.value = readable(to_doc * s.reference[k]);
      a.sigma_trans_m = 2.0 * kMoveSigmaM;
      a.sigma_rot_deg = 2.0 * kMoveSigmaDeg;
      a.note = fmt::format("cycle {}: shear guard, no edit names node {} "
                           "but its segment moved {:.3f} m from plain "
                           "optimize and fits worse",
                           r.index, s.ids[k], moved);
      const std::string comment =
          fmt::format("  # shear guard: node {} (its segment moved up to "
                      "{:.3f} m without an edit naming it and fits worse); "
                      "pin it where plain optimize had it\n",
                      s.ids[k], moved);
      const std::string item = comment + edit_yaml(a);
      r.suggestion_items.push_back(item);
      yaml += item;
      j["suggestions"].push_back({{"id", a.id}, {"shear_guard", true}});
    }
    r.suggestions_yaml = yaml;
    if (!gauge_comment.empty())
      j["gauge_segment_skipped"] = true;
    out += "suggestions           (paste under `edits:`: ONE move per cycle, "
           "the first is the best -- each is measured against the others as "
           "they are -- plus the shear guards when offered)\n";
    out += gauge_comment;
    out += held_back_comment;
    out +=
        yaml.empty() ? "  # none: no segment move lowers its residual\n" : yaml;
    j["suggestions_yaml"] = yaml;

    if (d.crossings) {
      out += fmt::format("crossings             {} of {} plan-wall crossings "
                         "are through a wall ({} steps)\n",
                         d.crossings->through_wall(),
                         d.crossings->crossings.size(), d.crossings->steps);
      j["crossings"] = {{"through_wall", d.crossings->through_wall()},
                        {"crossings", d.crossings->crossings.size()}};
    }
    for (const auto &w : d.warnings)
      warnings.push_back(w);
  }
  if (!s.plan_note.empty())
    out += fmt::format("plan                  not read: {}\n", s.plan_note);

  // --- preview cloud, wall support, images ---------------------------------
  double render_seconds = 0.0, preview_seconds = 0.0;
  const viz::PreviewCloudOptions popt = s.preview_options();
  std::shared_ptr<const viz::PreviewCloud> preview;
  if (!s.opt.render.empty() || s.plan) {
    const auto tr = Clock::now();
    try {
      preview = std::make_shared<const viz::PreviewCloud>(viz::preview_cloud(
          *s.db, popt, r.poses, s.depth_cache ? &*s.depth_cache : nullptr));
    } catch (const std::exception &e) {
      out += fmt::format("preview cloud         FAILED: {}\n", e.what());
    }
    preview_seconds = seconds_since(tr);
    render_seconds += preview_seconds;
  }
  if (s.plan && preview) {
    Cloud cloud;
    cloud.reserve(preview->points.size());
    for (const auto &p : preview->points) {
      PointT q;
      q.x = p.x();
      q.y = p.y();
      q.z = p.z();
      cloud.push_back(q);
    }
    const geometry::PlanSupportOptions support_opt;
    out += format_plan_support(
        geometry::plan_support(cloud, *s.plan, support_opt), support_opt);
  }
  if (!s.opt.render.empty() && preview) {
    const auto tr = Clock::now();
    try {
      viz::RenderOptions base;
      base.width = s.opt.render_width;
      base.height = s.opt.render_height;
      base.passes = popt;
      base.poses = r.poses;
      base.preview = preview;
      base.edits = edits;
      // The graph panel shows which edges fight THIS cycle's edits.
      base.graph_edges =
          candidate_graph_edges(*s.graph, s.edge_status, solve->result);
      base.frame_quantile = 0.002;
      if (drift)
        base.drift_heatmap =
            viz::drift_heatmap_from_json(slam::drift_report_to_json(*drift));
      const fs::path stem = s.opt.render;
      auto write = [&](const std::string &panel, const viz::RenderOptions &o) {
        const fs::path p =
            stem.parent_path() / (stem.stem().string() + "." + panel + ".png");
        if (p.has_parent_path())
          fs::create_directories(p.parent_path());
        const cv::Mat img = viz::render_view(*s.db, o);
        if (!cv::imwrite(p.string(), img))
          throw std::runtime_error("cannot write " + p.string());
        r.written.push_back(p);
      };
      // One plan cut per storey (1.2 m above its floor), or one.
      std::vector<std::pair<std::string, std::optional<double>>> cuts;
      if (s.storeys.size() > 1) {
        double z0 = s.storeys.front().z_floor;
        for (const auto &st : s.storeys)
          z0 = std::min(z0, st.z_floor);
        for (const auto &st : s.storeys)
          cuts.emplace_back(fmt::format("s{}.", st.idx), st.z_floor - z0 + 1.2);
      } else {
        cuts.emplace_back("", std::nullopt);
      }
      for (const auto &[prefix, h] : cuts) {
        viz::RenderOptions o = base;
        o.view = viz::ViewPreset::plan;
        o.cut = true;
        o.cut_height = h;
        o.layers = {viz::Layer::passes};
        write(prefix + "passes", o);
        o.layers = {viz::Layer::passes, viz::Layer::graph};
        write(prefix + "graph", o);
        if (o.drift_heatmap) {
          o.layers = {viz::Layer::passes, viz::Layer::drift};
          write(prefix + "drift", o);
        }
        if (s.plan) { // its own panel: the filled spaces hide the passes
          o.layers = {viz::Layer::passes};
          o.plan = s.plan;
          write(prefix + "plan", o);
        }
      }
      viz::RenderOptions o = base;
      o.view = viz::ViewPreset::front;
      o.layers = {viz::Layer::passes};
      write("front", o);
      // reusex-rcv.1.24: plain optimize (the reference) once, at load, so
      // every cycle's passes can be laid next to it: <stem>.ref.*.png.
      if (first) {
        viz::RenderOptions ref = base;
        ref.poses = s.reference;
        ref.preview = std::make_shared<const viz::PreviewCloud>(
            viz::preview_cloud(*s.db, popt, s.reference,
                               s.depth_cache ? &*s.depth_cache : nullptr));
        ref.edits.reset();
        ref.graph_edges.reset();
        ref.drift_heatmap.reset();
        for (const auto &[prefix, h] : cuts) {
          viz::RenderOptions q = ref;
          q.view = viz::ViewPreset::plan;
          q.cut = true;
          q.cut_height = h;
          q.layers = {viz::Layer::passes};
          write("ref." + prefix + "passes", q);
        }
        ref.view = viz::ViewPreset::front;
        ref.layers = {viz::Layer::passes};
        write("ref.front", ref);
      }
      r.rendered = true;
    } catch (const std::exception &e) {
      r.render_error = e.what();
    }
    render_seconds += seconds_since(tr);
  }
  if (!s.opt.render.empty())
    out +=
        r.rendered
            ? fmt::format("images                {}\n", images_line(r.written))
            : fmt::format("images                FAILED: {}\n", r.render_error);
  j["images"] = json::array();
  for (const auto &p : r.written)
    j["images"].push_back(p.string());

  // --- apply ----------------------------------------------------------------
  if (s.opt.apply) {
    try {
      ProjectDB::Transaction tx(*s.db);
      for (std::size_t k = 0; k < s.ids.size(); ++k)
        if (s.graph_index[k] >= 0)
          s.db->update_sensor_frame_pose(static_cast<int>(s.ids[k]),
                                         pg::row_major_from_matrix(r.poses[k]));
      tx.commit();
      r.applied = true;
      out += "project               poses WRITTEN to `transform` (--apply); "
             "run `rux create clouds` to rebuild the stored cloud\n";
    } catch (const std::exception &e) {
      out +=
          fmt::format("project               --apply FAILED: {}\n", e.what());
    }
  } else {
    out += "project               not written (preview only; --apply writes "
           "`transform`); the stored cloud is not rebuilt\n";
  }
  for (const auto &w : warnings)
    out += "warning: " + w + "\n";

  r.seconds = seconds_since(t0);
  out += fmt::format("timing                solve {:.1f} s, drift {:.1f} s, "
                     "preview + images {:.1f} s (preview {:.1f} s), cycle "
                     "{:.1f} s\n",
                     solve_seconds, drift_seconds, render_seconds,
                     preview_seconds, r.seconds);
  j["applied"] = r.applied;
  j["timing"] = {{"solve_s", solve_seconds},
                 {"drift_s", drift_seconds},
                 {"render_s", render_seconds},
                 {"preview_s", preview_seconds},
                 {"cycle_s", r.seconds}};
  r.json = j.dump(2);
  r.report = std::move(out);

  s.edits = edits;
  s.shown = r.poses;
  s.shown_pass = r.index;
  s.shown_block = r;
  if (!s.opt.json.empty()) {
    try {
      if (s.opt.json.has_parent_path())
        fs::create_directories(s.opt.json.parent_path());
      std::ofstream(s.opt.json, std::ios::trunc) << r.json << "\n";
    } catch (const std::exception &e) {
      spdlog::warn("--json {}: {}", s.opt.json.string(), e.what());
    }
  }
  return r;
}

void request_optimize_watch_stop() {
  g_stop.store(true, std::memory_order_release);
}

int run_optimize_watch(SubcommandOptimizeOptions const &opt,
                       const RuxOptions &global_opt) {
  if (opt.edits.empty()) {
    spdlog::error("optimize --watch needs --edits <file>: the edits are what "
                  "it watches");
    return RuxError::INVALID_ARGUMENT;
  }
  StopOnSignal stop_on_signal;
  OptimizeWatchSession session(opt, global_opt);
  reusex::utils::FileWatcher watcher(session.watched_paths());

  OptimizeWatchPass first = session.pass(true);
  std::cout << first.report << std::flush;
  if (first.shown_solve < 0)
    return RuxError::GENERIC; // the graph did not load

  const auto interval = std::chrono::milliseconds(opt.watch_interval_ms);
  int cycles = 0;
  if (opt.watch_cycles != 0)
    spdlog::info("Watching {} (Ctrl-C to stop)", opt.edits);
  while (opt.watch_cycles < 0 || cycles < opt.watch_cycles) {
    const auto changed = watcher.wait(interval, g_stop);
    if (changed.empty())
      break; // stopped
    std::cout << session.pass(true).report << std::flush;
    ++cycles;
  }
  spdlog::info("Watch stopped after {} re-solve(s)", cycles);
  return RuxError::SUCCESS;
}

} // namespace rux
