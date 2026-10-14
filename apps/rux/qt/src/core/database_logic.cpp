// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <rux_qt/database_logic.hpp>
#include <rux_qt/pair_alignment.hpp>
#include <set>
#include <tuple>

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstdio>
#include <iterator>
#include <limits>
#include <numeric>

namespace rux::qt {

// ----------------------------------------------------------- frame pair --

void FramePair::reset(std::vector<int> ids) {
  std::sort(ids.begin(), ids.end());
  ids.erase(std::unique(ids.begin(), ids.end()), ids.end());
  ids_ = std::move(ids);
  if (ids_.empty()) {
    a_ = b_ = -1;
    return;
  }
  a_ = 0;
  b_ = ids_.size() > 1 ? 1 : 0;
}

int FramePair::a_id() const { return a_ < 0 ? -1 : ids_[a_]; }
int FramePair::b_id() const { return b_ < 0 ? -1 : ids_[b_]; }

bool FramePair::set(int &slot, int index, int size) {
  if (index < 0 || index >= size || index == slot)
    return false;
  slot = index;
  return true;
}

bool FramePair::step_a(int delta) {
  if (empty())
    return false;
  return set(a_, std::clamp(a_ + delta, 0, size() - 1), size());
}

bool FramePair::step_b(int delta) {
  if (empty())
    return false;
  return set(b_, std::clamp(b_ + delta, 0, size() - 1), size());
}

bool FramePair::set_a_index(int index) { return set(a_, index, size()); }
bool FramePair::set_b_index(int index) { return set(b_, index, size()); }

bool FramePair::set_a_id(int id) {
  const int i = index_of(id);
  return i >= 0 && (set(a_, i, size()) || a_ == i);
}

bool FramePair::set_b_id(int id) {
  const int i = index_of(id);
  return i >= 0 && (set(b_, i, size()) || b_ == i);
}

int FramePair::index_of(int id) const {
  const auto it = std::lower_bound(ids_.begin(), ids_.end(), id);
  if (it == ids_.end() || *it != id)
    return -1;
  return static_cast<int>(it - ids_.begin());
}

int FramePair::nearest_index(int id) const {
  if (ids_.empty())
    return -1;
  const auto it = std::lower_bound(ids_.begin(), ids_.end(), id);
  if (it == ids_.begin())
    return 0;
  if (it == ids_.end())
    return size() - 1;
  const auto prev = std::prev(it);
  // Ties go to the lower id.
  return static_cast<int>((id - *prev <= *it - id ? prev : it) - ids_.begin());
}

BrowserKey browser_key(ArrowKey key, bool shift, bool text_focus) {
  if (text_focus)
    return BrowserKey::none;
  switch (key) {
  case ArrowKey::left:
    return shift ? BrowserKey::b_prev : BrowserKey::a_prev;
  case ArrowKey::right:
    return shift ? BrowserKey::b_next : BrowserKey::a_next;
  case ArrowKey::other:
    break;
  }
  return BrowserKey::none;
}

int browser_step(bool ctrl) { return ctrl ? 10 : 1; }

// ------------------------------------------------------- pending edits --

bool is_edge_type(std::string_view type) {
  return type == "odometry" || type == "loop_closure" || type == "panorama";
}

std::string edge_type_da(std::string_view type) {
  if (type == "odometry")
    return "Odometri";
  if (type == "loop_closure")
    return "Løkkelukning";
  if (type == "panorama")
    return "Panorama";
  return std::string(type);
}

double weight_from_icp_fitness(double rms_m) {
  constexpr double kFloorM = 0.01;
  const double sigma =
      std::isfinite(rms_m) ? std::max(rms_m, kFloorM) : kFloorM;
  return 1.0 / (sigma * sigma);
}

namespace {
bool same_number(double a, double b) {
  return a == b || (std::isnan(a) && std::isnan(b));
}

bool same_edge(const EdgeKey &a, const EdgeKey &b) {
  return same_edge_key(a, b);
}
} // namespace

EdgeKey canonical_key(EdgeKey key) {
  if (key.from > key.to)
    std::swap(key.from, key.to);
  return key;
}

bool same_edge_key(const EdgeKey &a, const EdgeKey &b) {
  return a.type == b.type && std::min(a.from, a.to) == std::min(b.from, b.to) &&
         std::max(a.from, a.to) == std::max(b.from, b.to);
}

bool same_record(const EdgeRecord &a, const EdgeRecord &b) {
  return a.key == b.key && same_number(a.residual, b.residual) &&
         same_number(a.weight, b.weight) && a.transform == b.transform &&
         a.information == b.information && a.source == b.source &&
         a.status == b.status &&
         a.seed_disagreement_m == b.seed_disagreement_m &&
         a.seed_disagreement_deg == b.seed_disagreement_deg &&
         a.deviation_m == b.deviation_m && a.deviation_deg == b.deviation_deg;
}

void PendingEdgeEdits::set_base(std::vector<EdgeRecord> edges) {
  base_ = std::move(edges);
  pending_.clear();
}

const EdgeRecord *PendingEdgeEdits::stored(const EdgeKey &k) const {
  for (const auto &e : base_)
    if (same_edge(e.key, k))
      return &e;
  return nullptr;
}

int PendingEdgeEdits::pending_index(const EdgeKey &k) const {
  for (std::size_t i = 0; i < pending_.size(); ++i)
    if (same_edge(pending_[i].edge.key, k))
      return static_cast<int>(i);
  return -1;
}

PendingEdgeEdits::Result PendingEdgeEdits::add(const EdgeRecord &staged) {
  if (staged.key.from == staged.key.to || !is_edge_type(staged.key.type) ||
      staged.weight <= 0.0)
    return Result::invalid;
  // The store's direction from here on: list, graph, strip and the saved
  // base compare the key the file will hold.
  const EdgeRecord edge = canonical_edge(staged);
  Op op;
  op.kind = Op::Kind::upsert;
  op.edge = edge;
  op.edge.status = "confirmed";
  op.edge.residual = std::numeric_limits<double>::quiet_NaN();
  op.edge.seed_disagreement_m.reset();
  op.edge.seed_disagreement_deg.reset();
  op.edge.deviation_m.reset();
  op.edge.deviation_deg.reset();
  const EdgeRecord *s = stored(edge.key);
  op.overwrite_rejected = s && s->status == "rejected";
  const int i = pending_index(edge.key);
  // Without a measurement the store only changes a stored row's status
  // (ProjectDB::upsert_pose_graph_edge): that is a confirmation.
  if (s && s->status == "confirmed" && !edge.transform) {
    if (i < 0)
      return Result::unchanged;
    pending_.erase(pending_.begin() + i); // back to the file state
    return Result::unstaged;
  }
  const Result r = !s                      ? Result::added
                   : op.overwrite_rejected ? Result::revived
                   : !edge.transform       ? Result::confirmed
                                           : Result::replaced;
  if (i >= 0) {
    const bool was_upsert =
        pending_[static_cast<std::size_t>(i)].kind == Op::Kind::upsert;
    pending_[static_cast<std::size_t>(i)] = op;
    if (!s || (was_upsert && r == Result::revived))
      return Result::replaced;
    return r;
  }
  pending_.push_back(op);
  return r;
}

PendingEdgeEdits::Result
PendingEdgeEdits::stage_status(const EdgeKey &k, const std::string &status,
                               Result on_change) {
  const EdgeRecord *s = stored(k);
  const int i = pending_index(k);
  if (i >= 0)
    pending_.erase(pending_.begin() + i);
  if (s->status == status)
    return i >= 0 ? Result::unstaged : Result::unchanged;
  Op op;
  op.kind = Op::Kind::set_status;
  op.edge.key = canonical_key(k);
  op.edge.status = status;
  op.from_status = s->status;
  pending_.push_back(op);
  return on_change;
}

PendingEdgeEdits::Result PendingEdgeEdits::confirm(const EdgeKey &k) {
  if (const int i = pending_index(k);
      i >= 0 && pending_[static_cast<std::size_t>(i)].kind == Op::Kind::upsert)
    return Result::unchanged; // a staged measurement is confirmed already
  if (!stored(k))
    return Result::not_found;
  return stage_status(k, "confirmed", Result::confirmed);
}

PendingEdgeEdits::Result PendingEdgeEdits::reject(const EdgeKey &k) {
  const EdgeRecord *s = stored(k);
  const int i = pending_index(k);
  if (!s) {
    if (i < 0)
      return Result::not_found;
    pending_.erase(pending_.begin() + i); // a staged new edge: just drop it
    return Result::unstaged;
  }
  return stage_status(k, "rejected", Result::rejected);
}

PendingEdgeEdits::Result PendingEdgeEdits::unreject(const EdgeKey &k) {
  const EdgeRecord *s = stored(k);
  if (!s)
    return Result::not_found;
  const int i = pending_index(k);
  if (s->status != "rejected") {
    if (i >= 0 &&
        pending_[static_cast<std::size_t>(i)].kind == Op::Kind::set_status &&
        pending_[static_cast<std::size_t>(i)].edge.status == "rejected") {
      pending_.erase(pending_.begin() + i);
      return Result::unstaged;
    }
    return Result::unchanged;
  }
  return stage_status(k, "auto", Result::unrejected);
}

PendingEdgeEdits::Result PendingEdgeEdits::unconfirm(const EdgeKey &k) {
  const EdgeRecord *s = stored(k);
  const int i = pending_index(k);
  if (!s)
    return i >= 0 ? Result::unchanged : Result::not_found;
  if (i >= 0) {
    const Op &p = pending_[static_cast<std::size_t>(i)];
    // A staged measurement is confirmed by definition: revert it instead.
    if (p.kind == Op::Kind::upsert && p.edge.transform)
      return Result::unchanged;
    if (p.edge.status == "confirmed") { // a staged confirmation: dropped
      pending_.erase(pending_.begin() + i);
      return Result::unstaged;
    }
    if (p.edge.status == "auto")
      return Result::unchanged; // un-confirmed or un-rejected already
  }
  if (s->status != "confirmed")
    return Result::unchanged;
  return stage_status(k, "auto", Result::unconfirmed);
}

PendingEdgeEdits::Result PendingEdgeEdits::revert(const EdgeKey &k) {
  const int i = pending_index(k);
  if (i < 0)
    return Result::unchanged;
  pending_.erase(pending_.begin() + i);
  return Result::unstaged;
}

std::vector<PendingEdgeEdits::Op> PendingEdgeEdits::ops() const {
  std::vector<Op> out;
  out.reserve(pending_.size());
  for (const auto &o : pending_)
    if (o.kind == Op::Kind::set_status)
      out.push_back(o);
  for (const auto &o : pending_)
    if (o.kind == Op::Kind::upsert)
      out.push_back(o);
  return out;
}

std::vector<EdgeView> PendingEdgeEdits::effective() const {
  std::vector<EdgeView> out;
  out.reserve(base_.size() + pending_.size());
  for (const auto &e : base_) {
    EdgeView v;
    v.edge = e;
    v.stored_status = e.status;
    if (const int i = pending_index(e.key); i >= 0) {
      const Op &o = pending_[static_cast<std::size_t>(i)];
      if (o.kind == Op::Kind::set_status) {
        v.edge.status = o.edge.status;
        v.pending = EdgeView::Pending::status;
      } else if (!o.edge.transform) {
        v.edge.status = o.edge.status; // the store keeps the measurement
        v.pending = EdgeView::Pending::status;
      } else {
        v.edge = o.edge;
        v.pending = EdgeView::Pending::replaced;
      }
    }
    out.push_back(std::move(v));
  }
  for (const auto &o : pending_)
    if (o.kind == Op::Kind::upsert && !stored(o.edge.key)) {
      EdgeView v;
      v.edge = o.edge;
      v.pending = EdgeView::Pending::added;
      out.push_back(std::move(v));
    }
  return out;
}

std::vector<EdgeView> PendingEdgeEdits::between(int a, int b) const {
  std::vector<EdgeView> out;
  for (auto &v : effective()) {
    const auto &k = v.edge.key;
    if ((k.from == a && k.to == b) || (k.from == b && k.to == a)) {
      v.reversed = k.from == b && a != b;
      out.push_back(std::move(v));
    }
  }
  return out;
}

std::map<int, int> PendingEdgeEdits::degrees() const {
  std::set<std::tuple<int, int, std::string>> seen;
  std::map<int, int> out;
  for (const auto &v : effective()) {
    if (v.edge.status == "rejected")
      continue;
    const auto &k = v.edge.key;
    if (!seen.insert({std::min(k.from, k.to), std::max(k.from, k.to), k.type})
             .second)
      continue;
    ++out[k.from];
    ++out[k.to];
  }
  return out;
}

int PendingEdgeEdits::degree(int frame) const {
  const auto all = degrees();
  const auto it = all.find(frame);
  return it == all.end() ? 0 : it->second;
}

void PendingEdgeEdits::commit_saved(const std::vector<Op> &saved) {
  for (const Op &o : saved) {
    EdgeRecord *s = nullptr;
    for (auto &e : base_)
      if (same_edge(e.key, o.edge.key))
        s = &e;
    // Mirror what the store wrote (ProjectDB::upsert_pose_graph_edge,
    // edge_row_from_record): status only without a measurement, and
    // "manual" for a record without a source.
    EdgeRecord written = o.edge;
    if (written.source.empty())
      written.source = "manual";
    if (o.kind == Op::Kind::set_status || (s && !o.edge.transform)) {
      if (s)
        s->status = o.edge.status;
    } else if (s) {
      *s = written; // staged canonical: what the store wrote
    } else {
      base_.push_back(written);
    }
    if (const int i = pending_index(o.edge.key); i >= 0) {
      const Op &p = pending_[static_cast<std::size_t>(i)];
      if (p.kind == o.kind && p.overwrite_rejected == o.overwrite_rejected &&
          same_record(p.edge, o.edge))
        pending_.erase(pending_.begin() + i);
    }
  }
  // Whatever is still pending is re-staged against the new base.
  std::vector<Op> rest;
  rest.swap(pending_);
  for (Op &o : rest) {
    const EdgeRecord *s = stored(o.edge.key);
    if (o.kind == Op::Kind::set_status) {
      if (s && s->status != o.edge.status)
        pending_.push_back(o);
    } else {
      o.overwrite_rejected = s && s->status == "rejected";
      pending_.push_back(o);
    }
  }
}

std::string pending_summary_da(const std::vector<PendingEdgeEdits::Op> &ops) {
  int measured = 0, rejected = 0, confirmed = 0, unrejected = 0,
      unconfirmed = 0;
  for (const auto &o : ops) {
    if (o.kind == PendingEdgeEdits::Op::Kind::upsert)
      ++measured;
    else if (o.edge.status == "rejected")
      ++rejected;
    else if (o.edge.status == "confirmed")
      ++confirmed;
    else if (o.from_status == "confirmed")
      ++unconfirmed;
    else
      ++unrejected;
  }
  std::vector<std::string> parts;
  if (measured)
    parts.push_back(measured == 1
                        ? "1 ny eller målt kant"
                        : std::to_string(measured) + " nye eller målte kanter");
  if (rejected)
    parts.push_back(std::to_string(rejected) + " afvist");
  if (confirmed)
    parts.push_back(std::to_string(confirmed) + " bekræftet");
  if (unrejected)
    parts.push_back(std::to_string(unrejected) + " genoplivet");
  if (unconfirmed)
    parts.push_back(std::to_string(unconfirmed) + " sat til auto");
  std::string out;
  for (std::size_t i = 0; i < parts.size(); ++i)
    out += (i ? ", " : "") + parts[i];
  return out;
}

bool pair_add_replaces(const std::vector<EdgeView> &views,
                       std::string_view type) {
  for (const EdgeView &v : views)
    if (v.edge.key.type == type && v.edge.status != "rejected")
      return true;
  return false;
}

// --------------------------------------------------------------- paging --

std::int64_t Paging::next_count() const {
  return std::max<std::int64_t>(0, std::min(page, total - loaded));
}

std::int64_t Paging::count_to_reach(std::int64_t row) const {
  if (row < loaded || row < 0 || page <= 0)
    return 0;
  const std::int64_t target = std::min(total, row + 1);
  const std::int64_t need = target - loaded;
  const std::int64_t pages = (need + page - 1) / page;
  return std::min(pages * page, total - loaded);
}

// ----------------------------------------------------------- formatting --

ImageSize fit_within(int width, int height, int max_side) {
  if (width <= 0 || height <= 0 || max_side <= 0)
    return {std::max(width, 0), std::max(height, 0)};
  const int longer = std::max(width, height);
  if (longer <= max_side)
    return {width, height};
  const double s = static_cast<double>(max_side) / longer;
  return {std::max(1, static_cast<int>(std::lround(width * s))),
          std::max(1, static_cast<int>(std::lround(height * s)))};
}

std::string format_decimal_da(double v, int decimals) {
  char buf[64];
  std::snprintf(buf, sizeof buf, "%.*f", std::max(0, decimals), v);
  std::string s(buf);
  if (s.size() > 1 && s[0] == '-' &&
      s.find_first_not_of("-0.") == std::string::npos)
    s.erase(0, 1); // no "-0,00"
  for (char &c : s)
    if (c == '.')
      c = ',';
  return s;
}

std::string row_key_title(
    const std::vector<std::pair<std::string, std::string>> &key_columns) {
  std::string out;
  for (const auto &[name, value] : key_columns) {
    if (!out.empty())
      out += " · ";
    out += name;
    out += ' ';
    out += value;
  }
  return out;
}

std::string format_bytes_da(std::uint64_t bytes) {
  if (bytes < 1000)
    return std::to_string(bytes) + " B";
  static const char *units[] = {"kB", "MB", "GB", "TB"};
  double v = static_cast<double>(bytes);
  int u = -1;
  while (v >= 999.95 && u < 3) {
    v /= 1000.0;
    ++u;
  }
  // GB and up keep two decimals: a project's size is read in hundreds of MB.
  return format_decimal_da(v, u >= 2 ? 2 : 1) + " " + units[u];
}

std::string sniff_blob(std::string_view h) {
  auto starts = [&](std::string_view p) {
    return h.size() >= p.size() && h.substr(0, p.size()) == p;
  };
  if (starts(std::string_view("\x89PNG\r\n\x1a\n", 8)))
    return "PNG";
  if (starts("\xff\xd8\xff"))
    return "JPEG";
  if (starts("ply\n") || starts("ply\r\n"))
    return "PLY";
  if (starts("%PDF"))
    return "PDF";
  if (starts("RIFF") && h.size() >= 12 && h.substr(8, 4) == "WEBP")
    return "WebP";
  if (starts("\x1f\x8b"))
    return "GZIP";
  if (starts("PK\x03\x04"))
    return "ZIP";
  // JSON only when the whole head is text: binary data (float chunks)
  // starts with '{' or '[' bytes often enough.
  const bool text = std::all_of(h.begin(), h.end(), [](char ch) {
    const auto u = static_cast<unsigned char>(ch);
    return u >= 0x20 || u == '\t' || u == '\r' || u == '\n';
  });
  const auto first = h.find_first_not_of(" \t\r\n");
  if (text && first != std::string_view::npos &&
      (h[first] == '{' || h[first] == '['))
    return "JSON";
  return {};
}

std::string blob_column_meaning(std::string_view table,
                                std::string_view column) {
  struct Known {
    std::string_view table, column, meaning;
  };
  static const Known known[] = {
      {"sensor_frames", "color", "Farvebillede"},
      {"sensor_frames", "depth", "Dybde"},
      {"sensor_frames", "confidence", "Konfidens"},
      {"sensor_frames", "transform", "Pose 4×4"},
      {"segmentation_images", "label_image", "Mærkatbillede"},
      {"glass_confidence_images", "confidence", "Glas-konfidens"},
      {"panoramic_images", "image_data", "Panorama"},
      {"panoramic_images", "pose", "Pose 4×4"},
      {"panorama_segmentation", "label_image", "Mærkatbillede"},
      {"point_cloud_data", "data", "Punktdata"},
      {"point_clouds", "tile_index", "Fliseindeks"},
      {"meshes", "data", "Mesh"},
      {"mesh_texture_data", "data", "Tekstur"},
      {"gaussian_splat_data", "data", "Splat-data"},
      {"material_thumbnails", "data", "Miniature"},
      {"report_pdfs", "pdf_blob", "Rapport"},
      {"building_components", "geometry", "Geometri"},
  };
  for (const auto &k : known)
    if (k.table == table && k.column == column)
      return std::string(k.meaning);
  return {};
}

std::string describe_blob(std::string_view table, std::string_view column,
                          std::string_view head, std::uint64_t size) {
  std::string out = blob_column_meaning(table, column);
  const std::string format = sniff_blob(head);
  auto add = [&](const std::string &part) {
    if (part.empty())
      return;
    if (!out.empty())
      out += " · ";
    out += part;
  };
  if (out.empty() && column == "transform" && size == 128)
    out = "Pose 4×4";
  add(format);
  add(format_bytes_da(size));
  return out;
}

PoseSummary summarize_pose(const std::array<double, 16> &m) {
  PoseSummary s;
  s.t = {m[3], m[7], m[11]};
  // R = Rz(yaw) * Ry(pitch) * Rx(roll), the ZYX convention.
  const double r00 = m[0], r10 = m[4], r20 = m[8], r21 = m[9], r22 = m[10];
  constexpr double kDeg = 180.0 / 3.14159265358979323846;
  s.pitch_deg = std::asin(std::clamp(-r20, -1.0, 1.0)) * kDeg;
  s.yaw_deg = std::atan2(r10, r00) * kDeg;
  s.roll_deg = std::atan2(r21, r22) * kDeg;
  const double trace = m[0] + m[5] + m[10];
  s.angle_deg = std::acos(std::clamp((trace - 1.0) / 2.0, -1.0, 1.0)) * kDeg;
  return s;
}

PoseDelta pose_delta(const std::array<double, 16> &a,
                     const std::array<double, 16> &b) {
  PoseDelta d;
  const double dx = b[3] - a[3], dy = b[7] - a[7], dz = b[11] - a[11];
  d.distance_m = std::sqrt(dx * dx + dy * dy + dz * dz);
  // trace(Ra^T Rb) = sum_ij Ra_ij Rb_ij over the 3x3 blocks.
  double trace = 0;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      trace += a[i * 4 + j] * b[i * 4 + j];
  constexpr double kDeg = 180.0 / 3.14159265358979323846;
  d.angle_deg = std::acos(std::clamp((trace - 1.0) / 2.0, -1.0, 1.0)) * kDeg;
  return d;
}

// --------------------------------------------------------- write errors --

WriteErrorKind classify_write_error(std::string_view what) {
  auto has = [&](std::string_view s) {
    return what.find(s) != std::string_view::npos;
  };
  if (has("locked") || has("busy"))
    return WriteErrorKind::locked;
  if (has("readonly") || has("read-only") || has("read only"))
    return WriteErrorKind::read_only;
  return WriteErrorKind::other;
}

std::string write_error_da(WriteErrorKind kind) {
  switch (kind) {
  case WriteErrorKind::read_only:
    return "Projektet er åbnet skrivebeskyttet, så ændringerne kan ikke "
           "gemmes. Åbn det med skriveadgang for at gemme.";
  case WriteErrorKind::locked:
    return "Projektet er låst af en anden proces, der skriver til det. "
           "Ændringerne er ikke gemt — prøv igen, når den er færdig.";
  case WriteErrorKind::other:
    break;
  }
  return "Ændringerne kunne ikke gemmes. De venter stadig; se detaljerne.";
}

} // namespace rux::qt
