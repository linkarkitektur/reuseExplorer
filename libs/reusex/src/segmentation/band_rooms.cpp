// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "reusex/segmentation/band_rooms.hpp"

#include "reusex/core/logging.hpp"
#include "reusex/geometry/level_frame.hpp"

#include <fmt/format.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <map>
#include <queue>
#include <stdexcept>
#include <tuple>
#include <vector>

namespace reusex::geometry {

void RoomGrid::resize(int w, int h) {
  width = w;
  height = h;
  const size_t n = static_cast<size_t>(w) * static_cast<size_t>(h);
  wall.assign(n, 0);
  footprint.assign(n, 0);
  graph.assign(n, 0);
  interior.assign(n, 0);
  clearance.assign(n, 0.0F);
  labels.assign(n, 0);
}

namespace {

/// Radius in cells of a metric length (at least 1).
int cells(double metres, double cell) {
  return std::max(1, static_cast<int>(std::lround(metres / cell)));
}

cv::Mat disk(int radius) {
  return cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                   cv::Size(2 * radius + 1, 2 * radius + 1));
}

/// A CV_8U view of a byte mask (0/1), no copy.
cv::Mat view(std::vector<uint8_t> &m, int w, int h) {
  return {h, w, CV_8U, m.data()};
}

/// 4-connected flood from every border cell through cells where
/// @p passable is set; returns the reached mask.
std::vector<uint8_t> flood_from_border(const std::vector<uint8_t> &passable,
                                       int w, int h) {
  std::vector<uint8_t> reached(passable.size(), 0);
  std::deque<int> queue;
  auto seed = [&](int c, int r) {
    const size_t i = static_cast<size_t>(r) * w + c;
    if (passable[i] && !reached[i]) {
      reached[i] = 1;
      queue.push_back(static_cast<int>(i));
    }
  };
  for (int c = 0; c < w; ++c) {
    seed(c, 0);
    seed(c, h - 1);
  }
  for (int r = 0; r < h; ++r) {
    seed(0, r);
    seed(w - 1, r);
  }
  while (!queue.empty()) {
    const int i = queue.front();
    queue.pop_front();
    const int c = i % w;
    const int r = i / w;
    if (c > 0)
      seed(c - 1, r);
    if (c + 1 < w)
      seed(c + 1, r);
    if (r > 0)
      seed(c, r - 1);
    if (r + 1 < h)
      seed(c, r + 1);
  }
  return reached;
}

/// Marker-controlled watershed of -clearance inside @p mask, 4-connected:
/// the cell with the largest clearance is flooded first, ties by insertion
/// order (skimage.segmentation.watershed semantics). Labels are written into
/// @p labels in place (markers already set, 0 = unlabelled).
void watershed_on_clearance(const std::vector<float> &clearance,
                            const std::vector<uint8_t> &mask, int w, int h,
                            std::vector<int32_t> &labels) {
  // (-(clearance), age, index): smallest first.
  using Item = std::tuple<float, uint64_t, int>;
  std::priority_queue<Item, std::vector<Item>, std::greater<>> heap;
  uint64_t age = 0;
  for (size_t i = 0; i < labels.size(); ++i)
    if (labels[i] > 0)
      heap.emplace(-clearance[i], age++, static_cast<int>(i));
  while (!heap.empty()) {
    const int i = std::get<2>(heap.top());
    heap.pop();
    const int c = i % w;
    const int r = i / w;
    const int32_t own = labels[static_cast<size_t>(i)];
    auto visit = [&](int cc, int rr) {
      const size_t j = static_cast<size_t>(rr) * w + cc;
      if (!mask[j] || labels[j] != 0)
        return;
      labels[j] = own;
      heap.emplace(-clearance[j], age++, static_cast<int>(j));
    };
    if (c > 0)
      visit(c - 1, r);
    if (c + 1 < w)
      visit(c + 1, r);
    if (r > 0)
      visit(c, r - 1);
    if (r + 1 < h)
      visit(c, r + 1);
  }
}

/// Every cell of @p allowed takes the label of its nearest labelled cell
/// (8-connected chamfer distance, ties by the lower cell index of the
/// source) — the rooms grown over their walls so wall points get a room.
std::vector<int32_t> grow_labels(const std::vector<int32_t> &labels,
                                 const std::vector<uint8_t> &allowed, int w,
                                 int h) {
  std::vector<int32_t> out(labels.size(), 0);
  std::vector<double> dist(labels.size(),
                           std::numeric_limits<double>::infinity());
  using Item = std::tuple<double, int, int>; // distance, source, cell
  std::priority_queue<Item, std::vector<Item>, std::greater<>> heap;
  for (size_t i = 0; i < labels.size(); ++i)
    if (labels[i] > 0) {
      dist[i] = 0.0;
      out[i] = labels[i];
      heap.emplace(0.0, static_cast<int>(i), static_cast<int>(i));
    }
  constexpr double kDiag = 1.4142135623730951;
  static constexpr int kDc[8] = {-1, 1, 0, 0, -1, -1, 1, 1};
  static constexpr int kDr[8] = {0, 0, -1, 1, -1, 1, -1, 1};
  while (!heap.empty()) {
    const auto [d, src, i] = heap.top();
    heap.pop();
    if (d > dist[static_cast<size_t>(i)])
      continue;
    const int c = i % w;
    const int r = i / w;
    for (int k = 0; k < 8; ++k) {
      const int cc = c + kDc[k];
      const int rr = r + kDr[k];
      if (cc < 0 || rr < 0 || cc >= w || rr >= h)
        continue;
      const size_t j = static_cast<size_t>(rr) * w + cc;
      if (!allowed[j])
        continue;
      const double nd = d + (k < 4 ? 1.0 : kDiag);
      if (nd < dist[j]) {
        dist[j] = nd;
        out[j] = out[static_cast<size_t>(i)];
        heap.emplace(nd, src, static_cast<int>(j));
      }
    }
  }
  return out;
}

/// Merge pockets into their neighbours, in place on @p labels (ids
/// 1..@p n): a region of at most @p max_cells cells whose 4-connected
/// boundary edges touch one other region on at least @p min_share of them
/// takes that region's id, if that region is larger (equal: the lower id).
/// Smallest regions first, one at a time, so a merged pocket's neighbour is
/// judged with it; deterministic (ties by id). Returns the regions merged.
int merge_pockets(std::vector<int32_t> &labels, int w, int h, int32_t n,
                  double max_cells, double min_share) {
  const auto N = static_cast<size_t>(n) + 1;
  int merged = 0;
  std::vector<uint8_t> done(N, 0);
  for (;;) {
    std::vector<size_t> area(N, 0);
    std::vector<size_t> edges(N, 0);
    std::map<std::pair<int32_t, int32_t>, size_t> contact;
    for (int r = 0; r < h; ++r)
      for (int c = 0; c < w; ++c) {
        const int32_t l = labels[static_cast<size_t>(r) * w + c];
        if (l <= 0)
          continue;
        ++area[static_cast<size_t>(l)];
        auto edge = [&](int cc, int rr) {
          const int32_t m = (cc < 0 || rr < 0 || cc >= w || rr >= h)
                                ? 0
                                : labels[static_cast<size_t>(rr) * w + cc];
          if (m == l)
            return;
          ++edges[static_cast<size_t>(l)];
          if (m > 0)
            ++contact[{l, m}];
        };
        edge(c - 1, r);
        edge(c + 1, r);
        edge(c, r - 1);
        edge(c, r + 1);
      }
    // The smallest unjudged region that is small enough.
    int32_t pocket = 0;
    for (int32_t l = 1; l <= n; ++l) {
      const auto k = static_cast<size_t>(l);
      if (done[k] || area[k] == 0 || static_cast<double>(area[k]) > max_cells)
        continue;
      if (pocket == 0 || area[k] < area[static_cast<size_t>(pocket)])
        pocket = l;
    }
    if (pocket == 0)
      break;
    done[static_cast<size_t>(pocket)] = 1;
    // Its neighbour with the most contact (ties: the lower id).
    int32_t into = 0;
    size_t best = 0;
    for (auto it = contact.lower_bound({pocket, 0});
         it != contact.end() && it->first.first == pocket; ++it)
      if (it->second > best) {
        best = it->second;
        into = it->first.second;
      }
    const auto pk = static_cast<size_t>(pocket);
    if (into == 0 || edges[pk] == 0 ||
        static_cast<double>(best) < min_share * static_cast<double>(edges[pk]))
      continue;
    const auto ik = static_cast<size_t>(into);
    if (area[ik] < area[pk] || (area[ik] == area[pk] && into > pocket))
      continue;
    debug("band rooms: region {} ({} cells) meets region {} on {} of its {} "
          "boundary edges: a pocket, merged",
          pocket, area[pk], into, best, edges[pk]);
    for (auto &l : labels)
      if (l == pocket)
        l = into;
    ++merged;
  }
  return merged;
}

} // namespace

int partition_room_grid(RoomGrid &g, const BandRoomsOptions &o,
                        int first_label) {
  const int w = g.width;
  const int h = g.height;
  const size_t n = static_cast<size_t>(w) * static_cast<size_t>(h);
  if (g.wall.size() != n || g.footprint.size() != n)
    throw std::invalid_argument(
        "partition_room_grid: wall/footprint size does not match the grid");
  g.interior.assign(n, 0);
  g.clearance.assign(n, 0.0F);
  g.labels.assign(n, 0);
  g.cores = g.rooms = g.dropped_regions = g.coreless = g.merged_pockets = 0;
  g.interior_area = 0.0;
  if (n == 0)
    return 0;
  const double cell = g.cell_size;
  const double cell_area = cell * cell;

  // Footprint: closed, then the exterior flooded from the border; whatever
  // the flood cannot reach is inside (holes under furniture, wall cells).
  std::vector<uint8_t> closed(n, 0);
  {
    cv::Mat src = view(g.footprint, w, h);
    cv::Mat dst = view(closed, w, h);
    cv::morphologyEx(src, dst, cv::MORPH_CLOSE,
                     disk(cells(o.footprint_close, cell)));
  }
  std::vector<uint8_t> outside(n);
  for (size_t i = 0; i < n; ++i)
    outside[i] = closed[i] ? 0 : 1;
  const auto exterior = flood_from_border(outside, w, h);
  size_t interior_cells = 0;
  for (size_t i = 0; i < n; ++i) {
    g.interior[i] = exterior[i] ? 0 : 1;
    interior_cells += g.interior[i];
  }
  g.interior_area = static_cast<double>(interior_cells) * cell_area;

  // Free space = interior minus the (dilated) walls; its distance transform.
  std::vector<uint8_t> wall_d(n, 0);
  {
    cv::Mat src = view(g.wall, w, h);
    cv::Mat dst = view(wall_d, w, h);
    cv::dilate(src, dst, disk(cells(o.wall_dilate, cell)));
  }
  // The wall graph and door cuts are blocked as they are: their width is
  // the wall's already.
  if (g.graph.size() == n)
    for (size_t i = 0; i < n; ++i)
      wall_d[i] = static_cast<uint8_t>(wall_d[i] | g.graph[i]);
  std::vector<uint8_t> free(n, 0);
  for (size_t i = 0; i < n; ++i)
    free[i] = (g.interior[i] && !wall_d[i]) ? 1 : 0;
  {
    cv::Mat src = view(free, w, h);
    cv::Mat dist;
    cv::distanceTransform(src, dist, cv::DIST_L2, cv::DIST_MASK_PRECISE,
                          CV_32F);
    for (int r = 0; r < h; ++r) {
      const float *row = dist.ptr<float>(r);
      for (int c = 0; c < w; ++c)
        g.clearance[g.index(c, r)] = row[c] * static_cast<float>(cell);
    }
  }

  // Cores: 4-connected components farther than core_radius from any wall
  // or the exterior, numbered in raster order of their first cell.
  std::vector<uint8_t> core(n, 0);
  for (size_t i = 0; i < n; ++i)
    core[i] = g.clearance[i] > static_cast<float>(o.core_radius) ? 1 : 0;
  cv::Mat core_lbl;
  const int ncomp = cv::connectedComponents(view(core, w, h), core_lbl, 4,
                                            CV_32S, cv::CCL_DEFAULT);
  std::vector<int> core_size(static_cast<size_t>(ncomp), 0);
  for (int r = 0; r < h; ++r) {
    const int *row = core_lbl.ptr<int>(r);
    for (int c = 0; c < w; ++c)
      ++core_size[static_cast<size_t>(row[c])];
  }
  // OpenCV's numbering is not documented as raster order across its
  // algorithms; renumber by first appearance so the ids are.
  std::vector<int32_t> remap(static_cast<size_t>(ncomp), -1);
  int32_t next = 0;
  const auto min_core_cells = o.min_core_area / cell_area;
  for (int r = 0; r < h; ++r) {
    const int *row = core_lbl.ptr<int>(r);
    for (int c = 0; c < w; ++c) {
      const int l = row[c];
      if (l == 0)
        continue;
      auto &m = remap[static_cast<size_t>(l)];
      if (m == -1)
        m = static_cast<double>(core_size[static_cast<size_t>(l)]) >=
                    min_core_cells
                ? ++next
                : 0;
      g.labels[g.index(c, r)] = m;
    }
  }
  g.cores = next;

  watershed_on_clearance(g.clearance, free, w, h, g.labels);

  // Free space no core reached: a component closed off by walls and door
  // cuts that is too narrow for a core (a WC stall). It is a room of its
  // own when it is wide enough somewhere; numbered after the cores, in
  // raster order of the component's first cell.
  if (o.coreless_min_clearance > 0.0) {
    std::vector<uint8_t> left(n, 0);
    for (size_t i = 0; i < n; ++i)
      left[i] = (free[i] && g.labels[i] == 0) ? 1 : 0;
    cv::Mat lbl;
    const int nl = cv::connectedComponents(view(left, w, h), lbl, 4, CV_32S,
                                           cv::CCL_DEFAULT);
    // Per component: its widest point, and what its boundary cells are --
    // outside the interior (open to the exterior: a footprint leak beyond a
    // window, cut off by the wall graph) or blocked (walls, door cuts).
    std::vector<float> widest(static_cast<size_t>(nl), 0.0F);
    std::vector<size_t> open_edge(static_cast<size_t>(nl), 0);
    std::vector<size_t> closed_edge(static_cast<size_t>(nl), 0);
    for (int r = 0; r < h; ++r) {
      const int *row = lbl.ptr<int>(r);
      for (int c = 0; c < w; ++c) {
        const int l = row[c];
        if (l <= 0)
          continue;
        const auto k = static_cast<size_t>(l);
        widest[k] = std::max(widest[k], g.clearance[g.index(c, r)]);
        auto edge = [&](int cc, int rr) {
          if (cc < 0 || rr < 0 || cc >= w || rr >= h) {
            ++open_edge[k];
            return;
          }
          if (lbl.at<int>(rr, cc) == l)
            return;
          ++(g.interior[g.index(cc, rr)] ? closed_edge[k] : open_edge[k]);
        };
        edge(c - 1, r);
        edge(c + 1, r);
        edge(c, r - 1);
        edge(c, r + 1);
      }
    }
    auto enclosed = [&](size_t k) {
      const size_t total = open_edge[k] + closed_edge[k];
      return total > 0 &&
             static_cast<double>(open_edge[k]) <=
                 o.coreless_max_open_share * static_cast<double>(total);
    };
    std::vector<int32_t> id_of(static_cast<size_t>(nl), -1);
    for (int r = 0; r < h; ++r) {
      const int *row = lbl.ptr<int>(r);
      for (int c = 0; c < w; ++c) {
        const int l = row[c];
        if (l == 0)
          continue;
        auto &m = id_of[static_cast<size_t>(l)];
        if (m == -1) {
          m = widest[static_cast<size_t>(l)] >=
                          static_cast<float>(o.coreless_min_clearance) &&
                      enclosed(static_cast<size_t>(l))
                  ? ++next
                  : 0;
          if (m > 0)
            ++g.coreless;
        }
        g.labels[g.index(c, r)] = m;
      }
    }
  }

  // Pockets (reusex-3r1): a small region that meets a neighbour through
  // free space along much of its boundary is part of that neighbour -- a
  // second core the watershed seeded behind furniture or a column, not a
  // room behind a door.
  if (o.pocket_min_contact_share > 0.0 && next > 1)
    g.merged_pockets =
        merge_pockets(g.labels, w, h, next, o.pocket_max_area / cell_area,
                      o.pocket_min_contact_share);

  // Regions below min_room_area are dropped; the rest renumbered in order.
  std::vector<size_t> area(static_cast<size_t>(next) + 1, 0);
  for (int32_t l : g.labels)
    if (l > 0)
      ++area[static_cast<size_t>(l)];
  std::vector<int32_t> final_id(static_cast<size_t>(next) + 1, 0);
  int32_t id = first_label;
  for (int32_t l = 1; l <= next; ++l) {
    if (area[static_cast<size_t>(l)] == 0)
      continue; // merged into another region as a pocket
    if (static_cast<double>(area[static_cast<size_t>(l)]) * cell_area >=
        o.min_room_area)
      final_id[static_cast<size_t>(l)] = id++;
    else {
      ++g.dropped_regions;
      if (l > g.cores)
        --g.coreless;
    }
  }
  for (auto &l : g.labels)
    l = l > 0 ? final_id[static_cast<size_t>(l)] : 0;
  g.rooms = id - first_label;
  return g.rooms;
}

namespace {

/// A door or a floor-reaching opening: the rooms either side meet there.
bool is_doorway(const core::OpeningRecord &o) {
  return o.kind == "door" || o.kind == "opening";
}

/// Set the cells of @p g.graph whose centre lies within @p half of the
/// stretch [s0, s1] of the axis a -> a + len * u.
size_t stamp_segment(RoomGrid &g, const Eigen::Vector2d &a,
                     const Eigen::Vector2d &u, double s0, double s1,
                     double half) {
  if (!(s1 > s0))
    return 0;
  const double cell = g.cell_size;
  const Eigen::Vector2d n(-u.y(), u.x());
  const Eigen::Vector2d p0 = a + s0 * u;
  const Eigen::Vector2d p1 = a + s1 * u;
  const double pad = half + cell;
  const auto lo = [&](double v, int origin) {
    return static_cast<int>(std::floor(v / cell)) - origin;
  };
  const int c0 = std::max(0, lo(std::min(p0.x(), p1.x()) - pad, g.col0));
  const int c1 =
      std::min(g.width - 1, lo(std::max(p0.x(), p1.x()) + pad, g.col0));
  const int r0 = std::max(0, lo(std::min(p0.y(), p1.y()) - pad, g.row0));
  const int r1 =
      std::min(g.height - 1, lo(std::max(p0.y(), p1.y()) + pad, g.row0));
  size_t set = 0;
  for (int r = r0; r <= r1; ++r)
    for (int c = c0; c <= c1; ++c) {
      const Eigen::Vector2d q((g.col0 + c + 0.5) * cell,
                              (g.row0 + r + 0.5) * cell);
      const Eigen::Vector2d d = q - a;
      const double s = d.dot(u);
      if (s < s0 || s > s1 || std::abs(d.dot(n)) > half)
        continue;
      auto &v = g.graph[g.index(c, r)];
      if (v == 0) {
        v = 1;
        ++set;
      }
    }
  return set;
}

} // namespace

size_t rasterise_wall_graph(RoomGrid &g, const BandRoomsPlan &plan,
                            const BandRoomsOptions &o) {
  const size_t n = static_cast<size_t>(g.width) * static_cast<size_t>(g.height);
  if (g.graph.size() != n)
    g.graph.assign(n, 0);
  g.graph_walls = 0;
  g.door_cuts = 0;
  if (n == 0 || !(o.use_wall_graph || o.cut_at_doors))
    return 0;
  const double min_half = std::max(o.wall_graph_half_width, 0.75 * g.cell_size);
  size_t set = 0;
  for (const auto &w : plan.walls) {
    if (w.storey != g.storey_idx)
      continue;
    if (w.seen_share >= 0.0 && w.seen_share < o.wall_graph_min_seen_share)
      continue;
    const Eigen::Vector2d a(w.x0, w.y0);
    const Eigen::Vector2d d(w.x1 - w.x0, w.y1 - w.y0);
    const double len = d.norm();
    if (!(len > 1e-6) || !std::isfinite(len))
      continue;
    const Eigen::Vector2d u = d / len;
    const double half =
        std::max(min_half, w.thickness > 0.0 ? 0.5 * w.thickness : 0.0);
    // The doorways of this wall, as [s0, s1] clamped to the axis, sorted.
    std::vector<std::pair<double, double>> doors;
    for (const auto &op : plan.openings)
      if (op.wall == w.guid && is_doorway(op)) {
        const double s0 = std::max(0.0, std::min(op.s_start, op.s_end));
        const double s1 = std::min(len, std::max(op.s_start, op.s_end));
        if (s1 > s0)
          doors.emplace_back(s0, s1);
      }
    std::sort(doors.begin(), doors.end());
    if (o.use_wall_graph) {
      ++g.graph_walls;
      // The axis, capped by its half-width at both ends so joins close;
      // with cut_at_doors the doorways are part of it, else skipped.
      double from = -half;
      if (!o.cut_at_doors)
        for (const auto &[s0, s1] : doors) {
          set += stamp_segment(g, a, u, from, s0, half);
          from = std::max(from, s1);
        }
      set += stamp_segment(g, a, u, from, len + half, half);
    }
    if (o.cut_at_doors)
      for (const auto &[s0, s1] : doors) {
        ++g.door_cuts;
        set += stamp_segment(g, a, u, s0 - o.door_cut_overlap,
                             s1 + o.door_cut_overlap, half);
      }
  }
  return set;
}

auto segment_band_rooms(const Cloud &cloud, const CloudN &normals,
                        const std::vector<core::StoreyRecord> &storeys,
                        const BandRoomsOptions &o, const pcl::Indices *evidence,
                        const BandRoomsPlan *plan) -> BandRoomsResult {
  if (storeys.empty())
    throw std::invalid_argument(
        "band rooms need the storeys table (level frame, floor and ceiling "
        "heights); run `rux create storeys` first");
  if (normals.size() != cloud.size())
    throw std::invalid_argument(
        fmt::format("band rooms: 'normals' has {} points but 'cloud' has {}",
                    normals.size(), cloud.size()));
  if (!(o.cell_size > 0.0))
    throw std::invalid_argument("band rooms: cell_size must be positive");

  BandRoomsResult result;
  result.labels.reset(new CloudL);
  result.labels->resize(cloud.size());
  result.labels->width = static_cast<uint32_t>(cloud.size());
  result.labels->height = 1;
  for (auto &l : result.labels->points)
    l.label = 0;

  std::vector<uint8_t> use(cloud.size(), evidence ? 0 : 1);
  if (evidence)
    for (const auto i : *evidence)
      if (i >= 0 && static_cast<size_t>(i) < use.size())
        use[static_cast<size_t>(i)] = 1;

  std::vector<core::StoreyRecord> order = storeys;
  std::stable_sort(order.begin(), order.end(),
                   [](const auto &a, const auto &b) { return a.idx < b.idx; });

  const double cell = o.cell_size;
  int32_t next_label = 1;
  std::vector<uint8_t> assigned(cloud.size(), 0);

  for (const auto &st : order) {
    if (o.cancel_token && o.cancel_token->load())
      break;
    const LevelFrame frame = LevelFrame::from_record(st);
    const Eigen::Matrix3d rot = frame.world_to_level.linear();
    const double zf = st.z_floor;
    const double zc = st.z_ceiling;
    const double z_lo = zf - o.storey_margin;
    const double z_hi = zc + o.storey_margin;
    const bool has_ceiling = zc > zf + o.band_bottom + 0.1;
    const double band_lo = zf + o.band_bottom;
    const double band_hi =
        has_ceiling ? std::min(zf + o.band_top, zc - o.ceiling_margin)
                    : zf + o.band_top;
    if (!has_ceiling)
      warn("band rooms: storey {} has no ceiling above its floor (floor "
           "{:.2f} m, ceiling {:.2f} m); the wall band runs to floor + {:.2f} "
           "m unclamped",
           st.idx, zf, zc, o.band_top);

    // Level coordinates of the storey's points.
    std::vector<size_t> members;
    std::vector<Eigen::Vector3d> lp;
    std::vector<double> nz;
    // Points above the top storey's ceiling (attic, roof, ridge) take the
    // room under them instead of staying unlabeled, so the mesh stage keeps
    // their volume; they give no evidence and do not size the grid.
    const bool top = &st == &order.back();
    std::vector<uint8_t> above;
    for (size_t i = 0; i < cloud.size(); ++i) {
      const auto &p = cloud.points[i];
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
        continue;
      const Eigen::Vector3d q = frame.to_level(Eigen::Vector3d(p.x, p.y, p.z));
      if (q.z() <= z_lo || (q.z() >= z_hi && !top))
        continue;
      members.push_back(i);
      above.push_back(q.z() >= z_hi ? 1 : 0);
      lp.push_back(q);
      const auto &nn = normals.points[i];
      const Eigen::Vector3d nl =
          rot * Eigen::Vector3d(nn.normal_x, nn.normal_y, nn.normal_z);
      nz.push_back(std::isfinite(nl.z()) ? std::abs(nl.z()) : 0.0);
    }

    RoomGrid g;
    g.storey_idx = st.idx;
    g.cell_size = cell;
    if (std::count(above.begin(), above.end(), uint8_t{0}) == 0) {
      warn("band rooms: storey {} has no point between {:.2f} and {:.2f} m "
           "in its level frame; no rooms",
           st.idx, z_lo, z_hi);
      result.grids.push_back(std::move(g));
      continue;
    }
    // Grid anchored to the level frame origin: integer cell indices.
    auto col_of = [&](double x) {
      return static_cast<int>(std::floor(x / cell));
    };
    int cmin = std::numeric_limits<int>::max();
    int cmax = std::numeric_limits<int>::min();
    int rmin = cmin;
    int rmax = cmax;
    for (size_t k = 0; k < lp.size(); ++k) {
      if (above[k])
        continue;
      const auto &q = lp[k];
      cmin = std::min(cmin, col_of(q.x()));
      cmax = std::max(cmax, col_of(q.x()));
      rmin = std::min(rmin, col_of(q.y()));
      rmax = std::max(rmax, col_of(q.y()));
    }
    const int pad = cells(o.grid_padding, cell);
    g.col0 = cmin - pad;
    g.row0 = rmin - pad;
    const long long w = static_cast<long long>(cmax - cmin) + 1 + 2LL * pad;
    const long long h = static_cast<long long>(rmax - rmin) + 1 + 2LL * pad;
    constexpr long long kMaxCells = 400LL * 1000 * 1000;
    if (w * h > kMaxCells)
      throw std::runtime_error(fmt::format(
          "band rooms: storey {} spans {} x {} cells of {} m (> {} cells); "
          "the cloud has far outliers or cell_size is too small",
          st.idx, w, h, cell, kMaxCells));
    g.resize(static_cast<int>(w), static_cast<int>(h));

    // Evidence rasterisation.
    std::vector<int> wall_count(g.wall.size(), 0);
    size_t n_wall_pts = 0;
    size_t n_fp_pts = 0;
    for (size_t k = 0; k < members.size(); ++k) {
      if (!use[members[k]] || above[k])
        continue;
      const auto &q = lp[k];
      const size_t ci = g.index(col_of(q.x()) - g.col0, col_of(q.y()) - g.row0);
      if (q.z() > band_lo && q.z() < band_hi && nz[k] < o.wall_max_abs_nz) {
        ++wall_count[ci];
        ++n_wall_pts;
      }
      if (nz[k] > o.horizontal_min_abs_nz &&
          (q.z() < zf + o.floor_max_height ||
           q.z() > zf + o.ceiling_min_height)) {
        g.footprint[ci] = 1;
        ++n_fp_pts;
      }
    }
    size_t wall_cells = 0;
    size_t fp_cells = 0;
    for (size_t i = 0; i < g.wall.size(); ++i) {
      g.wall[i] = wall_count[i] >= o.wall_min_points ? 1 : 0;
      wall_cells += g.wall[i];
      fp_cells += g.footprint[i];
    }

    const size_t graph_cells = plan ? rasterise_wall_graph(g, *plan, o) : 0;

    const int rooms = partition_room_grid(g, o, next_label);

    // Points take the room of the nearest room cell within the interior
    // grown by assign_margin.
    std::vector<uint8_t> reach(g.interior.size(), 0);
    {
      cv::Mat src = view(g.interior, g.width, g.height);
      cv::Mat dst = view(reach, g.width, g.height);
      cv::dilate(src, dst, disk(cells(o.assign_margin, cell)));
    }
    const auto grown = grow_labels(g.labels, reach, g.width, g.height);
    size_t labelled = 0;
    for (size_t k = 0; k < members.size(); ++k) {
      const size_t i = members[k];
      if (assigned[i])
        continue;
      const auto &q = lp[k];
      const int c = col_of(q.x()) - g.col0;
      const int r = col_of(q.y()) - g.row0;
      if (c < 0 || r < 0 || c >= g.width || r >= g.height)
        continue; // only an above-ceiling point can fall outside
      const int32_t l = grown[g.index(c, r)];
      if (l > 0) {
        result.labels->points[i].label = static_cast<uint32_t>(l);
        assigned[i] = 1;
        ++labelled;
      }
    }

    info("band rooms: storey {}: {} x {} cells of {} m, band {:.2f}-{:.2f} "
         "m; {} wall cells ({} points), footprint {} cells ({} points), "
         "interior {:.1f} m^2; wall graph {} wall(s), {} door cut(s), {} "
         "cells; {} core(s), {} room(s) ({} without a core), {} pocket(s) "
         "merged, {} region(s) below {} m^2 dropped; {} of {} storey points "
         "labelled",
         st.idx, g.width, g.height, cell, band_lo - zf, band_hi - zf,
         wall_cells, n_wall_pts, fp_cells, n_fp_pts, g.interior_area,
         g.graph_walls, g.door_cuts, graph_cells, g.cores, rooms, g.coreless,
         g.merged_pockets, g.dropped_regions, o.min_room_area, labelled,
         members.size());
    if (rooms == 0)
      warn("band rooms: storey {} produced no room: {} wall cells, footprint "
           "{} cells, interior {:.1f} m^2, {} core(s) farther than {} m from "
           "a wall",
           st.idx, wall_cells, fp_cells, g.interior_area, g.cores,
           o.core_radius);
    if (fp_cells == 0)
      warn("band rooms: storey {} has no floor or ceiling evidence (|n_z| > "
           "{} below floor + {} m or above floor + {} m); check the normals",
           st.idx, o.horizontal_min_abs_nz, o.floor_max_height,
           o.ceiling_min_height);

    next_label += rooms;
    result.rooms += rooms;
    result.grids.push_back(std::move(g));
  }

  for (const auto &l : result.labels->points)
    if (l.label == 0)
      ++result.unlabeled_points;
  if (result.rooms == 0)
    warn("band rooms: no room in {} storey(s), {} points left unlabeled",
         order.size(), result.unlabeled_points);
  return result;
}

} // namespace reusex::geometry
