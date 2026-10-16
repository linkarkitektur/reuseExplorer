// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "core/logging.hpp"
#include "geometry/utils.hpp"
#include "reconstruction/CellComplex.hpp"
#include "types.hpp"

// #include <CGAL/Arr_linear_traits_2.h>
#include <CGAL/Arr_non_caching_segment_traits_2.h>
#include <CGAL/Arr_walk_along_line_point_location.h>
#include <CGAL/Arrangement_with_history_2.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>

#include <range/v3/to_container.hpp>
#include <range/v3/view/transform.hpp>
#include <range/v3/view/zip.hpp>

#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
#include <unordered_map>

using Vd = reusex::geometry::CellComplex::Vertex;
using Fd = reusex::geometry::CellComplex::Vertex;
using Cd = reusex::geometry::CellComplex::Vertex;

namespace reusex::geometry {
CellComplex::CellComplex(
    std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>>
        &planes_vec,
    std::vector<size_t> &verticals, const std::vector<size_t> &horizontals,
    std::vector<std::pair<size_t, size_t>> &pairs, std::array<double, 2> min_xy,
    std::array<double, 2> max_xy,
    std::optional<
        std::function<void(size_t, std::vector<std::array<double, 3>> const &,
                           std::vector<int> const &)>>
        viz_func,
    size_t max_cells, const std::vector<std::array<double, 2>> &plane_z_ranges,
    double min_section_overlap_m, const std::vector<size_t> &obliques)
    : n_rooms(0), n_walls(pairs.size()) {

  if (!plane_z_ranges.empty() && plane_z_ranges.size() != planes_vec.size())
    throw std::invalid_argument(
        fmt::format("CellComplex: {} plane z-ranges for {} planes",
                    plane_z_ranges.size(), planes_vec.size()));

  using EPECK = CGAL::Exact_predicates_exact_constructions_kernel;
  using Traits = CGAL::Arr_non_caching_segment_traits_2<EPECK>;
  using Arrangement = CGAL::Arrangement_with_history_2<Traits>;
  using NT = EPECK::FT;

  using Plane_3 = EPECK::Plane_3;
  using Line_3 = EPECK::Line_3;
  using Iso_rectangle = EPECK::Iso_rectangle_2;

  using Point_2 = Traits::Point_2;
  using Line_2 = Traits::Line_2;
  using Segment = Traits::X_monotone_curve_2;
  using Direction_2 = Traits::Direction_2;

  using Curve_handle = Arrangement::Curve_handle;

  // ── BEGIN coordinate recentering (issue #216) ───────────────────────────
  // CGAL's inexact-construction predicates lose precision on georeferenced
  // (UTM / 500 km-scale) coordinates. Translate all input geometry by
  // `-recenter` into a frame near the origin before building the arrangement,
  // then translate every output vertex/face/cell position back by `+recenter`
  // at the end of the constructor. `planes_vec`, `min_xy`, `max_xy` are shifted
  // here; `recenter` is restored on output. This block is self-contained: it
  // introduces `recenter`, rebinds the plane/bounds locals, and is undone only
  // by the matching restore block below.
  Eigen::Vector3d recenter = Eigen::Vector3d::Zero();
  {
    // XY: bounding-box center. Z: mean height of the horizontal planes, so the
    // whole model straddles the origin on every axis.
    recenter.x() = 0.5 * (min_xy[0] + max_xy[0]);
    recenter.y() = 0.5 * (min_xy[1] + max_xy[1]);
    double z_sum = 0.0;
    size_t z_cnt = 0;
    for (size_t id : horizontals) {
      const auto &p = planes_vec[id]; // (a,b,c,d), height = -d/c for a z-plane
      if (std::abs(p[2]) > 1e-9) {
        z_sum += -p[3] / p[2];
        ++z_cnt;
      }
    }
    recenter.z() = z_cnt ? z_sum / static_cast<double>(z_cnt) : 0.0;

    // Shift each plane: a point x satisfies n·x + d = 0; in the recentered
    // frame x' = x - recenter, so d' = d + n·recenter.
    for (auto &p : planes_vec)
      p[3] += p.template head<3>().dot(recenter);

    min_xy[0] -= recenter.x();
    max_xy[0] -= recenter.x();
    min_xy[1] -= recenter.y();
    max_xy[1] -= recenter.y();

    reusex::debug("CellComplex: recentering geometry by (-{:.3f}, -{:.3f}, "
                  "-{:.3f}) before CGAL arrangement",
                  recenter.x(), recenter.y(), recenter.z());
  }
  // ── END coordinate recentering ──────────────────────────────────────────

  auto planes = planes_vec | ranges::views::transform([](auto const &p) {
                  return Plane_3(p[0], p[1], p[2], p[3]);
                }) |
                ranges::to<std::vector>();

  auto volumes = this->add_property_map<Cd, double>("c:volume").first;
  auto areas = this->add_property_map<Fd, double>("f:area").first;
  // Section (storey) index for each cell, written in the cell-creation loop
  // below. Lets the Solidifier partition the MIP per horizontal section
  // (issue #226) without re-deriving sections from cell z-positions.
  auto sections = this->add_property_map<Cd, int>("c:section").first;

  reusex::trace("Constructing arrangement with {} vertical planes and "
                "[({:.3f}),({:.3f})] bounding box",
                verticals.size(), fmt::join(min_xy, ","),
                fmt::join(max_xy, ","));

  Iso_rectangle rect(Point_2(min_xy[0], min_xy[1]),
                     Point_2(max_xy[0], max_xy[1]));

  // Attempt to build the clipped ground-plane segment for one plane. Returns
  // nullopt when the plane does not behave as a vertical wall (does not
  // intersect z=0, or clips to nothing) so the caller can drop it and report
  // a summary instead of aborting on the first bad plane (issue #213).
  auto try_make_segment =
      [&rect](Eigen::Vector4d const &v4d) -> std::optional<Segment> {
    auto const plane = Plane_3(v4d[0], v4d[1], v4d[2], v4d[3]);

    // Intersect with ground plane z=0
    const Plane_3 ground_plane(0, 0, 1, 0);
    auto intersection = CGAL::intersection(ground_plane, plane);
    if (!intersection)
      return std::nullopt; // horizontal / non-vertical plane

    const auto *line_3d_ptr = std::get_if<Line_3>(&intersection.value());
    if (!line_3d_ptr)
      return std::nullopt;
    const auto line_3d = *line_3d_ptr;
    const Point_2 p(line_3d.point().x(), line_3d.point().y());
    const Direction_2 d(line_3d.direction().dx(), line_3d.direction().dy());
    const Line_2 line_2d(p, d);

    // Clip line to bounding rectangle
    auto clipped = CGAL::intersection(rect, line_2d);
    if (!clipped)
      return std::nullopt;

    if (!std::holds_alternative<Segment>(clipped.value()))
      return std::nullopt;

    return std::get<Segment>(clipped.value());
  };

  // ── Wall segments ───────────────────────────────────────────────────────
  // One clipped ground-plane segment per vertical plane that behaves as a
  // wall; `kept` lists the positions (into `verticals`) of those that do.
  // Positions 0 .. verticals.size()-1 are the wall lines; the roof lines of
  // oblique planes (reusex-e6z) are appended per section further down.
  // seg_plane[i] is the plane a line comes from.
  std::vector<std::optional<Segment>> segments(verticals.size());
  std::vector<size_t> seg_plane(verticals.begin(), verticals.end());
  std::vector<size_t> kept;
  size_t dropped_non_vertical = 0;
  int example_dropped_plane = -1;
  for (size_t i = 0; i < verticals.size(); ++i) {
    segments[i] = try_make_segment(planes_vec[verticals[i]]);
    if (!segments[i]) {
      ++dropped_non_vertical;
      if (example_dropped_plane < 0)
        example_dropped_plane = static_cast<int>(verticals[i]);
      continue;
    }
    kept.push_back(i);
  }
  const size_t inserted_verticals = kept.size();

  // Report dropped non-vertical planes once, with a count and an example
  // (issue #213). This is loud (§5) because dropping planes silently changes
  // the reconstruction.
  if (dropped_non_vertical > 0) {
    const auto &ex = planes_vec[example_dropped_plane];
    reusex::warn(
        "CellComplex: dropped {} plane(s) tagged vertical that did not "
        "intersect the ground plane (kept {} of {}). Example plane {}: "
        "[{:.3f}, {:.3f}, {:.3f}, {:.3f}].",
        dropped_non_vertical, inserted_verticals, verticals.size(),
        example_dropped_plane, ex[0], ex[1], ex[2], ex[3]);
  }

  // One 2D arrangement per distinct set of walls, built once and shared by
  // every section (and level) that uses the same set. Without z-ranges every
  // section uses one set, the arrangement of all walls, as before reusex-sba.
  using Walk_pl = CGAL::Arr_walk_along_line_point_location<Arrangement>;
  struct ArrInfo {
    Arrangement arr;
    std::unordered_map<Curve_handle, int> ch_map;
    size_t bounded = 0;
    std::unique_ptr<Walk_pl> pl;
  };
  using WallSet = std::vector<size_t>; // sorted positions into `verticals`
  std::map<WallSet, std::unique_ptr<ArrInfo>> arrangements;
  auto arrangement_of = [&](const WallSet &set) -> ArrInfo * {
    auto &slot = arrangements[set];
    if (!slot) {
      slot = std::make_unique<ArrInfo>();
      for (size_t i : set)
        slot->ch_map[insert(slot->arr, *segments[i])] =
            static_cast<int>(seg_plane[i]);
      for (auto fit = slot->arr.faces_begin(); fit != slot->arr.faces_end();
           ++fit)
        slot->bounded += fit->is_unbounded() ? 0 : 1;
      slot->pl = std::make_unique<Walk_pl>(slot->arr);
    }
    return slot.get();
  };
  auto set_union = [](const WallSet &a, const WallSet &b) {
    WallSet out;
    std::set_union(a.begin(), a.end(), b.begin(), b.end(),
                   std::back_inserter(out));
    return out;
  };

  // Visualize the arrangement of every wall
  if (viz_func) {
    reusex::trace("Visualizing arrangement");
    const auto &arr = arrangement_of(kept)->arr;
    size_t count = 0;
    for (auto fit = arr.faces_begin(); fit != arr.faces_end(); ++fit, ++count) {
      if (fit->is_unbounded())
        continue;

      std::vector<std::array<double, 3>> pts;
      for (auto vit = fit->outer_ccb(), done = vit;;) {
        auto v = vit->source();
        pts.push_back({CGAL::to_double(v->point().x()),
                       CGAL::to_double(v->point().y()), 0.0});
        if (++vit == done)
          break;
      }

      std::vector<int> indices(pts.size() + 1);
      std::iota(indices.begin(), indices.end(), 0);
      indices.back() = 0,

      viz_func.value()(count, pts, indices);
    }
  }

  auto sorted_floors = horizontals;
  std::sort(sorted_floors.begin(), sorted_floors.end(),
            [&planes](auto a, auto b) {
              const auto &Pa = planes[a];
              const auto &Pb = planes[b];
              const double h1 = CGAL::to_double(-Pa.d() * Pa.c());
              const double h2 = CGAL::to_double(-Pb.d() * Pb.c());
              return h1 < h2;
            });
  const size_t n_levels = sorted_floors.size();
  const size_t n_sections = n_levels < 2 ? 0 : n_levels - 1;
  std::vector<double> level_z(n_levels);
  for (size_t k = 0; k < n_levels; ++k) {
    const auto &plane = planes[sorted_floors[k]];
    level_z[k] = CGAL::to_double(-plane.d() * plane.c());
  }

  // ── Walls per section (reusex-sba) ──────────────────────────────────────
  // A wall belongs to a section when its z-range overlaps the section by
  // min_section_overlap_m, or by half the section for a thin one (a slab).
  std::vector<WallSet> raw_sets(n_sections);
  for (size_t s = 0; s < n_sections; ++s) {
    const double h0 = level_z[s], h1 = level_z[s + 1];
    const double need = std::min(min_section_overlap_m, 0.5 * (h1 - h0));
    for (size_t i : kept) {
      if (plane_z_ranges.empty()) {
        raw_sets[s].push_back(i);
        continue;
      }
      const auto &r = plane_z_ranges[verticals[i]];
      const double lo = r[0] - recenter.z(), hi = r[1] - recenter.z();
      if (!(lo <= hi) || std::min(hi, h1) - std::max(lo, h0) >= need)
        raw_sets[s].push_back(i);
    }
  }
  // Roof lines (reusex-e6z): where each oblique plane reaching section s
  // crosses the section's mid-height, as a line in its arrangement. Appended to
  // `segments`, so the ids grow and each returned set is sorted.
  auto roof_lines = [&](size_t s) {
    WallSet out;
    if (plane_z_ranges.empty())
      return out;
    const double h0 = level_z[s], h1 = level_z[s + 1];
    const double need = std::min(min_section_overlap_m, 0.5 * (h1 - h0));
    for (size_t id : obliques) {
      const auto &p = planes_vec[id];
      const auto &r = plane_z_ranges[id];
      if (!(r[0] <= r[1]) || std::abs(p[2]) > 0.95 ||
          std::hypot(p[0], p[1]) < 1e-6)
        continue;
      const double lo = std::max(r[0] - recenter.z(), h0);
      const double hi = std::min(r[1] - recenter.z(), h1);
      if (hi - lo < need)
        continue;
      // The line at the section's mid-height, where its cell centres lie:
      // the face's two cells then lie on either side of the oblique plane
      // too, which is what Face::is_main (get_a / get_b) is decided by.
      const double z = 0.5 * (h0 + h1);
      // n . (x, y, z) + d = 0 at this height: a line in the plan.
      const Line_2 line(NT(p[0]), NT(p[1]), NT(p[2] * z + p[3]));
      auto clipped = CGAL::intersection(rect, line);
      if (!clipped || !std::holds_alternative<Segment>(clipped.value()))
        continue;
      segments.emplace_back(std::get<Segment>(clipped.value()));
      seg_plane.push_back(id);
      out.push_back(segments.size() - 1);
    }
    return out;
  };

  std::vector<WallSet> sets(n_sections);
  std::vector<WallSet> roofs(n_sections);
  std::vector<ArrInfo *> section_arr(n_sections, nullptr);
  for (size_t s = 0; s < n_sections; ++s) {
    sets[s] = raw_sets[s];
    if (arrangement_of(sets[s])->bounded == 0) {
      // A section no wall line reaches (or too few to enclose a face): a
      // slab between or around storeys, or an attic under oblique roof
      // planes. It takes the walls of the nearest sections whose walls do
      // enclose faces, so its cells line up with theirs: the window around
      // it grows one section at a time until the union encloses a face.
      // Only when no section does (no z-range reaches anywhere) does it
      // fall back to every wall.
      for (size_t r = 1; arrangement_of(sets[s])->bounded == 0; ++r) {
        if (r > s && s + r >= n_sections) {
          sets[s] = kept;
          break;
        }
        if (r <= s)
          sets[s] = set_union(sets[s], raw_sets[s - r]);
        if (s + r < n_sections)
          sets[s] = set_union(sets[s], raw_sets[s + r]);
      }
      // Under a pitched roof the neighbours' walls run out under the eaves;
      // the roof planes reaching into the section say where it ends.
      const auto roof = roof_lines(s);
      roofs[s] = roof;
      sets[s] = set_union(sets[s], roof);
      reusex::debug("CellComplex: section {} ({:.2f}-{:.2f} m) encloses no "
                    "face with its own {} wall line(s); using {} from the "
                    "neighbouring sections and {} roof line(s)",
                    s, level_z[s] + recenter.z(), level_z[s + 1] + recenter.z(),
                    raw_sets[s].size(), sets[s].size() - roof.size(),
                    roof.size());
    }
    section_arr[s] = arrangement_of(sets[s]);
  }
  // A thick section without faces of its own (an attic under a pitched
  // roof, not a slab) is not bounded by the nearest storey's walls: where
  // the roof space is open to a side no scanned wall closes, the cell
  // complex needs the lines of other storeys' walls there too, or the MIP
  // finds no wall to end the room at and drops it (house2, reusex-e6z). Such
  // a section takes every wall, as before reusex-sba, while the projected
  // cell count stays within max_cells; lowest section first.
  {
    size_t total = 0;
    for (auto *a : section_arr)
      total += a->bounded;
    for (size_t s = 0; s < n_sections; ++s) {
      const double thickness = level_z[s + 1] - level_z[s];
      if (sets[s] == raw_sets[s] || thickness < 2.0 * min_section_overlap_m)
        continue;
      auto all = set_union(kept, roofs[s]);
      if (all == sets[s])
        continue;
      auto *a = arrangement_of(all);
      const size_t grown = total - section_arr[s]->bounded + a->bounded;
      if (grown > max_cells) {
        reusex::debug("CellComplex: section {} ({:.2f}-{:.2f} m) keeps the "
                      "nearest walls: every wall would make {} cells > {}",
                      s, level_z[s] + recenter.z(),
                      level_z[s + 1] + recenter.z(), grown, max_cells);
        continue;
      }
      reusex::debug("CellComplex: section {} ({:.2f}-{:.2f} m) has no walls "
                    "of its own: using every wall ({} -> {} faces)",
                    s, level_z[s] + recenter.z(), level_z[s + 1] + recenter.z(),
                    section_arr[s]->bounded, a->bounded);
      total = grown;
      sets[s] = std::move(all);
      section_arr[s] = a;
    }
  }
  // Levels: the arrangement of a level is the overlay of the sections below
  // and above it (the arrangement of the union of their walls).
  std::vector<ArrInfo *> level_arr(n_levels, nullptr);
  for (size_t k = 0; k < n_levels && n_sections > 0; ++k) {
    WallSet set;
    if (k > 0)
      set = sets[k - 1];
    if (k < n_sections)
      set = set_union(set, sets[k]);
    level_arr[k] = arrangement_of(set);
  }
  const bool uniform =
      n_sections > 0 &&
      std::all_of(section_arr.begin(), section_arr.end(),
                  [&](ArrInfo *a) { return a == section_arr.front(); });

  // Cell-count guard (issue #213). The number of volumetric cells is the sum
  // over sections of that section's bounded 2D arrangement faces. Fail fast
  // with an actionable message before the cell construction and the
  // intractable MIP that would follow.
  size_t projected_cells = 0;
  for (auto *a : section_arr)
    projected_cells += a->bounded;
  size_t max_section_faces = 0;
  for (auto *a : section_arr)
    max_section_faces = std::max(max_section_faces, a->bounded);
  const size_t faces_for_report =
      n_sections == 0 ? arrangement_of(kept)->bounded : max_section_faces;
  if (uniform || n_sections == 0) {
    reusex::info("Arrangement statistics: {} vertical planes in ({} kept), "
                 "{} horizontal planes, {} bounded faces, {} sections => "
                 "{} projected cells",
                 verticals.size(), inserted_verticals, n_levels,
                 faces_for_report, n_sections, projected_cells);
  } else {
    std::string per_section;
    for (size_t s = 0; s < n_sections; ++s)
      per_section += fmt::format("{}{}w/{}f", s ? ", " : "", sets[s].size(),
                                 section_arr[s]->bounded);
    reusex::info("Arrangement statistics (storey-local): {} vertical planes "
                 "in ({} kept), {} horizontal planes, {} sections [{}] => {} "
                 "projected cells (all walls in every section: {} faces x {} "
                 "sections = {})",
                 verticals.size(), inserted_verticals, n_levels, n_sections,
                 per_section, projected_cells, arrangement_of(kept)->bounded,
                 n_sections, arrangement_of(kept)->bounded * n_sections);
  }

  // Zero-cell guard (issue reusex-qw0). A 0-cell complex is a structurally
  // degenerate arrangement, not a solver problem: feeding it to Solidifier
  // would build a vacuous 0-variable MIP and report a misleading "increase
  // --time-limit / use --solver highs / stronger plane merging" hint that
  // names the wrong cause. Fail fast here instead, naming the actual cause
  // (0 bounded faces, or fewer than 2 horizontal planes) and the plane
  // counts, before Solidifier is ever constructed — so no solve is attempted.
  if (projected_cells == 0) {
    const std::string reason =
        faces_for_report == 0
            ? "two vertical planes cannot enclose a cell"
            : "fewer than 2 horizontal planes bound no section to extrude "
              "cells into";
    throw std::runtime_error(fmt::format(
        "CellComplex: arrangement has {} bounded faces from {} vertical ({} "
        "kept) / {} horizontal planes, {} sections => 0 projected cells; {}. "
        "Check 'rux create planes' output (stronger region growing, more "
        "planes, or weaker merging) before retrying create mesh.",
        faces_for_report, verticals.size(), inserted_verticals, n_levels,
        n_sections, reason));
  }

  if (projected_cells > max_cells) {
    throw std::runtime_error(fmt::format(
        "CellComplex: projected cell count {} exceeds max_cells {} "
        "({} sections, up to {} bounded arrangement faces in one section; "
        "{} vertical planes). The plane arrangement is over-fragmented; "
        "consider stronger plane merging, --filter to a subregion, or "
        "raising the limit.",
        projected_cells, max_cells, n_sections, faces_for_report,
        inserted_verticals));
  }

  // ── Vertices: one per arrangement vertex of each level ──────────────────
  struct LessXY {
    bool operator()(const Point_2 &a, const Point_2 &b) const {
      return CGAL::compare_xy(a, b) == CGAL::SMALLER;
    }
  };
  std::vector<std::map<Point_2, Vd, LessXY>> point_map(n_levels);
  reusex::trace("Create vertices");
  for (size_t k = 0; k < n_levels; ++k) {
    const auto &arr = level_arr[k]->arr;
    for (auto vit = arr.vertices_begin(); vit != arr.vertices_end(); ++vit)
      point_map[k][vit->point()] = this->add_vertex(
          Eigen::Vector3d(CGAL::to_double(vit->point().x()),
                          CGAL::to_double(vit->point().y()), level_z[k]));
  }

  // ── Horizontal faces ────────────────────────────────────────────────────
  // A face of level k lies in exactly one face of the section below it and
  // one of the section above (each section's walls are a subset of the
  // level's). bottom[s][f] / top[s][f]: the level faces covering face f of
  // section s's arrangement from below / above.
  using FaceKey = Arrangement::Face_const_handle;
  using FaceLists = std::unordered_map<FaceKey, std::vector<Fd>>;
  std::vector<FaceLists> bottom(n_sections), top(n_sections);

  // The face of section s's arrangement that contains `face` of level k's.
  auto parent_face = [&](size_t s, size_t k, FaceKey face,
                         const Point_2 &inside) -> std::optional<FaceKey> {
    if (section_arr[s] == level_arr[k])
      return face;
    auto obj = section_arr[s]->pl->locate(inside);
    const auto *f = std::get_if<FaceKey>(&obj);
    if (!f || (*f)->is_unbounded())
      return std::nullopt;
    return *f;
  };

  auto make_level_face = [&](size_t k, FaceKey fit) {
    std::vector<Vd> verts{};
    NT cx = 0, cy = 0;
    std::vector<Point_2> pts;
    for (auto vit = fit->outer_ccb(), done = vit;;) {
      auto v = vit->source();
      cx += v->point().x();
      cy += v->point().y();
      verts.push_back(point_map[k][v->point()]);
      pts.emplace_back(v->point().x(), v->point().y());
      if (++vit == done)
        break;
    }
    cx /= static_cast<NT>(verts.size());
    cy /= static_cast<NT>(verts.size());

    // A bounded face of a line arrangement is convex, so the mean of its
    // corners lies strictly inside it.
    const Point_2 inside(cx, cy);
    const auto below =
        k > 0 ? parent_face(k - 1, k, fit, inside) : std::optional<FaceKey>{};
    const auto above = k < n_sections ? parent_face(k, k, fit, inside)
                                      : std::optional<FaceKey>{};
    if (!below && !above)
      return; // enclosed only by the two sections' walls together

    NT area_nt;
    CGAL::area_2(pts.begin(), pts.end(), area_nt);

    auto f = this->add_face(
        Eigen::Vector3d(CGAL::to_double(cx), CGAL::to_double(cy), level_z[k]),
        verts, sorted_floors[k] /* plane id */);
    areas[f] = CGAL::to_double(area_nt);
    if (below)
      top[k - 1][*below].push_back(f);
    if (above)
      bottom[k][*above].push_back(f);
  };

  reusex::trace("Create  horizontal faces");
  if (uniform) {
    // One arrangement for every level: faces in the order the code before
    // reusex-sba created them (face-major), so the complex is unchanged.
    const auto &arr = section_arr.front()->arr;
    for (auto fit = arr.faces_begin(); fit != arr.faces_end(); ++fit) {
      if (fit->is_unbounded())
        continue;
      for (size_t k = 0; k < n_levels; ++k)
        make_level_face(k, fit);
    }
  } else {
    for (size_t k = 0; k < n_levels; ++k) {
      const auto &arr = level_arr[k]->arr;
      for (auto fit = arr.faces_begin(); fit != arr.faces_end(); ++fit)
        if (!fit->is_unbounded())
          make_level_face(k, fit);
    }
  }

  // Level-k vertices strictly inside an edge of section s's arrangement:
  // where a finer section above or below meets this section's wall faces.
  // A wall face takes them as extra corners, so the mesh has no T-junctions.
  using EdgeKey = Arrangement::Halfedge_const_handle;
  auto edge_points = [&](size_t s, size_t k) {
    std::unordered_map<EdgeKey, std::vector<Point_2>> out;
    if (section_arr[s] == level_arr[k])
      return out;
    const auto &arr = level_arr[k]->arr;
    for (auto vit = arr.vertices_begin(); vit != arr.vertices_end(); ++vit) {
      auto obj = section_arr[s]->pl->locate(vit->point());
      if (const auto *he = std::get_if<EdgeKey>(&obj)) {
        out[*he].push_back(vit->point());
        out[(*he)->twin()].push_back(vit->point());
      }
    }
    return out;
  };
  // The points of `on` on the edge from `a` to `b`, ordered from a.
  auto between = [](const std::unordered_map<EdgeKey, std::vector<Point_2>> &on,
                    EdgeKey he, const Point_2 &a) {
    std::vector<Point_2> pts;
    if (auto it = on.find(he); it != on.end())
      pts = it->second;
    std::sort(pts.begin(), pts.end(), [&a](const Point_2 &p, const Point_2 &q) {
      return CGAL::has_smaller_distance_to_point(a, p, q);
    });
    return pts;
  };

  reusex::trace("Creating cells");
  for (size_t i = 0; i < n_sections; ++i) {
    const double h1 = level_z[i];
    const double h2 = level_z[i + 1];
    const double dist = h2 - h1;
    const double h = (h1 + h2) / 2.0;
    reusex::trace("Horizontal section thickness is {:.3}", dist);

    const auto on_bottom = edge_points(i, i);
    const auto on_top = edge_points(i, i + 1);
    std::unordered_map<Arrangement::Halfedge_handle, Fd> face_cache{};

    auto &arr = section_arr[i]->arr;
    for (auto fit = arr.faces_begin(); fit != arr.faces_end(); ++fit) {
      if (fit->is_unbounded())
        continue;
      std::vector<Fd> fc{};
      fc.reserve(10);
      const FaceKey key = fit;
      for (auto f : bottom[i][key])
        fc.push_back(f);
      for (auto f : top[i][key])
        fc.push_back(f);

      NT cx = 0, cy = 0;
      size_t n_edges = 0;
      std::vector<Point_2> corners;

      for (auto he : CGAL::Container_from_circulator(fit->outer_ccb())) {

        auto self = he.twin()->twin();
        auto twin = he.twin();
        // Runtime check (was assert, which vanishes in Release, issue #213).
        // A halfedge equal to its own twin signals corrupt / non-manifold
        // arrangement topology that would produce degenerate vertical faces.
        if (twin == self)
          throw std::runtime_error(fmt::format(
              "CellComplex: non-manifold topology in horizontal section {} "
              "(halfedge equals its own twin). The plane arrangement is "
              "degenerate; check for duplicate or near-coincident planes.",
              i));

        const Point_2 &src = he.source()->point();
        const Point_2 &tgt = he.target()->point();
        cx += src.x();
        cy += src.y();
        ++n_edges;
        corners.push_back(src);

        // INFO: Reuse vertical faces
        if (face_cache.contains(self)) {
          fc.push_back(face_cache[self]);
          continue;
        }

        // Bottom edge from source to target, then the top edge back.
        std::vector<Vd> pts{point_map[i][src]};
        for (const auto &p : between(on_bottom, self, src))
          pts.push_back(point_map[i][p]);
        pts.push_back(point_map[i][tgt]);
        pts.push_back(point_map[i + 1][tgt]);
        for (const auto &p : between(on_top, self, tgt))
          pts.push_back(point_map[i + 1][p]);
        pts.push_back(point_map[i + 1][src]);

        // Compute center of vertical face
        const Eigen::Vector3d p_src = (*this)[point_map[i][src]].pos;
        const Eigen::Vector3d p_tgt = (*this)[point_map[i][tgt]].pos;
        Eigen::Vector3d center = (p_src + p_tgt) / 2.0;
        center[2] = h;

        int plane_id = -1;
        for (auto cit = arr.originating_curves_begin(self);
             cit != arr.originating_curves_end(self); ++cit) {
          plane_id = section_arr[i]->ch_map[cit];
          break;
        }

        // Create vertical face
        auto f = this->add_face(center, pts, plane_id);
        areas[f] = dist * (p_src - p_tgt).norm();

        face_cache[twin] = f; // Cache twin halfedge
        fc.push_back(f);
      }

      // Compute center of cell
      cx /= static_cast<NT>(n_edges);
      cy /= static_cast<NT>(n_edges);
      Eigen::Vector3d center(CGAL::to_double(cx), CGAL::to_double(cy), h);

      NT footprint;
      CGAL::area_2(corners.begin(), corners.end(), footprint);

      // Create the cell
      auto c = this->add_cell(center, fc);
      volumes[c] = dist * CGAL::to_double(footprint);
      sections[c] = static_cast<int>(i);
    }
  }

  // Record the number of horizontal sections (storeys). Each section i spans
  // sorted_floors[i]..sorted_floors[i+1]; cells carry their section in
  // "c:section" (issue #226).
  this->n_sections = n_sections;

  // INFO: Assign wall ids to each cell
  reusex::info("Assigning wall ids to each cell");

  auto toEigenPlane = [](const Plane_3 &plane) {
    return Eigen::Vector4d(
        CGAL::to_double(plane.a()), CGAL::to_double(plane.b()),
        CGAL::to_double(plane.c()), CGAL::to_double(plane.d()));
  };

  // A wall pair whose faces are wall lines counts only in the sections those
  // lines reach (reusex-sba): a big cell of a storey without that wall may
  // have its centre between the two faces by chance. Pairs of other planes
  // (floor / ceiling slabs) count everywhere, as before.
  std::unordered_map<size_t, size_t> line_pos; // plane id -> kept position
  for (size_t i : kept)
    line_pos[verticals[i]] = i;
  std::vector<std::vector<bool>> pair_in_section(
      pairs.size(), std::vector<bool>(n_sections, true));
  for (size_t p = 0; p < pairs.size(); ++p) {
    const auto it1 = line_pos.find(pairs[p].first);
    const auto it2 = line_pos.find(pairs[p].second);
    if (it1 == line_pos.end() && it2 == line_pos.end())
      continue;
    for (size_t s = 0; s < n_sections; ++s) {
      auto has = [&](auto it) {
        return it != line_pos.end() &&
               std::binary_search(sets[s].begin(), sets[s].end(), it->second);
      };
      pair_in_section[p][s] = has(it1) || has(it2);
    }
  }

  for (size_t i = 0; i < pairs.size(); ++i) {
    const auto &[id1, id2] = pairs[i];
    auto eigen_plane1 = toEigenPlane(planes[id1]);
    auto eigen_plane2 = toEigenPlane(planes[id2]);

    for (auto cit = this->cells_begin(); cit != this->cells_end(); ++cit) {
      if (!pair_in_section[i][sections[*cit]])
        continue;
      auto p = (*this)[*cit].pos;
      auto dist1 = dist_plane_point(eigen_plane1, p);
      if (dist1 > 0)
        continue;
      auto dist2 = dist_plane_point(eigen_plane2, p);
      if (dist2 > 0)
        continue;

      std::get<CellData>((*this)[*cit].data).wall_ids.insert(i);
    }
  }

  // INFO: Set main cell for each face
  reusex::info("Setting main cell for each face");
  for (auto fit = this->faces_begin(); fit != this->faces_end(); ++fit) {
    // const auto face_id = (*this)[*fit].id;
    const auto plane_id = std::get<FaceData>((*this)[*fit].data).plane_id;

    if (plane_id < 0) {
      reusex::warn("Face {} has no associated plane", (*this)[*fit].id);
      continue;
    }

    const auto plane = planes[plane_id];
    Eigen::Vector4d plane_vec(
        CGAL::to_double(plane.a()), CGAL::to_double(plane.b()),
        CGAL::to_double(plane.c()), CGAL::to_double(plane.d()));

    auto [begin, end] = boost::out_edges(*fit, *this);
    for (auto eit = begin; eit != end; ++eit) {

      // Get the adjacent cell
      const auto t = boost::target(*eit, *this);
      const auto s = boost::source(*eit, *this);
      auto c = (*fit == t) ? s : t;
      if ((*this)[c].type != CellNodeType::cell)
        continue;

      // reusex::trace("Face {} is adjacent to cell {}",
      //               (*this)[*fit].id,
      //               (*this)[c].id);

      // const int cell_id = (*this)[c].id;
      // const PointT &center = cell_centers->points[cell_id];
      const auto &center = (*this)[c].pos;

      const auto dist = dist_plane_point(plane_vec, center);
      (*this)[*eit].is_main = dist >= 0;
      // auto status = (*this)[*eit].is_main
      //                   ? fmt::format(fmt::fg(fmt::rgb(0, 255, 0)), "true")
      //                   : fmt::format(fmt::fg(fmt::rgb(255, 0, 0)),
      //                   "false");
      // reusex::trace("Face {:<3} cell {:>3} is main {}",
      //               (*this)[*fit].id,
      //               (*this)[c].id, status);
    }
  }

  // ── BEGIN coordinate recentering restore (issue #216) ────────────────────
  // Undo the up-front translation: every node position (vertex, face center,
  // cell center) was constructed in the recentered frame; add `recenter` back
  // so outputs are in the original (georeferenced) coordinate system. All
  // orientation-only quantities (`is_main`, wall ids, areas, volumes) are
  // translation-invariant and need no adjustment.
  if (!recenter.isZero()) {
    auto [vb, ve] = boost::vertices(*this);
    for (auto vit = vb; vit != ve; ++vit)
      (*this)[*vit].pos += recenter;
  }
  // ── END coordinate recentering restore ───────────────────────────────────
}
} // namespace reusex::geometry
