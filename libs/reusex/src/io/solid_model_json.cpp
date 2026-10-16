// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "io/solid_model_json.hpp"

#include "core/ProjectDB.hpp"
#include "yaml_json.hpp"

#include <fmt/format.h>
#include <nlohmann/json.hpp>

#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Gps_segment_traits_2.h>

#include <algorithm>
#include <cctype>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <initializer_list>
#include <limits>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_set>

namespace reusex::io {
namespace {

using json = nlohmann::json;
using ojson = nlohmann::ordered_json;

[[noreturn]] void fail(const std::string &where, const std::string &what) {
  throw std::invalid_argument(
      fmt::format("solid model JSON: {}: {}", where, what));
}

/// One JSON object being read, with the name of the element it describes
/// ("walls[2] (guid 'w3')") so every error names its source.
class Reader {
    public:
  Reader(const json &j, std::string where) : j_(j), where_(std::move(where)) {
    if (!j_.is_object())
      fail(where_, "expected an object");
  }

  const std::string &where() const { return where_; }

  /// Reject keys outside @p allowed: a typo must not silently become a default.
  void only(std::initializer_list<const char *> allowed) const {
    for (const auto &[key, _] : j_.items()) {
      bool ok = false;
      for (const char *a : allowed)
        ok = ok || key == a;
      if (!ok)
        fail(where_, fmt::format("unknown key '{}'", key));
    }
  }

  bool has(const char *key) const { return j_.contains(key); }

  const json &at(const char *key) const {
    if (!j_.contains(key))
      fail(where_, fmt::format("missing required key '{}'", key));
    return j_.at(key);
  }

  double number(const char *key, double fallback) const {
    if (!has(key))
      return fallback;
    const json &v = j_.at(key);
    if (!v.is_number())
      fail(where_, fmt::format("'{}' must be a number", key));
    const double d = v.get<double>();
    if (!std::isfinite(d))
      fail(where_, fmt::format("'{}' must be finite", key));
    return d;
  }

  int integer(const char *key, int fallback) const {
    if (!has(key))
      return fallback;
    return to_int(j_.at(key), fmt::format("'{}'", key));
  }

  /// An integer that fits an `int`; anything else throws naming @p what.
  int to_int(const json &v, const std::string &what) const {
    constexpr auto lo = std::numeric_limits<int>::min();
    constexpr auto hi = std::numeric_limits<int>::max();
    bool ok = false;
    if (v.is_number_unsigned())
      ok = v.get<std::uint64_t>() <= static_cast<std::uint64_t>(hi);
    else if (v.is_number_integer())
      ok = v.get<std::int64_t>() >= lo && v.get<std::int64_t>() <= hi;
    if (!ok)
      fail(where_,
           fmt::format("{} must be an integer in [{}, {}]", what, lo, hi));
    return static_cast<int>(v.get<std::int64_t>());
  }

  /// A JSON array of exactly @p n finite numbers.
  std::vector<double> numbers(const char *key, std::size_t n,
                              const char *shape) const {
    const json &v = at(key);
    if (!v.is_array() || v.size() != n)
      fail(where_, fmt::format("'{}' must be {}", key, shape));
    std::vector<double> out;
    for (const auto &e : v) {
      if (!e.is_number() || !std::isfinite(e.get<double>()))
        fail(where_,
             fmt::format("'{}' must be {} (finite numbers)", key, shape));
      out.push_back(e.get<double>());
    }
    return out;
  }

  bool boolean(const char *key, bool fallback) const {
    if (!has(key))
      return fallback;
    const json &v = j_.at(key);
    if (!v.is_boolean())
      fail(where_, fmt::format("'{}' must be true or false", key));
    return v.get<bool>();
  }

  std::string text(const char *key, const std::string &fallback) const {
    if (!has(key))
      return fallback;
    const json &v = j_.at(key);
    if (!v.is_string())
      fail(where_, fmt::format("'{}' must be a string", key));
    return v.get<std::string>();
  }

  /// `metadata` is any JSON value in the document, text in the record.
  std::string metadata() const {
    if (!has("metadata") || j_.at("metadata").is_null())
      return {};
    return j_.at("metadata").dump();
  }

  core::PlanPoint point(const json &v, const std::string &what) const {
    if (!v.is_array() || v.size() != 2 || !v[0].is_number() ||
        !v[1].is_number())
      fail(where_, fmt::format("{} must be a [x, y] pair of numbers", what));
    core::PlanPoint p{v[0].get<double>(), v[1].get<double>()};
    if (!std::isfinite(p[0]) || !std::isfinite(p[1]))
      fail(where_, fmt::format("{} must be finite", what));
    return p;
  }

  /// An open ring. A repeated closing point (GeoJSON style) is dropped, so
  /// rings are always stored and written open.
  std::vector<core::PlanPoint> ring(const json &v,
                                    const std::string &what) const {
    if (!v.is_array())
      fail(where_, fmt::format("{} must be an array of [x, y] points", what));
    std::vector<core::PlanPoint> out;
    out.reserve(v.size());
    for (std::size_t i = 0; i < v.size(); ++i)
      out.push_back(point(v[i], fmt::format("{}[{}]", what, i)));
    if (out.size() > 3 && out.front() == out.back())
      out.pop_back();
    if (out.size() < 3)
      fail(where_,
           fmt::format("{} needs at least 3 points, has {}", what, out.size()));
    return out;
  }

    private:
  const json &j_;
  std::string where_;
};

std::string element_name(const char *array, std::size_t i, const json &e) {
  if (e.is_object() && e.contains("guid") && e.at("guid").is_string())
    return fmt::format("{}[{}] (guid '{}')", array, i,
                       e.at("guid").get<std::string>());
  return fmt::format("{}[{}]", array, i);
}

/// Iterate the array under @p key (if present) and mark it provided.
template <typename Fn>
bool for_each_element(const json &doc, const char *key, Fn &&fn) {
  if (!doc.contains(key))
    return false;
  const json &arr = doc.at(key);
  if (!arr.is_array())
    fail(key, "must be an array");
  for (std::size_t i = 0; i < arr.size(); ++i)
    fn(Reader(arr[i], element_name(key, i, arr[i])));
  return true;
}

void check_unique_guid(std::unordered_set<std::string> &seen,
                       const std::string &guid, const std::string &where) {
  if (guid.empty())
    return;
  if (!seen.insert(guid).second)
    fail(where, fmt::format("duplicate guid '{}'", guid));
}

void read_common(const Reader &r, std::string &source, bool &locked,
                 double &confidence, std::string &metadata) {
  source = r.text("source", source);
  if (source != "detected" && source != "manual" && source != "imported")
    fail(r.where(),
         fmt::format("'source' must be detected, manual or imported, not '{}'",
                     source));
  locked = r.boolean("locked", locked);
  confidence = r.number("confidence", confidence);
  metadata = r.metadata();
}

/// A record's `metadata` is JSON text by contract (the reader stores the
/// document's value as `dump()`); anything else could not be written so that
/// it reads back as the same record, so it is rejected.
ojson metadata_value(const std::string &text, const std::string &where) {
  try {
    return ojson::parse(text);
  } catch (const ojson::parse_error &) {
    fail(where, "'metadata' is not JSON text");
  }
}

template <typename R>
void write_common(ojson &o, const R &r, const std::string &where) {
  o["source"] = r.source;
  o["locked"] = r.locked;
  o["confidence"] = r.confidence;
  if (!r.metadata.empty())
    o["metadata"] = metadata_value(r.metadata, where);
}

std::string record_name(const char *array, std::size_t i,
                        const std::string &guid) {
  return guid.empty() ? fmt::format("{}[{}]", array, i)
                      : fmt::format("{}[{}] (guid '{}')", array, i, guid);
}

using EK = CGAL::Exact_predicates_exact_constructions_kernel;
using Traits = CGAL::Gps_segment_traits_2<EK>;
using Polygon = Traits::Polygon_2;
using PolygonWithHoles = Traits::Polygon_with_holes_2;

/// A simple ring with non-zero area, oriented counter-clockwise for the
/// outer boundary and clockwise for a hole.
Polygon checked_ring(const std::vector<core::PlanPoint> &ring,
                     const std::string &where, const std::string &what,
                     bool hole) {
  Polygon p;
  for (const auto &q : ring)
    p.push_back(EK::Point_2(q[0], q[1]));
  if (!p.is_simple())
    fail(where, fmt::format("{} is not a simple polygon (it touches or "
                            "crosses itself, or repeats a point)",
                            what));
  if (p.orientation() == CGAL::ZERO)
    fail(where, fmt::format("{} has zero area (collinear points)", what));
  if ((p.orientation() == CGAL::COUNTERCLOCKWISE) == hole)
    p.reverse_orientation();
  return p;
}

/// Outer ring and holes form one valid polygon with holes: every ring simple
/// and non-degenerate, every hole strictly inside the outer ring, holes
/// pairwise disjoint. Either winding is accepted; it is not changed.
void check_polygon(const core::SpacePolygon &poly, const std::string &where) {
  PolygonWithHoles pwh(checked_ring(poly.outer, where, "polygon.outer", false));
  for (std::size_t i = 0; i < poly.holes.size(); ++i)
    pwh.add_hole(checked_ring(poly.holes[i], where,
                              fmt::format("polygon.holes[{}]", i), true));
  if (!poly.holes.empty() && !CGAL::is_valid_polygon_with_holes(pwh, Traits()))
    fail(where, "polygon.holes must lie strictly inside polygon.outer and "
                "must not overlap or touch each other");
}

ojson ring_json(const std::vector<core::PlanPoint> &ring) {
  ojson a = ojson::array();
  for (const auto &p : ring)
    a.push_back({p[0], p[1]});
  return a;
}

// ── YAML front-end (reusex-yf4.1) ───────────────────────────────────────────
//
// `solid_model_from_yaml` converts the YAML text into a `nlohmann::json`
// value (yaml_json.hpp, shared with the pose-edits reader) and hands it to
// the same strict validator `solid_model_from_json` uses, so there is exactly
// one reader. `solid_model_to_yaml` emits the same ordered document
// `solid_model_to_json` builds, so both front ends agree on key order.

ojson build_document(const core::SolidModel &m) {
  ojson doc;
  doc["model_version"] = kSolidModelJsonVersion;
  doc["units"] = "m";
  if (!m.name.empty())
    doc["name"] = m.name;
  if (!m.description.empty())
    doc["description"] = m.description;

  if (m.has_storeys()) {
    ojson arr = ojson::array();
    for (const auto &s : m.storeys) {
      ojson o;
      if (!s.guid.empty())
        o["guid"] = s.guid;
      o["idx"] = s.idx;
      if (!s.name.empty())
        o["name"] = s.name;
      o["z_floor"] = s.z_floor;
      o["z_ceiling"] = s.z_ceiling;
      o["level_transform"] = s.level_transform;
      write_common(o, s, record_name("storeys", &s - m.storeys.data(), s.guid));
      arr.push_back(std::move(o));
    }
    doc["storeys"] = std::move(arr);
  }

  if (m.has_walls()) {
    ojson arr = ojson::array();
    for (const auto &w : m.walls) {
      ojson o;
      if (!w.guid.empty())
        o["guid"] = w.guid;
      o["storey"] = w.storey;
      o["axis"] = {{w.x0, w.y0}, {w.x1, w.y1}};
      o["z_base"] = w.z_base;
      o["height"] = w.height;
      o["thickness"] = w.thickness;
      o["thickness_ci"] = {w.thickness_ci_lo, w.thickness_ci_hi};
      o["lean_deg"] = w.lean_deg;
      o["is_external"] = w.is_external;
      o["assumed"] = w.assumed;
      o["seen_share"] = w.seen_share;
      o["face_points"] = {w.face_a_points, w.face_b_points};
      o["co_visibility"] = w.co_visibility;
      o["axis_family"] = w.axis_family;
      write_common(o, w, record_name("walls", &w - m.walls.data(), w.guid));
      arr.push_back(std::move(o));
    }
    doc["walls"] = std::move(arr);
  }

  if (!m.wall_joins.empty()) {
    ojson arr = ojson::array();
    for (const auto &jn : m.wall_joins)
      arr.push_back({{"wall_a", jn.wall_a},
                     {"wall_b", jn.wall_b},
                     {"kind", jn.kind},
                     {"at_a", jn.at_a},
                     {"at_b", jn.at_b}});
    doc["wall_joins"] = std::move(arr);
  }

  if (m.has_spaces()) {
    ojson arr = ojson::array();
    for (const auto &s : m.spaces) {
      ojson o;
      if (!s.guid.empty())
        o["guid"] = s.guid;
      o["storey"] = s.storey;
      if (!s.name.empty())
        o["name"] = s.name;
      o["kind"] = s.kind;
      ojson poly;
      poly["outer"] = ring_json(s.polygon.outer);
      if (!s.polygon.holes.empty()) {
        ojson holes = ojson::array();
        for (const auto &h : s.polygon.holes)
          holes.push_back(ring_json(h));
        poly["holes"] = std::move(holes);
      }
      o["polygon"] = std::move(poly);
      o["z_floor"] = s.z_floor;
      o["z_ceiling"] = s.z_ceiling;
      o["area"] = s.area;
      o["perimeter"] = s.perimeter;
      o["volume"] = s.volume;
      o["compactness"] = s.compactness;
      o["room_label"] = s.room_label;
      write_common(o, s, record_name("spaces", &s - m.spaces.data(), s.guid));
      arr.push_back(std::move(o));
    }
    doc["spaces"] = std::move(arr);
  }

  if (!m.space_boundaries.empty()) {
    ojson arr = ojson::array();
    for (const auto &b : m.space_boundaries)
      arr.push_back({{"space", b.space},
                     {"wall", b.wall},
                     {"side", b.side},
                     {"s_start", b.s_start},
                     {"s_end", b.s_end}});
    doc["space_boundaries"] = std::move(arr);
  }

  if (m.has_openings()) {
    ojson arr = ojson::array();
    for (const auto &op : m.openings) {
      ojson o;
      if (!op.guid.empty())
        o["guid"] = op.guid;
      o["wall"] = op.wall;
      o["kind"] = op.kind;
      o["s_start"] = op.s_start;
      o["s_end"] = op.s_end;
      o["z_sill"] = op.z_sill;
      o["z_head"] = op.z_head;
      o["state"] = op.state;
      o["is_external"] = op.is_external;
      if (!op.source_instance_guid.empty())
        o["source_instance_guid"] = op.source_instance_guid;
      write_common(o, op,
                   record_name("openings", &op - m.openings.data(), op.guid));
      arr.push_back(std::move(o));
    }
    doc["openings"] = std::move(arr);
  }

  if (!m.plane_classes.empty()) {
    ojson arr = ojson::array();
    for (const auto &p : m.plane_classes) {
      ojson o = {{"plane_label", p.plane_label},
                 {"orientation", p.orientation},
                 {"semantic_class", p.semantic_class},
                 {"class_confidence", p.class_confidence},
                 {"role", p.role},
                 {"storey", p.storey},
                 {"point_count", p.point_count},
                 {"area", p.area},
                 {"z_min", p.z_min},
                 {"z_max", p.z_max},
                 {"role_source", p.role_source}};
      if (!p.guid.empty())
        o["guid"] = p.guid;
      o["source"] = p.source;
      o["locked"] = p.locked;
      const auto where =
          record_name("plane_classes", &p - m.plane_classes.data(), p.guid);
      if (!p.metadata.empty())
        o["metadata"] = metadata_value(p.metadata, where);
      arr.push_back(std::move(o));
    }
    doc["plane_classes"] = std::move(arr);
  }

  if (!m.scene_relations.empty()) {
    ojson arr = ojson::array();
    for (const auto &r : m.scene_relations) {
      ojson o{{"from_guid", r.from_guid},
              {"to_guid", r.to_guid},
              {"relation_type", r.relation_type},
              {"weight", r.weight}};
      if (!r.metadata.empty())
        o["metadata"] = metadata_value(
            r.metadata,
            record_name("scene_relations", &r - m.scene_relations.data(), ""));
      arr.push_back(std::move(o));
    }
    doc["scene_relations"] = std::move(arr);
  }

  return doc;
}

} // namespace

std::string solid_model_to_json(const core::SolidModel &m, int indent) {
  return build_document(m).dump(indent);
}

std::string solid_model_to_yaml(const core::SolidModel &m) {
  return detail::json_to_yaml_text(build_document(m));
}

/// The validator shared by every front end: @p doc is already a
/// `nlohmann::json` value, however it got there (text, or a converted YAML
/// tree).
static core::SolidModel solid_model_from_json_value(const json &doc) {
  const Reader top(doc, "document");
  top.only({"$schema", "model_version", "units", "name", "description",
            "storeys", "walls", "wall_joins", "spaces", "space_boundaries",
            "openings", "plane_classes", "scene_relations"});

  const json &version = top.at("model_version");
  if (!version.is_number_integer())
    fail("document", "'model_version' must be an integer");
  if (version.dump() != std::to_string(kSolidModelJsonVersion))
    fail("document",
         fmt::format("model_version {} is not supported (this build reads {})",
                     version.dump(), kSolidModelJsonVersion));
  if (top.text("units", "m") != "m")
    fail("document", "'units' must be \"m\"");

  core::SolidModel m;
  m.name = top.text("name", "");
  m.description = top.text("description", "");

  std::unordered_set<std::string> guids;
  std::set<int> storey_idx;

  m.provided.storeys = for_each_element(doc, "storeys", [&](const Reader &r) {
    r.only({"guid", "idx", "name", "z_floor", "z_ceiling", "level_transform",
            "source", "locked", "confidence", "metadata"});
    core::StoreyRecord s;
    s.guid = r.text("guid", "");
    check_unique_guid(guids, s.guid, r.where());
    s.idx = r.integer("idx", static_cast<int>(m.storeys.size()));
    if (!storey_idx.insert(s.idx).second)
      fail(r.where(), fmt::format("duplicate storey idx {}", s.idx));
    s.name = r.text("name", "");
    s.z_floor = r.number("z_floor", s.z_floor);
    s.z_ceiling = r.number("z_ceiling", s.z_ceiling);
    if (r.has("level_transform")) {
      const auto t =
          r.numbers("level_transform", 16, "16 numbers (4x4 row-major)");
      std::copy(t.begin(), t.end(), s.level_transform.begin());
    }
    read_common(r, s.source, s.locked, s.confidence, s.metadata);
    m.storeys.push_back(std::move(s));
  });

  auto check_storey = [&](const Reader &r, int storey) {
    if (m.has_storeys() && storey_idx.count(storey) == 0)
      fail(r.where(),
           fmt::format("'storey' {} names no storey in 'storeys'", storey));
  };

  std::unordered_set<std::string> wall_guids;
  m.provided.walls = for_each_element(doc, "walls", [&](const Reader &r) {
    r.only({"guid", "storey", "axis", "z_base", "height", "thickness",
            "thickness_ci", "lean_deg", "is_external", "assumed", "seen_share",
            "face_points", "co_visibility", "axis_family", "source", "locked",
            "confidence", "metadata"});
    core::WallRecord w;
    w.guid = r.text("guid", "");
    check_unique_guid(guids, w.guid, r.where());
    if (!w.guid.empty())
      wall_guids.insert(w.guid);
    w.storey = r.integer("storey", 0);
    check_storey(r, w.storey);
    const json &axis = r.at("axis");
    if (!axis.is_array() || axis.size() != 2)
      fail(r.where(), "'axis' must be [[x0, y0], [x1, y1]]");
    const auto a = r.point(axis[0], "axis[0]");
    const auto b = r.point(axis[1], "axis[1]");
    if (std::hypot(b[0] - a[0], b[1] - a[1]) <= 0.0)
      fail(r.where(), "axis has zero length");
    w.x0 = a[0];
    w.y0 = a[1];
    w.x1 = b[0];
    w.y1 = b[1];
    w.z_base = r.number("z_base", w.z_base);
    w.height = r.number("height", w.height);
    w.thickness = r.number("thickness", w.thickness);
    if (r.has("thickness_ci")) {
      const auto ci = r.numbers("thickness_ci", 2, "[lo, hi]");
      w.thickness_ci_lo = ci[0];
      w.thickness_ci_hi = ci[1];
    }
    w.lean_deg = r.number("lean_deg", w.lean_deg);
    w.is_external = r.boolean("is_external", w.is_external);
    w.assumed = r.boolean("assumed", w.assumed);
    w.seen_share = r.number("seen_share", w.seen_share);
    if (r.has("face_points")) {
      const json &fp = r.at("face_points");
      if (!fp.is_array() || fp.size() != 2)
        fail(r.where(), "'face_points' must be [a, b] integers");
      w.face_a_points = r.to_int(fp[0], "'face_points'[0]");
      w.face_b_points = r.to_int(fp[1], "'face_points'[1]");
    }
    w.co_visibility = r.number("co_visibility", w.co_visibility);
    w.axis_family = r.integer("axis_family", w.axis_family);
    read_common(r, w.source, w.locked, w.confidence, w.metadata);
    m.walls.push_back(std::move(w));
  });

  auto check_wall = [&](const Reader &r, const std::string &key,
                        const std::string &guid) {
    if (wall_guids.count(guid) == 0)
      fail(r.where(),
           fmt::format("'{}' names wall '{}', which is not in 'walls'", key,
                       guid));
  };

  for_each_element(doc, "wall_joins", [&](const Reader &r) {
    r.only({"wall_a", "wall_b", "kind", "at_a", "at_b"});
    core::WallJoinRecord jn;
    jn.wall_a = r.text("wall_a", "");
    jn.wall_b = r.text("wall_b", "");
    check_wall(r, "wall_a", jn.wall_a);
    check_wall(r, "wall_b", jn.wall_b);
    jn.kind = r.text("kind", "corner");
    jn.at_a = r.text("at_a", "path");
    jn.at_b = r.text("at_b", "path");
    m.wall_joins.push_back(std::move(jn));
  });

  std::unordered_set<std::string> space_guids;
  m.provided.spaces = for_each_element(doc, "spaces", [&](const Reader &r) {
    r.only({"guid", "storey", "name", "kind", "polygon", "z_floor", "z_ceiling",
            "area", "perimeter", "volume", "compactness", "room_label",
            "source", "locked", "confidence", "metadata"});
    core::SpaceRecord s;
    s.guid = r.text("guid", "");
    check_unique_guid(guids, s.guid, r.where());
    if (!s.guid.empty())
      space_guids.insert(s.guid);
    s.storey = r.integer("storey", 0);
    check_storey(r, s.storey);
    s.name = r.text("name", "");
    s.kind = r.text("kind", s.kind);
    const Reader poly(r.at("polygon"), r.where() + ".polygon");
    poly.only({"outer", "holes"});
    s.polygon.outer = poly.ring(poly.at("outer"), "outer");
    if (poly.has("holes")) {
      const json &holes = poly.at("holes");
      if (!holes.is_array())
        fail(poly.where(), "'holes' must be an array of rings");
      for (std::size_t i = 0; i < holes.size(); ++i)
        s.polygon.holes.push_back(
            poly.ring(holes[i], fmt::format("holes[{}]", i)));
    }
    check_polygon(s.polygon, r.where());
    s.z_floor = r.number("z_floor", s.z_floor);
    s.z_ceiling = r.number("z_ceiling", s.z_ceiling);
    s.area = r.number("area", s.area);
    s.perimeter = r.number("perimeter", s.perimeter);
    s.volume = r.number("volume", s.volume);
    s.compactness = r.number("compactness", s.compactness);
    s.room_label = r.integer("room_label", s.room_label);
    read_common(r, s.source, s.locked, s.confidence, s.metadata);
    m.spaces.push_back(std::move(s));
  });

  for_each_element(doc, "space_boundaries", [&](const Reader &r) {
    r.only({"space", "wall", "side", "s_start", "s_end"});
    core::SpaceBoundaryRecord b;
    b.space = r.text("space", "");
    if (space_guids.count(b.space) == 0)
      fail(r.where(), fmt::format("'space' names space '{}', which is not in "
                                  "'spaces'",
                                  b.space));
    b.wall = r.text("wall", "");
    check_wall(r, "wall", b.wall);
    b.side = r.text("side", "a");
    if (b.side != "a" && b.side != "b")
      fail(r.where(), "'side' must be \"a\" or \"b\"");
    b.s_start = r.number("s_start", b.s_start);
    b.s_end = r.number("s_end", b.s_end);
    m.space_boundaries.push_back(std::move(b));
  });

  m.provided.openings = for_each_element(doc, "openings", [&](const Reader &r) {
    r.only({"guid", "wall", "kind", "s_start", "s_end", "z_sill", "z_head",
            "state", "is_external", "source_instance_guid", "source", "locked",
            "confidence", "metadata"});
    core::OpeningRecord o;
    o.guid = r.text("guid", "");
    check_unique_guid(guids, o.guid, r.where());
    o.wall = r.text("wall", "");
    check_wall(r, "wall", o.wall);
    o.kind = r.text("kind", o.kind);
    if (o.kind != "door" && o.kind != "window" && o.kind != "opening")
      fail(r.where(),
           fmt::format("'kind' must be door, window or opening, not '{}'",
                       o.kind));
    o.s_start = r.number("s_start", o.s_start);
    o.s_end = r.number("s_end", o.s_end);
    o.z_sill = r.number("z_sill", o.z_sill);
    o.z_head = r.number("z_head", o.z_head);
    if (!(o.s_end > o.s_start))
      fail(r.where(), fmt::format("s_end ({}) must exceed s_start ({})",
                                  o.s_end, o.s_start));
    if (!(o.z_head > o.z_sill))
      fail(r.where(), fmt::format("z_head ({}) must exceed z_sill ({})",
                                  o.z_head, o.z_sill));
    o.state = r.text("state", o.state);
    o.is_external = r.boolean("is_external", o.is_external);
    o.source_instance_guid = r.text("source_instance_guid", "");
    read_common(r, o.source, o.locked, o.confidence, o.metadata);
    m.openings.push_back(std::move(o));
  });

  for_each_element(doc, "plane_classes", [&](const Reader &r) {
    r.only({"plane_label", "orientation", "semantic_class", "class_confidence",
            "role", "storey", "point_count", "area", "z_min", "z_max",
            "role_source", "guid", "source", "locked", "metadata"});
    core::PlaneClassRecord p;
    p.plane_label = r.integer("plane_label", 0);
    p.orientation = r.text("orientation", "");
    p.semantic_class = r.integer("semantic_class", p.semantic_class);
    p.class_confidence = r.number("class_confidence", p.class_confidence);
    p.role = r.text("role", p.role);
    p.storey = r.integer("storey", p.storey);
    p.point_count = r.integer("point_count", p.point_count);
    p.area = r.number("area", p.area);
    p.z_min = r.number("z_min", p.z_min);
    p.z_max = r.number("z_max", p.z_max);
    p.role_source = r.text("role_source", p.role_source);
    p.guid = r.text("guid", "");
    double unused_confidence = -1.0; // plane classes carry class_confidence
    read_common(r, p.source, p.locked, unused_confidence, p.metadata);
    m.plane_classes.push_back(std::move(p));
  });

  for_each_element(doc, "scene_relations", [&](const Reader &r) {
    r.only({"from_guid", "to_guid", "relation_type", "weight", "metadata"});
    core::SceneRelationRecord rel;
    rel.from_guid = r.text("from_guid", "");
    rel.to_guid = r.text("to_guid", "");
    rel.relation_type = r.text("relation_type", "");
    rel.weight = r.number("weight", rel.weight);
    rel.metadata = r.metadata();
    m.scene_relations.push_back(std::move(rel));
  });

  return m;
}

core::SolidModel solid_model_from_json(std::string_view text) {
  json doc;
  try {
    doc = json::parse(text);
  } catch (const json::exception &e) { // parse_error, number overflow, ...
    fail("document", fmt::format("not valid JSON ({})", e.what()));
  }
  return solid_model_from_json_value(doc);
}

core::SolidModel solid_model_from_yaml(std::string_view text) {
  json doc;
  try {
    doc = detail::yaml_text_to_json(text);
  } catch (const std::invalid_argument &e) {
    fail("document", e.what());
  }
  return solid_model_from_json_value(doc);
}

core::SolidModel load_solid_model(const ProjectDB &db) {
  // Each table's WP fills its part here and sets the matching `provided`
  // flag once the project says something about that element kind, so a
  // project that has run the stage and found nothing is scored as a miss,
  // not as n/a.
  core::SolidModel m;
  // The storeys stage never writes zero rows (detect_storeys always yields
  // at least one storey, with the reason logged), so an empty table means
  // the stage has not run: n/a, not a miss.
  m.storeys = db.storeys();
  m.provided.storeys = !m.storeys.empty();
  // The classify stage writes one row per plane of the "planes" cloud, so
  // rows mean it ran; none means n/a (or a scan with no plane at all).
  m.plane_classes = db.plane_classes("planes");
  m.provided.plane_classes = !m.plane_classes.empty();
  // The walls stage writes walls and their joins (WP3). No rows means the
  // stage has not run (n/a); a run that found no wall is logged at warn by
  // the stage and still scores as n/a here, since the table cannot tell the
  // two apart.
  m.walls = db.walls();
  m.wall_joins = db.wall_joins();
  m.provided.walls = !m.walls.empty();
  return m;
}

/// `.yaml` / `.yml`, case-insensitive: every other extension (notably
/// `.json`, and no extension at all) reads/writes the JSON front end.
static bool has_yaml_extension(const std::filesystem::path &path) {
  std::string ext = path.extension().string();
  std::transform(ext.begin(), ext.end(), ext.begin(),
                 [](unsigned char c) { return std::tolower(c); });
  return ext == ".yaml" || ext == ".yml";
}

core::SolidModel solid_model_from_file(const std::filesystem::path &path) {
  std::ifstream in(path, std::ios::binary);
  if (!in)
    throw std::runtime_error(
        fmt::format("cannot read solid-model file '{}'", path.string()));
  std::stringstream ss;
  ss << in.rdbuf();
  if (in.bad())
    throw std::runtime_error(
        fmt::format("error reading solid-model file '{}'", path.string()));
  return has_yaml_extension(path) ? solid_model_from_yaml(ss.str())
                                  : solid_model_from_json(ss.str());
}

void solid_model_to_file(const core::SolidModel &model,
                         const std::filesystem::path &path, int indent) {
  const std::string text = has_yaml_extension(path)
                               ? solid_model_to_yaml(model)
                               : solid_model_to_json(model, indent) + "\n";
  if (path.has_parent_path())
    std::filesystem::create_directories(path.parent_path());
  std::ofstream out(path, std::ios::binary | std::ios::trunc);
  out << text;
  out.flush();
  if (!out)
    throw std::runtime_error(
        fmt::format("cannot write solid-model file '{}'", path.string()));
}

void mark_imported(core::SolidModel &model) {
  const auto mark = [](auto &records) {
    for (auto &r : records) {
      r.source = "imported";
      r.locked = true;
    }
  };
  mark(model.storeys);
  mark(model.walls);
  mark(model.spaces);
  mark(model.openings);
}

SolidModelStoreResult store_solid_model(ProjectDB & /*db*/,
                                        const core::SolidModel &model) {
  // The table-backed import plugs in here, kind by kind, as each table lands
  // (WP1 storeys, WP3 walls, WP4 spaces, WP5 openings): write the kind's rows
  // (replacing the project's imported rows of that kind), count them, and
  // stop listing the kind in `not_stored`. Until then nothing is written.
  SolidModelStoreResult r;
  if (model.has_storeys())
    r.not_stored.emplace_back("storeys");
  if (model.has_walls())
    r.not_stored.emplace_back("walls");
  if (model.has_spaces())
    r.not_stored.emplace_back("spaces");
  if (model.has_openings())
    r.not_stored.emplace_back("openings");
  return r;
}

} // namespace reusex::io
