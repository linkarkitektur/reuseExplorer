// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "io/pose_edits.hpp"

#include "core/pose_graph.hpp"
#include "yaml_json.hpp"

#include <fmt/format.h>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <initializer_list>
#include <set>
#include <sstream>
#include <stdexcept>

namespace reusex::io {
namespace {

using json = nlohmann::json;
using ojson = nlohmann::ordered_json;
using core::PoseEdit;
using core::PoseEditKind;

[[noreturn]] void fail(const std::string &where, const std::string &what) {
  throw std::runtime_error(fmt::format("pose edits: {}: {}", where, what));
}

/// One JSON object being read; `where` names the edit ("edit 'm-1'") and
/// `prefix` the field path inside it ("delta."), so every error names both.
class Reader {
    public:
  Reader(const json &j, std::string where, std::string prefix = {})
      : j_(j), where_(std::move(where)), prefix_(std::move(prefix)) {
    if (!j_.is_object())
      fail(where_, prefix_.empty()
                       ? std::string("expected an object")
                       : fmt::format("{}: expected an object",
                                     prefix_.substr(0, prefix_.size() - 1)));
  }

  [[noreturn]] void error(const char *key, const std::string &what) const {
    fail(where_, fmt::format("{}{}: {}", prefix_, key, what));
  }

  void only(std::initializer_list<const char *> allowed) const {
    for (const auto &[key, _] : j_.items()) {
      bool ok = false;
      for (const char *a : allowed)
        ok = ok || key == a;
      if (!ok)
        fail(where_, fmt::format("{}{}: unknown key", prefix_, key));
    }
  }

  bool has(const char *key) const { return j_.contains(key); }

  const json &at(const char *key) const {
    if (!has(key))
      error(key, "missing required field");
    return j_.at(key);
  }

  Reader object(const char *key) const {
    return Reader(at(key), where_, prefix_ + key + ".");
  }

  double number(const json &v, const char *key) const {
    if (!v.is_number())
      error(key, "must be a number");
    const double d = v.get<double>();
    if (!std::isfinite(d))
      error(key, "must be finite");
    return d;
  }

  double number(const char *key, double fallback) const {
    return has(key) ? number(j_.at(key), key) : fallback;
  }

  double positive(const char *key, double fallback) const {
    const double d = number(key, fallback);
    if (!(d > 0.0))
      error(key, "must be > 0");
    return d;
  }

  std::int64_t node(const json &v, const char *key) const {
    if (!v.is_number_integer())
      error(key, "must be an integer node id");
    return v.get<std::int64_t>();
  }

  std::int64_t node(const char *key) const { return node(at(key), key); }

  std::string text(const char *key, const std::string &fallback) const {
    if (!has(key))
      return fallback;
    if (!j_.at(key).is_string())
      error(key, "must be a string");
    return j_.at(key).get<std::string>();
  }

  std::string required_text(const char *key) const {
    at(key);
    const std::string s = text(key, {});
    if (s.empty())
      error(key, "must not be empty");
    return s;
  }

  template <size_t N>
  std::array<double, N> numbers(const char *key, const char *shape) const {
    const json &v = at(key);
    if (!v.is_array() || v.size() != N)
      error(key, fmt::format("must be {}", shape));
    std::array<double, N> out{};
    for (size_t i = 0; i < N; ++i) {
      if (!v[i].is_number() || !std::isfinite(v[i].get<double>()))
        error(key, fmt::format("must be {} (finite numbers)", shape));
      out[i] = v[i].get<double>();
    }
    return out;
  }

  /// `[first, last]` (inclusive) or `{list: [...]}`.
  core::NodeRange nodes(const char *key) const {
    const json &v = at(key);
    core::NodeRange r;
    if (v.is_array()) {
      if (v.size() != 2 || !v[0].is_number_integer() ||
          !v[1].is_number_integer())
        error(key, "must be a [first, last] pair of node ids or {list: [...]}");
      r.first = v[0].get<std::int64_t>();
      r.last = v[1].get<std::int64_t>();
      if (r.first > r.last)
        error(key, fmt::format("first ({}) > last ({})", r.first, r.last));
      return r;
    }
    if (v.is_object()) {
      const Reader o(v, where_, prefix_ + key + ".");
      o.only({"list"});
      const json &l = o.at("list");
      if (!l.is_array() || l.empty())
        o.error("list", "must be a non-empty array of node ids");
      std::set<std::int64_t> seen;
      for (const auto &e : l) {
        const auto id = o.node(e, "list");
        if (!seen.insert(id).second)
          o.error("list", fmt::format("node {} appears twice", id));
        r.list.push_back(id);
      }
      return r;
    }
    error(key, "must be a [first, last] pair of node ids or {list: [...]}");
  }

  std::array<double, 4> region(const char *key) const {
    const auto r = numbers<4>(key, "[x0, y0, x1, y1]");
    if (!(r[0] < r[2]) || !(r[1] < r[3]))
      error(key, "needs x0 < x1 and y0 < y1");
    return r;
  }

  std::uint8_t dofs(const char *key) const {
    if (!has(key))
      return core::Dof::all;
    const json &v = j_.at(key);
    if (!v.is_array() || v.empty())
      error(key, "must be a non-empty list of x, y, z, roll, pitch, yaw");
    std::uint8_t out = 0;
    for (const auto &e : v) {
      if (!e.is_string())
        error(key, "must be a list of x, y, z, roll, pitch, yaw");
      const std::string name = e.get<std::string>();
      const std::uint8_t bit = core::dof_from_string(name);
      if (bit == 0)
        error(key, fmt::format("unknown DoF '{}' (use x, y, z, roll, pitch, "
                               "yaw)",
                               name));
      if (out & bit)
        error(key, fmt::format("DoF '{}' appears twice", name));
      out |= bit;
    }
    return out;
  }

  /// A rigid transform object: {matrix: [16]} or {xyz, yaw_deg, pitch_deg,
  /// roll_deg[, pivot]}.
  core::RigidTransform rigid(const char *key, bool allow_pivot) const {
    const Reader o = object(key);
    core::RigidTransform t;
    if (o.has("matrix")) {
      o.only({"matrix"});
      const auto m = o.numbers<16>("matrix", "16 numbers (row-major 4x4)");
      if (!core::pose_graph::is_rigid_transform(m))
        o.error("matrix", "is not a rigid transform (rotation must be "
                          "orthonormal with det +1, last row 0 0 0 1)");
      t.matrix = m;
      return t;
    }
    if (allow_pivot)
      o.only({"xyz", "yaw_deg", "pitch_deg", "roll_deg", "pivot"});
    else
      o.only({"xyz", "yaw_deg", "pitch_deg", "roll_deg"});
    if (o.has("xyz"))
      t.xyz = o.numbers<3>("xyz", "[x, y, z]");
    t.yaw_deg = o.number("yaw_deg", 0.0);
    t.pitch_deg = o.number("pitch_deg", 0.0);
    t.roll_deg = o.number("roll_deg", 0.0);
    if (o.has("pivot"))
      t.pivot = o.numbers<3>("pivot", "[x, y, z]");
    return t;
  }

  core::PoseSpec pose(const char *key) const {
    core::PoseSpec p;
    if (!has(key))
      error(key, "missing required field (current, initial or a transform)");
    const json &v = j_.at(key);
    if (v.is_string()) {
      const std::string s = v.get<std::string>();
      if (s == "current")
        p.source = core::PoseSpec::Source::current;
      else if (s == "initial")
        p.source = core::PoseSpec::Source::initial;
      else
        error(key,
              fmt::format("'{}' is not current, initial or a transform", s));
      return p;
    }
    p.source = core::PoseSpec::Source::given;
    p.value = rigid(key, /*allow_pivot=*/false);
    return p;
  }

  void sigma(PoseEdit &e) const {
    if (!has("sigma"))
      return;
    const Reader s = object("sigma");
    s.only({"trans_m", "rot_deg"});
    e.sigma_trans_m = s.positive("trans_m", e.sigma_trans_m);
    e.sigma_rot_deg = s.positive("rot_deg", e.sigma_rot_deg);
  }

    private:
  const json &j_;
  std::string where_;
  std::string prefix_;
};

PoseEdit read_edit(const json &v, size_t index) {
  std::string where = fmt::format("edits[{}]", index);
  if (v.is_object() && v.contains("id") && v.at("id").is_string() &&
      !v.at("id").get<std::string>().empty())
    where = fmt::format("edit '{}'", v.at("id").get<std::string>());
  const Reader r(v, where);

  PoseEdit e;
  e.id = r.required_text("id");
  const std::string kind = r.required_text("kind");
  const auto k = core::pose_edit_kind_from_string(kind);
  if (!k)
    r.error("kind", fmt::format("unknown kind '{}' (anchor, move, relative, "
                                "same_surface, on_plan_wall, reject_edge)",
                                kind));
  e.kind = *k;
  e.note = r.text("note", {});

  switch (e.kind) {
  case PoseEditKind::anchor:
    r.only({"id", "kind", "note", "node", "pose", "dofs", "sigma"});
    e.nodes.first = e.nodes.last = r.node("node");
    e.pose = r.pose("pose");
    e.dofs = r.dofs("dofs");
    r.sigma(e);
    break;
  case PoseEditKind::move:
    r.only({"id", "kind", "note", "nodes", "delta", "dofs", "sigma"});
    e.nodes = r.nodes("nodes");
    e.delta = r.rigid("delta", /*allow_pivot=*/true);
    e.dofs = r.dofs("dofs");
    r.sigma(e);
    break;
  case PoseEditKind::relative:
    r.only({"id", "kind", "note", "a", "b", "pose", "dofs", "sigma"});
    e.nodes.first = e.nodes.last = r.node("a");
    e.nodes_b.first = e.nodes_b.last = r.node("b");
    if (e.nodes.first == e.nodes_b.first)
      r.error("b", "must differ from a");
    e.pose = r.pose("pose");
    e.dofs = r.dofs("dofs");
    r.sigma(e);
    break;
  case PoseEditKind::same_surface: {
    r.only({"id", "kind", "note", "a", "b", "sigma_m"});
    const Reader a = r.object("a");
    a.only({"nodes", "region"});
    e.nodes = a.nodes("nodes");
    e.region = a.region("region");
    const Reader b = r.object("b");
    b.only({"nodes", "region"});
    e.nodes_b = b.nodes("nodes");
    e.region_b = b.region("region");
    e.sigma_m = r.positive("sigma_m", e.sigma_m);
    break;
  }
  case PoseEditKind::on_plan_wall:
    r.only(
        {"id", "kind", "note", "plan", "wall", "nodes", "region", "sigma_m"});
    e.plan_path = r.required_text("plan");
    e.wall_guid = r.required_text("wall");
    e.nodes = r.nodes("nodes");
    if (r.has("region"))
      e.region = r.region("region");
    e.sigma_m = r.positive("sigma_m", e.sigma_m);
    break;
  case PoseEditKind::reject_edge: {
    r.only({"id", "kind", "note", "a", "b", "edge_type"});
    e.nodes.first = e.nodes.last = r.node("a");
    e.nodes_b.first = e.nodes_b.last = r.node("b");
    if (e.nodes.first == e.nodes_b.first)
      r.error("b", "must differ from a");
    e.edge_type = r.required_text("edge_type");
    namespace pg = core::pose_graph;
    if (e.edge_type != pg::kTypeOdometry &&
        e.edge_type != pg::kTypeLoopClosure && e.edge_type != pg::kTypePanorama)
      r.error("edge_type",
              fmt::format("'{}' is not odometry, loop_closure or panorama",
                          e.edge_type));
    break;
  }
  }
  return e;
}

core::PoseEdits from_value(const json &doc) {
  const Reader top(doc, "document");
  top.only({"$schema", "schema", "project_guid", "frame", "edits"});
  core::PoseEdits out;
  const std::string schema = top.required_text("schema");
  if (schema != kPoseEditsSchema)
    top.error("schema",
              fmt::format("'{}' is not {}", schema, kPoseEditsSchema));
  out.project_guid = top.text("project_guid", {});
  out.frame = top.text("frame", "world");
  if (out.frame != "world" &&
      (out.frame.rfind("storey:", 0) != 0 || out.frame.size() <= 7))
    top.error("frame",
              fmt::format("'{}' is not world or storey:<guid>", out.frame));
  const json &edits = top.at("edits");
  if (!edits.is_array())
    top.error("edits", "must be an array");
  std::set<std::string> ids;
  for (size_t i = 0; i < edits.size(); ++i) {
    PoseEdit e = read_edit(edits[i], i);
    if (!ids.insert(e.id).second)
      fail(fmt::format("edit '{}'", e.id), "id: duplicate id");
    out.edits.push_back(std::move(e));
  }
  return out;
}

ojson rigid_json(const core::RigidTransform &t) {
  ojson o;
  if (t.matrix) {
    o["matrix"] = *t.matrix;
    return o;
  }
  o["xyz"] = t.xyz;
  o["yaw_deg"] = t.yaw_deg;
  o["pitch_deg"] = t.pitch_deg;
  o["roll_deg"] = t.roll_deg;
  if (t.pivot)
    o["pivot"] = *t.pivot;
  return o;
}

ojson pose_json(const core::PoseSpec &p) {
  switch (p.source) {
  case core::PoseSpec::Source::current:
    return "current";
  case core::PoseSpec::Source::initial:
    return "initial";
  case core::PoseSpec::Source::given:
    break;
  }
  return rigid_json(p.value);
}

ojson nodes_json(const core::NodeRange &r) {
  if (r.is_list()) {
    ojson o;
    o["list"] = r.list;
    return o;
  }
  return ojson::array({r.first, r.last});
}

ojson sigma_json(const PoseEdit &e) {
  ojson o;
  o["trans_m"] = e.sigma_trans_m;
  o["rot_deg"] = e.sigma_rot_deg;
  return o;
}

ojson region_json(const std::optional<std::array<double, 4>> &r) {
  return r ? ojson(*r) : ojson();
}

ojson to_value(const core::PoseEdits &edits) {
  ojson doc;
  doc["schema"] = std::string(kPoseEditsSchema);
  if (!edits.project_guid.empty())
    doc["project_guid"] = edits.project_guid;
  doc["frame"] = edits.frame;
  ojson arr = ojson::array();
  for (const auto &e : edits.edits) {
    ojson o;
    o["id"] = e.id;
    o["kind"] = std::string(core::to_string(e.kind));
    switch (e.kind) {
    case PoseEditKind::anchor:
      o["node"] = e.nodes.first;
      o["pose"] = pose_json(e.pose);
      o["dofs"] = core::dof_names(e.dofs);
      o["sigma"] = sigma_json(e);
      break;
    case PoseEditKind::move:
      o["nodes"] = nodes_json(e.nodes);
      o["delta"] = rigid_json(e.delta);
      o["dofs"] = core::dof_names(e.dofs);
      o["sigma"] = sigma_json(e);
      break;
    case PoseEditKind::relative:
      o["a"] = e.nodes.first;
      o["b"] = e.nodes_b.first;
      o["pose"] = pose_json(e.pose);
      o["dofs"] = core::dof_names(e.dofs);
      o["sigma"] = sigma_json(e);
      break;
    case PoseEditKind::same_surface:
      o["a"] = {{"nodes", nodes_json(e.nodes)},
                {"region", region_json(e.region)}};
      o["b"] = {{"nodes", nodes_json(e.nodes_b)},
                {"region", region_json(e.region_b)}};
      o["sigma_m"] = e.sigma_m;
      break;
    case PoseEditKind::on_plan_wall:
      o["plan"] = e.plan_path;
      o["wall"] = e.wall_guid;
      o["nodes"] = nodes_json(e.nodes);
      if (e.region)
        o["region"] = *e.region;
      o["sigma_m"] = e.sigma_m;
      break;
    case PoseEditKind::reject_edge:
      o["a"] = e.nodes.first;
      o["b"] = e.nodes_b.first;
      o["edge_type"] = e.edge_type;
      break;
    }
    if (!e.note.empty())
      o["note"] = e.note;
    arr.push_back(std::move(o));
  }
  doc["edits"] = std::move(arr);
  return doc;
}

bool has_yaml_extension(const std::filesystem::path &path) {
  std::string ext = path.extension().string();
  std::transform(ext.begin(), ext.end(), ext.begin(),
                 [](unsigned char c) { return std::tolower(c); });
  return ext == ".yaml" || ext == ".yml";
}

} // namespace

core::PoseEdits pose_edits_from_json(std::string_view text) {
  json doc;
  try {
    doc = json::parse(text);
  } catch (const json::exception &e) {
    fail("document", fmt::format("not valid JSON ({})", e.what()));
  }
  return from_value(doc);
}

core::PoseEdits pose_edits_from_yaml(std::string_view text) {
  json doc;
  try {
    doc = detail::yaml_text_to_json(text);
  } catch (const std::invalid_argument &e) {
    fail("document", e.what());
  }
  return from_value(doc);
}

std::string pose_edits_to_json(const core::PoseEdits &edits, int indent) {
  return to_value(edits).dump(indent);
}

std::string pose_edits_to_yaml(const core::PoseEdits &edits) {
  return detail::json_to_yaml_text(to_value(edits));
}

void check_pose_edits_project(const core::PoseEdits &edits,
                              std::string_view project_guid) {
  if (edits.project_guid.empty() || project_guid.empty())
    return;
  if (edits.project_guid != project_guid)
    fail("project_guid",
         fmt::format("the edits were written for project '{}', but this "
                     "project is '{}'",
                     edits.project_guid, project_guid));
}

core::PoseEdits read_pose_edits(const std::filesystem::path &path,
                                std::string_view expected_project_guid) {
  std::ifstream in(path, std::ios::binary);
  if (!in)
    throw std::runtime_error(
        fmt::format("cannot read pose-edits file '{}'", path.string()));
  std::stringstream ss;
  ss << in.rdbuf();
  if (in.bad())
    throw std::runtime_error(
        fmt::format("error reading pose-edits file '{}'", path.string()));
  core::PoseEdits edits;
  try {
    edits = has_yaml_extension(path) ? pose_edits_from_yaml(ss.str())
                                     : pose_edits_from_json(ss.str());
  } catch (const std::runtime_error &e) {
    throw std::runtime_error(fmt::format("{}: {}", path.string(), e.what()));
  }
  check_pose_edits_project(edits, expected_project_guid);
  return edits;
}

void write_pose_edits(const core::PoseEdits &edits,
                      const std::filesystem::path &path) {
  const std::string text = has_yaml_extension(path)
                               ? pose_edits_to_yaml(edits)
                               : pose_edits_to_json(edits) + "\n";
  if (path.has_parent_path())
    std::filesystem::create_directories(path.parent_path());
  std::ofstream out(path, std::ios::binary | std::ios::trunc);
  out << text;
  out.flush();
  if (!out)
    throw std::runtime_error(
        fmt::format("cannot write pose-edits file '{}'", path.string()));
}

} // namespace reusex::io
