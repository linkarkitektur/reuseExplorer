// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "reusex/core/survey_measure.hpp"

#include "reusex/core/buildups.hpp"
#include "reusex/core/guid.hpp"
#include "reusex/core/logging.hpp"
#include "reusex/core/survey.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <stdexcept>

namespace reusex::core {

namespace {
using json = nlohmann::json;

/// One element of a solids index (the `solids` stage's mesh parameters).
struct SolidEntry {
  std::string guid;       ///< source guid: wall guid, or the slab's storey
  std::string predefined; ///< wall STANDARD; slab FLOOR | ROOF
  int part = 0;
  int storey = 0;
  double volume = 0.0;
  double depth = 0.0; ///< extrusion depth: a wall's height, a slab's thickness
  bool closed = false;
};

std::vector<SolidEntry> solids_index(const ProjectDB &db,
                                     const std::string &mesh) {
  std::vector<SolidEntry> out;
  if (!db.has_mesh(mesh))
    return out;
  const auto meta = db.mesh_metadata(mesh);
  json j;
  try {
    j = json::parse(meta.parameters);
  } catch (const json::parse_error &e) {
    reusex::warn("measure_survey: mesh '{}' has no readable solids index ({}); "
                 "its elements are not measured",
                 mesh, e.what());
    return out;
  }
  if (!j.contains("elements") || !j["elements"].is_array()) {
    reusex::warn("measure_survey: mesh '{}' carries no solids index; re-run "
                 "`rux create solids`",
                 mesh);
    return out;
  }
  for (const auto &e : j["elements"]) {
    SolidEntry s;
    s.guid = e.value("guid", "");
    s.predefined = e.value("predefined", "");
    s.part = e.value("part", 0);
    s.storey = e.value("storey", 0);
    s.volume = e.value("volume", 0.0);
    s.closed = e.value("closed", false);
    if (e.contains("profile") && e["profile"].is_object())
      s.depth = e["profile"].value("depth", 0.0);
    if (!s.guid.empty())
      out.push_back(std::move(s));
  }
  return out;
}

double wall_length(const WallRecord &w) {
  return std::hypot(w.x1 - w.x0, w.y1 - w.y0);
}

/// Net face area of @p w: length × height minus its openings, each clipped
/// to the wall's length and height.
double wall_net_area(const WallRecord &w, double height,
                     const std::vector<const OpeningRecord *> &openings) {
  const double len = wall_length(w);
  double holes = 0.0;
  for (const auto *o : openings) {
    const double s0 = std::clamp(std::min(o->s_start, o->s_end), 0.0, len);
    const double s1 = std::clamp(std::max(o->s_start, o->s_end), 0.0, len);
    const double h = std::clamp(o->z_head - o->z_sill, 0.0, height);
    holes += (s1 - s0) * h;
  }
  return std::max(0.0, len * height - holes);
}

double opening_area(const OpeningRecord &o, const WallRecord *host) {
  double s0 = std::min(o.s_start, o.s_end), s1 = std::max(o.s_start, o.s_end);
  if (host) {
    const double len = wall_length(*host);
    s0 = std::clamp(s0, 0.0, len);
    s1 = std::clamp(s1, 0.0, len);
  }
  return std::max(0.0, s1 - s0) * std::max(0.0, o.z_head - o.z_sill);
}

/// One measured element: what a part grounded in it gets.
struct Measured {
  std::string category; ///< element parts: their type's category
  double quantity = 0.0;
  std::string unit, source;
  int storey = -1;
};

bool entered(const std::vector<ProjectDB::ValueProvenanceRecord> &rows,
             std::string_view field) {
  return std::any_of(rows.begin(), rows.end(), [&](const auto &r) {
    return r.field == field && r.kind == "entered";
  });
}

double median(std::vector<double> v) {
  std::sort(v.begin(), v.end());
  const std::size_t n = v.size();
  return n % 2 ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
}
} // namespace

ElementTypeSeed element_type_seed(std::string_view category) {
  if (category == "ydervaeg")
    return {"Ydervæg", "21", std::string(kUnitArea)};
  if (category == "indervaeg")
    return {"Indervæg", "22", std::string(kUnitArea)};
  if (category == "daek")
    return {"Etagedæk", "23", std::string(kUnitVolume)};
  if (category == "tag")
    return {"Tag", "27", std::string(kUnitArea)};
  throw std::invalid_argument("element_type_seed: unknown category '" +
                              std::string(category) + "'");
}

SurveyMeasureReport measure_survey(ProjectDB &db,
                                   const SurveyMeasureOptions &opts) {
  if (!db.has_survey_mass_columns())
    throw std::runtime_error(
        "measure_survey: the project is below schema v37 (open it read-write "
        "once to migrate)");
  SurveyMeasureReport report;

  // --- The model ---------------------------------------------------------
  const auto walls = db.has_walls() ? db.walls() : std::vector<WallRecord>{};
  const auto openings =
      db.has_openings() ? db.openings() : std::vector<OpeningRecord>{};
  const auto storeys =
      db.has_storeys() ? db.storeys() : std::vector<StoreyRecord>{};
  std::map<std::string, const WallRecord *> wall_by_guid;
  for (const auto &w : walls)
    wall_by_guid.emplace(w.guid, &w);
  std::map<std::string, std::vector<const OpeningRecord *>> openings_of;
  std::map<std::string, const OpeningRecord *> opening_by_instance;
  for (const auto &o : openings) {
    openings_of[o.wall].push_back(&o);
    if (!o.source_instance_guid.empty())
      opening_by_instance.try_emplace(o.source_instance_guid, &o);
  }
  std::map<int, std::string> storey_name;
  for (const auto &s : storeys)
    storey_name[s.idx] = s.name;

  std::map<std::string, double> wall_solid_height;
  for (const auto &s : solids_index(db, opts.walls_mesh))
    if (s.depth > 0.0)
      wall_solid_height[s.guid] = s.depth;

  // Every element that becomes (or is) an element-backed part, in model
  // order: walls, then slab pieces.
  std::vector<std::pair<std::string, Measured>> elements;
  std::map<std::string, Measured> element_by_guid;
  for (const auto &w : walls) {
    const auto h = wall_solid_height.find(w.guid);
    const double height = h != wall_solid_height.end() ? h->second : w.height;
    if (wall_length(w) <= 0.0 || height <= 0.0)
      continue;
    const auto of = openings_of.find(w.guid);
    Measured m{w.is_external ? "ydervaeg" : "indervaeg",
               wall_net_area(w, height,
                             of != openings_of.end()
                                 ? of->second
                                 : std::vector<const OpeningRecord *>{}),
               std::string(kUnitArea), "wall_area", w.storey};
    if (!w.assumed && w.thickness > 0.0)
      ++report.walls_with_thickness;
    elements.emplace_back(w.guid, m);
  }
  for (const auto &s : solids_index(db, opts.slabs_mesh)) {
    if (s.volume <= 0.0)
      continue;
    const bool roof = s.predefined == "ROOF";
    // A slab piece has no guid of its own: derive one from its storey's
    // guid, kind and piece number (stable for a re-run of the same model).
    const std::string guid = deterministic_guid(
        s.guid, "slab:" + s.predefined + ":" + std::to_string(s.part));
    Measured m;
    m.storey = s.storey;
    if (roof) {
      if (s.depth <= 0.0)
        continue;
      m = {"tag", s.volume / s.depth, std::string(kUnitArea), "slab_area",
           s.storey};
    } else {
      m = {"daek", s.volume, std::string(kUnitVolume), "solid_volume",
           s.storey};
    }
    elements.emplace_back(guid, m);
  }
  for (const auto &[g, m] : elements)
    element_by_guid.emplace(g, m);
  if (walls.empty())
    reusex::warn("measure_survey: the project has no walls (run `rux create "
                 "walls`); no wall quantities");
  if (!db.has_mesh(opts.slabs_mesh))
    reusex::warn("measure_survey: no '{}' mesh (run `rux create solids`); no "
                 "slab or roof quantities",
                 opts.slabs_mesh);

  ProjectDB::Transaction tx(db);

  // --- Element types and parts --------------------------------------------
  std::map<std::string, int64_t> type_of_category;
  std::map<int64_t, ProjectDB::SurveyTypeRecord> types;
  for (const auto &t : db.survey_types()) {
    if (t.element_category)
      type_of_category.try_emplace(*t.element_category, t.id);
    types.emplace(t.id, t);
  }
  const auto type_for = [&](const std::string &cat) {
    if (const auto it = type_of_category.find(cat);
        it != type_of_category.end())
      return it->second;
    const auto seed = element_type_seed(cat);
    ProjectDB::SurveyTypeRecord t;
    t.name = seed.name;
    t.bim7aa_code = seed.bim7aa_code;
    t.unit = seed.unit;
    t.semantic_class = kManualSemanticClass; // never matched by sync_survey
    t.element_category = cat;
    const auto added = db.add_survey_type(t);
    ++report.types_created;
    types.emplace(added.id, added);
    type_of_category.emplace(cat, added.id);
    return added.id;
  };

  if (opts.seed_elements) {
    int next = db.max_survey_part_number();
    for (const auto &[guid, m] : elements) {
      if (db.survey_part_for_element(guid) || db.is_instance_dismissed(guid))
        continue;
      ProjectDB::SurveyPartRecord p;
      p.code = part_code(++next);
      p.type_id = type_for(m.category);
      p.quantity = m.quantity;
      if (const auto n = storey_name.find(m.storey); n != storey_name.end())
        p.room_name = n->second;
      p.element_guid = guid;
      p.geom_quantity = m.quantity;
      p.geom_unit = m.unit;
      p.geom_source = m.source;
      db.add_survey_part(p);
      db.set_value_provenance({"part", p.code, "quantity", "measured",
                               "measure:" + m.source, std::nullopt, ""});
      ++report.parts_created;
    }
  }

  // --- Every part ---------------------------------------------------------
  std::set<std::string> used_elements;
  for (const auto &p : db.survey_parts())
    if (p.element_guid)
      used_elements.insert(*p.element_guid);

  for (const auto &p : db.survey_parts()) {
    ++report.parts_seen;
    std::optional<std::string> element = p.element_guid;
    std::optional<Measured> m;
    if (element) {
      if (const auto it = element_by_guid.find(*element);
          it != element_by_guid.end()) {
        m = it->second;
      } else if (p.instance_guid) {
        // An opening-linked part: re-measure its opening below.
      } else {
        ++report.elements_missing;
        if (p.geom_quantity)
          db.set_survey_part_geometry(p.code, element, std::nullopt, "", "");
        continue;
      }
    }
    if (!m && p.instance_guid) {
      if (const auto o = opening_by_instance.find(*p.instance_guid);
          o != opening_by_instance.end()) {
        const auto &op = *o->second;
        // One part per opening: a second part from the same instance
        // stays a counted item.
        if (element == op.guid || !used_elements.contains(op.guid)) {
          const auto host = wall_by_guid.find(op.wall);
          m = Measured{"",
                       opening_area(op, host != wall_by_guid.end()
                                            ? host->second
                                            : nullptr),
                       std::string(kUnitArea), "opening_area", -1};
          element = op.guid;
          used_elements.insert(op.guid);
        }
      }
      if (!m) {
        if (element && !element_by_guid.contains(*element)) {
          // Its opening is gone: back to a counted item.
          ++report.elements_missing;
          element.reset();
        }
        m = Measured{"", 1.0, std::string(kUnitCount), "count", -1};
      }
    }
    if (!m) {
      ++report.parts_without_geometry; // a manual part
      continue;
    }
    if (m->source == "opening_area")
      ++report.opening_parts;
    else if (m->source == "wall_area")
      ++report.wall_parts;
    else if (m->source == "solid_volume")
      ++report.slab_parts;
    else if (m->source == "slab_area")
      ++report.roof_parts;
    else if (m->source == "count")
      ++report.count_parts;

    const bool changed = p.element_guid != element || !p.geom_quantity ||
                         std::abs(*p.geom_quantity - m->quantity) > 1e-9 ||
                         p.geom_unit != m->unit || p.geom_source != m->source;
    if (changed)
      db.set_survey_part_geometry(p.code, element, m->quantity, m->unit,
                                  m->source);
    // Seed the quantity when the units agree and no user entered it.
    const auto t = types.find(p.type_id);
    if (t != types.end() && t->second.unit == m->unit && m->source != "count" &&
        !entered(db.value_provenance("part", p.code), "quantity")) {
      if (std::abs(p.quantity - m->quantity) > 1e-9) {
        ProjectDB::SurveyPartPatch q;
        q.quantity = m->quantity;
        db.update_survey_part(p.code, q);
        ++report.quantities_seeded;
      }
      db.set_value_provenance({"part", p.code, "quantity", "measured",
                               "measure:" + m->source, std::nullopt, ""});
    }
  }

  // --- Build-up suggestions -----------------------------------------------
  if (opts.suggest_buildups) {
    const auto library = buildup_library(db);
    std::map<int64_t, std::vector<double>> thicknesses;
    for (const auto &p : db.survey_parts()) {
      if (p.dismissed || !p.element_guid)
        continue;
      const auto w = wall_by_guid.find(*p.element_guid);
      if (w != wall_by_guid.end() && !w->second->assumed &&
          w->second->thickness > 0.0)
        thicknesses[p.type_id].push_back(w->second->thickness);
    }
    for (const auto &[id, t] : types) {
      if (!t.element_category || t.buildup_id ||
          entered(db.value_provenance("type", std::to_string(id)),
                  "buildup_id"))
        continue;
      const auto cat = buildup_category_from_string(*t.element_category);
      std::vector<const Buildup *> candidates;
      for (const auto &b : library)
        if (cat && b.category == *cat)
          candidates.push_back(&b);
      if (candidates.empty())
        continue;
      // An element part is a solid of the model (a slab, a wall), so its
      // tonnes are its structure: prefer a build-up whose main layer is
      // structural (t x rho), not a membrane per m² (tagpap, kg_m2).
      const auto structural = std::find_if(
          candidates.begin(), candidates.end(), [](const Buildup *b) {
            return !b->main.kg_m2 && b->main.t && b->main.rho;
          });
      const Buildup *pick =
          structural != candidates.end() ? *structural : candidates.front();
      std::string why = "measure:category";
      const auto th = thicknesses.find(id);
      if (th != thicknesses.end() && !th->second.empty()) {
        const double med = median(th->second);
        pick =
            *std::min_element(candidates.begin(), candidates.end(),
                              [&](const Buildup *a, const Buildup *b) {
                                return std::abs(layered_thickness(*a) - med) <
                                       std::abs(layered_thickness(*b) - med);
                              });
        why = fmt::format("measure:thickness median {:.3f} m over {} wall(s)",
                          med, th->second.size());
      }
      ProjectDB::SurveyTypePatch patch;
      patch.buildup_id = pick->id;
      const std::string sid = std::to_string(id);
      if (t.eak_code.empty() && !pick->main.eak.empty()) {
        patch.eak_code = pick->main.eak;
        db.set_value_provenance({"type", sid, "eak_code", "suggested",
                                 "buildup:" + pick->id, std::nullopt, ""});
      }
      db.update_survey_type(id, patch);
      db.set_value_provenance(
          {"type", sid, "buildup_id", "suggested", why, std::nullopt, ""});
      ++report.buildups_suggested;
    }
  }
  tx.commit();

  reusex::info("measure_survey: {} part(s): {} wall, {} slab, {} roof, {} "
               "opening, {} counted; {} part(s) and {} type(s) created, {} "
               "quantit(ies) seeded, {} build-up(s) suggested",
               report.parts_seen, report.wall_parts, report.slab_parts,
               report.roof_parts, report.opening_parts, report.count_parts,
               report.parts_created, report.types_created,
               report.quantities_seeded, report.buildups_suggested);
  if (report.elements_missing > 0)
    reusex::warn("measure_survey: {} part(s) name a wall, slab or opening "
                 "the model no longer holds; their measured quantity was "
                 "cleared (the parts and their quantities are kept)",
                 report.elements_missing);
  if (report.walls_with_thickness == 0 && !walls.empty())
    reusex::warn("measure_survey: none of the {} wall(s) has a measured "
                 "thickness; wall build-ups are suggested by category only",
                 walls.size());
  return report;
}

} // namespace reusex::core
