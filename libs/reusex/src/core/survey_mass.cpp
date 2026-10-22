// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "reusex/core/survey_mass.hpp"

#include "reusex/core/logging.hpp"

#include <fmt/format.h>

#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>

namespace reusex::core {

namespace {

/// A measured thickness more than this far off the build-up's layered
/// thickness (relative) is a warning.
constexpr double kThicknessTolerance = 0.25;

struct MassContext {
  std::vector<Buildup> library;
  std::map<std::string, WallRecord> walls; ///< by guid
};

MassContext load_context(const ProjectDB &db) {
  MassContext c{buildup_library(db), {}};
  if (db.has_walls())
    for (auto &w : db.walls())
      c.walls.emplace(w.guid, std::move(w));
  return c;
}

PartMass part_mass(const MassContext &c, const ProjectDB::SurveyTypeRecord &t,
                   const Buildup *b, const ProjectDB::SurveyPartRecord &p,
                   std::set<std::string> &warnings) {
  PartMass m{p.code, std::nullopt, "", p.dismissed};
  if (!b)
    return m;
  const BuildupLayer &main = b->main;
  const double q = p.quantity;
  if (t.unit == kUnitArea) {
    if (main.kg_m2) {
      m.mass_t = q * *main.kg_m2 / 1000.0;
      m.method = "areal × kg/m²";
      return m;
    }
    const WallRecord *wall = nullptr;
    if (p.element_guid)
      if (const auto w = c.walls.find(*p.element_guid); w != c.walls.end())
        wall = &w->second;
    const bool measured = wall && !wall->assumed && wall->thickness > 0.0;
    if (measured && b->others.empty() && main.t && *main.t < 1.0) {
      m.mass_t = q * wall->thickness * main.rho.value_or(0.0) / 1000.0;
      m.method = "areal × målt tykkelse × densitet";
      return m;
    }
    if (measured) {
      const double layered = layered_thickness(*b);
      if (layered > 0.0 &&
          std::abs(wall->thickness - layered) / layered > kThicknessTolerance)
        warnings.insert(fmt::format(
            "målt vægtykkelse afviger fra opbygningen '{}' ({:.0f} mm)",
            b->name, layered * 1000.0));
    }
    m.mass_t = layer_tonnes(main, q, kUnitArea);
    m.method = "areal × tykkelse × densitet";
    return m;
  }
  if (t.unit == kUnitVolume) {
    if (main.kg_m2) {
      warnings.insert("opbygningen '" + b->name +
                      "' er pr. m² men typen er i m³");
      return m;
    }
    const double rho = main.rho.value_or(0.0);
    if (!main.t || *main.t >= 1.0) {
      m.mass_t = q * rho / 1000.0;
      m.method = "volumen × densitet";
    } else {
      const double layered = layered_thickness(*b);
      m.mass_t = q * rho * (layered > 0.0 ? *main.t / layered : 1.0) / 1000.0;
      m.method = "volumen × lagandel × densitet";
    }
    return m;
  }
  if (t.unit != kUnitCount)
    warnings.insert("enheden '" + t.unit + "' kan ikke regnes om til tons");
  return m;
}

TypeMass
mass_of(const MassContext &c, const ProjectDB::SurveyTypeRecord &t,
        const std::vector<const ProjectDB::SurveyPartRecord *> &parts) {
  TypeMass r;
  r.type_id = t.id;
  r.override_t = t.mass_t;
  r.buildup_id = t.buildup_id;
  std::set<std::string> warnings;
  std::optional<Buildup> b;
  if (t.buildup_id) {
    b = find_buildup(c.library, *t.buildup_id);
    if (!b)
      warnings.insert("ukendt opbygning '" + *t.buildup_id + "'");
  }
  std::set<std::string> methods;
  double sum = 0.0;
  bool any = false;
  for (const auto *p : parts) {
    auto pm = part_mass(c, t, b ? &*b : nullptr, *p, warnings);
    if (!p->dismissed) {
      ++r.parts_counted;
      if (pm.mass_t) {
        sum += *pm.mass_t;
        any = true;
        methods.insert(pm.method);
      }
    }
    r.parts.push_back(std::move(pm));
  }
  if (any) {
    r.calc_t = sum;
    r.method = methods.size() == 1 ? *methods.begin() : "blandet";
  }
  if (r.override_t) {
    r.effective_t = r.override_t;
    r.source = "override";
  } else if (r.calc_t) {
    r.effective_t = r.calc_t;
    r.source = "calculated";
  }
  r.warnings.assign(warnings.begin(), warnings.end());
  return r;
}

void type_fields_of(const ProjectDB::SurveyTypePatch &p,
                    std::vector<std::string> &set_fields,
                    std::vector<std::string> &cleared) {
  const auto add = [&](bool engaged, const char *f) {
    if (engaged)
      set_fields.emplace_back(f);
  };
  add(p.name.has_value(), "name");
  add(p.eak_code.has_value(), "eak_code");
  add(p.bim7aa_code.has_value(), "bim7aa_code");
  add(p.unit.has_value(), "unit");
  add(p.note.has_value(), "note");
  add(p.treatment.has_value(), "treatment");
  add(p.review_status.has_value(), "review_status");
  add(p.starred.has_value(), "starred");
  add(p.confidence && p.confidence->has_value(), "confidence");
  if (p.mass_t) {
    if (p.mass_t->has_value())
      set_fields.emplace_back("mass_t");
    else
      cleared.emplace_back("mass_t");
  }
  add(p.buildup_id.has_value(), "buildup_id"); // cleared counts as entered
}
} // namespace

std::map<int64_t, TypeMass> survey_masses(const ProjectDB &db) {
  const auto c = load_context(db);
  const auto parts = db.survey_parts();
  std::map<int64_t, std::vector<const ProjectDB::SurveyPartRecord *>> by_type;
  for (const auto &p : parts)
    by_type[p.type_id].push_back(&p);
  std::map<int64_t, TypeMass> out;
  for (const auto &t : db.survey_types())
    out.emplace(t.id, mass_of(c, t, by_type[t.id]));
  return out;
}

TypeMass type_mass(const ProjectDB &db, int64_t type_id) {
  const auto t = db.survey_type(type_id);
  if (!t)
    throw std::out_of_range("no survey type " + std::to_string(type_id));
  const auto c = load_context(db);
  const auto parts = db.survey_parts();
  std::vector<const ProjectDB::SurveyPartRecord *> mine;
  for (const auto &p : parts)
    if (p.type_id == type_id)
      mine.push_back(&p);
  return mass_of(c, *t, mine);
}

std::vector<ProjectDB::ValueProvenanceRecord>
type_provenance(const ProjectDB &db, const TypeMass &mass) {
  const std::string id = std::to_string(mass.type_id);
  auto rows = db.value_provenance("type", id);
  const auto stored = std::find_if(rows.begin(), rows.end(), [](const auto &r) {
    return r.field == "mass_t";
  });
  std::optional<ProjectDB::ValueProvenanceRecord> mass_row;
  if (mass.source == "override")
    mass_row =
        stored != rows.end()
            ? *stored
            : ProjectDB::ValueProvenanceRecord{
                  "type", id, "mass_t", "entered", "user", std::nullopt, ""};
  else if (mass.source == "calculated")
    mass_row = ProjectDB::ValueProvenanceRecord{
        "type",
        id,
        "mass_t",
        "calculated",
        "buildup:" + mass.buildup_id.value_or("") +
            (mass.method.empty() ? "" : " (" + mass.method + ")"),
        std::nullopt,
        ""};
  if (stored != rows.end())
    rows.erase(stored);
  if (mass_row) {
    const auto at = std::lower_bound(
        rows.begin(), rows.end(), *mass_row,
        [](const auto &a, const auto &b) { return a.field < b.field; });
    rows.insert(at, *mass_row);
  }
  return rows;
}

std::vector<ProjectDB::ValueProvenanceRecord>
part_provenance(const ProjectDB &db, std::string_view code) {
  return db.value_provenance("part", code);
}

void record_entered(ProjectDB &db, int64_t type_id,
                    const ProjectDB::SurveyTypePatch &patch,
                    std::string_view source) {
  if (!db.has_survey_mass_columns())
    return;
  std::vector<std::string> set_fields, cleared;
  type_fields_of(patch, set_fields, cleared);
  const std::string id = std::to_string(type_id);
  for (const auto &f : set_fields)
    db.set_value_provenance(
        {"type", id, f, "entered", std::string(source), std::nullopt, ""});
  for (const auto &f : cleared)
    db.delete_value_provenance("type", id, f);
}

void record_entered(ProjectDB &db, std::string_view code,
                    const ProjectDB::SurveyPartPatch &patch,
                    std::string_view source) {
  if (!db.has_survey_mass_columns())
    return;
  const std::string c(code);
  const auto mark = [&](bool engaged, const char *f) {
    if (engaged)
      db.set_value_provenance(
          {"part", c, f, "entered", std::string(source), std::nullopt, ""});
  };
  mark(patch.quantity.has_value(), "quantity");
  mark(patch.type_id.has_value(), "type_id");
  mark(patch.room_name.has_value(), "room_name");
  mark(patch.note.has_value(), "note");
}

ProjectDB::SurveyTypeRecord
update_survey_type_entered(ProjectDB &db, int64_t id,
                           const ProjectDB::SurveyTypePatch &patch,
                           std::string_view source) {
  ProjectDB::Transaction tx(db);
  auto rec = db.update_survey_type(id, patch);
  record_entered(db, id, patch, source);
  tx.commit();
  return rec;
}

ProjectDB::SurveyPartRecord
update_survey_part_entered(ProjectDB &db, std::string_view code,
                           const ProjectDB::SurveyPartPatch &patch,
                           std::string_view source) {
  ProjectDB::Transaction tx(db);
  auto rec = db.update_survey_part(code, patch);
  record_entered(db, code, patch, source);
  tx.commit();
  return rec;
}

} // namespace reusex::core
