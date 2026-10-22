// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "reusex/core/resources.hpp"

#include "reusex/core/frame_projection.hpp"
#include "reusex/core/logging.hpp"
#include "reusex/core/survey.hpp"
#include "reusex/core/survey_service.hpp"

#include <algorithm>
#include <map>
#include <set>

namespace reusex::core {
namespace {

struct Context {
  std::vector<ResourceKey> catalogue;
  std::map<int64_t, ProjectDB::SurveyTypeRecord> types;
  std::map<int64_t, EnvironmentStatus> env;
};

/// @p catalogue reuses a catalogue already built for this request.
Context load(const ProjectDB &db,
             std::optional<std::vector<ResourceKey>> catalogue = std::nullopt) {
  Context c;
  c.catalogue = catalogue ? std::move(*catalogue) : key_catalogue(db);
  for (auto &t : db.survey_types())
    c.types.emplace(t.id, std::move(t));
  c.env = environment_statuses(db);
  return c;
}

std::optional<std::string> builtin_value(const ResourceKey &k,
                                         const ProjectDB::SurveyPartRecord &p,
                                         const ProjectDB::SurveyTypeRecord &t,
                                         EnvironmentStatus e) {
  const auto &f = k.field;
  if (f == "name")
    return t.name;
  if (f == "quantity")
    return format_number(p.quantity);
  if (f == "unit")
    return t.unit;
  if (f == "eak")
    return t.eak_code;
  if (f == "bim7aa")
    return t.bim7aa_code;
  if (f == "treatment")
    return std::string(to_string(t.treatment));
  if (f == "environment")
    return std::string(to_string(e));
  if (f == "room")
    return p.room_name;
  if (f == "mass_t")
    return t.mass_t ? std::optional<std::string>(format_number(*t.mass_t))
                    : std::nullopt;
  if (f == "note")
    return p.note;
  if (f == "starred")
    return std::string(p.starred ? "true" : "false");
  if (f == "view")
    return p.view ? std::optional<std::string>(view_to_json(*p.view))
                  : std::nullopt;
  return std::nullopt;
}

/// add_material_passport stores every field, the unset ones as "", "[]"
/// or (TriState) "unknown" — the same as having no value.
bool is_blank_stored(const ResourceKey &k, const std::string &v) {
  if (v.empty() || v == "[]")
    return true;
  return k.source == KeySource::leksikon && k.data_type == "enum" &&
         v == "unknown";
}

Resource build(const ProjectDB &db, const Context &c,
               const ProjectDB::SurveyPartRecord &p,
               const std::optional<std::vector<std::string>> &keys) {
  Resource r{p.code, p.type_id, !p.instance_guid.has_value(), {}};
  r.view = p.view;
  r.dismissed = p.dismissed;
  const auto &t = c.types.at(p.type_id);
  const auto e = c.env.at(p.type_id);
  const auto stored = p.material_guid
                          ? db.passport_stored_properties(*p.material_guid)
                          : std::map<std::string, std::string>{};
  auto value = [&](const ResourceKey &k) -> std::optional<std::string> {
    if (k.source == KeySource::builtin)
      return builtin_value(k, p, t, e);
    const auto it = stored.find(k.field);
    if (it == stored.end() || is_blank_stored(k, it->second))
      return std::nullopt;
    return it->second;
  };
  if (keys) {
    for (const auto &id : *keys) {
      const auto *k = find_key(c.catalogue, id);
      r.values.push_back({id, k ? value(*k) : std::nullopt});
    }
    return r;
  }
  for (const auto &k : c.catalogue) {
    auto v = value(k);
    if (k.source == KeySource::builtin || v)
      r.values.push_back({k.id, std::move(v)});
  }
  return r;
}

bool leksikon_field_named(std::string_view name) {
  for (const auto &f : leksikon_fields())
    if (f.field_name == name)
      return true;
  return false;
}

void check_column_name(const ProjectDB &db, const std::string &name,
                       const std::string &except_id) {
  if (name.empty())
    throw std::invalid_argument("a column name cannot be empty");
  for (const auto &d : db.list_property_definitions())
    if (d.name == name && d.id != except_id)
      throw NameConflictError("a column named '" + name + "' already exists");
  if (leksikon_field_named(name))
    throw NameConflictError("'" + name +
                            "' is a leksikon field name; choose another "
                            "column name");
  // Values a deleted column left behind: a column under this name would
  // silently show them as its own.
  if (db.has_passport_field_values(name))
    throw NameConflictError("values are already stored under '" + name +
                            "' (left by a deleted column); choose another "
                            "column name");
}
} // namespace

std::optional<std::string> value_of(const Resource &r, std::string_view key) {
  for (const auto &v : r.values)
    if (v.key == key)
      return v.value;
  return std::nullopt;
}

std::vector<Resource>
list_resources(const ProjectDB &db,
               const std::optional<std::vector<std::string>> &keys) {
  const auto c = load(db);
  std::vector<Resource> out;
  for (const auto &p : db.survey_parts())
    out.push_back(build(db, c, p, keys));
  return out;
}

Resource resource(const ProjectDB &db, std::string_view code,
                  const std::optional<std::vector<std::string>> &keys) {
  const auto p = db.survey_part(code);
  if (!p)
    throw std::out_of_range("no resource '" + std::string(code) + "'");
  return build(db, load(db), *p, keys);
}

ResourcePatchResult
patch_resource(ProjectDB &db, std::string_view code,
               const std::vector<ResourceWrite> &writes,
               const std::optional<std::vector<std::string>> &keys) {
  const auto part = db.survey_part(code);
  if (!part)
    throw std::out_of_range("no resource '" + std::string(code) + "'");
  const auto catalogue = key_catalogue(db);

  // Validate everything first, write second (same rule as patch_material):
  // a refused request changes nothing.
  struct Planned {
    const ResourceKey *key;
    std::optional<std::string> value;
  };
  std::vector<Planned> plan;
  std::set<std::string> seen;
  for (const auto &w : writes) {
    const auto *k = find_key(catalogue, w.key);
    if (!k)
      throw KeyValueError(w.key, "unknown key '" + w.key + "'");
    if (!seen.insert(w.key).second)
      throw KeyValueError(w.key, "key '" + w.key + "' is written twice");
    auto value = normalise_value(*k, w.value);
    if (k->id == kViewKey && !value) {
      // A resource keeps its image once it has one: the setter refuses this
      // too (inside the write transaction), but refuse here so a bad clear
      // writes nothing (validate first, write second). Clearing an
      // already-absent view stays a no-op success.
      if (part->view)
        throw KeyValueError(k->id, "'" + k->id +
                                       "' already has a value and cannot be "
                                       "cleared");
    } else if (k->id == kViewKey && value) {
      // The setter checks this too, but inside the write transaction; refuse
      // here so a bad view writes nothing (validate first, write second).
      const auto view = parse_view(*value);
      const bool frame = view.kind == ViewKind::frame;
      if (frame ? !db.has_sensor_frame(view.id)
                : !db.has_panoramic_image(view.id))
        throw KeyValueError(k->id, "'" + k->id + "' names " +
                                       (frame ? "sensor frame " : "panorama ") +
                                       std::to_string(view.id) +
                                       ", which does not exist");
      // A frame picked without a region shows the instance's part of it.
      if (part->cloud_name && part->instance_id && *part->instance_id != 0) {
        const auto filled = complete_view_region(db, view, *part->cloud_name,
                                                 *part->instance_id);
        if (filled.region != view.region)
          value = view_to_json(filled);
      }
    }
    plan.push_back({k, std::move(value)});
  }

  ProjectDB::SurveyTypePatch tp;
  ProjectDB::SurveyPartPatch pp;
  std::optional<std::optional<ResourceView>> view_write;
  bool type_written = false;
  std::vector<Planned> passport;
  for (const auto &pl : plan) {
    if (pl.key->source != KeySource::builtin) {
      passport.push_back(pl);
      continue;
    }
    const auto &f = pl.key->field;
    const auto &v = pl.value; // normalise_value refused nulls where invalid
    if (f == "name")
      tp.name = *v;
    else if (f == "unit")
      tp.unit = *v;
    else if (f == "eak")
      tp.eak_code = v.value_or("");
    else if (f == "bim7aa")
      tp.bim7aa_code = v.value_or("");
    else if (f == "treatment")
      tp.treatment = *treatment_from_string(*v);
    else if (f == "mass_t")
      tp.mass_t = v ? std::optional<double>(*parse_number(*v))
                    : std::optional<double>{};
    else if (f == "quantity")
      pp.quantity = *parse_number(*v);
    else if (f == "room")
      pp.room_name = v.value_or("");
    else if (f == "note")
      pp.note = v.value_or("");
    else if (f == "starred")
      pp.starred = *v == "true";
    else if (f == "view")
      view_write =
          v ? std::optional<ResourceView>(parse_view(*v)) : std::nullopt;
    type_written = type_written || pl.key->scope == KeyScope::type;
  }

  {
    ProjectDB::Transaction tx(db);
    if (type_written)
      db.update_survey_type(part->type_id, tp);
    db.update_survey_part(code, pp); // no-op when the patch is empty
    if (view_write)
      db.set_survey_part_view(code, *view_write);
    if (!passport.empty()) {
      std::optional<std::string> guid = part->material_guid;
      const bool any_set =
          std::any_of(passport.begin(), passport.end(),
                      [](const Planned &p) { return p.value.has_value(); });
      if (any_set)
        guid = db.ensure_resource_passport(code);
      if (guid) {
        const auto stored = db.passport_stored_properties(*guid);
        for (const auto &pl : passport) {
          if (pl.value)
            db.set_passport_property(*guid, pl.key->field, *pl.value);
          else if (stored.count(pl.key->field) != 0)
            db.delete_passport_property(*guid, pl.key->field);
        }
      }
    }
    tx.commit();
  }

  // Columns did not change: one catalogue serves the whole patch.
  const auto c = load(db, catalogue);
  ResourcePatchResult out{build(db, c, *db.survey_part(code), keys), {}};
  if (type_written)
    for (const auto &p : db.survey_parts())
      if (p.type_id == part->type_id && p.code != code)
        out.siblings.push_back(build(db, c, p, keys));
  return out;
}

Resource create_resource(ProjectDB &db, int64_t type_id,
                         const std::optional<std::string> &name) {
  if (!db.survey_type(type_id))
    throw std::out_of_range("no survey type " + std::to_string(type_id));
  if (name && name->empty())
    throw std::invalid_argument("'name' must be non-empty when given");
  ProjectDB::SurveyPartRecord rec;
  rec.type_id = type_id;
  {
    ProjectDB::Transaction tx(db);
    // Inside BEGIN IMMEDIATE, so a second writer cannot take the same code.
    rec.code = part_code(db.max_survey_part_number() + 1);
    db.add_survey_part(rec);
    if (name) {
      const auto guid = db.ensure_resource_passport(rec.code);
      db.set_passport_property(guid, kDesignationField, *name);
    }
    tx.commit();
  }
  return resource(db, rec.code);
}

namespace {
/// delete_resource's body without the transaction, so delete_survey_type can
/// run it for every part inside its own.
void delete_part_in_tx(ProjectDB &db, const ProjectDB::SurveyPartRecord &part) {
  if (part.instance_guid)
    db.dismiss_instance(*part.instance_guid);
  // The part's own instance link goes with it; a link to another passport
  // is not the part's to drop.
  if (part.cloud_name && part.instance_id && part.material_guid &&
      db.instance_material_guid(*part.cloud_name,
                                static_cast<int>(*part.instance_id)) ==
          part.material_guid)
    db.unlink_instance_material(*part.cloud_name,
                                static_cast<int>(*part.instance_id));
  db.delete_survey_part(part.code);
  if (part.material_guid) {
    if (!db.is_passport_linked(*part.material_guid))
      db.delete_material_passport(*part.material_guid);
    else
      reusex::warn("delete_resource: kept passport '{}' of deleted part '{}' "
                   "— another part or an instance still links it",
                   *part.material_guid, part.code);
  }
}
} // namespace

void delete_resource(ProjectDB &db, std::string_view code) {
  const auto part = db.survey_part(code);
  if (!part)
    throw std::out_of_range("no resource '" + std::string(code) + "'");
  ProjectDB::Transaction tx(db);
  delete_part_in_tx(db, *part);
  tx.commit();
}

SurveyTypeDeletion delete_survey_type(ProjectDB &db, int64_t type_id) {
  if (!db.survey_type(type_id))
    throw std::out_of_range("no survey type " + std::to_string(type_id));
  SurveyTypeDeletion out;
  ProjectDB::Transaction tx(db);
  for (const auto &p : db.survey_parts()) {
    if (p.type_id != type_id)
      continue;
    delete_part_in_tx(db, p);
    ++out.parts_deleted;
    if (p.instance_guid)
      ++out.instances_dismissed;
  }
  db.delete_survey_type(type_id);
  tx.commit();
  reusex::info("delete_survey_type: deleted type {} with {} part(s), {} of "
               "them scan-backed (tombstoned)",
               type_id, out.parts_deleted, out.instances_dismissed);
  return out;
}

ProjectDB::PropertyDefinition create_column(ProjectDB &db,
                                            ProjectDB::PropertyDefinition def) {
  // Check and insert in one transaction: no second writer can take the
  // name in between.
  ProjectDB::Transaction tx(db);
  check_column_name(db, def.name, "");
  def.id = db.add_property_definition(def.name, def.type, def.options,
                                      def.sort_order, def.width);
  tx.commit();
  return def;
}

void delete_column(ProjectDB &db, const std::string &id) {
  const auto defs = db.list_property_definitions();
  const auto it = std::find_if(defs.begin(), defs.end(),
                               [&](const auto &d) { return d.id == id; });
  if (it == defs.end())
    throw std::out_of_range("no column '" + id + "'");
  ProjectDB::Transaction tx(db);
  const auto values = db.delete_passport_field_values(it->name);
  db.delete_property_definition(id);
  tx.commit();
  if (values > 0)
    reusex::warn("delete_column: deleted column '{}' ({}) and its {} stored "
                 "value(s)",
                 it->name, id, values);
}

ProjectDB::PropertyDefinition
update_column(ProjectDB &db, const std::string &id, const ColumnPatch &p) {
  const auto defs = db.list_property_definitions();
  const auto it = std::find_if(defs.begin(), defs.end(),
                               [&](const auto &d) { return d.id == id; });
  if (it == defs.end())
    throw std::out_of_range("no column '" + id + "'");
  auto def = *it;
  const auto old_name = def.name;
  if (p.name)
    def.name = *p.name;
  if (p.type)
    def.type = *p.type;
  if (p.options)
    def.options = *p.options;
  if (p.sort_order)
    def.sort_order = *p.sort_order;
  if (p.width)
    def.width = *p.width;
  if (def.name != old_name)
    check_column_name(db, def.name, id);
  ProjectDB::Transaction tx(db);
  if (def.name != old_name)
    db.rename_passport_field(old_name, def.name);
  db.update_property_definition(id, def.name, def.type, def.options,
                                def.sort_order, def.width);
  tx.commit();
  return def;
}

} // namespace reusex::core
