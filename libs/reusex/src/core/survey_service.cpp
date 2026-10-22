// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "reusex/core/survey_service.hpp"

#include "reusex/core/frame_projection.hpp"
#include "reusex/core/guid.hpp"
#include "reusex/core/instance_evidence.hpp"
#include "reusex/core/label_encoding.hpp"
#include "reusex/core/label_semantics.hpp"
#include "reusex/core/logging.hpp"
#include "reusex/core/resource_view.hpp"
#include "reusex/core/survey_mass.hpp"
#include "reusex/core/survey_measure.hpp"

#include <Eigen/Core>

#include <algorithm>
#include <climits>
#include <set>
#include <stdexcept>
#include <utility>

namespace reusex::core {

std::map<int64_t, EnvironmentStatus> environment_statuses(const ProjectDB &db) {
  // A sample bears on the types it links directly and on the type each of
  // its parts is filed under now (same rule as ProjectDB::samples_for_type),
  // counted once per type. A dismissed part bears on nothing.
  std::map<std::string, int64_t> part_type;
  for (const auto &p : db.survey_parts())
    if (!p.dismissed)
      part_type.emplace(p.code, p.type_id);
  std::map<int64_t, std::vector<SampleState>> linked;
  for (const auto &s : db.samples()) {
    std::set<int64_t> types(s.type_ids.begin(), s.type_ids.end());
    for (const auto &code : s.part_codes)
      if (const auto it = part_type.find(code); it != part_type.end())
        types.insert(it->second);
    for (auto t : types)
      linked[t].push_back({s.stage, s.result});
  }
  std::map<int64_t, EnvironmentStatus> out;
  for (const auto &t : db.survey_types())
    out[t.id] = environment_status(linked[t.id]);
  return out;
}

EnvironmentStatus environment_status_of(const ProjectDB &db, int64_t type_id) {
  std::vector<SampleState> linked;
  for (const auto &s : db.samples_for_type(type_id))
    linked.push_back({s.stage, s.result});
  return environment_status(linked);
}

ProjectDB::SurveyTypeRecord set_review_status(ProjectDB &db, int64_t type_id,
                                              ReviewStatus status) {
  if (!db.survey_type(type_id))
    throw std::out_of_range("no survey type " + std::to_string(type_id));
  if (status == ReviewStatus::approved &&
      environment_status_of(db, type_id) == EnvironmentStatus::afventer) {
    std::string codes;
    for (const auto &s : db.samples_for_type(type_id))
      if (s.stage != SampleStage::svar)
        codes += (codes.empty() ? "" : ", ") + s.code;
    throw SamplePendingError("survey type " + std::to_string(type_id) +
                             " cannot be approved while sample(s) " + codes +
                             " await a lab answer");
  }
  ProjectDB::SurveyTypePatch p;
  p.review_status = status;
  return db.update_survey_type(type_id, p);
}

std::vector<ProjectDB::SurveyPartRecord>
set_type_quantity(ProjectDB &db, int64_t type_id, double total) {
  if (total < 0.0)
    throw std::invalid_argument("quantity must not be negative");
  // Dismissed parts (schema v36) are not part of the type's quantity: they
  // keep theirs untouched and get no share of the new total.
  std::vector<ProjectDB::SurveyPartRecord> parts;
  for (auto &p : db.survey_parts())
    if (p.type_id == type_id && !p.dismissed)
      parts.push_back(std::move(p));
  if (parts.empty())
    throw std::invalid_argument("survey type " + std::to_string(type_id) +
                                " has no (non-dismissed) parts to distribute "
                                "a quantity over");
  std::vector<double> current;
  for (const auto &p : parts)
    current.push_back(p.quantity);
  const auto next = redistribute_quantity(current, total);
  std::vector<std::pair<std::string, double>> code_quantities;
  for (std::size_t i = 0; i < parts.size(); ++i)
    code_quantities.emplace_back(parts[i].code, next[i]);
  db.set_survey_part_quantities(code_quantities);
  if (db.has_survey_mass_columns()) {
    // The user set the type's total: every share is entered, so
    // measure_survey no longer seeds these quantities.
    ProjectDB::Transaction tx(db);
    for (const auto &[code, q] : code_quantities)
      db.set_value_provenance({"part", code, "quantity", "entered",
                               "type_total", std::nullopt, ""});
    tx.commit();
  }
  std::vector<ProjectDB::SurveyPartRecord> out;
  for (const auto &p : db.survey_parts())
    if (p.type_id == type_id)
      out.push_back(p);
  return out;
}

std::vector<TypeTotals> type_totals(const ProjectDB &db) {
  const auto env = environment_statuses(db);
  // The effective mass (schema v37): the user's override, else the mass
  // calculated from the build-up and the parts' quantities.
  const auto masses = survey_masses(db);
  std::vector<TypeTotals> out;
  for (const auto &t : db.survey_types())
    out.push_back({t.treatment, t.review_status, masses.at(t.id).effective_t,
                   t.eak_code, env.at(t.id), t.id, t.name});
  return out;
}

std::vector<ReportSurveyRow> report_survey_rows(const ProjectDB &db) {
  const auto env = environment_statuses(db);
  // Whatever blocks the waste report is withheld here as well, so the PDF's
  // survey rows and its fractions never disagree about what is reported.
  std::set<int64_t> blocked;
  const auto totals = type_totals(db);
  std::map<int64_t, std::optional<double>> mass;
  for (const auto &t : totals)
    mass[t.type_id] = t.mass_t; // effective mass
  for (const auto &b : fractions_by_eak(totals).blocking)
    blocked.insert(b.type_id);
  std::map<int64_t, double> quantity;
  for (const auto &p : db.survey_parts())
    if (!p.dismissed)
      quantity[p.type_id] += p.quantity;
  std::vector<ReportSurveyRow> rows;
  for (const auto &t : db.survey_types()) {
    const auto e = env.at(t.id);
    if (!reportable(t.review_status, e) || blocked.count(t.id) != 0)
      continue;
    rows.push_back({t.name, t.bim7aa_code, t.eak_code, t.unit, quantity[t.id],
                    mass[t.id], t.treatment, e});
  }
  return rows;
}

double type_quantity(const ProjectDB &db, int64_t type_id) {
  double sum = 0.0;
  for (const auto &p : db.survey_parts())
    if (p.type_id == type_id && !p.dismissed)
      sum += p.quantity;
  return sum;
}

std::vector<std::string> rooms_without_parts(const ProjectDB &db,
                                             std::string_view rooms_cloud) {
  std::vector<std::string> out;
  if (!db.has_point_cloud(rooms_cloud))
    return out;
  const auto cloud = db.point_cloud_label(rooms_cloud);
  if (!cloud)
    return out;
  std::set<std::uint32_t> present;
  for (const auto &p : *cloud)
    if (p.label != 0)
      present.insert(p.label);
  std::set<std::uint32_t> covered;
  for (const auto &p : db.survey_parts())
    if (p.room_id && !p.dismissed)
      covered.insert(*p.room_id);
  const auto names = db.label_definitions(rooms_cloud);
  for (auto r : present)
    if (!covered.contains(r)) {
      const auto n = names.find(static_cast<int>(r));
      out.push_back(n != names.end() ? n->second : "Rum " + std::to_string(r));
    }
  return out;
}

ProjectDB::SampleRecord
update_sample_checked(ProjectDB &db, int64_t id,
                      const ProjectDB::SamplePatch &patch) {
  const auto current = db.sample(id);
  if (!current)
    throw std::out_of_range("no sample " + std::to_string(id));
  validate_sample_state({patch.stage.value_or(current->stage),
                         patch.result.value_or(current->result)});
  return db.update_sample(id, patch);
}

namespace {
std::vector<std::uint32_t> labels_of(const ProjectDB &db,
                                     const std::string &name) {
  std::vector<std::uint32_t> out;
  if (const auto cloud = db.point_cloud_label(name))
    for (const auto &p : *cloud)
      out.push_back(p.label);
  return out;
}

/// The semantic class `rux create instances` recorded in an instance label's
/// "SM{class}-{id} (Np)" definition, or -1. A class above INT_MAX is the
/// pre-#214 unlabeled value (-1) wrapped to uint32 (4294967295).
int class_from_definition(const std::string &def) {
  if (def.size() < 4 || def.compare(0, 2, "SM") != 0)
    return -1;
  std::size_t i = 2;
  unsigned long long value = 0;
  while (i < def.size() && def[i] >= '0' && def[i] <= '9' && value <= INT_MAX)
    value = value * 10 + static_cast<unsigned>(def[i++] - '0');
  if (i == 2 || i >= def.size() || def[i] != '-' || value > INT_MAX)
    return -1;
  return static_cast<int>(value);
}

/// Write `instances` rows for an instance cloud that has labels but none: one
/// row per distinct label with its point count, a fresh guid, and the class
/// from its definition. Clouds from before schema v10 are in this state when
/// they had no material links, which is all the v10 migration backfilled.
std::size_t backfill_instance_rows(ProjectDB &db, const std::string &cloud) {
  std::map<std::uint32_t, int> points;
  for (const auto l : labels_of(db, cloud))
    if (is_valid_label(l))
      ++points[l];
  if (points.empty())
    return 0;
  const auto defs = db.label_definitions(cloud);
  std::vector<ProjectDB::InstanceRecord> rows;
  std::size_t classified = 0;
  for (const auto &[id, n] : points) {
    const auto d = defs.find(static_cast<int>(id));
    const int cls = d != defs.end() ? class_from_definition(d->second) : -1;
    if (cls >= 0)
      ++classified;
    rows.push_back({id, generate_guid(), cls, n});
  }
  db.save_instances(cloud, rows);
  reusex::warn("sync_survey: instance cloud '{}' had {} instance label(s) but "
               "no `instances` rows (written before schema v10); backfilled "
               "them, {} with a semantic class from the label definitions",
               cloud, rows.size(), classified);
  return rows.size();
}

/// Attach a view to every instance-backed part of opts.instances_cloud that
/// has none: the best-ranked frame of one instance_evidence pass over all
/// parts (as ruxd's PhotoEvidenceCache ranks them), region = the instance's
/// padded bbox in it. Never overwrites a view.
void attach_views(ProjectDB &db, const SurveySyncOptions &opts,
                  SurveySyncReport &report) {
  std::vector<ProjectDB::SurveyPartRecord> pending;
  for (auto &p : db.survey_parts())
    if (!p.view && p.instance_id && p.cloud_name == opts.instances_cloud)
      pending.push_back(std::move(p));
  if (pending.empty())
    return;
  const auto unavailable = [&](std::string_view why, bool loud) {
    report.views_unavailable = pending.size();
    if (loud)
      reusex::warn("sync_survey: {}; {} part(s) get no resource image", why,
                   pending.size());
    else
      reusex::info("sync_survey: {}; {} part(s) get no resource image", why,
                   pending.size());
  };
  if (db.sensor_frame_ids().empty())
    return unavailable("the project has no sensor frames", false);
  if (!db.has_point_cloud(opts.positions_cloud))
    return unavailable("no '" + opts.positions_cloud +
                           "' cloud to place instances in frames (run `rux "
                           "create clouds`)",
                       true);
  std::map<std::uint32_t, InstanceEvidence> evidence;
  std::map<std::uint32_t, std::vector<Eigen::Vector3f>> points;
  try {
    evidence = instance_evidence(db, opts.instances_cloud, PhotoQuery{},
                                 opts.positions_cloud);
    points = instance_points(db, opts.instances_cloud, opts.positions_cloud);
  } catch (const CloudMisalignedError &e) {
    return unavailable(e.what(), true);
  }
  std::vector<std::pair<std::string, ResourceView>> writes;
  for (const auto &p : pending) {
    const auto ev = evidence.find(*p.instance_id);
    if (ev == evidence.end() || ev->second.frames.empty()) {
      ++report.views_unavailable;
      continue;
    }
    const int frame = ev->second.frames.front().frame_id;
    const auto pts = points.find(*p.instance_id);
    writes.emplace_back(
        p.code,
        ResourceView{ViewKind::frame, frame,
                     pts == points.end()
                         ? std::nullopt
                         : project_instance_bbox(db, frame, pts->second)});
  }
  if (!writes.empty()) {
    ProjectDB::Transaction tx(db);
    for (const auto &[code, view] : writes)
      db.set_survey_part_view(code, view);
    tx.commit();
  }
  report.views_attached = writes.size();
  if (report.views_attached == 0)
    reusex::warn("sync_survey: no sensor frame sees any of the {} part(s) "
                 "without a resource image",
                 report.views_unavailable);
  else
    reusex::info("sync_survey: attached {} resource image(s); {} part(s) "
                 "seen by no frame keep none",
                 report.views_attached, report.views_unavailable);
}
} // namespace

SurveySyncReport sync_survey(ProjectDB &db, const SurveySyncOptions &opts) {
  if (!db.has_point_cloud(opts.instances_cloud))
    throw std::runtime_error("sync_survey: no instance cloud '" +
                             opts.instances_cloud +
                             "' — run `rux create instances` first");
  SurveySyncReport report;

  // Room per instance, when a rooms cloud aligned with the instance cloud
  // exists.
  std::map<std::uint32_t, std::uint32_t> room_of;
  std::map<int, std::string> room_names;
  if (db.has_point_cloud(opts.rooms_cloud)) {
    const auto inst = labels_of(db, opts.instances_cloud);
    const auto rooms = labels_of(db, opts.rooms_cloud);
    if (inst.size() == rooms.size()) {
      room_of = majority_room(inst, rooms);
      room_names = db.label_definitions(opts.rooms_cloud);
      report.rooms_assigned = true;
    } else {
      reusex::warn("sync_survey: '{}' has {} labels but '{}' has {} — clouds "
                   "are out of sync; "
                   "parts get no room (re-run `rux create rooms`)",
                   opts.rooms_cloud, rooms.size(), opts.instances_cloud,
                   inst.size());
    }
  } else {
    reusex::warn("sync_survey: no '{}' cloud; parts get no room (run `rux "
                 "create rooms`)",
                 opts.rooms_cloud);
  }

  warn_if_unstamped_semantic_cloud(db, opts.semantic_cloud);
  const auto class_names = db.has_point_cloud(opts.semantic_cloud)
                               ? db.label_definitions(opts.semantic_cloud)
                               : std::map<int, std::string>{};
  std::map<int, int64_t> type_for_class;
  for (const auto &t : db.survey_types())
    if (t.semantic_class != kManualSemanticClass)
      type_for_class.try_emplace(t.semantic_class, t.id);

  auto instances = db.instances(opts.instances_cloud);
  if (instances.empty()) {
    report.instances_backfilled =
        backfill_instance_rows(db, opts.instances_cloud);
    if (report.instances_backfilled > 0)
      instances = db.instances(opts.instances_cloud);
  }
  report.instances_seen = instances.size();
  std::sort(instances.begin(), instances.end(),
            [](const auto &a, const auto &b) {
              return a.instance_id < b.instance_id;
            });
  int next = db.max_survey_part_number();
  for (const auto &inst : instances) {
    if (db.has_survey_part_for(opts.instances_cloud, inst.instance_id)) {
      ++report.parts_existing;
      continue;
    }
    if (db.is_instance_dismissed(inst.guid)) {
      ++report.parts_dismissed;
      continue;
    }
    auto it = type_for_class.find(inst.semantic_class);
    if (it == type_for_class.end()) {
      ProjectDB::SurveyTypeRecord t;
      const auto name = class_names.find(inst.semantic_class);
      t.name = inst.semantic_class < 0 ? "Uklassificeret"
               : name != class_names.end()
                   ? name->second
                   : "Klasse " + std::to_string(inst.semantic_class);
      t.semantic_class = inst.semantic_class;
      it = type_for_class.emplace(inst.semantic_class, db.add_survey_type(t).id)
               .first;
      ++report.types_created;
    }
    ProjectDB::SurveyPartRecord part;
    part.code = part_code(++next);
    part.type_id = it->second;
    part.cloud_name = opts.instances_cloud;
    part.instance_id = inst.instance_id;
    if (const auto r = room_of.find(inst.instance_id); r != room_of.end()) {
      part.room_id = r->second;
      const auto n = room_names.find(static_cast<int>(r->second));
      part.room_name = n != room_names.end()
                           ? n->second
                           : "Rum " + std::to_string(r->second);
    }
    db.add_survey_part(part);
    ++report.parts_created;
  }
  if (report.parts_dismissed > 0)
    reusex::info("sync_survey: skipped {} instance(s) whose part was deleted "
                 "(survey_dismissed_instances)",
                 report.parts_dismissed);
  if (instances.empty())
    reusex::warn("sync_survey: instance cloud '{}' has no instances (every "
                 "point is unlabeled); nothing to survey",
                 opts.instances_cloud);

  // Instance links lost to a `rux create instances` re-run (save_instances
  // cascade-deletes instance_materials): a part that owns a passport gets its
  // instance's link back. survey_part() reports the owned passport first, so
  // an instance with no link but a part material_guid means an owned one; an
  // instance linked to a different passport is left alone (spec §4.1).
  {
    std::size_t conflicting = 0;
    std::vector<std::string> restore;
    for (const auto &p : db.survey_parts()) {
      if (!p.cloud_name || !p.instance_id || !p.material_guid)
        continue;
      const auto linked = db.instance_material_guid(
          *p.cloud_name, static_cast<int>(*p.instance_id));
      if (!linked)
        restore.push_back(p.code);
      else if (*linked != *p.material_guid)
        ++conflicting;
    }
    if (!restore.empty()) {
      ProjectDB::Transaction tx(db);
      for (const auto &code : restore)
        db.ensure_resource_passport(code); // owned: only restores the link
      tx.commit();
      report.links_restored = restore.size();
      reusex::info("sync_survey: restored {} instance link(s) to their "
                   "parts' passports",
                   report.links_restored);
    }
    if (conflicting > 0)
      reusex::warn("sync_survey: {} instance-backed part(s) own a passport "
                   "but their instance is linked to a different one; the "
                   "existing link(s) were left alone",
                   conflicting);
  }

  attach_views(db, opts, report);

  // Parts whose instance_guid is set but no longer resolves to an instance
  // row: the instance was deleted, or recreated without carrying the guid
  // over (e.g. `rux create instances --clear`). They keep their code and
  // quantity and still count toward their type's total — report, don't hide
  // or delete (that is a Phase 3 product decision).
  // A dismissed part is already marked as an error and counts toward nothing,
  // so it is not reported again here.
  for (const auto &p : db.survey_parts())
    if (p.instance_guid && !p.instance_id && !p.dismissed) {
      ++report.parts_orphaned;
      report.orphaned_codes.push_back(p.code);
    }
  if (opts.measure) {
    if (db.has_survey_mass_columns())
      report.measure = measure_survey(db);
    else
      reusex::warn("sync_survey: the project is below schema v37; parts are "
                   "not measured on the model");
  }

  if (report.parts_orphaned > 0) {
    std::string codes;
    const std::size_t shown =
        std::min<std::size_t>(20, report.orphaned_codes.size());
    for (std::size_t i = 0; i < shown; ++i)
      codes += (i ? ", " : "") + report.orphaned_codes[i];
    if (report.orphaned_codes.size() > shown)
      codes += "…";
    reusex::warn("sync_survey: {} part(s) link to instances that no longer "
                 "exist ({}); their quantities still count toward their "
                 "types — review or re-file them",
                 report.parts_orphaned, codes);
  }
  return report;
}

} // namespace reusex::core
