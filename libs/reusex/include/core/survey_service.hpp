// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

/// Survey operations that need a project. Thin over ProjectDB storage and the
/// pure rules in survey.hpp — the gate and redistribution live here, not in
/// ProjectDB, which stays storage-only.

#include "reusex/core/ProjectDB.hpp"
#include "reusex/core/survey.hpp"
#include "reusex/core/survey_measure.hpp"

#include <cstddef>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace reusex::core {

/// A type's miljøstatus folds every sample that bears on it: linked to the
/// type directly, or linked to a non-dismissed survey part currently filed
/// under it (schema v32/v36) — see ProjectDB::samples_for_type.
std::map<int64_t, EnvironmentStatus>
environment_statuses(const ProjectDB &db); // every type
EnvironmentStatus environment_status_of(const ProjectDB &db, int64_t type_id);
/// Approving a type whose miljøstatus is `afventer` throws SamplePendingError.
ProjectDB::SurveyTypeRecord set_review_status(ProjectDB &db, int64_t type_id,
                                              ReviewStatus status);
/// Scales the type's non-dismissed parts to sum to `total`
/// (redistribute_quantity); dismissed parts (schema v36) keep their quantity
/// and get no share. Returns every part of the type, dismissed included.
/// Throws std::invalid_argument when the type has no non-dismissed parts or
/// total < 0.
std::vector<ProjectDB::SurveyPartRecord>
set_type_quantity(ProjectDB &db, int64_t type_id, double total);
/// The type's quantity: the sum of its non-dismissed parts' quantities.
double type_quantity(const ProjectDB &db, int64_t type_id);
/// Names of the rooms (labels of @p rooms_cloud, 0 excluded, in label order)
/// that no non-dismissed part is placed in; a room with no label definition
/// reads "Rum <id>". Empty when the cloud does not exist.
std::vector<std::string>
rooms_without_parts(const ProjectDB &db,
                    std::string_view rooms_cloud = "rooms");
/// One per survey type, same order; mass_t is the type's effective mass
/// (core::survey_masses: the user's override, else the calculation).
std::vector<TypeTotals> type_totals(const ProjectDB &db);
/// Sample edit with the stage/result rule enforced (validate_sample_state on
/// the merged state).
ProjectDB::SampleRecord
update_sample_checked(ProjectDB &db, int64_t id,
                      const ProjectDB::SamplePatch &patch);

/// One survey type as the PDF report's Kortlægning section lists it.
struct ReportSurveyRow {
  std::string name, bim7aa_code, eak_code, unit;
  double quantity = 0.0;        // sum of the type's non-dismissed parts
  std::optional<double> mass_t; // effective mass (core::survey_masses)
  Treatment treatment = Treatment::genanvendelse;
  EnvironmentStatus environment = EnvironmentStatus::ren_screening;
};
/// One row per **reportable** survey type, in id order, for the PDF report's
/// survey section ("kun godkendte mængder indgår"). A type is listed when it is
/// approved and not awaiting a sample (core::reportable) and it does not block
/// the waste report — so the rows agree with fractions_by_eak: an approved,
/// non-bevaring type without tonnes blocks there (reason `mass`) and is left
/// out here too. A bevaring type without tonnes never blocks (it is not
/// waste), so it is listed with mass_t nullopt. `quantity` is the sum of the
/// type's non-dismissed parts (type_quantity).
std::vector<ReportSurveyRow> report_survey_rows(const ProjectDB &db);

/// Options for `sync_survey` — the cloud names it reads from.
struct SurveySyncOptions {
  std::string instances_cloud =
      std::string(kDefaultInstanceCloud); // mirrors `rux create materials`
  std::string semantic_cloud = "labels";
  std::string rooms_cloud = "rooms";
  /// Positions of the instance cloud's points (index-aligned), for placing
  /// instances in sensor frames when attaching views.
  std::string positions_cloud = "cloud";
  /// Then ground every part in the model (core::measure_survey, schema
  /// v37): wall / slab / roof element parts, opening areas, counts.
  bool measure = true;
};
/// Counts of what `sync_survey` changed.
struct SurveySyncReport {
  std::size_t types_created = 0;
  std::size_t parts_created = 0;
  std::size_t parts_existing = 0;
  /// Instances skipped because the user deleted their part (tombstoned in
  /// survey_dismissed_instances, schema v26).
  std::size_t parts_dismissed = 0;
  /// Instance rows considered: parts_created + parts_existing +
  /// parts_dismissed. Zero means the
  /// instance cloud holds no instances, as opposed to "all already surveyed".
  std::size_t instances_seen = 0;
  /// Instance rows sync wrote first because the cloud had labels but no
  /// `instances` rows: a cloud from before schema v10, whose migration only
  /// backfilled clouds that had material links.
  std::size_t instances_backfilled = 0;
  bool rooms_assigned = false;
  /// Parts whose instance_guid no longer resolves to an instance row (the
  /// instance was dropped, or re-created without carrying the guid over).
  /// Kept and still counted toward their type's quantity — review or re-file
  /// them; sync_survey does not delete or hide them. Dismissed parts are not
  /// counted here.
  std::size_t parts_orphaned = 0;
  std::vector<std::string> orphaned_codes;
  /// Instance links put back: instance-backed parts that own a passport
  /// (survey_parts.passport_guid) but whose instance had lost its
  /// instance_materials row — `rux create instances` re-runs cascade-delete
  /// them. A link to a different passport is never overwritten (warned).
  std::size_t links_restored = 0;
  /// Instance-backed parts without a view that got one: the best-ranked
  /// frame (core::instance_evidence), region = the instance's padded bbox
  /// in it (spec 2026-10-13-resource-view-record §2). Never overwrites.
  std::size_t views_attached = 0;
  /// Instance-backed parts of instances_cloud still without a view: no
  /// frame sees the instance, or the project has no sensor frames, no
  /// positions cloud, or misaligned clouds.
  std::size_t views_unavailable = 0;
  /// What measure_survey did (empty when SurveySyncOptions::measure is off
  /// or the project is below v37).
  SurveyMeasureReport measure{};
};
/// Fill survey_types / survey_parts from the instances table: one type per
/// semantic class, one bygningsdel (survey part) per instance, placed in the
/// room most of its points fall in. Idempotent — only adds instances that
/// have no part yet (has_survey_part_for, keyed on instance guid) and were
/// not dismissed (a deleted part stays deleted, see parts_dismissed), and never
/// touches existing types/parts, so edits made in the GUI survive a rerun —
/// a dismissed part (survey_parts.dismissed, v36) still exists, so its
/// instance counts as parts_existing and the part stays dismissed.
/// If the instance cloud has labels but no `instances` rows (a pre-v10
/// cloud), the rows are backfilled first from the cloud's labels and its
/// "SM{class}-{id}" definitions (see instances_backfilled). Lost instance
/// links of parts that own a passport are restored (see links_restored). Throws
/// std::runtime_error naming `rux create instances` if the instances cloud does
/// not exist. Then every instance-backed part without a view (new or
/// pre-existing: the backfill for old projects) gets the best-ranked frame as
/// its view; see views_attached / views_unavailable.
SurveySyncReport sync_survey(ProjectDB &db, const SurveySyncOptions &opts = {});

} // namespace reusex::core
