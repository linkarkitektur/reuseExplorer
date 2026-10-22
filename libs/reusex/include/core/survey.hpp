// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

/// Survey (Ressourcekortlægning) rules that need no database: the wire
/// vocabulary shared by ProjectDB, the REST API and the frontend, and the
/// derivations the GUI shows — miljøstatus from linked samples, tonnes per
/// affaldshierarki step, approved tonnes per EAK code. Everything here is pure
/// so it is testable without a project, and so `rux`, `ruxd` and a future
/// Qt client all compute the same numbers.

#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace reusex::core {

enum class Treatment {
  bevaring,
  genbrug,
  genanvendelse,
  nyttiggoerelse,
  bortskaffelse
};
inline constexpr std::size_t kTreatmentCount = 5;
/// semantic_class of a survey type created by hand, never matched by
/// sync_survey.
inline constexpr int kManualSemanticClass = -2;
/// Default name of the instance-label cloud `sync_survey` reads from, and the
/// default `SurveySyncOptions::instances_cloud`.
inline constexpr std::string_view kDefaultInstanceCloud = "instances";
enum class ReviewStatus { queue, approved, rejected };
enum class SampleStage { planlagt, udtaget, sendt, svar };
enum class SampleResult { none, ren, forurenet };
enum class EnvironmentStatus {
  ren_screening,
  afventer,
  forurenet,
  ren_proevesvar
};

/// Where a survey value came from (value_provenance.kind, schema v37, gap
/// 2): seen in the scan (an instance, a classifier confidence), measured on
/// the model geometry, calculated (mass from a build-up), entered by a user,
/// or suggested by a step and not yet confirmed.
enum class ProvenanceKind { scan, measured, calculated, entered, suggested };
std::string_view to_string(ProvenanceKind);
std::optional<ProvenanceKind> provenance_kind_from_string(std::string_view);

std::string_view to_string(Treatment);
std::string_view to_string(ReviewStatus);
std::string_view to_string(SampleStage);
std::string_view to_string(SampleResult); // none -> ""
std::string_view to_string(EnvironmentStatus);
/// Danish user-facing labels (the PDF report). Mirror the rux-frontend
/// repo's src/kortlaegning/vocab.ts TREATMENT_LABEL / ENV_LABEL.
std::string_view treatment_label_da(Treatment);
std::string_view environment_label_da(EnvironmentStatus);
std::optional<Treatment> treatment_from_string(std::string_view);
std::optional<ReviewStatus> review_status_from_string(std::string_view);
std::optional<SampleStage> sample_stage_from_string(std::string_view);
std::optional<SampleResult>
    sample_result_from_string(std::string_view); // "" -> none

struct SampleState {
  SampleStage stage = SampleStage::planlagt;
  SampleResult result = SampleResult::none;
};
void validate_sample_state(const SampleState &); // throws std::invalid_argument
EnvironmentStatus environment_status(const std::vector<SampleState> &linked);

class SamplePendingError : public std::runtime_error {
    public:
  using std::runtime_error::runtime_error;
};

/// A name that must be unique (template, user column, stored field) is
/// already taken. The GUI answers 409.
class NameConflictError : public std::runtime_error {
    public:
  using std::runtime_error::runtime_error;
};

struct TypeTotals {
  Treatment treatment = Treatment::genanvendelse;
  ReviewStatus status = ReviewStatus::queue;
  std::optional<double> mass_t;
  std::string eak_code;
  EnvironmentStatus environment = EnvironmentStatus::ren_screening;
  /// Identify the type in a blocking list; not used by the arithmetic.
  std::int64_t type_id = 0;
  std::string name{};
};
std::array<double, kTreatmentCount>
circularity_breakdown(const std::vector<TypeTotals> &);
struct Fraction {
  std::string eak_code;
  std::string name;
  Treatment treatment;
  double mass_t = 0.0;
  bool contaminated = false;
};
/// Why a type blocks the waste report; a type gets one reason, by precedence
/// sample > review > mass.
enum class BlockingReason { review, sample, mass };
std::string_view to_string(BlockingReason); // "review" | "sample" | "mass"
struct BlockingType {
  std::int64_t type_id = 0;
  std::string name;
  std::string eak_code;
  Treatment treatment = Treatment::genanvendelse;
  std::optional<double> mass_t;
  BlockingReason reason = BlockingReason::review;
};
struct FractionReport {
  std::vector<Fraction> fractions;
  double total_t = 0.0;
  std::size_t blocking_types = 0; // == blocking.size()
  std::vector<BlockingType> blocking;
};
/// True when a type's tonnes may be reported: it is approved and not awaiting
/// a sample (an afventer answer can still make it contaminated). The single
/// rule shared by fractions_by_eak and the report's tonnes (Phase 5, F2).
bool reportable(ReviewStatus status, EnvironmentStatus environment);
/// Approved tonnes per (EAK code, treatment, contaminated), for the
/// bygningsaffald.dk report (GUI Phase 5, R3). Rejected types are ignored.
/// `bevaring` never counts: it stays in the building, so it is not waste — but
/// an unapproved bevaring type still blocks. A type awaiting a sample blocks
/// (reason `sample`) and is withheld even when approved, because its answer
/// can make it contaminated. Any other unapproved type blocks (`review`). An
/// approved, reportable non-bevaring type without tonnes blocks (`mass`) and is
/// never counted as zero, so the report cannot be `ready` while incomplete.
/// Contaminated tonnes are never merged into a clean fraction. Rows are in
/// code, then waste-hierarchy, then clean-before-contaminated order; the
/// blocking list is in input order.
FractionReport fractions_by_eak(const std::vector<TypeTotals> &);
std::string_view eak_fraction_name(std::string_view code); // "" when unknown
std::map<std::uint32_t, std::uint32_t>
majority_room(const std::vector<std::uint32_t> &instance_labels,
              const std::vector<std::uint32_t> &room_labels);
std::vector<double> redistribute_quantity(const std::vector<double> &current,
                                          double total);
std::string part_code(int number);   // 1 -> "RX-001"
std::string sample_code(int number); // 1 -> "P-01"

} // namespace reusex::core
