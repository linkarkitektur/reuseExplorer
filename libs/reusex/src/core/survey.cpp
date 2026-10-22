// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "reusex/core/survey.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <numeric>
#include <tuple>
#include <unordered_map>

namespace reusex::core {
namespace {

template <typename E, std::size_t N>
std::optional<E>
from_table(std::string_view s,
           const std::array<std::pair<E, std::string_view>, N> &t) {
  for (const auto &[e, name] : t)
    if (name == s)
      return e;
  return std::nullopt;
}

template <typename E, std::size_t N>
std::string_view
to_table(E e, const std::array<std::pair<E, std::string_view>, N> &t) {
  for (const auto &[x, name] : t)
    if (x == e)
      return name;
  return {};
}

constexpr std::array<std::pair<Treatment, std::string_view>, 5> kTreatments{{
    {Treatment::bevaring, "bevaring"},
    {Treatment::genbrug, "genbrug"},
    {Treatment::genanvendelse, "genanvendelse"},
    {Treatment::nyttiggoerelse, "nyttiggoerelse"},
    {Treatment::bortskaffelse, "bortskaffelse"},
}};
constexpr std::array<std::pair<ReviewStatus, std::string_view>, 3> kStatuses{{
    {ReviewStatus::queue, "queue"},
    {ReviewStatus::approved, "approved"},
    {ReviewStatus::rejected, "rejected"},
}};
constexpr std::array<std::pair<SampleStage, std::string_view>, 4> kStages{{
    {SampleStage::planlagt, "planlagt"},
    {SampleStage::udtaget, "udtaget"},
    {SampleStage::sendt, "sendt"},
    {SampleStage::svar, "svar"},
}};
constexpr std::array<std::pair<SampleResult, std::string_view>, 3> kResults{{
    {SampleResult::none, ""},
    {SampleResult::ren, "ren"},
    {SampleResult::forurenet, "forurenet"},
}};
constexpr std::array<std::pair<EnvironmentStatus, std::string_view>, 4>
    kEnvironment{{
        {EnvironmentStatus::ren_screening, "ren_screening"},
        {EnvironmentStatus::afventer, "afventer"},
        {EnvironmentStatus::forurenet, "forurenet"},
        {EnvironmentStatus::ren_proevesvar, "ren_proevesvar"},
    }};

/// EAK (European Waste Catalogue) chapter 17 codes a building survey meets.
constexpr std::array<std::pair<std::string_view, std::string_view>, 15> kEak{{
    {"17.01.01", "Beton"},
    {"17.01.02", "Mursten"},
    {"17.01.03", "Tegl og keramik"},
    {"17.01.07", "Blandinger af beton, mursten, tegl"},
    {"17.02.01", "Træ"},
    {"17.02.02", "Glas"},
    {"17.02.03", "Plast"},
    {"17.03.02", "Bitumenblandinger"},
    {"17.04.01", "Kobber, bronze, messing"},
    {"17.04.02", "Aluminium"},
    {"17.04.05", "Jern og stål"},
    {"17.04.07", "Blandede metaller"},
    {"17.06.04", "Isoleringsmateriale"},
    {"17.08.02", "Gipsbaserede materialer"},
    {"17.09.04", "Blandet bygge- og nedrivningsaffald"},
}};

double round2(double x) { return std::round(x * 100.0) / 100.0; }

constexpr std::array<std::pair<ProvenanceKind, std::string_view>, 5>
    kProvenanceKinds{{
        {ProvenanceKind::scan, "scan"},
        {ProvenanceKind::measured, "measured"},
        {ProvenanceKind::calculated, "calculated"},
        {ProvenanceKind::entered, "entered"},
        {ProvenanceKind::suggested, "suggested"},
    }};

} // namespace

std::string_view to_string(ProvenanceKind v) {
  return to_table(v, kProvenanceKinds);
}
std::optional<ProvenanceKind> provenance_kind_from_string(std::string_view s) {
  return from_table(s, kProvenanceKinds);
}

std::string_view to_string(Treatment v) { return to_table(v, kTreatments); }
std::string_view to_string(ReviewStatus v) { return to_table(v, kStatuses); }
std::string_view to_string(SampleStage v) { return to_table(v, kStages); }
std::string_view to_string(SampleResult v) { return to_table(v, kResults); }
std::string_view to_string(EnvironmentStatus v) {
  return to_table(v, kEnvironment);
}

std::string_view treatment_label_da(Treatment t) {
  switch (t) {
  case Treatment::bevaring:
    return "Bevaring";
  case Treatment::genbrug:
    return "Genbrug";
  case Treatment::genanvendelse:
    return "Genanvendelse";
  case Treatment::nyttiggoerelse:
    return "Nyttiggørelse";
  case Treatment::bortskaffelse:
    return "Bortskaffelse";
  }
  return {};
}

std::string_view environment_label_da(EnvironmentStatus e) {
  switch (e) {
  case EnvironmentStatus::ren_screening:
    return "Ren";
  case EnvironmentStatus::afventer:
    return "Afventer prøve";
  case EnvironmentStatus::forurenet:
    return "Forurenet";
  case EnvironmentStatus::ren_proevesvar:
    return "Ren (prøvesvar)";
  }
  return {};
}
std::optional<Treatment> treatment_from_string(std::string_view s) {
  return from_table(s, kTreatments);
}
std::optional<ReviewStatus> review_status_from_string(std::string_view s) {
  return from_table(s, kStatuses);
}
std::optional<SampleStage> sample_stage_from_string(std::string_view s) {
  return from_table(s, kStages);
}
std::optional<SampleResult> sample_result_from_string(std::string_view s) {
  return from_table(s, kResults);
}

void validate_sample_state(const SampleState &s) {
  if (s.result != SampleResult::none && s.stage != SampleStage::svar)
    throw std::invalid_argument(
        "a sample result can only be set once the stage is 'svar' "
        "(answer received), not '" +
        std::string(to_string(s.stage)) + "'");
}

EnvironmentStatus environment_status(const std::vector<SampleState> &linked) {
  if (linked.empty())
    return EnvironmentStatus::ren_screening;
  bool pending = false;
  for (const auto &s : linked) {
    if (s.result == SampleResult::forurenet)
      return EnvironmentStatus::forurenet;
    if (s.stage != SampleStage::svar)
      pending = true;
  }
  return pending ? EnvironmentStatus::afventer
                 : EnvironmentStatus::ren_proevesvar;
}

std::array<double, kTreatmentCount>
circularity_breakdown(const std::vector<TypeTotals> &types) {
  std::array<double, kTreatmentCount> out{};
  for (const auto &t : types)
    if (t.status != ReviewStatus::rejected)
      out[static_cast<std::size_t>(t.treatment)] += t.mass_t.value_or(0.0);
  return out;
}

std::string_view to_string(BlockingReason r) {
  switch (r) {
  case BlockingReason::sample:
    return "sample";
  case BlockingReason::mass:
    return "mass";
  case BlockingReason::review:
    break;
  }
  return "review";
}

bool reportable(ReviewStatus status, EnvironmentStatus environment) {
  return status == ReviewStatus::approved &&
         environment != EnvironmentStatus::afventer;
}

FractionReport fractions_by_eak(const std::vector<TypeTotals> &types) {
  FractionReport report;
  std::map<std::tuple<std::string, int, bool>, double> grouped;
  for (const auto &t : types) {
    if (t.status == ReviewStatus::rejected)
      continue;
    if (!reportable(t.status, t.environment)) {
      const bool awaiting = t.environment == EnvironmentStatus::afventer;
      report.blocking.push_back(BlockingType{
          t.type_id, t.name, t.eak_code, t.treatment, t.mass_t,
          awaiting ? BlockingReason::sample : BlockingReason::review});
      continue;
    }
    if (t.treatment == Treatment::bevaring)
      continue;
    if (!t.mass_t) {
      // An unknown tonnage is not zero tonnes (STANDARDS §5).
      report.blocking.push_back(BlockingType{t.type_id, t.name, t.eak_code,
                                             t.treatment, t.mass_t,
                                             BlockingReason::mass});
      continue;
    }
    grouped[{t.eak_code, static_cast<int>(t.treatment),
             t.environment == EnvironmentStatus::forurenet}] += *t.mass_t;
  }
  for (const auto &[key, mass] : grouped) {
    const auto &[code, treatment, contaminated] = key;
    report.fractions.push_back(
        Fraction{code, std::string(eak_fraction_name(code)),
                 static_cast<Treatment>(treatment), mass, contaminated});
    report.total_t += mass;
  }
  report.blocking_types = report.blocking.size();
  return report;
}

std::string_view eak_fraction_name(std::string_view code) {
  for (const auto &[c, name] : kEak)
    if (c == code)
      return name;
  return {};
}

std::map<std::uint32_t, std::uint32_t>
majority_room(const std::vector<std::uint32_t> &instance_labels,
              const std::vector<std::uint32_t> &room_labels) {
  if (instance_labels.size() != room_labels.size())
    throw std::invalid_argument(
        "majority_room: " + std::to_string(instance_labels.size()) +
        " instance labels but " + std::to_string(room_labels.size()) +
        " room labels — the clouds are not index-aligned");
  std::map<std::uint32_t, std::map<std::uint32_t, std::size_t>> votes;
  for (std::size_t i = 0; i < instance_labels.size(); ++i)
    if (instance_labels[i] != 0 && room_labels[i] != 0)
      ++votes[instance_labels[i]][room_labels[i]];
  std::map<std::uint32_t, std::uint32_t> out;
  for (const auto &[inst, rooms] : votes) {
    // std::map iterates rooms ascending, and only a strictly larger count
    // replaces the leader, so a tie keeps the lower room id.
    std::uint32_t best = 0;
    std::size_t best_n = 0;
    for (const auto &[room, n] : rooms)
      if (n > best_n) {
        best = room;
        best_n = n;
      }
    out[inst] = best;
  }
  return out;
}

std::vector<double> redistribute_quantity(const std::vector<double> &current,
                                          double total) {
  if (total < 0.0)
    throw std::invalid_argument("quantity must not be negative");
  std::vector<double> out(current.size());
  if (current.empty())
    return out;
  const double sum = std::accumulate(current.begin(), current.end(), 0.0);
  double assigned = 0.0;
  for (std::size_t i = 0; i + 1 < current.size(); ++i) {
    const double share = sum > 0.0 ? current[i] / sum
                                   : 1.0 / static_cast<double>(current.size());
    out[i] = round2(total * share);
    assigned += out[i];
  }
  out.back() = std::max(0.0, round2(total - assigned));
  return out;
}

std::string part_code(int number) {
  char buf[16];
  std::snprintf(buf, sizeof buf, "RX-%03d", number);
  return buf;
}

std::string sample_code(int number) {
  char buf[16];
  std::snprintf(buf, sizeof buf, "P-%02d", number);
  return buf;
}

} // namespace reusex::core
