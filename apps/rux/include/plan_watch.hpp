// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// The ground-truth authoring loop's shared pieces (reusex-yf4.2): a plan file
// that keeps its last good version while it is being edited, and the text
// `rux render --watch` prints about it. Used by `rux render --watch` and
// `rux view --plan --watch`.

#include <reusex/core/solid_model.hpp>
#include <reusex/reconstruction/plan_support.hpp>

#include <filesystem>
#include <optional>
#include <string>

namespace rux {

/// A solid-model plan file (JSON or YAML by extension) being edited.
class PlanFile {
    public:
  explicit PlanFile(std::filesystem::path path) : path_(std::move(path)) {}

  /// Re-read the file. When it parses, it becomes model(), error() is
  /// cleared and generation() counts up. When it does not, model() keeps
  /// the last good plan (std::nullopt before the first one) and error() is
  /// the reader's message, which names the offending element
  /// (io::solid_model_from_file).
  /// @return whether the file was accepted.
  bool reload();

  [[nodiscard]] const std::optional<reusex::core::SolidModel> &model() const {
    return model_;
  }
  [[nodiscard]] const std::string &error() const { return error_; }
  /// Accepted reads so far (0 = none yet).
  [[nodiscard]] int generation() const { return generation_; }
  [[nodiscard]] const std::filesystem::path &path() const { return path_; }

    private:
  std::filesystem::path path_;
  std::optional<reusex::core::SolidModel> model_;
  std::string error_;
  int generation_ = 0;
};

/// "1 storey, 24 walls, 9 spaces, 6 openings".
std::string plan_counts(const reusex::core::SolidModel &plan);

/// The wall-support lines of a watch pass: the summary, then one line per
/// weak wall and one per supported wall whose face offset is 3 cm or more
/// (plan order each). Deterministic.
std::string
format_plan_support(const reusex::geometry::PlanSupportReport &report,
                    const reusex::geometry::PlanSupportOptions &opt);

} // namespace rux
