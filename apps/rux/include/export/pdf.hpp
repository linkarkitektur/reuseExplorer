// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "global-params.hpp"

#include <CLI/CLI.hpp>

#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>

namespace fs = std::filesystem;

struct SubcommandExportPdfOptions {
  fs::path output_path = fs::current_path() / "report.pdf";
  /// --dump-data: write data.json + images/ here instead of a PDF.
  fs::path dump_dir;
  std::optional<std::int64_t> resource_template_id;
  /// --template: Typst file to use instead of the embedded report.typ.
  fs::path template_path;
};

void setup_subcommand_export_pdf(CLI::App &parent,
                                 std::shared_ptr<RuxOptions> global_opt);
int run_subcommand_export_pdf(SubcommandExportPdfOptions const &opt,
                              const RuxOptions &global_opt);
