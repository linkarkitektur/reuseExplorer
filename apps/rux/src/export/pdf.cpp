// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "export/pdf.hpp"
#include "exit_status.hpp"
#include "global-params.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/report_generator.hpp>
#include <reusex/visualize/render_view.hpp>

#include <opencv2/imgcodecs.hpp>

#include <spdlog/spdlog.h>

#include <fstream>
#include <stdexcept>

void setup_subcommand_export_pdf(CLI::App &parent,
                                 std::shared_ptr<RuxOptions> global_opt) {
  auto opt = std::make_shared<SubcommandExportPdfOptions>();
  auto *sub = parent.add_subcommand(
      "pdf", "Export the Ressourcekortlægning PDF report (or its data)");

  sub->footer(R"(
DESCRIPTION:
  Renders the Ressourcekortlægning report with Typst (needs `typst` on PATH)
  from the project's survey, material passports and, optionally, a resource
  template's Ressourcetabel. --dump-data writes the report's input instead
  (data.json + images/), exactly as the PDF run sees it, for designing the
  template: `nix develop .#report` then `report-watch DIR`.

EXAMPLES:
  rux -p x.rux export pdf -o report.pdf
  rux -p x.rux export pdf -o report.pdf --resource-template 2
  rux -p x.rux export pdf -o report.pdf --template my-report.typ
  rux -p x.rux export pdf --dump-data shots/report-data

NOTES:
  - Template: --template, else $REUSEX_REPORT_TEMPLATE, else the built-in one
  - Data format: docs/guides/report-template.md
  - Cover image: the project's report/cover_image (a photo, 360° or upload,
    set through ruxd's /reports/cover), else an orbit render of the cloud
)");

  auto *out = sub->add_option("-o,--output", opt->output_path,
                              "Output PDF path (default: report.pdf)")
                  ->default_val(opt->output_path);
  sub->add_option("--resource-template", opt->resource_template_id,
                  "Add a Ressourcetabel through this resource template id");
  auto *tmpl = sub->add_option("--template", opt->template_path,
                               "Typst template file to use instead of the "
                               "built-in report.typ");
  auto *dump = sub->add_option("--dump-data", opt->dump_dir,
                               "Write data.json + images/ to this directory "
                               "instead of a PDF");
  dump->excludes(out);
  dump->excludes(tmpl);

  sub->callback([opt, global_opt]() {
    spdlog::trace("calling export pdf subcommand");
    rux::finish(run_subcommand_export_pdf(*opt, *global_opt));
  });
}

namespace {
/// The Forside's fallback image: the whole cloud from the orbit ring, wide —
/// the same shot the GUI preview showed (PagePreview's COVER_RENDER).
std::vector<std::uint8_t> render_cover(const reusex::ProjectDB &db) {
  namespace viz = reusex::visualize;
  viz::RenderOptions o;
  o.layers = {viz::Layer::cloud};
  o.view = viz::ViewPreset::orbit;
  o.orbit_index = 1;
  o.width = 1200;
  o.height = 560;
  const cv::Mat image = viz::render_view(db, o);
  std::vector<std::uint8_t> png;
  if (image.empty() || !cv::imencode(".png", image, png))
    return {};
  return png;
}
} // namespace

int run_subcommand_export_pdf(SubcommandExportPdfOptions const &opt,
                              const RuxOptions &global_opt) {
  try {
    reusex::ReportOptions report;
    report.resource_template_id = opt.resource_template_id;
    report.template_path = opt.template_path;
    report.cover_fallback = render_cover;

    if (opt.dump_dir.empty()) {
      // Fail before the (slow) typst run, not after it.
      const auto parent = fs::absolute(opt.output_path).parent_path();
      if (!fs::is_directory(parent)) {
        spdlog::error("Output directory does not exist: {}", parent.string());
        return RuxError::INVALID_ARGUMENT;
      }
      if (fs::is_directory(opt.output_path)) {
        spdlog::error("Output path is a directory: {}",
                      opt.output_path.string());
        return RuxError::INVALID_ARGUMENT;
      }
      if (!opt.template_path.empty() &&
          !fs::is_regular_file(opt.template_path)) {
        spdlog::error("Template file not found (or not a regular file): {}",
                      opt.template_path.string());
        return RuxError::INVALID_ARGUMENT;
      }
    }

    spdlog::info("Opening project database: {}",
                 global_opt.project_db.string());
    reusex::ProjectDB db(global_opt.project_db, /*readOnly=*/true);

    if (!opt.dump_dir.empty()) {
      reusex::dump_report_data(db, report, opt.dump_dir);
      spdlog::info("Wrote report data to {} (preview: nix develop .#report, "
                   "then report-watch {})",
                   opt.dump_dir.string(), opt.dump_dir.string());
      return RuxError::SUCCESS;
    }

    const auto pdf = reusex::generate_ressourcekortlaegning_pdf(db, report);
    std::ofstream ofs(opt.output_path, std::ios::binary);
    if (!ofs) {
      spdlog::error("Failed to open output file: {}", opt.output_path.string());
      return RuxError::IO;
    }
    ofs.write(reinterpret_cast<const char *>(pdf.data()),
              static_cast<std::streamsize>(pdf.size()));
    if (!ofs) {
      spdlog::error("Failed to write output file: {}",
                    opt.output_path.string());
      return RuxError::IO;
    }
    spdlog::info("Wrote {} ({} bytes)", opt.output_path.string(), pdf.size());
    return RuxError::SUCCESS;
  } catch (const std::out_of_range &e) {
    spdlog::error("Unknown resource template: {}", e.what());
    return RuxError::INVALID_ARGUMENT;
  } catch (const std::exception &e) {
    spdlog::error("export pdf failed: {}", e.what());
    return RuxError::IO;
  }
}
