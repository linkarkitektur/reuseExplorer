// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "export.hpp"
#include "export/colmap.hpp"
#include "export/csv.hpp"
#include "export/e57.hpp"
#include "export/materialepas.hpp"
#include "export/pdf.hpp"
#include "export/ply.hpp"
#include "export/rhino.hpp"
#include "export/semantic_images.hpp"
#include "export/speckle.hpp"

void setup_subcommand_export(CLI::App &app,
                             std::shared_ptr<RuxOptions> global_opt) {
  auto *sub = app.add_subcommand("export", "Export data to external formats");

  sub->footer(R"(
DESCRIPTION:
  Parent command for exporting data from ReUseX project databases to
  external formats. Supports point cloud formats (PLY, E57), CAD (Rhino 3DM),
  web platforms (Speckle), and material passport interchange format (JSON).

SUBCOMMANDS:
  ply              Export point cloud to PLY format (with optional filter)
  e57              Export point cloud to E57 format (with optional filter)
  rhino            Export labeled point cloud to Rhino 3DM format
  speckle          Export point cloud or mesh to Speckle platform
  materialepas     Export material passports to JSON file
  pdf              Export the Ressourcekortlægning PDF report (or its data)
  csv              Export all elements (components + passports) to CSV
  semantic-images  Export segmentation images as Glasbey-colored PNGs
  colmap           Export sensor frames as a COLMAP sparse model (MVS/3DGS)

EXAMPLES:
  rux export ply -o cloud.ply                # Export point cloud to PLY
  rux export ply -f 'room_labels in [1]'     # Export filtered points
  rux export e57 -o cloud.e57               # Export point cloud to E57
  rux export rhino -o model.3dm             # Export to Rhino CAD
  rux export speckle -p PROJECT_ID          # Upload to Speckle
  rux export materialepas -o data.json      # Export passports
  rux export pdf -o report.pdf              # Ressourcekortlægning PDF
  rux export csv -o elements.csv            # Export all elements to CSV
  rux export semantic-images -o ./labels    # Export colored label images

NOTES:
  - Use 'rux export <subcommand> --help' for detailed options
  - Export typically performed after processing pipeline completes
  - Multiple export formats can be used for different workflows
)");

  setup_subcommand_export_ply(*sub, global_opt);
  setup_subcommand_export_e57(*sub, global_opt);
  setup_subcommand_export_materialepas(*sub, global_opt);
  setup_subcommand_export_pdf(*sub, global_opt);
  setup_subcommand_export_csv(*sub, global_opt);
  setup_subcommand_export_rhino(*sub, global_opt);
  setup_subcommand_export_semantic_images(*sub, global_opt);
  setup_subcommand_export_speckle(*sub, global_opt);
  setup_subcommand_export_colmap(*sub, global_opt);

  sub->require_subcommand(1);
}
