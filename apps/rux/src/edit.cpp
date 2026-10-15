// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "edit.hpp"
#include "edit/downsample.hpp"
#include "edit/perturb_poses.hpp"
#include "edit/plan.hpp"

void setup_subcommand_edit(CLI::App &app,
                           std::shared_ptr<RuxOptions> global_opt) {
  auto *sub = app.add_subcommand(
      "edit", "Edit clouds, poses and the plan stored in the project");

  sub->footer(R"(
DESCRIPTION:
  Parent command for cloud-level edits that transform an existing cloud
  in the project database into another cloud of the same type. Edits
  read from and write to the project specified by -p/--project.

SUBCOMMANDS:
  downsample       Voxel-grid downsample (keeps all sibling clouds in sync)
  perturb-poses    Inject seeded synthetic trajectory drift (benchmark tool)
  plan             Import / export the solid-model plan as JSON

EXAMPLES:
  rux edit downsample -r 0.05
  rux edit downsample -i cloud -o cloud_5cm -r 0.05
  rux edit downsample -r 0.05 --only-primary --force-desync
  rux edit perturb-poses --seed 1 --drift-scale 1.0 --yes
  rux edit plan --import ref.plan.json --dry-run --export ref.plan.json

NOTES:
  - Use 'rux edit <subcommand> --help' for detailed options
  - Edits operate in place by default; pass -o/--output to write a new cloud
  - Downsample transforms every index-aligned sibling cloud with the same
    voxel partition so derived products never silently desynchronize
  - perturb-poses is DESTRUCTIVE and exists for the drift benchmark (#338);
    run it on a copy of a project, never on an original capture
)");

  setup_subcommand_edit_downsample(*sub, global_opt);
  setup_subcommand_edit_perturb_poses(*sub, global_opt);
  setup_subcommand_edit_plan(*sub, global_opt);

  sub->require_subcommand(1);
}
