// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "create.hpp"
#include "create/annotate.hpp"
#include "create/annotate_panorama.hpp"
#include "create/attributes.hpp"
#include "create/classify.hpp"
#include "create/clouds.hpp"
#include "create/dense.hpp"
#include "create/gsplat.hpp"
#include "create/instances.hpp"
#include "create/material.hpp"
#include "create/materials.hpp"
#include "create/mesh.hpp"
#include "create/openings.hpp"
#include "create/planes.hpp"
#include "create/project.hpp"
#include "create/rooms.hpp"
#include "create/segment_frame.hpp"
#include "create/solids.hpp"
#include "create/spaces.hpp"
#include "create/storeys.hpp"
#include "create/survey.hpp"
#include "create/texture.hpp"
#include "create/walls.hpp"
#include "create/windows.hpp"

void setup_subcommand_create(CLI::App &app,
                             std::shared_ptr<RuxOptions> global_opt) {
  auto *sub = app.add_subcommand("create", "Create derived data from project");

  sub->footer(R"(
DESCRIPTION:
  Parent command for creating derived data products from ReUseX project
  databases. Process sensor data through reconstruction, segmentation,
  annotation, and mesh generation pipelines.

SUBCOMMANDS:
  clouds       Reconstruct 3D point clouds from depth images
  dense        Generate dense point cloud via OpenMVS Multi-View Stereo
  annotate     Run ML inference on sensor frames
  planes       Detect and segment planar surfaces
  rooms        Segment rooms using Leiden clustering
  instances    Separate labels into spatial instances
  storeys      Detect storeys and the measured vertical
  classify     Classify planes: wall, floor, ceiling, furniture, ...
  walls        Walls with a measured thickness and their joins
  openings     Doors, windows and openings in the walls
  spaces       Spaces: band rooms as polygons against the walls
  solids       Typed solids: walls minus openings, slabs, space volumes
  materials    Create a material passport per instance
  survey       Fill the Ressourcekortlægning from instances
  mesh         Generate watertight mesh from planes
  texture      Apply textures to mesh from sensor frames
  gsplat       Train a 3D Gaussian Splatting model (CUDA)
  project      Project 2D labels onto 3D point cloud
  material     Create blank material passport
  windows      (deprecated: use openings) Window building components
  attributes   Extract per-instance attributes with a vision-language model

TYPICAL WORKFLOW:
  1. rux import rtabmap scan.db        # Import sensor data
  2. rux create clouds                 # Reconstruct geometry
  3. rux create planes                 # Segment surfaces
  4. rux create rooms                  # Segment rooms
  5. rux create mesh                   # Generate mesh

NOTES:
  - Use 'rux create <subcommand> --help' for detailed options
  - Commands read from and write to project database
  - Most commands depend on earlier pipeline stages
)");

  // Register all subcommands (ordered by pipeline flow)
  setup_subcommand_create_clouds(*sub, global_opt);
  setup_subcommand_create_dense(*sub, global_opt);
  setup_subcommand_create_annotate(*sub, global_opt);
  setup_subcommand_create_annotate_panorama(*sub, global_opt);
  setup_subcommand_create_material(*sub, global_opt);
  setup_subcommand_create_project(*sub, global_opt);
  setup_subcommand_create_planes(*sub, global_opt);
  setup_subcommand_create_rooms(*sub, global_opt);
  setup_subcommand_create_instances(*sub, global_opt);
  setup_subcommand_create_storeys(*sub, global_opt);
  setup_subcommand_create_classify(*sub, global_opt);
  setup_subcommand_create_walls(*sub, global_opt);
  setup_subcommand_create_openings(*sub, global_opt);
  setup_subcommand_create_spaces(*sub, global_opt);
  setup_subcommand_create_solids(*sub, global_opt);
  setup_subcommand_create_materials(*sub, global_opt);
  setup_subcommand_create_survey(*sub, global_opt);
  setup_subcommand_create_mesh(*sub, global_opt);
  setup_subcommand_create_texture(*sub, global_opt);
  setup_subcommand_create_windows(*sub, global_opt);
  setup_subcommand_create_attributes(*sub, global_opt);
  setup_subcommand_create_gsplat(*sub, global_opt);
  setup_subcommand_create_segment_frame(*sub, global_opt);

  sub->require_subcommand(1);
}
