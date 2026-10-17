// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

/// @file
/// The spaces as a room prior for the mesh's cell complex (reusex-6t4).
///
/// compute_room_probabilities() reads each cell's room from rays cast inside
/// it. A pocket the scan never saw from inside (house2's stair hall under
/// the stair) only sees back faces -- the treads from below, the
/// neighbouring rooms' walls from behind -- and reads as outside although
/// its plan footprint lies inside a stored space. apply_space_prior() blends
/// the space's room into the ray-cast probabilities of the cells inside a
/// space's footprint and storey height, as evidence next to the rays:
///
///   p' = (1 - w c) p + w c (e e_out + (1 - e) e_L)
///
/// with w the prior weight, c the share of the cell's sample points (its
/// centroid and the points halfway to its corners, the ray origins) that
/// lie inside the space, e the share of the cell's rays that escaped the
/// scan (CellComplex "c:escape_probability"), e_out outside and e_L the
/// space's room. Only the rays that hit something are re-weighed: a back
/// face may be an occluder (the stair above a pocket), a ray into the open
/// says outside whatever the space says. Below w = 0.5 a cell the rays call
/// outside with certainty (p_out = 1) stays outside; the prior decides only
/// where the rays are unsure. Cells in a wall's slab are left alone.
///
/// Measured (scripts/bench-solid.sh rows storeys..spaces,mesh, prior off ->
/// w = 0.3, 2026-10-17): house2 fit_mean 0.2116 -> 0.2039 m, within 5 cm
/// 0.179 -> 0.191, unsupported mesh 0.218 -> 0.214 (a room the rays left
/// outside is meshed); house1 identical; NewOffice fit 0.1923 -> 0.1920,
/// unsupported 0.1098 -> 0.1095. w = 0.45 meshes more of house2 (fit
/// 0.2014) but its unsupported share rises to 0.222. At w = 0.45, also
/// pulling wall-slab cells in cost NewOffice 1.2 cm fit and house1 1.0 to
/// 1.7 points of unsupported mesh (rooms merged through their walls), and
/// re-weighing the escaped rays too cost house1 0.7 and house2 0.2 points
/// (cells outside the building under a leaky polygon). House2's stair
/// pocket (reusex-6t4) is still outside: wall-line strips cross it (see
/// apply_space_prior()).

#include "reusex/reconstruction/CellComplex.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cstddef>
#include <vector>

namespace reusex::geometry {

/// One stored space's footprint, in its storey's level frame.
struct SpaceFootprint {
  /// The space's label in the "rooms" cloud (SpaceRecord::room_label).
  unsigned int room_label = 0;
  /// Outer ring and holes in the level frame (open rings, >= 3 points).
  std::vector<Eigen::Vector2d> outer;
  std::vector<std::vector<Eigen::Vector2d>> holes;
  /// World -> level frame of the space's storey (LevelFrame).
  Eigen::Affine3d world_to_level = Eigen::Affine3d::Identity();
  /// The storey's floor and ceiling in the level frame [m].
  double z_floor = 0.0, z_ceiling = 0.0;

  /// True when the world point @p p lies inside the footprint (outer ring,
  /// not in a hole) and between z_floor and z_ceiling.
  [[nodiscard]] bool contains(const Eigen::Vector3d &p) const;
};

/// What apply_space_prior() did.
struct SpacePriorStats {
  size_t spaces = 0;         ///< footprints used
  size_t spaces_skipped = 0; ///< no such room label in the rooms cloud
  size_t cells = 0;          ///< cells with at least one sample inside
  double volume = 0.0;       ///< their volume, weighted by coverage [m^3]
  /// Cells the prior turned from outside (p_out the largest) to a room.
  size_t cells_flipped = 0;
  double volume_flipped = 0.0; ///< their volume [m^3]
};

/// Blend each footprint's room into the "c:room_probabilities" of the cells
/// inside it (see the file comment). Needs compute_room_probabilities() to
/// have run (it records the room labels, CellComplex::room_labels). A
/// footprint whose room_label is not one of those (the spaces are older
/// than the rooms cloud) is skipped and counted. A cell inside several
/// footprints takes the one holding most of its samples. A cell inside a
/// wall's slab (CellData::wall_ids not empty: between the two faces of a
/// pair, along the pair's whole line) keeps its rays: a space drawn across a
/// wall must not merge the rooms either side. @p weight <= 0 or
/// no footprints leave the probabilities as they are.
/// @throws std::invalid_argument for a weight above 1.
auto apply_space_prior(CellComplex &cc,
                       const std::vector<SpaceFootprint> &spaces, double weight)
    -> SpacePriorStats;

} // namespace reusex::geometry
