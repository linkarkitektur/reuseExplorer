// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

/// @file
/// Walls with a measured thickness and a bounded wall graph (solid-model
/// WP3, reusex-z1v.5). Sub-steps as free, unit-testable functions:
///
///  1. axis_families(): the directions walls run in, per storey (W3.1).
///  2. wall_faces(): faces (one-sided sheets of wall points) and their runs
///     along the axis, from the offset profile of each family (W3.2).
///  3. pair_faces(), measure_thickness(), measure_walls(): which faces are
///     the two sides of one wall, its thickness with a bootstrap CI, its
///     lean, and whether it is external (W3.3).
///  4. join_walls(): the bounded wall graph -- snapping, collinear bridging,
///     corner and tee joins, seen share (W3.4); detect_walls() runs 1-4 on
///     one storey and wall_records() turns the result into core records.
///
/// Everything is in the level frame of one storey (core::StoreyRecord::
/// level_transform): z up, the storey's floor at its z_floor. Inputs are
/// records (core::PlaneClassRecord, core::StoreyRecord) and clouds, never
/// segmentation types: reconstruction must not link segmentation (STANDARDS
/// §1); the stage that composes them is pipeline's.
///
/// Deterministic (STANDARDS §6): no randomness, inputs visited in index
/// order, every iteration bounded.

#include "reusex/core/solid_model.hpp"
#include "reusex/types/point_types.hpp"

#include <Eigen/Core>

#include <limits>
#include <map>
#include <utility>
#include <vector>

namespace reusex::geometry {

/// Every tunable of the wall stage (STANDARDS §4).
struct WallOptions {
  // --- W3.1 axis families -------------------------------------------------
  /// Extra families allowed beyond the dominant orthogonal pair (decision D6:
  /// buildings are mostly orthogonal, but not always).
  int max_families = 2;
  /// A vertical-plane normal farther than this from both axes of the
  /// dominant pair is a candidate for an extra family; candidates whose
  /// angles are farther apart than this start a new cluster [deg]. Also the
  /// inlier window when the dominant pair is refined.
  double family_angle_deg = 5.0;
  /// An extra family needs at least this share of the total vertical-plane
  /// inlier mass.
  double min_family_share = 0.05;
  /// Use only planes whose role is "wall" for the families (falling back to
  /// every vertical plane when the storey has none).
  bool wall_role_only = true;
  /// Refinements of the dominant pair over its own inliers (bounded). The
  /// inlier window starts at family_angle_deg and halves each round down to
  /// family_refine_deg [deg].
  int family_refine_iterations = 4;
  double family_refine_deg = 1.0;

  // --- W3.2 wall faces ---------------------------------------------------
  /// A wall point belongs to the family whose axis its plane normal is
  /// nearest to, if within this angle [deg]; otherwise it is no evidence.
  double assign_angle_deg = 10.0;
  /// Square a wall plane within this of its family axis onto the axis,
  /// about the plane's centroid (W3.4's squaring, done on the evidence so a
  /// slightly drifted wall still gives one sharp profile peak) [deg].
  /// Points with no plane id are never squared.
  double square_deg = 8.0;
  /// Use the sign of the plane normal (it faces the sensor that saw it,
  /// CONTRACTS.md `clouds`): points seen from +d and from -d get separate
  /// offset profiles, so the two faces of a thin wall never merge. Off for
  /// legacy clouds whose normals faced the world origin.
  bool use_normal_sign = true;
  /// Offset-profile bin [m]; bins are anchored at the level-frame origin.
  double bin_m = 0.01;
  /// A profile peak (after the [1,2,3,2,1]/9 smoothing) is at least this
  /// many points per bin...
  double min_peak_points = 10.0;
  /// ...stands out from the deeper of its two bases by at least this share
  /// of its height (topographic prominence)...
  double min_peak_prominence_share = 0.3;
  /// ...and is at least this far from a higher accepted peak [m].
  double min_peak_separation_m = 0.03;
  /// Half-width of the slab of points a face takes around its peak [m]
  /// (narrowed to half the distance to a neighbouring peak).
  double face_window_m = 0.03;
  /// A cell_m wide column of a face is evidence ("solid") when its points
  /// cover at least this share of the storey's cell rows: a wall shows over
  /// much of its height, a row of furniture ends at one offset does not.
  double min_column_share = 0.3;
  /// A face splits into runs along the axis where no solid column is seen
  /// for longer than this [m]...
  double run_gap_m = 0.6;
  /// ...and runs shorter than this [m] or with fewer points are dropped.
  double min_run_m = 0.5;
  int min_run_points = 30;
  /// Side of the (s, z) occupancy cells [m], anchored at the frame origin.
  double cell_m = 0.25;
  /// wall_points(): points this far below the storey's floor / above its
  /// ceiling still belong to it [m].
  double storey_margin_m = 0.1;

  // --- W3.3 pairing, thickness, external test -----------------------------
  /// Two faces are the sides of one wall when they are this far apart [m]...
  double min_gap_m = 0.04, max_gap_m = 0.45;
  /// ...share at least this length along the axis [m]...
  double min_shared_m = 0.5;
  /// ...the emptiest 1 cm bin between them holds at most this share of the
  /// weaker face's peak bin (all the storey's points counted)...
  double max_valley_share = 0.15;
  /// ...and at least this share of the (s, z) cells seen on either face over
  /// the shared stretch are seen on both.
  double min_co_visibility = 0.3;
  /// Two faces closer than this whose normals point at each other cannot
  /// bound a room a sensor looked into: they are one wall seen from both
  /// sides by scans that drifted apart. Paired as a wall at their middle,
  /// thickness assumed [m].
  double max_crossed_gap_m = 0.15;
  /// A cell enters the thickness fit with at least this many points.
  int min_cell_points = 3;
  /// Theil-Sen residuals beyond this many robust sigmas drop a cell...
  double outlier_sigma_k = 3.0;
  /// ...with the robust sigma (1.4826 x MAD) at least this [m].
  double min_robust_sigma_m = 0.005;
  /// Bootstrap resamples (over cells) for the thickness CI; seeded
  /// (STANDARDS §6).
  int bootstrap_samples = 200;
  unsigned seed = 0;
  double ci_level = 0.95;
  /// Body assumed behind a wall seen from one side only, for its seen share,
  /// the external test and the graph's collinearity [m]. Its axis is the
  /// seen face and its record's thickness -1 (unknown; reusex-v8p).
  double default_thickness_m = 0.2;
  /// External test: a strip from near to far beyond each face [m]...
  double external_strip_near_m = 0.1, external_strip_far_m = 0.6;
  /// ...below this height above the floor [m]...
  double external_max_z_m = 1.2;
  /// ...holds fewer non-wall points per m^2 than this on one side: no room
  /// there, the wall is external.
  double min_room_density = 20.0;

  // --- W3.4 wall graph ------------------------------------------------------
  /// Snap the seen faces to the median of the raw wall points within this
  /// of them: a wide pass, then a narrow one [m].
  double snap_wide_m = 0.10, snap_narrow_m = 0.04;
  /// Collinear walls of one family (a face both saw, else the axes, within
  /// bridge_offset_m) with at most bridge_gap_m between them merge into one
  /// wall (a doorway, a window, an occluded stretch) [m]...
  double bridge_gap_m = 1.5;
  double bridge_offset_m = 0.05;
  /// ...unless both thicknesses are measured and differ by more than this;
  /// then they meet end to end (a collinear join) [m].
  double bridge_thickness_m = 0.05;
  /// Two one-sided walls seen from the same side whose faces lie within
  /// max_gap_m of each other, within bridge_gap_m along the axis, the
  /// shorter at most this long, are one wall with a recess (window niches
  /// between piers): they merge onto the face with more points [m].
  double recess_max_length_m = 1.0;
  /// Two non-parallel walls whose nearer ends both lie within corner_m of
  /// the crossing of their axes meet there (a corner) [m]...
  double corner_m = 0.9;
  /// ...one whose end lies within tee_m of a crossing that lies on the other
  /// wall (within tee_on_wall_m of its extent) ends there (a tee) [m].
  double tee_m = 2.2;
  double tee_on_wall_m = 0.1;
  /// A wall whose face area (25 cm cells over its length and the storey
  /// height) is seen less than this is flagged (WallSegment::low_seen).
  double min_seen_share = 0.5;
};

/// One direction family: walls whose normal is (anti)parallel to @ref d.
struct AxisFamily {
  int index = 0;
  /// Unit horizontal wall normal in the level frame; angle in [0, 180) deg.
  Eigen::Vector2d d = Eigen::Vector2d::UnitX();
  double angle_deg = 0.0; ///< atan2(d.y, d.x) in degrees, [0, 180)
  double weight = 0.0;    ///< inlier mass of the normals assigned to it
  bool dominant = false;  ///< one of the orthogonal pair (index 0 and 1)
};

/// A horizontal plane normal with a weight (inlier count).
struct WeightedNormal {
  Eigen::Vector2d n = Eigen::Vector2d::UnitX(); ///< need not be unit length
  double weight = 1.0;
};

/// The axis families of a set of horizontal wall normals (W3.1).
///
/// The dominant orthogonal pair comes from the weighted circular mean of 4θ
/// (θ = the normal's angle), so normals at θ, θ+90 and θ+180 all vote for
/// the same pair. It is seeded at the weighted mode (the angle mod 90 with
/// the most mass within family_angle_deg) and refined as the 4θ mean of the
/// normals within a window that narrows from family_angle_deg to
/// WallOptions::family_refine_deg. Normals farther from the pair are
/// sorted by angle (mod 180) and clustered where neighbours are at most
/// family_angle_deg apart; a cluster becomes an extra family when its weight
/// clears WallOptions::min_family_share of the total, heaviest first, up to
/// WallOptions::max_families. Family 0 is the dominant axis with the smaller
/// angle in [0, 90), family 1 is it + 90; extra families follow in order of
/// weight. Each family's weight is the mass of the normals nearest to it.
///
/// Returns an empty vector when there is no normal with positive weight.
std::vector<AxisFamily>
axis_families(const std::vector<WeightedNormal> &normals,
              const WallOptions &opt = {});

/// The axis families of one storey from the stored plane classes: the
/// vertical planes of @p storey (PlaneClassRecord::storey == storey.idx),
/// their normals (@p plane_normals, index = plane label - 1) rotated into the
/// storey's level frame, weighted by PlaneClassRecord::point_count. With
/// WallOptions::wall_role_only, only role "wall" planes vote unless the
/// storey has none.
///
/// @throws std::invalid_argument when a record's plane label has no normal,
///         or the storey's level_transform is not finite.
std::vector<AxisFamily>
axis_families(const std::vector<core::PlaneClassRecord> &planes,
              const CloudN &plane_normals, const core::StoreyRecord &storey,
              const WallOptions &opt = {});

/// The evidence of one storey, in its level frame (see wall_points()).
struct WallPoints {
  /// Points of wall-role planes: the wall evidence...
  std::vector<Eigen::Vector3d> points;
  /// ...the horizontal part of their plane's normal (index-aligned; need not
  /// be unit length; it faces the sensor that saw the plane)...
  std::vector<Eigen::Vector2d> normals;
  /// ...and their plane label (index-aligned, or empty: no plane ids). Points
  /// of one plane are squared together (WallOptions::square_deg); -1 = none.
  std::vector<int> plane;
  /// Every other point of the storey (floors, furniture, unsegmented): what
  /// the valley and external tests (W3.3) look at.
  std::vector<Eigen::Vector3d> others;
  double z_floor = 0.0, z_ceiling = 0.0; ///< the storey's height range
};

/// One point of a face in its family's coordinates: s = p . t along the axis
/// (t = d rotated +90 deg), z, and the offset o = p . d.
struct FaceSample {
  double s = 0.0, z = 0.0, o = 0.0;
};

/// A cell of the (s, z) occupancy grid: (floor(s / cell_m), floor(z / cell_m))
/// -- anchored at the level-frame origin, so a re-run lands on the same grid.
using FaceCell = std::pair<int, int>;

/// One face: a sheet of wall points seen from one side, at one offset, over
/// one contiguous run along the axis.
struct WallFace {
  int family = 0; ///< AxisFamily::index
  /// +1: seen from +d (its outward normal points along d), -1: from -d,
  /// 0: unknown (WallOptions::use_normal_sign off).
  int side = 0;
  double offset = 0.0;       ///< median offset of its points [m]
  double s0 = 0.0, s1 = 0.0; ///< run along the axis [m]
  double z0 = 0.0, z1 = 0.0; ///< height range of its points [m]
  int point_count = 0;
  /// Smoothed profile height at its peak [points per bin].
  double peak_height = 0.0;
  std::map<FaceCell, int> cells;   ///< points per occupied (s, z) cell
  std::vector<FaceSample> samples; ///< its points, sorted by (s, z, o)
};

/// The faces of a storey (W3.2). Per family, and per side when the normal
/// sign is used, a 1 cm histogram of the wall points' offsets o = p . d
/// (bins anchored at the level-frame origin), smoothed [1,2,3,2,1]/9; each
/// peak with enough height, prominence and separation is a face taking the
/// points within WallOptions::face_window_m of it; a face splits into runs
/// where no solid column (WallOptions::min_column_share of the storey's
/// height seen) occurs for longer than WallOptions::run_gap_m, and runs
/// shorter than WallOptions::min_run_m are dropped. A wall point goes to the
/// family its plane normal is nearest to (within assign_angle_deg); a plane
/// within square_deg of its family is squared onto it about its centroid.
///
/// Sorted by (family, offset, s0, side). Deterministic: the result does not
/// depend on the order of the points.
///
/// @throws std::invalid_argument when @p wp's normals (or non-empty plane
///         ids) are not index-aligned with its points.
std::vector<WallFace> wall_faces(const WallPoints &wp,
                                 const std::vector<AxisFamily> &families,
                                 const WallOptions &opt = {});

/// Two faces paired as the sides of one wall over [s0, s1].
struct FacePair {
  int face_lo = -1, face_hi = -1; ///< indices into the faces; lo offset < hi
  double s0 = 0.0, s1 = 0.0;      ///< the shared stretch [m]
  double gap = 0.0;               ///< hi.offset - lo.offset [m]
  double valley_share = 0.0;      ///< emptiest bin between / weaker peak
  double co_visibility = 0.0;     ///< cells seen on both / on either
  /// The faces look at each other (lo seen from +d, hi from -d): drift.
  bool crossed = false;
};

/// A stretch of a face no pair took: a wall seen from one side.
struct FaceSpan {
  int face = -1;
  double s0 = 0.0, s1 = 0.0;
};

struct FacePairing {
  std::vector<FacePair> pairs;   ///< in the order they were taken
  std::vector<FaceSpan> singles; ///< by face index, then s
  int rejected = 0; ///< candidates that failed the valley / co-visibility test
};

/// Pair the faces of one storey (W3.3). Candidates are faces of one family
/// whose gap is within [min_gap_m, max_gap_m] and that share at least
/// min_shared_m along the axis; with known sides, the lower face must be
/// seen from -d and the upper from +d (faces looking at each other bound a
/// room, not a wall -- unless they are within max_crossed_gap_m: a sensor
/// cannot see into so narrow a gap, so it is one wall whose two sides were
/// scanned with drift between them, a "crossed" pair). Greedy, longest shared
/// stretch first (ties: smaller gap, then index): a candidate takes the longest
/// stretch both faces still have free if it passes the empty-valley test (raw
/// counts of every point of the storey, @p wp's wall and other points) and the
/// co-visibility test; what is left of each face (at least min_run_m,
/// min_run_points) becomes a FaceSpan.
FacePairing pair_faces(const std::vector<WallFace> &faces, const WallPoints &wp,
                       const std::vector<AxisFamily> &families,
                       const WallOptions &opt = {});

/// The thickness of a wall from its two faces over [s0, s1].
struct ThicknessFit {
  bool ok = false;        ///< false: a face has fewer than 2 usable cells
  double thickness = 0.0; ///< at mid-height [m]
  double ci_lo = 0.0, ci_hi = 0.0; ///< bootstrap CI (WallOptions::ci_level)
  double lean_deg = 0.0; ///< mean slope of the faces; + leans toward +d
  double offset_lo = 0.0, offset_hi = 0.0; ///< the faces at mid-height [m]
  double z_mid = 0.0;                      ///< the mid-height used [m]
  int cells_lo = 0, cells_hi = 0; ///< cells kept per face after Theil-Sen
};

/// Thickness of the wall between faces @p lo and @p hi over [s0, s1]: per
/// face, the median offset and height of each (s, z) cell with
/// min_cell_points; a Theil-Sen fit offset(z) drops cells beyond
/// outlier_sigma_k robust sigmas (furniture, linings); an OLS fit over the
/// rest gives the face at mid-height (of the heights both faces were seen
/// at) and its slope. Thickness = hi - lo at mid-height, lean = the mean
/// slope, CI = percentiles of the thickness over bootstrap resamples of the
/// cells (seeded: the same inputs give the same CI).
ThicknessFit measure_thickness(const WallFace &lo, const WallFace &hi,
                               double s0, double s1,
                               const WallOptions &opt = {});

/// One wall in its family's coordinates, before the wall graph (W3.4).
struct WallSegment {
  int family = 0;                               ///< AxisFamily::index
  Eigen::Vector2d d = Eigen::Vector2d::UnitX(); ///< the family's normal
  /// The axis, p . d [m]: the centre line of a wall with two faces (or a
  /// crossed pair), the seen face of a one-sided wall (reusex-v8p).
  double axis_offset = 0.0;
  double s0 = 0.0, s1 = 0.0; ///< extent along t = d rotated +90 deg [m]
  /// The faces at mid-height [m]; NaN for a side that was not seen.
  double face_lo = std::numeric_limits<double>::quiet_NaN();
  double face_hi = std::numeric_limits<double>::quiet_NaN();
  int face_lo_index = -1, face_hi_index = -1; ///< into the faces, -1 none
  /// Measured, or default_thickness_m if assumed (a one-sided wall's body
  /// behind its face; its record says -1).
  double thickness = 0.0;
  double thickness_ci_lo = -1.0, thickness_ci_hi = -1.0; ///< -1: none
  double lean_deg = 0.0;
  bool assumed = false;             ///< one side seen: thickness defaulted
  bool is_external = false;         ///< no room evidence beyond one face
  double co_visibility = -1.0;      ///< -1 for a one-sided wall
  int points_lo = 0, points_hi = 0; ///< face points over [s0, s1]
  double seen_share = -1.0;         ///< set by join_walls (W3.4)
  bool low_seen = false;            ///< seen_share below min_seen_share (W3.4)
};

/// The walls of a storey from its faces and their pairing (W3.3): a pair is
/// a wall with a measured thickness (measure_thickness; when it cannot fit,
/// the profile peaks and no CI); a crossed pair a wall halfway between its
/// faces with an assumed thickness; a one-sided stretch is an assumed wall
/// whose axis is the seen face (reusex-v8p) and whose body, default_thickness_m
/// deep, lies behind it (away from the side it was seen from; with no
/// normal sign, away from the side with more floor-level evidence). External
/// test: a strip from
/// external_strip_near_m to external_strip_far_m beyond each face, below
/// external_max_z_m above the floor, holding fewer than min_room_density
/// non-wall points per m^2 on either side.
///
/// Sorted by (family, axis_offset, s0).
///
/// @throws std::invalid_argument when a face names a family that is not in
///         @p families.
std::vector<WallSegment> measure_walls(const std::vector<WallFace> &faces,
                                       const FacePairing &pairing,
                                       const WallPoints &wp,
                                       const std::vector<AxisFamily> &families,
                                       const WallOptions &opt = {});

/// How two walls meet (core::WallJoinRecord::kind).
enum class WallJoinKind { corner, tee, collinear };
/// Where on a wall a join is (core::WallJoinRecord::at_a / at_b).
enum class WallEnd { start, end, path };

/// A join between walls @ref a and @ref b (indices into WallGraph::walls).
struct WallJoin {
  int a = -1, b = -1;
  WallJoinKind kind = WallJoinKind::corner;
  WallEnd at_a = WallEnd::start, at_b = WallEnd::start;
};

struct WallGraph {
  std::vector<WallSegment> walls;
  std::vector<WallJoin> joins;
  int low_seen = 0; ///< walls flagged WallSegment::low_seen
};

/// The bounded wall graph of a storey (W3.4):
///  1. squaring: already done on the evidence (wall_faces squares every
///     wall plane within square_deg onto its family axis), so every wall is
///     parallel to its family;
///  2. snapping: each seen face moves to the median offset of the raw wall
///     points seen from its side within snap_wide_m, then snap_narrow_m;
///  3. bridging: collinear walls with a gap up to bridge_gap_m merge
///     (a measured thickness wins over an assumed one); when both are
///     measured and differ by more than bridge_thickness_m they meet in the
///     middle of the gap instead (a collinear join); a short one-sided wall
///     seen from the same side as a neighbour and standing within max_gap_m
///     behind or before it is a recess of that wall and merges into it
///     (recess_max_length_m);
///  4. corners and tees: non-parallel pairs, nearest crossing first; a
///     corner moves both ends to the crossing of the axes, a tee moves one
///     end onto the other wall's axis; each end joins at most once and a
///     move never leaves a wall shorter than min_run_m;
///  5. seen share: the share of the wall's 25 cm (s, z) cells over its
///     length and the storey height with wall evidence within its body;
///     below min_seen_share it is flagged.
/// Deterministic: walls are visited in a fixed order (sorted by family,
/// s0, axis offset), crossings by distance then index.
WallGraph join_walls(std::vector<WallSegment> walls,
                     const std::vector<AxisFamily> &families,
                     const WallPoints &wp, const WallOptions &opt = {});

/// Everything the wall sub-steps found on one storey.
struct WallsResult {
  std::vector<AxisFamily> families;
  std::vector<WallFace> faces;
  FacePairing pairing;
  WallGraph graph;
};

/// Steps 1-4 on one storey: axis_families() from the plane classes,
/// wall_points() from the clouds, then wall_faces(), pair_faces(),
/// measure_walls() and join_walls(). Logs (reusex::warn) when a storey
/// yields no family or no wall, with the numbers.
WallsResult
detect_walls(const Cloud &cloud, const CloudL &planes,
             const CloudN &plane_normals,
             const std::vector<core::PlaneClassRecord> &plane_classes,
             const core::StoreyRecord &storey, const WallOptions &opt = {});

/// The walls and joins of @p graph as core records in @p storey's level
/// frame. The guid is the stable key W3.5 hashes into a deterministic GUID:
/// "walls/<storey idx>/<x0>,<y0>,<x1>,<y1>" with the axis endpoints rounded
/// to 5 cm (a "#n" suffix keeps two walls that round alike apart). z_base /
/// height are the storey's floor and height; a one-sided wall has its axis
/// on its seen face and thickness -1 (unknown, reusex-v8p), any other
/// assumed wall the default thickness; assumed walls carry no CI; a flagged
/// wall carries
/// {"low_seen_share": true} as metadata.
std::pair<std::vector<core::WallRecord>, std::vector<core::WallJoinRecord>>
wall_records(const WallGraph &graph, const core::StoreyRecord &storey);

/// The evidence of one storey from the clouds: every finite point of
/// @p cloud whose level-frame height lies within the storey (with
/// WallOptions::storey_margin_m), taken into @p storey's level frame; a point
/// whose plane (@p planes, label = plane_label) has role "wall" in
/// @p plane_classes is wall evidence with its plane's normal
/// (@p plane_normals, index = label - 1), any other point goes to
/// WallPoints::others.
///
/// @throws std::invalid_argument when @p planes is not index-aligned with
///         @p cloud, a wall plane has no normal, or the storey's
///         level_transform is not finite.
WallPoints wall_points(const Cloud &cloud, const CloudL &planes,
                       const CloudN &plane_normals,
                       const std::vector<core::PlaneClassRecord> &plane_classes,
                       const core::StoreyRecord &storey,
                       const WallOptions &opt = {});

} // namespace reusex::geometry
