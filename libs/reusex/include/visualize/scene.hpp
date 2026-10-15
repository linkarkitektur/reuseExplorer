// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// The scene builder shared by `rux render` (render_view, headless) and the
// Qt client's 3D workspace (an interactive QVTKOpenGLNativeWidget).
//
// render_view() used to build its VTK scene inline, so the only way for an
// interactive viewer to show "the same thing rux render shows" was to copy
// that code. populate_scene() is the one definition instead: it loads a
// project's layers and adds them to a caller's vtkRenderer, and the camera and
// cut-plane helpers below place the camera the way the presets do. Both front
// ends call them; render_view() is now validation + populate_scene() + camera
// + an offscreen render window.
//
// Header hygiene (STANDARDS §2): VTK types are forward-declared, so a consumer
// that only needs SceneOptions/SceneInfo compiles no VTK header.

#include "reusex/core/solid_model.hpp"
#include "reusex/visualize/render_view.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

class vtkActor;
class vtkPolyData;
class vtkRenderer;

namespace reusex::visualize {

/// Axis-aligned bounds of the geometry a scene draws.
struct SceneBounds {
  std::array<double, 3> min{0, 0, 0};
  std::array<double, 3> max{0, 0, 0};
  bool valid = false;

  void add(double x, double y, double z);
  std::array<double, 3> center() const;
  double extent(int axis) const { return max[axis] - min[axis]; }
  double diagonal() const;
};

/// The categorical colour scale label layers are drawn with.
///
/// The default is the design tokens' `--label-0 … --label-7` scale
/// (Okabe-Ito, colourblind-safe) and `--label-unlabeled`, so a `rux render`,
/// the web viewport and the Qt client colour a label the same way and their
/// legends agree. `tests/unit/visualize/test_scene.cpp` pins these values to
/// `apps/rux/qt/theme/tokens.css` (a vendored copy of rux-frontend's
/// `src/tokens.css`); a design sync that changes the scale fails that test
/// until this table follows.
struct LabelPalette {
  std::vector<std::array<std::uint8_t, 3>> colors; ///< labels 1..N, cyclic
  std::array<std::uint8_t, 3> unlabeled{0, 0, 0};  ///< label 0 (STANDARDS §3)
  /// An out-of-contract label (core::is_out_of_contract_label: a wrapped -1).
  /// Not a class: never one of @ref colors.
  std::array<std::uint8_t, 3> invalid{0, 0, 0};
};

/// The tokens' scale (see LabelPalette).
const LabelPalette &default_label_palette();

/// Sentinel slots of label_palette_slot().
inline constexpr int kUnlabeledSlot = -1;
inline constexpr int kInvalidSlot = -2;

/// Palette slot of a point label: `(label - 1) % size`; kUnlabeledSlot for
/// label 0 or an empty palette; kInvalidSlot for an out-of-contract label
/// (a wrapped -1), which must not borrow a class's colour. The same rule as the
/// web viewport's `labelColorIndex()` — indexing with `label` itself would
/// shift every class by one against the legend. For a semantic cloud (point
/// label = API class + 1, STANDARDS §3.1) the slot is `api % size`, the same
/// colour the 2D segmentation overlays give that class.
int label_palette_slot(std::uint32_t label, std::size_t size);

/// The colour of @p label in @p palette.
std::array<std::uint8_t, 3> label_palette_color(const LabelPalette &palette,
                                                std::uint32_t label);

/// How a large cloud was thinned for drawing.
enum class LodMethod {
  all,           ///< every point is drawn
  morton_prefix, ///< a prefix of a bit-reversed Morton cloud (stratified)
  stride,        ///< every k-th point, evenly over the storage order
};

std::string_view to_string(LodMethod method);

/// Which of @p total stored points to draw within a budget of @p budget.
///
/// A cloud stored in bit-reversed Morton order (`rux create clouds`,
/// `storage_order` "morton_10bit_bitrev", #394/#396) is spatially stratified,
/// so any prefix is a uniform sample: the first @p budget points. Every other
/// order is thinned by an even stride — a plain Morton or insertion-order
/// prefix would cover one corner of the scene, not all of it.
/// @return the storage indices, ascending; empty when every point fits
///         (method == LodMethod::all).
std::vector<std::uint32_t> lod_indices(std::size_t total, std::size_t budget,
                                       std::string_view storage_order,
                                       LodMethod *method = nullptr);

/// Where a plan overlay (SceneOptions::plan) is drawn vertically.
enum class PlanPlacement {
  /// On its storey's floor, in 3D: each element at its level-frame height
  /// (a wall at `z_base`, a space at `z_floor`), mapped to world through the
  /// inverse of its storey's `level_transform`. For perspective and
  /// interactive views.
  level,
  /// Flattened onto a sheet just above everything else drawn, so a view from
  /// above shows the whole plan over the cloud, and exempt from the cut plane
  /// (apply_cut_plane) — what `rux render --view plan` uses.
  above_scene,
};

/// What populate_scene() draws, and how.
struct SceneOptions {
  /// Layers in draw order. Empty is an error.
  std::vector<Layer> layers{Layer::cloud};
  /// Named geometry cloud supplying point positions for every point layer.
  std::string cloud_name = "cloud";
  /// Named mesh drawn by Layer::mesh.
  std::string mesh_name = "mesh";
  /// Point sprite size in pixels. Must be positive.
  double point_size = 2.0;
  /// Paint one instance and dim the rest (see render_view.hpp).
  std::optional<InstanceHighlight> highlight;
  /// Colours of the label layers; std::nullopt = default_label_palette().
  std::optional<LabelPalette> palette;

  /// Draw at most this many points per point layer (0 = all). See
  /// lod_indices() for which.
  std::size_t max_points = 0;
  /// Also build a hidden coarse actor of at most this many points per point
  /// layer (0 = none), for an interactive viewer to show while the camera
  /// moves. Only built when it is smaller than what the full actor draws.
  std::size_t coarse_points = 0;

  /// Frustums drawn at most; posed frames beyond it are skipped evenly.
  std::size_t max_frustums = 300;
  /// Depth of a drawn frustum, in metres.
  double frustum_depth = 0.25;
  std::array<std::uint8_t, 3> frustum_rgb{150, 160, 176};
  std::array<std::uint8_t, 3> panorama_rgb{255, 176, 64};

  /// A solid-model plan (walls, spaces, openings) to draw over the layers —
  /// the visual check of an authored reference (reusex-z1v.2.6,
  /// docs/guides/solid-model-reference-authoring.md). Walls are drawn as
  /// their footprint (axis +/- thickness / 2, a translucent fill and an
  /// outline) plus the axis, spaces as a translucent fill and an outline,
  /// openings as a bar along the host wall's axis with a tick across the wall
  /// at each end. Coordinates are level-frame (core/solid_model.hpp); each
  /// element is mapped to world through the inverse of its storey's
  /// `level_transform` (identity when the plan has no such storey). An
  /// in-memory model rather than a file path, so an interactive client can
  /// draw a plan it is editing; `rux render --plan` reads the file.
  std::optional<core::SolidModel> plan;
  PlanPlacement plan_placement = PlanPlacement::level;
  /// Label each space with its name (its guid when unnamed) and each wall
  /// with its guid, so a render can be checked against the JSON.
  bool plan_labels = true;
  /// Plan colours: walls `--label-2`, spaces `--label-0`, doors `--label-5`,
  /// windows `--label-1`, other openings `--label-3` (default_label_palette).
  std::array<std::uint8_t, 3> plan_wall_rgb{0x00, 0x9e, 0x73};
  std::array<std::uint8_t, 3> plan_space_rgb{0xe6, 0x9f, 0x00};
  std::array<std::uint8_t, 3> plan_door_rgb{0xd5, 0x5e, 0x00};
  std::array<std::uint8_t, 3> plan_window_rgb{0x56, 0xb4, 0xe9};
  std::array<std::uint8_t, 3> plan_opening_rgb{0xf0, 0xe4, 0x42};

  /// How a `mesh` layer is drawn. MeshStyle::automatic resolves to
  /// MeshStyle::overlay when a point layer is drawn too.
  MeshStyle mesh_style = MeshStyle::automatic;
  /// Opacity of the MeshStyle::overlay surface.
  double mesh_overlay_opacity = 0.35;
  /// Dihedral angle above which a mesh edge counts as a feature edge [deg].
  double mesh_feature_angle_deg = 30.0;
  /// Feature-edge and section-line colour (`--label-5`, vermillion).
  std::array<std::uint8_t, 3> mesh_edge_rgb{0xd5, 0x5e, 0x00};

  /// Thresholds of the `deviation` layer.
  DeviationOptions deviation;

  /// Frame on the [q, 1 - q] per-axis quantiles of the geometry cloud (plus
  /// any mesh) instead of the full bounds, so a few stray points far outside
  /// the building do not shrink it to a speck (SceneInfo::frame_bounds).
  /// 0 = the full bounds.
  double frame_quantile = 0.0;
  /// Measure the framing box in the display frame turned by this yaw [deg]
  /// (SceneInfo::display_bounds) — the yaw the camera will use.
  double frame_yaw_deg = 0.0;
};

/// The `deviation` layer's colour of a point @p d metres from the mesh: the
/// `--label-2` green at 0, `--label-3` yellow at DeviationOptions::within_m,
/// `--label-5` vermillion at DeviationOptions::uncovered_m (linear between),
/// `--label-6` reddish purple beyond it, and the palette's unlabeled grey for
/// a NaN (a point with no distance).
std::array<std::uint8_t, 3> deviation_color(double d,
                                            const DeviationOptions &opts);

/// The colour stops of deviation_color(), for a legend: (distance [m], rgb).
/// The last stop is the "beyond uncovered_m" colour.
std::vector<std::pair<double, std::array<std::uint8_t, 3>>>
deviation_color_stops(const DeviationOptions &opts);

/// What a plan overlay added (SceneOptions::plan).
struct PlanOverlayInfo {
  /// Every overlay actor. Named kPlanOverlayActorName, which is how
  /// apply_cut_plane() leaves them uncut.
  std::vector<vtkActor *> actors;
  /// Text labels (stroke-font geometry lying in the plan, also named
  /// kPlanOverlayActorName), one per space and per wall.
  std::vector<vtkActor *> labels;
  std::size_t walls = 0;    ///< walls drawn
  std::size_t spaces = 0;   ///< spaces drawn
  std::size_t openings = 0; ///< openings drawn (an orphan is skipped)
};

/// vtkObject name of every plan-overlay actor.
inline constexpr const char *kPlanOverlayActorName = "reusex.plan_overlay";

/// vtkObject name of a mesh section line (add_mesh_section_line()); exempt
/// from the cut and the section slab like the plan overlay.
inline constexpr const char *kSectionLineActorName = "reusex.section_line";

/// The actors one layer added, so an interactive viewer can toggle it.
struct SceneLayer {
  Layer layer = Layer::cloud;
  /// Every actor of the layer (one for a point layer, one per component).
  std::vector<vtkActor *> actors;
  /// Point layers only: the hidden coarse twin (SceneOptions::coarse_points),
  /// or nullptr.
  vtkActor *coarse = nullptr;
  std::size_t points = 0; ///< points drawn (per point layer)
  std::size_t faces = 0;  ///< mesh faces drawn
  std::size_t items = 0;  ///< frustums / panoramas / components drawn
  /// Mesh layer only: the mesh's surface poly-data, the source of
  /// add_mesh_section_line(). Owned by a surface actor in the renderer (a
  /// hidden one, not in @ref actors, under MeshStyle::edges).
  vtkPolyData *mesh_surface = nullptr;
};

/// What populate_scene() added.
struct SceneInfo {
  SceneBounds bounds;
  /// What a camera should frame: @ref bounds, or the quantile box of
  /// SceneOptions::frame_quantile united with the mesh's bounds.
  SceneBounds frame_bounds;
  /// The framing box measured in the display frame (u, v, z) = world turned
  /// by @ref display_yaw_deg about z (counter-clockwise), when
  /// SceneOptions::frame_quantile or frame_yaw_deg was set; @ref frame_bounds
  /// is then its world AABB.
  std::optional<SceneBounds> display_bounds;
  double display_yaw_deg = 0.0;
  std::size_t drawn_points = 0;
  std::size_t drawn_faces = 0;
  std::vector<SceneLayer> layers;
  /// Points in the geometry cloud (0 when no point layer was drawn).
  std::size_t source_points = 0;
  LodMethod lod = LodMethod::all;
  /// Storage index of each point a full point actor draws (its vtk point id
  /// -> cloud index); empty when every point is drawn (the identity).
  std::vector<std::uint32_t> indices;
  /// The plan overlay, when SceneOptions::plan was set (empty otherwise).
  PlanOverlayInfo plan;
  /// The mesh style a `mesh` layer was drawn in (automatic resolved).
  MeshStyle mesh_style = MeshStyle::solid;
  /// The `deviation` layer's summary over every measured point, and over the
  /// points of a segmented plane (see RenderInfo::deviation_planar).
  std::optional<DeviationStats> deviation;
  std::optional<DeviationStats> deviation_planar;
};

/// Load @p opts' layers from @p db and add them to @p renderer.
///
/// Does not touch the camera, the background or any render window.
/// @throws std::runtime_error when a requested layer's data is missing or out
///         of sync (the message names the stage to run), or when the layers
///         produced no drawable geometry.
SceneInfo populate_scene(vtkRenderer *renderer, const ProjectDB &db,
                         const SceneOptions &opts);

/// A resolved horizontal cut: where the plane sits and how that was decided.
struct CutPlane {
  double z = 0.0;           ///< world z of the plane
  double floor_z = 0.0;     ///< the reference it was measured up from
  double height = 0.0;      ///< metres above @ref floor_z
  bool from_planes = false; ///< floor came from the segmentation, not the bbox
};

/// Decide where to cut @p bounds: @p height (or the default) above the floor
/// detected by `rux create planes`, or above the bounding box's lower face.
/// Warns when the cut is above or below everything drawn (STANDARDS §5).
CutPlane resolve_cut_plane(const ProjectDB &db, const SceneBounds &bounds,
                           std::optional<double> height);

/// Clip every actor of @p renderer to the half-space below z = @p z, except
/// the plan overlay's (kPlanOverlayActorName): a plan is a drawing laid over
/// the section, not part of it.
void apply_cut_plane(vtkRenderer *renderer, double z);

/// Remove every clipping plane from @p renderer's actors.
void clear_cut_planes(vtkRenderer *renderer);

/// A SectionSpec placed in a scene: the section line in plan and its slab.
struct ResolvedSection {
  std::array<double, 2> p0{0, 0}; ///< line start (x, y) [m]
  std::array<double, 2> p1{1, 0}; ///< line end (x, y) [m]
  /// Unit direction p0 -> p1 in plan; points right in the section view.
  std::array<double, 2> dir{1, 0};
  /// Unit view direction in plan, the left normal of @ref dir: the camera
  /// stands on the right of the line and looks across it.
  std::array<double, 2> view{0, 1};
  double thickness = 0.10; ///< slab thickness [m]
};

/// Place @p spec in @p bounds: `x=` / `y=` lines run across the bounds' plan
/// extent (padded by the slab thickness), a segment keeps its ends.
/// @throws std::invalid_argument on a non-positive thickness, a zero-length
///         segment, or an x/y position outside the bounds' plan extent.
ResolvedSection resolve_section(const SectionSpec &spec,
                                const SceneBounds &bounds);

/// Clip every actor of @p renderer to the slab of @p section: within half the
/// thickness of the line's vertical plane, and between its ends. The plan
/// overlay and section lines are exempt, as for apply_cut_plane().
void apply_section(vtkRenderer *renderer, const ResolvedSection &section);

/// Draw the intersection of the scene's `mesh` layer with a plane (origin
/// @p origin, normal @p normal) as a heavy line in SceneOptions::mesh_edge_rgb,
/// shifted by @p shift (e.g. toward the camera, so the slab's points in front
/// do not hide it). Named kSectionLineActorName.
/// @return the line actor, or nullptr when @p info has no mesh layer or the
///         plane misses the mesh.
vtkActor *add_mesh_section_line(vtkRenderer *renderer, const SceneInfo &info,
                                const std::array<double, 3> &origin,
                                const std::array<double, 3> &normal,
                                const std::array<double, 3> &shift,
                                const std::array<std::uint8_t, 3> &rgb,
                                double width = 3.0);

/// How a preset camera frames the scene.
struct CameraFraming {
  ViewPreset view = ViewPreset::top; ///< never ViewPreset::explicit_camera
  int orbit_count = 8;
  int orbit_index = 0;
  double orbit_elevation_deg = 25.0;
  double margin = 1.08;
  double aspect = 4.0 / 3.0; ///< viewport width / height
  /// Display rotation about world z, degrees, counter-clockwise seen from
  /// above (project_settings `view.yaw_deg`). Display only: the scene's
  /// coordinates are never changed. ViewPreset::section looks along its own
  /// line and uses the yaw only to read @ref display_bounds.
  double yaw_deg = 0.0;
  /// The geometry's box in the display frame of @ref yaw_deg
  /// (SceneInfo::display_bounds): frames top, plan and the elevations
  /// exactly. Must be measured at the same yaw.
  std::optional<SceneBounds> display_bounds;
  /// The slab ViewPreset::section looks at (required by that view).
  std::optional<ResolvedSection> section;
};

/// Place @p renderer's active camera for a preset view of @p bounds.
/// Deterministic: a pure function of the bounds (STANDARDS §6).
void place_preset_camera(vtkRenderer *renderer, const CameraFraming &framing,
                         const SceneBounds &bounds);

/// Place @p renderer's active camera at an explicit pose + pinhole camera,
/// for an image of @p width x @p height pixels.
/// @throws std::runtime_error on non-positive focal lengths.
void place_explicit_camera(vtkRenderer *renderer, const CameraSpec &spec,
                           int width, int height, const SceneBounds &bounds);

/// Route VTK's diagnostics into ReUseX logging, once per process (see
/// render_view()).
void install_vtk_log_bridge();

} // namespace reusex::visualize
