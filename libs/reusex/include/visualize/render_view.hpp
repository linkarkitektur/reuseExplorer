// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Headless render-to-image (#294).
//
// Renders the contents of a .rux project to an in-memory image without a
// display server. The backend is VTK's offscreen render window: the flake's
// VTK is built with VTK_OPENGL_HAS_EGL, and VTK transparently falls back from
// vtkXOpenGLRenderWindow to vtkEGLRenderWindow when no X/Wayland session is
// reachable, so `env -u DISPLAY rux render ...` renders on the GPU exactly as
// it would with a session attached.
//
// Keep this header light (STANDARDS §2): cv::Mat and ProjectDB are
// forward-declared, and no VTK header is exposed to consumers.

#include "reusex/core/pose_edits.hpp"
#include "reusex/core/solid_model.hpp"
#include "reusex/visualize/drift_layers.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace cv {
class Mat;
} // namespace cv

namespace reusex {

class ProjectDB;

namespace visualize {

/// One drawable body of content in a rendered view.
///
/// `cloud` draws the geometry cloud with its stored RGB colour; the four
/// label layers draw the same geometry recoloured by the corresponding
/// index-aligned `Label` cloud (CONTRACTS.md) on the design tokens'
/// categorical `--label-*` scale (visualize/scene.hpp); `mesh` draws a stored
/// mesh, `components` building-component outlines, `frustums` the camera
/// frustum of each posed sensor frame and `panoramas` a marker where each 360
/// panorama was taken. `deviation` draws the geometry cloud coloured by each
/// point's distance to the stored mesh (RenderOptions::mesh_name), the mesh-QA
/// view of reusex-8vx: green on the mesh, yellow to vermillion as the distance
/// grows to DeviationOptions::uncovered_m, reddish purple beyond it (a point
/// the mesh does not cover). The drift-loop layers (reusex-rcv.1.4,
/// visualize/drift_layers.hpp): `passes` draws the depth frames
/// back-projected on the fly, each point coloured by the time segment of its
/// frame (a doubled wall shows in two colours); `graph` the camera
/// trajectory, the stored pose-graph edges coloured by solve residual,
/// node-id labels and pose-edit pins; `drift` the pass-to-pass residual cells
/// of a drift report as a heatmap per storey.
enum class Layer {
  cloud,      ///< `cloud` (PointXYZRGB), stored per-point colour
  labels,     ///< `labels` (Label) — semantic classes
  planes,     ///< `planes` (Label) — planar segments
  rooms,      ///< `rooms` (Label) — room partition
  instances,  ///< `instances` (Label) — spatial instances
  mesh,       ///< a stored mesh from the `meshes` table
  components, ///< building-component outlines (windows / doors / walls)
  frustums,   ///< camera frustums of the posed sensor frames (subsampled)
  panoramas,  ///< a marker at each placed 360 panorama
  deviation,  ///< `cloud` coloured by distance to the mesh (mesh QA)
  passes,     ///< back-projected depth coloured by capture segment
  graph,      ///< trajectory + pose-graph edges by residual + edit pins
  drift,      ///< drift-report residual cells as a heatmap (per storey)
  walls,      ///< `walls` table: footprints in plan, boxes in 3D (WP3)
  openings,   ///< `openings` table: rectangles in the wall faces (WP5)
  spaces,     ///< `spaces` table: translucent polygons with names (WP4)
  solids,     ///< `solid_*` meshes: typed solids coloured by type (W6.1)
};

/// Parse a layer name as accepted by `rux render --layers`.
/// @return std::nullopt if @p name is not a layer.
std::optional<Layer> layer_from_string(std::string_view name);

/// The canonical name of @p layer (the inverse of layer_from_string).
std::string_view to_string(Layer layer);

/// Every layer, in draw order — the vocabulary `--layers` accepts.
const std::vector<Layer> &all_layers();

/// How the camera is placed.
enum class ViewPreset {
  /// Orthographic view looking straight down (-Z), north up. Shows whatever
  /// is topmost — in a closed interior, the ceiling. For a floor plan use
  /// ViewPreset::plan.
  top,
  /// A true floor plan: ViewPreset::top plus a horizontal cut (#306). The
  /// ceiling and everything else above RenderOptions::cut_height is clipped
  /// away, leaving the floor and the walls in section.
  plan,
  /// Orthographic elevation from the south, looking along +Y.
  front,
  /// Orthographic elevation from the north, looking along -Y.
  back,
  /// Orthographic elevation from the west, looking along +X.
  left,
  /// Orthographic elevation from the east, looking along -X.
  right,
  /// Orthographic view of a vertical section slab (RenderOptions::section),
  /// looking perpendicular to the section line with the line's direction
  /// pointing right. Everything outside the slab is clipped away.
  section,
  /// Perspective view on a ring around the scene; see
  /// RenderOptions::orbit_index / orbit_count / orbit_elevation_deg.
  orbit,
  /// Use RenderOptions::camera verbatim.
  explicit_camera,
};

/// The canonical name of @p view, as accepted by `rux render --view`.
///
/// `orbit` and `explicit_camera` name the *mode*; the CLI spells them
/// `orbit:N` and `frame:<node_id>` because both need a parameter.
std::string_view to_string(ViewPreset view);

/// Parse a bare view-preset name. @return std::nullopt if @p name is not one.
std::optional<ViewPreset> view_preset_from_string(std::string_view name);

/// An explicit camera: rigid pose plus pinhole intrinsics.
///
/// The layout deliberately matches what ProjectDB hands out for a captured
/// frame — `ProjectDB::sensor_frame_pose()` (row-major, camera-to-world) and
/// `SensorIntrinsics` — so rendering the scene from a recorded viewpoint is a
/// copy, not a conversion. Axes follow the OpenCV camera convention used
/// throughout the pipeline: +x right, +y down, +z forward along the view.
struct CameraSpec {
  /// Row-major 4x4 camera-to-world transform.
  std::array<double, 16> pose{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  double fx = 0.0; ///< focal length in pixels (x)
  double fy = 0.0; ///< focal length in pixels (y)
  double cx = 0.0; ///< principal point in pixels (x)
  double cy = 0.0; ///< principal point in pixels (y)
};

/// A vertical section through the scene (`rux render --section`).
///
/// A section is a line in plan plus a slab around it: everything farther than
/// half the thickness from the line's vertical plane, or beyond its ends, is
/// clipped away. `x=<m>` and `y=<m>` run across the whole scene; a segment
/// `<x0>,<y0>:<x1>,<y1>` only between its ends.
struct SectionSpec {
  enum class Kind {
    x,       ///< the plane x = at, line pointing +Y
    y,       ///< the plane y = at, line pointing +X
    segment, ///< the vertical plane through p0 -> p1
  };
  Kind kind = Kind::y;
  double at = 0.0;                ///< Kind::x / Kind::y position [m]
  std::array<double, 2> p0{0, 0}; ///< Kind::segment start [m]
  std::array<double, 2> p1{1, 0}; ///< Kind::segment end [m]
  double thickness = 0.10;        ///< slab thickness [m]; must be > 0
};

/// Parse `x=<m>`, `y=<m>` or `<x0>,<y0>:<x1>,<y1>` (thickness defaulted).
/// @throws std::invalid_argument on anything else, or a zero-length segment.
SectionSpec parse_section(std::string_view text);

/// The canonical text of @p section (the inverse of parse_section()).
std::string to_string(const SectionSpec &section);

/// How a `mesh` layer is drawn.
enum class MeshStyle {
  /// `overlay` when a point layer is drawn too, otherwise `solid`.
  automatic,
  /// An opaque shaded surface (the mesh alone).
  solid,
  /// A translucent surface plus its feature edges (boundary edges and
  /// creases), so the cloud shows through and the mesh's corners stay crisp.
  overlay,
  /// Feature edges only.
  edges,
};

std::string_view to_string(MeshStyle style);
std::optional<MeshStyle> mesh_style_from_string(std::string_view name);

/// Thresholds of the deviation layer (Layer::deviation) and its summary.
struct DeviationOptions {
  /// A point within this distance of the mesh fits it [m] (the summary's
  /// "within" share, and the end of the green part of the colour ramp).
  double within_m = 0.05;
  /// A point farther than this from the mesh is not covered by it [m] (the
  /// summary's "uncovered" share; the colour ramp saturates here).
  double uncovered_m = 0.20;
};

/// Summary of the deviation layer's point-to-mesh distances.
struct DeviationStats {
  std::size_t points = 0; ///< finite points measured
  double mean_m = 0.0;
  double median_m = 0.0; ///< nearest rank
  double p95_m = 0.0;    ///< nearest rank
  double max_m = 0.0;
  double within_share = 0.0;    ///< distance <= DeviationOptions::within_m
  double uncovered_share = 0.0; ///< distance > DeviationOptions::uncovered_m
};

/// Default height of the plan cut above the detected floor, in metres.
///
/// Waist height: above every sill and below every soffit, so a plan shows door
/// and window openings in section the way a drawn plan does.
inline constexpr double kDefaultCutHeightM = 1.2;

/// Fallback cut height when no floor plane is available, as a fraction of the
/// scene's vertical extent.
///
/// A height in metres would be meaningless there — with no segmentation to
/// locate the floor the reference is the bounding box's lower face, which a
/// single stray point can put well below the real floor.
inline constexpr double kDefaultCutBboxFraction = 0.45;

/// |n_z| above which a segmented plane counts as horizontal when looking for
/// the floor. Same split the real-scan fixture test asserts on the office
/// corridor, so "horizontal" means the same thing in both places.
inline constexpr double kHorizontalNormalZ = 0.95;

/// One instance to call out in a render (Kortlægning evidence, #265 Phase 2
/// Task 8).
struct InstanceHighlight {
  /// Named Label cloud supplying the per-point instance id, index-aligned
  /// with RenderOptions::cloud_name.
  std::string cloud_name = "instances";
  /// The instance id to paint; every other point is dimmed.
  std::uint32_t instance_id = 0;
};

/// Everything render_view() can be told to do.
///
/// This struct is the single definition of every render default; `rux render`
/// mirrors these values into its `--help` and never redefines them
/// (STANDARDS §4).
struct RenderOptions {
  /// Layers to draw. Empty is an error. All layers are opaque and
  /// depth-tested, so the order only matters where two layers put geometry at
  /// the same depth (e.g. `cloud` and `planes`, which draw the same points).
  std::vector<Layer> layers{Layer::cloud};

  /// Named geometry cloud supplying point positions for every point layer.
  std::string cloud_name = "cloud";
  /// Named mesh drawn by Layer::mesh.
  std::string mesh_name = "mesh";

  ViewPreset view = ViewPreset::top;

  /// Number of viewpoints on the orbit ring (`--view orbit:N`).
  int orbit_count = 8;
  /// Which of the @ref orbit_count viewpoints to render, in [0, orbit_count).
  int orbit_index = 0;
  /// Height of the orbit ring above the scene centre, in degrees.
  double orbit_elevation_deg = 25.0;

  /// View yaw in degrees, counter-clockwise seen from above; std::nullopt =
  /// the project's `view.yaw_deg` setting (0 when unset). Ignored by
  /// ViewPreset::explicit_camera.
  std::optional<double> yaw_deg;

  /// Camera used when view == ViewPreset::explicit_camera.
  std::optional<CameraSpec> camera;

  /// Clip away everything above a horizontal cut plane before rendering.
  ///
  /// Implied by ViewPreset::plan and settable on its own, so any view can be
  /// cut — `--view orbit:8 --cut-height 1.2` is a cut axonometric.
  bool cut = false;

  /// Height of the cut plane above the floor, in metres.
  ///
  /// std::nullopt means derive it: kDefaultCutHeightM above the lowest
  /// horizontal plane in `plane_centroids` / `plane_normals`, or
  /// kDefaultCutBboxFraction of the vertical extent above the bounding box's
  /// lower face when the project has no plane segmentation.
  std::optional<double> cut_height;

  int width = 1600;
  int height = 1200;

  /// Background colour as linear RGB in [0, 1].
  std::array<double, 3> background{0.11, 0.12, 0.14};

  /// Point sprite size in pixels.
  double point_size = 2.0;

  /// Framing slack around the scene bounding box: 1.0 is a tight fit, larger
  /// pulls back, smaller crops in. Must be positive.
  double margin = 1.08;

  /// Paint one instance in kHighlightRgb and dim the rest (Kortlægning
  /// evidence). Applies to every point layer; needs the named Label cloud,
  /// index-aligned with cloud_name.
  std::optional<InstanceHighlight> highlight;

  /// A solid-model plan to draw over the layers (`rux render --plan`), e.g. an
  /// authored reference (docs/guides/solid-model-reference-authoring.md). The
  /// top and plan views draw it flattened on top of everything and uncut
  /// (PlanPlacement::above_scene); every other view on its storey's floor
  /// (PlanPlacement::level). See SceneOptions::plan for what is drawn.
  std::optional<core::SolidModel> plan;
  /// Label spaces by name and walls by guid (SceneOptions::plan_labels).
  bool plan_labels = true;

  /// How the `mesh` layer is drawn.
  MeshStyle mesh_style = MeshStyle::automatic;

  /// The section view's slab (ViewPreset::section). Required by that view;
  /// ignored by every other.
  std::optional<SectionSpec> section;

  /// Thresholds of the `deviation` layer.
  DeviationOptions deviation;

  /// Draw a colour-ramp legend and the summary numbers into the image when
  /// the `deviation` layer is drawn, and a legend per drift-loop layer
  /// (`passes`, `graph`, `drift`).
  bool legend = true;

  /// The `passes` layer's back-projection and segments (also the
  /// trajectory's colours in the `graph` layer).
  PreviewCloudOptions passes;
  /// The `graph` layer.
  GraphLayerOptions graph;
  /// The `drift` layer's colour scale and storey filter. With a plan cut and
  /// no explicit storey, only the storey the cut lies in is drawn.
  DriftLayerOptions drift;
  /// The `drift` layer's cells. Required by that layer.
  std::optional<DriftHeatmap> drift_heatmap;
  /// Pose edits drawn as pins by the `graph` layer.
  std::optional<core::PoseEdits> edits;
  /// World poses replacing the stored ones for `passes` and `graph` (a
  /// candidate solve), index-aligned with ProjectDB::sensor_frame_ids();
  /// empty = the stored poses.
  std::vector<Eigen::Matrix4d> poses;
  /// The edges the `graph` layer draws, replacing the stored rows (with
  /// their stored residuals): a candidate solve's residuals
  /// (reusex-rcv.1.18). nullopt = ProjectDB::list_pose_graph_edges().
  std::optional<std::vector<GraphLayerEdge>> graph_edges;
  /// The `passes` layer's points, already back-projected with @ref passes
  /// and @ref poses (preview_cloud()): a caller that renders several views
  /// of one candidate solve (`rux optimize --watch`) makes it once. null =
  /// render_view() back-projects the frames itself.
  std::shared_ptr<const PreviewCloud> preview;

  /// Frame the camera on the [q, 1 - q] quantiles of the cloud (plus the
  /// mesh) rather than on everything drawn; 0 = everything
  /// (SceneOptions::frame_quantile).
  double frame_quantile = 0.0;

  /// Draw the mesh's intersection with the cut plane (plan views, `cut`) or
  /// with the section plane (ViewPreset::section) as a heavy line, when a
  /// mesh layer is drawn — the mesh outline to compare with the cut cloud.
  bool mesh_section_line = true;
};

/// What render_view() measured on the way, for a caller that reports it.
struct RenderInfo {
  std::size_t drawn_points = 0;
  std::size_t drawn_faces = 0;
  /// Axis-aligned bounds the camera framed (RenderOptions::frame_quantile).
  std::array<double, 3> bounds_min{0, 0, 0};
  std::array<double, 3> bounds_max{0, 0, 0};
  /// The view yaw used [deg], and the framed box in that display frame
  /// (u, v, z) = world turned counter-clockwise by the yaw (equal to the
  /// bounds above at yaw 0 without frame_quantile).
  double yaw_deg = 0.0;
  std::array<double, 3> display_min{0, 0, 0};
  std::array<double, 3> display_max{0, 0, 0};
  /// The `deviation` layer's summary, when it was drawn: every measured
  /// point, and the points that belong to a segmented plane (label != 0 in
  /// an index-aligned `planes` cloud) when the project has one — the clutter
  /// (furniture, people) a structural mesh is not meant to cover is left out
  /// of the second.
  std::optional<DeviationStats> deviation;
  std::optional<DeviationStats> deviation_planar;
  /// What the drift-loop layers drew (passes, graph, drift).
  DriftLegendInfo drift_legend;
};

/// This machine cannot create an offscreen OpenGL context (#313).
///
/// A distinct type, not a bare message, because "there is no GPU here" is not
/// a defect in the caller's request the way every other render error is: a
/// test can skip on it, and a batch job can downgrade rendering to optional,
/// while still failing hard on a bad layer or a missing cloud. It derives from
/// std::runtime_error, so existing catch sites keep working unchanged.
class OffscreenGlUnavailable : public std::runtime_error {
    public:
  using std::runtime_error::runtime_error;
};

/// Render one view of @p db to an image.
///
/// Deterministic: every preset camera is derived from the bounding box of the
/// geometry actually drawn, so two runs over the same project frame the same
/// shot (STANDARDS §6).
///
/// Side effect, once per process: VTK's own diagnostics are rerouted into
/// ReUseX logging (`core/logging.hpp`) and its stderr sink is switched off, so
/// library code does not write to stderr behind the caller's back. This is
/// global to VTK and is not undone — a process that also opens the interactive
/// viewer will see its VTK messages through the ReUseX log too.
///
/// @param db    An open project. Only read.
/// @param opts  What and how to draw.
/// @param info  When given, filled with what was drawn and measured.
/// @return An 8-bit 3-channel BGR image of size opts.width x opts.height,
///         ready for `cv::imwrite`.
/// @throws OffscreenGlUnavailable if the machine has no usable offscreen
///         OpenGL implementation — probed before any rendering happens, so
///         this is reported rather than crashed on (#313).
/// @throws std::runtime_error if the project is not open, the options are
///         invalid, a requested layer's data is missing (the message names the
///         stage to run first), or the framebuffer comes back at the wrong
///         size.
cv::Mat render_view(const ProjectDB &db, const RenderOptions &opts,
                    RenderInfo *info = nullptr);

/// Build the camera that reproduces a stored sensor frame's viewpoint.
///
/// Composes the frame's world pose with its camera-to-base local transform and
/// rescales the pinhole intrinsics from the captured frame size to @p width x
/// @p height. Lives here rather than in the CLI so that "camera-to-world =
/// pose * local_transform" has exactly one definition outside the
/// back-projection that owns it (STANDARDS §1/§4).
///
/// @throws std::runtime_error if the frame has no usable intrinsics, or no
///         usable stored pose (`ProjectDB::has_sensor_frame_pose()`) — a
///         render from the identity fallback would silently be a view from the
///         world origin (#336).
CameraSpec camera_from_sensor_frame(const ProjectDB &db, int node_id, int width,
                                    int height);

} // namespace visualize
} // namespace reusex
