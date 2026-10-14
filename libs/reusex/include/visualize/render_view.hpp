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

#include <array>
#include <cstdint>
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
/// panorama was taken.
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
  /// Orthographic elevation looking along +Y.
  front,
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
/// @return An 8-bit 3-channel BGR image of size opts.width x opts.height,
///         ready for `cv::imwrite`.
/// @throws OffscreenGlUnavailable if the machine has no usable offscreen
///         OpenGL implementation — probed before any rendering happens, so
///         this is reported rather than crashed on (#313).
/// @throws std::runtime_error if the project is not open, the options are
///         invalid, a requested layer's data is missing (the message names the
///         stage to run first), or the framebuffer comes back at the wrong
///         size.
cv::Mat render_view(const ProjectDB &db, const RenderOptions &opts);

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
