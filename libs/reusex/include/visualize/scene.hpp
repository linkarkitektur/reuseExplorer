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

#include "reusex/visualize/render_view.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

class vtkActor;
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
/// shift every class by one against the legend.
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
};

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
};

/// What populate_scene() added.
struct SceneInfo {
  SceneBounds bounds;
  std::size_t drawn_points = 0;
  std::size_t drawn_faces = 0;
  std::vector<SceneLayer> layers;
  /// Points in the geometry cloud (0 when no point layer was drawn).
  std::size_t source_points = 0;
  LodMethod lod = LodMethod::all;
  /// Storage index of each point a full point actor draws (its vtk point id
  /// -> cloud index); empty when every point is drawn (the identity).
  std::vector<std::uint32_t> indices;
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

/// Clip every actor of @p renderer to the half-space below z = @p z.
void apply_cut_plane(vtkRenderer *renderer, double z);

/// Remove every clipping plane from @p renderer's actors.
void clear_cut_planes(vtkRenderer *renderer);

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
  /// coordinates are never changed.
  double yaw_deg = 0.0;
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
