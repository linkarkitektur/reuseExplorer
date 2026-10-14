// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "visualize/scene.hpp"

#include "core/ProjectDB.hpp"
#include "core/SensorIntrinsics.hpp"
#include "core/label_semantics.hpp"
#include "core/logging.hpp"
#include "geometry/BuildingComponent.hpp"
#include "geometry/component_persistence.hpp"
#include "geometry/transform_utils.hpp"
#include "reusex/visualize/highlight.hpp"
#include "types/point_types.hpp"

#include <pcl/PolygonMesh.h>
#include <pcl/conversions.h>

#include <vtkActor.h>
#include <vtkActorCollection.h>
#include <vtkCamera.h>
#include <vtkCellArray.h>
#include <vtkLogger.h>
#include <vtkMapper.h>
#include <vtkNew.h>
#include <vtkPlane.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkUnsignedCharArray.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <numbers>
#include <stdexcept>
#include <string>

namespace reusex::visualize {

namespace {

// ── VTK diagnostics -> ReUseX logging ────────────────────────────────────────
//
// Two reasons to reroute VTK's own output: library code must log through
// core/logging (CLAUDE.md), and the headless path always emits one cosmetic
// warning — vtkXOpenGLRenderWindow complaining that it cannot reach an X
// server — immediately before VTK falls back to the EGL render window and
// succeeds. That message is expected on every display-less run, so it is
// demoted to debug while every other VTK warning still surfaces at warn.
//
// The interception point is vtkLogger rather than vtkOutputWindow. In VTK 9
// the vtkWarningMacro family routes through vtkOutputWindow's file/line-aware
// virtuals into vtkLogger, and some call sites reach vtkLogger directly;
// subclassing vtkOutputWindow catches only part of that traffic (verified: the
// X-connection warning still reached stderr). A logger callback catches all of
// it, and switching the stderr sink off routes VTK through our handler instead
// of duplicating every message.

/// The X-server probe that precedes the (successful) EGL fallback.
bool is_expected_headless_fallback(const char *text) {
  return text != nullptr &&
         std::strstr(text, "bad X server connection") != nullptr;
}

void vtk_log_callback(void * /*user_data*/, const vtkLogger::Message &message) {
  const char *text = message.message != nullptr ? message.message : "";
  const char *file = message.filename != nullptr ? message.filename : "?";

  if (message.verbosity <= vtkLogger::VERBOSITY_ERROR) {
    core::error("VTK [{}:{}]: {}", file, message.line, text);
  } else if (message.verbosity == vtkLogger::VERBOSITY_WARNING) {
    if (is_expected_headless_fallback(text)) {
      core::debug("VTK: no X server; falling back to offscreen EGL rendering "
                  "(expected when headless)");
    } else {
      core::warn("VTK [{}:{}]: {}", file, message.line, text);
    }
  } else {
    core::debug("VTK [{}:{}]: {}", file, message.line, text);
  }
}

// ── Colour helpers ───────────────────────────────────────────────────────────

void component_color(geometry::ComponentType type, double rgb[3]) {
  // Same convention as the interactive viewer (view/component_renderer.cpp).
  switch (type) {
  case geometry::ComponentType::door:
    rgb[0] = 0.0, rgb[1] = 1.0, rgb[2] = 1.0;
    return;
  case geometry::ComponentType::wall:
    rgb[0] = 1.0, rgb[1] = 1.0, rgb[2] = 0.0;
    return;
  case geometry::ComponentType::window:
  default:
    rgb[0] = 0.0, rgb[1] = 1.0, rgb[2] = 0.0;
    return;
  }
}

// ── Data loading ─────────────────────────────────────────────────────────────

/// Name of the pipeline stage that produces a layer's data, for error messages.
std::string producing_command(Layer layer) {
  switch (layer) {
  case Layer::cloud:
    return "rux create clouds";
  case Layer::labels:
    return "rux create annotate && rux create project";
  case Layer::planes:
    return "rux create planes";
  case Layer::rooms:
    return "rux create rooms";
  case Layer::instances:
    return "rux create instances";
  case Layer::mesh:
    return "rux create mesh";
  case Layer::components:
    return "rux create windows";
  case Layer::frustums:
    return "rux import rtabmap (or another importer of posed frames)";
  case Layer::panoramas:
    return "rux import 360";
  }
  return "the corresponding rux create subcommand";
}

/// Load the geometry cloud shared by every point layer, failing loudly.
CloudPtr load_geometry_cloud(const ProjectDB &db, const std::string &name) {
  if (!db.has_point_cloud(name)) {
    throw std::runtime_error("render: project has no point cloud named '" +
                             name + "' — run `" +
                             producing_command(Layer::cloud) + "` first");
  }
  CloudPtr cloud = db.point_cloud_xyzrgb(name);
  if (!cloud || cloud->empty()) {
    throw std::runtime_error("render: point cloud '" + name +
                             "' is empty — nothing to draw; re-run `" +
                             producing_command(Layer::cloud) + "`");
  }
  return cloud;
}

/// The label cloud a label layer draws with.
std::string label_cloud_name(Layer layer) {
  switch (layer) {
  case Layer::labels:
    return "labels";
  case Layer::planes:
    return "planes";
  case Layer::rooms:
    return "rooms";
  case Layer::instances:
    return "instances";
  default:
    return {};
  }
}

// ── VTK geometry construction ────────────────────────────────────────────────

/// Wrap positions + per-point colours in a renderable vertex poly-data.
///
/// @p colors holds 3 bytes for every point of @p cloud; @p indices selects
/// which points to draw (empty = all of them, in order).
vtkSmartPointer<vtkPolyData>
make_point_polydata(const Cloud &cloud,
                    const std::vector<unsigned char> &colors,
                    const std::vector<std::uint32_t> &indices) {
  const bool all = indices.empty();
  const vtkIdType n =
      static_cast<vtkIdType>(all ? cloud.size() : indices.size());

  vtkNew<vtkPoints> points;
  points->SetDataTypeToFloat();
  points->SetNumberOfPoints(n);

  vtkNew<vtkUnsignedCharArray> scalars;
  scalars->SetNumberOfComponents(3);
  scalars->SetName("colors");
  scalars->SetNumberOfTuples(n);

  vtkNew<vtkCellArray> verts;
  verts->AllocateEstimate(n, 1);

  for (vtkIdType i = 0; i < n; ++i) {
    const std::size_t k = all ? static_cast<std::size_t>(i)
                              : indices[static_cast<std::size_t>(i)];
    const auto &p = cloud[k];
    points->SetPoint(i, p.x, p.y, p.z);
    scalars->SetTypedComponent(i, 0, colors[3 * k + 0]);
    scalars->SetTypedComponent(i, 1, colors[3 * k + 1]);
    scalars->SetTypedComponent(i, 2, colors[3 * k + 2]);
    verts->InsertNextCell(1, &i);
  }

  auto poly = vtkSmartPointer<vtkPolyData>::New();
  poly->SetPoints(points);
  poly->SetVerts(verts);
  poly->GetPointData()->SetScalars(scalars);
  return poly;
}

vtkActor *add_points_actor(vtkRenderer *renderer, vtkPolyData *poly,
                           double point_size) {
  vtkNew<vtkPolyDataMapper> mapper;
  mapper->SetInputData(poly);
  mapper->SetScalarModeToUsePointData();
  mapper->SetColorModeToDirectScalars();

  vtkNew<vtkActor> actor;
  actor->SetMapper(mapper);
  actor->GetProperty()->SetPointSize(point_size);
  // Points carry their own colour; lighting would tint them.
  actor->GetProperty()->SetLighting(false);
  renderer->AddActor(actor);
  return actor; // the renderer holds the reference
}

/// Convert a stored PolygonMesh into a shaded surface actor.
vtkActor *add_mesh_actor(vtkRenderer *renderer, const pcl::PolygonMesh &mesh,
                         SceneBounds &bounds) {
  pcl::PointCloud<pcl::PointXYZ> vertices;
  pcl::fromPCLPointCloud2(mesh.cloud, vertices);

  vtkNew<vtkPoints> points;
  points->SetDataTypeToFloat();
  points->SetNumberOfPoints(static_cast<vtkIdType>(vertices.size()));
  for (std::size_t i = 0; i < vertices.size(); ++i) {
    const auto &v = vertices[i];
    points->SetPoint(static_cast<vtkIdType>(i), v.x, v.y, v.z);
    bounds.add(v.x, v.y, v.z);
  }

  vtkNew<vtkCellArray> polys;
  for (const auto &face : mesh.polygons) {
    if (face.vertices.size() < 3)
      continue;
    std::vector<vtkIdType> ids;
    ids.reserve(face.vertices.size());
    for (const auto idx : face.vertices)
      ids.push_back(static_cast<vtkIdType>(idx));
    polys->InsertNextCell(static_cast<vtkIdType>(ids.size()), ids.data());
  }

  vtkNew<vtkPolyData> poly;
  poly->SetPoints(points);
  poly->SetPolys(polys);

  vtkNew<vtkPolyDataMapper> mapper;
  mapper->SetInputData(poly);

  vtkNew<vtkActor> actor;
  actor->SetMapper(mapper);
  actor->GetProperty()->SetColor(0.82, 0.80, 0.76);
  actor->GetProperty()->SetAmbient(0.25);
  actor->GetProperty()->SetDiffuse(0.75);
  actor->GetProperty()->SetSpecular(0.05);
  renderer->AddActor(actor);
  return actor;
}

/// Draw building-component boundaries as closed polylines.
std::vector<vtkActor *>
add_component_actors(vtkRenderer *renderer,
                     const std::vector<geometry::BuildingComponent> &components,
                     SceneBounds &bounds) {
  std::vector<vtkActor *> out;
  for (const auto &comp : components) {
    const auto &verts = comp.boundary.vertices;
    if (verts.size() < 2)
      continue;

    vtkNew<vtkPoints> points;
    points->SetDataTypeToFloat();
    vtkNew<vtkCellArray> lines;
    const vtkIdType n = static_cast<vtkIdType>(verts.size());
    for (vtkIdType i = 0; i < n; ++i) {
      const auto &v = verts[static_cast<std::size_t>(i)];
      points->InsertNextPoint(v.x(), v.y(), v.z());
      bounds.add(v.x(), v.y(), v.z());
    }
    for (vtkIdType i = 0; i < n; ++i) {
      const vtkIdType seg[2] = {i, (i + 1) % n};
      lines->InsertNextCell(2, seg);
    }

    vtkNew<vtkPolyData> poly;
    poly->SetPoints(points);
    poly->SetLines(lines);

    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputData(poly);

    double rgb[3];
    component_color(comp.type, rgb);

    vtkNew<vtkActor> actor;
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(rgb[0], rgb[1], rgb[2]);
    actor->GetProperty()->SetLineWidth(3.0);
    actor->GetProperty()->SetLighting(false);
    renderer->AddActor(actor);
    out.push_back(actor);
  }
  return out;
}

// ── Horizontal cut plane (#306) ──────────────────────────────────────────────
//
// `--view top` on a real interior renders the ceiling, because that is what is
// topmost. A floor plan is the same camera with everything above waist height
// clipped away, which is what `--view plan` adds.
//
// Two decisions worth stating. The cut is applied per *mapper*, not by
// filtering the geometry, so every layer is cut by the same plane with no copy
// of the point data and no change to what the layers mean. And render_view()
// still frames the camera on the *uncut* bounding box, so a plan and a top
// view of the same project cover the same ground — the cut changes what is
// visible, never where the shot is aimed (STANDARDS §6).

/// How far below the drawn geometry a detected floor plane may sit and still
/// be believed, in metres. A plane centroid further down than this belongs to
/// a stale per-plane cloud from a different run, not to this scene's floor.
constexpr double kFloorPlaneSlackM = 0.5;

/// World z of the floor, from the plane segmentation if the project has one.
///
/// The lowest horizontal plane, matching how the real-scan fixture test
/// identifies floor and ceiling. Nothing in an interior scan segments as a
/// horizontal plane below the floor, so "lowest" is the floor; a table top or
/// a windowsill is horizontal too but always above it.
std::optional<double> detect_floor_z(const ProjectDB &db) {
  if (!db.has_point_cloud("plane_centroids") ||
      !db.has_point_cloud("plane_normals"))
    return std::nullopt;

  const auto centroids = db.point_cloud_xyz("plane_centroids");
  const auto normals = db.point_cloud_normal("plane_normals");
  const std::size_t n_centroids = centroids ? centroids->size() : 0;
  const std::size_t n_normals = normals ? normals->size() : 0;
  if (n_centroids == 0 || n_centroids != n_normals) {
    core::warn("render: 'plane_centroids' ({}) and 'plane_normals' ({}) are "
               "empty or out of sync; placing the cut plane from the bounding "
               "box instead — re-run `rux create planes`",
               n_centroids, n_normals);
    return std::nullopt;
  }

  double lowest = 0.0;
  bool found = false;
  int horizontal = 0;
  for (std::size_t i = 0; i < n_centroids; ++i) {
    if (std::abs(static_cast<double>(normals->points[i].normal_z)) <
        kHorizontalNormalZ)
      continue;
    ++horizontal;
    const double z = static_cast<double>(centroids->points[i].z);
    if (!found || z < lowest) {
      lowest = z;
      found = true;
    }
  }
  if (!found) {
    core::warn("render: none of the {} segmented planes is horizontal "
               "(|n_z| >= {}), so no floor could be identified; placing the "
               "cut plane from the bounding box instead",
               n_centroids, kHorizontalNormalZ);
    return std::nullopt;
  }
  core::debug("render: floor at z={:.3f} m, from {} horizontal plane(s) of {}",
              lowest, horizontal, n_centroids);
  return lowest;
}

/// One line-set actor in a single colour.
vtkActor *add_lines_actor(vtkRenderer *renderer, vtkPoints *points,
                          vtkCellArray *lines,
                          const std::array<std::uint8_t, 3> &rgb,
                          double width) {
  vtkNew<vtkPolyData> poly;
  poly->SetPoints(points);
  poly->SetLines(lines);
  vtkNew<vtkPolyDataMapper> mapper;
  mapper->SetInputData(poly);
  vtkNew<vtkActor> actor;
  actor->SetMapper(mapper);
  actor->GetProperty()->SetColor(rgb[0] / 255.0, rgb[1] / 255.0,
                                 rgb[2] / 255.0);
  actor->GetProperty()->SetLineWidth(width);
  actor->GetProperty()->SetLighting(false);
  renderer->AddActor(actor);
  return actor;
}

/// Camera frustums of the posed sensor frames, as one line set: the eye, the
/// four image corners at @p depth metres and the image rectangle. The camera
/// sits at pose * local_transform, as in camera_from_sensor_frame().
vtkActor *add_frustum_actor(vtkRenderer *renderer, const ProjectDB &db,
                            const SceneOptions &opts, SceneBounds &bounds,
                            std::size_t &drawn) {
  std::vector<int> posed;
  for (const int id : db.sensor_frame_ids())
    if (db.has_sensor_frame_pose(id))
      posed.push_back(id);
  if (posed.empty())
    throw std::runtime_error(
        "render: layer 'frustums' needs posed sensor frames, and this project "
        "has none — run `" +
        producing_command(Layer::frustums) + "` first");

  // Evenly spaced over the capture, so a long scan keeps its whole path.
  const std::size_t budget = std::max<std::size_t>(opts.max_frustums, 1);
  std::vector<int> ids;
  if (posed.size() <= budget) {
    ids = posed;
  } else {
    for (std::size_t i = 0; i < budget; ++i)
      ids.push_back(posed[i * posed.size() / budget]);
  }

  vtkNew<vtkPoints> points;
  points->SetDataTypeToFloat();
  vtkNew<vtkCellArray> lines;
  std::size_t skipped = 0;
  const double d = opts.frustum_depth;
  for (const int id : ids) {
    const core::SensorIntrinsics intr = db.sensor_frame_intrinsics(id);
    if (intr.fx <= 0.0 || intr.fy <= 0.0 || intr.width <= 0 ||
        intr.height <= 0) {
      ++skipped;
      continue;
    }
    const Eigen::Affine3d c2w = geometry::to_affine(db.sensor_frame_pose(id)) *
                                geometry::to_affine(intr.local_transform);
    const Eigen::Vector3d eye = c2w.translation();
    const double us[4] = {0.0, static_cast<double>(intr.width),
                          static_cast<double>(intr.width), 0.0};
    const double vs[4] = {0.0, 0.0, static_cast<double>(intr.height),
                          static_cast<double>(intr.height)};
    const vtkIdType e = points->InsertNextPoint(eye.x(), eye.y(), eye.z());
    bounds.add(eye.x(), eye.y(), eye.z());
    vtkIdType corner[4];
    for (int k = 0; k < 4; ++k) {
      const Eigen::Vector3d ray((us[k] - intr.cx) / intr.fx,
                                (vs[k] - intr.cy) / intr.fy, 1.0);
      const Eigen::Vector3d c = c2w * (ray * d);
      corner[k] = points->InsertNextPoint(c.x(), c.y(), c.z());
    }
    for (int k = 0; k < 4; ++k) {
      const vtkIdType spoke[2] = {e, corner[k]};
      lines->InsertNextCell(2, spoke);
      const vtkIdType edge[2] = {corner[k], corner[(k + 1) % 4]};
      lines->InsertNextCell(2, edge);
    }
    ++drawn;
  }
  if (drawn == 0)
    throw std::runtime_error("render: none of the " +
                             std::to_string(ids.size()) +
                             " posed sensor frames has usable intrinsics, so "
                             "no frustum can be drawn");
  if (skipped > 0)
    core::warn("render: {} of {} frames have no usable intrinsics; their "
               "frustums are not drawn",
               skipped, ids.size());
  return add_lines_actor(renderer, points, lines, opts.frustum_rgb, 1.0);
}

/// A marker where each placed 360 panorama was taken: its aligned pose, or
/// the timestamp-matched frame's pose. Points drawn as spheres.
vtkActor *add_panorama_actor(vtkRenderer *renderer, const ProjectDB &db,
                             const SceneOptions &opts, SceneBounds &bounds,
                             std::size_t &drawn) {
  const auto panoramas = db.list_panoramic_images();
  if (panoramas.empty())
    throw std::runtime_error(
        "render: layer 'panoramas' needs 360 panoramas, and this project has "
        "none — run `" +
        producing_command(Layer::panoramas) + "` first");
  vtkNew<vtkPoints> points;
  points->SetDataTypeToFloat();
  vtkNew<vtkCellArray> verts;
  std::size_t unplaced = 0;
  for (const auto &p : panoramas) {
    std::array<double, 16> pose{};
    if (p.has_pose)
      pose = p.pose;
    else if (p.node_id >= 0 && db.has_sensor_frame_pose(p.node_id))
      pose = db.sensor_frame_pose(p.node_id);
    else {
      ++unplaced;
      continue;
    }
    const vtkIdType id = points->InsertNextPoint(pose[3], pose[7], pose[11]);
    bounds.add(pose[3], pose[7], pose[11]);
    verts->InsertNextCell(1, &id);
    ++drawn;
  }
  if (drawn == 0)
    throw std::runtime_error("render: none of the " +
                             std::to_string(panoramas.size()) +
                             " panoramas has a pose or a posed matching frame "
                             "— run `rux align 360`");
  if (unplaced > 0)
    core::warn("render: {} of {} panoramas have no pose and no posed matching "
               "frame; they are not drawn",
               unplaced, panoramas.size());

  vtkNew<vtkPolyData> poly;
  poly->SetPoints(points);
  poly->SetVerts(verts);
  vtkNew<vtkPolyDataMapper> mapper;
  mapper->SetInputData(poly);
  vtkNew<vtkActor> actor;
  actor->SetMapper(mapper);
  const auto &rgb = opts.panorama_rgb;
  actor->GetProperty()->SetColor(rgb[0] / 255.0, rgb[1] / 255.0,
                                 rgb[2] / 255.0);
  actor->GetProperty()->SetPointSize(4.0 * opts.point_size);
  actor->GetProperty()->SetRenderPointsAsSpheres(true);
  actor->GetProperty()->SetLighting(false);
  renderer->AddActor(actor);
  return actor;
}

/// Parallel scale (half the viewport height in world units) that exactly fits
/// an @p across x @p up rectangle into an image of the given aspect ratio.
///
/// vtkRenderer::ResetCamera fits the bounding *sphere*, which for an
/// axis-aligned view wastes the whole depth extent — a 3.6 x 4.0 m room 2.9 m
/// tall ends up rendered at ~65 % of the size it could be. For the orthographic
/// presets the projected extents are known exactly, so compute the fit instead.
double parallel_scale_for(double across, double up, double aspect,
                          double margin) {
  const double half_up =
      std::max(0.5 * up, 0.5 * across / std::max(aspect, 1e-9));
  return std::max(half_up, 1e-6) * margin;
}

} // namespace

// ── Label palette and LOD ────────────────────────────────────────────────────

const LabelPalette &default_label_palette() {
  // tokens.css --label-0..7 and --label-unlabeled (Okabe-Ito); pinned to the
  // file by tests/unit/visualize/test_scene.cpp.
  // --label-invalid is PROVISIONAL and, unlike the rest of the scale, now
  // varies per theme (final review finding 4): a dim neutral in dark mode, a
  // muted one in light. This theme-unaware default mirrors the DARK value,
  // since headless `rux render` (the only consumer with no theme concept)
  // always draws on the near-black --color-canvas that both themes share.
  static const LabelPalette palette{{{0xe6, 0x9f, 0x00},
                                     {0x56, 0xb4, 0xe9},
                                     {0x00, 0x9e, 0x73},
                                     {0xf0, 0xe4, 0x42},
                                     {0x00, 0x72, 0xb2},
                                     {0xd5, 0x5e, 0x00},
                                     {0xcc, 0x79, 0xa7},
                                     {0x99, 0x99, 0x99}},
                                    {0x4a, 0x50, 0x5c},
                                    {0x2c, 0x3a, 0x48}};
  return palette;
}

int label_palette_slot(std::uint32_t label, std::size_t size) {
  if (core::is_out_of_contract_label(label))
    return kInvalidSlot;
  if (label == core::kUnlabeled || size == 0)
    return kUnlabeledSlot;
  return static_cast<int>((label - 1) % size);
}

std::array<std::uint8_t, 3> label_palette_color(const LabelPalette &palette,
                                                std::uint32_t label) {
  const int slot = label_palette_slot(label, palette.colors.size());
  if (slot == kInvalidSlot)
    return palette.invalid;
  return slot < 0 ? palette.unlabeled
                  : palette.colors[static_cast<std::size_t>(slot)];
}

std::string_view to_string(LodMethod method) {
  switch (method) {
  case LodMethod::all:
    return "all";
  case LodMethod::morton_prefix:
    return "morton_prefix";
  case LodMethod::stride:
    return "stride";
  }
  return "unknown";
}

std::vector<std::uint32_t> lod_indices(std::size_t total, std::size_t budget,
                                       std::string_view storage_order,
                                       LodMethod *method) {
  std::vector<std::uint32_t> out;
  if (budget == 0 || total <= budget) {
    if (method)
      *method = LodMethod::all;
    return out;
  }
  out.reserve(budget);
  if (storage_order == "morton_10bit_bitrev") {
    if (method)
      *method = LodMethod::morton_prefix;
    for (std::size_t i = 0; i < budget; ++i)
      out.push_back(static_cast<std::uint32_t>(i));
    return out;
  }
  if (method)
    *method = LodMethod::stride;
  for (std::size_t i = 0; i < budget; ++i)
    out.push_back(static_cast<std::uint32_t>(i * total / budget));
  return out;
}

// ── SceneBounds ──────────────────────────────────────────────────────────────

void SceneBounds::add(double x, double y, double z) {
  if (!valid) {
    min = {x, y, z};
    max = {x, y, z};
    valid = true;
    return;
  }
  min[0] = std::min(min[0], x);
  min[1] = std::min(min[1], y);
  min[2] = std::min(min[2], z);
  max[0] = std::max(max[0], x);
  max[1] = std::max(max[1], y);
  max[2] = std::max(max[2], z);
}

std::array<double, 3> SceneBounds::center() const {
  return {0.5 * (min[0] + max[0]), 0.5 * (min[1] + max[1]),
          0.5 * (min[2] + max[2])};
}

double SceneBounds::diagonal() const {
  const double dx = extent(0), dy = extent(1), dz = extent(2);
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// ── populate_scene ───────────────────────────────────────────────────────────

void install_vtk_log_bridge() {
  static const bool installed = [] {
    vtkLogger::SetStderrVerbosity(vtkLogger::VERBOSITY_OFF);
    vtkLogger::AddCallback("reusex", &vtk_log_callback, nullptr,
                           vtkLogger::VERBOSITY_INFO);
    return true;
  }();
  (void)installed;
}

SceneInfo populate_scene(vtkRenderer *renderer, const ProjectDB &db,
                         const SceneOptions &opts) {
  if (renderer == nullptr)
    throw std::invalid_argument("populate_scene: renderer is null");
  if (opts.layers.empty())
    throw std::runtime_error(
        "render: no layers selected — pass at least one of "
        "cloud, labels, planes, rooms, instances, mesh, "
        "components, frustums, panoramas");
  if (opts.point_size <= 0.0)
    throw std::runtime_error("render: point size must be positive (got " +
                             std::to_string(opts.point_size) + ")");

  const LabelPalette &palette =
      opts.palette ? *opts.palette : default_label_palette();
  SceneInfo info;
  SceneBounds &bounds = info.bounds;
  CloudPtr geometry; // loaded lazily; shared by every point layer
  std::vector<std::uint32_t> coarse_indices;
  bool coarse = false;

  const auto ensure_geometry = [&]() -> const Cloud & {
    if (!geometry) {
      geometry = load_geometry_cloud(db, opts.cloud_name);
      for (const auto &p : *geometry)
        bounds.add(p.x, p.y, p.z);
      // Which points to draw, the same for every point layer so a label
      // layer lines up with the geometry it recolours.
      const std::string order = db.point_cloud_storage_order(opts.cloud_name);
      info.source_points = geometry->size();
      info.indices =
          lod_indices(geometry->size(), opts.max_points, order, &info.lod);
      const std::size_t full =
          info.indices.empty() ? geometry->size() : info.indices.size();
      if (opts.coarse_points > 0 && opts.coarse_points < full) {
        coarse = true;
        coarse_indices =
            lod_indices(geometry->size(), opts.coarse_points, order, nullptr);
      }
      if (info.lod != LodMethod::all)
        core::info("render: drawing {} of {} points of '{}' ({})", full,
                   geometry->size(), opts.cloud_name, to_string(info.lod));
    }
    return *geometry;
  };

  // The full actor of a point layer, plus its hidden coarse twin.
  const auto add_point_layer = [&](SceneLayer &drawn, const Cloud &cloud,
                                   const std::vector<unsigned char> &colors) {
    drawn.actors.push_back(add_points_actor(
        renderer, make_point_polydata(cloud, colors, info.indices),
        opts.point_size));
    drawn.points = info.indices.empty() ? cloud.size() : info.indices.size();
    if (coarse) {
      drawn.coarse = add_points_actor(
          renderer, make_point_polydata(cloud, colors, coarse_indices),
          opts.point_size);
      drawn.coarse->SetVisibility(false);
    }
  };

  std::optional<std::vector<std::uint32_t>> highlight_labels;
  const auto highlight = [&](std::vector<unsigned char> &colors,
                             std::size_t points) {
    if (!opts.highlight)
      return;
    if (!highlight_labels) {
      const auto &h = *opts.highlight;
      if (!db.has_point_cloud(h.cloud_name))
        throw std::runtime_error("render: highlight needs the label cloud '" +
                                 h.cloud_name +
                                 "' — run `rux create instances` first");
      const CloudLPtr labels = db.point_cloud_label(h.cloud_name);
      highlight_labels.emplace();
      for (const auto &p : *labels)
        highlight_labels->push_back(p.label);
    }
    if (highlight_labels->size() != points)
      throw std::runtime_error(
          "render: highlight cloud '" + opts.highlight->cloud_name + "' has " +
          std::to_string(highlight_labels->size()) + " labels but '" +
          opts.cloud_name + "' has " + std::to_string(points) +
          " points — the clouds are out of sync");
    if (apply_instance_highlight(colors, *highlight_labels,
                                 opts.highlight->instance_id) == 0)
      core::warn(
          "render: instance {} has no points in '{}'; nothing is highlighted",
          opts.highlight->instance_id, opts.highlight->cloud_name);
  };

  for (const Layer layer : opts.layers) {
    SceneLayer drawn;
    drawn.layer = layer;
    switch (layer) {
    case Layer::cloud: {
      const Cloud &cloud = ensure_geometry();
      std::vector<unsigned char> colors(cloud.size() * 3);
      for (std::size_t i = 0; i < cloud.size(); ++i) {
        colors[3 * i + 0] = cloud[i].r;
        colors[3 * i + 1] = cloud[i].g;
        colors[3 * i + 2] = cloud[i].b;
      }
      highlight(colors, cloud.size());
      add_point_layer(drawn, cloud, colors);
      break;
    }

    case Layer::labels:
    case Layer::planes:
    case Layer::rooms:
    case Layer::instances: {
      const std::string name = label_cloud_name(layer);
      if (!db.has_point_cloud(name)) {
        throw std::runtime_error("render: layer '" +
                                 std::string(to_string(layer)) +
                                 "' needs the label cloud '" + name +
                                 "', which this project does not have — run `" +
                                 producing_command(layer) + "` first");
      }
      const Cloud &cloud = ensure_geometry();
      const CloudLPtr labels = db.point_cloud_label(name);
      if (!labels || labels->empty()) {
        throw std::runtime_error("render: label cloud '" + name +
                                 "' is empty — re-run `" +
                                 producing_command(layer) + "`");
      }
      if (labels->size() != cloud.size()) {
        // The per-scan clouds are index-aligned by contract (STANDARDS §3.2);
        // a mismatch means one of them is stale.
        throw std::runtime_error(
            "render: '" + name + "' has " + std::to_string(labels->size()) +
            " labels but '" + opts.cloud_name + "' has " +
            std::to_string(cloud.size()) +
            " points — the clouds are out of sync; re-run `" +
            producing_command(layer) + "`");
      }

      std::vector<unsigned char> colors(cloud.size() * 3);
      std::size_t unlabeled = 0;
      std::size_t invalid = 0;
      for (std::size_t i = 0; i < cloud.size(); ++i) {
        const std::uint32_t label = (*labels)[i].label;
        if (label == core::kUnlabeled)
          ++unlabeled;
        else if (core::is_out_of_contract_label(label))
          ++invalid;
        const auto c = label_palette_color(palette, label);
        colors[3 * i + 0] = c[0];
        colors[3 * i + 1] = c[1];
        colors[3 * i + 2] = c[2];
      }
      if (unlabeled == cloud.size()) {
        core::warn("render: every point in '{}' is unlabeled (0/{} labeled); "
                   "the '{}' layer will be uniformly grey",
                   name, cloud.size(), to_string(layer));
      }
      if (invalid > 0) {
        // STANDARDS §3.1: a -1 that wrapped to 0xFFFFFFFF. Drawn in the
        // palette's non-class `invalid` colour; the data needs fixing.
        core::warn("render: {} of {} points in '{}' carry an out-of-contract "
                   "label (> INT32_MAX, a wrapped -1); they are drawn as "
                   "invalid, not as a class — re-run `{}`",
                   invalid, cloud.size(), name, producing_command(layer));
      }
      highlight(colors, cloud.size());
      add_point_layer(drawn, cloud, colors);
      break;
    }

    case Layer::mesh: {
      if (!db.has_mesh(opts.mesh_name)) {
        throw std::runtime_error("render: project has no mesh named '" +
                                 opts.mesh_name + "' — run `" +
                                 producing_command(Layer::mesh) + "` first");
      }
      const auto mesh = db.mesh(opts.mesh_name);
      if (!mesh || mesh->polygons.empty()) {
        throw std::runtime_error("render: mesh '" + opts.mesh_name +
                                 "' has no faces — re-run `" +
                                 producing_command(Layer::mesh) + "`");
      }
      drawn.actors.push_back(add_mesh_actor(renderer, *mesh, bounds));
      drawn.faces = mesh->polygons.size();
      break;
    }

    case Layer::components: {
      const auto names = db.list_building_components();
      if (names.empty()) {
        throw std::runtime_error(
            "render: project has no building components — run `" +
            producing_command(Layer::components) + "` first");
      }
      std::vector<geometry::BuildingComponent> components;
      components.reserve(names.size());
      for (const auto &name : names)
        components.push_back(geometry::building_component(db, name));
      drawn.actors = add_component_actors(renderer, components, bounds);
      drawn.items = drawn.actors.size();
      break;
    }

    case Layer::frustums:
      drawn.actors.push_back(
          add_frustum_actor(renderer, db, opts, bounds, drawn.items));
      break;

    case Layer::panoramas:
      drawn.actors.push_back(
          add_panorama_actor(renderer, db, opts, bounds, drawn.items));
      break;
    }
    info.drawn_points += drawn.points;
    info.drawn_faces += drawn.faces;
    info.layers.push_back(std::move(drawn));
  }

  if (!bounds.valid) {
    throw std::runtime_error(
        "render: the selected layers produced no drawable geometry");
  }
  return info;
}

// ── Cut plane ────────────────────────────────────────────────────────────────

CutPlane resolve_cut_plane(const ProjectDB &db, const SceneBounds &bounds,
                           std::optional<double> height) {
  CutPlane cut;

  const std::optional<double> floor = detect_floor_z(db);
  if (floor && *floor >= bounds.min[2] - kFloorPlaneSlackM &&
      *floor <= bounds.max[2]) {
    cut.floor_z = *floor;
    cut.from_planes = true;
  } else {
    if (floor) {
      core::warn("render: the detected floor plane (z={:.3f}) lies outside the "
                 "drawn geometry (z in [{:.3f}, {:.3f}]); the per-plane clouds "
                 "are stale — placing the cut plane from the bounding box",
                 *floor, bounds.min[2], bounds.max[2]);
    }
    cut.floor_z = bounds.min[2];
  }

  cut.height = height.value_or(cut.from_planes ? kDefaultCutHeightM
                                               : kDefaultCutBboxFraction *
                                                     bounds.extent(2));
  cut.z = cut.floor_z + cut.height;

  // Neither of these is fatal — the render still means something — but a plan
  // that quietly turned back into a top view, or into an empty frame, must not
  // pass unremarked (STANDARDS §5).
  if (cut.z >= bounds.max[2]) {
    core::warn("render: the cut plane (z={:.3f}) is above everything drawn "
               "(z <= {:.3f}); nothing is cut away and this is a plain top "
               "view",
               cut.z, bounds.max[2]);
  } else if (cut.z <= bounds.min[2]) {
    core::warn("render: the cut plane (z={:.3f}) is below everything drawn "
               "(z >= {:.3f}); the frame will be empty — lower --cut-height or "
               "check the floor",
               cut.z, bounds.min[2]);
  }
  return cut;
}

/// vtkPlane keeps the side its normal points to, so the normal points down.
/// Clipping planes live on the mapper and are evaluated in the vertex shader,
/// which is what makes this work uniformly for point sprites, mesh triangles
/// and component polylines alike.
void apply_cut_plane(vtkRenderer *renderer, double z) {
  vtkNew<vtkPlane> plane;
  plane->SetOrigin(0.0, 0.0, z);
  plane->SetNormal(0.0, 0.0, -1.0);

  vtkActorCollection *actors = renderer->GetActors();
  actors->InitTraversal();
  while (vtkActor *actor = actors->GetNextActor()) {
    if (vtkMapper *mapper = actor->GetMapper())
      mapper->AddClippingPlane(plane);
  }
}

void clear_cut_planes(vtkRenderer *renderer) {
  vtkActorCollection *actors = renderer->GetActors();
  actors->InitTraversal();
  while (vtkActor *actor = actors->GetNextActor()) {
    if (vtkMapper *mapper = actor->GetMapper())
      mapper->RemoveAllClippingPlanes();
  }
}

// ── Camera placement ─────────────────────────────────────────────────────────

/// The view direction, up vector and projection type are always set
/// explicitly. Orthographic presets also get an exact parallel scale; the
/// perspective orbit delegates its distance to vtkRenderer::ResetCamera, whose
/// bounding-sphere fit is the right behaviour there because it keeps the scene
/// the same size at every point on the ring. Either way the result is a pure
/// function of the geometry, so the same project always frames the same shot
/// (STANDARDS §6).
void place_preset_camera(vtkRenderer *renderer, const CameraFraming &framing,
                         const SceneBounds &bounds) {
  const std::array<double, 3> c = bounds.center();
  double center[3] = {c[0], c[1], c[2]};
  const double reach = std::max(bounds.diagonal(), 1.0);
  const double aspect = framing.aspect;
  const double yaw = framing.yaw_deg * std::numbers::pi / 180.0;
  const double cy = std::cos(yaw), sy = std::sin(yaw);
  // The camera turns by -yaw so the model appears turned by +yaw. Extents in
  // the turned frame decide the framing.
  const double ex =
      std::abs(cy) * bounds.extent(0) + std::abs(sy) * bounds.extent(1);
  const double ey =
      std::abs(sy) * bounds.extent(0) + std::abs(cy) * bounds.extent(1);

  vtkCamera *cam = renderer->GetActiveCamera();
  cam->SetFocalPoint(center);

  switch (framing.view) {
  case ViewPreset::top:
  case ViewPreset::plan:
    // Straight down -Z with +Y up the page (turned by the yaw). `plan`
    // differs from `top` only in the cut plane applied to the actors, so the
    // two frame identically.
    cam->ParallelProjectionOn();
    cam->SetPosition(center[0], center[1], center[2] + reach);
    cam->SetViewUp(sy, cy, 0.0);
    cam->SetParallelScale(parallel_scale_for(ex, ey, aspect, framing.margin));
    renderer->ResetCameraClippingRange();
    return;

  case ViewPreset::front:
    // Orthographic elevation looking along +Y (turned by the yaw), world Z
    // up.
    cam->ParallelProjectionOn();
    cam->SetPosition(center[0] - reach * sy, center[1] - reach * cy, center[2]);
    cam->SetViewUp(0.0, 0.0, 1.0);
    cam->SetParallelScale(
        parallel_scale_for(ex, bounds.extent(2), aspect, framing.margin));
    renderer->ResetCameraClippingRange();
    return;

  case ViewPreset::orbit: {
    cam->ParallelProjectionOff();
    cam->SetViewAngle(40.0);
    const double azimuth = 2.0 * std::numbers::pi *
                               static_cast<double>(framing.orbit_index) /
                               static_cast<double>(framing.orbit_count) -
                           yaw;
    const double elevation =
        framing.orbit_elevation_deg * std::numbers::pi / 180.0;
    cam->SetPosition(
        center[0] + reach * std::cos(azimuth) * std::cos(elevation),
        center[1] + reach * std::sin(azimuth) * std::cos(elevation),
        center[2] + reach * std::sin(elevation));
    cam->SetViewUp(0.0, 0.0, 1.0);
    break;
  }

  case ViewPreset::explicit_camera:
    throw std::invalid_argument(
        "place_preset_camera: explicit_camera is not a preset; use "
        "place_explicit_camera");
  }

  // Fit @p bounds, not the renderer's visible props: a viewer frames the cut
  // part of the scene (and hidden LOD twins must not count either).
  double box[6] = {bounds.min[0], bounds.max[0], bounds.min[1],
                   bounds.max[1], bounds.min[2], bounds.max[2]};
  renderer->ResetCamera(box);
  cam->Zoom(1.0 / framing.margin); // margin is validated positive
  renderer->ResetCameraClippingRange();
}

void place_explicit_camera(vtkRenderer *renderer, const CameraSpec &spec,
                           int width, int height, const SceneBounds &bounds) {
  if (spec.fy <= 0.0 || spec.fx <= 0.0) {
    throw std::runtime_error(
        "render: explicit camera needs positive fx/fy (got fx=" +
        std::to_string(spec.fx) + ", fy=" + std::to_string(spec.fy) + ")");
  }

  const Eigen::Affine3f c2w = geometry::to_affine(spec.pose).cast<float>();
  const Eigen::Vector3f eye = c2w.translation();
  // OpenCV camera axes: +x right, +y down, +z along the view direction.
  const Eigen::Vector3f forward = c2w.linear() * Eigen::Vector3f::UnitZ();
  const Eigen::Vector3f down = c2w.linear() * Eigen::Vector3f::UnitY();

  vtkCamera *cam = renderer->GetActiveCamera();
  cam->ParallelProjectionOff();
  cam->SetPosition(eye.x(), eye.y(), eye.z());
  const Eigen::Vector3f target =
      eye + forward * static_cast<float>(std::max(bounds.diagonal(), 1.0));
  cam->SetFocalPoint(target.x(), target.y(), target.z());
  cam->SetViewUp(-down.x(), -down.y(), -down.z());

  // Vertical field of view from fy and the image height.
  const double view_angle =
      2.0 * std::atan2(0.5 * static_cast<double>(height), spec.fy) * 180.0 /
      std::numbers::pi;
  cam->SetViewAngle(view_angle);

  if (std::abs(spec.fx - spec.fy) > 1e-3 * spec.fy) {
    core::debug("render: anisotropic intrinsics (fx={:.2f}, fy={:.2f}); VTK "
                "renders square pixels, using fy",
                spec.fx, spec.fy);
  }

  // Principal-point offset. VTK's window center is a normalized shift of the
  // projection center with +x right and +y up, opposite in sign to an image
  // principal point measured from the top-left. Only the centered case is
  // covered by tests; see #294.
  if (spec.cx > 0.0 || spec.cy > 0.0) {
    const double wcx = -2.0 * (spec.cx - 0.5 * static_cast<double>(width)) /
                       static_cast<double>(width);
    const double wcy = 2.0 * (spec.cy - 0.5 * static_cast<double>(height)) /
                       static_cast<double>(height);
    if (std::abs(wcx) > 1e-6 || std::abs(wcy) > 1e-6)
      cam->SetWindowCenter(wcx, wcy);
  }

  renderer->ResetCameraClippingRange();
}

} // namespace reusex::visualize
