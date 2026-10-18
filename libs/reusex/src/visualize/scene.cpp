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
#include "geometry/point_mesh_distance.hpp"
#include "geometry/transform_utils.hpp"
#include "reusex/core/space_polygon.hpp"
#include "reusex/visualize/highlight.hpp"
#include "types/point_types.hpp"
#include "visualize/drift_layers.hpp"

#include <fmt/format.h>
#include <nlohmann/json.hpp>
#include <pcl/PolygonMesh.h>
#include <pcl/conversions.h>

#include <vtkActor.h>
#include <vtkActorCollection.h>
#include <vtkCamera.h>
#include <vtkCellArray.h>
#include <vtkCellData.h>
#include <vtkCleanPolyData.h>
#include <vtkCutter.h>
#include <vtkFeatureEdges.h>
#include <vtkIdList.h>
#include <vtkLogger.h>
#include <vtkMapper.h>
#include <vtkNew.h>
#include <vtkPlane.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolygon.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkUnsignedCharArray.h>
#include <vtkVectorText.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <map>
#include <numbers>
#include <stdexcept>
#include <string>
#include <unordered_map>

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
  case Layer::deviation:
    return "rux create mesh";
  case Layer::passes:
  case Layer::graph:
    return "rux import rtabmap (or another importer of posed depth frames)";
  case Layer::drift:
    return "rux analyze drift -o drift.json";
  case Layer::walls:
    return "rux create walls";
  case Layer::openings:
    return "rux create openings";
  case Layer::spaces:
    return "rux create spaces";
  case Layer::solids:
    return "rux create solids";
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

/// A stored PolygonMesh as VTK poly-data, coincident vertices merged (an
/// unwelded mesh would otherwise show every triangle edge as a boundary).
vtkSmartPointer<vtkPolyData> mesh_polydata(const pcl::PolygonMesh &mesh,
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

  vtkNew<vtkCleanPolyData> clean;
  clean->SetInputData(poly);
  clean->PointMergingOn();
  clean->ToleranceIsAbsoluteOn();
  clean->SetAbsoluteTolerance(0.0);
  clean->ConvertPolysToLinesOff();
  clean->ConvertLinesToPointsOff();
  clean->ConvertStripsToPolysOff();
  clean->Update();
  auto out = vtkSmartPointer<vtkPolyData>::New();
  out->ShallowCopy(clean->GetOutput());
  return out;
}

/// The surface actor of a mesh layer.
vtkActor *add_mesh_surface_actor(vtkRenderer *renderer, vtkPolyData *poly,
                                 double opacity) {
  vtkNew<vtkPolyDataMapper> mapper;
  mapper->SetInputData(poly);
  mapper->ScalarVisibilityOff();

  vtkNew<vtkActor> actor;
  actor->SetMapper(mapper);
  actor->GetProperty()->SetColor(0.82, 0.80, 0.76);
  actor->GetProperty()->SetAmbient(0.25);
  actor->GetProperty()->SetDiffuse(0.75);
  actor->GetProperty()->SetSpecular(0.05);
  actor->GetProperty()->SetOpacity(opacity);
  if (opacity < 1.0) {
    // Seen from either side: an interior mesh is mostly seen from inside.
    actor->GetProperty()->BackfaceCullingOff();
  }
  renderer->AddActor(actor);
  return actor;
}

/// The feature edges (boundary, non-manifold and crease edges) of a mesh.
vtkActor *add_mesh_edges_actor(vtkRenderer *renderer, vtkPolyData *poly,
                               const SceneOptions &opts) {
  vtkNew<vtkFeatureEdges> edges;
  edges->SetInputData(poly);
  edges->BoundaryEdgesOn();
  edges->FeatureEdgesOn();
  edges->NonManifoldEdgesOn();
  edges->ManifoldEdgesOff();
  edges->ColoringOff();
  edges->SetFeatureAngle(opts.mesh_feature_angle_deg);
  edges->Update();
  vtkNew<vtkPolyData> lines;
  lines->ShallowCopy(edges->GetOutput());

  vtkNew<vtkPolyDataMapper> mapper;
  mapper->SetInputData(lines);
  mapper->ScalarVisibilityOff();
  vtkNew<vtkActor> actor;
  actor->SetMapper(mapper);
  const auto &rgb = opts.mesh_edge_rgb;
  actor->GetProperty()->SetColor(rgb[0] / 255.0, rgb[1] / 255.0,
                                 rgb[2] / 255.0);
  actor->GetProperty()->SetLineWidth(2.0);
  actor->GetProperty()->SetLighting(false);
  renderer->AddActor(actor);
  return actor;
}

/// Draw a stored mesh in @p style into @p drawn.
void add_mesh_layer(vtkRenderer *renderer, const pcl::PolygonMesh &mesh,
                    MeshStyle style, const SceneOptions &opts,
                    SceneBounds &bounds, SceneLayer &drawn) {
  const vtkSmartPointer<vtkPolyData> poly = mesh_polydata(mesh, bounds);
  switch (style) {
  case MeshStyle::automatic:
  case MeshStyle::solid:
    drawn.actors.push_back(add_mesh_surface_actor(renderer, poly, 1.0));
    break;
  case MeshStyle::overlay:
    drawn.actors.push_back(
        add_mesh_surface_actor(renderer, poly, opts.mesh_overlay_opacity));
    drawn.actors.push_back(add_mesh_edges_actor(renderer, poly, opts));
    break;
  case MeshStyle::edges: {
    // The surface stays in the renderer, hidden, as the section-line source.
    vtkActor *hidden = add_mesh_surface_actor(renderer, poly, 1.0);
    hidden->SetVisibility(false);
    drawn.actors.push_back(add_mesh_edges_actor(renderer, poly, opts));
    break;
  }
  }
  drawn.mesh_surface = poly; // kept alive by the surface actor's mapper
}

/// Squared-free linear blend of two colours.
std::array<std::uint8_t, 3> lerp_rgb(const std::array<std::uint8_t, 3> &a,
                                     const std::array<std::uint8_t, 3> &b,
                                     double t) {
  std::array<std::uint8_t, 3> out{};
  for (int k = 0; k < 3; ++k)
    out[k] = static_cast<std::uint8_t>(
        std::lround(a[k] + (static_cast<double>(b[k]) - a[k]) * t));
  return out;
}

DeviationStats to_stats(const geometry::DistanceSummary &s) {
  DeviationStats d;
  d.points = s.points;
  d.mean_m = s.mean_m;
  d.median_m = s.median_m;
  d.p95_m = s.p95_m;
  d.max_m = s.max_m;
  d.within_share = s.within_share;
  d.uncovered_share = s.beyond_share;
  return d;
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

// ── Plan overlay (reusex-z1v.2.6) ────────────────────────────────────────────
//
// A solid-model plan drawn over the scene, the visual check of an authored
// reference. Everything is built in the level frame of the element's storey
// and mapped to world through the inverse of that storey's level_transform.

/// Lift of a PlanPlacement::level overlay above its floor, so a footprint
/// does not z-fight the floor points it lies on [m].
constexpr double kPlanLevelLiftM = 0.02;
/// Gap of a PlanPlacement::above_scene sheet over the drawn geometry [m].
constexpr double kPlanAboveGapM = 0.05;
/// Half-length of an opening's end tick beyond the wall face [m]; also the
/// half-width used for a wall of unknown thickness.
constexpr double kOpeningTickM = 0.1;
constexpr double kPlanFillOpacity = 0.30;
constexpr double kSpaceFillOpacity = 0.12;
/// Label heights in the plan [m].
constexpr double kSpaceLabelM = 0.30;
constexpr double kWallLabelM = 0.15;

/// Accumulates one actor's worth of plan geometry in world coordinates.
struct PlanGeometry {
  vtkNew<vtkPoints> points;
  vtkNew<vtkCellArray> lines;
  vtkNew<vtkCellArray> polys;
  PlanGeometry() { points->SetDataType(VTK_DOUBLE); }
  bool empty() const { return points->GetNumberOfPoints() == 0; }
};

class PlanBuilder {
    public:
  PlanBuilder(const core::SolidModel &plan, PlanPlacement placement,
              const SceneBounds &scene,
              std::optional<double> sheet_z = std::nullopt)
      : plan_(plan), placement_(placement) {
    if (placement_ == PlanPlacement::above_scene && sheet_z) {
      sheet_z_ = *sheet_z;
    } else if (placement_ == PlanPlacement::above_scene) {
      const double gap = scene.valid
                             ? std::max(kPlanAboveGapM, 0.01 * scene.extent(2))
                             : kPlanAboveGapM;
      sheet_z_ = (scene.valid ? scene.max[2] : 0.0) + gap;
    }
  }

  /// A level-frame point of storey @p storey at height @p z, in world.
  Eigen::Vector3d world(int storey, double x, double y, double z) {
    auto it = to_world_.find(storey);
    if (it == to_world_.end()) {
      Eigen::Affine3d t = Eigen::Affine3d::Identity();
      for (const auto &s : plan_.storeys)
        if (s.idx == storey)
          t = geometry::to_affine(s.level_transform).inverse();
      it = to_world_.emplace(storey, t).first;
    }
    Eigen::Vector3d p = it->second * Eigen::Vector3d(x, y, z + kPlanLevelLiftM);
    if (placement_ == PlanPlacement::above_scene)
      p.z() = sheet_z_;
    return p;
  }

  vtkIdType add(PlanGeometry &g, const Eigen::Vector3d &p) {
    points_seen_.push_back(p);
    return g.points->InsertNextPoint(p.x(), p.y(), p.z());
  }

  /// A closed ring as an outline (and a filled polygon when @p fill).
  void ring(PlanGeometry &outline, PlanGeometry *fill,
            const std::vector<Eigen::Vector3d> &pts) {
    const auto n = static_cast<vtkIdType>(pts.size());
    std::vector<vtkIdType> ids;
    for (const auto &p : pts)
      ids.push_back(add(outline, p));
    for (vtkIdType i = 0; i < n; ++i) {
      const vtkIdType seg[2] = {ids[static_cast<std::size_t>(i)],
                                ids[static_cast<std::size_t>((i + 1) % n)]};
      outline.lines->InsertNextCell(2, seg);
    }
    if (fill != nullptr) {
      std::vector<vtkIdType> fids;
      for (const auto &p : pts)
        fids.push_back(add(*fill, p));
      fill->polys->InsertNextCell(n, fids.data());
    }
  }

  void segment(PlanGeometry &g, const Eigen::Vector3d &a,
               const Eigen::Vector3d &b) {
    const vtkIdType seg[2] = {add(g, a), add(g, b)};
    g.lines->InsertNextCell(2, seg);
  }

  /// Fold every overlay point into @p bounds: fully for a level overlay, in
  /// plan only for the sheet (its z must not move the cut's reference).
  void extend(SceneBounds &bounds) const {
    const bool sheet = placement_ == PlanPlacement::above_scene && bounds.valid;
    const double zlo = bounds.min[2], zhi = bounds.max[2];
    for (const auto &p : points_seen_)
      bounds.add(p.x(), p.y(), sheet ? std::clamp(p.z(), zlo, zhi) : p.z());
  }

    private:
  const core::SolidModel &plan_;
  PlanPlacement placement_;
  double sheet_z_ = 0.0;
  std::unordered_map<int, Eigen::Affine3d> to_world_;
  std::vector<Eigen::Vector3d> points_seen_;
};

vtkActor *add_plan_actor(vtkRenderer *renderer, PlanGeometry &g,
                         const std::array<std::uint8_t, 3> &rgb, double width,
                         double opacity) {
  vtkNew<vtkPolyData> poly;
  poly->SetPoints(g.points);
  if (g.lines->GetNumberOfCells() > 0)
    poly->SetLines(g.lines);
  if (g.polys->GetNumberOfCells() > 0)
    poly->SetPolys(g.polys);
  vtkNew<vtkPolyDataMapper> mapper;
  mapper->SetInputData(poly);
  vtkNew<vtkActor> actor;
  actor->SetMapper(mapper);
  actor->SetObjectName(kPlanOverlayActorName);
  actor->GetProperty()->SetColor(rgb[0] / 255.0, rgb[1] / 255.0,
                                 rgb[2] / 255.0);
  actor->GetProperty()->SetLineWidth(width);
  actor->GetProperty()->SetOpacity(opacity);
  actor->GetProperty()->SetLighting(false);
  renderer->AddActor(actor);
  return actor;
}

/// A label as stroke-font geometry (vtkVectorText) lying flat in the plan,
/// centred on @p at, @p height metres tall. Geometry rather than a
/// screen-space text actor: it needs no text renderer or render window to
/// build (vtkBillboardTextActor3D crashes in vtkRenderer::GetActors() without
/// one), scales with the plan, and reads correctly from above.
vtkActor *
add_plan_label(vtkRenderer *renderer, const std::string &text,
               const Eigen::Vector3d &at,
               const std::array<std::uint8_t, 3> &rgb, double height,
               const Eigen::Vector2d &dir = Eigen::Vector2d::UnitX()) {
  vtkNew<vtkVectorText> source;
  source->SetText(text.c_str());
  source->Update();
  vtkPolyData *glyphs = source->GetOutput();
  double b[6];
  glyphs->GetBounds(b);
  // vtkVectorText's capitals are about one unit tall.
  const double scale = height;
  const double cx = 0.5 * (b[0] + b[1]), cy = 0.5 * (b[2] + b[3]);
  // The text's baseline runs along @p dir (world x by default).
  const Eigen::Vector2d u =
      dir.norm() > 0.0 ? dir.normalized() : Eigen::Vector2d::UnitX();
  const Eigen::Vector2d v(-u.y(), u.x());
  vtkNew<vtkPoints> points;
  points->SetDataType(VTK_DOUBLE);
  for (vtkIdType i = 0; i < glyphs->GetNumberOfPoints(); ++i) {
    double p[3];
    glyphs->GetPoint(i, p);
    const Eigen::Vector2d q =
        u * (scale * (p[0] - cx)) + v * (scale * (p[1] - cy));
    points->InsertNextPoint(at.x() + q.x(), at.y() + q.y(), at.z());
  }
  vtkNew<vtkPolyData> poly;
  poly->SetPoints(points);
  poly->SetPolys(glyphs->GetPolys());
  vtkNew<vtkPolyDataMapper> mapper;
  mapper->SetInputData(poly);
  vtkNew<vtkActor> actor;
  actor->SetMapper(mapper);
  actor->SetObjectName(kPlanOverlayActorName);
  actor->GetProperty()->SetColor(rgb[0] / 255.0, rgb[1] / 255.0,
                                 rgb[2] / 255.0);
  actor->GetProperty()->SetLighting(false);
  renderer->AddActor(actor);
  return actor;
}

/// Area centroid of a ring (vertex mean for a degenerate one).
std::array<double, 2> ring_centroid(const std::vector<core::PlanPoint> &ring) {
  double a = 0.0, cx = 0.0, cy = 0.0;
  for (std::size_t i = 0; i < ring.size(); ++i) {
    const auto &p = ring[i];
    const auto &q = ring[(i + 1) % ring.size()];
    const double c = p[0] * q[1] - q[0] * p[1];
    a += c;
    cx += (p[0] + q[0]) * c;
    cy += (p[1] + q[1]) * c;
  }
  if (std::abs(a) < 1e-12) {
    double mx = 0.0, my = 0.0;
    for (const auto &p : ring)
      mx += p[0], my += p[1];
    const double n = static_cast<double>(std::max<std::size_t>(ring.size(), 1));
    return {mx / n, my / n};
  }
  return {cx / (3.0 * a), cy / (3.0 * a)};
}

PlanOverlayInfo add_plan_overlay_to(vtkRenderer *renderer,
                                    const core::SolidModel &plan,
                                    const SceneOptions &opts,
                                    SceneBounds &bounds) {
  PlanOverlayInfo info;
  PlanBuilder b(plan, opts.plan_placement, bounds, opts.plan_sheet_z);
  PlanGeometry wall_fill, wall_outline, wall_axis, wall_face, space_fill,
      space_outline, doors, windows, openings;
  struct Label {
    std::string text;
    Eigen::Vector3d at;
    bool space;
  };
  std::vector<Label> labels;

  // Walls: footprint (fill + outline) and axis.
  std::unordered_map<std::string, const core::WallRecord *> walls;
  for (const auto &w : plan.walls) {
    const Eigen::Vector2d p0(w.x0, w.y0), p1(w.x1, w.y1);
    const double len = (p1 - p0).norm();
    if (!(len > 0.0)) {
      core::warn("render: plan wall '{}' has a zero-length axis; not drawn",
                 w.guid);
      continue;
    }
    walls.emplace(w.guid, &w);
    const Eigen::Vector2d d = (p1 - p0) / len;
    const Eigen::Vector2d n(-d.y(), d.x());
    const double z = w.z_base;
    // A wall of unknown thickness is only its axis: draw it heavier.
    b.segment(w.thickness > 0.0 ? wall_axis : wall_face,
              b.world(w.storey, p0.x(), p0.y(), z),
              b.world(w.storey, p1.x(), p1.y(), z));
    if (w.thickness > 0.0) {
      const double h = 0.5 * w.thickness;
      std::vector<Eigen::Vector3d> corners;
      for (const Eigen::Vector2d &c :
           {Eigen::Vector2d(p0 + n * h), Eigen::Vector2d(p1 + n * h),
            Eigen::Vector2d(p1 - n * h), Eigen::Vector2d(p0 - n * h)})
        corners.push_back(b.world(w.storey, c.x(), c.y(), z));
      b.ring(wall_outline, &wall_fill, corners);
    }
    const Eigen::Vector2d mid = 0.5 * (p0 + p1);
    labels.push_back({w.guid, b.world(w.storey, mid.x(), mid.y(), z), false});
    ++info.walls;
  }

  // Spaces: outer ring filled, holes outlined (the fill does not cut holes).
  for (const auto &s : plan.spaces) {
    if (s.polygon.outer.size() < 3) {
      core::warn("render: plan space '{}' has fewer than 3 points; not drawn",
                 s.guid);
      continue;
    }
    const auto to_world = [&](const std::vector<core::PlanPoint> &ring) {
      std::vector<Eigen::Vector3d> out;
      for (const auto &p : ring)
        out.push_back(b.world(s.storey, p[0], p[1], s.z_floor));
      return out;
    };
    b.ring(space_outline, &space_fill, to_world(s.polygon.outer));
    for (const auto &hole : s.polygon.holes)
      if (hole.size() >= 3)
        b.ring(space_outline, nullptr, to_world(hole));
    const auto c = ring_centroid(s.polygon.outer);
    labels.push_back({s.name.empty() ? s.guid : s.name,
                      b.world(s.storey, c[0], c[1], s.z_floor), true});
    ++info.spaces;
  }

  // Openings: a bar along the host axis and a tick across the wall at each
  // end.
  std::size_t orphans = 0;
  for (const auto &o : plan.openings) {
    const auto it = walls.find(o.wall);
    if (it == walls.end()) {
      ++orphans;
      continue;
    }
    const core::WallRecord &w = *it->second;
    const Eigen::Vector2d p0(w.x0, w.y0), p1(w.x1, w.y1);
    const Eigen::Vector2d d = (p1 - p0).normalized();
    const Eigen::Vector2d n(-d.y(), d.x());
    const double half =
        (w.thickness > 0.0 ? 0.5 * w.thickness : 0.0) + kOpeningTickM;
    PlanGeometry &g = o.kind == "door"     ? doors
                      : o.kind == "window" ? windows
                                           : openings;
    const Eigen::Vector2d a = p0 + d * o.s_start, e = p0 + d * o.s_end;
    const double z = w.z_base;
    b.segment(g, b.world(w.storey, a.x(), a.y(), z),
              b.world(w.storey, e.x(), e.y(), z));
    for (const Eigen::Vector2d &c : {a, e}) {
      const Eigen::Vector2d u = c + n * half, v = c - n * half;
      b.segment(g, b.world(w.storey, u.x(), u.y(), z),
                b.world(w.storey, v.x(), v.y(), z));
    }
    ++info.openings;
  }
  if (orphans > 0)
    core::warn("render: {} of {} plan openings name a wall the plan does not "
               "draw; they are skipped",
               orphans, plan.openings.size());

  // Translucent fills first, so the outlines draw over them.
  const auto add = [&](PlanGeometry &g, const std::array<std::uint8_t, 3> &rgb,
                       double width, double opacity) {
    if (!g.empty())
      info.actors.push_back(add_plan_actor(renderer, g, rgb, width, opacity));
  };
  add(space_fill, opts.plan_space_rgb, 1.0, kSpaceFillOpacity);
  add(wall_fill, opts.plan_wall_rgb, 1.0, kPlanFillOpacity);
  add(space_outline, opts.plan_space_rgb, 2.0, 1.0);
  add(wall_outline, opts.plan_wall_rgb, 2.0, 1.0);
  add(wall_axis, opts.plan_wall_rgb, 1.0, 1.0);
  add(wall_face, opts.plan_wall_rgb, 3.0, 1.0);
  add(openings, opts.plan_opening_rgb, 5.0, 1.0);
  add(windows, opts.plan_window_rgb, 5.0, 1.0);
  add(doors, opts.plan_door_rgb, 5.0, 1.0);

  if (opts.plan_labels)
    for (const auto &l : labels)
      info.labels.push_back(
          add_plan_label(renderer, l.text, l.at,
                         l.space ? opts.plan_space_rgb : opts.plan_wall_rgb,
                         l.space ? kSpaceLabelM : kWallLabelM));

  b.extend(bounds);
  if (info.walls + info.spaces + info.openings == 0)
    core::warn("render: the plan overlay has no wall, space or opening to "
               "draw");
  else
    core::info("render: plan overlay: {} walls, {} spaces, {} openings",
               info.walls, info.spaces, info.openings);
  return info;
}

// ── Walls layer (reusex-z1v.5.6) ─────────────────────────────────────────────
//
// The `walls` table drawn from its axes, thicknesses and heights, in each
// wall's storey frame mapped to world (the plan overlay's PlanBuilder). In 3D
// (PlanPlacement::level) a wall is a box from z_base to z_base + height; in
// plan (PlanPlacement::above_scene, `rux render --view plan|top`) its
// footprint on a sheet over the cut cloud, exempt from the cut.

/// A wall that is assumed or seen over less than half its face: drawn dimmer.
bool wall_is_dim(const core::WallRecord &w) {
  return w.assumed || (w.seen_share >= 0.0 && w.seen_share < 0.5);
}

/// Half-width a wall of unknown thickness is drawn with [m].
constexpr double kUnknownWallHalfM = 0.05;

std::vector<vtkActor *> add_walls_layer(vtkRenderer *renderer,
                                        const ProjectDB &db,
                                        const SceneOptions &opts,
                                        SceneBounds &bounds,
                                        SceneLayer &drawn) {
  const auto walls = db.walls();
  if (walls.empty())
    throw std::runtime_error("render: project has no walls — run `" +
                             producing_command(Layer::walls) + "` first");
  core::SolidModel frames;
  frames.storeys = db.storeys();
  const bool sheet = opts.plan_placement == PlanPlacement::above_scene;
  PlanBuilder b(frames, opts.plan_placement, bounds, opts.plan_sheet_z);
  PlanGeometry fill, fill_dim, outline, outline_dim, axis;
  std::size_t skipped = 0;
  for (const auto &w : walls) {
    const Eigen::Vector2d p0(w.x0, w.y0), p1(w.x1, w.y1);
    const double len = (p1 - p0).norm();
    if (!(len > 0.0) || !std::isfinite(len)) {
      ++skipped;
      continue;
    }
    const bool dim = wall_is_dim(w);
    PlanGeometry &f = dim ? fill_dim : fill;
    PlanGeometry &o = dim ? outline_dim : outline;
    const Eigen::Vector2d d = (p1 - p0) / len;
    const Eigen::Vector2d n(-d.y(), d.x());
    const double h = w.thickness > 0.0 ? 0.5 * w.thickness : kUnknownWallHalfM;
    const std::array<Eigen::Vector2d, 4> c{p0 + n * h, p1 + n * h, p1 - n * h,
                                           p0 - n * h};
    const double z0 = w.z_base;
    const double z1 = w.z_base + std::max(w.height, 0.0);
    const auto at = [&](const Eigen::Vector2d &q, double z) {
      return b.world(w.storey, q.x(), q.y(), z);
    };
    if (sheet) {
      b.ring(o, &f, {at(c[0], z0), at(c[1], z0), at(c[2], z0), at(c[3], z0)});
      b.segment(axis, at(p0, z0), at(p1, z0));
    } else {
      std::array<Eigen::Vector3d, 8> v;
      for (std::size_t i = 0; i < 4; ++i) {
        v[i] = at(c[i], z0);
        v[i + 4] = at(c[i], z1);
      }
      // Outline: bottom and top rings, four vertical edges.
      b.ring(o, nullptr, {v[0], v[1], v[2], v[3]});
      b.ring(o, nullptr, {v[4], v[5], v[6], v[7]});
      for (std::size_t i = 0; i < 4; ++i)
        b.segment(o, v[i], v[i + 4]);
      // Faces: the four sides and the top (the bottom lies on the floor).
      const auto quad = [&](std::size_t a0, std::size_t a1, std::size_t a2,
                            std::size_t a3) {
        std::vector<vtkIdType> ids;
        for (const std::size_t k : {a0, a1, a2, a3})
          ids.push_back(b.add(f, v[k]));
        f.polys->InsertNextCell(4, ids.data());
      };
      quad(0, 1, 5, 4);
      quad(1, 2, 6, 5);
      quad(2, 3, 7, 6);
      quad(3, 0, 4, 7);
      quad(4, 5, 6, 7);
    }
    ++drawn.items;
    if (dim)
      ++drawn.dim;
  }
  if (skipped > 0)
    core::warn("render: {} of {} walls have a zero-length or non-finite axis; "
               "not drawn",
               skipped, walls.size());

  const auto &rgb = opts.walls_rgb;
  const std::array<std::uint8_t, 3> grey{0x80, 0x80, 0x80};
  const auto dim_rgb = lerp_rgb(rgb, grey, 0.5);
  std::vector<vtkActor *> actors;
  const auto add = [&](PlanGeometry &g, const std::array<std::uint8_t, 3> &c,
                       double width, double opacity) {
    if (g.empty())
      return;
    vtkActor *a = add_plan_actor(renderer, g, c, width, opacity);
    if (!sheet)
      a->SetObjectName(kWallsActorName); // cut with the scene in 3D
    actors.push_back(a);
  };
  // Translucent fills first, so the outlines draw over them.
  add(fill_dim, dim_rgb, 1.0, sheet ? 0.18 : 0.12);
  add(fill, rgb, 1.0, sheet ? 0.45 : 0.35);
  add(outline_dim, dim_rgb, 1.5, 0.7);
  add(outline, rgb, 2.5, 1.0);
  add(axis, rgb, 1.0, 1.0);
  b.extend(bounds);
  core::info("render: walls: {} drawn, {} dimmer (assumed or seen under 50 %)",
             drawn.items, drawn.dim);
  return actors;
}

// ── Spaces layer (reusex-z1v.6.3) ────────────────────────────────────────────
//
// The `spaces` table: each space's polygon at its floor height (z_floor of
// its storey's level frame, PlanPlacement::level) or on a sheet over the cut
// (PlanPlacement::above_scene), as a translucent fill in its room's label
// colour (the `rooms` layer's colour for room_label, so the two read
// together) with an outline, holes outlined, and its name and area at the
// centroid.

constexpr double kSpacesFillOpacity = 0.25;

std::vector<vtkActor *>
add_spaces_layer(vtkRenderer *renderer, const ProjectDB &db,
                 const SceneOptions &opts, const LabelPalette &palette,
                 SceneBounds &bounds, SceneLayer &drawn) {
  const auto spaces = db.spaces();
  if (spaces.empty())
    throw std::runtime_error("render: project has no spaces — run `" +
                             producing_command(Layer::spaces) + "` first");
  core::SolidModel frames;
  frames.storeys = db.storeys();
  const bool sheet = opts.plan_placement == PlanPlacement::above_scene;
  PlanBuilder b(frames, opts.plan_placement, bounds, opts.plan_sheet_z);
  // One fill and one outline geometry per colour.
  std::map<std::array<std::uint8_t, 3>, std::array<PlanGeometry, 2>> g;
  struct Label {
    std::string text;
    Eigen::Vector3d at;
    std::array<std::uint8_t, 3> rgb;
    Eigen::Vector2d dir;
  };
  std::vector<Label> labels;
  std::size_t untriangulated = 0;
  for (std::size_t i = 0; i < spaces.size(); ++i) {
    const auto &s = spaces[i];
    const std::uint32_t key = s.room_label > 0
                                  ? static_cast<std::uint32_t>(s.room_label)
                                  : static_cast<std::uint32_t>(i + 1);
    const auto rgb = label_palette_color(palette, key);
    auto &geo = g[rgb];
    const auto to_world = [&](const std::vector<core::PlanPoint> &ring) {
      std::vector<Eigen::Vector3d> out;
      for (const auto &p : ring)
        out.push_back(b.world(s.storey, p[0], p[1], s.z_floor));
      return out;
    };
    const auto outer = to_world(s.polygon.outer);
    b.ring(geo[1], nullptr, outer);
    for (const auto &hole : s.polygon.holes)
      b.ring(geo[1], nullptr, to_world(hole));
    // The fill: the outer ring ear-cut into triangles (a space is rarely
    // convex, and a polygon cell is drawn as a fan). Holes are outlined,
    // not cut out of the fill, as in the plan overlay.
    {
      vtkNew<vtkPolygon> polygon;
      polygon->GetPointIds()->SetNumberOfIds(
          static_cast<vtkIdType>(outer.size()));
      polygon->GetPoints()->SetNumberOfPoints(
          static_cast<vtkIdType>(outer.size()));
      for (std::size_t k = 0; k < outer.size(); ++k) {
        polygon->GetPointIds()->SetId(static_cast<vtkIdType>(k),
                                      static_cast<vtkIdType>(k));
        polygon->GetPoints()->SetPoint(static_cast<vtkIdType>(k),
                                       outer[k].data());
      }
      vtkNew<vtkIdList> tris;
      std::vector<vtkIdType> ids;
      for (const auto &p : outer)
        ids.push_back(b.add(geo[0], p));
      if (polygon->TriangulateLocalIds(0, tris) &&
          tris->GetNumberOfIds() >= 3) {
        for (vtkIdType t = 0; t + 2 < tris->GetNumberOfIds(); t += 3) {
          const vtkIdType tri[3] = {
              ids[static_cast<std::size_t>(tris->GetId(t))],
              ids[static_cast<std::size_t>(tris->GetId(t + 1))],
              ids[static_cast<std::size_t>(tris->GetId(t + 2))]};
          geo[0].polys->InsertNextCell(3, tri);
        }
      } else {
        ++untriangulated; // drawn as one polygon cell
        geo[0].polys->InsertNextCell(static_cast<vtkIdType>(ids.size()),
                                     ids.data());
      }
    }
    const auto c = core::space_polygon_centroid(s.polygon);
    // Upright in a view turned by SceneOptions::frame_yaw_deg (the
    // project's view yaw), as the drift layers' labels.
    const double yaw = -opts.frame_yaw_deg * std::numbers::pi / 180.0;
    labels.push_back(
        {fmt::format("{} ({:.1f} m2)",
                     s.name.empty() ? s.guid.substr(0, 8) : s.name, s.area),
         b.world(s.storey, c[0], c[1], s.z_floor), rgb,
         Eigen::Vector2d(std::cos(yaw), std::sin(yaw))});
    ++drawn.items;
  }

  std::vector<vtkActor *> actors;
  const auto add = [&](PlanGeometry &geo, const std::array<std::uint8_t, 3> &c,
                       double width, double opacity) {
    if (geo.empty())
      return;
    vtkActor *a = add_plan_actor(renderer, geo, c, width, opacity);
    if (!sheet)
      a->SetObjectName(kSpacesActorName); // cut with the scene in 3D
    actors.push_back(a);
  };
  for (auto &[rgb, geo] : g) // translucent fills first
    add(geo[0], rgb, 1.0, kSpacesFillOpacity);
  for (auto &[rgb, geo] : g)
    add(geo[1], rgb, 2.0, 1.0);
  if (opts.plan_labels)
    for (const auto &l : labels) {
      vtkActor *a = add_plan_label(renderer, l.text, l.at, l.rgb,
                                   kSpaceLabelM * 0.8, l.dir);
      if (!sheet)
        a->SetObjectName(kSpacesActorName);
      actors.push_back(a);
    }
  b.extend(bounds);
  if (untriangulated > 0)
    core::warn("render: spaces: {} of {} polygon(s) could not be ear-cut and "
               "are filled as one polygon (may draw wrongly if concave)",
               untriangulated, spaces.size());
  core::info("render: spaces: {} drawn", drawn.items);
  return actors;
}

// ── Solids layer (reusex-z1v.8.1) ────────────────────────────────────────────
//
// The meshes of the solids stage (solid_walls / solid_slabs / solid_spaces),
// in world coordinates, coloured by type; spaces translucent. A wall face
// that lies inside one of its opening voids (a jamb, sill or head: inside
// the void's box and not facing across the wall) is coloured by the
// opening's kind, read from the element index in the mesh's parameters.

/// The cells of @p mesh whose triangle is a reveal of one of its walls'
/// opening voids, with the void's kind (0 door, 1 window, 2 opening).
std::vector<int> solid_reveals(const pcl::PolygonMesh &mesh,
                               const std::string &parameters) {
  std::vector<int> kind(mesh.polygons.size(), -1);
  nlohmann::json meta;
  try {
    meta = nlohmann::json::parse(parameters);
  } catch (const std::exception &) {
    return kind;
  }
  if (!meta.is_object() || !meta.contains("elements"))
    return kind;
  pcl::PointCloud<pcl::PointXYZ> v;
  pcl::fromPCLPointCloud2(mesh.cloud, v);
  const auto P = [&](std::uint32_t i) {
    return Eigen::Vector3d(v[i].x, v[i].y, v[i].z);
  };
  for (const auto &e : meta.at("elements")) {
    if (!e.contains("voids") || e.at("voids").empty() || !e.contains("faces"))
      continue;
    const auto f0 = e.at("faces").at(0).get<std::size_t>();
    const auto fn = e.at("faces").at(1).get<std::size_t>();
    for (const auto &vd : e.at("voids")) {
      const auto &pr = vd.at("profile");
      const auto pl = pr.at("placement").get<std::array<double, 16>>();
      Eigen::Matrix4d m;
      for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
          m(r, c) = pl[static_cast<std::size_t>(4 * r + c)];
      const Eigen::Affine3d to_local(m.inverse());
      double xmax = 0.0, ymax = 0.0;
      for (const auto &q : pr.at("outer")) {
        xmax = std::max(xmax, q.at(0).get<double>());
        ymax = std::max(ymax, q.at(1).get<double>());
      }
      const double zmax = pr.at("depth").get<double>();
      const std::string k = vd.value("kind", "opening");
      const int code = k == "door" ? 0 : k == "window" ? 1 : 2;
      constexpr double eps = 2e-3;
      for (std::size_t f = f0; f < f0 + fn && f < mesh.polygons.size(); ++f) {
        const auto &t = mesh.polygons[f].vertices;
        if (t.size() != 3)
          continue;
        const Eigen::Vector3d a = to_local * P(t[0]), b = to_local * P(t[1]),
                              c = to_local * P(t[2]);
        const Eigen::Vector3d g = (a + b + c) / 3.0;
        const Eigen::Vector3d n = (b - a).cross(c - a);
        if (n.norm() <= 0.0 || std::abs(n.normalized().y()) > 0.5)
          continue; // a face of the wall, not a reveal
        if (g.x() > -eps && g.x() < xmax + eps && g.y() > -eps &&
            g.y() < ymax + eps && g.z() > -eps && g.z() < zmax + eps)
          kind[f] = code;
      }
    }
  }
  return kind;
}

void add_solids_layer(vtkRenderer *renderer, const ProjectDB &db,
                      const SceneOptions &opts, SceneBounds &bounds,
                      SceneLayer &drawn) {
  bool any = false;
  const std::array<std::array<std::uint8_t, 3>, 3> reveal_rgb{
      opts.plan_door_rgb, opts.plan_window_rgb, opts.plan_opening_rgb};
  for (std::size_t k = 0; k < kSolidMeshNames.size(); ++k) {
    const char *name = kSolidMeshNames[k];
    if (!db.has_mesh(name))
      continue;
    const auto mesh = db.mesh(name);
    if (!mesh || mesh->polygons.empty())
      continue;
    any = true;
    const auto meta = db.mesh_metadata(name);
    try {
      const auto j = nlohmann::json::parse(meta.parameters);
      drawn.items += j.at("elements").size();
    } catch (const std::exception &) {
      core::warn("render: solids: mesh '{}' has no element index in its "
                 "parameters; re-run `{}`",
                 name, producing_command(Layer::solids));
    }
    const auto &base = k == 0   ? opts.solid_wall_rgb
                       : k == 1 ? opts.solid_slab_rgb
                                : opts.solid_space_rgb;
    // Per-cell colours: the type's, or a reveal's kind.
    const std::vector<int> reveal =
        k == 0 ? solid_reveals(*mesh, meta.parameters) : std::vector<int>{};
    vtkNew<vtkUnsignedCharArray> colors;
    colors->SetName("solid_rgb");
    colors->SetNumberOfComponents(3);
    std::size_t reveals = 0;
    for (std::size_t f = 0; f < mesh->polygons.size(); ++f) {
      const int r = reveal.empty() ? -1 : reveal[f];
      const auto &c = r >= 0 ? reveal_rgb[static_cast<std::size_t>(r)] : base;
      reveals += r >= 0 ? 1 : 0;
      colors->InsertNextTypedTuple(c.data());
    }
    pcl::PointCloud<pcl::PointXYZ> vertices;
    pcl::fromPCLPointCloud2(mesh->cloud, vertices);
    vtkNew<vtkPoints> points;
    for (const auto &p : vertices) {
      points->InsertNextPoint(p.x, p.y, p.z);
      bounds.add(p.x, p.y, p.z);
    }
    vtkNew<vtkCellArray> polys;
    for (const auto &face : mesh->polygons) {
      std::vector<vtkIdType> ids(face.vertices.begin(), face.vertices.end());
      polys->InsertNextCell(static_cast<vtkIdType>(ids.size()), ids.data());
    }
    vtkNew<vtkPolyData> poly;
    poly->SetPoints(points);
    poly->SetPolys(polys);
    poly->GetCellData()->SetScalars(colors);

    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputData(poly);
    mapper->SetScalarModeToUseCellData();
    mapper->SetColorModeToDirectScalars();
    mapper->ScalarVisibilityOn();
    vtkNew<vtkActor> actor;
    actor->SetMapper(mapper);
    const bool space = k == 2;
    actor->GetProperty()->SetAmbient(0.3);
    actor->GetProperty()->SetDiffuse(0.7);
    actor->GetProperty()->SetSpecular(0.05);
    actor->GetProperty()->SetOpacity(space ? opts.solid_space_opacity : 1.0);
    actor->SetObjectName(kSolidsActorName);
    renderer->AddActor(actor);
    drawn.actors.push_back(actor);

    // Feature edges in a darker shade of the type's colour: the solid's
    // corners, and the outline of every void.
    vtkNew<vtkFeatureEdges> edges;
    edges->SetInputData(poly);
    edges->BoundaryEdgesOn();
    edges->FeatureEdgesOn();
    edges->NonManifoldEdgesOff();
    edges->ManifoldEdgesOff();
    edges->ColoringOff();
    edges->SetFeatureAngle(30.0);
    edges->Update();
    vtkNew<vtkPolyData> lines;
    lines->ShallowCopy(edges->GetOutput());
    lines->GetCellData()->SetScalars(nullptr);
    vtkNew<vtkPolyDataMapper> emapper;
    emapper->SetInputData(lines);
    emapper->ScalarVisibilityOff();
    vtkNew<vtkActor> eactor;
    eactor->SetMapper(emapper);
    const auto dark = lerp_rgb(base, {0, 0, 0}, 0.45);
    eactor->GetProperty()->SetColor(dark[0] / 255.0, dark[1] / 255.0,
                                    dark[2] / 255.0);
    eactor->GetProperty()->SetLineWidth(space ? 1.0 : 1.5);
    eactor->GetProperty()->SetOpacity(space ? 0.5 : 1.0);
    eactor->GetProperty()->SetLighting(false);
    eactor->SetObjectName(kSolidsActorName);
    renderer->AddActor(eactor);
    drawn.actors.push_back(eactor);

    drawn.faces += mesh->polygons.size();
    core::info("render: solids: '{}' {} faces{}", name, mesh->polygons.size(),
               k == 0 ? fmt::format(", {} opening reveal face(s)", reveals)
                      : std::string());
  }
  if (!any)
    throw std::runtime_error("render: project has no solids — run `" +
                             producing_command(Layer::solids) + "` first");
}

// ── Openings layer (reusex-z1v.7.3) ──────────────────────────────────────────
//
// The `openings` table drawn in its host walls' faces, through the host
// wall's storey frame (the plan overlay's PlanBuilder). In 3D
// (PlanPlacement::level) each opening is a rectangle s_start..s_end x
// z_sill..z_head on both faces of its wall (axis +- half the thickness, a
// centimetre proud), outline plus a translucent fill, so elevations and
// sections from either side show it; in plan (PlanPlacement::above_scene) a
// bar along the axis with a tick across the wall at each end, the plan
// overlay's symbol, on a sheet over the cut. Coloured by kind: door, window,
// opening (SceneOptions::plan_door_rgb, plan_window_rgb, plan_opening_rgb).

/// How far proud of the wall face a 3D opening rectangle is drawn [m].
constexpr double kOpeningProudM = 0.01;

std::vector<vtkActor *> add_openings_layer(vtkRenderer *renderer,
                                           const ProjectDB &db,
                                           const SceneOptions &opts,
                                           SceneBounds &bounds,
                                           SceneLayer &drawn) {
  const auto openings = db.openings();
  if (openings.empty())
    throw std::runtime_error("render: project has no openings — run `" +
                             producing_command(Layer::openings) + "` first");
  std::unordered_map<std::string, core::WallRecord> walls;
  for (auto &w : db.walls())
    walls.emplace(w.guid, std::move(w));
  core::SolidModel frames;
  frames.storeys = db.storeys();
  const bool sheet = opts.plan_placement == PlanPlacement::above_scene;
  PlanBuilder b(frames, opts.plan_placement, bounds, opts.plan_sheet_z);
  // [kind][0 = outline, 1 = fill]: door, window, opening.
  std::array<std::array<PlanGeometry, 2>, 3> g;
  std::size_t orphans = 0;
  for (const auto &o : openings) {
    const auto it = walls.find(o.wall);
    if (it == walls.end()) {
      ++orphans;
      continue;
    }
    const core::WallRecord &w = it->second;
    const Eigen::Vector2d p0(w.x0, w.y0), p1(w.x1, w.y1);
    const double len = (p1 - p0).norm();
    if (!(len > 0.0) || !std::isfinite(len)) {
      ++orphans;
      continue;
    }
    const Eigen::Vector2d d = (p1 - p0) / len;
    const Eigen::Vector2d n(-d.y(), d.x());
    const double half = w.thickness > 0.0 ? 0.5 * w.thickness : 0.0;
    const std::size_t k = o.kind == "door" ? 0 : o.kind == "window" ? 1 : 2;
    const Eigen::Vector2d a = p0 + d * o.s_start, e = p0 + d * o.s_end;
    const auto at = [&](const Eigen::Vector2d &q, double z) {
      return b.world(w.storey, q.x(), q.y(), z);
    };
    if (sheet) {
      const double tick = half + kOpeningTickM;
      b.segment(g[k][0], at(a, w.z_base), at(e, w.z_base));
      for (const Eigen::Vector2d &c : {a, e})
        b.segment(g[k][0], at(c + n * tick, w.z_base),
                  at(c - n * tick, w.z_base));
    } else {
      for (const double side : {1.0, -1.0}) {
        const Eigen::Vector2d off = n * (side * (half + kOpeningProudM));
        b.ring(g[k][0], &g[k][1],
               {at(a + off, o.z_sill), at(e + off, o.z_sill),
                at(e + off, o.z_head), at(a + off, o.z_head)});
      }
    }
    ++drawn.items;
  }
  if (orphans > 0)
    core::warn("render: {} of {} openings name a wall the walls table does not "
               "hold (walls ran again since `{}`); not drawn",
               orphans, openings.size(), producing_command(Layer::openings));

  const std::array<std::array<std::uint8_t, 3>, 3> rgb{
      opts.plan_door_rgb, opts.plan_window_rgb, opts.plan_opening_rgb};
  std::vector<vtkActor *> actors;
  const auto add = [&](PlanGeometry &geo, const std::array<std::uint8_t, 3> &c,
                       double width, double opacity) {
    if (geo.empty())
      return;
    vtkActor *a = add_plan_actor(renderer, geo, c, width, opacity);
    if (!sheet)
      a->SetObjectName(kOpeningsActorName); // cut with the scene in 3D
    actors.push_back(a);
  };
  for (std::size_t k = 0; k < 3; ++k)
    add(g[k][1], rgb[k], 1.0, 0.35); // translucent fills first
  for (std::size_t k = 0; k < 3; ++k)
    add(g[k][0], rgb[k], sheet ? 5.0 : 3.0, 1.0);
  b.extend(bounds);
  core::info("render: openings: {} drawn", drawn.items);
  return actors;
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

// ── Drift-loop layers (reusex-rcv.1.4) ───────────────────────────────────────

/// One vtkVectorText label lying flat at @p at (see add_plan_label()), named
/// @p name.
/// Turned by @p yaw_deg clockwise about @p at, so it reads upright in a view
/// turned counter-clockwise by that yaw (SceneOptions::frame_yaw_deg).
vtkActor *add_flat_label(vtkRenderer *renderer, const std::string &text,
                         const Eigen::Vector3d &at,
                         const std::array<std::uint8_t, 3> &rgb, double height,
                         const char *name, double yaw_deg) {
  vtkActor *a = add_plan_label(renderer, text, at, rgb, height);
  a->SetObjectName(name);
  if (yaw_deg != 0.0) {
    a->SetOrigin(at.x(), at.y(), at.z());
    a->RotateZ(-yaw_deg);
  }
  return a;
}

/// World pose of every sensor frame (index-aligned with sensor_frame_ids()),
/// from @p override when given; std::nullopt for an unposed frame.
std::vector<std::optional<Eigen::Matrix4d>>
frame_poses(const ProjectDB &db, const std::vector<int> &ids,
            const std::vector<Eigen::Matrix4d> &override_poses) {
  if (!override_poses.empty() && override_poses.size() != ids.size())
    throw std::runtime_error(
        "render: " + std::to_string(override_poses.size()) +
        " override poses for " + std::to_string(ids.size()) + " sensor frames");
  std::vector<std::optional<Eigen::Matrix4d>> out(ids.size());
  for (std::size_t k = 0; k < ids.size(); ++k) {
    if (!override_poses.empty())
      out[k] = override_poses[k];
    else if (db.has_sensor_frame_pose(ids[k]))
      out[k] = geometry::to_affine(db.sensor_frame_pose(ids[k])).matrix();
  }
  return out;
}

/// The `passes` layer: the preview cloud, each point in its segment's
/// colour.
vtkActor *add_passes_actor(vtkRenderer *renderer, const ProjectDB &db,
                           const SceneOptions &opts,
                           const LabelPalette &palette, SceneBounds &bounds,
                           SceneLayer &drawn, DriftLegendInfo &legend) {
  if (db.sensor_frame_ids().empty())
    throw std::runtime_error(
        "render: layer 'passes' needs posed sensor frames with depth, and "
        "this project has none — run `" +
        producing_command(Layer::passes) + "` first");
  const stopwatch timer;
  PreviewCloud computed;
  try {
    if (!opts.preview)
      computed = preview_cloud(db, opts.passes, opts.poses);
  } catch (const std::invalid_argument &e) {
    throw std::runtime_error(std::string("render: layer 'passes': ") +
                             e.what());
  } catch (const std::runtime_error &e) {
    throw std::runtime_error(std::string("render: layer 'passes': ") +
                             e.what() + " — run `" +
                             producing_command(Layer::passes) + "` first");
  }
  const PreviewCloud &pc = opts.preview ? *opts.preview : computed;
  if (static_cast<std::size_t>(opts.passes.segments) > palette.colors.size())
    core::warn("render: {} pass segments but {} pass colours; segments {} "
               "apart share a colour",
               opts.passes.segments, palette.colors.size(),
               palette.colors.size());
  // A budget thins by an even stride (the preview has no Morton order).
  const std::size_t n = pc.points.size();
  const std::size_t step = opts.max_points > 0 && n > opts.max_points
                               ? (n + opts.max_points - 1) / opts.max_points
                               : 1;
  vtkNew<vtkPoints> points;
  points->SetDataTypeToFloat();
  vtkNew<vtkUnsignedCharArray> rgb;
  rgb->SetNumberOfComponents(3);
  rgb->SetName("colors");
  vtkNew<vtkCellArray> verts;
  for (std::size_t i = 0; i < n; i += step) {
    const auto &p = pc.points[i];
    const vtkIdType id = points->InsertNextPoint(p.x(), p.y(), p.z());
    const auto c = pass_color(palette, pc.frame_segment[pc.frame[i]]);
    rgb->InsertNextTypedTuple(c.data());
    verts->InsertNextCell(1, &id);
    bounds.add(p.x(), p.y(), p.z());
  }
  vtkNew<vtkPolyData> poly;
  poly->SetPoints(points);
  poly->SetVerts(verts);
  poly->GetPointData()->SetScalars(rgb);
  drawn.points = static_cast<std::size_t>(points->GetNumberOfPoints());
  legend.passes_drawn = true;
  legend.pass_points = drawn.points;
  legend.passes.clear();
  for (const auto &s : pc.segments)
    legend.passes.push_back({s.index, s.first_node, s.last_node,
                             pass_color(palette, s.index), s.points});
  core::info("render: passes — {} points of {} frames in {} segments "
             "({:.2f}s)",
             drawn.points, pc.node_ids.size(), pc.segments.size(),
             timer.elapsed());
  vtkActor *actor = add_points_actor(renderer, poly, opts.point_size);
  actor->SetObjectName(kPassesActorName);
  return actor;
}

/// The nodes an edit pins: the first and last of each node set it names.
std::vector<std::int64_t> pinned_nodes(const core::PoseEdit &e) {
  std::vector<std::int64_t> out;
  for (const core::NodeRange *r : {&e.nodes, &e.nodes_b}) {
    if (r->is_list()) {
      out.push_back(r->list.front());
      if (r->list.back() != r->list.front())
        out.push_back(r->list.back());
    } else if (r->first >= 0) {
      out.push_back(r->first);
      if (r->last >= 0 && r->last != r->first)
        out.push_back(r->last);
    }
  }
  return out;
}

/// The `graph` layer: trajectory, edges by residual, node labels, edit pins.
void add_graph_actors(vtkRenderer *renderer, const ProjectDB &db,
                      const SceneOptions &opts, const LabelPalette &palette,
                      SceneBounds &bounds, SceneLayer &drawn,
                      DriftLegendInfo &legend) {
  const std::vector<int> ids = db.sensor_frame_ids();
  const auto poses = frame_poses(db, ids, opts.poses);
  // Segment of every frame: the one of its sampled frame (frame_step), the
  // passes layer's rule, so the path and the passes share colours.
  const std::size_t step =
      static_cast<std::size_t>(std::max(opts.passes.frame_step, 1));
  const std::size_t sampled = (ids.size() + step - 1) / step;
  std::unordered_map<int, Eigen::Vector3d> where;
  vtkNew<vtkPoints> path_points;
  path_points->SetDataType(VTK_DOUBLE);
  vtkNew<vtkUnsignedCharArray> path_rgb;
  path_rgb->SetNumberOfComponents(3);
  path_rgb->SetName("colors");
  std::vector<std::pair<int, Eigen::Vector3d>> posed;
  for (std::size_t k = 0; k < ids.size(); ++k) {
    if (!poses[k])
      continue;
    const Eigen::Vector3d c = poses[k]->topRightCorner<3, 1>();
    if (!c.allFinite())
      continue;
    where[ids[k]] = c;
    posed.emplace_back(ids[k], c);
    path_points->InsertNextPoint(c.x(), c.y(), c.z());
    const auto rgb = pass_color(
        palette, pass_segment(k / step, sampled, opts.passes.segments));
    path_rgb->InsertNextTypedTuple(rgb.data());
    bounds.add(c.x(), c.y(), c.z());
  }
  if (posed.empty())
    throw std::runtime_error(
        "render: layer 'graph' needs posed sensor frames, and this project "
        "has none — run `" +
        producing_command(Layer::graph) + "` first");
  const auto name = [](vtkActor *a) {
    a->SetObjectName(kGraphActorName);
    return a;
  };

  // The trajectory, coloured by segment.
  {
    vtkNew<vtkCellArray> lines;
    for (vtkIdType i = 0; i + 1 < path_points->GetNumberOfPoints(); ++i) {
      const vtkIdType seg[2] = {i, i + 1};
      lines->InsertNextCell(2, seg);
    }
    vtkNew<vtkPolyData> poly;
    poly->SetPoints(path_points);
    poly->SetLines(lines);
    poly->GetPointData()->SetScalars(path_rgb);
    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputData(poly);
    mapper->SetScalarModeToUsePointData();
    mapper->SetColorModeToDirectScalars();
    vtkNew<vtkActor> actor;
    actor->SetMapper(mapper);
    actor->GetProperty()->SetLineWidth(opts.graph.trajectory_width);
    actor->GetProperty()->SetLighting(false);
    renderer->AddActor(actor);
    drawn.actors.push_back(name(actor));
  }

  // The stored edges (or the caller's, a candidate solve's), coloured by
  // residual.
  std::vector<GraphLayerEdge> edges;
  if (opts.graph_edges) {
    edges = *opts.graph_edges;
  } else {
    for (const auto &e : db.list_pose_graph_edges())
      edges.push_back(
          {e.from_node_id, e.to_node_id, e.edge_type, e.status, e.residual});
  }
  {
    vtkNew<vtkPoints> pts;
    pts->SetDataType(VTK_DOUBLE);
    vtkNew<vtkCellArray> lines;
    vtkNew<vtkUnsignedCharArray> rgb;
    rgb->SetNumberOfComponents(3);
    rgb->SetName("colors");
    std::size_t missing = 0;
    for (const auto &e : edges) {
      if (e.edge_type == "odometry" && !opts.graph.odometry_edges)
        continue;
      if (e.status == "rejected" && !opts.graph.rejected_edges)
        continue;
      const auto a = where.find(static_cast<int>(e.from_node_id));
      const auto b = where.find(static_cast<int>(e.to_node_id));
      if (a == where.end() || b == where.end()) {
        ++missing;
        continue;
      }
      const vtkIdType seg[2] = {
          pts->InsertNextPoint(a->second.x(), a->second.y(), a->second.z()),
          pts->InsertNextPoint(b->second.x(), b->second.y(), b->second.z())};
      lines->InsertNextCell(2, seg);
      const auto c = residual_color(e.residual, opts.graph);
      rgb->InsertNextTypedTuple(c.data());
      ++legend.graph_edges;
      if (!std::isfinite(e.residual))
        ++legend.graph_edges_unsolved;
      else if (e.residual >= opts.graph.residual_bad)
        ++legend.graph_edges_bad;
    }
    if (missing > 0)
      core::warn("render: {} pose-graph edges name a frame without a pose; "
                 "they are not drawn",
                 missing);
    if (lines->GetNumberOfCells() > 0) {
      vtkNew<vtkPolyData> poly;
      poly->SetPoints(pts);
      poly->SetLines(lines);
      poly->GetCellData()->SetScalars(rgb);
      vtkNew<vtkPolyDataMapper> mapper;
      mapper->SetInputData(poly);
      mapper->SetScalarModeToUseCellData();
      mapper->SetColorModeToDirectScalars();
      vtkNew<vtkActor> actor;
      actor->SetMapper(mapper);
      actor->GetProperty()->SetLineWidth(opts.graph.edge_width);
      actor->GetProperty()->SetLighting(false);
      renderer->AddActor(actor);
      drawn.actors.push_back(name(actor));
    }
  }

  // Node-id labels along the path, a little to the side of it.
  SceneBounds path;
  for (const auto &[id, c] : posed)
    path.add(c.x(), c.y(), c.z());
  const double h =
      opts.graph.label_height_m > 0.0
          ? opts.graph.label_height_m
          : std::max(0.15, 0.012 * std::max(path.extent(0), path.extent(1)));
  const std::array<std::uint8_t, 3> ink{0xf0, 0xf0, 0xf0};
  if (opts.graph.label_every > 0)
    for (std::size_t i = 0; i < posed.size();
         i += static_cast<std::size_t>(opts.graph.label_every)) {
      const auto &[id, c] = posed[i];
      drawn.actors.push_back(add_flat_label(
          renderer, std::to_string(id), c + Eigen::Vector3d(0.0, h, 0.0), ink,
          h, kGraphActorName, opts.frame_yaw_deg));
    }

  // Pose-edit pins: a short post at each pinned node, the edit id beside.
  if (opts.edits) {
    vtkNew<vtkPoints> pts;
    pts->SetDataType(VTK_DOUBLE);
    vtkNew<vtkCellArray> lines;
    std::size_t unknown = 0;
    for (const auto &e : opts.edits->edits)
      for (const std::int64_t node : pinned_nodes(e)) {
        const auto it = where.find(static_cast<int>(node));
        if (it == where.end()) {
          ++unknown;
          continue;
        }
        const Eigen::Vector3d c = it->second;
        const vtkIdType seg[2] = {
            pts->InsertNextPoint(c.x(), c.y(), c.z()),
            pts->InsertNextPoint(c.x(), c.y(), c.z() + 4.0 * h)};
        lines->InsertNextCell(2, seg);
        // A cross at the foot reads as a pin from above.
        const vtkIdType xs[2] = {
            pts->InsertNextPoint(c.x() - h, c.y() - h, c.z()),
            pts->InsertNextPoint(c.x() + h, c.y() + h, c.z())};
        lines->InsertNextCell(2, xs);
        const vtkIdType ys[2] = {
            pts->InsertNextPoint(c.x() - h, c.y() + h, c.z()),
            pts->InsertNextPoint(c.x() + h, c.y() - h, c.z())};
        lines->InsertNextCell(2, ys);
        drawn.actors.push_back(add_flat_label(
            renderer, e.id, c + Eigen::Vector3d(1.5 * h, -h, 0.0),
            opts.graph.pin_rgb, h, kGraphActorName, opts.frame_yaw_deg));
        ++legend.graph_pins;
      }
    if (unknown > 0)
      core::warn("render: {} pose-edit pins name a node without a pose; "
                 "they are not drawn",
                 unknown);
    if (lines->GetNumberOfCells() > 0)
      drawn.actors.push_back(
          name(add_lines_actor(renderer, pts, lines, opts.graph.pin_rgb, 3.0)));
  }
  legend.graph_drawn = true;
  legend.graph_nodes = posed.size();
  legend.graph = opts.graph;
  drawn.items = posed.size();
}

/// World -> level frame of a heat storey.
Eigen::Affine3d heat_level(const DriftHeatStorey &s) {
  return geometry::to_affine(s.level_transform);
}

/// The `drift` layer: one actor of heat cells per storey shown.
void add_drift_actors(vtkRenderer *renderer, const SceneOptions &opts,
                      SceneBounds &bounds, SceneLayer &drawn,
                      DriftLegendInfo &legend) {
  if (!opts.drift_heatmap)
    throw std::runtime_error(
        "render: layer 'drift' needs a drift report — run `" +
        producing_command(Layer::drift) +
        "` and pass the file (rux render --drift-report)");
  if (!(opts.drift.good_m > 0.0) || !(opts.drift.bad_m > opts.drift.good_m))
    throw std::runtime_error(
        "render: drift colour scale needs 0 < good < bad (got " +
        std::to_string(opts.drift.good_m) + ", " +
        std::to_string(opts.drift.bad_m) + ")");
  const DriftHeatmap &hm = *opts.drift_heatmap;
  std::map<int, const DriftHeatStorey *> storeys;
  for (const auto &s : hm.storeys)
    storeys[s.idx] = &s;
  std::map<int, std::vector<const DriftHeatCell *>> groups;
  for (const auto &c : hm.cells)
    if (!opts.drift.storey || c.storey == *opts.drift.storey)
      groups[c.storey].push_back(&c);
  legend.drift_drawn = true;
  legend.drift = opts.drift;
  legend.noise_floor_m = hm.noise_floor_m;
  legend.drift_storey = opts.drift.storey;
  if (opts.drift.storey)
    if (const auto it = storeys.find(*opts.drift.storey); it != storeys.end())
      legend.drift_storey_name = it->second->name;
  if (groups.empty()) {
    core::warn("render: the drift report has {} cells{}; the drift layer "
               "draws nothing",
               hm.cells.size(),
               opts.drift.storey
                   ? fmt::format(", none of storey {}", *opts.drift.storey)
                   : std::string());
    return;
  }
  const double half = 0.5 * hm.cell_m;
  for (const auto &[storey, cells] : groups) {
    Eigen::Affine3d to_world = Eigen::Affine3d::Identity();
    std::optional<double> floor;
    if (storey >= 0) {
      const auto it = storeys.find(storey);
      if (it == storeys.end()) {
        core::warn("render: {} drift cells name storey {}, which the report "
                   "does not list; drawn in world coordinates",
                   cells.size(), storey);
      } else {
        to_world = heat_level(*it->second).inverse(Eigen::Isometry);
        floor = it->second->z_floor;
      }
    }
    vtkNew<vtkPoints> pts;
    pts->SetDataType(VTK_DOUBLE);
    vtkNew<vtkCellArray> quads;
    vtkNew<vtkUnsignedCharArray> rgb;
    rgb->SetNumberOfComponents(3);
    rgb->SetName("colors");
    for (const DriftHeatCell *c : cells) {
      const double z = (floor ? *floor : c->z_m) + opts.drift.lift_m;
      vtkIdType q[4];
      const double dx[4] = {-half, half, half, -half};
      const double dy[4] = {-half, -half, half, half};
      for (int k = 0; k < 4; ++k) {
        const Eigen::Vector3d w =
            to_world * Eigen::Vector3d(c->x + dx[k], c->y + dy[k], z);
        q[k] = pts->InsertNextPoint(w.x(), w.y(), w.z());
        bounds.add(w.x(), w.y(), w.z());
      }
      quads->InsertNextCell(4, q);
      const auto col = drift_color(c->residual_m, opts.drift);
      rgb->InsertNextTypedTuple(col.data());
      ++legend.drift_cells;
      legend.drift_max_m = std::max(legend.drift_max_m, c->residual_m);
    }
    vtkNew<vtkPolyData> poly;
    poly->SetPoints(pts);
    poly->SetPolys(quads);
    poly->GetCellData()->SetScalars(rgb);
    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputData(poly);
    mapper->SetScalarModeToUseCellData();
    mapper->SetColorModeToDirectScalars();
    vtkNew<vtkActor> actor;
    actor->SetMapper(mapper);
    actor->GetProperty()->SetOpacity(opts.drift.opacity);
    actor->GetProperty()->SetLighting(false);
    renderer->AddActor(actor);
    drawn.actors.push_back(actor);
    drawn.actor_storeys.push_back(storey);
  }
  drawn.items = legend.drift_cells;
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

// ── Deviation colours ────────────────────────────────────────────────────────

std::vector<std::pair<double, std::array<std::uint8_t, 3>>>
deviation_color_stops(const DeviationOptions &opts) {
  // tokens.css --label-2 (green), --label-3 (yellow), --label-5
  // (vermillion), --label-6 (reddish purple): Okabe-Ito, colourblind-safe.
  return {{0.0, {0x00, 0x9e, 0x73}},
          {opts.within_m, {0xf0, 0xe4, 0x42}},
          {opts.uncovered_m, {0xd5, 0x5e, 0x00}},
          {std::numeric_limits<double>::infinity(), {0xcc, 0x79, 0xa7}}};
}

std::array<std::uint8_t, 3> deviation_color(double d,
                                            const DeviationOptions &opts) {
  if (!std::isfinite(d))
    return default_label_palette().unlabeled;
  const auto stops = deviation_color_stops(opts);
  if (d > opts.uncovered_m)
    return stops[3].second;
  if (d <= 0.0)
    return stops[0].second;
  if (d <= opts.within_m)
    return lerp_rgb(stops[0].second, stops[1].second, d / opts.within_m);
  return lerp_rgb(stops[1].second, stops[2].second,
                  (d - opts.within_m) / (opts.uncovered_m - opts.within_m));
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
        "components, frustums, panoramas, deviation, passes, graph, drift, "
        "walls, openings, spaces, solids");
  if (!(opts.deviation.within_m > 0.0) ||
      !(opts.deviation.uncovered_m > opts.deviation.within_m))
    throw std::runtime_error(
        "render: deviation thresholds need 0 < within < uncovered (got " +
        std::to_string(opts.deviation.within_m) + ", " +
        std::to_string(opts.deviation.uncovered_m) + ")");
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

  // MeshStyle::automatic: see the cloud through the mesh when both are drawn.
  info.mesh_style = opts.mesh_style;
  if (info.mesh_style == MeshStyle::automatic) {
    const bool with_points =
        std::any_of(opts.layers.begin(), opts.layers.end(), [](Layer l) {
          return l != Layer::mesh && l != Layer::components &&
                 l != Layer::frustums && l != Layer::panoramas &&
                 l != Layer::graph && l != Layer::drift && l != Layer::walls &&
                 l != Layer::openings && l != Layer::spaces &&
                 l != Layer::solids;
        });
    info.mesh_style = with_points ? MeshStyle::overlay : MeshStyle::solid;
  }

  std::optional<std::size_t> walls_layer, openings_layer, spaces_layer;
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

    case Layer::deviation: {
      if (!db.has_mesh(opts.mesh_name)) {
        throw std::runtime_error("render: layer 'deviation' needs the mesh '" +
                                 opts.mesh_name +
                                 "', which this project does not have — run `" +
                                 producing_command(Layer::mesh) + "` first");
      }
      const Cloud &cloud = ensure_geometry();
      const auto mesh = db.mesh(opts.mesh_name);
      if (!mesh || mesh->polygons.empty())
        throw std::runtime_error("render: mesh '" + opts.mesh_name +
                                 "' has no faces — re-run `" +
                                 producing_command(Layer::mesh) + "`");
      const stopwatch timer;
      std::vector<double> d;
      try {
        d = geometry::point_to_mesh_distances(cloud, *mesh);
      } catch (const std::invalid_argument &e) {
        throw std::runtime_error(std::string("render: layer 'deviation': ") +
                                 e.what());
      }
      const auto &dv = opts.deviation;
      std::vector<unsigned char> colors(cloud.size() * 3);
      for (std::size_t i = 0; i < cloud.size(); ++i) {
        const auto c = deviation_color(d[i], dv);
        colors[3 * i + 0] = c[0];
        colors[3 * i + 1] = c[1];
        colors[3 * i + 2] = c[2];
      }
      info.deviation = to_stats(
          geometry::summarize_distances(d, dv.within_m, dv.uncovered_m));
      // The same summary over the points of a segmented plane: the clutter a
      // structural mesh is not meant to cover drops out.
      if (db.has_point_cloud("planes")) {
        const CloudLPtr planes = db.point_cloud_label("planes");
        if (planes && planes->size() == cloud.size()) {
          std::vector<double> planar(d.size(),
                                     std::numeric_limits<double>::quiet_NaN());
          for (std::size_t i = 0; i < d.size(); ++i)
            if ((*planes)[i].label != core::kUnlabeled &&
                !core::is_out_of_contract_label((*planes)[i].label))
              planar[i] = d[i];
          info.deviation_planar = to_stats(geometry::summarize_distances(
              planar, dv.within_m, dv.uncovered_m));
        } else {
          core::warn("render: 'planes' ({} labels) is out of sync with '{}' "
                     "({} points); no planar deviation summary",
                     planes ? planes->size() : 0, opts.cloud_name,
                     cloud.size());
        }
      }
      const auto &st = *info.deviation;
      core::info("render: deviation of '{}' from mesh '{}': {} points in "
                 "{:.2f}s, mean {:.3f} m, P95 {:.3f} m, {:.1f}% within "
                 "{:.2f} m, {:.1f}% beyond {:.2f} m (uncovered)",
                 opts.cloud_name, opts.mesh_name, st.points, timer.elapsed(),
                 st.mean_m, st.p95_m, 100.0 * st.within_share, dv.within_m,
                 100.0 * st.uncovered_share, dv.uncovered_m);
      if (st.points == 0)
        core::warn("render: no point of '{}' has a finite distance to mesh "
                   "'{}'; the deviation layer is uniformly grey",
                   opts.cloud_name, opts.mesh_name);
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
      add_mesh_layer(renderer, *mesh, info.mesh_style, opts, bounds, drawn);
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

    case Layer::passes:
      drawn.actors.push_back(add_passes_actor(
          renderer, db, opts, palette, bounds, drawn, info.drift_legend));
      break;

    case Layer::graph:
      add_graph_actors(renderer, db, opts, palette, bounds, drawn,
                       info.drift_legend);
      break;

    case Layer::drift:
      add_drift_actors(renderer, opts, bounds, drawn, info.drift_legend);
      break;

    case Layer::walls:
      // Drawn after every other layer: in plan its sheet sits above all of
      // them (PlanBuilder measures the bounds when it is made).
      walls_layer = info.layers.size();
      break;

    case Layer::openings:
      // After the walls, so in plan its sheet sits over theirs.
      openings_layer = info.layers.size();
      break;

    case Layer::spaces:
      // Before the walls, so in plan the walls draw over the fills.
      spaces_layer = info.layers.size();
      break;

    case Layer::solids:
      add_solids_layer(renderer, db, opts, bounds, drawn);
      break;
    }
    info.drawn_points += drawn.points;
    info.drawn_faces += drawn.faces;
    info.layers.push_back(std::move(drawn));
  }

  if (spaces_layer) {
    SceneLayer &drawn = info.layers[*spaces_layer];
    drawn.actors = add_spaces_layer(renderer, db, opts, palette, bounds, drawn);
  }
  if (walls_layer) {
    SceneLayer &drawn = info.layers[*walls_layer];
    drawn.actors = add_walls_layer(renderer, db, opts, bounds, drawn);
  }
  if (openings_layer) {
    SceneLayer &drawn = info.layers[*openings_layer];
    drawn.actors = add_openings_layer(renderer, db, opts, bounds, drawn);
  }

  if (opts.plan)
    info.plan = add_plan_overlay_to(renderer, *opts.plan, opts, bounds);

  if (!bounds.valid) {
    throw std::runtime_error(
        "render: the selected layers produced no drawable geometry");
  }
  info.frame_bounds = bounds;
  if (opts.frame_quantile > 0.0 || opts.frame_yaw_deg != 0.0) {
    // The box to frame, in the display frame (u, v, z) — world turned by the
    // yaw, so a building drawn square to the screen is framed tightly — over
    // the [q, 1 - q] quantiles of the cloud plus every mesh vertex.
    const double q = std::clamp(opts.frame_quantile, 0.0, 0.49);
    const double yaw = opts.frame_yaw_deg * std::numbers::pi / 180.0;
    const double c = std::cos(yaw), sn = std::sin(yaw);
    std::array<std::vector<double>, 3> axes;
    if (geometry)
      for (const auto &p : *geometry) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
          continue;
        axes[0].push_back(c * p.x - sn * p.y);
        axes[1].push_back(sn * p.x + c * p.y);
        axes[2].push_back(p.z);
      }
    SceneBounds db;
    if (!axes[0].empty()) {
      std::array<double, 3> lo{}, hi{};
      for (int k = 0; k < 3; ++k) {
        auto &v = axes[static_cast<std::size_t>(k)];
        const auto at = [&](double f) {
          const auto i = static_cast<std::size_t>(
              std::floor(f * static_cast<double>(v.size() - 1)));
          std::nth_element(v.begin(), v.begin() + static_cast<long>(i),
                           v.end());
          return v[i];
        };
        lo[static_cast<std::size_t>(k)] = at(q);
        hi[static_cast<std::size_t>(k)] = at(1.0 - q);
      }
      db.add(lo[0], lo[1], lo[2]);
      db.add(hi[0], hi[1], hi[2]);
    }
    for (const auto &layer : info.layers)
      if (layer.mesh_surface != nullptr)
        for (vtkIdType i = 0; i < layer.mesh_surface->GetNumberOfPoints();
             ++i) {
          double p[3];
          layer.mesh_surface->GetPoint(i, p);
          db.add(c * p[0] - sn * p[1], sn * p[0] + c * p[1], p[2]);
        }
    if (db.valid) {
      info.display_bounds = db;
      info.display_yaw_deg = opts.frame_yaw_deg;
      // Its world AABB, for what frames in world (orbit, sections).
      SceneBounds fb;
      for (const double u : {db.min[0], db.max[0]})
        for (const double v : {db.min[1], db.max[1]})
          for (const double z : {db.min[2], db.max[2]})
            fb.add(c * u + sn * v, -sn * u + c * v, z);
      info.frame_bounds = fb;
      core::debug("render: framing the [{:g}, {:g}] quantile box at yaw "
                  "{:g} deg: u [{:.2f}, {:.2f}] v [{:.2f}, {:.2f}] z [{:.2f}, "
                  "{:.2f}]",
                  q, 1.0 - q, opts.frame_yaw_deg, db.min[0], db.max[0],
                  db.min[1], db.max[1], db.min[2], db.max[2]);
    }
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

namespace {
/// The plan overlay and the mesh section lines are drawings laid over a cut
/// or a section, not part of it.
bool is_exempt_from_clipping(vtkActor *actor) {
  const std::string name = actor->GetObjectName();
  return name == kPlanOverlayActorName || name == kSectionLineActorName;
}
} // namespace

/// vtkPlane keeps the side its normal points to, so the normal points down.
/// Clipping planes live on the mapper and are evaluated in the vertex shader,
/// which is what makes this work uniformly for point sprites, mesh triangles
/// and component polylines alike.
void apply_cut_plane(vtkRenderer *renderer, double z) {
  vtkNew<vtkPlane> plane;
  plane->SetOrigin(0.0, 0.0, z);
  plane->SetNormal(0.0, 0.0, -1.0);

  // The graph keeps a slab around the cut instead (kGraphActorName).
  vtkNew<vtkPlane> graph_top;
  graph_top->SetOrigin(0.0, 0.0, z + kGraphCutAboveM);
  graph_top->SetNormal(0.0, 0.0, -1.0);
  vtkNew<vtkPlane> graph_bottom;
  graph_bottom->SetOrigin(0.0, 0.0, z - kGraphCutBelowM);
  graph_bottom->SetNormal(0.0, 0.0, 1.0);
  // ... and the passes a band below it (kPassesActorName).
  vtkNew<vtkPlane> passes_bottom;
  passes_bottom->SetOrigin(0.0, 0.0, z - kPassesCutDepthM);
  passes_bottom->SetNormal(0.0, 0.0, 1.0);

  vtkActorCollection *actors = renderer->GetActors();
  actors->InitTraversal();
  while (vtkActor *actor = actors->GetNextActor()) {
    if (is_exempt_from_clipping(actor))
      continue; // a drawing over the section, not part of it
    vtkMapper *mapper = actor->GetMapper();
    if (!mapper)
      continue;
    const std::string name = actor->GetObjectName();
    if (name == kGraphActorName) {
      mapper->AddClippingPlane(graph_top);
      mapper->AddClippingPlane(graph_bottom);
    } else {
      mapper->AddClippingPlane(plane);
      if (name == kPassesActorName)
        mapper->AddClippingPlane(passes_bottom);
    }
  }
}

std::optional<int> show_drift_storey_at(SceneInfo &info, double z,
                                        const SceneOptions &opts) {
  std::optional<int> shown;
  if (opts.drift_heatmap)
    for (const auto &s : opts.drift_heatmap->storeys) {
      const Eigen::Affine3d to_world =
          geometry::to_affine(s.level_transform).inverse(Eigen::Isometry);
      const double z0 = (to_world * Eigen::Vector3d(0, 0, s.z_floor)).z();
      const double z1 = (to_world * Eigen::Vector3d(0, 0, s.z_ceiling)).z();
      if (z >= std::min(z0, z1) && z <= std::max(z0, z1)) {
        shown = s.idx;
        info.drift_legend.drift_storey_name = s.name;
        break;
      }
    }
  for (auto &layer : info.layers) {
    if (layer.layer != Layer::drift)
      continue;
    for (std::size_t i = 0; i < layer.actors.size(); ++i) {
      const int st =
          i < layer.actor_storeys.size() ? layer.actor_storeys[i] : -1;
      layer.actors[i]->SetVisibility(!shown || st == *shown);
    }
  }
  info.drift_legend.drift_storey = shown;
  if (!shown)
    info.drift_legend.drift_storey_name.clear();
  // The legend counts what is visible now.
  if (opts.drift_heatmap) {
    auto &lg = info.drift_legend;
    lg.drift_cells = 0;
    lg.drift_max_m = 0.0;
    for (const auto &c : opts.drift_heatmap->cells)
      if ((!shown || c.storey == *shown) &&
          (!opts.drift.storey || c.storey == *opts.drift.storey)) {
        ++lg.drift_cells;
        lg.drift_max_m = std::max(lg.drift_max_m, c.residual_m);
      }
  }
  return shown;
}

void clear_cut_planes(vtkRenderer *renderer) {
  vtkActorCollection *actors = renderer->GetActors();
  actors->InitTraversal();
  while (vtkActor *actor = actors->GetNextActor()) {
    if (vtkMapper *mapper = actor->GetMapper())
      mapper->RemoveAllClippingPlanes();
  }
}

// ── Vertical sections (reusex-8vx) ───────────────────────────────────────────

ResolvedSection resolve_section(const SectionSpec &spec,
                                const SceneBounds &bounds) {
  if (!(spec.thickness > 0.0))
    throw std::invalid_argument(
        "render: section thickness must be positive (got " +
        std::to_string(spec.thickness) + ")");
  ResolvedSection r;
  r.thickness = spec.thickness;
  const double pad = spec.thickness;
  switch (spec.kind) {
  case SectionSpec::Kind::x:
    if (spec.at < bounds.min[0] || spec.at > bounds.max[0])
      throw std::invalid_argument(fmt::format(
          "render: section x={:.3f} lies outside the scene (x in [{:.3f}, "
          "{:.3f}])",
          spec.at, bounds.min[0], bounds.max[0]));
    r.p0 = {spec.at, bounds.min[1] - pad};
    r.p1 = {spec.at, bounds.max[1] + pad};
    break;
  case SectionSpec::Kind::y:
    if (spec.at < bounds.min[1] || spec.at > bounds.max[1])
      throw std::invalid_argument(fmt::format(
          "render: section y={:.3f} lies outside the scene (y in [{:.3f}, "
          "{:.3f}])",
          spec.at, bounds.min[1], bounds.max[1]));
    r.p0 = {bounds.min[0] - pad, spec.at};
    r.p1 = {bounds.max[0] + pad, spec.at};
    break;
  case SectionSpec::Kind::segment:
    r.p0 = spec.p0;
    r.p1 = spec.p1;
    break;
  }
  const double dx = r.p1[0] - r.p0[0], dy = r.p1[1] - r.p0[1];
  const double len = std::hypot(dx, dy);
  if (!(len > 1e-9))
    throw std::invalid_argument("render: a section segment needs two distinct "
                                "end points");
  r.dir = {dx / len, dy / len};
  r.view = {-r.dir[1], r.dir[0]};
  return r;
}

void apply_section(vtkRenderer *renderer, const ResolvedSection &section) {
  const double h = 0.5 * section.thickness;
  const auto &p0 = section.p0;
  const auto &p1 = section.p1;
  const auto &v = section.view;
  const auto &d = section.dir;
  // vtkPlane keeps the side its normal points to.
  const auto plane = [](double ox, double oy, double nx, double ny) {
    auto p = vtkSmartPointer<vtkPlane>::New();
    p->SetOrigin(ox, oy, 0.0);
    p->SetNormal(nx, ny, 0.0);
    return p;
  };
  const vtkSmartPointer<vtkPlane> planes[4] = {
      plane(p0[0] + v[0] * h, p0[1] + v[1] * h, -v[0], -v[1]),
      plane(p0[0] - v[0] * h, p0[1] - v[1] * h, v[0], v[1]),
      plane(p0[0], p0[1], d[0], d[1]), plane(p1[0], p1[1], -d[0], -d[1])};

  vtkActorCollection *actors = renderer->GetActors();
  actors->InitTraversal();
  while (vtkActor *actor = actors->GetNextActor()) {
    if (is_exempt_from_clipping(actor))
      continue;
    if (vtkMapper *mapper = actor->GetMapper())
      for (const auto &p : planes)
        mapper->AddClippingPlane(p);
  }
}

vtkActor *add_mesh_section_line(vtkRenderer *renderer, const SceneInfo &info,
                                const std::array<double, 3> &origin,
                                const std::array<double, 3> &normal,
                                const std::array<double, 3> &shift,
                                const std::array<std::uint8_t, 3> &rgb,
                                double width) {
  vtkPolyData *surface = nullptr;
  for (const auto &layer : info.layers)
    if (layer.layer == Layer::mesh && layer.mesh_surface != nullptr)
      surface = layer.mesh_surface;
  if (surface == nullptr)
    return nullptr;

  vtkNew<vtkPlane> plane;
  plane->SetOrigin(origin[0], origin[1], origin[2]);
  plane->SetNormal(normal[0], normal[1], normal[2]);
  vtkNew<vtkCutter> cutter;
  cutter->SetInputData(surface);
  cutter->SetCutFunction(plane);
  cutter->GenerateTrianglesOff();
  cutter->Update();
  vtkPolyData *cut = cutter->GetOutput();
  if (cut == nullptr || cut->GetNumberOfLines() == 0) {
    core::debug("render: the section plane misses the mesh; no section line");
    return nullptr;
  }

  vtkNew<vtkPoints> points;
  points->SetDataType(VTK_DOUBLE);
  points->SetNumberOfPoints(cut->GetNumberOfPoints());
  for (vtkIdType i = 0; i < cut->GetNumberOfPoints(); ++i) {
    double p[3];
    cut->GetPoint(i, p);
    points->SetPoint(i, p[0] + shift[0], p[1] + shift[1], p[2] + shift[2]);
  }
  vtkNew<vtkPolyData> poly;
  poly->SetPoints(points);
  poly->SetLines(cut->GetLines());

  vtkNew<vtkPolyDataMapper> mapper;
  mapper->SetInputData(poly);
  mapper->ScalarVisibilityOff();
  vtkNew<vtkActor> actor;
  actor->SetMapper(mapper);
  actor->SetObjectName(kSectionLineActorName);
  actor->GetProperty()->SetColor(rgb[0] / 255.0, rgb[1] / 255.0,
                                 rgb[2] / 255.0);
  actor->GetProperty()->SetLineWidth(width);
  actor->GetProperty()->SetLighting(false);
  renderer->AddActor(actor);
  return actor;
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
  std::array<double, 3> c = bounds.center();
  const double reach = std::max(bounds.diagonal(), 1.0);
  const double aspect = framing.aspect;
  const double yaw = framing.yaw_deg * std::numbers::pi / 180.0;
  const double cy = std::cos(yaw), sy = std::sin(yaw);
  // The camera turns by -yaw so the model appears turned by +yaw. Extents in
  // the turned frame decide the framing: exact when the caller measured the
  // geometry in that frame (display_bounds), else those of the turned AABB.
  double ex = std::abs(cy) * bounds.extent(0) + std::abs(sy) * bounds.extent(1);
  double ey = std::abs(sy) * bounds.extent(0) + std::abs(cy) * bounds.extent(1);
  if (framing.display_bounds && framing.display_bounds->valid) {
    const SceneBounds &d = *framing.display_bounds;
    ex = d.extent(0);
    ey = d.extent(1);
    const auto m = d.center();
    c = {cy * m[0] + sy * m[1], -sy * m[0] + cy * m[1], m[2]};
  }
  double center[3] = {c[0], c[1], c[2]};

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
  case ViewPreset::back:
  case ViewPreset::left:
  case ViewPreset::right: {
    // Orthographic elevations, world Z up, each turned by the yaw: front
    // looks along +Y (from the south), back along -Y, left along +X (from
    // the west), right along -X. The base direction rotated by -yaw is
    // (sy, cy) for front, as before.
    double bx = 0.0, by = 1.0;
    if (framing.view == ViewPreset::back)
      bx = 0.0, by = -1.0;
    else if (framing.view == ViewPreset::left)
      bx = 1.0, by = 0.0;
    else if (framing.view == ViewPreset::right)
      bx = -1.0, by = 0.0;
    const double vx = cy * bx + sy * by;
    const double vy = -sy * bx + cy * by;
    // Screen right is display u for front/back and display v for
    // left/right.
    const bool sideways =
        framing.view == ViewPreset::left || framing.view == ViewPreset::right;
    const double across = sideways ? ey : ex;
    cam->ParallelProjectionOn();
    cam->SetPosition(center[0] - reach * vx, center[1] - reach * vy, center[2]);
    cam->SetViewUp(0.0, 0.0, 1.0);
    cam->SetParallelScale(
        parallel_scale_for(across, bounds.extent(2), aspect, framing.margin));
    renderer->ResetCameraClippingRange();
    return;
  }

  case ViewPreset::section: {
    if (!framing.section)
      throw std::invalid_argument(
          "place_preset_camera: the section view needs a section");
    const ResolvedSection &sec = *framing.section;
    // Frame the part of the line inside the scene, at the scene's height.
    const auto along = [&](double x, double y) {
      return (x - sec.p0[0]) * sec.dir[0] + (y - sec.p0[1]) * sec.dir[1];
    };
    double lo = std::numeric_limits<double>::infinity();
    double hi = -lo;
    if (framing.display_bounds && framing.display_bounds->valid) {
      // The display box's corners, in world: tight for a section square to
      // the display frame.
      const SceneBounds &d = *framing.display_bounds;
      for (const double u : {d.min[0], d.max[0]})
        for (const double v : {d.min[1], d.max[1]}) {
          const double a = along(cy * u + sy * v, -sy * u + cy * v);
          lo = std::min(lo, a);
          hi = std::max(hi, a);
        }
    } else {
      for (const double x : {bounds.min[0], bounds.max[0]})
        for (const double y : {bounds.min[1], bounds.max[1]}) {
          lo = std::min(lo, along(x, y));
          hi = std::max(hi, along(x, y));
        }
    }
    const double len = std::hypot(sec.p1[0] - sec.p0[0], sec.p1[1] - sec.p0[1]);
    lo = std::clamp(lo, 0.0, len);
    hi = std::clamp(hi, 0.0, len);
    if (hi <= lo)
      lo = 0.0, hi = len;
    const double mid = 0.5 * (lo + hi);
    const double fx = sec.p0[0] + sec.dir[0] * mid;
    const double fy = sec.p0[1] + sec.dir[1] * mid;
    cam->ParallelProjectionOn();
    cam->SetFocalPoint(fx, fy, center[2]);
    cam->SetPosition(fx - reach * sec.view[0], fy - reach * sec.view[1],
                     center[2]);
    cam->SetViewUp(0.0, 0.0, 1.0);
    cam->SetParallelScale(
        parallel_scale_for(hi - lo, bounds.extent(2), aspect, framing.margin));
    renderer->ResetCameraClippingRange();
    return;
  }

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

// ── Plan overlay, standalone (reusex-yf4.2) ─────────────────────────────────

PlanOverlayInfo add_plan_overlay(vtkRenderer *renderer,
                                 const core::SolidModel &plan,
                                 const SceneOptions &opts,
                                 const SceneBounds &scene_bounds) {
  SceneBounds bounds = scene_bounds; // the overlay must not move the scene's
  return add_plan_overlay_to(renderer, plan, opts, bounds);
}

void remove_plan_overlay(vtkRenderer *renderer, PlanOverlayInfo &overlay) {
  for (auto *list : {&overlay.actors, &overlay.labels})
    for (vtkActor *actor : *list)
      if (renderer != nullptr && actor != nullptr)
        renderer->RemoveActor(actor);
  overlay = PlanOverlayInfo{};
}

} // namespace reusex::visualize
