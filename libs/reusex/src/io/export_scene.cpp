// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "io/export_scene.hpp"
#include "core/logging.hpp"

#include <reusex/core/MaterialPassport.hpp>
#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/SensorIntrinsics.hpp>
#include <reusex/core/label_encoding.hpp>
#include <reusex/geometry/BuildingComponent.hpp>
#include <reusex/geometry/cgal_utils.hpp>
#include <reusex/geometry/component_persistence.hpp>
#include <reusex/geometry/unweld.hpp>

#include <pcl/common/colors.h>

#include <fmt/format.h>

#include <cstddef>
#include <cstdlib>
#include <limits>
#include <regex>

namespace reusex::io {

namespace {

/// Parse instance label_definitions format: "SM{semantic_class}-{instance_id}
/// ({size}p)"
/// Returns the semantic class ID, or -1 on failure.
int parse_instance_semantic_class(const std::string &def) {
  // Match "SM<digits>-<digits> (<digits>p)"
  static const std::regex re(R"(SM(\d+)-\d+\s+\(\d+p\))");
  std::smatch match;
  if (std::regex_match(def, match, re)) {
    auto val = std::stoul(match[1].str());
    if (val > static_cast<unsigned long>(std::numeric_limits<int>::max()))
      return -1;
    return static_cast<int>(val);
  }
  return -1;
}

/// Extract translation from a row-major 4x4 SE(3) matrix.
void extract_position(const std::array<double, 16> &pose, double &x, double &y,
                      double &z) {
  x = pose[3];
  y = pose[7];
  z = pose[11];
}

/// Multiply two row-major 4x4 matrices: result = a * b.
std::array<double, 16> matmul4x4(const std::array<double, 16> &a,
                                 const std::array<double, 16> &b) {
  std::array<double, 16> r{};
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j) {
      double s = 0.0;
      for (int k = 0; k < 4; ++k)
        s += a[i * 4 + k] * b[k * 4 + j];
      r[i * 4 + j] = s;
    }
  return r;
}

} // namespace

ExportScene gather_export_scene(const ProjectDB &db) {
  ExportScene scene;

  // --- Cloud layer ---
  if (db.has_point_cloud("cloud")) {
    reusex::debug("ExportScene: loading 'cloud'");
    ExportScene::CloudLayer layer;
    layer.cloud = db.point_cloud_xyzrgb("cloud");

    if (db.has_point_cloud("normals"))
      layer.normals = db.point_cloud_normal("normals");

    scene.cloud = std::move(layer);
    reusex::debug("ExportScene: cloud has {} points",
                  scene.cloud->cloud->size());
  }

  // --- Semantic layer ---
  if (db.has_point_cloud("instances") && db.has_point_cloud("cloud")) {
    // Instance-based export
    reusex::debug("ExportScene: loading instance-based semantic data");

    auto instance_labels = db.point_cloud_label("instances");
    auto instance_defs = db.label_definitions("instances");

    // Get semantic name mapping from "labels" definitions
    std::map<int, std::string> semantic_names;
    if (db.has_point_cloud("labels")) {
      core::warn_if_unstamped_semantic_cloud(db, "labels");
      semantic_names = db.label_definitions("labels");
    }

    auto cloud = db.point_cloud_xyzrgb("cloud");

    // Group instance IDs by semantic class
    // instance_id -> semantic_class_id
    std::map<uint32_t, int> instance_to_semantic;
    for (const auto &[inst_id, def] : instance_defs) {
      int sem_class = parse_instance_semantic_class(def);
      if (sem_class >= 0)
        instance_to_semantic[static_cast<uint32_t>(inst_id)] = sem_class;
    }

    // semantic_class_id -> list of instance_ids
    std::map<int, std::vector<uint32_t>> semantic_to_instances;
    for (const auto &[inst_id, sem_class] : instance_to_semantic)
      semantic_to_instances[sem_class].push_back(inst_id);

    // Build per-instance point clouds
    // instance_id -> CloudPtr
    std::map<uint32_t, CloudPtr> instance_clouds;
    for (size_t i = 0; i < instance_labels->size(); ++i) {
      uint32_t inst = instance_labels->points[i].label;
      if (inst == 0)
        continue; // skip unlabeled
      auto [it, inserted] = instance_clouds.try_emplace(inst);
      if (inserted)
        it->second = CloudPtr(new Cloud());
      it->second->points.push_back(cloud->points[i]);
    }

    // Build SemanticCategory entries
    for (const auto &[sem_class, inst_ids] : semantic_to_instances) {
      ExportScene::SemanticCategory cat;
      cat.label_id = sem_class;

      // Look up name from "labels" label_definitions
      auto it = semantic_names.find(sem_class);
      cat.name = (it != semantic_names.end())
                     ? it->second
                     : "class_" + std::to_string(sem_class);

      // Glasbey color
      auto c = pcl::GlasbeyLUT::at(sem_class % pcl::GlasbeyLUT::size());
      cat.color = {c.r, c.g, c.b};

      // Add instances
      for (uint32_t inst_id : inst_ids) {
        auto cloud_it = instance_clouds.find(inst_id);
        if (cloud_it == instance_clouds.end() || cloud_it->second->empty())
          continue;

        ExportScene::SemanticInstance si;
        si.instance_id = inst_id;
        si.points = cloud_it->second;
        cat.instances.push_back(std::move(si));
      }

      if (!cat.instances.empty())
        scene.semantic.push_back(std::move(cat));
    }

    reusex::debug("ExportScene: {} semantic categories (instance-based)",
                  scene.semantic.size());

  } else if (db.has_point_cloud("labels") && db.has_point_cloud("cloud")) {
    // Label-based export (no instances)
    reusex::debug("ExportScene: loading label-based semantic data");

    auto labels = db.point_cloud_label("labels");
    core::warn_if_unstamped_semantic_cloud(db, "labels");
    auto label_defs = db.label_definitions("labels");
    auto cloud = db.point_cloud_xyzrgb("cloud");

    // Group points by label
    std::map<uint32_t, CloudPtr> label_clouds;
    for (size_t i = 0; i < labels->size(); ++i) {
      uint32_t lbl = labels->points[i].label;
      if (lbl == 0)
        continue; // skip unlabeled/background
      auto [it, inserted] = label_clouds.try_emplace(lbl);
      if (inserted)
        it->second = CloudPtr(new Cloud());
      it->second->points.push_back(cloud->points[i]);
    }

    for (const auto &[lbl, lbl_cloud] : label_clouds) {
      ExportScene::SemanticCategory cat;
      cat.label_id = static_cast<int>(lbl);

      auto it = label_defs.find(static_cast<int>(lbl));
      cat.name = (it != label_defs.end()) ? it->second
                                          : "class_" + std::to_string(lbl);

      auto c = pcl::GlasbeyLUT::at(lbl % pcl::GlasbeyLUT::size());
      cat.color = {c.r, c.g, c.b};

      ExportScene::SemanticInstance si;
      si.instance_id = 0;
      si.points = lbl_cloud;
      cat.instances.push_back(std::move(si));

      scene.semantic.push_back(std::move(cat));
    }

    reusex::debug("ExportScene: {} semantic categories (label-based)",
                  scene.semantic.size());
  }

  // --- Mesh layer ---
  for (const auto &name : db.list_meshes()) {
    reusex::debug("ExportScene: loading mesh '{}'", name);
    auto mesh = db.mesh(name);
    if (!mesh || mesh->polygons.empty())
      continue;

    auto components = geometry::cgal::decompose_mesh(*mesh);
    if (components.size() == 1) {
      auto unwelded = geometry::unweld_mesh(*components[0], 0.7854f);
      scene.meshes.push_back({name, unwelded});
    } else {
      for (size_t i = 0; i < components.size(); ++i) {
        auto unwelded = geometry::unweld_mesh(*components[i], 0.7854f);
        scene.meshes.push_back({fmt::format("{}_{}", name, i), unwelded});
      }
    }
    reusex::debug("ExportScene: mesh '{}' -> {} components", name,
                  components.size());
  }

  // --- 360 Panoramas ---
  std::size_t panos_without_pose = 0;
  for (const auto &pano : db.list_panoramic_images()) {
    ExportScene::PanoEntry entry;
    entry.image_name = pano.filename;

    // Prefer the content-aligned pose (`rux align 360`) when present; fall
    // back to the timestamp-matched sensor frame's pose.
    //
    // The seed frame's pose has to be checked, not just its existence: an
    // unposed frame would fall through `sensor_frame_pose()`'s identity
    // fallback and drop the panorama marker at the world origin, which reads
    // in an exported scene as a real placement (#336). Leaving x/y/z at their
    // defaults is the same "unplaced" state a panorama with no linked frame
    // already gets.
    if (pano.has_pose) {
      extract_position(pano.pose, entry.x, entry.y, entry.z);
    } else if (pano.node_id >= 0 && db.has_sensor_frame(pano.node_id) &&
               db.has_sensor_frame_pose(pano.node_id)) {
      auto pose = db.sensor_frame_pose(pano.node_id);
      extract_position(pose, entry.x, entry.y, entry.z);
    } else {
      ++panos_without_pose;
    }

    scene.panoramas.push_back(std::move(entry));
  }

  if (!scene.panoramas.empty())
    reusex::debug("ExportScene: {} panoramic images", scene.panoramas.size());
  if (panos_without_pose > 0)
    reusex::warn("ExportScene: {} of {} panoramas have no usable pose (no "
                 "aligned pose and no posed seed frame); exported unplaced",
                 panos_without_pose, scene.panoramas.size());

  // --- Materials ---
  std::size_t materials_without_pose = 0;
  for (const auto &passport : db.all_material_passports()) {
    ExportScene::MaterialEntry entry;
    entry.name = passport.description.designation.empty()
                     ? passport.metadata.document_guid
                     : passport.description.designation;

    // Try to get stored properties
    try {
      entry.properties =
          db.passport_stored_properties(passport.metadata.document_guid);
    } catch (...) {
      // No stored properties - that's OK
    }

    // Pull the original filename out of properties (set by "rux import
    // photos") and onto a dedicated field so exporters don't have to peek
    // into the property bag.
    if (auto it = entry.properties.find("fileName");
        it != entry.properties.end()) {
      entry.image_filename = it->second;
      entry.properties.erase(it);
    }

    // Position + orientation: photo imports link the passport to the
    // nearest sensor frame via material_passports.id == node_id, so look
    // that up and pull the frame's world pose. The pose stored on the
    // frame is the sensor BODY pose (RTABMap convention); compose it with
    // the camera→base local_transform to get the optical frame's world
    // pose so the camera-frustum marker points along the actual viewing
    // direction.
    try {
      auto node_id =
          db.passport_linked_node_id(passport.metadata.document_guid);
      if (node_id && db.has_sensor_frame(*node_id)) {
        // A linked frame with no usable stored pose yields the identity
        // fallback, i.e. a camera frustum at the world origin pointing down
        // +Z — indistinguishable from a real photo position (#336). Keep the
        // entry's default identity transform instead, which is the same state
        // an unlinked passport gets, and count it.
        if (!db.has_sensor_frame_pose(*node_id)) {
          reusex::debug("ExportScene: material '{}' links node {}, which has "
                        "no usable stored pose; exported unplaced",
                        entry.name, *node_id);
          ++materials_without_pose;
        } else {
          auto pose = db.sensor_frame_pose(*node_id);
          try {
            auto intr = db.sensor_frame_intrinsics(*node_id);
            entry.transform = matmul4x4(pose, intr.local_transform);
          } catch (...) {
            // No intrinsics stored — fall back to the body pose.
            entry.transform = pose;
          }
          extract_position(entry.transform, entry.x, entry.y, entry.z);
        }
      }
    } catch (...) {
      // No linked frame -> keep identity transform / origin.
    }

    scene.materials.push_back(std::move(entry));
  }

  if (!scene.materials.empty())
    reusex::debug("ExportScene: {} material passports", scene.materials.size());
  if (materials_without_pose > 0)
    reusex::warn("ExportScene: {} of {} material passports link a sensor frame "
                 "with no usable stored pose; exported unplaced",
                 materials_without_pose, scene.materials.size());

  // --- Building Components ---
  for (const auto &name : db.list_building_components()) {
    auto comp = geometry::building_component(db, name);
    ExportScene::ComponentEntry entry;
    entry.name = comp.name;
    entry.type = comp.type;
    entry.boundary = comp.boundary;
    entry.confidence = comp.confidence;
    entry.notes = comp.notes;

    entry.properties["type"] = std::string(geometry::to_string(comp.type));
    if (comp.confidence >= 0)
      entry.properties["confidence"] = std::to_string(comp.confidence);
    if (!comp.notes.empty())
      entry.properties["notes"] = comp.notes;

    if (auto *w = std::get_if<geometry::WindowData>(&comp.data)) {
      if (!w->style.empty())
        entry.properties["style"] = w->style;
      if (w->pane_count > 0)
        entry.properties["pane_count"] = std::to_string(w->pane_count);
      entry.properties["operable"] = w->operable ? "true" : "false";
    } else if (auto *d = std::get_if<geometry::DoorData>(&comp.data)) {
      if (!d->style.empty())
        entry.properties["style"] = d->style;
      if (!d->swing.empty())
        entry.properties["swing"] = d->swing;
    }

    scene.components.push_back(std::move(entry));
  }

  if (!scene.components.empty())
    reusex::debug("ExportScene: {} building components",
                  scene.components.size());

  return scene;
}

} // namespace reusex::io
