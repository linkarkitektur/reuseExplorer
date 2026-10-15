// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "database/cloud_router.hpp"
#include "database/plane_class_router.hpp"

#include <reusex/core/label_encoding.hpp>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <spdlog/spdlog.h>

namespace rux::database {

std::vector<std::string> CloudRouter::list() const {
  auto names = db_->list_point_clouds();
  std::sort(names.begin(), names.end()); // Alphabetical order (deterministic)
  return names;
}

nlohmann::json CloudRouter::get_metadata(std::string_view name) const {
  if (!db_->has_point_cloud(name)) {
    throw std::runtime_error("Point cloud not found: " + std::string(name));
  }

  nlohmann::json meta;
  meta["name"] = name;
  meta["type"] = db_->point_cloud_type(name);

  // Get point count based on type
  std::string type = meta["type"];
  size_t point_count = 0;

  if (type == "PointXYZRGB") {
    auto cloud = db_->point_cloud_xyzrgb(name);
    point_count = cloud ? cloud->size() : 0;
  } else if (type == "Normal") {
    auto cloud = db_->point_cloud_normal(name);
    point_count = cloud ? cloud->size() : 0;
  } else if (type == "Label") {
    auto cloud = db_->point_cloud_label(name);
    point_count = cloud ? cloud->size() : 0;
  } else if (type == "PointXYZ") {
    auto cloud = db_->point_cloud_xyz(name);
    point_count = cloud ? cloud->size() : 0;
  }

  meta["point_count"] = point_count;
  return meta;
}

std::vector<uint8_t>
CloudRouter::get_cloud_binary(std::string_view name) const {
  if (!db_->has_point_cloud(name)) {
    throw std::runtime_error("Point cloud not found: " + std::string(name));
  }

  std::string type = db_->point_cloud_type(name);

  // Use temp file for PCL I/O (PCL doesn't support stream-based I/O)
  auto temp_file = std::filesystem::temp_directory_path() /
                   ("rux_cloud_" + std::string(name) + ".pcd");
  bool success = false;

  if (type == "PointXYZRGB") {
    auto cloud = db_->point_cloud_xyzrgb(name);
    if (cloud) {
      success = (pcl::io::savePCDFileBinary(temp_file.string(), *cloud) == 0);
    }
  } else if (type == "Normal") {
    auto cloud = db_->point_cloud_normal(name);
    if (cloud) {
      success = (pcl::io::savePCDFileBinary(temp_file.string(), *cloud) == 0);
    }
  } else if (type == "Label") {
    auto cloud = db_->point_cloud_label(name);
    if (cloud) {
      success = (pcl::io::savePCDFileBinary(temp_file.string(), *cloud) == 0);
    }
  } else if (type == "PointXYZ") {
    auto cloud = db_->point_cloud_xyz(name);
    if (cloud) {
      success = (pcl::io::savePCDFileBinary(temp_file.string(), *cloud) == 0);
    }
  } else {
    throw std::runtime_error("Unknown cloud type: " + type);
  }

  if (!success) {
    std::filesystem::remove(temp_file);
    throw std::runtime_error("Failed to serialize point cloud: " +
                             std::string(name));
  }

  // Read temp file into buffer
  std::ifstream file(temp_file, std::ios::binary);
  if (!file) {
    std::filesystem::remove(temp_file);
    throw std::runtime_error("Failed to read temp file");
  }

  std::vector<uint8_t> buffer((std::istreambuf_iterator<char>(file)),
                              std::istreambuf_iterator<char>());
  file.close();
  std::filesystem::remove(temp_file);

  return buffer;
}

DataPayload CloudRouter::get(const std::vector<PathComponent> &components) {
  if (components.empty()) {
    // Collection level: return list of cloud names
    auto names = list();
    return nlohmann::json(names);
  }

  // Resolve index if present
  std::string item_name;

  if (components[0].is_index()) {
    // Array indexing: clouds[0]
    auto resolved = resolve_index(*components[0].index);
    if (!resolved) {
      throw std::runtime_error("Array index out of range: " +
                               std::to_string(*components[0].index));
    }
    item_name = *resolved;
  } else if (components[0].is_item()) {
    // Direct item access: clouds.mycloud
    item_name = components[0].value;
  } else {
    throw std::runtime_error("Expected item or index after collection");
  }

  // Check if wildcard (should have been expanded before calling this)
  if (item_name.find('*') != std::string::npos) {
    throw std::runtime_error("Wildcard not expanded: " + item_name);
  }

  if (components.size() == 1) {
    // clouds.mycloud → return binary cloud data
    return get_cloud_binary(item_name);
  }

  // Property access: clouds.mycloud.property
  const auto &prop = components[1].value;

  if (prop == "metadata") {
    return get_metadata(item_name);
  } else if (prop == "type") {
    return db_->point_cloud_type(item_name);
  } else if (prop == "point_count") {
    auto meta = get_metadata(item_name);
    return std::to_string(meta["point_count"].get<size_t>());
  } else if (prop == "classes") {
    // The plane_classes rows of a plane-label cloud (`rux create classify`).
    return plane_classes_json(*db_, item_name);
  } else {
    throw std::runtime_error(
        "Unknown property: " + prop +
        "\nAvailable properties: metadata, type, point_count, classes");
  }
}

void CloudRouter::set_cloud_from_binary(std::string_view name,
                                        const std::vector<uint8_t> &data) {
  // Write data to temp file (PCL doesn't support stream-based I/O)
  auto temp_file = std::filesystem::temp_directory_path() /
                   ("rux_cloud_import_" + std::string(name) + ".pcd");

  std::ofstream file(temp_file, std::ios::binary);
  if (!file) {
    throw std::runtime_error("Failed to create temp file");
  }
  file.write(reinterpret_cast<const char *>(data.data()), data.size());
  file.close();

  // Load generically to inspect field names before typed conversion
  pcl::PCLPointCloud2 cloud2;
  if (pcl::io::loadPCDFile(temp_file.string(), cloud2) != 0) {
    std::filesystem::remove(temp_file);
    throw std::runtime_error("Failed to parse PCD data for cloud: " +
                             std::string(name));
  }
  std::filesystem::remove(temp_file);

  auto has_field = [&](std::string_view field_name) {
    return std::any_of(cloud2.fields.begin(), cloud2.fields.end(),
                       [&](const auto &f) { return f.name == field_name; });
  };

  if (has_field("label")) {
    pcl::PointCloud<pcl::Label>::Ptr cloud_l(new pcl::PointCloud<pcl::Label>);
    pcl::fromPCLPointCloud2(cloud2, *cloud_l);
    // The semantic cloud is stored as point label = API class + 1; a set of
    // the default semantic cloud is taken to be in that encoding and stamped
    // so readers and `rux validate` do not flag it (STANDARDS §3.1). Other
    // label clouds keep whatever encoding their author gave them.
    if (reusex::core::is_semantic_label_cloud(name, ""))
      db_->save_point_cloud(name, *cloud_l, /*stage=*/"",
                            reusex::core::stamp_label_encoding(""));
    else
      db_->save_point_cloud(name, *cloud_l);
    spdlog::info("Saved label cloud '{}' ({} points)", name, cloud_l->size());
  } else if (has_field("normal_x")) {
    pcl::PointCloud<pcl::Normal>::Ptr cloud_n(new pcl::PointCloud<pcl::Normal>);
    pcl::fromPCLPointCloud2(cloud2, *cloud_n);
    db_->save_point_cloud(name, *cloud_n);
    spdlog::info("Saved normal cloud '{}' ({} points)", name, cloud_n->size());
  } else if (has_field("rgb") || has_field("rgba")) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(cloud2, *cloud);
    db_->save_point_cloud(name, *cloud);
    spdlog::info("Saved point cloud '{}' ({} points)", name, cloud->size());
  } else {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(cloud2, *cloud_xyz);
    db_->save_point_cloud(name, *cloud_xyz);
    spdlog::info("Saved XYZ cloud '{}' ({} points)", name, cloud_xyz->size());
  }
}

void CloudRouter::set_cloud_from_text(std::string_view name,
                                      std::string_view data) {
  // Convert text to binary and reuse binary handler
  std::vector<uint8_t> binary(data.begin(), data.end());
  set_cloud_from_binary(name, binary);
}

void CloudRouter::set(const std::vector<PathComponent> &components,
                      const DataPayload &data) {
  if (components.empty()) {
    throw std::runtime_error(
        "Cannot set collection directly. Specify a cloud name: clouds.<name>");
  }

  // Get item name (no index support for set)
  if (!components[0].is_item()) {
    throw std::runtime_error("Expected item name after collection");
  }

  std::string item_name = components[0].value;

  if (components.size() == 1) {
    // clouds.mycloud → set cloud data
    if (std::holds_alternative<std::vector<uint8_t>>(data)) {
      set_cloud_from_binary(item_name, std::get<std::vector<uint8_t>>(data));
    } else if (std::holds_alternative<std::string>(data)) {
      set_cloud_from_text(item_name, std::get<std::string>(data));
    } else {
      throw std::runtime_error(
          "Invalid data type for cloud. Expected binary PCD data.");
    }
  } else {
    throw std::runtime_error(
        "Cannot set individual cloud properties. Set the entire cloud.");
  }
}

void CloudRouter::del(const std::vector<PathComponent> &components) {
  if (components.empty()) {
    throw std::runtime_error(
        "Cannot delete entire collection. Specify a cloud name: clouds.<name>");
  }

  if (!components[0].is_item()) {
    throw std::runtime_error("Expected item name after collection");
  }

  std::string item_name = components[0].value;

  if (!db_->has_point_cloud(item_name)) {
    throw std::runtime_error("Point cloud not found: " + item_name);
  }

  db_->delete_point_cloud(item_name);
  spdlog::info("Deleted point cloud '{}'", item_name);
}

} // namespace rux::database
