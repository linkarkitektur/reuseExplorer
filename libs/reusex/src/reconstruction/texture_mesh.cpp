// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
#include "reconstruction/texture_mesh.hpp"
#include "core/logging.hpp"
#include "core/processing_observer.hpp"
#include "geometry/utils.hpp"
#include "types.hpp"

#include <Eigen/Dense>
#include <fmt/format.h>
#include <pcl/common/io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/surface/texture_mapping.h>
#include <range/v3/view/enumerate.hpp>

#include <atomic>
#include <cmath>
#include <filesystem>
#include <random>
#include <stdexcept>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace reusex::geometry {

std::filesystem::path
prepare_texture_dir(const std::filesystem::path &requested) {
  namespace fs = std::filesystem;

  // A directory the caller named: make it absolute (so the paths we write
  // into pcl::TexMaterial::tex_file — and from there into the MTL — never
  // depend on the process working directory) and create it if missing.
  // Its existing content is deliberately left alone: the library must never
  // delete a path it did not create itself (issue #245).
  if (!requested.empty()) {
    fs::path dir = fs::absolute(requested);
    std::error_code ec;
    fs::create_directories(dir, ec);
    if (ec)
      throw std::runtime_error(
          fmt::format("Failed to create texture directory '{}': {}",
                      dir.string(), ec.message()));
    if (!fs::is_directory(dir))
      throw std::runtime_error(
          fmt::format("Texture directory '{}' exists but is not a directory",
                      dir.string()));
    reusex::core::debug("Staging textures in caller-supplied directory: {}",
                        dir.string());
    return dir;
  }

  // Nothing requested: stage into a fresh, uniquely named directory under the
  // system temp directory. create_directory() returns true only when it
  // actually created the directory, so a name that is already taken is simply
  // retried — two concurrent runs can never share a staging directory.
  static std::atomic<unsigned long long> counter{0};
  std::random_device rd;
  const fs::path base = fs::temp_directory_path();

  constexpr int kMaxAttempts = 64;
  for (int attempt = 0; attempt < kMaxAttempts; ++attempt) {
    fs::path candidate =
        base / fmt::format("reusex-textures-{:08x}{:08x}-{}", rd(), rd(),
                           counter.fetch_add(1, std::memory_order_relaxed));
    std::error_code ec;
    if (fs::create_directory(candidate, ec)) {
      reusex::core::debug("Staging textures in temporary directory: {}",
                          candidate.string());
      return candidate;
    }
    if (ec)
      throw std::runtime_error(
          fmt::format("Failed to create texture staging directory '{}': {}",
                      candidate.string(), ec.message()));
    // No error and not created => the name already existed; try another.
  }

  throw std::runtime_error(fmt::format(
      "Failed to create a unique texture staging directory under '{}' "
      "after {} attempts",
      base.string(), kMaxAttempts));
}

/**
 * @brief Verify polygon-to-atlas-tile mapping is correct (debug mode)
 *
 * For each polygon, check that its UV coordinates map to the expected atlas
 * tile. Reports any polygons that sample wrong tiles (UV mapping bugs).
 *
 * @param textured_mesh Mesh with merged atlas texture
 * @param grid_size Grid dimensions (e.g., 8 for 8x8 grid)
 * @param polygon_to_material Mapping of polygon index to original material/tile
 * ID
 *
 * The tile pixel size is not needed here: the check works entirely in
 * normalized UV space, where a tile is 1/grid_size wide regardless of its
 * resolution.
 */
static void
verify_atlas_mapping(const pcl::TextureMesh::Ptr &textured_mesh, int grid_size,
                     const std::vector<size_t> &polygon_to_material) {
  if (textured_mesh->tex_polygons.size() != 1) {
    reusex::core::warn("Atlas verification requires merged structure");
    return;
  }

  const auto &polygons = textured_mesh->tex_polygons[0];
  const auto &uv_coords = textured_mesh->tex_coordinates[0];
  const auto &uv_indices = textured_mesh->tex_coord_indices[0];

  if (polygons.size() != uv_indices.size()) {
    reusex::core::error("Polygon count ({}) != UV index count ({})",
                        polygons.size(), uv_indices.size());
    return;
  }

  reusex::core::debug("Verifying atlas mapping for {} polygons...",
                      polygons.size());

  std::map<int, int> tile_usage_count; // tile_id -> polygon count
  std::map<int, std::vector<size_t>>
      tile_to_polygons; // tile_id -> list of polygon indices
  int out_of_bounds_count = 0;
  int mismatched_count = 0;

  for (size_t poly_idx = 0; poly_idx < polygons.size(); ++poly_idx) {
    const auto &uv_idx = uv_indices[poly_idx];

    if (uv_idx.vertices.empty()) {
      reusex::core::warn("Polygon {} has no UV indices", poly_idx);
      continue;
    }

    // Get expected tile based on original material assignment
    const size_t expected_tile = poly_idx < polygon_to_material.size()
                                     ? polygon_to_material[poly_idx]
                                     : 9999; // Invalid marker

    // Get first UV coordinate for this polygon
    const uint32_t first_uv_idx = uv_idx.vertices[0];
    if (first_uv_idx >= uv_coords.size()) {
      reusex::core::error("Polygon {} has invalid UV index {} (max {})",
                          poly_idx, first_uv_idx, uv_coords.size() - 1);
      continue;
    }

    const auto &uv = uv_coords[first_uv_idx];

    // Check if UV is out of bounds
    if (uv[0] < 0.0f || uv[0] > 1.0f || uv[1] < 0.0f || uv[1] > 1.0f) {
      if (out_of_bounds_count < 5) { // Only log first few to avoid spam
        reusex::core::warn("Polygon {} has out-of-bounds UV ({:.3f}, {:.3f})",
                           poly_idx, uv[0], uv[1]);
      }
      out_of_bounds_count++;
      continue;
    }

    // Determine which atlas tile this UV points to
    // Clamp to valid range to avoid edge cases
    const int atlas_grid_x =
        std::min(static_cast<int>(uv[0] * grid_size), grid_size - 1);
    const int atlas_grid_y =
        std::min(static_cast<int>(uv[1] * grid_size), grid_size - 1);
    const int actual_tile = atlas_grid_y * grid_size + atlas_grid_x;

    tile_usage_count[actual_tile]++;
    tile_to_polygons[actual_tile].push_back(poly_idx);

    // Check if actual tile matches expected tile
    if (static_cast<size_t>(actual_tile) != expected_tile) {
      if (mismatched_count < 10) { // Log first 10 mismatches
        reusex::core::warn("MISMATCH! Polygon {} expected tile {} but uses "
                           "tile {} (UV index {} -> UV {:.4f}, {:.4f})",
                           poly_idx, expected_tile, actual_tile, first_uv_idx,
                           uv[0], uv[1]);
      }
      mismatched_count++;
    } else {
      // Log first few correct mappings for verification
      if (poly_idx < 10) {
        reusex::core::trace("Polygon {} correctly uses tile {} (UV index {} -> "
                            "UV {:.4f}, {:.4f})",
                            poly_idx, actual_tile, first_uv_idx, uv[0], uv[1]);
      }
    }
  }

  if (mismatched_count > 0) {
    reusex::core::error(
        "CRITICAL BUG: {} polygons use WRONG tiles (expected != actual)",
        mismatched_count);
  }

  if (out_of_bounds_count > 0) {
    reusex::core::warn("Total out-of-bounds UVs: {}", out_of_bounds_count);
  }

  // Report tile usage statistics
  reusex::core::info("Atlas tile usage:");
  for (const auto &[tile_id, count] : tile_usage_count) {
    const int grid_x = tile_id % grid_size;
    const int grid_y = tile_id / grid_size;
    reusex::core::info("  Tile {} (grid position {},{}) -> {} polygons",
                       tile_id, grid_x, grid_y, count);
  }

  // Check for unused tiles (would appear as black in atlas or indicate wrong
  // mapping)
  const int total_tiles = grid_size * grid_size;
  std::vector<int> unused_tiles;
  for (int tile_id = 0; tile_id < total_tiles; ++tile_id) {
    if (tile_usage_count.find(tile_id) == tile_usage_count.end()) {
      unused_tiles.push_back(tile_id);
    }
  }

  if (!unused_tiles.empty()) {
    // Format tile list manually since fmt::join may not be available
    std::string tile_list;
    for (size_t i = 0; i < unused_tiles.size(); ++i) {
      if (i > 0)
        tile_list += ", ";
      tile_list += std::to_string(unused_tiles[i]);
    }
    reusex::core::warn("{} unused tiles (will appear black on mesh): {}",
                       unused_tiles.size(), tile_list);
  }

  // Check for tiles used by multiple polygons (potential repetition bug)
  std::vector<int> multi_use_tiles;
  for (const auto &[tile_id, count] : tile_usage_count) {
    if (count >
        10) { // Arbitrary threshold - normal is 1-2 polygons per material
      multi_use_tiles.push_back(tile_id);
    }
  }

  if (!multi_use_tiles.empty()) {
    reusex::core::warn("Tiles with unexpectedly high polygon counts (possible "
                       "UV mapping bug):");
    for (int tile_id : multi_use_tiles) {
      reusex::core::warn("  Tile {} -> {} polygons", tile_id,
                         tile_usage_count[tile_id]);
    }
  }
}

/**
 * @brief Compute texture atlas layout and merge individual textures
 *
 * Takes a TextureMesh with multiple materials/textures and combines them into
 * a single texture atlas. Updates UV coordinates to reference the atlas.
 *
 * @param textured_mesh Mesh with multiple textures to be atlased
 * @param tile_size Size of each individual texture tile (e.g., 1024)
 * @param output_path Path where the atlas texture will be saved
 * @param debug_distinct_colors Enable debug verification of atlas mapping
 * @return Number of textures that were atlased
 */
static size_t create_texture_atlas(pcl::TextureMesh::Ptr textured_mesh,
                                   int tile_size,
                                   const std::filesystem::path &output_path,
                                   bool debug_distinct_colors = false) {
  const size_t num_materials = textured_mesh->tex_materials.size();

  if (num_materials == 0) {
    reusex::core::warn("No materials to atlas");
    return 0;
  }

  if (num_materials == 1) {
    reusex::core::debug("Only one material, no atlasing needed");
    return 1;
  }

  // Calculate atlas dimensions: arrange tiles in a grid
  // Use ceiling of square root to get grid dimensions (e.g., 364 tiles = 20x20
  // grid)
  const int grid_size = static_cast<int>(
      std::ceil(std::sqrt(static_cast<double>(num_materials))));
  const int atlas_width = grid_size * tile_size;
  const int atlas_height = grid_size * tile_size;

  reusex::core::info(
      "Creating texture atlas: {} textures -> {}x{} atlas ({} grid)",
      num_materials, atlas_width, atlas_height, grid_size);

  // DEBUG: Check UV coordinates BEFORE remapping
  if (debug_distinct_colors) {
    reusex::core::debug("Checking UV coordinates BEFORE atlas remapping:");
    for (size_t mat_idx = 0; mat_idx < std::min(size_t(5), num_materials);
         ++mat_idx) {
      if (mat_idx < textured_mesh->tex_coordinates.size() &&
          !textured_mesh->tex_coordinates[mat_idx].empty()) {
        const auto &first_uv = textured_mesh->tex_coordinates[mat_idx][0];
        reusex::core::debug("  Material {}: first UV = ({:.4f}, {:.4f})",
                            mat_idx, first_uv[0], first_uv[1]);
      }
    }
  }

  // Create atlas image (initially black)
  cv::Mat atlas(atlas_height, atlas_width, CV_8UC3, cv::Scalar(0, 0, 0));

  // Copy each texture into its grid position and update UV coordinates
  for (size_t mat_idx = 0; mat_idx < num_materials; ++mat_idx) {
    const auto &material = textured_mesh->tex_materials[mat_idx];

    // Calculate grid position for this material
    const int grid_x = static_cast<int>(mat_idx % grid_size);
    const int grid_y = static_cast<int>(mat_idx / grid_size);
    const int atlas_x = grid_x * tile_size;
    const int atlas_y = grid_y * tile_size;

    // Load individual texture (or use black if file doesn't exist/empty)
    cv::Mat tile;
    if (std::filesystem::exists(material.tex_file)) {
      tile = cv::imread(material.tex_file, cv::IMREAD_COLOR);
    }

    if (tile.empty()) {
      // Create black tile if texture doesn't exist
      tile = cv::Mat(tile_size, tile_size, CV_8UC3, cv::Scalar(0, 0, 0));
    } else if (tile.cols != tile_size || tile.rows != tile_size) {
      // Resize if needed
      cv::resize(tile, tile, cv::Size(tile_size, tile_size));
    }

    // Copy tile into atlas
    tile.copyTo(atlas(cv::Rect(atlas_x, atlas_y, tile_size, tile_size)));

    // Update UV coordinates for this material's polygons
    // UV coords are in [0,1] range for the tile, need to remap to atlas region
    if (mat_idx < textured_mesh->tex_coordinates.size()) {
      if (debug_distinct_colors && mat_idx < 5) {
        const auto &first_uv_before =
            textured_mesh->tex_coordinates[mat_idx][0];
        reusex::core::trace("  Material {} BEFORE remap: UV ({:.4f}, {:.4f})",
                            mat_idx, first_uv_before[0], first_uv_before[1]);
      }

      for (auto &uv : textured_mesh->tex_coordinates[mat_idx]) {
        // Transform from tile space [0,1] to atlas space
        // Eigen::Vector2f uses [0] for u, [1] for v
        uv[0] = (uv[0] * tile_size + atlas_x) / atlas_width;
        uv[1] = (uv[1] * tile_size + atlas_y) / atlas_height;
      }

      if (debug_distinct_colors && mat_idx < 5) {
        const auto &first_uv_after = textured_mesh->tex_coordinates[mat_idx][0];
        reusex::core::trace(
            "  Material {} AFTER remap to tile ({},{}): UV ({:.4f}, {:.4f})",
            mat_idx, grid_x, grid_y, first_uv_after[0], first_uv_after[1]);
      }
    }
  }

  // Write atlas to disk
  if (!cv::imwrite(output_path.string(), atlas)) {
    throw std::runtime_error("Failed to write texture atlas: " +
                             output_path.string());
  }

  reusex::core::info("Texture atlas saved: {} ({}x{} px, {:.1f} MB)",
                     output_path.filename().string(), atlas_width, atlas_height,
                     (atlas_width * atlas_height * 3) / 1024.0 / 1024.0);

  // Replace all materials with single atlas material
  pcl::TexMaterial atlas_material;
  atlas_material.tex_file = output_path.string();
  atlas_material.tex_name = "texture_atlas";
  atlas_material.tex_Ka.r = 0.2f;
  atlas_material.tex_Ka.g = 0.2f;
  atlas_material.tex_Ka.b = 0.2f;
  atlas_material.tex_Kd.r = 0.8f;
  atlas_material.tex_Kd.g = 0.8f;
  atlas_material.tex_Kd.b = 0.8f;
  atlas_material.tex_Ks.r = 1.0f;
  atlas_material.tex_Ks.g = 1.0f;
  atlas_material.tex_Ks.b = 1.0f;
  atlas_material.tex_d = 1.0f;
  atlas_material.tex_Ns = 75.0f;
  atlas_material.tex_illum = 2;

  textured_mesh->tex_materials.clear();
  textured_mesh->tex_materials.push_back(atlas_material);

  // ===== PRE-MERGE VALIDATION =====
  // Verify structure consistency before merging to detect misalignment early
  reusex::core::debug("Pre-merge validation: {} materials", num_materials);

  if (textured_mesh->tex_polygons.size() != num_materials) {
    throw std::runtime_error(fmt::format(
        "Material count mismatch: {} materials but {} polygon groups",
        num_materials, textured_mesh->tex_polygons.size()));
  }

  if (textured_mesh->tex_coordinates.size() != num_materials) {
    throw std::runtime_error(fmt::format(
        "Material count mismatch: {} materials but {} UV coordinate groups",
        num_materials, textured_mesh->tex_coordinates.size()));
  }

  if (textured_mesh->tex_coord_indices.size() != num_materials) {
    throw std::runtime_error(fmt::format(
        "Material count mismatch: {} materials but {} UV index groups",
        num_materials, textured_mesh->tex_coord_indices.size()));
  }

  // Validate polygon counts match UV index counts for each material
  size_t total_polys_pre_merge = 0;
  size_t materials_with_no_polygons = 0;

  for (size_t mat_idx = 0; mat_idx < num_materials; ++mat_idx) {
    const size_t num_polys = textured_mesh->tex_polygons[mat_idx].size();
    const size_t num_uv_indices =
        textured_mesh->tex_coord_indices[mat_idx].size();
    const size_t num_uvs = textured_mesh->tex_coordinates[mat_idx].size();

    if (num_polys != num_uv_indices) {
      throw std::runtime_error(
          fmt::format("Material {} has {} polygons but {} UV index sets",
                      mat_idx, num_polys, num_uv_indices));
    }

    if (num_polys == 0) {
      materials_with_no_polygons++;
      reusex::core::warn(
          "Material {} (tile {}) has NO polygons - will create unused tile",
          mat_idx, mat_idx);
    }

    total_polys_pre_merge += num_polys;

    reusex::core::trace(
        "Material {} (tile {}): {} polygons, {} UVs, {} UV indices", mat_idx,
        mat_idx, num_polys, num_uvs, num_uv_indices);
  }

  if (materials_with_no_polygons > 0) {
    reusex::core::warn(
        "{} materials have no polygons (will create {} unused tiles)",
        materials_with_no_polygons, materials_with_no_polygons);
  }

  reusex::core::debug("Total polygons before merge: {}", total_polys_pre_merge);

  // Merge all polygon groups into single group (all use same material now)
  std::vector<pcl::Vertices> merged_polygons;
  std::vector<size_t> polygon_to_original_material; // Track which material each
                                                    // polygon came from
  size_t total_polygons = 0;

  for (size_t mat_idx = 0; mat_idx < textured_mesh->tex_polygons.size();
       ++mat_idx) {
    const auto &poly_group = textured_mesh->tex_polygons[mat_idx];
    merged_polygons.insert(merged_polygons.end(), poly_group.begin(),
                           poly_group.end());

    // Track original material for each polygon
    for (size_t i = 0; i < poly_group.size(); ++i) {
      polygon_to_original_material.push_back(mat_idx);
    }

    total_polygons += poly_group.size();
  }

  reusex::core::debug("Merged {} polygons from {} groups",
                      merged_polygons.size(),
                      textured_mesh->tex_polygons.size());

  textured_mesh->tex_polygons.clear();
  textured_mesh->tex_polygons.push_back(merged_polygons);

  // Merge all UV coordinates into single group with proper index offsetting
  std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>
      merged_uvs;
  std::vector<pcl::Vertices> merged_uv_indices;

  uint32_t uv_offset = 0; // Track current offset into merged UV array

  for (size_t mat_idx = 0; mat_idx < textured_mesh->tex_coordinates.size();
       ++mat_idx) {
    const auto &uv_group = textured_mesh->tex_coordinates[mat_idx];
    const auto &idx_group = textured_mesh->tex_coord_indices[mat_idx];

    // Copy UV coordinates
    merged_uvs.insert(merged_uvs.end(), uv_group.begin(), uv_group.end());

    // Copy UV indices with offset adjustment
    for (const auto &uv_idx : idx_group) {
      pcl::Vertices offset_idx;
      offset_idx.vertices.reserve(uv_idx.vertices.size());

      for (const auto &idx : uv_idx.vertices) {
        offset_idx.vertices.push_back(idx + uv_offset);
      }

      merged_uv_indices.push_back(offset_idx);
    }

    // Update offset for next material
    uv_offset += static_cast<uint32_t>(uv_group.size());
  }

  reusex::core::debug("Atlas merging: {} UV coords total, offset range 0-{}",
                      merged_uvs.size(), uv_offset);
  reusex::core::debug("Merged {} UV indices (expected {})",
                      merged_uv_indices.size(), total_polygons);

  // Validate polygon/UV index correspondence before finalizing
  if (merged_uv_indices.size() != total_polygons) {
    throw std::runtime_error(
        fmt::format("Polygon/UV index count mismatch after merge: {} polygons, "
                    "{} UV indices",
                    total_polygons, merged_uv_indices.size()));
  }

  textured_mesh->tex_coordinates.clear();
  textured_mesh->tex_coordinates.push_back(merged_uvs);

  textured_mesh->tex_coord_indices.clear();
  textured_mesh->tex_coord_indices.push_back(merged_uv_indices);

  // ===== POST-MERGE VALIDATION =====
  reusex::core::debug("Post-merge validation:");
  reusex::core::debug("  Materials: {}", textured_mesh->tex_materials.size());
  reusex::core::debug("  Polygon groups: {}",
                      textured_mesh->tex_polygons.size());
  reusex::core::debug("  UV coord groups: {}",
                      textured_mesh->tex_coordinates.size());
  reusex::core::debug("  UV index groups: {}",
                      textured_mesh->tex_coord_indices.size());

  if (textured_mesh->tex_materials.size() != 1) {
    throw std::runtime_error("Expected 1 atlas material after merge");
  }

  if (textured_mesh->tex_polygons.size() != 1) {
    throw std::runtime_error("Expected 1 polygon group after merge");
  }

  if (textured_mesh->tex_coordinates.size() != 1) {
    throw std::runtime_error("Expected 1 UV coord group after merge");
  }

  if (textured_mesh->tex_coord_indices.size() != 1) {
    throw std::runtime_error("Expected 1 UV index group after merge");
  }

  const size_t num_polygons = textured_mesh->tex_polygons[0].size();
  const size_t num_uv_indices = textured_mesh->tex_coord_indices[0].size();

  if (num_polygons != num_uv_indices) {
    throw std::runtime_error(fmt::format(
        "Final polygon/UV index mismatch: {} polygons, {} UV indices",
        num_polygons, num_uv_indices));
  }

  reusex::core::info("Atlas merge successful: {} polygons with {} UV indices",
                     num_polygons, num_uv_indices);

  // Verify atlas mapping in debug mode
  if (debug_distinct_colors) {
    verify_atlas_mapping(textured_mesh, grid_size,
                         polygon_to_original_material);
  }

  return num_materials;
}

/**
 * @brief Generate distinct colors using Glasby LUT approach
 *
 * Uses HSV color space with evenly distributed hues for maximum distinction.
 *
 * @param index Material/polygon group index
 * @return BGR color (OpenCV format)
 */
static cv::Vec3b get_distinct_color(size_t index) {
  // Glasby-style LUT: use distinct hues with high saturation and value
  const int num_base_colors = 12;
  const int hue = (index * 360 / num_base_colors) % 360; // Evenly spaced hues

  // Vary saturation and value slightly for more colors
  const int sat_level = (index / num_base_colors) % 3;
  const int val_level = (index / (num_base_colors * 3)) % 3;

  const float saturation = 0.7f + (sat_level * 0.15f); // 0.7, 0.85, 1.0
  const float value = 0.7f + (val_level * 0.15f);      // 0.7, 0.85, 1.0

  // Convert HSV to RGB
  const float h = hue / 60.0f;
  const float c = value * saturation;
  const float x = c * (1.0f - std::abs(std::fmod(h, 2.0f) - 1.0f));
  const float m = value - c;

  float r, g, b;
  if (h < 1.0f) {
    r = c;
    g = x;
    b = 0;
  } else if (h < 2.0f) {
    r = x;
    g = c;
    b = 0;
  } else if (h < 3.0f) {
    r = 0;
    g = c;
    b = x;
  } else if (h < 4.0f) {
    r = 0;
    g = x;
    b = c;
  } else if (h < 5.0f) {
    r = x;
    g = 0;
    b = c;
  } else {
    r = c;
    g = 0;
    b = x;
  }

  // Convert to 0-255 range and BGR format (OpenCV)
  return cv::Vec3b(static_cast<uint8_t>((b + m) * 255),
                   static_cast<uint8_t>((g + m) * 255),
                   static_cast<uint8_t>((r + m) * 255));
}

/**
 * @brief Fill texture with distinct solid color for debugging UV mapping
 *
 * @param image Output texture image to fill
 * @param material_idx Index of this material/polygon group
 */
static void fill_texture_with_distinct_color(cv::Mat &image,
                                             size_t material_idx) {
  const cv::Vec3b color = get_distinct_color(material_idx);
  image.setTo(cv::Scalar(color[0], color[1], color[2]));

  // Add material index as text for extra clarity
  const std::string text = std::to_string(material_idx);
  const int font = cv::FONT_HERSHEY_SIMPLEX;
  const double font_scale = 2.0;
  const int thickness = 3;

  // Get text size for centering
  int baseline = 0;
  cv::Size text_size =
      cv::getTextSize(text, font, font_scale, thickness, &baseline);
  cv::Point text_pos((image.cols - text_size.width) / 2,
                     (image.rows + text_size.height) / 2);

  // Draw text with black outline for visibility
  cv::putText(image, text, text_pos, font, font_scale, cv::Scalar(0, 0, 0),
              thickness + 2);
  cv::putText(image, text, text_pos, font, font_scale,
              cv::Scalar(255, 255, 255), thickness);
}

/**
 * @brief Project full point cloud onto mesh surface for photo-realistic
 * textures
 *
 * Algorithm:
 * 1. For each texture pixel, compute 3D position on mesh surface
 * 2. Query k-d tree for point cloud points within search radius
 * 3. For each found point:
 *    - Project point onto polygon plane
 *    - Compute distance to plane
 *    - Filter by distance threshold (10cm)
 *    - Check if projection falls within polygon bounds (2D containment)
 *    - Optionally filter by normal consistency
 * 4. Blend colors from accepted points using distance weighting
 *
 * @param image Output texture image to fill (modified in-place)
 * @param poly_group Polygons sharing this material/texture
 * @param cloud_xyz Mesh vertices in 3D space
 * @param cloud_rgb Full point cloud with RGB colors to project
 * @param cloud_normals Optional point cloud normals for filtering (can be
 * nullptr)
 * @param u_axis Local U axis for 2D coordinate system
 * @param v_axis Local V axis for 2D coordinate system
 * @param poly_normal Normal vector of the polygon plane
 * @param poly_centroid Reference point on the polygon plane
 * @param uv_coords UV coordinates for each vertex
 * @param uv_indices UV index mapping for polygons
 */
static void project_pointcloud_to_mesh_texture(
    cv::Mat &image, const std::vector<pcl::Vertices> &poly_group,
    const CloudLocConstPtr &cloud_xyz, const CloudConstPtr &cloud_rgb,
    const CloudNConstPtr &cloud_normals, const Eigen::Vector3f &u_axis,
    const Eigen::Vector3f &v_axis, const Eigen::Vector3f &poly_normal,
    const Eigen::Vector3f &poly_centroid,
    const std::vector<Eigen::Vector2f,
                      Eigen::aligned_allocator<Eigen::Vector2f>> &uv_coords,
    const std::vector<pcl::Vertices> &uv_indices,
    const TextureQualityParams &quality) {

  const int tex_size = image.rows;

  // Build k-d tree for efficient spatial queries
  pcl::KdTreeFLANN<PointT> kdtree;
  kdtree.setInputCloud(cloud_rgb);

  const float distance_threshold = quality.distance_threshold;
  const float search_radius = quality.search_radius;
  const int max_neighbors = quality.max_neighbors;

  // Statistics
  int pixels_processed = 0;
  int pixels_colored = 0;
  int total_points_found = 0;
  int total_points_accepted = 0;

  // Process each pixel in texture (parallelized with OpenMP)
#pragma omp parallel for collapse(2)                                           \
    reduction(+ : pixels_processed, pixels_colored, total_points_found,        \
                  total_points_accepted)
  for (int py = 0; py < tex_size; ++py) {
    for (int px = 0; px < tex_size; ++px) {
      pixels_processed++;

      // Convert pixel to normalized UV [0,1]
      const float u_norm = static_cast<float>(px) / (tex_size - 1);
      const float v_norm =
          1.0f - (static_cast<float>(py) / (tex_size - 1)); // Flip V

      // Find which polygon contains this pixel and compute 3D position
      Eigen::Vector3f point_3d;
      bool inside_polygon = false;
      size_t matched_poly_idx = 0;

      for (size_t poly_idx = 0; poly_idx < poly_group.size(); ++poly_idx) {
        const auto &poly = poly_group[poly_idx];
        const auto &uv_idx = uv_indices[poly_idx];

        if (poly.vertices.size() < 3 || uv_idx.vertices.size() < 3) {
          continue;
        }

        // Check all triangles in this polygon (triangle fan)
        for (size_t tri_idx = 0; tri_idx + 2 < poly.vertices.size();
             ++tri_idx) {
          const Eigen::Vector2f &uv0 = uv_coords[uv_idx.vertices[0]];
          const Eigen::Vector2f &uv1 = uv_coords[uv_idx.vertices[tri_idx + 1]];
          const Eigen::Vector2f &uv2 = uv_coords[uv_idx.vertices[tri_idx + 2]];

          const Eigen::Vector2f pixel_uv(u_norm, v_norm);
          const float area = (uv1 - uv0).x() * (uv2 - uv0).y() -
                             (uv2 - uv0).x() * (uv1 - uv0).y();

          if (std::abs(area) < 1e-6f)
            continue;

          const float w0 = ((uv1 - pixel_uv).x() * (uv2 - pixel_uv).y() -
                            (uv2 - pixel_uv).x() * (uv1 - pixel_uv).y()) /
                           area;
          const float w1 = ((uv2 - pixel_uv).x() * (uv0 - pixel_uv).y() -
                            (uv0 - pixel_uv).x() * (uv2 - pixel_uv).y()) /
                           area;
          const float w2 = 1.0f - w0 - w1;

          // Check if inside triangle
          if (w0 >= -0.01f && w1 >= -0.01f && w2 >= -0.01f) {
            // Compute 3D position via barycentric interpolation
            const Eigen::Vector3f &p0 =
                cloud_xyz->points[poly.vertices[0]].getVector3fMap();
            const Eigen::Vector3f &p1 =
                cloud_xyz->points[poly.vertices[tri_idx + 1]].getVector3fMap();
            const Eigen::Vector3f &p2 =
                cloud_xyz->points[poly.vertices[tri_idx + 2]].getVector3fMap();

            point_3d = w0 * p0 + w1 * p1 + w2 * p2;
            matched_poly_idx = poly_idx;
            inside_polygon = true;
            break;
          }
        }
        if (inside_polygon)
          break;
      }

      if (!inside_polygon) {
        continue;
      }

      // Precompute the matched polygon's vertices in the (u_axis, v_axis)
      // plane frame so we can test projected-point containment below.
      const auto &matched_poly = poly_group[matched_poly_idx];
      std::vector<Eigen::Vector2f> poly_2d;
      poly_2d.reserve(matched_poly.vertices.size());
      for (auto v_idx : matched_poly.vertices) {
        const Eigen::Vector3f vp =
            cloud_xyz->points[v_idx].getVector3fMap() - poly_centroid;
        poly_2d.emplace_back(vp.dot(u_axis), vp.dot(v_axis));
      }

      // Query k-d tree for nearby point cloud points
      PointT search_point;
      search_point.x = point_3d.x();
      search_point.y = point_3d.y();
      search_point.z = point_3d.z();

      std::vector<int> neighbor_indices;
      std::vector<float> neighbor_distances;

      const int num_found =
          kdtree.radiusSearch(search_point, search_radius, neighbor_indices,
                              neighbor_distances, max_neighbors);
      if (num_found == 0) {
        continue;
      }

      total_points_found += num_found;

      // Project each found point onto polygon plane and filter
      float r_sum = 0.0f, g_sum = 0.0f, b_sum = 0.0f;
      float total_weight = 0.0f;
      int points_accepted = 0;

      for (size_t i = 0; i < neighbor_indices.size(); ++i) {
        const auto &pt = cloud_rgb->points[neighbor_indices[i]];
        const Eigen::Vector3f pt_pos(pt.x, pt.y, pt.z);

        // Compute distance from point to polygon plane
        const float plane_distance =
            std::abs(poly_normal.dot(pt_pos - poly_centroid));

        // Filter by distance threshold (10cm)
        if (plane_distance > distance_threshold) {
          continue;
        }

        // Project point onto plane
        const Eigen::Vector3f projected =
            pt_pos - poly_normal * poly_normal.dot(pt_pos - poly_centroid);

        // Convert projected 3D point to 2D plane coordinates
        const float u_proj = (projected - poly_centroid).dot(u_axis);
        const float v_proj = (projected - poly_centroid).dot(v_axis);
        const Eigen::Vector2f proj_2d(u_proj, v_proj);

        // Check if the projection falls inside the matched polygon's 2D
        // footprint (triangle-fan barycentric test, same tolerance as the
        // pixel-in-polygon test above). Skips contributions from points that
        // project outside the polygon — important for concave shapes and
        // along boundaries shared with neighbouring polygons.
        bool inside_poly_2d = false;
        for (size_t tri_idx = 0; tri_idx + 2 < poly_2d.size(); ++tri_idx) {
          const Eigen::Vector2f &q0 = poly_2d[0];
          const Eigen::Vector2f &q1 = poly_2d[tri_idx + 1];
          const Eigen::Vector2f &q2 = poly_2d[tri_idx + 2];
          const float tri_area =
              (q1 - q0).x() * (q2 - q0).y() - (q2 - q0).x() * (q1 - q0).y();
          if (std::abs(tri_area) < 1e-6f)
            continue;
          const float b0 = ((q1 - proj_2d).x() * (q2 - proj_2d).y() -
                            (q2 - proj_2d).x() * (q1 - proj_2d).y()) /
                           tri_area;
          const float b1 = ((q2 - proj_2d).x() * (q0 - proj_2d).y() -
                            (q0 - proj_2d).x() * (q2 - proj_2d).y()) /
                           tri_area;
          const float b2 = 1.0f - b0 - b1;
          if (b0 >= -0.01f && b1 >= -0.01f && b2 >= -0.01f) {
            inside_poly_2d = true;
            break;
          }
        }
        if (!inside_poly_2d) {
          continue;
        }

        // Optional: Normal filtering if normals are available
        bool normal_ok = true;
        if (cloud_normals != nullptr &&
            // radiusSearch only ever emits valid, non-negative point indices,
            // so widening to size_t cannot wrap here.
            static_cast<std::size_t>(neighbor_indices[i]) <
                cloud_normals->size()) {
          const auto &pt_normal = cloud_normals->points[neighbor_indices[i]];
          const Eigen::Vector3f pt_normal_vec =
              pt_normal.getNormalVector3fMap();

          // Check if point normal is roughly aligned with polygon normal
          const float normal_similarity =
              std::abs(poly_normal.dot(pt_normal_vec));
          if (normal_similarity < 0.5f) { // ~60° tolerance
            normal_ok = false;
          }
        }

        if (!normal_ok) {
          continue;
        }

        // Weight by inverse distance (closer points have more influence)
        // Use quadratic falloff for sharper detail, linear for smoother
        // blending
        const float dist_3d = (pt_pos - point_3d).norm();
        const float weight =
            quality.use_quadratic_falloff
                ? 1.0f / (dist_3d * dist_3d + 1e-6f) // 1/d² - sharper
                : 1.0f / (dist_3d + 1e-4f);          // 1/d - smoother

        r_sum += pt.r * weight;
        g_sum += pt.g * weight;
        b_sum += pt.b * weight;
        total_weight += weight;
        points_accepted++;
      }

      total_points_accepted += points_accepted;

      // Set pixel color if we found valid points
      if (total_weight > 0.0f) {
        const int r =
            std::clamp(static_cast<int>(r_sum / total_weight), 0, 255);
        const int g =
            std::clamp(static_cast<int>(g_sum / total_weight), 0, 255);
        const int b =
            std::clamp(static_cast<int>(b_sum / total_weight), 0, 255);

        image.at<cv::Vec3b>(py, px) = cv::Vec3b(b, g, r); // OpenCV uses BGR
        pixels_colored++;
      }
    }
  }

  reusex::core::debug("Point cloud projection: {}/{} pixels colored, "
                      "{} points found, {} points accepted ({}% pass rate)",
                      pixels_colored, pixels_processed, total_points_found,
                      total_points_accepted,
                      total_points_found > 0
                          ? (100 * total_points_accepted / total_points_found)
                          : 0);
}

pcl::TextureMesh::Ptr
texture_mesh_with_cloud(pcl::PolygonMesh::Ptr mesh, CloudConstPtr cloud,
                        CloudNConstPtr normals, bool debug_distinct_colors,
                        const TextureQualityParams &quality) {
  reusex::core::trace("Entering reusex::geometry::texture_mesh_with_cloud");

  reusex::core::info("Texture quality settings:");
  reusex::core::info(
      "  Adaptive resolution: {:.0f} px/m ({}x{} to {}x{} pixels)",
      quality.texels_per_meter, quality.min_resolution, quality.min_resolution,
      quality.max_resolution, quality.max_resolution);
  reusex::core::info("  Distance threshold: {:.1f} cm (smaller = sharper)",
                     quality.distance_threshold * 100);
  reusex::core::info("  Search radius: {:.1f} cm", quality.search_radius * 100);
  reusex::core::info("  Falloff: {}", quality.use_quadratic_falloff
                                          ? "quadratic (1/d²)"
                                          : "linear (1/d)");

  if (debug_distinct_colors) {
    reusex::core::warn("DEBUG MODE: Using distinct colors (Glasby LUT) for UV "
                       "mapping verification");
  } else {
    if (normals) {
      reusex::core::info("Using point cloud normals for texture filtering");
    } else {
      reusex::core::info("No point cloud normals - skipping normal filtering");
    }
  }

  // Copy cloud and polygons
  pcl::TextureMesh::Ptr textured_mesh(new pcl::TextureMesh);
  textured_mesh->cloud = mesh->cloud;

  reusex::core::trace("Converting mesh cloud to PointXYZ");
  CloudLocPtr cloud_xyz(new CloudLoc);
  pcl::fromPCLPointCloud2(mesh->cloud, *cloud_xyz);
  CloudNPtr mesh_normals(new CloudN);
  mesh_normals->resize(cloud_xyz->size());
  mesh_normals->width = cloud_xyz->width;
  mesh_normals->height = cloud_xyz->height;
  for (auto &p : mesh_normals->points)
    p.getNormalVector3fMap() = Eigen::Vector3f::Zero();

  // INFO: Merge triangles into polygons
  // Assumption triangles making up polygons are consecutive in the list of
  // polygons
  Eigen::Vector3f prev_normal = Eigen::Vector3f::Zero();
  for (const auto &poly : mesh->polygons) {

    // Compute normal of the polygon
    const Eigen::Vector3f f_normal = compute_polygon_normal(poly, cloud_xyz);

    // Assign normal to all vertices in the polygon
    for (const auto &vertex : poly.vertices)
      mesh_normals->points[vertex].getNormalVector3fMap() += f_normal;

    // Start a new polygon
    if (f_normal.dot(prev_normal) < 0.95f) {
      textured_mesh->tex_polygons.push_back(std::vector<pcl::Vertices>());
      prev_normal = f_normal;
    }
    textured_mesh->tex_polygons.back().push_back(poly);
  }

  // Normalize all normals
  for (auto &p : mesh_normals->points)
    p.getNormalVector3fMap().normalize();

  reusex::core::trace("Concatenateing fields normals to the mesh cloud");
  pcl::PointCloud<pcl::PointNormal> cloud_with_normals;
  pcl::concatenateFields(*cloud_xyz, *mesh_normals, cloud_with_normals);
  pcl::toPCLPointCloud2(cloud_with_normals, textured_mesh->cloud);

  reusex::core::trace("Create directory for textures");
  std::filesystem::path base_path = prepare_texture_dir(quality.texture_dir);

  reusex::core::trace("Resizing texture mesh structures");
  const size_t nr_polygons = textured_mesh->tex_polygons.size();
  textured_mesh->tex_materials.resize(nr_polygons);
  textured_mesh->tex_coordinates.resize(nr_polygons);
  textured_mesh->tex_coord_indices.resize(nr_polygons);
  for (size_t i = 0; i < textured_mesh->tex_polygons.size(); ++i)
    textured_mesh->tex_coord_indices[i].resize(
        textured_mesh->tex_polygons[i].size());

  reusex::core::trace("Create materials");
  for (auto &&[idx, mat] :
       textured_mesh->tex_materials | ranges::views::enumerate) {

    auto name = fmt::format("texture-{:06}", idx);
    std::filesystem::path texture_path =
        base_path / fmt::format("{}.jpg", name);

    mat.tex_file = texture_path.string();
    mat.tex_name = name;

    mat.tex_Ka.r = 0.2f;
    mat.tex_Ka.g = 0.2f;
    mat.tex_Ka.b = 0.2f;

    mat.tex_Kd.r = 0.8f;
    mat.tex_Kd.g = 0.8f;
    mat.tex_Kd.b = 0.8f;

    mat.tex_Ks.r = 1.0f;
    mat.tex_Ks.g = 1.0f;
    mat.tex_Ks.b = 1.0f;

    mat.tex_d = 1.0f;
    mat.tex_Ns = 75.0f;
    mat.tex_illum = 2;
  }

  reusex::core::trace("Create textures from mesh vertex colors");

  // Use the original cloud with RGB data (passed as parameter)
  // Note: mesh->cloud has been converted to PointNormal and no longer has RGB
  CloudConstPtr mesh_cloud = cloud;

  {
    auto observer = reusex::core::ProgressObserver(
        reusex::core::Stage::creating_material, nr_polygons);

    for (auto &&[idx, mat] :
         textured_mesh->tex_materials | ranges::views::enumerate) {

      const auto &poly_group = textured_mesh->tex_polygons[idx];

      if (poly_group.empty()) {
        // Empty polygon group - create minimal texture
        cv::Mat image(quality.min_resolution, quality.min_resolution, CV_8UC3,
                      cv::Scalar(0, 0, 0));
        cv::imwrite(mat.tex_file, image);
        ++observer;
        continue;
      }

      // Compute bounding box of all vertices in this polygon group
      Eigen::Vector3f min_pt(std::numeric_limits<float>::max(),
                             std::numeric_limits<float>::max(),
                             std::numeric_limits<float>::max());
      Eigen::Vector3f max_pt(std::numeric_limits<float>::lowest(),
                             std::numeric_limits<float>::lowest(),
                             std::numeric_limits<float>::lowest());

      for (const auto &poly : poly_group) {
        for (const auto &vertex_idx : poly.vertices) {
          const auto &pt = cloud_xyz->points[vertex_idx];
          min_pt = min_pt.cwiseMin(pt.getVector3fMap());
          max_pt = max_pt.cwiseMax(pt.getVector3fMap());
        }
      }

      // Compute polygon normal (use first polygon's normal)
      Eigen::Vector3f poly_normal =
          compute_polygon_normal(poly_group[0], cloud_xyz);
      poly_normal.normalize();

      // Compute polygon centroid (average of all vertices in group)
      Eigen::Vector3f poly_centroid = Eigen::Vector3f::Zero();
      int vertex_count = 0;
      for (const auto &poly : poly_group) {
        for (const auto &vertex_idx : poly.vertices) {
          poly_centroid += cloud_xyz->points[vertex_idx].getVector3fMap();
          vertex_count++;
        }
      }
      if (vertex_count > 0) {
        poly_centroid /= static_cast<float>(vertex_count);
      }

      // Create local 2D coordinate system on the polygon plane
      Eigen::Vector3f u_axis, v_axis;
      if (std::abs(poly_normal.z()) < 0.9f) {
        u_axis = poly_normal.cross(Eigen::Vector3f::UnitZ()).normalized();
      } else {
        u_axis = poly_normal.cross(Eigen::Vector3f::UnitX()).normalized();
      }
      v_axis = poly_normal.cross(u_axis).normalized();

      // Compute 2D bounds in local coordinate system
      float min_u = std::numeric_limits<float>::max();
      float max_u = std::numeric_limits<float>::lowest();
      float min_v = std::numeric_limits<float>::max();
      float max_v = std::numeric_limits<float>::lowest();

      for (const auto &poly : poly_group) {
        for (const auto &vertex_idx : poly.vertices) {
          const Eigen::Vector3f pt =
              cloud_xyz->points[vertex_idx].getVector3fMap();
          float u = pt.dot(u_axis);
          float v = pt.dot(v_axis);
          min_u = std::min(min_u, u);
          max_u = std::max(max_u, u);
          min_v = std::min(min_v, v);
          max_v = std::max(max_v, v);
        }
      }

      float u_range = max_u - min_u;
      float v_range = max_v - min_v;
      if (u_range < 1e-6f)
        u_range = 1.0f;
      if (v_range < 1e-6f)
        v_range = 1.0f;

      // ADAPTIVE RESOLUTION: Calculate texture size based on polygon physical
      // size Use the larger dimension to ensure square texture covers full area
      const float max_dimension = std::max(u_range, v_range);

      // Calculate target resolution: physical_size * texels_per_meter
      int tex_size = static_cast<int>(max_dimension * quality.texels_per_meter);

      // Round to nearest power of 2 for efficiency (optional but recommended)
      tex_size = std::pow(2, std::round(std::log2(tex_size)));

      // Clamp to min/max bounds
      tex_size =
          std::clamp(tex_size, quality.min_resolution, quality.max_resolution);

      reusex::core::debug(
          "Polygon group {}: {:.2f}m x {:.2f}m -> {}x{} texture ({:.0f} px/m)",
          idx, u_range, v_range, tex_size, tex_size, tex_size / max_dimension);

      // Create texture image with adaptive resolution
      cv::Mat image(tex_size, tex_size, CV_8UC3, cv::Scalar(0, 0, 0));

      // Create UV coordinates for all vertices in this polygon group
      std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>
          uv_coords;
      std::vector<pcl::Vertices> uv_indices;

      for (const auto &poly : poly_group) {
        pcl::Vertices uv_idx;
        uv_idx.vertices.reserve(poly.vertices.size());

        for (const auto &vertex_idx : poly.vertices) {
          const Eigen::Vector3f pt_3d =
              cloud_xyz->points[vertex_idx].getVector3fMap();

          // Project to 2D local coordinates
          float u = pt_3d.dot(u_axis);
          float v = pt_3d.dot(v_axis);

          // Normalize to [0, 1]
          float u_norm = (u - min_u) / u_range;
          float v_norm = (v - min_v) / v_range;

          // Store UV coordinate
          uv_coords.emplace_back(u_norm, v_norm);
          uv_idx.vertices.push_back(
              static_cast<uint32_t>(uv_coords.size() - 1));
        }

        uv_indices.push_back(uv_idx);
      }

      if (debug_distinct_colors) {
        // Debug mode: Fill with distinct solid color for UV mapping
        // verification
        reusex::core::debug(
            "Filling texture {} with distinct color (debug mode)", idx);
        fill_texture_with_distinct_color(image, idx);
      } else {
        // Normal mode: Project full point cloud onto mesh surface
        reusex::core::debug("Projecting point cloud onto texture {} ({}x{} px, "
                            "{} polygons, {:.0f}cm threshold)",
                            idx, tex_size, tex_size, poly_group.size(),
                            quality.distance_threshold * 100);
        project_pointcloud_to_mesh_texture(
            image, poly_group, cloud_xyz,
            mesh_cloud, // Full point cloud with RGB colors
            normals,    // Optional point cloud normals for filtering
            u_axis, v_axis, poly_normal, poly_centroid, uv_coords, uv_indices,
            quality // Quality parameters for sharp textures
        );
      }

      // Store UV coordinates for this material
      textured_mesh->tex_coordinates[idx] = uv_coords;
      textured_mesh->tex_coord_indices[idx] = uv_indices;

      reusex::core::debug("Texture {}: {} UV coords, {} UV indices", idx,
                          uv_coords.size(), uv_indices.size());

      cv::imwrite(mat.tex_file, image);
      ++observer;
    }
  }

  reusex::core::debug("Number of materials: {}",
                      textured_mesh->tex_materials.size());
  reusex::core::debug("Number of texture polygons: {}",
                      textured_mesh->tex_polygons.size());
  reusex::core::debug("Number of texture coordinates: {}",
                      textured_mesh->tex_coordinates.size());
  for (auto [idx, poly_group] :
       textured_mesh->tex_polygons | ranges::views::enumerate) {
    reusex::core::debug("  Number of polygons in group {}: {}", idx,
                        poly_group.size());
  }
  reusex::core::trace("Nuber of coordinates {}",
                      textured_mesh->tex_coord_indices.size());

  // Merge all textures into a single atlas for PCL visualization compatibility
  // Auto-scale atlas tile size based on number of textures to stay within
  // OpenCV limits
  const size_t num_materials = textured_mesh->tex_materials.size();
  const int grid_size = static_cast<int>(
      std::ceil(std::sqrt(static_cast<double>(num_materials))));

  // OpenCV limit: CV_IO_MAX_IMAGE_PIXELS = 1e9 (1 billion pixels)
  // Leave 10% safety margin
  constexpr int64_t max_pixels = static_cast<int64_t>(1e9 * 0.9);
  const int max_safe_tile_size =
      static_cast<int>(std::sqrt(max_pixels) / grid_size);

  int actual_tile_size = std::min(quality.atlas_tile_size, max_safe_tile_size);

  // Round down to nearest power of 2 for efficiency
  actual_tile_size = std::pow(2, std::floor(std::log2(actual_tile_size)));

  const int64_t atlas_pixels =
      static_cast<int64_t>(grid_size * actual_tile_size) *
      static_cast<int64_t>(grid_size * actual_tile_size);

  if (actual_tile_size < quality.atlas_tile_size) {
    reusex::core::warn("Reducing atlas tile size from {}x{} to {}x{} for {} "
                       "textures (OpenCV limit)",
                       quality.atlas_tile_size, quality.atlas_tile_size,
                       actual_tile_size, actual_tile_size, num_materials);
  }

  reusex::core::info(
      "Atlas: {}x{} grid, {}x{} tiles = {}x{} total ({:.1f}M pixels)",
      grid_size, grid_size, actual_tile_size, actual_tile_size,
      grid_size * actual_tile_size, grid_size * actual_tile_size,
      atlas_pixels / 1e6);

  std::filesystem::path atlas_path = base_path / "texture_atlas.jpg";
  size_t atlased_count = create_texture_atlas(
      textured_mesh, actual_tile_size, atlas_path, debug_distinct_colors);

  reusex::core::info(
      "Texture atlas created from {} textures -> 1 atlas material",
      atlased_count);
  reusex::core::debug(
      "After atlasing - Materials: {}, Polygons: {}, Coordinates: {}",
      textured_mesh->tex_materials.size(), textured_mesh->tex_polygons.size(),
      textured_mesh->tex_coordinates.size());

  return textured_mesh;
}

pcl::TextureMesh::Ptr
texture_mesh(pcl::PolygonMesh::Ptr mesh,
             std::map<int, rtabmap::Transform> const &poses,
             std::map<int, rtabmap::Signature> const &nodes,
             const std::filesystem::path &texture_dir) {
  reusex::core::trace("Entering reusex::geometry::texture_mesh");

  // Copy cloud and polygons
  pcl::TextureMesh::Ptr textured_mesh(new pcl::TextureMesh);
  textured_mesh->cloud = mesh->cloud;

  reusex::core::trace("Converting mesh cloud to PointXYZ");
  CloudLocPtr cloud(new CloudLoc);
  pcl::fromPCLPointCloud2(mesh->cloud, *cloud);
  CloudNPtr normals(new CloudN);
  normals->resize(cloud->size());
  normals->width = cloud->width;
  normals->height = cloud->height;
  for (auto &p : normals->points)
    p.getNormalVector3fMap() = Eigen::Vector3f::Zero();

  // INFO: Merge triangles into polygons
  // Assumption triangles making up polygons are consecutive in the list of
  // polygons
  textured_mesh->tex_polygons.resize(1);
  textured_mesh->tex_polygons[0] = mesh->polygons;

  for (const auto &poly : mesh->polygons) {

    // Compute normal of the polygon
    const Eigen::Vector3f f_normal = compute_polygon_normal(poly, cloud);

    // Assign normal to all vertices in the polygon
    for (const auto &vertex : poly.vertices)
      normals->points[vertex].getNormalVector3fMap() += f_normal;
  }

  // Normalize all normals
  for (auto &p : normals->points)
    p.getNormalVector3fMap().normalize();

  reusex::core::trace("Concatenateing fields normals to the mesh cloud");
  pcl::PointCloud<pcl::PointNormal> cloud_with_normals;
  pcl::concatenateFields(*cloud, *normals, cloud_with_normals);
  pcl::toPCLPointCloud2(cloud_with_normals, textured_mesh->cloud);

  reusex::core::debug("Number of polygons: {}",
                      textured_mesh->tex_polygons.size());
  size_t num_faces = 0;
  for (const auto &poly_group : textured_mesh->tex_polygons)
    num_faces += poly_group.size();
  reusex::core::debug("Total number of faces: {}", num_faces);
  reusex::core::debug("Number of vertices: {}", cloud->size());

  reusex::core::trace("Create directory for textures");
  std::filesystem::path base_path = prepare_texture_dir(texture_dir);

  reusex::core::trace("Retrive all cameras and textures from the database");
  pcl::texture_mapping::CameraVector cameras{};
  cameras.resize(poses.size());
  {
    auto observer = reusex::core::ProgressObserver(
        reusex::core::Stage::retrieving_textures, poses.size());

    for (auto &&[i, inner] : poses | ranges::views::enumerate) {
      auto &[id, pose] = inner;

      rtabmap::Signature node = nodes.find(id)->second;
      rtabmap::SensorData data = node.sensorData();
      data.uncompressData();

      cv::Mat image = data.imageRaw();
      if (image.empty()) {
        reusex::core::warn("Empty image for node id {}", id);
        ++observer;
        continue;
      }
      auto name = fmt::format("texture-{:06}", i);
      std::filesystem::path texture_path =
          base_path / fmt::format("{}.jpg", name);
      cv::imwrite(texture_path.string(), image);

      cameras[i] = pcl::texture_mapping::Camera();
      cameras[i].focal_length_w = data.cameraModels().at(0).fx();
      cameras[i].focal_length_h = data.cameraModels().at(0).fy();
      cameras[i].center_w = data.cameraModels().at(0).cx();
      cameras[i].center_h = data.cameraModels().at(0).cy();
      cameras[i].width = static_cast<int>(image.cols);
      cameras[i].height = static_cast<int>(image.rows);
      cameras[i].pose = Eigen::Affine3f(pose.toEigen4f());
      cameras[i].texture_file = texture_path.string();

      pcl::TexMaterial material;
      material.tex_file = texture_path.string();
      material.tex_name = name;

      material.tex_Ka.r = 0.2f;
      material.tex_Ka.g = 0.2f;
      material.tex_Ka.b = 0.2f;

      material.tex_Kd.r = 0.8f;
      material.tex_Kd.g = 0.8f;
      material.tex_Kd.b = 0.8f;

      material.tex_Ks.r = 1.0f;
      material.tex_Ks.g = 1.0f;
      material.tex_Ks.b = 1.0f;

      material.tex_d = 1.0f;
      material.tex_Ns = 75.0f;
      material.tex_illum = 2;

      textured_mesh->tex_materials.push_back(material);
      ++observer;
    }
  }

  reusex::core::debug("Number of cameras: {}", cameras.size());
  reusex::core::debug("Number of materials: {}",
                      textured_mesh->tex_materials.size());
  reusex::core::debug("Number of texture polygons: {}",
                      textured_mesh->tex_polygons.size());
  reusex::core::debug("Number of texture coordinates: {}",
                      textured_mesh->tex_coordinates.size());

  pcl::TextureMapping<pcl::PointXYZ> tm;
  tm.textureMeshwithMultipleCameras(*textured_mesh, cameras);

  if (textured_mesh->tex_polygons.size() >
      textured_mesh->tex_materials.size()) {
    pcl::TexMaterial material;
    material.tex_name = "texture-unassigned";
    material.tex_file = textured_mesh->tex_materials[0].tex_file;
    reusex::core::warn(
        "Number of texture polygons ({}) is greater than number of "
        "materials ({}). Adding default material '{}' at {}.",
        textured_mesh->tex_polygons.size(), textured_mesh->tex_materials.size(),
        material.tex_name, material.tex_file);
    textured_mesh->tex_materials.push_back(material);
  }

  reusex::core::debug("Number of materials: {}",
                      textured_mesh->tex_materials.size());
  reusex::core::debug("Number of texture polygons: {}",
                      textured_mesh->tex_polygons.size());
  reusex::core::debug("Number of texture coordinates: {}",
                      textured_mesh->tex_coordinates.size());
  for (auto [idx, poly_group] :
       textured_mesh->tex_polygons | ranges::views::enumerate) {
    reusex::core::debug("  Number of polygons in group {}: {}", idx,
                        poly_group.size());
  }
  reusex::core::trace("Nuber of coordinates {}",
                      textured_mesh->tex_coord_indices.size());
  if (textured_mesh->tex_coord_indices.size() !=
      textured_mesh->tex_polygons.size()) {
    reusex::core::warn(
        "Number of texture coordinate indices ({}) does not match "
        "number of texture polygons ({}).",
        textured_mesh->tex_coord_indices.size(),
        textured_mesh->tex_polygons.size());
    textured_mesh->tex_coord_indices.resize(textured_mesh->tex_polygons.size());
    for (size_t i = 0; i < textured_mesh->tex_polygons.size(); ++i) {
      textured_mesh->tex_coord_indices[i].resize(
          textured_mesh->tex_polygons[i].size());
    }
  }

  return textured_mesh;
}

pcl::TextureMesh::Ptr texture_mesh(pcl::PolygonMesh::Ptr mesh,
                                   std::map<int, CameraData> const &cameras,
                                   const std::filesystem::path &texture_dir) {
  reusex::core::trace(
      "Entering reusex::geometry::texture_mesh (ProjectDB API)");

  // Copy cloud and polygons
  pcl::TextureMesh::Ptr textured_mesh(new pcl::TextureMesh);
  textured_mesh->cloud = mesh->cloud;

  reusex::core::trace("Converting mesh cloud to PointXYZ");
  CloudLocPtr cloud(new CloudLoc);
  pcl::fromPCLPointCloud2(mesh->cloud, *cloud);
  CloudNPtr normals(new CloudN);
  normals->resize(cloud->size());
  normals->width = cloud->width;
  normals->height = cloud->height;
  for (auto &p : normals->points)
    p.getNormalVector3fMap() = Eigen::Vector3f::Zero();

  // Merge triangles into polygons
  textured_mesh->tex_polygons.resize(1);
  textured_mesh->tex_polygons[0] = mesh->polygons;

  for (const auto &poly : mesh->polygons) {
    const Eigen::Vector3f f_normal = compute_polygon_normal(poly, cloud);
    for (const auto &vertex : poly.vertices)
      normals->points[vertex].getNormalVector3fMap() += f_normal;
  }

  // Normalize all normals
  for (auto &p : normals->points)
    p.getNormalVector3fMap().normalize();

  reusex::core::trace("Concatenating normals to mesh cloud");
  pcl::PointCloud<pcl::PointNormal> cloud_with_normals;
  pcl::concatenateFields(*cloud, *normals, cloud_with_normals);
  pcl::toPCLPointCloud2(cloud_with_normals, textured_mesh->cloud);

  reusex::core::debug("Number of polygons: {}",
                      textured_mesh->tex_polygons.size());
  size_t num_faces = 0;
  for (const auto &poly_group : textured_mesh->tex_polygons)
    num_faces += poly_group.size();
  reusex::core::debug("Total number of faces: {}", num_faces);
  reusex::core::debug("Number of vertices: {}", cloud->size());

  reusex::core::trace("Creating directory for textures");
  std::filesystem::path base_path = prepare_texture_dir(texture_dir);

  reusex::core::trace("Extracting cameras and textures from ProjectDB data");
  pcl::texture_mapping::CameraVector pcl_cameras{};
  pcl_cameras.resize(cameras.size());
  {
    auto observer = reusex::core::ProgressObserver(
        reusex::core::Stage::retrieving_textures, cameras.size());

    for (auto &&[i, inner] : cameras | ranges::views::enumerate) {
      auto &[id, cam_data] = inner;

      const cv::Mat &image = cam_data.image;
      if (image.empty()) {
        reusex::core::warn("Empty image for camera id {}", id);
        ++observer;
        continue;
      }

      auto name = fmt::format("texture-{:06}", i);
      std::filesystem::path texture_path =
          base_path / fmt::format("{}.jpg", name);
      cv::imwrite(texture_path.string(), image);

      // Convert pose from Eigen::Matrix4d to Eigen::Affine3f
      Eigen::Affine3f pose_affine(cam_data.pose.cast<float>());

      pcl_cameras[i] = pcl::texture_mapping::Camera();
      pcl_cameras[i].focal_length_w = cam_data.intrinsics.fx;
      pcl_cameras[i].focal_length_h = cam_data.intrinsics.fy;
      pcl_cameras[i].center_w = cam_data.intrinsics.cx;
      pcl_cameras[i].center_h = cam_data.intrinsics.cy;
      pcl_cameras[i].width = image.cols;
      pcl_cameras[i].height = image.rows;
      pcl_cameras[i].pose = pose_affine;
      pcl_cameras[i].texture_file = texture_path.string();

      pcl::TexMaterial material;
      material.tex_file = texture_path.string();
      material.tex_name = name;

      material.tex_Ka.r = 0.2f;
      material.tex_Ka.g = 0.2f;
      material.tex_Ka.b = 0.2f;

      material.tex_Kd.r = 0.8f;
      material.tex_Kd.g = 0.8f;
      material.tex_Kd.b = 0.8f;

      material.tex_Ks.r = 1.0f;
      material.tex_Ks.g = 1.0f;
      material.tex_Ks.b = 1.0f;

      material.tex_d = 1.0f;
      material.tex_Ns = 75.0f;
      material.tex_illum = 2;

      textured_mesh->tex_materials.push_back(material);
      ++observer;
    }
  }

  reusex::core::debug("Number of cameras: {}", pcl_cameras.size());
  reusex::core::debug("Number of materials: {}",
                      textured_mesh->tex_materials.size());

  pcl::TextureMapping<pcl::PointXYZ> tm;
  tm.textureMeshwithMultipleCameras(*textured_mesh, pcl_cameras);

  if (textured_mesh->tex_polygons.size() >
      textured_mesh->tex_materials.size()) {
    pcl::TexMaterial material;
    material.tex_name = "texture-unassigned";
    material.tex_file = textured_mesh->tex_materials[0].tex_file;
    reusex::core::warn("Number of texture polygons ({}) > materials ({}). "
                       "Adding default material.",
                       textured_mesh->tex_polygons.size(),
                       textured_mesh->tex_materials.size());
    textured_mesh->tex_materials.push_back(material);
  }

  reusex::core::debug("Final - Materials: {}, Polygons: {}, Coordinates: {}",
                      textured_mesh->tex_materials.size(),
                      textured_mesh->tex_polygons.size(),
                      textured_mesh->tex_coordinates.size());

  if (textured_mesh->tex_coord_indices.size() !=
      textured_mesh->tex_polygons.size()) {
    textured_mesh->tex_coord_indices.resize(textured_mesh->tex_polygons.size());
    for (size_t i = 0; i < textured_mesh->tex_polygons.size(); ++i) {
      textured_mesh->tex_coord_indices[i].resize(
          textured_mesh->tex_polygons[i].size());
    }
  }

  return textured_mesh;
}
} // namespace reusex::geometry
