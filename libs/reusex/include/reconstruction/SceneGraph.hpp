// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "reusex/core/logging.hpp"
#include "reusex/reconstruction/Registry.hpp"
#include "reusex/types.hpp"

#include <Eigen/StdVector>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/graphviz.hpp> // write_graphviz, read_graphviz
#include <boost/iterator/filter_iterator.hpp>
#include <boost/property_map/dynamic_property_map.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <fmt/format.h>
#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <range/v3/range/concepts.hpp>
#include <range/v3/view/zip.hpp>

#include <fstream>
#include <set>
#include <string>
#include <vector>

namespace reusex::geometry {

enum class SceneNodeType { point_cluster, plane, object };

struct PointCluster {
  std::vector<int> point_indices;
};

struct ScenePlane {
  Eigen::Vector4d coefficients;
  Eigen::Vector3d origin;
};

struct SceneObject {
  int label;
};

// Generic vertex bundle
struct SceneVertexData {
  Eigen::Vector3d centroid;
  SceneNodeType type;
  std::variant<PointCluster, ScenePlane> data;
};

struct SceneEdgeData {
  //
};

// Superseded (decision D4, reusex-z1v.1.4): this stub is replaced by the
// solid-model scene_relations table (reusex-z1v.8.6), not completed as
// originally envisioned here — no HDF5 serialization anywhere in this build.
class SceneGraph
    : public boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS,
                                   SceneVertexData, SceneEdgeData>,
      public Registry {
    protected:
  using Graph =
      boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS,
                            SceneVertexData, SceneEdgeData>;

    public:
  using Vertex = boost::graph_traits<Graph>::vertex_descriptor;

    protected:
    private:
  CloudConstPtr cloud_ = nullptr;
  std::map<int, std::string> label_map_ = {};
  std::filesystem::path database_path_ = "";

    public:
  SceneGraph() = default;
  ~SceneGraph() = default;

  /** @brief Construct a scene graph from a point cloud and its normals
   *
   * This constructor takes a point cloud and its corresponding normals as
   * input and constructs a hierarchical scene graph representation of the
   * scene.
   */
  SceneGraph(CloudConstPtr cloud, CloudNConstPtr normals);

  /** @brief Extract point clouds corresponding to a specific label
   *
   * This method extracts all point clouds from the scene graph that are
   * associated with a specific semantic label.
   */
  std::vector<CloudConstPtr> extract(int label) const;

  /** @brief Extract point clouds corresponding to a specific label name
   *
   * This method extracts all point clouds from the scene graph that are
   * associated with a specific semantic label name.
   */
  std::vector<CloudConstPtr> extract(std::string const label_name) const;

  /** @brief Get all unique labels in the scene graph
   *
   * This method returns a set of all unique semantic labels present in the
   * scene graph.
   */
  std::set<int> get_labels() const;

  /** @brief Set the database path
   *
   * This method sets the path to the database containing the precomputed
   * labels for the point cloud views.
   */
  void set_database_path(std::filesystem::path db_path);

  /** @brief Save the scene graph to a file
   *
   * This method serizalizes the scene graph and saves it to a file.
   */
  int save(const std::filesystem::path &path) const;

  /** @brief Export the scene graph to differnet formats
   *
   * This method exports the scene graph to different formats such as .dot or
   * json for use in other applications.
   */
  int export(const std::filesystem::path &path) const;

  /** @brief Load the scene graph from a file
   *
   * This method deserializes the scene graph from a file.
   */
  static SceneGraph load(const std::filesystem::path &path);

    private:
  /** @brief Segment the point cloud into similar patches which are the smallest
   * building block of the graph
   *
   * This method uses super point to segment the point cloud into small patches
   * based on spatial proximity and normal similarity. Those patches are then
   * the leaf nodes in the scene graph and all higher level nodes are
   * constructed as compositions of those patches.
   */
  void patch_segmentation();

  /** @brief Grow planar regions from the patches
   *
   * This method iteratively grows planar regions from the patches by merging
   * neighboring patches that satisfy certain planarity criteria such as normal
   * similarity and distance to the plane.
   *
   * The result are Plane nodes in the scene graph that connect to the leaf
   * patches
   */
  void planar_region_growing();

  /** @brief Project the labels stored in the database on tho the assebled point
   * cloud.
   *
   * For each view in the database this constructs a croped point cloud,
   * computes a z-buffer and the assigns the precomputed labels to the closses
   * point in the point cloud.
   *
   * The result are semantic label nodes which are connected to the leaf
   * patches.
   */
  void project_labels_from_database();

  /** @brief Segment the scene into rooms
   *
   * This method segments the scene into rooms based on spatial visibility
   * relations. It uses raytracing to build a visibility graph between scene
   * parts, then applies the Leiden community detection algorithm (igraph) to
   * cluster them into rooms.
   *
   * The result are Room nodes in the scene graph that connect to
   * the semantic label nodes.
   */
  void segment_rooms();
};
} // namespace reusex::geometry

template <> struct fmt::formatter<reusex::geometry::SceneGraph> {
  // Parse function (optional)
  template <typename ParseContext> constexpr auto parse(ParseContext &ctx) {
    return ctx.begin();
  }

  // Format function
  template <typename FormatContext>
  auto format(const reusex::geometry::SceneGraph &obj,
              FormatContext &ctx) const {
    return fmt::format_to(ctx.out(), "SceneGraph");
  }
};

// Implementation files
