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
#include <boost/iterator/filter_iterator.hpp>
#include <fmt/format.h>
#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <range/v3/range/concepts.hpp>
#include <range/v3/view/zip.hpp>

#include <set>
#include <string>
#include <vector>

namespace reusex::geometry {

// Tags to distinguish node types in CellComplex
enum class CellNodeType { cell, face, vertex };

struct CellVertexData {};
struct FaceData {
  int plane_id;
};
struct CellData {
  std::set<int> wall_ids{};
};

// Generic vertex bundle. `id` is shared across all node types; it is treated
// as a per-type counter (vertex/face/cell) by the construction code. Moving
// it into CellVertexData/FaceData/CellData and accessing through std::visit
// would be more type-safe but would force every existing `(*this)[v].id`
// call site to thread a visitor — not currently worth the churn.
struct VertexProperties {
  CellNodeType type;
  int id;
  Eigen::Vector3d pos;
  std::variant<CellVertexData, FaceData, CellData> data;
};

struct CellEdgeData {
  bool is_main = false;
};

class CellComplex
    : public boost::adjacency_list<boost::vecS,        // edge container
                                   boost::vecS,        // vertex container
                                   boost::undirectedS, // cells–faces are
                                   // bidirectional
                                   // boost::directedS,
                                   VertexProperties, // vertex bundle
                                   CellEdgeData      // edge bundle
                                   >,
      public Registry {
    protected:
  using Graph =
      boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS,
                            VertexProperties, CellEdgeData>;

    public:
  using Vertex = boost::graph_traits<Graph>::vertex_descriptor;
  using GraphIter = boost::graph_traits<CellComplex>::vertex_iterator;
  using AdjacencyIter = boost::graph_traits<CellComplex>::adjacency_iterator;

    protected:
  template <CellNodeType T> struct Is_Type {
    const CellComplex *g;
    Is_Type() = delete;
    explicit Is_Type(const CellComplex *graph) : g(graph) {}
    bool operator()(CellComplex::vertex_descriptor vd) const {
      auto const &prop = (*g)[vd];
      return prop.type == T;
    }
  };
  using IsVertex = Is_Type<CellNodeType::vertex>;
  using IsCell = Is_Type<CellNodeType::cell>;
  using IsFace = Is_Type<CellNodeType::face>;
  struct IsFaceBetweenCells {
    const CellComplex *g;
    IsFaceBetweenCells() = delete;
    // cppcheck-suppress uninitMemberVar
    explicit IsFaceBetweenCells(const CellComplex *graph) : g(graph) {}
    bool operator()(CellComplex::vertex_descriptor vd) const {
      auto const &prop = (*g)[vd];
      int n_adj = 0;
      auto [begin, end] = boost::adjacent_vertices(vd, *g);
      for (auto it = begin; it != end; ++it)
        if ((*g)[*it].type == CellNodeType::cell)
          ++n_adj;
      return prop.type == CellNodeType::face && n_adj >= 2;
    }
  };

  using VertexIterator = boost::filter_iterator<IsVertex, GraphIter>;
  using FaceIterator = boost::filter_iterator<IsFace, GraphIter>;
  using FaceBetweenCellIterator =
      boost::filter_iterator<IsFaceBetweenCells, GraphIter>;
  using CellIterator = boost::filter_iterator<IsCell, GraphIter>;
  using VertexOnFaceIterator = boost::filter_iterator<IsVertex, AdjacencyIter>;
  using FaceOnCellIterator = boost::filter_iterator<IsFace, AdjacencyIter>;
  using CellOnFaceIterator = boost::filter_iterator<IsCell, AdjacencyIter>;

    private:
  int vertex_count = 0;
  int face_count = 0;
  int cell_count = 0;

    public:
  size_t n_rooms, n_walls;

  /// Number of horizontal sections (storeys) in the arrangement, i.e.
  /// (# horizontal planes - 1). Cells are partitioned into these sections by
  /// the "c:section" property map; the Solidifier can solve the MIP per
  /// section to keep multi-room buildings tractable (issue #226).
  size_t n_sections = 0;

  CellComplex() = delete;

  /// Upper bound on the number of cells the arrangement may generate before
  /// construction aborts with a descriptive error (issue #213). The 2D
  /// arrangement grows ~quadratically with the vertical-plane count, so
  /// over-fragmented input can explode the downstream MIP. A run that would
  /// exceed this fails fast instead of hanging in the solver.
  static constexpr size_t default_max_cells = 5000;

  /// Default for `min_section_overlap_m` (reusex-sba): a wall reaches a
  /// section when its z-range overlaps the section by at least this much, or
  /// by half the section's height for a section thinner than twice this (a
  /// slab between storeys).
  static constexpr double default_min_section_overlap_m = 0.3;

  /**
   * @param plane_z_ranges World z-range [lo, hi] of each plane, index-aligned
   *   with @p planes (reusex-sba). When given, each horizontal section gets
   *   its own 2D arrangement, built only from the vertical planes whose range
   *   reaches into it, so one storey's walls no longer split every other
   *   storey: the cell count is the sum of the per-section arrangements
   *   instead of (faces of all walls) x (sections). The horizontal faces of a
   *   level between two sections are the overlay of the two arrangements,
   *   each shared by one cell below and one above. A range with lo > hi or a
   *   NaN reaches every section. A section no wall reaches (a slab between
   *   or around storeys) takes the walls of the sections next to it. Empty:
   *   every vertical plane spans every section (the behaviour before sba).
   * @param min_section_overlap_m See default_min_section_overlap_m.
   * @param obliques Indices into @p planes of oblique planes (roof slopes;
   *   reusex-e6z), used only with @p plane_z_ranges. A section whose own
   *   walls enclose no face (an attic under a pitched roof, whose only walls
   *   are its gables) takes, besides the walls of the nearest sections, the
   *   line where each oblique plane reaching into it by min_section_overlap_m
   *   crosses the section's mid-height: the attic's cells then stop under
   *   the roof instead of running out under the eaves. Such wall faces carry
   *   the oblique plane's id. An oblique plane with no z-range (NaN) or
   *   nearly horizontal (|n_z| > 0.95) is ignored.
   * @throws std::invalid_argument if @p plane_z_ranges is neither empty nor
   *   as long as @p planes.
   */
  CellComplex(
      std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>>
          &planes,
      std::vector<size_t> &verticals, const std::vector<size_t> &horizontals,
      std::vector<std::pair<size_t, size_t>> &pairs,
      std::array<double, 2> min_xy, std::array<double, 2> max_xy,
      std::optional<
          std::function<void(size_t, std::vector<std::array<double, 3>> const &,
                             std::vector<int> const &)>>
          viz_func = std::nullopt,
      size_t max_cells = default_max_cells,
      const std::vector<std::array<double, 2>> &plane_z_ranges = {},
      double min_section_overlap_m = default_min_section_overlap_m,
      const std::vector<size_t> &obliques = {});

  // Per-face coverage values are written to the "f:support_probability"
  // vertex property map and queried from there by downstream code.
  template <typename PointT = pcl::PointXYZ>
  auto compute_face_coverage(pcl::PointCloud<PointT>::ConstPtr cloud,
                             EigenVectorContainer<double, 4> &planes,
                             std::vector<pcl::IndicesPtr> &inliers,
                             const double grid_size = 0.2) -> void;

  template <typename PointT = pcl::PointXYZ, typename PointN = pcl::Normal,
            typename PointL = pcl::Label>
  auto compute_room_probabilities(pcl::PointCloud<PointT>::ConstPtr cloud,
                                  pcl::PointCloud<PointN>::ConstPtr normals,
                                  pcl::PointCloud<PointL>::ConstPtr labels,
                                  const double grid_size = 0.2) -> void;

  inline Vertex add_vertex(Eigen::Vector3d pos) {
    auto v = boost::add_vertex(*this);
    (*this)[v].type = CellNodeType::vertex;
    (*this)[v].id = vertex_count++;
    (*this)[v].pos = pos;
    (*this)[v].data = CellVertexData{};
    return v;
  };

  template <typename T>
  inline Vertex add_face(Eigen::Vector3d pos, T begin, T end,
                         int plane_id = -1) {
    auto f = boost::add_vertex(*this);
    (*this)[f].type = CellNodeType::face;
    (*this)[f].id = face_count++;
    (*this)[f].pos = pos;
    (*this)[f].data = FaceData{plane_id};
    for (auto it = begin; it != end; ++it)
      boost::add_edge(*it, f, *this);
    return f;
  };
  template <typename Range>
  inline Vertex add_face(Eigen::Vector3d pos, Range vertices,
                         int plane_id = -1) {
    return add_face(pos, std::begin(vertices), std::end(vertices), plane_id);
  };

  template <typename T>
  inline Vertex add_cell(Eigen::Vector3d pos, T begin, T end) {
    auto c = boost::add_vertex(*this);
    (*this)[c].type = CellNodeType::cell;
    (*this)[c].id = cell_count++;
    (*this)[c].pos = pos;
    (*this)[c].data = CellData{};
    for (auto it = begin; it != end; ++it)
      boost::add_edge(*it, c, *this);
    return c;
  };
  template <typename Range>
  inline Vertex add_cell(Eigen::Vector3d pos, Range faces) {
    return add_cell(pos, std::begin(faces), std::end(faces));
  };

  size_t num_vertices() const;
  size_t num_faces() const;
  size_t num_cells() const;

  std::ostream &operator<<(std::ostream &os) const;

  auto vertices_begin() const -> VertexIterator;
  auto vertices_end() const -> VertexIterator;

  auto faces_begin() const -> FaceIterator;
  auto faces_end() const -> FaceIterator;

  auto faces_between_cells_begin() const -> FaceBetweenCellIterator;
  auto faces_between_cells_end() const -> FaceBetweenCellIterator;

  auto cells_begin() const -> CellIterator;
  auto cells_end() const -> CellIterator;

  // CGAL-style adjacency iterators: "<elements>_of_<container>".
  auto vertices_of_face_begin(Vertex f) const -> VertexOnFaceIterator;
  auto vertices_of_face_end(Vertex f) const -> VertexOnFaceIterator;

  // auto cells_of_face_begin(Vertex f) const -> CellOnFaceIterator;
  // auto cells_of_face_end(Vertex f) const -> CellOnFaceIterator;

  auto faces_of_cell_begin(Vertex c) const -> FaceOnCellIterator;
  auto faces_of_cell_end(Vertex c) const -> FaceOnCellIterator;

  auto get_a(Vertex f) const -> Vertex;
  auto get_b(Vertex f) const -> Vertex;
};
} // namespace reusex::geometry

template <> struct fmt::formatter<reusex::geometry::CellComplex> {
  // Parse function (optional)
  template <typename ParseContext> constexpr auto parse(ParseContext &ctx) {
    return ctx.begin();
  }

  // Format function
  template <typename FormatContext>
  auto format(const reusex::geometry::CellComplex &obj,
              FormatContext &ctx) const {
    return fmt::format_to(ctx.out(), "[{}c {}f {}v {}r  {}w]", obj.num_cells(),
                          obj.num_faces(), obj.num_vertices(), obj.n_rooms,
                          obj.n_walls);
  }
};

// Implementation files
#include "reusex/reconstruction/CellComplexFaceCoverage.hpp"
#include "reusex/reconstruction/CellComplexRoomProbabilities.hpp"
