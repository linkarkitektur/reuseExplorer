// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: MIT

#pragma once
#include "reusex/core/logging.hpp"
#include "reusex/core/processing_observer.hpp"

#include <cmath>

#include <pcl/PointIndices.h>
#include <pcl/Vertices.h>
#include <pcl/common/pca.h>
#include <pcl/common/utils.h>
#include <pcl/memory.h>
#include <pcl/pcl_base.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>

#include <embree4/rtcore.h>

#include <igraph/igraph.h>

#include "pmmintrin.h"
#include "xmmintrin.h" // for SSE intrinsics raytracing

#include <algorithm>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

using IT = int32_t; // Index type
using NT = float;   // Numeric type

// Define OpenMP reduction for std::vector<std::tuple<int64_t, int64_t,
// double>>
#pragma omp declare reduction(                                                 \
        mergeTriplets : std::vector<std::tuple<IT, IT, NT>> : omp_out.insert(  \
                omp_out.end(), omp_in.begin(), omp_in.end()))                  \
    initializer(omp_priv = std::vector<std::tuple<IT, IT, NT>>())

namespace pcl {

/// Membership value for a vertex that no kept community claims.
constexpr int kNoCommunity = -1;

/** \brief Dissolve communities smaller than \p min_size into their
 * neighbours.
 *
 * Modularity-based community detection gives every isolated vertex, and every
 * small connected component, a community of its own. On a visibility graph
 * those are sample points that see (almost) nothing — points under furniture,
 * on an exterior face, behind clutter — not rooms. Each vertex of a community
 * with fewer than \p min_size members is moved to the kept community it has
 * the largest total edge weight to (ties go to the smallest community id).
 * Passes repeat, with every pass reading the previous pass's assignment, so a
 * dissolved vertex that only sees other dissolved vertices is reached once
 * they are assigned; the result does not depend on vertex order. A vertex
 * that never reaches a kept community gets \ref kNoCommunity.
 *
 * Kept communities are renumbered 0..k-1 in their original order.
 *
 * \param[in] n          number of vertices
 * \param[in] edges      undirected edges (vertex index pairs)
 * \param[in] weights    one weight per edge
 * \param[in] membership community id per vertex (0-based, dense)
 * \param[in] min_size   minimum community size to keep; <= 1 keeps all
 * \return the new membership, \ref kNoCommunity for unassigned vertices
 */
inline std::vector<int> dissolveSmallCommunities(
    std::size_t n, const std::vector<std::pair<int, int>> &edges,
    const std::vector<double> &weights, const std::vector<int> &membership,
    std::size_t min_size) {
  if (membership.size() != n || weights.size() != edges.size())
    throw std::invalid_argument(
        "dissolveSmallCommunities: membership/weights size mismatch");

  int n_communities = 0;
  for (const int c : membership) {
    if (c < 0)
      throw std::invalid_argument(
          "dissolveSmallCommunities: negative community id");
    n_communities = std::max(n_communities, c + 1);
  }

  std::vector<std::size_t> sizes(static_cast<std::size_t>(n_communities), 0);
  for (const int c : membership)
    ++sizes[static_cast<std::size_t>(c)];

  // Renumber kept communities densely, preserving their order.
  std::vector<int> new_id(static_cast<std::size_t>(n_communities),
                          kNoCommunity);
  int next_id = 0;
  for (std::size_t c = 0; c < sizes.size(); ++c)
    if (sizes[c] >= min_size)
      new_id[c] = next_id++;

  std::vector<int> out(n, kNoCommunity);
  for (std::size_t v = 0; v < n; ++v)
    out[v] = new_id[static_cast<std::size_t>(membership[v])];

  if (min_size <= 1)
    return out;

  // Adjacency in CSR form.
  std::vector<std::size_t> offset(n + 1, 0);
  for (const auto &[a, b] : edges) {
    ++offset[static_cast<std::size_t>(a) + 1];
    ++offset[static_cast<std::size_t>(b) + 1];
  }
  for (std::size_t v = 0; v < n; ++v)
    offset[v + 1] += offset[v];
  std::vector<std::pair<int, double>> adjacency(offset[n]);
  {
    std::vector<std::size_t> fill(offset.begin(), offset.end() - 1);
    for (std::size_t e = 0; e < edges.size(); ++e) {
      const auto [a, b] = edges[e];
      adjacency[fill[static_cast<std::size_t>(a)]++] = {b, weights[e]};
      adjacency[fill[static_cast<std::size_t>(b)]++] = {a, weights[e]};
    }
  }

  // Each pass assigns at least one vertex or ends the loop, so n passes bound
  // it (STANDARDS §5: no unbounded loops).
  std::vector<double> pull(static_cast<std::size_t>(next_id), 0.0);
  for (std::size_t pass = 0; pass < n; ++pass) {
    std::vector<int> next = out;
    bool changed = false;
    for (std::size_t v = 0; v < n; ++v) {
      if (out[v] != kNoCommunity)
        continue;
      std::vector<int> touched;
      for (std::size_t k = offset[v]; k < offset[v + 1]; ++k) {
        const int c = out[static_cast<std::size_t>(adjacency[k].first)];
        if (c == kNoCommunity)
          continue;
        if (pull[static_cast<std::size_t>(c)] == 0.0)
          touched.push_back(c);
        pull[static_cast<std::size_t>(c)] += adjacency[k].second;
      }
      int best = kNoCommunity;
      double best_pull = 0.0;
      for (const int c : touched) {
        const double p = pull[static_cast<std::size_t>(c)];
        if (p > best_pull || (p == best_pull && c < best)) {
          best = c;
          best_pull = p;
        }
        pull[static_cast<std::size_t>(c)] = 0.0;
      }
      if (best != kNoCommunity) {
        next[v] = best;
        changed = true;
      }
    }
    out.swap(next);
    if (!changed)
      break;
  }
  return out;
}

/** \brief Community-based point cloud clustering using Leiden algorithm.
 *
 * Builds a visibility graph between point cloud centroids using Embree ray
 * tracing, then partitions it into communities using the igraph Leiden
 * algorithm. Replaces the former Markov Clustering (MCL) approach.
 */
template <typename PointT, typename PointNT = pcl::Normal,
          typename PointLT = pcl::Label>
class PCL_EXPORTS CommunityClustering : public PCLBase<PointT> {
    public:
  using Ptr = std::shared_ptr<CommunityClustering<PointT, PointNT, PointLT>>;
  using ConstPtr =
      std::shared_ptr<const CommunityClustering<PointT, PointNT, PointLT>>;

  using Indices = pcl::Indices;
  using IndicesPtr = pcl::IndicesPtr;
  using IndicesConstPtr = pcl::IndicesConstPtr;

  using PointCloud = pcl::PointCloud<PointT>;
  using PointCloudPtr = typename PointCloud::Ptr;
  using PointCloudConstPtr = typename PointCloud::ConstPtr;

  using PointCloudN = pcl::PointCloud<PointNT>;
  using PointCloudNPtr = typename PointCloudN::Ptr;
  using PointCloudNConstPtr = typename PointCloudN::ConstPtr;

  using PointCloudL = pcl::PointCloud<PointLT>;
  using PointCloudLPtr = typename PointCloudL::Ptr;
  using PointCloudLConstPtr = typename PointCloudL::ConstPtr;

  using VisualizationCallback = std::function<void(
      typename pcl::PointCloud<pcl::PointXYZ>::Ptr,
      std::shared_ptr<std::vector<pcl::Vertices>>, CorrespondencesPtr)>;

    protected:
  using PCLBase<PointT>::input_;
  using PCLBase<PointT>::indices_;

  using Vertex = struct Vertex {
    float x, y, z, r; // x, y, z coordinates and radius
  };

  using Normal = struct Normal {
    float x, y, z; // x, y, z components of the normal vector
  };

    public:
  CommunityClustering() = default;
  ~CommunityClustering() override = default;

  inline void setInputNormals(const PointCloudNConstPtr &normals) {
    normals_ = normals;
  }

  inline PointCloudNConstPtr getInputNormals() const { return (normals_); }

  inline void setResolution(double resolution) { resolution_ = resolution; }

  inline double getResolution() const { return resolution_; }

  inline void setBeta(double beta) { beta_ = beta; }

  inline double getBeta() const { return beta_; }

  inline void setMaxIterations(int max_iter) { max_iter_ = max_iter; }

  inline int getMaxIterations() const { return max_iter_; }

  inline void setRadius(float radius) { radius_ = radius; }

  inline float getRadius() const { return radius_; }

  inline void setGridSize(float grid_size) { radius_ = grid_size * M_SQRT2; }

  inline float getGridSize() const { return radius_ / M_SQRT2; }

  inline size_t getNumClusters() const { return numClusters_; }

  /** \brief Label written for a sampled point that no kept community claims
   * (see setMinClusterSize()). */
  static constexpr std::uint32_t kUnassigned =
      std::numeric_limits<std::uint32_t>::max();

  /** \brief Communities with fewer vertices than this are dissolved into the
   * community they see most (see dissolveSmallCommunities()); <= 1 keeps all.
   */
  inline void setMinClusterSize(std::size_t min_size) {
    min_cluster_size_ = min_size;
  }

  inline std::size_t getMinClusterSize() const { return min_cluster_size_; }

  inline void registerVisualizationCallback(VisualizationCallback callback) {
    visualization_callback_ = callback;
  }

  /** \brief Cluster the input point cloud into communities.
   * \param[out] labels a point cloud for the community labels of each point.
   */
  void cluster(PointCloudL &labels) {

    reusex::core::trace("Initializing segmentation class");
    if (!initCompute()) {
      deinitCompute();
      return;
    }

    createScene();

    std::vector<std::tuple<IT, IT, NT>> triplets;

    rayTracing(triplets);

    if (triplets.empty())
      reusex::core::warn("No triplets found during ray tracing. Exiting.");

    if (visualization_callback_) {
      reusex::core::trace("Creating visualization context");
      using CloudV = pcl::PointCloud<pcl::PointXYZ>;
      using CloudVPtr = CloudV::Ptr;
      using Vertices = std::vector<pcl::Vertices>;
      using VecticesPtr = std::shared_ptr<Vertices>;

      CloudVPtr points(new CloudV);
      VecticesPtr vertices(new Vertices);
      create_visualization_context(points, vertices);
      CorrespondencesPtr correspondences(new Correspondences);

      correspondences->resize(triplets.size());
      for (size_t i = 0; i < triplets.size(); ++i) {
        const auto &[s, t, v] = triplets[i];
        (*correspondences)[i].index_query = static_cast<int>(s * (N_ + 1));
        (*correspondences)[i].index_match = static_cast<int>(t * (N_ + 1));
      }

      visualization_callback_(points, vertices, correspondences);
    }

    // Build igraph from triplets and run Leiden community detection
    const igraph_int_t n = static_cast<igraph_int_t>(indices_->size());

    // Build edge list and weights from triplets
    igraph_vector_int_t edges;
    igraph_vector_t weights;
    igraph_vector_int_init(&edges,
                           2 * static_cast<igraph_int_t>(triplets.size()));
    igraph_vector_init(&weights, static_cast<igraph_int_t>(triplets.size()));

    for (size_t i = 0; i < triplets.size(); ++i) {
      const auto &[s, t, w] = triplets[i];
      VECTOR(edges)[2 * i] = static_cast<igraph_int_t>(s);
      VECTOR(edges)[2 * i + 1] = static_cast<igraph_int_t>(t);
      VECTOR(weights)[i] = static_cast<igraph_real_t>(w);
    }
    triplets.clear();

    // Create undirected graph
    igraph_t graph;
    igraph_create(&graph, &edges, n, IGRAPH_UNDIRECTED);
    igraph_vector_int_destroy(&edges);

    // Run Leiden algorithm
    igraph_vector_int_t membership;
    igraph_vector_int_init(&membership, n);
    igraph_int_t nb_clusters = 0;
    igraph_real_t quality = 0.0;

    reusex::core::trace("Running Leiden community detection");
    reusex::core::stopwatch sw;

    igraph_int_t n_iter =
        (max_iter_ < 0) ? -1 : static_cast<igraph_int_t>(max_iter_);

    igraph_error_t err = igraph_community_leiden_simple(
        &graph,
        &weights, // edge_weights
        IGRAPH_LEIDEN_OBJECTIVE_MODULARITY,
        resolution_, // resolution parameter
        beta_,       // randomness in refinement phase
        false,       // start: false = fresh partition
        n_iter,      // n_iterations (-1 = until convergence)
        &membership, &nb_clusters, &quality);

    if (err != IGRAPH_SUCCESS) {
      reusex::core::error("igraph_community_leiden failed with error code {}",
                          static_cast<int>(err));
    }

    reusex::core::info("Leiden community detection completed in {:.3f} seconds",
                       sw);

    // Compute modularity for reporting
    igraph_real_t modularity = 0.0;
    igraph_modularity(&graph, &membership, &weights, resolution_,
                      IGRAPH_UNDIRECTED, &modularity);
    reusex::core::info(
        "Leiden results: {} clusters, quality: {:.4f}, modularity: {:.4f}",
        nb_clusters, quality, modularity);

    // Every isolated vertex and every small component is a community of its
    // own; dissolve those below the minimum size (reusex-3vc).
    std::vector<std::pair<int, int>> edge_list(
        static_cast<std::size_t>(igraph_ecount(&graph)));
    std::vector<double> weight_list(edge_list.size());
    std::size_t isolated = 0;
    {
      std::vector<char> has_edge(static_cast<std::size_t>(n), 0);
      for (igraph_int_t e = 0; e < igraph_ecount(&graph); ++e) {
        const auto a = static_cast<int>(IGRAPH_FROM(&graph, e));
        const auto b = static_cast<int>(IGRAPH_TO(&graph, e));
        edge_list[static_cast<std::size_t>(e)] = {a, b};
        weight_list[static_cast<std::size_t>(e)] = VECTOR(weights)[e];
        has_edge[static_cast<std::size_t>(a)] = 1;
        has_edge[static_cast<std::size_t>(b)] = 1;
      }
      for (const char h : has_edge)
        if (h == 0)
          ++isolated;
    }
    std::vector<int> leiden_membership(static_cast<std::size_t>(n));
    for (igraph_int_t i = 0; i < n; ++i)
      leiden_membership[static_cast<std::size_t>(i)] =
          static_cast<int>(VECTOR(membership)[i]);

    const std::vector<int> kept = dissolveSmallCommunities(
        static_cast<std::size_t>(n), edge_list, weight_list, leiden_membership,
        min_cluster_size_);

    int n_kept = 0;
    std::size_t unassigned = 0;
    std::size_t dissolved = 0; // vertices whose Leiden community was dropped
    {
      std::vector<std::size_t> leiden_size(
          static_cast<std::size_t>(nb_clusters), 0);
      for (const int c : leiden_membership)
        ++leiden_size[static_cast<std::size_t>(c)];
      for (std::size_t v = 0; v < kept.size(); ++v) {
        n_kept = std::max(n_kept, kept[v] + 1);
        if (kept[v] == kNoCommunity)
          ++unassigned;
        if (leiden_size[static_cast<std::size_t>(leiden_membership[v])] <
            min_cluster_size_)
          ++dissolved;
      }
    }
    reusex::core::info(
        "Visibility graph: {} vertices, {} edges, {} isolated. {} of {} "
        "communities have >= {} vertices; of the {} vertices in smaller "
        "ones, {} joined the community they see most and {} see none and "
        "stay unassigned",
        n, edge_list.size(), isolated, n_kept, nb_clusters, min_cluster_size_,
        dissolved, dissolved - unassigned, unassigned);

    // Extract membership into labels
    labels.resize(input_->size());
    for (igraph_int_t i = 0; i < n; ++i) {
      const int c = kept[static_cast<std::size_t>(i)];
      labels.points[indices_->at(i)].label =
          c == kNoCommunity ? kUnassigned : static_cast<std::uint32_t>(c);
    }
    numClusters_ = static_cast<size_t>(n_kept);

    // Cleanup igraph objects
    igraph_vector_int_destroy(&membership);
    igraph_vector_destroy(&weights);
    igraph_destroy(&graph);

    deinitCompute();
  }

  void rayTracing(std::vector<std::tuple<IT, IT, NT>> &triplets) {

    reusex::core::trace("Running ray tracing");
    reusex::core::stopwatch sw;

    const size_t N = indices_->size();
    auto observer = reusex::core::ProgressObserver(
        reusex::core::Stage::ray_tracing, N * (N - 1) / 2);

#pragma omp parallel for reduction(mergeTriplets : triplets)                   \
    schedule(dynamic, 4)
    for (size_t i = 0; i < N; i++) {
      for (size_t j = i + 1; j < N; j++) {
        const size_t idx_s = indices_->at(i);
        const size_t idx_t = indices_->at(j);

        Eigen::Vector3f point_s = input_->points[idx_s].getVector3fMap();
        Eigen::Vector3f point_t = input_->points[idx_t].getVector3fMap();

        Eigen::Vector3f normal_s =
            normals_->points[idx_s].getNormalVector3fMap();
        Eigen::Vector3f normal_t =
            normals_->points[idx_t].getNormalVector3fMap();

        point_s = point_s + (offset_ * normal_s);
        point_t = point_t + (offset_ * normal_t);

        RTCRay ray;
        ray.org_x = point_s.x();
        ray.org_y = point_s.y();
        ray.org_z = point_s.z();

        Eigen::Vector3f dir = point_t - point_s;

        ray.tnear = epsilon_;
        const auto dir_norm = dir.norm();
        ray.tfar = dir_norm - epsilon_;

        dir.normalize();

        ray.dir_x = dir.x();
        ray.dir_y = dir.y();
        ray.dir_z = dir.z();

        ray.time = 0.0f; // motion blur time, not used here
        ray.mask = -1;   // mask for ray types, not used here
        ray.id = 0;      // ray ID, not used here
        ray.flags = 0;   // ray flags, not used here

        ++observer;
        rtcOccluded1(scene_, &ray);

        // When no intersection is found, the ray data is not updated. Incase
        // a hit was found, the tfar component of the ray is set to -inf.
        if (ray.tfar == -INFINITY) // A.k.a An intersection was found
          continue;

        triplets.emplace_back(i, j, 1.0);
      }
    }
    reusex::core::debug("Ray tracing completed in {} seconds", sw);
  }

    protected:
  bool initCompute() {
    reusex::core::trace("Initializing CommunityClustering");

    _MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);
    _MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);

    reusex::core::trace("Creating device and scene for ray tracing");
    device_ = rtcNewDevice("verbose=0"); // 0-3
    scene_ = rtcNewScene(device_);
    rtcSetSceneFlags(scene_, RTC_SCENE_FLAG_ROBUST);
    rtcSetSceneBuildQuality(scene_, RTC_BUILD_QUALITY_HIGH);
    assert(device_ != nullptr && "Error creating Embree device");
    assert(scene_ != nullptr && "Error creating Embree scene");
    rtcSetDeviceErrorFunction(device_, rtcCheckError, nullptr);

    reusex::core::trace("Checking if input cloud is set");
    if (!PCLBase<PointT>::initCompute()) {
      PCL_ERROR("[pcl::%s::initCompute] Init failed.\n",
                getClassName().c_str());
      deinitCompute();
      return (false);
    }

    reusex::core::trace("Checking if input normals are set");
    if (!normals_) {
      PCL_ERROR("[pcl::%s::segment] Must specify normals.\n",
                getClassName().c_str());
      deinitCompute();
      return (false);
    }

    reusex::core::trace(
        "Checking if input normals size matches input cloud size");
    if (normals_->size() != input_->size()) {
      PCL_ERROR("[pcl::%s::segment] Number of points in input cloud "
                "(%zu) and normal "
                "cloud (%zu) do not match!\n",
                getClassName().c_str(),
                static_cast<std::size_t>(input_->size()),
                static_cast<std::size_t>(normals_->size()));
      deinitCompute();
      return (false);
    }

    reusex::core::trace("Checking if input normals are normalized");
    for (const auto &point : normals_->points) {
      if ((point.getNormalVector3fMap().norm() - 1.0f) > epsilon_) {
        PCL_WARN("[pcl::%s::initCompute] Input normals are not normalized.\n",
                 getClassName().c_str());
        return (false);
      }
    }

    reusex::core::trace("Checking if input normals contain NaN values");
    for (const auto &point : normals_->points) {
      if (std::isnan(point.normal_x) || std::isnan(point.normal_y) ||
          std::isnan(point.normal_z)) {
        PCL_WARN("[pcl::%s::initCompute] Input normals contain NaN values.\n",
                 getClassName().c_str());
        return (false);
      }
    }

    reusex::core::debug("Initialized with:");
    reusex::core::debug("resolution: {:.3}", resolution_);
    reusex::core::debug("beta: {:.3}", beta_);
    reusex::core::debug("max_iter: {}", max_iter_);
    reusex::core::debug("radius: {:.3f}", radius_);

    return (true);
  }

  void deinitCompute() {
    reusex::core::trace("Deinitializing CommunityClustering");
    pcl::PCLBase<PointT>::deinitCompute();
    reusex::core::trace("Releasing scene and device");
    rtcReleaseScene(scene_);
    rtcReleaseDevice(device_);

    _mm_setcsr(old_mxcsr); // restore old flags
  }

  void createScene() {
    reusex::core::trace("Creating scene geometry for ray tracing");

    geometry_ = rtcNewGeometry(device_, RTC_GEOMETRY_TYPE_ORIENTED_DISC_POINT);

    Vertex *vb = (Vertex *)rtcSetNewGeometryBuffer(
        geometry_, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT4, sizeof(Vertex),
        indices_->size());

    Normal *nb = (Normal *)rtcSetNewGeometryBuffer(
        geometry_, RTC_BUFFER_TYPE_NORMAL, 0, RTC_FORMAT_FLOAT3, sizeof(Normal),
        indices_->size());

    for (size_t i = 0; i < indices_->size(); ++i) {
      const auto id = indices_->at(i);
      const auto &p = input_->points[id];
      const auto &n = normals_->points[id];

      vb[i].x = static_cast<float>(p.x);
      vb[i].y = static_cast<float>(p.y);
      vb[i].z = static_cast<float>(p.z);
      vb[i].r = static_cast<float>(radius_) * 1.05f;

      nb[i].x = static_cast<float>(n.normal_x);
      nb[i].y = static_cast<float>(n.normal_y);
      nb[i].z = static_cast<float>(n.normal_z);
    }

    rtcSetGeometryMask(geometry_, 0xFFFFFFFF);

    rtcCommitGeometry(geometry_);
    rtcAttachGeometry(scene_, geometry_);
    rtcReleaseGeometry(geometry_);

    reusex::core::trace("Committing scene");
    rtcCommitScene(scene_);
  }

  static void rtcCheckError(void *, RTCError err, const char *str) {
    reusex::core::error("Embree error {}:", str);
    switch (err) {
    case RTC_ERROR_NONE:
      break;
    case RTC_ERROR_UNKNOWN:
      throw std::runtime_error("Embree: An unknown error has occurred.");
      break;
    case RTC_ERROR_INVALID_ARGUMENT:
      throw std::runtime_error("Embree: An invalid argument was specified.");
      break;
    case RTC_ERROR_INVALID_OPERATION:
      throw std::runtime_error(
          "Embree: The operation is not allowed for the specified object.");
      break;
    case RTC_ERROR_OUT_OF_MEMORY:
      throw std::runtime_error("Embree: There is not enough memory left to "
                               "complete the operation.");
      break;
    case RTC_ERROR_UNSUPPORTED_CPU:
      throw std::runtime_error(
          "Embree: The CPU is not supported as it does not support SSE2.");
      break;
    case RTC_ERROR_CANCELLED:
      throw std::runtime_error(
          "Embree: The operation got cancelled by a Memory Monitor "
          "Callback or Progress Monitor Callback function.");
      break;
    default:
      throw std::runtime_error("Embree: An invalid error has occurred.");
      break;
    }
  }

  void create_visualization_context(
      pcl::PointCloud<pcl::PointXYZ>::Ptr points,
      std::shared_ptr<std::vector<pcl::Vertices>> vertices) {

    Vertex *vb = (Vertex *)rtcGetGeometryBufferData(geometry_,
                                                    RTC_BUFFER_TYPE_VERTEX, 0);
    Normal *nb = (Normal *)rtcGetGeometryBufferData(geometry_,
                                                    RTC_BUFFER_TYPE_NORMAL, 0);
    assert(vb != nullptr && nb != nullptr);

    points->resize(indices_->size() * (N_ + 1)); // +1 for center point
    vertices->resize(indices_->size());

    for (size_t i = 0; i < indices_->size(); ++i) {
      pcl::Vertices *circle = &(*vertices)[i];
      circle->vertices.resize(N_);

      Eigen::Vector3d center(vb[i].x, vb[i].y, vb[i].z);
      Eigen::Vector3d dir(nb[i].x, nb[i].y, nb[i].z);

      points->points[i * (N_ + 1)] =
          pcl::PointXYZ(center.x(), center.y(), center.z());

      dir = dir.normalized();

      // pick an arbitrary vector not parallel to n
      Eigen::Vector3d arbitrary(1.0, 0.0, 0.0);
      if (std::fabs(dir.dot(arbitrary)) > 0.9) { // too close to parallel
        arbitrary = Eigen::Vector3d(0.0, 1.0, 0.0);
      }

      Eigen::Vector3d u, v;
      u = dir.cross(arbitrary).normalized(); // first basis vector
      v = dir.cross(u).normalized();         // second basis vector

      for (unsigned int j = 0; j < N_; ++j) {

        double theta = 2.0 * M_PI * j / N_;
        Eigen::Vector3d local = radius_ * (cos(theta) * u + sin(theta) * v);
        Eigen::Vector3d global = center + local;

        const size_t idx = i * (N_ + 1) + (j + 1);
        points->points[idx] = pcl::PointXYZ(global.x(), global.y(), global.z());
        circle->vertices[j] = static_cast<uint32_t>(idx);
      }
    };
  }

  inline const std::string getClassName() const {
    return ("CommunityClustering");
  }

    protected:
  PointCloudNConstPtr normals_{nullptr};

  long unsigned int numClusters_ = 0;

  float offset_{0.10f};
  float radius_{0.05f};

  RTCDevice device_{};
  RTCScene scene_{};
  RTCGeometry geometry_{};

  double resolution_{1.0};
  double beta_{0.01};
  int max_iter_{-1};                // negative = iterate until convergence
  std::size_t min_cluster_size_{1}; // 1 = keep every community

  VisualizationCallback visualization_callback_{nullptr};
  static constexpr unsigned int N_ =
      6; // Number of vertices to represent a disc

  constexpr static double epsilon_ = 1e-4;

  unsigned int old_mxcsr = _mm_getcsr(); // save current flags

    public:
  PCL_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace pcl
