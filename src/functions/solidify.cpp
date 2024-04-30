#include "types/PointCloud.hh"
#include "types/PlanarPointSet.hh"
#include "algorithms/markov_clustering.hh"
#include "algorithms/refine.hh"
#include "algorithms/clustering.hh"
#include "types/surface.hh"

#include <pcl/filters/random_sample.h>
#include <pcl/filters/experimental/functor_filter.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/PointIndices.h>

#include <embree3/rtcore.h>

template<typename PointT>
class MatchCondition
{
public:
    MatchCondition(){}

    // Overloaded function operator to be used as a functor
    bool operator()(const pcl::PointCloud<PointT> & cloud, pcl::index_t index)
    {
        auto is_inventory = 0 != cloud.points[index].semantic;
        auto is_surface = 0 != cloud.points[index].label;

        return !is_inventory && is_surface;
    }

};

using PointIndices = pcl::PointIndices;
using Indices = pcl::Indices;
using Clusters = std::vector<pcl::PointIndices>;
using Filter = pcl::experimental::advanced::FunctorFilter<linkml::PointCloud::PointType, MatchCondition<linkml::PointCloud::PointType>>;


Clusters extract_clusters(linkml::PointCloud::ConstPtr cloud){

    // Collect all cluster indices
    std::unordered_set<size_t> cluster_indices_set;
    for (size_t i = 0; i < cloud->points.size(); i++)
        cluster_indices_set.insert(cloud->points[i].label);

    cluster_indices_set.erase(0);

    Indices cluster_indices(cluster_indices_set.begin(), cluster_indices_set.end());

    // Create clusters
    Clusters clusters(cluster_indices.size());
    #pragma omp parallel for
    for (size_t i = 0; i < cluster_indices.size(); i++){
        size_t cluster_index = cluster_indices[i];
        for (size_t j = 0; j < cloud->points.size(); j++)
            if (cloud->points[j].label == cluster_index)
                clusters[i].indices.push_back(j);
    }

    return clusters;
}
Clusters filter_clusters(Clusters clusters){

    std::sort(clusters.begin(), clusters.end(), [](const auto& lhs, const auto& rhs){return lhs.indices.size() > rhs.indices.size();});

    auto temp = Clusters();
    for (int i = 0; i < std::min(30, (int)clusters.size()); i++)
        temp.push_back(clusters[i]);

    return temp;
}
linkml::PointCloud::Ptr filter_cloud(linkml::PointCloud::Ptr cloud, Clusters & clusters){

    std::unordered_set<size_t> cluster_indices_set;
    for (size_t i = 0; i < clusters.size(); i++)
        for (size_t j = 0; j < clusters[i].indices.size(); j++)
            cluster_indices_set.insert(clusters[i].indices[j]);

    pcl::PointIndices::Ptr selection(new pcl::PointIndices);
    selection->indices = pcl::Indices(cluster_indices_set.begin(), cluster_indices_set.end());

    pcl::ExtractIndices<linkml::PointCloud::PointType> filter;
    filter.setInputCloud(cloud);
    filter.setIndices(selection);
    filter.filter(*cloud);

    Clusters clusters_2 = extract_clusters(cloud);

    #pragma omp parallel for
    for (size_t i = 0; i < cloud->points.size(); i++)
        cloud->points[i].label = -1;

    #pragma omp parallel for
    for (size_t i = 0; i < clusters_2.size(); i++)
        for (size_t j = 0; j < clusters_2[i].indices.size(); j++)
            cloud->points[clusters_2[i].indices[j]].label = i;

    return cloud;
}


namespace linkml
{
    PointCloud PointCloud::solidify()
    {


        //FIXME: This is bad as makeShared() will make a copy of the whole point cloud
        auto cloud = this->makeShared();

        // Remove everyting that is not a surface
        auto match_filter = MatchCondition<PointCloud::PointType>();
        Filter pass(match_filter);
        pass.setInputCloud(cloud);
        pass.filter(*cloud);


        pcl::RandomSample<PointCloud::PointType> sampler;
        sampler.setInputCloud(cloud);
        sampler.setSample(500000);
        sampler.setSeed(0);
        sampler.filter(*cloud);

        // Extract clusters
        Clusters clusters = extract_clusters(cloud);

        std::cout << "Number of clusters: " << clusters.size() << std::endl;

        // Simplify clusters
        //clusters = refine(cloud, clusters);


        clusters = filter_clusters(clusters);

        std::cout << "Number of clusters after simplification: " << clusters.size() << std::endl;


        cloud = filter_cloud(cloud, clusters);

        std::cout << "Number of points after filtering: " << cloud->points.size() << std::endl;




        // Create surfaces
        std::vector<Surface> surfaces(clusters.size());
        auto surface_bar = util::progress_bar(surfaces.size(), "Creating surfaces");
        for (size_t i = 0; i < clusters.size(); i++){
            surfaces[i] = Surface(cloud, clusters[i].indices);
            surface_bar.update();
        }
        surface_bar.stop();



        // Ray tracing

        RTCDevice device = rtcNewDevice(NULL);
        RTCScene scene   = rtcNewScene(device);

        std::unordered_map<unsigned int, pcl::Indices> point_map;
        Rays rays;

        // TODO: Make this section parallel
        // Ensure Crate_Embree_Geometry is thread safe
        // Make custom reduction for rays
        auto embree_bar = util::progress_bar(surfaces.size(), "Creating Embree geometries");
        for (size_t i = 0; i < surfaces.size(); i++){
            auto r = surfaces[i].Create_Embree_Geometry(device, scene, point_map);
            rays.insert(rays.end(), r.begin(), r.end());
            embree_bar.update();
        }
        embree_bar.stop();

        rtcCommitScene(scene);

        // Instatiate the context
        RTCIntersectContext context;
        rtcInitIntersectContext(&context);

        // Intersect all rays with the scene
        rtcIntersect1M(scene, &context, (RTCRayHit*)&rays[0], (unsigned int)rays.size(), sizeof(RTCRayHit));

        std::unordered_map<size_t, unsigned int> id_to_int;
        std::unordered_map<unsigned int, size_t> int_to_id;

        size_t face_index = 0;
        for (auto & [id,_]:   point_map){
                id_to_int[id] = face_index;
                int_to_id[face_index] = id;
                face_index++;
        }

        // Initialize the matrix
        SpMat matrix = SpMat(point_map.size(), point_map.size());
        matrix += Eigen::VectorXd::Ones(matrix.cols()).template cast<FT>().asDiagonal();


        auto result_bar = util::progress_bar(rays.size(), "Processing rays");
        for (auto & ray : rays){
            if (ray.hit.geomID != RTC_INVALID_GEOMETRY_ID) {

                int source_int = id_to_int[ray.ray.id];
                int target_int = id_to_int[ray.hit.geomID];

                auto vaule = matrix.coeff(source_int, target_int) + 1;

                // Increment the matrix
                matrix.coeffRef(source_int, target_int) = vaule;
                matrix.coeffRef(target_int, source_int) = vaule;

            }
            result_bar.update();
        }
        result_bar.stop();
    
        // Release the scene and device
        rtcReleaseScene(scene);
        rtcReleaseDevice(device);


        // Markov Clustering
        //// 2.5 < infaltion < 2.8  => 3.5
        matrix = markov_clustering::run_mcl(matrix, 2, 2.5);
        std::set<std::vector<size_t>> mc_clusters = markov_clustering::get_clusters(matrix);
        std::printf("n_clusters sp_matrix: %d\n", (int)clusters.size());

        u_int8_t cluster_index = 0;
        for (const std::vector<size_t> & cluster: mc_clusters){
            for (const size_t & idx: cluster)
                cloud->points[int_to_id[(int)idx]].instance = cluster_index;
            cluster_index += 1;
        }
        std::cout << "Number of clusters after Markov Clustering: " << (int)cluster_index << std::endl;

        polyscope::myinit();
        cloud->display("cloud");

        
        return *cloud;
    }
} // namespace linkml
