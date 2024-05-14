#include "types/PointCloud.hh"
#include "types/PlanarPointSet.hh"
#include "algorithms/markov_clustering.hh"
#include "algorithms/refine.hh"
#include "algorithms/clustering.hh"
#include "types/surface.hh"
#include "functions/color.hh"

#include <pcl/filters/random_sample.h>
#include <pcl/filters/experimental/functor_filter.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/PointIndices.h>

#include <embree3/rtcore.h>

#include <opencv4/opencv2/opencv.hpp>
#include <omp.h>

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

        std::cout << "Number of planes: " << clusters.size() << std::endl;


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

        // Create scene
        RTCScene scene   = rtcNewScene(device);
        auto embree_bar = util::progress_bar(surfaces.size(), "Creating Embree Scene");
        for (size_t i = 0; i < surfaces.size(); i++)
            surfaces[i].Create_Embree_Geometry(device, scene);
        embree_bar.stop();


        rtcCommitScene(scene);

        auto point_map_bar = util::progress_bar(surfaces.size(), "Creating point map");
        // Create a map of point indicies
        std::unordered_map<unsigned int, pcl::Indices> point_map;
        for (auto & surface: surfaces)
            for (const auto & [id, indices]: surface)
                std::copy(indices.begin(), indices.end(), std::back_insert_iterator(point_map[id]));
        point_map_bar.stop();


        using Tile = std::pair<unsigned int, Surface *>;

        auto tile_bar = util::progress_bar(surfaces.size(), "Creating tiles");
        std::vector<Tile> tiles;
        for (auto & surface: surfaces)
            for (const auto & [id, _]: surface)
                tiles.push_back(std::make_pair(id, &surface));
        tile_bar.stop();
        std::cout << "Number of tiles: " << tiles.size() << std::endl;


        // Create rays
        using Rays = std::vector<std::pair<Ray, Tile*>>;
        FT const constexpr offset = 0.10;
        size_t const nrays = tiles.size()*(tiles.size()-1)/2;
        Rays rays(nrays);
        std::cout << "Number of rays: " << nrays << std::endl;
        auto creating_rays_bar = util::progress_bar(nrays, "Creating rays");
        #pragma omp parallel for collapse(2)
        for (size_t i = 0; i < tiles.size(); i++){
            for (size_t j = i+1; j < tiles.size(); j++){

                auto [id1, surface1] = tiles[i];
                auto [id2, surface2] = tiles[j];

                Point_3 source = surface1->GetCentroid(id1);
                Point_3 target = surface2->GetCentroid(id2);

                Vector_3 normal1 = surface1->plane.orthogonal_vector();
                Vector_3 normal2 = surface2->plane.orthogonal_vector();

                source = source + offset*normal1;
                target = target + offset*normal2;

                // Create ray
                Ray ray;

                ray.ray.org_x = source.x();
                ray.ray.org_y = source.y();
                ray.ray.org_z = source.z();

                Vector_3 dir = target - source;
                dir = dir / CGAL::sqrt(dir.squared_length());

                ray.ray.dir_x = dir.x();
                ray.ray.dir_y = dir.y();
                ray.ray.dir_z = dir.z();

                ray.ray.tnear = 0.0f;
                ray.ray.tfar = std::sqrt(CGAL::squared_distance(source, target));

                ray.hit.geomID = RTC_INVALID_GEOMETRY_ID; // Geometry ID, if the ray hits something, this will be the id of the geometry

                ray.ray.id = id2; // Target id, if the ray does not hit anything, this will be the id of the target

                size_t idx = i*(2*tiles.size()-i-1)/2 + j-i-1;
                rays[idx] = std::make_pair(ray, &tiles[i]);
                
            }
        }
        creating_rays_bar.stop();

        
        // Instatiate the context
        RTCIntersectContext context;
        rtcInitIntersectContext(&context);


        // Intersect all rays with the scene
        auto rays_bar = util::progress_bar(rays.size(), "Intersecting rays");
        //rtcIntersect1M(scene, &context, (RTCRayHit*)&rays[0], rays.size(), sizeof(std::pair<Ray, Tile*>));
        #pragma omp parallel for
        for (size_t i = 0; i < rays.size(); i++){
            rtcIntersect1(scene, &context, (RTCRayHit*)&rays[i].first);
            rays_bar.update();
        }
        rays_bar.stop();



        
        // Release the scene and device
        rtcReleaseScene(scene);
        rtcReleaseDevice(device);


        std::unordered_map<size_t, unsigned int> id_to_int;
        std::unordered_map<unsigned int, size_t> int_to_id;

        size_t spm_idx = 0;
        for (auto & [id,_]:   tiles){
                id_to_int[id] = spm_idx;
                int_to_id[spm_idx] = id;
                spm_idx++;
        }


        // Initialize the matrix
        SpMat matrix = SpMat(point_map.size(), point_map.size());
        //auto matrix = Eigen::MatrixX<FT>::Zero(point_map.size(), point_map.size());
        matrix += Eigen::VectorXd::Ones(matrix.cols()).template cast<FT>().asDiagonal();


        cv::Mat img = cv::Mat::zeros(point_map.size(), point_map.size(), CV_8UC3);

        auto matrix_bar = util::progress_bar(rays.size(), "Creating matrix");
        for (size_t i = 0; i < rays.size(); i++){
            matrix_bar.update();

            auto & [ray, tile] = rays[i];

            // If we did not hit anything, there is a clear line of sight between the two points
            double value = (ray.hit.geomID == RTC_INVALID_GEOMETRY_ID) ? 1 : 0;
            if (value == 0)
                continue;
            
            int source_int = id_to_int[tile->first];
            int target_int = id_to_int[ray.ray.id];

            // Increment the matrix
            matrix.coeffRef(source_int, target_int) = value;
            matrix.coeffRef(target_int, source_int) = value;

            img.at<cv::Vec3b>(source_int, target_int) = cv::Vec3b(255, 255, 255);
            img.at<cv::Vec3b>(target_int, source_int) = cv::Vec3b(255, 255, 255);

        }
        matrix_bar.stop();

        cv::imwrite("matrix_bw.png", img); 



        // Markov Clustering
        //// 2.5 < infaltion < 2.8  => 3.5
        auto clustering_bar = util::progress_bar(1, "Markov Clustering");
        matrix = markov_clustering::run_mcl(matrix, 5, 2);
        std::set<std::vector<size_t>> mc_clusters = markov_clustering::get_clusters(matrix);
        clustering_bar.stop();
        std::printf("Number of clusters after Markov Clustering: %d\n", (int)mc_clusters.size());



        // Assigne the clusters to the point cloud
        u_int8_t cluster_index = 1;
        for (const std::vector<size_t> & cluster: mc_clusters){
            auto color = get_color_forom_angle(sample_circle(cluster_index));
            for (const size_t & idx: cluster){
                for (size_t i = 0; i < img.rows; i++)
                    if (img.at<cv::Vec3b>(idx, i)[0] > 0)
                        img.at<cv::Vec3b>(idx, i) = cv::Vec3b(color.b*255, color.g*255, color.r*255);
                img.at<cv::Vec3b>(idx, idx) = cv::Vec3b(color.b*255, color.g*255, color.r*255);
                

                for(const auto & index : point_map[int_to_id[(int)idx]])
                    cloud->points[index].instance = cluster_index;   
            }
            cluster_index += 1;
        }
        std::cout << "Number of clusters after Markov Clustering: " << (int)cluster_index-1 << std::endl;

        cv::imwrite("matrix_color.png", img);

        cloud->display("cloud");

        return *cloud;
    }
} // namespace linkml
