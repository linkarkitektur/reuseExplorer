#pragma once
#include "types/PointCloud.hh"
#include "types/PlanarPointSet.hh"
#include "algorithms/refine.hh"
#include "types/surface.hh"
#include "functions/progress_bar.hh"
#include "functions/color.hh"
#include "algorithms/mcl.hh"

#include <functions/polyscope.hh>

#include <pcl/filters/random_sample.h>
#include <pcl/filters/experimental/functor_filter.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/PointIndices.h>

#include <embree3/rtcore.h>

#include <opencv4/opencv2/opencv.hpp>
#include <omp.h>

#include <opencv4/opencv2/core/eigen.hpp>


#define UseIntersection 0
#define SaveMatrix 0


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

using Filter = pcl::experimental::advanced::FunctorFilter<linkml::PointCloud::Cloud::PointType, MatchCondition<linkml::PointCloud::Cloud::PointType>>;
using Clusters = std::vector<pcl::PointIndices>;
using Tiles = std::vector<std::pair<unsigned int, linkml::Surface *>>;
using Indices = pcl::Indices;

class FaceMap{
    class FaceIterator{
        public:
        FaceIterator(int idx) : index(idx){}

        FaceIterator & operator++(){
            index++;
            return *this;
        }
        bool operator==(const FaceIterator & rhs) const{
            return index == rhs.index;
        }
        bool operator!=(const FaceIterator & rhs) const{
            return index != rhs.index;
        }

       std::array<int, 4> operator*(){
            int v1 = index * offset;
            int v2 = index * offset + 1;
            int v3 = index * offset + 2;
            int v4 = index * offset + 3;
            return std::array<int, 4>{v1, v2, v3, v4};
        }
        private:
        constexpr static int offset = 4;
        size_t index = 0;
    };


    public:
    FaceMap(Tiles & tiles) : tiles(tiles){}

    FaceIterator begin() const {
        return FaceIterator(0);
    }
    FaceIterator end() const {
        return FaceIterator(tiles.size());
    }
    size_t size() const { return tiles.size();}
    private:
    Tiles & tiles;
};
class VertexMap{
    class VertexIterator{
        public:
        VertexIterator(Tiles & tiles, RTCScene & scene, size_t index) : tiles(tiles), scene(scene), index(index){}

        VertexIterator & operator++(){
            vertex++;
            if (vertex == offset){
                vertex = 0;
                index++;
            }
            return *this;
        }
        bool operator==(const VertexIterator & rhs) const{
            return index == rhs.index;
        }
        bool operator!=(const VertexIterator & rhs) const{
            return index != rhs.index;
        }

        std::array<float,3> operator*(){
            auto & [id, surface] = tiles[index];
            RTCGeometry geo = rtcGetGeometry(scene, id);
            float * points = (float*)rtcGetGeometryBufferData(geo, RTC_BUFFER_TYPE_VERTEX, 0);
            return std::array<float, 3>{points[vertex*3], points[vertex*3+1], points[vertex*3+2]};
        }
        private:
        Tiles & tiles;
        RTCScene & scene;
        size_t index = 0;
        size_t vertex = 0;
        constexpr static int offset = 4;
    };


    public:
    VertexMap(Tiles & tiles, RTCScene & scene) : tiles(tiles), scene(scene){}

    VertexIterator begin() const {
        return VertexIterator(tiles, scene, 0);
    }
    VertexIterator end()const {
        return VertexIterator(tiles, scene, tiles.size());
    }
    size_t size() const { return tiles.size()*4; }
    private:
    Tiles & tiles;
    RTCScene & scene;
};
class FaceIDMap{
    class FaceIDIterator{
        public:
        FaceIDIterator(Tiles & tiles, size_t index) : tiles(tiles), index(index){}

        FaceIDIterator & operator++(){
            index++;
            return *this;
        }
        bool operator==(const FaceIDIterator & rhs) const{
            return index == rhs.index;
        }
        bool operator!=(const FaceIDIterator & rhs) const{
            return index != rhs.index;
        }

        int operator*(){
            auto & [id, surface] = tiles[index];
            int address = (int)(size_t)surface;
            return address;
        }
        private:
        Tiles & tiles;
        size_t index = 0;
    };

    public:
    FaceIDMap(Tiles & tiles) : tiles(tiles){}
    FaceIDIterator begin() const {
        return FaceIDIterator(tiles, 0);
    }
    FaceIDIterator end() const {
        return FaceIDIterator(tiles, tiles.size());
    }
    size_t size() const { return tiles.size();}
    private:
    Tiles & tiles;
};


static Clusters extract_clusters(linkml::PointCloud::Cloud::ConstPtr cloud){

    // Collect all cluster indices
    std::unordered_set<size_t> cluster_indices_set;
    for (size_t i = 0; i < cloud->points.size(); i++)
        cluster_indices_set.insert(cloud->points[i].label);

    cluster_indices_set.erase(-1);

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

namespace linkml
{
    
    template<typename PointT>
    std::vector<pcl::PointIndices::Ptr> cluster_rooms(
        typename pcl::PointCloud<PointT>::Ptr cloud,
        unsigned int downsample_size = 5000000,
        double sx = 0.4,
        double sy = 0.4,
        double expand_factor = 2, 
        double inflate_factor = 2, 
        double max_loop = 10.0, 
        double mult_factor = 1.0
    )
    {

        // Remove everyting that is not a surface
        auto match_filter = MatchCondition<PointCloud::Cloud::PointType>();
        Filter pass(match_filter);
        pass.setInputCloud(cloud);
        pass.filter(*cloud);


        pcl::RandomSample<PointCloud::Cloud::PointType> sampler;
        sampler.setInputCloud(cloud);
        sampler.setSample(downsample_size);
        sampler.setSeed(0);
        sampler.filter(*cloud);

        // Extract clusters
        Clusters clusters = extract_clusters(cloud);

        std::cout << "Number of planes: " << clusters.size() << std::endl;

        // Create surfaces
        std::vector<Surface> surfaces(clusters.size());
        auto surface_bar = util::progress_bar(surfaces.size(), "Creating surfaces");
#if 1
        for (size_t i = 0; i < clusters.size(); i++){
            surfaces[i] = Surface(cloud, clusters[i].indices, sx, sy);
            surface_bar.update();
        }
#else
        #pragma omp parallel
        {
            #pragma omp single
            {
                for (size_t i = 0; i < clusters.size(); i++){
                
                    #pragma omp task
                    {
                        surfaces[i] = Surface(cloud, clusters[i].indices);
                        surface_bar.update();
                    }
                }
            }
            #pragma omp taskwait
        }
#endif
        surface_bar.stop();




        // polyscope::myinit();
        // for (int i = 0; i< 202 /*surfaces.size()*/; i++)
        //     polyscope::display< linkml::Surface const& >(surfaces[i], "Surface " + std::to_string(i));
        // polyscope::myshow();
            




        // Ray tracing
        RTCDevice device = rtcNewDevice(NULL);

        // Create scene
        RTCScene scene   = rtcNewScene(device);
        rtcSetSceneFlags(scene, RTC_SCENE_FLAG_ROBUST);

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
        double const constexpr offset = 0.10;
        size_t const nrays = tiles.size()*(tiles.size()-1)/2;
        std::cout << "Number of rays: " << nrays << std::endl;
        std::cout << "Size of Ray: " << sizeof(std::pair<Ray, Tile *>) << std::endl;
        std::cout << "Total size of rays: " << sizeof(std::pair<Ray, Tile *>)*nrays << std::endl;
        std::cout << "Size of RTCRayHit"  << sizeof(RTCRayHit) << std::endl;
        std::cout << "Size of RTCRay" << sizeof(RTCRay) << std::endl;
        

        Rays rays(nrays);
        auto creating_rays_bar = util::progress_bar(nrays, "Creating rays");
        #pragma omp parallel for collapse(2)
        for (size_t i = 0; i < tiles.size(); i++){
            for (size_t j = i+1; j < tiles.size(); j++){

                auto [id1, surface1] = tiles[i];
                auto [id2, surface2] = tiles[j];

                Point_3 source = surface1->GetCentroid(id1);
                Point_3 target = surface2->GetCentroid(id2);

                auto normal1 = surface1->plane.orthogonal_vector();
                auto normal2 = surface2->plane.orthogonal_vector();

                source = source + FT(offset)*normal1;
                target = target + FT(offset)*normal2;

                // Create ray
                Ray ray;

                ray.ray.org_x = CGAL::to_double(source.x());
                ray.ray.org_y = CGAL::to_double(source.y());
                ray.ray.org_z = CGAL::to_double(source.z());

                Vector_3 dir = target - source;
                dir = dir / CGAL::sqrt(CGAL::to_double(dir.squared_length()));

                ray.ray.dir_x = CGAL::to_double(dir.x());
                ray.ray.dir_y = CGAL::to_double(dir.y());
                ray.ray.dir_z = CGAL::to_double(dir.z());

                ray.ray.tnear = 0.0f;
                ray.ray.tfar = CGAL::sqrt(CGAL::to_double(CGAL::squared_distance(source, target)));

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
        //FIXME: This is not realably parallel, sometimes all and other times a single thread is used.
        //rtcIntersect1M(scene, &context, (RTCRayHit*)&rays[0], rays.size(), sizeof(std::pair<Ray, Tile*>));
        #pragma omp parallel for
        for (size_t i = 0; i < rays.size(); i++){
#if UseIntersection
            rtcIntersect1(scene, &context, (RTCRayHit*)&rays[i].first);
#else
            rtcOccluded1(scene, &context, (RTCRay*)&rays[i].first.ray);
#endif
            rays_bar.update();
        }
        rays_bar.stop();


        //polyscope::myinit();
        //auto ps_tiles = polyscope::registerSurfaceMesh("tiles", VertexMap(tiles, scene ), FaceMap(tiles));
        //ps_tiles->addFaceScalarQuantity("id", FaceIDMap(tiles));
        
        // Release the scene and device
        rtcReleaseScene(scene);
        rtcReleaseDevice(device);


        // TODO Write out tab file.

        std::unordered_map<size_t, unsigned int> id_to_int;
        std::unordered_map<unsigned int, size_t> int_to_id;

        size_t spm_idx = 0;
        for (auto & [id,_]:   tiles){
                id_to_int[id] = spm_idx;
                int_to_id[spm_idx] = id;
                spm_idx++;
        }


        // Initialize the matrix
        Eigen::MatrixXd matrix = Eigen::MatrixXd::Zero(point_map.size(), point_map.size());


        // // Write output file
        // auto output_file_bar = util::progress_bar(1, "Writing output file");
        // std::ofstream file("./rays");
        // if (file.is_open()){

        //     for (size_t i = 0; i < rays.size(); i++){

        //         auto & [ray, tile] = rays[i];

        //         // If we did not hit anything, there is a clear line of sight between the two points
        //         const double value = (ray.hit.geomID == RTC_INVALID_GEOMETRY_ID) ? 1 : 0;
                
        //         if (value == 0)
        //             continue;
        
        //         int source_int = id_to_int[tile->first];
        //         int target_int = id_to_int[ray.ray.id];

        //         file << source_int << " " << target_int << "  " << value << std::endl;
        //     }
        //     file.close();
        // }
        // else 
        //     std::cerr << "Unable to open file" << std::endl;
        
        // output_file_bar.stop();


        auto matrix_bar = util::progress_bar(rays.size(), "Creating matrix");
        #pragma omp parallel for
        for (size_t i = 0; i < rays.size(); i++){

            auto & [ray, tile] = rays[i];

            // If we did not hit anything, there is a clear line of sight between the two points
#if UseIntersection
            const double value = (ray.hit.geomID == RTC_INVALID_GEOMETRY_ID) ? 1 : 0;
#else
            const double value = (ray.ray.tfar == -INFINITY) ? 0 : 1;
#endif

            // const auto  color = (ray.hit.geomID == RTC_INVALID_GEOMETRY_ID) ? cv::Vec3b(255, 255, 255) : cv::Vec3b(0, 0, 0);
            
            int source_int = id_to_int[tile->first];
            int target_int = id_to_int[ray.ray.id];

            // Increment the matrix
            matrix(source_int, target_int) = value;
            matrix(target_int, source_int) = value;


            matrix_bar.update();
        }
        matrix_bar.stop();


        //Check for blank (white) rows
        for (size_t i = 0; i < matrix.rows(); i++){
            bool all_white = true;
            for (size_t j = i+1; j < matrix.cols(); j++){
                if (matrix(i, j) < 0.5){
                    all_white = false;
                    break;
                }
            }
            if (all_white){
                for (size_t j = 0; j < matrix.cols(); j++){
                    matrix(i, j) = 0;
                    matrix(j, i) = 0;
                }
            }
        }


        
#if SaveMatrix
        cv::Mat img;
        cv::eigen2cv(matrix,img);
        cv::imwrite("matrix_rays.png", img*255);
        cv::Mat colorImage(matrix.rows(), matrix.cols(), CV_8UC3, cv::Scalar(0, 0, 0));

        #pragma omp parallel for collapse(2)
        for (size_t i = 0; i < matrix.rows(); i++){
            for (size_t j = 0; j < matrix.cols(); j++){
                if (matrix(i, j) > 0.000001){
                    colorImage.at<cv::Vec3b>(i, j) = cv::Vec3d(255, 255,255);
                }
            }
        }
#endif



        std::unordered_map<size_t, pcl::Indices> cluster_map{};

        // // Markov Clustering
        // //// 2.5 < infaltion < 2.8  => 3.5
        auto clustering_bar = util::progress_bar(1, "Markov Clustering");
        auto asign_cluster = [&](size_t cluster_j, size_t member_i){
            // Assigne the clusters to the point cloud
            // printf("Cluster %d: %d\n", cluster_j, member_i);
  
  #if SaveMatrix
            auto color = get_color_forom_angle(sample_circle(cluster_j));
            auto color_vec = cv::Vec3d(color.r * 255, color.g * 255, color.b * 255);
            for (size_t i = 0; i < matrix.rows(); i++){
                if (matrix(member_i, i) > 0.000001){
                    colorImage.at<cv::Vec3b>(member_i, i) = color_vec;
                    colorImage.at<cv::Vec3b>(i, member_i) = color_vec;
                }
            }
#endif

            #pragma omp parallel for
            for(const auto & index : point_map[int_to_id[(int)member_i]]){
                cloud->points[index].instance = cluster_j;
                #pragma omp critical
                cluster_map[cluster_j].push_back(index);
            }
        };
        auto mcl = mcl::mcl<mcl::MCLAlgorithm::CLI, Eigen::MatrixXd>(matrix, asign_cluster);
        mcl.cluster_mcl(expand_factor, inflate_factor, max_loop, mult_factor);

#if SaveMatrix
        cv::imwrite("matrix_mcl.png", colorImage);
#endif
        clustering_bar.stop();



        std::vector<pcl::PointIndices::Ptr> clusters2 = std::vector<pcl::PointIndices::Ptr>();
        for (auto & [_, indices]: cluster_map){
            pcl::PointIndices::Ptr cluster = pcl::PointIndices::Ptr(new pcl::PointIndices());
            cluster->indices = indices;
            clusters2.push_back(cluster);
        }
        return clusters2;


    }
} // namespace linkml
