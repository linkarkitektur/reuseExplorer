#include <types/PointCloud.hh>
#include <functions/region_growing.hh>
#include <functions/progress_bar.hh>
#include <functions/polyscope.hh>
#include <functions/fit_plane_thorugh_points.hh>

#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/planar_region.h>
#include <pcl/common/centroid.h>

#include <types/Plane.hh>
#include <types/PointCloud.hh>

#include <algorithms/refine.hh>


#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/geometry.h>

#include <Eigen/Core>
#include <typed-geometry/tg.hh>

#include <tbb/parallel_sort.h>
#include <polyscope/polyscope.h>


// Custom reduction function for std::unordered_set
void merge_sets(std::unordered_set<int>& lhs, const std::unordered_set<int>& rhs) {
    for (auto& element : rhs) {
        lhs.insert(element);
    }
}

#pragma omp declare reduction(merge : std::unordered_set<int> : merge_sets(omp_out, omp_in)) \
    initializer(omp_priv = std::unordered_set<int>())

linkml::PointCloud linkml::PointCloud::region_growing(
    int minClusterSize, 
    int numberOfNeighbours, 
    float smoothnessThreshold, 
    float curvatureThreshold
    ){
        //cloud::PointCloud, 
        //n::Integer; 
        //max_tries_per_region = 100, 
        //min_size = 100,  
        //α = 5°, 
        //ϵ = 0.1, 
        float interval_0 = 16; 
        float interval_factor = 1.5;

        PointCloud::Ptr cloud = this->makeShared();

        auto clusters = std::vector<std::unordered_set<int>>();
        auto plane_origins = std::vector<Eigen::Vector3f>();
        auto plane_normals = std::vector<Eigen::Vector3f>();


        // Reverse sort points by curvature
        pcl::Indices indices;
        indices.resize(cloud->size());
        std::iota(indices.begin(), indices.end(), 0);
        tbb::parallel_sort(indices.begin(), indices.end(), [&](int i, int j){return cloud->at(i).curvature < cloud->at(j).curvature;});

        std::unordered_map<int, int> index_map;
        for (size_t i = 0; i < indices.size(); ++i)
            index_map[indices[i]] = i;
        

        // Create KDTree
        pcl::KdTreeFLANN<PointCloud::PointType> tree;
        tree.setInputCloud(cloud); // Takes some time


        // Parameters for update
        float next_update; 
        size_t total = indices.size();


        // TODO: Remove when done
        polyscope::myinit();
        tg::aabb3 aabb =  cloud->get_bbox();
        auto pcd = polyscope::registerPointCloud("cloud", cloud->points);
        std::vector<float> vals(cloud->size(), 0);
        auto color = pcd->addScalarQuantity("plane",vals);
        
        
        
        auto progress = util::progress_bar(indices.size(), "Region Growing");
        // FIXME: Chage 0.3 to a parameter
        int plane_idx = 1;
        while (indices.size() > 0 && indices.size() > total * 0.3){ 

            // Select seed
            int seed = indices.back();
            indices.pop_back();


            // Region front
            std::vector<int> front;
            front.push_back(seed);


            // Cluster indices, indecies of points in the point cloud that are part of the cluster
            std::unordered_set<int> cluster_indices;
            cluster_indices.insert(seed);


            // Plane
            Eigen::Vector3f plane_normal = cloud->at(seed).getNormalVector3fMap();
            Eigen::Vector3f plane_origin = cloud->at(seed).getVector3fMap();
            Plane p = Plane(
                plane_normal.x(), 
                plane_normal.y(), 
                plane_normal.z(), 
                0, 
                plane_origin.x(), 
                plane_origin.y(), 
                plane_origin.z());


            next_update = interval_0;


            while (front.size() > 0){

                auto results = std::unordered_set<int>();
                #pragma omp parallel for reduction(merge:results)
                for (size_t i = 0; i < front.size(); ++i){
                    int current = front[i];

                    std::vector<float> nn_dists;
                    std::vector<int> nn_indices;

                    // FIXME: Change radius
                    tree.radiusSearch(current, 0.1, nn_indices, nn_dists);

                    auto plane_dist = std::vector<float>(nn_indices.size());
                    auto angles = std::vector<float>(nn_indices.size());

                    #pragma omp parallel for
                    for (size_t i = 0; i < nn_indices.size(); ++i){
                
                        // Compute distance to plane                        
                        // Project point on plane
                        Eigen::Vector3f p;
                        pcl::geometry::project(cloud->at(nn_indices[i]).getVector3fMap(), plane_origin, plane_normal, p);
                        // Point to point distance
                        Eigen::Vector3f diff = cloud->at(nn_indices[i]).getVector3fMap() - p;
                        plane_dist[i] = diff.norm();


                        // Compute angle to plane
                        auto point_normal = cloud->at(nn_indices[i]).getNormalVector3fMap();
                        angles[i] = std::abs(plane_normal.dot(point_normal));
                    }

                    // Filter points
                    for (size_t i = 0; i < nn_indices.size(); ++i){

                        // FIXME: Change distance
                        if (plane_dist[i] > 0.1) 
                            continue;
                        // FIXME: Change angle
                        if (angles[i] < 0.96592583 /*tg::cos(tg::angle::from_degree(25).radians())*/ )
                            continue;

                        results.insert(nn_indices[i]);
                    }
                
                }


                // Update front 
                // TODO: Maybe use a second set and parallelize
                front.clear();
                for (auto i : results){
                    if (cluster_indices.find(i) != cluster_indices.end())
                        continue;

                    // TODO: Use the return value to check if the index needs to be inserted in to the front
                    cluster_indices.insert(i); 
                    front.push_back(i);
                }

                // Update plane
                if (cluster_indices.size() > next_update){
                    pcl::Indices indices_local(cluster_indices.begin(), cluster_indices.end());

                    // TODO: Replace with pcl native implementation
                    // This should make it easier to port the code
                    /*Plane*/p = fit_plane_thorugh_points(cloud, indices_local);

                    plane_normal = {p.normal.x, p.normal.y, p.normal.z};
                    plane_origin = {p.origin.x, p.origin.y, p.origin.z};

                    next_update *= interval_factor;
                }
                
                // Update Color
                color->values.ensureHostBufferAllocated();
                for (auto i : cluster_indices)
                    color->values.data[i] = plane_idx;
                color->values.markHostBufferUpdated();
                color->setMapRange({0, plane_idx});

                
                polyscope::display(p, aabb, "plane");
                polyscope::frameTick();
            }



            // Check cluster size
            if (cluster_indices.size() < minClusterSize){
                for (auto i : cluster_indices)
                    color->values.data[i] = 0;
                progress.update(1);
                continue;
            }

            // Update Color
            for (auto i : cluster_indices)
                color->values.data[i] = plane_idx;
            

            //polyscope::frameTick();

            // Remove cluster from indices
            // TODO: See if parralelization is possible
            for (auto i : cluster_indices){
                indices[index_map[i]] = -1;
            }

            // Remove -1 from indices (Compactify indices) and update map
            index_map.clear();
            size_t count = 0;
            for (size_t i = 0; i < indices.size(); ++i){
                if (indices.at(i) == -1)
                    continue;
                indices[count] = indices.at(i);
                index_map[indices[count]] = count;
                count++;
            }
            indices.resize(count);

            // Add cluster to results
            clusters.push_back(cluster_indices);
            plane_origins.push_back(plane_origin);
            plane_normals.push_back(plane_normal);

            progress.update(cluster_indices.size());

            polyscope::show();
            plane_idx++;
        }
        progress.stop();
        
        // Color Point Cloud with cluster colors
        #pragma omp parallel for
        for (size_t i = 0; i < clusters.size(); ++i){
            for (auto idx: clusters[i]){
                cloud->at(idx).label = i+1;
            }
        }

        


        //// Display planes
        //for (size_t i = 0; i < clusters.size(); ++i){
        //    Eigen::Vector3f p;
        //    Eigen::Vector3f origin = {0,0,0};
        //    pcl::geometry::project(origin, plane_origins[i], plane_normals[i], p);
        //    // Point to point distance
        //    Eigen::Vector3f diff = origin - p;
        //    float dist = diff.norm();
        //    Plane plane = Plane(
        //        plane_normals[i].x(), 
        //        plane_normals[i].y(), 
        //        plane_normals[i].z(), 
        //        -dist, 
        //        plane_origins[i].x(), 
        //        plane_origins[i].y(), 
        //        plane_origins[i].z());
        //    polyscope::display(plane, aabb, "plane_" + std::to_string(i));
        //}


        polyscope::myshow();


        return *this;


}
