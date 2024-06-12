#include <types/PointCloud.hh>
#include <functions/region_growing.hh>
#include <functions/progress_bar.hh>
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


// Custom reduction function for std::unordered_set
void merge_sets(std::unordered_set<int>& lhs, const std::unordered_set<int>& rhs) {
    for (auto& element : rhs) {
        lhs.insert(element);
    }
}

#pragma omp declare reduction(merge : std::unordered_set<int> : merge_sets(omp_out, omp_in)) \
    initializer(omp_priv = std::unordered_set<int>())

void linkml::PointCloud::region_growing(
    float angle_threshold,
    float plane_dist_threshold,
    int minClusterSize, 
    float early_stop,
    float radius,
    float interval_0, 
    float interval_factor
    ){

        PointCloud cloud = *this;

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
        pcl::KdTreeFLANN<PointCloud::Cloud::PointType> tree;
        tree.setInputCloud(cloud); // Takes some time


        // Parameters for update
        float next_update; 
        size_t total = indices.size();


        auto progress = util::progress_bar(total, "Region Growing");
        while (indices.size() > 0 && indices.size() > total * early_stop){ 

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



            next_update = interval_0;


            while (front.size() > 0){

                auto results = std::unordered_set<int>();
                #pragma omp parallel for reduction(merge:results)
                for (size_t i = 0; i < front.size(); ++i){
                    int current = front[i];

                    // Find neighbors
                    std::vector<float> nn_dists;
                    std::vector<int> nn_indices;

                    tree.radiusSearch(current, radius, nn_indices, nn_dists);

                    // Filter neighbors that are already part of other clusters
                    std::vector<int> temp;
                    temp.reserve(nn_indices.size());
                    // TODO: Find better way to filter
                    std::copy_if (nn_indices.begin(), nn_indices.end(), std::back_inserter(temp),
                        [&](int i){
                            return index_map.find(i) != index_map.end();
                        } );
                    std::swap(nn_indices, temp);

                    // Compute distance to plane and angle to plane
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

                        if (plane_dist[i] > plane_dist_threshold) 
                            continue;
                        if (angles[i] < angle_threshold)
                            continue;

                        results.insert(nn_indices[i]);
                    }
                
                }


                // Update front 
                // TODO: Maybe use a second set and parallelize
                front.clear();
                for (auto i : results){

                    //FIXME: This is beeing added to as set and will always be unique
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
                    Plane p = fit_plane_thorugh_points(cloud, indices_local);

                    plane_normal = {p.normal.x, p.normal.y, p.normal.z};
                    plane_origin = {p.origin.x, p.origin.y, p.origin.z};

                    next_update *= interval_factor;
                }
            }



            // Check cluster size
            if (cluster_indices.size() < minClusterSize){
                progress.update(1);
                continue;
            }

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

        }
        progress.stop();
        
        // Color Point Cloud with cluster colors
        #pragma omp parallel for
        for (size_t i = 0; i < clusters.size(); ++i){
            for (auto idx: clusters[i]){
                (*this)->at(idx).label = i+1;
            }
        }
}
