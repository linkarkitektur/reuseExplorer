#include <functions/region_growing.hh>
#include <functions/progress_bar.hh>
#include <functions/polyscope.hh>
#include <functions/fit_plane_thorugh_points.hh>

#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/planar_region.h>
#include <pcl/common/centroid.h>

#include <types/plane.hh>
#include <types/point_cloud.hh>

#include <algorithms/refine.hh>



linkml::PointCloud::Ptr linkml::region_growing(
    linkml::PointCloud::Ptr cloud, 
    int minClusterSize, 
    int numberOfNeighbours, 
    float smoothnessThreshold, 
    float curvatureThreshold
    ){

        tg::aabb3 aabb = get_bbox(*cloud);

        // Region growing
        ///////////////////////////////////////////////////////////////////////////////
        auto progress = util::progress_bar(0,"Region growing");
        pcl::search::Search<PointCloud::PointType>::Ptr tree (new pcl::search::KdTree<PointCloud::PointType>);
        // pcl::search::KdTree<PointCloud::PointType>::Ptr tree (new pcl::search::KdTree<PointCloud::PointType>);
        pcl::RegionGrowing<PointCloud::PointType, PointCloud::PointType> reg;

        std::printf("minClusterSize: %d\n", minClusterSize);
        std::printf("numberOfNeighbours: %d\n", numberOfNeighbours);
        std::printf("smoothnessThreshold: %f\n", smoothnessThreshold);
        std::printf("curvatureThreshold: %f\n", curvatureThreshold);


        // Setting parameters
        reg.setSearchMethod (tree);
        reg.setMinClusterSize (minClusterSize); 
        reg.setNumberOfNeighbours (numberOfNeighbours);
        reg.setSmoothnessThreshold (smoothnessThreshold);
        reg.setCurvatureThreshold (curvatureThreshold);

        // Set input cloud
        reg.setInputCloud (cloud);
        reg.setInputNormals (cloud);

        // Extract the clusters
        std::vector <pcl::PointIndices> clusters;
        reg.extract (clusters);

        //TODO: Add fileter to remove small clusters


        std::printf("Number of clusters is equal to %d\n", clusters.size());

        //TODO: Check default parameters
        clusters = linkml::refine(cloud, clusters, linkml::refinement_parameters());

        std::printf("Number of clusters after refinement is equal to %d\n", clusters.size());


        //TODO: Add fileter to remove even more clusters ?


        polyscope::myinit();

        std::vector<linkml::Plane> planes;
        planes.resize(clusters.size());


        // Set labels
        #pragma omp parallel for
        for (size_t i = 0; i < clusters.size(); ++i){
            for (size_t j = 0; j < clusters[i].indices.size (); ++j){
                cloud->at(clusters[i].indices[j]).label = i;
            }

            planes[i] = linkml::fit_plane_thorugh_points(cloud, clusters[i]);

        }


        
        for (size_t i = 0; i < clusters.size(); ++i)
            polyscope::display(planes[i], aabb, "plane_" + std::to_string(i));

        
        progress.stop();

        polyscope::display(*cloud);
        polyscope::myshow();

        return cloud;

}
