#include "functions/downsample.hh"
#include "functions/progress_bar.hh"
#include "types/PointCloud.hh"
#include "types/Accumulators.hh"

#include <pcl/point_types.h>
#include <pcl/octree/octree.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/types.h>



namespace linkml
{
    void downsample(PointCloud::Cloud::Ptr cloud, double leaf_size){


        // TODO: Try out differnet way to downsample the point cloud.
        // Looking for a way that is more memory efficient.
        // Another approach woud be to make a grid and then use the point in the grid to do a look up in the cloud and then merge those values.
        // https://stackoverflow.com/questions/49819248/which-pcl-filter-to-use-to-downsample-a-point-cloud
        // This has potential to reduce the memory usage.


        // using PointT = PointCloud::Cloud::PointType;
        // using GridPoint = pcl::PointXYZ;
        // using Grid = pcl::PointCloud<GridPoint>;

        // // Create the look up grid
        // Grid::Ptr grid (new Grid);
        // pcl::copyPointCloud(*cloud, *grid);
        // pcl::VoxelGrid<GridPoint> sor;
        // sor.setInputCloud(grid);
        // sor.setLeafSize(leaf_size, leaf_size, leaf_size);
        // sor.filter(*grid);

        // // Create KD-Tree
        // pcl::KdTreeFLANN<PointT>::Ptr tree (new pcl::KdTreeFLANN<PointT>);
        // tree->setInputCloud(cloud);

        // // Create the filtered cloud
        // PointCloud::Cloud::Ptr filtered_cloud (new PointCloud::Cloud);
        // filtered_cloud->resize(grid->size());


        // // Downsample the point cloud
        // Accumulators<PointCloud::Cloud::PointType>::type acc;
        // pcl::PointXYZ gp;
        // PointT searchPoint;
        // pcl::Indices pointIdxNKNSearch = pcl::Indices();
        // std::vector<float> pointNKNSquaredDistance = std::vector<float>();
        
        // #pragma omp parallel for private(searchPoint, gp) shared(grid, cloud, filtered_cloud) firstprivate(acc, pointIdxNKNSearch, pointNKNSquaredDistance)
        // for (size_t i = 0; i < grid->size(); i++){

        //     gp =  grid->at(i);
        //     searchPoint = PointT(gp.x, gp.y, gp.z);

        //     if (tree->radiusSearch(searchPoint, leaf_size, pointIdxNKNSearch, pointNKNSquaredDistance) > 0){

        //         for (auto const index : pointIdxNKNSearch){
        //             PointCloud::Cloud::PointType point = cloud->at(index);
        //             boost::fusion::for_each (acc, pcl::detail::AddPoint<PointCloud::Cloud::PointType> (point));
        //         }

        //         boost::fusion::for_each (acc, pcl::detail::GetPoint< PointCloud::Cloud::PointType> (filtered_cloud->at(i), pointIdxNKNSearch.size()));
        //     }
        // }

        // std::swap(*cloud, *filtered_cloud);

        pcl::octree::OctreePointCloudPointVector<PointCloud::Cloud::PointType> octree(leaf_size);
        octree.setInputCloud(cloud);
        octree.addPointsFromInputCloud();


        std::vector<pcl::octree::OctreePointCloudPointVector<PointCloud::Cloud::PointType>::LeafNodeIterator> nodes;
        for (auto it = octree.leaf_depth_begin(), it_end = octree.leaf_depth_end(); it != it_end; ++it)
            nodes.push_back(it);

        size_t leaf_count = octree.getLeafCount();
        PointCloud filtered_cloud = PointCloud();
        filtered_cloud->resize(leaf_count);

        
        #pragma omp parallel for shared(filtered_cloud, nodes)
        for (size_t i = 0; i < leaf_count; i++){
            pcl::octree::OctreeContainerPointIndices& container = nodes[i].getLeafContainer();

            pcl::Indices indexVector;
            container.getPointIndices(indexVector);

            Accumulators<PointCloud::Cloud::PointType>::type acc;

            for (auto const& index : indexVector){
                PointCloud::Cloud::PointType point = cloud->at(index);
                boost::fusion::for_each (acc, pcl::detail::AddPoint<PointCloud::Cloud::PointType> (point));
            }

            boost::fusion::for_each (acc, pcl::detail::GetPoint< PointCloud::Cloud::PointType> (filtered_cloud->at(i), indexVector.size()));

        }

        std::swap(*cloud, *filtered_cloud);
    }
    

} // namespace linkml
