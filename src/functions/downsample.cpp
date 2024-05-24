#include "functions/downsample.hh"
#include <types/PointCloud.hh>
#include <types/Accumulators.hh>
#include <pcl/octree/octree.h>
#include <functions/progress_bar.hh>


namespace linkml
{
    void downsample(PointCloud::Ptr cloud, double leaf_size){

        pcl::octree::OctreePointCloudPointVector<PointCloud::PointType> octree(leaf_size);
        octree.setInputCloud(cloud);
        octree.addPointsFromInputCloud();


        std::vector<pcl::octree::OctreePointCloudPointVector<PointCloud::PointType>::LeafNodeIterator> nodes;
        for (auto it = octree.leaf_depth_begin(), it_end = octree.leaf_depth_end(); it != it_end; ++it)
            nodes.push_back(it);

        size_t leaf_count = octree.getLeafCount();
        PointCloud::Ptr filtered_cloud = PointCloud::Ptr(new PointCloud());
        filtered_cloud->resize(leaf_count);

        
        #pragma omp parallel for shared(filtered_cloud, nodes)
        for (size_t i = 0; i < leaf_count; i++){
            pcl::octree::OctreeContainerPointIndices& container = nodes[i].getLeafContainer();

            pcl::Indices indexVector;
            container.getPointIndices(indexVector);

            Accumulators<PointCloud::PointType>::type acc;

            for (auto const& index : indexVector){
                PointCloud::PointType point = cloud->at(index);
                boost::fusion::for_each (acc, pcl::detail::AddPoint<PointCloud::PointType> (point));
            }

            boost::fusion::for_each (acc, pcl::detail::GetPoint< PointCloud::PointType> (filtered_cloud->at(i), indexVector.size()));

        }

        std::swap(*cloud, *filtered_cloud);

    }
    
    PointCloud PointCloud::downsample(double leaf_size){

        // TODO: Add a check to see if we are running out of memory

        auto octree_bar = util::progress_bar(1, "Building Octree");
        pcl::octree::OctreePointCloudPointVector<PointCloud::PointType> octree(leaf_size);
        octree.setInputCloud(this->makeShared());
        octree.addPointsFromInputCloud();
        octree_bar.stop();



        auto nodes_bar = util::progress_bar(1, "Getting Nodes");
        std::vector<pcl::octree::OctreePointCloudPointVector<PointCloud::PointType>::LeafNodeIterator> nodes;
        for (auto it = octree.leaf_depth_begin(), it_end = octree.leaf_depth_end(); it != it_end; ++it)
            nodes.push_back(it);
        nodes_bar.stop();

        size_t leaf_count = octree.getLeafCount();
        //PointCloud filtered_cloud = PointCloud();
        PointCloud::Ptr filtered_cloud = PointCloud::Ptr(new PointCloud());
        filtered_cloud->resize(leaf_count);


        
        auto downsample_bar = util::progress_bar(leaf_count, "Downsampling");
        #pragma omp parallel for shared(filtered_cloud, nodes)
        for (size_t i = 0; i < leaf_count; i++){
            pcl::octree::OctreeContainerPointIndices& container = nodes[i].getLeafContainer();

            pcl::Indices indexVector;
            container.getPointIndices(indexVector);

            Accumulators<PointCloud::PointType>::type acc;

            for (auto const& index : indexVector){
                PointCloud::PointType point = this->at(index);
                boost::fusion::for_each (acc, pcl::detail::AddPoint<PointCloud::PointType> (point));
            }

            boost::fusion::for_each (acc, pcl::detail::GetPoint< PointCloud::PointType> (filtered_cloud->at(i), indexVector.size()));
            downsample_bar.update();

        }
        downsample_bar.stop();


        return *filtered_cloud;

    }
} // namespace linkml
