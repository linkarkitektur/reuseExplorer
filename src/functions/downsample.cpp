#include "functions/downsample.hh"
#include <types/PointCloud.hh>
#include <types/Accumulators.hh>
#include <pcl/octree/octree.h>
#include <functions/progress_bar.hh>


namespace linkml
{
    void downsample(PointCloud::Cloud::Ptr cloud, double leaf_size){

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
    
    void PointCloud::downsample(double leaf_size){

        auto octree_bar = util::progress_bar(1, "Building Octree");
        pcl::octree::OctreePointCloudPointVector<PointCloud::Cloud::PointType> octree(leaf_size);
        octree.setInputCloud(*this);
        octree.addPointsFromInputCloud();
        octree_bar.stop();



        auto nodes_bar = util::progress_bar(1, "Getting Nodes");
        std::vector<pcl::octree::OctreePointCloudPointVector<PointCloud::Cloud::PointType>::LeafNodeIterator> nodes;
        for (auto it = octree.leaf_depth_begin(), it_end = octree.leaf_depth_end(); it != it_end; ++it)
            nodes.push_back(it);
        nodes_bar.stop();

        size_t leaf_count = octree.getLeafCount();
        //PointCloud filtered_cloud = PointCloud();
        //filtered_cloud->resize(leaf_count);
        (*this)->resize(leaf_count);


        
        auto downsample_bar = util::progress_bar(leaf_count, "Downsampling");
        #pragma omp parallel for shared(nodes)
        for (size_t i = 0; i < leaf_count; i++){
            pcl::octree::OctreeContainerPointIndices& container = nodes[i].getLeafContainer();

            pcl::Indices indexVector;
            container.getPointIndices(indexVector);

            Accumulators<PointCloud::Cloud::PointType>::type acc;

            for (auto const& index : indexVector){
                PointCloud::Cloud::PointType point = (*this)->at(index);
                boost::fusion::for_each (acc, pcl::detail::AddPoint<PointCloud::Cloud::PointType> (point));
            }

            boost::fusion::for_each (acc, pcl::detail::GetPoint< PointCloud::Cloud::PointType> ((*this)->at(i), indexVector.size()));
            downsample_bar.update();

        }
        downsample_bar.stop();
        
    }
} // namespace linkml
