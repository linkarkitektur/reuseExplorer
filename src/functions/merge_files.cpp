#include <functions/merge_files.hh>

#include <types/point_cloud.hh>
#include <types/accumulators.hh>

#include <functions/progress_bar.hh>

#include <pcl/octree/octree.h>
#include <pcl/gpu/octree/octree.hpp>
#include <pcl/io/pcd_io.h>


#include <filesystem>
#include <vector>


// using fs = std::filesystem;

namespace linkml{

    static size_t get_total_size(std::vector<PointCloud::Ptr, Eigen::aligned_allocator<PointCloud::Ptr>> const& clouds){
        size_t total_size = 0;
        for (auto const& cloud : clouds){
            total_size += cloud->size();
        }
        return total_size;
    }
    

    template <typename PointCloud>
    static std::vector<typename PointCloud::Ptr, Eigen::aligned_allocator<typename  PointCloud::Ptr>> load(std::vector<std::string> const & files){
        std::vector<typename PointCloud::Ptr, Eigen::aligned_allocator<typename PointCloud::Ptr>> clouds;
        clouds.reserve(files.size());
        for (auto const& file : files){
            typename  PointCloud::Ptr cloud (new PointCloud);
            pcl::io::loadPCDFile<PointCloud::PointType> (file, *cloud);
            clouds.push_back(cloud);
        }
        return clouds;
    }

    template <typename PointCloud>
    static typename  PointCloud::Ptr merge(std::vector< typename  PointCloud::Ptr, Eigen::aligned_allocator< typename PointCloud::Ptr>> const & clouds){
        typename PointCloud::Ptr merged_cloud (new PointCloud);
        merged_cloud->reserve(get_total_size(clouds));
        for (auto const& cloud : clouds)
            *merged_cloud += *cloud;

        return merged_cloud;

    }


    template <typename PointCloud>
    static typename  PointCloud::Ptr downsample( typename  PointCloud::Ptr cloud){
        pcl::octree::OctreePointCloudPointVector<typename PointCloud::PointType> octree(0.02);
        typename  PointCloud::Ptr filtered_cloud (new PointCloud);
        octree.setInputCloud(cloud);
        octree.addPointsFromInputCloud();
        filtered_cloud->resize(octree.getLeafCount());


        std::vector<typename pcl::octree::OctreePointCloudPointVector<typename PointCloud::PointType>::LeafNodeIterator> nodes;
        for (auto it = octree.leaf_depth_begin(), it_end = octree.leaf_depth_end(); it != it_end; ++it)
            nodes.push_back(it);

        
        #pragma omp parallel for shared(octree, filtered_cloud, nodes, cloud)
        for (size_t i = 0; i < octree.getLeafCount(); i++){
            pcl::octree::OctreeContainerPointIndices& container = nodes[i].getLeafContainer();

            pcl::Indices indexVector;
            container.getPointIndices(indexVector);

            typename Accumulators<typename PointCloud::PointType>::type acc;

            for (auto const& index : indexVector){
                typename PointCloud::PointType point = cloud->at(index);
                boost::fusion::for_each (acc, pcl::detail::AddPoint<typename  PointCloud::PointType> (point));
            }

            boost::fusion::for_each (acc, pcl::detail::GetPoint< typename PointCloud::PointType> (filtered_cloud->at(i), indexVector.size()));

        }

        return filtered_cloud;
    }

    PointCloud::Ptr merge_files(const std::string& input_dir, const std::string& output_file, int chunk_size){

        std::vector<std::string> files;

        //TODO: Find a way to speed up this process.
        // Currently the bottleneck is waiting for octree.
        // pentential solution is to run octree in parallel.
        // Consider using openMP tasks.


        std::transform(
            std::filesystem::directory_iterator(input_dir), 
            std::filesystem::directory_iterator(), std::back_inserter(files), 
            [](const auto& entry){return entry.path();});
        std::sort(files.begin(), files.end());


        int n_chucks = (int)std::ceil((double)files.size() / chunk_size);
        auto  filtered_chunks = std::vector<PointCloud::Ptr, Eigen::aligned_allocator<PointCloud::Ptr>>(n_chucks);


        auto bar = util::progress_bar(files.size(), "Merging Chunks files");
        for (int i = 0; i < n_chucks; i++){


            // Load clouds
            ///////////////////////////////////////////////////////////////////////////////
            auto  chunk_clouds = std::vector<PointCloud::Ptr, Eigen::aligned_allocator<PointCloud::Ptr>>();
            size_t chunk_size_ = std::min(chunk_size, (int)files.size() - i * chunk_size);

            chunk_clouds.resize(chunk_size_);
            #pragma omp parallel for shared(chunk_clouds, files)
            for (size_t j = 0; j < chunk_size_; j++){
                int index = i * chunk_size + j;
                chunk_clouds[j] = PointCloud::Ptr(new PointCloud);
                pcl::io::loadPCDFile<PointCloud::PointType> (files[index], *chunk_clouds[j]);
            }


            // Merge clouds
            ///////////////////////////////////////////////////////////////////////////////
            auto merged_chunks = merge<PointCloud>(chunk_clouds);
            
            // Downsampling
            ///////////////////////////////////////////////////////////////////////////////
            filtered_chunks[i] = downsample<PointCloud>(merged_chunks);


            bar.update(chunk_size_);
        }
        bar.stop();


        // Merge clouds
        ///////////////////////////////////////////////////////////////////////////////
        auto merged_cloud = merge<PointCloud>(filtered_chunks);

        // Downsampling
        ///////////////////////////////////////////////////////////////////////////////
        auto filtered_cloud = downsample<PointCloud>(merged_cloud);
        pcl::io::savePCDFileBinary(output_file, *filtered_cloud);

        return filtered_cloud;

    }
}