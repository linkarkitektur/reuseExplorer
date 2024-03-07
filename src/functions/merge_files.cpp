#include <functions/merge_files.hh>
#include <functions/filter.hh>

#include <types/PointCloud.hh>
#include <types/Accumulators.hh>

#include <functions/progress_bar.hh>

#include <pcl/octree/octree.h>
#include <pcl/gpu/octree/octree.hpp>
#include <pcl/io/pcd_io.h>


#include <filesystem>
#include <vector>

#include <fmt/printf.h>


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
    static typename  PointCloud::Ptr merge(typename  PointCloud::Ptr left, typename PointCloud::Ptr right){
        left->reserve(left->size() + right->size());
        *left += *right;
        return left;

        // typename PointCloud::Ptr merged_cloud (new PointCloud);
        // merged_cloud->reserve(left->size() + right->size());
        // *merged_cloud += *left;
        // *merged_cloud += *right;
        // return merged_cloud;
    }


    template <typename PointCloud>
    static typename  PointCloud::Ptr downsample( typename  PointCloud::Ptr cloud){

        pcl::octree::OctreePointCloudPointVector<typename PointCloud::PointType> octree(0.02);
        octree.setInputCloud(cloud);
        octree.addPointsFromInputCloud();

        std::vector<typename pcl::octree::OctreePointCloudPointVector<typename PointCloud::PointType>::LeafNodeIterator> nodes;
        for (auto it = octree.leaf_depth_begin(), it_end = octree.leaf_depth_end(); it != it_end; ++it)
            nodes.push_back(it);

        typename  PointCloud::Ptr filtered_cloud (new PointCloud);
        filtered_cloud->resize(octree.getLeafCount());
        
        // #pragma omp parallel for shared(octree, filtered_cloud, nodes, cloud)
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

    PointCloud::Ptr merge_files(std::vector<std::string>::iterator begin, std::vector<std::string>::iterator end){

    
        // If there is only one file, just load it and return
        if (std::distance(begin, end) == 1){
            PointCloud::Ptr cloud (new PointCloud);
            cloud->load(*begin);
            return cloud;
        }

        // If ther are more files split the list and merge them separately
        PointCloud::Ptr left, right;
        auto middle = begin + std::distance(begin, end) / 2;

        #pragma omp task shared(left)
        left = merge_files(begin, middle);
        #pragma omp task shared(right)
        right = merge_files(middle, end);

        // Merge clouds
        #pragma omp taskwait
        PointCloud::Ptr merged_cloud = merge<PointCloud>(left, right);


        return merged_cloud;

        // if (merged_cloud->size() < 200'000)
        //     return merged_cloud;
        // return downsample<PointCloud>(merged_cloud);


    }

    PointCloud::Ptr merge_files(const std::string& input_dir, const std::string& output_file, int chunk_size){

        fmt::printf("Entering merging files function\n");
        std::vector<std::string> files;

        //TODO: Find a way to speed up this process.
        // Currently the bottleneck is waiting for octree.
        // pentential solution is to run octree in parallel.
        // Consider using openMP tasks.


        // Load and sort files
        fmt::printf("Loading file paths\n");
        std::transform(
            std::filesystem::directory_iterator(input_dir), 
            std::filesystem::directory_iterator(), std::back_inserter(files), 
            [](const auto& entry){return entry.path();});
        fmt::printf("Sorting file paths\n");
        std::sort(files.begin(), files.end());

        // Divide-and-conquer algorithm to merge files
        fmt::printf("Merging {} files\n", files.size());
        PointCloud::Ptr point_cloud;
        #pragma omp parallel
        {
            #pragma omp single
                point_cloud = merge_files(files.begin(), files.end());
        }

        fmt::printf("Saving merged cloud to {}\n", output_file);
        pcl::io::savePCDFileBinary(output_file, *point_cloud);


        point_cloud = filter(point_cloud);
        point_cloud = downsample<PointCloud>(point_cloud);


        fmt::printf("Return from function\n");
        return point_cloud;


    }
}