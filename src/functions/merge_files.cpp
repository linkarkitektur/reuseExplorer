#include <types/PointCloud.hh>
#include <types/PointClouds.hh>
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

    template <typename PointCloud>
    static typename  PointCloud::Ptr merge(typename  PointCloud::Ptr left, typename PointCloud::Ptr right){
        left->reserve(left->size() + right->size());
        *left += *right;
        return left;
    }

    template <typename T>
    PointCloud::Ptr merge_files(typename PointClouds<T>::iterator begin, typename PointClouds<T>::iterator end){

        // If there is only one file, just load it and return
        if (std::distance(begin, end) == 1){

            // If the underlying type is a string, load the file
            if constexpr (std::is_same<T, std::string>::value){
                PointCloud::Ptr cloud (new PointCloud);

                cloud = PointCloud::load(*begin);
                return cloud;
            }

            // Oterwise it is assumed to be a PointCloud::Ptr
            else {
                return *begin;
            }
        }

        // If ther are more files split the list and merge them separately
        PointCloud::Ptr left, right;
        auto middle = begin + std::distance(begin, end) / 2;

        #pragma omp task shared(left)
        left = merge_files<T>(begin, middle);
        #pragma omp task shared(right)
        right = merge_files<T>(middle, end);

        // Merge clouds
        #pragma omp taskwait
        PointCloud::Ptr merged_cloud = merge<PointCloud>(left, right);
        return merged_cloud;

    }


    // TODO: Initialise the template rather the implementing the cases indeviually
    // See the annotioation mehtod for an example
    template <>
    PointCloud::Ptr PointCloudsOnDisk::merge(){

        PointCloud::Ptr cloud;
        #pragma omp parallel
        {
            #pragma omp single
                cloud = merge_files<std::string>(data.begin(), data.end());
        }
        return cloud;
    }

    template <>
    PointCloud::Ptr PointCloudsInMemory::merge(){

        PointCloud::Ptr cloud;
        #pragma omp parallel
        {
            #pragma omp single
                cloud = merge_files<PointCloud::Ptr>(data.begin(), data.end());
        }

        return cloud;
    }

}