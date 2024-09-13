#include <types/PointCloud.hh>
#include <types/PointClouds.hh>
#include <types/Accumulators.hh>

#include <functions/progress_bar.hh>

#include <pcl/octree/octree.h>
#include <pcl/gpu/octree/octree.hpp>
#include <pcl/io/pcd_io.h>

#include <functions/progress_bar.hh>
#include "functions/downsample.hh"

#include <filesystem>
#include <vector>

#include <fmt/printf.h>
#include <omp.h>


// using fs = std::filesystem;

namespace linkml{

    template <typename T>
    T merge( T left, T right);

    template <>
    std::string merge( std::string left, std::string right){

        PointCloud cloud_1, cloud_2;

        cloud_1 = PointCloud::load(left);
        cloud_2 = PointCloud::load(right);
        

        cloud_1->reserve(cloud_1->size() + cloud_2->size());
        *cloud_1 += *cloud_2;
        downsample(cloud_1, 0.01);

        cloud_1.save(left);
        std::filesystem::remove(right);
        return left;

    }

    template <>
    PointCloud merge( PointCloud left, PointCloud right){

        (*left).reserve((*left).size() + (*right).size());
        *left += *right;

        return left;
    }



    // Custom reduction function for PointCloud::Ptr
    inline void merge_clouds(PointCloud lhs, const PointCloud rhs) {
        *lhs += *rhs;
    }


    #pragma omp declare reduction(+ : PointCloud : merge_clouds(omp_out, omp_in)) \
        initializer(omp_priv = PointCloud())



    template <typename T>
    PointCloud PointClouds<T>::merge(){

        static const size_t GROUP_SIZE = 250;

        if (data.size()==0)
            throw std::runtime_error("No point clouds to merge");

        PointCloud cloud;

        auto merge_bar = util::progress_bar(data.size()-1, "Merging");
        if constexpr (std::is_same<T, PointCloud>::value){
            if (data.size() == 1)
                return data[0];

            #pragma omp parallel for reduction(+:cloud)
            for (std::size_t i = 0; i < data.size(); ++i){
                *cloud += *data[i];
                //downsample(cloud, 0.01);
                merge_bar.update();
            }
        }
        else if constexpr (std::is_same<T, std::string>::value){

            size_t loop_count = 0;


            while (data.size() != 1){

                std::cout << "Loop: " << loop_count << ", #Elements: " << data.size() << std::endl;


                std::vector<std::string> new_paths(data.size(), "");

                size_t threads =  (data.size() * GROUP_SIZE > (size_t)omp_get_max_threads() )? (size_t)omp_get_max_threads():1;
                omp_set_num_threads(threads);

                #pragma omp parallel for shared(data, new_paths)
                for (std::size_t i = 0; i < data.size(); i+=GROUP_SIZE){

                    PointCloud merged = PointCloud();

                    //#pragma omp parallel for reduction(+:merged)
                    for (std::size_t j = 0; j < GROUP_SIZE; ++j)
                        if (i+j < data.size())
                            *merged += *PointCloud::load(data[i+j]);

                    merged.save(data[i]);
                    new_paths[i] = data[i];

                    //#pragma omp parallel for reduction(+:merged)
                    for (std::size_t j = 1 /*Don't remove the file we just saved*/; j < GROUP_SIZE; ++j){
                        if (i+j < data.size()){
                            std::filesystem::remove(data[i+j]);
                            auto header = std::filesystem::path(data[i+j]).replace_extension(".head");
                            if (std::filesystem::exists(header))
                                std::filesystem::remove(header);
                        }
                    }
                }


                int total_before = data.size();
                data.clear();
                std::copy_if(new_paths.begin(), new_paths.end(), std::back_inserter(data), [](std::string path){return path != "";});
                int diff = total_before - data.size();

                std::cout << "Merging done! Total before: " << total_before << ", Total after: " << data.size() << ", Diff: " << diff << std::endl;

                // Downsample
                auto downsample_bar = util::progress_bar(data.size(), "Downsampling");
                static const size_t MAX_THREADS = 10;
                size_t reduction = loop_count * 5;

                threads = (reduction < MAX_THREADS)? MAX_THREADS-reduction:1;
                omp_set_num_threads(threads);
                #pragma omp parallel for
                for (std::size_t i = 0; i < data.size(); ++i){
                    auto cloud = PointCloud::load(data[i]);
                    downsample(cloud, 0.01);
                    cloud.save(data[i]);
                    downsample_bar.update();
                }
                downsample_bar.stop();


                
                merge_bar.update(diff);

                loop_count++;
            }

            cloud = PointCloud::load(data[0]);
        }
        merge_bar.stop();

        return cloud;
        
    }

    template PointCloud PointCloudsInMemory::merge();
    template PointCloud PointCloudsOnDisk::merge();


}