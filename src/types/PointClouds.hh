#pragma once
#include "PointCloud.hh"
#include "Dataset.hh"

#include <Eigen/Core>
#include <vector>
#include <valarray>
#include <filesystem>
#include <optional>

namespace linkml
{
    /// @brief A collection of point clouds.
    /// Point clouds can be a clollection of point cloud files, or in memory point clouds.
    /// @tparam T The type underlying type either `std::string` or `PointCloud`
    template <typename T>
    class PointClouds
    {
        public:
            using Ptr = pcl::shared_ptr<PointClouds<T>>;
            using ConstPtr = pcl::shared_ptr<const PointClouds<T>>;

            using DataT = std::vector<T, Eigen::aligned_allocator<T>>;
            using iterator = typename DataT::iterator;

        private:
            DataT data = DataT();

        public:

            PointClouds() = default;

            // Constructors
            PointClouds(std::vector<T> data) {
                data.resize(data.size());
                #pragma omp parallel for shared(data)
                for (std::size_t i = 0; i < data.size(); ++i){
                    this->data[i] = data[i];
                }
            };
            PointClouds(std::string directory) {

                if constexpr (std::is_same<T, std::string>::value){
                    for (const auto & entry : std::filesystem::directory_iterator(directory))
                        if (entry.is_regular_file() && entry.path().extension() == ".pcd")
                            data.push_back(entry.path());
                    std::sort(data.begin(), data.end());
                } else {
                    std::vector<std::string> files;
                    for (const auto & entry : std::filesystem::directory_iterator(directory))
                        if (entry.is_regular_file() && entry.path().extension() == ".pcd")
                            files.push_back(entry.path());
                    std::sort(files.begin(), files.end());

                    data.resize(files.size());

                    #pragma omp parallel for shared(files, data)
                    for (std::size_t i = 0; i < files.size(); ++i){
                        data[i] = PointCloud::load(files[i]);
                    }

                }
            };
            
            static PointClouds load(std::string directory){
                return PointClouds(directory);
            }

            // Modify the point clouds
            PointClouds filter( typename PointCloud::Cloud::PointType::LableT value = 2);
            PointClouds register_clouds();
            PointClouds annotate(std::string yolo_path, std::optional<Dataset> & dataset);


            // Return a point cloud or subset of point clouds
            PointCloud merge();

            PointClouds<T> display(bool show_clouds=false);

            std::size_t size() const {
                return data.size();
            }
            void resize(typename DataT::size_type size) {
                data.resize(size);
            }
            void reserve(typename DataT::size_type size) {
                data.reserve(size);
            }
            typename DataT::iterator begin() {
                return data.begin();
            }
            typename DataT::iterator end() {
                return data.end();
            }
            void push_back(const T& value) {
                data.push_back(value);
            }
            void pop_back() {
                data.pop_back();
            }



            PointCloud::Cloud::Ptr operator[](std::size_t index) const {
                if constexpr (std::is_same<T, std::string>::value)
                    return PointCloud::load(data[index]);
                else
                    return data[index];
            };
            PointClouds<T> operator[](std::slice slice) const {

                auto start = slice.start();
                auto size = slice.size();
                auto stride = slice.stride();

                auto result = PointClouds<T>();

                result.reserve(size);

                for (std::size_t i = start; i < size; i+=stride){
                    result.push_back(data[i]);
                }

                return result;
            }
            
    };

    template class PointClouds<PointCloud>;
    template class PointClouds<std::string>;

    using PointCloudsOnDisk = PointClouds<std::string>;
    using PointCloudsInMemory = PointClouds<PointCloud>;

} // namespace linkml



