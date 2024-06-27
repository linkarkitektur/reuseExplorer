#include <types/PointCloud.hh>
#include <types/PointClouds.hh>

#include <vector>
#include <types/PointCloud.hh>
#include <functions/progress_bar.hh>


namespace linkml
{
    void PointCloud::filter(){

        // Only keep highest confidence
        ///////////////////////////////////////////////////////////////////////////////
        size_t j = 0;
        for (size_t k = 0; k < (*this)->size(); k++){
            if ((*this)->at(k).confidence >= 2U){
                (*this)->at(j) = (*this)->at(k);
                j++;
            }
        }
        (*this)->resize(j);
        (*this)->width = j;
        (*this)->height = 1;
        (*this)->is_dense = false;


    }

    template <typename T>
    PointClouds<T> PointClouds<T>::filter(){

        // Only keep highest confidence
        ///////////////////////////////////////////////////////////////////////////////
        auto filter_bar = util::progress_bar(data.size(), "Filtering");
        #pragma omp parallel for
        for (size_t i = 0; i < data.size(); i++){
            if constexpr (std::is_same<T, std::string>::value){
                auto cloud = PointCloud::load(data[i]);
                cloud.filter();
                cloud.save(data[i]);  
            } else if constexpr (std::is_same<T, PointCloud>::value){
                data[i].filter();
            }
            filter_bar.update();
        }
        filter_bar.stop();

        return *this;

    }

    template PointCloudsInMemory  PointCloudsInMemory::filter();
    template PointCloudsOnDisk  PointCloudsOnDisk::filter();

} // namespace linkml
