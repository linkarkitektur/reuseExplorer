#include <types/PointCloud.hh>
#include <types/PointClouds.hh>

#include <vector>
#include <types/PointCloud.hh>
#include <functions/progress_bar.hh>

static linkml::PointCloud::Ptr filterPointCloud(linkml::PointCloud::Ptr cloud){

    // Only keep highest confidence
    ///////////////////////////////////////////////////////////////////////////////
    size_t j = 0;
    for (size_t k = 0; k < cloud->size(); k++){
        if (cloud->at(k).confidence >= 2U){
            cloud->at(j) = cloud->at(k);
            j++;
        }
    }
    cloud->resize(j);
    cloud->width = j;
    cloud->height = 1;
    cloud->is_dense = false;

    return cloud;
}

namespace linkml
{
    PointCloud::Ptr PointCloud::filter(){
        return filterPointCloud(pcl::make_shared<PointCloud>(*this));
    }


    // TODO: Initilaise the timplate rather then the classes indevidually
    // For an example see the annotation implementation
    template<>
    PointClouds<std::string>::Ptr PointClouds<std::string>::filter(){

        // Only keep highest confidence
        ///////////////////////////////////////////////////////////////////////////////
        #pragma omp parallel for
        for (size_t i = 0; i < data.size(); i++){
            PointCloud(data[i]).filter()->save(data[i]);
        }

        return std::make_shared<PointClouds<std::string>>(*this);

    }

    template<>
    PointClouds<PointCloud::Ptr>::Ptr PointClouds<PointCloud::Ptr>::filter(){

        // Only keep highest confidence
        ///////////////////////////////////////////////////////////////////////////////
        #pragma omp parallel for
        for (size_t i = 0; i < data.size(); i++){
            data[i] = filterPointCloud(data[i]);
        }

        return pcl::make_shared<PointClouds<PointCloud::Ptr>>(*this);
    }
} // namespace linkml
