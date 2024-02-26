#pragma once
#include <vector>
#include <types/point_cloud.hh>
#include <functions/progress_bar.hh>

namespace linkml
{
    PointCloud::Ptr filter(PointCloud::Ptr const & cloud){
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
}