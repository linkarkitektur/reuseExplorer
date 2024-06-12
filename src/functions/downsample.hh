#pragma once
#include "types/PointCloud.hh"


namespace linkml
{
    void downsample(PointCloud::Cloud::Ptr cloud, double leaf_size);
}