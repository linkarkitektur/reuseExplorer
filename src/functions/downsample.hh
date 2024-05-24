#pragma once
#include "types/PointCloud.hh"


namespace linkml
{
    void downsample(PointCloud::Ptr cloud, double leaf_size);
}