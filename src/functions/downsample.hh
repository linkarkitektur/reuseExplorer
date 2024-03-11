#pragma once
#include <types/PointCloud.hh>

namespace linkml
{
    template <typename T> // T is currently a PointCloud::Ptr
    static T downsample(T cloud);
} // namespace linkml
