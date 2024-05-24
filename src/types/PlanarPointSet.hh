#pragma once
#include <pcl/types.h>
#include <Eigen/Core>
#include <types/Plane.hh>

namespace linkml
{
    struct PlanarPointSet
    {
        pcl::Indices indices = pcl::Indices();
        Eigen::Vector3f centroid = Eigen::Vector3f();
        Eigen::Vector4f coefficients = Eigen::Vector4f();


        Plane get_Plane() const
        {
            return linkml::Plane( coefficients[0], coefficients[1], coefficients[2], coefficients[3], centroid[0], centroid[1], centroid[2] );
        }
    };
} // namespace linkml
