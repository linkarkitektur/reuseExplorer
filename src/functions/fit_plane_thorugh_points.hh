#pragma once

#include <typed-geometry/tg.hh>
#include "../types/Plane.hh"
#include "../types/PointCloud.hh"


#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
// #include <pcl/segmentation/region_growing.h>
// #include <pcl/segmentation/planar_region.h>

namespace linkml {

    static linkml::Plane fit_plane_thorugh_points(std::vector<tg::pos3> const& points){

        std::vector<tg::pos3> centered = std::vector<tg::pos3>();

        tg::pos3 com = tg::mean(points);
        for (size_t i = 0; i < points.size(); i++)
            centered.push_back(points.at(i) - (tg::vec3)com);

        tg::mat3x3 cov = tg::covariance_matrix(centered);
        tg::array<float,3> eigenvalues = tg::eigenvalues_symmetric(cov);
        tg::array<tg::vec3,3> eigenvectors = tg::eigenvectors_symmetric(cov);

        tg::vec3 normal = tg::normalize_safe(eigenvectors[tg::min_index(eigenvalues)]);
        float distance = tg::dot(-com,normal);

        return linkml::Plane(normal.x,normal.y,normal.z,distance, com.x, com.y, com.z);

    }

    static linkml::Plane fit_plane_thorugh_points(PointCloud::Cloud::ConstPtr cloud,  pcl::Indices const & indecies){


        Eigen::Vector4f vp = Eigen::Vector4f::Zero ();
        Eigen::Vector4f clust_centroid = Eigen::Vector4f::Zero ();
        Eigen::Matrix3f clust_cov;

        pcl::computeMeanAndCovarianceMatrix (*cloud, indecies, clust_cov, clust_centroid);
        Eigen::Vector4f plane_params;
        
        EIGEN_ALIGN16 Eigen::Vector3f::Scalar eigen_value;
        EIGEN_ALIGN16 Eigen::Vector3f eigen_vector;
        pcl::eigen33 (clust_cov, eigen_value, eigen_vector);
        plane_params[0] = eigen_vector[0];
        plane_params[1] = eigen_vector[1];
        plane_params[2] = eigen_vector[2];
        plane_params[3] = 0;
        plane_params[3] = -1 * plane_params.dot (clust_centroid);

        vp -= clust_centroid;
        float cos_theta = vp.dot (plane_params);
        if (cos_theta < 0)
        {
            plane_params *= -1;
            plane_params[3] = 0;
            plane_params[3] = -1 * plane_params.dot (clust_centroid);
        }

        return linkml::Plane(plane_params[0], plane_params[1], plane_params[2], plane_params[3], clust_centroid[0], clust_centroid[1], clust_centroid[2]);
    }
}