#pragma once

#include <typed-geometry/tg.hh>

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

    static linkml::Plane fit_plane_thorugh_points(linkml::point_cloud const& cloud, std::vector<int> const& indecies){

        std::vector<tg::pos3> points = std::vector<tg::pos3>();
        for (size_t i = 0; i < indecies.size(); i++ )
            points.push_back(cloud.pts.at(indecies[i]));

        return fit_plane_thorugh_points(points);
    }
}