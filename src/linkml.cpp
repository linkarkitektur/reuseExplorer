#include "linkml.h"
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <optional>
#include<tuple>
#include <typed-geometry/tg.hh>
#include <nanoflann.hpp>
#include <mutex>
#include <shared_mutex>
#include <thread>
#include <barrier>
//#include <typed-geometry/functions/random/random.hh>
#include <algorithm>
#include <set>
//#include <omp.h>

namespace linkml {

tg::rng rng;

bool is_perfect_square(int num)
{
    int sqrt_floor = std::floor(std::pow(num , 0.5));
    return sqrt_floor * sqrt_floor == num;
}

bool is_fibonacci(int num)
{
    if (num == 0 or num ==1)
        return true;

    int check1 = 5 * num * num + 4;
    int check2 = 5 * num * num - 4;

    if (is_perfect_square(check1) or is_perfect_square( check2) )
        return true;

    return false;

}

void update_register(reg &reg){

    std::vector<int> compacted = std::vector<int>();

    #pragma omp parallel for
    for (size_t i = 0; i< reg.mask.size(); i++)
        if ( reg.mask.at(i) == 1  )
            compacted.push_back(i);

    reg.indecies = compacted;
}


int random_integert_in_range(int min, int max)
{
    std::srand(std::time(NULL));
    return min + (rand() % (max - min + 1));
}

int get_random_index_not_in_register(std::vector<int> const &ps, int size)
{

    std::vector<int> all = std::vector<int>();
    std::vector<int> options = std::vector<int>();


    #pragma omp parallel for
    for (int i = 0; i<size; i++)
        all.push_back(i);

           //    std::set_difference(
           //        all.begin(), all.end(),
           //        ps.begin(), ps.end(),
           //        std::inserter(options, options.begin()));

    std::ranges::set_difference(all, ps, std::back_inserter(options) );


           //    std::vector<int> options_mask = std::vector<int>();
           //    std::vector<int> options = std::vector<int>();

           //    #pragma omp parallel for
           //    for (int i = 0; i < size; i++)
           //        options_mask.push_back(0);

           //    #pragma omp parallel for
           //    for (size_t i = 0; i < ps.size(); i++)
           //        options_mask[i] = 1;

           //    #pragma omp parallel for
           //    for (size_t i = 0; i < options_mask.size(); i++)
           //        if (options_mask[i] == 0)
           //            options.push_back(i);
       return tg::uniform(rng, options);

//    return options[random_integert_in_range(0,options.size()-1)];
}



Plane fit_plane_thorugh_points(const point_cloud &cloud, const std::vector<int> &indecies){

    std::vector<tg::pos3> points = std::vector<tg::pos3>();
    std::vector<tg::pos3> centered = std::vector<tg::pos3>();

    for (size_t i = 0; i < indecies.size(); i++ )
        points.push_back(cloud.pts.at(indecies[i]));

    tg::pos3 com = tg::mean(points);
    for (size_t i = 0; i < points.size(); i++)
        centered.push_back(points.at(i) - (tg::vec3)com);

    tg::mat3x3 cov = tg::covariance_matrix(centered);
    tg::array<float,3> eigenvalues = tg::eigenvalues_symmetric(cov);
    tg::array<tg::vec3,3> eigenvectors = tg::eigenvectors_symmetric(cov);

    tg::vec3 normal = tg::normalize_safe(eigenvectors[tg::min_index(eigenvalues)]);
    float distance = tg::dot(-com,normal);

    return Plane(normal.x,normal.y,normal.z,distance, com.x, com.y, com.z);

}


std::vector<int> filter_by_normal_angle(const point_cloud &cloud, const Plane plane, float cosalpha, std::vector<int> indecies_in ){

    std::vector<int> indecies_out = std::vector<int>();

    #pragma omp parallel for
    for( size_t i = 0; i< indecies_in.size(); i++)
    {
        int index = indecies_in.at(i);

        // Normalize Point Normal
        tg::vec3 unitised = tg::normalize(cloud.norm.at(index));
        tg::vec3 plane_normal = tg::normalize((tg::vec3)plane.normal);


        // Dot product  (Cos angele)
        float cos = std::abs(
            unitised.x*plane_normal.x+
            unitised.y*plane_normal.y+
            unitised.z*plane_normal.z
            );

        if (cos > cosalpha)
            indecies_out.push_back(index);

    }

    return indecies_out;
}

std::vector<int> filter_by_normal_distance(const point_cloud &cloud, const std::vector<int> indecies_in, const Plane plane, float distance_thresh){

    std::vector<int> indecies_out = std::vector<int>();

    #pragma omp parallel for
    for (size_t i = 0; i < indecies_in.size(); i++)
    {
        float dist = tg::distance(plane, cloud.pts.at(indecies_in.at(i)));

//        std::cout << "Dist :" << dist << " Threshh. :" << distance_thresh << std::endl;

        if (dist < distance_thresh){
            indecies_out.push_back(indecies_in.at(i));
        }
    }

    return indecies_out;
}


plane_fit_resutl fit_plane_(
    point_cloud const &cloud,
    plane_fitting_parameters const  &params,
    std::vector<int> const  &processed,
    int initial_point_idx){

    reg front_reg = reg(cloud.pts.size());
    reg plane_reg = reg(cloud.pts.size());


    // Set initila point in front
    front_reg.mask[initial_point_idx] = 1;
    front_reg.indecies.push_back(initial_point_idx);



    // Set initila point in plane
    plane_reg.mask[initial_point_idx] = 1;
    plane_reg.indecies.push_back(initial_point_idx);


    // Set plane
    Plane plane = Plane();
    plane.normal = tg::normalize(cloud.norm.at(initial_point_idx));;
    plane.origin = cloud.pts.at(initial_point_idx);
    plane.dis = 0;



    // Keep growing the plane
    int front_loop = 0;
    while (true){

        // Stop if the are no more points in the front
        if (front_reg.indecies.size() == 0)
            break;

        // Clear the front register mask
        front_reg.mask.assign(front_reg.mask.size(), 0);

        // Loop all items in the current front


        for (size_t i = 0; i < front_reg.indecies.size(); i++){


            std::vector<int> indecies = cloud.radiusSearch(
                front_reg.indecies.at(i),
                params.distance_threshhold );


            if (indecies.size() == 0)
                break;


            // Filter Angle
            indecies = filter_by_normal_angle(cloud, plane, params.cosalpha, indecies);
            if (indecies.size() == 0)
                break;


            // Filter Distance
            indecies = filter_by_normal_distance(cloud, indecies, plane, params.normal_distance_threshhold);
            if (indecies.size() == 0)
                break;

            //Assign values
            for (size_t i= 0; i< indecies.size(); i++)
                front_reg.mask[indecies.at(i)] = 1;

        }



        //Remove points that are already part of the current plane
        for (size_t i = 0; i < plane_reg.indecies.size(); i++)
            front_reg.mask[plane_reg.indecies.at(i)] = 0;


        //Remove point that have already been processed
        for (size_t i = 0; i < processed.size(); i++)
            front_reg.mask[processed.at(i)] = 0;

        update_register(front_reg);

        // Update Plane with furrent front
        for (size_t i = 0; i < front_reg.indecies.size(); i++)
            plane_reg.mask[front_reg.indecies.at(i)] = 1;


        // Update plane
        if (is_fibonacci(front_loop)){
            update_register(plane_reg);
            if (plane_reg.indecies.size() > 3)
                plane = fit_plane_thorugh_points(cloud, plane_reg.indecies);
        }

        front_loop++;
    }


    update_register(plane_reg);


    // Check if plane meets requered size
    if ((int)plane_reg.indecies.size() < params.plane_size_threshhold)
        return plane_fit_resutl(initial_point_idx);

    plane = fit_plane_thorugh_points(cloud, plane_reg.indecies);



    //Clear plane register
    return plane_fit_resutl(plane, plane_reg.indecies);


};



plane_fit_resutl fit_plane(
    point_cloud const &cloud,
    plane_fitting_parameters const  &params,
    std::vector<int> const  &processed,
    int initial_point_idx){

    return fit_plane_(cloud, params, processed, initial_point_idx);
}


plane_fit_resutl fit_plane(
    point_cloud const &cloud,
    plane_fitting_parameters const &params,
    std::vector<int> const processed){

    int initial_point_idx = get_random_index_not_in_register(processed, cloud.pts.size() );;

    return fit_plane_(cloud, params, processed, initial_point_idx );
}


plane_fit_resutl fit_plane(
    point_cloud const &cloud,
    plane_fitting_parameters const &params
    ){

    auto processed = std::vector<int>();
    int initial_point_idx = get_random_index_not_in_register(processed, cloud.pts.size() );;

    return fit_plane_(cloud, params, processed, initial_point_idx);
}


plane_fit_resutl fit_plane(
    point_cloud const &cloud,
    plane_fitting_parameters const &params,
    int const initial_point_idx){

    auto processed = std::vector<int>();

    return fit_plane_(cloud, params, processed, initial_point_idx );
}

}
