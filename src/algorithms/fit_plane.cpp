#include <algorithms/fit_plane.hh>
#include <functions/fit_plane_thorugh_points.hh>

#include <types/plane.hh>
#include <types/plane_fit_parameters.hh>
#include <types/point_cloud.hh>
#include <types/result_fit_plane.hh>
#include <types/reg.hh>

#include <cstdlib>
#include <ctime>
#include <iostream>
#include <optional>
#include <tuple>
#include <typed-geometry/tg.hh>
#include <nanoflann.hpp>
#include <mutex>
#include <shared_mutex>
#include <thread>
#include <barrier>
#include <algorithm>
#include <set>
#include <iterator>
#include <vector>
//#include <omp.h>


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



int random_integert_in_range(int min, int max)
{
    std::srand(std::time(NULL));
    return min + (rand() % (max - min + 1));
}

int get_random_index_not_in_register(std::vector<int> const &ps, int size)
{

    std::vector<int> all = std::vector<int>();
    std::vector<int> options = std::vector<int>();


    // pragma omp parallel for
    for (int i = 0; i<size; i++)
        all.push_back(i);

           //    std::set_difference(
           //        all.begin(), all.end(),
           //        ps.begin(), ps.end(),
           //        std::inserter(options, options.begin()));


    // Find the difference between all an ps and store it in options
    std::set_difference(
        all.begin(), all.end(),
        ps.begin(), ps.end(),
        std::back_inserter(options));



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

std::vector<int> filter_by_normal_angle(const linkml::point_cloud &cloud, const linkml::Plane plane, float cosalpha, std::vector<int> indecies_in ){

    std::vector<int> indecies_out = std::vector<int>();

    // pragma omp parallel for
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

std::vector<int> filter_by_normal_distance(const linkml::point_cloud &cloud, const std::vector<int> indecies_in, const linkml::Plane plane, float distance_thresh){

    std::vector<int> indecies_out = std::vector<int>();

    // pragma omp parallel for
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


namespace linkml{

    result_fit_plane fit_plane_(
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

            front_reg.update();

            // Update Plane with furrent front
            for (size_t i = 0; i < front_reg.indecies.size(); i++)
                plane_reg.mask[front_reg.indecies.at(i)] = 1;


            // Update plane
            if (is_fibonacci(front_loop)){
                plane_reg.update();
                if (plane_reg.indecies.size() > 3)
                    plane = fit_plane_thorugh_points(cloud, plane_reg.indecies);
            }

            front_loop++;
        }


        plane_reg.update();


        // Check if plane meets requered size
        if ((int)plane_reg.indecies.size() < params.plane_size_threshhold)
            return linkml::result_fit_plane(initial_point_idx);

        plane = fit_plane_thorugh_points(cloud, plane_reg.indecies);



        //Clear plane register
        return linkml::result_fit_plane(plane, plane_reg.indecies);


    };



    result_fit_plane fit_plane(
        point_cloud const &cloud,
        plane_fitting_parameters const  &params,
        std::vector<int> const  &processed,
        int initial_point_idx){

        return fit_plane_(cloud, params, processed, initial_point_idx);
    }


    result_fit_plane fit_plane(
        point_cloud const &cloud,
        plane_fitting_parameters const &params,
        std::vector<int> const processed){

        int initial_point_idx = get_random_index_not_in_register(processed, cloud.pts.size() );

        return fit_plane_(cloud, params, processed, initial_point_idx );
    }


    result_fit_plane fit_plane(
        point_cloud const &cloud,
        plane_fitting_parameters const &params
        ){

        auto processed = std::vector<int>();
        int initial_point_idx = get_random_index_not_in_register(processed, cloud.pts.size() );

        return fit_plane_(cloud, params, processed, initial_point_idx);
    }


    result_fit_plane fit_plane(
        point_cloud const &cloud,
        plane_fitting_parameters const &params,
        int const initial_point_idx){

        auto processed = std::vector<int>();

        return fit_plane_(cloud, params, processed, initial_point_idx );
    }
}
