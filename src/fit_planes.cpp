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
#include <set>
#include <map>
#include <deque>

//#include <format>
//#include <iostream>
//#include <string>
//#include <string_view>
//#include <omp.h>


namespace linkml {


    template <typename T>
    class CircularBuffer {
    public:
        CircularBuffer(size_t maxSize) : buffer_(), maxSize_(maxSize) {
        }

        void push(const T& value) {
            buffer_.push_back(value);
            if (buffer_.size() > maxSize_) {
                buffer_.pop_front();
            }
        }

        T& operator[](size_t index) {
            return buffer_[index];
        }

        size_t size() const {
            return buffer_.size();
        }

    private:
        std::deque<T> buffer_;
        size_t maxSize_;
    };


    //////////////////////////////
    // Result data              //
    //////////////////////////////

    struct thread_result{
        bool vaild = false;
        std::set<int> proccesed = std::set<int>();
        std::vector<int> indecies = std::vector<int>();
        Plane plan = Plane();
    };


    //////////////////////////////
    // Non thread safe function //
    //////////////////////////////


    std::tuple<std::vector<Plane>,std::vector<std::vector<int>>> collect_results(std::vector<thread_result> thread_results ){

        std::vector<Plane> planes_results = std::vector<Plane>();
        std::vector<std::vector<int>> indecies_results = std::vector<std::vector<int>>();


        for (auto& thread : thread_results)
            if (thread.vaild){
                planes_results.push_back(thread.plan);
                indecies_results.push_back(thread.indecies);
            };

        return std::make_tuple( planes_results, indecies_results);
    }

    std::vector<int> get_sorted_index(std::vector<std::vector<int>> key ){
        std::vector<int> sizes = std::vector<int>();
        for (auto& indecies: key)
            sizes.push_back(indecies.size());


        std::vector<int> index(sizes.size());
        for (int i = 0; i < (int)sizes.size(); i++)
            index[i] = i;

        auto customComparator = [&sizes](int a, int b) {
            // Use the key list (keys) to compare elements
            return sizes[a] > sizes[b];
        };

        std::sort(index.begin(), index.end(),customComparator );

        return index;
    }

    std::vector<int> check_for_overlap(std::vector<int> index,std::vector<std::vector<int>> indecies_results ){
        std::vector<int> skip = std::vector<int>();


        for (int i = 1; i < (int)index.size(); i++){
            bool overlap = false;

            auto a_v = indecies_results.at(index[i]);
            std::set<int> a(a_v.begin(), a_v.end());


            for (int j = i-1; j > 0; j--){

                auto it = std::find(skip.begin(), skip.end(), index[j]);

                if (it != skip.end())
                    break;

                auto b_v = indecies_results.at(index[j]);
                std::set<int> b(a_v.begin(), a_v.end());

                std::set<int> intersection;
                std::set_intersection(a.begin(), a.end(), b.begin(), b.end(), std::inserter(intersection, intersection.begin()));

                if (!intersection.empty())
                    overlap= true;

            }

            if (overlap)
                skip.push_back(index[i]);
        }

        return skip;
    }

    std::vector<int> filter(std::vector<int> index, std::vector<int>skip) {

        std::vector<int> out = std::vector<int>();

        for (auto& val : index){
            auto it = std::find(skip.begin(), skip.end(), val);

            if (it != skip.end())
                break;

            out.push_back(val);
        }

        return out;
    }



    float gradien(CircularBuffer<int> buffer){

        std::vector<float> difference = std::vector<float>();

        if (buffer.size() < 2)
            return INFINITY;

        for (int i = 0; i < buffer.size() -1; i++){
            difference.push_back(buffer[i+1] - buffer[i]);
        }

        return tg::sum(difference) / difference.size();

    }

    float gradien(CircularBuffer<float> buffer){

        std::vector<float> difference = std::vector<float>();

        if (buffer.size() < 2)
            return INFINITY;

        for (int i = 0; i < buffer.size() -1; i++){
            difference.push_back(buffer[i+1] - buffer[i]);
        }

        return tg::sum(difference) / difference.size();

    }





    //////////////////////////////
    // Shared data              //
    //////////////////////////////

    std::shared_mutex dataMutex;
    std::atomic halt{false};
    std::set<int> processed = std::set<int>();
    std::vector<Plane> planes = std::vector<Plane>();
    std::vector<std::vector<int>> indecies = std::vector<std::vector<int>>();
    std::vector<thread_result> thread_results = std::vector<thread_result>();
    CircularBuffer<float> precet_buffer = CircularBuffer<float>(5);
    CircularBuffer<int> plane_count_buffer = CircularBuffer<int>(5);


    //////////////////////////////
    // Thread safe function     //
    //////////////////////////////

    std::vector<int> copy_thread_safe(std::set<int> vec){
        std::unique_lock<std::shared_mutex> lock(dataMutex);
        std::vector<int> output = std::vector<int>();
        std::copy(vec.begin(), vec.end(), std::back_inserter(output));
        return output;
    }

    bool break_checker(int cloud_size)
    {
        // TODO: Implement different stoping contidions
        //      All points serached
        //      60% > searched etc.

        //Aquired lock enjure no one else is updating the data.
        std::shared_lock<std::shared_mutex> lock(dataMutex);


        int remaining = cloud_size - processed.size();
        float progress_percent = 1 - (((float)(remaining)) / ((float)cloud_size));

        int found_planes = planes.size();

        precet_buffer.push(progress_percent);
        plane_count_buffer.push(found_planes);

        auto percent_gradien = gradien(precet_buffer);
        auto plane_count_gradien = gradien(plane_count_buffer);

        std::cout <<  "Check: " << (int)(progress_percent * 100) << "% - " << remaining << " - " << found_planes  << " percent gradient " << percent_gradien << " plane gradient count: " << plane_count_gradien <<std::endl;

//        if (progress_percent > 0.7)
//            return true;

        if ( plane_count_gradien == 0 and percent_gradien < 0.01  )
            return true;

        if (processed.size() > (ulong)cloud_size)
            return true;

        return false;
    }

    void update_results(){
        std::shared_lock<std::shared_mutex> lock(dataMutex);

        //update processed
        for (auto& thread : thread_results)
            for (auto& val : thread.proccesed)
                processed.insert(val);

        //Colect all results
        auto out = collect_results(thread_results);
        auto planes_results = std::get<0>(out);
        auto indecies_results = std::get<1>(out);

        // Check if there are valid results
        if (indecies_results.size() < 1)
            return;

        // Sort the results buy size
        auto index = get_sorted_index(indecies_results);


        // Check for overlap
        std::vector<int> skip = check_for_overlap(index,indecies_results);


        // Filter
        auto index_final = filter(index, skip);


        // Loop over final selection
        for (auto & idx : index_final){

            //Add Result to ouput
            planes.push_back(planes_results.at(idx));
            indecies.push_back(indecies_results.at(idx));


            //Mark selected indecies as proccesed
            for (auto& idx :indecies_results.at(idx))
                processed.insert(idx);
        }

    }



    //////////////////////////////
    // Thread Workers           //
    //////////////////////////////

    void plane_finder(
        std::stop_token st,
        std::barrier<> &b1,
        std::barrier<> &b2,
        int thread_num,
        point_cloud const &cloud,
        plane_fitting_parameters const &params){

        while (!st.stop_requested()){
            std::vector<int> proccesed_local = copy_thread_safe(processed);
            thread_result result_local = thread_result();

            int counter =0;


            while(!halt and !st.stop_requested()){

                if (counter > 1000){
                    halt = true;
                    break;
                }
                counter++;

                auto r = fit_plane(cloud, params, proccesed_local);
                if (!r.valid){
                    result_local.proccesed.insert(r.index);
                    proccesed_local.push_back(r.index);
                    continue;
                }

                result_local.plan = r.plane;
                result_local.indecies = r.indecies;
                result_local.vaild = true;
                halt = true;
                break;
            }

            thread_results.at(thread_num) = result_local;
            b1.arrive_and_wait();
            // The data from the indevidual threads is being colected and compared and updated.
            b2.arrive_and_wait();
        }
    }


    void results_processes(
        std::stop_token st,
        std::stop_source &ss,
        std::barrier<> &b1,
        std::barrier<> &b2,
        point_cloud const &cloud)
    {
        while (!st.stop_requested()){
            b1.arrive_and_wait();
            update_results();
            halt = false;
            if (break_checker(cloud.pts.size()))
                ss.request_stop();
            b2.arrive_and_wait();
        }
    };



    //////////////////////////
    // Function             //
    //////////////////////////


    fit_planes_resutl fit_planes(
        point_cloud const &cloud,
        plane_fitting_parameters const &params
        )
    {

        processed.clear();
        thread_results.clear();
        planes.clear();
        indecies.clear();


        auto stop_source = std::stop_source();


        int num_threads(16);
        auto token = stop_source.get_token();

        std::barrier b1(num_threads);
        std::barrier b2(num_threads);



        std::vector<std::jthread> threads = std::vector<std::jthread>();
        thread_results.assign(num_threads, thread_result());


        // Launch a set of worker threads looking for planes.
        for ( int i = 0; i < num_threads - 1 ; i++)
            threads.emplace_back(std::jthread(plane_finder, token, std::ref(b1), std::ref(b2), i, cloud, params));

        // Launching a mamager thread that will update the results
        auto update_thread = std::jthread(results_processes, token, std::ref(stop_source), std::ref(b1), std::ref(b2), cloud);


        // Work is being done
        for (auto& thread : threads)
            if (thread.joinable())
                thread.join();

        update_thread.join();

        auto result = fit_planes_resutl();
        result.planes = planes;
        result.indecies = indecies;
        return result;
    }



}
