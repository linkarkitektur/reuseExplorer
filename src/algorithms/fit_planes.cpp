#include <algorithms/fit_planes.h>
#include <algorithms/fit_plane.h>

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
#include <set>
#include <map>
#include <deque>

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <omp.h>


#include <iostream>
#include <fstream>

#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>


namespace linkml {


       //////////////////////////////
       // Non thread safe function //
       //////////////////////////////

std::tuple<std::vector<Plane>,std::vector<std::vector<int>>> PlaneFit_Solver::collect_results(std::vector<thread_result> thread_results ){

    std::vector<Plane> planes_results = std::vector<Plane>();
    std::vector<std::vector<int>> indecies_results = std::vector<std::vector<int>>();


    for (auto& thread : thread_results)
        if (thread.vaild){
            planes_results.push_back(thread.plan);
            indecies_results.push_back(thread.indecies);
        };

    return std::make_tuple( planes_results, indecies_results);
}

std::vector<int> PlaneFit_Solver::get_sorted_index(std::vector<std::vector<int>> key ){
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

std::vector<int> PlaneFit_Solver::check_for_overlap(std::vector<int> index,std::vector<std::vector<int>> indecies_results ){
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

std::vector<int> PlaneFit_Solver::filter(std::vector<int> index, std::vector<int>skip) {

    std::vector<int> out = std::vector<int>();

    for (auto& val : index){
        auto it = std::find(skip.begin(), skip.end(), val);

        if (it != skip.end())
            break;

        out.push_back(val);
    }

    return out;
}

float PlaneFit_Solver::gradien(CircularBuffer<int> buffer){

    std::vector<float> difference = std::vector<float>();

    if (buffer.size() < 2)
        return INFINITY;

    for (int i = 0; i < (int)buffer.size() -1; i++){
        difference.push_back(buffer[i+1] - buffer[i]);
    }

    return tg::sum(difference) / difference.size();

}

float PlaneFit_Solver::gradien(CircularBuffer<float> buffer){

    std::vector<float> difference = std::vector<float>();

    if (buffer.size() < 2)
        return INFINITY;

    for (int i = 0; i < (int)buffer.size() -1; i++){
        difference.push_back(buffer[i+1] - buffer[i]);
    }

    return tg::sum(difference) / difference.size();

}

void PlaneFit_Solver::save(int cloud_size){
    int l = std::ceil(std::sqrt(cloud_size));
    cv::Mat image(l, l, CV_8UC3); // CV_8UC3 |  CV_8UC1


    auto map = std::map<int, PlaneFit_Solver::pair>();
    int index = 0;
#pragma omp parallel for
    for (int row = 0; row < l; ++row) {
        for (int col = 0; col < l; ++col) {
            auto p = PlaneFit_Solver::pair();
            p.row = row;
            p.col = col;
            map[index] = p;

            if (index < cloud_size){

                       // Not Proccesed
                image.at<cv::Vec3b>(row, col)[0] = 255;
                image.at<cv::Vec3b>(row, col)[1] = 0;
                image.at<cv::Vec3b>(row, col)[2] = 0;

            }
            else {
                image.at<cv::Vec3b>(row, col)[0] = 0;
                image.at<cv::Vec3b>(row, col)[1] = 0;
                image.at<cv::Vec3b>(row, col)[2] = 0;
            }
            index++;
        }
    }

    for (auto& i : processed)
    {
        auto p = map[i];
        image.at<cv::Vec3b>(p.row, p.col)[0] = 0;
        image.at<cv::Vec3b>(p.row, p.col)[1] = 255;
        image.at<cv::Vec3b>(p.row, p.col)[2] = 0;
    }


    //        cv::Mat downscaledImage;
    //        cv::resize(image, downscaledImage, cv::Size(1000, 1000));

    std::stringstream name;
    name << "output_image_" << counter << ".png";
    cv::imwrite(name.str(), image);



           // Define the file path where you want to save the vector
    std::stringstream filePath;
    filePath <<  "processed_" << counter << ".csv";

           // Create an output file stream
    std::ofstream outputFile(filePath.str());

           // Check if the file stream is open
    if (outputFile.is_open()) {
        // Write the vector elements to the file
        for (const int& element : processed) {
            outputFile << element << ",";
        }

               // Close the file stream
        outputFile.close();

    }

    counter++;
}


       //////////////////////////////
       // Thread safe function     //
       //////////////////////////////

std::vector<int> PlaneFit_Solver::copy_thread_safe(std::set<int> vec){
    std::unique_lock<std::shared_mutex> lock(dataMutex);
    std::vector<int> output = std::vector<int>();
    std::copy(vec.begin(), vec.end(), std::back_inserter(output));
    return output;
}

bool PlaneFit_Solver::break_checker(int cloud_size)
{
    // TODO: Implement different stoping contidions
    //      All points serached
    //      60% > searched etc.

           //Aquired lock enjure no one else is updating the data.
    std::shared_lock<std::shared_mutex> lock(dataMutex);


    int remaining = cloud_size - processed.size();
    float progress_percent = 1 - (((float)(remaining)) / ((float)cloud_size));


    //        save(cloud_size);

    std::vector<int> img = std::vector<int>(cloud_size, 255);
    for (auto& i : processed)
        img[i] = 0;


    int found_planes = planes.size();

    precet_buffer.push(progress_percent);
    plane_count_buffer.push(found_planes);

    auto percent_gradien = gradien(precet_buffer);
    auto plane_count_gradien = gradien(plane_count_buffer);

    std::cout.flush();
    std::cout <<  "Check: " << (int)(progress_percent * 100) << "% - " << remaining << " - " << found_planes  << " percent gradient " << percent_gradien << " plane gradient count: " << plane_count_gradien <<std::endl;

    if (progress_percent > 0.7)
        return true;

    //        if ( plane_count_gradien == 0 and percent_gradien < 0.01  )
    //            return true;

    if (processed.size() >= (ulong)cloud_size)
        return true;

    return false;
}

void PlaneFit_Solver::update_results(){
    std::shared_lock<std::shared_mutex> lock(dataMutex);

           //update processed
    for (auto& thread : thread_results)
        for (auto& val : thread.proccesed)
            processed.insert(val);

           //Colect all results
    auto out = this->collect_results(thread_results);
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

void PlaneFit_Solver::update_results(thread_result r){
    std::shared_lock<std::shared_mutex> lock(dataMutex);

    for (auto& val : r.proccesed )
        processed.insert(val);


    if (r.vaild){
        planes.push_back(r.plan);
        indecies.push_back(r.indecies);

        for (auto& val : r.indecies )
            processed.insert(val);


               //Signal all threads to update
        for (auto& b:   update)
            b = true;
    }

}

    void PlaneFit_Solver::push_result(thread_result r ){
        std::shared_lock<std::shared_mutex> lock(dataMutex);
        notFull.wait(lock, [this]{ return (int)buffer.size() < BUFFER_SIZE; });
        buffer.push(r);
    }

    PlaneFit_Solver::thread_result PlaneFit_Solver::pop_result(){
        std::shared_lock<std::shared_mutex> lock(dataMutex);

        notEmpty.wait(lock, [this]{ return !buffer.empty(); });

        thread_result data = buffer.front();
        buffer.pop();
        return data;

    }

       //////////////////////////////
       // Thread Workers           //
       //////////////////////////////


void PlaneFit_Solver::producer(std::stop_token st, int thread_num, point_cloud const &cloud, plane_fitting_parameters const &params ){
    std::vector<int> proccesed_local = copy_thread_safe(processed);
    thread_result result_local = thread_result();
    int counter =0;

    while(!st.stop_requested()){
        if (update[thread_num]){
            proccesed_local = copy_thread_safe(processed);
            counter =0;
            update[thread_num] = false;
        }
        auto r = linkml::fit_plane(cloud, params, proccesed_local);
        if (r.valid){
            result_local.plan = r.plane;
            result_local.indecies = r.indecies;
            result_local.vaild = true;


                   // Push data
            push_result(result_local);
            //                std::unique_lock<std::mutex> lock(bufferMutex);
            //                notFull.wait(lock, []{ return buffer.size() < BUFFER_SIZE; });
            //                buffer.push(result_local);
            //                lock.unlock();

                   // Update and reset local data
            proccesed_local = copy_thread_safe(processed);
            counter =0;
            result_local = thread_result();

        } else{
            result_local.proccesed.insert(r.index);
            proccesed_local.push_back(r.index);
        }

               // If we have done more then 1000 iterrations updated the main thread anyway.
        if (counter > 1000){
            // Push data
            push_result(result_local);
            //                std::unique_lock<std::mutex> lock(bufferMutex);
            //                notFull.wait(lock, []{ return buffer.size() < BUFFER_SIZE; });
            //                buffer.push(result_local);
            //                lock.unlock();

                   // Update and reset local data
            proccesed_local = copy_thread_safe(processed);
            counter =0;
            result_local = thread_result();
        }
        counter++;
    };
}
void PlaneFit_Solver::consumer(std::stop_token st, std::stop_source &ss, point_cloud const  &cloud){

    while(!st.stop_requested()){

        std::this_thread::sleep_for(std::chrono::milliseconds(300)); // Simulate some work

        //            std::unique_lock<std::mutex> lock(bufferMutex);
        //            notEmpty.wait(lock, []{ return !buffer.empty(); });

        //            thread_result data = buffer.front();
        //            buffer.pop();
        auto data = pop_result();

        update_results(data);


        //            lock.unlock();
        notFull.notify_all();


        if (break_checker(cloud.pts.size()))
            ss.request_stop();

    }
}

void PlaneFit_Solver::plane_finder(
    std::stop_token st,
    PlaneFit_Solver *instatce,
    std::barrier<> &b1,
    std::barrier<> &b2,
    int thread_num,
    point_cloud const &cloud,
    plane_fitting_parameters const &params){

    while (!st.stop_requested()){
        std::vector<int> proccesed_local = instatce->copy_thread_safe(instatce->processed);
        thread_result result_local = thread_result();

        int counter =0;
        int halt_counter =0;


        while(!st.stop_requested()){

            if (instatce->halt){
                if (halt_counter > 200)
                    break;
                halt_counter++;
            }

            if (counter > 1000){
                instatce->halt = true;
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
            instatce->halt = true;
            break;
        }

        instatce->thread_results.at(thread_num) = result_local;
        b1.arrive_and_wait();
        // The data from the indevidual threads is being colected and compared and updated.
        b2.arrive_and_wait();
    }
}


void PlaneFit_Solver::results_processes(
    std::stop_token st,
    PlaneFit_Solver *instatce,
    std::stop_source &ss,
    std::barrier<> &b1,
    std::barrier<> &b2,
    point_cloud const &cloud)
{
    while (!st.stop_requested()){
        b1.arrive_and_wait();
        instatce->update_results();
        instatce->halt = false;
        if (instatce->break_checker(cloud.pts.size()))
            ss.request_stop();
        b2.arrive_and_wait();
    }
};



       //////////////////////////
       // Function             //
       //////////////////////////


    void PlaneFit_Solver::interupt_solver(int s ){
        std::cout << "Stoping search early as per user request" << std::endl;
        stop_source.request_stop();
    }

    std::vector<PlaneFit_Solver *> PlaneFit_Solver::instances = std::vector<PlaneFit_Solver *>();


    result_fit_planes PlaneFit_Solver::run(
        point_cloud const &cloud,
        plane_fitting_parameters const &params
        )
    {

        int num_threads(16);
        auto token = stop_source.get_token();

        std::barrier b1(num_threads);
        std::barrier b2(num_threads);


        // Handel user interuption
        struct sigaction sigIntHandler;

        sigIntHandler.sa_handler = callHandlers;
        sigemptyset(&sigIntHandler.sa_mask);
        sigIntHandler.sa_flags = 0;

        sigaction(SIGINT, &sigIntHandler, NULL);
//        sigaction(SIGTERM, &sigIntHandler, NULL);

        // Single thread
        if (true){
            while (!token.stop_requested() and !break_checker(cloud.pts.size())){
                std::vector<int> proccesed_local = copy_thread_safe(processed);
                result_fit_plane result = fit_plane(cloud, params, proccesed_local);

                if (result.valid){
                    for (size_t i =0; i < result.indecies.size();i++)
                        processed.insert(result.indecies.at(i));

                    indecies.push_back(result.indecies);
                    planes.push_back(result.plane);
                }
                else
                    processed.insert(result.index);
            }
        }


        // Multithreading
        if (false){
            std::vector<std::jthread> threads = std::vector<std::jthread>();
            thread_results.assign(num_threads, thread_result());


                   // Launch a set of worker threads looking for planes.
            for ( int i = 0; i < num_threads - 1 ; i++)
                threads.emplace_back(std::jthread(plane_finder, token, this, std::ref(b1), std::ref(b2), i, cloud, params));

                   // Launching a mamager thread that will update the results
            auto update_thread = std::jthread(results_processes, token, this, std::ref(stop_source), std::ref(b1), std::ref(b2), cloud);


                   // Work is being done
            for (auto& thread : threads)
                if (thread.joinable())
                    thread.join();

            update_thread.join();
        }


        // Multithreading 2
        if (false){

            update.clear();
            buffer = std::queue<thread_result>();

            std::vector<std::jthread> producer_threads = std::vector<std::jthread>();
            thread_results.assign(num_threads, thread_result());

            update = std::vector<std::atomic_bool>(num_threads);
            for (auto& a :update)
                a = false;

            // Launch a set of worker threads looking for planes.
            for ( int i = 0; i < num_threads - 1 ; i++)
//                producer_threads.emplace_back(std::jthread(this->producer, token, i, cloud, params));
                producer_threads.emplace_back(std::jthread(std::bind(&PlaneFit_Solver::producer, this, token, i, std::ref(cloud), std::ref(params))));

            // Launching a mamager thread that will update the results
//            auto consumer_thread = std::jthread(this->consumer, token, std::ref(stop_source), cloud);
            auto consumer_thread = std::jthread(std::bind(&PlaneFit_Solver::consumer, this, token, std::ref(stop_source), std::ref(cloud)));



            // Work is being done
            for (auto& thread : producer_threads)
                if (thread.joinable())
                    thread.join();

            consumer_thread.join();

        }

        auto result = result_fit_planes();
        result.planes = planes;
        result.indecies = indecies;

        return result;
    }

}
