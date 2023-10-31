#pragma once

#include "linkml.h"
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
#include <csignal>


namespace linkml {



class PlaneFit_Solver {

    struct thread_result{
        bool vaild = false;
        std::set<int> proccesed = std::set<int>();
        std::vector<int> indecies = std::vector<int>();
        Plane plan = Plane();
    };


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


public:

    PlaneFit_Solver(){
//        stop_source = std::stop_source();
        processed = std::set<int>();
        thread_results = std::vector<thread_result>();
        planes = std::vector<Plane>();
        indecies = std::vector<std::vector<int>>();
        instances.push_back(this);
        }

    fit_planes_resutl run(
        point_cloud const &cloud,
        plane_fitting_parameters const &params);



private:

    struct pair{
        int row;
        int col;
    };

   //////////////////////////////
   // Shared data              //
   //////////////////////////////

    std::shared_mutex dataMutex;
    std::atomic<bool> halt{false};
    std::set<int> processed;
    std::vector<Plane> planes;
    std::vector<std::vector<int>> indecies;
    std::vector<thread_result> thread_results;
    CircularBuffer<float> precet_buffer = CircularBuffer<float>(5);
    CircularBuffer<int> plane_count_buffer = CircularBuffer<int>(5);


    const int BUFFER_SIZE = 1;
    //    std::mutex bufferMutex;
    std::queue<thread_result> buffer;
    std::condition_variable_any notEmpty, notFull;
    std::vector<std::atomic_bool> update;
    int counter = 0;

    std::stop_source stop_source = std::stop_source();

   //////////////////////////////
   // Non thread safe function //
   //////////////////////////////


    std::tuple<std::vector<Plane>,std::vector<std::vector<int>>> collect_results(std::vector<thread_result> thread_results );
    std::vector<int> get_sorted_index(std::vector<std::vector<int>> key );
    std::vector<int> check_for_overlap(std::vector<int> index,std::vector<std::vector<int>> indecies_results );
    std::vector<int> filter(std::vector<int> index, std::vector<int>skip);

    float gradien(CircularBuffer<int> buffer);
    float gradien(CircularBuffer<float> buffer);



    void save(int cloud_size);



    void interupt_solver(int s );


    static void callHandlers (int signum) // calls the handlers
    {
        std::for_each(instances.begin(),
                      instances.end(),
                      std::bind2nd(std::mem_fn(&PlaneFit_Solver::interupt_solver), signum));
    }

    static std::vector<PlaneFit_Solver *> instances;

   //////////////////////////////
   // Thread safe function     //
   //////////////////////////////

    std::vector<int> copy_thread_safe(std::set<int> vec);
    bool break_checker(int cloud_size);
    void update_results();
    void update_results(thread_result r);
    void push_result(thread_result r );
    thread_result pop_result();



   //////////////////////////////
   // Thread Workers           //
   //////////////////////////////

    void producer(std::stop_token st, int thread_num, point_cloud const &cloud, plane_fitting_parameters const &params );
    void consumer(std::stop_token st, std::stop_source &ss, point_cloud const  &cloud);

    static void plane_finder(
        std::stop_token st,
        PlaneFit_Solver *instatce,
        std::barrier<> &b1,
        std::barrier<> &b2,
        int thread_num,
        point_cloud const &cloud,
        plane_fitting_parameters const &params);

    static void results_processes(
        std::stop_token st,
        PlaneFit_Solver *instatce,
        std::stop_source &ss,
        std::barrier<> &b1,
        std::barrier<> &b2,
        point_cloud const &cloud);

};
}
