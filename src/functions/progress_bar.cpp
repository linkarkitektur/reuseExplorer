#include "progress_bar.hh"

#include <cstdio>

#include <iostream>
#include <chrono>
#include <thread>
#include <iomanip>
#include <stdexcept>

#include <fmt/format.h>
#include <fmt/printf.h>



#ifdef _OPENMP
  ///Multi-threading - yay!
  #include <omp.h>
#else
  ///Macros used to disguise the fact that we do not have multithreading enabled.
  #define omp_get_thread_num()  0
  #define omp_get_num_threads() 1
#endif




void util::progress_bar::do_work(int work){

    //Quick return if this isn't the main thread
    if(omp_get_thread_num()!=0)
      return;

    _finished_work += work*(omp_get_num_threads());
}
void util::progress_bar::print()
{

    //Provide simple way of optimizing out progress updates
    #ifdef NOPROGRESS
      return;
    #endif

    //Quick return if this isn't the main thread
    if(omp_get_thread_num()!=0)
      return;



    // //Quick return if insufficient progress has occurred
    // if(_finished_work<_next_update)
    //   return;

    // //Update the next time at which we'll do the expensive update stuff
    // _next_update += _call_diff;


    reset_line();

    auto const ratio = float(_finished_work) / float(_total_work);
    auto const position = int(ratio * _bar_width);

    //Print an update string which looks like this:
    //  [===============================================>  ] (10/100)  10.01% - 1.0s - 4 threads)
    std::printf("[");
    for (auto i = 0; i < _bar_width; ++i)
    {
        if (i < position)
            std::printf("=");
        else if (i == position)
            std::printf(">");
        else
            std::printf(" ");
    }
    fmt::print("] ({:>5d}/{:<5d}) {: >3.0f}%", _finished_work, _total_work, ratio * 100);
    fmt::print(" - {: ^ 5.0f}s - {} threads", _timer.lap() / ratio*(1-ratio), omp_get_num_threads());

    if (!_task_name.empty())
        std::printf(" - (%s)", _task_name.c_str());
    if (finished())
        std::printf("\n");
    std::fflush(stdout);

}

void util::progress_bar::reset_line()
{
    std::printf("\r");
    // on ansi terminals we might want to use
    //    std::printf("\r\033[2K[");
}

double util::progress_bar::stop(){
    reset_line();

    _timer.stop();
    return _timer.accumulated();
}