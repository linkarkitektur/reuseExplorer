#include "progress_bar.hh"

#include <cstdio>

#include <iostream>
#include <chrono>
#include <thread>
#include <iomanip>
#include <stdexcept>

#include <fmt/format.h>
#include <fmt/printf.h>
#include <fmt/chrono.h>
#include <fmt/color.h>



#ifdef _OPENMP
  ///Multi-threading - yay!
  #include <omp.h>
#else
  ///Macros used to disguise the fact that we do not have multithreading enabled.
  #define omp_get_thread_num()  0
  #define omp_get_num_threads() 1
#endif


using namespace fmt::literals;


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


    //Quick return if insufficient progress has occurred
    if(_finished_work < _next_update)
      return;

    //Update the next time at which we'll do the expensive update stuff
    _next_update += _total_work/200;

    // Handle overflow
    if (_finished_work > _total_work)
        _finished_work = _total_work;


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


    int n = std::to_string(_total_work).length();
    double seconds = std::chrono::duration_cast<std::chrono::seconds>(_timer.lap()).count();

    fmt::print("] ({f_work:>{w}d}/{t_work:<{w}d}) {r: >3.0f}%", "f_work"_a = _finished_work, "t_work"_a = _total_work, "r"_a = ratio * 100, "w"_a = n);
    fmt::print(" - {: > 5.0f}s - {: > 3d} threads", seconds / ratio*(1-ratio), omp_get_num_threads());

    if (!_task_name.empty())
        std::printf(" - (%s)", _task_name.c_str());
    if (finished()){
        std::printf("\n");
        _finalized = true;
    }
    std::fflush(stdout);

}

void util::progress_bar::reset_line()
{
    std::printf("\r");
    // on ansi terminals we might want to use
    //    std::printf("\r\033[2K[");
}

std::chrono::nanoseconds util::progress_bar::stop(){
    reset_line();
    _timer.stop();


    #ifdef NOPROGRESS
      return _timer.accumulated();
    #endif

    if (!_finalized){
      std::printf("\n");
      _finalized = true;
    }

    // use fmt to print duration with hours minutes and seconds
    fmt::print(fg(fmt::color::gray), "Total time: ");
    fmt::print(fmt::emphasis::italic,  "{:%H:%M:%S}s", std::chrono::duration_cast<std::chrono::seconds>(_timer.accumulated()));
    if (!_task_name.empty())
        fmt::print(fg(fmt::color::gray), " ({})\n",  _task_name.c_str());
    else
        fmt::printf("\n");
        


    // std::cout << "Total time: " << _timer.accumulated() << "s" << std::endl;
    std::fflush(stdout);


    return _timer.accumulated();

}