#include "progress_bar.hh"

#include <cstdio>

#include <iostream>
#include <chrono>
#include <thread>
#include <iomanip>
#include <stdexcept>


#ifdef _OPENMP
  ///Multi-threading - yay!
  #include <omp.h>
#else
  ///Macros used to disguise the fact that we do not have multithreading enabled.
  #define omp_get_thread_num()  0
  #define omp_get_num_threads() 1
#endif


namespace util{

void progress_bar::update(uint32_t work_done0){
    //Provide simple way of optimizing out progress updates
    #ifdef NOPROGRESS
      return;
    #endif

    //Quick return if this isn't the main thread
    if(omp_get_thread_num()!=0)
      return;

    //Update the amount of work done
    work_done = work_done0;

    //Quick return if insufficient progress has occurred
    if(work_done<next_update)
      return;

    //Update the next time at which we'll do the expensive update stuff
    next_update += call_diff;

    //Use a uint16_t because using a uint8_t will cause the result to print as a
    //character instead of a number
    uint16_t percent = (uint8_t)(work_done*omp_get_num_threads()*100/total_work);

    //Handle overflows
    if(percent>100)
      percent=100;

    //In the case that there has been no update (which should never be the case,
    //actually), skip the expensive screen print
    if(percent==old_percent)
      return;

    //Update old_percent accordingly
    old_percent=percent;

    //Print an update string which looks like this:
    //  [================================================  ] (96% - 1.0s - 4 threads)
    std::cerr<<"\r\033[2K["
             <<std::string(percent/2, '=')<<std::string(50-percent/2, ' ')
             <<"] ("
             <<percent<<"% - "
             <<std::fixed<<std::setprecision(1)<<timer.lap()/percent*(100-percent)
             <<"s - "
             <<omp_get_num_threads()<< " threads)"<<std::flush;
};

progress_bar& progress_bar::operator++(){
    //Quick return if this isn't the main thread
    if(omp_get_thread_num()!=0)
    return *this;

    work_done++;
    update(work_done);
    return *this;
}

double progress_bar::stop(){
    clearConsoleLine();

    timer.stop();
    return timer.accumulated();
}

}


// int main(){
//   progress_bar pg;
//   pg.start(100);
//   //You should use 'default(none)' by default: be specific about what you're
//   //sharing
//   #pragma omp parallel for default(none) schedule(static) shared(pg)
//   for(int i=0;i<100;i++){
//     pg.update(i);
//     std::this_thread::sleep_for(std::chrono::seconds(1));
//   }
// }





// void util::progress_bar::print()
// {
//     //Quick return if this isn't the main thread
//     if(omp_get_thread_num()!=0)
//       return;

//     reset_line();

//     auto const ratio = float(_finished_work) / float(_total_work);
//     auto const position = int(ratio * _bar_width);

//     std::printf("[");
//     for (auto i = 0; i < _bar_width; ++i)
//     {
//         if (i < position)
//             std::printf("=");
//         else if (i == position)
//             std::printf(">");
//         else
//             std::printf(" ");
//     }
//     std::printf("] (%d/%d)", _finished_work, _total_work);
//     if (!_task_name.empty())
//         std::printf(" - (%s)", _task_name.c_str());
//     if (finished())
//         std::printf("\n");
//     std::fflush(stdout);
// }

// void util::progress_bar::reset_line()
// {
//     std::printf("\r");
//     // on ansi terminals we might want to use
//     //    std::printf("\r\033[2K[");
// }
