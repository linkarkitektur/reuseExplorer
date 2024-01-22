#pragma once

#include <iostream>

#include <clean-core/string.hh>
#include <clean-core/string_view.hh>
#include <types/Timer.hh>




// TODO: Consider making the progress_bar support multithreading.
// https://stackoverflow.com/questions/28050669/can-i-report-progress-for-openmp-tasks
namespace util
{


///@brief Manages a console-based progress bar to keep the user entertained.
///
///Defining the global `NOPROGRESS` will
///disable all progress operations, potentially speeding up a program. The look
///of the progress bar is shown in progress_bar.hpp.
class progress_bar{
 private:
  std::string task_name;  ///< Name of the task being performed
  uint32_t total_work;    ///< Total work to be accomplished
  uint32_t next_update;   ///< Next point to update the visible progress bar
  uint32_t call_diff;     ///< Interval between updates in work units
  uint32_t work_done;
  uint16_t old_percent;   ///< Old percentage value (aka: should we update the progress bar) TODO: Maybe that we do not need this
  Timer    timer;         ///< Used for generating ETA


  ///Clear current line on console so a new progress bar can be written
  void clearConsoleLine() const {
    std::cerr<<"\r\033[2K"<<std::flush;
  }

 public:

  progress_bar(uint32_t total_work, std::string task_name = "")
      : task_name(task_name) {this->start(100);}

  ///@brief Start/reset the progress bar.
  ///@param total_work  The amount of work to be completed, usually specified in cells.
  void start(uint32_t total_work){
    timer = Timer();
    timer.start();
    this->total_work = total_work;
    next_update      = 0;
    call_diff        = total_work/200;
    old_percent      = 0;
    work_done        = 0;
    clearConsoleLine();
  }

  ///@brief Update the visible progress bar, but only if enough work has been done.
  ///
  ///Define the global `NOPROGRESS` flag to prevent this from having an
  ///effect. Doing so may speed up the program's execution.
  void update(uint32_t work_done0);

  ///Increment by one the work done and update the progress bar
  progress_bar& operator++();

  ///Stop the progress bar. Throws an exception if it wasn't started.
  ///@return The number of seconds the progress bar was running.
  double stop();

  ///@return Return the time the progress bar ran for.
  double time_it_took(){
    return timer.accumulated();
  }

  uint32_t cellsProcessed() const {
    return work_done;
  }
};

// struct progress_bar
// {
// public:
//     progress_bar() = default;

//     progress_bar(int total_work) : _total_work{total_work} {}

//     progress_bar(int total_work, cc::string_view task_name) : _task_name{task_name}, _total_work{total_work} {}

//     void update(int work = 1)
//     {
//         do_work(work);
//         print();
//     }

//     void do_work(int work = 1) { _finished_work += work; }

//     void print();

//     bool finished() const { return _finished_work >= _total_work; }

//     progress_bar& bar_width(int width)
//     {
//         _bar_width = width;
//         return *this;
//     }

//     progress_bar& total_work(int total_work)
//     {
//         _total_work = total_work;
//         return *this;
//     }

//     progress_bar& task_name(cc::string_view task_name)
//     {
//         _task_name = task_name;
//         return *this;
//     }

// private:
//     cc::string _task_name;
//     int _total_work = 0;
//     int _finished_work = 0;
//     int _bar_width = 70;

// private:
//     void reset_line();
// };
}
