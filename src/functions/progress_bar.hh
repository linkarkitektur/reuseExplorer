#pragma once

#include <iostream>

#include <clean-core/string.hh>
#include <clean-core/string_view.hh>
#include <types/Timer.hh>


namespace util
{


///@brief Manages a console-based progress bar to keep the user entertained.
///
///Defining the global `NOPROGRESS` will
///disable all progress operations, potentially speeding up a program. The look
///of the progress bar is shown in progress_bar.hpp.
struct progress_bar
{
public:
    progress_bar() = default;

    progress_bar(int total_work) : _total_work(total_work)/*, _call_diff(total_work/200)*/ {start();}

    progress_bar(int total_work, cc::string_view task_name) : _task_name(task_name), _total_work(total_work)/*, _call_diff(total_work/200)*/ {start();}

    void start() { _timer.start(); }

    ///Increment by one the work done and update the progress bar
    progress_bar& operator++() { do_work(1); return *this; }

    ///@brief Update the progress bar and increment the work done.
    void update(int work = 1)
    {
        do_work(work);
        print();
    }

    ///@brief Increment the work done.
    void do_work(int work = 1);

    ///@brief Update the visible progress bar, but only if enough work has been done.
    ///
    ///Define the global `NOPROGRESS` flag to prevent this from having an
    ///effect. Doing so may speed up the program's execution.
    void print();

    ///@return Return true if the progress bar has finished.
    bool finished() const { return _finished_work >= _total_work; }

    ///Stop the progress bar. Throws an exception if it wasn't started.
    ///@return The number of seconds the progress bar was running.
    std::chrono::nanoseconds stop();

    ///@return Return the time the progress bar ran for.
    std::chrono::nanoseconds time_it_took(){ return _timer.accumulated(); }


    // Setiing functions

    progress_bar& bar_width(int width)
    {
        _bar_width = width;
        return *this;
    }

    progress_bar& total_work(int total_work)
    {
        _total_work = total_work;
        return *this;
    }

    progress_bar& task_name(cc::string_view task_name)
    {
        _task_name = task_name;
        return *this;
    }

private:
    cc::string    _task_name      = "";     ///< Name of the task being performed
    uint32_t      _total_work     = 0;      ///< Total work to be accomplished
    uint32_t      _finished_work  = 0;      ///< Amount of work done
    uint32_t      _next_update    = 0;      ///< Next point to update the visible progress bar
    bool          _finalized      = false;  ///< True if the final line has been reutrn

    uint32_t      _bar_width      = 60;     ///< Width of the progress bar
    // uint32_t      _call_diff      = 1;    ///< Interval between updates in work units

    Timer         _timer;                   ///< Used for generating ETA


private:
    void reset_line();
};
}
