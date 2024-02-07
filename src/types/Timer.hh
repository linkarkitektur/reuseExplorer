#pragma once

#include <chrono>

namespace util
{
///@brief Used to time how intervals in code.
///
///Such as how long it takes a given function to run, or how long I/O has taken.
class Timer{
 private:
  typedef std::chrono::high_resolution_clock clock;
  typedef std::chrono::duration<double, std::ratio<1> > second;

  std::chrono::time_point<clock> start_time; ///< Last time the timer was started
  std::chrono::nanoseconds accumulated_time; ///< Accumulated running time since creation
  bool running;                              ///< True when the timer is running

 public:
  Timer(){
    accumulated_time = std::chrono::nanoseconds(0);
    running          = false;
  }

  ///Start the timer. Throws an exception if timer was already running.
  void start(){
    if(running)
      throw std::runtime_error("Timer was already started!");
    running=true;
    start_time = clock::now();
  }

  ///Stop the timer. Throws an exception if timer was already stopped.
  ///Calling this adds to the timer's accumulated time.
  ///@return The accumulated time in seconds.
  std::chrono::nanoseconds stop(){
    if(!running)
      throw std::runtime_error("Timer was already stopped!");

    accumulated_time += lap();
    running           = false;

    return accumulated_time;
  }

  ///Returns the timer's accumulated time. Throws an exception if the timer is
  ///running.
  std::chrono::nanoseconds accumulated(){
    if(running)
      throw std::runtime_error("Timer is still running!");
    return accumulated_time;
  }

  ///Returns the time between when the timer was started and the current
  ///moment. Throws an exception if the timer is not running.
  std::chrono::nanoseconds lap(){
    if(!running)
      throw std::runtime_error("Timer was not started!");
    return clock::now() - start_time;
    // return std::chrono::duration_cast<second> (clock::now() - start_time).count();
  }

  ///Stops the timer and resets its accumulated time. No exceptions are thrown
  ///ever.
  void reset(){
    accumulated_time = std::chrono::nanoseconds(0);
    running          = false;
  }
};
}