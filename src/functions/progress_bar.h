#pragma once

#include <clean-core/string.hh>
#include <clean-core/string_view.hh>

namespace util
{
struct progress_bar
{
public:
    progress_bar() = default;

    progress_bar(int total_work) : _total_work{total_work} {}

    progress_bar(int total_work, cc::string_view task_name) : _task_name{task_name}, _total_work{total_work} {}

    void update(int work = 1)
    {
        do_work(work);
        print();
    }

    void do_work(int work = 1) { _finished_work += work; }

    void print();

    bool finished() const { return _finished_work >= _total_work; }

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
    cc::string _task_name;
    int _total_work = 0;
    int _finished_work = 0;
    int _bar_width = 70;

private:
    void reset_line();
};
}
