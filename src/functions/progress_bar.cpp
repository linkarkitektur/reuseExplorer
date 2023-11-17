#include "progress_bar.h"

#include <cstdio>

void util::progress_bar::print()
{
    reset_line();

    auto const ratio = float(_finished_work) / float(_total_work);
    auto const position = int(ratio * _bar_width);

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
    std::printf("] (%d/%d)", _finished_work, _total_work);
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
