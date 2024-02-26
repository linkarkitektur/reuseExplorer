#pragma once
#include <string>


namespace linkml{
    void parse_input_files(std::string const&  path, size_t start = 0, size_t step = 5, size_t n_frames = 0, bool inference = true);
}