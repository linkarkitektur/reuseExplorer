#pragma once
#include <types/Dataset.hh>
#include <string>
#include <cmath>
#include <optional>


namespace linkml{
    void parse_input_files(std::string const&  path, size_t start = 0, size_t step = 5, size_t n_frames = 0, bool inference = true);

    void parse_Dataset( 
        Dataset const & dataset, 
        std::string const & output_path,
        int start = 0,
        std::optional<int> stop = std::optional<int>{},
        int step = 5
        );
    
}