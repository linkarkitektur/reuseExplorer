#pragma once
#include <string>
#include <pybind11/embed.h>

#include <types/point_cloud.hh>

namespace py = pybind11;

namespace linkml
{
    class speckle
    {
    private:
        std::string status;
    public:
        speckle(std::string const & stream_url, std::string const & token);
        ~speckle();

        std::string get_status(){ return status;}

        py::object ToSpeckle(linkml::point_cloud const & cloud);

    };
    
    
} // namespace linkml



