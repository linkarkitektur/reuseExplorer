#include <types/speckle.h>
#include <pybind11/embed.h>
#include <iostream>

namespace py = pybind11;


linkml::speckle::speckle(std::string const & stream_url, std::string const & token){

    if ( Py_IsInitialized() == 0 ) {
        //interpreter is not running, do you work
        // py::scoped_interpreter python;
        py::scoped_interpreter guard{};
    }

    auto StreamWrapper = py::module_::import("specklepy.api").attr("wrapper").attr("StreamWrapper");

    auto wrapper = StreamWrapper(stream_url);
    auto client = wrapper.attr("get_client")(token);
    status = client.attr("__repr__")().cast<std::string>();

    client.release();

}

linkml::speckle::~speckle()
{
}