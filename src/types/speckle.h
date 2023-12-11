#pragma once
#include <string>

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
    };
    
    
} // namespace linkml



