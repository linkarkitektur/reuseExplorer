#include "algorithms/mcl.hh"
#include "types/surface.hh"

#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

#include <map>
#include <vector>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <array>
#include <fstream>

#include <embree3/rtcore.h>



using std::vector;
using std::map;


using Tile = std::pair<unsigned int, linkml::Surface *>;
using RayCollection = std::vector<std::pair<RTCRayHit, Tile*>>;


static void executeCommand(const char* cmd) {
    std::array<char, 128> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
    if (!pipe) {
        throw std::runtime_error("popen() failed!");
    }
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr)
        std::cout << buffer.data();
}

static void write_file(const std::string& filename, const Eigen::MatrixXd& matrix)
{
    // Assuming symmetrical matrix
    std::ofstream file(filename.c_str());
    if (file.is_open())
    {
        for (int i = 0; i < matrix.rows(); i++)
            for (int j = i; j < matrix.cols(); j++)
                if (matrix(i, j) > 0.000001)
                    file << i << " " << j << " " << matrix(i, j) << std::endl;
        file.close();
    }
    else
        std::cerr << "Unable to open file: " << filename << std::endl;
}

static void write_file(const std::string& filename, const RayCollection& rays){
    std::ofstream file(filename.c_str());
    if (file.is_open())
    {
        for (size_t i = 0; i < rays.size(); i++)
            if (rays[i].first.hit.geomID == RTC_INVALID_GEOMETRY_ID){
                int source = rays[i].second->first;
                int target = rays[i].first.ray.id;
                file << source  << " " << target<< " " << 1 << std::endl;
            }
        file.close();
    }
    else
        std::cerr << "Unable to open file: " << filename << std::endl;
}

static void read_results(const std::string& filename, std::function< void(size_t cls_i, size_t mem_j) > f){

    std::ifstream file(filename.c_str());
    if (file.is_open())
    {
        std::string line;
        size_t cls_i = 0;
        while (std::getline(file, line))
        {
            std::istringstream iss(line);
            int mem_j;
            
            while (iss >> mem_j) {
                f(cls_i, mem_j);
            }

            cls_i++;
        }
        file.close();
    }
    else
        std::cerr << "Unable to open file: " << filename << std::endl;

}


static std::string input_path = "./rays.in";
static std::string output_path = "./rays.out";





namespace mcl
{

    //! @brief MCL ctor. Register a callback function to return the cluster results
    template<>
    mcl<MCLAlgorithm::CLI, Eigen::MatrixXd>::mcl(Eigen::MatrixXd& Min, std::function< void(size_t cluster_j, size_t member_i) > f): Data(Min), ClusterResultCallback(f) {}

    template<>
    mcl<MCLAlgorithm::CLI, const RayCollection>::mcl(const RayCollection& rays, std::function< void(size_t cluster_j, size_t member_i) > f): Data(rays), ClusterResultCallback(f) {}

    /*! @brief Apply Markov clustering algorithm with specified parameters and return clusters
    For each cluster, returns the list of node-ids that belong to that cluster
    */
    template<>
    void mcl<MCLAlgorithm::CLI, Eigen::MatrixXd>::cluster_mcl(double expand_factor, double inflate_factor, double max_loop, double mult_factor)
    {
        try {
            write_file(input_path, Data);
            std::stringstream ss;
            int threads = 30;
            ss << "mcl " << input_path << " --abc -I " << inflate_factor << " -te " << threads << " -o " << output_path;
            std::string cmd = ss.str();
            
            executeCommand( cmd.c_str() );

            read_results(output_path, ClusterResultCallback);

        } catch (const std::exception& e) {
            std::cerr << "Error: " << e.what() << std::endl;
        }
    }
    template<>
    void mcl<MCLAlgorithm::CLI, const RayCollection>::cluster_mcl(double expand_factor, double inflate_factor, double max_loop, double mult_factor)
    {
        try {
            write_file(input_path, Data);
            std::stringstream ss;
            int threads = 30;
            ss << "mcl " << input_path << " --abc -I " << inflate_factor << " -te " << threads << " -o " << output_path;
            std::string cmd = ss.str();
            
            executeCommand( cmd.c_str() );

            read_results(output_path, ClusterResultCallback);

        } catch (const std::exception& e) {
            std::cerr << "Error: " << e.what() << std::endl;
        }
    }



}