#include "parse_input_files.hh"

#include <pcl/point_types.h>
#include <h5pp/h5pp.h>
#include <omp.h>
#include <string>


namespace linkml{
    void parse_input_files(std::string const& input_file, std::string const& output_file){

        // // Read the input file
        // h5pp::File file(input_file, h5pp::FilePermission::READONLY);
        // auto cloud = file.readDataset<pcl::PointCloud<pcl::PointXYZ>>("points");
        // auto normals = file.readDataset<pcl::PointCloud<pcl::Normal>>("normals");
        // auto colors = file.readDataset<pcl::PointCloud<pcl::PointXYZRGB>>("colors");

        // // Write the output file
        // h5pp::File file(output_file, h5pp::FilePermission::REPLACE);
        // file.writeDataset("points", cloud);
        // file.writeDataset("normals", normals);
        // file.writeDataset("colors", colors);
    }
}