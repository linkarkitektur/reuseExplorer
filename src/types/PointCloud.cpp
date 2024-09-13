#include "types/PointCloud.hh"

#include "functions/progress_bar.hh"
#include "functions/polyscope.hh"
#include "functions/downsample.hh"
#include "functions/solidify.hh"
#include "functions/cluster_rooms.hh"

// #include <fmt/printf.h>

namespace linkml {

    /// Constructors
    /// @brief Load the point cloud from a file.
    PointCloud::PointCloud(std::string const & filename ) : Cloud::Ptr(new Cloud())
    {
        //Cloud::Ptr cloud = *this;
        pcl::io::loadPCDFile(filename, *cloud);
    }

    /// Load helper functions
    /// @brief Load the point cloud from a file.
    PointCloud PointCloud::load(std::string const& filename){

        std::filesystem::path pcd = filename;
        PointCloud cloud = PointCloud();
        
        pcl::io::loadPCDFile(pcd, *cloud);

        cloud->header = load_header(filename);

        return cloud;
    }

    /// @brief Save the point cloud to a file.
    void  PointCloud::save(std::string const& filename, bool binary) const {
        pcl::io::savePCDFile(filename, **this, binary);

        // Save header information
        std::filesystem::path p(filename);
        p.replace_extension(".head");
        std::filesystem::exists(p) ? std::filesystem::remove(p) : std::filesystem::create_directories(p.parent_path());
        std::ofstream header_file(p);
        if (header_file.is_open()){
            header_file << cloud->header.frame_id << std::endl;
            header_file << cloud->header.stamp << std::endl;
            header_file << cloud->header.seq << std::endl;
            header_file.close();
        }
    }


    /// @brief Load the header from a file.
    pcl::PCLHeader PointCloud::load_header(std::string const& filename) {
        pcl::PCLHeader header;

        std::filesystem::path head = filename;
        head.replace_extension(".head");

        if (std::filesystem::exists(head)){
            std::ifstream header_file(head);
            if (header_file.is_open()){
                header_file >> header.frame_id;
                header_file >> header.stamp;
                header_file >> header.seq;
                header_file.close();
            }
        }
        return header;
    }

    /// @brief Get the bounding box of the point cloud.
    tg::aabb3  PointCloud::get_bbox() const {
        float x_min = std::numeric_limits<float>::max();
        float y_min = std::numeric_limits<float>::max();
        float z_min = std::numeric_limits<float>::max();
        float x_max = std::numeric_limits<float>::min();
        float y_max = std::numeric_limits<float>::min();
        float z_max = std::numeric_limits<float>::min();

        #pragma omp parallel for reduction(min:x_min, y_min, z_min) reduction(max:x_max, y_max, z_max)
        for (size_t i = 0; i < cloud->size(); i++){
            auto p = cloud->at(i);
            if (p.x < x_min) x_min = p.x;
            if (p.y < y_min) y_min = p.y;
            if (p.z < z_min) z_min = p.z;
            if (p.x > x_max) x_max = p.x;
            if (p.y > y_max) y_max = p.y;
            if (p.z > z_max) z_max = p.z;
        }

        return tg::aabb3(tg::pos3(x_min, y_min, z_min), tg::pos3(x_max, y_max, z_max));

    }

    /// @brief Transform the point cloud.
    void PointCloud::downsample(double leaf_size){
        auto bar = util::progress_bar(1, "Downsampling");
        linkml::downsample(*this, leaf_size);
        bar.stop();
    }

    /// @brief Cluster and solidify the point cloud.
    std::vector<Brep> PointCloud::solidify(
        unsigned int downsample_size,
        double sx,
        double sy,
        double expand_factor, 
        double inflate_factor, 
        double max_loop, 
        double mult_factor,
        double fitting,
        double coverage,
        double complexity
    ){


        auto cloud_copy = PointCloud::Cloud::Ptr(new PointCloud::Cloud(*cloud));

        auto clusters = linkml::cluster_rooms<PointCloud::Cloud::PointType>(
            cloud_copy, 
            downsample_size, 
            sx, 
            sy,
            expand_factor, 
            inflate_factor, 
            max_loop, 
            mult_factor
        );
        std::cout << "Number of clusters: " << clusters.size() << "\n";


        // std::ofstream file_out("clusters.txt");
        // for (size_t i = 0; i < clusters.size(); i++){
        //     for (size_t j = 0; j < clusters[i]->indices.size(); j++)
        //         file_out << clusters[i]->indices[j] << " ";
        //     file_out << std::endl;
        // }
        // file_out.close();


        // std::ifstream file_in("clusters.txt");
        // std::vector<pcl::PointIndices::Ptr> clusters;
        // std::string line;
        // while (std::getline(file_in, line)){
        //     std::istringstream iss(line);
        //     pcl::PointIndices::Ptr cluster(new pcl::PointIndices());
        //     int index;
        //     while (iss >> index)
        //         cluster->indices.push_back(index);
        //     clusters.push_back(cluster);
        // }
        // file_in.close();



        // polyscope::myinit();
        // this->display("Cloud");
        
        auto meshes = linkml::solidify<PointCloud::Cloud::PointType>(cloud, clusters, fitting, coverage, complexity);

        std::vector<Brep> breps;
        std::transform(meshes.begin(), meshes.end(), std::back_inserter(breps), [](Surface_mesh const& mesh){
            return Brep(mesh);
        });

        printf("Number of breps: {}\n", breps.size());

        // for (size_t i = 0; i < breps.size(); i++)
        //     breps[i].display("Mesh" + std::to_string(i));

        // std::cout << "Displaying cloud\n";

        // this->display("Cloud");

        return breps;
    }

    /// @brief Cluster and solidify the point cloud.
    PointCloud::Cloud::PointType PointCloud::at(size_t x, size_t y) {
        return cloud->points.at(y * cloud->width + x);
    }

    /// @brief get fields as image
    cv::Mat PointCloud::image(std::string field_name) const {

        if (!cloud->is_dense)
            throw std::runtime_error("Clouds must be dense");

        cv::Mat img = cv::Mat::zeros(cloud->height, cloud->width, CV_8UC3);

        if ( field_name == "rgb" ){
        #pragma omp parallel for shared(img)
        for (size_t i = 0; i < cloud->size(); i++){
            auto point = cloud->at(i);
            size_t y = i % cloud->width;
            size_t x = i / cloud->width;
            img.at<cv::Vec3b>(x, y) = cv::Vec3b(point.r, point.g, point.b);
        }
        }
        else if ( field_name == "semantic"){
        #pragma omp parallel for shared(img)
        for (size_t i = 0; i < cloud->size(); i++){
            auto point = cloud->at(i);
            size_t y = i % cloud->width;
            size_t x = i / cloud->width;
            auto c = linkml::get_color_forom_angle(linkml::sample_circle(point.semantic));
            img.at<cv::Vec3b>(x, y) = cv::Vec3b(c.r * 255, c.g * 255, c.b * 255);
        }
        }
        else {
        PCL_WARN ("Filed not implemented");
        }


        return img;
    }

    /// @brief Display the point cloud.
    void PointCloud::display(std::string name) const {

        polyscope::myinit();
        polyscope::display<pcl::PointCloud<PointT> const&>(*cloud, name);
        polyscope::show();
    }

} // namespace linkml
