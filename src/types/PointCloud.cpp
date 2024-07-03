#include "PointCloud.hh"

#include "functions/polyscope.hh"
#include "functions/solidify.hh"
#include "functions/cluster_rooms.hh"


namespace linkml
{

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


    /// @brief Cluster and solidify the point cloud.
    std::vector<Brep> PointCloud::solidify(){
        polyscope::myinit();
        // auto clusters_out = linkml::cluster_rooms<PointCloud::Cloud::PointType>(cloud);
        // std::cout << "Number of clusters: " << clusters_out.size() << "\n";


        // this->save("output.pcd", true);

        pcl::io::loadPCDFile("output.pcd", *cloud);

        auto clusters = std::vector<pcl::PointIndices::Ptr>();
        auto cluster_map = std::unordered_map<int, pcl::PointIndices::Ptr>();

        for (size_t i = 0; i < cloud->points.size(); i++){
            int instance = cloud->points[i].instance;
            if (cluster_map.find(instance) == cluster_map.end())
                cluster_map[instance] = pcl::PointIndices::Ptr(new pcl::PointIndices());
            
            cluster_map[instance]->indices.push_back(i);
        }

        for (auto it = cluster_map.begin(); it != cluster_map.end(); it++)
            clusters.push_back(it->second);


        
        auto meshes = linkml::solidify<PointCloud::Cloud::PointType>(cloud, clusters);

        std::vector<Brep> breps;
        std::transform(meshes.begin(), meshes.end(), std::back_inserter(breps), [](Surface_mesh const& mesh){
            return Brep(mesh);
        });

        fmt::print("Number of breps: {}\n", breps.size());

        for (size_t i = 0; i < breps.size(); i++)
            breps[i].display("Mesh" + std::to_string(i));


        this->display("Cloud");

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
