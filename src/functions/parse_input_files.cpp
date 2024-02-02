
// #include <h5pp/h5pp.h>
// #include <omp.h>
// #include <string>
// #include <fstream>
// #include <filesystem>
// #include <fmt/printf.h>
// #include <optional>

// #include <vector>
// #include <any>

// #include <opencv4/opencv2/opencv.hpp>
// #include <functions/progress_bar.hh>

// #include <functions/lodepng.hh>

#define PCL_NO_PRECOMPILE
#include <pcl/memory.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/filters/filter.h>

#include <pcl/features/normal_3d_omp.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include <pcl/filters/conditional_removal.h>

#include <typed-geometry/tg-std.hh>
#include <typed-geometry/tg.hh>
#include <functions/depth_to_3d.hh>

#include <functions/polyscope.hh>
#include <polyscope/polyscope.h>
#include <polyscope/point_cloud.h>
#include <polyscope/curve_network.h>

#include <fmt/printf.h>
#include <fmt/color.h>

#include "parse_input_files.hh"
#include <types/dataset.hh>
#include <functions/progress_bar.hh>

#include <Eigen/Dense>

#include <mutex>
#include <thread>


namespace linkml{


    // Define a new point representation for < x, y, z, curvature >
    class MyPointRepresentation : public pcl::PointRepresentation<PointT>
    {
        using pcl::PointRepresentation<PointT>::nr_dimensions_;
        public:
        MyPointRepresentation ()
        {
            // Define the number of dimensions
            nr_dimensions_ = 3;
        }

        // Override the copyToFloatArray method to define our feature vector
        virtual void copyToFloatArray (const PointT &p, float * out) const
        {
            // < x, y, z, curvature >
            out[0] = p.x;
            out[1] = p.y;
            out[2] = p.z;
            // out[3] = p.curvature;
        }
    };

    ////////////////////////////////////////////////////////////////////////////////
    /** \brief Align a pair of PointCloud datasets and return the result
    * \param cloud_src the source PointCloud
    * \param cloud_tgt the target PointCloud
    * \param output the resultant aligned source PointCloud
    * \param final_transform the resultant transform between source and target
    */
    void pairAlign (
        PointCloud::Ptr cloud_src, 
        const PointCloud::Ptr cloud_tgt, 
        Eigen::Matrix4f &pairTransform, 
        bool downsample = false
        ){
        //
        // Downsample for consistency and speed
        // \note enable this for large datasets

        PointCloud::Ptr src(new PointCloud);
        PointCloud::Ptr tgt(new PointCloud);
        pcl::VoxelGrid<PointT> grid;

        if (downsample){
            auto grid_size = 0.1;
            grid.setLeafSize(grid_size, grid_size, grid_size);
            grid.setInputCloud (cloud_src);
            grid.filter (*src);

            grid.setInputCloud(cloud_tgt);
            grid.filter (*tgt);
        }
        else{
            src = cloud_src;
            tgt = cloud_tgt;
        }


        //
        // Instantiate our custom point representation (defined above) ...
        MyPointRepresentation point_representation;

        // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
        float alpha[4] = {1.0, 1.0, 1.0, 1.0};
        point_representation.setRescaleValues (alpha);

        //
        // Align
        pcl::IterativeClosestPoint<PointT,PointT> reg; //NonLinear
        reg.setTransformationEpsilon (1e-8);

        // Set the maximum distance between two correspondences (src<->tgt) to 10cm
        // Note: adjust this based on the size of your datasets
        reg.setMaxCorrespondenceDistance(0.05);  

        // Set the point representation
        reg.setPointRepresentation(pcl::make_shared<const MyPointRepresentation>(point_representation));

        reg.setInputSource(src);
        reg.setInputTarget(tgt);

        //
        // Run the same optimization in a loop and visualize the results

        PointCloud::Ptr reg_result = src;
        reg.setMaximumIterations(50);
        reg.setRANSACIterations(1000);
        reg.setRANSACOutlierRejectionThreshold(0.05);
        reg.align(*reg_result);
        if (reg.hasConverged())
        {
            pairTransform = reg.getFinalTransformation().inverse();
            pcl::transformPointCloudWithNormals(*cloud_src,*cloud_src, pairTransform);
        }

    }

    void setConficence(PointCloud & cloud, Eigen::MatrixXd const confidences){

        if (confidences.size() != cloud.size())
            std::cout << "Confidences size is not equal to cloud size" << std::endl;

        #pragma omp parallel for collapse(2) shared(cloud, confidences)
        for (int row = 0; row < confidences.rows(); row++){
            for (int col = 0; col < confidences.cols(); col++){
                size_t index = row * confidences.cols() + col;
                cloud[index].confidence = confidences(row,col);
            }
        }
    }

    void filterConfidences(PointCloud::Ptr cloud, int threshold = 2){

            pcl::ConditionAnd<PointT>::Ptr range_cond (new
                            pcl::ConditionAnd<PointT> ());
            range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new
                pcl::FieldComparison<PointT>("confidence", pcl::ComparisonOps::GE, threshold)));

            // build the filter
            pcl::ConditionalRemoval<PointT> condrem;
            condrem.setCondition(range_cond);
            condrem.setInputCloud(cloud);
            condrem.setKeepOrganized(false);
            condrem.filter(*cloud);
    }

    void setHeader(PointCloud & cloud, size_t const i,  Eigen::MatrixXd const & odometry){

        uint64_t time_stamp = odometry(0,0);
        std::string frame_id = std::to_string(odometry(0,0));

        auto pos = odometry.block<1,3>(0,2);  // x,y,z
        auto quat = odometry.block<1,4>(0,5); // x,y,z,w

        Eigen::Vector4f origin = Eigen::Vector4f(0, 0, 0, 0);
        origin.x() = pos(0);
        origin.y() = pos(1);
        origin.z() = pos(2);

        Eigen::Quaternionf orientation = Eigen::Quaternionf::Identity();
        orientation.x() = quat(0);
        orientation.y() = quat(1);
        orientation.z() = quat(2);
        orientation.w() = quat(3);

        cloud.header.seq = i;
        cloud.header.frame_id = frame_id;
        cloud.header.stamp = time_stamp;

        cloud.sensor_origin_[0] = origin[0];
        cloud.sensor_origin_[1] = origin[1];
        cloud.sensor_origin_[2] = origin[2];
        cloud.sensor_origin_[3] = origin[3];

        cloud.sensor_orientation_ = orientation;

    }

    std::string print_matrix(Eigen::MatrixXd const& matrix, int n_rows = 10, int n_cols = 10, int precision = 3){

        // FIXME: Cell width is not correct
        // Example:
        // Matrix: 256x192
        //     |     0     1     2     3     4   ...  187   188   189   190   191 
        //     | ------------------------------------------------------------------
        //    0| 0.0234 0.0195 0.0195 0.0195 0.0195   ...0.0234 0.0234 0.0234 0.0234 0.0234 
        //    1| 0.0234 0.0234 0.0234 0.0234 0.0234   ...0.0234 0.0234 0.0234 0.0234 0.0234 
        //    2| 0.0234 0.0234 0.0234 0.0234 0.0234   ...0.0234 0.0234 0.0234 0.0234 0.0234 
        //    3| 0.0234 0.0234 0.0234 0.0234 0.0234   ...0.0156 0.0156 0.0156 0.0156 0.0156 
        //    4| 0.0234 0.0195 0.0195 0.0234 0.0234   ...0.0234 0.0234 0.0234 0.0234 0.0234 
                    
        //  251| 0.0195 0.0195 0.0195 0.0195 0.0195   ...0.0195 0.0195 0.0195 0.0195 0.0195 
        //  252| 0.0195 0.0195 0.0195 0.0195 0.0195   ...0.0234 0.0234 0.0234 0.0234 0.0234 
        //  253| 0.0234 0.0234 0.0234 0.0273 0.0273   ...0.0195 0.0195 0.0195 0.0195 0.0195 
        //  254| 0.0195 0.0195 0.0195 0.0195 0.0195   ...0.0195 0.0195 0.0195 0.0195 0.0195 
        //  255| 0.0195 0.0195 0.0195 0.0195 0.0195   ...0.0195 0.0195 0.0195 0.0195 0.0195 


        int cols = matrix.cols();
        int rows = matrix.rows();

        auto columns_indexies = std::vector<int>();
        auto row_indexies = std::vector<int>();

        if (cols > n_cols){
            int n_cols_half = n_cols / 2;

            // First half
            for (int i = 0; i < n_cols_half; i++){
                columns_indexies.push_back(i);
            }

            // Sepparator
            columns_indexies.push_back(-1);

            // Second half
            for (int i = cols - n_cols_half; i < cols; i++){
                columns_indexies.push_back(i);
            }
        }
        else {
            for (int i = 0; i < cols; i++){
                columns_indexies.push_back(i);
            }
        }

        if (rows > n_rows){
            int n_rows_half = n_rows / 2;

            // First half
            for (int i = 0; i < n_rows_half; i++){
                row_indexies.push_back(i);
            }
            // Sepparator
            row_indexies.push_back(-1);

            // Second half
            for (int i = rows - n_rows_half; i < rows; i++){
                row_indexies.push_back(i);
            }
        }
        else {
            for (int i = 0; i < rows; i++){
                row_indexies.push_back(i);
            }
        }

        std::stringstream ss;

        ss << "Matrix: " << rows << "x" << cols << "\n";

        //Header
        ss << std::right << std::setw(7) << " " << "| ";
        for (auto const& col_index : columns_indexies){
            if (col_index == -1){
                ss << std::setw(5) << "...";
                continue;
            }
            ss << std::setw(precision+2) << std::setprecision(precision) << col_index << " ";
        }
        ss << "\n";

        // Sepparator
        ss << std::right << std::setw(7) << " " << "| ";
        for (auto const& col_index : columns_indexies){
            ss << std::setw(precision+2) << std::setprecision(precision) << "-----"  << "-";
        }
        ss << "\n";

        // Body
        for (auto const& row_index : row_indexies){


            // Row index
            if (row_index == -1) ss << " "; else  ss << std::right << std::setw(7) << row_index << "| ";

            // Cells
            for (auto const& col_index : columns_indexies){

                if (row_index == -1){
                    ss << " ";
                    continue;
                }

                if (col_index == -1){
                    ss << std::setw(5) << "...";
                    continue;
                }
                // FIXME: Cell width is not correct 

                ss << std::setw(precision+2) << std::setprecision(precision) << matrix(row_index, col_index) << " ";
            }
            
            // End of Row
            ss << "\n";
        }

        return ss.str();

    }

    template<typename T>
    void print_matrix(Eigen::Matrix<T,4,4> const & matrix){
        fmt::printf ("Rotation matrix :\n");
        fmt::printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
        fmt::printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
        fmt::printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
        fmt::printf ("Translation vector :\n");
        fmt::printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
    }

    void parse_input_files(std::string const& path, size_t start, size_t step, size_t n_frames){
  
        // Load dataset
        auto dataset = Dataset(path, {Field::COLOR, Field::DEPTH, Field::CONFIDENCE, Field::ODOMETRY, /*Field::IMU,*/ Field::POSES});
        auto camera_intrisic_matrix = dataset.intrinsic_matrix();

        // Check config
        // TODO: This function needs to been cleaned up
        if (start > dataset.size()) start = dataset.size()-1;
        if (n_frames == 0) n_frames = dataset.size();
        if (start + ((n_frames -1) * step > dataset.size())) n_frames = (dataset.size() - start) / step;

        double ratio = (double)n_frames / (double)dataset.size() * 100;
        size_t end;
        if (n_frames != 0) end = start + ((n_frames -1) * step);
        else  end = start;

        // Print info
        fmt::print("Number of frames: ");
        fmt::print(fg(fmt::color::red), "{}", n_frames);
        fmt::print(fmt::emphasis::italic,  " -> ( start: {}; step: {}; end: {} ) {:3.2f}% \n", start, step, end, ratio);

        // Because of custom print functions
        if (n_frames == 0) return;
        
        // Init Polyscope
        polyscope::init();
        polyscope::options::groundPlaneMode = polyscope::GroundPlaneMode::ShadowOnly;
        polyscope::view::setUpDir(polyscope::UpDir::YUp);


        // Setup voxel grid
        pcl::VoxelGrid<PointT> grid;
        auto grid_size = 0.02;
        grid.setLeafSize(grid_size, grid_size, grid_size);



        // Load data
        ///////////////////////////////////////////////////////////////////////////////
        auto  point_clouds = std::vector<PointCloud::Ptr, Eigen::aligned_allocator<PointCloud::Ptr>>(n_frames);
        auto ld_bar = util::progress_bar(n_frames,"Loading data");
        #pragma omp parallel for firstprivate(step, start) shared(point_clouds)
        for (size_t i=0; i< point_clouds.size(); i++){

            size_t index = start + (i * step);
            auto data = dataset[index];

            point_clouds[i] = PointCloud::Ptr(new PointCloud);
            
            setHeader(*point_clouds[i], i , data.get<Field::ODOMETRY>());

            depth_to_3d(
                        *point_clouds[i],
                        data.get<Field::DEPTH>(),
                        camera_intrisic_matrix, 
                        data.get<Field::POSES>(), 
                        data.get<Field::COLOR>());
            
            setConficence(*point_clouds[i], data.get<Field::CONFIDENCE>());
            ld_bar.update();
        }

        polyscope::display(*point_clouds[0], "Cloud Pre ICP");

        // Filter
        ///////////////////////////////////////////////////////////////////////////////
        auto filter_bar = util::progress_bar(n_frames,"Filtering data");
        #pragma omp parallel for shared(point_clouds)
        for (size_t i = 0; i < point_clouds.size(); i++){
            
            size_t j = 0;
            for (size_t k = 0; k < point_clouds[i]->size(); k++){
                if (point_clouds[i]->at(k).confidence >= 2){
                    point_clouds[i]->at(j) = point_clouds[i]->at(k);
                    j++;
                }
            }

            point_clouds[i]->resize(j);
            point_clouds[i]->width = j;
            point_clouds[i]->height = 1;
            point_clouds[i]->is_dense = false;

            // // build rules
            // pcl::ConditionAnd<PointT>::Ptr range_cond (new pcl::ConditionAnd<PointT> ());
            // range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new
            //     pcl::FieldComparison<PointT>("confidence", pcl::ComparisonOps::EQ, 2)));


            // // build the filter
            // pcl::ConditionalRemoval<PointT> condrem;
            // condrem.setCondition(range_cond);
            // condrem.setInputCloud(point_clouds[i]);
            // condrem.setKeepOrganized(false);
            // condrem.filter(*point_clouds[i]);

            // // filterConfidences(point_clouds[i], 2);
            filter_bar.update();
        }



        // Compute Normals
        ///////////////////////////////////////////////////////////////////////////////
        auto n_bar = util::progress_bar(n_frames,"Computing normals");
        #pragma omp parallel for shared(point_clouds)
        for (size_t i=0; i < point_clouds.size(); ++i){
            // Compute surface normals and curvature
            pcl::NormalEstimationOMP<PointT, PointT> ne;
            pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
            ne.setSearchMethod(tree);
            // ne.setRadiusSearch (0.05);
            ne.setKSearch(15);

            ne.setInputCloud(point_clouds[i]);
            ne.compute(*point_clouds[i]);
            n_bar.update();
        }

        // // Remove NaN Normals
        // ///////////////////////////////////////////////////////////////////////////////
        // for (size_t i=0; i < point_clouds.size(); ++i){
        //     std::vector<int> indices;
        //     pcl::removeNaNNormalsFromPointCloud(*point_clouds[i].cloud, *point_clouds[i].cloud, indices);
        // }

        // Registartion
        ///////////////////////////////////////////////////////////////////////////////
        auto transforms = std::vector<Eigen::Matrix4f>(point_clouds.size(), Eigen::Matrix4f::Identity());
        auto reg_bar = util::progress_bar(n_frames,"Registration");
        reg_bar.update();
        #pragma omp parallel for shared(point_clouds, transforms)
        for (std::size_t i = 1; i < point_clouds.size() ; ++i){

            PointCloud::Ptr target = point_clouds[i-1];
            PointCloud::Ptr source = point_clouds[i];
            Eigen::Matrix4f pairTransform;

            // ICP
            pairAlign(source, target, pairTransform, true);
            transforms[i] = pairTransform;
            reg_bar.update();
        }
        
        std::cout << "Transforms: " << std::endl;

        // Update the position of all point clouds
        Eigen::Matrix4f global_transform = Eigen::Matrix4f::Identity();
        auto move_bar = util::progress_bar(n_frames,"Moving clouds");
        move_bar.update();
        for (std::size_t i = 1; i < point_clouds.size() ; ++i){
            global_transform *= transforms[i];
            pcl::transformPointCloudWithNormals(*point_clouds[i], *point_clouds[i], global_transform);
            move_bar.update();
        }

        std::cout << "\nDone Transforms: " << std::endl;


        polyscope::display(*point_clouds[0], "Cloud Post ICP");

        // Display
        ///////////////////////////////////////////////////////////////////////////////
        PointCloud::Ptr temp2 (new PointCloud);
        auto pdp_bar2 = util::progress_bar(n_frames,"Displaying data");
        // #pragma omp parallel for shared(point_clouds)
        for (size_t i = 0; i < point_clouds.size(); i++){
            PointCloud::Ptr temp3(new PointCloud);
            grid.setInputCloud(point_clouds[i]);
            grid.filter(*temp3);
            #pragma omp critical
            *temp2 += *temp3;
            pdp_bar2.update();
        }


        grid.setInputCloud(temp2);
        grid.filter(*temp2);

        polyscope::display(*temp2, "Cloud");
    
        polyscope::show();

        // pcl::io::savePCDFileBinaryCompressed("TEST", *point_clouds[0].cloud);

    }

}

