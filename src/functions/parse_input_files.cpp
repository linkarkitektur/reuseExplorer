
#define PCL_NO_PRECOMPILE
#include <pcl/memory.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree.h>
#include <pcl/common/impl/accumulators.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
//#include <pcl/segmentation/region_growing.h>

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
#include <types/Yolo.hh>
#include <types/Accumulators.hh>

#include <functions/polyscope.hh>
#include <polyscope/polyscope.h>
#include <polyscope/point_cloud.h>
#include <polyscope/curve_network.h>

#include <fmt/printf.h>
#include <fmt/color.h>

#include "parse_input_files.hh"
#include <types/Dataset.hh>
#include <functions/progress_bar.hh>

#include <Eigen/Dense>
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/ml.hpp>

#include <mutex>
#include <thread>
#include <future>


namespace fs = std::filesystem;


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
        PointCloud::Cloud::Ptr cloud_src, 
        const PointCloud::Cloud::Ptr cloud_tgt, 
        Eigen::Matrix4f &pairTransform, 
        std::optional<float> grid_size = {},
        std::optional<int32_t> confidence_filter = {}
        ){
        //
        // Downsample for consistency and speed
        // \note enable this for large datasets


        PointCloud::Cloud::Ptr input_src(new PointCloud::Cloud);
        PointCloud::Cloud::Ptr input_tgt(new PointCloud::Cloud);

        input_src = cloud_src;
        input_tgt = cloud_tgt; 

        PointCloud::Cloud::Ptr src(new PointCloud::Cloud);
        PointCloud::Cloud::Ptr tgt(new PointCloud::Cloud);
        

        if (confidence_filter.has_value()){
            pcl::ConditionAnd<PointT>::Ptr range_cond (new pcl::ConditionAnd<PointT> ());
            range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new
                pcl::FieldComparison<PointT>("confidence", pcl::ComparisonOps::EQ, confidence_filter.value())
                ));

            // build the filter
            pcl::ConditionalRemoval<PointT> condrem;
            condrem.setCondition(range_cond);
            condrem.setKeepOrganized(false);

            condrem.setInputCloud(input_src);
            condrem.filter(*src);

            condrem.setInputCloud(input_src);
            condrem.filter(*tgt);

            input_src = src;
            input_tgt = tgt; 
        }

        if (grid_size.has_value()){
            auto size = grid_size.value();
            pcl::VoxelGrid<PointT> grid;
            grid.setLeafSize(size, size, size);
            grid.setInputCloud (input_src);
            grid.filter (*src);

            grid.setInputCloud(input_tgt);
            grid.filter (*tgt);
        }

        // Compute point features
        // https://github.com/evil0sheep/pcl-slam/blob/master/src/pcl_slam.cpp


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

        PointCloud::Cloud::Ptr reg_result = src;
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

    template<typename PointCloud>
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

    template<typename PointCloud>
    void setHeader(PointCloud & cloud, size_t const i,  Eigen::MatrixXd const & odometry){

        uint64_t time_stamp = odometry(0,0);
        std::string frame_id = fmt::format("{:06}", (int)odometry(0,1));

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

    template<typename T>
    static T max_ocurance(std::vector<T> const& vec){
        std::map<T,int> m;
        int max = 0;
        int most_common = 0;
        for (auto const& vi : vec) {
            m[vi]++;
            if (m[vi] > max) {
                max = m[vi]; 
                most_common = vi;
            }
        }
        return most_common;
    }

    static size_t get_total_size(std::vector<PointCloud::Cloud::Ptr, Eigen::aligned_allocator<PointCloud::Cloud::Ptr>> const& clouds){
        size_t total_size = 0;
        for (auto const& cloud : clouds){
            total_size += cloud->size();
        }
        return total_size;
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


    void parse_Dataset( 
        Dataset const & dataset, 
        std::string const & output_path,
        int start,
        std::optional<int> stop_in,
        int step){

            // TODO: Make output path optional.
            // And in case it is not set use a temp folder
            //auto path = fs::temp_directory_path() / "linkml";
            
            // TODO: Optionally clear the output folder before saving the files
            //if (fs::exists(path))
            //    fs::remove_all(path);
            

            auto fields = dataset.fields();

            bool check = true;
            check &= std::count(fields.begin(), fields.end(), Field::COLOR) == 1;
            check &= std::count(fields.begin(), fields.end(), Field::DEPTH) == 1;
            check &= std::count(fields.begin(), fields.end(), Field::CONFIDENCE) == 1;
            check &= std::count(fields.begin(), fields.end(), Field::ODOMETRY) == 1;
            check &= std::count(fields.begin(), fields.end(), Field::POSES) == 1;

            if( !check){
                fmt::print(fg(fmt::color::red), "Dataset does not contain all required fields [COLOR, DEPTH, CONFIDENCE, ODOMETRY, POSES]");
                return;
            }


            int stop;
            if (stop_in.has_value()) stop = stop_in.value();
            else stop = dataset.size();

            if (start < 0) start = dataset.size() + start;
            if (stop < 0) stop = dataset.size() + stop;
            if (start < 0 || start >= dataset.size() || stop < 0 || stop > dataset.size() || step == 0){
                fmt::print(fg(fmt::color::red), "Invalid start, stop or step\n");
                return;
            }

            
            size_t n_frames = (stop - start) / step;
            double ratio = (double)n_frames / (double)dataset.size() * 100;

            // Print info
            fmt::print("Number of frames: ");
            fmt::print(fg(fmt::color::red), "{}", n_frames);
            fmt::print(fmt::emphasis::italic,  " -> ( start: {}; step: {}; end: {} ) {:3.2f}% \n", start, step, stop, ratio);

            if (n_frames == 0) return;


            auto progress_bar = util::progress_bar(n_frames,"Processing data");
            #pragma omp parallel for firstprivate(step, start, output_path) shared(dataset)
            for (size_t i = 0; i < n_frames; i++ ){

                size_t index = start + (i * step);
                auto data = dataset[index];

                auto cloud = PointCloud();
                setHeader(*cloud, i , data.get<Field::ODOMETRY>());
                depth_to_3d(
                            *cloud,
                            data.get<Field::DEPTH>(),
                            dataset.intrinsic_matrix(), 
                            data.get<Field::POSES>(), 
                            data.get<Field::COLOR>());
                setConficence(*cloud, data.get<Field::CONFIDENCE>());


                // Compute surface normals and curvature
                pcl::NormalEstimationOMP<PointT, PointT> ne;
                pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
                ne.setSearchMethod(tree);
                // ne.setRadiusSearch (0.05);
                ne.setKSearch(15);

                ne.setInputCloud(cloud);
                ne.compute(*cloud);

                // Save cloud
                std::filesystem::create_directory(output_path);
                std::string filename = fmt::format("{}/cloud_{}.pcd", output_path, cloud->header.frame_id);
                cloud.save(filename, true);


                progress_bar.update();
            }
            progress_bar.stop();

        }
 

} // namespace linkml

