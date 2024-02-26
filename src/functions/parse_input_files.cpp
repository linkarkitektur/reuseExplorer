
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
#include <pcl/octree/octree.h>
#include <pcl/common/impl/accumulators.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/segmentation/region_growing.h>

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
#include <types/accumulators.hh>

#include <functions/polyscope.hh>
#include <polyscope/polyscope.h>
#include <polyscope/point_cloud.h>
#include <polyscope/curve_network.h>

#include <fmt/printf.h>
#include <fmt/color.h>

#include "parse_input_files.hh"
#include <types/dataset.hh>
#include <functions/progress_bar.hh>
#include <algorithms/surface_reconstruction.hh>

#include <Eigen/Dense>
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/ml.hpp>

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
        std::optional<float> grid_size = {},
        std::optional<uint8_t> confidence_filter = {}
        ){
        //
        // Downsample for consistency and speed
        // \note enable this for large datasets


        PointCloud::Ptr input_src(new PointCloud);
        PointCloud::Ptr input_tgt(new PointCloud);

        input_src = cloud_src;
        input_tgt = cloud_tgt; 

        PointCloud::Ptr src(new PointCloud);
        PointCloud::Ptr tgt(new PointCloud);
        

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

    static size_t get_total_size(std::vector<PointCloud::Ptr, Eigen::aligned_allocator<PointCloud::Ptr>> const& clouds){
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

    void parse_input_files(std::string const& path, size_t start, size_t step, size_t n_frames, bool inference){
  
        // Load dataset
        auto yolo_async =std::async([](){ return Yolov8Seg("./yolov8x-seg.onnx", true);});
        auto dataset = Dataset(path, {Field::COLOR, Field::DEPTH, Field::CONFIDENCE, Field::ODOMETRY, /*Field::IMU,*/ Field::POSES});
        auto camera_intrisic_matrix = dataset.intrinsic_matrix();

        // Check config
        // TODO: This function needs to been cleaned up
        if (start > dataset.size()) start = dataset.size()-1;
        if (n_frames == 0) n_frames = dataset.size();
        if (start + ((n_frames -1) * step) > dataset.size()) n_frames = (dataset.size() - start) / step;

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


        // Track runtimes
        ///////////////////////////////////////////////////////////////////////////////
        auto start_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::nanoseconds(0);


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
        duration += ld_bar.stop();


        // Run Inference
        ///////////////////////////////////////////////////////////////////////////////
        if (inference){
            // auto model = Yolov8Seg("./yolov8x-seg.onnx", false);
            auto model = yolo_async.get();

            std::vector<cv::Mat> blobs;
            blobs.resize(n_frames);
            std::vector<cv::Vec4d> params;
            params.resize(n_frames);
            std::vector<std::vector<cv::Mat>> outputs;
            outputs.resize(n_frames);
            std::vector<cv::Mat> source_images;
            source_images.resize(n_frames);


            // Factering out 
            auto infrence_precrocessing_bar = util::progress_bar(n_frames,"Preprocessing");
            #pragma omp parallel for shared(dataset, blobs, params)
            for (size_t i = 0; i < point_clouds.size(); i++){
                size_t index = start + (i * step);
                auto image = dataset[index].get<Field::COLOR>();
                cv::rotate(image, image, cv::ROTATE_90_CLOCKWISE); // <- Image in video is sideways

                source_images[i] = image;
                
                Yolov8Seg::Preprocess(image, blobs[i], params[i]);
            }
            duration += infrence_precrocessing_bar.stop();

            // The neural network is not thread safe
            // _At least I am not able to make it thread safe_
            auto inference_bar = util::progress_bar(n_frames,"Running Inference");
            for (size_t i = 0; i < point_clouds.size(); i++){
                outputs[i] = model.Detect(blobs[i]);
                inference_bar.update();
            }
            duration +=  inference_bar.stop();

            auto inference_postprocessing_bar = util::progress_bar(n_frames,"Postprocessing");
            #pragma omp parallel for shared(point_clouds, outputs, params)
            for (size_t i = 0; i < point_clouds.size(); i++){

                auto results = Yolov8Seg::Postprocess(outputs[i], params[i], source_images[i]);

                auto color_size = dataset.color_size();
                auto depth_size = dataset.depth_size();
                auto h = color_size.height;
                auto w = color_size.width;
                auto color_size_right_side_up = cv::Size(h, w);

                for (OutputParams const& param: results){

                    auto param_rotated = param.Rotate<cv::ROTATE_90_COUNTERCLOCKWISE>(color_size_right_side_up).Scale(color_size, depth_size);

                    auto row_start = param_rotated.box.y;
                    auto row_end = param_rotated.box.y + param_rotated.box.height;

                    auto col_start = param_rotated.box.x;
                    auto col_end = param_rotated.box.x + param_rotated.box.width;

                    #pragma omp parallel for collapse(2) shared(point_clouds, param_rotated)
                    for (int row = row_start; row < row_end; row++){
                        for (int col = col_start; col < col_end; col++){
                            size_t index = row * depth_size.width + col;

                            if (param_rotated.boxMask.at<uchar>(row - row_start, col - col_start) > 0.1)
                                point_clouds[i]->points[index].semantic = param_rotated.id;
                        }
                    }

                }
                
                inference_postprocessing_bar.update();
            }
            duration +=  inference_postprocessing_bar.stop();


            blobs.clear();
            params.clear();
            outputs.clear();
            source_images.clear();

            // auto inference_bar = util::progress_bar(n_frames,"Running Inference");
            // for (size_t i = 0; i < point_clouds.size(); i++){
            //     size_t index = start + (i * step);
            //     auto color = dataset[index].get<Field::COLOR>();
            //     cv::rotate(color, color, cv::ROTATE_90_CLOCKWISE); // <- Image in video is sideways
            //     auto output = model.Detect(color);
            //     if (!output.has_value()) { inference_bar.update(); continue;}
            //     auto results = output.value();
            //     cv::Mat sideways;
            //     cv::rotate(color, sideways, cv::ROTATE_90_COUNTERCLOCKWISE);
            //     cv::resize(sideways, sideways, dataset.depth_size());
            //     for (OutputParams const& param: results){
            //         auto param_rotated = param.Rotate<cv::ROTATE_90_COUNTERCLOCKWISE>(color.size()).Scale(dataset.color_size(), dataset.depth_size());
            //         auto row_start = param_rotated.box.y;
            //         auto row_end = param_rotated.box.y + param_rotated.box.height;
            //         auto col_start = param_rotated.box.x;
            //         auto col_end = param_rotated.box.x + param_rotated.box.width;
            //         #pragma omp parallel for collapse(2) shared(point_clouds, param_rotated)
            //         for (int row = row_start; row < row_end; row++){
            //             for (int col = col_start; col < col_end; col++){
            //                 size_t index = row * dataset.depth_size().width + col;
            //                 if (param_rotated.boxMask.at<uchar>(row - row_start, col - col_start) > 0.1)
            //                     point_clouds[i]->points[index].semantic = param_rotated.id;
            //             }
            //         }
            //     }
            //     // Run Inference
            //     inference_bar.update();
            // }
            // duration +=  inference_bar.stop();
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
        duration += n_bar.stop();


        // Calculate Registartion
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
            pairAlign(source, target, pairTransform, 0.1f, 2U);
            transforms[i] = pairTransform;
            reg_bar.update();
        }
        duration += reg_bar.stop();
        

        // Update Position
        ///////////////////////////////////////////////////////////////////////////////
        Eigen::Matrix4f global_transform = Eigen::Matrix4f::Identity();
        auto move_bar = util::progress_bar(n_frames,"Moving clouds");
        move_bar.update();
        for (std::size_t i = 1; i < point_clouds.size() ; ++i){
            global_transform *= transforms[i];
            pcl::transformPointCloudWithNormals(*point_clouds[i], *point_clouds[i], global_transform);
            move_bar.update();
        }
        duration += move_bar.stop();


        // Save indevidula clous
        ///////////////////////////////////////////////////////////////////////////////
        auto cwd = std::filesystem::current_path();
        std::filesystem::create_directory("clouds");
        std::filesystem::current_path("clouds");
        auto save_bar = util::progress_bar(n_frames,"Saving clouds");
        #pragma omp parallel for shared(point_clouds)
        for (size_t i=0; i < point_clouds.size(); ++i){
            std::string filename = fmt::format("cloud_{}.pcd", point_clouds[i]->header.frame_id);
            pcl::io::savePCDFileBinary(filename, *point_clouds[i]);
            save_bar.update();
        }
        std::filesystem::current_path(cwd);

        return;

        // Merge
        ///////////////////////////////////////////////////////////////////////////////
        PointCloud::Ptr merged_cloud (new PointCloud);
        merged_cloud->reserve(get_total_size(point_clouds));
        auto merging_bar = util::progress_bar(n_frames,"Merging data");
        for (size_t i = 0; i < point_clouds.size(); i++){
            *merged_cloud += *point_clouds[i];
            merging_bar.update();
        }
        duration += merging_bar.stop();


        // Downsampling
        ///////////////////////////////////////////////////////////////////////////////
        pcl::octree::OctreePointCloudPointVector<PointT> octree(0.02);
        PointCloud::Ptr filtered_cloud (new PointCloud);
        octree.setInputCloud(merged_cloud);
        octree.addPointsFromInputCloud();
        filtered_cloud->resize(octree.getLeafCount());
        auto grid_bar = util::progress_bar(octree.getLeafCount(),"Downsampling");
        std::vector<pcl::octree::OctreePointCloudPointVector<PointT>::LeafNodeIterator> nodes;
        for (auto it = octree.leaf_depth_begin(), it_end = octree.leaf_depth_end(); it != it_end; ++it){
            nodes.push_back(it);
        }
        #pragma omp parallel for shared(octree, filtered_cloud, nodes)
        for (size_t i = 0; i < octree.getLeafCount(); i++){
            pcl::octree::OctreeContainerPointIndices& container = nodes[i].getLeafContainer();

            pcl::Indices indexVector;
            container.getPointIndices(indexVector);

            typename Accumulators<PointCloud::PointType>::type acc;

            for (auto const& index : indexVector){
                PointCloud::PointType point = merged_cloud->at(index);
                boost::fusion::for_each (acc, pcl::detail::AddPoint<PointCloud::PointType> (point));
            }

            boost::fusion::for_each (acc, pcl::detail::GetPoint<PointCloud::PointType> (filtered_cloud->at(i), indexVector.size()));


            grid_bar.update();
        }
        duration += grid_bar.stop();



        // Only keep highest confidence
        ///////////////////////////////////////////////////////////////////////////////
        auto filter_bar = util::progress_bar(n_frames,"Filtering data");
        size_t j = 0;
        for (size_t k = 0; k < filtered_cloud->size(); k++){
            if (filtered_cloud->at(k).confidence >= 2U){
                filtered_cloud->at(j) = filtered_cloud->at(k);
                j++;
            }
        }
        filtered_cloud->resize(j);
        filtered_cloud->width = j;
        filtered_cloud->height = 1;
        filtered_cloud->is_dense = false;
        duration += filter_bar.stop();


        // Filter
        ///////////////////////////////////////////////////////////////////////////////
        // if (false){
        //     auto filter_bar = util::progress_bar(n_frames,"Filtering data");
        //     #pragma omp parallel for shared(point_clouds)
        //     for (size_t i = 0; i < point_clouds.size(); i++){   
        //         // size_t j = 0;
        //         // for (size_t k = 0; k < point_clouds[i]->size(); k++){
        //         //     if (point_clouds[i]->at(k).confidence >= 2){
        //         //         point_clouds[i]->at(j) = point_clouds[i]->at(k);
        //         //         j++;
        //         //     }
        //         // }
        //         // point_clouds[i]->resize(j);
        //         // point_clouds[i]->width = j;
        //         // point_clouds[i]->height = 1;
        //         // point_clouds[i]->is_dense = false;
        //         // build rules
        //         pcl::ConditionAnd<PointT>::Ptr range_cond (new pcl::ConditionAnd<PointT> ());
        //         range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new
        //             pcl::FieldComparison<PointT>("confidence", pcl::ComparisonOps::EQ, uint8_t(2))));
        //         // build the filter
        //         pcl::ConditionalRemoval<PointT> condrem;
        //         condrem.setCondition(range_cond);
        //         condrem.setInputCloud(point_clouds[i]);
        //         condrem.setKeepOrganized(false);
        //         condrem.filter(*point_clouds[i]);
        //         filter_bar.update();
        //     }
        //     duration += filter_bar.stop();
        // }


        // // Remove NaN Normals
        // ///////////////////////////////////////////////////////////////////////////////
        // for (size_t i=0; i < point_clouds.size(); ++i){
        //     std::vector<int> indices;
        //     pcl::removeNaNNormalsFromPointCloud(*point_clouds[i].cloud, *point_clouds[i].cloud, indices);
        // }


        // // Make an index vector of point wihtout sematic labels
        // ///////////////////////////////////////////////////////////////////////////////
        // pcl::IndicesPtr indices (new std::vector <int>);


        // Region growing
        ///////////////////////////////////////////////////////////////////////////////
        pcl::search::Search<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
        pcl::RegionGrowing<PointT, PointT> reg;
        reg.setMinClusterSize (500);
        // reg.setMaxClusterSize (1000000);
        reg.setSearchMethod (tree);
        reg.setNumberOfNeighbours (30);
        reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
        reg.setCurvatureThreshold (0.5);
        // reg.setIndices (indices);
        reg.setInputCloud (filtered_cloud);
        reg.setInputNormals (filtered_cloud);
        std::vector <pcl::PointIndices> clusters;
        reg.extract (clusters);

        std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
        std::cout << "First cluster has " << clusters[0].indices.size () << " points." << std::endl;

        for (size_t i = 0; i < clusters.size(); ++i){
            for (size_t j = 0; j < clusters[i].indices.size (); ++j){
                filtered_cloud->at(clusters[i].indices[j]).label = i;
            }
        }


        // Ray tracing
        ///////////////////////////////////////////////////////////////////////////////


        // Surface reconstruction
        ///////////////////////////////////////////////////////////////////////////////
        // Surface_mesh mesh = surface_reconstruction(*filtered_cloud, clusters);


        // Print runtimes
        ///////////////////////////////////////////////////////////////////////////////
        auto end_time = std::chrono::high_resolution_clock::now();
        fmt::print(fg(fmt::color::green), "Total time: ");
        fmt::print(fmt::emphasis::italic,  "{:3.2f}ms\n", std::chrono::duration<double, std::milli>(end_time - start_time).count());
        fmt::print(fg(fmt::color::green), "Total duration: ");
        fmt::print(fmt::emphasis::italic,  "{:3.2f}ms\n", std::chrono::duration<double, std::milli>(duration).count());


        // Display
        ///////////////////////////////////////////////////////////////////////////////
        polyscope::display(*filtered_cloud, "Cloud");
        // polyscope::display(mesh, "Mesh");
        polyscope::show();
        cv::destroyAllWindows();


        // Save to disk
        ///////////////////////////////////////////////////////////////////////////////
        pcl::io::savePCDFileBinaryCompressed("filtered cloud.pcd", *filtered_cloud);


        

    }

}

