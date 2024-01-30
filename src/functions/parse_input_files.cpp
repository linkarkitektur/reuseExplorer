#include "parse_input_files.hh"
#include <types/dataset.hh>

#include <Eigen/Dense>



#include <mutex>
#include <thread>


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

#include <typed-geometry/tg-std.hh>
#include <typed-geometry/tg.hh>
#include <functions/depth_to_3d.hh>

#include <functions/polyscope.hh>
#include <polyscope/polyscope.h>
#include <polyscope/point_cloud.h>
#include <polyscope/curve_network.h>

#include <pcl/memory.h>  // for pcl::make_shared
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/common/transforms.h>


namespace linkml{

    //convenient typedefs
    typedef pcl::PointXYZ                           PointT;
    typedef pcl::PointCloud<PointT>                 PointCloud;
    typedef pcl::PointNormal                        PointNormalT;
    typedef pcl::PointCloud<PointNormalT>           PointCloudWithNormals;


    // using PointT = tg::pos3;
    // using PointCloud = pcl::PointCloud<PointT>;
    // using PointNormalT = std::tuple<tg::pos3,tg::vec3>;
    // using PointCloudWithNormals = pcl::PointCloud<PointNormalT>;


    //convenient structure to handle our pointclouds
    struct PCD{
        PointCloud::Ptr cloud;
        std::string f_name;
        PCD() : cloud( new PointCloud ) {};
    };

    struct PCDComparator
    {
        bool operator () (const PCD& p1, const PCD& p2){
            return (p1.f_name < p2.f_name);
        }

    };

    // Define a new point representation for < x, y, z, curvature >
    class MyPointRepresentation : public pcl::PointRepresentation <PointNormalT>{

        using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;

        public:

            MyPointRepresentation (){
                // Define the number of dimensions
                nr_dimensions_ = 4;
            }


            // Override the copyToFloatArray method to define our feature vector
            virtual void copyToFloatArray (const PointNormalT &p, float * out) const{
                // < x, y, z, curvature >
                out[0] = p.x;
                out[1] = p.y;
                out[2] = p.z;
                out[3] = p.curvature;
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
        const PointCloud::Ptr cloud_src, 
        const PointCloud::Ptr cloud_tgt, 
        PointCloud::Ptr output, 
        Eigen::Matrix4f &final_transform, 
        bool downsample = false
        ){
        //
        // Downsample for consistency and speed
        // \note enable this for large datasets

        PointCloud::Ptr src(new PointCloud);
        PointCloud::Ptr tgt(new PointCloud);
        pcl::VoxelGrid<PointT> grid;

        if (downsample){

            grid.setLeafSize (0.05, 0.05, 0.05);
            grid.setInputCloud (cloud_src);
            grid.filter (*src);


            grid.setInputCloud (cloud_tgt);
            grid.filter (*tgt);
        }

        else{
            src = cloud_src;
            tgt = cloud_tgt;
        }



        // Compute surface normals and curvature

        PointCloudWithNormals::Ptr points_with_normals_src(new PointCloudWithNormals);
        PointCloudWithNormals::Ptr points_with_normals_tgt(new PointCloudWithNormals);

        pcl::NormalEstimation<PointT, PointNormalT> norm_est;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
        norm_est.setSearchMethod (tree);
        norm_est.setKSearch (30);

        norm_est.setInputCloud (src);
        norm_est.compute (*points_with_normals_src);
        pcl::copyPointCloud (*src, *points_with_normals_src);

        norm_est.setInputCloud (tgt);
        norm_est.compute (*points_with_normals_tgt);
        pcl::copyPointCloud (*tgt, *points_with_normals_tgt);


        //
        // Instantiate our custom point representation (defined above) ...
        MyPointRepresentation point_representation;

        // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
        float alpha[4] = {1.0, 1.0, 1.0, 1.0};
        point_representation.setRescaleValues (alpha);


        //
        // Align
        pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
        reg.setTransformationEpsilon (1e-6);

        // Set the maximum distance between two correspondences (src<->tgt) to 10cm
        // Note: adjust this based on the size of your datasets
        reg.setMaxCorrespondenceDistance (0.1);  

        // Set the point representation
        reg.setPointRepresentation (pcl::make_shared<const MyPointRepresentation> (point_representation));

        reg.setInputSource (points_with_normals_src);
        reg.setInputTarget (points_with_normals_tgt);


        //
        // Run the same optimization in a loop and visualize the results
        Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
        PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
        reg.setMaximumIterations (2);
        for (int i = 0; i < 30; ++i){

            PCL_INFO ("Iteration Nr. %d.\n", i);

            // save cloud for visualization purpose
            points_with_normals_src = reg_result;

            // Estimate
            reg.setInputSource (points_with_normals_src);
            reg.align (*reg_result);

                //accumulate transformation between each Iteration
            Ti = reg.getFinalTransformation () * Ti;

                //if the difference between this transformation and the previous one
                //is smaller than the threshold, refine the process by reducing
                //the maximal correspondence distance

            if (std::abs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
            reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);

            prev = reg.getLastIncrementalTransformation ();


        }

            //
        // Get the transformation from target to source
        targetToSource = Ti.inverse();

        //
        // Transform target back in source frame
        pcl::transformPointCloud (*cloud_tgt, *output, targetToSource);

        // p->removePointCloud ("source");
        // p->removePointCloud ("target");

        // pcl::PointCloudColorHandlerCustom<PointT> cloud_tgt_h (output, 0, 255, 0);
        // pcl::PointCloudColorHandlerCustom<PointT> cloud_src_h (cloud_src, 255, 0, 0);

        // p->addPointCloud (output, cloud_tgt_h, "target", vp_2);
        // p->addPointCloud (cloud_src, cloud_src_h, "source", vp_2);

        //     PCL_INFO ("Press q to continue the registration.\n");

        // p->spin ();

        // p->removePointCloud ("source"); 
        // p->removePointCloud ("target");

        //add the source to the transformed target
        *output += *cloud_src;

        final_transform = targetToSource;
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

    void parse_input_files(std::string const& path){

        auto dataset = Dataset(path, {Field::COLOR, Field::DEPTH, Field::CONFIDENCE, Field::ODOMETRY, Field::IMU, Field::POSES});

        polyscope::init();
        polyscope::options::groundPlaneMode = polyscope::GroundPlaneMode::ShadowOnly;
        polyscope::view::setUpDir(polyscope::UpDir::NegYUp);


        auto camera_intrisic_matrix = dataset.intrinsic_matrix();

        std::vector<point_cloud, Eigen::aligned_allocator<point_cloud>> point_clouds;


        for (size_t i=0; i< 11; i+=5){
            auto data = dataset[i];
            auto point_cloud = depth_to_3d(
                data.get<Field::DEPTH>(),
                camera_intrisic_matrix, 
                data.get<Field::POSES>(), 
                data.get<Field::COLOR>());

            point_clouds.push_back(point_cloud);

            // std::string name = "Cloud " + std::to_string(i);
            // polyscope::display(point_cloud, name);

        }




        // Convert point clouds
        ///////////////////////////////////////////////////////////////////////////////

        std::vector<PCD, Eigen::aligned_allocator<PCD>> data;

        for (auto & point_cloud : point_clouds){
            PCD temp;
            for (auto const& point : point_cloud.pts){
                temp.cloud->push_back(pcl::PointXYZ(point.x, point.y, point.z));
            }
            data.push_back(temp);
        }


        ///////////////////////////////////////////////////////////////////////////////


        PointCloud::Ptr result(new PointCloud), source, target;

        Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity (), pairTransform;

    
        for (std::size_t i = 0; i < data.size()-1 ; ++i){

            target = data[i].cloud;
            source = data[i+1].cloud;


            PointCloud::Ptr temp (new PointCloud);
            pairAlign (source, target, temp, pairTransform, true);

            //transform current pair into the global transform
            pcl::transformPointCloud (*temp, *result, GlobalTransform);


            //update the global transform
            GlobalTransform *= pairTransform;

            data[i+1].cloud = result;

        }

        // #pragma omp parallel for
        for (int i = 0; i < data.size(); i++){
            for (int j = 0; j < data[i].cloud->points.size(); j++){
                auto & point = data[i].cloud->at(j);
                point_clouds[i].pts[j].x = point.x;
                point_clouds[i].pts[j].y = point.y;
                point_clouds[i].pts[j].z = point.z;
            }
        }

        for (int i = 0; i < point_clouds.size(); i++){
            std::string name = "Cloud " + std::to_string(i);
            polyscope::display(point_clouds[i], name);
        }

        polyscope::show();

    }

}

