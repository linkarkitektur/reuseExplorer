#include <types/PointClouds.hh>
#include <functions/polyscope.hh>

#include <polyscope/polyscope.h>
#include <polyscope/point_cloud.h>
#include <polyscope/curve_network.h>
#include <functions/color.hh>

#include <fmt/format.h>

template <typename T>
linkml::PointClouds<T> linkml::PointClouds<T>::display(bool show_clouds){


    polyscope::myinit();

    //if(show_clouds) for (size_t i = 0; i < data.size(); ++i)
    //        data[i]->display(fmt::format("Cloud {}", i).c_str());

    std::vector<Eigen::Vector4f> path_node;
    std::vector<Eigen::Vector3f> x_axis;
    std::vector<Eigen::Vector3f> y_axis;
    std::vector<Eigen::Vector3f> z_axis;
    std::vector<int> seq;
    std::vector<std::array<size_t, 2>> path_edges;

    path_node.resize(data.size());
    x_axis.resize(data.size());
    y_axis.resize(data.size());
    z_axis.resize(data.size());
    seq.resize(data.size());
    path_edges.resize(data.size()-1);

    #pragma omp parallel for shared(data, path_node, path_edges)
    for (size_t i = 0; i < data.size(); ++i){

        PointCloud::Cloud::Ptr cloud;
        
        if constexpr (std::is_same<T, std::string>::value){
            cloud = PointCloud::load(data[i]);
        } else {
            cloud = data[i];
        }

        auto matrix = cloud->sensor_orientation_.toRotationMatrix().normalized();

        path_node[i] = cloud->sensor_origin_;
        x_axis[i] = matrix.col(0);
        y_axis[i] = matrix.col(1);
        z_axis[i] = matrix.col(2);

        seq[i] = cloud->header.seq;

        if(i > 0){
            path_edges[i-1] = {i-1, i};
        }
    }

    auto ps_path = polyscope::registerCurveNetwork("Path", path_node, path_edges);
    ps_path->addNodeScalarQuantity("Sequence", seq);
    
    auto ps_Xaxis = ps_path->addNodeVectorQuantity("XAxis", x_axis);
    auto ps_Yaxis = ps_path->addNodeVectorQuantity("YAxis", y_axis);
    auto ps_Zaxis = ps_path->addNodeVectorQuantity("ZAxis", z_axis);

    ps_Xaxis->setVectorColor({1, 0, 0});
    ps_Yaxis->setVectorColor({0, 1, 0});
    ps_Zaxis->setVectorColor({0, 0, 1});

    ps_Xaxis->setVectorRadius(0.001);
    ps_Yaxis->setVectorRadius(0.001);
    ps_Zaxis->setVectorRadius(0.001);

    ps_Xaxis->setVectorLengthScale(0.02);
    ps_Yaxis->setVectorLengthScale(0.02);
    ps_Zaxis->setVectorLengthScale(0.02);

    ps_Xaxis->setEnabled(true);
    ps_Yaxis->setEnabled(true);
    ps_Zaxis->setEnabled(true);


    polyscope::myshow();

    return *this;

}


template linkml::PointCloudsInMemory linkml::PointCloudsInMemory::display(bool show_clouds);
template linkml::PointCloudsOnDisk linkml::PointCloudsOnDisk::display(bool show_clouds);