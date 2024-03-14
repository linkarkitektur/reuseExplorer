#include <types/PointClouds.hh>
#include <functions/polyscope.hh>

#include <polyscope/polyscope.h>
#include <polyscope/point_cloud.h>
#include <polyscope/curve_network.h>
#include <functions/color.hh>

#include <fmt/format.h>

template <typename T>
void linkml::PointClouds<T>::display(bool show_clouds){

    polyscope::myinit();

    if(show_clouds) for (size_t i = 0; i < data.size(); ++i)
            data[i].display(fmt::format("Cloud {}", i).c_str());

    std::vector<std::array<float, 3>> path_node;
    std::vector<std::array<size_t, 2>> path_edges;

    path_node.resize(data.size());
    path_edges.resize(data.size()-1);

    #pragma omp parallel for shared(data, path_node, path_edges)
    for (size_t i = 0; i < data.size(); ++i){
        auto origin = data[i]->sensor_origin;
        path_node[i] = {origin.x, origin.y, origin.z};

        if(i > 0){
            path_edges[i-1] = {i-1, i};
        }
    }

    polyscope::registerCurveNetwork("Path", path_node, path_edges);


    std::vector<std::array<float, 3>> veiw_node;
    std::vector<std::array<double, 3>> view_node_colors;
    std::vector<std::array<size_t, 2>> view_edges;

    veiw_node.resize(data.size()*4);
    view_node_colors.resize(data.size()*4);
    view_edges.resize(data.size()*3);

    #pragma omp parallel for shared(data, veiw_node, view_edges)
    for (size_t i = 0; i < data.size(); ++i){


        auto origin = data[i]->sensor_origin_;
        Eigen::Vector3f rotation = data[i]->sensor_orientation_.toRotationMatrix().eulerAngles(0, 1, 2);

        auto x = rotation.x();
        auto y = rotation.y();
        auto z = rotation.z();

        veiw_node[i*4] = {origin.x, origin.y, origin.z};
        veiw_node[i*4+1] = {origin.x() + x, origin.y() + y, origin.z() + z};
        veiw_node[i*4+2] = {origin.x() + y, origin.y() + z, origin.z() + x};
        veiw_node[i*4+3] = {origin.x() + z, origin.y() + x, origin.z() + y};

        view_node_colors[i*4] = {1, 1, 1};
        view_node_colors[i*4+1] = {1, 0, 0};
        view_node_colors[i*4+2] = {0, 1, 0};
        view_node_colors[i*4+3] = {0, 0, 1};


        if(i > 0){
            view_edges[(i-1)*3] = {i*4, i*4+1};
            view_edges[(i-1)*3+1] = {i*4, i*4+2};
            view_edges[(i-1)*3+2] = {i*4, i*4+3};
        }
    }

    polyscope::registerCurveNetwork("Views", veiw_node, view_edges);
    polyscope::getCurveNetwork("Views")->addNodeColorQuantity("Axis Colors", view_node_colors);

    polyscope::myshow();

}


template <>
void linkml::PointCloudsInMemory::display(bool show_clouds){


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

        auto cloud = data[i];
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

}

template <>
void linkml::PointCloudsOnDisk::display(bool show_clouds){


    polyscope::myinit();

    

    //if(show_clouds) for (size_t i = 0; i < data.size(); ++i)
    //        PointCloud::load(data[i])->display(fmt::format("Cloud {}", i).c_str());

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

        auto cloud = PointCloud::load(data[i]);
        auto matrix = cloud->sensor_orientation_.toRotationMatrix().normalized();

        path_node[i] = cloud->sensor_origin_;
        x_axis[i] = matrix.col(0);
        y_axis[i] = matrix.col(1);
        z_axis[i] = matrix.col(2);

        //seq[i] = cloud->header.seq;
        seq[i] = i;

        if(i > 0){
            path_edges[i-1] = {i-1, i};
        }
    }

    auto ps_path = polyscope::registerCurveNetwork("Path", path_node, path_edges);
    ps_path->setRadius(0.001);
    ps_path->setEnabled(true);

    auto ps_seq = ps_path->addNodeScalarQuantity("Sequence", seq);
    ps_seq->setEnabled(true);

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


}

