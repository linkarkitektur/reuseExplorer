#include "types/PointCloud.hh"
#include "types/PlanarPointSet.hh"
#include "algorithms/markov_clustering.hh"
#include "algorithms/refine.hh"
#include "algorithms/clustering.hh"
#include "types/surface.hh"

#include <pcl/filters/random_sample.h>
#include <pcl/filters/experimental/functor_filter.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/PointIndices.h>

#include <embree3/rtcore.h>

#include <opencv4/opencv2/opencv.hpp>

template<typename PointT>
class MatchCondition
{
public:
    MatchCondition(){}

    // Overloaded function operator to be used as a functor
    bool operator()(const pcl::PointCloud<PointT> & cloud, pcl::index_t index)
    {
        auto is_inventory = 0 != cloud.points[index].semantic;
        auto is_surface = 0 != cloud.points[index].label;

        return !is_inventory && is_surface;
    }

};

using PointIndices = pcl::PointIndices;
using Indices = pcl::Indices;
using Clusters = std::vector<pcl::PointIndices>;
using Filter = pcl::experimental::advanced::FunctorFilter<linkml::PointCloud::PointType, MatchCondition<linkml::PointCloud::PointType>>;


Clusters extract_clusters(linkml::PointCloud::ConstPtr cloud){

    // Collect all cluster indices
    std::unordered_set<size_t> cluster_indices_set;
    for (size_t i = 0; i < cloud->points.size(); i++)
        cluster_indices_set.insert(cloud->points[i].label);

    cluster_indices_set.erase(0);

    Indices cluster_indices(cluster_indices_set.begin(), cluster_indices_set.end());

    // Create clusters
    Clusters clusters(cluster_indices.size());

    #pragma omp parallel for
    for (size_t i = 0; i < cluster_indices.size(); i++){
        size_t cluster_index = cluster_indices[i];
        for (size_t j = 0; j < cloud->points.size(); j++)
            if (cloud->points[j].label == cluster_index)
                clusters[i].indices.push_back(j);
    }

    return clusters;
}
Clusters filter_clusters(Clusters clusters){

    std::sort(clusters.begin(), clusters.end(), [](const auto& lhs, const auto& rhs){return lhs.indices.size() > rhs.indices.size();});

    auto temp = Clusters();
    for (int i = 0; i < std::min(30, (int)clusters.size()); i++)
        temp.push_back(clusters[i]);

    return temp;
}
linkml::PointCloud::Ptr filter_cloud(linkml::PointCloud::Ptr cloud, Clusters & clusters){

    std::unordered_set<size_t> cluster_indices_set;
    for (size_t i = 0; i < clusters.size(); i++)
        for (size_t j = 0; j < clusters[i].indices.size(); j++)
            cluster_indices_set.insert(clusters[i].indices[j]);

    pcl::PointIndices::Ptr selection(new pcl::PointIndices);
    selection->indices = pcl::Indices(cluster_indices_set.begin(), cluster_indices_set.end());

    pcl::ExtractIndices<linkml::PointCloud::PointType> filter;
    filter.setInputCloud(cloud);
    filter.setIndices(selection);
    filter.filter(*cloud);

    Clusters clusters_2 = extract_clusters(cloud);

    #pragma omp parallel for
    for (size_t i = 0; i < cloud->points.size(); i++)
        cloud->points[i].label = -1;

    #pragma omp parallel for
    for (size_t i = 0; i < clusters_2.size(); i++)
        for (size_t j = 0; j < clusters_2[i].indices.size(); j++)
            cloud->points[clusters_2[i].indices[j]].label = i;

    return cloud;
}


namespace linkml
{
    PointCloud PointCloud::solidify()
    {


        //FIXME: This is bad as makeShared() will make a copy of the whole point cloud
        auto cloud = this->makeShared();

        // Remove everyting that is not a surface
        auto match_filter = MatchCondition<PointCloud::PointType>();
        Filter pass(match_filter);
        pass.setInputCloud(cloud);
        pass.filter(*cloud);


        pcl::RandomSample<PointCloud::PointType> sampler;
        sampler.setInputCloud(cloud);
        sampler.setSample(500000);
        sampler.setSeed(0);
        sampler.filter(*cloud);

        // Extract clusters
        Clusters clusters = extract_clusters(cloud);

        std::cout << "Number of clusters: " << clusters.size() << std::endl;

        // Simplify clusters
        //clusters = refine(cloud, clusters);


        //clusters = filter_clusters(clusters);

        //std::cout << "Number of clusters after simplification: " << clusters.size() << std::endl;


        //cloud = filter_cloud(cloud, clusters);

        //std::cout << "Number of points after filtering: " << cloud->points.size() << std::endl;




        // Create surfaces
        std::vector<Surface> surfaces(clusters.size());
        auto surface_bar = util::progress_bar(surfaces.size(), "Creating surfaces");
        for (size_t i = 0; i < clusters.size(); i++){
            surfaces[i] = Surface(cloud, clusters[i].indices);
            surface_bar.update();
        }
        surface_bar.stop();


        // Ray tracing

        RTCDevice device = rtcNewDevice(NULL);
        RTCScene scene   = rtcNewScene(device);



        std::unordered_map<unsigned int, pcl::Indices> point_map;
        Rays rays;


        //CGAL::Surface_mesh<tg::pos3> mesh;
        //auto face_id = mesh.template add_property_map<CGAL::Surface_mesh<tg::pos3>::Face_index, std::size_t>("f:id").first;

        // TODO: Make this section parallel
        // Ensure Crate_Embree_Geometry is thread safe
        // Make custom reduction for rays
        //auto embree_bar = util::progress_bar(surfaces.size(), "Creating Embree geometries");
        for (size_t i = 0; i < surfaces.size(); i++){
            surfaces[i].Create_Embree_Geometry(device, scene, point_map, std::back_inserter(rays)) ;
            //rays.insert(rays.end(), r.begin(), r.end());
            //embree_bar.update();
        }
        //embree_bar.stop();


        // Display rays
        //polyscope::myinit();
        //std::vector<tg::pos3> verts(rays.size()*2);
        //std::vector<std::array<size_t, 2>> edges(rays.size());
        //for (size_t i = 0; i < rays.size(); i++){
        //    verts[i*2] = tg::pos3(rays[i].ray.org_x , rays[i].ray.org_y, rays[i].ray.org_z);
        //    verts[i*2+1] = tg::pos3(
        //        rays[i].ray.org_x + (rays[i].ray.dir_x * 0.1), 
        //        rays[i].ray.org_y + (rays[i].ray.dir_y * 0.1), 
        //        rays[i].ray.org_z + (rays[i].ray.dir_z * 0.1)
        //        );
        //    edges[i] = {i*2, i*2+1};
        //}
        //auto ps_rays = polyscope::registerCurveNetwork("rays", verts, edges);
        //ps_rays->setRadius(0.0001);
        //// Display mesh
        //std::vector<tg::pos3> mesh_verts;
        //std::vector<std::array<size_t, 3>> mesh_faces;
        //std::vector<double> face_id_val;
        //for (auto & f: mesh.faces()){
        //    auto h = mesh.halfedge(f);
        //    auto v0 = mesh.point(mesh.source(h));
        //    auto v1 = mesh.point(mesh.target(h));
        //    auto v2 = mesh.point(mesh.target(mesh.next(h)));
        //    mesh_verts.push_back(tg::pos3(v0[0], v0[1], v0[2]));
        //    mesh_verts.push_back(tg::pos3(v1[0], v1[1], v1[2]));
        //    mesh_verts.push_back(tg::pos3(v2[0], v2[1], v2[2]));
        //    mesh_faces.push_back({mesh_verts.size()-3, mesh_verts.size()-2, mesh_verts.size()-1});
        //    face_id_val.push_back(face_id[f]);
        //}
        //auto ps_mesh = polyscope::registerSurfaceMesh("mesh", mesh_verts, mesh_faces);
        //ps_mesh->addFaceScalarQuantity("face_id", face_id_val)->setEnabled(true);


        //RTCGeometry geom = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_TRIANGLE);

        //float* vb = (float*) rtcSetNewGeometryBuffer(geom,
        //    RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, 3*sizeof(float), 3);
        //vb[0] = 0.f; vb[1] = 0.f; vb[2] = 0.f; // 1st vertex
        //vb[3] = 1.f; vb[4] = 0.f; vb[5] = 0.f; // 2nd vertex
        //vb[6] = 0.f; vb[7] = 1.f; vb[8] = 0.f; // 3rd vertex

        //unsigned* ib = (unsigned*) rtcSetNewGeometryBuffer(geom,
        //    RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, 3*sizeof(unsigned), 1);
        //ib[0] = 0; ib[1] = 1; ib[2] = 2;

        //rtcCommitGeometry(geom);
        //rtcAttachGeometry(scene, geom);
        //rtcReleaseGeometry(geom);        


        rtcCommitScene(scene);


        //RTCRayHit rayhit; 
        //rayhit.ray.org_x  = 0.5f; rayhit.ray.org_y = 0.5f; rayhit.ray.org_z = -2.f;
        //rayhit.ray.dir_x  = 0.f; rayhit.ray.dir_y = 0.f; rayhit.ray.dir_z =  1.f;
        //rayhit.ray.tnear  = 0.f;
        //rayhit.ray.tfar   = std::numeric_limits<float>::infinity();
        //rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;

        //RTCBounds bounds;
        //rtcGetSceneBounds(scene, &bounds);
        //auto bbox = cloud->get_bbox();

        // Instatiate the context
        RTCIntersectContext context;
        rtcInitIntersectContext(&context);


        //rtcIntersect1(scene, &context, &rayhit);
        //if (rayhit.hit.geomID != RTC_INVALID_GEOMETRY_ID) {
        //  std::cout << "Intersection at t = " << rayhit.ray.tfar << std::endl;
        //} else {
        //  std::cout << "No Intersection" << std::endl;
        //} 


            
        //for (size_t i = 0; i < rays.size(); i++){
        //    rtcIntersect1(scene, &context, (RTCRayHit*)&rays[i]);
        //    if (rays[i].hit.geomID != RTC_INVALID_GEOMETRY_ID){
        //        std::cout << "Hit: " << rays[i].hit.geomID << std::endl;
        //    }
        //    else{

        //        if (rays[i].ray.org_x < bbox.min.x || rays[i].ray.org_x > bbox.max.x ||
        //            rays[i].ray.org_y < bbox.min.y || rays[i].ray.org_y > bbox.max.y ||
        //            rays[i].ray.org_z < bbox.min.z || rays[i].ray.org_z > bbox.max.z)
        //        {
        //            std::cout << "Missed: " << rays[i].ray.id << std::endl;
        //            std::cout << "Origin: " << rays[i].ray.org_x << " " << rays[i].ray.org_y << " " << rays[i].ray.org_z << std::endl;
        //            std::cout << "Direction: " << rays[i].ray.dir_x << " " << rays[i].ray.dir_y << " " << rays[i].ray.dir_z << std::endl;
        //            std::cout << "N: " << rays[i].ray.tnear << std::endl;
        //            std::cout << "F: " << rays[i].ray.tfar << std::endl;
        //            std::cout << "Time: " << rays[i].ray.time << std::endl;
        //            std::cout << "Flags: " << rays[i].ray.flags << std::endl;
        //            std::cout << "Mask: " << rays[i].ray.mask << std::endl;
        //            std::cout << std::endl;
        //        }
        //    }
        //}

        // Intersect all rays with the scene
        rtcIntersect1M(scene, &context, (RTCRayHit*)rays.data(), rays.size(), sizeof(RTCRayHit));

        std::unordered_map<size_t, unsigned int> id_to_int;
        std::unordered_map<unsigned int, size_t> int_to_id;

        size_t spm_idx = 0;
        for (auto & [id,_]:   point_map){
                id_to_int[id] = spm_idx;
                int_to_id[spm_idx] = id;
                spm_idx++;
        }




        // Initialize the matrix
        SpMat matrix = SpMat(point_map.size(), point_map.size());
        matrix += Eigen::VectorXd::Ones(matrix.cols()).template cast<FT>().asDiagonal();

        bool any_hit = false;

        //auto result_bar = util::progress_bar(rays.size(), "Processing rays");
        for (size_t i = 0; i < rays.size(); i++){
            if (rays[i].hit.geomID != RTC_INVALID_GEOMETRY_ID) {

                int source_int = id_to_int[rays[i].ray.id];
                int target_int = id_to_int[rays[i].hit.geomID];

                auto vaule = matrix.coeff(source_int, target_int) + 1;

                // Increment the matrix
                matrix.coeffRef(source_int, target_int) = vaule;
                matrix.coeffRef(target_int, source_int) = vaule;

                any_hit = true;

            }
            //result_bar.update();
        }
        //result_bar.stop();


        //std::cout << "Error: " << rtcGetDeviceError(device) << std::endl;
    
        // Release the scene and device
        rtcReleaseScene(scene);
        rtcReleaseDevice(device);

        std::cout << "Any hit: " << any_hit << std::endl;
        std::cout << "Number of rays: " << rays.size() << std::endl;
        std::cout << "Number of tiles: " << point_map.size() << std::endl;

        

        //cv::Mat img = cv::Mat::zeros(point_map.size(), point_map.size(), CV_8UC1);
        //for (int k = 0; k < matrix.outerSize(); ++k)
        //    for (SpMat::InnerIterator it(matrix, k); it; ++it)
        //        img.at<u_int8_t>(k, it) = (u_int8_t)(it.value()*255);
        //cv::imshow("Matrix", img);
        //cv::waitKey(0);
        //cv::destroyAllWindows();
        //std::cout << "Imaged shown" << std::endl;


        // Markov Clustering
        //// 2.5 < infaltion < 2.8  => 3.5
        matrix = markov_clustering::run_mcl(matrix, 4, 1.5);
        std::set<std::vector<size_t>> mc_clusters = markov_clustering::get_clusters(matrix);
        std::printf("Number of clusters after Markov Clustering: %d\n", (int)mc_clusters.size());

        u_int8_t cluster_index = 1;
        for (const std::vector<size_t> & cluster: mc_clusters){
            for (const size_t & idx: cluster)
                for(const size_t & index : point_map[int_to_id[(int)idx]])
                    cloud->points[index].instance = cluster_index;

            cluster_index += 1;
        }
        std::cout << "Number of clusters after Markov Clustering: " << (int)cluster_index-1 << std::endl;


        cloud->display("cloud");
        //polyscope::myinit();
        //polyscope::display(*cloud, "cloud");
        //polyscope::getPointCloud("cloud")->getQuantity("Instance Colors")->setEnabled(true);
        //polyscope::show();

        return *cloud;
    }
} // namespace linkml
