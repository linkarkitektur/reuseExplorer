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


        clusters = filter_clusters(clusters);

        std::cout << "Number of clusters after simplification: " << clusters.size() << std::endl;


        cloud = filter_cloud(cloud, clusters);

        std::cout << "Number of points after filtering: " << cloud->points.size() << std::endl;

        Surface surface(cloud, clusters[0].indices);


        //polyscope::myinit();
        //polyscope::display(*cloud);
        //polyscope::show();


        // Visual clustering
        auto visual_clusters = linkml::clustering(cloud);

        
        return *this;
    }
} // namespace linkml
