#include <types/PointCloud.hh>
#include "functions/progress_bar.hh"

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/experimental/functor_filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <iomanip> // for setw, setfill



template<typename PointT>
class MatchCondition
{
public:
    MatchCondition(std::int32_t value) : value_(value) {}

    // Overloaded function operator to be used as a functor
    bool operator()(const pcl::PointCloud<PointT> & cloud, pcl::index_t index)
    {
        return value_ == cloud.points[index].semantic;
    }

private:
    std::int32_t value_;
};

void linkml::PointCloud::clustering(
    double cluster_tolerance,
    pcl::uindex_t min_cluster_size,
    pcl::uindex_t max_cluster_size
){

    using Filter = pcl::experimental::advanced::FunctorFilter<PointCloud::Cloud::PointType, MatchCondition<PointCloud::Cloud::PointType>>;

    auto cloud = *this;

    // Create a set of all labes
    std::unordered_set<int32_t> lables;
    for (size_t i = 0; i < (*this)->points.size(); ++i)
        lables.insert((*this)->points[i].semantic);
    
    lables.erase(0U); //Do not cluster the unlabeled points

    std::vector<int> lables_vec(lables.begin(), lables.end());

    pcl::search::KdTree<PointCloud::Cloud::PointType>::Ptr tree (new pcl::search::KdTree<PointCloud::Cloud::PointType>);
    tree->setInputCloud (cloud);

    auto bar = util::progress_bar(lables_vec.size(), "Clustering");
    #pragma omp parallel for shared(cloud, lables_vec) private(tree) 
    for (size_t i = 0; i < lables_vec.size(); i++){

        pcl::PointIndices::Ptr points (new pcl::PointIndices);
        MatchCondition<PointCloud::Cloud::PointType> match_filter(lables_vec[i]);
        Filter pass(match_filter);
        pass.setInputCloud(cloud);
        pass.filter(points->indices);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<PointCloud::Cloud::PointType> ec;
        ec.setClusterTolerance (cluster_tolerance); // 2cm
        ec.setMinClusterSize (min_cluster_size);
        ec.setMaxClusterSize (max_cluster_size);
        ec.setSearchMethod (tree);
        ec.setInputCloud (cloud);
        ec.setIndices(points);
        ec.extract (cluster_indices);

        //#pragma omp parallel // for collapse(2)
        for (size_t j = 0; j < cluster_indices.size(); j++)
            for (size_t k = 0; k < cluster_indices[j].indices.size(); k++)
                (*this)->points[cluster_indices[j].indices[k]].instance = j + 1;

        bar.update();
    }
    bar.stop();

}