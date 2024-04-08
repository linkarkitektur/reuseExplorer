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
    MatchCondition(std::uint8_t value) : value_(value) {}

    // Overloaded function operator to be used as a functor
    bool operator()(const pcl::PointCloud<PointT> & cloud, pcl::index_t index)
    {
        return value_ == cloud.points[index].semantic;
    }

private:
    std::uint8_t value_;
};

linkml::PointCloud linkml::PointCloud::clustering(){

    using Filter = pcl::experimental::advanced::FunctorFilter<PointCloud::PointType, MatchCondition<PointCloud::PointType>>;

    // FIXME: This is bad as it is making a full copy of the point cloud
    auto cloud = this->makeShared();

    // Create a set of all labes
    std::unordered_set<uint8_t> lables;
    for (size_t i = 0; i < this->points.size(); ++i)
        lables.insert(this->points[i].semantic);
    
    lables.erase(0U); //Do not cluster the unlabeled points

    std::vector<int> lables_vec(lables.begin(), lables.end());
    

    pcl::search::KdTree<PointCloud::PointType>::Ptr tree (new pcl::search::KdTree<PointCloud::PointType>);
    tree->setInputCloud (cloud);

    auto bar = util::progress_bar(lables_vec.size(), "Clustering");
    #pragma omp parallel for shared(cloud, lables_vec) private(tree) 
    for (size_t i = 0; i < lables_vec.size(); i++){

        pcl::PointIndices::Ptr points (new pcl::PointIndices);
        MatchCondition<PointCloud::PointType> match_filter(lables_vec[i]);
        Filter pass(match_filter);
        pass.setInputCloud(cloud);
        pass.filter(points->indices);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<PointCloud::PointType> ec;
        ec.setClusterTolerance (0.02); // 2cm
        ec.setMinClusterSize (100);
        //ec.setMaxClusterSize (25000);
        ec.setSearchMethod (tree);
        ec.setInputCloud (cloud);
        ec.setIndices(points);
        ec.extract (cluster_indices);

        #pragma omp parallel // for collapse(2)
        for (size_t j = 0; j < cluster_indices.size(); j++)
            for (size_t k = 0; k < cluster_indices[j].indices.size(); k++)
                this->points[cluster_indices[j].indices[k]].instance = j + 1;

        bar.update();
    }
    bar.stop();


    return *this;
}