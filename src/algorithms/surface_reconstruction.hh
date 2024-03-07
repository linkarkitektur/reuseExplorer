#pragma once
#include <vector>
#include <types/PointCloud.hh>

#include <pcl/PointIndices.h>

#include <CGAL/Surface_mesh.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel					Kernel;
typedef Kernel::Point_3														Point_3;
typedef CGAL::Surface_mesh<Point_3>                                         Surface_mesh;


namespace linkml{
    Surface_mesh surface_reconstruction( const linkml::PointCloud & cloud, const std::vector <pcl::PointIndices> & planes);
}
