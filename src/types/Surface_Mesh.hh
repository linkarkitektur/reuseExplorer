#pragma once
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
//#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>

namespace linkml {
    using Kernel =  CGAL::Exact_predicates_inexact_constructions_kernel;
    //using Kernel =  CGAL::Exact_predicates_exact_constructions_kernel;
    using Point_3 = Kernel::Point_3;
    using Surface_mesh = CGAL::Surface_mesh<Point_3>;
}