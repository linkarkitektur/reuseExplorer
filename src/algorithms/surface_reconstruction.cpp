#include <algorithms/surface_reconstruction.hh>

#include <types/PointCloud.hh>


#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/property_map.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polygonal_surface_reconstruction.h>
#include <types/Polyhonal_surface_reconstruction_custom.hh>

#ifdef CGAL_USE_GUROBI  // defined (or not) by CMake scripts, do not define by hand/
#include <types/GUROBI_mixed_integer_program_traits.hh>
typedef CGAL::GUROBI_mixed_integer_program_traits<double>					MIP_Solver;
#else
#include <CGAL/SCIP_mixed_integer_program_traits.h>
typedef CGAL::SCIP_mixed_integer_program_traits<double>						MIP_Solver;
// #elif defined(CGAL_USE_GLPK)  // defined (or not) by CMake scripts, do not define by hand
// #include <CGAL/GLPK_mixed_integer_program_traits.h>
// typedef CGAL::GLPK_mixed_integer_program_traits<double>                        MIP_Solver;
#endif

// #if defined(CGAL_USE_GLPK) || defined(CGAL_USE_SCIP)

#include <CGAL/Timer.h>
#include <fstream>

typedef Kernel::Vector_3													Vector;
typedef        CGAL::Polygonal_surface_reconstruction<Kernel>				Polygonal_surface_reconstruction;
typedef        CGAL::Polygonal_surface_reconstruction_custom<Kernel>		Polygonal_surface_reconstruction_custom;



// Point with normal, and plane index
typedef boost::tuple<Point_3, Vector, int>									PNI;
typedef std::vector<PNI>													Point_vector;
typedef CGAL::Nth_of_tuple_property_map<0, PNI>								Point_map;
typedef CGAL::Nth_of_tuple_property_map<1, PNI>								Normal_map;
typedef CGAL::Nth_of_tuple_property_map<2, PNI>								Plane_index_map;


#include <CGAL/Shape_detection/Region_growing/Region_growing.h>
#include <CGAL/IO/read_points.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>

Surface_mesh linkml::surface_reconstruction( const linkml::PointCloud & cloud, const std::vector <pcl::PointIndices> & planes){


	// Load point, normal and plan idx in to points
	// apparently -1 is a vailied value if a point does not belong to any plane.
	Point_vector points{};
    for (size_t i = 0; i < planes.size(); i++){
        for (size_t j = 0; j < planes[i].indices.size(); j++){
            auto pt = (*cloud)[planes[i].indices[j]];
            points.push_back(PNI(Point_3(pt.x, pt.y, pt.z), Vector(pt.normal_x,pt.normal_y, pt.normal_z), i ));
        }
    }

	CGAL::Timer t;
	std::cout << "Generating candidate faces...";
	// t.reset();
	t.start();
	Polygonal_surface_reconstruction_custom algo(
		points,
		Point_map(),
		Normal_map(),
		Plane_index_map()
	);


	std::cout << " Done. Time: " << t.time() << " sec." << std::endl;
	Surface_mesh model;
	std::cout << "Reconstructing...";

	double wt_fitting 		= 0.43;		///< weight for the data fitting term.		default => 0.43
    double wt_coverage 		= 0.27;		///< weight for the point coverage term.	default => 0.27
    double wt_complexity 	= 0.30;		///< weight for the model complexity term.	default => 0.30
	t.reset();
	if (!algo.reconstruct<MIP_Solver>(model, wt_fitting, wt_coverage, wt_complexity)) {
		std::cerr << " Failed: " << algo.error_message() << std::endl;
	}

	CGAL::Polygon_mesh_processing::triangulate_faces(model);

	if(!CGAL::is_triangle_mesh(model))
	{
		std::cerr << "Input geometry is not triangulated." << std::endl;
	}
    
    return model;

}

