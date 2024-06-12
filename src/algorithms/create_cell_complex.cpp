#include <algorithms/create_cell_complex.hh>

#include <types/CellComplex.hh>
#include <types/PointCloud.hh>

// #include <vector>
// #include <map>
// #include <execution>

// #include <functions/progress_bar.hh>
#include <functions/color.hh>
// #include <functions/polyscope_helpers.hh>
// #include <functions/crop_plane_with_aabb.hh>
// #include <functions/convex_hull_2d.hh>
// #include <functions/project_2d.hh>
// #include <functions/split_cellcomplex_with_planes.hh>
// #include <functions/color_facetes.hh>
// #include <functions/decimate_cell_complex.hh>
// #include <functions/compute_coverage.hh>
// #include <functions/linear_program.hh>
// #include <functions/linear_program_solver.hh>
// #include <functions/constuct_adjacency.hh>


// #include <typed-geometry/tg.hh>
// #include <typed-geometry/types/objects/plane.hh>
// #include <typed-geometry/feature/std-interop.hh>


// #include <clean-core/map.hh>

// #include <polymesh/pm.hh>
// #include <polymesh/copy.hh>
// #include <polymesh/Mesh.hh>
// #include <polymesh/algorithms.hh>
// #include <polymesh/algorithms/deduplicate.hh>
// #include <polymesh/algorithms/fill_hole.hh>
// #include <polymesh/attributes/partitioning.hh>
// #include <polymesh/ranges.hh>


// // #include "polyscope/surface_mesh.h"
// // #include "polyscope/point_cloud.h"
// // #include "polyscope/curve_network.h"

#include <polyscope/polyscope.h>
#include <functions/polyscope.hh>



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
typedef CGAL::Exact_predicates_inexact_constructions_kernel					Kernel;
typedef Kernel::Point_3														Point;
typedef Kernel::Vector_3													Vector;
typedef        CGAL::Polygonal_surface_reconstruction<Kernel>				Polygonal_surface_reconstruction;
typedef        CGAL::Polygonal_surface_reconstruction_custom<Kernel>		Polygonal_surface_reconstruction_custom;

typedef CGAL::Surface_mesh<Point>											Surface_mesh;

// Point with normal, and plane index
typedef boost::tuple<Point, Vector, int>									PNI;
typedef std::vector<PNI>													Point_vector;
typedef CGAL::Nth_of_tuple_property_map<0, PNI>								Point_map;
typedef CGAL::Nth_of_tuple_property_map<1, PNI>								Normal_map;
typedef CGAL::Nth_of_tuple_property_map<2, PNI>								Plane_index_map;


/*
* The following example shows the reconstruction using user-provided
* planar segments stored in PLY format. In the PLY format, a property
* named "segment_index" stores the plane index for each point (-1 if
* the point is not assigned to a plane).
*/


// 

#include <CGAL/Shape_detection/Region_growing/Region_growing.h>
// #include <CGAL/Shape_detection/Region_growing/Point_set.h>



// #include <boost/range/irange.hpp>
// typedef Kernel::FT       FT;

// using Point_map_region_growing = CGAL::Compose_property_map<CGAL::Random_access_property_map<Point_vector>, Point_map >;
// using Normal_map_region_growing = CGAL::Compose_property_map<CGAL::Random_access_property_map<Point_vector>, Normal_map >;
// using Region_type = CGAL::Shape_detection::Point_set::Least_squares_plane_fit_region<Kernel, std::size_t, Point_map_region_growing, Normal_map_region_growing>;
// using Neighbor_query = CGAL::Shape_detection::Point_set::Sphere_neighbor_query<Kernel, std::size_t, Point_map_region_growing>;
// using Region_growing = CGAL::Shape_detection::Region_growing<Neighbor_query, Region_type>;

#include <CGAL/IO/read_points.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>

void linkml::create_cell_complex(linkml::PointCloud& cloud, std::vector<pcl::PointIndices> const & clusters ){


	// const std::string input_file = CGAL::data_file_path("/home/mephisto/Downloads/ball.ply");
	// std::ifstream input_stream(input_file.c_str());
	// std::vector<PNI> points; // store points
	// std::cout << "Loading point cloud: " << input_file << "...";
	// CGAL::Timer t;
	// t.start();
	// if (!CGAL::IO::read_PLY_with_properties(input_stream,
	// 										std::back_inserter(points),
	// 										CGAL::make_ply_point_reader(Point_map()),
	// 										CGAL::make_ply_normal_reader(Normal_map()),
	// 										std::make_pair(Plane_index_map(), CGAL::PLY_property<int>("segment_index"))))
	// {
	// std::cerr << "Error: cannot read file " << input_file << std::endl;
	// }
	// else
	// std::cout << " Done. " << points.size() << " points. Time: " << t.time() << " sec." << std::endl;

	// linkml::point_cloud ball{};
	// for (auto & pni: points){
	// 	auto p = pni.get_head();
	// 	// auto n = pni.get_tail().get_head();
	// 	auto i = pni.get_tail().get_tail().get_head();
	// 	ball.pts.push_back(tg::pos3(p.x(), p.y(),p.z()));
	// 	ball.colors.push_back(get_color_forom_angle(sample_circle(i)));
	// }
	


	// Load point, normal and plan idx in to points
	// apparently -1 is a vailied value if a point does not belong to any plane.
	Point_vector points{};
	for (int i = 0; i < clusters.size(); i++){
		for (int j = 0; j < clusters[i].indices.size(); j++){

			auto point_idx = clusters[i].indices[j];
			auto plane_idx =  i;

			auto pt = cloud->points[point_idx].getPos();
			auto norm = cloud->points[point_idx].getNormal();

			points.push_back(PNI(Point(pt.x, pt.y, pt.z), Vector(norm.x, norm.y, norm.z), plane_idx ));
		}
	}



		
	// // Shape detection.
	// // Default parameter values for the data file cube.pwn.
	// const FT          search_sphere_radius = FT(2) / FT(100);
	// const FT          max_distance_to_plane = FT(2) / FT(1000);
	// const FT          max_accepted_angle = FT(25);
	// const std::size_t min_region_size = 200;

	// Point_map_region_growing point_map_rg(CGAL::make_random_access_property_map(points));
	// Normal_map_region_growing normal_map_rg(CGAL::make_random_access_property_map(points));


	// // Create instances of the classes Neighbor_query and Region_type.
	// Neighbor_query neighbor_query(
	// 	boost::irange<std::size_t>(0, points.size()), CGAL::parameters::sphere_radius(search_sphere_radius).point_map(point_map_rg));
	
	// Region_type region_type(
	// 	CGAL::parameters::
	// 	maximum_distance(max_distance_to_plane).
	// 	maximum_angle(max_accepted_angle).
	// 	minimum_region_size(min_region_size).
	// 	point_map(point_map_rg).
	// 	normal_map(normal_map_rg));
	
	// // Create an instance of the region growing class.
	// Region_growing region_growing(
	// 	boost::irange<std::size_t>(0, points.size()), neighbor_query, region_type);
	
	// std::cout << "Extracting planes...";
	// std::vector<typename Region_growing::Primitive_and_region> regions;
	
	// t.reset();
	// region_growing.detect(std::back_inserter(regions));
	// std::cout << " Done. " << regions.size() << " planes extracted. Time: "
	// 	<< t.time() << " sec." << std::endl;
	// // Stores the plane index of each point as the third element of the tuple.
	// for (std::size_t i = 0; i < points.size(); ++i)
	// 	// Uses the get function from the property map that accesses the 3rd element of the tuple.
	// 	points[i].get<2>() = static_cast<int>(get(region_growing.region_map(), i));
	// // Reconstruction.




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


	CellComplex cw;
	for (auto & idx: model.vertices()){
		auto h = cw.vertices().add();
		auto p = model.point(idx);
		cw.pos[h] = tg::pos3(p.x(), p.y(), p.z());
	}
	for (auto & face_index: model.faces()){

    	CGAL::Vertex_around_face_circulator<Surface_mesh> vcirc(model.halfedge(face_index), model), done(vcirc);
		std::vector<uint32_t> indices;
   		do {
			indices.push_back(*vcirc++);
		} while (vcirc != done);

		auto v0 = cw.vertices()[indices[0]];
		auto v1 = cw.vertices()[indices[1]];
		auto v2 = cw.vertices()[indices[2]];

		auto h = cw.faces().add(v0, v1, v2 );
	}

	polyscope::myinit();
	polyscope::display(cw);
	polyscope::display(cloud);
	polyscope::show();



	// // Saves the mesh model
	// const std::string& output_file("user_provided_planes_result.off");
	// if (CGAL::IO::write_OFF(output_file, model))
	// 	std::cout << " Done. Saved to " << output_file << ". Time: " << t.time() << " sec." << std::endl;
	// else {
	// 	std::cerr << " Failed saving file." << std::endl;
	// }
	

}
