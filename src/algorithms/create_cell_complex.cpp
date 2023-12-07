#include <algorithms/create_cell_complex.h>

#include <types/result_fit_planes.h>
#include <types/CellComplex.h>

#include <vector>
#include <execution>

#include <functions/progress_bar.h>
#include <functions/get_aabb.h>
#include <functions/color.h>
#include <functions/polyscope_helpers.h>
#include <functions/crop_plane_with_aabb.h>
#include <functions/convex_hull_2d.h>
#include <functions/project_2d.h>
#include <functions/split_cellcomplex_with_planes.h>
#include <functions/color_facetes.h>
#include <functions/decimate_cell_complex.h>
#include <functions/compute_coverage.h>
#include <functions/linear_program.h>
#include <functions/linear_program_solver.h>


#include <typed-geometry/tg.hh>
#include <typed-geometry/feature/std-interop.hh>


#include <clean-core/map.hh>

#include <polymesh/pm.hh>
#include <polymesh/copy.hh>
#include <polymesh/Mesh.hh>
#include <polymesh/algorithms.hh>
#include <polymesh/algorithms/deduplicate.hh>
#include <polymesh/algorithms/fill_hole.hh>
#include <polymesh/attributes/partitioning.hh>
#include <polymesh/ranges.hh>


#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"
#include "polyscope/point_cloud.h"
#include "polyscope/curve_network.h"

// template <class T>
auto surface_area(pm::minmax_t<tg::pos3> box){

	auto width  = box.max.x - box.min.x;
	auto height = box.max.y - box.min.y;
	auto depth  = box.max.z - box.min.z;

	return 2*(width*height + depth * height + width * depth);
}



void linkml::create_cell_complex(linkml::point_cloud& cloud, linkml::result_fit_planes& results){

    linkml::CellComplex cw;

    auto box = get_aabb(cloud.pts);

    // Intersect CW with bounding box to generate bounded planes.
    linkml::crop_plane_with_aabb(cw, box, results);

    // Intersect CW with planes
    linkml::split_cellcomplex_with_planes(cw, results);

    // Group facets
    // TODO: Simplify and rename
    // Make two class methods out of this.
    const std::vector<int> default_id(results.planes.size()+1, 0);
    cw.facets =  pm::face_attribute<std::vector<int>>(cw, default_id);
    linkml::color_facets(cw, results);
    int i = 0;
    auto cell_color_look_up = cw.facets.to_map([&](std::vector<int>){auto c = get_color_forom_angle(sample_circle(i)); i++; return c;});
    for (auto face : cw.faces()){
        cw.facet_colors[face] = cell_color_look_up[cw.facets[face]];
    }


    // Decimated
    linkml::decimate_cell_complex(cw);


    // Compute Coverage
    linkml::compute_coverage(cw, cloud);


    // Extract Adjacency
	// Adjacency extract_adjacency(Map* mesh) {
	// vertex_source_planes_.bind(mesh, "VertexSourcePlanes");

	// FIXME: Index vs Handle
	// an edge is denoted by its two end points
	typedef typename std::set<tg::pos3>										PosCollection;
	// PosCollection::iterator
	typedef typename tg::pos3*												UniquePos;
	//pm::halfedge_handle
	typedef typename std::map< UniquePos, std::set<int>>					Edge_map;
	typedef typename std::map< UniquePos, Edge_map >						Face_pool;
	struct SuperEdge : public std::vector<int> {
		UniquePos s;
		UniquePos t;
	};
	typedef typename std::vector<SuperEdge>									Adjacency;
	
	Face_pool face_pool;
	PosCollection unique_vertecies;
	for (auto & p: cw.pos) unique_vertecies.insert(p);

	for (auto h: cw.halfedges()){

		// Check if half edge has face
		if (h.face().is_invalid())
			continue;

		pm::vertex_handle sd = h.opposite().vertex_to();
		pm::vertex_handle td = h.vertex_to();

		// Obtain iterators to unique vertices
		auto s_iter = unique_vertecies.find(cw.pos[sd]);
		auto t_iter = unique_vertecies.find(cw.pos[td]);

		// Ensure that an edge is unique
		if (s_iter != unique_vertecies.end() && t_iter != unique_vertecies.end()) {
			UniquePos s = const_cast<UniquePos>(&(*s_iter));
			UniquePos t = const_cast<UniquePos>(&(*t_iter));

			if (s > t)
				std::swap(s, t);

			face_pool[s][t].insert(int(h));
		}
	}

	Adjacency fans;
	Face_pool::const_iterator it = face_pool.begin();
	for (; it != face_pool.end(); ++it) {
		const auto s = it->first;
		const Edge_map& tmp = it->second;
		Edge_map::const_iterator cur = tmp.begin();
		for (; cur != tmp.end(); ++cur) {
			const auto t = cur->first;
			const auto faces = cur->second;
			SuperEdge fan;
			fan.s = s;
			fan.t = t;

			fan.insert(fan.end(), faces.begin(), faces.end());
			fans.push_back(fan);
		}
	}

	// vertex_source_planes_.unbind();

		// return fans;
// }
	auto adjacency = fans;


    ////////////////////////////////////
    // New Section [ set up program ] //
    ////////////////////////////////////



    // Get the vertecies, edges and faces
	// edge_source_planes_.bind_if_defined(model_, "EdgeSourcePlanes");
	// vertex_source_planes_.bind_if_defined(model_, "VertexSourcePlanes");
	// facet_attrib_supporting_plane_.bind_if_defined(model_, "FacetSupportingPlane");

	//////////////////////////////////////////////////////////////////////////

	double total_points = double(cloud.pts.size());

    // Indecies of faces
	// std::size_t idx = 0;
    // auto facet_indices = cw.faces().make_attribute<int>();
    // auto facet_indices = cw.faces().to_map([&](pm::face_handle h){ 
    //     // facet_indices[h] = idx;
    //     // ++idx;
	// 	return int(h);
    //     });


	//-------------------------------------

	// StopWatch w;
    std::cout << "-" << "face selection..." << std::endl;

	//-------------------------------------

	// binary variables:
	// x[0] ... x[num_faces - 1] : binary labels of all the input faces
	// x[num_faces] ... x[num_faces + num_edges] : binary labels of all the intersecting edges (remain or not)
	// x[num_faces + num_edges] ... x[num_faces + num_edges + num_edges] : binary labels of corner edges (sharp edge of not)

	std::cout << "-" << "formulating binary program...." << std::endl;
	// w.start();

	std::size_t num_faces = cw.faces().size(); // model_->size_of_facets();
	std::size_t num_edges = 0;

	std::map<const SuperEdge*, std::size_t> edge_usage_status;	// keep or remove an intersecting edges
	for (std::size_t i = 0; i < adjacency.size(); ++i) {
		const SuperEdge& fan = adjacency[i];
		if (fan.size() == 4) {
			std::size_t var_idx = num_faces + num_edges;
			edge_usage_status[&fan] = var_idx;
			++num_edges;
		}
	}

	//double coeff_data_fitting = Method::lambda_data_fitting / total_points;
	//double coeff_coverage = Method::lambda_model_coverage / model_->bbox().area();
	//double coeff_complexity = Method::lambda_model_complexity / double(fans.size());
	// choose a better scale
	// TODO: Make those values configurable.
	// Also it apreas that the three values should be normalized where the sum of all three should be 1.
	double coeff_data_fitting = 0.43; //0.43
	double coeff_coverage = total_points * 0.27 / surface_area(cw.pos.aabb()); //0.27
	double coeff_complexity = total_points * 0.3 / double(adjacency.size()); //0.3



    // Make Program
    auto program_ = linkml::LinearProgram();
    program_.clear();
	LinearObjective* objective = program_.create_objective(LinearObjective::MINIMIZE);

	std::map<const SuperEdge*, std::size_t> edge_sharp_status;	// the edge is sharp or not
	std::size_t num_sharp_edges = 0;
	for (std::size_t i = 0; i < adjacency.size(); ++i) {
		const SuperEdge& fan = adjacency[i];
		if (fan.size() == 4) {
			std::size_t var_idx = num_faces + num_edges + num_sharp_edges;
			edge_sharp_status[&fan] = var_idx;

			// accumulate model complexity term
			objective->add_coefficient(var_idx, coeff_complexity);
			++num_sharp_edges;
		}
	}
	assert(num_edges == num_sharp_edges);


	for (auto f : cw.faces()){

		std::size_t var_idx = int(f); //facet_indices[f];

		// accumulate data fitting term
		double num = cw.supporting_point_num[f];
		objective->add_coefficient(var_idx, -coeff_data_fitting * num);

		// accumulate model coverage term
		double uncovered_area = (cw.facet_area[f] - cw.covered_area[f]);
		objective->add_coefficient(var_idx, coeff_coverage * uncovered_area);
	}

	std::size_t total_variables = num_faces + num_edges + num_sharp_edges;

	std::cout << "-" << "#total variables: " << total_variables << std::endl;
	std::cout << "-" << "    - face is selected: " << num_faces << std::endl;
	std::cout << "-" << "    - edge is used: " << num_edges << std::endl;
	std::cout << "-" << "    - edge is sharp: " << num_sharp_edges << std::endl;


	const std::vector<Variable*>& variables = program_.create_n_variables(total_variables);
	for (std::size_t i = 0; i < total_variables; ++i) {
		Variable* v = variables[i];
		v->set_variable_type(Variable::BINARY);
	}


	//////////////////////////////////////////////////////////////////////////

	// Add constraints: the number of faces associated with an edge must be either 2 or 0
	std::size_t var_edge_used_idx = 0;
	for (std::size_t i = 0; i < adjacency.size(); ++i) {
		LinearConstraint* c = program_.create_constraint(LinearConstraint::FIXED, 0.0, 0.0);
		const SuperEdge& fan = adjacency[i];
		for (std::size_t j = 0; j < fan.size(); ++j) {
			auto f = cw.halfedges()[fan[j]].face();
			std::size_t var_idx = int(f);//facet_indices[f];
			c->add_coefficient(var_idx, 1.0);
		}

		if (fan.size() == 4) {
			std::size_t var_idx = num_faces + var_edge_used_idx;
			c->add_coefficient(var_idx, -2.0);  // 
			++var_edge_used_idx;
		}
		else { // boundary edge
		    // will be set to 0 (i.e., we don't allow open surface)
		}
	}

	// Add constraints: for the sharp edges. The explanation of posing this constraint can be found here:
	// https://user-images.githubusercontent.com/15526536/30185644-12085a9c-942b-11e7-831d-290dd2a4d50c.png
	double M = 1.0;
	for (std::size_t i = 0; i < adjacency.size(); ++i) {
		const SuperEdge& fan = adjacency[i];
		if (fan.size() != 4)
			continue;

		// if an edge is sharp, the edge must be selected first:
		// X[var_edge_usage_idx] >= X[var_edge_sharp_idx]	
		LinearConstraint* c = program_.create_constraint();
		std::size_t var_edge_usage_idx = edge_usage_status[&fan];
		c->add_coefficient(var_edge_usage_idx, 1.0);
		std::size_t var_edge_sharp_idx = edge_sharp_status[&fan];
		c->add_coefficient(var_edge_sharp_idx, -1.0);
		c->set_bound(LinearConstraint::LOWER, 0.0);

		for (std::size_t j = 0; j < fan.size(); ++j) {
			auto f1 = cw.halfedges()[fan[j]].face();

			// FIXME: Suposedly this is invalid
			// Apparenty all faces are invalid?

			auto plane1 = cw.supporting_plans[f1];
			std::size_t fid1 = int(f1);//facet_indices[f1];
			for (std::size_t k = j + 1; k < fan.size(); ++k) {
				auto f2 = cw.halfedges()[fan[k]].face();
				auto plane2 = cw.supporting_plans[f2];
				std::size_t fid2 = int(f2);//facet_indices[f2];

				if (plane1 != plane2) {
					// the constraint is:
					//X[var_edge_sharp_idx] + M * (3 - (X[fid1] + X[fid2] + X[var_edge_usage_idx])) >= 1
					// which equals to  
					//X[var_edge_sharp_idx] - M * X[fid1] - M * X[fid2] - M * X[var_edge_usage_idx] >= 1 - 3M
					c = program_.create_constraint();
					c->add_coefficient(var_edge_sharp_idx, 1.0);
					c->add_coefficient(fid1, -M);
					c->add_coefficient(fid2, -M);
					c->add_coefficient(var_edge_usage_idx, -M);
					c->set_bound(LinearConstraint::LOWER, 1.0 - 3.0 * M);
				}
			}
		}
	}

#if 1
    // Add some optional constraints: border faces must be removed
    for (std::size_t i = 0; i < adjacency.size(); ++i) {
        const SuperEdge &fan = adjacency[i];
        if (fan.size() == 1) { // boundary edge
            auto f = cw.halfedges()[fan[0]].face();
            std::size_t var_idx = int(f);//facet_indices[f];
            LinearConstraint* c = program_.create_constraint(LinearConstraint::FIXED, 0.0, 0.0);
            c->add_coefficient(var_idx, 1.0);
        }
    }
#endif

	std::cout << "-" << "#total constraints: " << program_.constraints().size() << std::endl;
	// std::cout << "-" << "formulating binary program done. " << w.elapsed() << " sec" << std::endl;

	//////////////////////////////////////////////////////////////////////////

	// Optimize model
	std::cout << "-" << "solving the binary program. Please wait..." << std::endl;
	// w.start();

#if 0
    // Save the problem into a file (in lp format), allowing me to use other solvers to
    // solve it (easy to compare the performance of different solvers).
    program_.save("D:/tmp/bunny.lp");
#endif

	LinearProgramSolver solver;
	if (solver.solve(&program_,  LinearProgramSolver::SolverName::SCIP )) {
		// std::cout << "-" << "solving the binary program done. " << w.elapsed() << " sec" << std::endl;

		// mark results
		const std::vector<double>& X = solver.solution();
		std::vector<pm::face_handle> to_delete;
		for (auto f: cw.faces()){
			std::size_t fid = int(f);//facet_indices[f];
			//if (static_cast<int>(X[fid]) == 0) { // Liangliang: be careful, floating point!!!
			//if (static_cast<int>(X[fid]) != 1) { // Liangliang: be careful, floating point!!!
			if (static_cast<int>(std::round(X[fid])) == 0) {
				to_delete.push_back(f);
			}
		}

		for (auto f : to_delete)
			cw.faces().remove(f);

		//////////////////////////////////////////////////////////////////////////

		// // mark the sharp edges
		// MapHalfedgeAttribute<bool> edge_is_sharp(model_, "SharpEdge");
		// FOR_EACH_EDGE(Map, model_, it)
		// 	edge_is_sharp[it] = false;

		// for (std::size_t i = 0; i < adjacency.size(); ++i) {
		// 	const SuperEdge& fan = adjacency[i];
		// 	if (fan.size() != 4)
		// 		continue;

		// 	std::size_t idx_sharp_var = edge_sharp_status[&fan];
		// 	if (static_cast<int>(X[idx_sharp_var]) == 1) {
		// 		for (std::size_t j = 0; j < fan.size(); ++j) {
		// 			Map::Halfedge* e = fan[j];
		// 			Map::Facet* f = e->facet();
		// 			if (f) { // some faces may be deleted
		// 				std::size_t fid = facet_indices[f];
		// 				// if (static_cast<int>(X[fid]) == 1) { // Liangliang: be careful, floating point!!!
		// 				if (static_cast<int>(std::round(X[fid])) == 1) {
		// 					edge_is_sharp[e] = true;
		// 					break;
		// 				}
		// 			}
		// 		}
		// 	}
		// }
	}
	else {
        // std::cout << "-" << "solving the binary program failed. " << w.elapsed() << " sec." << std::endl;
	}

	// facet_attrib_supporting_vertex_group_.unbind();
	// facet_attrib_supporting_point_num_.unbind();
	// facet_attrib_facet_area_.unbind();
	// facet_attrib_covered_area_.unbind();

	// vertex_source_planes_.unbind();
	// edge_source_planes_.unbind();
	// facet_attrib_supporting_plane_.unbind();





    


    cw.compactify();
    // pm::deduplicate(cw, cw.pos);

    polyscope::init();

    polyscope::options::groundPlaneMode = polyscope::GroundPlaneMode::ShadowOnly;
    polyscope::view::setUpDir(polyscope::UpDir::ZUp);

    auto pcd = polyscope::registerPointCloud("Cloud", cloud.pts);
    pcd->setPointRadius(0.001);
    // auto pcd_color =  pcd->addColorQuantity("RGB", cloud.colors);
    // pcd_color->setEnabled(true)
    
    auto ps_mesh = polyscope::registerSurfaceMesh("Mesh", ps_helpers::vertecies(cw, cw.pos), ps_helpers::faces(cw));
    auto face_color = ps_mesh->addFaceColorQuantity("Face Color", cw.faces().map([&](pm::face_handle h){ 
        auto c = cw.plane_colors[h]; 
        return cc::array<float,3>{c.r,c.g,c.b};
    }).to_vector());
    auto facet_color = ps_mesh->addFaceColorQuantity("Facets Color", cw.faces().map([&](pm::face_handle h){ 
        auto c = cw.facet_colors[h]; 
        return cc::array<float,3>{c.r,c.g,c.b};
    }).to_vector());
    facet_color->setEnabled(true);
    ps_mesh->addFaceScalarQuantity("Coverage", cw.faces().map([&](pm::face_handle h){
        return cw.coverage[h];
    }).to_vector());

    
    polyscope::show();



}
