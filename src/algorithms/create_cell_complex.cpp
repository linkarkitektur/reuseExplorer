#include <algorithms/create_cell_complex.h>

#include <types/result_fit_planes.h>
#include <types/CellComplex.h>

#include <vector>
#include <map>
#include <execution>

#include <functions/progress_bar.h>
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
#include <functions/constuct_adjacency.h>


#include <typed-geometry/tg.hh>
#include <typed-geometry/types/objects/plane.hh>
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


// #include "polyscope/polyscope.h"
// #include "polyscope/surface_mesh.h"
// #include "polyscope/point_cloud.h"
// #include "polyscope/curve_network.h"
#include <functions/polyscope.h>

// template <class T>
auto surface_area(pm::minmax_t<tg::pos3> box){

	auto width  = box.max.x - box.min.x;
	auto height = box.max.y - box.min.y;
	auto depth  = box.max.z - box.min.z;

	return 2*(width*height + depth * height + width * depth);
}




void linkml::create_cell_complex(linkml::point_cloud& cloud, linkml::result_fit_planes& results){

	polyscope::myinit();

    linkml::CellComplex cw;


    auto box = cloud.get_bbox();

	polyscope::display(box);
	polyscope::display(cloud);
	polyscope::show();


	// FIXME: Remove hardcoded bounding box
	// box.max.x = 6.66951;
	// box.max.y = 3.48438;
	// box.max.z = 1.38951;
	// box.min.x = -4.31573;
	// box.min.y = -7.60451;
	// box.min.z = -1.86951;


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


	// Deduplicate edge vertecies
	auto cloud2 = linkml::point_cloud();
	for (int i = 0; i < results.planes.size(); i++){
		for (int j = i+1; j < results.planes.size(); j++){
			for ( int k = j+1; k < results.planes.size(); k++){

				tg::plane3 p0 = results.planes[i];
				tg::plane3 p1 = results.planes[j];
				tg::plane3 p2 = results.planes[k];

				if (tg::abs(tg::dot(p0.normal, p1.normal))-1 > 0.001) continue;
				auto l = tg::intersection(p0,p1);
					
				if (!tg::intersects(l, p2)) continue;
				auto pt = tg::intersection(l, p2).first();

				if (!tg::contains(box, pt)) continue;
				cloud2.pts.push_back(pt);

			}
		}
	}
	cloud2.buildIndex();
	for (auto& pt : cw.pos){
		auto r = cloud2.radiusSearch(pt, 0.005);
		if (r.size() >=1)
			pt = cloud2.pts[r[0]];
	}
	

    // Decimated
    linkml::decimate_cell_complex(cw);

	// Remove edges that are very short
	auto short_edges = cw.edges().where([&](pm::edge_handle h){ return tg::length(tg::segment3(cw.pos[h.vertexA()], cw.pos[h.vertexB()])) < 0.0001;});

	std::cout << "short_edges.count(): " << short_edges.count() << std::endl;
	for (auto h :short_edges)
		cw.edges().remove(h);

	cw.compactify();

    // Compute Coverage
    linkml::compute_coverage(cw, cloud);

    // Extract Adjacency
	auto adjacency = linkml::constuct_adjacency(cw);

	std::cout << "adjacency size: " << adjacency.size() << std::endl;



    ////////////////////////////////////
    // New Section [ set up program ] //
    ////////////////////////////////////


    // Get the vertecies, edges and faces
	// edge_source_planes_.bind_if_defined(model_, "EdgeSourcePlanes");
	// vertex_source_planes_.bind_if_defined(model_, "VertexSourcePlanes");
	// facet_attrib_supporting_plane_.bind_if_defined(model_, "FacetSupportingPlane");

	//////////////////////////////////////////////////////////////////////////

	double total_points = double(cloud.pts.size());
	std::cout << " - " << "total_points: " << total_points << std::endl;


    // Indecies of faces
	auto facets_map = std::map<std::vector<int>, std::unordered_set<pm::face_handle>>();
	for (auto f: cw.faces())
		facets_map[cw.facets[f]].insert(f);

	std::size_t idx = 0;
    auto facet_indices = cw.faces().make_attribute<int>();
	for (auto &pair : facets_map){
		for (auto f : pair.second){
			facet_indices[f] = idx;
		}
		idx++;
	}

	std::cout << " - " << "idx->facet_indices: " << idx << std::endl;



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

	// FIXME: Change to the actual count of facets.
	std::size_t num_faces = facets_map.size(); // model_->size_of_facets();
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

	
	std::cout << " - " << "double(adjacency.size()): " << double(adjacency.size()) << std::endl;
	std::cout << " - " << "surface_area(cw.pos.aabb()): " << surface_area(cw.pos.aabb()) << std::endl;

	std::cout << " - " << "coeff_data_fitting: " << coeff_data_fitting << std::endl;
	std::cout << " - " << "coeff_coverage: " << coeff_coverage << std::endl;
	std::cout << " - " << "coeff_complexity: " << coeff_complexity << std::endl;

    // Make Program
    auto program_ = linkml::LinearProgram();
    program_.clear();
	LinearObjective* objective = program_.create_objective(LinearObjective::MINIMIZE);

	std::map<const SuperEdge*, std::size_t> edge_sharp_status;	// the edge is sharp or not
	std::size_t num_sharp_edges = 0;
	for (std::size_t i = 0; i < adjacency.size(); ++i) {
		const SuperEdge& fan = adjacency[i];
		if (fan.size() == 4) {
			std::size_t var_idx = num_faces + num_edges + num_sharp_edges; // Running index
			edge_sharp_status[&fan] = var_idx;

			// accumulate model complexity term
			objective->add_coefficient(var_idx, coeff_complexity);
			++num_sharp_edges;
		}
	}

	assert(num_edges == num_sharp_edges);


	for (auto &pair: facets_map){
		// Get handle to any face in the facet
		// All faces in the facet shoul have identical information.
		auto f = *pair.second.begin();

		std::size_t var_idx = facet_indices[f];

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
			auto f = fan[j];
			std::size_t var_idx = facet_indices[f];
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
			auto f1 = fan[j];

			auto plane1 = cw.supporting_plans[f1];
			std::size_t fid1 = facet_indices[f1];
			for (std::size_t k = j + 1; k < fan.size(); ++k) {
				auto f2 = fan[k];
				auto plane2 = cw.supporting_plans[f2];
				std::size_t fid2 = facet_indices[f2];

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
            auto f = fan[0];
            std::size_t var_idx = facet_indices[f];
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

#if 1
    // Save the problem into a file (in lp format), allowing me to use other solvers to
    // solve it (easy to compare the performance of different solvers).
    program_.save("/home/mephisto/repos/linkml_cpp/test.lp");
#endif

	LinearProgramSolver solver;
	if (solver.solve(&program_,  LinearProgramSolver::SolverName::SCIP )) {
		// std::cout << "-" << "solving the binary program done. " << w.elapsed() << " sec" << std::endl;

		// mark results
		const std::vector<double>& X = solver.solution();
		std::vector<pm::face_handle> to_delete;

		for (auto &pair: facets_map){
			auto f = *pair.second.begin();
			std::size_t fid = facet_indices[f];
			//if (static_cast<int>(X[fid]) == 0) { // Liangliang: be careful, floating point!!!
			//if (static_cast<int>(X[fid]) != 1) { // Liangliang: be careful, floating point!!!
			if (static_cast<int>(std::round(X[fid])) == 0) {
				auto faces_of_facet = facets_map[cw.facets[f]];
				for (auto h : faces_of_facet)
					to_delete.push_back(h);
			}
		}


		for (auto f : to_delete) cw.faces().remove(f);
		

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




    


    cw.compactify();
    // pm::deduplicate(cw, cw.pos);


	polyscope::display(cw);
	polyscope::show();


}
