#pragma once

#include <CGAL/Mixed_integer_program_traits.h>

#include <gurobi_c++.h>

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

namespace CGAL {

/// \ingroup PkgSolverInterfaceMIP
///
/// This class provides an interface for formulating and solving
/// constrained or unconstrained mixed integer programs using
/// \ref thirdparty GUROBI (which must be available on the system).
///
/// \cgalModels `MixedIntegerProgramTraits`
///
/// \sa `GLPK_mixed_integer_program_traits`
template <typename FT>
class GUROBI_mixed_integer_program_traits
  : public Mixed_integer_program_traits<FT>
{
  /// \cond SKIP_IN_MANUAL
public:
  typedef CGAL::Mixed_integer_program_traits<FT>          Base_class;
  typedef typename Base_class::Variable                   Variable;
  typedef typename Base_class::Linear_constraint          Linear_constraint;
  typedef typename Base_class::Linear_objective           Linear_objective;
  typedef typename Linear_objective::Sense                Sense;
  typedef typename Variable::Variable_type                Variable_type;

public:
  /// Solves the program. Returns `false` if fails.
  virtual bool solve()
  {
    Base_class::error_message_.clear();

	try {

		// I am using an academic license of Gurobi. Each time when a Gurobi environment is created, it pops up a
		// notice "Academic license - for non-commercial use only".
		// It is not possible to suppress this notice completely, but we can get the Gurobi environment only once and
		// reuse it later on.
		static GRBEnv env = GRBEnv();
		env.set(GRB_IntParam_LogToConsole, 0);

		GRBModel model = GRBModel(env);

		// create variables
		std::vector<GRBVar> X(Base_class::variables_.size());
		for (std::size_t i = 0; i < Base_class::variables_.size(); ++i) {
			const Variable* var = Base_class::variables_[i];

			double lb, ub;
			var->get_bounds(lb, ub);


			char vtype = GRB_CONTINUOUS;
			if (var->variable_type() == Variable::INTEGER)
				vtype = GRB_INTEGER;
			else if (var->variable_type() == Variable::BINARY)
				vtype = GRB_BINARY;
      // char vtype;
      // switch (var->variable_type())
      // {
      //   case Variable::CONTINUOUS:
      //     vtype = GRB_CONTINUOUS;
      //   case Variable::INTEGER:
      //     vtype = GRB_INTEGER;
      //     break;
      //   case Variable::BINARY:
      //     vtype = GRB_BINARY;
      //     break;
      // }

			X[i] = model.addVar(lb, ub, 0.0, vtype);

		}

		// Integrate new variables
		model.update();


		// Add constraints
		for (std::size_t i = 0; i < Base_class::constraints_.size(); ++i) {
			GRBLinExpr expr;
			const Linear_constraint* c = Base_class::constraints_[i];
      const std::unordered_map<const Variable*, double>& coeffs = c->coefficients();
      typename std::unordered_map<const Variable*, double>::const_iterator cur = coeffs.begin();

			for (; cur != coeffs.end(); ++cur) {
        std::size_t var_idx = cur->first->index();
				double coeff = cur->second;
				expr += coeff * X[var_idx];
			}

      double lb, ub;
      c->get_bounds(lb, ub);
      model.addConstr(expr >= lb);
      model.addConstr(expr <= ub);


			// switch (c->bound_type())
			// {
      //   case Linear_constraint::FIXED:
      //     model.addConstr(expr == c->get_bound());
      //     break;
      //   case Linear_constraint::LOWER:
      //     model.addConstr(expr >= c->get_bound());
      //     break;
      //   case Linear_constraint::UPPER:
      //     model.addConstr(expr <= c->get_bound());
      //     break;
      //   case Linear_constraint::DOUBLE: {
      //     double lb, ub;
      //     c->get_bounds(lb, ub);
      //     model.addConstr(expr >= lb);
      //     model.addConstr(expr <= ub);
      //     break;
      //     }
      //   default:
      //     break;
			// }
		}




		// Set objective
    // Determines the coefficient of each variable in the objective function
		GRBLinExpr obj;
    const std::unordered_map<const Variable*, double>& obj_coeffs = Base_class::objective_->coefficients();
		typename std::unordered_map<const Variable*, double>::const_iterator it = obj_coeffs.begin();
		for (; it != obj_coeffs.end(); ++it) {
      const Variable* var = it->first;
			double coeff = it->second;
			obj += coeff * X[var->index()];
		}



		// Set objective function sense
		bool minimize = (Base_class::objective_->sense() == Linear_objective::MINIMIZE);
		model.setObjective(obj, minimize ? GRB_MINIMIZE : GRB_MAXIMIZE);


#if 0 //Switched to async optimization
		// Optimize model
    //std::cout << " - " << "using the GUROBI solver (version " << GRB_VERSION_MAJOR << "." << GRB_VERSION_MINOR << ")." << std::endl;
		model.optimize();
#else

    std::cout << "Optimization started: " << std::this_thread::get_id() << std::endl;

    // timer that check if the model is still running and cnacels it if it takes too long
    model.optimizeasync();
    auto start = std::chrono::high_resolution_clock::now();

    while (model.get(GRB_IntAttr_Status) == GRB_INPROGRESS && std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - start).count() < 30){
      // std::cout << "Optimization is running: " << std::this_thread::get_id() << std::endl;
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    
    if (model.get(GRB_IntAttr_Status) != GRB_OPTIMAL){
      std::cout << "Optimization took too long, cancelling: " << std::this_thread::get_id() << std::endl;
      model.terminate();
    }


    std::cout << "Optimization finished: " << std::this_thread::get_id() << std::endl;
    model.sync();
    std::cout << "Optimization synced: " << std::this_thread::get_id() << std::endl;
#endif


    int status = model.get(GRB_IntAttr_Status);
    switch (status) {
		  case GRB_OPTIMAL: {
        // Base_class::objective_value_ = model.get(GRB_DoubleAttr_ObjVal);
        // auto objective_value_ = model.get(GRB_DoubleAttr_ObjVal);
			  Base_class::result_.resize(Base_class::variables_.size());
        for (std::size_t i = 0; i < Base_class::variables_.size(); ++i) {
          Base_class::result_[i] = X[i].get(GRB_DoubleAttr_X);
        }
        // upload_solution(Base_class::program_);
        break;
		  }
		
      case GRB_INF_OR_UNBD:
        std::cerr << "model is infeasible or unbounded" << std::endl;
        break;

      case GRB_INFEASIBLE:
        std::cerr << "model is infeasible" << std::endl;
        break;

      case GRB_UNBOUNDED:
        std::cerr << "model is unbounded" << std::endl;
        break;

      case GRB_INTERRUPTED:
        std::cerr << "optimization was interrupted" << std::endl;
        break;

      default:
        std::cerr << "optimization was stopped with status = " << status << std::endl;
        break;
		}

		return (status == GRB_OPTIMAL);
	}
	catch (GRBException e) {
        std::cout << " - " << e.getMessage() << " (error code: " << e.getErrorCode() << ")." << std::endl;
        if (e.getErrorCode() == GRB_ERROR_NO_LICENSE) {
            std::cout << " - "  << "Gurobi installed but license is missing or expired. Please choose another solver, e.g., SCIP." << std::endl;
        }
	  }
	catch (...) {
		std::cerr << "Exception during optimization" << std::endl;
  }

    return false;
  }
  /// \endcond
};

} // namespace CGAL







