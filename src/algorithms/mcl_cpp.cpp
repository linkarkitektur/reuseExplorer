#include "algorithms/mcl.hh"

#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

#include <map>
#include <vector>


using std::vector;
using std::map;


namespace mcl
{

	template<>
	class mcl<MCLAlgorithm::CPP, Eigen::MatrixXd>
	{
	public:
		//! @brief MCL ctor. Register a callback function to return the cluster results
		mcl(const Eigen::MatrixXd& Min, std::function< void(size_t cluster_j, size_t member_i) > f) : M(Min), ClusterResultCallback(f) {}

		/*! @brief Apply Markov clustering algorithm with specified parameters and return clusters
			For each cluster, returns the list of node-ids that belong to that cluster
			*/
		void cluster_mcl(double expand_factor = 2, double inflate_factor = 2, double max_loop = 10, double mult_factor = 1)
		{
			Eigen::MatrixXd M_selfloop = M + (mult_factor * Eigen::MatrixXd::Identity(M.cols(), M.rows()));
			Eigen::MatrixXd M_normalized = normalize(M_selfloop);
			for (int i = 0; i < max_loop && !stop(M_normalized); i++)
			{
				expand(M_normalized, expand_factor);
				inflate(M_normalized, inflate_factor);
			}
			//return std::move(M_normalized);
			for (auto row = 0; row<M_normalized.rows(); row++)
			{
				if (M_normalized(row, row) > threshold)
				{
					for (auto col = 0; col < M_normalized.cols(); col++)
					{
						if (M_normalized.coeff(row, col) > threshold)
							ClusterResultCallback(row, col);
					}
				}
			}
			// return std::move(M_normalized);
		}

	private:
		bool stop(const Eigen::MatrixXd& in)
		{
			Eigen::MatrixXd m = in*in;
			Eigen::MatrixXd diff = (m - in);
			//double mx = diff.maxCoeff(), mn = diff.minCoeff();
			return (diff.maxCoeff() == diff.minCoeff());
		}

		Eigen::MatrixXd normalize(Eigen::MatrixXd& in)
		{
			Eigen::MatrixXd one_over_col_sum = in.colwise().sum().cwiseInverse();
			Eigen::MatrixXd M_normalized = in * one_over_col_sum.asDiagonal();
			return std::move(M_normalized);
		}

		inline void expand(Eigen::MatrixXd& in, double expand_factor)
		{
			//auto copy = Eigen::MatrixXd(in);
			//Eigen::MatrixPower<Eigen::MatrixXd> Apow(copy);
			//in = Apow(expand_factor);
			//in = in.pow(expand_factor);
			//Eigen::MatrixPower<Eigen::MatrixXd> Apow{};
			//Apow.compute(in, expand_factor);
			Eigen::MatrixXd copy;
			copy.resize(in.rows(), in.cols());
			Eigen::MatrixPower<Eigen::MatrixXd> Apow(in);
			Apow.compute(copy, expand_factor);
			in = std::move(copy);
		}

		void inflate(Eigen::MatrixXd& in, double inflate_factor)
		{
			auto lam = [inflate_factor](double x) -> double { return std::pow(x, inflate_factor); };
			in = in.unaryExpr(lam);
			in = normalize(in);
		}

		const Eigen::MatrixXd & M;
		std::function< void(size_t cls_i, size_t mem_j) > ClusterResultCallback;

		constexpr static double epsilon = 0.00001;
		constexpr static double threshold = 0.5;
	};






}