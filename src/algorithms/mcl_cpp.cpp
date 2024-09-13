#include "algorithms/mcl.hh"

#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

#include <map>
#include <vector>


using std::vector;
using std::map;


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


constexpr static double epsilon = 0.00001;
constexpr static double threshold = 0.5;


namespace mcl
{

    //! @brief MCL ctor. Register a callback function to return the cluster results
    template<>
    mcl<MCLAlgorithm::CPP, Eigen::MatrixXd>::mcl(Eigen::MatrixXd& Min, std::function< void(size_t cluster_j, size_t member_i) > f): Data(Min), ClusterResultCallback(f) {}


    /*! @brief Apply Markov clustering algorithm with specified parameters and return clusters
    For each cluster, returns the list of node-ids that belong to that cluster
    */
    template<>
    void mcl<MCLAlgorithm::CPP, Eigen::MatrixXd>::cluster_mcl(double expand_factor, double inflate_factor, double max_loop, double mult_factor)
    {
			Eigen::MatrixXd M_selfloop = Data + (mult_factor * Eigen::MatrixXd::Identity(Data.cols(), Data.rows()));
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
}