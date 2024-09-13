#pragma once
#include <Eigen/Dense>

namespace mcl
{
	
	enum class MCLAlgorithm
	{
		CPP,
		CLI,
		GPU
	};

	template <enum MCLAlgorithm, typename Storage>
	class mcl
	{
	public:
		//! @brief MCL ctor. Register a callback function to return the cluster results
		mcl(Storage& Data, std::function< void(size_t cluster_j, size_t member_i) > f);

		/*! @brief Apply Markov clustering algorithm with specified parameters and return clusters
			For each cluster, returns the list of node-ids that belong to that cluster
			*/
		void cluster_mcl(double expand_factor = 2, double inflate_factor = 2, double max_loop = 10, double mult_factor = 1);

	private:
		Storage& Data;
		std::function< void(size_t cluster_j, size_t member_i) > ClusterResultCallback;


	};


    template class mcl<MCLAlgorithm::CLI, Eigen::MatrixXd>;
    template class mcl<MCLAlgorithm::CPP, Eigen::MatrixXd>;

}