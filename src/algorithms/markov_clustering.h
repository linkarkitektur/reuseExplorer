# pragma once
// import numpy as np
// from scipy.sparse import isspmatrix, dok_matrix, csc_matrix
// import sklearn.preprocessing
// from .utils import MessagePrinter


namespace linkml{
namespace markov_clustering{


/// @brief Retrieve the clusters from the matrix
/// @tparam ScalarT 
/// @param matrix The matrix produced by the MCL algorithm
/// @return A list of tuples where each tuple represents a cluster and contains the indices of the nodes belonging to the cluster
template <class ScalarT>
std::vector<std::set<size_t>> get_clusters(Eigen::SparseMatrix<ScalarT> matrix);


/// @brief Perform MCL on the given similarity matrix
/// @tparam ScalarT 
/// @param matrix The similarity matrix to cluster
/// @param expansion The cluster expansion factor
/// @param inflation The cluster inflation factor
/// @param loop_value Initialization value for self-loops
/// @param iterations Maximum number of iterations (actual number of iterations will be less if convergence is reached)
/// @param pruning_threshold Threshold below which matrix elements will be set to 0
/// @param pruning_frequency Perform pruning every 'pruning_frequency' iterations.
/// @param convergence_check_frequency Perform the check for convergence every convergence_check_frequency iterations.
/// @param verbose Print extra information to the console
/// @return The final matrix
template <class ScalarT>
Eigen::SparseMatrix<ScalarT> run_mcl(Eigen::SparseMatrix<ScalarT> matrix, double expansion=2, double inflation=2, int loop_value=1,
            int iterations=100, double pruning_threshold=0.001, int pruning_frequency=1,
            int convergence_check_frequency=1, bool verbose=false);

}
}