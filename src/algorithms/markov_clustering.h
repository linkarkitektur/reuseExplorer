# pragma once
#include <Eigen/Sparse>
#include <iostream>
#include <cassert>
#include <vector>
#include <set>
#include <limits>
#include <fmt/format.h>



// using SpMat = Eigen::SparseMatrix<scalar_t>; // declares a column-major sparse matrix type of double
// using Triplet = Eigen::Triplet<scalar_t>;


namespace linkml{
namespace markov_clustering{

class MessagePrinter
{
private:
    bool enabled = false;
public:
    MessagePrinter(bool enable) : enabled(enable) {}

    void enable() { enabled = true; }
    void disable() { enabled = false; }
    void print(std::string string) { if (enabled) std::cout << string << std::endl; }
};

    
/// @brief Version of np.allclose for use with sparse matrices
/// @tparam ScalarT 
/// @param a Matrix
/// @param b Matrix
/// @param rtol Relative tolerance
/// @param atol Absolute tolerance
/// @return 

template <class ScalarT>
bool sparse_allclose(Eigen::SparseMatrix<ScalarT> a, Eigen::SparseMatrix<ScalarT> b, double rtol=1e-5, double atol=1e-8){

    auto max = std::numeric_limits<ScalarT>::min();

    //using KeyT = typename std::decay<decltype(kf(m.vertices().first()))>::type;
    
    for (int k = 0; k < a.outerSize(); ++k){
        for (typename Eigen::SparseMatrix<ScalarT>::InnerIterator it(a, k); it; ++it){

            auto value1 = std::abs(it.value() - b.coeff(it.row(), it.col()));
            auto value2 = rtol * std::abs(b.coeff(it.row(), it.col()));

            auto difference = value1 - value2;

            if (difference > max)
                max = difference;
        }
    }
    
    return max <= atol;



    // auto c = Eigen::abs(a - b) - rtol * Eigen::abs(b);
    // noinspection PyUnresolvedReferences
    // return c.max() <= atol;
}


/// @brief Normalize the columns of the given matrix
/// @tparam ScalarT 
/// @param matrix The matrix to be normalized
/// @return The normalized matrix
template <class ScalarT>
Eigen::SparseMatrix<ScalarT> normalize(Eigen::SparseMatrix<ScalarT> matrix){

    // normalize each column

    // return sklearn.preprocessing.normalize(matrix, norm="l1", axis=0);

    for (int k = 0; k < matrix.outerSize(); ++k){
        auto sum = 0.0;
        for (typename Eigen::SparseMatrix<ScalarT>::InnerIterator it(matrix, k); it; ++it){
            sum += it.value();
        }
        for (typename Eigen::SparseMatrix<ScalarT>::InnerIterator it(matrix, k); it; ++it){
            it.valueRef() /= sum;
        }
    }

    return matrix;
}


/// @brief Apply cluster inflation to the given matrix by raising each element to the given power.
/// @tparam ScalarT 
/// @param matrix The matrix to be inflated
/// @param power Cluster inflation parameter
/// @return The inflated matrix
template <class ScalarT>
Eigen::SparseMatrix<ScalarT> inflate(Eigen::SparseMatrix<ScalarT> matrix, double power){

    // raise the matrix to the given power
    for (int k = 0; k < matrix.outerSize(); ++k){
        for (typename Eigen::SparseMatrix<ScalarT>::InnerIterator it(matrix, k); it; ++it){
            it.valueRef() = std::pow(it.value(), power);
        }
    }

    return normalize(matrix);


    // return normalize(matrix.power(power));
    // return normalize(np.power(matrix, power))

}


 
/// @brief Apply cluster expansion to the given matrix by raising the matrix to the given power.
/// @tparam ScalarT 
/// @param matrix The matrix to be expanded
/// @param power Cluster expansion parameter
/// @return The expanded matrix
template <class ScalarT>
Eigen::SparseMatrix<ScalarT> expand(Eigen::SparseMatrix<ScalarT> matrix, int power){

    if (power == 0){
        Eigen::SparseMatrix<ScalarT> eye(matrix.rows(), matrix.cols());
        eye.setIdentity();
        return eye;
    }

    for (int i = 0; i < power - 1; i++ ){
        matrix = matrix * matrix;
    }

    return matrix;
    
}


/// @brief Add self-loops to the matrix by setting the diagonal to loop_value
/// @tparam ScalarT 
/// @param matrix The matrix to add loops to
/// @param loop_value Value to use for self-loops
/// @return The matrix with self-loops
template <class ScalarT>
Eigen::SparseMatrix<ScalarT> add_self_loops(Eigen::SparseMatrix<ScalarT> matrix, int loop_value){

    assert( matrix.cols() == matrix.rows() && "Error, matrix is not square");

    // Enusre that the has a diagonal
    matrix += Eigen::VectorXd::Ones(matrix.cols()).template cast<ScalarT>().asDiagonal();
    
    //set diagonal to loop_value
    matrix.diagonal().array() << Eigen::VectorXd::Ones(matrix.cols()).template cast<ScalarT>() * static_cast<ScalarT>(loop_value);

    return matrix;
}



/// @brief Prune the matrix so that very small edges are removed. The maximum value in each column is never pruned.
/// @tparam ScalarT 
/// @param matrix The matrix to be pruned
/// @param threshold The value below which edges will be removed
/// @return  The pruned matrix
template <class ScalarT>
Eigen::SparseMatrix<ScalarT> prune(Eigen::SparseMatrix<ScalarT> matrix, ScalarT threshold){

    Eigen::SparseMatrix<ScalarT> pruned = Eigen::SparseMatrix<ScalarT>(matrix);


    // Set values below threshold to 0
    for (int col = 0; col < pruned.outerSize(); ++col){
        for (typename Eigen::SparseMatrix<ScalarT>::InnerIterator it(pruned, col); it; ++it) {
            if (it.value() < threshold) {
                it.valueRef() = 0;
            }
        }
    }
    // Remove 0 values
    pruned.prune(ScalarT(0));


    // if isspmatrix(matrix):
    //     pruned = dok_matrix(matrix.shape)
    //     pruned[matrix >= threshold] = matrix[matrix >= threshold]
    //     pruned = pruned.tocsc()
    // else:
    //     pruned = matrix.copy()
    //     pruned[pruned < threshold] = 0

    // keep max value in each column. same behaviour for dense/sparse


    // auto row_indices = matrix.argmax(axis=0).reshape((num_cols,));
    // pruned[row_indices, col_indices] = matrix[row_indices, col_indices];

    // Keep max value in each column
    for (int col = 0; col < matrix.outerSize(); ++col){
        ScalarT maxVal = -std::numeric_limits<ScalarT>::infinity();
        int maxRow = -1;
        for (typename Eigen::SparseMatrix<ScalarT>::InnerIterator it(matrix, col); it; ++it) {
            if (it.value() > maxVal) {
                maxVal = it.value();
                maxRow = it.row();
            }
        }
        if (maxRow != -1) {
            pruned.coeffRef(maxRow, col) = matrix.coeffRef(maxRow, col);
        }
    }

    // // auto row_indices = matrix.argmax(axis=0).reshape((num_cols,));
    // // pruned[row_indices, col_indices] = matrix[row_indices, col_indices];

    return pruned;
}


/// @brief Check for convergence by determining if matrix1 and matrix2 are approximately equal.
/// @tparam ScalarT 
/// @param matrix1 The matrix to compare with matrix2
/// @param matrix2 The matrix to compare with matrix1
/// @return True if matrix1 and matrix2 approximately equal
template <class ScalarT>
bool converged(Eigen::SparseMatrix<ScalarT> matrix1, Eigen::SparseMatrix<ScalarT> matrix2){

    // if isspmatrix(matrix1) or isspmatrix(matrix2):
    return sparse_allclose(matrix1, matrix2);

    // return np.allclose(matrix1, matrix2)
}




/// @brief Run a single iteration (expansion + inflation) of the mcl algorithm
/// @tparam ScalarT 
/// @param matrix The matrix to perform the iteration on
/// @param expansion Cluster expansion factor
/// @param inflation Cluster inflation factor
/// @return 
template <class ScalarT>
Eigen::SparseMatrix<ScalarT> iterate(Eigen::SparseMatrix<ScalarT> matrix, int expansion, double inflation){

    // Expansion
    matrix = expand(matrix, expansion);

    // Inflation
    matrix = inflate(matrix, inflation);

    return matrix;
}




/// @brief Retrieve the clusters from the matrix
/// @tparam ScalarT 
/// @param matrix The matrix produced by the MCL algorithm
/// @return A list of tuples where each tuple represents a cluster and contains the indices of the nodes belonging to the cluster
template <class ScalarT>
std::set<std::vector<size_t>> get_clusters(Eigen::SparseMatrix<ScalarT> matrix){

    // if not isspmatrix(matrix):
    //     // cast to sparse so that we don't need to handle different 
    //     // matrix types
    //     matrix = csc_matrix(matrix)

    // get the attractors - non-zero elements of the matrix diagonal
    auto attractors = std::vector<size_t>();
    for (int i = 0; i < matrix.rows(); i++)
        if (matrix.coeff(i, i) > 0)
            attractors.push_back(i);


    // somewhere to put the clusters
    auto clusters = std::set<std::vector<size_t>>();

    // the nodes in the same row as each attractor form a cluster
    for (auto attractor : attractors){
        // auto cluster = std::make_tuple<int>(matrix. getrow(attractor).nonzero()[1].tolist());
        auto cluster = std::vector<size_t>();
        for (int i = 0; i < matrix.cols(); i++)
            if (matrix.coeff(attractor, i) > 0)
                cluster.push_back(i);
        clusters.insert(cluster);
    }

    return clusters;

    // return sorted(list(clusters));
}



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
Eigen::SparseMatrix<ScalarT> run_mcl(Eigen::SparseMatrix<ScalarT> matrix, int expansion=2, double inflation=2, int loop_value=1,
            int iterations=100, ScalarT pruning_threshold=0.001, int pruning_frequency=1,
            int convergence_check_frequency=1, bool verbose=false){

    assert(expansion > 1 && "Invalid expansion parameter");
    assert(inflation > 1 && "Invalid inflation parameter");
    assert(loop_value >= 0 && "Invalid loop_value");
    assert(iterations > 0 && "Invalid number of iterations");
    assert(pruning_threshold >= 0 && "Invalid pruning_threshold");
    assert(pruning_frequency > 0 && "Invalid pruning_frequency");
    assert(convergence_check_frequency > 0 && "Invalid convergence_check_frequency");

    auto printer = MessagePrinter(verbose);

    printer.print(std::string(50, '-'));
    printer.print("MCL Parameters");
    printer.print(fmt::format("Expansion: {}", expansion));
    printer.print(fmt::format("Inflation: {}", inflation));
    if (pruning_threshold > 0){
        printer.print(fmt::format("Pruning threshold: {}, frequency: {} iteration{}",pruning_threshold, pruning_frequency, (pruning_frequency > 1) ? "s" : "" ));
    }
    else { printer.print("No pruning");}
    printer.print(fmt::format("Convergence check: {} iteration{}",convergence_check_frequency,  (convergence_check_frequency > 1) ? "s" : ""));
    printer.print(fmt::format("Maximum iterations: {}",iterations));
    printer.print(std::string(50, '-'));

    // Initialize self-loops
    if (loop_value > 0)
        matrix = add_self_loops(matrix, loop_value);

    // Normalize
    matrix = normalize(matrix);

    // iterations
    for (size_t i = 0; i < iterations; i++){
        printer.print(fmt::format("Iteration {}", i + 1));

        // store current matrix for convergence checking
        auto last_mat = Eigen::SparseMatrix<ScalarT>(matrix);

        // perform MCL expansion and inflation
        matrix = iterate(matrix, expansion, inflation);

        // prune
        if (pruning_threshold > 0 && i % pruning_frequency == pruning_frequency - 1){

            printer.print("Pruning");
            matrix = prune(matrix, pruning_threshold);

        }
        // Check for convergence
        if (i % convergence_check_frequency == convergence_check_frequency - 1){

            printer.print("Checking for convergence");
            if (converged(matrix, last_mat)){
                printer.print(fmt::format("Converged after {} iteration{}", i + 1, (i > 0 )? "s" : ""));
                break;
            }
        }
    }

    printer.print(std::string(50, '-'));


    return matrix;
}
}
}