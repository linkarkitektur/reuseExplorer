# pragma once
#include <Eigen/Sparse>
#include <unsupported/Eigen/MatrixFunctions>

#include <iostream>
#include <cassert>
#include <vector>
#include <set>
#include <limits>
#include <fmt/format.h>


namespace linkml{
namespace markov_clustering{

/// @brief Helper class for printing messages to the console
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
static bool sparse_allclose(const Eigen::SparseMatrix<ScalarT> & a, const Eigen::SparseMatrix<ScalarT> & b, const double rtol=1e-5, const double  atol=1e-8){

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
template <class ScalarT>
static bool sparse_allclose(const Eigen::MatrixX<ScalarT> & a, const Eigen::MatrixX<ScalarT> & b, const double rtol=1e-5, const double atol=1e-8){
    auto max = std::numeric_limits<ScalarT>::min();

    #pragma omp parallel for collapse(2) reduction(max:max)
    for (size_t i = 0; i < (size_t)a.rows(); i++){
        for (size_t j = 0; j < (size_t)a.cols(); j++){
            auto value1 = std::abs(a(i, j) - b(i, j));
            auto value2 = rtol * std::abs(b(i, j));

            auto difference = value1 - value2;

            if (difference > max)
                max = difference;
        }
    }

    return max <= atol;
}

/// @brief Normalize the columns of the given matrix
/// @tparam ScalarT 
/// @param matrix The matrix to be normalized
/// @return The normalized matrix
template <class ScalarT>
static void normalize(Eigen::SparseMatrix<ScalarT> &  matrix){

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

}
template <class ScalarT>
static void normalize(Eigen::MatrixX<ScalarT> & matrix){

    #pragma omp parallel for
    for (size_t i = 0; i < (size_t)matrix.cols(); i++){

        ScalarT sum = 0.0;

        #pragma omp parallel for reduction(+:sum)
        for (size_t j = 0; j < (size_t)matrix.rows(); j++)
            sum += matrix(j, i);
        
        #pragma omp parallel for
        for (size_t j = 0; j < (size_t)matrix.rows(); j++)
            matrix(j, i) /= sum;
        
    }

}

/// @brief Apply cluster inflation to the given matrix by raising each element to the given power.
/// @tparam ScalarT 
/// @param matrix The matrix to be inflated
/// @param power Cluster inflation parameter
/// @return The inflated matrix
template <class ScalarT>
static void inflate(Eigen::SparseMatrix<ScalarT> & matrix, const double power){

    // raise the matrix to the given power
    for (int k = 0; k < matrix.outerSize(); ++k){
        for (typename Eigen::SparseMatrix<ScalarT>::InnerIterator it(matrix, k); it; ++it){
            it.valueRef() = std::pow(it.value(), power);
        }
    }

    normalize(matrix);

}
template <class ScalarT>
static void inflate(Eigen::MatrixX<ScalarT> & matrix, const double power){

    #pragma omp parallel for collapse(2)
    for (size_t i = 0; i < (size_t)matrix.rows(); i++)
        for (size_t j = 0; j < (size_t)matrix.cols(); j++)
            matrix(i,j) = std::pow(matrix(i, j), power);

    normalize(matrix);
}

/// @brief Apply cluster expansion to the given matrix by raising the matrix to the given power.
/// @tparam ScalarT 
/// @param matrix The matrix to be expanded
/// @param power Cluster expansion parameter
/// @return The expanded matrix
template <class ScalarT>
static void expand(Eigen::SparseMatrix<ScalarT> & matrix,const double power){

    for (int i=0; i < int(power) - 1; i++)
        matrix = matrix * matrix;

}
template <class ScalarT>
static void expand(Eigen::MatrixX<ScalarT> & matrix, const double power){

    for (int i=0; i < int(power) - 1; i++)
        matrix = matrix * matrix;

    //auto matrix_copy = Eigen::MatrixX<ScalarT>(matrix);
    //Eigen::MatrixPower<Eigen::MatrixX<ScalarT>> Pow(matrix_copy);
    //Pow.compute(matrix, power);
}

/// @brief Add self-loops to the matrix by setting the diagonal to loop_value
/// @tparam ScalarT 
/// @param matrix The matrix to add loops to
/// @param loop_value Value to use for self-loops
/// @return The matrix with self-loops
template <class ScalarT>
static void add_self_loops(Eigen::SparseMatrix<ScalarT> & matrix, int loop_value){

    assert( matrix.cols() == matrix.rows() && "Error, matrix is not square");

    //FIXME: This is not working

    // Enusring that the diagonal exists
    auto diagonal = Eigen::VectorXd::Ones(matrix.cols()).template cast<ScalarT>().asDiagonal();
    //matrix += diagonal;

    // Setting the diagonal to loop_value
    //matrix.diagonal() = Eigen::VectorXd::Constant(matrix.cols(), static_cast<ScalarT>(loop_value)).template cast<ScalarT>();

}
template <class ScalarT>
static void add_self_loops(Eigen::MatrixX<ScalarT> & matrix, const int loop_value){

    assert( matrix.cols() == matrix.rows() && "Error, matrix is not square");

    #pragma omp parallel for
    for (int i = 0; i < matrix.rows(); i++)
        matrix(i, i) = loop_value;
}

/// @brief Prune the matrix so that very small edges are removed. The maximum value in each column is never pruned.
/// @tparam ScalarT 
/// @param matrix The matrix to be pruned
/// @param threshold The value below which edges will be removed
/// @return  The pruned matrix
template <class ScalarT>
static Eigen::SparseMatrix<ScalarT> prune(Eigen::SparseMatrix<ScalarT> & matrix, const ScalarT threshold){

    Eigen::SparseMatrix<ScalarT> pruned = Eigen::SparseMatrix<ScalarT>(matrix);

    for (int k = 0; k < pruned.outerSize(); ++k){
        for (typename Eigen::SparseMatrix<ScalarT>::InnerIterator it(pruned, k); it; ++it){
            if (it.value() < threshold )
                it.valueRef() = 0;
        }
    }

    // std::cout << pruned << std::endl;

    // Ensure that the maximum value in each column is not pruned
    for (int k = 0; k < matrix.outerSize(); ++k){

        ScalarT max = std::numeric_limits<ScalarT>::min();
        int max_index = -1;

        for (typename Eigen::SparseMatrix<ScalarT>::InnerIterator it(matrix, k); it; ++it){
            if (it.value() > max){
                max = it.value();
                max_index = it.row();
            }
        }

        if (max_index != -1)
            pruned.coeffRef(max_index, k) = max;
    }

    // std::cout << pruned << std::endl;


    pruned.prune(threshold);
    return pruned;
}
template <class ScalarT>
static Eigen::MatrixX<ScalarT> prune(Eigen::MatrixX<ScalarT> & matrix, const ScalarT threshold){

    // TODO: Get rid of this copy operration

    Eigen::MatrixX<ScalarT> pruned = Eigen::MatrixX<ScalarT>(matrix);

    for (size_t i = 0; i < matrix.rows(); i++){
        for (size_t j = 0; j < matrix.cols(); j++){
            if (matrix(i, j) < threshold)
                pruned(i, j) = 0;
        }
    }

    for (size_t i = 0; i < matrix.cols(); i++){
        auto max = std::numeric_limits<ScalarT>::min();
        int max_index = -1;
        for (size_t j = 0; j < matrix.rows(); j++){
            if (matrix(j, i) > max){
                max = matrix(j, i);
                max_index = j;
            }
        }

        if (max_index != -1)
            pruned(max_index, i) = max;
    }


    return pruned;
}

/// @brief Check for convergence by determining if matrix1 and matrix2 are approximately equal.
/// @tparam ScalarT 
/// @param matrix1 The matrix to compare with matrix2
/// @param matrix2 The matrix to compare with matrix1
/// @return True if matrix1 and matrix2 approximately equal
template <class MatrixT>
inline static bool converged(const MatrixT & matrix1, const MatrixT & matrix2){

    return sparse_allclose(matrix1, matrix2);

}

/// @brief Run a single iteration (expansion + inflation) of the mcl algorithm
/// @tparam ScalarT 
/// @param matrix The matrix to perform the iteration on
/// @param expansion Cluster expansion factor
/// @param inflation Cluster inflation factor
/// @return 
template <class MatrixT>
static void iterate(MatrixT & matrix, const double expansion, const double inflation){

    // Expansion
    std::cout << "Expanding" << std::endl;
    expand(matrix, expansion);
    std::cout << "Inflating" << std::endl;
    // Inflation
    inflate(matrix, inflation);
    std::cout << "Done intteration" << std::endl;
}

/// @brief Retrieve the clusters from the matrix
/// @tparam ScalarT 
/// @param matrix The matrix produced by the MCL algorithm
/// @return A list of tuples where each tuple represents a cluster and contains the indices of the nodes belonging to the cluster
template <class MatrixT>
static std::set<std::vector<size_t>> get_clusters(MatrixT & matrix){

    // get the attractors - non-zero elements of the matrix diagonal
    auto attractors = std::vector<size_t>();
    for (int i = 0; i < matrix.rows(); i++)
        if (matrix.coeff(i, i) > 0)
            attractors.push_back(i);


    // somewhere to put the clusters
    auto clusters = std::set<std::vector<size_t>>();

    // the nodes in the same row as each attractor form a cluster
    for (auto attractor : attractors){
        auto cluster = std::vector<size_t>();
        for (int i = 0; i < matrix.cols(); i++)
            if (matrix.coeff(attractor, i) > 0)
                cluster.push_back(i);
        clusters.insert(cluster);
    }

    return clusters;
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
template <class MatrixT>
static MatrixT run_mcl(MatrixT & matrix, const int expansion=2, const double inflation=2, const int loop_value=1,
            const size_t iterations=100, const double pruning_threshold=0.001, const int pruning_frequency=1,
            const int convergence_check_frequency=1, const bool verbose=false){

    using ScalarT = typename MatrixT::value_type;

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
        add_self_loops(matrix, loop_value);

    // Normalize
    normalize(matrix);

    // iterations
    for (size_t i = 0; i < iterations; i++){
        printer.print(fmt::format("Iteration {}", i + 1));

        // store current matrix for convergence checking
        auto last_mat = MatrixT(matrix);

        // perform MCL expansion and inflation
        iterate(matrix, expansion, inflation);

        // prune
        if (pruning_threshold > 0 && i % pruning_frequency == pruning_frequency - 1){

            printer.print("Pruning");
            matrix = prune(matrix, ScalarT(pruning_threshold));
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