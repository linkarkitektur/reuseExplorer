#include <nexus/test.hh>
#include <nexus/config.hh>
#include <Eigen/Sparse>
#include <algorithms/markov_clustering.h>

namespace linkml {
    namespace markov_clustering
    {
        TEST("Sparse matrices are equal within tolerance") {
            Eigen::SparseMatrix<double> a(2, 2);
            a.insert(0, 0) = 1.0;
            a.insert(1, 1) = 2.0;

            Eigen::SparseMatrix<double> b(2, 2);
            b.insert(0, 0) = 1.00001;
            b.insert(1, 1) = 2.00001;

            double rtol = 1e-5;
            double atol = 1e-8;

            bool result = sparse_allclose(a, b, rtol, atol);

            CHECK(result);
        }

        TEST("Sparse matrices are not equal within tolerance"){
            Eigen::SparseMatrix<double> a(2, 2);
            a.insert(0, 0) = 1.0;
            a.insert(1, 1) = 2.0;

            Eigen::SparseMatrix<double> b(2, 2);
            b.insert(0, 0) = 1.0001;
            b.insert(1, 1) = 2.0001;

            double rtol = 1e-5;
            double atol = 1e-8;

            bool result = sparse_allclose(a, b, rtol, atol);

            CHECK(!result);
        }
        
        TEST("Normalizing a 3x3 matrix"){
            Eigen::SparseMatrix<double> matrix(3, 3);
            matrix.insert(0, 0) = 1.0;
            matrix.insert(0, 1) = 2.0;
            matrix.insert(0, 2) = 3.0;
            matrix.insert(1, 0) = 4.0;
            matrix.insert(1, 1) = 5.0;
            matrix.insert(1, 2) = 6.0;
            matrix.insert(2, 0) = 7.0;
            matrix.insert(2, 1) = 8.0;
            matrix.insert(2, 2) = 9.0;

            Eigen::SparseMatrix<double> normalized_matrix = normalize(matrix);

            Eigen::SparseMatrix<double> expected(3, 3);

            expected.insert(0, 0) = 0.08333333333333333;
            expected.insert(0, 1) = 0.13333333333333333;
            expected.insert(0, 2) = 0.16666666666666666;
            expected.insert(1, 0) = 0.3333333333333333;
            expected.insert(1, 1) = 0.3333333333333333;
            expected.insert(1, 2) = 0.3333333333333333;
            expected.insert(2, 0) = 0.5833333333333334;
            expected.insert(2, 1) = 0.5333333333333333;
            expected.insert(2, 2) = 0.5;

            for (int k = 0; k < expected.outerSize(); ++k)
                for (Eigen::SparseMatrix<double>::InnerIterator it(expected, k); it; ++it)
                    CHECK(normalized_matrix.coeff(it.row(), it.col()) == it.value());

        }
            
        TEST("Matrix inflated with power 2"){
            Eigen::SparseMatrix<double> matrix(3, 3);
            matrix.setIdentity();
            matrix.insert(2,0) = 2; 
            double power = 2.0;

            auto inflated_matrix = inflate(matrix, power);

            Eigen::SparseMatrix<double> expected(3, 3);
            expected.insert(0, 0) = 0.2;
            expected.insert(2, 0) = 0.8;
            expected.insert(1, 1) = 1.0;
            expected.insert(2, 2) = 1.0;

            for (int k = 0; k < expected.outerSize(); ++k)
                for (Eigen::SparseMatrix<double>::InnerIterator it(expected, k); it; ++it)
                    CHECK(inflated_matrix.coeff(it.row(), it.col()) == it.value());

        }

        TEST("Matrix inflated with power 0.5"){

            Eigen::SparseMatrix<double> random_matrix(3, 3);
            random_matrix.insert(0, 0) = 1.0;
            random_matrix.insert(0, 1) = 2.0;
            random_matrix.insert(1, 1) = 3.0;
            random_matrix.insert(2, 2) = 4.0;

            double power = 0.5;
            auto inflated_matrix = inflate(random_matrix, power);

            Eigen::SparseMatrix<double> expected(3, 3);
            expected.insert(0, 0) = 1.0;
            expected.insert(0, 1) = 0.4494897427831781;
            expected.insert(1, 1) = 0.5505102572168218;
            expected.insert(2, 2) = 1.0;

            for (int k = 0; k < expected.outerSize(); ++k)
                for (Eigen::SparseMatrix<double>::InnerIterator it(expected, k); it; ++it)
                    CHECK(inflated_matrix.coeff(it.row(), it.col()) == it.value());

        }
    
        TEST("Matrix expanded with power 2"){
            // Create a sparse matrix
            Eigen::SparseMatrix<double> matrix(3, 3);
            matrix.insert(0, 0) = 1.0;
            matrix.insert(1, 1) = 2.0;
            matrix.insert(2, 2) = 3.0;

            // Call the expand function
            Eigen::SparseMatrix<double> expandedMatrix = expand(matrix, 2.0);

            // Check the values in the expanded matrix
            CHECK(expandedMatrix.coeff(0, 0) == 1.0);
            CHECK(expandedMatrix.coeff(1, 1) == 4.0);
            CHECK(expandedMatrix.coeff(2, 2) == 9.0);
        }
        
        TEST("Expanding an empty matrix"){
            // Create an empty sparse matrix
            Eigen::SparseMatrix<double> matrix(0, 0);

            // Call the expand function
            Eigen::SparseMatrix<double> expandedMatrix = expand(matrix, 2.0);

            // Check that the expanded matrix is also empty
            CHECK(expandedMatrix.rows() == 0);
            CHECK(expandedMatrix.cols() == 0);
        }
        
        TEST("Identity Matrix with self-loops and value of 5"){
            // Create a sparse matrix
            Eigen::SparseMatrix<double> matrix(3, 3);
            matrix.insert(0, 0) = 1.0;
            matrix.insert(0, 1) = 1.0;
            matrix.insert(1, 1) = 2.0;
            matrix.insert(2, 2) = 3.0;

            // Call the add_self_loops function
            Eigen::SparseMatrix<double> matrixWithLoops = add_self_loops(matrix, 5);

            // Check the values in the matrix with self-loops
            CHECK(matrixWithLoops.coeff(0, 1) == 1); // Check that the original values are unchanged
            CHECK(matrixWithLoops.coeff(0, 0) == 5);
            CHECK(matrixWithLoops.coeff(1, 1) == 5);
            CHECK(matrixWithLoops.coeff(2, 2) == 5);
        }

        TEST("Empty matrix with self-loops and value of 5"){
            // Create an empty sparse matrix
            Eigen::SparseMatrix<double> matrix(0, 0);

            // Call the add_self_loops function
            Eigen::SparseMatrix<double> matrixWithLoops = add_self_loops(matrix, 5);

            // Check that the matrix with self-loops is also empty
            CHECK(matrixWithLoops.rows() == 0);
            CHECK(matrixWithLoops.cols() == 0);
        }
        
        TEST("Non-square matrix with self-loops and value of 5", should_fail){
            // Create a non-square sparse matrix
            Eigen::SparseMatrix<double> matrix(2, 3);
            matrix.insert(0, 0) = 1.0;
            matrix.insert(1, 1) = 2.0;

            try
            {
                // Call the add_self_loops function
                // This should trigger an assertion error
                add_self_loops(matrix, 5);
                CHECK(true);
            }
            catch(const std::exception& e)
            {
                CHECK(false);
            }
        }

        TEST("Pruning a 3x3 matrix"){
            // Create a sparse matrix
            Eigen::SparseMatrix<double> matrix(3, 3);
            matrix.insert(0, 0) = 1.0;
            matrix.insert(1, 1) = 2.0;
            matrix.insert(2, 2) = 3.0;

            // Call the prune function
            Eigen::SparseMatrix<double> prunedMatrix = prune(matrix, 2.0);

            // Check the values in the pruned matrix
            CHECK(prunedMatrix.coeff(0, 0) == 0.0);
            CHECK(prunedMatrix.coeff(1, 1) == 2.0);
            CHECK(prunedMatrix.coeff(2, 2) == 3.0);
        }

        TEST("Pruning an empty matrix"){
            // Create an empty sparse matrix
            Eigen::SparseMatrix<double> matrix(0, 0);

            // Call the prune function
            Eigen::SparseMatrix<double> prunedMatrix = prune(matrix, 2.0);

            // Check that the pruned matrix is also empty
            CHECK(prunedMatrix.rows() == 0);
            CHECK(prunedMatrix.cols() == 0);
        }
        
        TEST("Pruning a matrix with threshold zero"){
            // Create a sparse matrix
            Eigen::SparseMatrix<double> matrix(3, 3);
            matrix.insert(0, 0) = 1.0;
            matrix.insert(1, 1) = 2.0;
            matrix.insert(2, 2) = 3.0;

            // Call the prune function with threshold zero
            Eigen::SparseMatrix<double> prunedMatrix = prune(matrix, 0.0);

            // Check that all values in the pruned matrix are zero
            for (int i = 0; i < prunedMatrix.rows(); i++) {
                for (int j = 0; j < prunedMatrix.cols(); j++) {
                    CHECK(prunedMatrix.coeff(i, j)== 0.0);
                }
            }
        }

        TEST("Check if two equal matrices converge"){
            Eigen::SparseMatrix<double> matrix1(3, 3);
            matrix1.insert(0, 0) = 1.0;
            matrix1.insert(1, 1) = 2.0;
            matrix1.insert(2, 2) = 3.0;

            Eigen::SparseMatrix<double> matrix2(3, 3);
            matrix2.insert(0, 0) = 1.0;
            matrix2.insert(1, 1) = 2.0;
            matrix2.insert(2, 2) = 3.0;

            bool result = converged(matrix1, matrix2);
            CHECK(result);
        }

        TEST("Single MC iteration"){
            Eigen::SparseMatrix<double> matrix(3, 3);
            matrix.insert(0, 0) = 1.0;
            matrix.insert(1, 1) = 2.0;
            matrix.insert(2, 2) = 3.0;

            Eigen::SparseMatrix<double> iteratedMatrix = iterate(matrix, 2.0, 2.0);

            // Check the values in the iterated matrix
            CHECK(iteratedMatrix.coeff(0, 0) == 1.0);
            CHECK(iteratedMatrix.coeff(1, 1) == 4.0);
            CHECK(iteratedMatrix.coeff(2, 2) == 9.0);
        }

        TEST("Get clusters from a 3x3 matrix"){
            Eigen::SparseMatrix<double> matrix(3, 3);
            matrix.insert(0, 0) = 1.0;
            matrix.insert(1, 1) = 2.0;
            matrix.insert(2, 2) = 3.0;

            std::set<std::vector<size_t>> clusters = get_clusters(matrix);

            // Check the number of clusters
            REQUIRE(clusters.size() == 3);

            // Check the indices in each cluster
            std::vector<size_t> cluster1 = {0};
            std::vector<size_t> cluster2 = {1};
            std::vector<size_t> cluster3 = {2};
            CHECK(clusters.count(cluster1) > 0);
            CHECK(clusters.count(cluster2) > 0);
            CHECK(clusters.count(cluster3) > 0);
        }

        TEST("Markov Clustering on a 3x3 matrix"){
            Eigen::SparseMatrix<double> matrix(3, 3);
            matrix.insert(0, 0) = 1.0;
            matrix.insert(1, 1) = 2.0;
            matrix.insert(2, 2) = 3.0;

            Eigen::SparseMatrix<double> finalMatrix = run_mcl(matrix, 2.0, 2.0, 1, 100, 0.001, 1, 1, false);

            // Check the values in the final matrix
            CHECK(finalMatrix.coeff(0, 0) == 1.0);
            CHECK(finalMatrix.coeff(1, 1) == 4.0);
            CHECK(finalMatrix.coeff(2, 2) == 9.0);
        }
    
    } // namespace markov_clustering
}
