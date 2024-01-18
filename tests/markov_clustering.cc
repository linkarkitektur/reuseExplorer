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

        TEST("Matrix expanded with power 2.0 and random values"){
            Eigen::SparseMatrix<double> matrix(3, 3);
            matrix.insert(0, 0) = 1.35707694;
            matrix.insert(0, 1) = 1.0320616;
            matrix.insert(0, 2) = 0.55297442;
            matrix.insert(1, 0) = 2.04922403;
            matrix.insert(1, 1) = 1.2799754;
            matrix.insert(1, 2) = 2.85502367;
            matrix.insert(2, 0) = 5.0;
            matrix.insert(2, 1) = 1.59869312;
            matrix.insert(2, 2) = 1.50298171;

            Eigen::SparseMatrix<double> expandedMatrix = expand(matrix, 2.0);

            CHECK( std::clamp(expandedMatrix.coeff(0, 0) - 6.72145533 , -1e-5, 1e-5));
            CHECK( std::clamp(expandedMatrix.coeff(0, 1) - 3.60563684 , -1e-5, 1e-5));
            CHECK( std::clamp(expandedMatrix.coeff(0, 2) - 4.52809956 , -1e-5, 1e-5));
            CHECK( std::clamp(expandedMatrix.coeff(1, 0) - 19.67902938, -1e-5, 1e-5));
            CHECK( std::clamp(expandedMatrix.coeff(1, 1) - 8.31756915 , -1e-5, 1e-5));
            CHECK( std::clamp(expandedMatrix.coeff(1, 2) - 9.07857691 , -1e-5, 1e-5));
            CHECK( std::clamp(expandedMatrix.coeff(2, 0) - 17.57637361, -1e-5, 1e-5));
            CHECK( std::clamp(expandedMatrix.coeff(2, 1) - 9.60940237 , -1e-5, 1e-5));
            CHECK( std::clamp(expandedMatrix.coeff(2, 2) - 9.58813282 , -1e-5, 1e-5));

        }

        TEST("Matrix expanded with power 5 and random values"){
            Eigen::SparseMatrix<double> matrix(3, 3);
            matrix.insert(0, 0) = 1.35707694;
            matrix.insert(0, 1) = 1.0320616;
            matrix.insert(0, 2) = 0.55297442;
            matrix.insert(1, 0) = 2.04922403;
            matrix.insert(1, 1) = 1.2799754;
            matrix.insert(1, 2) = 2.85502367;
            matrix.insert(2, 0) = 5.0;
            matrix.insert(2, 1) = 1.59869312;
            matrix.insert(2, 2) = 1.50298171;

            Eigen::SparseMatrix<double> expandedMatrix = expand(matrix, 5);

            CHECK( std::clamp(expandedMatrix.coeff(0, 0) - 998.82208405 , -1e-5, 1e-5));
            CHECK( std::clamp(expandedMatrix.coeff(0, 1) - 497.49529662 , -1e-5, 1e-5));
            CHECK( std::clamp(expandedMatrix.coeff(0, 2) - 547.46786862 , -1e-5, 1e-5));
            CHECK( std::clamp(expandedMatrix.coeff(1, 0) - 2342.45929475, -1e-5, 1e-5));
            CHECK( std::clamp(expandedMatrix.coeff(1, 1) - 1163.50184029, -1e-5, 1e-5));
            CHECK( std::clamp(expandedMatrix.coeff(1, 2) - 1279.30965158, -1e-5, 1e-5));
            CHECK( std::clamp(expandedMatrix.coeff(2, 0) - 2421.91378773, -1e-5, 1e-5));
            CHECK( std::clamp(expandedMatrix.coeff(2, 1) - 1206.05182561, -1e-5, 1e-5));
            CHECK( std::clamp(expandedMatrix.coeff(2, 2) - 1324.17647407, -1e-5, 1e-5));

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
        
        TEST("Self loop with missing identety diagonal"){
            // Create a non-square sparse matrix
            Eigen::SparseMatrix<double> matrix(3, 3);
            matrix.insert(0, 0) = 1.0;
            matrix.insert(1, 1) = 2.0;

            matrix = add_self_loops(matrix, 5);
            for (int i = 0; i<matrix.rows(); i++){
                CHECK(matrix.coeff(i, i) == 5);
            }
        }

        TEST("Pruning a 3x3 matrix"){
            // Create a sparse matrix
            Eigen::SparseMatrix<double> matrix(3, 3);
            matrix.insert(0, 0) = 1.35707694;
            matrix.insert(0, 1) = 1.0320616;
            matrix.insert(0, 2) = 0.55297442;
            matrix.insert(1, 0) = 2.04922403;
            matrix.insert(1, 1) = 1.2799754;
            matrix.insert(1, 2) = 2.85502367;
            matrix.insert(2, 0) = 5.0;
            matrix.insert(2, 1) = 1.59869312;
            matrix.insert(2, 2) = 1.50298171;


            // Call the prune function
            Eigen::SparseMatrix<double> prunedMatrix = prune(matrix, 2.0);



            // Check the values in the pruned matrix
            CHECK(prunedMatrix.coeff(0, 0) == 0.0);
            CHECK(prunedMatrix.coeff(0, 1) == 0.0);
            CHECK(prunedMatrix.coeff(0, 2) == 0.0);
            CHECK(prunedMatrix.coeff(1, 0) == 2.04922403);
            CHECK(prunedMatrix.coeff(1, 1) == 0.0);
            CHECK(prunedMatrix.coeff(1, 2) == 2.85502367);
            CHECK(prunedMatrix.coeff(2, 0) == 5.0);
            CHECK(prunedMatrix.coeff(2, 1) == 1.59869312);
            CHECK(prunedMatrix.coeff(2, 2) == 0.0);
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
            matrix.insert(0, 0) = 1.35707694;
            matrix.insert(0, 1) = 1.0320616;
            matrix.insert(0, 2) = 0.55297442;
            matrix.insert(1, 0) = 2.04922403;
            matrix.insert(1, 1) = 1.2799754;
            matrix.insert(1, 2) = 2.85502367;
            matrix.insert(2, 0) = 5.0;
            matrix.insert(2, 1) = 1.59869312;
            matrix.insert(2, 2) = 1.50298171;



            // Call the prune function with threshold zero
            Eigen::SparseMatrix<double> prunedMatrix = prune(matrix, 0.0);


            CHECK(prunedMatrix.coeff(0, 0) == 1.35707694);
            CHECK(prunedMatrix.coeff(0, 1) == 1.0320616 );
            CHECK(prunedMatrix.coeff(0, 2) == 0.55297442);
            CHECK(prunedMatrix.coeff(1, 0) == 2.04922403);
            CHECK(prunedMatrix.coeff(1, 1) == 1.2799754 );
            CHECK(prunedMatrix.coeff(1, 2) == 2.85502367);
            CHECK(prunedMatrix.coeff(2, 0) == 5.0);
            CHECK(prunedMatrix.coeff(2, 1) == 1.59869312);
            CHECK(prunedMatrix.coeff(2, 2) == 1.50298171);


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
            matrix.insert(0, 0) = 1.35707694;
            matrix.insert(0, 1) = 1.0320616;
            matrix.insert(0, 2) = 0.55297442;
            matrix.insert(1, 0) = 2.04922403;
            matrix.insert(1, 1) = 1.2799754;
            matrix.insert(1, 2) = 2.85502367;
            matrix.insert(2, 0) = 5.0;
            matrix.insert(2, 1) = 1.59869312;
            matrix.insert(2, 2) = 1.50298171;


            Eigen::SparseMatrix<double> iteratedMatrix = iterate(matrix, 2.0, 2.0);

            // Check the values in the iterated matrix
            CHECK( std::clamp(iteratedMatrix.coeff(0, 0) - 0.06093839, -1e-5, 1e-5));
            CHECK( std::clamp(iteratedMatrix.coeff(0, 1) - 0.0744922 , -1e-5, 1e-5));
            CHECK( std::clamp(iteratedMatrix.coeff(0, 2) - 0.1052245 , -1e-5, 1e-5));
            CHECK( std::clamp(iteratedMatrix.coeff(1, 0) - 0.52236217, -1e-5, 1e-5));
            CHECK( std::clamp(iteratedMatrix.coeff(1, 1) - 0.39640553, -1e-5, 1e-5));
            CHECK( std::clamp(iteratedMatrix.coeff(1, 2) - 0.42298073, -1e-5, 1e-5));
            CHECK( std::clamp(iteratedMatrix.coeff(2, 0) - 0.41669944, -1e-5, 1e-5));
            CHECK( std::clamp(iteratedMatrix.coeff(2, 1) - 0.52910227, -1e-5, 1e-5));
            CHECK( std::clamp(iteratedMatrix.coeff(2, 2) - 0.47179475, -1e-5, 1e-5));
        }

        TEST("Get clusters from a 3x3 matrix([[1,0,0],[0,2,0],[0,0,3]])"){
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

        TEST("Get clusters from a 3x3 matrix([[0,0,0],[1,1,1],[0,0,0]])"){
            Eigen::SparseMatrix<double> matrix(3, 3);
            matrix.insert(1, 0) = 1;
            matrix.insert(1, 1) = 1;
            matrix.insert(1, 2) = 1;

            std::set<std::vector<size_t>> clusters = get_clusters(matrix);

            // Check the number of clusters
            REQUIRE(clusters.size() == 1);

            // Check the indices in each cluster
            std::vector<size_t> cluster1 = {0, 1, 2};
            CHECK(clusters.count(cluster1) > 0);
        }

        TEST("Markov Clustering on a 3x3 matrix"){
            Eigen::SparseMatrix<double> matrix(3, 3);
            matrix.insert(0, 0) = 1.35707694;
            matrix.insert(0, 1) = 1.0320616;
            matrix.insert(0, 2) = 0.55297442;
            matrix.insert(1, 0) = 2.04922403;
            matrix.insert(1, 1) = 1.2799754;
            matrix.insert(1, 2) = 2.85502367;
            matrix.insert(2, 0) = 5.0;
            matrix.insert(2, 1) = 1.59869312;
            matrix.insert(2, 2) = 1.50298171;

            Eigen::SparseMatrix<double> finalMatrix = run_mcl(matrix, 2, 2.0, 1, 100, 0.001, 1, 1, false);

            // Check the values in the final matrix
            CHECK(finalMatrix.coeff(0, 0) == 0.0);
            CHECK(finalMatrix.coeff(0, 1) == 0.0);
            CHECK(finalMatrix.coeff(0, 2) == 0.0);
            CHECK(finalMatrix.coeff(1, 0) == 1.0);
            CHECK(finalMatrix.coeff(1, 1) == 1.0);
            CHECK(finalMatrix.coeff(1, 2) == 1.0);
            CHECK(finalMatrix.coeff(2, 0) == 0.0);
            CHECK(finalMatrix.coeff(2, 1) == 0.0);
            CHECK(finalMatrix.coeff(2, 2) == 0.0);
        }
    
    } // namespace markov_clustering
}
