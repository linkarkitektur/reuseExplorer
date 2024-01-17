#include <nexus/test.hh>
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
            
    } // namespace markov_clustering
}
