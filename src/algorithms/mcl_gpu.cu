#include "algorithms/mcl.hh"

#include <Eigen/Dense>

#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <cusolverDn.h>
#include <cublas_v2.h>
#include <functional>
#include <iostream>
#include <vector>
#include <cmath>
#include <cassert>

#include <map>
#include <vector>


using std::vector;
using std::map;


namespace mcl
{
	template<>
	class mcl<MCLAlgorithm::GPU>
	{
	private:

		//Eigen::MatrixXd const& M;
        double* d_M;
        std::size_t numRows;
        std::size_t numCols;
        std::function< void(size_t cls_i, size_t mem_j) > ClusterResultCallback;

        constexpr static double epsilon = 0.00001;
        constexpr static double threshold = 0.5;
		
		cudaError_t cudaStat;
		cublasStatus_t stat;
		cublasHandle_t handle;

	public:
		//! @brief MCL ctor. Register a callback function to return the cluster results
		mcl_gpu(Eigen::MatrixXd const& M, std::function< void(size_t cluster_j, size_t member_i) > f) 
            : /*M(M),*/ ClusterResultCallback(f), numRows(M.rows()), numCols(M.cols()) {

			cudaStat =  cudaMalloc(&d_M, sizeof(double) * numRows * numCols);
            cudaMemcpy(d_M, M.data(), sizeof(double) * numRows * numCols, cudaMemcpyHostToDevice);

			if (cudaStat != cudaSuccess) {
				printf ("device memory allocation failed");
				free (d_M);
				throw std::runtime_error("device memory allocation failed");
			}


			//stat = cublasCreate(&handle);
			//if (stat != CUBLAS_STATUS_SUCCESS) {
			//	printf ("CUBLAS initialization failed\n");
			//	free (a);
			//	cublasDestroy(handle);
			//	throw std::runtime_error("CUBLAS initialization failed");
			//}

			////stat = cublasSetMatrix (Min.rows(), Min.cols(), sizeof(*a), a, M, devPtrA, M);
			//if (stat != CUBLAS_STATUS_SUCCESS) {
			//	printf ("data download failed");
			//	free (a);
			//	cudaFree (devPtrA);
			//	cublasDestroy(handle);
			//	//return EXIT_FAILURE;
			//}
			

			// https://docs.nvidia.com/cuda/pdf/CUBLAS_Library.pdf
		}
		~mcl_gpu() {
			cudaFree(d_M);
		}

		/*! @brief Apply Markov clustering algorithm with specified parameters and return clusters
			For each cluster, returns the list of node-ids that belong to that cluster
			*/
		Eigen::MatrixXd cluster_mcl(double expand_factor = 2, double inflate_factor = 2, double max_loop = 10, double mult_factor = 1)
		{
            addSelfLoop(d_M, mult_factor, numRows, numCols);
            normalize(d_M, numRows, numCols);

            for (int i = 0; i < max_loop && !stop(d_M, numRows, numCols); i++)
            {
                expand(d_M, expand_factor, numRows, numCols);
                inflate(d_M, inflate_factor, numRows, numCols);
            }

            Eigen::MatrixXd M_normalized(numRows, numCols);
            cudaMemcpy(M_normalized.data(), d_M, sizeof(double) * numRows * numCols, cudaMemcpyDeviceToHost);

            for (size_t row = 0; row < numRows; row++)
            {
                if (M_normalized.data()[row * numCols + row] > threshold)
                {
                    for (size_t col = 0; col < numCols; col++)
                    {
                        if (M_normalized.data()[row * numCols + col] > threshold)
                            ClusterResultCallback(row, col);
                    }
                }
            }
            return M_normalized;
		}

	private:

		bool stop(double* d_in, size_t rows, size_t cols)
		{
			// Implement the stop condition on GPU
			// Here, implement your logic for the stop condition using CUDA kernels

			// For simplicity, assume false for now
			return true;
		}

		void addSelfLoop(double* d_in, double mult_factor, size_t rows, size_t cols)
		{
			// Implement adding self-loop on GPU
			// Kernel to add self-loop
			//addSelfLoopKernel<<<(rows * cols + 255) / 256, 256>>>(d_in, mult_factor, rows, cols);
			cudaDeviceSynchronize();
		}

        void normalize(double* d_in, size_t rows, size_t cols)
        {
            // Implement normalization on GPU
            // Kernel to normalize the matrix
            //normalizeKernel<<<(rows * cols + 255) / 256, 256>>>(d_in, rows, cols);
            cudaDeviceSynchronize();
        }

        void expand(double* d_in, double expand_factor, size_t rows, size_t cols)
        {
            // Implement expand operation on GPU using cuBLAS for matrix multiplication
            cublasHandle_t handle;
            cublasCreate(&handle);

            double alpha = 1.0;
            double beta = 0.0;
            for (int i = 1; i < expand_factor; i++)
            {
                cublasDgemm(handle, CUBLAS_OP_N, CUBLAS_OP_N, rows, cols, rows, &alpha, d_in, rows, d_in, rows, &beta, d_in, rows);
            }

            cublasDestroy(handle);
        }

        void inflate(double* d_in, double inflate_factor, size_t rows, size_t cols)
        {
            // Implement inflate operation on GPU
            //inflateKernel<<<(rows * cols + 255) / 256, 256>>>(d_in, inflate_factor, rows, cols);
            normalize(d_in, rows, cols);
            cudaDeviceSynchronize();
        }

        static __global__ void addSelfLoopKernel(double* d_in, double mult_factor, size_t rows, size_t cols)
        {
            int idx = blockIdx.x * blockDim.x + threadIdx.x;
            if (idx < rows * cols)
            {
                int row = idx / cols;
                int col = idx % cols;
                if (row == col)
                {
                    d_in[idx] += mult_factor;
                }
            }
        }

        static __global__ void normalizeKernel(double* d_in, size_t rows, size_t cols)
        {
            int idx = blockIdx.x * blockDim.x + threadIdx.x;
            if (idx < rows * cols)
            {
                int col = idx % cols;
                double sum = 0.0;
                for (int row = 0; row < rows; row++)
                {
                    sum += d_in[row * cols + col];
                }
                if (sum > 0.0)
                {
                    for (int row = 0; row < rows; row++)
                    {
                        d_in[row * cols + col] /= sum;
                    }
                }
            }
        }

        static __global__ void inflateKernel(double* d_in, double inflate_factor, size_t rows, size_t cols)
        {
            int idx = blockIdx.x * blockDim.x + threadIdx.x;
            if (idx < rows * cols)
            {
                d_in[idx] = pow(d_in[idx], inflate_factor);
            }
        }



	};


}