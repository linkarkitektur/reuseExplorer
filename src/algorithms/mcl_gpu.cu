#pragma once

#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

#include <map>
#include <vector>

#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <cublas_v2.h>

using std::vector;
using std::map;


namespace mcl_cpp
{
	class mcl_gpu
	{
	private:
	cudaError_t cudaStat;
	cublasStatus_t stat;
	cublasHandle_t handle;
	size_t i, j;
	float* devPtrA;
	float* a;

	public:
		//! @brief MCL ctor. Register a callback function to return the cluster results
		mcl_gpu(const Eigen::MatrixXd& Min, std::function< void(size_t cluster_j, size_t member_i) > f): ClusterResultCallback(f) {
			cudaStat = cudaMalloc(&M, Min.rows() * Min.cols() * sizeof(float));
			if (cudaStat != cudaSuccess) {
				printf ("device memory allocation failed");
				//free (a);
				//return EXIT_FAILURE;
			}

			stat = cublasCreate(&handle);
			if (stat != CUBLAS_STATUS_SUCCESS) {
				printf ("CUBLAS initialization failed\n");
				free (a);
				cudaFree (devPtrA);
				//return EXIT_FAILURE;
			}

			//stat = cublasSetMatrix (Min.rows(), Min.cols(), sizeof(*a), a, M, devPtrA, M);
			if (stat != CUBLAS_STATUS_SUCCESS) {
				printf ("data download failed");
				free (a);
				cudaFree (devPtrA);
				cublasDestroy(handle);
				//return EXIT_FAILURE;
			}

			// https://docs.nvidia.com/cuda/pdf/CUBLAS_Library.pdf
			// Example 2

			//cudaMemcpy(M, Min.data(), Min.rows() * Min.cols() * sizeof(float) , cudaMemcpyHostToDevice);
			//cublasSetMatrix(M.rows(), M.cols(), sizeof(float), M, M.rows(), M, M.rows());
		}
		~mcl_gpu() {
			cudaFree(M);
		}

		/*! @brief Apply Markov clustering algorithm with specified parameters and return clusters
			For each cluster, returns the list of node-ids that belong to that cluster
			*/
		Eigen::MatrixXd cluster_mcl(double expand_factor = 2, double inflate_factor = 2, double max_loop = 10, double mult_factor = 1)
		{
			//Eigen::MatrixXd M_selfloop = M + (mult_factor * Eigen::MatrixXd::Identity(M.cols(), M.rows()));
			//Eigen::MatrixXd M_normalized = normalize(M_selfloop);
			//for (int i = 0; i < max_loop && !stop(M_normalized); i++)
			//{
			//	expand(M_normalized, expand_factor);
			//	inflate(M_normalized, inflate_factor);
			//}
			////return std::move(M_normalized);
			//for (auto row = 0; row<M_normalized.rows(); row++)
			//{
			//	if (M_normalized(row, row) > threshold)
			//	{
			//		for (auto col = 0; col < M_normalized.cols(); col++)
			//		{
			//			if (M_normalized.coeff(row, col) > threshold)
			//				ClusterResultCallback(row, col);
			//		}
			//	}
			//}
			//return std::move(M_normalized);
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

		float * M;
		std::function< void(size_t cls_i, size_t mem_j) > ClusterResultCallback;

		constexpr static double epsilon = 0.00001;
		constexpr static double threshold = 0.5;
	};






}