/*
 * tensorInterface.h
 * @brief Interfacing tensors template library and gtsam
 * Created on: Feb 12, 2010
 * @author: Frank Dellaert
 */

#include "tensors.h"
#include "Matrix.h""

namespace gtsam {

	/** Reshape 3*3 rank 2 tensor into Matrix */
	template<class A, class I, class J>
	Matrix reshape(const tensors::Tensor2Expression<A, I, J>& T, int m, int n) {
		if (m * n != 9) throw std::invalid_argument(
				"reshape: incompatible dimensions");
		Matrix M(m, n);
		size_t t = 0;
		for (int j = 0; j < J::dim; j++)
			for (int i = 0; i < I::dim; i++)
				M.data()[t++] = T(i, j);
		return M;
	}

	/** Reshape Vector into rank 2 tensor */
	template<int N1, int N2>
	tensors::Tensor2<N1, N2> reshape2(const Vector& v) {
		if (v.size() != N1 * N2) throw std::invalid_argument(
				"reshape2: incompatible dimensions");
		double data[N1][N2];
		int t = 0;
		for (int j = 0; j < N2; j++)
			for (int i = 0; i < N1; i++)
				data[j][i] = v(t++);
		return tensors::Tensor2<N1, N2>(data);
	}

	/** Reshape rank 3 tensor into Matrix */
	template<class A, class I, class J, class K>
	Matrix reshape(const tensors::Tensor3Expression<A, I, J, K>& T, int m, int n) {
		if (m * n != I::dim * J::dim * K::dim) throw std::invalid_argument(
				"reshape: incompatible dimensions");
		Matrix M(m, n);
		int t = 0;
		for (int k = 0; k < K::dim; k++)
			for (int j = 0; j < J::dim; j++)
				for (int i = 0; i < I::dim; i++)
					M.data()[t++] = T(i, j, k);
		return M;
	}

	/** Reshape Vector into rank 3 tensor */
	template<int N1, int N2, int N3>
	tensors::Tensor3<N1, N2, N3> reshape3(const Vector& v) {
		if (v.size() != N1 * N2 * N3) throw std::invalid_argument(
				"reshape3: incompatible dimensions");
		double data[N1][N2][N3];
		int t = 0;
		for (int k = 0; k < N3; k++)
			for (int j = 0; j < N2; j++)
				for (int i = 0; i < N1; i++)
					data[k][j][i] = v(t++);
		return tensors::Tensor3<N1, N2, N3>(data);
	}

	/** Reshape rank 5 tensor into Matrix */
	template<class A, class I, class J, class K, class L, class M>
	Matrix reshape(const tensors::Tensor5Expression<A, I, J, K, L, M>& T, int m,
			int n) {
		if (m * n != I::dim * J::dim * K::dim * L::dim * M::dim) throw std::invalid_argument(
				"reshape: incompatible dimensions");
		Matrix R(m, n);
		int t = 0;
		for (int m = 0; m < M::dim; m++)
			for (int l = 0; l < L::dim; l++)
				for (int k = 0; k < K::dim; k++)
					for (int j = 0; j < J::dim; j++)
						for (int i = 0; i < I::dim; i++)
							R.data()[t++] = T(i, j, k, l, m);
		return R;
	}

	/**
	 * Find Vector v that minimizes algebraic error A*v
	 * Returns rank of A, minimum error, and corresponding eigenvector
	 */
	boost::tuple<int,double,Vector> DLT(const Matrix& A);

} // namespace gtsam
