#include <linear_algebra/LinearAlgebra.h>
#include <Eigen/Dense>
#include <cmath>

namespace linear_algebra {

/**
 * This function should return the vector (2, 1, 3) as an Eigen vector.
 */
Eigen::Vector3d LinearAlgebra::vectorA() {
	Eigen::Vector3d result;

	// TODO: set the vector "result" to (2, 1, 3).
	result << 2, 1, 3;
	//std::cout<<"result:"<<result<<endl;

	return result;
}

/**
 * This function should return the vector (-1, -5, 2) as an Eigen vector.
 */
Eigen::Vector3d LinearAlgebra::vectorB() {
	Eigen::Vector3d result;

	// TODO: set the vector "result" to (-1, -5, 2).
	result << -1, -5, 2;
	//result << 4, 2, 6;
	//std::cout<<"result:"<<result<<endl;

	return result;
}

Eigen::Matrix3d LinearAlgebra::matrixM() {
	Eigen::Matrix3d result;

	// TODO: fill in the matrix elements
	result << 	1, 2, 7,
				0, 2, 0,
				1, 0, -1;
	//std::cout<<"result:"<<result<<endl;

	return result;
}

Eigen::Matrix3d LinearAlgebra::invMatrixM(const Eigen::Matrix3d& M) {
	Eigen::Matrix3d result;

	// TODO: return the inverse of matrix M
	result = M.inverse();

	return result;
}

Eigen::Matrix3d LinearAlgebra::transposeMatrixM(const Eigen::Matrix3d& M) {
	Eigen::Matrix3d result;

	// TODO: return the transpose of matrix M
	result = M.transpose();

	return result;
}

double LinearAlgebra::detOfMatrixM(const Eigen::Matrix3d& M)
{
	double result;

	// TODO: return the determinant of matrix M
	result = M.determinant();

	return result;
}

double LinearAlgebra::dotProduct(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
	double result;

	// TODO: return the dot product of vectors a and b.
	result = a.dot(b);

	return result;
}


bool LinearAlgebra::isLinearIndependent(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
	bool result = false;
	int rank;

	/* TODO: test if the vectors a and b are linear independent.
	   Return true if they are independent, false if they are dependent.*/
	Eigen::MatrixXd A(2,3);
	A << 	a.x(), a.y(), a.z(),
			b.x(), b.y(), b.z();
	//std::cout<<"A:"<<A<<endl;
	Eigen::FullPivLU<Eigen::MatrixXd> luA(A);
	rank = luA.rank();

	if (rank == 2)
		result = true;
	else
		result = false;

	return result;
}

Eigen::Vector3d LinearAlgebra::solveLinearSystem(const Eigen::Matrix3d& M, const Eigen::Vector3d& a) {
	Eigen::Vector3d result;

	// TODO: Solve Mx = a for x and return x.
	result = M.fullPivLu().solve(a);

	return result;
}

}  // namespace linear_algebra
