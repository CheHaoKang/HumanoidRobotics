#include <linear_algebra/LinearAlgebra.h>
#include <iostream>
#include <cmath>

using namespace linear_algebra;

int main(int argc, char *argv[]) {
	const Eigen::Vector3d a = LinearAlgebra::vectorA();
	const Eigen::Vector3d b = LinearAlgebra::vectorB();
	const Eigen::Matrix3d M = LinearAlgebra::matrixM();

	std::cout << "The inverse of matrix M is: "<< std::endl  << LinearAlgebra::invMatrixM(M) << std::endl << std::endl;
	std::cout << "The transpose of matrix M is: "<< std::endl  << LinearAlgebra::transposeMatrixM(M) << std::endl << std::endl;
	std::cout << "The determinant of matrix M is: " << LinearAlgebra::detOfMatrixM(M) << std::endl << std::endl;
	std::cout << "The dot product of a and b is: " << LinearAlgebra::dotProduct(a, b) << std::endl << std::endl;
	std::cout << "a and b are linear " <<  (LinearAlgebra::isLinearIndependent(a, b) ? "independent" : "dependent") << std::endl << std::endl;
	std::cout << "Mx = a solved for x is: " << std::endl << LinearAlgebra::solveLinearSystem(M, a) << std::endl << std::endl;

    return 0;
}
