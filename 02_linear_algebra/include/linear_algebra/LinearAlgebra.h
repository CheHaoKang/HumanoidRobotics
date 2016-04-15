#ifndef LINEAR_ALGEBRA_H_
#define LINEAR_ALGEBRA_H_

#include <Eigen/Core>

namespace linear_algebra{

class LinearAlgebra {
public:
	static Eigen::Vector3d vectorA();
	static Eigen::Vector3d vectorB();
	static Eigen::Matrix3d matrixM();
    static Eigen::Matrix3d invMatrixM(const Eigen::Matrix3d& M) ;
    static Eigen::Matrix3d transposeMatrixM(const Eigen::Matrix3d& M);
    static double detOfMatrixM(const Eigen::Matrix3d& M);
    static double dotProduct(const Eigen::Vector3d& a, const Eigen::Vector3d& b);
    static bool isLinearIndependent(const Eigen::Vector3d& a, const Eigen::Vector3d& b);
    static Eigen::Vector3d solveLinearSystem(const Eigen::Matrix3d& M, const Eigen::Vector3d& a);
};

}  // namespace linear_algebra

#endif  // LINEAR_ALGEBRA_H_
