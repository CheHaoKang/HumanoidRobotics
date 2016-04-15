#ifndef SIGNED_DISTANCE_FUNCTION_H_
#define SIGNED_DISTANCE_FUNCTION_H_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>

namespace signed_distance_function {

//http://eigen.tuxfamily.org/dox-devel/group__TopicStlContainers.html
typedef std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > VectorOfPoints;
struct Measurement {
	Eigen::Vector2d robotPose;
	VectorOfPoints laserPoints;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class SignedDistanceFunction {
public:
	SignedDistanceFunction() {};
	virtual ~SignedDistanceFunction() {};

	static double calculateDistance(const Eigen::Vector2d& pointA, const Eigen::Vector2d& pointB);
	static double truncateDistance(const double& signedDistance, const double& delta);
	static double calculateWeight(const double& signedDistance, const double& delta, const double& epsilon);
	static double updateMap(const double& signedDistance, const double& weight,
			const double& oldSignedDistance, const double& oldWeight);
	static double updateWeight(const double& weight, const double& oldWeight);
	static void integrateLaserScan(Eigen::MatrixXd& map, Eigen::MatrixXd& weights, const Measurement& measurement);
	static VectorOfPoints bresenham(const Eigen::Vector2d& pointA, const Eigen::Vector2d& pointB,
			const size_t& numRows, const size_t& numCols);


};

}  // signed_distance_function

#endif  // SIGNED_DISTANCE_FUNCTION_H_
