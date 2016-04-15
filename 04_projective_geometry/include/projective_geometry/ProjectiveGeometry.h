#ifndef PROJECTIVE_GEOMETRY_H_
#define PROJECTIVE_GEOMETRY_H_

#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>

namespace projective_geometry 
{
struct cameraParameters{
	double xH;
	double yH;
	double m;
	double c;

	Eigen::Vector3d X0;
	double rotX;
	double rotY;
	double rotZ;
};

class ProjectiveGeometry 
{
public:
	static const double PI;
	ProjectiveGeometry() {};
	virtual ~ProjectiveGeometry() {};
	

	static Eigen::Vector4d euclideanToHomogeneous(Eigen::Vector3d point);
	static Eigen::Vector2d homogeneousToEuclidean(Eigen::Vector3d point);
	static cameraParameters setCameraParameters(const double alpha);
	static Eigen::Matrix3d calibrationMatrix(cameraParameters param);
	static Eigen::MatrixXd projectionMatrix(Eigen::Matrix3d calibrationMatrix,cameraParameters param);
	static Eigen::Vector2d projectPoint(Eigen::Vector3d point,Eigen::MatrixXd projectionMatrix);

};

}  // namespace projective_geometry

#endif  // PROJECTIVE_GEOMETRY_H_
