#include <projective_geometry/ProjectiveGeometry.h>
#include <math.h>
#include <iostream>

namespace projective_geometry 
{
const double ProjectiveGeometry::PI = 3.141592654;

/**
 * \brief Converts a 3D Euclidean coordinates to homogeneous coordinates.
 * \param[in] 3D point in Euclidean coordinates.
 * \return 3D point in homogeneous coordinates.
 */
Eigen::Vector4d ProjectiveGeometry::euclideanToHomogeneous(Eigen::Vector3d point)
{
	Eigen::Vector4d result;
	//TODO
	result << point.x(), point.y(), point.z(), 1;

	return result;
}
/**
 * \brief Converts a 2D point in homogeneous coordinates into Euclidean coordinates.
 * \param[in] 2D point in homogeneous coordinates.
 * \return 2D point in Euclidean coordinates.
 */
Eigen::Vector2d ProjectiveGeometry::homogeneousToEuclidean(Eigen::Vector3d point)
{
	Eigen::Vector2d result;
	//TODO
	result << point.x()/point.z(), point.y()/point.z();

	return result;
}
/**
 * \brief Assigns the values of the camera's extrinsic and intrinsic parameters.
 * \param[in] alpha The camera's current rotation angle.
 * \return a struct 'cameraParameters' which contains the camera parameters.
 */
cameraParameters ProjectiveGeometry::setCameraParameters(const double alpha)
{
	cameraParameters results;
	//TODO
	results.xH = 400;
	results.yH = 300;
	results.c = 550;
	results.m = 0.0025;

    results.X0.x() = 0.4;
    results.X0.y() = 0;
    results.X0.z() = 10;

    results.rotX = 0;
    results.rotY = alpha;
    results.rotZ = 0;

	return results;
}
/**
 * \brief Computes the calibration matrix based on the camera's intrinsic parameters.
 * \param[in] camera parameters (cameraParameters struct).
 * \return Calibration matrix.
 */
Eigen::Matrix3d ProjectiveGeometry::calibrationMatrix(cameraParameters param)
{
	Eigen::Matrix3d result;
	//TODO
	result << 	param.c, 0, param.xH,
				0, param.c*(1+param.m), param.yH,
				0, 0, 1;

	return result;

}
/**
 * \brief Computes the projection matrix based on the camera's parameters and the pre-computed calibration matrix.
 * \param[in] Calibration matrix.
 * \param[in] Camera parameters (cameraParameters struct).
 * \return Projection matrix.
 */
Eigen::MatrixXd ProjectiveGeometry::projectionMatrix(Eigen::Matrix3d calibrationMatrix,cameraParameters param)
{
	Eigen::MatrixXd result(3,4);
	//TODO
	Eigen::Matrix3d Rx;
	Eigen::Matrix3d Ry;
	Eigen::Matrix3d Rz;

	Rx << 	1, 0, 0,
			0, cos(param.rotX), -sin(param.rotX),
			0, sin(param.rotX), cos(param.rotX);

	Ry << 	cos(param.rotY), 0, sin(param.rotY),
			0, 1, 0,
			-sin(param.rotY), 0, cos(param.rotY);

	Rz << 	cos(param.rotZ), -sin(param.rotZ), 0,
			sin(param.rotZ), cos(param.rotZ), 0,
			0, 0, 1;
	
	Eigen::Matrix3d frontTransformation = Rx*Ry*Rz;
	Eigen::MatrixXd rearTransformation(3, 1);
	rearTransformation = -frontTransformation*(param.X0);
	Eigen::MatrixXd transformation(3, 4);

	transformation << 	frontTransformation(0, 0), frontTransformation(0, 1), frontTransformation(0, 2), rearTransformation(0),
						frontTransformation(1, 0), frontTransformation(1, 1), frontTransformation(1, 2), rearTransformation(1),
						frontTransformation(2, 0), frontTransformation(2, 1), frontTransformation(2, 2), rearTransformation(2);

	result = calibrationMatrix*transformation;

	return result;
}
/**
 * \brief Applies the pre-computed projection matrix on a 3D point in Euclidean coordinates.
 * \param[in] 3D point in Euclidean coordinates.
 * \param[in] Projection matrix.
 * \return 2D point in Euclidean coordinates.
 */
Eigen::Vector2d ProjectiveGeometry::projectPoint(Eigen::Vector3d point,Eigen::MatrixXd projectionMatrix)
{
	Eigen::Vector2d result;
	//TODO
//	Eigen::Vector4d homogeneous;
//	homogeneous = euclideanToHomogeneous(point);

	result = homogeneousToEuclidean(projectionMatrix*euclideanToHomogeneous(point));

	return result;	
}


}  // namespace projective_geometry
