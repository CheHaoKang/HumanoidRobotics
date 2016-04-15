#ifndef CAMERA_CALIBRATION_H_
#define CAMERA_CALIBRATION_H_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>

namespace camera_calibration {

typedef std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d> > HomographyVectorType;

class CameraCalibration {
    CameraCalibration() {};
    virtual ~CameraCalibration() {};

public:
	static Eigen::Matrix3d homographyFromPoints(const Eigen::Vector3d worldCoordinates[4], const Eigen::Vector2d imageCoordinates[4]);
	static Eigen::VectorXd getV(const Eigen::Matrix3d& h, const size_t& i, const size_t& j);
	static Eigen::Matrix3d calculateB(const HomographyVectorType& homography);
	static Eigen::Matrix3d calibrationMatrix(const Eigen::Matrix3d& B);
	static Eigen::MatrixXd cameraPose(const Eigen::Matrix3d& homography, const Eigen::Matrix3d& K);

	static Eigen::VectorXd solveV(const Eigen::MatrixXd& V);

};

}  // namespace camera_calibration

#endif  // camera_calibration_H_
