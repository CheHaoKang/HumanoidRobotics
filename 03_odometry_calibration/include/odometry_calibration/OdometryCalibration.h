#ifndef ODOMETRYCALIBRATION_H_
#define ODOMETRYCALIBRATION_H_

#include <odometry_calibration/CalibrationData.h>
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>

namespace odometry_calibration {

class OdometryCalibration {
public:
	OdometryCalibration() {};
	virtual ~OdometryCalibration() {};

	static Eigen::Vector3d errorFunction(const Odometry& groundTruth, const Odometry& observation, const Eigen::Matrix3d& calibrationMatrix);
	static Eigen::Matrix3Xd jacobian(const Odometry& observation);

	static Eigen::Matrix3d calibrateOdometry(const std::vector<MeasurementData>& measurements);
	static Odometry applyOdometryCorrection(const Odometry& uncalibratedOdometry, const Eigen::Matrix3d& calibrationMatrix);

	static Eigen::Matrix3d odometryToAffineTransformation(const Odometry& odometry);
	static Pose2D affineTransformationToPose(const Eigen::Matrix3d& transformation);
	static std::vector<Pose2D> calculateTrajectory(const std::vector<Odometry>& calibratedOdometry);
};

} /* namespace odometry_calibration */

#endif /* ODOMETRYCALIBRATION_H_ */
