#include <gtest/gtest.h>
#include <odometry_calibration/FileIO.h>
#include <odometry_calibration/OdometryCalibration.h>
#include <ros/package.h>
#include <stdexcept>

using namespace odometry_calibration;

TEST(OdometryCalibration, ErrorFunction) {
	Odometry gt, uncalibrated;
	gt.ux = 1.0;
	gt.uy = 2.0;
	gt.utheta = 3.0;
	uncalibrated.ux = 1.1;
	uncalibrated.uy = 2.4;
	uncalibrated.utheta = 3.5;
	Eigen::Matrix3d calibrationMatrix;
	calibrationMatrix << 1.1, 2.2, 3.3, -1.1, -3.2, -5.2, 1.1, -3.5, 2.2;
	Eigen::Vector3d error = OdometryCalibration::errorFunction(gt, uncalibrated, calibrationMatrix);
	ASSERT_DOUBLE_EQ(-17.04, (double) error(0));
	ASSERT_DOUBLE_EQ( 29.09, (double) error(1));
	ASSERT_DOUBLE_EQ(  2.49, (double) error(2));
}

TEST(OdometryCalibration, Jacobian) {
	Odometry odometry;
	odometry.ux = 2.0;
	odometry.uy = 3.0;
	odometry.utheta = M_PI;
	const Eigen::Matrix3Xd jacobian = OdometryCalibration::jacobian(odometry);
	ASSERT_EQ(3, (int) jacobian.rows());
	ASSERT_EQ(9, (int) jacobian.cols());
	ASSERT_DOUBLE_EQ(-2.0,  (double) jacobian(0, 0));
	ASSERT_DOUBLE_EQ(-3.0,  (double) jacobian(0, 1));
	ASSERT_DOUBLE_EQ(-M_PI, (double) jacobian(0, 2));
	ASSERT_DOUBLE_EQ(0.0,   (double) jacobian(0, 3));
	ASSERT_DOUBLE_EQ(-M_PI, (double) jacobian(2, 8));
	ASSERT_DOUBLE_EQ(-15-3*M_PI, jacobian.sum());
}

TEST(OdometryCalibration, Calibration) {
	FileIO fileIO;
	std::string path = ros::package::getPath("odometry_calibration") + "/data/calib.dat";
	try {
		fileIO.loadFromFile(path.c_str());
	} catch(const std::runtime_error& e) {
		ASSERT_TRUE(false) << "Could not load calibration data file";
	}
	Eigen::Matrix3d calibrationMatrix = OdometryCalibration::calibrateOdometry(fileIO.measurements);
	ASSERT_NEAR(-0.0754092, calibrationMatrix.determinant(), 1e-5);
}

TEST(OdometryCalibration, OdometryToAffine) {
	Odometry odometry;
	odometry.ux = 2.0;
	odometry.uy = -1.0;
	odometry.utheta = M_PI / 2;
	const Eigen::Matrix3d matrix = OdometryCalibration::odometryToAffineTransformation(odometry);
	ASSERT_NEAR(0.0, (double) matrix(0, 0), 1e-5);
	ASSERT_DOUBLE_EQ(-1.0, (double) matrix(0, 1));
	ASSERT_DOUBLE_EQ(odometry.ux, (double) matrix(0, 2));
	ASSERT_DOUBLE_EQ( 1.0, (double) matrix(1, 0));
	ASSERT_NEAR(0.0, (double) matrix(1, 1), 1e-5);
	ASSERT_DOUBLE_EQ(odometry.uy, (double) matrix(1, 2));
	ASSERT_DOUBLE_EQ( 0.0, (double) matrix(2, 0));
	ASSERT_DOUBLE_EQ( 0.0, (double) matrix(2, 1));
	ASSERT_DOUBLE_EQ( 1.0, (double) matrix(2, 2));
}

TEST(OdometryCalibration, AffineToPose) {
	Eigen::Matrix3d affine;
	affine << 0.0, -1.0,  5.0,
			  1.0,  0.0, -4.0,
			  0.0,  0.0,  1.0;
	const Pose2D pose = OdometryCalibration::affineTransformationToPose(affine);
	ASSERT_DOUBLE_EQ( 5.0, pose.x);
	ASSERT_DOUBLE_EQ(-4.0, pose.y);
	ASSERT_DOUBLE_EQ(M_PI/2, pose.theta);
}

TEST(OdometryCalibration, CalculateTrajectory) {
	Odometry odometry;
	std::vector<Odometry> ov;
	odometry.ux = 1.0;
	odometry.uy = 0.0;
	odometry.utheta = 0.0;
	ov.push_back(odometry);
	odometry.ux = 0.0;
	odometry.uy = 1.0;
	odometry.utheta = 0.0;
	ov.push_back(odometry);
	odometry.ux = 0.0;
	odometry.uy = 0.0;
	odometry.utheta = M_PI/2;
	ov.push_back(odometry);
	std::vector<Pose2D> poses = OdometryCalibration::calculateTrajectory(ov);
	ASSERT_EQ(ov.size(), poses.size());
	ASSERT_DOUBLE_EQ(1.0, poses[0].x);
	ASSERT_DOUBLE_EQ(0.0, poses[0].y);
	ASSERT_DOUBLE_EQ(0.0, poses[0].theta);
	ASSERT_DOUBLE_EQ(1.0, poses[1].x);
	ASSERT_DOUBLE_EQ(1.0, poses[1].y);
	ASSERT_DOUBLE_EQ(0.0, poses[1].theta);
	ASSERT_DOUBLE_EQ(1.0, poses[2].x);
	ASSERT_DOUBLE_EQ(1.0, poses[2].y);
	ASSERT_DOUBLE_EQ(M_PI/2, poses[2].theta);

}


int main(int argc, char *argv[]) {
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
