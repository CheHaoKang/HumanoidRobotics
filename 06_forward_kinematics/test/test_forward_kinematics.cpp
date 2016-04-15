#include <forward_kinematics/ForwardKinematics.h>
#include <forward_kinematics/FileIO.h>
#include <ros/package.h>
#include <gtest/gtest.h>

using namespace forward_kinematics;

TEST(ForwardKinematicsTest, rotationX) {
	const double epsilon = 1e-7;
	Eigen::Vector4d X;
	X << 2, 3, 4, 1;
	Eigen::Matrix4d result = ForwardKinematics::rotationX(M_PI / 2);
	Eigen::Matrix4d real;
	real << 1, 0, 0, 0,
			0, 0, -1, 0,
			0, 1, 0, 0,
			0, 0, 0, 1;
	for (size_t i = 0; i < 4; ++i) {
		for (size_t j = 0; j < 4; ++j) {
			ASSERT_NEAR((double) real(i,j), (double) result(i,j), epsilon);
		}
	}
}

TEST(ForwardKinematicsTest, rotationZ) {
	const double epsilon = 1e-7;
	Eigen::Vector4d X;
	X << 2, 3, 4, 1;
	Eigen::Matrix4d result = ForwardKinematics::rotationZ(M_PI / 2);
	Eigen::Matrix4d real;
	real << 0, -1, 0, 0,
			1, 0, 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1;
	for (size_t i = 0; i < 4; ++i) {
		for (size_t j = 0; j < 4; ++j) {
			ASSERT_NEAR((double) real(i,j), (double) result(i,j), epsilon);
		}
	}
}

TEST(ForwardKinematicsTest, translationZ) {
	const double epsilon = 1e-7;
	Eigen::Vector4d X;
	X << 2, 3, 4, 1;
	Eigen::Matrix4d result = ForwardKinematics::translationZ(-3.0);
	Eigen::Matrix4d real;
	real << 1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1,-3,
			0, 0, 0, 1;
	for (size_t i = 0; i < 4; ++i) {
		for (size_t j = 0; j < 4; ++j) {
			ASSERT_NEAR((double) real(i,j), (double) result(i,j), epsilon);
		}
	}
}


TEST(ForwardKinematicsTest, translationX) {
	const double epsilon = 1e-7;
	Eigen::Vector4d X;
	X << 2, 3, 4, 1;
	Eigen::Matrix4d result = ForwardKinematics::translationX(-3.0);
	Eigen::Matrix4d real;
	real << 1, 0, 0,-3,
			0, 1, 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1;
	for (size_t i = 0; i < 4; ++i) {
		for (size_t j = 0; j < 4; ++j) {
			ASSERT_NEAR((double) real(i,j), (double) result(i,j), epsilon);
		}
	}
}

TEST(ForwardKinematicsTest, getA) {
	const double epsilon = 1e-5;
	ForwardKinematics::DH dh;
	dh.a = 1.0;
	dh.alpha = M_PI / 4.0;
	dh.d = 2.0;
	dh.theta = -M_PI / 4.0;
	const double q = M_PI / 3.0;
	Eigen::Matrix4d result = ForwardKinematics::getA(dh, q);
	Eigen::Matrix4d real;
	real << 0.965926, -0.183013,  0.183013,  0.965926,
			0.258819,  0.683013, -0.683013,  0.258819,
			       0,  0.707107,  0.707107,         2,
			       0,         0,         0,         1;

	for (size_t i = 0; i < 4; ++i) {
		for (size_t j = 0; j < 4; ++j) {
			ASSERT_NEAR((double) real(i,j), (double) result(i,j), epsilon);
		}
	}
}

TEST(ForwardKinematicsTest, computeHandTransform) {
	const double epsilon = 1e-5;
	const std::string packagePath = ros::package::getPath("forward_kinematics");
	if (packagePath.empty()) {
		FAIL() << "Error: Could not find package forward_kinematics. Make sure that you call source/devel/setup.bash first.";
	}
	ForwardKinematics fk;
	FileIO fio(packagePath + "/data/joints.txt");
	fio.results.reserve(fio.jointAngles.size());
	for (size_t i = 0; i < fio.jointAngles.size(); ++i) {
		 Eigen::Matrix4d result = fk.computeHandTransform(&fio.jointAngles[i][0]);
		 Eigen::Vector3d translation = result.topRightCorner(3, 1);
		 for (size_t j = 0; j < 3; ++j) {
			 ASSERT_NEAR((double) fio.groundTruth[i][j], (double) translation(j), epsilon);
		 }
	}

}


int main(int argc, char *argv[]) {
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
