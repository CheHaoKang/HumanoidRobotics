#include <gtest/gtest.h>
#include <camera_calibration/FileIO.h>
#include <camera_calibration/CameraCalibration.h>
#include <fstream>

using namespace camera_calibration;

class CameraCalibrationTest: public ::testing::Test {
protected:
	Eigen::Matrix3d K;
	Eigen::Matrix3d B;
	Eigen::Matrix3d B_;
	Eigen::Matrix3d K_;
	Eigen::Matrix3d R1R2T[3];
	Eigen::MatrixXd RT[3];
	Eigen::Matrix3d H[3];
	Eigen::MatrixXd P[3];
	double epsilon;

	virtual void SetUp() {
		epsilon = 1e-7;
		K << 1, 1, 3,
			 0, 2, 4,
			 0, 0, 1;
		B = K.inverse().transpose() * K.inverse();

		for (size_t i = 0; i < 3; ++i) {
			const Eigen::Vector3d phi = Eigen::Vector3d::Random();
			Eigen::Matrix3d R1, R2, R3;
			R1 << cos((double) phi(0)), -sin((double) phi(0)), 0,
				  sin((double) phi(0)),  cos((double) phi(0)), 0,
					0,                     0, 1;
			R2 << 1, 0, 0,
					0, cos((double) phi(1)), -sin((double) phi(1)),
					0, sin((double) phi(1)),  cos((double) phi(1));

			R3 <<  cos((double) phi(2)), 0, sin((double) phi(2)),
					0, 1, 0,
					-sin((double) phi(2)), 0, cos((double) phi(2));

			const Eigen::Matrix3d R = R1 * R2 * R3;
			// Make sure that R is in SO(3)
			EXPECT_NEAR(1.0, R.determinant(), epsilon);
			EXPECT_NEAR(1.0, R.col(0).norm(), epsilon);
			EXPECT_NEAR(1.0, R.col(1).norm(), epsilon);
			EXPECT_NEAR(1.0, R.col(2).norm(), epsilon);
			EXPECT_NEAR(0.0, R.col(0).transpose() * R.col(1), epsilon);
			EXPECT_NEAR(0.0, R.col(1).transpose() * R.col(2), epsilon);
			EXPECT_NEAR(0.0, R.col(0).transpose() * R.col(2), epsilon);

			const Eigen::Vector3d t = Eigen::Vector3d::Random();
			RT[i].resize(3, 4);
			RT[i] << R, t;
			R1R2T[i] << R.col(0), R.col(1), t;
			P[i] = K * RT[i];
			H[i] = K * R1R2T[i];

			EXPECT_NEAR(0.0, (R.col(0) - K.inverse() * H[i].col(0)).norm(), epsilon); // r1 = K^-1 h1
			EXPECT_NEAR(0.0, (R.col(1) - K.inverse() * H[i].col(1)).norm(), epsilon); // r2 = K^-1 h2
			EXPECT_NEAR(0.0, (double) (H[i].col(0).transpose() * K.inverse().transpose() * K.inverse() * H[i].col(1)), epsilon);
			const double d = H[i].col(0).transpose() * K.inverse().transpose() * K.inverse() * H[i].col(0);
			const double e = H[i].col(1).transpose() * K.inverse().transpose() * K.inverse() * H[i].col(1);
			EXPECT_NEAR(0.0, d - e, epsilon);

		}
		B_ = CameraCalibration::calculateB(HomographyVectorType(&H[0], &H[3]));
		K_ = CameraCalibration::calibrationMatrix(B_);
	}
};

TEST_F(CameraCalibrationTest, getV) {
	Eigen::Matrix3d H;
	H << 1, 2, 3,
	     4, 5, 6,
		 7, 8, 9;

	const Eigen::VectorXd v_00 = CameraCalibration::getV(H, 0, 0);
	const Eigen::VectorXd v_01 = CameraCalibration::getV(H, 0, 1);
	const Eigen::VectorXd v_11 = CameraCalibration::getV(H, 1, 1);
	ASSERT_DOUBLE_EQ( 1, (double) v_00(0));
	ASSERT_DOUBLE_EQ( 8, (double) v_00(1));
	ASSERT_DOUBLE_EQ(14, (double) v_00(2));
	ASSERT_DOUBLE_EQ(16, (double) v_00(3));
	ASSERT_DOUBLE_EQ(56, (double) v_00(4));
	ASSERT_DOUBLE_EQ(49, (double) v_00(5));
	ASSERT_DOUBLE_EQ( 2, (double) v_01(0));
	ASSERT_DOUBLE_EQ(13, (double) v_01(1));
	ASSERT_DOUBLE_EQ(22, (double) v_01(2));
	ASSERT_DOUBLE_EQ(20, (double) v_01(3));
	ASSERT_DOUBLE_EQ(67, (double) v_01(4));
	ASSERT_DOUBLE_EQ(56, (double) v_01(5));
	ASSERT_DOUBLE_EQ( 4, (double) v_11(0));
	ASSERT_DOUBLE_EQ(20, (double) v_11(1));
	ASSERT_DOUBLE_EQ(32, (double) v_11(2));
	ASSERT_DOUBLE_EQ(25, (double) v_11(3));
	ASSERT_DOUBLE_EQ(80, (double) v_11(4));
	ASSERT_DOUBLE_EQ(64, (double) v_11(5));

}

inline int sgn(const double val) {
    return (0.0 < val) - (val < 0.0);
}

TEST_F(CameraCalibrationTest, homographyFromPoints) {
	Eigen::Vector3d worldCoordinates[4];
	Eigen::Vector2d imageCoordinates[4];
	for (size_t r = 0; r < 3; ++r) {
		for (size_t i = 0; i < 4; ++i) {
			worldCoordinates[i] << Eigen::Vector2d::Random(), 0;
			Eigen::Vector4d wH; wH << worldCoordinates[i], 1;
			Eigen::Vector3d iH = P[r] * wH;
			imageCoordinates[i] << iH(0) / iH(2), iH(1) / iH(2);
		}
		const Eigen::Matrix3d H_ = CameraCalibration::homographyFromPoints(worldCoordinates, imageCoordinates);
		const Eigen::Matrix3d H_normalized = H[r].normalized();
		Eigen::Matrix3d H_computed_normalized = H_.normalized();
		if (sgn((double) H_normalized(2, 2)) * sgn((double) H_computed_normalized(2, 2)) == -1) {
			H_computed_normalized = -H_computed_normalized;
		}
		for (size_t i = 0; i < 3; ++i) {
			for (size_t j = 0; j < 3; ++j) {
				ASSERT_NEAR((double) H_normalized(i, j), (double) H_computed_normalized(i, j), epsilon);
			}
		}
	}
}

TEST_F(CameraCalibrationTest, calculateB) {
	const Eigen::Matrix3d B_normalized = B.normalized();
	Eigen::Matrix3d B_computed_normalized = B_.normalized();
	if (sgn((double) B_normalized(2, 2)) * sgn((double) B_computed_normalized(2, 2)) == -1) {
		B_computed_normalized = -B_computed_normalized;
	}

	for (size_t i = 0; i < 3; ++i) {
		for (size_t j = 0; j < 3; ++j) {
			ASSERT_NEAR((double) B_normalized(i, j), (double) B_computed_normalized(i, j), epsilon);
		}
	}
}

TEST_F(CameraCalibrationTest, calibrationMatrix) {
	Eigen::Matrix3d K_computed = CameraCalibration::calibrationMatrix(B * 2.0);
        if (fabs(1.0 - static_cast<double>(K_computed(2, 2))) > epsilon) {
                FAIL() << "The calibration matrix is not normalized. The bottom right entry should be 1.";
        }
	for (size_t i = 0; i < 3; ++i) {
		for (size_t j = 0; j < 3; ++j) {
			ASSERT_NEAR((double) K(i, j), (double) K_computed(i, j), epsilon);
		}
	}
}

TEST_F(CameraCalibrationTest, cameraPose) {
	for (size_t k = 0; k < 3; ++k) {
		const  Eigen::MatrixXd RT_ = CameraCalibration::cameraPose(H[k], K);
		EXPECT_NEAR(1.0, RT_.col(0).norm(), epsilon);
		EXPECT_NEAR(1.0, RT_.col(1).norm(), epsilon);
		EXPECT_NEAR(1.0, RT_.col(2).norm(), epsilon);
		EXPECT_NEAR(0.0, RT_.col(0).dot(RT_.col(1)), epsilon);
		EXPECT_NEAR(0.0, RT_.col(0).dot(RT_.col(2)), epsilon);
		EXPECT_NEAR(0.0, RT_.col(1).dot(RT_.col(2)), epsilon);
		for (size_t i = 0; i < 3; ++i) {
			for (size_t j = 0; j < 3; ++j) {
				EXPECT_NEAR((double) RT[k](i,j), (double) RT_(i, j), epsilon);
			}
		}
	}
}

TEST_F(CameraCalibrationTest, finalCalibrationMatrix) {
	for(size_t i = 0; i < 3; ++i) {
		for (size_t j = 0; j < 3; ++j) {
			EXPECT_NEAR((double) K(i,j), (double) K_(i,j), epsilon);
		}
	}
}

int main(int argc, char *argv[]) {
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
