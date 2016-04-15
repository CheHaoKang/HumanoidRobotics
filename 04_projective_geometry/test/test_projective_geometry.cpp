#include <gtest/gtest.h>
#include <projective_geometry/ProjectiveGeometry.h>
#include <Eigen/Dense>
#include <cstdlib>

using namespace projective_geometry;

const double PI = 3.141592654;



TEST(ProjectiveGeometry, euclideanToHomogeneous) 
{
	Eigen::Vector3d x(1,3,2);
	Eigen::Vector4d result = ProjectiveGeometry::euclideanToHomogeneous(x);
	ASSERT_FLOAT_EQ(1, (double) result(0));
	ASSERT_FLOAT_EQ(3, (double) result(1));
	ASSERT_FLOAT_EQ(2, (double) result(2));
	ASSERT_FLOAT_EQ(1, (double) result(3));
}
TEST(ProjectiveGeometry, homogeneousToEuclidean) 
{
	Eigen::Vector3d x(6,9,3);
	Eigen::Vector2d result = ProjectiveGeometry::homogeneousToEuclidean(x);

	ASSERT_FLOAT_EQ(2, (double) result(0));
	ASSERT_FLOAT_EQ(3, (double) result(1));
}
TEST(ProjectiveGeometry, setCameraParameters) 
{
	cameraParameters result = ProjectiveGeometry::setCameraParameters(4.0*PI/180.0);
	ASSERT_FLOAT_EQ(400, (double) result.xH);
	ASSERT_FLOAT_EQ(300, (double) result.yH);
	ASSERT_FLOAT_EQ(0.0025, (double) result.m);
	ASSERT_FLOAT_EQ(550, (double) result.c);
	ASSERT_FLOAT_EQ(0.4, (double) result.X0(0));
	ASSERT_FLOAT_EQ(0, (double) result.X0(1));
	ASSERT_FLOAT_EQ(10, (double) result.X0(2));
	ASSERT_FLOAT_EQ(0, (double) result.rotX);
	ASSERT_FLOAT_EQ(4.0*PI/180.0, (double) result.rotY);
	ASSERT_FLOAT_EQ(0, (double) result.rotZ);
}
TEST(ProjectiveGeometry, calibrationMatrix) 
{
	cameraParameters param;
	param.xH = 500;		
	param.yH = 270;		
	param.m = 0.0022;		
	param.c = 570;		
	param.X0<<0.4,0,11;		
	param.rotX = 0;		
	param.rotY = 7*PI/180;	
	param.rotZ = 0;			
	Eigen::Matrix3d result = ProjectiveGeometry::calibrationMatrix(param);
	ASSERT_FLOAT_EQ(570, (double) result(0,0));
	ASSERT_FLOAT_EQ(0, (double) result(0,1));
	ASSERT_FLOAT_EQ(500, (double) result(0,2));
	ASSERT_FLOAT_EQ(0, (double) result(1,0));
	ASSERT_FLOAT_EQ(571.254, (double) result(1,1));
	ASSERT_FLOAT_EQ(270, (double) result(1,2));
	ASSERT_FLOAT_EQ(0, (double) result(2,0));
	ASSERT_FLOAT_EQ(0, (double) result(2,1));
	ASSERT_FLOAT_EQ(1, (double) result(2,2));
}
TEST(ProjectiveGeometry, projectionMatrix) 
{
	cameraParameters param;
	param.xH = 500;		
	param.yH = 270;		
	param.m = 0.0022;		
	param.c = 570;		
	param.X0<<0.4,0,11;		
	param.rotX = 0;		
	param.rotY = 7*PI/180;	
	param.rotZ = 0;		
	Eigen::Matrix3d calibrationMatrix;
	calibrationMatrix<<570,0,500,0,571.254,270,0,0,1;
	Eigen::MatrixXd result = ProjectiveGeometry::projectionMatrix(calibrationMatrix,param);
	const double epsilon = 0.01;
	ASSERT_NEAR(504.817,  (double) result(0,0), epsilon);
	ASSERT_NEAR(0,        (double) result(0,1), epsilon);
	ASSERT_NEAR(565.739,  (double) result(0,2), epsilon);
	ASSERT_NEAR(-6425.05, (double) result(0,3), epsilon);
	ASSERT_NEAR(-32.905,  (double) result(1,0), epsilon);
	ASSERT_NEAR(571.254,  (double) result(1,1), epsilon);
	ASSERT_NEAR(267.987,  (double) result(1,2), epsilon);
	ASSERT_NEAR(-2934.7,  (double) result(1,3), epsilon);
	ASSERT_NEAR(-0.122,   (double) result(2,0), epsilon);
	ASSERT_NEAR(0,        (double) result(2,1), epsilon);
	ASSERT_NEAR(0.993,    (double) result(2,2), epsilon);
	ASSERT_NEAR(-10.869,  (double) result(2,3), epsilon);
}
TEST(ProjectiveGeometry, projectPoint) 
{
	Eigen::Vector3d x(1,0,2);
	Eigen::MatrixXd projectionMatrix(3,4);
	projectionMatrix<<	504.817,	0,		565.739, 	-6425.05,
			  	-32.9047,	571.254,	267.987,	-2934.7,
				-0.121869,	0,		0.992546,	-10.8693;

	Eigen::Vector2d result = ProjectiveGeometry::projectPoint(x,projectionMatrix);

	ASSERT_NEAR(531.725, (double) result(0), 0.01);
	ASSERT_NEAR(269.999, (double) result(1), 0.01);
}
int main(int argc, char *argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();   
}
