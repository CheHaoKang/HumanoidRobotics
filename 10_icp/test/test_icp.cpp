#include <gtest/gtest.h>
#include <icp/ICP.h>
#include <Eigen/Dense>
#include <cstdlib>
#include <math.h>
#include <vector>

using namespace icp;

TEST(ICP, distance) {
	Eigen::Vector2d p1,p2;

	p1<<-10,10;  p2<<5,7;
	ASSERT_NEAR( 15.2971, (double) ICP::distance(p1,p2) , 0.0001);
	p1<<-10,-3.3;  p2<<-7,2;
	ASSERT_NEAR( 6.0902, (double) ICP::distance(p1,p2) , 0.0001);
	p1<<0,2;  p2<<0,0;
	ASSERT_NEAR( 2.0, (double) ICP::distance(p1,p2), 0.0001);

}
TEST(ICP, closestPointOnLine) {
	Eigen::Vector2d p0,p1,p2;

	p0<< 8,2 ;  p1<<-10,10;  p2<<5,7;
	ASSERT_NEAR( 8.85, (double) ICP::closestPointOnLine(p0,p1,p2)(0) , 0.01);
	ASSERT_NEAR( 6.23, (double) ICP::closestPointOnLine(p0,p1,p2)(1) , 0.01);

	p0<< -6,3 ;  p1<<-10,-3.3;  p2<<-7,2;
	ASSERT_NEAR(-6.33, (double) ICP::closestPointOnLine(p0,p1,p2)(0) , 0.01);
	ASSERT_NEAR( 3.19, (double) ICP::closestPointOnLine(p0,p1,p2)(1) , 0.01);

	p0<< 4,-7 ;  p1<<0,2;  p2<<0,0;
	if (isnan((double) ICP::closestPointOnLine(p0,p1,p2)(0))) {
		FAIL() << "The implementation does not handle the special case of vertical lines (pL1.x == pL2.x) correctly.";
	}
	ASSERT_NEAR( 0, (double) ICP::closestPointOnLine(p0,p1,p2)(0) , 0.01);
	ASSERT_NEAR(-7, (double) ICP::closestPointOnLine(p0,p1,p2)(1) , 0.01);

	p0<< 1,1 ;  p1<<2,0;  p2<<0,0;
	if (isnan((double) ICP::closestPointOnLine(p0,p1,p2)(0))) {
		FAIL() << "The implementation does not handle the special case of horizontal lines (pL1.y == pL2.y) correctly.";
	}
	ASSERT_NEAR( 1, (double) ICP::closestPointOnLine(p0,p1,p2)(0) , 0.01);
	ASSERT_NEAR( 0, (double) ICP::closestPointOnLine(p0,p1,p2)(1) , 0.01);
}
TEST(ICP, min) 
{
	std::vector<double> v;
	double v1[12] = {12,-7, 5, 0.7, -8, -100, -0.1, 8, -100, 5, 0 ,0 };
	for(int i=0;i<12;i++)
	{
		v.push_back( v1[i] );
	}

	ASSERT_DOUBLE_EQ( -100.0, ICP::min(v));

	std::vector<double> v2;
	v2.push_back(1.0);
	v2.push_back(2.0);
	v2.push_back(3.0);
	v2.push_back(4.0);
	if (ICP::min(v2) == 2.0) {
		FAIL() << "The implementation fails if the first element is the minimum.";
	}

	std::vector<double> v3;
	v3.push_back(4.0);
	v3.push_back(3.0);
	v3.push_back(2.0);
	v3.push_back(1.0);
	if (ICP::min(v3) == 2.0) {
		FAIL() << "The implementation fails if the last element is the minimum.";
	}
}

TEST(ICP, euclideanCorrespondences) 
{
	StdVectorOfVector2d Q;
	StdVectorOfVector2d P;
	Eigen::MatrixXd q(7,2);
	q<<-1,2, 0,3, 1,4, 2,5, 3,4, 4,3, 5,2;
	Eigen::MatrixXd p(7,2);
	p<<-0.5,3, 0.4,4.1, 1.3,5.2, 2.2,6.3, 3.1,5.4, 4.0,4.5, 4.9,3.6;
	//p<<0.5,3, 0.4,4.1, 1.3,5.2, 2.2,6.3, 3.1,5.4, 4.0,4.5, 4.9,3.6;
	//http://eigen.tuxfamily.org/dox/group__TutorialBlockOperations.html
	for(int i=0;i<q.rows();i++)
		Q.push_back((q.block<1,2>(i,0)).transpose());
	for(int i=0;i<p.rows();i++)
		P.push_back((p.block<1,2>(i,0)).transpose());

	const StdVectorOfVector2d C = ICP::euclideanCorrespondences(Q,P);

	if (C.empty()) {
		FAIL() << "The method returns an empty vector.";
	}
	ASSERT_EQ(Q.size(), C.size());
	ASSERT_DOUBLE_EQ( -0.5, (double) C[0](0) );
	ASSERT_DOUBLE_EQ(   3, 	(double) C[0](1) );
	ASSERT_DOUBLE_EQ( -0.5, (double) C[1](0) );
	ASSERT_DOUBLE_EQ(   3, 	(double) C[1](1) );
	ASSERT_DOUBLE_EQ( 0.4,  (double) C[2](0) );
	ASSERT_DOUBLE_EQ( 4.1,  (double) C[2](1) );
	ASSERT_DOUBLE_EQ( 1.3,  (double) C[3](0) );
	ASSERT_DOUBLE_EQ( 5.2,  (double) C[3](1) );
	ASSERT_DOUBLE_EQ( 4, 	(double) C[4](0) );
	ASSERT_DOUBLE_EQ( 4.5,  (double) C[4](1) );
	ASSERT_DOUBLE_EQ( 4.9,  (double) C[5](0) );
	ASSERT_DOUBLE_EQ( 3.6,  (double) C[5](1) );
	ASSERT_DOUBLE_EQ( 4.9,  (double) C[6](0) );
	ASSERT_DOUBLE_EQ( 3.6,  (double) C[6](1) );
}

TEST(ICP, closestPointToLineCorrespondences) 
{
	StdVectorOfVector2d Q;
	StdVectorOfVector2d P;
	Eigen::MatrixXd q(7,2);
	q<<-1,2, 0,3, 1,4, 2,5, 3,4, 4,3, 5,2;
	Eigen::MatrixXd p(7,2);
	p<<-0.5,3, 0.4,4.1, 1.3,5.2, 2.2,6.3, 3.1,5.4, 4.0,4.5, 4.9,3.6;
	for(int i=0;i<q.rows();i++)
		Q.push_back((q.block<1,2>(i,0)).transpose());
	for(int i=0;i<p.rows();i++)
		P.push_back((p.block<1,2>(i,0)).transpose());

	const StdVectorOfVector2d C = ICP::closestPointToLineCorrespondences(Q,P);

	if (C.empty()) {
		FAIL() << "The method returns an empty vector.";
	}
    ASSERT_EQ(Q.size(), C.size());
	ASSERT_NEAR(-1.19, (double) C[0](0), 0.01);
	ASSERT_NEAR( 2.16, (double) C[0](1), 0.01);
	ASSERT_NEAR(-0.30, (double) C[1](0), 0.10);
	ASSERT_NEAR( 3.25, (double) C[1](1), 0.01);
	ASSERT_NEAR( 0.60, (double) C[2](0), 0.10);
	ASSERT_NEAR( 4.33, (double) C[2](1), 0.01);
	ASSERT_NEAR( 1.48, (double) C[3](0), 0.01);
	ASSERT_NEAR( 5.42, (double) C[3](1), 0.01);
	ASSERT_DOUBLE_EQ( 3.75, (double) C[4](0) );
	ASSERT_DOUBLE_EQ( 4.75, (double) C[4](1) );
	ASSERT_DOUBLE_EQ( 4.75, (double) C[5](0) );
	ASSERT_DOUBLE_EQ( 3.75, (double) C[5](1) );
	ASSERT_DOUBLE_EQ( 5.75, (double) C[6](0) );
	ASSERT_DOUBLE_EQ( 2.75, (double) C[6](1) );
}

TEST(ICP, calculateAffineTransformation) 
{
	StdVectorOfVector2d Q;
	StdVectorOfVector2d C;
	Eigen::MatrixXd q(7,2);
	q<<-1,2, 0,3, 1,4, 2,5, 3,4, 4,3, 5,2;
	for(int i=0;i<q.rows();i++)
		Q.push_back((q.block<1,2>(i,0)).transpose());
	Eigen::MatrixXd c(7,2);
	c << -1.19059, 2.15594,-0.299505,3.24505,0.591584,4.33416,1.48267,5.42327,3.75, 4.75,4.75, 3.75,5.75, 2.75;
	for(int i=0;i<c.rows();i++)
		C.push_back((c.block<1,2>(i,0)).transpose());
	const Eigen::Matrix3d A = ICP::calculateAffineTransformation(Q,C);
	

	ASSERT_NEAR( 0.993669, (double) A(0,0), 0.001);
	ASSERT_NEAR(0.112347, (double) A(0,1), 0.001);
	ASSERT_NEAR( -0.529593, (double) A(0,2), 0.001);
	ASSERT_NEAR(-0.112347, (double) A(1,0), 0.001);
	ASSERT_NEAR(  0.993669, (double) A(1,1), 0.001);
	ASSERT_NEAR(-0.224951, (double) A(1,2), 0.001);
	ASSERT_NEAR( 0, (double) A(2,0), 0.01);
	ASSERT_NEAR( 0, (double) A(2,1), 0.01);
	ASSERT_NEAR( 1, (double) A(2,2), 0.01);
}

TEST(ICP, applyTransformation) 
{
	StdVectorOfVector2d P;
	Eigen::MatrixXd p(7,2);
	p<<-0.5,3, 0.4,4.1, 1.3,5.2, 2.2,6.3, 3.1,5.4, 4.0,4.5, 4.9,3.6;
	for(int i=0;i<p.rows();i++)
		P.push_back((p.block<1,2>(i,0)).transpose());
	Eigen::Matrix3d A;
	A<< 0.997923, -0.0644194 ,  0.128267, 0.0644194 ,  0.997923 , -0.615596 , 0   , 0  , 1;
	
	const StdVectorOfVector2d V = ICP::applyTransformation(A,P);

	if (V.empty()) {
		FAIL() << "The method returns an empty vector.";
	}
	ASSERT_EQ(p.rows(), V.size());
	ASSERT_NEAR(-0.56, (double) V[0](0), 0.01);
	ASSERT_NEAR( 2.35, (double) V[0](1), 0.01);
	ASSERT_NEAR( 0.26, (double) V[1](0), 0.01);
	ASSERT_NEAR( 3.50, (double) V[1](1), 0.01);
	ASSERT_NEAR( 1.09, (double) V[2](0), 0.01);
	ASSERT_NEAR( 4.66, (double) V[2](1), 0.01);
	ASSERT_NEAR( 1.92, (double) V[3](0), 0.01);
	ASSERT_NEAR( 5.81, (double) V[3](1), 0.01);
	ASSERT_NEAR( 2.87, (double) V[4](0), 0.01);
	ASSERT_NEAR( 4.97, (double) V[4](1), 0.01);
	ASSERT_NEAR( 3.83, (double) V[5](0), 0.01);
	ASSERT_NEAR( 4.13, (double) V[5](1), 0.01);
	ASSERT_NEAR( 4.79, (double) V[6](0), 0.01);
	ASSERT_NEAR( 3.29, (double) V[6](1), 0.01);
}

TEST(ICP, computeError) 
{
	StdVectorOfVector2d Q;
	StdVectorOfVector2d C;
	Eigen::MatrixXd q(7,2);
	q<<-1,2, 0,3, 1,4, 2,5, 3,4, 4,3, 5,2;
	for(int i=0;i<q.rows();i++)
		Q.push_back((q.block<1,2>(i,0)).transpose());
	Eigen::MatrixXd c(7,2);
	c << -1.19059, 2.15594,-0.299505,3.24505,0.591584,4.33416,1.48267,5.42327,3.75, 4.75,4.75, 3.75,5.75, 2.75;
	for(int i=0;i<c.rows();i++)
		C.push_back((c.block<1,2>(i,0)).transpose());

	Eigen::Matrix3d A;
	A<< 0.997923, -0.0644194 ,  0.128267, 0.0644194 ,  0.997923 , -0.615596 , 0   , 0  , 1;
	
	const double error = ICP::computeError(Q,C,A);

	ASSERT_NEAR(3.33, error, 0.01);
}

TEST(ICP, iterateOnce)
{
	StdVectorOfVector2d Q;
	Eigen::MatrixXd q(7,2);
	q<<-1,2, 0,3, 1,4, 2,5, 3,4, 4,3, 5,2;
	for(int i=0;i<q.rows();i++)
		Q.push_back((q.block<1,2>(i,0)).transpose());
	
	StdVectorOfVector2d P;
	Eigen::MatrixXd p(7,2);
	p<<-0.5,3, 0.4,4.1, 1.3,5.2, 2.2,6.3, 3.1,5.4, 4.0,4.5, 4.9,3.6;
	for(int i=0;i<p.rows();i++)
		P.push_back((p.block<1,2>(i,0)).transpose());
	int flag =0;

	StdVectorOfVector2d P1_0 = ICP::iterateOnce(Q,P,flag,0,6);
	
	if (P1_0.empty()) {
		FAIL() << "The method returns an empty vector.";
	}
	ASSERT_EQ(P.size(), P1_0.size());
	ASSERT_NEAR(-0.665473, (double) P1_0[0](0), 0.01);
	ASSERT_NEAR( 2.79358, (double) P1_0[0](1), 0.01);
	ASSERT_NEAR( 0.378424, (double) P1_0[1](0), 0.01);
	ASSERT_NEAR( 3.75809, (double) P1_0[1](1), 0.01);
	ASSERT_NEAR( 1.42232, (double) P1_0[2](0), 0.01);
	ASSERT_NEAR( 4.7226, (double) P1_0[2](1), 0.01);
	ASSERT_NEAR( 2.46622, (double) P1_0[3](0), 0.01);
	ASSERT_NEAR( 5.68711, (double) P1_0[3](1), 0.01);
	ASSERT_NEAR( 3.23266, (double) P1_0[4](0), 0.01);
	ASSERT_NEAR( 4.67096, (double) P1_0[4](1), 0.01);
	ASSERT_NEAR( 3.9991, (double) P1_0[5](0), 0.01);
	ASSERT_NEAR( 3.65481, (double) P1_0[5](1), 0.01);
	ASSERT_NEAR( 4.76555, (double) P1_0[6](0), 0.01);
	ASSERT_NEAR( 2.63866, (double) P1_0[6](1), 0.01);
	ASSERT_EQ(1, flag);

	flag = 0;

	StdVectorOfVector2d P1_1 = ICP::iterateOnce(Q,P,flag,1,6);

	ASSERT_EQ(P.size(), P1_1.size());
	ASSERT_NEAR(-0.665473, (double) P1_0[0](0), 0.01);
	ASSERT_NEAR( 2.79358, (double) P1_0[0](1), 0.01);
	ASSERT_NEAR( 0.378424, (double) P1_0[1](0), 0.01);
	ASSERT_NEAR( 3.75809, (double) P1_0[1](1), 0.01);
	ASSERT_NEAR( 1.42232, (double) P1_0[2](0), 0.01);
	ASSERT_NEAR( 4.7226, (double) P1_0[2](1), 0.01);
	ASSERT_NEAR( 2.46622, (double) P1_0[3](0), 0.01);
	ASSERT_NEAR( 5.68711, (double) P1_0[3](1), 0.01);
	ASSERT_NEAR( 3.23266, (double) P1_0[4](0), 0.01);
	ASSERT_NEAR( 4.67096, (double) P1_0[4](1), 0.01);
	ASSERT_NEAR( 3.9991, (double) P1_0[5](0), 0.01);
	ASSERT_NEAR( 3.65481, (double) P1_0[5](1), 0.01);
	ASSERT_NEAR( 4.76555, (double) P1_0[6](0), 0.01);
	ASSERT_NEAR( 2.63866, (double) P1_0[6](1), 0.01);
	ASSERT_EQ(1, flag);
}

int main(int argc, char *argv[]) {
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}