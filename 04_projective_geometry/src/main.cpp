#include <projective_geometry/ProjectiveGeometry.h>
#include <iostream>
#include <cmath>
using namespace projective_geometry;
int main(int argc, char *argv[]) 
{
	Eigen::Vector3d x1(1,0,2);
	Eigen::Vector3d x2(2,3,0);
	
	cameraParameters param = ProjectiveGeometry::setCameraParameters(4.0 * ProjectiveGeometry::PI / 180.0);

	Eigen::Matrix3d calibrationMatrix = ProjectiveGeometry::calibrationMatrix(param);						
	std::cout << "The calibrationMatrix is: "<< std::endl  << calibrationMatrix << std::endl << std::endl;

	Eigen::MatrixXd projectionMatrix = ProjectiveGeometry::projectionMatrix(calibrationMatrix,param);
	
	
	std::cout << "The projectionMatrix is: "<< std::endl  << projectionMatrix << std::endl << std::endl;
	
	std::cout << "The projection of x1 is: "<< std::endl  << ProjectiveGeometry::projectPoint(x1,projectionMatrix) << std::endl << std::endl;
	std::cout << "The projection of x2 is: "<< std::endl  << ProjectiveGeometry::projectPoint(x2,projectionMatrix) << std::endl << std::endl;
	

	return 0;
}
