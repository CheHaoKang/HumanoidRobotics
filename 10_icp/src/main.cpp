#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <icp/ICP.h>
#include <Eigen/Dense>
#include <math.h>
#include <vector>
#include <icp/FileIO.h>

using namespace icp;

int main(int argc, char *argv[]) 
{
	ros::init(argc, argv, "icp");

	const std::string packagePath = ros::package::getPath("icp");
	if (packagePath.empty()) {
		std::cerr << "Error: Could not find package icp. Make sure that you call" << std::endl;
		std::cerr << "    source devel/setup.bash" << std::endl;
		std::cerr << "from your catkin workspace first before running this program." << std::endl;
		return 1;
	}

	FileIO fileIO(packagePath + "/data/data.txt");

	double threshold = 0.5;
	std::cout<<"Apply one iteration of ICP using closet point corresponding points: \n";
	StdVectorOfVector2d P_1 = fileIO.P;
	int convergenceFlag = 0;
	for(int i=0;i<10000;i++)
	{
		P_1 = ICP::iterateOnce(fileIO.Q,P_1,convergenceFlag,0,threshold);
		if (convergenceFlag==1)
		{
			std::cout<<" \n **********###########################********* \n";
			break;
		}
	}
	std::cout << "List of points obtained: "<< std::endl;
	for(int i=0;i<P_1.size();i++)
		std::cout << P_1[i].transpose() << std::endl;

	fileIO.writeMap(P_1, packagePath + "/data/closestPointResult.txt");

	std::cout<<"***************************************** \n";

	std::cout<<"Apply one iteration of ICP using point-to-line corresponding points: \n";
	StdVectorOfVector2d P_2 = fileIO.P;
	convergenceFlag = 0;
	for(int i=0;i<10000;i++)
	{
		P_2 = ICP::iterateOnce(fileIO.Q,P_2,convergenceFlag,1,threshold);
		if (convergenceFlag==1)
		{
			std::cout<<" \n **********###########################********* \n";
			break;
		}
	}
	std::cout << "List of points obtained: "<< std::endl;
	for(int i=0;i<P_2.size();i++)
		std::cout << P_2[i].transpose() << std::endl;

	fileIO.writeMap(P_2, packagePath + "/data/pointToLineResult.txt");

	return 0;
}