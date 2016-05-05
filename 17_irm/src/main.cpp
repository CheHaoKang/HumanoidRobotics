#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <irm/IRM.h>
#include <irm/FileIO.h>

using namespace irm;

int main(int argc, char *argv[]) 
{
	ros::init(argc, argv, "irm");
	srand(time(NULL));

	const std::string packagePath = ros::package::getPath("irm");
	if (packagePath.empty()) {
		std::cerr << "Error: Could not find package irm. Make sure that you call" << std::endl;
		std::cerr << "    source devel/setup.bash" << std::endl;
		std::cerr << "from your catkin workspace first before running this program." << std::endl;
		return 1;
	}

	FileIO fileIO(packagePath);

	IRM irm;
	irm.computeRM(50000);
	fileIO.writeRM(irm);

	irm.doComputeIRM();
	fileIO.writeIRM(irm);

	return 0;
}
