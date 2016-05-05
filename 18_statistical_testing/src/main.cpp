#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <statistical_testing/StatisticalTesting.h>

using namespace statistical_testing;

int main(int argc, char *argv[]) 
{
	ros::init(argc, argv, "statistical_testing");

	const std::string packagePath = ros::package::getPath("statistical_testing");
	if (packagePath.empty()) {
		std::cerr << "Error: Could not find package statistical_testing. Make sure that you call" << std::endl;
		std::cerr << "    source devel/setup.bash" << std::endl;
		std::cerr << "from your catkin workspace first before running this program." << std::endl;
		return 1;
	}

	StatisticalTesting st;
	std::cout << "Student test: " << std::endl;
	st.germanStudentsTest();
	std::cout << std::endl;
	std::cout << "Cars in the neighborhood: " << std::endl;
	st.cars();
	std::cout << std::endl;
	std::cout << "Planner implementation: " << std::endl;
	st.planning();
	std::cout << std::endl;

	return 0;
}
