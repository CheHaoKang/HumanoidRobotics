#include <signed_distance_function/SignedDistanceFunction.h>
#include <signed_distance_function/FileIO.h>
#include <ros/ros.h>
#include <ros/package.h>

using namespace signed_distance_function;

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "signed_distance_function");

	const std::string packagePath = ros::package::getPath("signed_distance_function");
	if (packagePath.empty()) {
		std::cerr << "Error: Could not find package signed_distance_function. Make sure that you call" << std::endl;
		std::cerr << "    source devel/setup.bash" << std::endl;
		std::cerr << "from your catkin workspace first before running this program." << std::endl;
		return 1;
	}

	FileIO fileIO(packagePath + "/data/data.txt");
	SignedDistanceFunction sdf;
	Eigen::MatrixXd map = Eigen::MatrixXd::Zero(fileIO.sizeX, fileIO.sizeY);
	Eigen::MatrixXd weights = Eigen::MatrixXd::Zero(fileIO.sizeX, fileIO.sizeY);

	for(FileIO::MeasurementsVector::const_iterator it = fileIO.measurements.begin(); it != fileIO.measurements.end(); ++it) {
		sdf.integrateLaserScan(map, weights, *it);
	}
	fileIO.writeMap(map, packagePath + "/data/result.txt");

	return 0;
}
