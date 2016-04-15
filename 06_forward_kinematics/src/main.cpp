#include <forward_kinematics/ForwardKinematics.h>
#include <forward_kinematics/RosIO.h>
#include <forward_kinematics/FileIO.h>
#include <ros/package.h>


using namespace forward_kinematics;

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "forward_kinematics");
	const bool live = (argc >= 2) && (std::string(argv[1]).compare("--live") == 0);

	ForwardKinematics fk;
	if (live) {
		RosIO rosIO(&fk);
		ros::spin();
	} else {
		const std::string packagePath = ros::package::getPath("forward_kinematics");
		if (packagePath.empty()) {
			std::cerr << "Error: Could not find package forward_kinematics. Make sure that you call" << std::endl;
			std::cerr << "    source devel/setup.bash" << std::endl;
			std::cerr << "from your catkin workspace first before running this program." << std::endl;
			return 1;
		}

		FileIO fio(packagePath + "/data/joints.txt");
		fio.results.reserve(fio.jointAngles.size());
		for (std::vector<std::vector<double> >::const_iterator it = fio.jointAngles.begin(); it != fio.jointAngles.end(); ++it) {
			 fio.results.push_back(fk.computeHandTransform(&(*it)[0]));
		}
		const std::string outputFile = packagePath + "/data/result.txt";
		if (fio.writeToFile(outputFile)) {
			std::cout << "Wrote " << fio.results.size() << " points to " << outputFile << std::endl;
		}
	}
	return 0;
}
