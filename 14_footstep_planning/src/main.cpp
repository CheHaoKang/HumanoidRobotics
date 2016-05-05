#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <footstep_planning/FileIO.h>
#include <footstep_planning/FootstepPlanning.h>

using namespace footstep_planning;

int main(int argc, char *argv[]) 
{
	ros::init(argc, argv, "footstep_planning");

	const std::string packagePath = ros::package::getPath("footstep_planning");
	if (packagePath.empty()) {
		std::cerr << "Error: Could not find package footstep_planning. Make sure that you call" << std::endl;
		std::cerr << "    source devel/setup.bash" << std::endl;
		std::cerr << "from your catkin workspace first before running this program." << std::endl;
		return 1;
	}

	footstep_planning::FileIO fileIO(packagePath);
	path_planning::FileIO pfileIO;
	pfileIO.openLogfile(packagePath + "/data/log.txt");
	OpenList::fileIO = &pfileIO;
	ClosedList::fileIO = &pfileIO;
	FootstepPlanning planning(fileIO.map);
	FootstepNode *start = FootstepNode::get(1.0, 0.2, 0.0, LEFT);
	FootstepNode *goal = FootstepNode::get(2.0, 2.0, -0.5 * M_PI, RIGHT);
	std::deque<const AbstractNode *> path = planning.planPath(start, goal);

	pfileIO.logPath(packagePath + "/data/path.txt", path);

	std::cout << std::endl << "Final path: ";
	if (path.empty()) {
		std::cout << "empty" << std::endl;
	} else {
		std::cout << std::endl;
		for (std::deque<const AbstractNode *>::const_iterator it = path.begin(); it != path.end(); ++it) {
			std::cout << (*it)->toString() << std::endl;
		}
	}

	std::cout << "Done. " << std::endl;
	return 0;
}
