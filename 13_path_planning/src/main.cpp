#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <path_planning/PathPlanning.h>
#include <path_planning/FileIO.h>

using namespace path_planning;

int main(int argc, char *argv[]) 
{
	ros::init(argc, argv, "path_planning");

	const std::string packagePath = ros::package::getPath("path_planning");
	if (packagePath.empty()) {
		std::cerr << "Error: Could not find package path_planning. Make sure that you call" << std::endl;
		std::cerr << "    source devel/setup.bash" << std::endl;
		std::cerr << "from your catkin workspace first before running this program." << std::endl;
		return 1;
	}

	FileIO fileIO;
	fileIO.openLogfile(packagePath + "/data/log.txt");
	OpenList::fileIO = &fileIO;
	ClosedList::fileIO = &fileIO;

	GridMap *map = fileIO.loadMap(packagePath + "/data/map.pbm");
	if (!map) {
		return 1;
	}

	StraightLineDistanceHeuristic heuristic;
	GridPathPlanning planning(*map, &heuristic);
	const GridNode *startNode = GridNode::get(4, 9);
	const GridNode *goalNode = GridNode::get(2, 7);
	std::deque<const AbstractNode *> path = planning.planPath(startNode, goalNode);

	fileIO.logPath(packagePath + "/data/path.txt", path);

	std::cout << std::endl << "Final path: ";
	if (path.empty()) {
		std::cout << "empty" << std::endl;
	} else {
		std::cout << std::endl;
		for (std::deque<const AbstractNode *>::const_iterator it = path.begin(); it != path.end(); ++it) {
			std::cout << (*it)->toString() << std::endl;
		}
	}

	delete map;
	return 0;
}
