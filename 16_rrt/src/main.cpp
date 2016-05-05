#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <rrt/RRT.h>
#include <rrt/FileIO.h>

using namespace rrt;

class RRTWithLoggers : public RRT {
public:
	RRTWithLoggers(FileIO * const fileIO) : fileIO(fileIO), RRT(), listNameForward("forward"), listNameBackward("backward"), numExtendedNodes(0) {}
	virtual ~RRTWithLoggers() {}
	const size_t& getNumExtendedNodes() const { return numExtendedNodes; }

protected:
	FileIO * const fileIO;
	const std::string listNameForward;
	const std::string listNameBackward;
	mutable size_t numExtendedNodes;

	virtual AbstractNode * extendClosestNode(GridNode * const randomNode, GridNode * const closestNode,
			std::vector<AbstractNode *> & list, const GridMap& map,
			const std::vector<AbstractNode *> & otherList) const {
		++numExtendedNodes;
		const std::string * listName;
		if (list[0] == GridNode::get(7, 7)) {
			listName = &listNameForward;
		} else if (list[0] == GridNode::get(11, 33)) {
			listName = &listNameBackward;
		}
		std::vector<AbstractNode *> listBefore(list.begin(), list.end());
		AbstractNode * connectionNode = RRT::extendClosestNode(randomNode, closestNode, list, map, otherList);
		for (std::vector<AbstractNode *>::const_reverse_iterator it = list.rbegin(); it != list.rend(); ++it) {
			if (std::find(listBefore.begin(), listBefore.end(), *it) == listBefore.end()) {
				fileIO->logExtend((GridNode *) closestNode, (GridNode *) (*it), (GridNode *) randomNode, listName);
				return connectionNode;
			}
		}
		fileIO->logFailedExtend((GridNode *) closestNode, (GridNode *) randomNode, listName);
		return connectionNode;
	}
};

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "rrt");
	srand(time(NULL));

	const std::string packagePath = ros::package::getPath("rrt");
	if (packagePath.empty()) {
		std::cerr << "Error: Could not find package rrt. Make sure that you call" << std::endl;
		std::cerr << "    source devel/setup.bash" << std::endl;
		std::cerr << "from your catkin workspace first before running this program." << std::endl;
		return 1;
	}

	FileIO fileIO;
	fileIO.openLogfile(packagePath + "/data/log.txt");

	GridMap *map = fileIO.loadMap(packagePath + "/data/map.pbm");
	if (!map) {
		return 1;
	}

	GridNode *startNode = GridNode::get(7, 7);
	GridNode *goalNode = GridNode::get(11, 33);

	RRTWithLoggers rrtTree(&fileIO);

	std::deque<AbstractNode *> path = rrtTree.planPath(startNode, goalNode, *map, 10000);

	fileIO.logPath(packagePath + "/data/path.txt", path);

	std::cout << std::endl << "Extended " << rrtTree.getNumExtendedNodes() << " nodes. Final path: ";
	if (path.empty()) {
		std::cout << "empty" << std::endl;
	} else {
		std::cout << "length " << path.size() << std::endl;
	}

	delete map;
	return 0;

}
