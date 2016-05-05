#include <gtest/gtest.h>
#include <iostream>
#include <rrt/RRT.h>
#include <rrt/FileIO.h>
#include <math.h>

using namespace rrt;

class RRTTest : public ::testing::Test {
protected:
	GridMap *map;
	GridNode *goal;
	GridNode *start;
	RRT rrtTree;

	virtual void SetUp() {
		std::vector<bool> data(100, false);
		data[0 * 10 + 0] = true;
		data[0 * 10 + 2] = true;
		data[0 * 10 + 3] = true;
		data[1 * 10 + 2] = true;
		data[1 * 10 + 3] = true;
		data[2 * 10 + 2] = true;
		data[2 * 10 + 3] = true;
		data[4 * 10 + 1] = true;
		data[4 * 10 + 2] = true;
		data[4 * 10 + 3] = true;
		data[4 * 10 + 6] = true;
		data[4 * 10 + 7] = true;
		data[5 * 10 + 3] = true;
		data[5 * 10 + 6] = true;
		data[5 * 10 + 7] = true;
		data[5 * 10 + 8] = true;
		data[6 * 10 + 3] = true;
		data[6 * 10 + 4] = true;
		data[6 * 10 + 9] = true;
		data[7 * 10 + 2] = true;
		data[7 * 10 + 3] = true;
		data[7 * 10 + 5] = true;
		data[8 * 10 + 0] = true;
		data[8 * 10 + 7] = true;
		data[8 * 10 + 9] = true;
		data[9 * 10 + 1] = true;
		data[9 * 10 + 8] = true;
		data[9 * 10 + 9] = true;

		map = new GridMap(10, 10, data);

		goal = GridNode::get(0, 4);
		start = GridNode::get(7, 9);
	}
	virtual void TearDown() {
		delete map;
		map = NULL;
	}

	void testNeighbor(GridNode * const currentNode, const std::vector<AbstractNode *>& neighbors,
			const std::vector<std::pair<int, int> >& result, const std::vector<AbstractNode *>& list) const;
};

TEST_F(RRTTest, getRandomNode) {
	std::vector<AbstractNode *> list;
	//col,row
	list.push_back(GridNode::get(5, 9));
	list.push_back(GridNode::get(5, 8));
	list.push_back(GridNode::get(4, 7));
	list.push_back(GridNode::get(7, 7));
	list.push_back(GridNode::get(6, 6));
	list.push_back(GridNode::get(7, 6));
	list.push_back(GridNode::get(4, 5));
	list.push_back(GridNode::get(5, 5));
	list.push_back(GridNode::get(6, 9));
	list.push_back(GridNode::get(6, 8));
	list.push_back(GridNode::get(6, 7));
	list.push_back(GridNode::get(5, 6));
	list.push_back(GridNode::get(7, 6));

	AbstractNode * randomNode;
	int counter = 0;
	const int iterations = 15000;
	for (int i = 0; i < iterations; i++) {
		randomNode = rrtTree.getRandomNode(*map, list, goal);
		if (!randomNode) {
			FAIL() << "The method returns NULL.";
		}

		if (((GridNode*) randomNode)->x >= map->width || ((GridNode*) randomNode)->y >= map->height)
			FAIL()<<"The method returned a cell that is outside the map boundaries.";
		if (map->isOccupied(((GridNode*) randomNode)->x, ((GridNode*) randomNode)->y))
			FAIL()<<"The method returned a cell that is occupied.";
		if (std::find(list.begin(), list.end(), GridNode::get(((GridNode*) randomNode)->x, ((GridNode*) randomNode)->y)) != list.end())
			FAIL()<<"The method returned a node that has already been explored.";

		if (randomNode == goal)
			counter++;
	}
	double fraction = static_cast<double>(counter) / static_cast<double>(iterations);

	if (fraction > 0.15)
		FAIL()<<"The RRT-connect seems to be overbiased (>15% of the nodes are the goal node)";
	else if (fraction < 0.07)
		FAIL()<<"The RRT-connect seems to be underbiased (<7% of the nodes are the goal node)";
}

TEST_F(RRTTest, distance) {
	if (rrtTree.distance(GridNode::get(0, 4), GridNode::get(5, 3)) == 0.0) {
		FAIL() << "The method does not do anything.";
	}
	ASSERT_NEAR(5.099019514, rrtTree.distance(GridNode::get(0, 4), GridNode::get(5, 3)), 1e-4);
	ASSERT_NEAR(2.236067977, rrtTree.distance(GridNode::get(7, 4), GridNode::get(5, 3)), 1e-4);
}

TEST_F(RRTTest,getClosestNodeInList ) {
	std::vector<AbstractNode*> list;
	//col,row
	list.push_back(GridNode::get(5, 9));
	list.push_back(GridNode::get(5, 8));
	list.push_back(GridNode::get(4, 7));
	list.push_back(GridNode::get(7, 7));
	list.push_back(GridNode::get(6, 6));
	list.push_back(GridNode::get(7, 6));
	list.push_back(GridNode::get(4, 5));
	list.push_back(GridNode::get(5, 5));
	list.push_back(GridNode::get(6, 9));
	list.push_back(GridNode::get(6, 8));
	list.push_back(GridNode::get(6, 7));
	list.push_back(GridNode::get(5, 6));
	list.push_back(GridNode::get(7, 6));

	std::vector<AbstractNode*> emptyList;

	if (!rrtTree.getClosestNodeInList(GridNode::get(2, 5), list)) {
		FAIL() << "The method does not return the nearest node.";
	}
	if (rrtTree.getClosestNodeInList(GridNode::get(2, 5), list) != GridNode::get(4, 5))
		FAIL()<<"The returned node is not the nearest node.";
	if (rrtTree.getClosestNodeInList(GridNode::get(1, 1), list) != GridNode::get(4, 5))
		FAIL()<<"The returned node is not the nearest node.";
	if (rrtTree.getClosestNodeInList(GridNode::get(9, 4), list) != GridNode::get(7, 6))
		FAIL()<<"The returned node is not the nearest node.";
	if (rrtTree.getClosestNodeInList(GridNode::get(9, 4), emptyList) != NULL) {
		FAIL() << "The method returns a node although the list is empty. It should return NULL instead. Make sure your algorithm does not try to access list elements beyond the end of the list.";
	}
}

void RRTTest::testNeighbor(GridNode * const currentNode, const std::vector<AbstractNode *>& neighbors,
		const std::vector<std::pair<int, int> >& result, const std::vector<AbstractNode *>& list) const {
	if (neighbors.empty()) {
		FAIL() << "The method returned an empty vector.";
	}
	std::vector<bool> resultFound(result.size(), false);
	for (std::vector<AbstractNode *>::const_iterator it = neighbors.begin(); it != neighbors.end(); ++it) {
		GridNode *node = static_cast<GridNode *>(*it);
		if (node->x < 0 || node->x >= map->width || node->y < 0 || node->y >= map->height) {
			FAIL() << "The method returned a neighbor node that is outside the map bounds.";
		}
		if (node->x == currentNode->x && node->y == currentNode->y) {
			FAIL() << "The method returned the current node as its own neighbor.";
		}
		if (map->isOccupied(node->x, node->y)) {
			FAIL() << "The method returned a neighbor cell that is occupied.";
		}
		if (std::find(list.begin(), list.end(), *it) != list.end()) {
			FAIL() << "The method returns a neighbor cell that is already on the list of expanded nodes and should not be returned anymore";
		}
		bool found = false;
		for (size_t i = 0; i < result.size(); ++i) {
			if (result[i].first == node->x && result[i].second == node->y) {
				if (resultFound[i]) {
					FAIL() << "The method returns a neighbor twice.";
				} else {
					found = true;
					resultFound[i] = true;
					break;
				}
			}
		}
		if (!found) {
			FAIL() << "The method returned a node that is not a neighbor node of the current cell.";
		}
	}
	for (size_t i = 0; i < result.size(); ++i) {
		if (!resultFound[i]) {
			FAIL() << "The result vector does not contain an expected neighbor node.";
		}
	}
}

TEST_F(RRTTest, getNeighbors) {
	std::vector<AbstractNode *> list;
	//col,row
	list.push_back(GridNode::get(5, 9));
	list.push_back(GridNode::get(5, 8));
	list.push_back(GridNode::get(4, 7));
	list.push_back(GridNode::get(7, 7));
	list.push_back(GridNode::get(6, 6));
	list.push_back(GridNode::get(7, 6));
	list.push_back(GridNode::get(4, 5));
	list.push_back(GridNode::get(5, 5));
	list.push_back(GridNode::get(6, 9));
	list.push_back(GridNode::get(6, 8));
	list.push_back(GridNode::get(6, 7));
	list.push_back(GridNode::get(5, 6));

	GridNode * const currentNode1 = GridNode::get(1, 1);
	std::vector<AbstractNode *> neighbors1 = rrtTree.getNeighbors(currentNode1, list, *map);
	std::vector<std::pair<int, int> > result1;
	result1.push_back(std::make_pair<int, int>(0, 1));
	result1.push_back(std::make_pair<int, int>(0, 2));
	result1.push_back(std::make_pair<int, int>(1, 0));
	result1.push_back(std::make_pair<int, int>(1, 2));
	testNeighbor(currentNode1, neighbors1, result1, list);

	GridNode * const currentNode2 = GridNode::get(0, 9);
	std::vector<AbstractNode *> neighbors2 = rrtTree.getNeighbors(currentNode2, list, *map);
	std::vector<std::pair<int, int> > result2;
	result2.push_back(std::make_pair<int, int>(1, 8));
	testNeighbor(currentNode2, neighbors2, result2, list);

	GridNode * const currentNode3 = GridNode::get(5, 4);
	std::vector<AbstractNode *> neighbors3 = rrtTree.getNeighbors(currentNode3, list, *map);
	std::vector<std::pair<int, int> > result3;
	result3.push_back(std::make_pair<int, int>(4, 3));
	result3.push_back(std::make_pair<int, int>(4, 4));
	result3.push_back(std::make_pair<int, int>(5, 3));
	result3.push_back(std::make_pair<int, int>(6, 3));
	testNeighbor(currentNode3, neighbors3, result3, list);
}

TEST_F(RRTTest, tryToConnect) {
	GridNode * const currentNode1 = GridNode::get(4, 4);
	GridNode * const connectionNode1 = GridNode::get(5, 4);
	std::vector<AbstractNode *> neighbors1;
	neighbors1.push_back(GridNode::get(4, 3));
	neighbors1.push_back(connectionNode1);
	std::vector<AbstractNode *> otherList1;
	otherList1.push_back(GridNode::get(5, 5));
	otherList1.push_back(connectionNode1);
	AbstractNode * connectionNode = rrtTree.tryToConnect(currentNode1, neighbors1, otherList1);
	if (!connectionNode) {
		FAIL() << "The method does not return a connection node although the trees can be connected.";
	} else {
		GridNode * cn = static_cast<GridNode *>(connectionNode);
		if (cn->x == currentNode1->x && cn->y == currentNode1->y) {
			FAIL() << "The method returns the current node as the connection node. It should return the neighbor node that establishes the connection instead.";
		}
		if (cn->x != connectionNode1->x && cn->y != connectionNode1->y) {
			FAIL() << "The method returns a wrong node as the connection node.";
		}
		if (connectionNode->getConnection() == NULL) {
			FAIL() << "The method does not connect the trees by calling neighbor->setConnection(currentNode)";
		} else {
			if (connectionNode->getConnection() != currentNode1) {
				FAIL() << "The method calls neighbor->setConnection(node) with a wrong node as the argument. It should be the current node.";
			}
		}
	}

	GridNode * const currentNode2 = GridNode::get(4, 4);
	std::vector<AbstractNode *> neighbors2;
	neighbors2.push_back(GridNode::get(4, 3));
	neighbors2.push_back(GridNode::get(5, 4));
	std::vector<AbstractNode *> otherList2;
	otherList2.push_back(GridNode::get(5, 9));
	otherList2.push_back(GridNode::get(6, 9));
	connectionNode = rrtTree.tryToConnect(currentNode2, neighbors2, otherList2);
	if (connectionNode) {
		FAIL() << "The method returns a connection node although the trees cannot be connected.";
	}
}

TEST_F(RRTTest, addNearestNeighbor) {
	GridNode * const currentNode = GridNode::get(4, 4);
	GridNode * const randomNode = GridNode::get(7, 1);
	std::vector<AbstractNode*> neighbors;
	neighbors.push_back(GridNode::get(4, 5));
	neighbors.push_back(GridNode::get(5, 5));
	neighbors.push_back(GridNode::get(5, 3));
	neighbors.push_back(GridNode::get(5, 4));
	neighbors.push_back(GridNode::get(4, 3));
	std::vector<AbstractNode*> list;
	list.push_back(GridNode::get(3, 3));
	rrtTree.addNearestNeighbor(currentNode, neighbors, randomNode, list);
	if (std::find(list.begin(), list.end(), GridNode::get(3, 3)) == list.end()) {
		FAIL() << "The method deletes a node from the list of expanded nodes.";
	}
	if (list.size() > 2) {
		FAIL() << "The method adds more than one neighbor to the list of expanded nodes. It should only add the one that is closest to the random node.";
	}
	if (std::find(list.begin(), list.end(), GridNode::get(5, 3)) == list.end()) {
		FAIL() << "The method adds a cell that is not the closest one to the random node.";
	}
	if (!GridNode::get(5, 3)->getPredecessor()) {
		FAIL() << "The method does not set the predecessor of the new node.";
	}
	if (GridNode::get(5, 3)->getPredecessor() != currentNode) {
		FAIL() << "The method sets the predecessor of the new node to the wrong node. The predecessor should be the current node.";
	}
}

TEST_F(RRTTest,extendClosestNode ) {
	std::vector<AbstractNode *> list;
	//col,row
	list.push_back(GridNode::get(5, 9));
	list.push_back(GridNode::get(5, 8));
	list.push_back(GridNode::get(4, 7));
	list.push_back(GridNode::get(7, 7));
	list.push_back(GridNode::get(6, 6));
	list.push_back(GridNode::get(7, 6));
	list.push_back(GridNode::get(4, 5));
	list.push_back(GridNode::get(5, 5));
	list.push_back(GridNode::get(6, 9));
	list.push_back(GridNode::get(6, 8));
	list.push_back(GridNode::get(6, 7));
	list.push_back(GridNode::get(5, 6));


	std::vector<AbstractNode *> otherList;

	AbstractNode * connectionNode;
	connectionNode = rrtTree.extendClosestNode(GridNode::get(1, 1), rrtTree.getClosestNodeInList(GridNode::get(1, 1), list), list, *map, otherList);
	if (std::find(list.begin(), list.end(), GridNode::get(4, 4)) == list.end())
		FAIL()<<"Could not expand the nearest node to the random one correctly";
	if (connectionNode)
		FAIL()<<"The method returned a connection node although the trees are not connected.";

	otherList.push_back(GridNode::get(3, 8));

	connectionNode = rrtTree.extendClosestNode(GridNode::get(9, 4), rrtTree.getClosestNodeInList(GridNode::get(9, 4), list), list, *map, otherList);
	if (std::find(list.begin(), list.end(), GridNode::get(8, 6)) == list.end())
		FAIL()<<"Could not expand the nearest node to the random one correctly";
	if (connectionNode)
		FAIL()<<"The method returned a connection node although the trees are not connected.";

	connectionNode = rrtTree.extendClosestNode(GridNode::get(1, 7), rrtTree.getClosestNodeInList(GridNode::get(1, 7), list), list, *map, otherList);
	if (std::find(list.begin(), list.end(), GridNode::get(3, 8)) == list.end()
			&& std::find(otherList.begin(), otherList.end(), GridNode::get(3, 8)) == otherList.end())
		FAIL()<<"Could not expand the nearest node to the random one correctly";
	if (!connectionNode)
		FAIL()<<"The method did not return a connection node although the trees can be connected.";

	}
TEST_F(RRTTest,constructPath ) {
	GridNode::get(4, 4)->setPredecessor(GridNode::get(3, 3));
	GridNode::get(3, 3)->setPredecessor(GridNode::get(2, 3));
	GridNode::get(2, 3)->setPredecessor(GridNode::get(1, 3));
	GridNode::get(1, 3)->setPredecessor(goal);

	GridNode::get(4, 5)->setPredecessor(GridNode::get(5, 6));
	GridNode::get(5, 6)->setPredecessor(GridNode::get(6, 7));
	GridNode::get(6, 7)->setPredecessor(GridNode::get(6, 8));
	GridNode::get(6, 8)->setPredecessor(start);

	GridNode::get(4, 5)->setConnection(GridNode::get(4, 4));

	std::deque<AbstractNode *> path = rrtTree.constructPath(GridNode::get(4, 5), start, goal);
	if (path.empty()) {
		FAIL() << "The method returns an empty path.";
	}

	std::deque<AbstractNode *> correctPath;
	correctPath.push_back(start);
	correctPath.push_back(GridNode::get(6, 8));
	correctPath.push_back(GridNode::get(6, 7));
	correctPath.push_back(GridNode::get(5, 6));
	correctPath.push_back(GridNode::get(4, 5));
	correctPath.push_back(GridNode::get(4, 4));
	correctPath.push_back(GridNode::get(3, 3));
	correctPath.push_back(GridNode::get(2, 3));
	correctPath.push_back(GridNode::get(1, 3));
	correctPath.push_back(goal);

	if (path.size() > correctPath.size())
		FAIL()<<"Path is longer than expected, there are some wrong nodes added to it";
	if (path.size() < correctPath.size())
		FAIL()<<"Path is shorter than expected, there are some nodes missing";

	if (path[0] != start)
		FAIL()<<"The returned path does not begin at the start node.";

	if (path[path.size() - 1] != goal)
		FAIL()<<"The returned path does not end at the goal node.";

	for (int i = 0; i < correctPath.size(); i++) {
		if (correctPath[i] != path[i])
			FAIL()<<"Path is wrong";
		}

		//reverse
	GridNode::get(4, 5)->setPredecessor(GridNode::get(4, 4));
	GridNode::get(4, 5)->setConnection(GridNode::get(5, 6));

	std::deque<AbstractNode *> reversedPath = rrtTree.constructPath(GridNode::get(4, 5), start, goal);

	if (reversedPath[0] != start)
		FAIL()<<"The returned path does not begin at the start node.";

	if (reversedPath[reversedPath.size() - 1] != goal)
		FAIL()<<"The returned path does not end at the goal node.";
	}

TEST_F(RRTTest,planPath) {
	std::deque<AbstractNode*> path = rrtTree.planPath(start, goal, *map, 10000);
	if (path.empty()) {
		FAIL() << "The method returns an empty path.";
	}

	if (path[0]!=start)
		FAIL()<<"The returned path does not begin at the start node.";
	if (path[path.size() - 1] != goal)
		FAIL()<<"The returned path does not end at the goal node.";

	
	for (int i = 1; i < path.size(); i++) {
		GridNode * const currNode = (GridNode *) path[i];
		GridNode * const prevNode = (GridNode *) path[i-1];
		if (currNode->x < 0 || currNode->x > map->width || currNode->y < 0 || currNode->y > map->height) {
			FAIL() << "The planned path leaves the map.";
		}
		if (fabs(currNode->x - prevNode->x) > 1 || fabs(currNode->y - prevNode->y) > 1) {
			FAIL()<<"The planned path jumps from " << path[i-1]->toString() << " to " << path[i]->toString() << ", those are not neighboring cells.";
		}
		if (map->isOccupied(currNode->x, currNode->y)) {
			FAIL() << "The planned path passes through an occupied node.";
		}
	}

	std::deque<AbstractNode*> timeoutPath = rrtTree.planPath(start, goal, *map, 1);
	if (!timeoutPath.empty()) {
		FAIL()<<"The planner runs more iterations than given in maxIterations. If there is no possible path, then your algorithm will run forever.";
	}
}
int main(int argc, char *argv[]) {
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
