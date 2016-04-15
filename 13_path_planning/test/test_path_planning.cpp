#include <gtest/gtest.h>
#include <path_planning/PathPlanning.h>
#include <math.h>

using namespace path_planning;

TEST(PathPlanning, getCosts) {
	std::vector<bool> data(100, false);
	GridMap map(10, 10, data);
	StraightLineDistanceHeuristic heuristic;
	GridPathPlanning planner(map, &heuristic);
	EXPECT_DOUBLE_EQ(5.0, planner.getCosts(GridNode::get(2, 3), GridNode::get(5, 7)));
}

TEST(PathPlanning, straightLineDistanceHeuristic) {
	std::vector<bool> data(100, false);
	GridMap map(10, 10, data);
	StraightLineDistanceHeuristic heuristic;
	GridPathPlanning planner(map, &heuristic);
	EXPECT_DOUBLE_EQ(5.0, planner.heuristic(GridNode::get(2, 3), GridNode::get(5, 7)));
}

TEST(PathPlanning, manhattanDistanceHeuristic) {
	std::vector<bool> data(100, false);
	GridMap map(10, 10, data);
	ManhattanDistanceHeuristic heuristic;
	GridPathPlanning planner(map, &heuristic);
	EXPECT_DOUBLE_EQ(7.0, planner.heuristic(GridNode::get(2, 3), GridNode::get(5, 7)));
}

TEST(PathPlanning, getNeighborNodes) {
	std::vector<bool> data(100, false);
	data[5 * 10 + 5] = true;
	GridMap map(10, 10, data);
	StraightLineDistanceHeuristic heuristic;
	GridPathPlanning planner(map, &heuristic);
	std::vector<AbstractNode *> v = planner.getNeighborNodes(GridNode::get(3, 7));
	if (v.empty()) {
		FAIL() << "The method returns an empty vector.";
	}
	for (size_t i = 0; i < v.size(); ++i) {
		const GridNode * node = static_cast<const GridNode *>(v[i]);
		if (node->x < 2 || node->x > 4 || node->y < 6 || node->y > 8) {
			FAIL() << "The method returns (" << node->x << ", " << node->y << ") as a neighbor of (3, 7), although it is not a neighboring cell.";
		}
		if (node->x == 3 && node->y == 7) {
			FAIL() << "The method includes the current node as a neighbor node. The current node must not be returned as a neighbor in order to prevent infinite loops.";
		}
	}
	for (size_t x = 2; x <= 4; ++x) {
		for (size_t y = 6; y <= 8; ++y) {
			if (x == 3 && y == 7) {
				continue;
			}
			bool found = false;
			for (size_t i = 0; i < v.size() && !found; ++i) {
				const GridNode * node = static_cast<const GridNode *>(v[i]);
				if (node->x == x && node->y == y) {
					found = true;
				}
			}
			if (!found) {
				FAIL() << "The node (" << x << ", " << y << ") should be returned as a neighbor of (3, 7), but it is not in the result vector.";
			}
		}
	}

	v = planner.getNeighborNodes(GridNode::get(4, 4));
	for (size_t i = 0; i < v.size(); ++i) {
		const GridNode * node = static_cast<const GridNode *>(v[i]);
		if (node->x == 5 && node->y == 5) {
			FAIL() << "The method returns nodes that are occupied in the map. These nodes must not be returned as neighbors.";
		}
	}

	v = planner.getNeighborNodes(GridNode::get(9, 9));
	for (size_t i = 0; i < v.size(); ++i) {
		const GridNode * node = static_cast<const GridNode *>(v[i]);
		if (node->x > 9 || node->y > 9) {
			FAIL() << "The method returns cells that are outside the map. The method should make sure only cells within the map bounds get returned.";
		}
	}

	v = planner.getNeighborNodes(GridNode::get(0, 0));
	for (size_t i = 0; i < v.size(); ++i) {
		const GridNode * node = static_cast<const GridNode *>(v[i]);
		if (node->x < 0 || node->y < 0) {
			FAIL() << "The method returns cells that are outside the map. The method should make sure only cells within the map bounds get returned.";
		}
	}
}

TEST(PathPlanning, expandNode) 
{
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
	
	GridMap map(10, 10, data);
	ManhattanDistanceHeuristic heuristic;
	GridPathPlanning planner(map, &heuristic);

	OpenList openList;
   	ClosedList closedList;

	GridNode *goal = GridNode::get(0, 4);
	//col,row
	openList.enqueue(GridNode::get(5, 9), 12);
	(GridNode::get(5, 9))->costs = 2;
	openList.enqueue(GridNode::get(5, 8), 11.41);
	(GridNode::get(5, 8))->costs = 2.414213562;
	openList.enqueue(GridNode::get(4, 7), 12.24);
	(GridNode::get(4, 7))->costs = 5.242640687;
	openList.enqueue(GridNode::get(7, 7), 12.82);
	(GridNode::get(7, 7))->costs = 2.828427125;
	openList.enqueue(GridNode::get(6, 6), 11.14);
	(GridNode::get(6, 6))->costs = 3.414213562;
	openList.enqueue(GridNode::get(7, 6), 12.83);
	(GridNode::get(7, 6))->costs = 3.828427125;
	openList.enqueue(GridNode::get(4, 5), 10.24);
	(GridNode::get(4, 5))->costs = 5.242640687;
	openList.enqueue(GridNode::get(5, 5), 10.83);
	(GridNode::get(5, 5))->costs = 4.828427125;


	closedList.add(GridNode::get(6, 9));
	(GridNode::get(6, 9))->costs = 1;
	closedList.add(GridNode::get(6, 8));
	(GridNode::get(6, 8))->costs = 1.414213562;
	closedList.add(GridNode::get(6, 7));
	(GridNode::get(6, 7))->costs = 2.414213562;
	closedList.add(GridNode::get(5, 6));
	(GridNode::get(5, 6))->costs = 3.828427125;
	
	closedList.add(GridNode::get(7, 6));
	planner.expandNode(GridNode::get(7, 6), goal, openList, closedList);

	if (openList.contains(GridNode::get(8,7)) == false || openList.contains(GridNode::get(8,6)) == false)
		FAIL() << "An expected expanded node is missing";
	if (openList.contains(GridNode::get(6,5) ) == true || openList.contains(GridNode::get(7,5) ) == true || openList.contains(GridNode::get(8, 5) ) == true)
		FAIL() << "An occupied cell is added to the open list";
	if (openList.contains(GridNode::get(6, 7)) == true )
		FAIL() << "A closed node is added to the open list";
	if (GridNode::get(8, 6)->costs == 0.0)
		FAIL() << "The method does not set node->costs to the current costs from the start node";
	if (fabs(GridNode::get(8, 6)->costs - (2.0 + 2.0 * sqrt(2.0))) > 1e-3) {
		FAIL() << "node->costs does not contain the correct costs of the path from the start node to the current node";
	}
	if (GridNode::get(8, 6)->getPredecessor() == NULL) {
		FAIL() << "The method does not set the predecessor of the newly added nodes";
	}
	if (GridNode::get(8, 6)->getPredecessor() != GridNode::get(7, 6)) {
		FAIL() << "The method does not set the predecessor of the newly added nodes to the correct predecessor node.";
	}

	if (round((GridNode::get(7, 7))->costs * 100)/100 != 2.83)
		FAIL() << "Updated the cost of a node that is not supposed to be updated (i.e. the cost of that node should remain unchanged, because it is the minimum cost that can be achieved  at that moment)";
	if (round((GridNode::get(6, 6))->costs * 100)/100 != 3.41)
		FAIL() << "Updated the cost of a node that is not supposed to be updated (i.e. the cost of that node should remain unchanged, because it is the minimum cost that can be achieved at that moment)";

	if (round((GridNode::get(8, 6))->costs * 100)/100 != 4.83)
		FAIL() << "Costs are not updated correctly";
	if (round((GridNode::get(8, 7))->costs * 100)/100 != 5.24)
		FAIL() << "Costs are not updated correctly";

	if (round(openList.getCosts(GridNode::get(8, 6)) * 100) / 100 == 4.83) {
		FAIL() << "The method inserts new nodes to the openList with costs g (= costs from start), but it should be f = g + h (costs so far plus heuristic to goal).";
	}
	if (round(openList.getCosts(GridNode::get(8, 6)) * 100) / 100 != 4.83 + 10.0) {
		FAIL() << "The method inserts new nodes to the openList with invalid costs, they should be f = g + h (costs so far plus heuristic to goal).";
	}

	return;
 
}

TEST(PathPlanning, planPath) 
{
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
	
	GridMap map(10, 10, data);
	StraightLineDistanceHeuristic heuristic;
	GridPathPlanning planner(map, &heuristic);

	OpenList openList;
   	ClosedList closedList;

	GridNode *goal = GridNode::get(0, 4);
	GridNode *start = GridNode::get(7, 9);

	std::deque<const AbstractNode*> path = planner.planPath(start,goal);
	std::deque<const AbstractNode*> shortestPath;


	if (path[0] != start)
		FAIL() << "The start node is not included in the path";
	if (path[path.size()-1] != goal)
		FAIL() << "The goal node is not included in the path";

	if (path.size()>10)
		FAIL()<<"Path is taller than expected";
	else if (path.size()<10)
		FAIL()<<"Path is shorter than expected";

	int lx = start->x, ly = start->y;
	double length = 0.0;
	for (size_t i = 0; i < path.size(); ++i) {
		const int x = static_cast<const GridNode*>(path[i])->x;
		const int y = static_cast<const GridNode*>(path[i])->y;
		if (map.isOccupied(x, y)) {
			FAIL() << "The planned path leads through an occupied cell.";
		}
		const double dx = static_cast<double>(x - lx);
		const double dy = static_cast<double>(y - ly);
		length += sqrt(dx * dx + dy * dy);
		lx = x;
		ly = y;
	}
	const double optLength = (6. + 3. * sqrt(2.));
	if (length - optLength > 0.1) {
		FAIL() << "The planned path is suboptimal. It has length " << length << ", but the shortest possible path has length " << optLength;
	}
	
	return;
 
}
TEST(PathPlanning, followPath) {
	std::vector<bool> data(100, false);
	GridMap map(10, 10, data);
	StraightLineDistanceHeuristic heuristic;
	GridPathPlanning planner(map, &heuristic);

	GridNode *a = GridNode::get(5, 1);
	GridNode *b = GridNode::get(2, 2);
	GridNode *c = GridNode::get(2, 5);
	GridNode *d = GridNode::get(3, 2);
	a->setPredecessor(b);
	b->setPredecessor(c);
	c->setPredecessor(d);

	std::deque<const AbstractNode *> path = planner.followPath(a);
	if (path.empty()) {
		FAIL() << "The method returns an empty path.";
	}
	if (path.size() == 1) {
		if (path[0] == a) {
			FAIL() << "The method returns only the current node instead of the complete path.";
		} else if (path[0] == d) {
			FAIL() << "The method returns only the start node instead of the complete path.";
		} else {
			FAIL() << "The method returns only a single node instead of the complete path.";
		}
	} else if (path.size() < 4) {
		FAIL() << "The method returns a path that is too short.";
	} else if (path.size() > 4) {
		FAIL() << "The method returns a path that is too long.";
	}
	if (path.size() == 4 && path[0] == a && path[1] == b && path[2] == c && path[3] == d) {
		return;
	}
	if (path[0] == a && path[1] == b && path[2] == c && path[3] == d) {
		FAIL() << "The method returns the path in reverse (from the goal to the start).";
	}
}


int main(int argc, char *argv[]) {
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

