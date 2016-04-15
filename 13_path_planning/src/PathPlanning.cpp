#include <path_planning/PathPlanning.h>

namespace path_planning {

/**
 * \brief Calculates the costs for traveling from currentNode to successorNode.
 * \param[in] currentNode The current grid cell.
 * \param[in] successorNode The next grid cell where the robot will travel to.
 * \return The costs (i.e., the Euclidean distance) for traveling from currentNode to successorNode.
 */
double GridPathPlanning::getCosts(const GridNode * const currentNode, const GridNode * const successorNode) const {
	double result = 0.0;

	/* TODO: calculate the costs for traveling from currentNode to successorNode
	 * (= Euclidean distance between the two grid cells) */

	/* Available fields:
	 * - node->x: the x index of the cell
	 * - node->y: the y index of the cell
	 */
	result = sqrt((currentNode->x-successorNode->x)*(currentNode->x-successorNode->x) +
					(currentNode->y-successorNode->y)*(currentNode->y-successorNode->y));

	return result;
}


/**
 * \brief Calculates the straight line heuristic between the current node and the goal.
 * \param[in] currentNode The current grid cell.
 * \param[in] goalNode The goal grid cell.
 * \return The straight line distance between the current cell and the goal.
 */
double StraightLineDistanceHeuristic::heuristic(const GridNode* const currentNode, const GridNode* const goalNode) const {
	double result = 0.0;

	/* TODO: calculate the straight line distance heuristic between the
	 * current node and the goal node. 	 */

	/* Available fields:
	 * - node->x: the x index of the cell
	 * - node->y: the y index of the cell
	 */

	result = sqrt((currentNode->x-goalNode->x)*(currentNode->x-goalNode->x) +
			(currentNode->y-goalNode->y)*(currentNode->y-goalNode->y));

	return result;
}

/**
 * \brief Calculates the Manhattan heuristic between the current node and the goal.
 * \param[in] currentNode The current grid cell.
 * \param[in] goalNode The goal grid cell.
 * \return The Manhattan distance between the current cell and the goal.
 */
double ManhattanDistanceHeuristic::heuristic(const GridNode* const currentNode, const GridNode* const goalNode) const {
	double result = 0.0;

	/* TODO: calculate the Manhattan distance heuristic between the
	 * current node and the goal node.  */

	/* Available fields:
	 * - node->x: the x index of the cell
	 * - node->y: the y index of the cell
	 */
	result = fabs(currentNode->x-goalNode->x) + fabs(currentNode->y-goalNode->y);


	return result;
}

/**
 * \brief Returns a vector of neighbor nodes of the current node where the robot can travel to.
 * \param[in] currentNode The current grid cell.
 * \param[in] map The grid map of the environment.
 * \return A vector of neighbor grid cells that are accessible to the robot.
 */
std::vector<AbstractNode *> GridPathPlanning::getNeighborNodes(const GridNode * const currentNode, const GridMap& map) {
	std::vector<AbstractNode *> result;

	/* TODO: Fill the vector "result" with the eight neighbor nodes of currentNode.
	 * Only add nodes that are within the map bounds and that are not occupied by
	 * an obstacle in the map.  */

	/* Available methods and fields:
	 * - node->x: the x index of the cell
	 * - node->y: the y index of the cell
	 * - map.width: width of the map in cells
	 * - map.height: height of the map in cells
	 * - map.isOccupied(int x, int y): returns true iff the cell is occupied by an obstacle
	 * - GridNode::get(int x, int y): creates and returns a new node representing a cell.
	 */

	std::cout<<"+++++++++"<<std::endl;
	std::cout<<"width:"<<map.width<<" height:"<<map.height<<std::endl;

	for(int i=-1; i <= 1; i++){
		for(int j=-1; j <= 1; j++) {
			if(i==0 && j==0)
				continue;

			std::cout<<"i:"<<i<<" j:"<<j<<std::endl;
			std::cout<<"x:"<<currentNode->x<<" y:"<<currentNode->y<<std::endl;
			std::cout<<"---------"<<std::endl;

			if(currentNode->x+i >= 0 && currentNode->x+i < map.width
					&& currentNode->y+j >= 0 && currentNode->y+j < map.height) {
				if(!(map.isOccupied(currentNode->x+i, currentNode->y+j))) {
					//if(abs(i)==1 && abs(j)==1 && map.isOccupied(currentNode->x+i, currentNode->y) && map.isOccupied(currentNode->x, currentNode->y+j))
					//	continue; // test
					result.push_back(GridNode::get(currentNode->x+i, currentNode->y+j));
				}
			}
		}
	}

	return result;
}


/**
 * \brief Expands the current node and adds its neighbor cells to the list of open cells.
 * \param[in] currentNode The current node.
 * \param[in] goalNode The goal node where the robot should travel to.
 * \param[in,out] openList The list of open nodes.
 * \param[in] closedList The list of nodes that are already visited and closed by the algorithm.
 */
void PathPlanning::expandNode(const AbstractNode * const currentNode, const AbstractNode * const goalNode,
		OpenList& openList, const ClosedList& closedList) {

	/* TODO: Expand the currentNode and add the neighbor cells to the openList. */

	/* Available methods and fields:
	 * - getNeighborNodes(AbstractNode* node): defined above, returns a vector of neighbor nodes.
	 * - getCosts(AbstractNode* from, AbstractNode* to): defined above, returns the costs for traveling
	 *     from one node to another node.
	 * - heuristic(AbstractNode* node, AbstractNode* goalNode): defined above, returns the value of
	 *     one of the heuristics
	 *
	 * - node->costs: field for storing cost values, read/write access
	 * - node->setPredecessor(AbstractNode* node): store the predecessor node for a node (required
	 *     later for extracting the path)
	 *
	 * - closedList.contains(AbstractNode* node): returns true if the node is already on list of closed nodes.
	 *
	 * - openList.enqueue(AbstractNode *node, double costs): put a node into the open list queue, the
	 *     node with the lowest costs will be processed next.
	 * - openList.updateCosts(AbstractNode *node, double costs): update the costs of a node that is
	 *     already in the open list queue.
	 * - openList.contains(AbstractNode* node): returns true if the node is already on the list of open nodes.
	 */

	std::vector<AbstractNode *> neighbours;
	neighbours = getNeighborNodes(currentNode);


	for(int i=0; i < neighbours.size(); i++) {
		if(!(closedList.contains(neighbours[i]))) {
			if(openList.contains(neighbours[i])) {
				if((currentNode->costs + getCosts(currentNode, neighbours[i])) < neighbours[i]->costs) {
					neighbours[i]->setPredecessor(currentNode);
					neighbours[i]->costs = currentNode->costs + getCosts(currentNode, neighbours[i]);
					openList.updateCosts(neighbours[i], currentNode->costs + getCosts(currentNode, neighbours[i]) + heuristic(neighbours[i], goalNode));
				}
			} else {
				neighbours[i]->setPredecessor(currentNode);
				neighbours[i]->costs = currentNode->costs + getCosts(currentNode, neighbours[i]);
				openList.enqueue(neighbours[i], currentNode->costs + getCosts(currentNode, neighbours[i]) + heuristic(neighbours[i], goalNode));
			}
		}
	}
//	for(int i=0; i<neighbours.size(); i++) {
//		//neighbours[i]->costs = currentNode->costs + getCosts(neighbours[i], currentNode) + heuristic(neighbours[i], goalNode);
//
//		neighbours[i]->setPredecessor(currentNode);
//
//		if(openList.contains(neighbours[i]))
//			openList.updateCosts(neighbours[i], neighbours[i]->costs);
//		else if(!(closedList.contains(neighbours[i])))
//			openList.enqueue(neighbours[i], neighbours[i]->costs);
//	}
}

/**
 * \brief Returns true if the current node is close to the goal node.
 * \param[in] currentNode The current node.
 * \param[in] goalNode The goal node.
 * \return True iff the current grid cell equals the goal grid cell.
 */
bool GridPathPlanning::isCloseToGoal(const GridNode * const currentNode, const GridNode * const goalNode) {
	// We already implemented this method for you, nothing to do here.
	return ((currentNode->x == goalNode->x) && (currentNode->y == goalNode->y));
}


/**
 * \brief Plans a path from a start node to a goal node.
 * \param[in] startNode The start node.
 * \param[in] goalNode The goal node.
 * \return The optimal path from the start node to the end node.
 */
std::deque<const AbstractNode*> PathPlanning::planPath(const AbstractNode * const startNode, const AbstractNode * const goalNode) {
	// Create empty lists for open nodes and closed nodes
   	OpenList openList;
   	ClosedList closedList;

   	std::deque<const AbstractNode*> resultPath;
   	const AbstractNode * currentNode;

   	/* TODO: Fill resultPath by planning a path from the startNode to the goalNode */

   	/* Available methods:
   	 * - isCloseToGoal(AbstractNode *node, AbstractNode *goalNode): defined above, use this method to check
   	 *     whether the robot has reached the goal.
	 * - openList.enqueue(AbstractNode* node, double costs): put a node into the open list queue, the
	 *     node with the lowest costs will be processed next.
	 * - openList.removeMin(): returns the node with the lowest costs from the open list queue
	 *     and removes the node from the queue.
	 * - closedList.add(AbstractNode* node): put a node into the list of closed nodes
     * - closedList.contains(AbstractNode* node): returns true if the node is already on list of closed nodes.
     * - expandNode(...): defined above, adds the valid neighbors of the current node to the open list.
     * - followPath(AbstractNode* node): defined below, extracts the path from the start node
     *     to the current node by following the chain of predecessors.
   	 */
   	openList.enqueue(startNode, heuristic(startNode, goalNode));

   	do {
   		currentNode = openList.removeMin();
   		expandNode(currentNode, goalNode, openList, closedList);
   		closedList.add(currentNode);
   	} while(!(isCloseToGoal(currentNode, goalNode)));

   	resultPath = followPath(goalNode);

	return resultPath;
}

/**
 * \brief Extracts the path from the currentNode back to the start node.
 * \param[in] node The current node.
 * \return The path from the start node up to the current node.
 */
std::deque<const AbstractNode*> PathPlanning::followPath(const AbstractNode * const node) {
	std::deque<const AbstractNode *> path;
	/* TODO: Fill the path by following the predecessors from the current node back
	 * to the start node. */

	/* Available methods:
	 * - node->getPredecessor(): returns the predecessor saved with setPredecessor()
	 * - path.push_front(AbstractNode* node): Inserts the node at the beginning of the path
	 * - path.push_back(AbstractNode* node): Inserts the node at the end of the path
	 */
	//const AbstractNode *temp;

	const AbstractNode * currentNode = node;
	while(currentNode) {
		path.push_front(currentNode);
		//temp = node->getPredecessor();
		//node = node->getPredecessor();
		//node->setPredecessor(node->getPredecessor());
		currentNode = currentNode->getPredecessor();
	}

//	AbstractNode *temp;
//
//	temp = node;
//
//	while(temp->getPredecessor()) {
//		path.push_front(node->getPredecessor());
//		node->setPredecessor(node->getPredecessor());
//	}
//	//temp = node->getPredecessor();
//
////	if(node->getPredecessor()) {
////		path.push_back(node->getPredecessor());
////		return followPath(node->getPredecessor());
////	} else
	return path;
}

}  // namespace path_planning
