#include <rrt/RRT.h>
#include <stdlib.h>  
#include <math.h> 
#include <time.h>
#include <climits>

namespace rrt {

/**
 * \brief Draws a random grid node that is not occupied and not in the list.
 * \param[in] map The grid map of the environment.
 * \param[in] list The list of nodes that have already been explored.
 * \param[in] listGoal The goal of the tree.
 * \return The random node.
 */
AbstractNode * RRT::getRandomNode(const GridMap& map, const std::vector<AbstractNode *>& list, AbstractNode * const listGoal) const {
	AbstractNode* randomNode = NULL;
	// TODO: Draw a random grid cell (90% probability) or return the listGoal (10% probability)

	/* Available methods and fields:
	 * - map.width: width of the map in cells 
	 * - map.height: height of the map in cells
	 * - map.isOccupied(int x, int y): returns true iff the cell is occupied by an obstacle
	 * - GridNode::get(int x, int y): creates and returns a new node representing a cell.
	 */
	double randomNumber =(double) rand() / (RAND_MAX);
		if(randomNumber <= 0.9)
		{
			int x = (int)rand()%map.width ;
			int y = (int)rand()%map.height;
			bool genNext = true;
			while(genNext)
			{
				if(!map.isOccupied(x,y)){
					randomNode = GridNode::get(x,y);
					if (std::find(list.begin(), list.end(), randomNode) == list.end())
						genNext = false;
				}
				x =  (int)rand()%map.width ;
				y =  (int)rand()%map.height;
			 }
		}
		else
			randomNode = listGoal;
	return randomNode;
}

/**
 * \brief Calculates the Euclidean distance between two nodes.
 * \param[in] node1 The first node.
 * \param[in] node2 The second node.
 * \return The Euclidean distance between the two nodes.
 */
double RRT::distance(GridNode * const node1, GridNode * const node2) const {
	double dist = 0.0;

	// TODO: Return the Euclidean distance between the two nodes.

	/* Available methods and fields:
	 * - node->x: the x index of the cell
	 * - node->y: the y index of the cell
	 */

	dist = sqrt(pow(node1->x - node2->x,2)+pow(node1->y - node2->y,2));
	return dist;
}

/**
 * \brief Given a node, this method returns the index of the nearest node in a list.
 * \param[in] node The reference node.
 * \param[in] list The list of nodes that should be searched for the closest node.
 * \return The closest node, or NULL if the list is empty.
 */
AbstractNode * RRT::getClosestNodeInList(AbstractNode * const node, const std::vector<AbstractNode *>& list) const {
	AbstractNode * nearestNode = NULL;

	// TODO: Return the index of the closest node from the list.

	/* Available methods:
	 * - distance(node1, node2): Defined above, returns the Euclidean distance
	 */
	 double dist = 10000;
	 for(size_t i = 0; i < list.size(); ++i){
	 	double tmp_dist = RRT::distance(node, list[i]);
	 	if(tmp_dist < dist){
	 		nearestNode = list[i];
	 		dist = tmp_dist;
	 	}
	 }

	return nearestNode;
}

/**
 * \brief Returns the neighbors of a grid cell that are not occupied and not already expanded.
 * \param[in] currentNode The current grid node.
 * \param[in] list The list of already expanded nodes in the current tree.
 * \param[in] map The grid map.
 */
std::vector<AbstractNode*> RRT::getNeighbors(GridNode * const currentNode, const std::vector<AbstractNode*>& list, const GridMap& map) const {
	std::vector<AbstractNode*> neighbors;

	/* TODO: Fill the neighbors vector with neighbor cells of currentNode that are
	 * within the map bounds, not occupied, and not in the list of already expanded nodes.
	 */

	/* Available methods and fields:
	 * - node->x: the x index of the cell
	 * - node->y: the y index of the cell
	 * - map.width: width of the map in cells
	 * - map.height: height of the map in cells
	 * - map.isOccupied(int x, int y): returns true iff the cell is occupied by an obstacle
	 * - GridNode::get(int x, int y): creates and returns a new node representing a cell.
	 */
	int x = currentNode->x;
	int y = currentNode->y;
	GridNode* topRight = NULL;
	GridNode* top = NULL;
	GridNode* topLeft= NULL;
	GridNode* left = NULL;
	GridNode* bottomLeft = NULL;
	GridNode* bottomRight = NULL;
	GridNode* bottom = NULL;
	GridNode* right = NULL;
	std::vector<GridNode *> tmp_grid_vector;

	if(x>0 && y>0)
	{
		topRight = GridNode::get(x-1,y-1);
	}

	if(y>0){
		top = GridNode::get(x,y-1);

	}



	if(x<map.width-1 && y>0){
		topLeft = GridNode::get(x+1,y-1);
}


	if(x<map.width-1){
		left = GridNode::get(x+1,y);

	}

	if(x<map.width-1 && y<map.height-1){
		bottomLeft = GridNode::get(x+1,y+1);
}

	if(x>0 && y<map.height-1){
		bottomRight = GridNode::get(x-1, y+1);

	}

	if(y<map.height-1){		bottom = GridNode::get(x,y+1);

	}

	if(x>0){
		right = GridNode::get(x-1,y);

	}

	tmp_grid_vector.push_back(topRight);
	tmp_grid_vector.push_back(topLeft);
	tmp_grid_vector.push_back(right);
	tmp_grid_vector.push_back(bottomRight);
	tmp_grid_vector.push_back(bottom);
	tmp_grid_vector.push_back(bottomLeft);
	tmp_grid_vector.push_back(left);
	tmp_grid_vector.push_back(top);
	
	std::vector<GridNode *>::iterator it;
	for (it =tmp_grid_vector.begin();it!=tmp_grid_vector.end();it++ )
	{
		if (*it)
		{
		GridNode * tGridNode = *it;
		if(!map.isOccupied(tGridNode->x,tGridNode->y))
			if (std::find(list.begin(), list.end(), tGridNode) == list.end())
				neighbors.push_back(*it);
		}
	}


	return neighbors;
}

/**
 * \brief Tries to connect the two trees and returns the connection node.
 * \param[in] currentNode The current node.
 * \param[in] neighbors The list of neighbors of the current node.
 * \param[in] otherList The list of already expanded nodes in the other tree.
 * \return The neighbor node that connects both trees, or NULL if the trees cannot be connected.
 */
AbstractNode * RRT::tryToConnect(GridNode* const currentNode, const std::vector<AbstractNode*>& neighbors,
		const std::vector<AbstractNode*>& otherList) const {
	AbstractNode* connectionNode = NULL;

	/* TODO: Check if one of the neighbors is already contained in the "otherList"
	 * (list of already expanded nodes in the other tree). If so, return that neighbor
	 * as the connection node and establish the connection with neighbor->setConnection(closestNode).

	/* Available methods and fields:
	 * - node->setConnection(AbstractNode * connection): sets the other predecessor node of the
	 *      current node (must be from the other list) (i.e. set connection between the two lists).
	 */

	std::vector<AbstractNode*>::const_iterator it;
	for(it = neighbors.begin(); it!=neighbors.end();it++)
	{
		if (std::find(otherList.begin(), otherList.end(), *it) != otherList.end())
		{
			connectionNode = *it;
			connectionNode->setConnection(currentNode);
		}
	}
 return connectionNode;
}

/**
 * \brief Determines the neighbor that is closest to the random node, sets its predecessor
 * to the current node, and adds it to the list of explored nodes.
 * \param[in] currentNode The current node.
 * \param[in] neighbors The list of neighbors of the current node.
 * \param[in] randomNode The randomly drawn node.
 * \param[in,out] list The list of already expanded nodes.
 */
void RRT::addNearestNeighbor(GridNode* const currentNode, const std::vector<AbstractNode*>& neighbors,
		GridNode* const randomNode, std::vector<AbstractNode*>& list) const {

	/* TODO: Determine the neighbor that is closest to the random node, set its predecessor
	 * to the current node, and add it to the list of explored nodes.
	 */

	/* Available methods and fields:
	 * - node->setPredecessor(AbstractNode* node): store the predecessor node for a node (required
	 *     later for extracting the path)
	 * - getClosestNodeInList(node, list): Defined above
	 */


	int min_cost = INT_MAX;
	AbstractNode* minNode = NULL;
	std::vector<AbstractNode *>::const_iterator it;
	it = neighbors.begin();
	for (;it != neighbors.end(); it++ )
	{
		double cost = distance(*it,randomNode);
		if (cost < min_cost)
		{
			minNode = *it;
			min_cost = cost;
		}
	}
	minNode->setPredecessor(currentNode)  ;
	list.push_back(minNode);


	// int min_cost = INT_MAX;
	// AbstractNode* minNode = NULL;
	// std::vector<AbstractNode*>::iterator it;
	// for(it=neighbors.begin() ; it < neighbors.end(); it++)
	// {
	// 	double cost = distance(*it,randomNode);
	// 	if (cost < min_cost)
	// 	{
	// 		minNode = *it;
	// 		min_cost = cost;
	// 	}
	// }
	// minNode->predecessor = currentNode;
	// list.push_back(minNode);

}

/**
 * \brief Given a node, this method expands the nearest node in a list, where the new expanded neighbor is the closest neighbor with respect to the given node.
 * \param[in] randomNode The reference node based on which the list should be expanded.
 * \param[in] closestNodeIndex The index of the closest node in the list to the randomNode.
 * \param[in,out] list The list of nodes that should be expanded for the closest node.
 * \param[in] map The grid map of the environment.
 * \return The connection node if both trees are connected, or NULL otherwise.
 */
AbstractNode * RRT::extendClosestNode(GridNode * const randomNode, GridNode * const closestNode,
		std::vector<AbstractNode *> & list, const GridMap& map,
		const std::vector<AbstractNode *> & otherList) const {

	// Nothing to do in this method - we've already implemented it for you :-)

	const std::vector<AbstractNode *> neighbors = getNeighbors(closestNode, list, map);
	AbstractNode * connectionNode = tryToConnect(closestNode, neighbors, otherList);
	if (connectionNode == NULL && !neighbors.empty()) {
		addNearestNeighbor(closestNode, neighbors, randomNode, list);
	}
	return connectionNode;
}

/**
 * \brief Reconstructs the path from the start to the goal once the connection is found.
 * \param[in] connectionNode The connection node where both trees meet.
 * \param[in] startNode The start node.
 * \param[in] goalNode The goal node.
 * \return The path from the start node to the goal node.
 */
std::deque<AbstractNode *> RRT::constructPath(AbstractNode * const connectionNode, AbstractNode * const startNode, AbstractNode * const goalNode) const {
	std::deque<AbstractNode *> path;
	if (connectionNode == NULL) {
		return path;
	}

	/* TODO: Reconstruct the path from the start node to the goal node in the correct order.
	 *
	 * Hints:
	 * - Start with the connection node and follow the chain of predecessors in both trees.
	 * - Depending on which tree the connection node is part of, you may have to reverse
	 *   the order of the nodes in the end.
	 * */

	/* Available methods:
	 * - node->getPredecessor(): returns the predecessor saved with setPredecessor()
	 * - node->getConnection() : returns the predecessor from the other list saved with setConnection()
	 * - path.push_front(AbstractNode* node): Inserts the node at the beginning of the path
	 * - path.push_back(AbstractNode* node): Inserts the node at the end of the path
	 */

	 for(AbstractNode * _node = connectionNode;_node!=NULL;_node = _node->getPredecessor())
	 {
		 path.push_front(_node);
	 }
	 for(AbstractNode * _node = connectionNode->getConnection();_node!=NULL;_node = _node->getPredecessor())
		 {
			 path.push_back(_node);
		 }

	 if (path[0]!=startNode)
	 {
		 std::deque<AbstractNode *> _path;
		 for (int i =0;i<path.size();i++)
		 {
			 _path.push_front(path[i]);
		 }
		 path = _path;
	 }
	return path;
}

/**
 * \brief Plans a path on a grid map using RRT.
 * \param[in] startNode The start node of the path.
 * \param[in] goalNode The goal node where the path should end up.
 * \param[in] map The occupancy grid map of the environment.
 * \param[in] maxIterations The maximum number of iterations.
 * \return The planned path, or an empty path in case the algorithm exceeds the maximum number of iterations.
 */
std::deque<AbstractNode *> RRT::planPath(AbstractNode * const startNode, AbstractNode * const goalNode, const GridMap& map, const size_t& maxIterations) {
	std::deque<AbstractNode *> result;

	std::vector<AbstractNode *> startList;
	std::vector<AbstractNode *> goalList;

	// Add the start and goal nodes to the corresponding lists:
	startList.push_back(startNode);
	goalList.push_back(goalNode);

	/* TODO:  Expand trees from both the start node and the goal node at the same time
	 * until they meet. When extendClosestNode() returns a connection node, then call
	 * constructPath() to find the complete path and return it. */

	int iter = 0;
	while(iter < maxIterations)
	{
		AbstractNode * Qrand;
		Qrand = getRandomNode(map,startList,goalList.back() );
		AbstractNode * Qclosest= getClosestNodeInList(Qrand,startList);

		AbstractNode * Qconnection = extendClosestNode(Qrand,Qclosest,startList,map,goalList);
		if (Qconnection->getConnection() !=goalNode) {
			startList.push_back(Qconnection);
			startList.swap(goalList);
					iter++;
		}
		else {
			result = constructPath(Qconnection,startNode,goalNode);
			return	result;
		}


	}

	return result;
}
}  // namespace rrt
