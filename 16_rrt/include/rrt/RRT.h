#ifndef RRT_H_
#define RRT_H_

#include <rrt/GridNode.h>
#include <rrt/GridMap.h>
#include <vector>
#include <deque>

namespace rrt {

class RRT {
public:
	RRT() {}
	virtual ~RRT() {}
	virtual std::deque<AbstractNode *> planPath(AbstractNode * const start, AbstractNode * const goal, const GridMap& map, const size_t& maxIterations);

	virtual AbstractNode * getRandomNode(const GridMap& map, const std::vector<AbstractNode *>& list, AbstractNode * const listGoal) const;

	virtual double distance(GridNode * const node1, GridNode * const node2) const;
	virtual double distance(AbstractNode * const node1, AbstractNode * const node2) const {
		return distance(static_cast<GridNode *>(node1), static_cast<GridNode *>(node2));
	}

	virtual AbstractNode * getClosestNodeInList(AbstractNode * const randomNode, const std::vector<AbstractNode *>& list) const;

	virtual AbstractNode * extendClosestNode(GridNode * const randomNode, GridNode * const closestNode, std::vector<AbstractNode *> & list, const GridMap& map,
				const std::vector<AbstractNode *> & otherList) const;

	virtual AbstractNode * extendClosestNode(AbstractNode * const randomNode, AbstractNode * const closestNode, std::vector<AbstractNode *> & list, const GridMap& map,
				const std::vector<AbstractNode *> & otherList) const {
		return extendClosestNode(static_cast<GridNode * const>(randomNode), static_cast<GridNode * const>(closestNode), list, map, otherList);
	}

	virtual std::deque<AbstractNode *> constructPath(AbstractNode * const connectionNode, AbstractNode * const startNode, AbstractNode * const goalNode) const;
	AbstractNode* tryToConnect(GridNode* const currentNode, const std::vector<AbstractNode*>& neighbors,
			const std::vector<AbstractNode*>& otherList) const;

	std::vector<AbstractNode*> getNeighbors(GridNode* const currentNode, const std::vector<AbstractNode*>& list, const GridMap& map) const;
	void addNearestNeighbor(GridNode* const currentNode, const std::vector<AbstractNode*>& neighbors, GridNode* const randomNode,
			std::vector<AbstractNode*>& list) const;
};

}  // namespace rrt

#endif  // RRT_H_
