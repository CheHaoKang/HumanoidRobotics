#ifndef PATH_PLANNING_H_
#define PATH_PLANNING_H_

#include <boost/fusion/include/pair.hpp>
#include <sstream>

#include <path_planning/AbstractNode.h>
#include <path_planning/GridNode.h>
#include <path_planning/GridMap.h>
#include <path_planning/Heuristic.h>
#include "ClosedList.h"
#include "OpenList.h"

namespace path_planning {

class GridHeuristic : public Heuristic {
public:
	GridHeuristic() {};
	virtual ~GridHeuristic() {};

	virtual double heuristic(const GridNode * const currentNode, const GridNode * const goalNode) const = 0;

	double heuristic(const AbstractNode * const currentNode, const AbstractNode * const goalNode) const {
		return heuristic(static_cast<const GridNode *>(currentNode), static_cast<const GridNode *>(goalNode));
	}
};

class StraightLineDistanceHeuristic : public GridHeuristic {
public:
	StraightLineDistanceHeuristic() {};
	virtual ~StraightLineDistanceHeuristic() {};
	double heuristic(const GridNode * const currentNode, const GridNode * const goalNode) const;
};

class ManhattanDistanceHeuristic : public GridHeuristic {
public:
	ManhattanDistanceHeuristic() {};
	virtual ~ManhattanDistanceHeuristic() {};
	double heuristic(const GridNode * const currentNode, const GridNode * const goalNode) const;
};

class PathPlanning
{
public:
	PathPlanning() {};
   	virtual ~PathPlanning() {};
   	std::deque<const AbstractNode*> planPath(const AbstractNode * const startNode, const AbstractNode * const goalNode);
   	std::deque<const AbstractNode*> followPath(const AbstractNode * const node);
   	void expandNode(const AbstractNode * const currentNode, const AbstractNode * const goalNode,
   			OpenList& openList, const ClosedList& closedList);
   	virtual std::vector<AbstractNode *> getNeighborNodes(const AbstractNode * const currentNode) = 0;
   	virtual double heuristic(const AbstractNode * const currentNode, const AbstractNode * const goalNode) = 0;
	virtual double getCosts(const AbstractNode * const currentNode, const AbstractNode * const successorNode) const = 0;
	virtual bool isCloseToGoal(const AbstractNode * const currentNode, const AbstractNode * const goalNode) = 0;
};


class GridPathPlanning : public PathPlanning {
public:
	GridPathPlanning(const GridMap& map, const GridHeuristic *heuristic) : map_(map), heuristic_(heuristic) {};
	virtual ~GridPathPlanning() {};
	double getCosts(const GridNode * const currentNode, const GridNode * const successorNode) const;
	std::vector<AbstractNode *> getNeighborNodes(const GridNode * const currentNode, const GridMap& map);
	bool isCloseToGoal(const GridNode * const currentNode, const GridNode * const goalNode);

   	virtual double heuristic(const AbstractNode * const currentNode, const AbstractNode * const goalNode) {
   		return heuristic_->heuristic(static_cast<const GridNode *>(currentNode), static_cast<const GridNode *>(goalNode));
   	}
	virtual double getCosts(const AbstractNode * const currentNode, const AbstractNode * const successorNode) const {
		return getCosts(static_cast<const GridNode *>(currentNode), static_cast<const GridNode *>(successorNode));
	}
	virtual std::vector<AbstractNode *> getNeighborNodes(const AbstractNode * const currentNode) {
		return getNeighborNodes(static_cast<const GridNode *>(currentNode), map_);
	}
	virtual bool isCloseToGoal(const AbstractNode * const currentNode, const AbstractNode * const goalNode) {
		return isCloseToGoal(static_cast<const GridNode *>(currentNode), static_cast<const GridNode *>(goalNode));
	}


private:
   	const GridMap map_;
   	const GridHeuristic *heuristic_;
};

}  // namespace path_planning


#endif  // PATH_PLANNING_H_
