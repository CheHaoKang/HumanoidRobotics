#ifndef PATH_PLANNING_HEURISTIC_H_
#define PATH_PLANNING_HEURISTIC_H_

#include <path_planning/AbstractNode.h>

namespace path_planning {

class Heuristic {
public:
	Heuristic() {};
	virtual ~Heuristic() {};

	virtual double heuristic(const AbstractNode * const currentNode, const AbstractNode * const goalNode) const = 0;
};

}



#endif /* PATH_PLANNING_HEURISTIC_H_ */
