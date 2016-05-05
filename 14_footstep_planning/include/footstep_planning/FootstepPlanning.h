#ifndef FOOTSTEP_PLANNING_H_
#define FOOTSTEP_PLANNING_H_

#include <path_planning/PathPlanning.h>
#include <footstep_planning/FootstepPlanning.h>
#include <footstep_planning/FootstepMap.h>
#include <footstep_planning/FootstepNode.h>
#include <angles/angles.h>

using namespace path_planning;

namespace footstep_planning {

class FootstepHeuristic : public Heuristic {
public:
	FootstepHeuristic() {};
	virtual ~FootstepHeuristic() {};

	double heuristic(const FootstepNode * const currentFootstep, const FootstepNode * const goalFootstep) const;

	double heuristic(const AbstractNode * const currentNode, const AbstractNode * const goalNode) const {
		return heuristic(static_cast<const FootstepNode *>(currentNode), static_cast<const FootstepNode *>(goalNode));
	}
};

class FootstepPlanning : public PathPlanning {
public:
	struct FootstepAction {
		double dx, dy, dtheta;
		Foot foot;
		FootstepAction(const double& dx, const double& dy, const double& dtheta, const Foot& foot)
		: dx(dx), dy(dy), dtheta(dtheta), foot(foot) {};
	};

	std::vector<FootstepAction> leftFootActions, rightFootActions;

	FootstepPlanning(const FootstepMap * const footstepMap) : footstepMap(footstepMap) {
		rightFootActions.push_back(FootstepAction( 0.00,  0.16,   0.00,  RIGHT));
		rightFootActions.push_back(FootstepAction( 0.08,  0.09,   0.00,  RIGHT));
		rightFootActions.push_back(FootstepAction(-0.04,  0.09,   0.00,  RIGHT));
		rightFootActions.push_back(FootstepAction( 0.00,  0.12,   0.00,  RIGHT));
		rightFootActions.push_back(FootstepAction( 0.05,  0.14,   0.00,  RIGHT));
		rightFootActions.push_back(FootstepAction( 0.01,  0.13,  -0.50,  RIGHT));
		rightFootActions.push_back(FootstepAction( 0.015, 0.100,  0.500, RIGHT));
		rightFootActions.push_back(FootstepAction( 0.04,  0.12,   0.30,  RIGHT));
		rightFootActions.push_back(FootstepAction(-0.03,  0.12,   0.50,  RIGHT));
		rightFootActions.push_back(FootstepAction( 0.06,  0.12,   0.00,  RIGHT));
		rightFootActions.push_back(FootstepAction( 0.04,  0.10,   0.00,  RIGHT));
		rightFootActions.push_back(FootstepAction(-0.02,  0.12,   0.00,  RIGHT));
		for (std::vector<FootstepAction>::const_iterator it = rightFootActions.begin(); it != rightFootActions.end(); ++it) {
			leftFootActions.push_back(FootstepAction(it->dx, -it->dy, it->dtheta, LEFT));
		}
	};
   	virtual ~FootstepPlanning() {};

   	virtual std::vector<AbstractNode *> getNeighborNodes(const FootstepNode * const currentFootstep);
   	virtual double getCosts(const FootstepNode * const currentFootstep, const FootstepNode * const successorFootstep) const;

   	virtual double heuristic(const AbstractNode * const currentNode, const AbstractNode * const goalNode) {
   		return heuristic_.heuristic(currentNode, goalNode);
   	}
   	virtual double getCosts(const AbstractNode * const currentNode, const AbstractNode * const successorNode) const {
   		return getCosts(static_cast<const FootstepNode *>(currentNode), static_cast<const FootstepNode *>(successorNode));
   	}
   	virtual std::vector<AbstractNode *> getNeighborNodes(const AbstractNode * const currentNode) {
   		return getNeighborNodes(static_cast<const FootstepNode *>(currentNode));
   	}

   	virtual double getDistanceToNearestObstacle(const FootstepNode * const step) const {
   		return footstepMap->getDistanceToNearestObstacle(step->x, step->y, step->theta);
   	}

   	virtual bool isColliding(const FootstepNode * const step) {
   		return footstepMap->getDistanceToNearestObstacle(step->x, step->y, step->theta) < 0.01;
   	}

   	virtual bool isCloseToGoal(const FootstepNode * const currentNode, const FootstepNode * const goalNode);

   	virtual bool isCloseToGoal(const AbstractNode * const currentNode, const AbstractNode * const goalNode) {
   		return isCloseToGoal(static_cast<const FootstepNode *>(currentNode), static_cast<const FootstepNode *>(goalNode));
   	}

   	virtual FootstepNode* executeFootstep(const FootstepNode * const currentFootstep, const FootstepAction& action);

protected:
   	FootstepHeuristic heuristic_;
   	const FootstepMap * const footstepMap;
};

}  // namespace footstep_planning

#endif  // FOOTSTEP_PLANNING_H_
