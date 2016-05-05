#ifndef FOOTSTEP_PLANNING_FOOTSTEPNODE_H_
#define FOOTSTEP_PLANNING_FOOTSTEPNODE_H_

#include <path_planning/AbstractNode.h>
#include <boost/unordered_map.hpp>

namespace footstep_planning {

enum Foot { LEFT = 0, RIGHT = 1 };

class FootstepNode : public path_planning::AbstractNode {
public:
	double x;       //< x position in meters
	double y;       //< y position in meters
	double theta;   //< orientation in radians
	Foot foot;      //< foot (LEFT or RIGHT)

	virtual ~FootstepNode();

	static FootstepNode* get(const double& x, const double&y, const double& theta, const Foot& foot);
	bool operator==(const FootstepNode& other) const;
	std::string toString() const;
	std::string toLogString() const;

private:
	typedef int IndexType;
	typedef boost::unordered_map<IndexType, FootstepNode*> MapType;
	static MapType nodes;
	FootstepNode(const double& x, const double& y, const double& theta, const Foot& foot);

};

int hash_value(const FootstepNode &node);

}

#endif /* FOOTSTEP_PLANNING_FOOTSTEPNODE_H_ */
