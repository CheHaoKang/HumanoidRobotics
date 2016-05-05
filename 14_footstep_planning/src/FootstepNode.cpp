#include <footstep_planning/FootstepNode.h>
#include <cmath>
#include <angles/angles.h>

namespace footstep_planning {

FootstepNode::MapType FootstepNode::nodes;

int footstep_hash_value(const double& x, const double& y, const double& theta, const Foot& foot) {
	return    static_cast<int>(x * 100) * 1000000
			+ static_cast<int>(y * 100) * 20
			+ static_cast<int>(angles::normalize_angle_positive(theta) / M_PI * 180.0) / 18
			+ static_cast<int>(foot);
}

int hash_value(const FootstepNode &node) {
	return footstep_hash_value(node.x, node.y, node.theta, node.foot);
}

FootstepNode::FootstepNode(const double& x, const double& y, const double& theta, const Foot& foot)
: x(x), y(y), theta(theta), foot(foot) {
}

FootstepNode::~FootstepNode() {}


FootstepNode* FootstepNode::get(const double& x, const double&y,
		const double& theta, const Foot& foot) {
	FootstepNode *result = NULL;
	IndexType index = footstep_hash_value(x, y, theta, foot);
	MapType::const_iterator it = nodes.find(index);
	if (it == nodes.end()) {
		result = new FootstepNode(x, y, theta, foot);
		nodes[index] = result;
	} else {
		result = it->second;
	}
	return result;
}

std::string FootstepNode::toString() const {
	std::stringstream ss;
	ss << "(" << x << ", " << y << ", " << angles::to_degrees(angles::normalize_angle_positive(theta)) << " deg, "
			<< (foot == LEFT ? "left" : "right") << ")";
	return ss.str();
}

std::string FootstepNode::toLogString() const {
	std::stringstream ss;
	ss << x << " " << y << " " << theta << " " << (foot == LEFT ? "left" : "right");
	return ss.str();
}

}  // namespace footstep_planning
