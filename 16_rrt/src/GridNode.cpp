/*
 * Just some helper functions, nothing to do here.
 */

#include <rrt/GridNode.h>

namespace rrt {

GridNode::MapType GridNode::nodes;

int hash_value(GridNode &node) {
	return node.x * 1000000 + node.y;
}

GridNode* GridNode::get(const int& x, const int&y) {
	GridNode *result = NULL;
	IndexType index = std::make_pair(x, y);
	MapType::const_iterator it = nodes.find(index);
	if (it == nodes.end()) {
		result = new GridNode(x, y);
		nodes[index] = result;
	} else {
		result = it->second;
	}
	return result;
}

std::string GridNode::toString() const {
	std::stringstream ss;
	ss << "(" << x << ", " << y << ")";
	return ss.str();
}

std::string GridNode::toLogString() const {
	std::stringstream ss;
	ss << x << " " << y;
	return ss.str();
}

}  // namespace rrt
