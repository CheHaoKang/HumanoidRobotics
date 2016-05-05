#ifndef RRT_GRIDNODE_H_
#define RRT_GRIDNODE_H_

#include <boost/unordered_map.hpp>
#include <sstream>
#include <string>
#include <rrt/AbstractNode.h>

namespace rrt {

class GridNode: public AbstractNode {
public:
	const int x;
	const int y;

	static GridNode* get(const int& x, const int&y);
	std::string toString() const;
	std::string toLogString() const;

private:
	GridNode(const int& x, const int& y) : x(x), y(y) {}
	typedef std::pair<int, int> IndexType;
	typedef boost::unordered_map<IndexType, GridNode*> MapType;
	static MapType nodes;
};

int hash_value(GridNode &node);

}  // namespace rrt

#endif /* RRT_GRIDNODE_H_ */
