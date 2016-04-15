#ifndef PATH_PLANNING_GRIDNODE_H_
#define PATH_PLANNING_GRIDNODE_H_

#include <boost/unordered_map.hpp>
#include <sstream>
#include <string>
#include <path_planning/AbstractNode.h>

namespace path_planning {

class GridNode : public AbstractNode {
public:
	const int x;
	const int y;

	static GridNode* get(const int& x, const int&y);
	std::string toString() const;
	std::string toLogString() const;

private:
	GridNode(const int& x, const int& y) : x(x), y(y) {};
	typedef std::pair<int, int> IndexType;
	typedef boost::unordered_map<IndexType, GridNode*> MapType;
	static MapType nodes;
};

int hash_value(const GridNode &node);

}  // namespace path_planning


#endif /* PATH_PLANNING_GRIDNODE_H_ */
