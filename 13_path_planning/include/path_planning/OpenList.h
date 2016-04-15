#ifndef PATH_PLANNING_OPENLIST_HPP_
#define PATH_PLANNING_OPENLIST_HPP_

#include <boost/heap/fibonacci_heap.hpp>
#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>
#include <path_planning/FileIO.h>
#include <path_planning/AbstractNode.h>

namespace path_planning {

class OpenList {
public:
	OpenList();
	virtual ~OpenList();

	void enqueue(const AbstractNode * const node, const double costs);
	const AbstractNode * removeMin();
	void updateCosts(const AbstractNode * const node, const double costs);
	bool isEmpty() const;
	bool contains(const AbstractNode * const node) const;
	double getCosts(const AbstractNode * const node) const;

private:
	struct NodeWrapper;
	struct CompareNode {
		bool operator()(const NodeWrapper* const node1, const NodeWrapper* const node2) const {
			return node1->costs > node2->costs;
		}
	};
	struct NodeWrapper {
		const AbstractNode * node;
		double costs;
		typename boost::heap::fibonacci_heap<NodeWrapper*, boost::heap::compare<CompareNode> >::handle_type handle;

		NodeWrapper(const AbstractNode * const node, const double costs) : node(node), costs(costs) {}
	};

	typedef boost::heap::fibonacci_heap<NodeWrapper*, boost::heap::compare<CompareNode> > Queue;
	typedef boost::unordered_map<const AbstractNode *, NodeWrapper*> HashMap;
	Queue openList;
	HashMap openMap;
	bool duplicateWarning;

public:
	static FileIO *fileIO;
};

}  // namespace planning

#endif /* PATH_PLANNING_OPENLIST_HPP_ */
