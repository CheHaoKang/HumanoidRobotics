#ifndef PATH_PLANNING_CLOSEDLIST_HPP_
#define PATH_PLANNING_CLOSEDLIST_HPP_

#include <boost/unordered_set.hpp>
#include <path_planning/AbstractNode.h>
#include <path_planning/FileIO.h>
#include <iostream>

namespace path_planning {

class ClosedList {
public:
	ClosedList();
	virtual ~ClosedList();

	void add(const AbstractNode * const node);
	bool contains(const AbstractNode * const node) const;

private:
	boost::unordered_set<const AbstractNode *> list;
	bool duplicateWarning;

public:
	static FileIO *fileIO;
};

}  // namespace path_planning

#endif /* PATH_PLANNING_CLOSEDLIST_HPP_ */
