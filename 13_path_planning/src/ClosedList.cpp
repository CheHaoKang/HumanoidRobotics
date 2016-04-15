#include <path_planning/ClosedList.h>
#include <path_planning/FileIO.h>
#include <boost/unordered_set.hpp>
#include <path_planning/AbstractNode.h>
#include <path_planning/FileIO.h>
#include <iostream>

namespace path_planning {

FileIO *ClosedList::fileIO = NULL;

ClosedList::ClosedList() : duplicateWarning(false) {
}

ClosedList::~ClosedList() {
}


void ClosedList::add(const AbstractNode * const node) {
	if (fileIO) {
		fileIO->logCloseNode(node);
	} else {
		std::cout << "close " << node->toString() << std::endl;
	}
	if (!duplicateWarning && list.find(node) != list.end()) {
		std::cerr << "Warning: adding node " << node->toString()
				<< " multiple times to the closed list." << std::endl;
		duplicateWarning = true;
	}
	list.insert(node);
}
bool ClosedList::contains(const AbstractNode * const node) const {
	return list.find(node) != list.end();
}

}  // namespace path_planning
