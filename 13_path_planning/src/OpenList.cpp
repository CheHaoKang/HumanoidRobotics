#include <path_planning/OpenList.h>
#include <boost/heap/fibonacci_heap.hpp>
#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>
#include <path_planning/FileIO.h>
#include <path_planning/AbstractNode.h>

namespace path_planning {

FileIO *OpenList::fileIO = NULL;
OpenList::OpenList() :
		duplicateWarning(false) {
}

OpenList::~OpenList() {
}


void OpenList::enqueue(const AbstractNode * const node, const double costs) {
	if (fileIO) {
		fileIO->logOpenNode(node);
	} else {
		std::cout << "enqueue " << node->toString() << std::endl;
	}
	NodeWrapper *wrapper = new NodeWrapper(node, costs);
	wrapper->handle = openList.push(wrapper);
	if (!duplicateWarning && openMap.find(node) != openMap.end()) {
		std::cerr << "Warning: adding node " << node->toString()
				<< " multiple times to the open list. Use updateCosts() instead for changing the costs of a node."
				<< std::endl;
		duplicateWarning = true;
	}
	openMap[node] = wrapper;
}
const AbstractNode * OpenList::removeMin() {
	NodeWrapper *wrapper = openList.top();
	const AbstractNode *node = openList.top()->node;
	openList.pop();
	openMap.erase(node);
	delete wrapper;
	return node;
}
void OpenList::updateCosts(const AbstractNode * const node,
		const double costs) {
	HashMap::iterator nodeIt = openMap.find(node);
	if (nodeIt == openMap.end()) {
		throw std::runtime_error(
				"Tried to update the costs for a node that has not been enqueued to the open list.");
	}
	NodeWrapper *wrapper = nodeIt->second;
	wrapper->costs = costs;
	openList.update(wrapper->handle, wrapper);
}

double OpenList::getCosts(const AbstractNode * const node) const {
	HashMap::const_iterator nodeIt = openMap.find(node);
	if (nodeIt == openMap.end()) {
		throw std::runtime_error(
				"Tried to get the costs for a node that has not been enqueued to the open list.");
	}
	NodeWrapper *wrapper = nodeIt->second;
	return wrapper->costs;
}

bool OpenList::isEmpty() const {
	return openList.empty();
}
bool OpenList::contains(const AbstractNode * const node) const {
	return openMap.find(node) != openMap.end();
}

}  // namespace planning
