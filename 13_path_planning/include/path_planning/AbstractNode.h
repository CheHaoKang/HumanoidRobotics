#ifndef PATH_PLANNING_ABSTRACTNODE_H_
#define PATH_PLANNING_ABSTRACTNODE_H_

#include <stdexcept>
#include <string>

namespace path_planning {

class AbstractNode {
public:
	double costs;

   	void setPredecessor(const AbstractNode *predecessor) {
   		if (predecessor == this) {
   			throw std::runtime_error("Setting node as its own predecessor creates an infinite loop");
   		}
   		this->predecessor = predecessor;
   	}
   	const AbstractNode* getPredecessor() const {
   		return this->predecessor;
   	}

   	virtual std::string toString() const = 0;
   	virtual std::string toLogString() const = 0;

protected:
	const AbstractNode *predecessor;
	AbstractNode() : predecessor(NULL), costs(0.0) {};
	virtual ~AbstractNode() {};

};

}  // namespace path_planning

#endif /* PATH_PLANNING_ABSTRACTNODE_H_ */
