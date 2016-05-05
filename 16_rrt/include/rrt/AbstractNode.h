#ifndef RRT_ABSTRACTNODE_H_
#define RRT_ABSTRACTNODE_H_

#include <stdexcept>
#include <string>

namespace rrt {

class AbstractNode {
public:
	double costs;
	void setPredecessor(AbstractNode * predecessor) {
		if (predecessor == this) {
			throw std::runtime_error("Setting node as its own predecessor creates an infinite loop");
		}
		this->predecessor = predecessor;
	}
	AbstractNode * getPredecessor() const {
		return this->predecessor;
	}

	void setConnection(AbstractNode * connection) {
		if (connection == this) {
			throw std::runtime_error("Setting node as its own predecessor creates an infinite loop");
		}
		this->connection = connection;
	}
	AbstractNode * getConnection() const {
		return this->connection;
	}

	virtual std::string toString() const = 0;

	virtual std::string toLogString() const = 0;

	bool predecessorExists() {
		return predecessor != NULL;
	}

protected:
	AbstractNode * predecessor;
	AbstractNode * connection;
	AbstractNode() :
			predecessor(NULL), connection(NULL), costs(0.0) {
	}

	virtual ~AbstractNode() {
	}

};

}  // namespace rrt

#endif /* RRT_ABSTRACTNODE_H_ */
