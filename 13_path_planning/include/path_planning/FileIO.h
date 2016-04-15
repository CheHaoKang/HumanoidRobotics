#ifndef PATH_PLANNING_FILEIO_H_
#define PATH_PLANNING_FILEIO_H_

#include <fstream>
#include <deque>
#include <path_planning/AbstractNode.h>
#include <path_planning/GridNode.h>
#include <path_planning/GridMap.h>

namespace path_planning {

class FileIO {
public:
	FileIO();
	virtual ~FileIO();
	bool openLogfile(const std::string& filename);
	void logPath(const std::string& filename, std::deque<const AbstractNode *> path);
	void logOpenNode(const AbstractNode * const node);
	void logCloseNode(const AbstractNode * const node);
	static GridMap* loadMap(const std::string& filename);

private:
	std::ofstream logfile;
};

} /* namespace path_planning */

#endif /* PATH_PLANNING_FILEIO_H_ */
