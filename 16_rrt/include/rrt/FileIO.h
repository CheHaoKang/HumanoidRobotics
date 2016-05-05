#ifndef RRT_FILEIO_H_
#define RRT_FILEIO_H_

#include <fstream>
#include <deque>
#include <rrt/AbstractNode.h>
#include <rrt/GridNode.h>
#include <rrt/GridMap.h>

namespace rrt {

class FileIO {
public:
        FileIO();
        virtual ~FileIO();
        bool openLogfile(const std::string& filename);
        void logPath(const std::string& filename, const std::deque<AbstractNode *>& path);
        void logExtend(const GridNode * const from, const GridNode * const to, const GridNode * const randomNode, const std::string * const listName);
        void logFailedExtend(const GridNode * const from, const GridNode * const randomNode, const std::string * const listName);
        static GridMap* loadMap(const std::string& filename);

private:
        std::ofstream logfile;
};

} /* namespace rrt */

#endif /* RRT_FILEIO_H_ */
