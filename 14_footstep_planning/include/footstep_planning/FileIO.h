#ifndef FOOTSTEP_PLANNING_FILEIO_H_
#define FOOTSTEP_PLANNING_FILEIO_H_

#include <footstep_planning/FootstepMap.h>
#include <string>

namespace footstep_planning {

class FileIO {
public:
	FileIO(const std::string& package_path);
	virtual ~FileIO();

	const FootstepMap *map;
};

} /* namespace footstep_planning */

#endif /* FOOTSTEP_PLANNING_FILEIO_H_ */
