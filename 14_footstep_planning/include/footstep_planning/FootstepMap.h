#ifndef PATH_PLANNING_FOOTSTEPMAP_H_
#define PATH_PLANNING_FOOTSTEPMAP_H_

#include <vector>
#include <limits>
#include <path_planning/GridMap.h>
#include <angles/angles.h>

namespace footstep_planning {

struct FootstepMap : path_planning::GridMap {
public:
	FootstepMap(const size_t& width, const size_t& height, const std::vector<bool>& data,
			const std::vector<std::vector<unsigned char> >& distanceMaps);
	virtual ~FootstepMap();

	double getDistanceToNearestObstacle(const double& x, const double& y, const double& theta) const;

private:
	const std::vector<std::vector<unsigned char> > distanceMaps;
	const double resolution;
	const double scale;
};

}  // namespace footstep_planning

#endif /* PATH_PLANNING_FOOTSTEPMAP_H_ */
