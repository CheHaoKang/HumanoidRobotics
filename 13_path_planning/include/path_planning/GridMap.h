#ifndef PATH_PLANNING_GRIDMAP_H_
#define PATH_PLANNING_GRIDMAP_H_

#include <stdexcept>
#include <vector>

namespace path_planning {

struct GridMap {
public:
	const size_t width;
	const size_t height;
	bool isOccupied(const int& x, const int& y) const {
		if (x < 0 || x >= width || y < 0 || y >= width) {
			throw std::runtime_error("Index out of bounds in call to isOccupied()");
		}
		return data[y * width + x];
	}

	GridMap(const size_t& width, const size_t& height, const std::vector<bool>& data) : width(width), height(height), data(data) {};

private:
	const std::vector<bool> data;
};

}  // namespace path_planning

#endif /* PATH_PLANNING_GRIDMAP_H_ */
