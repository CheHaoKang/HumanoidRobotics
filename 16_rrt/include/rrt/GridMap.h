#ifndef RRT_GRIDMAP_H_
#define RRT_GRIDMAP_H_

#include <stdexcept>
#include <vector>

namespace rrt {

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

	GridMap(const size_t& width, const size_t& height, std::vector<bool>& data) :
			width(width), height(height), data(data) {
	}

private:
	std::vector<bool> data;
};

}  // namespace rrt

#endif /* RRT_GRIDMAP_H_ */
