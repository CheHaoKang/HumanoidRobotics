#include <signed_distance_function/FileIO.h>
#include <fstream>
#include <iostream>

namespace signed_distance_function {

FileIO::FileIO(const std::string& filename) : sizeX(0), sizeY(0), numLaserScans(0) {
	std::ifstream ifs(filename.c_str());
	if (!ifs.good()) {
		std::cerr << "Could not open file " << filename << " for reading." << std::endl;
		return;
	}
	ifs >> numLaserScans >> sizeX >> sizeY;
	Eigen::Vector2d laserPoint;
	while (ifs.good()) {
		Measurement measurement;
		measurement.laserPoints.reserve(numLaserScans);
		ifs >> measurement.robotPose(0) >> measurement.robotPose(1);
		for (size_t i = 0; i < numLaserScans; ++i) {
			ifs >> laserPoint(0) >> laserPoint(1);
			if (laserPoint(0) < 0 || laserPoint(1) < 0 || laserPoint(0) > sizeX || laserPoint(1) > sizeY) {
				std::cerr << "Point (" << laserPoint(0) << ", " << laserPoint(1) << ") is out of range. ";
				std::cout << std::endl;
			}
			measurement.laserPoints.push_back(laserPoint);
		}
		measurements.push_back(measurement);
	}
	ifs.close();
	std::cout << "Loaded " << measurements.size() << " measurements from " << filename << std::endl;
}

void FileIO::writeMap(const Eigen::MatrixXd& map, const std::string& filename) {
	std::ofstream ofs(filename.c_str());
	if (!ofs.good()) {
		std::cerr << "Could not open file " << filename << " for writing" << std::endl;
		return;
	}

	size_t numCols = map.cols();
	size_t numRows = map.rows();

	for (size_t i = 0; i < numRows; ++i) {
		for (size_t j = 0; j < numCols; ++j) {
			ofs << map(i, j) << " ";
		}
		ofs << std::endl;
	}
	ofs.close();
	std::cout << "Wrote a " << map.rows() << " x " << map.cols() << " map to " << filename << std::endl;
}

}  // namespace signed_distance_function
