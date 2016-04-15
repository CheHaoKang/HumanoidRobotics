#include <odometry_calibration/FileIO.h>
#include <odometry_calibration/OdometryCalibration.h>
#include <fstream>
#include <stdexcept>
#include <iostream>

namespace odometry_calibration {

const std::vector<MeasurementData>&  FileIO::loadFromFile(const char *filename) {
	measurements.clear();
	uncalibrated.clear();
	std::ifstream ifs(filename);
	if (!ifs.is_open()) {
		throw std::runtime_error("Could not open input file");
	}
	if (ifs.good()) {
		// skip the first line (contains comment only)
		std::string line;
		std::getline(ifs, line);
	}

	while(ifs.good()) {
		MeasurementData data;
		ifs >> data.groundTruth.ux >> data.groundTruth.uy >> data.groundTruth.utheta;
		ifs >> data.uncalibrated.ux >> data.uncalibrated.uy >> data.uncalibrated.utheta;
		measurements.push_back(data);
		uncalibrated.push_back(data.uncalibrated);
	}

	// !!! To prevent read the last data twice !!!
	//measurements.pop_back();
	//uncalibrated.pop_back();

	ifs.close();
	std::cout << "Loaded " << measurements.size() << " odometry measurements from " << filename << std::endl;
	return measurements;
}

void FileIO::writeToFile(const char *filename, const std::vector<Odometry>& calibrated) {
	std::ofstream ofs(filename);
	if (!ofs.is_open()) {
		throw std::runtime_error("Could not open output file");
	}
	ofs << "# x y theta" << std::endl;
	std::vector<Pose2D> poses = OdometryCalibration::calculateTrajectory(calibrated);
	for (std::vector<Pose2D>::const_iterator it = poses.begin(); it != poses.end(); ++it) {
		ofs << it->x << " " << it->y << " " << it->theta << std::endl;
	}
	ofs.close();
	std::cout << "Wrote " << calibrated.size() << " trajectory points to " << filename << std::endl;
}


} /* namespace odometry_calibration */
