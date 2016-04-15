#include <odometry_calibration/FileIO.h>
#include <odometry_calibration/OdometryCalibration.h>
#include <odometry_calibration/CalibrationData.h>
#include <iostream>
#include <stdexcept>

using namespace odometry_calibration;

int main(int argc, char *argv[]) {
	if (argc != 3) {
		std::cerr << "Usage: " << argv[0] << " inputfile outputfile" << std::endl;
		return 1;
	}
	FileIO fileIO;
	try {
		fileIO.loadFromFile(argv[1]);
	} catch(const std::runtime_error& e) {
		std::cerr << e.what() << std::endl;
		return 2;
	}

	std::cout << "Calibrating..." << std::endl;
	Eigen::Matrix3d calibrationMatrix = OdometryCalibration::calibrateOdometry(fileIO.measurements);
	std::cout << "Calibration finished. The calibration matrix is: " << std::endl;
	std::cout << calibrationMatrix << std::endl;
	std::vector<Odometry> calibratedOdometry;
	calibratedOdometry.reserve(fileIO.uncalibrated.size());
	for (std::vector<Odometry>::const_iterator it = fileIO.uncalibrated.begin(); it != fileIO.uncalibrated.end(); ++it) {
		calibratedOdometry.push_back(OdometryCalibration::applyOdometryCorrection(*it, calibrationMatrix));
	}
	try {
		fileIO.writeToFile(argv[2], calibratedOdometry);
	} catch(const std::runtime_error& e) {
		std::cerr << e.what() << std::endl;
		return 3;
	}

	return 0;

}
