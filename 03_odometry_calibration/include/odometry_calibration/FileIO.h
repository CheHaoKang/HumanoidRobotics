#ifndef FILEIO_H_
#define FILEIO_H_

#include <odometry_calibration/CalibrationData.h>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>

namespace odometry_calibration {

class FileIO {
public:
	FileIO() {};
	virtual ~FileIO() {};

	const std::vector<MeasurementData>& loadFromFile(const char *filename);
	void writeToFile(const char *filename, const std::vector<Odometry>& calibrated);

	std::vector<MeasurementData> measurements;
	std::vector<Odometry> uncalibrated;
};

} /* namespace odometry_calibration */

#endif /* FILEIO_H_ */
