#ifndef CALIBRATIONDATA_H_
#define CALIBRATIONDATA_H_

namespace odometry_calibration {

struct Pose2D {
	double x;
	double y;
	double theta;
};

struct Odometry {
	double ux;
	double uy;
	double utheta;
};

struct MeasurementData {
	Odometry groundTruth;
	Odometry uncalibrated;
};

}  // namespace odometry_calibration

#endif /* CALIBRATIONDATA_H_ */
