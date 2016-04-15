#ifndef FORWARD_KINEMATICS_FILEIO_H_
#define FORWARD_KINEMATICS_FILEIO_H_

#include <Eigen/StdVector>
#include <signed_distance_function/SignedDistanceFunction.h>

namespace signed_distance_function {

class FileIO {
public:
	typedef std::vector<Measurement, Eigen::aligned_allocator<Measurement> > MeasurementsVector;

	size_t numLaserScans;
	size_t sizeX;
	size_t sizeY;
	MeasurementsVector measurements;

	FileIO(const std::string& filename);
	virtual ~FileIO() {};

	void writeMap(const Eigen::MatrixXd& map, const std::string& filename);
};

}  // namespace signed_distance_function

#endif  //FORWARD_KINEMATICS_FILEIO_H_
