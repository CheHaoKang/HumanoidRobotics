#ifndef FORWARD_KINEMATICS_FILEIO_H_
#define FORWARD_KINEMATICS_FILEIO_H_

#include <string>
#include <vector>
#include <Eigen/Core>
#include <Eigen/StdVector>

namespace forward_kinematics {

class FileIO {
public:
	FileIO(const std::string& filename);
	virtual ~FileIO() {};

	std::vector<std::vector<double> > jointAngles;
	std::vector<std::vector<double> > groundTruth;
	typedef std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > VectorOfTransforms;
	VectorOfTransforms results;

	bool writeToFile(const std::string &filename);
};


}  // namespace forward_kinematics


#endif  // FORWARD_KINEMATICS_FILEIO_H_
