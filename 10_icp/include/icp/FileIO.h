#ifndef FORWARD_KINEMATICS_FILEIO_H_
#define FORWARD_KINEMATICS_FILEIO_H_

#include <Eigen/StdVector>
#include <icp/ICP.h>

namespace icp {

class FileIO {
public:
	typedef std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > StdVectorOfVector2d;
	FileIO(const std::string& filename);
	virtual ~FileIO() {};
	StdVectorOfVector2d  Q;
	StdVectorOfVector2d  P;

	void writeMap(const StdVectorOfVector2d & P, const std::string& filename);
};

}  // namespace icp

#endif  //FORWARD_KINEMATICS_FILEIO_H_
