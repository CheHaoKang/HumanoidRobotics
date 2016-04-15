#ifndef ICP_H_
#define ICP_H_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>

namespace icp{

typedef std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > StdVectorOfVector2d;

class ICP 
{
public:
	ICP() {};
   	virtual ~ICP() {};

	static double distance(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2);
	static Eigen::Vector2d closestPointOnLine(const Eigen::Vector2d& pX, const Eigen::Vector2d& pL1, const Eigen::Vector2d& pL2);
	static double min(const std::vector<double>& dist);
	static StdVectorOfVector2d euclideanCorrespondences(const StdVectorOfVector2d& Q, const StdVectorOfVector2d& P);
	static StdVectorOfVector2d closestPointToLineCorrespondences(const StdVectorOfVector2d& Q, const StdVectorOfVector2d& P);
	static Eigen::Matrix3d calculateAffineTransformation(const StdVectorOfVector2d& Q, const StdVectorOfVector2d& C);
	static StdVectorOfVector2d applyTransformation(const Eigen::Matrix3d& A, const StdVectorOfVector2d& P);
	static double computeError(const StdVectorOfVector2d& Q, const StdVectorOfVector2d& C, const Eigen::Matrix3d& A);
	static StdVectorOfVector2d iterateOnce(const StdVectorOfVector2d& Q, const StdVectorOfVector2d& P,int & conversionFlag, const bool pointToLineFlag,double threshold);
};
}  // namespace icp_namespace

#endif  // ICP_H_
