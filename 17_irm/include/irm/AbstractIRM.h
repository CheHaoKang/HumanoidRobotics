#ifndef INCLUDE_IRM_ABSTRACTIRM_H_
#define INCLUDE_IRM_ABSTRACTIRM_H_

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <boost/unordered_map.hpp>
#include <Eigen/Dense>

namespace irm {

class AbstractIRM {
public:
	struct JointConfiguration {
		Eigen::Vector3d jointAngles;
		double manipulability;
	};
	struct RMEntry {
		Eigen::Vector3d endeffectorPose;
		Eigen::Vector3d jointAngles;
		double manipulability;
		std::vector<JointConfiguration> configurations;
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
		RMEntry(Eigen::Vector3d pose) : endeffectorPose(pose), manipulability(std::numeric_limits<double>::max()) {};
	};

	struct MapConfig {
		double min, max, res;
		size_t numCells;
	};
	typedef boost::unordered_map<size_t, RMEntry*> RMMapType;
	struct RMVoxel {
		std::vector<JointConfiguration> configurations;
	};


	struct IRMEntry {
		Eigen::Vector3d basePose;
		Eigen::Vector3d jointAngles;
		double manipulability;
		std::vector<JointConfiguration> configurations;
		IRMEntry(Eigen::Vector3d pose) : basePose(pose), manipulability(std::numeric_limits<double>::max()) {};
	};
	typedef boost::unordered_map<size_t, IRMEntry*> IRMMapType;

private:
   	KDL::Chain * chain;
	KDL::ChainFkSolverPos_recursive * fksolver1;
	KDL::ChainIkSolverVel_pinv * iksolver1v;
	KDL::ChainIkSolverPos_NR * iksolver1;

	std::vector<MapConfig> rmMapConfig, irmMapConfig;
	RMMapType rm, rm2d;
	IRMMapType irm2d;

public:
	AbstractIRM();
   	virtual ~AbstractIRM();
	static const size_t INVALID = -1;
	const RMMapType& getReachabilityMap() const { return rm; }
	const RMMapType& get2DReachabilityMap() const { return rm2d; }
	const IRMMapType& get2DInverseReachabilityMap() const { return irm2d; }
	const std::vector<MapConfig>& getRMMapConfig() const { return rmMapConfig; }
	const std::vector<MapConfig>& getIRMMapConfig() const { return irmMapConfig; }
   	void doComputeIRM();
   	bool forwardKinematics(const Eigen::Vector3d& jointAngles, Eigen::Vector3d& endeffectorPose) const;
   	bool inverseKinematics(const Eigen::Vector3d& endeffectorPose, Eigen::Vector3d& jointAngles) const;

protected:
   	virtual void computeIRM(const std::vector<RMVoxel>& voxels) = 0;
   	virtual void addToRM(const Eigen::Vector3d& jointAngles, const Eigen::Vector3d& endeffectorPose, const double& manipulability);
   	virtual void addToIRM(const Eigen::Vector3d& basePose, const Eigen::Vector3d& jointAngles, const double& manipulability);

private:
   	size_t getIndex(const std::vector<MapConfig>& mapConfig, const Eigen::Vector3d& pose, const bool& use2d) const;
   	friend class FileIO;
   	friend class IRM_computeManipulability_Test;
};

}




#endif /* INCLUDE_IRM_ABSTRACTIRM_H_ */
