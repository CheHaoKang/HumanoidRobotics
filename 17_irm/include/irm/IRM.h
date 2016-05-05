#ifndef IRM_H_
#define IRM_H_

#include <irm/AbstractIRM.h>
#include <Eigen/Dense>

namespace irm {

class IRM : public AbstractIRM {
public:
	IRM() {};
	virtual ~IRM() {};
   	virtual Eigen::Vector3d sampleConfiguration() const;
   	virtual void computeRM(const size_t& numSamples);
   	virtual double computeManipulability(const Eigen::Vector3d& jointAngles, const Eigen::Vector3d& endeffectorPose) const;
protected:
   	virtual void computeIRM(const std::vector<RMVoxel>& voxels);


};

}  // namespace irm

#endif  // IRM_H_
