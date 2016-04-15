#ifndef FORWARD_KINEMATICS_H_
#define FORWARD_KINEMATICS_H_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <map>

namespace forward_kinematics {

class ForwardKinematics {
public:
	enum Joint {
		LShoulderPitch = 0,
		LShoulderRoll = 1,
		LElbowYaw = 2,
		LElbowRoll = 3,
		LWristYaw = 4
	};

	struct DH {
		double d;
		double a;
		double theta;
		double alpha;
	} dh[5];

	ForwardKinematics();
	virtual ~ForwardKinematics() {};
	static Eigen::Matrix4d rotationX(const double& angle);
	static Eigen::Matrix4d rotationZ(const double& angle);
	static Eigen::Matrix4d translationX(const double& distance);
	static Eigen::Matrix4d translationZ(const double& distance);
	static Eigen::Matrix4d getA(const DH& dh, const double& encoderReading);

	Eigen::Matrix4d computeHandTransform(const double encoderReading[5]);
};


}  // namespace forward_kinematics


#endif  // FORWARD_KINEMATICS_H_
