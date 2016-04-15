#include <forward_kinematics/ForwardKinematics.h>
#include <iostream>
#include <cmath>

namespace forward_kinematics {

/**
 * \brief Class for computing the forward kinematics of Nao's left arm.
 *
 * The constructor already stores the Denavit-Hartenberg parameters for
 * each joint. You can use these values in computeHandTransform().
 */
ForwardKinematics::ForwardKinematics() {
	dh[LShoulderPitch].d     = 0.0;
	dh[LShoulderPitch].a     = 0.0;
	dh[LShoulderPitch].theta = 0.0;
	dh[LShoulderPitch].alpha = M_PI / 2.0;

	dh[LShoulderRoll].d     = 0.0;
	dh[LShoulderRoll].a     = 0.0;
	dh[LShoulderRoll].theta = M_PI / 2.0;
	dh[LShoulderRoll].alpha = M_PI / 2.0;

	dh[LElbowYaw].d     = 0.0900;
	dh[LElbowYaw].a     = 0.0;
	dh[LElbowYaw].theta = 0.0;
	dh[LElbowYaw].alpha = -M_PI / 2.0;

	dh[LElbowRoll].d     = 0.0;
	dh[LElbowRoll].a     = 0.0;
	dh[LElbowRoll].theta = 0.0;
	dh[LElbowRoll].alpha = M_PI / 2.0;

	dh[LWristYaw].d     = 0.05055;
	dh[LWristYaw].a     = 0.0;
	dh[LWristYaw].theta = 0.0;
	dh[LWristYaw].alpha = M_PI / 2.0;
}

/**
 * \brief Homogeneous matrix representing a rotation around the X axis.
 * \param[in] angle The angle of the rotation in radians.
 * \return The homogenous matrix representing the rotation.
 */
Eigen::Matrix4d ForwardKinematics::rotationX(const double& angle) {
	Eigen::Matrix4d M;
	//TODO: write the rotation matrix
	M << 	1, 0, 0, 0,
			0, cos(angle), -sin(angle), 0,
			0, sin(angle), cos(angle), 0,
			0, 0, 0, 1;

	return M;
}

/**
 * \brief Homogeneous matrix representing a rotation around the Z axis.
 * \param[in] angle The angle of the rotation in radians.
 * \return The homogenous matrix representing the rotation.
 */
Eigen::Matrix4d ForwardKinematics::rotationZ(const double& angle) {
	Eigen::Matrix4d M;
	//TODO: write the rotation matrix
	M << 	cos(angle), -sin(angle), 0, 0,
			sin(angle), cos(angle), 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1;

	return M;
}

/**
 * \brief Homogenous matrix representing a translation along the X axis.
 * \param[in] distance The distance of the translation in meters.
 * \return The homogeneous matrix representing the translation.
 */
Eigen::Matrix4d ForwardKinematics::translationX(const double& distance) {
	Eigen::Matrix4d M;
	M << 	1, 0, 0, distance,
			0, 1, 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1;

	return M;
}

/**
 * \brief Homogenous matrix representing a translation along the Z axis.
 * \param[in] distance The distance of the translation in meters.
 * \return The homogeneous matrix representing the translation.
 */
Eigen::Matrix4d ForwardKinematics::translationZ(const double& distance) {
	Eigen::Matrix4d M;
	M << 	1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, distance,
			0, 0, 0, 1;

	return M;
}

/**
 * \brief The transformation of a single joint, composed of two screw displacements.
 * \param[in] dh The Denavit-Hartenberg parameters of the joint.
 * \param[in] The current joint angle in radians, measured by a magnetic rotary encoder.
 * \return The homogeneous transformation matrix of the joint.
 */
Eigen::Matrix4d ForwardKinematics::getA(const DH& dh, const double& encoderReading) {
	Eigen::Matrix4d A;
	//TODO: Calculate the transformation for a single joint.
	A = rotationZ(dh.theta+encoderReading)*translationZ(dh.d)*rotationX(dh.alpha)*translationX(dh.a);

	return A;
}

/**
 * \brief Compute the transformation of the hand coordinate system to the robot's
 * body coordinate system located in the torso.
 * \param[in] encoderReading Array of the joint encoder readings of the 5 joints.
 * \return The homogeneous matrix representing the transformation from the hand
 * coordinate system to the body coordinate system.
 */
Eigen::Matrix4d ForwardKinematics::computeHandTransform(const double encoderReading[5]) {
	Eigen::Matrix4d M;
	Eigen::Matrix4d T_Bshoulder;
	Eigen::MatrixXd T_LWYE(4, 4);
	//TODO Calculate the whole transformation from the hand to the body.

	T_Bshoulder << 	1, 0, 0, 0,
					0, 0, 1, 0.098,
					0, -1, 0, 0.100,
					0, 0, 0, 1;

	T_LWYE << 	0, 1, 0, 0.0159,
				1, 0, 0, 0.0580,
				0, 0, -1, 0,
				0, 0, 0, 1;

	M = T_Bshoulder*getA(dh[0], encoderReading[0])*getA(dh[1], encoderReading[1])*getA(dh[2], encoderReading[2])*getA(dh[3], encoderReading[3])*getA(dh[4], encoderReading[4])*T_LWYE;

	return M;
}


}  // namespace forward_kinematics
