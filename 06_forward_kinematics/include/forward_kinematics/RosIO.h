#ifndef FORWARD_KINEMATICS_ROSIO_H_
#define FORWARD_KINEMATICS_ROSIO_H_

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <forward_kinematics/ForwardKinematics.h>
#include <visualization_msgs/MarkerArray.h>

namespace forward_kinematics {

class RosIO {
public:
	RosIO(ForwardKinematics *fk);
	virtual ~RosIO();

protected:
	ros::NodeHandle nh;
	ros::Subscriber jointSub;
	ros::Publisher visualizationPub;
	ForwardKinematics *fk;

	void jointCallback(const sensor_msgs::JointStateConstPtr& state);
};

} /* namespace forward_kinematics */

#endif /* FORWARD_KINEMATICS_ROSIO_H_ */
