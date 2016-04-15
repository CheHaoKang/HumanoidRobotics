#include <forward_kinematics/RosIO.h>

namespace forward_kinematics {

RosIO::RosIO(ForwardKinematics *fk_) : fk(fk_) {
	jointSub = nh.subscribe("/joint_states", 10, &RosIO::jointCallback, this);
	visualizationPub = nh.advertise<visualization_msgs::MarkerArray>("/hand_pose", 10, false);
}

RosIO::~RosIO() {
}

void RosIO::jointCallback(const sensor_msgs::JointStateConstPtr& state) {
	double encoderReadings[5];
	for (size_t i = 0; i < state->name.size(); ++i) {
		if (state->name[i].compare("LShoulderRoll") == 0) {
			encoderReadings[ForwardKinematics::LShoulderRoll] = state->position[i];
		} else if(state->name[i].compare("LShoulderPitch") == 0) {
			encoderReadings[ForwardKinematics::LShoulderPitch] = state->position[i];
		} else if(state->name[i].compare("LElbowRoll") == 0) {
			encoderReadings[ForwardKinematics::LElbowRoll] = state->position[i];
		} else if(state->name[i].compare("LElbowYaw") == 0) {
			encoderReadings[ForwardKinematics::LElbowYaw] = state->position[i];
		} else if(state->name[i].compare("LWristYaw") == 0) {
			encoderReadings[ForwardKinematics::LWristYaw] = state->position[i];
		}
	}

	Eigen::Matrix4d t = fk->computeHandTransform(encoderReadings);
	const double scale = 0.2;
	Eigen::Vector4d hand[4];
	hand[0] << 0, 0, 0, 1;
	hand[1] << 1, 0, 0, 1 / scale;
	hand[2] << 0, 1, 0, 1 / scale;
	hand[3] << 0, 0, 1, 1 / scale;

	Eigen::Vector4d base[4];
	geometry_msgs::Point msg[4];
	for (size_t i = 0; i < 4; ++i) {
		base[i] = t * hand[i];
		msg[i].x = base[i][0] / base[i][3];
		msg[i].y = base[i][1] / base[i][3];
		msg[i].z = base[i][2] / base[i][3];
	}

	visualization_msgs::Marker xAxis, yAxis, zAxis;
	xAxis.action = visualization_msgs::Marker::ADD;
	xAxis.header.frame_id = state->header.frame_id;
	xAxis.header.stamp = state->header.stamp;
	xAxis.type = visualization_msgs::Marker::ARROW;
	xAxis.ns = "hand";
	xAxis.pose.orientation.w = 1.0;
	xAxis.scale.x = 0.01; // shaft diameter
	xAxis.scale.y = 0.05; // head diameter
	xAxis.scale.z = 0.1;  // head length

	yAxis = xAxis;
	zAxis = xAxis;

	xAxis.color.r = 1.0;
	xAxis.color.g = 0.0;
	xAxis.color.b = 0.0;
	xAxis.color.a = 1.0;
	xAxis.id = 2;
	xAxis.points.push_back(msg[0]);
	xAxis.points.push_back(msg[1]);

	yAxis.color.r = 0.0;
	yAxis.color.g = 1.0;
	yAxis.color.b = 0.0;
	yAxis.color.a = 1.0;
	yAxis.id = 3;
	yAxis.points.push_back(msg[0]);
	yAxis.points.push_back(msg[2]);

	zAxis.color.r = 0.0;
	zAxis.color.g = 0.0;
	zAxis.color.b = 1.0;
	zAxis.color.a = 1.0;
	zAxis.id = 4;
	zAxis.points.push_back(msg[0]);
	zAxis.points.push_back(msg[3]);

	visualization_msgs::MarkerArray markerArray;
	markerArray.markers.push_back(xAxis);
	markerArray.markers.push_back(yAxis);
	markerArray.markers.push_back(zAxis);
	visualizationPub.publish(markerArray);
}

} /* namespace forward_kinematics */
