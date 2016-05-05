#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <inverse_kinematics/InverseKinematics.h>
#include <angles/angles.h>
#include <fstream>

using namespace inverse_kinematics;

int main(int argc, char *argv[]) 
{
	ros::init(argc, argv, "inverse_kinematics");

	const std::string packagePath = ros::package::getPath("inverse_kinematics");
	if (packagePath.empty()) {
		std::cerr << "Error: Could not find package inverse_kinematics. Make sure that you call" << std::endl;
		std::cerr << "    source devel/setup.bash" << std::endl;
		std::cerr << "from your catkin workspace first before running this program." << std::endl;
		return 1;
	}

	double a0 = 3.0;
	double a1 = 2.0;
	double a2 = 1.0;
	double h = 0.5;
	double alpha = 0.1;
	const InverseKinematics_2Links ik2(a0, a1, h, alpha);
	EndeffectorPose goal2(2); goal2 << 3.0, 3.0;
	const double maxError = 0.01;
	const double maxAngularError = angles::from_degrees(1.0);

	JointAngles q = ik2.computeIK(goal2, maxError);
	std::cout << "Kinematic chain with 2 links: " << std::endl;
	std::cout << "Goal: (" << goal2.transpose(); std::cout << ")" << std::endl;
	std::cout << "Computed joint angles: (" << q.transpose(); std::cout << ")" << std::endl;
	std::cout << "Resulting pose: (" << ik2.forwardKinematic(q).transpose() << ")"; std::cout << ")" << std::endl;
	std::cout << std::endl;

	a0 = 1.6;
	a1 = 1.2;
	a2 = 0.7;
	h = 0.1;

	const InverseKinematics_3Links ik3(a0, a1, a2, h, alpha);
	q = JointAngles::Zero(3);
	EndeffectorPose goal3(3); goal3 << 2.5, 1.5, 0.0;
	q = ik3.computeIK(goal3, maxError, maxAngularError);
	std::cout << "Kinematic chain with 3 links: " << std::endl;
	std::cout << "Goal: (" << goal3.transpose(); std::cout << ")" << std::endl;
	std::cout << "Computed joint angles: (" << q.transpose(); std::cout << ")" << std::endl;
	std::cout << "Resulting pose: (" << ik3.forwardKinematic(q).transpose() << ")"; std::cout << ")" << std::endl;

	const std::string filename = packagePath + "/data/glass.log";
	std::ofstream ofs(filename.c_str());
	if (!ofs.good()) {
		std::cerr << "Could not open " << filename << " for writing log file." << std::endl;
		return 1;
	}
	const double rx1 = 0.1, ry1 = 0.2;
	const double rx2 = 0.1, ry2 = 0.2;
	const double tx = 2.5, ty = 0.8;
	const double k = 0.1;
	for (double angle = 0; angle <= 2.0 * M_PI; angle += 0.01 * M_PI) {
		double x, y;
		if (angle <= 0.5 * M_PI) {
			x = tx + rx1 * cos(angle);
			y = ty + ry1 * sin(angle);
		} else {
			x = tx + (rx2 + k*(angle - 0.5 * M_PI)) * cos(2.0 * M_PI - angle);
			y = ty + ry1 + ry2 + ry2 * sin(2.0 * M_PI - angle);
		}

		goal3 << x, y, 0.0;
		q = ik3.computeIK(goal3, maxError, maxAngularError);
		for (size_t i = 0; i < 3; ++i) {
			ofs << angles::normalize_angle((double) q(i)) << " ";
		}
		ofs << ik3.forwardKinematic(q).transpose();
		ofs << std::endl;
	}
	ofs.close();
	std::cout << std::endl << "Wrote joint angles for carrying glass to " << filename.c_str() << std::endl;

	return 0;
}
