#include <gtest/gtest.h>
#include <inverse_kinematics/InverseKinematics.h>
#include <angles/angles.h>
#include <iostream>
#include <iomanip>

using namespace inverse_kinematics;

TEST(InverseKinematics, 2Links_forwardKinematic) {
	const double tolerance = 1e-3;
	InverseKinematics_2Links ik(4, 2, 1, 0.1);
	EndeffectorPose e1 = ik.forwardKinematic(Eigen::Vector2d(0.0, 0.0));
	EndeffectorPose e2 = ik.forwardKinematic(Eigen::Vector2d(0.0, M_PI/2));
	EndeffectorPose e3 = ik.forwardKinematic(Eigen::Vector2d(M_PI/2, 0));
	EndeffectorPose e4 = ik.forwardKinematic(Eigen::Vector2d(M_PI/2, M_PI/2));
	if (e1.isZero() && e2.isZero() && e3.isZero()) {
		FAIL() << "The method does not do anything.";
	}
	if(!e1.isApprox(Eigen::Vector2d( 7.0,  0.0), tolerance) ||
	   !e2.isApprox(Eigen::Vector2d( 4.0,  3.0), tolerance) ||
	   !e3.isApprox(Eigen::Vector2d( 0.0,  7.0), tolerance) ||
	   !e4.isApprox(Eigen::Vector2d(-3.0,  4.0), tolerance)) {
		FAIL() << "The endeffector position is wrong.";
	}
}

TEST(InverseKinematics, 2Links_jacobian) {
	const double tolerance = 1e-3;
	InverseKinematics_2Links ik(4, 2, 1, 0.1);
	Jacobian jacobian[] = {
			ik.jacobian(Eigen::Vector2d(0.0, 0.0)),
			ik.jacobian(Eigen::Vector2d(M_PI, 0.0)),
			ik.jacobian(Eigen::Vector2d(0.0, M_PI)),
			ik.jacobian(Eigen::Vector2d(M_PI, M_PI)),
			ik.jacobian(Eigen::Vector2d(M_PI/2, M_PI/2)),
			ik.jacobian(Eigen::Vector2d(-M_PI/2, M_PI/2)),
			ik.jacobian(Eigen::Vector2d(M_PI/2, -M_PI/2)),
			ik.jacobian(Eigen::Vector2d(-M_PI/2, -M_PI/2)),
	};
	Eigen::Matrix2d trueJacobian[8];
	trueJacobian[0] << 0, 0, 7, 3;
	trueJacobian[1] << 0, 0, -7, -3;
	trueJacobian[2] << 0, 0, 1, -3;
	trueJacobian[3] << 0, 0, -1, 3;
	trueJacobian[4] << -4, 0, -3, -3;
	trueJacobian[5] << 4, 0, 3, 3;
	trueJacobian[6] << -4, 0, 3, 3;
	trueJacobian[7] << 4, 0, -3, -3;

	size_t correct[2][2] = {{0, 0}, {0, 0}};
	size_t numZero = 0;
	size_t numAllCorrect = 0;
	for (size_t i = 0; i < 8; ++i) {
		if (jacobian[i].isZero()) {
			++numZero;
		}
		bool incorrect = false;
		for (size_t j = 0; j < 2; ++j) {
			for (size_t k = 0; k < 2; ++k) {
				if (fabs((double) (jacobian[i](j, k) - trueJacobian[i](j, k))) < tolerance) {
					++correct[j][k];
				} else {
					incorrect = true;
				}
			}
		}
		if (!incorrect) {
			++numAllCorrect;
		}
	}

	if (numZero == 8) {
		FAIL() << "The method does not do anything.";
	}
	if (numAllCorrect == 8) {
		return;
	}

	std::string labels[] = {"top left", "top right", "bottom left", "bottom right"};
	std::vector<size_t> correctIdx;
	std::vector<size_t> wrongIdx;
	for (size_t j = 0; j < 2; ++j) {
		for (size_t k = 0; k < 2; ++k) {
			if (correct[j][k] == 8) {
				correctIdx.push_back(2 * j + k);
			} else {
				wrongIdx.push_back(2 * j + k);
			}
		}
	}
	switch (correctIdx.size()) {
	case 0:
		FAIL() << "All components of the Jacobian are wrong.";
		break;
	case 1:
		FAIL() << "The " << labels[correctIdx[0]] << " component is correct, all others are wrong.";
		break;
	case 2:
		FAIL() << "The " << labels[wrongIdx[0]] << " and the " << labels[wrongIdx[1]] << " components are wrong.";
		break;
	case 3:
		FAIL() << "The " << labels[wrongIdx[0]] << " component is wrong.";
		break;
	}
}

TEST(InverseKinematics, chooseStep) {
	const InverseKinematics_2Links ik(4, 2, 1, 0.1);
	EndeffectorPose e1(2), e2(2), g(2), e4(4), g4(4);
	e1 << 2, 3;
	e2 << 5, 5;
	g << 4, 7;
	e4 << 1, 2, 3, 1;
	g4 << 4, 3, -2, -7;
	const EndeffectorPose r1 = ik.chooseStep(e1, g);
	const EndeffectorPose r2 = ik.chooseStep(e2, g);
	const EndeffectorPose r3 = ik.chooseStep(g, g);
	const EndeffectorPose r4 = ik.chooseStep(e4, g4);
	if (r1.isZero() && r2.isZero() && r3.isZero()) {
		FAIL() << "The method does not do anything.";
	}
	if (r1.isApprox(Eigen::Vector2d(2.0, 4.0))) {
		FAIL() << "The method must scale the difference with the constant factor alpha (given as a class member variable), otherwise the iteration is likely to overshoot the target value.";
	}
	if (r1(0) < 0 && r1(1) < 0 && r2(0) > 0 && r2(1) < 0) {
		FAIL() << "The sign of the result is wrong.";
	}
	if (!r1.isApprox(Eigen::Vector2d(0.2, 0.4)) || !r2.isApprox(Eigen::Vector2d(-0.1, 0.2))) {
		FAIL() << "The result vector is wrong.";
	}
	if (r4.size() != 4 || !r4.isApprox(Eigen::Vector4d(.3, .1, -.5, -.8))) {
		FAIL() << "The method works for vectors of length 2, but it must also work for vectors of arbitrary length as it will be used with more joints in the following exercise parts.";
	}
}

TEST(InverseKinematics, computeJointChange) {
	const double tolerance = 1e-3;
	const InverseKinematics_2Links ik(4, 2, 1, 0.1);
	Jacobian j(2, 2);
	j << 1, 2, 3, 4;
	Jacobian j3(3, 3);
	j3 << 1, 2, 3, 4, 3, 2, 2, 7, -1;
	JointAngles r1 = ik.computeJointChange(Eigen::Vector2d(5.0, 7.0), j);
	JointAngles r2 = ik.computeJointChange(Eigen::Vector2d(5.0, 7.0), Jacobian::Identity(2, 2));
	JointAngles r3 = ik.computeJointChange(Eigen::Vector3d(5.0, 7.0, 2.0), j3);
	JointAngles r3t = Eigen::Vector3d(1.01538, 0.169231, 1.21538);
	if (r1.isZero()) {
		FAIL() << "The method does not do anything.";
	}
	if (r1.isApprox(Eigen::Vector2d(19., 43.))) {
		FAIL() << "The method uses the Jacobian instead of its inverse.";
	}
	if (!r1.isApprox(Eigen::Vector2d(-3., 4.), tolerance) || !r2.isApprox(Eigen::Vector2d(5., 7.), tolerance)) {
		FAIL() << "The result vector is wrong.";
	}
	if (r3.size() != 3 || !r3.isApprox(r3t, tolerance)) {
		FAIL() << "The method works for vectors of length 2, but it must also work for vectors of arbitrary length as it will be used with more joints in the following exercise parts.";
	}
}

TEST(InverseKinematics, applyChangeToJoints) {
	const InverseKinematics_2Links ik(4, 2, 1, 0.1);
	JointAngles r1 = ik.applyChangeToJoints(Eigen::Vector2d(2, 3), Eigen::Vector2d(4, -5));
	if (!r1.isApprox(Eigen::Vector2d(6., -2.))) {
		FAIL() << "The result vector is wrong.";
	}
	JointAngles r2 = ik.applyChangeToJoints(Eigen::Vector4d(2, 3, 1, -1), Eigen::Vector4d(4, -5, 2, 3));
	if (r2.size() < 4 || !r2.isApprox(Eigen::Vector4d(6., -2., 3., 2.))) {
		FAIL() << "The method works for vectors of length 2, but it must also work for vectors of arbitrary length as it will be used with more joints in the following exercise parts.";
	}
}

TEST(InverseKinematics, oneIteration) {
	const double tolerance = 1e-3;
	const InverseKinematics_2Links ik(4, 2, 1, 0.1);
	JointAngles q1(2); q1 << 0.1, 0.1;
	JointAngles q2(2); q2 << 0.3, 0.2;
	EndeffectorPose e1 = EndeffectorPose::Zero(2), e2 = EndeffectorPose::Zero(2);
	EndeffectorPose g(2); g << 3.0, 3.0;
	ik.oneIteration(q1, e1, g);
	ik.oneIteration(q2, e2, g);
	if (e1.isZero() && e2.isZero()) {
		FAIL() << "The method does not update the endeffector pose e.";
	}
	if (q1.isApprox(Eigen::Vector2d(0.1, 0.1)) && q2.isApprox(Eigen::Vector2d(0.3, 0.2))) {
		FAIL() << "The method does not update the joint angles q.";
	}
	if (!q1.isApprox(Eigen::Vector2d(0.985527, -1.88219), tolerance) || !q2.isApprox(Eigen::Vector2d(0.812286, -0.941904), tolerance)) {
		FAIL() << "The new joint angles q are incorrect.";
	}
	if (!e1.isApprox(Eigen::Vector2d(4.08235, 0.990506), tolerance) || !e2.isApprox(Eigen::Vector2d(5.7262, 2.51568), tolerance)) {
			FAIL() << "The new endeffector pose e is incorrect.";
	}
}

class WhileConditionChecker_2Links : public InverseKinematics_2Links {
public:
	WhileConditionChecker_2Links(const double& a0, const double& a1, const double& h, const double& alpha, const JointAngles& q, const EndeffectorPose& e)
: InverseKinematics_2Links(a0, a1, h, alpha), qOpt(q), eOpt(e) {};
	~WhileConditionChecker_2Links() {};

	void oneIteration(JointAngles& q, EndeffectorPose& e, const EndeffectorPose& g) const {
		static size_t n = 0;
		if (n > 0) {
			FAIL() << "The while loop does not end when e is close to g";
		}
		e = eOpt;
		q = qOpt;
		++n;
	}

	const JointAngles qOpt;
	const EndeffectorPose eOpt;
};

class ProgressChecker_2Links : public InverseKinematics_2Links {
public:
	ProgressChecker_2Links(const double& a0, const double& a1, const double& h, const double& alpha)
: InverseKinematics_2Links(a0, a1, h, alpha) {};
	~ProgressChecker_2Links() {};

	void oneIteration(JointAngles& q, EndeffectorPose& e, const EndeffectorPose& g) const {
		static bool first = true;
		static size_t n = 0;
		static JointAngles qLast(JointAngles::Zero(2));
		static EndeffectorPose eLast(JointAngles::Zero(2));
		if (first) {
			first = false;
		} else {
			if (q.isApprox(qLast) && e.isApprox(eLast)) {
				FAIL() << "The while loop calls oneIteration with the same values over and over again without making progress.";
			}
			if (n > 1000) {
				FAIL() << "The method needs too many iterations of the while loop, aborted after 1000 iterations.";
			}
		}
		JointAngles qTmp(q);
		EndeffectorPose eTmp(e);
		InverseKinematics_2Links::oneIteration(q, e, g);
		qLast = qTmp;
		eLast = eTmp;
		++n;
	}
};

TEST(InverseKinematics, 2Links_computeIK) {
	EndeffectorPose g(2); g << 3.25, 2.82;
	WhileConditionChecker_2Links whileChecker(4, 2, 1, 0.1, Eigen::Vector2d(0.43998831, 0.39757276), Eigen::Vector2d(3.2, 2.8));
	JointAngles q = whileChecker.computeIK(g, 0.1);
	if (q.isZero()) {
		FAIL() << "The method returns a zero vector.";
	}
	ProgressChecker_2Links progressChecker(4, 2, 1, 0.1);
	progressChecker.computeIK(g, 0.1);

	const double tolerance = 1e-3;
	const double maxErr = 0.1;
	const InverseKinematics_2Links ik(4, 2, 1, 0.1);

	q = ik.computeIK(g, maxErr);
	const double x = 4.0 * cos((double) q(0)) + 3.0 * cos((double) (q(0)+q(1)));
	const double y = 4.0 * sin((double) q(0)) + 3.0 * sin((double) (q(0)+q(1)));
	const double err = sqrt((double) ((x - g(0))*(x - g(0)) + (y - g(1))*(y - g(1))));
	if (err > maxErr) {
		FAIL() << "The method returns joint angles that put the endeffector too far away from the goal pose.";
	}
}

TEST(InverseKinematics, 3Links_forwardKinematic) {
	const double tolerance = 1e-3;
	InverseKinematics_3Links ik(4, 2, 1, 3, 0.1);
	EndeffectorPose e1 = ik.forwardKinematic(Eigen::Vector3d(0.0, 0.0, 0.0));
	EndeffectorPose e2 = ik.forwardKinematic(Eigen::Vector3d(M_PI/2, M_PI/2, M_PI/2));
	EndeffectorPose e3 = ik.forwardKinematic(Eigen::Vector3d(M_PI/2, 0, -M_PI/2));
	EndeffectorPose e4 = ik.forwardKinematic(Eigen::Vector3d(M_PI, -M_PI/2, M_PI/2));
	if (e1.isZero() && e2.isZero() && e3.isZero()) {
		FAIL() << "The method does not do anything.";
	}
	if (e1.size() != 3 || e2.size() != 3 || e3.size() != 3) {
		FAIL() << "The size of the pose vector is wrong. It should be (e_x, e_y, e_theta).";
	}
	if (!e1.topRows(2).isApprox(Eigen::Vector2d(10.0, 0.0)) ||
		!e2.topRows(2).isApprox(Eigen::Vector2d(-2.0, 0.0)) ||
		!e3.topRows(2).isApprox(Eigen::Vector2d( 4.0, 6.0)) ||
		!e4.topRows(2).isApprox(Eigen::Vector2d(-8.0, 2.0))) {
		FAIL() << "The position (e_x, e_y) of the endeffector is wrong.";
	}
	if (fabs(angles::shortest_angular_distance((double) e1(2), 0.0)) > tolerance ||
		fabs(angles::shortest_angular_distance((double) e2(2), 1.5 * M_PI)) > tolerance ||
		fabs(angles::shortest_angular_distance((double) e3(2), 0.0)) > tolerance ||
		fabs(angles::shortest_angular_distance((double) e4(2), M_PI)) > tolerance)  {
		FAIL() << "The orientation e_theta of the endeffector is wrong.";
	}
}

TEST(InverseKinematics, 3Links_jacobian) {
	const double tolerance = 1e-3;
	InverseKinematics_3Links ik(4, 2, 1, 3, 0.1);
	Jacobian jacobian[] = {
			ik.jacobian(Eigen::Vector3d(    0.0,  0.0,    0.0    )),
			ik.jacobian(Eigen::Vector3d(   M_PI,  0.0,    M_PI/2 )),
			ik.jacobian(Eigen::Vector3d(    0.0,  M_PI,   0.0    )),
			ik.jacobian(Eigen::Vector3d(   M_PI,  M_PI,   M_PI/2)),
			ik.jacobian(Eigen::Vector3d( M_PI/2,  M_PI/2, 0.0    )),
			ik.jacobian(Eigen::Vector3d(-M_PI/2,  M_PI/2, -M_PI/2)),
			ik.jacobian(Eigen::Vector3d( M_PI/2, -M_PI/2, 0.0    )),
			ik.jacobian(Eigen::Vector3d(-M_PI/2, -M_PI/2, -M_PI/2)),
	};

	Eigen::Matrix3d trueJacobian[8];
	trueJacobian[0] <<  0,   10,   1,   0,   6,   1,   0,    4,    1;
	trueJacobian[1] <<  4,   -6,   1,   4,  -2,   1,   4,    0,    1;
	trueJacobian[2] <<  0,   -2,   1,   0,  -6,   1,   0,   -4,    1;
	trueJacobian[3] << -4,   -2,   1,  -4,   2,   1,  -4,    0,    1;
	trueJacobian[4] << -4,   -6,   1,   0,  -6,   1,   0,   -4,    1;
	trueJacobian[5] <<  8,    2,   1,   4,   2,   1,   4,    0,    1;
	trueJacobian[6] << -4,    6,   1,   0,   6,   1,   0,    4,    1;
	trueJacobian[7] <<  0,   -2,   1,  -4,  -2,   1,  -4,    0,    1;

	size_t correct[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
	size_t numZero = 0;
	size_t numAllCorrect = 0;
	for (size_t i = 0; i < 8; ++i) {
		if (jacobian[i].isZero()) {
			++numZero;
		}
		bool incorrect = false;
		for (size_t j = 0; j < 3; ++j) {
			for (size_t k = 0; k < 3; ++k) {
				if (fabs((double) (jacobian[i](j, k) - trueJacobian[i](j, k))) < tolerance) {
					++correct[j][k];
				} else {
					incorrect = true;
				}
			}
		}
		if (!incorrect) {
			++numAllCorrect;
		}
	}

	if (numZero == 8) {
		FAIL() << "The method does not do anything.";
	}
	if (numAllCorrect == 8) {
		return;
	}

	std::vector<std::pair<size_t, size_t> > correctIdx;
	std::vector<std::pair<size_t, size_t> > wrongIdx;
	for (size_t j = 0; j < 3; ++j) {
		for (size_t k = 0; k < 3; ++k) {
			if (correct[j][k] == 8) {
				correctIdx.push_back(std::make_pair(j, k));
			} else {
				wrongIdx.push_back(std::make_pair(j, k));
			}
		}
	}

	if (correctIdx.empty()) {
		FAIL() << "All components of the returned Jacobian are wrong.";
	}
	std::stringstream msg;
	msg << "The following components of the returned Jacobian are wrong: ";
	for (size_t i = 0; i < wrongIdx.size(); ++i) {
		msg << "(" << wrongIdx[i].first << ", " << wrongIdx[i].second << ")";
		if (i != wrongIdx.size() - 1) {
			msg << ", ";
		}
	}

}

class WhileConditionChecker_3Links : public InverseKinematics_3Links {
public:
	WhileConditionChecker_3Links(const double& a0, const double& a1, const double& a2, const double& h, const double& alpha, const JointAngles& q, const EndeffectorPose& e)
: InverseKinematics_3Links(a0, a1, a2, h, alpha), qOpt(q), eOpt(e) {};
	~WhileConditionChecker_3Links() {};

	void oneIteration(JointAngles& q, EndeffectorPose& e, const EndeffectorPose& g) const {
		static size_t n = 0;
		if (n > 0) {
			FAIL() << "The while loop does not end when e is close to g";
		}
		e = eOpt;
		q = qOpt;
		++n;
	}

	const JointAngles qOpt;
	const EndeffectorPose eOpt;
};

class ProgressChecker_3Links : public InverseKinematics_3Links {
public:
	ProgressChecker_3Links(const double& a0, const double& a1, const double& a2, const double& h, const double& alpha)
: InverseKinematics_3Links(a0, a1, a2, h, alpha) {};
	~ProgressChecker_3Links() {};

	void oneIteration(JointAngles& q, EndeffectorPose& e, const EndeffectorPose& g) const {
		static bool first = true;
		static size_t n = 0;
		static JointAngles qLast(JointAngles::Zero(2));
		static EndeffectorPose eLast(JointAngles::Zero(2));
		if (first) {
			first = false;
		} else {
			if (q.isApprox(qLast) && e.isApprox(eLast)) {
				FAIL() << "The while loop calls oneIteration with the same values over and over again without making progress.";
			}
			if (n > 1000) {
				FAIL() << "The method needs too many iterations of the while loop, aborted after 1000 iterations.";
			}
		}
		JointAngles qTmp(q);
		EndeffectorPose eTmp(e);
		InverseKinematics_3Links::oneIteration(q, e, g);
		qLast = qTmp;
		eLast = eTmp;
		++n;
	}
};

TEST(InverseKinematics, 3Links_computeIK) {
	const double tolerance = 1e-3;
	const double maxErr = 0.1;
	const double maxAngErr = 0.1;

	EndeffectorPose g(3); g << 2.55, 1.55, 0.1;
	WhileConditionChecker_3Links whileChecker(1.6, 1.2, 0.7, 0.1, 0.1, Eigen::Vector3d(1.2484, -1.3294, 0.180799), Eigen::Vector3d(2.5, 1.5, 0.1));
	JointAngles q = whileChecker.computeIK(g, maxErr, maxAngErr);
	if (q.isZero()) {
		FAIL() << "The method returns a zero vector.";
	}
	ProgressChecker_3Links progressChecker(1.6, 1.2, 0.7, maxErr, maxAngErr);
	progressChecker.computeIK(g, 0.1, 0.1);

	const InverseKinematics_3Links ik(1.6, 1.2, 0.7, maxErr, maxAngErr);
	g << 2.5, 1.5, 0.1;
	q = ik.computeIK(g, maxErr, maxAngErr);
	EndeffectorPose e = ik.forwardKinematic(q);
	if (!e.topRows(2).isApprox(g.topRows(2), maxErr)) {
		FAIL() << "The method returns joint angles that put the endeffector too far away from the goal position.";
	}
	if (fabs(angles::shortest_angular_distance((double) e(2), (double) g(2))) > maxAngErr) {
		FAIL() << "The method returns joint angles with a too high error in the endeffector orientation.";
	}
}

int main(int argc, char *argv[]) {
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
