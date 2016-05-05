#include <gtest/gtest.h>
#include <irm/IRM.h>
#include <Eigen/StdVector>
#include <angles/angles.h>

using namespace irm;

TEST(IRM, sampleConfiguration) {
	IRM irm;
	const size_t numIterations = 1000;
	bool seenNonzero = false;
	Eigen::Vector3d old = Eigen::Vector3d::Zero();
	Eigen::Vector3d mean = Eigen::Vector3d::Zero();
	Eigen::Vector3d variance = Eigen::Vector3d::Zero();
	Eigen::Vector3d expectedMean, expectedVariance;
	expectedMean << M_PI/4, 0, 0;
	expectedVariance << M_PI*M_PI/48., M_PI*M_PI/3., M_PI*M_PI/3.;
	size_t sameCount = 0;
	for (size_t i = 0; i < numIterations; ++i) {
		const Eigen::Vector3d v = irm.sampleConfiguration();
		if (v[0] < 0 || v[0] > M_PI/2) {
			FAIL() << "The first angle is outside the range [0, pi/2].";
		}
		if (v[1] < -M_PI || v[1] > M_PI) {
			FAIL() << "The second angle is outside the range [-pi, pi].";
		}
		if (v[2] < -M_PI || v[2] > M_PI) {
			FAIL() << "The second angle is outside the range [-pi, pi].";
		}
		if (!v.isZero()) {
			seenNonzero = true;
		}
		if (i == 4 && !seenNonzero) {
			FAIL() << "The method does not do anything.";
		}
		if (old.isApprox(v)) {
			++sameCount;
		}
		if (sameCount > 5) {
			FAIL() << "The method returned the same vector 5 times in a row, so it is probably not random. DO NOT call srand() inside the method!";
		}
		old = v;
		mean += v;
		variance += (v - expectedMean).cwiseProduct(v - expectedMean);

	}
	mean /= numIterations;
	variance /= numIterations;
	if (!mean.isApprox(expectedMean, 0.3)) {
		FAIL() << "The mean of the sample vectors deviates significantly from the expected mean of a uniform distribution.";
	}
	if (!variance.isApprox(expectedVariance, 0.1)) {
		FAIL() << "The variance of the sample vectors deviates significantly from the expected variance of a uniform distribution.";
	}
}

TEST(IRM, computeManipulability) {
	IRM irm;
	Eigen::Vector3d jointAngles[3];
	jointAngles[0] << 0.1, 0.1, 0.1;
	jointAngles[1] << M_PI/4, M_PI/2, 0.1;
	jointAngles[2] << M_PI/4, 0.4, -0.3;
	double expectedManipulability[3] = {0.742042, 0.570458, 0.695458};
	Eigen::Vector3d endeffectorPose;
	for (size_t i = 0; i < 3; ++i) {
		if (irm.forwardKinematics(jointAngles[i], endeffectorPose)) {
			double manipulability = irm.computeManipulability(jointAngles[0], endeffectorPose);
			if (manipulability == 0.0) {
				FAIL() << "The method does not return the manipulability score.";
			}
			ASSERT_NEAR(expectedManipulability[i], manipulability, 0.05);
		}
	}
}

class RMTest : public IRM {
private:
	struct Testcase {
		Eigen::Vector3d jointAngles;
		Eigen::Vector3d endeffectorPose;
		double manipulability;
		bool shouldAdd;
		bool added;
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	};
	std::vector<Testcase> testcases;
	size_t numAdded;
	size_t numExpected;

public:
	RMTest() {
		testcases.resize(3);
		testcases[0].jointAngles << 0.0, 0.0, -M_PI/2;
		testcases[0].endeffectorPose << 0.0, 0.0, 0.0;
		testcases[0].manipulability = 0.0;
		testcases[0].shouldAdd = false;
		testcases[0].added = false;
		testcases[1].jointAngles << 0.1, 0.1, 0.1;
		testcases[1].endeffectorPose << 5.42281, 0.844599, 0.3;
		testcases[1].manipulability = 0.742042;
		testcases[1].shouldAdd = true;
		testcases[1].added = false;
		testcases[2].jointAngles << 0.3, 0.1, M_PI/2;
		testcases[2].endeffectorPose << 4.51342, 2.12593, 1.970796;
		testcases[2].manipulability = 0.55570;
		testcases[2].shouldAdd = true;
		testcases[2].added = false;
		numAdded = 0;
		numExpected = 2;

	}
	Eigen::Vector3d sampleConfiguration() const {
		Eigen::Vector3d result;
		static size_t counter = 0;
		if (counter < testcases.size()) {
			result << testcases[counter].jointAngles;
		} else {
			ADD_FAILURE() << "The method called sampleConfiguration() more often than necessary. Maybe valid samples were dropped or there might be an infinite loop.";
			result << Eigen::Vector3d::Random();
		}
		++counter;
		return result;
	}

	void addToRM(const Eigen::Vector3d& jointAngles, const Eigen::Vector3d& endeffectorPose, const double& manipulability) {
		if (endeffectorPose[1] < 0) {
			FAIL() << "Tried to add a configuration to the reachability map where the end effector is below the ground.";
		}
		bool found = false;
		for (size_t i = 0; i < testcases.size() && !found; ++i) {
			if (jointAngles.isApprox(testcases[i].jointAngles, 1e-3)) {
				found = true;
				if (!testcases[i].shouldAdd) {
					FAIL() << "The method tried to add a configuration that should not be added to the reachability map.";
				}
				if (!endeffectorPose.isApprox(testcases[i].endeffectorPose, 0.1)) {
					FAIL() << "The endeffector pose is wrong for the given joint angles.";
				}
				if (fabs(manipulability - testcases[i].manipulability) > 1e-3) {
					FAIL() << "The manipulability score is wrong for the given joint angles.";
				}
				if (testcases[i].added) {
					FAIL() << "The method added the sample sample twice to the reachability map.";
				}
				testcases[i].added = true;
				++numAdded;
			}
		}
		if (!found) {
			FAIL() << "The method tried to add a configuration with joint angles that were not generated by sampleConfiguration().";
		}
	}

	void finish() {
		if (numAdded == 0) {
			FAIL() << "The method did not add any valid configuration to the reachability map.";
		}
		if (numAdded < numExpected) {
			FAIL() << "The method added less than the requested number of samples to the reachability map.";
		}
		for (size_t i = 0; i < testcases.size(); ++i) {
			if (testcases[i].shouldAdd && !testcases[i].added) {
				FAIL() << "The method did not add an expected configuration to the reachability map.";
			}
		}
	}

	size_t getNumExpected() const {
		return numExpected;
	}
};

TEST(IRM, computeRM) {
	RMTest irm;
	irm.computeRM(irm.getNumExpected());
	irm.finish();
}

class IRMTest : public IRM {
private:
	struct Testcase {
		Eigen::Vector3d jointAngles;
		Eigen::Vector3d basePose;
		double manipulability;
		bool added;
	};
	size_t numAdded;
	std::vector<Testcase> testcases;
	std::vector<IRM::RMVoxel> voxels;

public:
	IRMTest() {
		testcases.resize(4);
		testcases[0].jointAngles << 0.1, 0.1, 0.1;
		testcases[0].basePose << -5.43021, 0.795675, -0.3;
		testcases[0].manipulability = 0.742042;
		testcases[0].added = false;
		testcases[1].jointAngles << 0.1, 0.1, 0.102;
		testcases[1].basePose << -5.42861, 0.805534, -0.302;
		testcases[1].manipulability = 0.742042;
		testcases[1].added = false;
		testcases[2].jointAngles << 0.3, 0.1, M_PI/2;
		testcases[2].basePose << -0.2005, 4.98501, -1.9708;
		testcases[2].manipulability = 0.55570;
		testcases[2].added = false;
		testcases[3].jointAngles << 0.3, 0.102, M_PI/2;
		testcases[3].basePose << -0.19453, 4.98441, -1.9728;
		testcases[3].manipulability = 0.55570;
		testcases[3].added = false;
		voxels.resize(2);
		voxels[0].configurations.resize(2);
		voxels[1].configurations.resize(2);
		for (size_t i = 0; i < 2; ++i) {
			for (size_t j = 0; j < 2; ++j) {
				voxels[i].configurations[j].jointAngles = testcases[2*i+j].jointAngles;
				voxels[i].configurations[j].manipulability = testcases[2*i+j].manipulability;
			}
		}
		numAdded = 0;
	}
	const std::vector<IRM::RMVoxel>& getVoxels() const {
		return voxels;
	}
	void addToIRM(const Eigen::Vector3d& basePose, const Eigen::Vector3d& jointAngles, const double& manipulability) {
		static size_t counter = 0;
		bool found = false;
		for (size_t i = 0; i < testcases.size() && !found; ++i) {
			if (jointAngles.isApprox(testcases[i].jointAngles, 1e-3)) {
				found = true;
				if (!basePose.isApprox(testcases[i].basePose, 1e-3)) {
					FAIL() << "The base pose for the given joint angles is not correct.";
				}
				if (fabs(manipulability - testcases[i].manipulability) > 1e-3) {
					FAIL() << "The manipulability score is wrong for the given joint angles.";
				}
				if (testcases[i].added) {
					FAIL() << "The method added the sample sample twice to the inverse reachability map.";
				}
				testcases[i].added = true;
				++numAdded;
			}
		}
		if (!found) {
			FAIL() << "The method tried to add a configuration with joint angles that were not in the reachability map.";
		}
		++counter;
	}

	void finish() {
		if (numAdded == 0) {
			FAIL() << "The method did not add any valid configuration to the inverse reachability map.";
		}
		if (numAdded < testcases.size()) {
			FAIL() << "The method did not add all configurations to the inverse reachability map.";
		}
		for (size_t i = 0; i < testcases.size(); ++i) {
			if (!testcases[i].added) {
				FAIL() << "The method did not add an expected configuration to the inverse reachability map.";
			}
		}
	}

	friend class IRM_computeIRM_Test;
};

TEST(IRM, computeIRM) {
	IRMTest irm;
	irm.computeIRM(irm.getVoxels());
	irm.finish();
}

int main(int argc, char *argv[]) {
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
